#pragma once

#include "FakeWire.hpp"
#include "../src/I2CManager.hpp"
#include <algorithm>

/**
 * Sensirion CRC8, written out independently of the driver's so the two can be
 * checked against each other (and against the datasheet's example).
 */
inline uint8_t fakeCrc8(uint8_t hi, uint8_t lo) {
    uint8_t crc = 0xFF;
    const uint8_t bytes[2] = {hi, lo};
    for (uint8_t byte : bytes) {
        crc ^= byte;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31) : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}


/**
 * Device with a register pointer that auto-increments: a write of [reg, data...]
 * writes registers, a write of [reg] sets the pointer for the next read.
 */
class FakeRegisterDevice : public FakeDevice {
    public:
        std::array<uint8_t, 256> regs{};
        std::vector<std::pair<uint8_t, uint8_t>> writes;

        bool onWrite(const std::vector<uint8_t>& bytes) override {
            if (bytes.empty()) return true;
            _pointer = bytes[0];
            for (size_t i = 1; i < bytes.size(); ++i) {
                regs[_pointer] = bytes[i];
                writes.emplace_back(_pointer, bytes[i]);
                onRegisterWrite(_pointer, bytes[i]);
                ++_pointer;
            }
            return true;
        }

        size_t onRead(uint8_t* out, size_t len) override {
            for (size_t i = 0; i < len; ++i) {
                out[i] = readRegister(_pointer++);
            }
            return len;
        }

        /** @return The last value written to a register, or -1 if never written. */
        int lastWrite(uint8_t reg) const {
            for (auto it = writes.rbegin(); it != writes.rend(); ++it) {
                if (it->first == reg) return it->second;
            }
            return -1;
        }

    protected:
        virtual void onRegisterWrite(uint8_t, uint8_t) {}
        virtual uint8_t readRegister(uint8_t reg) { return regs[reg]; }

        uint8_t _pointer = 0;
};


/**
 * SHT31 command protocol: 16-bit commands, then reads of word+CRC triplets.
 */
class FakeSHT31 : public FakeDevice {
    public:
        uint16_t rawTemperature = 0x6666; // 25.0 °C
        uint16_t rawHumidity = 0x8000;    // 50.0 %
        uint16_t status = 0x8010;         // Alert pending, reset detected
        bool heater = false;
        bool corruptCrc = false;
        int resets = 0;
        int measurements = 0;
        uint16_t lastCommand = 0;

        bool onWrite(const std::vector<uint8_t>& bytes) override {
            if (bytes.size() != 2) return false;
            const uint16_t cmd = static_cast<uint16_t>((bytes[0] << 8) | bytes[1]);
            lastCommand = cmd;

            switch (cmd) {
                case 0x30A2: // Soft reset
                    ++resets;
                    heater = false;
                    _pending.clear();
                    return true;
                case 0x306D: heater = true; return true;
                case 0x3066: heater = false; return true;
                case 0x3041: // Clear status: alert and reset flags
                    status &= static_cast<uint16_t>(~((1u << 15) | (1u << 11) | (1u << 10) | (1u << 4)));
                    return true;
                case 0xF32D:
                    _pending.clear();
                    pushWord(static_cast<uint16_t>(status | (heater ? (1u << 13) : 0)));
                    return true;
                case 0x2400: case 0x240B: case 0x2416: // Single shot, no stretching
                    ++measurements;
                    _pending.clear();
                    pushWord(rawTemperature);
                    pushWord(rawHumidity);
                    return true;
                default:
                    return false; // Unknown command: NACK
            }
        }

        size_t onRead(uint8_t* out, size_t len) override {
            const size_t n = std::min(len, _pending.size());
            std::copy(_pending.begin(), _pending.begin() + n, out);
            _pending.erase(_pending.begin(), _pending.begin() + n);
            return n;
        }

    private:
        void pushWord(uint16_t word) {
            const uint8_t hi = word >> 8;
            const uint8_t lo = word & 0xFF;
            const uint8_t crc = fakeCrc8(hi, lo);
            _pending.push_back(hi);
            _pending.push_back(lo);
            _pending.push_back(corruptCrc ? static_cast<uint8_t>(crc ^ 0xFF) : crc);
        }

        std::vector<uint8_t> _pending;
};


/**
 * Adafruit_BMP3XX's cal_crc(), ported as-is, to cross-check the driver's
 * table-free CRC-8 implementation.
 */
inline uint8_t adafruitCalCrc(uint8_t seed, uint8_t data) {
    const int8_t poly = 0x1D;
    for (uint8_t i = 0; i < 8; i++) {
        const int8_t var2 = ((seed & 0x80) ^ (data & 0x80)) ? 1 : 0;
        seed = static_cast<uint8_t>((seed & 0x7F) << 1);
        data = static_cast<uint8_t>((data & 0x7F) << 1);
        seed = seed ^ static_cast<uint8_t>(poly * var2);
    }
    return seed;
}

inline uint8_t adafruitTrimmingCrc(const std::array<uint8_t, 21>& calibration) {
    uint8_t crc = 0xFF;
    for (uint8_t byte : calibration) crc = adafruitCalCrc(crc, byte);
    return crc ^ 0xFF;
}


/**
 * BMP388/BMP390 register map with forced-mode conversions.
 */
class FakeBMP3 : public FakeRegisterDevice {
    public:
        uint32_t rawPressure = 0;
        uint32_t rawTemperature = 0;
        bool neverReady = false;
        int resets = 0;
        int measurements = 0;

        FakeBMP3(uint8_t chipId, const std::array<uint8_t, 21>& calibration) {
            regs[0x00] = chipId;
            regs[0x03] = 0x10; // cmd_rdy
            regs[0x30] = adafruitTrimmingCrc(calibration);
            std::copy(calibration.begin(), calibration.end(), regs.begin() + 0x31);
        }

    protected:
        void onRegisterWrite(uint8_t reg, uint8_t value) override {
            if (reg == 0x7E && value == 0xB6) {
                ++resets;
                regs[0x1B] = 0x00;
                regs[0x1C] = 0x02;
                regs[0x1F] = 0x00;
                return;
            }

            const uint8_t mode = (value >> 4) & 0x03;
            if (reg == 0x1B && (mode == 0x01 || mode == 0x02)) {
                ++measurements;
                const uint8_t data[6] = {
                    static_cast<uint8_t>(rawPressure), static_cast<uint8_t>(rawPressure >> 8), static_cast<uint8_t>(rawPressure >> 16),
                    static_cast<uint8_t>(rawTemperature), static_cast<uint8_t>(rawTemperature >> 8), static_cast<uint8_t>(rawTemperature >> 16),
                };
                std::copy(data, data + 6, regs.begin() + 0x04);
                if (!neverReady) regs[0x03] |= 0x60;
                regs[0x1B] &= 0x0F; // Back to sleep after one conversion
            }
        }

        uint8_t readRegister(uint8_t reg) override {
            if (reg >= 0x04 && reg <= 0x09) regs[0x03] &= static_cast<uint8_t>(~0x60);
            return regs[reg];
        }
};


/**
 * MPU6050 register map with settable big-endian sensor outputs.
 */
class FakeMPU6050 : public FakeRegisterDevice {
    public:
        explicit FakeMPU6050(uint8_t whoAmI = 0x68) {
            regs[0x75] = whoAmI;
            regs[0x6B] = 0x40; // Asleep after power-on
        }

        void setGyro(int16_t x, int16_t y, int16_t z) { setXYZ(0x43, x, y, z); }
        void setAccel(int16_t x, int16_t y, int16_t z) { setXYZ(0x3B, x, y, z); }
        void setTemperature(int16_t raw) { setWord(0x41, raw); }

    private:
        void setWord(uint8_t reg, int16_t value) {
            regs[reg] = static_cast<uint8_t>(static_cast<uint16_t>(value) >> 8);
            regs[reg + 1] = static_cast<uint8_t>(value & 0xFF);
        }

        void setXYZ(uint8_t reg, int16_t x, int16_t y, int16_t z) {
            setWord(reg, x);
            setWord(reg + 2, y);
            setWord(reg + 4, z);
        }
};


/** Fresh state for a test: no sensors registered, no devices on either bus. */
inline void resetFakeBuses() {
    I2CManager& manager = I2CManager::getInstance();
    manager.clear();
    for (uint8_t bus = 0; bus < I2CManager::BUS_COUNT; ++bus) {
        manager.wire(bus)->detachAll();
    }
}
