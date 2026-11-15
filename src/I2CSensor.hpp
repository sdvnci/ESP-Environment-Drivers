#pragma once

#include "Platform.hpp"
#include "I2CManager.hpp"
#include <atomic>
#include <cstddef>
#include <cstdint>

/**
 * Base class for I2C sensors.
 *
 * Thread safety is layered:
 *  - Every bus transaction (write, read, write-then-read with a repeated start)
 *    holds the bus mutex, so sensors sharing a bus never interleave bytes.
 *  - Every public operation on a sensor holds that sensor's mutex, so a
 *    multi-step operation (eg: trigger a conversion, wait, read it back) is
 *    never interleaved with another task's operation on the same sensor. The
 *    bus is free for other sensors while one waits on a conversion.
 *  - Cached readings are returned as one snapshot so a value pair (eg:
 *    temperature and humidity) always comes from the same measurement.
 */
class I2CSensor {
    friend class I2CManager;

    public:

        virtual ~I2CSensor();

        I2CSensor(const I2CSensor&) = delete;
        I2CSensor& operator=(const I2CSensor&) = delete;

        /**
         * @brief Register the sensor on its bus and configure the device.
         * Safe to call again to re-initialise (eg: after a power cycle).
         * @return True if the device answered and was configured, false otherwise - see lastError().
         */
        [[nodiscard]] bool init();

        /**
         * @brief Take a fresh reading from the device and cache it.
         * @return True on success, false otherwise - see lastError().
         */
        [[nodiscard]] virtual bool update(void) = 0;

        uint8_t getAddress(void) const { return _i2c_addr; }
        uint8_t getBusNum(void) const { return _bus_num; }
        int getSdaPin(void) const { return _sda_pin; }
        int getSclPin(void) const { return _scl_pin; }
        uint32_t getMinClock(void) const { return _min_clock_hz; }
        uint32_t getMaxClock(void) const { return _max_clock_hz; }
        uint32_t getMaxSamples(void) const { return i2cs::MAX_SAMPLES; }
        [[nodiscard]] bool isInitialized(void) const { return _is_initialized.load(); }

        /** @brief The result of the most recent operation on this sensor. */
        I2CError lastError(void) const { return _last_error.load(); }

        /**
         * Performs a CRC8 calculation (polynomial 0x31, init 0xFF) as used by Sensirion sensors.
         * @param data  Pointer to the data to use when calculating the CRC8.
         * @param len   The number of bytes in 'data'.
         * @return The computed CRC8 value.
         */
        static uint8_t crc8(const uint8_t* data, size_t len);

    protected:

        I2CSensor(
            uint8_t addr,
            uint8_t bus_num,
            uint8_t sda,
            uint8_t scl,
            uint32_t min_clk,
            uint32_t max_clk);

        /**
         * Implemented by derived sensor classes to check the device identity
         * and configure it. Called by init() with the sensor lock held.
         * @return True if the setup is successful, false otherwise (after calling fail()).
         */
        virtual bool deviceSpecificSetup() = 0;

        /**
         * @brief Acquire this sensor for one operation. Check owns_lock() before proceeding.
         * The lock is recursive, so public methods may call each other.
         */
        i2cs::Lock lockSensor(void) const;

        /** @brief Record an error and return false, for `return fail(...)`. */
        bool fail(I2CError err) {
            _last_error.store(err);
            return false;
        }

        /** @brief Record success and return true, for `return succeed()`. */
        bool succeed(void) {
            _last_error.store(I2CError::NONE);
            return true;
        }

        /**
         * Bus transactions. Each is atomic on the bus and records its outcome
         * in lastError().
         */

        /** @brief Write raw bytes to the device. */
        [[nodiscard]] bool writeBytes(const uint8_t* data, size_t len);

        /** @brief Write one byte to a register. */
        [[nodiscard]] bool writeReg(uint8_t reg, uint8_t value);

        /** @brief Write a 16-bit command, high byte first. */
        [[nodiscard]] bool writeCommand(uint16_t cmd);

        /** @brief Read raw bytes from the device without addressing a register first. */
        [[nodiscard]] bool readBytes(uint8_t* out, size_t len);

        /** @brief Read consecutive registers, using a repeated start between address and read. */
        [[nodiscard]] bool readRegs(uint8_t reg, uint8_t* out, size_t len);

        /** @brief Read a single register. */
        [[nodiscard]] bool readReg(uint8_t reg, uint8_t& out) { return readRegs(reg, &out, 1); }

        const uint8_t _i2c_addr;
        const uint8_t _bus_num;
        const uint8_t _sda_pin;
        const uint8_t _scl_pin;
        const uint32_t _min_clock_hz;
        const uint32_t _max_clock_hz;

        std::atomic<bool> _is_initialized{false};

    private:

        /** Called by I2CManager on (un)registration. */
        void attach(I2CBus* bus) { _bus.store(bus); }
        void detach(void) {
            _bus.store(nullptr);
            _is_initialized.store(false);
        }

        /** Lock the bus for a transaction, or record why we couldn't. */
        I2CBus* acquireBus(i2cs::Lock& lock);

        std::atomic<I2CBus*> _bus{nullptr};
        std::atomic<I2CError> _last_error{I2CError::NONE};
        mutable i2cs::RecursiveMutex _sensor_mutex;
};
