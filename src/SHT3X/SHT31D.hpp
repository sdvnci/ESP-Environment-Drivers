#pragma once

#include "../I2CSensor.hpp"
#include <cmath>
#include <mutex>

namespace sht3x {
    /** Default address (ADDR pin low). */
    constexpr uint8_t ADDRESS_PRIMARY = 0x44;
    /** Alternate address (ADDR pin high). */
    constexpr uint8_t ADDRESS_SECONDARY = 0x45;

    /**
     * SHT3x command codes from the datasheet.
     */
    enum CMDS : uint16_t {
        READSTATUS = 0xF32D,
        CLEARSTATUS = 0x3041,
        SOFTRESET = 0x30A2,
        HEATER_ON = 0x306D,
        HEATER_OFF = 0x3066,
    };

    /** Bit positions in the status register. */
    enum STATUS_BITS : uint8_t {
        STATUS_HEATER = 13,
        STATUS_RESET_DETECTED = 4,
    };

    /**
     * Single-shot measurement commands, without clock stretching.
     * Without stretching the bus is free while the sensor converts, and we
     * wait the datasheet's max conversion time before reading.
     */
    enum MEASUREMENT_MODE : uint16_t {
        HIGHREP = 0x2400,
        MEDREP = 0x240B,
        LOWREP = 0x2416,
    };

    /**
     * Max conversion time in ms for a single-shot measurement.
     */
    constexpr uint32_t measurementTimeMs(MEASUREMENT_MODE mode) {
        return mode == HIGHREP ? 16 : mode == MEDREP ? 7 : 5;
    }

    /** One temperature/humidity measurement. */
    struct Reading {
        /** Temperature in °C. */
        float temperature = NAN;
        /** Relative humidity in %. */
        float humidity = NAN;
    };
};

/** @deprecated Use sht3x::ADDRESS_PRIMARY. */
#define SHT31_ADDRESS sht3x::ADDRESS_PRIMARY


/**
 * Driver for the Sensirion SHT30/SHT31/SHT35 temperature and humidity sensor.
 */
class SHT31 : public I2CSensor {

    public:

        SHT31(
            uint8_t bus_num,
            uint8_t sda,
            uint8_t scl,
            uint8_t address = sht3x::ADDRESS_PRIMARY,
            sht3x::MEASUREMENT_MODE mode = sht3x::MEASUREMENT_MODE::HIGHREP
        ) : I2CSensor(address, bus_num, sda, scl, 10000, 1000000), _mode(mode) {}

        /**
         * @brief Soft-reset the sensor. Settings (eg: the heater) return to defaults.
         * @return True if the sensor acknowledged the reset.
         */
        [[nodiscard]] bool reset(void);

        /**
         * @brief Turn the internal heater on, eg: to drive off condensation.
         * Readings are biased warm while it is on.
         */
        [[nodiscard]] bool enableHeater(void);

        /** @brief Turn the internal heater off. */
        [[nodiscard]] bool disableHeater(void);

        /**
         * @brief Read the heater state back from the sensor.
         * @param enabled Set to the heater state on success.
         * @return True if the status register was read.
         */
        [[nodiscard]] bool isHeaterEnabled(bool& enabled);

        /**
         * @brief Read the status register.
         * @param status Set to the register value on success.
         * @return True if the register was read and its CRC matched.
         */
        [[nodiscard]] bool readStatus(uint16_t& status);

        /** @brief Clear the status register's alert and reset flags. */
        [[nodiscard]] bool clearStatus(void);

        /** @brief Set the repeatability used by update(). Higher is less noisy but slower. */
        void setMode(sht3x::MEASUREMENT_MODE mode);

        /**
         * @brief Take a measurement and cache it.
         * @return True on success, false otherwise - see lastError().
         */
        [[nodiscard]] bool update(void) override;

        /** @brief The last successful measurement, temperature and humidity from the same sample. */
        sht3x::Reading reading(void) const;

        /** @brief Temperature in °C from the last successful update(), or NaN. */
        float getTemperature(void) const { return reading().temperature; }

        /** @brief Relative humidity in % from the last successful update(), or NaN. */
        float getHumidity(void) const { return reading().humidity; }

    protected:

        [[nodiscard]] bool deviceSpecificSetup() override;

    private:

        /** Send a command, then leave the sensor its minimum idle time before the next. */
        [[nodiscard]] bool command(uint16_t cmd);

        /** Read a word followed by its CRC. */
        [[nodiscard]] bool readWord(uint16_t& word);

        std::atomic<sht3x::MEASUREMENT_MODE> _mode;

        mutable std::mutex _data_mutex;
        sht3x::Reading _reading;
};
