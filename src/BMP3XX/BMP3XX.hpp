#pragma once

#include "../I2CSensor.hpp"
#include <cmath>
#include <mutex>

namespace bmp3 {
    /** Address with SDO pulled low. */
    constexpr uint8_t ADDRESS_PRIMARY = 0x76;
    /** Address with SDO pulled high - the default on most breakout boards. */
    constexpr uint8_t ADDRESS_SECONDARY = 0x77;

    constexpr uint8_t CHIP_ID_BMP388 = 0x50;
    constexpr uint8_t CHIP_ID_BMP390 = 0x60;

    /**
     * Register map (BMP388/BMP390 datasheets, section 4).
     * CALIB_CRC is undocumented in the datasheet; Bosch stores a CRC of the
     * calibration block there, which Adafruit_BMP3XX validates.
     */
    enum REG : uint8_t {
        CHIP_ID = 0x00,
        ERR = 0x02,
        STATUS = 0x03,
        DATA = 0x04,
        PWR_CTRL = 0x1B,
        OSR = 0x1C,
        ODR = 0x1D,
        CONFIG = 0x1F,
        CALIB_CRC = 0x30,
        CALIB = 0x31,
        CMD = 0x7E,
    };

    constexpr uint8_t CMD_SOFTRESET = 0xB6;

    constexpr uint8_t STATUS_CMD_READY = 1 << 4;
    constexpr uint8_t STATUS_PRESS_READY = 1 << 5;
    constexpr uint8_t STATUS_TEMP_READY = 1 << 6;

    constexpr uint8_t ERR_FATAL = 1 << 0;
    constexpr uint8_t ERR_CMD = 1 << 1;
    constexpr uint8_t ERR_CONF = 1 << 2;

    constexpr uint8_t PWR_PRESS_EN = 1 << 0;
    constexpr uint8_t PWR_TEMP_EN = 1 << 1;
    constexpr uint8_t PWR_MODE_FORCED = 0x01 << 4;

    constexpr size_t CALIB_LEN = 21;
    constexpr size_t DATA_LEN = 6;

    /** Oversampling - more is less noisy, slower and uses more power. */
    enum class Oversampling : uint8_t {
        X1 = 0,
        X2 = 1,
        X4 = 2,
        X8 = 3,
        X16 = 4,
        X32 = 5,
    };

    /** IIR filter coefficient - smooths short spikes (eg: wind gusts, door slams). */
    enum class IIRFilter : uint8_t {
        OFF = 0,
        COEF_1 = 1,
        COEF_3 = 2,
        COEF_7 = 3,
        COEF_15 = 4,
        COEF_31 = 5,
        COEF_63 = 6,
        COEF_127 = 7,
    };

    /**
     * Calibration coefficients, already scaled to floating point
     * (BMP388 datasheet section 9.1, as in Bosch's BMP3_DOUBLE_PRECISION_COMPENSATION).
     */
    struct Calibration {
        double par_t1, par_t2, par_t3;
        double par_p1, par_p2, par_p3, par_p4, par_p5, par_p6;
        double par_p7, par_p8, par_p9, par_p10, par_p11;

        /** Parse the 21 raw bytes read from REG::CALIB. */
        static Calibration fromRegisters(const uint8_t* raw);
    };

    /**
     * @brief CRC of the calibration block: CRC-8, polynomial 0x1D, init 0xFF,
     * final XOR 0xFF - the same check as Adafruit_BMP3XX's validate_trimming_param().
     */
    uint8_t calibrationCrc(const uint8_t* raw, size_t len);

    /**
     * @brief Compensated temperature from a raw 24-bit sample.
     * @return Temperature in °C - also t_lin, the input to compensatePressure().
     */
    double compensateTemperature(uint32_t raw, const Calibration& calib);

    /**
     * @brief Compensated pressure from a raw 24-bit sample.
     * @param temperature The result of compensateTemperature() for the same measurement.
     * @return Pressure in Pa.
     */
    double compensatePressure(uint32_t raw, double temperature, const Calibration& calib);

    /**
     * @brief Max time in µs for one forced-mode measurement of pressure and temperature
     * (BMP388 datasheet section 3.9.2).
     */
    uint32_t measurementTimeUs(Oversampling pressure, Oversampling temperature);

    /** One pressure/temperature measurement. */
    struct Reading {
        /** Temperature in °C. */
        float temperature = NAN;
        /** Pressure in Pa. */
        float pressure = NAN;
    };
};


/**
 * Driver for the Bosch BMP388 and BMP390 barometric pressure sensors.
 *
 * Follows Adafruit_BMP3XX: same default address, default settings, calibration
 * CRC check and double-precision compensation, plus its method names so
 * existing sketches port over. Differences:
 *  - Forced mode only: each update() triggers one conversion and the sensor
 *    sleeps in between. (Adafruit also only uses forced mode; its ODR setting
 *    has no effect there, so it is not offered.)
 *  - update() waits for the data-ready flags before reading. Adafruit reads
 *    straight after triggering, which returns the previous conversion.
 *  - Squares and cubes stay in double; Bosch's pow_bmp3() rounds them to
 *    float. The results agree to well under 0.1 Pa.
 */
class BMP3XX : public I2CSensor {

    public:

        BMP3XX(
            uint8_t bus_num,
            uint8_t sda,
            uint8_t scl,
            uint8_t address = bmp3::ADDRESS_SECONDARY
        ) : I2CSensor(address, bus_num, sda, scl, 10000, 1000000) {}

        /**
         * @brief Configure oversampling and filtering. Applied immediately if
         * initialised, otherwise during init().
         * Defaults match Adafruit_BMP3XX: no oversampling, no filter. For a
         * weather station Bosch suggests x8 pressure / x1 temperature with a
         * light filter (COEF_3).
         */
        [[nodiscard]] bool configure(
            bmp3::Oversampling pressure = bmp3::Oversampling::X1,
            bmp3::Oversampling temperature = bmp3::Oversampling::X1,
            bmp3::IIRFilter filter = bmp3::IIRFilter::OFF
        );

        /** @brief Soft-reset the sensor and re-apply the current settings. */
        [[nodiscard]] bool reset(void);

        /**
         * @brief Trigger a measurement, wait for it, and cache it.
         * @return True on success, false otherwise - see lastError().
         */
        [[nodiscard]] bool update(void) override;

        /** @brief The last successful measurement. */
        bmp3::Reading reading(void) const;

        /** @brief Temperature in °C from the last successful update(), or NaN. */
        float getTemperature(void) const { return reading().temperature; }

        /** @brief Pressure in Pa from the last successful update(), or NaN. */
        float getPressure(void) const { return reading().pressure; }

        /**
         * @brief Altitude from the last successful update(), or NaN.
         * @param qnhHpa Sea-level pressure in hPa (eg: from the nearest METAR).
         */
        float getAltitude(float qnhHpa) const;

        /** @brief CHIP_ID read during init(): bmp3::CHIP_ID_BMP388 or bmp3::CHIP_ID_BMP390, or 0. */
        uint8_t chipId(void) const { return _chip_id.load(); }

        /**
         * Adafruit_BMP3XX-compatible names.
         * The read* methods take a fresh measurement, as Adafruit's do, and
         * return NaN if it fails.
         */

        /** @brief Same as init() - the bus, pins and address are set in the constructor. */
        [[nodiscard]] bool begin_I2C(void) { return init(); }
        [[nodiscard]] bool performReading(void) { return update(); }
        float readTemperature(void) { return update() ? getTemperature() : NAN; }
        float readPressure(void) { return update() ? getPressure() : NAN; }
        float readAltitude(float seaLevelHpa) { return update() ? getAltitude(seaLevelHpa) : NAN; }
        uint8_t chipID(void) const { return chipId(); }
        [[nodiscard]] bool setTemperatureOversampling(bmp3::Oversampling os);
        [[nodiscard]] bool setPressureOversampling(bmp3::Oversampling os);
        [[nodiscard]] bool setIIRFilterCoeff(bmp3::IIRFilter filter);

    protected:

        [[nodiscard]] bool deviceSpecificSetup() override;

    private:

        /** Write the current settings to the device. Requires the sensor lock. */
        [[nodiscard]] bool applySettings(void);

        /** Turn a set ERR register into lastError(). */
        [[nodiscard]] bool checkDeviceErrors(void);

        std::atomic<uint8_t> _chip_id{0};

        /** Soft-reset and wait for the device to come back. Requires the sensor lock. */
        [[nodiscard]] bool softReset(void);

        bmp3::Oversampling _osr_p = bmp3::Oversampling::X1;
        bmp3::Oversampling _osr_t = bmp3::Oversampling::X1;
        bmp3::IIRFilter _filter = bmp3::IIRFilter::OFF;
        bmp3::Calibration _calib{};

        mutable std::mutex _data_mutex;
        bmp3::Reading _reading;
};
