#pragma once

#include "../I2CSensor.hpp"
#include "../Stats.hpp"
#include <array>

#define MPU6050_ADDR 0x68
#define MPU6050_ADDR_ALT 0x69
#define MPU6050_WHO_AM_I_REG 0x75
#define MPU6050_PWR_MGMT_1_REG 0x6B

namespace mpu {
    /**
     * Digital Low Pass Filter (DLPF) settings for the CONFIG register.
     * These determine the cutoff frequency of the gyro and accelerometer filter.
     */
    enum DLPF_CFG : uint8_t {
        DLPF_256HZ = 0x00,
        DLPF_188HZ = 0x01,
        DLPF_98HZ  = 0x02,
        DLPF_42HZ  = 0x03,
        DLPF_20HZ  = 0x04,
        DLPF_10HZ  = 0x05,
        DLPF_5HZ   = 0x06
    };

    /**
     * Gyro full-scale range settings for the GYRO_CONFIG register,
     * named after their sensitivity in LSB per degree/second.
     */
    enum LSB_SENSITIVITY : uint8_t {
        LSB_131P0 = 0x00, // ±250 °/s
        LSB_65P5 = 0x08,  // ±500 °/s
        LSB_32P8 = 0x10,  // ±1000 °/s
        LSB_16P4 = 0x18   // ±2000 °/s
    };

    /**
     * Accelerometer full-scale range settings for the ACCEL_CONFIG register.
     */
    enum ACCEL_RANGE : uint8_t {
        ACCEL_2G = 0x00,
        ACCEL_4G = 0x08,
        ACCEL_8G = 0x10,
        ACCEL_16G = 0x18
    };

    /**
     * MPU6050 register addresses.
     */
    enum MPU6050_REG : uint8_t {
        GYRO_LPF = 0x1A,
        GYRO_SENS = 0x1B,
        ACCEL_CFG = 0x1C,
        ACCEL_XOUT_H = 0x3B,
        TEMP_OUT_H = 0x41,
        GYRO_XOUT_H = 0x43,
        GYRO_XOUT_L = 0x44,
        GYRO_YOUT_H = 0x45,
        GYRO_YOUT_L = 0x46,
        GYRO_ZOUT_H = 0x47,
        GYRO_ZOUT_L = 0x48,
        PWR_MGMT_1 = 0x6B,
        WHO_AM_I = 0x75
    };

    /** PWR_MGMT_1: awake, clocked from the X gyro PLL (recommended over the internal oscillator). */
    constexpr uint8_t PWR_CLOCK_PLL_X = 0x01;

    /** @brief LSB per degree/second for a gyro range. */
    constexpr float gyroScale(LSB_SENSITIVITY s) {
        return s == LSB_131P0 ? 131.0f : s == LSB_65P5 ? 65.5f : s == LSB_32P8 ? 32.8f : 16.4f;
    }

    /** @brief LSB per g for an accelerometer range. */
    constexpr float accelScale(ACCEL_RANGE r) {
        return r == ACCEL_2G ? 16384.0f : r == ACCEL_4G ? 8192.0f : r == ACCEL_8G ? 4096.0f : 2048.0f;
    }

    /**
     * @brief Whether a WHO_AM_I value belongs to an MPU6050 or a register-compatible part.
     * 0x68 is the genuine MPU6050; 0x70 (MPU6500) and 0x72 are common on clone boards.
     */
    constexpr bool isKnownDeviceId(uint8_t id) {
        return id == 0x68 || id == 0x70 || id == 0x72;
    }
};


using MPU_XYZ = std::array<float, 3>;

/**
 * Driver for the InvenSense MPU6050 gyroscope and accelerometer.
 */
class MPU6050 : public I2CSensor {

    public:

        MPU6050(
            uint8_t bus_num,
            uint8_t sda,
            uint8_t scl,
            mpu::DLPF_CFG filter = mpu::DLPF_CFG::DLPF_256HZ,
            mpu::LSB_SENSITIVITY lsb = mpu::LSB_SENSITIVITY::LSB_65P5,
            mpu::ACCEL_RANGE accel = mpu::ACCEL_RANGE::ACCEL_2G,
            uint8_t address = MPU6050_ADDR
        ): I2CSensor(address, bus_num, sda, scl, 10000, 400000),
           _filter(filter), _gyro_range(lsb), _accel_range(accel) {}

        /**
         * This sensor is read on demand, so there is nothing to refresh.
         * @return Whether the sensor is initialised.
         */
        [[nodiscard]] bool update(void) override {
            return isInitialized() ? succeed() : fail(I2CError::NOT_INITIALIZED);
        }

        /** @brief Wake the sensor from sleep. */
        [[nodiscard]] bool powerOn(void);

        /** @brief Set the digital low pass filter. */
        [[nodiscard]] bool setLPF(mpu::DLPF_CFG newFilter = mpu::DLPF_CFG::DLPF_10HZ);

        /** @brief Set the gyro range. */
        [[nodiscard]] bool setSensitivity(mpu::LSB_SENSITIVITY newSensitivity = mpu::LSB_SENSITIVITY::LSB_65P5);

        /** @brief Set the accelerometer range. */
        [[nodiscard]] bool setAccelRange(mpu::ACCEL_RANGE range = mpu::ACCEL_RANGE::ACCEL_2G);

        /**
         * @brief Measure the gyro's resting bias and subtract it from later readings.
         * Keep the sensor still while this runs.
         * @return True if enough samples were read.
         */
        [[nodiscard]] bool calibrateGyro(uint16_t samples = i2cs::MAX_SAMPLES);

        /**
         * @brief Read the gyro, scaled and offset-corrected.
         * @param out Angular rate in degrees/second for X, Y, Z.
         * @return True on success, false otherwise - see lastError().
         */
        [[nodiscard]] bool readGyro(MPU_XYZ& out);

        /** @brief Read the gyro, or all zeros on failure - see lastError(). */
        MPU_XYZ readGyro(void);

        /**
         * @brief Read and average several gyro samples, discarding outliers.
         * @param out The filtered mean for each axis.
         * @param samples Number of samples to take, clamped to [30, MAX_SAMPLES].
         * @return True on success, false otherwise - see lastError().
         */
        [[nodiscard]] bool readGyroSampled(MPU_XYZ& out, uint16_t samples = i2cs::MAX_SAMPLES);

        /** @brief Sampled gyro read, or all zeros on failure - see lastError(). */
        MPU_XYZ readGyroSampled(uint16_t samples = i2cs::MAX_SAMPLES);

        /**
         * @brief Read the accelerometer.
         * @param out Acceleration in g for X, Y, Z.
         * @return True on success, false otherwise - see lastError().
         */
        [[nodiscard]] bool readAccel(MPU_XYZ& out);

        /**
         * @brief Tilt from level, from the accelerometer (sensor at rest).
         * @param pitch Rotation about the Y axis in degrees.
         * @param roll Rotation about the X axis in degrees.
         * @return True on success, false otherwise - see lastError().
         */
        [[nodiscard]] bool readTilt(float& pitch, float& roll);

        /**
         * @brief Read the die temperature.
         * @param out Temperature in °C - runs warmer than the air around it.
         */
        [[nodiscard]] bool readTemperature(float& out);

    protected:

        [[nodiscard]] bool deviceSpecificSetup() override;

    private:

        /** Read the gyro without offsets. Requires the sensor lock. */
        [[nodiscard]] bool readRawGyro(MPU_XYZ& out);

        /** Read three big-endian int16 words starting at a register. */
        [[nodiscard]] bool readXYZ(uint8_t reg, std::array<int16_t, 3>& out);

        mpu::DLPF_CFG _filter;
        mpu::LSB_SENSITIVITY _gyro_range;
        mpu::ACCEL_RANGE _accel_range;
        MPU_XYZ _gyro_offsets = {0.0f, 0.0f, 0.0f};
};
