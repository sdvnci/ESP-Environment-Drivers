#include "MPU6050.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace {
    /** Gyro start-up time is 30 ms - leave margin after waking. */
    constexpr uint32_t WAKE_TIME_MS = 50;
    constexpr uint16_t MIN_SAMPLES = 30;
    constexpr float RAD_TO_DEG_F = 57.2957795f;
}


bool MPU6050::deviceSpecificSetup() {
    uint8_t id = 0;
    if (!readReg(mpu::MPU6050_REG::WHO_AM_I, id)) return false;
    if (!mpu::isKnownDeviceId(id)) return fail(I2CError::BAD_DEVICE_ID);

    return powerOn()
        && setLPF(_filter)
        && setSensitivity(_gyro_range)
        && setAccelRange(_accel_range);
}


bool MPU6050::powerOn(void) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);

    if (!writeReg(mpu::MPU6050_REG::PWR_MGMT_1, mpu::PWR_CLOCK_PLL_X)) return false;
    i2cs::sleepMs(WAKE_TIME_MS);
    return true;
}


bool MPU6050::setLPF(mpu::DLPF_CFG newFilter) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);

    if (!writeReg(mpu::MPU6050_REG::GYRO_LPF, newFilter)) return false;
    _filter = newFilter;
    return true;
}


bool MPU6050::setSensitivity(mpu::LSB_SENSITIVITY newSensitivity) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);

    if (!writeReg(mpu::MPU6050_REG::GYRO_SENS, newSensitivity)) return false;
    _gyro_range = newSensitivity;
    return true;
}


bool MPU6050::setAccelRange(mpu::ACCEL_RANGE range) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);

    if (!writeReg(mpu::MPU6050_REG::ACCEL_CFG, range)) return false;
    _accel_range = range;
    return true;
}


bool MPU6050::readXYZ(uint8_t reg, std::array<int16_t, 3>& out) {
    uint8_t data[6];
    if (!readRegs(reg, data, sizeof(data))) return false;

    for (size_t i = 0; i < 3; ++i) {
        out[i] = static_cast<int16_t>((data[2 * i] << 8) | data[2 * i + 1]);
    }
    return true;
}


bool MPU6050::readRawGyro(MPU_XYZ& out) {
    std::array<int16_t, 3> raw;
    if (!readXYZ(mpu::MPU6050_REG::GYRO_XOUT_H, raw)) return false;

    const float scale = mpu::gyroScale(_gyro_range);
    for (size_t i = 0; i < 3; ++i) {
        out[i] = raw[i] / scale;
    }
    return true;
}


bool MPU6050::readGyro(MPU_XYZ& out) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);
    if (!isInitialized()) return fail(I2CError::NOT_INITIALIZED);

    MPU_XYZ raw;
    if (!readRawGyro(raw)) return false;

    for (size_t i = 0; i < 3; ++i) {
        out[i] = raw[i] - _gyro_offsets[i];
    }
    return true;
}


MPU_XYZ MPU6050::readGyro(void) {
    MPU_XYZ out = {0.0f, 0.0f, 0.0f};
    if (!readGyro(out)) return {0.0f, 0.0f, 0.0f};
    return out;
}


bool MPU6050::readGyroSampled(MPU_XYZ& out, uint16_t samples) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);
    if (!isInitialized()) return fail(I2CError::NOT_INITIALIZED);

    samples = std::max(MIN_SAMPLES, std::min(samples, i2cs::MAX_SAMPLES));

    std::array<std::vector<float>, 3> axes;
    for (std::vector<float>& axis : axes) axis.reserve(samples);

    for (uint16_t i = 0; i < samples; ++i) {
        MPU_XYZ sample;
        if (!readGyro(sample)) continue;
        for (size_t j = 0; j < 3; ++j) axes[j].push_back(sample[j]);
    }

    if (axes[0].size() < MIN_SAMPLES) {
        return lastError() == I2CError::NONE ? fail(I2CError::SHORT_READ) : false;
    }

    for (size_t i = 0; i < 3; ++i) {
        out[i] = stats::robustMean(axes[i]);
    }
    return succeed();
}


MPU_XYZ MPU6050::readGyroSampled(uint16_t samples) {
    MPU_XYZ out = {0.0f, 0.0f, 0.0f};
    if (!readGyroSampled(out, samples)) return {0.0f, 0.0f, 0.0f};
    return out;
}


bool MPU6050::calibrateGyro(uint16_t samples) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);

    const MPU_XYZ previous = _gyro_offsets;
    _gyro_offsets = {0.0f, 0.0f, 0.0f};

    MPU_XYZ bias;
    if (!readGyroSampled(bias, samples)) {
        _gyro_offsets = previous;
        return false;
    }

    _gyro_offsets = bias;
    return true;
}


bool MPU6050::readAccel(MPU_XYZ& out) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);
    if (!isInitialized()) return fail(I2CError::NOT_INITIALIZED);

    std::array<int16_t, 3> raw;
    if (!readXYZ(mpu::MPU6050_REG::ACCEL_XOUT_H, raw)) return false;

    const float scale = mpu::accelScale(_accel_range);
    for (size_t i = 0; i < 3; ++i) {
        out[i] = raw[i] / scale;
    }
    return true;
}


bool MPU6050::readTilt(float& pitch, float& roll) {
    MPU_XYZ g;
    if (!readAccel(g)) return false;

    pitch = std::atan2(-g[0], std::sqrt(g[1] * g[1] + g[2] * g[2])) * RAD_TO_DEG_F;
    roll = std::atan2(g[1], g[2]) * RAD_TO_DEG_F;
    return true;
}


bool MPU6050::readTemperature(float& out) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);
    if (!isInitialized()) return fail(I2CError::NOT_INITIALIZED);

    uint8_t data[2];
    if (!readRegs(mpu::MPU6050_REG::TEMP_OUT_H, data, sizeof(data))) return false;

    const int16_t raw = static_cast<int16_t>((data[0] << 8) | data[1]);
    out = raw / 340.0f + 36.53f;
    return true;
}
