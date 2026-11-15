#include "SHT31D.hpp"

namespace {
    /** Min idle time between commands, and the soft-reset time, rounded up. */
    constexpr uint32_t COMMAND_GAP_MS = 1;
    constexpr uint32_t RESET_TIME_MS = 2;

    uint16_t toWord(const uint8_t* data) {
        return static_cast<uint16_t>((data[0] << 8) | data[1]);
    }
}


bool SHT31::deviceSpecificSetup() {
    if (!reset()) return false;

    // The status register is the only thing to read back - a CRC-valid
    // answer means a Sensirion device is really at this address.
    uint16_t status;
    if (!readStatus(status)) return false;

    return clearStatus();
}


bool SHT31::command(uint16_t cmd) {
    if (!writeCommand(cmd)) return false;
    i2cs::sleepMs(COMMAND_GAP_MS);
    return true;
}


bool SHT31::readWord(uint16_t& value) {
    uint8_t data[3];
    if (!readBytes(data, sizeof(data))) return false;
    if (data[2] != crc8(data, 2)) return fail(I2CError::CRC_MISMATCH);

    value = toWord(data);
    return succeed();
}


bool SHT31::reset(void) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);

    if (!writeCommand(sht3x::CMDS::SOFTRESET)) return false;
    i2cs::sleepMs(RESET_TIME_MS);
    return true;
}


bool SHT31::enableHeater(void) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);
    return command(sht3x::CMDS::HEATER_ON);
}


bool SHT31::disableHeater(void) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);
    return command(sht3x::CMDS::HEATER_OFF);
}


bool SHT31::isHeaterEnabled(bool& enabled) {
    uint16_t status;
    if (!readStatus(status)) return false;
    enabled = (status >> sht3x::STATUS_BITS::STATUS_HEATER) & 0x1;
    return true;
}


bool SHT31::readStatus(uint16_t& status) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);

    if (!command(sht3x::CMDS::READSTATUS)) return false;
    return readWord(status);
}


bool SHT31::clearStatus(void) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);
    return command(sht3x::CMDS::CLEARSTATUS);
}


void SHT31::setMode(sht3x::MEASUREMENT_MODE mode) {
    _mode.store(mode);
}


bool SHT31::update(void) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);
    if (!isInitialized()) return fail(I2CError::NOT_INITIALIZED);

    const sht3x::MEASUREMENT_MODE mode = _mode.load();
    if (!writeCommand(mode)) return false;

    // The bus is free for other sensors while this one converts.
    i2cs::sleepMs(sht3x::measurementTimeMs(mode));

    uint8_t data[6];
    if (!readBytes(data, sizeof(data))) return false;

    if (data[2] != crc8(data, 2) || data[5] != crc8(data + 3, 2)) {
        return fail(I2CError::CRC_MISMATCH);
    }

    sht3x::Reading fresh;
    fresh.temperature = -45.0f + 175.0f * (toWord(data) / 65535.0f);
    fresh.humidity = 100.0f * (toWord(data + 3) / 65535.0f);

    {
        std::lock_guard<std::mutex> guard(_data_mutex);
        _reading = fresh;
    }
    return succeed();
}


sht3x::Reading SHT31::reading(void) const {
    std::lock_guard<std::mutex> guard(_data_mutex);
    return _reading;
}
