#include "I2CSensor.hpp"


I2CSensor::I2CSensor(
    uint8_t addr,
    uint8_t bus_num,
    uint8_t sda,
    uint8_t scl,
    uint32_t min_clk,
    uint32_t max_clk)
    :
    _i2c_addr(addr),
    _bus_num(bus_num),
    _sda_pin(sda),
    _scl_pin(scl),
    _min_clock_hz(min_clk),
    _max_clock_hz(max_clk) {
    // Construct the manager before any sensor finishes constructing, so it is
    // destroyed after every sensor and ~I2CSensor can safely unregister.
    (void)I2CManager::getInstance();
}


I2CSensor::~I2CSensor() {
    if (_bus.load() != nullptr) {
        I2CManager::getInstance().unregisterSensor(*this);
    }
}


bool I2CSensor::init() {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);

    _is_initialized.store(false);

    if (!I2CManager::getInstance().registerSensor(*this)) {
        return fail(I2CError::REGISTRATION_FAILED);
    }

    if (!deviceSpecificSetup()) {
        // Free the address so a retry (or another device) can register cleanly.
        const I2CError reason = lastError();
        I2CManager::getInstance().unregisterSensor(*this);
        return fail(reason);
    }

    _is_initialized.store(true);
    return succeed();
}


i2cs::Lock I2CSensor::lockSensor(void) const {
    i2cs::Lock lock(_sensor_mutex, std::defer_lock);
    (void)lock.try_lock_for(i2cs::SENSOR_TIMEOUT);
    return lock;
}


uint8_t I2CSensor::crc8(const uint8_t* data, size_t len) {
    constexpr uint8_t POLYNOMIAL = 0x31;
    uint8_t crc = 0xFF;

    for (size_t j = 0; j < len; ++j) {
        crc ^= data[j];
        for (int i = 0; i < 8; ++i) {
            crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
        }
    }
    return crc;
}


I2CBus* I2CSensor::acquireBus(i2cs::Lock& lock) {
    I2CBus* bus = _bus.load();
    if (bus == nullptr) {
        fail(I2CError::NOT_INITIALIZED);
        return nullptr;
    }

    lock = i2cs::Lock(bus->mutex, std::defer_lock);
    if (!lock.try_lock_for(i2cs::BUS_TIMEOUT)) {
        fail(I2CError::BUS_TIMEOUT);
        return nullptr;
    }
    return bus;
}


bool I2CSensor::writeBytes(const uint8_t* data, size_t len) {
    i2cs::Lock lock;
    I2CBus* bus = acquireBus(lock);
    if (bus == nullptr) return false;

    bus->wire->beginTransmission(_i2c_addr);
    bus->wire->write(data, len);
    if (bus->wire->endTransmission(true) != 0) return fail(I2CError::NACK);

    return succeed();
}


bool I2CSensor::writeReg(uint8_t reg, uint8_t value) {
    const uint8_t buffer[2] = {reg, value};
    return writeBytes(buffer, sizeof(buffer));
}


bool I2CSensor::writeCommand(uint16_t cmd) {
    const uint8_t buffer[2] = {
        static_cast<uint8_t>(cmd >> 8),
        static_cast<uint8_t>(cmd & 0xFF)
    };
    return writeBytes(buffer, sizeof(buffer));
}


bool I2CSensor::readBytes(uint8_t* out, size_t len) {
    i2cs::Lock lock;
    I2CBus* bus = acquireBus(lock);
    if (bus == nullptr) return false;

    const size_t received = bus->wire->requestFrom(_i2c_addr, static_cast<uint8_t>(len));
    if (received != len) {
        while (bus->wire->available()) bus->wire->read();
        return fail(received == 0 ? I2CError::NACK : I2CError::SHORT_READ);
    }

    for (size_t i = 0; i < len; ++i) {
        out[i] = static_cast<uint8_t>(bus->wire->read());
    }
    return succeed();
}


bool I2CSensor::readRegs(uint8_t reg, uint8_t* out, size_t len) {
    i2cs::Lock lock;
    I2CBus* bus = acquireBus(lock);
    if (bus == nullptr) return false;

    // Repeated start: nobody else can grab the bus between address and read.
    bus->wire->beginTransmission(_i2c_addr);
    bus->wire->write(reg);
    if (bus->wire->endTransmission(false) != 0) return fail(I2CError::NACK);

    // Still holding the bus lock - the recursive mutex lets readBytes re-take it.
    return readBytes(out, len);
}
