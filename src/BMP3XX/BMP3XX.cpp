#include "BMP3XX.hpp"
#include "../EnvMath.hpp"

namespace {
    /** Power-on / soft-reset start-up time is 2 ms - leave margin. */
    constexpr uint32_t STARTUP_TIME_MS = 5;
    /** Extra time to poll for data-ready past the computed conversion time. */
    constexpr uint32_t READY_POLL_MS = 2;
    constexpr uint8_t READY_POLL_ATTEMPTS = 25;

    /** Plausibility window, a little wider than the datasheet's operating range. */
    constexpr double MIN_TEMPERATURE_C = -45.0;
    constexpr double MAX_TEMPERATURE_C = 90.0;
    constexpr double MIN_PRESSURE_PA = 25000.0;
    constexpr double MAX_PRESSURE_PA = 130000.0;

    uint16_t u16(const uint8_t* raw) {
        return static_cast<uint16_t>(raw[0] | (raw[1] << 8));
    }

    int16_t s16(const uint8_t* raw) {
        return static_cast<int16_t>(u16(raw));
    }

    int8_t s8(const uint8_t* raw) {
        return static_cast<int8_t>(raw[0]);
    }

    uint32_t u24(const uint8_t* raw) {
        return static_cast<uint32_t>(raw[0]) |
               (static_cast<uint32_t>(raw[1]) << 8) |
               (static_cast<uint32_t>(raw[2]) << 16);
    }
}


bmp3::Calibration bmp3::Calibration::fromRegisters(const uint8_t* raw) {
    Calibration c;
    c.par_t1 = u16(raw + 0) / std::pow(2.0, -8);
    c.par_t2 = u16(raw + 2) / std::pow(2.0, 30);
    c.par_t3 = s8(raw + 4) / std::pow(2.0, 48);
    c.par_p1 = (s16(raw + 5) - 16384.0) / std::pow(2.0, 20);
    c.par_p2 = (s16(raw + 7) - 16384.0) / std::pow(2.0, 29);
    c.par_p3 = s8(raw + 9) / std::pow(2.0, 32);
    c.par_p4 = s8(raw + 10) / std::pow(2.0, 37);
    c.par_p5 = u16(raw + 11) / std::pow(2.0, -3);
    c.par_p6 = u16(raw + 13) / std::pow(2.0, 6);
    c.par_p7 = s8(raw + 15) / std::pow(2.0, 8);
    c.par_p8 = s8(raw + 16) / std::pow(2.0, 15);
    c.par_p9 = s16(raw + 17) / std::pow(2.0, 48);
    c.par_p10 = s8(raw + 19) / std::pow(2.0, 48);
    c.par_p11 = s8(raw + 20) / std::pow(2.0, 65);
    return c;
}


double bmp3::compensateTemperature(uint32_t raw, const Calibration& c) {
    const double d1 = static_cast<double>(raw) - c.par_t1;
    const double d2 = d1 * c.par_t2;
    return d2 + (d1 * d1) * c.par_t3;
}


double bmp3::compensatePressure(uint32_t raw, double t_lin, const Calibration& c) {
    const double t2 = t_lin * t_lin;
    const double t3 = t2 * t_lin;
    const double p = static_cast<double>(raw);

    const double offset = c.par_p5 + c.par_p6 * t_lin + c.par_p7 * t2 + c.par_p8 * t3;
    const double sensitivity = p * (c.par_p1 + c.par_p2 * t_lin + c.par_p3 * t2 + c.par_p4 * t3);
    const double nonlinear = (p * p) * (c.par_p9 + c.par_p10 * t_lin) + (p * p * p) * c.par_p11;

    return offset + sensitivity + nonlinear;
}


uint8_t bmp3::calibrationCrc(const uint8_t* raw, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= raw[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x1D) : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc ^ 0xFF;
}


uint32_t bmp3::measurementTimeUs(Oversampling pressure, Oversampling temperature) {
    const uint32_t osrP = 1u << static_cast<uint8_t>(pressure);
    const uint32_t osrT = 1u << static_cast<uint8_t>(temperature);
    return 234 + (392 + osrP * 2020) + (163 + osrT * 2020);
}


bool BMP3XX::deviceSpecificSetup() {
    uint8_t id = 0;
    if (!readReg(bmp3::REG::CHIP_ID, id)) return false;
    if (id != bmp3::CHIP_ID_BMP388 && id != bmp3::CHIP_ID_BMP390) {
        return fail(I2CError::BAD_DEVICE_ID);
    }
    _chip_id.store(id);

    if (!softReset()) return false;

    // The stored CRC sits just before the calibration block - read both at once.
    uint8_t raw[1 + bmp3::CALIB_LEN];
    if (!readRegs(bmp3::REG::CALIB_CRC, raw, sizeof(raw))) return false;
    if (raw[0] != bmp3::calibrationCrc(raw + 1, bmp3::CALIB_LEN)) {
        return fail(I2CError::CRC_MISMATCH);
    }
    _calib = bmp3::Calibration::fromRegisters(raw + 1);

    return applySettings();
}


bool BMP3XX::softReset(void) {
    // The device only takes a command once the previous one has finished.
    uint8_t status = 0;
    for (uint8_t attempt = 0; ; ++attempt) {
        if (!readReg(bmp3::REG::STATUS, status)) return false;
        if (status & bmp3::STATUS_CMD_READY) break;
        if (attempt + 1 >= READY_POLL_ATTEMPTS) return fail(I2CError::DATA_NOT_READY);
        i2cs::sleepMs(READY_POLL_MS);
    }

    if (!writeReg(bmp3::REG::CMD, bmp3::CMD_SOFTRESET)) return false;
    i2cs::sleepMs(STARTUP_TIME_MS);
    return checkDeviceErrors();
}


bool BMP3XX::applySettings(void) {
    const uint8_t osr = static_cast<uint8_t>(_osr_p) | (static_cast<uint8_t>(_osr_t) << 3);
    if (!writeReg(bmp3::REG::OSR, osr)) return false;
    if (!writeReg(bmp3::REG::CONFIG, static_cast<uint8_t>(_filter) << 1)) return false;
    return checkDeviceErrors();
}


bool BMP3XX::checkDeviceErrors(void) {
    uint8_t err = 0;
    if (!readReg(bmp3::REG::ERR, err)) return false;
    if (err & (bmp3::ERR_FATAL | bmp3::ERR_CMD | bmp3::ERR_CONF)) {
        return fail(I2CError::DEVICE_ERROR);
    }
    return succeed();
}


bool BMP3XX::configure(bmp3::Oversampling pressure, bmp3::Oversampling temperature, bmp3::IIRFilter filter) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);

    _osr_p = pressure;
    _osr_t = temperature;
    _filter = filter;

    if (!isInitialized()) return succeed(); // Applied by init()
    return applySettings();
}


bool BMP3XX::reset(void) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);
    if (!isInitialized()) return fail(I2CError::NOT_INITIALIZED);

    if (!softReset()) return false;
    return applySettings();
}


bool BMP3XX::update(void) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);
    if (!isInitialized()) return fail(I2CError::NOT_INITIALIZED);

    const uint8_t pwr = bmp3::PWR_PRESS_EN | bmp3::PWR_TEMP_EN | bmp3::PWR_MODE_FORCED;
    if (!writeReg(bmp3::REG::PWR_CTRL, pwr)) return false;

    // The bus is free for other sensors while this one converts.
    i2cs::sleepMs(bmp3::measurementTimeUs(_osr_p, _osr_t) / 1000 + 1);

    const uint8_t ready = bmp3::STATUS_PRESS_READY | bmp3::STATUS_TEMP_READY;
    uint8_t status = 0;
    for (uint8_t attempt = 0; ; ++attempt) {
        if (!readReg(bmp3::REG::STATUS, status)) return false;
        if ((status & ready) == ready) break;
        if (attempt + 1 >= READY_POLL_ATTEMPTS) {
            (void)checkDeviceErrors(); // Surface a device fault in lastError() if there is one
            return fail(I2CError::DATA_NOT_READY);
        }
        i2cs::sleepMs(READY_POLL_MS);
    }

    uint8_t data[bmp3::DATA_LEN];
    if (!readRegs(bmp3::REG::DATA, data, sizeof(data))) return false;

    const double temperature = bmp3::compensateTemperature(u24(data + 3), _calib);
    const double pressure = bmp3::compensatePressure(u24(data), temperature, _calib);

    // No CRC on this chip, so a corrupted transfer shows up as an impossible value.
    if (!(temperature >= MIN_TEMPERATURE_C && temperature <= MAX_TEMPERATURE_C) ||
        !(pressure >= MIN_PRESSURE_PA && pressure <= MAX_PRESSURE_PA)) {
        return fail(I2CError::OUT_OF_RANGE);
    }

    {
        std::lock_guard<std::mutex> guard(_data_mutex);
        _reading.temperature = static_cast<float>(temperature);
        _reading.pressure = static_cast<float>(pressure);
    }
    return succeed();
}


bmp3::Reading BMP3XX::reading(void) const {
    std::lock_guard<std::mutex> guard(_data_mutex);
    return _reading;
}


float BMP3XX::getAltitude(float qnhHpa) const {
    return env::altitude(reading().pressure, qnhHpa);
}


bool BMP3XX::setTemperatureOversampling(bmp3::Oversampling os) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);
    return configure(_osr_p, os, _filter);
}


bool BMP3XX::setPressureOversampling(bmp3::Oversampling os) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);
    return configure(os, _osr_t, _filter);
}


bool BMP3XX::setIIRFilterCoeff(bmp3::IIRFilter filter) {
    i2cs::Lock lock = lockSensor();
    if (!lock.owns_lock()) return fail(I2CError::BUS_TIMEOUT);
    return configure(_osr_p, _osr_t, filter);
}
