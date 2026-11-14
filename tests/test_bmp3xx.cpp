#include <AUnit.h>
#include "../src/BMP3XX/BMP3XX.hpp"
#include "FakeDevices.hpp"

namespace {
    constexpr uint8_t TEST_SDA = 21;
    constexpr uint8_t TEST_SCL = 22;

    /**
     * Synthetic calibration with realistic magnitudes:
     * T1=27500 T2=19000 T3=-7 P1=30000 P2=-2500 P3=35 P4=0 P5=987 P6=29000
     * P7=3 P8=-6 P9=17000 P10=8 P11=-60.
     * Expected values below were computed by an independent Python
     * implementation of the datasheet formulas (section 9).
     */
    const std::array<uint8_t, 21> CALIBRATION = {
        0x6C, 0x6B, 0x38, 0x4A, 0xF9, 0x30, 0x75, 0x3C, 0xF6, 0x23, 0x00,
        0xDB, 0x03, 0x48, 0x71, 0x03, 0xFA, 0x68, 0x42, 0x08, 0xC4
    };
    constexpr uint8_t CALIBRATION_CRC = 0x83;
    constexpr uint32_t RAW_TEMPERATURE = 8450000;
    constexpr double EXPECTED_TEMPERATURE = 24.9006897235;
    constexpr uint32_t RAW_PRESSURE = 6600000;
    constexpr double EXPECTED_PRESSURE = 101332.868626;
    constexpr uint32_t RAW_PRESSURE_LOW = 6400000;
    constexpr double EXPECTED_PRESSURE_LOW = 98792.344517;

    struct Fixture {
        FakeBMP3 device{bmp3::CHIP_ID_BMP390, CALIBRATION};
        BMP3XX sensor{0, TEST_SDA, TEST_SCL};

        Fixture() {
            resetFakeBuses();
            device.rawTemperature = RAW_TEMPERATURE;
            device.rawPressure = RAW_PRESSURE;
            I2CManager::getInstance().wire(0)->attach(bmp3::ADDRESS_SECONDARY, &device);
        }
    };
}


test(BMP3_CalibrationCrcMatchesAdafruit) {
    uint8_t raw[21];
    std::copy(CALIBRATION.begin(), CALIBRATION.end(), raw);
    assertEqual(adafruitTrimmingCrc(CALIBRATION), CALIBRATION_CRC);
    assertEqual(bmp3::calibrationCrc(raw, sizeof(raw)), CALIBRATION_CRC);

    // And on data that exercises every bit pattern.
    std::array<uint8_t, 21> ramp;
    for (size_t i = 0; i < ramp.size(); ++i) ramp[i] = static_cast<uint8_t>(i * 37 + 0x80);
    assertEqual(bmp3::calibrationCrc(ramp.data(), ramp.size()), adafruitTrimmingCrc(ramp));
}


test(BMP3_CalibrationParsing) {
    const bmp3::Calibration c = bmp3::Calibration::fromRegisters(CALIBRATION.data());
    assertNear(c.par_t1, 27500.0 * 256.0, 1e-9);
    assertNear(c.par_t3, -7.0 / 281474976710656.0, 1e-25);
    assertNear(c.par_p1, (30000.0 - 16384.0) / 1048576.0, 1e-12);
    assertNear(c.par_p2, (-2500.0 - 16384.0) / 536870912.0, 1e-15);
    assertNear(c.par_p5, 987.0 * 8.0, 1e-9);
    assertNear(c.par_p8, -6.0 / 32768.0, 1e-12);
    assertNear(c.par_p9, 17000.0 / 281474976710656.0, 1e-25);
}


test(BMP3_CompensationMatchesReference) {
    const bmp3::Calibration c = bmp3::Calibration::fromRegisters(CALIBRATION.data());
    const double t = bmp3::compensateTemperature(RAW_TEMPERATURE, c);
    assertNear(t, EXPECTED_TEMPERATURE, 1e-8);
    assertNear(bmp3::compensatePressure(RAW_PRESSURE, t, c), EXPECTED_PRESSURE, 1e-4);
    assertNear(bmp3::compensatePressure(RAW_PRESSURE_LOW, t, c), EXPECTED_PRESSURE_LOW, 1e-4);
}


test(BMP3_MeasurementTime) {
    // 234 + 392 + 2020 + 163 + 2020 µs at x1/x1.
    assertEqual(bmp3::measurementTimeUs(bmp3::Oversampling::X1, bmp3::Oversampling::X1), (uint32_t)4829);
    assertEqual(bmp3::measurementTimeUs(bmp3::Oversampling::X8, bmp3::Oversampling::X1), (uint32_t)18969);
}


test(BMP3_InitUsesAdafruitDefaults) {
    Fixture f;
    assertTrue(f.sensor.init());
    assertEqual(f.sensor.chipId(), bmp3::CHIP_ID_BMP390);
    assertEqual(f.device.resets, 1);
    assertEqual(f.device.lastWrite(bmp3::REG::OSR), 0x00);
    assertEqual(f.device.lastWrite(bmp3::REG::CONFIG), 0x00);
}


test(BMP3_AcceptsBMP388) {
    resetFakeBuses();
    FakeBMP3 device(bmp3::CHIP_ID_BMP388, CALIBRATION);
    I2CManager::getInstance().wire(0)->attach(bmp3::ADDRESS_PRIMARY, &device);
    BMP3XX sensor(0, TEST_SDA, TEST_SCL, bmp3::ADDRESS_PRIMARY);

    assertTrue(sensor.init());
    assertEqual(sensor.chipID(), bmp3::CHIP_ID_BMP388);
}


test(BMP3_RejectsOtherChips) {
    resetFakeBuses();
    FakeBMP3 bmp280(0x58, CALIBRATION);
    I2CManager::getInstance().wire(0)->attach(bmp3::ADDRESS_SECONDARY, &bmp280);
    BMP3XX sensor(0, TEST_SDA, TEST_SCL);

    assertFalse(sensor.init());
    assertEqual(toString(sensor.lastError()), "BAD_DEVICE_ID");
    assertEqual(I2CManager::getInstance().deviceCount(0), 0);
}


test(BMP3_RejectsCorruptCalibration) {
    Fixture f;
    f.device.regs[0x30] ^= 0x01;
    assertFalse(f.sensor.init());
    assertEqual(toString(f.sensor.lastError()), "CRC_MISMATCH");
}


test(BMP3_Configure) {
    Fixture f;
    assertTrue(f.sensor.init());
    assertTrue(f.sensor.configure(bmp3::Oversampling::X8, bmp3::Oversampling::X2, bmp3::IIRFilter::COEF_3));
    assertEqual(f.device.lastWrite(bmp3::REG::OSR), (0x01 << 3) | 0x03);
    assertEqual(f.device.lastWrite(bmp3::REG::CONFIG), 0x02 << 1);

    assertTrue(f.sensor.setPressureOversampling(bmp3::Oversampling::X32));
    assertEqual(f.device.lastWrite(bmp3::REG::OSR), (0x01 << 3) | 0x05);
    assertTrue(f.sensor.setIIRFilterCoeff(bmp3::IIRFilter::OFF));
    assertEqual(f.device.lastWrite(bmp3::REG::CONFIG), 0x00);
}


test(BMP3_ConfigureBeforeInitIsAppliedByInit) {
    Fixture f;
    assertTrue(f.sensor.configure(bmp3::Oversampling::X16, bmp3::Oversampling::X1, bmp3::IIRFilter::COEF_7));
    assertEqual(f.device.lastWrite(bmp3::REG::OSR), -1);
    assertTrue(f.sensor.init());
    assertEqual(f.device.lastWrite(bmp3::REG::OSR), 0x04);
    assertEqual(f.device.lastWrite(bmp3::REG::CONFIG), 0x03 << 1);
}


test(BMP3_Update) {
    Fixture f;
    assertTrue(f.sensor.init());
    assertTrue(f.sensor.update());

    assertEqual(f.device.measurements, 1);
    assertEqual(f.device.lastWrite(bmp3::REG::PWR_CTRL), 0x13);
    const bmp3::Reading r = f.sensor.reading();
    assertNear(r.temperature, (float)EXPECTED_TEMPERATURE, 1e-4f);
    assertNear(r.pressure, (float)EXPECTED_PRESSURE, 0.02f);

    // A fresh conversion each time - never the previous one.
    f.device.rawPressure = RAW_PRESSURE_LOW;
    assertNear(f.sensor.readPressure(), (float)EXPECTED_PRESSURE_LOW, 0.02f);
    assertEqual(f.device.measurements, 2);
}


test(BMP3_Altitude) {
    Fixture f;
    assertTrue(f.sensor.init());
    assertTrue(f.sensor.update());

    const float qnh = (float)EXPECTED_PRESSURE / 100.0f;
    assertNear(f.sensor.getAltitude(qnh), 0.0f, 0.01f);
    assertNear(f.sensor.readAltitude(qnh + 1.0f), 8.3f, 0.1f);
}


test(BMP3_DataNeverReady) {
    Fixture f;
    assertTrue(f.sensor.init());
    f.device.neverReady = true;

    assertFalse(f.sensor.update());
    assertEqual(toString(f.sensor.lastError()), "DATA_NOT_READY");
    assertTrue(std::isnan(f.sensor.getPressure()));
}


test(BMP3_ImpossibleValuesRejected) {
    Fixture f;
    assertTrue(f.sensor.init());
    f.device.rawPressure = 0;

    assertFalse(f.sensor.update());
    assertEqual(toString(f.sensor.lastError()), "OUT_OF_RANGE");
    assertTrue(std::isnan(f.sensor.readTemperature()));
}


test(BMP3_UpdateBeforeInit) {
    Fixture f;
    assertFalse(f.sensor.update());
    assertEqual(toString(f.sensor.lastError()), "NOT_INITIALIZED");
}


test(BMP3_ResetReappliesSettings) {
    Fixture f;
    assertTrue(f.sensor.configure(bmp3::Oversampling::X8));
    assertTrue(f.sensor.init());

    assertTrue(f.sensor.reset());
    assertEqual(f.device.resets, 2);
    assertEqual(f.device.regs[bmp3::REG::OSR], 0x03);
}
