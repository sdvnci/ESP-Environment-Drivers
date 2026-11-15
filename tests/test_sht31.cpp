#include <AUnit.h>
#include "../src/SHT3X/SHT31D.hpp"
#include "FakeDevices.hpp"

namespace {
    constexpr uint8_t TEST_SDA = 21;
    constexpr uint8_t TEST_SCL = 22;
}


test(SHT31_Crc8DatasheetExample) {
    // SHT3x datasheet, section 4.12: CRC(0xBEEF) = 0x92.
    const uint8_t data[2] = {0xBE, 0xEF};
    assertEqual(I2CSensor::crc8(data, 2), 0x92);
    assertEqual(fakeCrc8(0xBE, 0xEF), 0x92);
}


test(SHT31_CommandCodes) {
    // Single shot, clock stretching disabled (datasheet table 9).
    assertEqual((uint16_t)sht3x::MEASUREMENT_MODE::HIGHREP, 0x2400);
    assertEqual((uint16_t)sht3x::MEASUREMENT_MODE::MEDREP, 0x240B);
    assertEqual((uint16_t)sht3x::MEASUREMENT_MODE::LOWREP, 0x2416);
}


test(SHT31_InitResetsAndClearsStatus) {
    resetFakeBuses();
    FakeSHT31 device;
    I2CManager::getInstance().wire(0)->attach(sht3x::ADDRESS_PRIMARY, &device);
    SHT31 sensor(0, TEST_SDA, TEST_SCL);

    assertTrue(sensor.init());
    assertEqual(device.resets, 1);
    assertEqual(device.status & (1u << 4), 0u);
    assertTrue(std::isnan(sensor.getTemperature()));
}


test(SHT31_Update) {
    resetFakeBuses();
    FakeSHT31 device;
    I2CManager::getInstance().wire(0)->attach(sht3x::ADDRESS_PRIMARY, &device);
    SHT31 sensor(0, TEST_SDA, TEST_SCL);
    assertTrue(sensor.init());

    assertTrue(sensor.update());
    assertEqual(device.lastCommand, (uint16_t)0x2400);
    const sht3x::Reading r = sensor.reading();
    assertNear(r.temperature, 25.0f, 0.01f);
    assertNear(r.humidity, 50.0f, 0.01f);

    // Full-scale endpoints of the conversion formulas.
    device.rawTemperature = 0x0000;
    device.rawHumidity = 0xFFFF;
    assertTrue(sensor.update());
    assertNear(sensor.getTemperature(), -45.0f, 0.001f);
    assertNear(sensor.getHumidity(), 100.0f, 0.001f);
}


test(SHT31_ModesSendTheirCommand) {
    resetFakeBuses();
    FakeSHT31 device;
    I2CManager::getInstance().wire(0)->attach(sht3x::ADDRESS_PRIMARY, &device);
    SHT31 sensor(0, TEST_SDA, TEST_SCL, sht3x::ADDRESS_PRIMARY, sht3x::MEASUREMENT_MODE::MEDREP);
    assertTrue(sensor.init());

    assertTrue(sensor.update());
    assertEqual(device.lastCommand, (uint16_t)0x240B);

    sensor.setMode(sht3x::MEASUREMENT_MODE::LOWREP);
    assertTrue(sensor.update());
    assertEqual(device.lastCommand, (uint16_t)0x2416);
}


test(SHT31_CrcMismatchKeepsLastReading) {
    resetFakeBuses();
    FakeSHT31 device;
    I2CManager::getInstance().wire(0)->attach(sht3x::ADDRESS_PRIMARY, &device);
    SHT31 sensor(0, TEST_SDA, TEST_SCL);
    assertTrue(sensor.init());
    assertTrue(sensor.update());

    device.corruptCrc = true;
    device.rawTemperature = 0x0000;
    assertFalse(sensor.update());
    assertEqual(toString(sensor.lastError()), "CRC_MISMATCH");
    assertNear(sensor.getTemperature(), 25.0f, 0.01f);
}


test(SHT31_CrcMismatchFailsInit) {
    resetFakeBuses();
    FakeSHT31 device;
    device.corruptCrc = true;
    I2CManager::getInstance().wire(0)->attach(sht3x::ADDRESS_PRIMARY, &device);
    SHT31 sensor(0, TEST_SDA, TEST_SCL);

    assertFalse(sensor.init());
    assertEqual(toString(sensor.lastError()), "CRC_MISMATCH");
}


test(SHT31_UpdateBeforeInit) {
    resetFakeBuses();
    SHT31 sensor(0, TEST_SDA, TEST_SCL);
    assertFalse(sensor.update());
    assertEqual(toString(sensor.lastError()), "NOT_INITIALIZED");
}


test(SHT31_Heater) {
    resetFakeBuses();
    FakeSHT31 device;
    I2CManager::getInstance().wire(0)->attach(sht3x::ADDRESS_PRIMARY, &device);
    SHT31 sensor(0, TEST_SDA, TEST_SCL);
    assertTrue(sensor.init());

    bool enabled = true;
    assertTrue(sensor.isHeaterEnabled(enabled));
    assertFalse(enabled);

    assertTrue(sensor.enableHeater());
    assertTrue(sensor.isHeaterEnabled(enabled));
    assertTrue(enabled);

    assertTrue(sensor.disableHeater());
    assertTrue(sensor.isHeaterEnabled(enabled));
    assertFalse(enabled);
}


test(SHT31_SecondaryAddress) {
    resetFakeBuses();
    FakeSHT31 device;
    I2CManager::getInstance().wire(0)->attach(sht3x::ADDRESS_SECONDARY, &device);
    SHT31 wrong(0, TEST_SDA, TEST_SCL);
    SHT31 right(0, TEST_SDA, TEST_SCL, sht3x::ADDRESS_SECONDARY);

    assertFalse(wrong.init());
    assertEqual(toString(wrong.lastError()), "NACK");
    assertTrue(right.init());
    assertTrue(right.update());
}
