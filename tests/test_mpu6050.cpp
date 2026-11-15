#include <AUnit.h>
#include "../src/MPU6050/MPU6050.hpp"
#include "FakeDevices.hpp"

namespace {
    constexpr uint8_t TEST_SDA = 21;
    constexpr uint8_t TEST_SCL = 22;
}


test(MPU6050_InitAppliesConstructorSettings) {
    resetFakeBuses();
    FakeMPU6050 device;
    I2CManager::getInstance().wire(0)->attach(MPU6050_ADDR, &device);
    MPU6050 sensor(0, TEST_SDA, TEST_SCL, mpu::DLPF_CFG::DLPF_42HZ, mpu::LSB_SENSITIVITY::LSB_16P4, mpu::ACCEL_RANGE::ACCEL_8G);

    assertTrue(sensor.init());
    assertEqual(device.lastWrite(mpu::MPU6050_REG::PWR_MGMT_1), (int)mpu::PWR_CLOCK_PLL_X);
    assertEqual(device.lastWrite(mpu::MPU6050_REG::GYRO_LPF), 0x03);
    assertEqual(device.lastWrite(mpu::MPU6050_REG::GYRO_SENS), 0x18);
    assertEqual(device.lastWrite(mpu::MPU6050_REG::ACCEL_CFG), 0x10);
}


test(MPU6050_WhoAmI) {
    resetFakeBuses();
    FakeMPU6050 clone(0x72);
    FakeMPU6050 other(0x11);
    I2CManager::getInstance().wire(0)->attach(MPU6050_ADDR, &clone);
    I2CManager::getInstance().wire(0)->attach(MPU6050_ADDR_ALT, &other);
    MPU6050 cloneSensor(0, TEST_SDA, TEST_SCL);
    MPU6050 otherSensor(0, TEST_SDA, TEST_SCL, mpu::DLPF_256HZ, mpu::LSB_65P5, mpu::ACCEL_2G, MPU6050_ADDR_ALT);

    assertTrue(cloneSensor.init());
    assertFalse(otherSensor.init());
    assertEqual(toString(otherSensor.lastError()), "BAD_DEVICE_ID");
}


test(MPU6050_ReadGyro) {
    resetFakeBuses();
    FakeMPU6050 device;
    I2CManager::getInstance().wire(0)->attach(MPU6050_ADDR, &device);
    MPU6050 sensor(0, TEST_SDA, TEST_SCL);
    assertTrue(sensor.init());

    device.setGyro(655, -1310, 0); // At 65.5 LSB per °/s
    MPU_XYZ gyro;
    assertTrue(sensor.readGyro(gyro));
    assertNear(gyro[0], 10.0f, 1e-4f);
    assertNear(gyro[1], -20.0f, 1e-4f);
    assertNear(gyro[2], 0.0f, 1e-4f);

    assertTrue(sensor.setSensitivity(mpu::LSB_SENSITIVITY::LSB_131P0));
    assertNear(sensor.readGyro()[0], 5.0f, 1e-4f);
}


test(MPU6050_CalibrateRemovesBias) {
    resetFakeBuses();
    FakeMPU6050 device;
    I2CManager::getInstance().wire(0)->attach(MPU6050_ADDR, &device);
    MPU6050 sensor(0, TEST_SDA, TEST_SCL);
    assertTrue(sensor.init());

    device.setGyro(131, -65, 33);
    assertTrue(sensor.calibrateGyro(30));

    const MPU_XYZ gyro = sensor.readGyroSampled(30);
    assertEqual(toString(sensor.lastError()), "NONE");
    for (float axis : gyro) assertNear(axis, 0.0f, 1e-4f);
}


test(MPU6050_AccelTiltTemperature) {
    resetFakeBuses();
    FakeMPU6050 device;
    I2CManager::getInstance().wire(0)->attach(MPU6050_ADDR, &device);
    MPU6050 sensor(0, TEST_SDA, TEST_SCL);
    assertTrue(sensor.init());

    device.setAccel(0, 0, 16384); // Flat, 1 g on Z at ±2 g
    MPU_XYZ g;
    assertTrue(sensor.readAccel(g));
    assertNear(g[2], 1.0f, 1e-4f);

    float pitch, roll;
    assertTrue(sensor.readTilt(pitch, roll));
    assertNear(pitch, 0.0f, 1e-3f);
    assertNear(roll, 0.0f, 1e-3f);

    device.setAccel(0, 11585, 11585); // Rolled 45°
    assertTrue(sensor.readTilt(pitch, roll));
    assertNear(roll, 45.0f, 0.01f);

    device.setTemperature(-340);
    float temperature;
    assertTrue(sensor.readTemperature(temperature));
    assertNear(temperature, 35.53f, 1e-3f);
}


test(MPU6050_NotInitialized) {
    resetFakeBuses();
    MPU6050 sensor(0, TEST_SDA, TEST_SCL);

    const MPU_XYZ gyro = sensor.readGyro();
    assertEqual(toString(sensor.lastError()), "NOT_INITIALIZED");
    for (float axis : gyro) assertEqual(axis, 0.0f);
    assertFalse(sensor.update());
}
