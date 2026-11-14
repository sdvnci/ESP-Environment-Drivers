#include <AUnit.h>
#include "../src/SHT3X/SHT31D.hpp"
#include "../src/BMP3XX/BMP3XX.hpp"
#include "../src/MPU6050/MPU6050.hpp"
#include "FakeDevices.hpp"
#include <atomic>
#include <thread>
#include <vector>

namespace {
    constexpr uint8_t TEST_SDA = 21;
    constexpr uint8_t TEST_SCL = 22;
    constexpr int ITERATIONS = 40;

    const std::array<uint8_t, 21> CALIBRATION = {
        0x6C, 0x6B, 0x38, 0x4A, 0xF9, 0x30, 0x75, 0x3C, 0xF6, 0x23, 0x00,
        0xDB, 0x03, 0x48, 0x71, 0x03, 0xFA, 0x68, 0x42, 0x08, 0xC4
    };

    /** A failure seen on a worker thread, reported back to the test thread. */
    struct Failures {
        std::atomic<int> count{0};
        std::atomic<int> wrongValues{0};
    };
}


test(Concurrency_FakeBusDetectsInterleaving) {
    // Sanity check of the harness: two threads using the wire directly,
    // with no lock, overlapping on purpose.
    resetFakeBuses();
    FakeWire* wire = I2CManager::getInstance().wire(0);
    FakeRegisterDevice device;
    wire->attach(0x10, &device);
    wire->begin(TEST_SDA, TEST_SCL);

    std::atomic<bool> started{false};
    std::atomic<bool> intruded{false};

    std::thread first([&] {
        wire->beginTransmission(0x10);
        wire->write(0x00);
        started = true;
        while (!intruded) std::this_thread::yield();
        wire->endTransmission(true);
    });
    std::thread second([&] {
        while (!started) std::this_thread::yield();
        wire->beginTransmission(0x10);
        intruded = true;
        wire->endTransmission(true);
    });
    first.join();
    second.join();

    assertEqual(wire->interleavings(), 1);
    wire->end();
}


test(Concurrency_SensorsSharingABus) {
    resetFakeBuses();
    FakeWire* wire = I2CManager::getInstance().wire(0);
    FakeSHT31 shtDevice;
    FakeBMP3 bmpDevice(bmp3::CHIP_ID_BMP390, CALIBRATION);
    FakeMPU6050 mpuDevice;
    bmpDevice.rawTemperature = 8450000;
    bmpDevice.rawPressure = 6600000;
    mpuDevice.setGyro(655, 655, 655);
    wire->attach(sht3x::ADDRESS_PRIMARY, &shtDevice);
    wire->attach(bmp3::ADDRESS_SECONDARY, &bmpDevice);
    wire->attach(MPU6050_ADDR, &mpuDevice);

    SHT31 sht(0, TEST_SDA, TEST_SCL);
    BMP3XX bmp(0, TEST_SDA, TEST_SCL);
    MPU6050 gyro(0, TEST_SDA, TEST_SCL);
    assertTrue(sht.init());
    assertTrue(bmp.init());
    assertTrue(gyro.init());
    const int setupTransactions = wire->transactions();

    Failures failures;
    std::vector<std::thread> workers;

    workers.emplace_back([&] {
        for (int i = 0; i < ITERATIONS; ++i) {
            if (!sht.update()) { ++failures.count; continue; }
            const sht3x::Reading r = sht.reading();
            if (std::fabs(r.temperature - 25.0f) > 0.01f || std::fabs(r.humidity - 50.0f) > 0.01f) {
                ++failures.wrongValues;
            }
        }
    });
    workers.emplace_back([&] {
        for (int i = 0; i < ITERATIONS; ++i) {
            if (!bmp.update()) { ++failures.count; continue; }
            if (std::fabs(bmp.getPressure() - 101332.87f) > 0.05f) ++failures.wrongValues;
        }
    });
    // The gyro is read in a tight loop for as long as the slow sensors are
    // converting, so its transactions overlap theirs as much as possible.
    std::atomic<bool> slowDone{false};
    std::atomic<int> gyroReads{0};
    std::thread gyroWorker([&] {
        while (!slowDone) {
            MPU_XYZ g;
            if (!gyro.readGyro(g)) { ++failures.count; continue; }
            ++gyroReads;
            for (float axis : g) {
                if (std::fabs(axis - 10.0f) > 1e-4f) ++failures.wrongValues;
            }
        }
    });
    for (std::thread& worker : workers) worker.join();
    slowDone = true;
    gyroWorker.join();

    assertEqual(failures.count.load(), 0);
    assertEqual(failures.wrongValues.load(), 0);
    assertEqual(wire->interleavings(), 0);
    assertEqual(shtDevice.measurements, ITERATIONS);
    assertEqual(bmpDevice.measurements, ITERATIONS);
    assertMore(gyroReads.load(), ITERATIONS * 10);
    assertMore(wire->transactions() - setupTransactions, ITERATIONS * 10);
}


test(Concurrency_TightLoopsOnOneBus) {
    // Two identical drivers at different addresses, hammering the bus at
    // the same time. Without the bus lock their address writes and reads
    // cross over and each gets the other's (or a short) reply.
    resetFakeBuses();
    FakeWire* wire = I2CManager::getInstance().wire(0);
    FakeMPU6050 deviceA;
    FakeMPU6050 deviceB;
    deviceA.setGyro(655, 655, 655);     // 10 °/s
    deviceB.setGyro(-1310, -1310, -1310); // -20 °/s
    wire->attach(MPU6050_ADDR, &deviceA);
    wire->attach(MPU6050_ADDR_ALT, &deviceB);

    MPU6050 a(0, TEST_SDA, TEST_SCL);
    MPU6050 b(0, TEST_SDA, TEST_SCL, mpu::DLPF_256HZ, mpu::LSB_65P5, mpu::ACCEL_2G, MPU6050_ADDR_ALT);
    assertTrue(a.init());
    assertTrue(b.init());

    Failures failures;
    std::atomic<bool> go{false};
    auto worker = [&](MPU6050& sensor, float expected) {
        while (!go) std::this_thread::yield();
        for (int i = 0; i < 2000; ++i) {
            MPU_XYZ g;
            if (!sensor.readGyro(g)) { ++failures.count; continue; }
            if (std::fabs(g[0] - expected) > 1e-4f) ++failures.wrongValues;
        }
    };
    std::thread ta(worker, std::ref(a), 10.0f);
    std::thread tb(worker, std::ref(b), -20.0f);
    go = true;
    ta.join();
    tb.join();

    assertEqual(failures.count.load(), 0);
    assertEqual(failures.wrongValues.load(), 0);
    assertEqual(wire->interleavings(), 0);
}


test(Concurrency_SameSensorFromTwoTasks) {
    // Two tasks driving one SHT31: each trigger -> wait -> read must not be
    // split by the other's trigger, or the read would come back short.
    resetFakeBuses();
    FakeWire* wire = I2CManager::getInstance().wire(0);
    FakeSHT31 device;
    wire->attach(sht3x::ADDRESS_PRIMARY, &device);
    SHT31 sensor(0, TEST_SDA, TEST_SCL);
    assertTrue(sensor.init());

    Failures failures;
    auto worker = [&] {
        for (int i = 0; i < ITERATIONS / 2; ++i) {
            if (!sensor.update()) ++failures.count;
        }
    };
    std::thread a(worker);
    std::thread b(worker);
    a.join();
    b.join();

    assertEqual(failures.count.load(), 0);
    assertEqual(wire->interleavings(), 0);
    assertEqual(device.measurements, ITERATIONS);
}


test(Concurrency_ReadersSeeConsistentPairs) {
    // While one task updates, readers must never see temperature from one
    // measurement and humidity from another.
    resetFakeBuses();
    FakeWire* wire = I2CManager::getInstance().wire(0);
    FakeSHT31 device;
    wire->attach(sht3x::ADDRESS_PRIMARY, &device);
    SHT31 sensor(0, TEST_SDA, TEST_SCL);
    assertTrue(sensor.init());

    std::atomic<bool> done{false};
    std::atomic<int> torn{0};

    std::thread reader([&] {
        while (!done) {
            const sht3x::Reading r = sensor.reading();
            if (std::isnan(r.temperature)) continue;
            // The writer keeps raw humidity == raw temperature, so the
            // converted pair always satisfies this relation.
            const float expectedHumidity = (r.temperature + 45.0f) * 100.0f / 175.0f;
            if (std::fabs(r.humidity - expectedHumidity) > 0.01f) ++torn;
        }
    });

    for (uint16_t raw = 1000; raw < 1000 + ITERATIONS * 500; raw += 500) {
        device.rawTemperature = raw;
        device.rawHumidity = raw;
        (void)sensor.update();
    }
    done = true;
    reader.join();

    assertEqual(torn.load(), 0);
}
