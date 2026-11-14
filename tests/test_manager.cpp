#include <AUnit.h>
#include "../src/I2CManager.hpp"
#include "../src/I2CSensor.hpp"
#include "FakeDevices.hpp"

namespace {
    /** Sensor whose setup optionally probes the bus, for registration tests. */
    class MockSensor : public I2CSensor {
        public:
            MockSensor(uint8_t addr, uint8_t bus, uint8_t sda, uint8_t scl,
                       uint32_t min_clk, uint32_t max_clk, bool probe = false)
                : I2CSensor(addr, bus, sda, scl, min_clk, max_clk), _probe(probe) {}

            bool update() override { return isInitialized(); }

        protected:
            bool deviceSpecificSetup() override {
                if (!_probe) return true;
                uint8_t value;
                return readReg(0x00, value);
            }

        private:
            bool _probe;
    };
}


test(Manager_Singleton) {
    assertEqual(&I2CManager::getInstance(), &I2CManager::getInstance());
}


test(Manager_RegisterStartsBus) {
    resetFakeBuses();
    MockSensor sensor(0x48, 0, 21, 22, 100000, 400000);

    assertTrue(sensor.init());
    assertTrue(sensor.isInitialized());
    assertEqual(toString(sensor.lastError()), "NONE");

    FakeWire* wire = I2CManager::getInstance().wire(0);
    assertTrue(wire->isBegun());
    assertEqual(wire->sda(), 21);
    assertEqual(wire->clock(), (uint32_t)400000);
    assertEqual(I2CManager::getInstance().deviceCount(0), 1);
}


test(Manager_AddressConflictOnlyWithinABus) {
    resetFakeBuses();
    MockSensor first(0x48, 0, 21, 22, 100000, 400000);
    MockSensor clash(0x48, 0, 21, 22, 100000, 400000);
    MockSensor otherBus(0x48, 1, 16, 17, 100000, 400000);

    assertTrue(first.init());
    assertFalse(clash.init());
    assertEqual(toString(clash.lastError()), "REGISTRATION_FAILED");
    assertTrue(otherBus.init());
}


test(Manager_PinMismatch) {
    resetFakeBuses();
    MockSensor first(0x48, 0, 21, 22, 100000, 400000);
    MockSensor wrongPins(0x49, 0, 19, 20, 100000, 400000);

    assertTrue(first.init());
    assertFalse(wrongPins.init());
}


test(Manager_ClockNegotiation) {
    resetFakeBuses();
    I2CManager& manager = I2CManager::getInstance();
    MockSensor fast(0x48, 1, 21, 22, 100000, 1000000);
    MockSensor slow(0x49, 1, 21, 22, 100000, 400000);
    MockSensor needsFast(0x4A, 1, 21, 22, 500000, 1000000);

    assertTrue(fast.init());
    assertEqual(manager.busClock(1), (uint32_t)1000000);

    assertTrue(slow.init());
    assertEqual(manager.busClock(1), (uint32_t)400000);
    assertEqual(manager.wire(1)->clock(), (uint32_t)400000);

    // 400 kHz is below its minimum - rejected, and the bus is left alone.
    assertFalse(needsFast.init());
    assertEqual(manager.busClock(1), (uint32_t)400000);
}


test(Manager_RejectsSensorTooFastForExisting) {
    resetFakeBuses();
    MockSensor needsFast(0x48, 0, 21, 22, 500000, 1000000);
    MockSensor slow(0x49, 0, 21, 22, 10000, 100000);

    assertTrue(needsFast.init());
    assertFalse(slow.init());
    assertEqual(I2CManager::getInstance().busClock(0), (uint32_t)1000000);
}


test(Manager_ReinitDoesNotConflictWithItself) {
    resetFakeBuses();
    MockSensor sensor(0x48, 0, 21, 22, 100000, 400000);

    assertTrue(sensor.init());
    assertTrue(sensor.init());
    assertEqual(I2CManager::getInstance().deviceCount(0), 1);
}


test(Manager_UnregisterLastSensorEndsBus) {
    resetFakeBuses();
    I2CManager& manager = I2CManager::getInstance();
    MockSensor sensor(0x48, 0, 21, 22, 100000, 400000);

    assertTrue(sensor.init());
    assertTrue(manager.unregisterSensor(sensor));
    assertFalse(sensor.isInitialized());
    assertFalse(manager.wire(0)->isBegun());
    assertEqual(manager.busClock(0), (uint32_t)0);
    assertFalse(manager.unregisterSensor(sensor));
}


test(Manager_DestructorUnregisters) {
    resetFakeBuses();
    {
        MockSensor scoped(0x48, 0, 21, 22, 100000, 400000);
        assertTrue(scoped.init());
        assertEqual(I2CManager::getInstance().deviceCount(0), 1);
    }
    assertEqual(I2CManager::getInstance().deviceCount(0), 0);

    // The address is free again.
    MockSensor again(0x48, 0, 21, 22, 100000, 400000);
    assertTrue(again.init());
}


test(Manager_FailedSetupFreesAddress) {
    resetFakeBuses();
    // Nothing attached at 0x48, so the probe is not acknowledged.
    MockSensor missing(0x48, 0, 21, 22, 100000, 400000, true);

    assertFalse(missing.init());
    assertEqual(toString(missing.lastError()), "NACK");
    assertFalse(missing.isInitialized());
    assertEqual(I2CManager::getInstance().deviceCount(0), 0);

    MockSensor present(0x48, 0, 21, 22, 100000, 400000);
    assertTrue(present.init());
}


test(Manager_UnknownBus) {
    resetFakeBuses();
    MockSensor sensor(0x48, 7, 21, 22, 100000, 400000);

    assertFalse(sensor.init());
    assertEqual(toString(sensor.lastError()), "REGISTRATION_FAILED");
}


test(Manager_ClearDetachesSensors) {
    resetFakeBuses();
    MockSensor sensor(0x48, 0, 21, 22, 100000, 400000, true);
    FakeRegisterDevice device;
    I2CManager::getInstance().wire(0)->attach(0x48, &device);

    assertTrue(sensor.init());
    I2CManager::getInstance().clear();
    assertFalse(sensor.isInitialized());
    assertEqual(I2CManager::getInstance().deviceCount(0), 0);
}


test(ErrorNames) {
    assertEqual(toString(I2CError::NONE), "NONE");
    assertEqual(toString(I2CError::CRC_MISMATCH), "CRC_MISMATCH");
    assertEqual(toString(I2CError::OUT_OF_RANGE), "OUT_OF_RANGE");
}
