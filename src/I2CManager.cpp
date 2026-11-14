#include "I2CManager.hpp"
#include "I2CSensor.hpp"
#include <algorithm>


I2CManager::I2CManager() {
    #ifdef I2CS_FAKE_WIRE
        static FakeWire w0;
        static FakeWire w1;
        _buses[0].wire = &w0;
        _buses[1].wire = &w1;
    #else
        // Use the core's global instances so we drive the same peripheral as
        // any other code calling Wire / Wire1 directly.
        _buses[0].wire = &Wire;
        #if SOC_I2C_NUM > 1
            _buses[1].wire = &Wire1;
        #endif
    #endif
}


uint8_t I2CManager::deviceCount(uint8_t bus_num) const {
    if (bus_num >= BUS_COUNT) return 0;
    std::lock_guard<std::mutex> guard(_registryMutex);
    return _buses[bus_num].devices.size();
}


uint32_t I2CManager::busClock(uint8_t bus_num) const {
    if (bus_num >= BUS_COUNT) return 0;
    std::lock_guard<std::mutex> guard(_registryMutex);
    return _buses[bus_num].current_clock;
}


i2cs::WireType* I2CManager::wire(uint8_t bus_num) {
    if (bus_num >= BUS_COUNT) return nullptr;
    return _buses[bus_num].wire;
}


i2cs::RecursiveMutex* I2CManager::busMutex(uint8_t bus_num) {
    if (bus_num >= BUS_COUNT) return nullptr;
    return &_buses[bus_num].mutex;
}


/**
 * @brief Registers a sensor with the manager.
 * @param sensor Ref to the sensor instance.
 * @return True if registration is successful, false otherwise.
 */
[[nodiscard]]
bool I2CManager::registerSensor(I2CSensor& sensor) {
    const uint8_t bus_num = sensor.getBusNum();
    if (bus_num >= BUS_COUNT || _buses[bus_num].wire == nullptr) {
        return false; // Bus not found
    }

    std::lock_guard<std::mutex> guard(_registryMutex);
    I2CBus& bus = _buses[bus_num];

    if (std::find(bus.devices.cbegin(), bus.devices.cend(), &sensor) != bus.devices.cend()) {
        return true; // Already registered (eg: init() called twice)
    }

    // A bus that is in use is pinned to the pins it was started on.
    if (bus.is_initialized &&
        (bus.sda_pin != sensor.getSdaPin() || bus.scl_pin != sensor.getSclPin())) {
        return false; // Pin mismatch
    }

    if (std::any_of(
            bus.devices.cbegin(),
            bus.devices.cend(),
            [&sensor](const I2CSensor* existing) {
                return existing->getAddress() == sensor.getAddress();
            }
        )
    ) {
        return false; // Address conflict
    }

    /**
     * Negotiate the clock: the fastest speed every device on the bus supports,
     * which must also be at least every device's minimum.
     */
    uint32_t new_clock = sensor.getMaxClock();
    for (const I2CSensor* existing : bus.devices) {
        new_clock = std::min(new_clock, existing->getMaxClock());
    }

    if (new_clock < sensor.getMinClock()) {
        return false; // Too slow for the new sensor
    }

    if (std::any_of(
            bus.devices.cbegin(),
            bus.devices.cend(),
            [new_clock](const I2CSensor* existing) {
                return new_clock < existing->getMinClock();
            }
        )
    ) {
        return false; // Too slow for a sensor already on the bus
    }

    {
        // Don't reconfigure the bus under a transaction in flight on another task.
        i2cs::Lock busLock(bus.mutex);

        if (!bus.is_initialized) {
            bus.sda_pin = sensor.getSdaPin();
            bus.scl_pin = sensor.getSclPin();
            bus.wire->begin(bus.sda_pin, bus.scl_pin);
            bus.is_initialized = true;
        }

        if (bus.current_clock != new_clock) {
            bus.wire->setClock(new_clock);
            bus.current_clock = new_clock;
        }
    }

    bus.devices.push_back(&sensor);
    sensor.attach(&bus);

    return true;
}


/**
 * @brief Unregisters a sensor from the manager.
 * @param sensor Ref to the sensor instance.
 * @return True if unregistration is successful, false otherwise.
 */
bool I2CManager::unregisterSensor(I2CSensor& sensor) {
    const uint8_t bus_num = sensor.getBusNum();
    if (bus_num >= BUS_COUNT) {
        return false; // Bus not found
    }

    std::lock_guard<std::mutex> guard(_registryMutex);
    I2CBus& bus = _buses[bus_num];

    auto it = std::find(bus.devices.begin(), bus.devices.end(), &sensor);
    if (it == bus.devices.end()) {
        return false; // Sensor not found on the bus
    }

    bus.devices.erase(it);
    sensor.detach();

    if (bus.devices.empty()) {
        resetBus(bus);
    }
    // The clock is left as is when other devices remain - it can only have
    // been capped by the departing sensor, and the remaining ones still accept it.

    return true;
}


/**
 * @brief Unregisters all sensors and resets every bus.
 */
void I2CManager::clear() {
    std::lock_guard<std::mutex> guard(_registryMutex);
    for (I2CBus& bus : _buses) {
        for (I2CSensor* sensor : bus.devices) {
            sensor->detach();
        }
        bus.devices.clear();
        resetBus(bus);
    }
}


void I2CManager::resetBus(I2CBus& bus) {
    if (bus.wire == nullptr) return;

    i2cs::Lock busLock(bus.mutex);
    if (bus.is_initialized) {
        bus.wire->end();
    }
    bus.is_initialized = false;
    bus.sda_pin = -1;
    bus.scl_pin = -1;
    bus.current_clock = 0;
}
