#pragma once

#include "Platform.hpp"
#include <array>
#include <mutex>
#include <vector>

// Forward declaration to avoid circular dependency
class I2CSensor;

/**
 * State of one I2C bus (hardware peripheral).
 *
 * The bus mutex serialises transactions from every sensor on the bus, so two
 * tasks reading two different sensors on the same wires can never interleave
 * their bytes.
 */
struct I2CBus {
    i2cs::WireType* wire = nullptr;
    i2cs::RecursiveMutex mutex;
    bool is_initialized = false;
    int sda_pin = -1;
    int scl_pin = -1;
    uint32_t current_clock = 0;
    std::vector<I2CSensor*> devices;
};

class I2CManager {
public:

    /** Number of hardware buses managed. */
    static constexpr uint8_t BUS_COUNT = 2;

    /**
     * @brief Singleton instance getter for I2CManager.
     * @return Reference to the singleton instance of I2CManager.
     */
    [[nodiscard]]
    static I2CManager& getInstance() {
        static I2CManager instance;
        return instance;
    }

    I2CManager(I2CManager const&) = delete;
    void operator=(I2CManager const&) = delete;

    /**
     * @brief Returns the number of devices registered on a specific bus.
     * @param bus_num The bus number to check.
     * @return The number of devices on the bus, or 0 if the bus does not exist.
     */
    uint8_t deviceCount(uint8_t bus_num) const;

    /**
     * @brief Returns the clock speed negotiated for a bus.
     * @param bus_num The bus number to check.
     * @return The clock in Hz, or 0 if the bus does not exist or is not in use.
     */
    uint32_t busClock(uint8_t bus_num) const;

    /**
     * @brief Returns the wire driving a bus, or nullptr if the bus does not exist.
     * Use this to share the bus with code outside this library - hold
     * busMutex() around any transaction you do on it.
     */
    i2cs::WireType* wire(uint8_t bus_num);

    /**
     * @brief Returns the mutex guarding a bus, or nullptr if the bus does not exist.
     */
    i2cs::RecursiveMutex* busMutex(uint8_t bus_num);

    /**
     * @brief Registers a sensor with the manager.
     * Registering a sensor that is already registered is a no-op that succeeds.
     * @param sensor Ref to the sensor instance.
     * @return True if registration is successful, false otherwise.
     */
    [[nodiscard]]
    bool registerSensor(I2CSensor& sensor);

    /**
     * @brief Unregisters a sensor from the manager.
     * @param sensor Ref to the sensor instance.
     * @return True if unregistration is successful, false otherwise.
     */
    bool unregisterSensor(I2CSensor& sensor);

    /**
     * @brief Unregisters all sensors and resets every bus.
     * Not safe while other tasks are still using the sensors.
     */
    void clear();


private:
    std::array<I2CBus, BUS_COUNT> _buses;

    /** Guards bus membership and configuration (not transactions - see I2CBus::mutex). */
    mutable std::mutex _registryMutex;

    I2CManager();

    void resetBus(I2CBus& bus);
};
