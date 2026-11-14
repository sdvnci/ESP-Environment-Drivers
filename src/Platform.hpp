#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <chrono>
#include <cstdint>
#include <mutex>

/**
 * Platform shims so the drivers build both for the ESP32 (FreeRTOS) and for
 * host-side unit tests (EpoxyDuino).
 *
 * Host tests define I2CS_FAKE_WIRE and put a FakeWire.hpp on the include path,
 * which swaps the hardware TwoWire for a scriptable fake bus.
 */
#ifdef I2CS_FAKE_WIRE
    #include "FakeWire.hpp"
#endif

#ifndef EPOXY_DUINO
    #include <freertos/FreeRTOS.h>
    #include <freertos/semphr.h>
    #include <freertos/task.h>
#endif

namespace i2cs {

    #ifdef I2CS_FAKE_WIRE
        using WireType = FakeWire;
    #else
        using WireType = TwoWire;
    #endif

    /** Max time to wait for exclusive access to a bus for one transaction. */
    constexpr std::chrono::milliseconds BUS_TIMEOUT = std::chrono::milliseconds(100);

    /**
     * Max time to wait for exclusive access to a sensor for one operation.
     * Longer than BUS_TIMEOUT since an operation (eg: trigger -> wait -> read)
     * can span several bus transactions and a conversion delay.
     */
    constexpr std::chrono::milliseconds SENSOR_TIMEOUT = std::chrono::milliseconds(1000);

    constexpr uint16_t MAX_SAMPLES = 100;

    /**
     * @brief Sleep the calling task, yielding the CPU (and the bus) to others.
     * @param ms Milliseconds to sleep.
     */
    inline void sleepMs(uint32_t ms) {
        #ifdef EPOXY_DUINO
            delay(ms);
        #else
            vTaskDelay(pdMS_TO_TICKS(ms));
        #endif
    }

    /**
     * Recursive mutex with a timed lock, usable with std::unique_lock.
     *
     * On the ESP32 this is a FreeRTOS recursive mutex (statically allocated so
     * it is safe to construct in global constructors, and with priority
     * inheritance). On the host it wraps std::recursive_timed_mutex.
     */
    class RecursiveMutex {
        public:
            RecursiveMutex() {
                #ifndef EPOXY_DUINO
                    _handle = xSemaphoreCreateRecursiveMutexStatic(&_buffer);
                #endif
            }

            RecursiveMutex(const RecursiveMutex&) = delete;
            RecursiveMutex& operator=(const RecursiveMutex&) = delete;

            void lock() {
                #ifdef EPOXY_DUINO
                    _mutex.lock();
                #else
                    xSemaphoreTakeRecursive(_handle, portMAX_DELAY);
                #endif
            }

            bool try_lock() {
                #ifdef EPOXY_DUINO
                    return _mutex.try_lock();
                #else
                    return xSemaphoreTakeRecursive(_handle, 0) == pdTRUE;
                #endif
            }

            template <typename Rep, typename Period>
            bool try_lock_for(const std::chrono::duration<Rep, Period>& timeout) {
                #ifdef EPOXY_DUINO
                    return _mutex.try_lock_for(timeout);
                #else
                    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
                    return xSemaphoreTakeRecursive(_handle, pdMS_TO_TICKS(ms)) == pdTRUE;
                #endif
            }

            void unlock() {
                #ifdef EPOXY_DUINO
                    _mutex.unlock();
                #else
                    xSemaphoreGiveRecursive(_handle);
                #endif
            }

        private:
            #ifdef EPOXY_DUINO
                std::recursive_timed_mutex _mutex;
            #else
                StaticSemaphore_t _buffer;
                SemaphoreHandle_t _handle = nullptr;
            #endif
    };

    using Lock = std::unique_lock<RecursiveMutex>;
};

/**
 * Result of the most recent operation on a sensor.
 */
enum class I2CError : uint8_t {
    NONE = 0,
    /** The sensor is not registered on a bus - call init() first. */
    NOT_INITIALIZED,
    /** init() could not register the sensor: unknown bus, pin mismatch, address conflict or clock conflict. */
    REGISTRATION_FAILED,
    /** Timed out waiting for another task to release the bus or the sensor. */
    BUS_TIMEOUT,
    /** The device did not acknowledge its address or the data. */
    NACK,
    /** The device returned fewer bytes than requested. */
    SHORT_READ,
    /** The device returned data whose checksum did not match. */
    CRC_MISMATCH,
    /** The device at the address is not the expected chip. */
    BAD_DEVICE_ID,
    /** The device did not finish a conversion in time. */
    DATA_NOT_READY,
    /** The device reported an internal error. */
    DEVICE_ERROR,
    /** A reading was outside the sensor's physical range - likely bus corruption. */
    OUT_OF_RANGE,
};

/**
 * @brief Human readable name for an I2CError, for logs and status reports.
 */
inline const char* toString(I2CError err) {
    switch (err) {
        case I2CError::NONE: return "NONE";
        case I2CError::NOT_INITIALIZED: return "NOT_INITIALIZED";
        case I2CError::REGISTRATION_FAILED: return "REGISTRATION_FAILED";
        case I2CError::BUS_TIMEOUT: return "BUS_TIMEOUT";
        case I2CError::NACK: return "NACK";
        case I2CError::SHORT_READ: return "SHORT_READ";
        case I2CError::CRC_MISMATCH: return "CRC_MISMATCH";
        case I2CError::BAD_DEVICE_ID: return "BAD_DEVICE_ID";
        case I2CError::DATA_NOT_READY: return "DATA_NOT_READY";
        case I2CError::DEVICE_ERROR: return "DEVICE_ERROR";
        case I2CError::OUT_OF_RANGE: return "OUT_OF_RANGE";
    }
    return "UNKNOWN";
}
