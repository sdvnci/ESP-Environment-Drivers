#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

/**
 * A device on the fake bus. Calls arrive with the bus's internal lock held,
 * so device models don't need their own locking.
 */
class FakeDevice {
    public:
        virtual ~FakeDevice() = default;

        /**
         * @brief One write transaction (without the address byte).
         * @return False to NACK the data.
         */
        virtual bool onWrite(const std::vector<uint8_t>& bytes) = 0;

        /**
         * @brief Supply up to `len` bytes for a read.
         * @return The number of bytes supplied.
         */
        virtual size_t onRead(uint8_t* out, size_t len) = 0;
};


/**
 * Stand-in for TwoWire on the host, with the subset of its API the drivers use.
 *
 * Besides routing bytes to FakeDevices it watches for interleaving: a
 * transaction from one thread starting while another thread's transaction
 * (write, repeated start, read until drained) is still in flight. That is
 * exactly the corruption the bus mutex exists to prevent.
 */
class FakeWire {
    public:
        bool begin(int sda, int scl) {
            std::lock_guard<std::mutex> guard(_mutex);
            _begun = true;
            _sda = sda;
            _scl = scl;
            return true;
        }

        void end(void) {
            std::lock_guard<std::mutex> guard(_mutex);
            _begun = false;
        }

        void setClock(uint32_t hz) {
            std::lock_guard<std::mutex> guard(_mutex);
            _clock = hz;
        }

        void beginTransmission(uint8_t address) {
            std::lock_guard<std::mutex> guard(_mutex);
            startTransaction();
            _tx_address = address;
            _tx.clear();
        }

        size_t write(uint8_t byte) {
            std::this_thread::yield(); // Widen the window for a racing thread.
            std::lock_guard<std::mutex> guard(_mutex);
            _tx.push_back(byte);
            return 1;
        }

        size_t write(const uint8_t* data, size_t len) {
            for (size_t i = 0; i < len; ++i) write(data[i]);
            return len;
        }

        /**
         * @return 0 on success, 2 if the address was not acknowledged, 3 if the data was not.
         */
        uint8_t endTransmission(bool sendStop = true) {
            std::lock_guard<std::mutex> guard(_mutex);
            ++_transactions;

            uint8_t result = 0;
            auto it = _devices.find(_tx_address);
            if (!_begun || it == _devices.end()) {
                result = 2;
            } else if (!it->second->onWrite(_tx)) {
                result = 3;
            }

            if (sendStop || result != 0) _active = false;
            return result;
        }

        uint8_t requestFrom(uint8_t address, uint8_t len) {
            std::lock_guard<std::mutex> guard(_mutex);
            if (!_active) startTransaction();
            ++_transactions;

            _rx.clear();
            auto it = _devices.find(address);
            if (_begun && it != _devices.end()) {
                std::vector<uint8_t> buffer(len);
                const size_t supplied = it->second->onRead(buffer.data(), len);
                _rx.insert(_rx.end(), buffer.begin(), buffer.begin() + supplied);
            }

            if (_rx.empty()) _active = false;
            return static_cast<uint8_t>(_rx.size());
        }

        int available(void) {
            std::lock_guard<std::mutex> guard(_mutex);
            return static_cast<int>(_rx.size());
        }

        int read(void) {
            std::this_thread::yield();
            std::lock_guard<std::mutex> guard(_mutex);
            if (_rx.empty()) return -1;
            const uint8_t byte = _rx.front();
            _rx.pop_front();
            if (_rx.empty()) _active = false;
            return byte;
        }

        /** Test controls. */

        void attach(uint8_t address, FakeDevice* device) {
            std::lock_guard<std::mutex> guard(_mutex);
            _devices[address] = device;
        }

        void detachAll(void) {
            std::lock_guard<std::mutex> guard(_mutex);
            _devices.clear();
            _rx.clear();
            _active = false;
            _interleavings = 0;
            _transactions = 0;
        }

        bool isBegun(void) {
            std::lock_guard<std::mutex> guard(_mutex);
            return _begun;
        }

        uint32_t clock(void) {
            std::lock_guard<std::mutex> guard(_mutex);
            return _clock;
        }

        int sda(void) {
            std::lock_guard<std::mutex> guard(_mutex);
            return _sda;
        }

        /** Transactions that started while another thread's was in flight. */
        int interleavings(void) const { return _interleavings.load(); }

        /** Completed write or read phases. */
        int transactions(void) const { return _transactions.load(); }

    private:
        /** Requires _mutex. */
        void startTransaction(void) {
            const std::thread::id me = std::this_thread::get_id();
            if (_active && _owner != me) ++_interleavings;
            _owner = me;
            _active = true;
        }

        std::mutex _mutex;
        std::map<uint8_t, FakeDevice*> _devices;
        bool _begun = false;
        int _sda = -1;
        int _scl = -1;
        uint32_t _clock = 0;

        uint8_t _tx_address = 0;
        std::vector<uint8_t> _tx;
        std::deque<uint8_t> _rx;

        bool _active = false;
        std::thread::id _owner;
        std::atomic<int> _interleavings{0};
        std::atomic<int> _transactions{0};
};
