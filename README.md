# Thread-Safe I2C Sensor Drivers for ESP32

[![Compile](https://github.com/sdvnci/ESP-Environment-Drivers/actions/workflows/compile.yml/badge.svg?branch=main)](https://github.com/sdvnci/ESP-Environment-Drivers/actions/workflows/compile.yml)
[![Unit Tests](https://github.com/sdvnci/ESP-Environment-Drivers/actions/workflows/testing.yml/badge.svg?branch=main)](https://github.com/sdvnci/ESP-Environment-Drivers/actions/workflows/testing.yml)
[![Library](https://github.com/sdvnci/ESP-Environment-Drivers/actions/workflows/library.yml/badge.svg?branch=main)](https://github.com/sdvnci/ESP-Environment-Drivers/actions/workflows/library.yml)
![Doxygen](/assets/doxygen_badge.svg)

While building the [ESP-Sky-Imager](https://github.com/sdvnci/ESP-Sky-Imager) and ESP-QuadCopter projects, I needed sensor drivers that could be used from more than one FreeRTOS task. The Adafruit stack (sensor library + BusIO) serialises nothing above single `Wire` calls, so two tasks reading two sensors on one bus can interleave their transactions. This library is the replacement, with no Adafruit dependencies.

## Supported Sensors

| Sensor | Class | Measures |
|---|---|---|
| SHT30 / SHT31 / SHT35 | `SHT31` | Temperature, humidity, heater |
| BMP388 / BMP390 | `BMP3XX` | Pressure, temperature, altitude |
| MPU6050 (+ common clones) | `MPU6050` | Gyro, accelerometer, tilt, die temperature |

Plus `env::dewPoint`, `env::altitude` and `env::seaLevelPressure`, and `stats::` helpers for filtering samples.

## Installation

**PlatformIO** - add to `platformio.ini`:

```ini
lib_deps =
    https://github.com/sdvnci/ESP-Environment-Drivers.git#v1.0.0
```

**Arduino IDE / arduino-cli** - download the repository as a ZIP and use *Sketch → Include Library → Add .ZIP Library*, or:

```bash
arduino-cli lib install --git-url https://github.com/sdvnci/ESP-Environment-Drivers.git#v1.0.0
```

(`--git-url` needs `library.enable_unsafe_install: true` in the arduino-cli config.) A Library Manager listing is planned.

**As a submodule** - see [Using It From ESP-Sky-Imager](#using-it-from-esp-sky-imager).

### Requirements

- An ESP32 board on the [arduino-esp32](https://github.com/espressif/arduino-esp32) core. CI compiles the examples for the ESP32 and ESP32-S3 on the current core release.
- C++11 or later. `[[nodiscard]]` warnings need GCC 7+, which every arduino-esp32 release ships.
- No other libraries.

## Usage

```cpp
#include <ESPEnvironmentDrivers.h>

SHT31 sht(0, SDA, SCL);   // bus 0, the board's default pins
BMP3XX bmp(0, SDA, SCL);  // same bus - that's fine

void setup() {
    Serial.begin(115200);
    if (!sht.init()) Serial.println(toString(sht.lastError()));
    if (!bmp.init()) Serial.println(toString(bmp.lastError()));
}

void loop() {
    if (sht.update()) {
        const sht3x::Reading r = sht.reading();   // temperature + humidity from the same sample
        Serial.printf("%.2f °C, %.2f %%, dew point %.2f °C\n",
                      r.temperature, r.humidity, env::dewPoint(r.temperature, r.humidity));
    }
    if (bmp.update()) {
        Serial.printf("%.2f hPa\n", bmp.getPressure() / 100.0f);
    }
    delay(1000);
}
```

Every fallible call returns `bool` and is `[[nodiscard]]`. On failure, `lastError()` says why:

| `I2CError` | Meaning |
|---|---|
| `NOT_INITIALIZED` | `init()` hasn't succeeded yet |
| `REGISTRATION_FAILED` | Unknown bus, pin mismatch, address conflict or incompatible clock |
| `BUS_TIMEOUT` | Another task held the bus or sensor too long |
| `NACK` | Nothing answered at the address, or the device rejected the data |
| `SHORT_READ` | The device sent fewer bytes than asked for |
| `CRC_MISMATCH` | Data or calibration failed its checksum |
| `BAD_DEVICE_ID` | A different chip is at that address |
| `DATA_NOT_READY` | A conversion didn't finish in time |
| `DEVICE_ERROR` | The chip reported an internal error |
| `OUT_OF_RANGE` | A physically impossible reading, likely bus corruption |

`toString(err)` gives the name, for logs or a status report.

See [`examples/`](examples) for each sensor, and [`SharedBus`](examples/SharedBus/SharedBus.ino) for two tasks sharing one bus.

## Thread Safety

Three layers, all RAII (scoped locks, never manual lock/unlock):

1. **Bus lock.** Each bus has a FreeRTOS recursive mutex (with priority inheritance). Every transaction (write, read, or register write + repeated start + read) holds it, so no other task can put bytes on the bus mid-transaction.
2. **Sensor lock.** Each public operation holds that sensor's mutex, so a multi-step operation like "trigger a conversion, wait, read it back" can't be split by another task driving the *same* sensor. The bus lock is released during the wait, so other sensors keep working while one converts.
3. **Snapshots.** Cached readings come out as one struct under a lock, so you never get temperature from one measurement and humidity from the next.

Sensors unregister themselves from the bus in their destructor.

To share a bus with code outside the library, hold `I2CManager::getInstance().busMutex(bus)` around your own transactions on `I2CManager::getInstance().wire(bus)`.

### Bus Management

`I2CManager` owns the core's `Wire` (bus 0) and `Wire1` (bus 1). On registration it checks for address conflicts and pin mismatches, and negotiates the fastest clock every device on the bus supports.

## Compared With Adafruit_BMP3XX

`BMP3XX` follows [Adafruit_BMP3XX](https://github.com/adafruit/Adafruit_BMP3XX) as the baseline. It uses the same default address (0x77), default settings (no oversampling, no IIR filter), calibration CRC check, double-precision Bosch compensation, and altitude formula. It also offers Adafruit's method names (`begin_I2C`, `performReading`, `readTemperature`, `readPressure`, `readAltitude`, `chipID`, `setTemperatureOversampling`, `setPressureOversampling`, `setIIRFilterCoeff`), so sketches port over with few changes. Differences:

- **Waits for the conversion.** Adafruit triggers a forced-mode conversion and reads the data registers immediately, which returns the *previous* conversion. This driver polls the data-ready flags first.
- **Forced mode only, no `setOutputDataRate`.** Adafruit sets ODR too, but it has no effect in forced mode, which is the only mode Adafruit uses.
- **Full double precision.** Bosch's `pow_bmp3()` rounds squares and cubes to `float`; here they stay `double`. Results agree to well under 0.1 Pa.
- **Rejects impossible readings** (`OUT_OF_RANGE`), since the chip has no CRC on its data.

## Footprint

The point of this library is correctness and thread safety, not size. Measured with `-Os` and linker garbage collection (as in an ESP32 build), on x86-64 as a stand-in (ESP32 byte counts will differ, the comparison holds):

| Sketch | Adafruit | This library |
|---|---|---|
| BMP3xx only | 5.1 KB | 5.6 KB |
| SHT31 + BMP3xx | 6.0 KB | 7.5 KB |

About 3 KB of that is the shared bus manager and sensor base class, paid once however many sensors you use. Not included compared with Adafruit_BMP3XX: SPI wiring, normal (continuous) mode, the FIFO and interrupt-pin setup.

## Using It From ESP-Sky-Imager

Arduino only compiles sources in a sketch's root and its `src/` folder, so add this repo as a submodule **under `src/`**:

```bash
git submodule add https://github.com/sdvnci/ESP-Environment-Drivers src/ESP-Environment-Drivers
```

```cpp
#include "src/ESP-Environment-Drivers/src/ESPEnvironmentDrivers.h"
```

Or install it as a library (it has a `library.properties`) and `#include <ESPEnvironmentDrivers.h>`.

The camera driver sets up its own I2C (SCCB) port. Keep the sensors on a different bus from the camera, or check which port your camera config claims.

## Building and Testing

The tests run on the host with [EpoxyDuino](https://github.com/bxparks/EpoxyDuino) and [AUnit](https://github.com/bxparks/AUnit), against a fake I2C bus ([`tests/FakeWire.hpp`](tests/FakeWire.hpp)) with protocol-level models of each chip ([`tests/FakeDevices.hpp`](tests/FakeDevices.hpp)). The fake bus flags any transaction that starts while another thread's is still in flight, and the concurrency tests drive several sensors from several threads to prove the locks hold. Removing any one of the three lock layers fails a test.

```bash
git clone --depth 1 --branch v1.7.1 https://github.com/bxparks/AUnit.git
git clone --depth 1 --branch v1.6.0 https://github.com/bxparks/EpoxyDuino.git
cd tests
make EPOXY_DUINO_DIR=../EpoxyDuino AUNIT_DIR=../AUnit
./tests.out
```

The BMP3xx compensation is checked against an independent Python implementation of the datasheet formulas, using synthetic calibration data. It has not yet been checked against a real sensor's output.

API docs: run `doxygen` in the repo root, then open `docs/html/index.html`. CI also builds them as a downloadable artifact.

## Releasing

The version lives in four places: `library.properties`, `library.json`, `ESP_ENVIRONMENT_DRIVERS_VERSION` in [`src/ESPEnvironmentDrivers.h`](src/ESPEnvironmentDrivers.h), and `PROJECT_NUMBER` in the `Doxyfile`. CI fails if they disagree, or if [CHANGELOG.md](CHANGELOG.md) has no entry for the version. Bump all four, add the changelog entry, then tag `vX.Y.Z`.

## License

This project is licensed under the Business Source License. See [LICENSE](LICENSE) for details.
