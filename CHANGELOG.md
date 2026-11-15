# Changelog

## 1.0.0

First release as a standalone Arduino / PlatformIO library.

### Added
- `BMP3XX` driver for the BMP388 and BMP390, using [Adafruit_BMP3XX](https://github.com/adafruit/Adafruit_BMP3XX) as the baseline: same defaults, calibration CRC check, double-precision compensation and altitude formula, plus Adafruit-compatible method names. Waits for data-ready before reading, so it never returns the previous conversion.
- MPU6050 accelerometer, tilt and die temperature (`readAccel`, `readTilt`, `readTemperature`), and a `WHO_AM_I` check that accepts common clones.
- SHT31 alternate address (0x45), selectable repeatability, status register and heater readback.
- `lastError()` on every sensor, with `I2CError` codes and `toString()`.
- `env::dewPoint`, `env::altitude`, `env::seaLevelPressure`.
- Examples for each sensor and for two FreeRTOS tasks sharing one bus.
- Host-side tests against a fake I2C bus that detects interleaved transactions.

### Fixed
- Thread safety: the lock was per sensor, so two sensors on one bus could interleave. Each bus now has its own mutex, held for a whole transaction, including the repeated start between a register write and its read.
- SHT31 medium repeatability sent `0x2C0B`, which is not an SHT3x command; it is now `0x240B`.
- `mean()` accumulated into a `long`, truncating float samples.
- MPU6050 ignored the filter and range passed to its constructor.

### Changed (breaking)
- Library layout: sources live in `src/`. Include `<ESPEnvironmentDrivers.h>`.
- The SHT31 namespace is `sht3x` (was `sht`), so `SHT31 sht(...)` doesn't clash with it.
- Statistics helpers moved from `I2CSensor` member functions to the `stats` namespace.
- Fallible calls return `bool` and are `[[nodiscard]]`. `MPU6050::readGyro()` also has a `bool readGyro(MPU_XYZ&)` overload; the old form returns zeros on failure.
- Bus 0 and 1 use the core's global `Wire` and `Wire1` instead of private `TwoWire` instances.

### Removed
- The unfinished port of Bosch's `bmp3.c` under `BMP/`.
