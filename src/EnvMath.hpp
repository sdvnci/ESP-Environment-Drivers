#pragma once

#include <cmath>

/**
 * Derived environmental quantities.
 */
namespace env {

    /**
     * @brief Dew point from temperature and relative humidity (Magnus formula,
     * Sonntag 1990 constants - within 0.35 °C over -45 to 60 °C).
     * @param temperatureC Air temperature in °C.
     * @param humidity Relative humidity in %.
     * @return Dew point in °C, or NaN if humidity is not in (0, 100].
     */
    inline float dewPoint(float temperatureC, float humidity) {
        if (!(humidity > 0.0f && humidity <= 100.0f)) return NAN;
        constexpr double B = 17.62;
        constexpr double C = 243.12;
        const double gamma = std::log(humidity / 100.0) + (B * temperatureC) / (C + temperatureC);
        return static_cast<float>((C * gamma) / (B - gamma));
    }

    /**
     * @brief Altitude from pressure, given the sea-level pressure (QNH), using
     * the international barometric formula with the same constants as
     * Adafruit_BMP3XX::readAltitude().
     * @param pressurePa Measured station pressure in Pa.
     * @param qnhHpa Sea-level pressure in hPa (eg: from the nearest METAR).
     * @return Altitude in metres.
     */
    inline float altitude(float pressurePa, float qnhHpa) {
        return static_cast<float>(44330.0 * (1.0 - std::pow(pressurePa / (qnhHpa * 100.0), 0.1903)));
    }

    /**
     * @brief Sea-level pressure from a station pressure at a known altitude.
     * The inverse of altitude().
     * @param pressurePa Measured station pressure in Pa.
     * @param altitudeM Station altitude in metres.
     * @return Sea-level pressure in hPa.
     */
    inline float seaLevelPressure(float pressurePa, float altitudeM) {
        return static_cast<float>((pressurePa / 100.0) / std::pow(1.0 - altitudeM / 44330.0, 1.0 / 0.1903));
    }
};
