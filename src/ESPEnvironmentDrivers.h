#pragma once

/**
 * Umbrella header - include this to get every driver.
 */

#define ESP_ENVIRONMENT_DRIVERS_VERSION_MAJOR 1
#define ESP_ENVIRONMENT_DRIVERS_VERSION_MINOR 0
#define ESP_ENVIRONMENT_DRIVERS_VERSION_PATCH 0
#define ESP_ENVIRONMENT_DRIVERS_VERSION "1.0.0"

#include "Platform.hpp"
#include "I2CManager.hpp"
#include "I2CSensor.hpp"
#include "Stats.hpp"
#include "EnvMath.hpp"
#include "SHT3X/SHT31D.hpp"
#include "BMP3XX/BMP3XX.hpp"
#include "MPU6050/MPU6050.hpp"
