/**
 * Two FreeRTOS tasks reading different sensors on the same I2C bus, with no
 * locking in the sketch: each driver serialises its transactions on the bus.
 */
#include <ESPEnvironmentDrivers.h>

SHT31 sht(0, SDA, SCL);
BMP3XX bmp(0, SDA, SCL);

void shtTask(void*) {
    for (;;) {
        if (sht.update()) {
            const sht3x::Reading r = sht.reading();
            Serial.printf("[sht] %.2f °C %.2f %%RH\n", r.temperature, r.humidity);
        } else {
            Serial.printf("[sht] %s\n", toString(sht.lastError()));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void bmpTask(void*) {
    for (;;) {
        if (bmp.update()) {
            Serial.printf("[bmp] %.2f hPa\n", bmp.getPressure() / 100.0f);
        } else {
            Serial.printf("[bmp] %s\n", toString(bmp.lastError()));
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void setup() {
    Serial.begin(115200);

    if (!sht.init()) Serial.printf("SHT31 init failed: %s\n", toString(sht.lastError()));
    if (!bmp.init()) Serial.printf("BMP3xx init failed: %s\n", toString(bmp.lastError()));

    xTaskCreatePinnedToCore(shtTask, "sht", 4096, nullptr, 1, nullptr, 0);
    xTaskCreatePinnedToCore(bmpTask, "bmp", 4096, nullptr, 1, nullptr, 1);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
