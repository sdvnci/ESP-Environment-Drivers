#include <ESPEnvironmentDrivers.h>

// The board's default I2C pins - change to match your wiring.
SHT31 sht(0, SDA, SCL);

void setup() {
    Serial.begin(115200);

    if (!sht.init()) {
        Serial.printf("SHT31 init failed: %s\n", toString(sht.lastError()));
        return;
    }
}

void loop() {
    if (sht.update()) {
        const sht3x::Reading r = sht.reading();
        Serial.printf("%.2f °C  %.2f %%RH  dew point %.2f °C\n",
                      r.temperature, r.humidity, env::dewPoint(r.temperature, r.humidity));
    } else {
        Serial.printf("SHT31 read failed: %s\n", toString(sht.lastError()));
    }
    delay(2000);
}
