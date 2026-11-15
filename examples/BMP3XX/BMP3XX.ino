#include <ESPEnvironmentDrivers.h>

// Sea-level pressure in hPa - for real altitudes, use the nearest airport's QNH.
constexpr float QNH_HPA = 1013.25f;

// 0x77 is the default on most breakouts; use bmp3::ADDRESS_PRIMARY (0x76) if SDO is grounded.
BMP3XX bmp(0, SDA, SCL);

void setup() {
    Serial.begin(115200);

    // Bosch's recommended weather-monitoring settings. Without this call the
    // driver uses Adafruit_BMP3XX's defaults (no oversampling, no filter).
    (void)bmp.configure(bmp3::Oversampling::X8, bmp3::Oversampling::X1, bmp3::IIRFilter::COEF_3);

    if (!bmp.init()) {
        Serial.printf("BMP3xx init failed: %s\n", toString(bmp.lastError()));
        return;
    }
    Serial.printf("Found %s\n", bmp.chipId() == bmp3::CHIP_ID_BMP390 ? "BMP390" : "BMP388");
}

void loop() {
    if (bmp.update()) {
        Serial.printf("%.2f °C  %.2f hPa  %.1f m\n",
                      bmp.getTemperature(), bmp.getPressure() / 100.0f, bmp.getAltitude(QNH_HPA));
    } else {
        Serial.printf("BMP3xx read failed: %s\n", toString(bmp.lastError()));
    }
    delay(2000);
}
