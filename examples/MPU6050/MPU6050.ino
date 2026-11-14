#include <ESPEnvironmentDrivers.h>

MPU6050 mpu6050(0, SDA, SCL);

void setup() {
    Serial.begin(115200);

    if (!mpu6050.init()) {
        Serial.printf("MPU6050 init failed: %s\n", toString(mpu6050.lastError()));
        return;
    }

    Serial.println("Calibrating gyro - keep the sensor still.");
    if (!mpu6050.calibrateGyro()) {
        Serial.printf("Calibration failed: %s\n", toString(mpu6050.lastError()));
    }
}

void loop() {
    MPU_XYZ gyro;
    float pitch, roll;
    if (mpu6050.readGyro(gyro) && mpu6050.readTilt(pitch, roll)) {
        Serial.printf("Gyro X %.2f Y %.2f Z %.2f °/s  pitch %.1f° roll %.1f°\n",
                      gyro[0], gyro[1], gyro[2], pitch, roll);
    } else {
        Serial.printf("MPU6050 read failed: %s\n", toString(mpu6050.lastError()));
    }
    delay(500);
}
