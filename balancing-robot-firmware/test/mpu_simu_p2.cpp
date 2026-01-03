#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

static constexpr float LOOP_HZ = 100.0f;
static constexpr uint32_t LOOP_US = (uint32_t)(1e6f / LOOP_HZ);
static constexpr uint32_t CALIB_MS = 2000;

static inline float rad2deg(float rad) { return rad * 57.2957795f; }

float biasGx = 0, biasGy = 0, biasGz = 0; // deg/s
float roll = 0, pitch = 0, yaw = 0;
uint32_t lastUs = 0;

void calibrateGyroBias()
{
    Serial.println("CALIB: keep still for 2 seconds...");
    uint32_t t0 = millis();

    double sx = 0, sy = 0, sz = 0;
    uint32_t n = 0;

    while (millis() - t0 < CALIB_MS)
    {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        sx += rad2deg(g.gyro.x);
        sy += rad2deg(g.gyro.y);
        sz += rad2deg(g.gyro.z);
        n++;

        delay(2);
    }

    biasGx = (n ? (float)(sx / n) : 0);
    biasGy = (n ? (float)(sy / n) : 0);
    biasGz = (n ? (float)(sz / n) : 0);

    Serial.print("CALIB DONE | biasGx=");
    Serial.print(biasGx, 6);
    Serial.print(" biasGy=");
    Serial.print(biasGy, 6);
    Serial.print(" biasGz=");
    Serial.println(biasGz, 6);
}

void setup()
{
    Serial.begin(115200);
    delay(200);
    Wire.begin();

    if (!mpu.begin())
    {
        Serial.println("MPU6050 not found!");
        while (1)
            delay(100);
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    calibrateGyroBias();

    roll = pitch = yaw = 0;
    lastUs = micros();

    // Output: roll,pitch,yaw
}

void loop()
{
    uint32_t nowUs = micros();
    if (nowUs - lastUs < LOOP_US)
        return;
    float dt = (nowUs - lastUs) * 1e-6f;
    lastUs = nowUs;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float gx = rad2deg(g.gyro.x) - biasGx;
    float gy = rad2deg(g.gyro.y) - biasGy;
    float gz = rad2deg(g.gyro.z) - biasGz;

    roll += gx * dt;
    pitch += gy * dt;
    yaw += gz * dt;

    Serial.print(roll, 2);
    Serial.print(",");
    Serial.print(pitch, 2);
    Serial.print(",");
    Serial.println(yaw, 2);
}
