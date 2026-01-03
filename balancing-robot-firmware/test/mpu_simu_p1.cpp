#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

static constexpr float LOOP_HZ = 100.0f;
static constexpr uint32_t LOOP_US = (uint32_t)(1e6f / LOOP_HZ);

static inline float rad2deg(float rad) { return rad * 57.2957795f; }

float roll = 0, pitch = 0, yaw = 0;
uint32_t lastUs = 0;

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

    lastUs = micros();

    // Output: roll,pitch,yaw  (deg)
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

    // gyro rad/s -> deg/s
    float gx = rad2deg(g.gyro.x);
    float gy = rad2deg(g.gyro.y);
    float gz = rad2deg(g.gyro.z);

    // integrate (NO bias correction)
    roll += gx * dt;
    pitch += gy * dt;
    yaw += gz * dt;

    Serial.print(roll, 2);
    Serial.print(",");
    Serial.print(pitch, 2);
    Serial.print(",");
    Serial.println(yaw, 2);
}
