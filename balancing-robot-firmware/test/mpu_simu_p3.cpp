#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

static constexpr float LOOP_HZ = 100.0f;
static constexpr uint32_t LOOP_US = (uint32_t)(1e6f / LOOP_HZ);
static constexpr uint32_t CALIB_MS = 2000;
static constexpr float ALPHA = 0.98f; // 0.97~0.995 tùy rung/độ mượt

static inline float rad2deg(float rad) { return rad * 57.2957795f; }

float biasGx = 0, biasGy = 0, biasGz = 0; // deg/s
float rollRef = 0, pitchRef = 0;          // accel-based zero reference (deg)

float roll = 0, pitch = 0, yaw = 0;
uint32_t lastUs = 0;

void calibrateBiasAndZero()
{
    Serial.println("CALIB: keep still & flat for 2 seconds...");
    uint32_t t0 = millis();

    double sx = 0, sy = 0, sz = 0;
    double sRollAcc = 0, sPitchAcc = 0;
    uint32_t n = 0;

    while (millis() - t0 < CALIB_MS)
    {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        float ax = a.acceleration.x;
        float ay = a.acceleration.y;
        float az = a.acceleration.z;

        // accel -> roll/pitch (deg) with X forward
        float rollAcc = rad2deg(atan2(ay, az));
        float pitchAcc = rad2deg(atan2(-ax, sqrt(ay * ay + az * az)));

        sx += rad2deg(g.gyro.x);
        sy += rad2deg(g.gyro.y);
        sz += rad2deg(g.gyro.z);

        sRollAcc += rollAcc;
        sPitchAcc += pitchAcc;
        n++;

        delay(2);
    }

    biasGx = (n ? (float)(sx / n) : 0);
    biasGy = (n ? (float)(sy / n) : 0);
    biasGz = (n ? (float)(sz / n) : 0);

    rollRef = (n ? (float)(sRollAcc / n) : 0);
    pitchRef = (n ? (float)(sPitchAcc / n) : 0);

    Serial.print("CALIB DONE | biasGx=");
    Serial.print(biasGx, 6);
    Serial.print(" biasGy=");
    Serial.print(biasGy, 6);
    Serial.print(" biasGz=");
    Serial.print(biasGz, 6);
    Serial.print(" | rollRef=");
    Serial.print(rollRef, 3);
    Serial.print(" pitchRef=");
    Serial.println(pitchRef, 3);
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

    calibrateBiasAndZero();

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

    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;

    // accel angles (deg)
    float rollAcc = rad2deg(atan2(ay, az));
    float pitchAcc = rad2deg(atan2(-ax, sqrt(ay * ay + az * az)));

    // zero reference so "flat" -> ~0 deg
    float rollAccZ = rollAcc - rollRef;
    float pitchAccZ = pitchAcc - pitchRef;

    // gyro deg/s with bias removed
    float gx = rad2deg(g.gyro.x) - biasGx;
    float gy = rad2deg(g.gyro.y) - biasGy;
    float gz = rad2deg(g.gyro.z) - biasGz;

    // complementary for roll/pitch
    roll = ALPHA * (roll + gx * dt) + (1.0f - ALPHA) * rollAccZ;
    pitch = ALPHA * (pitch + gy * dt) + (1.0f - ALPHA) * pitchAccZ;

    // yaw from gyro only (will drift long-term)
    yaw += gz * dt;

    Serial.print(roll, 2);
    Serial.print(",");
    Serial.print(pitch, 2);
    Serial.print(",");
    Serial.println(yaw, 2);
}
