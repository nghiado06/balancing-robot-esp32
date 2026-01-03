#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 Mpu;

// ====== Tuning ======
static constexpr float LOOP_HZ = 200.0f;
static constexpr uint32_t LOOP_US = (uint32_t)(1e6f / LOOP_HZ);
static constexpr uint32_t CALIB_MS = 2000;
static constexpr float ALPHA = 0.98f;

// ✅ Giảm log: in theo tần số thấp hơn + lấy trung bình
static constexpr float PRINT_HZ = 20.0f;                                // 20 Hz (mỗi 50 ms)
static constexpr uint32_t PRINT_EVERY = (uint32_t)(LOOP_HZ / PRINT_HZ); // 200/20 = 10 mẫu
// ====================

static inline float rad2deg(float rad) { return rad * 57.2957795f; }

bool calibDone = false;

// Calib accumulators
double gyroSum = 0.0;
double pitchAccSum = 0.0;
uint32_t calibCount = 0;

float gyroBiasY_dps = 0.0f;
float pitchRef_deg = 0.0f; // mốc “nằm ngang = 0°”

float pitchGyro = 0.0f;
float pitchFused = 0.0f;

uint32_t lastUs = 0;
uint32_t startMs = 0;

// ✅ Print-averaging accumulators
double sumPitchAccZero = 0.0;
double sumGyroY_dps = 0.0;
double sumPitchGyro = 0.0;
double sumPitchFused = 0.0;
uint32_t printCount = 0;

void setup()
{
    Serial.begin(115200);
    delay(300);

    Wire.begin();
    if (!Mpu.begin())
    {
        Serial.println("MPU6050 not found!");
        while (1)
            delay(100);
    }

    Mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("CALIB: keep MPU still & flat for 2 seconds...");
    Serial.println("t,pitchAcc_deg_avg,gyroY_dps_avg,pitchGyro_deg_avg,pitchFused_deg_avg");

    startMs = millis();
    lastUs = micros();
}

void loop()
{
    uint32_t nowUs = micros();
    if (nowUs - lastUs < LOOP_US)
        return;

    float dt = (nowUs - lastUs) * 1e-6f;
    lastUs = nowUs;

    sensors_event_t a, g, temp;
    Mpu.getEvent(&a, &g, &temp);

    // --- Compute pitch from accel (deg) ---
    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;

    float pitchAcc = rad2deg(atan2(-ax, sqrt(ay * ay + az * az)));

    // --- Gyro Y (deg/s) ---
    float gyroY_dps = rad2deg(g.gyro.y);

    // --- Calibration for first 2 seconds ---
    if (!calibDone)
    {
        gyroSum += gyroY_dps;
        pitchAccSum += pitchAcc;
        calibCount++;

        if (millis() - startMs >= CALIB_MS)
        {
            gyroBiasY_dps = (calibCount > 0) ? (float)(gyroSum / (double)calibCount) : 0.0f;
            pitchRef_deg = (calibCount > 0) ? (float)(pitchAccSum / (double)calibCount) : 0.0f;

            calibDone = true;

            Serial.print("CALIB DONE | samples=");
            Serial.print(calibCount);
            Serial.print(" | gyroBiasY_dps=");
            Serial.print(gyroBiasY_dps, 6);
            Serial.print(" | pitchRef_deg=");
            Serial.println(pitchRef_deg, 3);

            // reset góc để bắt đầu từ 0
            pitchGyro = 0.0f;
            pitchFused = 0.0f;
            startMs = millis();

            // reset bộ gom log
            sumPitchAccZero = sumGyroY_dps = sumPitchGyro = sumPitchFused = 0.0;
            printCount = 0;
        }
        return; // chưa calib xong thì khỏi tính/print tiếp
    }

    // --- Apply zero reference ---
    float pitchAccZero = pitchAcc - pitchRef_deg; // nằm ngang -> 0
    float gyroY_cal = gyroY_dps - gyroBiasY_dps;  // gyro đã trừ bias

    pitchGyro += gyroY_cal * dt;
    pitchFused = ALPHA * (pitchFused + gyroY_cal * dt) + (1.0f - ALPHA) * pitchAccZero;

    // ✅ Gom N mẫu rồi in trung bình
    sumPitchAccZero += pitchAccZero;
    sumGyroY_dps += gyroY_dps; // nếu muốn trung bình gyro đã trừ bias thì đổi thành gyroY_cal
    sumPitchGyro += pitchGyro;
    sumPitchFused += pitchFused;
    printCount++;

    if (printCount >= PRINT_EVERY)
    {
        float t = (millis() - startMs) / 1000.0f;

        Serial.print(t, 3);
        Serial.print(",");
        Serial.print((float)(sumPitchAccZero / printCount), 2);
        Serial.print(",");
        Serial.print((float)(sumGyroY_dps / printCount), 2);
        Serial.print(",");
        Serial.print((float)(sumPitchGyro / printCount), 2);
        Serial.print(",");
        Serial.println((float)(sumPitchFused / printCount), 2);

        sumPitchAccZero = sumGyroY_dps = sumPitchGyro = sumPitchFused = 0.0;
        printCount = 0;
    }
}
