#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 Mpu;

// ====== Tuning ======
static constexpr float LOOP_HZ = 200.0f;
static constexpr uint32_t LOOP_US = (uint32_t)(1e6f / LOOP_HZ);
static constexpr uint32_t CALIB_MS = 2000; // 2s lấy bias
// ====================

static inline float rad2deg(float rad) { return rad * 57.2957795f; }

float gyroBiasY_dps = 0.0f;
float angle_uncal = 0.0f;
float angle_cal = 0.0f;

uint32_t lastUs = 0;
uint32_t startMs = 0;

// Bias accumulator (2s đầu)
double biasSum = 0.0;
uint32_t biasCount = 0;
bool biasReady = false;

void setup()
{
    Serial.begin(115200);
    delay(300); // giúp Serial bắt kịp sau reset (ESP32 hay cần)

    Wire.begin();

    if (!Mpu.begin())
    {
        Serial.println("MPU6050 not found. Check wiring!");
        while (1)
            delay(100);
    }

    Mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("CALIB: Keep MPU still for 2 seconds...");
    Serial.println("t,gy_dps,angle_uncal,angle_cal");

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

    // MPU nằm ngang, X forward => test drift gy (quanh trục Y)
    float gy_dps = rad2deg(g.gyro.y);

    // Góc tích phân chưa trừ bias (sẽ drift)
    angle_uncal += gy_dps * dt;

    // Tính bias trong 2 giây đầu
    if (!biasReady)
    {
        biasSum += gy_dps;
        biasCount++;

        if (millis() - startMs >= CALIB_MS)
        {
            gyroBiasY_dps = (biasCount > 0) ? (float)(biasSum / (double)biasCount) : 0.0f;
            biasReady = true;

            // ✅ LOG RÕ RÀNG NGAY KHI ĐỦ 2 GIÂY
            Serial.print("CALIB DONE after ");
            Serial.print(CALIB_MS);
            Serial.print(" ms | samples=");
            Serial.print(biasCount);
            Serial.print(" | gyroBiasY_dps=");
            Serial.println(gyroBiasY_dps, 6);

            // (tuỳ chọn) reset lại góc để nhìn drift từ 0 cho dễ
            angle_uncal = 0.0f;
            angle_cal = 0.0f;
            startMs = millis(); // reset mốc thời gian cho log t bắt đầu lại từ 0
        }
    }

    // Góc tích phân đã trừ bias
    float gy_cal_dps = gy_dps - gyroBiasY_dps;
    angle_cal += gy_cal_dps * dt;

    float t = (millis() - startMs) / 1000.0f;
    Serial.print(t, 3);
    Serial.print(",");
    Serial.print(gy_dps, 4);
    Serial.print(",");
    Serial.print(angle_uncal, 3);
    Serial.print(",");
    Serial.println(angle_cal, 3);
}
