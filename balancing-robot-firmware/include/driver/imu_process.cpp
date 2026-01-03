#include "imu_process.h"

bool ImuProcess::begin()
{
    Wire.begin();
    if (!mpu_.begin())
    {
        Serial.println("MPU6050 not found!");
        return false;
    }

    mpu_.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("MPU6050 initialized.");

    return true;
}

void ImuProcess::process(float &pitch_out, float &gyro_out, uint32_t dt_us, bool needCalib)
{
    // Get sensor events
    sensors_event_t a, g, temp;
    mpu_.getEvent(&a, &g, &temp);

    // Time delta in seconds
    float dt = dt_us * 1e-6f;

    // Compute pitch from accel and gyro
    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;

    float gyroY_dps = rad2deg(g.gyro.y); // Gyro Y in degrees per second
    float pitchAcc_deg = rad2deg(atan2(-ax, sqrt(ay * ay + az * az)));

    // Calibration process
    if (needCalib && !calibDone)
    {
        gyroSum += gyroY_dps;
        pitchAccSum += pitchAcc_deg;
        calibCount++;

        if (millis() - calibStartMs >= calibDurationMs)
        {
            // Compute biases
            gyroBiasY_dps = gyroSum / calibCount;
            pitchRef_deg = pitchAccSum / calibCount;
            calibDone = true;

            // Reset accumulators
            gyroSum = 0.0;
            pitchAccSum = 0.0;
            pitchFused = 0.0;
            pitchGyro = 0.0;
            calibCount = 0;
            printCount = 0;

            // Reset logging accumulators
            sumPitchAccZero = 0.0;
            sumGyroY_dps = 0.0;
            sumPitchGyro = 0.0;
            sumPitchFused = 0.0;

            Serial.println("Calibration done.");
        }
        return; // Skip processing until calibration is done
    }

    // Remove bias from gyro
    float gyroY_cal = gyroY_dps - gyroBiasY_dps;
    float picthAcc_cal = pitchAcc_deg - pitchRef_deg;

    // Integrate gyro to get pitch angle
    pitchGyro += gyroY_cal * dt;

    // Complementary filter to fuse accel and gyro
    pitchFused = ALPHA * (pitchFused + gyroY_cal * dt) + (1.0f - ALPHA) * picthAcc_cal;
    // Output results
    pitch_out = pitchFused;
    gyro_out = gyroY_cal;

    // Accumulate for logging
    sumPitchAccZero += picthAcc_cal;
    sumGyroY_dps += gyroY_cal;
    sumPitchGyro += pitchGyro;
    sumPitchFused += pitchFused;
    printCount++;
}

void ImuProcess::logData(uint32_t printEvery, uint32_t nowMs)
{
    if (printCount >= printEvery)
    {
        float avgPitchAcc = static_cast<float>(sumPitchAccZero / printCount);
        float avgGyroY = static_cast<float>(sumGyroY_dps / printCount);
        float avgPitchGyro = static_cast<float>(sumPitchGyro / printCount);
        float avgPitchFused = static_cast<float>(sumPitchFused / printCount);

        Serial.print("IMU LOG @");
        Serial.print(nowMs);
        Serial.print(" ms | Avg Pitch Acc: ");
        Serial.print(avgPitchAcc, 3);
        Serial.print(" deg | Avg Gyro Y: ");
        Serial.print(avgGyroY, 3);
        Serial.print(" deg/s | Avg Pitch Gyro: ");
        Serial.print(avgPitchGyro, 3);
        Serial.print(" deg | Avg Pitch Fused: ");
        Serial.print(avgPitchFused, 3);
        Serial.println(" deg");

        // Reset accumulators
        sumPitchAccZero = 0.0;
        sumGyroY_dps = 0.0;
        sumPitchGyro = 0.0;
        sumPitchFused = 0.0;
        printCount = 0;
    }
}