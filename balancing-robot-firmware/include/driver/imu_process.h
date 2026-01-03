#ifndef IMU_PROCESS_H
#define IMU_PROCESS_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "config.h"

class ImuProcess
{
public:
    ImuProcess() = default;
    ~ImuProcess() = default;

    // Initialize
    bool begin();

    // Compute
    void process(float &pitch_out, float &gyro_out, uint32_t dt_us, bool needCalib = false);

    // Logging
    void logData(uint32_t printEvery, uint32_t nowMs);

    // Update calibration duration
    void setCalibDurationMs(uint32_t calibMs)
    {
        calibDurationMs = calibMs;
        calibStartMs = millis();
        calibDone = false;
    }

private:
    // MPU6050 object
    Adafruit_MPU6050 mpu_;

    // Calibration
    bool calibDone = false; // Calibration done flag

    // Calib accumulators
    double gyroSum = 0.0;                // Sum of gyro Y readings for bias calculation
    double pitchAccSum = 0.0;            // Sum of pitch from accel for reference calculation
    uint32_t calibCount = 0;             // Number of samples collected for calibration
    uint32_t calibDurationMs = CALIB_MS; // Calibration duration in milliseconds
    uint32_t calibStartMs = 0;           // Calibration start time in milliseconds

    float gyroBiasY_dps = 0.0f; // Gyro Y bias in degrees per second
    float pitchRef_deg = 0.0f;  // Reference pitch angle from accel

    float pitchGyro = 0.0f;  // Integrated pitch from gyro
    float pitchFused = 0.0f; // Fused pitch angle

    // Print-averaging accumulators
    double sumPitchAccZero = 0.0; // Sum of pitch from accel (zero-referenced)
    double sumGyroY_dps = 0.0;    // Sum of gyro Y readings
    double sumPitchGyro = 0.0;    // Sum of integrated pitch from gyro
    double sumPitchFused = 0.0;   // Sum of fused pitch angle
    uint32_t printCount = 0;      // Number of samples collected for printing

    // Helpers
    float rad2deg(float rad) { return rad * 57.2957795f; } // Radians to degrees conversion
};

#endif