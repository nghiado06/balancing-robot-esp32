#ifndef CONFIG_H
#define CONFIG_H

#include <cmath>
#include <Arduino.h>

// Loop Config
constexpr float LOOP_HZ = 200.0f;                        // loop frequency in Hz
constexpr uint32_t LOOP_US = (uint32_t)(1e6f / LOOP_HZ); // loop period in microseconds

// Imu Config
constexpr uint32_t CALIB_MS = 2000; // calib time in milliseconds
constexpr float ALPHA = 0.98f;      // Complementary filter coefficient

// Motor Config
constexpr uint32_t FULL_STEPS_PER_REV = 200;                       // 1.8deg
constexpr uint32_t MICROSTEP = 16;                                 // e.g., 1/16
constexpr uint32_t STEPS_PER_REV = FULL_STEPS_PER_REV * MICROSTEP; // 3200

constexpr bool INVERT_DIR_L = false;
constexpr bool INVERT_DIR_R = false;

constexpr int LEDC_CH_L = 0;
constexpr int LEDC_CH_R = 1;
constexpr int LEDC_RES_BITS = 10;                            // 0..1023
constexpr uint32_t LEDC_DUTY_50 = 1U << (LEDC_RES_BITS - 1); // ~50%

constexpr float STEP_HZ_MIN_RUN = 1.0f; // <1Hz consider as stop
constexpr float STEP_HZ_MAX = 20000.0f; // you can increase if

constexpr float OMEGA_DEADBAND_DEG_S = 1.5f; // deg/s
constexpr float FALL_ANGLE_DEG = 30.0f;      // angle to consider as fall

#endif // CONFIG_H