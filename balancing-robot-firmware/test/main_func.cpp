#include <Arduino.h>
#include "config.h"
#include "imu_process.h"
#include "stepper_control.h"

ImuProcess imu;
StepperControl stepper;

// Timing
uint32_t lastUs = 0;
uint32_t startMs = 0;

// Imu data
float pitch = 0.0f;
float gyroY = 0.0f;
bool needCalibrate = true;

void setup()
{
    Serial.begin(115200);
    delay(300); // Allow time for Serial to initialize
    Serial.println("Balancing Robot Firmware Starting...");

    imu.begin();
    stepper.begin();

    // Initialize timing
    startMs = millis();
    lastUs = micros();
}

void loop()
{
    // Timing
    uint32_t nowUs = micros();
    uint32_t nowMs = millis();
    if (nowUs - lastUs < LOOP_US)
        return;

    float dt_us = (nowUs - lastUs);
    float dt = (nowUs - lastUs) * 1e-6f;
    lastUs = nowUs;
}