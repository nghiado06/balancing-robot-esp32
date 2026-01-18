#include <Arduino.h>
#include "config.h"
#include "imu_process.h"
#include "stepper_control.h"
#include "data_handling.h"
#include "tuning.h"

ImuProcess imu;
StepperControl stepper;
DataHandling dataHandler;
ControlUI ui;

// Timing
uint32_t lastUs = 0;
uint32_t startMs = 0;

// PID parameters
float kp = 31.0f;
float ki = 0.02f;
float kd = 1.2f;

// Imu data
float pitch = 0.0f;
float gyroY = 0.0f;
bool needCalibrate = true;

void offBuzzer();

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

  // Buzzer indication
  pinMode(BUZZER_PIN, OUTPUT);
  offBuzzer();

  // Remote UI
  ui.begin();

  ui.setName(1, "Kp+");
  ui.setName(2, "Kp-");
  ui.setName(3, "Kd+");
  ui.setName(4, "Kd-");
  ui.setName(5, "Ki+");
  ui.setName(6, "Ki-");
}

void loop()
{
  // ui.remoteProcess();
  // Timing
  uint32_t nowUs = micros();
  uint32_t nowMs = millis();
  if (nowUs - lastUs < LOOP_US)
    return;

  float dt_us = (nowUs - lastUs);
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;

  // IMU Processing
  imu.process(pitch, gyroY, dt_us, needCalibrate);
  if (fabsf(pitch) > FALL_ANGLE_DEG)
  {
    stepper.stopAll();
    dataHandler.resetPID();
    return;
  }

  // Pid Control
  float omegaCmd = 0.0f;
  dataHandler.pidCompute(pitch, gyroY, dt, omegaCmd);

  // Debug Output
  static uint32_t lastPrintMs = 0;
  if (millis() - lastPrintMs >= 50)
  { // 20 Hz
    Serial.printf("Pitch=%.2f deg, GyroY=%.2f dps, Omega=%.2f deg/s\n",
                  pitch, gyroY, omegaCmd);
    lastPrintMs = millis();
  }

  // Set Motor Speeds
  stepper.setOmegaDegS(omegaCmd, omegaCmd);

  // // Tuning
  // if (ui.isFunction1Pressed())
  // {
  //   kp += 1.0f;
  // }
  // if (ui.isFunction2Pressed())
  // {
  //   kp -= 1.0f;
  // }
  // if (ui.isFunction3Pressed())
  // {
  //   kd += 0.1f;
  // }
  // if (ui.isFunction4Pressed())
  // {
  //   kd -= 0.1f;
  // }
  // if (ui.isFunction5Pressed())
  // {
  //   ki += 0.01f;
  // }
  // if (ui.isFunction6Pressed())
  // {
  //   ki -= 0.01f;
  // }
  // dataHandler.setPIDParams(kp, ki, kd);
  // ui.addLog("Kp: " + String(kp, 2), 0);
  // ui.addLog("Kd: " + String(kd, 2), 1);
  // ui.addLog("Ki: " + String(ki, 3), 2);
}

void offBuzzer()
{
  digitalWrite(BUZZER_PIN, LOW);
}