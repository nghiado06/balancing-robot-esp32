#ifndef PID_CONTROL_SERVICE_H
#define PID_CONTROL_SERVICE_H

#include <Arduino.h>
#include "Param_Config.h"
#include "Types.h"
#include "IoHwAb_Buzzer.h"
#include "IoHwAb_Mpu.h"
#include "IoHwAb_Stepper.h"

void PIDControl_Service_Init();

void PIDControl_Service_Calibrate(uint16_t samples = 2000, uint32_t us_dt = 1000);

ImuObs PIDControl_Service_IMUHandle(float dt);

ControlOut PIDControl_Service_Compute(const ImuObs &imu, const ControlIn &u, float dt);

void PIDControl_Service_AutoTrim(float dt);

#endif // PID_CONTROL_SERVICE_H