#ifndef IOHWAB_STEPPER_H
#define IOHWAB_STEPPER_H

#include <Arduino.h>
#include <AccelStepper.h>
#include "Pin_Config.h"
#include "Param_Config.h"

void IoHwAb_Stepper_Init();

void IoHwAb_Stepper_Apply(float fL, bool dL, float fR, bool dR);

#endif // IOHWAB_STEPPER_H