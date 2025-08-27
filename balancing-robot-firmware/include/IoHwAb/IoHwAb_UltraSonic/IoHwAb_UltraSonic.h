#ifndef IOHWAB_ULTRASONIC_H
#define IOHWAB_ULTRASONIC_H

#include <Arduino.h>
#include "Pin_Config.h"
#include "Param_Config.h"

void IoHwAb_UltraSonic_Init();

float IoHwAb_UltraSonic_IsTriggered();

#endif // IOHWAB_ULTRASONIC_H