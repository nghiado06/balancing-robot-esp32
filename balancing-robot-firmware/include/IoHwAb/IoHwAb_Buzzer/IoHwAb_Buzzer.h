#ifndef IOHWAB_BUZZER_H
#define IOHWAB_BUZZER_H

#include "Pin_Config.h"
#include "Param_Config.h"
#include <Arduino.h>

void IoHwAb_Buzzer_Init();

void IoHwAb_Beep();

void IoHwAb_ModeTransition_Sound();

void IoHwAb_Calib_Sound();

void IoHwAb_Starting_Sound();

#endif // IOHWAB_BUZZER_H