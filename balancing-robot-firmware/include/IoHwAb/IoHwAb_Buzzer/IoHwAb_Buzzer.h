#ifndef IOHWAB_BUZZER_H
#define IOHWAB_BUZZER_H

#include "Pin_Config.h"
#include "Param_Config.h"
#include <Arduino.h>

void IoHwAb_Buzzer_Init();
void IoHwAb_Buzzer_Tick();
bool IoHwAb_Buzzer_IsIdle();
void IoHwAb_Buzzer_Stop();

void IoHwAb_Buzzer_Beep();

void IoHwAb_Buzzer_Function_Sound();

void IoHwAb_Buzzer_Starting_Sound();

#endif // IOHWAB_BUZZER_H