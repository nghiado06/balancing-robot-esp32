#ifndef IOHWAB_BUTTON_H
#define IOHWAB_BUTTON_H

#include "Pin_Config.h"
#include <Arduino.h>

void IoHwAb_Button_Init();

bool IoHwAb_Button_Calib();

bool IoHwAb_Button_ModeTransition();

bool IoHwAb_Button_Arm();

#endif // IOHWAB_BUTTON_H