#ifndef IOHWAB_BUTTON_H
#define IOHWAB_BUTTON_H

#include "Pin_Config.h"
#include <Arduino.h>

void IoHwAb_Button_Init();

bool IoHwAb_Calib();

bool IoHwAb_ModeTransition();

#endif // IOHWAB_BUTTON_H