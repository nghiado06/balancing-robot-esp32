#include "IoHwAb_Button.h"

void IoHwAb_Button_Init()
{
    pinMode(BUTTON_CALIB_PIN, INPUT_PULLUP);
    pinMode(BUTTON_MODE_PIN, INPUT_PULLUP);
}

bool IoHwAb_Button_Calib()
{
    if (digitalRead(BUTTON_CALIB_PIN) == LOW)
    {
        return true;
    }
    return false;
}

bool IoHwAb_Button_ModeTransition()
{
    if (digitalRead(BUTTON_MODE_PIN) == LOW)
    {
        return true;
    }
    return false;
}

bool IoHwAb_Button_Arm()
{
    if (digitalRead(BUTTON_ARM_PIN) == LOW)
    {
        return true;
    }
    return false;
}