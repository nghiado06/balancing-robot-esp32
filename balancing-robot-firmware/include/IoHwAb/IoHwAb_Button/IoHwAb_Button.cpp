#include "IoHwAb_Button.h"

void IoHwAb_Button_Init()
{
    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);
}

bool IoHwAb_Calib()
{
    if (digitalRead(BUTTON1_PIN) == LOW)
    {
        return true;
    }
    return false;
}

bool IoHwAb_ModeTransition()
{
    if (digitalRead(BUTTON2_PIN) == LOW)
    {
        return true;
    }
    return false;
}