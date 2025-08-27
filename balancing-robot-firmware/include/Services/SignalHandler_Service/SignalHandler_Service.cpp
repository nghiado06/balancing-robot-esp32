#include "SignalHandler_Service.h"

void SignalHandler_Service_Init()
{
    // Initialize signal handling
}

void SignalHandler_Service_Handle(Mode &currentMode, bool &armed)
{
    static uint32_t tCalib = 0;
    static uint32_t tMode = 0;
    static uint32_t tArm = 0;

    if (IoHwAb_Button_Arm && millis() - tArm > 200)
    {
        armed = !armed;
        IoHwAb_Stepper_Enable(armed);
        IoHwAb_Buzzer_Function_Sound();
        tArm = millis();
    }

    if (IoHwAb_Button_ModeTransition() && millis() - tMode > 300)
    {
        IoHwAb_Buzzer_Function_Sound();
        tMode = millis();
        currentMode = (currentMode == MODE_IDLE) ? MODE_BLE : (currentMode == MODE_BLE) ? MODE_HAND_CONTROL
                                                                                        : MODE_IDLE;
    }

    if (IoHwAb_Button_Calib() && millis() - tCalib > 500)
    {
        IoHwAb_Buzzer_Function_Sound();
        if (!armed)
        {
            PIDControl_Service_Calibrate();
        }
        tCalib = millis();
    }
}
