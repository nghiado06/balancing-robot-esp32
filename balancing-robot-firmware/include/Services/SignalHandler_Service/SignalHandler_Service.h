#ifndef SIGNAL_HANDLER_SERVICE_H
#define SIGNAL_HANDLER_SERVICE_H

#include <Arduino.h>
#include "Param_Config.h"
#include "Types.h"
#include "Pin_Config.h"
#include "IoHwAb_Button.h"
#include "IoHwAb_Stepper.h"
#include "IoHwAb_Buzzer.h"
#include "PIDControl_Service.h"

void SignalHandler_Service_Init();

Mode SignalHandler_Service_Handle(Mode currentMode);

#endif // SIGNAL_HANDLER_SERVICE_H