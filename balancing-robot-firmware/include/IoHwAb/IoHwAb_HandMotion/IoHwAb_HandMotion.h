#ifndef IOHWAB_HANDMOTION_H
#define IOHWAB_HANDMOTION_H

#include <Arduino.h>
#include "Pin_Config.h"
#include "Param_Config.h"
#include <esp_now.h>
#include <esp_wifi.h>

void IoHwAb_HandMotion_Init();

bool IoHwAb_HandMotion_GetData(float &v_cmd, float &yaw_cmd);

#endif // IOHWAB_HANDMOTION_H