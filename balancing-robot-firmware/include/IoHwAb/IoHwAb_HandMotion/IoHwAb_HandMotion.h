#ifndef IOHWAB_HANDMOTION_H
#define IOHWAB_HANDMOTION_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "Pin_Config.h"
#include "Param_Config.h"
struct HandMotionRaw
{
    uint8_t ver;
    uint32_t seq;
    uint32_t ts_ms;
    float ax, ay, az; // g
    float gx, gy, gz; // dps
    uint16_t vbat_mV;
    uint8_t flags;
    uint16_t crc16;
};

void IoHwAb_HandMotion_Init();
bool IoHwAb_HandMotion_GetRaw(HandMotionRaw *out);
bool IoHwAb_HandMotion_LinkOK(uint32_t timeout_ms = 400);
uint32_t IoHwAb_HandMotion_LastRxMs();
uint32_t IoHwAb_HandMotion_LastSeq();
void IoHwAb_HandMotion_SetEnabled(bool en);
void IoHwAb_HandMotion_Clear();

#endif // IOHWAB_HANDMOTION_H