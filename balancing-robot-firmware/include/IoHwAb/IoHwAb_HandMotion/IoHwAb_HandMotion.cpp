#include "IoHwAb_HandMotion.h"

static HandMotionRaw s_last{};
static volatile bool s_hasNew = false;
static volatile uint32_t s_lastRxMs = 0;
static volatile uint32_t s_lastSeq = 0;

static uint16_t crc16(const uint8_t *data, size_t n)
{
    uint16_t c = 0xFFFF;
    for (size_t i = 0; i < n; ++i)
    {
        c ^= data[i];
        for (int b = 0; b < 8; ++b)
            c = (c & 1) ? (c >> 1) ^ 0xA001 : (c >> 1);
    }
    return c;
}

static void onRecvCb(const uint8_t *mac, const uint8_t *data, int len)
{
    if (len != (int)sizeof(HandMotionRaw))
        return;
    HandMotionRaw tmp;
    memcpy(&tmp, data, sizeof(tmp));
    uint16_t c = tmp.crc16;
    tmp.crc16 = 0;
    if (crc16(reinterpret_cast<const uint8_t *>(&tmp), sizeof(tmp)) != c)
    {
        return;
    }
    s_last = tmp;
    s_lastRxMs = millis();
    s_lastSeq = tmp.seq;
    s_hasNew = true;
}

void IoHwAb_HandMotion_Init()
{
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK)
    {
        return;
    }
    esp_now_register_recv_cb(onRecvCb);
}

bool IoHwAb_HandMotion_GetRaw(HandMotionRaw *out)
{
    if (!out)
        return false;
    if (!s_hasNew)
        return false;

    HandMotionRaw tmp = s_last;
    *out = tmp;
    s_hasNew = false;
    return true;
}

bool IoHwAb_HandMotion_LinkOK(uint32_t timeout_ms)
{
    uint32_t now = millis();
    return (now - s_lastRxMs) <= timeout_ms;
}

uint32_t IoHwAb_HandMotion_LastRxMs() { return s_lastRxMs; }
uint32_t IoHwAb_HandMotion_LastSeq() { return s_lastSeq; }
