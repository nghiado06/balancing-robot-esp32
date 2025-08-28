#include "IoHwAb_Buzzer.h"

struct ToneStep
{
    uint16_t freq;
    uint16_t dur_ms;
};

static const ToneStep PATTERN_BEEP[] = {
    {2000, 100}, {0, 0}};

static const ToneStep PATTERN_START[] = {
    {NOTE_C6, 80},
    {0, 20},
    {NOTE_G5, 70},
    {0, 15},
    {NOTE_E5, 70},
    {0, 15},
    {NOTE_C5, 90},
    {0, 10},
};

static const ToneStep PATTERN_FUNC[] = {
    {1500, 60}, {0, 40}, {1800, 60}};

static const ToneStep *s_cur = nullptr;
static size_t s_count = 0;
static size_t s_idx = 0;
static uint32_t s_deadline = 0;

static inline void buzzerWrite(uint16_t freq)
{
    ledcWriteTone(LEDC_CH, freq);
}

static void playPattern(const ToneStep *p, size_t n)
{
    if (!p || n == 0)
        return;
    s_cur = p;
    s_count = n;
    s_idx = 0;
    s_deadline = 0;
}

void IoHwAb_Buzzer_Init()
{
    ledcSetup(LEDC_CH, 2000, 10);
    ledcAttachPin(BUZZER_PIN, LEDC_CH);
    buzzerWrite(0);
}

void IoHwAb_Buzzer_Tick()
{
    if (!s_cur)
        return;

    const uint32_t now = millis();

    if (s_deadline == 0)
    {
        buzzerWrite(s_cur[s_idx].freq);
        s_deadline = now + s_cur[s_idx].dur_ms;
        return;
    }

    if ((int32_t)(now - s_deadline) >= 0)
    {
        s_idx++;
        if (s_idx >= s_count)
        {
            buzzerWrite(0);
            s_cur = nullptr;
            s_count = 0;
            s_idx = 0;
            s_deadline = 0;
        }
        else
        {
            buzzerWrite(s_cur[s_idx].freq);
            s_deadline = now + s_cur[s_idx].dur_ms;
        }
    }
}

bool IoHwAb_Buzzer_IsIdle()
{
    return (s_cur == nullptr);
}

void IoHwAb_Buzzer_Stop()
{
    s_cur = nullptr;
    s_count = 0;
    s_idx = 0;
    s_deadline = 0;
    buzzerWrite(0);
}

void IoHwAb_Buzzer_Beep()
{
    if (!IoHwAb_Buzzer_IsIdle())
        return;
    playPattern(PATTERN_BEEP, sizeof(PATTERN_BEEP) / sizeof(PATTERN_BEEP[0]));
}

void IoHwAb_Buzzer_Starting_Sound()
{
    if (!IoHwAb_Buzzer_IsIdle())
        return;
    playPattern(PATTERN_START, sizeof(PATTERN_START) / sizeof(ToneStep));
}

void IoHwAb_Buzzer_Function_Sound()
{
    if (!IoHwAb_Buzzer_IsIdle())
        return;
    playPattern(PATTERN_FUNC, sizeof(PATTERN_FUNC) / sizeof(ToneStep));
}
