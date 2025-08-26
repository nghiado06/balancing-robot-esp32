#include "IoHwAb_Buzzer.h"

void IoHwAb_Buzzer_Init()
{
    ledcSetup(LEDC_CH, 2000, 10);
    ledcAttachPin(BUZZER_PIN, LEDC_CH);
}

void IoHwAb_Beep()
{
    ledcWriteTone(LEDC_CH, 2000);
    delay(100);
    ledcWriteTone(LEDC_CH, 0);
}

void IoHwAb_Starting_Sound()
{
    ledcWriteTone(LEDC_CH, NOTE_C5);
    delay(200);

    ledcWriteTone(LEDC_CH, NOTE_E5);
    delay(200);

    ledcWriteTone(LEDC_CH, NOTE_G5);
    delay(300);

    ledcWriteTone(LEDC_CH, NOTE_C6);
    delay(400);

    ledcWriteTone(LEDC_CH, 0);
}

void IoHwAb_ModeTransition_Sound()
{
    ledcWriteTone(LEDC_CH, NOTE_C6);
    delay(200);

    ledcWriteTone(LEDC_CH, NOTE_G5);
    delay(200);

    ledcWriteTone(LEDC_CH, NOTE_E5);
    delay(300);

    ledcWriteTone(LEDC_CH, NOTE_C5);
    delay(400);

    ledcWriteTone(LEDC_CH, 0);
}

void IoHwAb_Calib_Sound()
{
    ledcWriteTone(LEDC_CH, NOTE_C6);
    delay(200);

    ledcWriteTone(LEDC_CH, NOTE_G5);
    delay(200);

    ledcWriteTone(LEDC_CH, NOTE_E5);
    delay(300);

    ledcWriteTone(LEDC_CH, NOTE_C5);
    delay(400);

    ledcWriteTone(LEDC_CH, 0);
}