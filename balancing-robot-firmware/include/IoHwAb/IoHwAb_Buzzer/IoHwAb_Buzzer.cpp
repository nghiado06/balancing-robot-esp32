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

void IoHwAb_ModeTransition_Sound()
{
    ledcWriteTone(LEDC_CH, NOTE_C5);
    delay(110);
    ledcWriteTone(LEDC_CH, 0);
    delay(30);

    ledcWriteTone(LEDC_CH, NOTE_E5);
    delay(110);
    ledcWriteTone(LEDC_CH, 0);
    delay(30);

    ledcWriteTone(LEDC_CH, NOTE_G5);
    delay(120);
    ledcWriteTone(LEDC_CH, 0);
    delay(30);

    ledcWriteTone(LEDC_CH, NOTE_C6);
    delay(220);
    ledcWriteTone(LEDC_CH, 0);
    delay(10);

    ledcWriteTone(LEDC_CH, 0);
}

void IoHwAb_Starting_Sound()
{
    ledcWriteTone(LEDC_CH, NOTE_C6);
    delay(80);
    ledcWriteTone(LEDC_CH, 0);
    delay(20);

    ledcWriteTone(LEDC_CH, NOTE_G5);
    delay(70);
    ledcWriteTone(LEDC_CH, 0);
    delay(15);

    ledcWriteTone(LEDC_CH, NOTE_E5);
    delay(70);
    ledcWriteTone(LEDC_CH, 0);
    delay(15);

    ledcWriteTone(LEDC_CH, NOTE_C5);
    delay(90);
    ledcWriteTone(LEDC_CH, 0);
    delay(10);

    ledcWriteTone(LEDC_CH, 0);
}