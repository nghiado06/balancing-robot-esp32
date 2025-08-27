#include "IoHwAb_UltraSonic.h"

void IoHwAb_UltraSonic_Init()
{
#ifndef ULTRASONIC_INIT
#define ULTRASONIC_INIT
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);
#endif
}

float IoHwAb_UltraSonic_IsTriggered()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // đo độ rộng xung ECHO, timeout ~25ms (≈4.3 m)
    unsigned long us = pulseIn(ECHO_PIN, HIGH, TIMEOUT_DURATION);
    if (us == 0)
        return 0.0f;

    float d = us * 0.0343f / 2.0f;

    if (d < THRESHOLD_DISTANCE)
    {
        return 1.0f;
    }

    return 0.0f;
}
