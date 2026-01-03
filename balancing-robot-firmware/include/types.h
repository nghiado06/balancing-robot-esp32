#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

enum class MotorSide : uint8_t
{
    Left = 0,
    Right = 1
};

struct MotorLEDC
{
    int stepPin;
    int dirPin;
    bool invertDir;
    int channel;

    bool lastForward;
    float lastStepHz;
};

#endif // TYPES_H