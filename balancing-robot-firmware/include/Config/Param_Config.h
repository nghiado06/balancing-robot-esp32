#ifndef PARAM_CONFIG_H
#define PARAM_CONFIG_H

#include <Arduino.h>

// Tần số nốt (Hz)
#define NOTE_C5 523
#define NOTE_E5 659
#define NOTE_G5 784
#define NOTE_C6 1047

// Hình dáng mắt
const uint8_t EYE_SHAPE[8] = {
    0b00111100,
    0b01000010,
    0b10010101,
    0b10100001,
    0b10100001,
    0b10010101,
    0b01000010,
    0b00111100};

#endif // PARAM_CONFIG_H