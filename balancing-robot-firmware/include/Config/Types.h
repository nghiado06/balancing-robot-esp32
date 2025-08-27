#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

typedef struct Calib
{
    float gyro_bias_dps = 0.0f;
    float angle_offset_deg = 0.0f;
    float theta_trim_deg = 0.0f;
} Calib;

typedef struct UserCmd
{
    float v_cmd_mm_s = 0.0f;
    float yaw_cmd_dps = 0.0f;
    uint32_t ts_ms = 0;
} UserCmd;

enum Mode
{
    MODE_IDLE = 0,
    MODE_BLE,
    MODE_HAND_CONTROL
};

typedef struct ImuObs
{
    float theta_deg;
    float omega_dps;
} ImuObs;

typedef struct ControlIn
{
    float v_cmd_mm_s;
    float yaw_cmd_dps;
    float avoid_gain;
    bool link_ok;
    bool armed;
} ControlIn;

typedef struct ControlOut
{
    float fL, fR;
    bool dL, dR;
    float vL, vR, v_fwd, theta_ref;
} ControlOut;

#endif // TYPES_H