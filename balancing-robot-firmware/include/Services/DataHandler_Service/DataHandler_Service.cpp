#include "DataHandler_Service.h"
#include <math.h>

static bool s_forwardIsX = true;
static float s_pitch_off = 0.0f;
static float s_roll_off = 0.0f;
static float s_pitch_hat = 0.0f;
static float s_roll_hat = 0.0f;

static inline float pitch_from_acc(float ax, float ay, float az, bool forwardIsX)
{
    if (forwardIsX)
        return atan2f(ax, sqrtf(ay * ay + az * az)) * 57.29578f;
    else
        return atan2f(ay, sqrtf(ax * ax + az * az)) * 57.29578f;
}
static inline float roll_from_acc(float ax, float ay, float az, bool forwardIsX)
{
    if (forwardIsX)
        return atan2f(ay, sqrtf(ax * ax + az * az)) * 57.29578f;
    else
        return atan2f(ax, sqrtf(ay * ay + az * az)) * 57.29578f;
}

void DataHandler_Service_Init(bool forwardAxisIsX)
{
    s_forwardIsX = forwardAxisIsX;
    s_pitch_off = s_roll_off = 0.0f;
    s_pitch_hat = s_roll_hat = 0.0f;
}

void DataHandler_Service_CalibrateNeutral(uint16_t samples, uint32_t us_dt)
{
    double sumP = 0.0, sumR = 0.0;
    for (uint16_t i = 0; i < samples; ++i)
    {
        HandMotionRaw raw;

        if (!IoHwAb_HandMotion_GetRaw(&raw))
        {
        }
        else
        {
            float p = pitch_from_acc(raw.ax, raw.ay, raw.az, s_forwardIsX);
            float r = roll_from_acc(raw.ax, raw.ay, raw.az, s_forwardIsX);
            sumP += p;
            sumR += r;
        }
        delayMicroseconds(us_dt);
    }
    s_pitch_off = (float)(sumP / (double)samples);
    s_roll_off = (float)(sumR / (double)samples);
    s_pitch_hat = 0.0f; // reset filter
    s_roll_hat = 0.0f;
}

static inline float apply_dead_clamp(float x, float dead, float lo, float hi)
{
    if (fabsf(x) < dead)
        x = 0.0f;
    if (x > hi)
        x = hi;
    if (x < lo)
        x = lo;
    return x;
}

bool DataHandler_Service_Update(float dt, ControlIn *u_out)
{
    if (!u_out)
        return false;

    const bool link_ok = IoHwAb_HandMotion_LinkOK(HM_LINK_TIMEOUT_MS);

    HandMotionRaw raw;
    bool got = IoHwAb_HandMotion_GetRaw(&raw);
    if (!got)
    {
        u_out->v_cmd_mm_s = 0.0f;
        u_out->yaw_cmd_dps = 0.0f;
        u_out->link_ok = link_ok;
        return false;
    }

    float pitch_acc = pitch_from_acc(raw.ax, raw.ay, raw.az, s_forwardIsX) - s_pitch_off;
    float roll_acc = roll_from_acc(raw.ax, raw.ay, raw.az, s_forwardIsX) - s_roll_off;

    float pitch_gyro = s_forwardIsX ? raw.gx : raw.gy;
    float roll_gyro = s_forwardIsX ? raw.gy : raw.gx;

    s_pitch_hat = HM_ALPHA * (s_pitch_hat + pitch_gyro * dt) + (1.0f - HM_ALPHA) * pitch_acc;
    s_roll_hat = HM_ALPHA * (s_roll_hat + roll_gyro * dt) + (1.0f - HM_ALPHA) * roll_acc;

    float v_cmd = K_VCMD_MM_S_PER_DEG * s_pitch_hat;
    float yaw_cmd = K_YAWCMD_DPS_PER_DEG * s_roll_hat;

    v_cmd = apply_dead_clamp(v_cmd, V_DEAD, -VMAX_MM_S, VMAX_MM_S);
    yaw_cmd = apply_dead_clamp(yaw_cmd, YAW_DEAD, -180.0f, 180.0f);

    u_out->v_cmd_mm_s = v_cmd;
    u_out->yaw_cmd_dps = yaw_cmd;
    u_out->link_ok = link_ok;

    return true;
}
