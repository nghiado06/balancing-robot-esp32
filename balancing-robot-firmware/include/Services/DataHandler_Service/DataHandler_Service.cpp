#include "DataHandler_Service.h"
#include <math.h>

// ==== Tham số ánh xạ & lọc cho điều khiển tay ====
// mm/s mỗi 1 độ pitch (nghiêng tới lui)
#ifndef K_VCMD_MM_S_PER_DEG
#define K_VCMD_MM_S_PER_DEG 25.0f
#endif
// dps mỗi 1 độ roll (nghiêng trái phải)
#ifndef K_YAWCMD_DPS_PER_DEG
#define K_YAWCMD_DPS_PER_DEG 20.0f
#endif
// Lọc complementary cho góc tay cầm (nhanh hơn IMU thân xe)
#ifndef HM_ALPHA
#define HM_ALPHA 0.90f
#endif
// Timeout link (ms): nếu quá ngưỡng coi như mất link
#ifndef HM_LINK_TIMEOUT_MS
#define HM_LINK_TIMEOUT_MS 400
#endif

static bool s_forwardIsX = true; // trục "tiến" của tay cầm (X hoặc Y)
static float s_pitch_off = 0.0f; // offset trung tính
static float s_roll_off = 0.0f;
static float s_pitch_hat = 0.0f; // góc đã lọc (deg)
static float s_roll_hat = 0.0f;

// công thức tính pitch/roll từ accel theo trục lựa chọn
static inline float pitch_from_acc(float ax, float ay, float az, bool forwardIsX)
{
    if (forwardIsX)
        return atan2f(ax, sqrtf(ay * ay + az * az)) * 57.29578f;
    else
        return atan2f(ay, sqrtf(ax * ax + az * az)) * 57.29578f;
}
static inline float roll_from_acc(float ax, float ay, float az, bool forwardIsX)
{
    // roll định nghĩa quanh trục tiến → dùng trục còn lại
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
    // Lấy trung bình pitch/roll từ accel (tay cầm giữ tư thế trung tính)
    double sumP = 0.0, sumR = 0.0;
    for (uint16_t i = 0; i < samples; ++i)
    {
        HandMotionRaw raw;
        // nếu không có gói mới thì vẫn dùng gói gần nhất (tay bạn giữ yên)
        if (!IoHwAb_HandMotion_GetRaw(&raw))
        {
            // no-op
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

// Áp deadzone và kẹp
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

    // Kiểm tra link
    const bool link_ok = IoHwAb_HandMotion_LinkOK(HM_LINK_TIMEOUT_MS);

    HandMotionRaw raw;
    bool got = IoHwAb_HandMotion_GetRaw(&raw);
    if (!got)
    {
        // không có sample mới: vẫn trả link_ok để hệ thống fail-safe ở tầng trên
        u_out->v_cmd_mm_s = 0.0f;
        u_out->yaw_cmd_dps = 0.0f;
        u_out->link_ok = link_ok;
        return false;
    }

    // Tính pitch/roll từ accel
    float pitch_acc = pitch_from_acc(raw.ax, raw.ay, raw.az, s_forwardIsX) - s_pitch_off;
    float roll_acc = roll_from_acc(raw.ax, raw.ay, raw.az, s_forwardIsX) - s_roll_off;

    // Gyro trục tương ứng (đơn vị dps) để bổ sung
    float pitch_gyro = s_forwardIsX ? raw.gx : raw.gy;
    float roll_gyro = s_forwardIsX ? raw.gy : raw.gx;

    // Complementary (nhanh): hat = α*(hat + gyro*dt) + (1-α)*acc
    s_pitch_hat = HM_ALPHA * (s_pitch_hat + pitch_gyro * dt) + (1.0f - HM_ALPHA) * pitch_acc;
    s_roll_hat = HM_ALPHA * (s_roll_hat + roll_gyro * dt) + (1.0f - HM_ALPHA) * roll_acc;

    // Map → lệnh
    float v_cmd = K_VCMD_MM_S_PER_DEG * s_pitch_hat;   // dương = nghiêng tới = chạy tới
    float yaw_cmd = K_YAWCMD_DPS_PER_DEG * s_roll_hat; // dương = nghiêng phải = quay phải (tuỳ bạn)

    // Deadzone & kẹp (dùng hằng của bạn từ Param_Config.h)
    v_cmd = apply_dead_clamp(v_cmd, V_DEAD, -VMAX_MM_S, VMAX_MM_S);
    yaw_cmd = apply_dead_clamp(yaw_cmd, YAW_DEAD, -180.0f, 180.0f);

    // Xuất ra ControlIn
    u_out->v_cmd_mm_s = v_cmd;
    u_out->yaw_cmd_dps = yaw_cmd;
    u_out->link_ok = link_ok; // armed, avoid_gain set ở tầng Application
    // u_out->armed     // do SignalHandler quản lý
    // u_out->avoid_gain// do UltraSonic/Compute logic
    return true;
}
