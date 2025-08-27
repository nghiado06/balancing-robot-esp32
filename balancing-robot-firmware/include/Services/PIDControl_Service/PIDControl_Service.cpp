#include "PIDControl_Service.h"

inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi
                                    : x;
}

inline float slew(float prev, float target, float dmax)
{
    float d = target - prev;
    if (d > dmax)
        d = dmax;
    else if (d < -dmax)
        d = -dmax;
    return prev + d;
}

inline float deg2rad(float d)
{
    return d * PI / 180.0f;
}

inline void vel_to_step(float v_mm_s, float &f_hz, bool &dir)
{
    dir = (v_mm_s >= 0.0f);
    float steps_per_s = fabsf(v_mm_s) * STEPS_PER_MM;
    f_hz = clampf(steps_per_s, 0.0f, F_MAX_HZ);
}

Calib calib;
float theta_hat = 0.0f;
float Iterm = 0.0f;

void PIDControl_Service_Init()
{
#ifndef PID_CONTROL_SERVICE_INIT
#define PID_CONTROL_SERVICE_INIT
    IoHwAb_Buzzer_Init();
    IoHwAb_Mpu_Init();
    IoHwAb_Stepper_Init();
#endif
}

void PIDControl_Service_Calibrate(uint16_t samples, uint32_t us_dt)
{
    // IoHwAb_Buzzer_Function_Sound();

    double sumGy = 0;
    double sumThetaAcc = 0;

    for (uint16_t i = 0; i < samples; i++)
    {
        float gy = IoHwAb_Mpu_ReadGyro(); // deg/s / Or gx
        float ax = IoHwAb_Mpu_ReadAngleX();
        float ay = IoHwAb_Mpu_ReadAngleY();
        float az = IoHwAb_Mpu_ReadAngleZ();
        float theta_acc = atan2f(ax, sqrtf(ay * ay + az * az)) * 57.29578f;
        sumGy += gy;
        sumThetaAcc += theta_acc;
        delayMicroseconds(us_dt);
    }
    calib.gyro_bias_dps = (float)(sumGy / samples);
    calib.angle_offset_deg = (float)(sumThetaAcc / samples);
    calib.theta_trim_deg = 0.0f; // reset trim
    theta_hat = 0.0f;
    Iterm = 0.0f;

    // IoHwAb_Buzzer_Function_Sound();
}

void PIDControl_Service_AutoTrim(float dt)
{
    calib.theta_trim_deg += K_TRIM * (theta_hat - calib.theta_trim_deg) * dt;
    calib.theta_trim_deg = clampf(calib.theta_trim_deg, -TRIM_MAX, +TRIM_MAX);
}

ImuObs PIDControl_Service_IMUHandle(float dt)
{
    float gy = IoHwAb_Mpu_ReadGyro() - calib.gyro_bias_dps; // deg/s
    float ax = IoHwAb_Mpu_ReadAngleX();
    float ay = IoHwAb_Mpu_ReadAngleY();
    float az = IoHwAb_Mpu_ReadAngleZ();
    float theta_acc = atan2f(ax, sqrtf(ay * ay + az * az)) * 57.29578f - calib.angle_offset_deg;
    theta_hat = ALPHA * (theta_hat + gy * dt) + (1.0f - ALPHA) * theta_acc;
    return {theta_hat, gy};
}

ControlOut PIDControl_Service_Compute(const ImuObs &imu, const ControlIn &u, float dt)
{
    static float prev_fL = 0.0f, prev_fR = 0.0f;

    // 1) Chuẩn hóa lệnh (dead-zone, limit)
    float v_cmd = (fabsf(u.v_cmd_mm_s) < V_DEAD) ? 0.0f : u.v_cmd_mm_s;
    float yaw = (fabsf(u.yaw_cmd_dps) < YAW_DEAD) ? 0.0f : u.yaw_cmd_dps;
    v_cmd = clampf(v_cmd, -VMAX_MM_S, +VMAX_MM_S);

    // 2) Tránh vật + mất link
    float v_ref = v_cmd * u.avoid_gain;
    float yaw_ref = u.link_ok ? yaw : 0.0f;
    if (!u.link_ok)
        v_ref = 0.0f;

    // 3) Tham chiếu nghiêng (kèm auto-trim)
    float theta_ref = clampf(K_V * v_ref, -THETA_REF_LIMIT, +THETA_REF_LIMIT) + calib.theta_trim_deg;

    // 4) PID cân bằng -> v_fwd
    float e = theta_ref - imu.theta_deg;
    // Anti-windup nghèo: khóa I khi v_fwd sẽ bão hòa
    float tentative_I = Iterm + KI * e * dt;

    float v_fwd_raw = KP * e - KD * imu.omega_dps + tentative_I;
    float v_fwd = clampf(v_fwd_raw, -VMAX_MM_S, +VMAX_MM_S);
    // nếu không bão hòa -> nhận I, bão hòa -> khóa I
    if (v_fwd == v_fwd_raw)
        Iterm = tentative_I;

    // 5) Trộn rẽ (động học chuẩn)
    float Omega = deg2rad(yaw_ref); // rad/s
    float vL = v_fwd - 0.5f * Omega * L_MM;
    float vR = v_fwd + 0.5f * Omega * L_MM;
    vL = clampf(vL, -VMAX_MM_S, +VMAX_MM_S);
    vR = clampf(vR, -VMAX_MM_S, +VMAX_MM_S);

    // 6) mm/s -> (STEP Hz, DIR) + ramp
    float fL, fR;
    bool dL, dR;
    vel_to_step(vL, fL, dL);
    vel_to_step(vR, fR, dR);

    if (!u.armed)
    {
        fL = 0;
        fR = 0;
        Iterm = 0;
    } // DISARM: dừng & reset I

    float fL_r = slew(prev_fL, fL, DF_MAX_HZ);
    float fR_r = slew(prev_fR, fR, DF_MAX_HZ);
    prev_fL = fL_r;
    prev_fR = fR_r;

    // 7) Gọi motor
    IoHwAb_Stepper_Apply(fL_r, dL, fR_r, dR);

    return {fL_r, fR_r, dL, dR, vL, vR, v_fwd, theta_ref};
}