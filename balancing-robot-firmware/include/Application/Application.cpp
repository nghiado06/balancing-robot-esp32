#include "Application.h"

float avoid_gain;
static bool armed = false;
UserCmd userBLE, userESPNOW;
Mode gMode = MODE_IDLE;

bool readBLE(float &v, float &yaw)
{
}

bool readESPNOW(float &v, float &yaw)
{
}

ControlOut ModeIdleHandler(float dt)
{
    ImuObs imu = PIDControl_Service_IMUHandle(dt);

    avoid_gain = 0.0f;

    bool nearlyStill = (fabsf(imu.omega_dps) < 5.0f) && (fabsf(imu.theta_deg) < 5.0f);
    static uint32_t stillStart = 0;
    if (nearlyStill)
    {
        if (stillStart == 0)
            stillStart = millis();
        else if (millis() - stillStart > 1200)
            PIDControl_Service_AutoTrim(dt);
    }
    else
        stillStart = 0;

    ControlIn u{0.0f, 0.0f, avoid_gain, true, armed};
    return PIDControl_Service_Compute(imu, u, dt);
}

ControlOut ModeBLEHandler(float dt)
{
    ImuObs imu = PIDControl_Service_IMUHandle(dt);

    avoid_gain = IoHwAb_UltraSonic_IsTriggered();

    // Đọc BLE
    float v = 0;
    float yaw = 0;
    bool got = readBLE(v, yaw);
    if (got)
    {
        userBLE.v_cmd_mm_s = v;
        userBLE.yaw_cmd_dps = yaw;
        userBLE.ts_ms = millis();
    }
    bool link_ok = (millis() - userBLE.ts_ms) <= 300;

    // Auto-trim khi gần đứng yên
    bool nearlyStill = (fabsf(imu.omega_dps) < 5.0f) && (fabsf(userBLE.v_cmd_mm_s) < VMAX_MM_S * 0.05f) && (fabsf(userBLE.yaw_cmd_dps) < 3.0f);
    static uint32_t stillStart = 0;
    if (nearlyStill)
    {
        if (stillStart == 0)
            stillStart = millis();
        else if (millis() - stillStart > 1200)
            PIDControl_Service_AutoTrim(dt);
    }
    else
        stillStart = 0;

    ControlIn u{userBLE.v_cmd_mm_s, userBLE.yaw_cmd_dps, avoid_gain, link_ok, armed};
    return PIDControl_Service_Compute(imu, u, dt);
}

ControlOut ModeHandMotionHandler(float dt)
{
    ImuObs imu = PIDControl_Service_IMUHandle(dt);

    avoid_gain = IoHwAb_UltraSonic_IsTriggered();

    float v = 0;
    float yaw = 0;
    bool got = readESPNOW(v, yaw);
    if (got)
    {
        userESPNOW.v_cmd_mm_s = v;
        userESPNOW.yaw_cmd_dps = yaw;
        userESPNOW.ts_ms = millis();
    }
    bool link_ok = (millis() - userESPNOW.ts_ms) <= 300;

    bool nearlyStill = (fabsf(imu.omega_dps) < 5.0f) && (fabsf(userESPNOW.v_cmd_mm_s) < VMAX_MM_S * 0.05f) && (fabsf(userESPNOW.yaw_cmd_dps) < 3.0f);
    static uint32_t stillStart = 0;
    if (nearlyStill)
    {
        if (stillStart == 0)
            stillStart = millis();
        else if (millis() - stillStart > 1200)
            PIDControl_Service_AutoTrim(dt);
    }
    else
        stillStart = 0;

    ControlIn u{userESPNOW.v_cmd_mm_s, userESPNOW.yaw_cmd_dps, avoid_gain, link_ok, armed};
    return PIDControl_Service_Compute(imu, u, dt);
}

void Application_Init()
{
    PIDControl_Service_Init();
}

void Application_Run()
{
    IoHwAb_Stepper_RunSpeed();
    IoHwAb_Buzzer_Tick();
    SignalHandler_Service_Handle(gMode, armed);

    static uint32_t last = micros();
    const uint32_t PERIOD_US = 2000;
    uint32_t now = micros();
    if ((uint32_t)(now - last) < PERIOD_US)
        return;

    float dt = ((now - last) * 1e-6f);
    last = now;
    if (dt > 0.010f)
        dt = 0.010f;

    if (gMode == MODE_IDLE)
    {
        ModeIdleHandler(dt);
    }
    else if (gMode == MODE_BLE)
    {
        ModeBLEHandler(dt);
    }
    else
    {
        ModeHandMotionHandler(dt);
    }
}
