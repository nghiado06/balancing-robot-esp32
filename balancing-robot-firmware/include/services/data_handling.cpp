#include "data_handling.h"

// Balance Control - PID Compute
void DataHandling::pidCompute(float pitch_in, float gyro_in, float dt_s, float &omega_cmd, float pitch_ref)
{
    // Validate dt_s
    if (dt_s <= 0.0f || dt_s > 0.1f)
    {
        omega_cmd = 0.0f;
        return;
    }

    // Compute error
    float error = pitch_ref - pitch_in;

    // Deadzone for error
    if (fabsf(error) < eDead)
        error = 0.0f;

    // Proportional term
    float P = kp * error;

    // Integral term
    integral += error * dt_s;

    // Anti-windup
    if (integral > Imax)
        integral = Imax;
    else if (integral < -Imax)
        integral = -Imax;

    float I = ki * integral;

    // Derivative term
    float D = kd * gyro_in;

    // Compute command
    omega_cmd = P + I - D;

    // Command limiting
    if (omega_cmd > Umax)
        omega_cmd = Umax;
    else if (omega_cmd < -Umax)
        omega_cmd = -Umax;
}