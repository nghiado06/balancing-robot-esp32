#ifndef DATA_HANDLING_H
#define DATA_HANDLING_H

#include <Arduino.h>

class DataHandling
{
public:
    DataHandling() {}
    ~DataHandling() {}

    // Balance Control - PID Compute
    void pidCompute(float pitch_in, float gyro_in, float dt_s, float &omega_cmd, float pitch_ref = 0.0f);

    // Set PID parameters
    void setPIDParams(float kp_in, float ki_in, float kd_in)
    {
        kp = kp_in;
        ki = ki_in;
        kd = kd_in;
    }

    // Reset PID integrator
    void resetPID()
    {
        integral = 0.0f;
    }

    // Supervisor Control

    // Differential Drive Control

    // Motor Limitor

private:
    // PID parameters
    float kp = 30.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    // PID Computing variables
    float integral = 0.0f;

    // Limiters
    float Imax = 0.3f;
    float Umax = 35.0f;
    float eDead = 0.0f;

    // Helpers
    // Deg to Rad
    float degToRad(float deg)
    {
        return deg * (M_PI / 180.0f); // PI / 180
    }
};

#endif // DATA_HANDLING_H