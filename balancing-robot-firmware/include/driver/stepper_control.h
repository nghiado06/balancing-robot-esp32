#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <Arduino.h>
#include "types.h"
#include "config.h"
#include "pins.h"

class StepperControl
{
public:
    // Constructor
    StepperControl();

    // Destructor
    ~StepperControl();

    // Initialize
    void begin();

    // Enable or disable motors
    void enable(bool enable);

    // Stop motor
    void stop(MotorSide side);

    // Set motor direction
    void setDirection(MotorSide side, bool forward);

    // Set step frequency using LEDC (steps/s)
    void setStepHz(MotorSide side, float f_step_hz);

    // Set omega (deg/s): auto direction, step using LEDC
    void setOmegaDegS(MotorSide side, float omega_deg_s);

    // Set both motors' omega (deg/s)
    void setOmegaDegS(float omegaL_deg_s, float omegaR_deg_s);

    // Stop all motors
    void stopAll();

private:
    // Stepper Objects
    MotorLEDC motorLeft_;
    MotorLEDC motorRight_;

    // Helpers
    float omegaToStepHz(float omega_rad_s);
    float clampStepHz(float f);
    MotorLEDC &getMotor(MotorSide side);
    float degToRad(float deg) { return deg * (M_PI / 180.0f); } // PI / 180
};

#endif // STEPPER_CONTROL_H