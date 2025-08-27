#include "IoHwAb_Stepper.h"

AccelStepper stepperL(AccelStepper::DRIVER, STEP_L_PIN, DIR_L_PIN);
AccelStepper stepperR(AccelStepper::DRIVER, STEP_R_PIN, DIR_R_PIN);

void IoHwAb_Stepper_Init()
{
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);
    stepperL.setMaxSpeed(F_MAX_HZ);
    stepperL.setAcceleration(0);
    stepperR.setMaxSpeed(F_MAX_HZ);
    stepperR.setAcceleration(0);
}

void IoHwAb_Stepper_Apply(float fL, bool dL, float fR, bool dR)
{
    digitalWrite(DIR_L_PIN, dL ? HIGH : LOW);
    digitalWrite(DIR_R_PIN, dR ? HIGH : LOW);
    stepperL.setSpeed(dL ? fL : -fL);
    stepperR.setSpeed(dR ? fR : -fR);
    stepperL.runSpeed();
    stepperR.runSpeed();
}
