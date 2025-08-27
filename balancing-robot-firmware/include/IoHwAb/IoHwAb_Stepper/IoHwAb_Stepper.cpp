#include "IoHwAb_Stepper.h"

AccelStepper stepperL(AccelStepper::DRIVER, STEP_L_PIN, DIR_L_PIN);
AccelStepper stepperR(AccelStepper::DRIVER, STEP_R_PIN, DIR_R_PIN);

static bool s_enabled = false;

void IoHwAb_Stepper_Init()
{
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);
    stepperL.setMaxSpeed(F_MAX_HZ);
    stepperR.setMaxSpeed(F_MAX_HZ);
    stepperL.setAcceleration(0);
    stepperR.setAcceleration(0);

    digitalWrite(STEP_L_PIN, LOW);
    digitalWrite(STEP_R_PIN, LOW);

    s_enabled = true;
}

void IoHwAb_Stepper_Apply(float fL, bool dL, float fR, bool dR)
{
    if (!s_enabled)
    {
        stepperL.setSpeed(0.0f);
        stepperR.setSpeed(0.0f);
        return;
    }

    if (fL < 0)
        fL = 0;
    if (fR < 0)
        fR = 0;
    if (fL > F_MAX_HZ)
        fL = F_MAX_HZ;
    if (fR > F_MAX_HZ)
        fR = F_MAX_HZ;

    const float sL = dL ? +fL : -fL;
    const float sR = dR ? +fR : -fR;

    stepperL.setSpeed(sL);
    stepperR.setSpeed(sR);
}

void IoHwAb_Stepper_Enable(bool en)
{
    s_enabled = en;
    digitalWrite(ENABLE_PIN, en ? LOW : HIGH);

    if (!en)
    {
        stepperL.setSpeed(0.0f);
        stepperR.setSpeed(0.0f);
    }
}

void IoHwAb_Stepper_RunSpeed()
{
    stepperL.runSpeed();
    stepperR.runSpeed();
}
