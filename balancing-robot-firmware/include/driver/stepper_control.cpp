#include "stepper_control.h"

StepperControl::StepperControl()
    : motorLeft_{STEP_L_PIN, DIR_L_PIN, INVERT_DIR_L, LEDC_CH_L, true, 0.0f},
      motorRight_{STEP_R_PIN, DIR_R_PIN, INVERT_DIR_R, LEDC_CH_R, true, 0.0f}
{
}

StepperControl::~StepperControl()
{
}

void StepperControl::begin()
{
    pinMode(motorLeft_.stepPin, OUTPUT);
    pinMode(motorLeft_.dirPin, OUTPUT);
    pinMode(motorRight_.stepPin, OUTPUT);
    pinMode(motorRight_.dirPin, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);

    digitalWrite(motorLeft_.stepPin, LOW);
    digitalWrite(motorRight_.stepPin, LOW);

    enable(true);

    // Setup LEDC channels (initial freq is placeholder)
    ledcSetup(motorLeft_.channel, 1000, LEDC_RES_BITS);
    ledcAttachPin(motorLeft_.stepPin, motorLeft_.channel);
    ledcWrite(motorLeft_.channel, 0);

    ledcSetup(motorRight_.channel, 1000, LEDC_RES_BITS);
    ledcAttachPin(motorRight_.stepPin, motorRight_.channel);
    ledcWrite(motorRight_.channel, 0);

    // Default DIR
    setDirection(MotorSide::Left, true);
    setDirection(MotorSide::Right, true);
    motorLeft_.lastForward = true;
    motorRight_.lastForward = true;
}

void StepperControl::enable(bool enable)
{
    // A4988 normal: LOW=enable, HIGH=disable
    digitalWrite(ENABLE_PIN, enable ? LOW : HIGH);
}

void StepperControl::stop(MotorSide side)
{
    MotorLEDC &m = getMotor(side);
    ledcWrite(m.channel, 0); // duty = 0 => stop stepping
    m.lastStepHz = 0.0f;
}

void StepperControl::setDirection(MotorSide side, bool forward)
{
    MotorLEDC &m = getMotor(side);
    bool logic = m.invertDir ? !forward : forward;
    digitalWrite(m.dirPin, logic ? HIGH : LOW);
}

void StepperControl::setStepHz(MotorSide side, float f_step_hz)
{
    MotorLEDC &m = getMotor(side);

    float f_clamped = clampStepHz(f_step_hz);
    if (f_clamped < STEP_HZ_MIN_RUN)
    {
        // Consider as stop
        stop(side);
        return;
    }

    ledcChangeFrequency(m.channel, (uint32_t)f_clamped, LEDC_RES_BITS);
    ledcWrite(m.channel, LEDC_DUTY_50);

    m.lastStepHz = f_clamped;
}

void StepperControl::setOmegaDegS(MotorSide side, float omega_deg_s)
{
    MotorLEDC &m = getMotor(side);

    if (fabsf(omega_deg_s) < OMEGA_DEADBAND_DEG_S)
    {
        // Consider as stop
        stop(side);
        return;
    }

    // Deg to Rad
    float omega_rad_s = degToRad(omega_deg_s);

    bool forward = (omega_rad_s >= 0.0f);

    // If direction changed: stop PWM -> change DIR -> start again (avoid glitch)
    if (forward != m.lastForward)
    {
        stop(side);
        delayMicroseconds(5);
        setDirection(side, forward);
        m.lastForward = forward;
        delayMicroseconds(5);
    }

    float f_step = omegaToStepHz(fabsf(omega_rad_s));
    setStepHz(side, f_step);
}

void StepperControl::setOmegaDegS(float omegaL_deg_s, float omegaR_deg_s)
{
    setOmegaDegS(MotorSide::Left, omegaL_deg_s);
    setOmegaDegS(MotorSide::Right, omegaR_deg_s);
}

void StepperControl::stopAll()
{
    stop(MotorSide::Left);
    stop(MotorSide::Right);
}

float StepperControl::omegaToStepHz(float omega_rad_s)
{
    // f_step = (omega_rad_s * STEPS_PER_REV) / (2 * pi)
    return (float)((double)omega_rad_s * (double)STEPS_PER_REV) / (2.0 * M_PI);
}

float StepperControl::clampStepHz(float f)
{
    if (f < 0.0f)
        f = -f;
    if (f > STEP_HZ_MAX)
        return STEP_HZ_MAX;
    return f;
}

MotorLEDC &StepperControl::getMotor(MotorSide side)
{
    if (side == MotorSide::Left)
        return motorLeft_;
    else
        return motorRight_;
}