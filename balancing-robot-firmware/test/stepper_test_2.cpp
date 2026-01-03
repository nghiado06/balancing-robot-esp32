#include <Arduino.h>
#include <math.h>

/*
 * =========================================================
 * ESP32 + A4988 Stepper Pulse Generator using LEDC (HW PWM)
 * - 2 motors (STEP/DIR), mỗi motor 1 kênh LEDC
 * - Set tốc độ bằng step frequency (steps/s) hoặc omega (rad/s)
 * - Demo: chạy 5s -> dừng 1s -> chạy ngược 5s -> dừng 1s -> lặp
 * =========================================================
 */

// ======================= MOTOR CONFIG =======================
static constexpr uint32_t FULL_STEPS_PER_REV = 200;                       // 1.8deg
static constexpr uint32_t MICROSTEP = 16;                                 // ví dụ 1/16
static constexpr uint32_t STEPS_PER_REV = FULL_STEPS_PER_REV * MICROSTEP; // 3200

// ======================= GPIO =======================
static constexpr int PIN_STEP_L = 14;
static constexpr int PIN_DIR_L = 27;

static constexpr int PIN_STEP_R = 25;
static constexpr int PIN_DIR_R = 33;

static constexpr int PIN_ENABLE = 16; // A4988: thường LOW=enable

static constexpr bool INVERT_DIR_L = false;
static constexpr bool INVERT_DIR_R = false;

// ======================= LEDC CONFIG =======================
static constexpr int LEDC_CH_L = 0;
static constexpr int LEDC_CH_R = 1;
static constexpr int LEDC_RES_BITS = 10;                            // 0..1023
static constexpr uint32_t LEDC_DUTY_50 = 1U << (LEDC_RES_BITS - 1); // ~50%

// Giới hạn step frequency (steps/s) để an toàn test
static constexpr float STEP_HZ_MIN_RUN = 1.0f; // <1Hz coi như stop
static constexpr float STEP_HZ_MAX = 20000.0f; // bạn có thể tăng nếu driver & motor chịu

// Deadband quanh 0 để tránh đảo chiều liên tục
static constexpr float OMEGA_DEADBAND = 0.3f; // rad/s

enum class MotorSide : uint8_t
{
    Left = 0,
    Right = 1
};

struct MotorLEDC
{
    int stepPin;
    int dirPin;
    bool invertDir;
    int channel;

    bool lastForward;
    float lastStepHz;
};

MotorLEDC mL{PIN_STEP_L, PIN_DIR_L, INVERT_DIR_L, LEDC_CH_L, true, 0.0f};
MotorLEDC mR{PIN_STEP_R, PIN_DIR_R, INVERT_DIR_R, LEDC_CH_R, true, 0.0f};

// ======================= UTILS =======================
static inline float omegaToStepHz(float omega_rad_s)
{
    // steps/s = stepsPerRev * omega / (2π)
    return (float)((double)STEPS_PER_REV * (double)omega_rad_s / (2.0 * M_PI));
}

static inline float clampStepHz(float f)    
{
    if (f < 0)
        f = -f;
    if (f > STEP_HZ_MAX)
        f = STEP_HZ_MAX;
    return f;
}

static inline MotorLEDC &getM(MotorSide side)
{
    return (side == MotorSide::Left) ? mL : mR;
}

// ======================= DRIVER ENABLE =======================
void motorsEnable(bool enable)
{
    // A4988 thường: LOW=enable, HIGH=disable
    digitalWrite(PIN_ENABLE, enable ? LOW : HIGH);
}

// ======================= DIR =======================
void motorSetDirection(MotorSide side, bool forward)
{
    MotorLEDC &m = getM(side);
    bool logic = m.invertDir ? !forward : forward;
    digitalWrite(m.dirPin, logic ? HIGH : LOW);
}

// ======================= LEDC STEP CONTROL =======================
void motorStopLEDC(MotorSide side)
{
    MotorLEDC &m = getM(side);
    ledcWrite(m.channel, 0); // duty = 0 => ngừng phát xung
    m.lastStepHz = 0.0f;
}

// Set step frequency bằng LEDC (steps/s). PWM duty 50%.
void motorSetStepHzLEDC(MotorSide side, float f_step_hz)
{
    MotorLEDC &m = getM(side);

    f_step_hz = clampStepHz(f_step_hz);
    if (f_step_hz < STEP_HZ_MIN_RUN)
    {
        motorStopLEDC(side);
        return;
    }

    // Đổi tần số PWM phần cứng
    ledcChangeFrequency(m.channel, (uint32_t)f_step_hz, LEDC_RES_BITS);
    ledcWrite(m.channel, LEDC_DUTY_50);

    m.lastStepHz = f_step_hz;
}

// Set omega (rad/s): tự đổi chiều theo dấu, phát step bằng LEDC
void motorSetOmegaRadS_LEDC(MotorSide side, float omega_rad_s)
{
    MotorLEDC &m = getM(side);

    // Deadband để tránh đảo chiều liên tục quanh 0
    if (fabsf(omega_rad_s) < OMEGA_DEADBAND)
    {
        motorStopLEDC(side);
        return;
    }

    bool forward = (omega_rad_s >= 0.0f);

    // Nếu đổi chiều: tắt PWM -> đổi DIR -> bật lại (tránh glitch)
    if (forward != m.lastForward)
    {
        motorStopLEDC(side);
        delayMicroseconds(5);
        motorSetDirection(side, forward);
        m.lastForward = forward;
        delayMicroseconds(5);
    }

    float f_step = omegaToStepHz(fabsf(omega_rad_s));
    motorSetStepHzLEDC(side, f_step);
}

void motorsSetOmegaRadS_LEDC(float omegaL, float omegaR)
{
    motorSetOmegaRadS_LEDC(MotorSide::Left, omegaL);
    motorSetOmegaRadS_LEDC(MotorSide::Right, omegaR);
}

void motorsStopAll()
{
    motorStopLEDC(MotorSide::Left);
    motorStopLEDC(MotorSide::Right);
}

// ======================= INIT =======================
void motorsInitLEDC()
{
    pinMode(PIN_STEP_L, OUTPUT);
    pinMode(PIN_DIR_L, OUTPUT);
    pinMode(PIN_STEP_R, OUTPUT);
    pinMode(PIN_DIR_R, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);

    digitalWrite(PIN_STEP_L, LOW);
    digitalWrite(PIN_STEP_R, LOW);

    motorsEnable(true);

    // Setup LEDC channels (freq khởi tạo chỉ là placeholder)
    ledcSetup(mL.channel, 1000, LEDC_RES_BITS);
    ledcAttachPin(mL.stepPin, mL.channel);
    ledcWrite(mL.channel, 0);

    ledcSetup(mR.channel, 1000, LEDC_RES_BITS);
    ledcAttachPin(mR.stepPin, mR.channel);
    ledcWrite(mR.channel, 0);

    // DIR mặc định
    motorSetDirection(MotorSide::Left, true);
    motorSetDirection(MotorSide::Right, true);
    mL.lastForward = true;
    mR.lastForward = true;
}

// ======================= DEMO =======================
enum class DemoState : uint8_t
{
    RunFwd,
    Stop1,
    RunRev,
    Stop2
};
DemoState st = DemoState::RunFwd;
uint32_t stMs = 0;

static constexpr uint32_t RUN_MS = 5000;
static constexpr uint32_t STOP_MS = 1000;

// Tốc độ test (rad/s). Bạn đổi số này để nghe khác biệt.
static constexpr float TEST_OMEGA = 35.0f;

void setup()
{
    Serial.begin(115200);
    delay(200);

    motorsInitLEDC();
    Serial.println("LEDC stepper demo: run 5s -> stop 1s -> reverse 5s -> stop 1s -> repeat");

    stMs = millis();
    motorsSetOmegaRadS_LEDC(TEST_OMEGA, TEST_OMEGA);
    Serial.println("State: Run Forward");
}

void loop()
{
    uint32_t now = millis();

    switch (st)
    {
    case DemoState::RunFwd:
        if (now - stMs >= RUN_MS)
        {
            st = DemoState::Stop1;
            stMs = now;
            motorsStopAll();
            Serial.println("State: Stop");
        }
        break;

    case DemoState::Stop1:
        if (now - stMs >= STOP_MS)
        {
            st = DemoState::RunRev;
            stMs = now;
            motorsSetOmegaRadS_LEDC(-TEST_OMEGA, -TEST_OMEGA);
            Serial.println("State: Run Reverse");
        }
        break;

    case DemoState::RunRev:
        if (now - stMs >= RUN_MS)
        {
            st = DemoState::Stop2;
            stMs = now;
            motorsStopAll();
            Serial.println("State: Stop");
        }
        break;

    case DemoState::Stop2:
        if (now - stMs >= STOP_MS)
        {
            st = DemoState::RunFwd;
            stMs = now;
            motorsSetOmegaRadS_LEDC(TEST_OMEGA, TEST_OMEGA);
            Serial.println("State: Run Forward");
        }
        break;
    }
}
