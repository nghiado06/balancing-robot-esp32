#include <Arduino.h>
#include <math.h>

/*
 * =========================================================
 * ESP32 Step/Dir Pulse Generator (A4988/DRV8825/TMC...) - Dual Motor
 * - Timer tick cố định (TICK_HZ)
 * - Mỗi motor có phase accumulator Q0.32 riêng -> tạo step theo tần số mục tiêu
 * - Hỗ trợ set tốc độ + chiều theo dấu (omega âm -> đảo DIR)
 * - API sạch: chọn motor LEFT/RIGHT hoặc gọi hàm cho cả 2
 * =========================================================
 */

// ======================= CẤU HÌNH CHUNG =======================
static constexpr uint32_t TICK_HZ = 20000;             // 20 kHz => 50 us/tick
static constexpr uint32_t STEP_HIGH_TICKS = 1;         // STEP HIGH giữ 1 tick (50 us)
static constexpr uint32_t STEP_LOW_COOLDOWN_TICKS = 1; // STEP LOW tối thiểu 1 tick (an toàn)
static constexpr double TWO_POW_32 = 4294967296.0;     // 2^32 cho Q0.32

// Motor config (1.8deg, microstep 1/16)
static constexpr uint32_t FULL_STEPS_PER_REV = 200;
static constexpr uint32_t MICROSTEP = 16;
static constexpr uint32_t STEPS_PER_REV = FULL_STEPS_PER_REV * MICROSTEP; // 3200 steps/rev

// ======================= CHÂN GPIO =======================
static constexpr int PIN_STEP_L = 14;
static constexpr int PIN_DIR_L = 27;

static constexpr int PIN_STEP_R = 25;
static constexpr int PIN_DIR_R = 33;

static constexpr int PIN_ENABLE = 16; // A4988 EN thường LOW=enable (hãy kiểm tra driver bạn)

// Nếu cơ khí/wiring khiến “forward” bị ngược, bạn đảo ở đây:
static constexpr bool INVERT_DIR_L = false;
static constexpr bool INVERT_DIR_R = false;

// ======================= FAST GPIO (DÙNG TRONG ISR) =======================
static inline void IRAM_ATTR gpioSetHigh(int pin)
{
    if (pin < 32)
        GPIO.out_w1ts = (1UL << pin);
    else
        GPIO.out1_w1ts.val = (1UL << (pin - 32));
}

static inline void IRAM_ATTR gpioSetLow(int pin)
{
    if (pin < 32)
        GPIO.out_w1tc = (1UL << pin);
    else
        GPIO.out1_w1tc.val = (1UL << (pin - 32));
}

// ======================= MOTOR API =======================
enum class MotorSide : uint8_t
{
    Left = 0,
    Right = 1
};

struct MotorState
{
    int stepPin;
    int dirPin;
    bool invertDir;

    // Phase accumulator Q0.32
    volatile uint32_t phaseAcc;
    volatile uint32_t phaseInc;

    // Pulse shaping
    volatile bool stepHigh;
    volatile uint32_t highTicksRemain;
    volatile uint32_t lowCooldownTicks;
};

static MotorState g_motor[2] = {
    // Left
    {PIN_STEP_L, PIN_DIR_L, INVERT_DIR_L, 0, 0, false, 0, 0},
    // Right
    {PIN_STEP_R, PIN_DIR_R, INVERT_DIR_R, 0, 0, false, 0, 0}};

// Timer + lock
static hw_timer_t *g_timer = nullptr;
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// ======================= HÀM NỘI BỘ =======================
static inline MotorState &M(MotorSide side)
{
    return g_motor[static_cast<uint8_t>(side)];
}

static inline void IRAM_ATTR emitStepPulse(MotorState &m)
{
    // Không phát xung nếu đang HIGH hoặc đang cooldown LOW
    if (m.stepHigh)
        return;
    if (m.lowCooldownTicks > 0)
        return;

    gpioSetHigh(m.stepPin);
    m.stepHigh = true;
    m.highTicksRemain = STEP_HIGH_TICKS;
}

static inline void IRAM_ATTR tickOneMotor(MotorState &m)
{
    // (1) Hạ STEP nếu đang HIGH
    if (m.stepHigh)
    {
        if (m.highTicksRemain > 0)
            m.highTicksRemain--;
        if (m.highTicksRemain == 0)
        {
            gpioSetLow(m.stepPin);
            m.stepHigh = false;
            m.lowCooldownTicks = STEP_LOW_COOLDOWN_TICKS; // đảm bảo LOW tối thiểu
        }
    }
    else
    {
        // (2) Đếm cooldown LOW
        if (m.lowCooldownTicks > 0)
            m.lowCooldownTicks--;
    }

    // (3) Nếu phaseInc = 0 -> dừng
    const uint32_t inc = m.phaseInc;
    if (inc == 0)
        return;

    // (4) Phase accumulator: overflow => phát 1 step
    const uint32_t prev = m.phaseAcc;
    m.phaseAcc += inc;

    if (m.phaseAcc < prev)
    {
        emitStepPulse(m);
    }
}

// ======================= ISR TIMER =======================
void IRAM_ATTR onTickISR()
{
    tickOneMotor(g_motor[0]); // Left
    tickOneMotor(g_motor[1]); // Right
}

// ======================= CHUYỂN ĐỔI TỐC ĐỘ =======================
// omega(rad/s) -> f_step(steps/s) : stepsPerRev / (2π) * omega
static inline float omegaToStepHz(float omega_rad_s)
{
    return (float)((double)STEPS_PER_REV * (double)omega_rad_s / (2.0 * M_PI));
}

// Giới hạn tần số step an toàn theo tick và pulse shaping
static inline float clampStepHz(float f_step_hz)
{
    if (f_step_hz < 0)
        f_step_hz = -f_step_hz;

    // Vì có HIGH 1 tick + LOW cooldown 1 tick -> max ~ TICK_HZ/2
    const float f_max = (float)TICK_HZ / (float)(STEP_HIGH_TICKS + STEP_LOW_COOLDOWN_TICKS);
    if (f_step_hz > f_max)
        f_step_hz = f_max;

    return f_step_hz;
}

// ======================= API PUBLIC =======================

// Enable/Disable driver chung
void motorsEnable(bool enable)
{
    // A4988 EN thường: LOW=enable, HIGH=disable
    digitalWrite(PIN_ENABLE, enable ? LOW : HIGH);
}

// Set DIR cho 1 motor
void motorSetDirection(MotorSide side, bool forward)
{
    MotorState &m = M(side);

    // xử lý invert nếu cần
    const bool dirLogic = m.invertDir ? !forward : forward;
    digitalWrite(m.dirPin, dirLogic ? HIGH : LOW);
}

// Set step frequency (steps/s) cho 1 motor (chỉ tần số, không đổi DIR)
void motorSetStepFrequencyHz(MotorSide side, float f_step_hz)
{
    MotorState &m = M(side);
    f_step_hz = clampStepHz(f_step_hz);

    const uint32_t newInc = (uint32_t)((double)f_step_hz * (TWO_POW_32 / (double)TICK_HZ));

    portENTER_CRITICAL(&g_mux);
    m.phaseInc = newInc;
    portEXIT_CRITICAL(&g_mux);
}

// Stop 1 motor
void motorStop(MotorSide side)
{
    motorSetStepFrequencyHz(side, 0.0f);
}

// Set omega (rad/s) cho 1 motor: tự xử lý chiều theo dấu
void motorSetOmegaRadS(MotorSide side, float omega_rad_s)
{
    // an toàn khi đổi chiều: dừng -> đổi DIR -> chạy lại
    motorStop(side);
    delayMicroseconds(50); // DIR setup time

    const bool forward = (omega_rad_s >= 0.0f);
    motorSetDirection(side, forward);

    const float f_step = omegaToStepHz(fabsf(omega_rad_s));
    motorSetStepFrequencyHz(side, f_step);
}

// ---- Wrapper cho cả 2 motor ----
void motorsStopAll()
{
    motorStop(MotorSide::Left);
    motorStop(MotorSide::Right);
}

void motorsSetOmegaRadS(float omegaLeft, float omegaRight)
{
    motorSetOmegaRadS(MotorSide::Left, omegaLeft);
    motorSetOmegaRadS(MotorSide::Right, omegaRight);
}

// ======================= INIT TIMER + GPIO =======================
void motorsInit()
{
    pinMode(PIN_STEP_L, OUTPUT);
    pinMode(PIN_DIR_L, OUTPUT);
    pinMode(PIN_STEP_R, OUTPUT);
    pinMode(PIN_DIR_R, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);

    gpioSetLow(PIN_STEP_L);
    gpioSetLow(PIN_STEP_R);

    motorsEnable(true);

    // Timer clock ESP32: 80 MHz
    // prescaler=80 => 1 MHz => 1 tick = 1 us
    g_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(g_timer, &onTickISR, true);

    const uint32_t alarm_us = 1000000UL / TICK_HZ;
    timerAlarmWrite(g_timer, alarm_us, true);
    timerAlarmEnable(g_timer);
}

// ======================= DEMO =======================
void setup()
{
    Serial.begin(115200);

    motorsInit();

    // Test: L quay âm, R quay dương
    float wL = -35.0f;
    float wR = 20.0f;

    motorsSetOmegaRadS(wL, wR);

    Serial.printf("Left omega=%.2f rad/s, Right omega=%.2f rad/s\n", wL, wR);
}

void loop()
{
    // Demo đổi chiều mỗi 5 giây
    static uint32_t t0 = millis();
    static bool flip = false;

    if (millis() - t0 >= 5000)
    {
        t0 = millis();
        flip = !flip;

        if (flip)
        {
            motorsSetOmegaRadS(-35.0f, -35.0f);
            Serial.println("Both: -35 rad/s");
        }
        else
        {
            motorsSetOmegaRadS(35.0f, 35.0f);
            Serial.println("Both: +35 rad/s");
        }
    }
}
