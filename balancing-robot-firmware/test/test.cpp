/* =========================================================
 *  ROBOT CÂN BẰNG 2 BÁNH – KIẾN TRÚC TỐI GIẢN
 *  - A() = ComputeControl(): hàm tính toán điều khiển CHÍNH
 *  - B() = Calib & Auto-Trim: hiệu chuẩn + bù chậm
 *  - C() = ModeIdleHandler(): không điều khiển (tự cân bằng)
 *  - D() = ModeBLEHandler(): điều khiển bằng tay cầm (BLE)
 *  - E() = ModeESPNOWHandler(): điều khiển bằng cử chỉ tay (ESP-NOW)
 *  Phụ: Complementary filter, đổi mm/s->STEP, ramp, fail-safe...
 * ========================================================= */

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <AccelStepper.h> // có thể tắt và dùng ISR timer – xem cuối file

// ====== CHỌN TRÌNH PHÁT XUNG ======
#define USE_ACCELSTEPPER 1

// ====== PIN HARDWARE (điền theo mạch của bạn) ======
#define STEP_L 25
#define DIR_L 26
#define STEP_R 27
#define DIR_R 14
#define ENA_PIN 12 // Enable A4988 (LOW: enable tùy module – kiểm tra)

#define BTN_CALIB 32
#define BTN_MODE 33
#define BTN_ARM 15
#define BUZZER_PIN 2

// ====== IMU ======
MPU6050 mpu(Wire);

// ====== THÔNG SỐ CƠ KHÍ & RÀNG BUỘC ======
const float R_mm = 35.0f;      // bán kính bánh
const float L_mm = 160.0f;     // khoảng cách 2 bánh
const int STEPS_PER_REV = 200; // NEMA17
const int MICROSTEP = 16;      // A4988
const float CIRC_mm = 2.0f * PI * R_mm;
const float STEPS_PER_MM = (STEPS_PER_REV * MICROSTEP) / CIRC_mm;

const float VMAX_mm_s = 400.0f; // giới hạn tốc độ tuyến tính
const float F_MAX_HZ = 4500.0f; // tần số step tối đa
const float DF_MAX_HZ = 600.0f; // độ dốc step mỗi tick (ramp)

// ====== PID & MAPPING ======
float Kp = 12.0f, Ki = 1.0f, Kd = 0.25f;
float k_v = 0.03f;             // đổi v_ref -> theta_ref (deg / (mm/s))
float theta_ref_limit = 12.0f; // giới hạn tham chiếu nghiêng

// ====== COMPLEMENTARY FILTER ======
const float ALPHA = 0.98f; // 0.96–0.99
float theta_hat = 0.0f;    // góc nghiêng ước lượng (deg)

// ====== CALIB + AUTO-TRIM (Hàm B) ======
struct Calib
{
    float gyro_bias_y_dps = 0.0f;  // bias gyro Y
    float angle_offset_deg = 0.0f; // tare: offset góc
    float theta_trim_deg = 0.0f;   // auto-trim chậm
} calib;

const float K_TRIM = 0.02f;  // 1/s ~ tau ≈ 50 s
const float TRIM_MAX = 3.0f; // ±3°

float Iterm = 0.0f; // tích phân PID

// ====== COMMANDS & SNAPSHOT ======
struct UserCmd
{
    float v_cmd_mm_s = 0.0f;  // lệnh tiến/lùi
    float yaw_cmd_dps = 0.0f; // lệnh quay (deg/s)
    uint32_t ts_ms = 0;       // timestamp gói lệnh
} userBLE, userESPNOW;

enum Mode
{
    MODE_IDLE = 0,
    MODE_BLE,
    MODE_ESPNOW
};
volatile Mode gMode = MODE_IDLE;
bool armed = false;

// ====== SIÊU ÂM / TRÁNH VẬT (điền phần đo khoảng cách của bạn) ======
float avoid_gain = 1.0f; // 0..1 (1=thoáng), g(d)
float v_dead = 8.0f;     // mm/s
float yaw_dead = 2.0f;   // deg/s

// ====== STEP TARGETS ======
float fL_cmd = 0, fR_cmd = 0; // Hz
bool dirL = true, dirR = true;

// ====== AccelStepper (nếu dùng) ======
#if USE_ACCELSTEPPER
AccelStepper mL(AccelStepper::DRIVER, STEP_L, DIR_L);
AccelStepper mR(AccelStepper::DRIVER, STEP_R, DIR_R);
#endif

// ====== TIỆN ÍCH ======
inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline float slew(float prev, float target, float dmax)
{
    float d = target - prev;
    if (d > dmax)
        d = dmax;
    else if (d < -dmax)
        d = -dmax;
    return prev + d;
}
inline float deg2rad(float d) { return d * PI / 180.0f; }
inline bool isPressed(int pin) { return digitalRead(pin) == LOW; } // nếu kéo lên/đấu phù hợp

/* =========================================================
 *  HÀM B: CALIB + AUTO-TRIM
 *  - doCalib(): TARE + GYRO BIAS (gọi khi DISARM, đang đứng yên)
 *  - updateAutoTrim(): bù rất chậm khi đứng yên
 * ========================================================= */

void doCalib(uint16_t samples = 2000, uint32_t us_dt = 1000)
{
    // Beep báo bắt đầu
    tone(BUZZER_PIN, 2000, 100);

    double sumGy = 0, sumThetaAcc = 0;

    for (uint16_t i = 0; i < samples; i++)
    {
        mpu.update();
        float gy = mpu.getGyroY(); // deg/s
        float ax = mpu.getAccX();
        float ay = mpu.getAccY();
        float az = mpu.getAccZ();
        // chú ý trục: tùy gắn MPU mà công thức atan2 thay đổi; dùng AccX vs sqrt(ay^2+az^2) là ví dụ
        float theta_acc = atan2f(ax, sqrtf(ay * ay + az * az)) * 57.29578f;
        sumGy += gy;
        sumThetaAcc += theta_acc;
        delayMicroseconds(us_dt);
    }
    calib.gyro_bias_y_dps = (float)(sumGy / samples);
    calib.angle_offset_deg = (float)(sumThetaAcc / samples);
    calib.theta_trim_deg = 0.0f; // reset trim
    theta_hat = 0.0f;
    Iterm = 0.0f;

    // TODO: lưu calib vào NVS nếu muốn

    // Beep báo xong
    tone(BUZZER_PIN, 2600, 120);
}

// chỉ gọi khi gần đứng yên (điều kiện kích hoạt ở dưới)
void updateAutoTrim(float dt)
{
    // EMA cực chậm: theta_trim <- theta_trim + k*(theta_hat - theta_trim)*dt
    calib.theta_trim_deg += K_TRIM * (theta_hat - calib.theta_trim_deg) * dt;
    calib.theta_trim_deg = clampf(calib.theta_trim_deg, -TRIM_MAX, +TRIM_MAX);
}

/* =========================================================
 *  Complementary Filter: cập nhật theta_hat (deg) & omega_y (deg/s)
 * ========================================================= */
struct ImuObs
{
    float theta_deg; // theta_hat sau lọc
    float omega_dps; // gyro Y đã bỏ bias
};
ImuObs readIMU(float dt)
{
    mpu.update();
    float gy = mpu.getGyroY() - calib.gyro_bias_y_dps; // deg/s
    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();
    float theta_acc = atan2f(ax, sqrtf(ay * ay + az * az)) * 57.29578f - calib.angle_offset_deg;
    theta_hat = ALPHA * (theta_hat + gy * dt) + (1.0f - ALPHA) * theta_acc;
    return {theta_hat, gy};
}

/* =========================================================
 *  ĐỔI V(mm/s) -> (STEP Hz, DIR)
 * ========================================================= */
void vel_to_step(float v_mm_s, float &f_hz, bool &dir)
{
    dir = (v_mm_s >= 0.0f);
    float steps_per_s = fabsf(v_mm_s) * STEPS_PER_MM;
    f_hz = clampf(steps_per_s, 0.0f, F_MAX_HZ);
}

/* =========================================================
 *  APPLY MOTOR TARGETS (AcccelStepper hoặc ISR)
 * ========================================================= */
#if USE_ACCELSTEPPER
void motorsInit()
{
    pinMode(ENA_PIN, OUTPUT);
    digitalWrite(ENA_PIN, LOW); // enable (tùy module)
    mL.setMaxSpeed(F_MAX_HZ);
    mR.setMaxSpeed(F_MAX_HZ);
    mL.setAcceleration(0);
    mR.setAcceleration(0); // dùng runSpeed
}
void applyMotorTargets(float fL, bool dL, float fR, bool dR)
{
    digitalWrite(DIR_L, dL ? HIGH : LOW);
    digitalWrite(DIR_R, dR ? HIGH : LOW);
    mL.setSpeed(dL ? fL : -fL); // steps/s
    mR.setSpeed(dR ? fR : -fR);
    // GỌI THẬT THƯỜNG XUYÊN trong loop() hoặc timer:
    mL.runSpeed();
    mR.runSpeed();
}
#else
// === KHUNG ISR TIMER (nếu bạn muốn thời gian thực hơn) ===
// TODO: triển khai hw_timer_t + DDS phát xung STEP_L/STEP_R dựa fL_cmd/fR_cmd
void motorsInit()
{
    pinMode(ENA_PIN, OUTPUT);
    digitalWrite(ENA_PIN, LOW);
    pinMode(STEP_L, OUTPUT);
    pinMode(DIR_L, OUTPUT);
    pinMode(STEP_R, OUTPUT);
    pinMode(DIR_R, OUTPUT);
    // ... timerBegin(), timerAttachInterrupt(), timerAlarmWrite() ...
}
void applyMotorTargets(float fL, bool dL, float fR, bool dR)
{
    // Ghi mục tiêu cho ISR: fL_cmd,fR_cmd,dirL,dirR (biến volatile)
    dirL = dL;
    dirR = dR;
    fL_cmd = fL;
    fR_cmd = fR;
    digitalWrite(DIR_L, dL ? HIGH : LOW);
    digitalWrite(DIR_R, dR ? HIGH : LOW);
    // ISR sẽ lo phát xung đều theo f*_cmd
}
#endif

/* =========================================================
 *  HÀM A – ComputeControl(): tính toàn bộ điều khiển & gọi motor
 *  INPUT:
 *    - imu:  theta_hat (deg), omega_y (deg/s)
 *    - cmd:  v_cmd (mm/s), yaw_cmd (deg/s)
 *    - avoid_gain (0..1)
 *    - armed, link_ok
 *  LOGIC:
 *    v_ref -> theta_ref -> PID -> v_fwd -> (vL,vR) -> (f,dir) -> ramp -> motor
 * ========================================================= */

struct ControlIn
{
    float v_cmd_mm_s;
    float yaw_cmd_dps;
    float avoid_gain;
    bool link_ok;
    bool armed;
};
struct ControlOut
{
    float fL, fR;
    bool dL, dR;
    float vL, vR, v_fwd, theta_ref;
};

ControlOut ComputeControl(const ImuObs &imu, const ControlIn &u, float dt)
{
    static float prev_fL = 0.0f, prev_fR = 0.0f;

    // 1) Chuẩn hóa lệnh (dead-zone, limit)
    float v_cmd = (fabsf(u.v_cmd_mm_s) < v_dead) ? 0.0f : u.v_cmd_mm_s;
    float yaw = (fabsf(u.yaw_cmd_dps) < yaw_dead) ? 0.0f : u.yaw_cmd_dps;
    v_cmd = clampf(v_cmd, -VMAX_mm_s, +VMAX_mm_s);

    // 2) Tránh vật + mất link
    float v_ref = v_cmd * u.avoid_gain;
    float yaw_ref = u.link_ok ? yaw : 0.0f;
    if (!u.link_ok)
        v_ref = 0.0f;

    // 3) Tham chiếu nghiêng (kèm auto-trim)
    float theta_ref = clampf(k_v * v_ref, -theta_ref_limit, +theta_ref_limit) + calib.theta_trim_deg;

    // 4) PID cân bằng -> v_fwd
    float e = theta_ref - imu.theta_deg;
    // Anti-windup nghèo: khóa I khi v_fwd sẽ bão hòa
    float tentative_I = Iterm + Ki * e * dt;

    float v_fwd_raw = Kp * e - Kd * imu.omega_dps + tentative_I;
    float v_fwd = clampf(v_fwd_raw, -VMAX_mm_s, +VMAX_mm_s);
    // nếu không bão hòa -> nhận I, bão hòa -> khóa I
    if (v_fwd == v_fwd_raw)
        Iterm = tentative_I;

    // 5) Trộn rẽ (động học chuẩn)
    float Omega = deg2rad(yaw_ref); // rad/s
    float vL = v_fwd - 0.5f * Omega * L_mm;
    float vR = v_fwd + 0.5f * Omega * L_mm;
    vL = clampf(vL, -VMAX_mm_s, +VMAX_mm_s);
    vR = clampf(vR, -VMAX_mm_s, +VMAX_mm_s);

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
    applyMotorTargets(fL_r, dL, fR_r, dR);

    return {fL_r, fR_r, dL, dR, vL, vR, v_fwd, theta_ref};
}

/* =========================================================
 *  HÀM C/D/E – XỬ LÝ MODE
 *  - C(): IDLE – chỉ cân bằng; v_cmd=0, yaw=0; có thể bấm CALIB
 *  - D(): BLE  – đọc BLE -> v_cmd,yaw; có thể bấm CALIB
 *  - E(): ESPNOW – đọc ESP-NOW -> v_cmd,yaw; có thể bấm CALIB
 * ========================================================= */

// ======= STUB ĐỌC TAY CẦM – BẠN ĐIỀN =========
bool readBLE(float &v_cmd, float &yaw_cmd)
{
    // TODO: đọc BLE; trả true nếu có gói mới
    return false;
}
bool readESPNOW(float &v_cmd, float &yaw_cmd)
{
    // TODO: đọc ESP-NOW; trả true nếu có gói mới
    return false;
}
// ==============================================

ControlOut ModeIdleHandler(float dt)
{
    ImuObs imu = readIMU(dt);

    // Điều kiện kích hoạt auto-trim (gần đứng yên)
    bool nearlyStill = (fabsf(imu.omega_dps) < 5.0f) && (fabsf(imu.theta_deg) < 5.0f);
    static uint32_t stillStart = 0;
    if (nearlyStill)
    {
        if (stillStart == 0)
            stillStart = millis();
        else if (millis() - stillStart > 1200)
            updateAutoTrim(dt); // dwell 1.2 s
    }
    else
        stillStart = 0;

    ControlIn u{0.0f, 0.0f, avoid_gain, true, armed};
    return ComputeControl(imu, u, dt);
}

ControlOut ModeBLEHandler(float dt)
{
    ImuObs imu = readIMU(dt);

    // Đọc BLE
    float v = 0, yaw = 0;
    bool got = readBLE(v, yaw);
    if (got)
    {
        userBLE.v_cmd_mm_s = v;
        userBLE.yaw_cmd_dps = yaw;
        userBLE.ts_ms = millis();
    }
    bool link_ok = (millis() - userBLE.ts_ms) <= 300;

    // Auto-trim khi gần đứng yên
    bool nearlyStill = (fabsf(imu.omega_dps) < 5.0f) && (fabsf(userBLE.v_cmd_mm_s) < VMAX_mm_s * 0.05f) && (fabsf(userBLE.yaw_cmd_dps) < 3.0f);
    static uint32_t stillStart = 0;
    if (nearlyStill)
    {
        if (stillStart == 0)
            stillStart = millis();
        else if (millis() - stillStart > 1200)
            updateAutoTrim(dt);
    }
    else
        stillStart = 0;

    ControlIn u{userBLE.v_cmd_mm_s, userBLE.yaw_cmd_dps, avoid_gain, link_ok, armed};
    return ComputeControl(imu, u, dt);
}

ControlOut ModeESPNOWHandler(float dt)
{
    ImuObs imu = readIMU(dt);

    float v = 0, yaw = 0;
    bool got = readESPNOW(v, yaw);
    if (got)
    {
        userESPNOW.v_cmd_mm_s = v;
        userESPNOW.yaw_cmd_dps = yaw;
        userESPNOW.ts_ms = millis();
    }
    bool link_ok = (millis() - userESPNOW.ts_ms) <= 300;

    bool nearlyStill = (fabsf(imu.omega_dps) < 5.0f) && (fabsf(userESPNOW.v_cmd_mm_s) < VMAX_mm_s * 0.05f) && (fabsf(userESPNOW.yaw_cmd_dps) < 3.0f);
    static uint32_t stillStart = 0;
    if (nearlyStill)
    {
        if (stillStart == 0)
            stillStart = millis();
        else if (millis() - stillStart > 1200)
            updateAutoTrim(dt);
    }
    else
        stillStart = 0;

    ControlIn u{userESPNOW.v_cmd_mm_s, userESPNOW.yaw_cmd_dps, avoid_gain, link_ok, armed};
    return ComputeControl(imu, u, dt);
}

/* =========================================================
 *  NÚT NHẤN & STATE (ARM/DISARM, MODE, CALIB)
 * ========================================================= */
void handleButtons()
{
    static uint32_t tCalib = 0, tMode = 0, tArm = 0;
    // Debounce đơn giản
    if (isPressed(BTN_ARM) && millis() - tArm > 200)
    {
        armed = !armed;
        digitalWrite(ENA_PIN, armed ? LOW : HIGH); // tùy module
        tone(BUZZER_PIN, armed ? 3000 : 1500, 80);
        tArm = millis();
    }
    if (isPressed(BTN_MODE) && millis() - tMode > 300)
    {
        gMode = (gMode == MODE_IDLE) ? MODE_BLE : (gMode == MODE_BLE) ? MODE_ESPNOW
                                                                      : MODE_IDLE;
        tone(BUZZER_PIN, 2200, 80);
        // khi đổi mode: xả lệnh về 0 một nhịp
        userBLE = {};
        userESPNOW = {};
        tMode = millis();
    }
    if (isPressed(BTN_CALIB) && millis() - tCalib > 500)
    {
        if (!armed)
        {
            doCalib();
        } // chỉ calib khi DISARM
        else
            tone(BUZZER_PIN, 700, 100); // cảnh báo không cho calib lúc ARMED
        tCalib = millis();
    }
}

/* =========================================================
 *  SETUP / LOOP
 * ========================================================= */
void setup()
{
    Serial.begin(115200);
    pinMode(BTN_CALIB, INPUT_PULLUP);
    pinMode(BTN_MODE, INPUT_PULLUP);
    pinMode(BTN_ARM, INPUT_PULLUP);
    pinMode(BUZZER_PIN, OUTPUT);

    // IMU
    Wire.begin();
    mpu.begin();
    mpu.calcGyroOffsets(true); // nhanh – ta vẫn tự calib lại
    doCalib();                 // tare + gyro bias

    // Motor
    pinMode(DIR_L, OUTPUT);
    pinMode(DIR_R, OUTPUT);
    motorsInit();

    // Enable driver ban đầu
    digitalWrite(ENA_PIN, HIGH); // disable
    armed = false;

    tone(BUZZER_PIN, 1800, 100);
}

void loop()
{
    static uint32_t last = micros();
    uint32_t now = micros();
    float dt = (now - last) * 1e-6f; // s
    if (dt < 0.002f)
    { // ~500 Hz control
// Nếu dùng AccelStepper: vẫn cần gọi runSpeed() rất thường xuyên:
#if USE_ACCELSTEPPER
        mL.runSpeed();
        mR.runSpeed();
#endif
        handleButtons();
        return;
    }
    last = now;

    handleButtons();

    // ====== CHỌN MODE ======
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
        ModeESPNOWHandler(dt);
    }

// Nếu dùng AccelStepper: chạy lại 1 lần nữa cho nhịp xung mượt
#if USE_ACCELSTEPPER
    mL.runSpeed();
    mR.runSpeed();
#endif
}
