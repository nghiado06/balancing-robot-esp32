#include <Arduino.h>
#include <AccelStepper.h>

/****************************************************************************************************
 * ========================================== [ STEPPER 1 ] =========================================
 ***************************************************************************************************/
#define DIR_L_PIN 27
#define STEP_L_PIN 14
#define ENABLE_PIN 16 // enable chung cho cả 2 driver (nếu bạn nối chung)

/****************************************************************************************************
 * ========================================== [ STEPPER 2 ] =========================================
 ***************************************************************************************************/
#define DIR_R_PIN 33
#define STEP_R_PIN 25

// ============================ Tùy chỉnh theo driver của bạn ============================
// Nhiều driver (A4988/DRV8825/TMC...) thường ENABLE active LOW (LOW = enable, HIGH = disable).
// Nếu driver của bạn ngược lại, đổi ENABLE_ACTIVE_LOW = false.
static constexpr bool ENABLE_ACTIVE_LOW = true;

// Tạo 2 stepper theo mode DRIVER (STEP/DIR)
AccelStepper StepperL(AccelStepper::DRIVER, STEP_L_PIN, DIR_L_PIN);
AccelStepper StepperR(AccelStepper::DRIVER, STEP_R_PIN, DIR_R_PIN);

// Demo: số bước đi tới/ lui
static constexpr long MOVE_STEPS = 2000;

void enableDrivers(bool en)
{
    if (ENABLE_ACTIVE_LOW)
        digitalWrite(ENABLE_PIN, en ? LOW : HIGH);
    else
        digitalWrite(ENABLE_PIN, en ? HIGH : LOW);
}

void setup()
{
    Serial.begin(115200);

    // Enable pin
    pinMode(ENABLE_PIN, OUTPUT);
    enableDrivers(true);

    // Cấu hình chuyển động
    StepperL.setMaxSpeed(1200);     // steps/s
    StepperL.setAcceleration(2000); // steps/s^2

    StepperR.setMaxSpeed(1200);
    StepperR.setAcceleration(2000);

    // (Tuỳ chọn) đảo chiều 1 bên nếu 2 motor lắp đối xứng mà chạy ngược nhau
    // StepperR.setPinsInverted(true, false, false); // đảo DIR (tham số: dir, step, enable)

    // Vị trí ban đầu
    StepperL.setCurrentPosition(0);
    StepperR.setCurrentPosition(0);

    // Target đầu tiên
    StepperL.moveTo(+MOVE_STEPS);
    StepperR.moveTo(+MOVE_STEPS);

    Serial.println("Start AccelStepper dual-motor demo.");
}

void loop()
{
    // Chạy non-blocking: phải gọi liên tục
    StepperL.run();
    StepperR.run();

    // Khi cả 2 đã tới đích -> đổi hướng
    if (StepperL.distanceToGo() == 0 && StepperR.distanceToGo() == 0)
    {
        long nextL = -StepperL.currentPosition();
        long nextR = -StepperR.currentPosition();

        StepperL.moveTo(nextL);
        StepperR.moveTo(nextR);

        Serial.print("Next targets: L=");
        Serial.print(nextL);
        Serial.print("  R=");
        Serial.println(nextR);

        // delay(300); // nghỉ nhẹ (tuỳ bạn, bỏ cũng được)
    }

    // (Tuỳ chọn) Nếu muốn disable driver khi đứng yên để đỡ nóng:
    // if (StepperL.distanceToGo() == 0 && StepperR.distanceToGo() == 0) enableDrivers(false);
    // else enableDrivers(true);
}
