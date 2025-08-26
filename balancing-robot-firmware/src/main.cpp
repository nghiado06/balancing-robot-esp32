#include <Arduino.h>
#include "Pin_Config.h"
#include "IoHwAb_Mpu.h"

void setup()
{
  Serial.begin(115200);
  IoHwAb_Mpu_Init();
}

void loop()
{
  Serial.println("Read AngleX: " + String(IoHwAb_Mpu_ReadAngle()) + " | " + "Read GyroX: " + String(IoHwAb_Mpu_ReadGyro()));
  delay(100);
}
