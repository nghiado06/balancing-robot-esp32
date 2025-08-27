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
  delay(100);
}
