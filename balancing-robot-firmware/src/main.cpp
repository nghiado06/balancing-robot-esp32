#include <Arduino.h>
#include "IoHwAb_Mpu.h"

void setup()
{
  Serial.begin(9600);
  IoHwAb_Mpu_Init();
}

void loop()
{
  Serial.print("accX : ");
  Serial.println(IoHwAb_Mpu_Read());
}