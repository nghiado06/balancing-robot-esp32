#ifndef IOHWAB_MPU_H
#define IOHWAB_MPU_H

#include <MPU6050_tockn.h>
#include <Wire.h>

void IoHwAb_Mpu_Init();

float IoHwAb_Mpu_ReadAngleX();

float IoHwAb_Mpu_ReadAngleY();

float IoHwAb_Mpu_ReadAngleZ();

float IoHwAb_Mpu_ReadGyro();

#endif