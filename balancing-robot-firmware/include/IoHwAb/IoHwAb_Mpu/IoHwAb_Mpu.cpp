#include "IoHwAb_Mpu.h"

MPU6050 mpu(Wire);

void IoHwAb_Mpu_Init()
{
    Wire.begin();
    mpu.begin();
    mpu.calcGyroOffsets(true);
}

float IoHwAb_Mpu_ReadAngle()
{
    mpu.update();
    return mpu.getAccX(); // or AccY
}

float IoHwAb_Mpu_ReadGyro()
{
    mpu.update();
    return mpu.getGyroX(); // or GyroY
}