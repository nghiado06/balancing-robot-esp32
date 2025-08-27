#include "IoHwAb_Mpu.h"

MPU6050 mpu(Wire);

void IoHwAb_Mpu_Init()
{
#ifndef MPU_INIT
#define MPU_INIT
    Wire.begin();
    mpu.begin();
    mpu.calcGyroOffsets(true);
#endif
}

float IoHwAb_Mpu_ReadAngleX()
{
    mpu.update();
    return mpu.getAccX();
}

float IoHwAb_Mpu_ReadAngleY()
{
    mpu.update();
    return mpu.getAccY();
}

float IoHwAb_Mpu_ReadAngleZ()
{
    mpu.update();
    return mpu.getAccZ();
}

float IoHwAb_Mpu_ReadGyro()
{
    mpu.update();
    return mpu.getGyroX(); // or GyroY
}