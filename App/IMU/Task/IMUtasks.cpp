#include "IMUtasks.h"
#include "mpu6050.hpp"

App::IMU::MPU6050_c *mpu6050 = new App::IMU::MPU6050_c(App::IMU::RotationOrder::ZYX, false);

void imuTask(void const * argument)
{
    mpu6050->Init();
    while (1)
    {
        mpu6050->Update();
    }
    
}
