#include "MoveAttitudeTask.h"
#include "MoveAttitude.hpp"

extern App::IMU::MPU6050_c *mpu6050;
extern App::Translate *translate;

App::MoveAttitude *moveAttitude = new App::MoveAttitude();

void moveAttitudeTask(void const * argument)
{
   
    moveAttitude->Init( mpu6050, translate);
    while (1)
    {
        moveAttitude->Update();
        osDelay(1);
    }
}
