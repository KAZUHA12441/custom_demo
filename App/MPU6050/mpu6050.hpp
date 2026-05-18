#pragma once
#include "i2c.h"
#include "Alg_PID.hpp"
#include "bsp_dwt.hpp"
#include "filter.hpp"

namespace App
{
    class MPU6050_c
    {
    public:
       MPU6050_c();
       void Init(void);
       void READ_ACCEL(void);
       void READ_GYRO(void);
       void Handle(void);
    private:
         uint8_t accel_buffer[6];
         uint8_t gyro_buffer[6];
         int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
         int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
         float Accel[3];
         float Gyro[3];
         bool is_init = false; //是否纠正零偏
         float offset_gyro[3] = {0.0f, 0.0f, 0.0f}; //陀螺仪零偏
         uint16_t gyro_data_num = 0;
    };


}