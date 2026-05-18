#pragma once

#include "i2c.h"
#include "Alg_PID.hpp"
#include "bsp_dwt.hpp"
#include "filter.hpp"
#include "Mahony.hpp"
#include "config.h"
#include "solution.hpp"

namespace App
{
    namespace IMU
    {
        class MPU6050_c
        {
        public:
            MPU6050_c(RotationOrder order = RotationOrder::ZYX, bool rotation_matrix_update = false);
            void Init(void);
            void readACCEL(void);
            void readGYRO(void);
            void Update(void);
            void offsetGyroCalibrate(void);
            float *initBody(void);
            inline ZYZEulerAngle*getZYZEulerAngle(void);
            inline ZYXEulerAngle*getZYXEulerAngle(void);
            inline float *getRotationMatrix(void);
            inline float *getQuaternion(void);

        private:
            uint8_t accel_buffer[6];
            uint8_t gyro_buffer[6];
            int16_t accelf[3];
            int16_t gyrof[3];
            float Accel[3];
            float Gyro[3];
            bool is_init = false;                      // 是否纠正零偏
            float offset_gyro[3] = {0.0f, 0.0f, 0.0f}; // 陀螺仪零偏
            float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // 四元数
            ZYZEulerAngle zyz_eula_angle = {0,0,0}; // 欧拉角
            ZYXEulerAngle zyx_eula_angle = {0,0,0}; // 欧拉角
            float rotation_matrix[3][3] = {0.0f};           // 旋转矩阵
            RotationOrder order;
            bool rotation_matrix_update = false; // 旋转矩阵是否已更新
        };
        
    }

}