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
            inline bool initFlag() { return is_init; }
            inline ZYZEulerAngle *getZYZEulerAngle(void) { return (order == RotationOrder::ZYZ) ? &zyz_eula_angle : nullptr; }
            inline ZYXEulerAngle *getZYXEulerAngle(void) { return (order == RotationOrder::ZYX) ? &zyx_eula_angle : nullptr; }
            inline float *getRotationMatrix(void) { return rotation_matrix_update ? &rotation_matrix[0][0] : nullptr; }
            inline float *getQuaternion(void) { return q; }

        private:
            uint8_t accel_buffer[6] = {0};
            uint8_t gyro_buffer[6] = {0};
            int16_t accelf[3] = {0, 0, 0};
            int16_t gyrof[3] = {0, 0, 0};
            float Accel[3] = {0.0f, 0.0f, 0.0f};
            float Gyro[3] = {0.0f, 0.0f, 0.0f};
            bool is_init = false;                      // 是否纠正零偏
            float offset_gyro[3] = {0.0f, 0.0f, 0.0f}; // 陀螺仪零偏
            float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // 四元数
            ZYZEulerAngle zyz_eula_angle = {0,0,0}; // 欧拉角
            ZYXEulerAngle zyx_eula_angle = {0,0,0}; // 欧拉角
            ZYXEulerAngle zyx_eula_angle_temp = {0,0,0}; // 欧拉角
            float rotation_matrix[3][3] = {0.0f};           // 旋转矩阵
            RotationOrder order;
            bool rotation_matrix_update = false; // 旋转矩阵是否已更新
        };
        
    }

}