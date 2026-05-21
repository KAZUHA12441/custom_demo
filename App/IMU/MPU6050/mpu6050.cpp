#include "mpu6050.hpp"
#include <math.h>
#include "cmsis_os.h"

static uint32_t count = 0;
namespace App
{
    namespace IMU
    {
        MPU6050_c::MPU6050_c(RotationOrder order, bool rotation_matrix_update) : order(order), rotation_matrix_update(rotation_matrix_update) {}

        void MPU6050_c::Init(void)
        {
            uint8_t check;
            uint8_t Data;

            HAL_I2C_Mem_Read(&I2C_PORT, MPU6050_ADDR, WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &check, 1, 100);
            if (check == 0x68)
            {
                Data = 0x00;
                HAL_I2C_Mem_Write(&I2C_PORT, MPU6050_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &Data, 1, 100);
                Data = 0x01;
                HAL_I2C_Mem_Write(&I2C_PORT, MPU6050_ADDR, SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &Data, 1, 100);
                Data = 0x03;
                HAL_I2C_Mem_Write(&I2C_PORT, MPU6050_ADDR, CONFIG, I2C_MEMADD_SIZE_8BIT, &Data, 1, 100);
                // 陀螺仪量程±500°/s
                Data = 0x08;
                HAL_I2C_Mem_Write(&I2C_PORT, MPU6050_ADDR, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &Data, 1, 100);
                // 加速度计量程±4g
                Data = 0x08;
                HAL_I2C_Mem_Write(&I2C_PORT, MPU6050_ADDR, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &Data, 1, 100);

                // 检查连接
                HAL_I2C_Mem_Read(&I2C_PORT, MPU6050_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &check, 1, 100);
                if (check == 0x00)
                {
                    osDelay(500);  // 预热 500ms 让温度初步稳定
                    offsetGyroCalibrate();
                    initBody();
                    is_init = true;
                }
            }
        }

        void MPU6050_c::offsetGyroCalibrate()
        {
            uint16_t success_count = 0;
            offset_gyro[0] = 0.0f;
            offset_gyro[1] = 0.0f;
            offset_gyro[2] = 0.0f;

            while (success_count < 1000)
            {
                if (HAL_I2C_Mem_Read(&I2C_PORT, MPU6050_ADDR, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, gyro_buffer, 6, 100) != HAL_OK)
                {
                    continue;
                }

                gyrof[0] = (int16_t)((gyro_buffer[0] << 8) | gyro_buffer[1]);
                gyrof[1] = (int16_t)((gyro_buffer[2] << 8) | gyro_buffer[3]);
                gyrof[2] = (int16_t)((gyro_buffer[4] << 8) | gyro_buffer[5]);

                Gyro[1] = (float)gyrof[1] / GYRO_SENSITIVITY;
                Gyro[2] = (float)gyrof[2] / GYRO_SENSITIVITY;
                Gyro[0] = (float)gyrof[0] / GYRO_SENSITIVITY;

                offset_gyro[0] += Gyro[0];
                offset_gyro[1] += Gyro[1];
                offset_gyro[2] += Gyro[2];
                success_count++;
                osDelay(2);  // 间隔 2ms 确保读到新数据
            }
            offset_gyro[0] /= success_count;
            offset_gyro[1] /= success_count;
            offset_gyro[2] /= success_count;
        }

        float *MPU6050_c::initBody(void)
        {
            float acc_init[3] = {0.0f, 0.0f, 0.0f};
            float gravity_norm[3] = {0.0f, 0.0f, 1.0f}; // 导航系重力加速度矢量
            float axis_rot[3] = {0.0f, 0.0f, 0.0f};

            // 读取100次加速度计数据，取平均值作为初始值
            for (uint8_t i = 0; i < 100; ++i)
            {
                readACCEL();
                acc_init[0] += Accel[0];
                acc_init[1] += Accel[1];
                acc_init[2] += Accel[2];
                BSP_n::DWT_c::Get_DwtInstance()->Delay_s(0.001f);
            }

            for (uint8_t i = 0; i < 3; ++i)
                acc_init[i] /= 100.0f;

            // 归一化
            float norm = invSqrt(acc_init[0] * acc_init[0] + acc_init[1] * acc_init[1] + acc_init[2] * acc_init[2]);
            acc_init[0] *= norm;
            acc_init[1] *= norm;
            acc_init[2] *= norm;

            // 计算原始加速度矢量和导航系重力加速度矢量的夹角
            float angle = acosf(acc_init[2]);

            // 外积: acc_init × gravity_norm = [acc_y, -acc_x, 0]
            axis_rot[0] = acc_init[1];
            axis_rot[1] = -acc_init[0];
            axis_rot[2] = 0.0f;

            // 归一化旋转轴
            float axis_norm = invSqrt(axis_rot[0] * axis_rot[0] + axis_rot[1] * axis_rot[1]);
            axis_rot[0] *= axis_norm;
            axis_rot[1] *= axis_norm;

            // 轴角公式，第三轴为0（无z轴分量）
            q[0] = cosf(angle * 0.5f);
            q[1] = axis_rot[0] * sinf(angle * 0.5f);
            q[2] = axis_rot[1] * sinf(angle * 0.5f);
            q[3] = 0.0f;

            return q;
        }

        void MPU6050_c::readACCEL(void)
        {
            if (HAL_I2C_Mem_Read(&I2C_PORT, MPU6050_ADDR, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, accel_buffer, 6, 100) != HAL_OK)
                return;

            accelf[0] = (int16_t)((accel_buffer[0] << 8) | accel_buffer[1]);
            accelf[1] = (int16_t)((accel_buffer[2] << 8) | accel_buffer[3]);
            accelf[2] = (int16_t)((accel_buffer[4] << 8) | accel_buffer[5]);

            

            //换个方向，并转换为g
            Accel[0] = (float)accelf[0] / ACCEL_SENSITIVITY;
            Accel[1] = -(float)accelf[2] / ACCEL_SENSITIVITY;
            Accel[2] = (float)accelf[1] / ACCEL_SENSITIVITY;

            // 滤下高频振动尖峰，避免对姿态解算的干扰，不知道为什么就是滤不干净
            {
                static float ax_f = 0, ay_f = 0, az_f = 0;
                constexpr float alpha = 0.05f;
                ax_f += alpha * (Accel[0] - ax_f);
                ay_f += alpha * (Accel[1] - ay_f);
                az_f += alpha * (Accel[2] - az_f);
                Accel[0] = ax_f;
                Accel[1] = ay_f;
                Accel[2] = az_f;
            }
            
            
           
        }

        void MPU6050_c::readGYRO(void)
        {
            if (HAL_I2C_Mem_Read(&I2C_PORT, MPU6050_ADDR, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, gyro_buffer, 6, 100) != HAL_OK)
                return;

            gyrof[0] = (int16_t)((gyro_buffer[0] << 8) | gyro_buffer[1]);
            gyrof[1] = (int16_t)((gyro_buffer[2] << 8) | gyro_buffer[3]);
            gyrof[2] = (int16_t)((gyro_buffer[4] << 8) | gyro_buffer[5]);



            //换一个方向，并转换为弧度/s
            constexpr float DEG_TO_RAD = 0.01745329252f;
            Gyro[0] = ((float)gyrof[0] / GYRO_SENSITIVITY - offset_gyro[0]) * DEG_TO_RAD;
            Gyro[1] = -((float)gyrof[2] / GYRO_SENSITIVITY - offset_gyro[2]) * DEG_TO_RAD;
            Gyro[2] = ((float)gyrof[1] / GYRO_SENSITIVITY - offset_gyro[1]) * DEG_TO_RAD;

            // 一阶低通: fc≈4Hz @500Hz, 衰减高频噪声尖峰
            {
                static float gx_f = 0, gy_f = 0, gz_f = 0;
                constexpr float alpha = 0.05f;
                gx_f += alpha * (Gyro[0] - gx_f);
                gy_f += alpha * (Gyro[1] - gy_f);
                gz_f += alpha * (Gyro[2] - gz_f);
                Gyro[0] = gx_f;
                Gyro[1] = gy_f;
                Gyro[2] = gz_f;
            }

            // 设个死区，抑制小范围内的噪声尖峰
            if (fabsf(Gyro[0]) < 0.01f) Gyro[0] = 0.0f;
            if (fabsf(Gyro[1]) < 0.01f) Gyro[1] = 0.0f;
            if (fabsf(Gyro[2]) < 0.01f) Gyro[2] = 0.0f;
        }

        void MPU6050_c::Update(void)
        {
            if (is_init)
            {
                readACCEL();
                readGYRO();
                
                MahonyAHRSupdateIMU(q, Gyro[0], Gyro[1], Gyro[2], Accel[0], Accel[1], Accel[2]);

                
                // constexpr float mw = 0.70710678f;
                // constexpr float mx = -0.70710678f;
                // float mq[4];
                // mq[0] = q[0]*mw - q[1]*mx;             // q0*m0 - q1*m1
                // mq[1] = q[0]*mx + q[1]*mw;             // q0*m1 + q1*m0
                // mq[2] = q[2]*mw + q[3]*mx;             // q2*m0 + q3*m1 (note: m2=0,m3=0)
                // mq[3] = q[3]*mw - q[2]*mx;             // q3*m0 - q2*m1

                if (order == RotationOrder::ZYX)
                {
                    zyx_eula_angle_temp = ZYX_Solution(q);
                }
                else if (order == RotationOrder::ZYZ)
                {
                    zyz_eula_angle = ZYZ_Solution(q);
                }
                if (rotation_matrix_update)
                {
                    quaternionToRotationMatrix(q, rotation_matrix);
                }
                count++;
                
            }
            osDelay(2);
            
        }

    }

}