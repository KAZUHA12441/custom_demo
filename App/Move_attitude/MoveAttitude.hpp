#pragma once

#include "matrix.hpp"
#include "mpu6050.hpp"
#include "translate.hpp"
#include "solution.hpp"

namespace App
{
    struct QuaternionPosition
    {
        float q[4]; // 四元数
        float x, y, z; // 位置
    };
    class MoveAttitude
    {
        public:
        MoveAttitude(void);
        void Init(IMU::MPU6050_c *mpu6050, Translate *translate);
        void Update();
        QuaternionPosition getQuaternionAndPosition();
        private:
        QuaternionPosition pa = {{1.0f, 0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 0.0f}; // 四元数和位置
        IMU::MPU6050_c *mpu6050; // IMU对象
        Translate *translate; // 平移对象
        Matrixf<4,4> *transform_matrix; // 变换矩阵
        Matrixf<3,1> *pos; //位置矩阵
        Matrixf<3,1> *temp_pos; //临时位置矩阵
        Matrixf<3,3> *rotation_matrix; //旋转矩阵
        Matrixf<1,4> *bottom;
        

    };
    
  
    
}