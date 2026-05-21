#include "MoveAttitude.hpp"

namespace App
{
    MoveAttitude::MoveAttitude(void)
    {
        transform_matrix = new Matrixf<4,4>();
        pos = new Matrixf<3,1>();
        rotation_matrix = new Matrixf<3,3>();
        float bottom_data[4] = {0.0f, 0.0f, 0.0f, 1.0f};
        bottom =  new Matrixf<1,4>(bottom_data);
       

        *transform_matrix = Matrixf<4,4>::eye();
        *rotation_matrix = Matrixf<3,3>::eye();
        *pos = Matrixf<3,1>::zeros();
    }

    void MoveAttitude::Init(IMU::MPU6050_c *mpu6050, Translate *translate)
    {
        
        this->mpu6050 = mpu6050;
        this->translate = translate;
        temp_pos = translate->get_pos();
    }

    void MoveAttitude::Update()
    {
        // 获取四元数和位置
        float *q = mpu6050->getQuaternion();
        for (int i = 0; i < 4; i++)
        {
            pa.q[i] = q[i];
        }
        IMU::quaternionToRotationMatrix(pa.q, reinterpret_cast<float(*)[3]>(rotation_matrix->arm_mat_.pData));

        //拼成4x4的变换矩阵
        auto tmp1 = hstack(*rotation_matrix, *pos);
        *transform_matrix = vstack(tmp1, *bottom);
        
        auto AugmentedPos = vstack(*temp_pos, Matrixf<1,1>::eye());


        auto new_pos = *transform_matrix * AugmentedPos;
        
        
        pa.x = new_pos.arm_mat_.pData[0];
        pa.y = new_pos.arm_mat_.pData[1];
        pa.z = new_pos.arm_mat_.pData[2];

        pos->arm_mat_.pData[0] = pa.x;
        pos->arm_mat_.pData[1] = pa.y;
        pos->arm_mat_.pData[2] = pa.z;
         
    }

    QuaternionPosition MoveAttitude::getQuaternionAndPosition()
    {
        return pa;
    }

}