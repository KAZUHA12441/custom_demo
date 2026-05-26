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
        if(mpu6050->initFlag())
        {
        // 获取四元数和位置
        float *q = mpu6050->getQuaternion();
        for (int i = 0; i < 4; i++)
        {
            pa.q[i] = q[i];
        }
        IMU::quaternionToRotationMatrix(pa.q, reinterpret_cast<float(*)[3]>(rotation_matrix->arm_mat_.pData));

        auto new_pos = *rotation_matrix * (*temp_pos);
        
        
        pa.x = new_pos.arm_mat_.pData[0];
        pa.y = new_pos.arm_mat_.pData[1];
        pa.z = new_pos.arm_mat_.pData[2];

        static uint32_t tick = 0;

        send_data = {0xEE,pa.x,pa.y,pa.z,q[1],q[2],q[3],q[0],0xED};
        if (CDC_Transmit_FS(reinterpret_cast<uint8_t*>(&send_data), sizeof(send_data)) != USBD_OK)
        {
            // USBD_BUSY: 上一次传输尚未完成，跳过本帧
            // USBD_FAIL: USB未配置，跳过本帧
        }
        tick++;
        }
    }

    QuaternionPosition MoveAttitude::getQuaternionAndPosition()
    {
        return pa;
    }

}