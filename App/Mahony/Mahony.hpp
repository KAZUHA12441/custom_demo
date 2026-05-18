#pragma once
//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq 1000.0f     // sample frequency in Hz
#define twoKpDef (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef (2.0f * 0.0f) // 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef; // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef; // 2 * integral gain (Ki)
// volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

namespace App
{
    float invSqrt(float x)
    {
        float xhalf = 0.5f * x;
        int i = *(int *)&x;             // get bits for floating value
        i = 0x5f375a86 - (i >> 1);      // gives initial guess
        x = *(float *)&i;               // convert bits back to float
        x = x * (1.5f - xhalf * x * x); // Newton step
        return x;
    }

    
    void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az)
    {
        float recipNorm;
        float halfvx, halfvy, halfvz;
        float halfex, halfey, halfez;
        float qa, qb, qc;

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        // 只在加速度计数据有效时才进行运算
        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
        {

            // Normalise accelerometer measurement
            // 将加速度计得到的实际重力加速度向量v单位化
            recipNorm = invSqrt(ax * ax + ay * ay + az * az); // 该函数返回平方根的倒数
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Estimated direction of gravity
            // 通过四元数得到理论重力加速度向量g
            // 注意，这里实际上是矩阵第三列*1/2，在开头对Kp Ki的宏定义均为2*增益
            // 这样处理目的是减少乘法运算量
            halfvx = q[1] * q[3] - q[0] * q[2];
            halfvy = q[0] * q[1] + q[2] * q[3];
            halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];

            // Error is sum of cross product between estimated and measured direction of gravity
            // 对实际重力加速度向量v与理论重力加速度向量g做外积
            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);

            // Compute and apply integral feedback if enabled
            // 在PI补偿器中积分项使能情况下计算并应用积分项
            if (twoKi > 0.0f)
            {
                // integral error scaled by Ki
                // 积分过程
                integralFBx += twoKi * halfex * (1.0f / sampleFreq);
                integralFBy += twoKi * halfey * (1.0f / sampleFreq);
                integralFBz += twoKi * halfez * (1.0f / sampleFreq);

                // apply integral feedback
                // 应用误差补偿中的积分项
                gx += integralFBx;
                gy += integralFBy;
                gz += integralFBz;
            }
            else
            {
                // prevent integral windup
                // 避免为负值的Ki时积分异常饱和
                integralFBx = 0.0f;
                integralFBy = 0.0f;
                integralFBz = 0.0f;
            }

            // Apply proportional feedback
            // 应用误差补偿中的比例项
            gx += twoKp * halfex;
            gy += twoKp * halfey;
            gz += twoKp * halfez;
        }

        // Integrate rate of change of quaternion
        // 微分方程迭代求解
        gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
        gy *= (0.5f * (1.0f / sampleFreq));
        gz *= (0.5f * (1.0f / sampleFreq));
        qa = q[0];
        qb = q[1];
        qc = q[2];
        q[0] += (-qb * gx - qc * gy - q[3] * gz);
        q[1] += (qa * gx + qc * gz - q[3] * gy);
        q[2] += (qa * gy - qb * gz + q[3] * gx);
        q[3] += (qa * gz + qb * gy - qc * gx);

        // Normalise quaternion
        // 单位化四元数 保证四元数在迭代过程中保持单位性质
        recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] *= recipNorm;
        q[1] *= recipNorm;
        q[2] *= recipNorm;
        q[3] *= recipNorm;

        // Mahony官方程序到此结束，使用时只需在函数外进行四元数反解欧拉角即可完成全部姿态解算过程
    }

}