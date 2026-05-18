#include "Mahony.hpp"
#include <math.h>

namespace App
{

void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 加速度模长偏离 1g 超过 15% 时跳过修正，防止冲击/运动污染积分项
    float acc_norm_sq = ax * ax + ay * ay + az * az;
    bool acc_ok = (acc_norm_sq > 0.72f) && (acc_norm_sq < 1.32f)
                  && !((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f));

    // 迟滞：连续 100ms 稳定才启用 PI，避免冲击余振反复清零积分
    static int stable_cnt = 50;
    if (acc_ok)
    {
        if (stable_cnt < 50) stable_cnt++;
    }
    else
    {
        stable_cnt = 0;
    }
    bool acc_valid = (stable_cnt >= 50);

    static bool was_valid = true;
    if (acc_valid && !was_valid)
    {
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }
    was_valid = acc_valid;

    if (acc_valid)
    {
        recipNorm = invSqrt(acc_norm_sq);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        halfvx = q[1] * q[3] - q[0] * q[2];
        halfvy = q[0] * q[1] + q[2] * q[3];
        halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];

        // 检查重力方向是否翻转（误差 >90°，叉积会反推）
        float dot = halfvx * ax + halfvy * ay + halfvz * az;
        if (dot < -0.1f)
        {
            // 重置四元数：从加速度计重建初始姿态
            float angle = acosf(az);
            float axis_x = ay;
            float axis_y = -ax;
            float axis_sq = axis_x * axis_x + axis_y * axis_y;
            if (axis_sq < 1e-6f) { axis_x = 1.0f; axis_y = 0.0f; }
            float axis_n = axis_sq < 1e-6f ? 1.0f : invSqrt(axis_sq);
            float half_a = angle * 0.5f;
            q[0] = cosf(half_a);
            q[1] = axis_x * axis_n * sinf(half_a);
            q[2] = axis_y * axis_n * sinf(half_a);
            q[3] = 0.0f;
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
            // 本帧跳过修正，下帧用新四元数继续
        }
        else
        {

        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        if (twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq);
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);

            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
        }  // else (dot >= -0.1f)
    }      // if (acc_valid)

    gx *= (0.5f * (1.0f / sampleFreq));
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);

    recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;
}

}
