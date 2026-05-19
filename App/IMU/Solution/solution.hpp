#pragma once
#include "arm_math.h"


namespace App
{
    namespace IMU
    {

         // 旋转顺序，不同的旋转有不同表达
        enum class RotationOrder
        {
            ZYZ = 0,
            ZYX = 1
        };
        
        struct ZYXEulerAngle
        {
            float Yaw;
            float Pitch;
            float Roll;
        };

        struct ZYZEulerAngle
        {
            float Yaw1;
            float Pitch;
            float Yaw2;
        };

        constexpr float RAD_TO_DEG = 57.29577951308232f; // 180/PI, float to avoid double-promotion from M_PI

        inline ZYXEulerAngle ZYX_Solution(float *q)
        {
            float Yaw, Pitch, Roll;
            Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * RAD_TO_DEG;
            Pitch = asinf(2.0f * (q[0] * q[2] - q[3] * q[1])) * RAD_TO_DEG;
            Roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * RAD_TO_DEG;
            return {Yaw,Pitch,Roll}; // 或者返回 Pitch 和 Roll，根据需要
        }
        inline void quaternionToRotationMatrix(float *q, float R[3][3])
        {
            R[0][0] = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
            R[0][1] = 2.0f * (q[1] * q[2] - q[0] * q[3]);
            R[0][2] = 2.0f * (q[1] * q[3] + q[0] * q[2]);
            R[1][0] = 2.0f * (q[1] * q[2] + q[0] * q[3]);
            R[1][1] = 1.0f - 2.0f * (q[1] * q[1] + q[3] * q[3]);
            R[1][2] = 2.0f * (q[2] * q[3] - q[0] * q[1]);
            R[2][0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
            R[2][1] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
            R[2][2] = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
        }

        inline ZYZEulerAngle ZYZ_Solution(float *q)
        {
            float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

            // sin²θ₂ = 4(q₀² + q₃²)(q₁² + q₂²)
            float sinPitch = 2.0f * sqrtf((q0 * q0 + q3 * q3) * (q1 * q1 + q2 * q2));
            // cosθ₂ = 1 - 2(q₁² + q₂²)
            float cosPitch = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

            float Yaw1, Pitch, Yaw2;
            Pitch = atan2f(sinPitch, cosPitch);

            const float eps = 1e-6f;
            if (sinPitch < eps)
            {
                // gimbal lock: only Yaw1 ± Yaw2 is determined
                Yaw1 = 0.0f;
                Yaw2 = atan2f(2.0f * (q0 * q3 - q1 * q2),
                              1.0f - 2.0f * (q2 * q2 + q3 * q3));
            }
            else
            {
                // Yaw1 = atan2(r23, r13) = atan2(2(q₂q₃ - q₀q₁), 2(q₁q₃ + q₀q₂))
                Yaw1 = atan2f(2.0f * (q2 * q3 - q0 * q1),
                              2.0f * (q1 * q3 + q0 * q2));
                // Yaw2 = atan2(r32, -r31) = atan2(2(q₂q₃ + q₀q₁), 2(q₀q₂ - q₁q₃))
                Yaw2 = atan2f(2.0f * (q2 * q3 + q0 * q1),
                              2.0f * (q0 * q2 - q1 * q3));
            }
            return {Yaw1, Pitch, Yaw2};
        }

        
    }
}