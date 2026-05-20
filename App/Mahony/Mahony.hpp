#pragma once
//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq 500.0f     // sample frequency in Hz
#define twoKpDef (2.0f * 2.0f) // 2 * proportional gain
#define twoKiDef (2.0f * 0.0f) // 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

namespace App
{
    inline volatile float twoKp = twoKpDef; // 2 * proportional gain (Kp)
    inline volatile float twoKi = twoKiDef; // 2 * integral gain (Ki)
    inline volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

    inline float invSqrt(float x)
    {
        float xhalf = 0.5f * x;
        int i = *(int *)&x;             // get bits for floating value
        i = 0x5f375a86 - (i >> 1);      // gives initial guess
        x = *(float *)&i;               // convert bits back to float
        x = x * (1.5f - xhalf * x * x); // Newton step
        return x;
    }

    void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);

}