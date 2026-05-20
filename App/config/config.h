#pragma once

#include "main.h"

#define MPU6050_ADDR 0x68 << 1
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define INT_ENABLE 0x38
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define WHO_AM_I 0x75


#define ACCEL_SENSITIVITY 8192.0f
#define GYRO_SENSITIVITY 65.5f



#define END_BASE_YAW 0.0f
#define END_BASE_PITCH 0.0f
#define END_BASE_ROLL -90.0f

#define I2C_PORT hi2c1
#define ADC_PORT hadc1

// 摇杆中位 ADC（实测）
#define TRASLATE_ADC_MID_X 3204.0f
#define TRASLATE_ADC_MID_Y 2961.0f
// 正负分开 scale：正 delta→4095-mid，负 delta→mid-0，满偏均映射到 ±256
#define TRASLATE_ADC_SCALE_X_POS (256.0f / (4095.0f - TRASLATE_ADC_MID_X))  // ≈0.287
#define TRASLATE_ADC_SCALE_X_NEG (256.0f / TRASLATE_ADC_MID_X)               // ≈0.08
#define TRASLATE_ADC_SCALE_Y_POS (256.0f / (4095.0f - TRASLATE_ADC_MID_Y))  // ≈0.226
#define TRASLATE_ADC_SCALE_Y_NEG (256.0f / TRASLATE_ADC_MID_Y)               // ≈0.086

