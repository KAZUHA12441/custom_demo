#pragma once

#include "main.h"

#define MPU6050_ADDR 0xD0
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define INT_ENABLE 0x38
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

#define I2C_PORT &hi2c1