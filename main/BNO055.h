#pragma once
#include "driver/i2c_master.h"
#include "esp_err.h"


#define BNO055_ADDR  0x28
#define NDOF 0x0C
#define ACCGYRO 0x05
#define BNO_MODE ACCGYRO
#define EUL_PIT 0x1E



void BNO055_Init();

float Gyro_X();
float Gyro_Y();
float Gyro_Z();
float Accel_X();
float Accel_Y();
float Accel_Z();




