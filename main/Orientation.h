#pragma once
#define NUM_SAMPLES 1000 // number of samples for gyro bias calculation
// weights for complememtary filter
#define GYRO_WEIGHT 0.98f
#define ACCEL_WEIGHT (1.0f-GYRO_WEIGHT)
#define PI 3.1415

/*
Calculates the gyro bias over NUM_SAMPLES to be remopved from readings befofore integration.
I2C_Init() and BNO055_Init() must be run first
*/
void Orient_Init(void);

float GetPitch_Accel(void);

float GetPitch_Gyro(void);

float GetPitch();

void UpdatePitch(float dt);

