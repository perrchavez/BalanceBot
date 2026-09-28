#pragma once

#define MOTOR_RPWM_GPIO  17   // wire to RPWM tx2
#define MOTOR_LPWM_GPIO  18   // wire to LPWM d18
#define MOTOR_RPWM_GPIO2 26    
#define MOTOR_LPWM_GPIO2 27   

void Motor_Init(void);

void Encoder_Init();

void motor_set_speedR(int percent);

void motor_set_speedL(int percent);

int get_encoder_count();
int get_encoder_count2();

void update_velocity(float dt);

float get_velocity();

float get_velocity2();