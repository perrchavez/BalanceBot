#pragma once
// Tuning parameters
//inner balance loop
#define KP 6.5f
#define KI 0.05f
#define KD 0.8f

//outer speed/turning loop
#define KPV 0.0044f
#define KIV 0.0006f //0.0002 originally
#define KPTURN 0.03f //0.01 originally

#define GWEIGHT 0.80f //gyro filtering 0-1. 1 is only current gyro, no filtering
#define MOTOR_MIN 2.6f // motor deadband
#define OFFSET_PITCH -0.35f 

#define MAX_D 15.0f // limits the derivative term, can reduces jittering while keeping damping
#define MAX_OUTPUT 100.0f
#define MAX_SET_PITCH 12.0f // another way to limit how hard the speed controller can act; originally 10.0f
#define FALL_ANGLE 20.0f // Motors cutoff after this angle; originally 14.0f

#define VEL_ALPHA 0.80f //encoder velocity filter
#define VEL_WINDOW 10 //encoder moving average filter
#define LOOP_DELAY_DEFAULT 10 //default pid loop speed assumption


/**
 * inits the balance
 * takes the pid loop frequency
 */
void PID_Init(int hz);

void Update_PID(void);

void PID_SetSpeed(int speed);

void PID_SetTurn(int turn);

int PID_GetSetSpeed();
int PID_GetSetTurn();
