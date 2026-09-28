#include "PID.h"
#include "esp_timer.h"
#include "Motor_Encoder.h"
#include "BNO055.h"
#include "Orientation.h"


static float loop_time = LOOP_DELAY_DEFAULT;

static float integral = 0.0f;
static float speedIntegral = 0.0f;
static float gyro_filtered = 0.0f;

static float wheelVelRaw = 0.0f;
static float wheelVel = 0.0f;
static float turnVel = 0.0f;
static float set_pitch = 0.0f;

static volatile float setSpeed = 0.0f;
static volatile float setTurn = 0.0f;

//encoder moving average filter
static int encoderR_history[VEL_WINDOW] = {0};
static int encoderL_history[VEL_WINDOW] = {0};
static int64_t time_history[VEL_WINDOW] = {0};

static int vel_index = 0;
static int vel_samples = 0;

static int64_t last_time = 0;

//inits loop time and last time
void PID_Init(int hz){
    if (hz != 0)
        loop_time = 1.0f/(float)hz;
    last_time = esp_timer_get_time();
}

//updates motor commands
void Update_PID(void){
    int64_t now = esp_timer_get_time();
    float dt = (now - last_time) / 1000000.0f; //manual calculation of elapsed time
    last_time = now;

    //default loop time
    if (dt <= 0.0f || dt > 0.1f)
        dt = loop_time / 1000.0f;
    
    // Gyro / D
    float gyro = Gyro_Y();

    // IIR filter the gyro to remove high frequency noise
    gyro_filtered = (1.0f - GWEIGHT) * gyro_filtered + GWEIGHT * gyro;

    float d = KD * gyro_filtered;
    //limit d term
    if (d > MAX_D) d = MAX_D;
    else if (d < -MAX_D) d = -MAX_D;

    // Pitch
    UpdatePitch(dt);
    float pitch = GetPitch() - OFFSET_PITCH;

    // Encoders
    int encoderR = get_encoder_count();
    int encoderL = get_encoder_count2();

    // Velocity estimator
    if (vel_samples < VEL_WINDOW) //fills window first
    {
        encoderR_history[vel_index] = encoderR;
        encoderL_history[vel_index] = encoderL;
        time_history[vel_index] = now;

        vel_index++;
        if (vel_index >= VEL_WINDOW)
            vel_index = 0;

        vel_samples++;
    }
    else
    {
        int oldest = vel_index;

        int deltaR = encoderR - encoderR_history[oldest];
        int deltaL = encoderL - encoderL_history[oldest];

        float vel_dt =
            (now - time_history[oldest]) /
            1000000.0f;

        if (vel_dt > 0.0f)
        {
            float velR = (float)deltaR / vel_dt;
            float velL = (float)deltaL / vel_dt;

            wheelVelRaw = 0.5f * (velR + velL);
            // wheel velocity IIR filter
            wheelVel += VEL_ALPHA * (wheelVelRaw - wheelVel);
            turnVel = 0.5f * (velR - velL);
        }

        encoderR_history[oldest] = encoderR;
        encoderL_history[oldest] = encoderL;
        time_history[oldest] = now;

        vel_index++;
        if (vel_index >= VEL_WINDOW)
            vel_index = 0;
    }

    // Outer speed PI loop
    float speedError = setSpeed - wheelVel;
    speedIntegral += speedError * dt;
    set_pitch = KPV * speedError + KIV * speedIntegral;

    if (set_pitch > MAX_SET_PITCH)
        set_pitch = MAX_SET_PITCH;
    else if (set_pitch < -MAX_SET_PITCH)
        set_pitch = -MAX_SET_PITCH;

    // Turning P loop
    float turnError = setTurn - turnVel;
    float turnCorrection = KPTURN * turnError;

    // Inner angle PID loop
    float error = set_pitch - pitch;

    float p = KP * error;

    integral += error * dt;
    float i = KI * integral;

    float u = p + i + d;

    // Output limit
    if (u > MAX_OUTPUT) u = MAX_OUTPUT;
    else if (u < -MAX_OUTPUT) u = -MAX_OUTPUT;

    // Motor deadzone compensation
    if (u > 0.0f)
        u = MOTOR_MIN + (1.0f - MOTOR_MIN / 100.0f) * u;
    else if (u < 0.0f)
        u = -MOTOR_MIN + (1.0f - MOTOR_MIN / 100.0f) * u;

    float motorR = u + turnCorrection;
    float motorL = u - turnCorrection;

    if (motorR > MAX_OUTPUT) motorR = MAX_OUTPUT;
    else if (motorR < -MAX_OUTPUT) motorR = -MAX_OUTPUT;

    if (motorL > MAX_OUTPUT) motorL = MAX_OUTPUT;
    else if (motorL < -MAX_OUTPUT) motorL = -MAX_OUTPUT;

    // Fall cutoff
    if (pitch > FALL_ANGLE || pitch < -FALL_ANGLE)
    {
        motor_set_speedR(0);
        motor_set_speedL(0);

        integral = 0.0f;
        speedIntegral = 0.0f;
        set_pitch = 0.0f;
    }
    else
    {
        motor_set_speedR((int)motorR);
        motor_set_speedL((int)motorL);
    }
}

void PID_SetSpeed(int speed){
    setSpeed = speed;
    speedIntegral = 0.0f;
}

void PID_SetTurn(int turn){
    setTurn = turn;
}

int PID_GetSetSpeed(){
    return setSpeed;
}
int PID_GetSetTurn(){
    return setTurn;
}
