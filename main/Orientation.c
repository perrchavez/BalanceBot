#include "Orientation.h"
#include <math.h>
#include "BNO055.h"

static float bias =0;
static float filtered_angle = 0;
void Orient_Init(){
    float gyroy = 0;
    for(int i =0; i< NUM_SAMPLES; i++){
        gyroy = Gyro_Y();
        bias+= gyroy;
    }
    bias = bias/ NUM_SAMPLES;
}

void UpdatePitch(float dt){
    float ax = Accel_X();
    float az = Accel_Z();
    float accelPitch = atan2f(ax, az) * (180.0f/ PI);
    float gyroy = Gyro_Y();

    filtered_angle = GYRO_WEIGHT* (filtered_angle - (gyroy-bias)*dt) + ACCEL_WEIGHT* (accelPitch);
}

float GetPitch(){
    return filtered_angle;
}