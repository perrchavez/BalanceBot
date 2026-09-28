#include "driver/ledc.h"
#include "Motor_Encoder.h"
#include "driver/gpio.h"

#define PWM_MODE    LEDC_LOW_SPEED_MODE
#define PWM_TIMER   LEDC_TIMER_0
#define PWM_FREQ_HZ 19500               // max with 12 bit value
#define PWM_RES     LEDC_TIMER_12_BIT  
#define CH_RPWM     LEDC_CHANNEL_0
#define CH_LPWM     LEDC_CHANNEL_1
#define CH_RPWM2    LEDC_CHANNEL_2
#define CH_LPWM2    LEDC_CHANNEL_3

//encoder defines
#define ENCODER_A GPIO_NUM_23
#define ENCODER_B GPIO_NUM_25
#define ENCODER_C 32
#define ENCODER_D 33

//encoder count
static volatile int count =0;
static float velocity = 0;

static volatile int count2 = 0;
static float velocity2 = 0;

typedef enum{
    A =2,
    B =3,
    C =1,
    D =0
} State;
static volatile State prev_state = A;
static volatile State prev_state2 = A;

// helper function, converts a percentage to duty cycle
static uint32_t pct_to_duty(int pct) {
    if (pct < 0) pct = -pct;
    if (pct > 100) pct = 100;
    uint32_t max = (1U << PWM_RES) - 1U;
    return (uint32_t)(pct * max / 100);
}

// encoder isr state machines
static void encoder_isr(void *arg){
    uint8_t encA = gpio_get_level(ENCODER_A);
    uint8_t encB = gpio_get_level(ENCODER_B);
    State state = (State)((encA<<1)| encB);
    switch(prev_state){
        case A:
            if(state == B)
                count++;
            else if(state == D)
                count--;
            break;

        case B:
            if(state == C)
                count++;
            else if(state ==A)
                count--;
            break;

        case C:
            if(state ==D)
                count++;
            else if(state == B)
                count--;
            break;

        case D:
            if(state == A)
                count++;
            else if(state == C)
                count--;
            break;
        default:
            break;
    }
    prev_state = state;
}
static void encoder_isr2(void *arg){
    uint8_t encC = gpio_get_level(ENCODER_C);
    uint8_t encD = gpio_get_level(ENCODER_D);
    State state = (State)((encC<<1)| encD);
    switch(prev_state2){
        case A:
            if(state == B)
                count2++;
            else if(state == D)
                count2--;
            break;

        case B:
            if(state == C)
                count2++;
            else if(state ==A)
                count2--;
            break;

        case C:
            if(state ==D)
                count2++;
            else if(state == B)
                count2--;
            break;

        case D:
            if(state == A)
                count2++;
            else if(state == C)
                count2--;
            break;
        default:
            break;
    }
    prev_state2 = state;

}

void Motor_Init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQ_HZ,
        .duty_resolution = PWM_RES,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {
        .speed_mode = PWM_MODE,
        .timer_sel  = PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0,
        .intr_type  = LEDC_INTR_DISABLE,
        .flags.output_invert = 0
    };
    // right motor
    ch.channel = CH_RPWM; 
    ch.gpio_num = MOTOR_RPWM_GPIO; 
    ledc_channel_config(&ch);
    ch.channel = CH_LPWM; 
    ch.gpio_num = MOTOR_LPWM_GPIO; 
    ledc_channel_config(&ch);
    //left motor
    ch.channel = CH_RPWM2; 
    ch.gpio_num = MOTOR_RPWM_GPIO2; 
    ledc_channel_config(&ch);

    ch.channel = CH_LPWM2; 
    ch.gpio_num = MOTOR_LPWM_GPIO2; 
    ledc_channel_config(&ch);
}


void Encoder_Init(){
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << ENCODER_A) | (1ULL << ENCODER_B)| (1ULL << ENCODER_C)| (1ULL << ENCODER_D), // bitmask to select gpio pins
        .mode = GPIO_MODE_INPUT,
        .pull_up_en =  GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE // sets inturrpt type to rising and falling edge
    };
    gpio_config(&config);
    gpio_install_isr_service(0); // set up interupts
    // adds pins to isr; NULL is no args
    gpio_isr_handler_add(ENCODER_A, encoder_isr, NULL);
    gpio_isr_handler_add(ENCODER_B, encoder_isr, NULL);

    gpio_isr_handler_add(ENCODER_C, encoder_isr2, NULL);
    gpio_isr_handler_add(ENCODER_D, encoder_isr2, NULL);
    //init state
    uint8_t encA = gpio_get_level(ENCODER_A);
    uint8_t encB = gpio_get_level(ENCODER_B);
    prev_state = (State)((encA << 1) | encB);
    uint8_t encC = gpio_get_level(ENCODER_C);
    uint8_t encD = gpio_get_level(ENCODER_D);
    prev_state2 = (State)((encC << 1) | encD);
}



void motor_set_speedR(int percent)
{
    if (percent > 100) percent = 100;
    if (percent < -100) percent = -100;

    uint32_t full = pct_to_duty(100);

    if (percent > 0) {
        /*
         * Positive direction:
         *
         * DRIVE: CH_LPWM = 1, CH_RPWM = 0
         * BRAKE: CH_LPWM = 1, CH_RPWM = 1
         */
        uint32_t brake_duty = pct_to_duty(100 - percent);

        ledc_set_duty(PWM_MODE, CH_LPWM, full);
        ledc_set_duty(PWM_MODE, CH_RPWM, brake_duty);

        ledc_update_duty(PWM_MODE, CH_LPWM);
        ledc_update_duty(PWM_MODE, CH_RPWM);
    }
    else if (percent < 0) {
        /*
         * Negative direction:
         *
         * DRIVE: CH_RPWM = 1, CH_LPWM = 0
         * BRAKE: CH_RPWM = 1, CH_LPWM = 1
         */
        int magnitude = -percent;

        uint32_t brake_duty = pct_to_duty(100 - magnitude);

        ledc_set_duty(PWM_MODE, CH_RPWM, full);
        ledc_set_duty(PWM_MODE, CH_LPWM, brake_duty);

        ledc_update_duty(PWM_MODE, CH_RPWM);
        ledc_update_duty(PWM_MODE, CH_LPWM);
    }
    else {
        /*
         * Active brake:
         * IN1 = 1
         * IN2 = 1
         */
        ledc_set_duty(PWM_MODE, CH_RPWM, full);
        ledc_set_duty(PWM_MODE, CH_LPWM, full);

        ledc_update_duty(PWM_MODE, CH_RPWM);
        ledc_update_duty(PWM_MODE, CH_LPWM);
    }
}


void motor_set_speedL(int percent)
{
    if (percent > 100) percent = 100;
    if (percent < -100) percent = -100;

    uint32_t full = pct_to_duty(100);

    if (percent > 0) {
        uint32_t brake_duty = pct_to_duty(100 - percent);

        ledc_set_duty(PWM_MODE, CH_LPWM2, full);
        ledc_set_duty(PWM_MODE, CH_RPWM2, brake_duty);

        ledc_update_duty(PWM_MODE, CH_LPWM2);
        ledc_update_duty(PWM_MODE, CH_RPWM2);
    }
    else if (percent < 0) {
        int magnitude = -percent;

        uint32_t brake_duty = pct_to_duty(100 - magnitude);

        ledc_set_duty(PWM_MODE, CH_RPWM2, full);
        ledc_set_duty(PWM_MODE, CH_LPWM2, brake_duty);

        ledc_update_duty(PWM_MODE, CH_RPWM2);
        ledc_update_duty(PWM_MODE, CH_LPWM2);
    }
    else {
        ledc_set_duty(PWM_MODE, CH_RPWM2, full);
        ledc_set_duty(PWM_MODE, CH_LPWM2, full);

        ledc_update_duty(PWM_MODE, CH_RPWM2);
        ledc_update_duty(PWM_MODE, CH_LPWM2);
    }
}


int get_encoder_count(){
    return count;
}
int get_encoder_count2(){
    return count2;
}
void update_velocity(float dt){
    static int prevCount = 0;
    static int prevCount2 = 0;
    int newCount = get_encoder_count();
    int newCount2 = get_encoder_count2();
    float newV = ((float)(newCount- prevCount))/dt;
    float newV2 = ((float)(newCount2- prevCount2))/dt;
    velocity  = newV;
    velocity2 = newV2;
    prevCount = newCount;
    prevCount2 = newCount2;
}

float get_velocity(){
    return velocity;
}
float get_velocity2(){
    return velocity2;
}



#ifdef MOTOR_ENCODER_TEST
/**
 * TESTING for ESP IDF
 * must take main out of cmake file
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

 void app_main(void){
    Motor_Init();
    Encoder_Init();
    int count = get_encoder_count();
    int count2 = get_encoder_count2();
    int prev = 0;
    int prev2 = 0;
    while(1){
        count = get_encoder_count();
        count2 = get_encoder_count2();
        if(count != prev || count2 != prev2){
            printf("count: %7.2d | count2: %7.2d\n", count, count2);
        }
    }
 }


 #endif