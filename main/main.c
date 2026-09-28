
/**
 * all pinouts:
 * Motor forward: 17 tx2
 * Motor back: 18
 * Encoder: 23, 25, 3.3v, gnd, 32, 33
 * BNO055: 3.3v, gnd, D21(SDA), D22 (SCL)
 * motor2: 26, 27
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "I2C.h"
#include "BNO055.h"
#include "esp_timer.h"
#include "Motor_Encoder.h"
#include "Orientation.h"
#include "Bluetooth.h"
#include "PID.h"

#include <fcntl.h>

#include <unistd.h>
//bt includes
#include <string.h>
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gatt/ble_svc_gatt.h"

#define BLUETOOTH_CONTROL_TEST

#ifdef BLUETOOTH_CONTROL_TEST
#define HZ 100
#define LOOP_TIME_MS (int)((1.0f/(float)HZ)*1000)
void app_main(void){
    I2C_Init();
    BNO055_Init();
    Orient_Init();
    Motor_Init();
    Encoder_Init();
    PID_Init(HZ);
    Bluetooth_Init();
    while(1){
        Update_PID();
        vTaskDelay(pdMS_TO_TICKS(LOOP_TIME_MS));
    }
}

#endif

#ifdef BLUETOOTH_CONTROL_FIRST_TEST
#define KP 6.5f
#define KI 0.05f
#define KD 0.8f

#define KPV 0.0044f
#define KIV 0.0002f
#define KPTURN 0.01f

#define GWEIGHT 0.80f
#define MOTOR_MIN 2.6f
#define OFFSET_PITCH -0.35f

#define MAX_D 15.0f
#define MAX_OUTPUT 100.0f
#define MAX_SET_PITCH 10.0f
#define FALL_ANGLE 14.0f

#define VEL_ALPHA 0.80f
#define VEL_WINDOW 10
#define LOOP_DELAY_MS 10

#define SPEED_STEP 100.0f
#define TURN_STEP 150.0f

static volatile float setSpeed = 0.0f;
static volatile float setTurn = 0.0f;

static uint8_t own_addr_type;

static int command_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg
);

static const ble_uuid16_t service_uuid =
    BLE_UUID16_INIT(0xFFF0);

static const ble_uuid16_t command_uuid =
    BLE_UUID16_INIT(0xFFF1);

static const struct ble_gatt_svc_def gatt_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &command_uuid.u,
                .access_cb = command_access,
                .flags = BLE_GATT_CHR_F_WRITE
            },
            {0}
        }
    },
    {0}
};

static int command_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg
)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
    {
        uint8_t command = 0;
        uint16_t length = 0;

        int rc = ble_hs_mbuf_to_flat(
            ctxt->om,
            &command,
            1,
            &length
        );

        if (rc != 0)
            return BLE_ATT_ERR_UNLIKELY;

        if (length != 1)
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

        if (command == 'w')
            setSpeed += SPEED_STEP;
        else if (command == 's')
            setSpeed -= SPEED_STEP;
        else if (command == 'x')
            setSpeed = 0.0f;
        else if (command == 'a')
            setTurn += TURN_STEP;
        else if (command == 'd')
            setTurn -= TURN_STEP;
        else if (command == 'c')
            setTurn = 0.0f;

        printf(
            "Command: %c | Speed: %.2f | Turn: %.2f\n",
            command,
            setSpeed,
            setTurn
        );
    }

    return 0;
}

void start_advertising(void)
{
    struct ble_hs_adv_fields fields = {0};
    const char *name = "BalanceBot";

    fields.flags =
        BLE_HS_ADV_F_DISC_GEN |
        BLE_HS_ADV_F_BREDR_UNSUP;

    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params params = {0};

    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_gap_adv_start(
        own_addr_type,
        NULL,
        BLE_HS_FOREVER,
        &params,
        NULL,
        NULL
    );
}

void ble_on_sync(void)
{
    ble_hs_util_ensure_addr(0);

    ble_hs_id_infer_auto(
        0,
        &own_addr_type
    );

    start_advertising();
}

void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    I2C_Init();
    BNO055_Init();
    Orient_Init();
    Motor_Init();
    Encoder_Init();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nimble_port_init());

    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(gatt_services);

    if (rc != 0)
    {
        printf("Failed to count GATT services: %d\n", rc);
        return;
    }

    rc = ble_gatts_add_svcs(gatt_services);

    if (rc != 0)
    {
        printf("Failed to add GATT services: %d\n", rc);
        return;
    }

    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);

    float integral = 0.0f;
    float speedIntegral = 0.0f;
    float gyro_filtered = 0.0f;

    float wheelVelRaw = 0.0f;
    float wheelVel = 0.0f;
    float turnVel = 0.0f;
    float set_pitch = 0.0f;

    int encoderR_history[VEL_WINDOW] = {0};
    int encoderL_history[VEL_WINDOW] = {0};
    int64_t time_history[VEL_WINDOW] = {0};

    int vel_index = 0;
    int vel_samples = 0;

    int64_t last_time = esp_timer_get_time();

    while (1)
    {
        int64_t now = esp_timer_get_time();
        float dt = (now - last_time) / 1000000.0f;
        last_time = now;

        if (dt <= 0.0f || dt > 0.1f)
            dt = LOOP_DELAY_MS / 1000.0f;

        // Gyro / D
        float gyro = Gyro_Y();

        gyro_filtered =
            (1.0f - GWEIGHT) * gyro_filtered +
            GWEIGHT * gyro;

        float d = KD * gyro_filtered;

        if (d > MAX_D) d = MAX_D;
        else if (d < -MAX_D) d = -MAX_D;

        // Pitch
        UpdatePitch(dt);
        float pitch = GetPitch() - OFFSET_PITCH;

        // Encoders
        int encoderR = get_encoder_count();
        int encoderL = get_encoder_count2();

        // Velocity estimator
        if (vel_samples < VEL_WINDOW)
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

            int deltaR =
                encoderR -
                encoderR_history[oldest];

            int deltaL =
                encoderL -
                encoderL_history[oldest];

            float vel_dt =
                (now - time_history[oldest]) /
                1000000.0f;

            if (vel_dt > 0.0f)
            {
                float velR =
                    (float)deltaR / vel_dt;

                float velL =
                    (float)deltaL / vel_dt;

                wheelVelRaw =
                    0.5f * (velR + velL);

                wheelVel +=
                    VEL_ALPHA *
                    (wheelVelRaw - wheelVel);

                turnVel =
                    0.5f * (velR - velL);
            }

            encoderR_history[oldest] = encoderR;
            encoderL_history[oldest] = encoderL;
            time_history[oldest] = now;

            vel_index++;

            if (vel_index >= VEL_WINDOW)
                vel_index = 0;
        }

        // Outer speed PI loop
        float speedError =
            setSpeed -
            wheelVel;

        speedIntegral +=
            speedError *
            dt;

        set_pitch =
            KPV * speedError +
            KIV * speedIntegral;

        if (set_pitch > MAX_SET_PITCH)
            set_pitch = MAX_SET_PITCH;
        else if (set_pitch < -MAX_SET_PITCH)
            set_pitch = -MAX_SET_PITCH;

        // Turning P loop
        float turnError =
            setTurn -
            turnVel;

        float turnCorrection =
            KPTURN *
            turnError;

        // Inner angle PID loop
        float error =
            set_pitch -
            pitch;

        float p = KP * error;

        integral +=
            error *
            dt;

        float i = KI * integral;

        float u =
            p +
            i +
            d;

        if (u > MAX_OUTPUT)
            u = MAX_OUTPUT;
        else if (u < -MAX_OUTPUT)
            u = -MAX_OUTPUT;

        // Motor deadzone compensation
        if (u > 0.0f)
            u =
                MOTOR_MIN +
                (1.0f - MOTOR_MIN / 100.0f) * u;
        else if (u < 0.0f)
            u =
                -MOTOR_MIN +
                (1.0f - MOTOR_MIN / 100.0f) * u;

        float motorR =
            u +
            turnCorrection;

        float motorL =
            u -
            turnCorrection;

        if (motorR > MAX_OUTPUT)
            motorR = MAX_OUTPUT;
        else if (motorR < -MAX_OUTPUT)
            motorR = -MAX_OUTPUT;

        if (motorL > MAX_OUTPUT)
            motorL = MAX_OUTPUT;
        else if (motorL < -MAX_OUTPUT)
            motorL = -MAX_OUTPUT;

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

        vTaskDelay(
            pdMS_TO_TICKS(LOOP_DELAY_MS)
        );
    }
}

#endif


#ifdef BLUETOOTH_PRINT_TEST
#include <string.h>
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gatt/ble_svc_gatt.h"

static uint8_t own_addr_type;

// ============================================================
// GATT TEST
// ============================================================

// Function that NimBLE will call when the characteristic is accessed
static int command_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg
);

// UUID for our BalanceBot service
static const ble_uuid16_t service_uuid = BLE_UUID16_INIT(0xFFF0);
static const ble_uuid16_t command_uuid = BLE_UUID16_INIT(0xFFF1);

// Describe the services and characteristics that our GATT server has
static const struct ble_gatt_svc_def gatt_services[] =
{
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &service_uuid.u,

        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                .uuid = &command_uuid.u,
                .access_cb = command_access,
                .flags = BLE_GATT_CHR_F_WRITE
            },

            {0}
        }
    },

    {0}
};

// Called by NimBLE when something accesses our command characteristic
static int command_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg
)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
    {
        printf("BLE write received!\n");
    }

    return 0;
}

// Start advertising
void start_advertising(void)
{
    // the info inside ad packets
    struct ble_hs_adv_fields fields = {0};

    const char *name = "BalanceBot";
    //discover mode | not classis bluetooth device
    fields.flags =
        BLE_HS_ADV_F_DISC_GEN |
        BLE_HS_ADV_F_BREDR_UNSUP;

    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    // the ad behavior
    struct ble_gap_adv_params params = {0};
    // undirected connectable; any bt device can connect, not looking for specific device
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    //general discovery mode
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_gap_adv_start(
        own_addr_type, //local address
        NULL,   //Peer address, directed advertising only, not needed
        BLE_HS_FOREVER, //advertise forever, no timeout
        &params, 
        NULL, //GAP callback
        NULL //User arg for callback
    );
}

// Called when NimBLE is ready
void ble_on_sync(void)
{
    // gives nimBLE usable ble address
    ble_hs_util_ensure_addr(0);

    //type of address written to address
    ble_hs_id_infer_auto(
        0,
        &own_addr_type
    );

    start_advertising();
}

// NimBLE FreeRTOS task
void ble_host_task(void *param)
{
    //start event loop
    nimble_port_run();

    //cleanup
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    ESP_ERROR_CHECK(nimble_port_init());
    // Initialize NimBLE's built-in GATT service
    ble_svc_gatt_init();
    // Tell NimBLE how much space our GATT services require
    int rc = ble_gatts_count_cfg(gatt_services);

    if (rc != 0)
    {
        printf("Failed to count GATT services: %d\n", rc);
        return;
    }

    // Add our service and characteristic to NimBLE
    rc = ble_gatts_add_svcs(gatt_services);

    if (rc != 0)
    {
        printf("Failed to add GATT services: %d\n", rc);
        return;
    }

    ble_hs_cfg.sync_cb = ble_on_sync;

    nimble_port_freertos_init(ble_host_task);
}

#endif



#ifdef VELOCITY_CONTROL_TEST

#define KP 6.5f
#define KI 0.05f
#define KD 0.8f

#define KPV 0.0044f
#define KIV 0.0002f

#define KPTURN 0.01f

#define GWEIGHT 0.80f
#define MOTOR_MIN 2.6f
#define OFFSET_PITCH -0.35f

#define MAX_D 15.0f
#define MAX_OUTPUT 100.0f
#define MAX_SET_PITCH 10.0f
#define FALL_ANGLE 14.0f

#define VEL_ALPHA 0.80f
#define VEL_WINDOW 10
#define LOOP_DELAY_MS 10

#define SPEED_STEP 50.0f
#define TURN_STEP 50.0f

void app_main(void)
{
    I2C_Init();
    BNO055_Init();
    Orient_Init();
    Motor_Init();
    Encoder_Init();

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    float integral = 0.0f;
    float speedIntegral = 0.0f;
    float gyro_filtered = 0.0f;

    float wheelVelRaw = 0.0f;
    float wheelVel = 0.0f;
    float turnVel = 0.0f;
    float set_pitch = 0.0f;

    float setSpeed = 0.0f;
    float setTurn = 0.0f;

    int encoderR_history[VEL_WINDOW] = {0};
    int encoderL_history[VEL_WINDOW] = {0};
    int64_t time_history[VEL_WINDOW] = {0};

    int vel_index = 0;
    int vel_samples = 0;

    int64_t last_time = esp_timer_get_time();

    while (1)
    {
        int64_t now = esp_timer_get_time();
        float dt = (now - last_time) / 1000000.0f;
        last_time = now;

        if (dt <= 0.0f || dt > 0.1f)
            dt = LOOP_DELAY_MS / 1000.0f;

        // Keyboard input
        int key = getchar();

        if (key != EOF)
        {
            if (key == 'w')
            {
                setSpeed += SPEED_STEP;
                printf("setSpeed = %.2f\n", setSpeed);
            }
            else if (key == 's')
            {
                setSpeed -= SPEED_STEP;
                printf("setSpeed = %.2f\n", setSpeed);
            }
            else if (key == 'x')
            {
                setSpeed = 0.0f;
                printf("setSpeed = %.2f\n", setSpeed);
            }
            else if (key == 'a')
            {
                setTurn -= TURN_STEP;
                printf("setTurn = %.2f\n", setTurn);
            }
            else if (key == 'd')
            {
                setTurn += TURN_STEP;
                printf("setTurn = %.2f\n", setTurn);
            }
            else if (key == 'c')
            {
                setTurn = 0.0f;
                printf("setTurn = %.2f\n", setTurn);
            }
        }

        // Gyro / D
        float gyro = Gyro_Y();

        gyro_filtered =
            (1.0f - GWEIGHT) * gyro_filtered +
            GWEIGHT * gyro;

        float d = KD * gyro_filtered;

        if (d > MAX_D) d = MAX_D;
        else if (d < -MAX_D) d = -MAX_D;

        // Pitch
        UpdatePitch(dt);
        float pitch = GetPitch() - OFFSET_PITCH;

        // Encoders
        int encoderR = get_encoder_count();
        int encoderL = get_encoder_count2();

        // Velocity estimator
        if (vel_samples < VEL_WINDOW)
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

                wheelVel +=
                    VEL_ALPHA *
                    (wheelVelRaw - wheelVel);

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

        set_pitch =
            KPV * speedError +
            KIV * speedIntegral;

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
            u = MOTOR_MIN +
                (1.0f - MOTOR_MIN / 100.0f) * u;
        else if (u < 0.0f)
            u = -MOTOR_MIN +
                (1.0f - MOTOR_MIN / 100.0f) * u;

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

        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
}

#endif


#ifdef WORKING_BALANCE_LOOP
#define KP 6.5f
#define KI 0.05f
#define KD 0.8f

#define KPV 0.0044f
#define KIV 0.0002f

#define GWEIGHT 0.80f
#define MOTOR_MIN 2.6f
#define OFFSET_PITCH -0.35f

#define MAX_D 15.0f
#define MAX_OUTPUT 100.0f
#define MAX_SET_PITCH 10.0f
#define FALL_ANGLE 14.0f

#define VEL_ALPHA 0.80f
#define VEL_WINDOW 10
#define LOOP_DELAY_MS 10

void app_main(void)
{
    I2C_Init();
    BNO055_Init();
    Orient_Init();
    Motor_Init();
    Encoder_Init();

    float integral = 0.0f;
    float speedIntegral = 0.0f;
    float gyro_filtered = 0.0f;

    float wheelVelRaw = 0.0f;
    float wheelVel = 0.0f;
    float set_pitch = 0.0f;

    int encoderR_history[VEL_WINDOW] = {0};
    int encoderL_history[VEL_WINDOW] = {0};
    int64_t time_history[VEL_WINDOW] = {0};

    int vel_index = 0;
    int vel_samples = 0;

    int64_t last_time = esp_timer_get_time();

    while (1)
    {
        int64_t now = esp_timer_get_time();
        float dt = (now - last_time) / 1000000.0f;
        last_time = now;

        if (dt <= 0.0f || dt > 0.1f)
            dt = LOOP_DELAY_MS / 1000.0f;

        // Gyro / D
        float gyro = Gyro_Y();

        gyro_filtered =
            (1.0f - GWEIGHT) * gyro_filtered +
            GWEIGHT * gyro;

        float d = KD * gyro_filtered;

        if (d > MAX_D) d = MAX_D;
        else if (d < -MAX_D) d = -MAX_D;

        // Pitch
        UpdatePitch(dt);
        float pitch = GetPitch() - OFFSET_PITCH;

        // Encoders
        int encoderR = get_encoder_count();
        int encoderL = get_encoder_count2();

        // Velocity estimator
        if (vel_samples < VEL_WINDOW)
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

                wheelVel +=
                    VEL_ALPHA *
                    (wheelVelRaw - wheelVel);
            }

            encoderR_history[oldest] = encoderR;
            encoderL_history[oldest] = encoderL;
            time_history[oldest] = now;

            vel_index++;
            if (vel_index >= VEL_WINDOW)
                vel_index = 0;
        }

        // Outer speed PI loop
        float speedError = -wheelVel;

        speedIntegral += speedError * dt;

        set_pitch =
            KPV * speedError +
            KIV * speedIntegral;

        if (set_pitch > MAX_SET_PITCH)
            set_pitch = MAX_SET_PITCH;
        else if (set_pitch < -MAX_SET_PITCH)
            set_pitch = -MAX_SET_PITCH;

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
            u = MOTOR_MIN +
                (1.0f - MOTOR_MIN / 100.0f) * u;
        else if (u < 0.0f)
            u = -MOTOR_MIN +
                (1.0f - MOTOR_MIN / 100.0f) * u;

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
            motor_set_speedR((int)u);
            motor_set_speedL((int)u);
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
}


#endif


#ifdef ADVANCED_TUNING_LOGGER

#define MOTOR_MIN 2.6
#define GWEIGHT 0.50
#define OFFSET_PITCH -0.35

#define KP 6.5
#define KD 0.8
#define KI 0.05

#define KPV 0.0044
#define KIV 0.0000

#define MAXD 15.0
#define MAX_SET_PITCH 10.0

#define VEL_WINDOW_MAX 20

// Reduced to fit ESP32 DRAM.
// ~1500 samples at ~5 ms = ~7.5 seconds.
#define LOG_SIZE 1500


typedef struct
{
    int64_t time_us;

    float pitch;
    float set_pitch;

    float gyro_raw;
    float gyro_filtered;

    float p;
    float d;
    float i;

    float u_raw;
    float u_final;

    float wheelVelRaw;
    float wheelVel;

    int encoderR;
    int encoderL;

} LogSample;


static LogSample logBuffer[LOG_SIZE];

static int logIndex = 0;
static bool logging = false;

// After a log is dumped, normal printing/tuning stays paused
// until ANY key is pressed.
static bool log_pause = false;


// ============================================================
// CONTROL MENU
// ============================================================

static void print_controls(void)
{
    printf("\n");
    printf("====================================================================\n");
    printf("                     BALANCE CONTROLLER\n");
    printf("====================================================================\n");

    printf(" ANGLE PID              SPEED LOOP             MOTOR / IMU\n");
    printf(" q/a  Kp +/-            r/f  Kpv +/-           y/h  Motor Min +/-\n");
    printf(" w/s  Kd +/-            t/g  Kiv +/-           u/j  GWeight +/-\n");
    printf(" e/d  Ki +/-                                  i/k  Pitch Offset +/-\n");
    printf("                                              o/p  Max D +/-\n");

    printf("\n");
    printf(" OUTER / VELOCITY\n");
    printf(" z/x  Max Set Pitch +/-\n");
    printf(" c/v  Velocity Alpha +/-\n");
    printf(" b/n  Velocity Window +/-\n");

    printf("\n");
    printf(" LOGGING\n");
    printf(" l    Start / Stop log\n");
    printf(" ?    Show controls\n");

    printf("====================================================================\n\n");
}


// ============================================================
// SETTINGS DISPLAY
// ============================================================

static void print_settings(
    float kp,
    float kd,
    float ki,
    float kpv,
    float kiv,
    float motor_min,
    float gweight,
    float offset_pitch,
    float max_d,
    float max_set_pitch,
    float vel_alpha,
    int vel_window)
{
    printf("\n");
    printf("--------------------------------------------------------------------\n");

    printf(
        "ANGLE | KP:%5.2f  KD:%5.2f  KI:%6.3f\n",
        kp,
        kd,
        ki);

    printf(
        "SPEED | KPV:%7.4f  KIV:%7.4f\n",
        kpv,
        kiv);

    printf(
        "OTHER | MMIN:%5.2f  GW:%4.2f  OFFSET:%6.2f  MAXD:%5.1f\n",
        motor_min,
        gweight,
        offset_pitch,
        max_d);

    printf(
        "VEL   | MAXSET:%4.1f  ALPHA:%4.2f  WINDOW:%2d\n",
        max_set_pitch,
        vel_alpha,
        vel_window);

    printf("--------------------------------------------------------------------\n");
}


// ============================================================
// CSV LOG DUMP
// ============================================================

static void dump_log(
    float kp,
    float kd,
    float ki,
    float kpv,
    float kiv,
    float motor_min,
    float gweight,
    float offset_pitch,
    float max_d,
    float max_set_pitch,
    float vel_alpha,
    int vel_window)
{
    printf("\n\n");

    printf("LOG_METADATA\n");

    printf("KP,%.6f\n", kp);
    printf("KD,%.6f\n", kd);
    printf("KI,%.6f\n", ki);

    printf("KPV,%.8f\n", kpv);
    printf("KIV,%.8f\n", kiv);

    printf("MOTOR_MIN,%.6f\n", motor_min);
    printf("GWEIGHT,%.6f\n", gweight);
    printf("OFFSET_PITCH,%.6f\n", offset_pitch);

    printf("MAXD,%.6f\n", max_d);
    printf("MAX_SET_PITCH,%.6f\n", max_set_pitch);

    printf("VEL_ALPHA,%.6f\n", vel_alpha);
    printf("VEL_WINDOW,%d\n", vel_window);

    printf("SAMPLES,%d\n", logIndex);

    printf("\n");

    printf(
        "time_us,"
        "pitch,"
        "set_pitch,"
        "gyro_raw,"
        "gyro_filtered,"
        "p,"
        "d,"
        "i,"
        "u_raw,"
        "u_final,"
        "wheelVelRaw,"
        "wheelVel,"
        "encoderR,"
        "encoderL\n");


    for (int n = 0; n < logIndex; n++)
    {
        LogSample *s = &logBuffer[n];

        printf(
            "%lld,"
            "%.5f,"
            "%.5f,"
            "%.5f,"
            "%.5f,"
            "%.5f,"
            "%.5f,"
            "%.5f,"
            "%.5f,"
            "%.5f,"
            "%.5f,"
            "%.5f,"
            "%d,"
            "%d\n",

            s->time_us,

            s->pitch,
            s->set_pitch,

            s->gyro_raw,
            s->gyro_filtered,

            s->p,
            s->d,
            s->i,

            s->u_raw,
            s->u_final,

            s->wheelVelRaw,
            s->wheelVel,

            s->encoderR,
            s->encoderL);


        if ((n % 10) == 9)
            vTaskDelay(1);
    }

    printf("\nEND_LOG\n");
    printf("====================================================================\n");
    printf(" LOG OUTPUT PAUSED\n");
    printf(" Copy/save the CSV now.\n");
    printf(" Press ANY key when finished to return to tuning mode.\n");
    printf("====================================================================\n");
}


// ============================================================
// MAIN
// ============================================================

void app_main(void)
{
    I2C_Init();
    printf("i2c done\n");

    BNO055_Init();
    printf("bno done\n");

    Orient_Init();
    printf("orient done\n");

    Motor_Init();
    printf("motor done\n");

    Encoder_Init();
    printf("encoder done\n");


    // ========================================================
    // LIVE PARAMETERS
    // ========================================================

    float kp = KP;
    float kd = KD;
    float ki = KI;

    float kpv = KPV;
    float kiv = KIV;

    float motor_min = MOTOR_MIN;

    float gweight = GWEIGHT;

    float offset_pitch = OFFSET_PITCH;

    float max_d = MAXD;

    float max_set_pitch = MAX_SET_PITCH;

    float vel_alpha = 1.0f;

    int vel_window = 10;


    // ========================================================
    // CONTROLLER STATE
    // ========================================================

    float pitch = 0.0f;
    float set_pitch = 0.0f;

    float integral = 0.0f;

    float speedError = 0.0f;
    float speedIntegral = 0.0f;

    float wheelVelRaw = 0.0f;
    float wheelVel = 0.0f;

    float gyro_filtered = 0.0f;


    // ========================================================
    // VELOCITY ESTIMATOR
    // ========================================================

    int encoderR_history[VEL_WINDOW_MAX] = {0};
    int encoderL_history[VEL_WINDOW_MAX] = {0};

    int64_t time_history[VEL_WINDOW_MAX] = {0};

    int vel_index = 0;
    int vel_samples = 0;


    // ========================================================
    // TERMINAL / TIMING
    // ========================================================

    int print_idx = 0;

    int flags =
        fcntl(STDIN_FILENO, F_GETFL, 0);

    fcntl(
        STDIN_FILENO,
        F_SETFL,
        flags | O_NONBLOCK);


    int64_t last_time =
        esp_timer_get_time();


    print_controls();

    print_settings(
        kp,
        kd,
        ki,
        kpv,
        kiv,
        motor_min,
        gweight,
        offset_pitch,
        max_d,
        max_set_pitch,
        vel_alpha,
        vel_window);


    // ========================================================
    // MAIN LOOP
    // ========================================================

    while (1)
    {
        // ====================================================
        // LOG-DUMP PAUSE
        //
        // Motors remain OFF and nothing else is printed.
        // Any key exits the pause, but that key is consumed
        // and DOES NOT change a gain.
        // ====================================================

        if (log_pause)
        {
            motor_set_speedR(0);
            motor_set_speedL(0);

            char pause_key;

            ssize_t pause_read =
                read(
                    STDIN_FILENO,
                    &pause_key,
                    1);

            if (pause_read == 1)
            {
                log_pause = false;

                integral = 0.0f;
                speedIntegral = 0.0f;

                set_pitch = 0.0f;

                wheelVel = 0.0f;
                wheelVelRaw = 0.0f;

                vel_index = 0;
                vel_samples = 0;

                gyro_filtered = 0.0f;

                print_idx = 0;

                last_time =
                    esp_timer_get_time();

                printf("\n\n");
                printf("TUNING MODE\n");

                print_settings(
                    kp,
                    kd,
                    ki,
                    kpv,
                    kiv,
                    motor_min,
                    gweight,
                    offset_pitch,
                    max_d,
                    max_set_pitch,
                    vel_alpha,
                    vel_window);
            }

            vTaskDelay(pdMS_TO_TICKS(10));

            continue;
        }


        // ====================================================
        // TIMING
        // ====================================================

        int64_t now =
            esp_timer_get_time();

        float dt =
            (now - last_time) /
            1000000.0f;

        last_time = now;


        if (dt <= 0.0f || dt > 0.1f)
            dt = 0.005f;


        // ====================================================
        // GYRO
        // ====================================================

        float gyro =
            Gyro_Y();


        gyro_filtered =
            (1.0f - gweight) *
                gyro_filtered +
            gweight *
                gyro;


        float d =
            kd *
            gyro_filtered;


        if (d > max_d)
            d = max_d;

        else if (d < -max_d)
            d = -max_d;


        // ====================================================
        // PITCH
        // ====================================================

        UpdatePitch(dt);

        pitch =
            GetPitch() -
            offset_pitch;


        // ====================================================
        // ENCODERS
        // ====================================================

        int encoderR =
            get_encoder_count();

        int encoderL =
            get_encoder_count2();


        // ====================================================
        // ROLLING VELOCITY WINDOW
        // ====================================================

        if (vel_samples < vel_window)
        {
            encoderR_history[vel_index] =
                encoderR;

            encoderL_history[vel_index] =
                encoderL;

            time_history[vel_index] =
                now;


            vel_index++;


            if (vel_index >= vel_window)
                vel_index = 0;


            vel_samples++;
        }

        else
        {
            int oldest =
                vel_index;


            int deltaR =
                encoderR -
                encoderR_history[oldest];


            int deltaL =
                encoderL -
                encoderL_history[oldest];


            float vel_dt =
                (now -
                 time_history[oldest]) /
                1000000.0f;


            if (vel_dt > 0.0f)
            {
                float velR =
                    (float)deltaR /
                    vel_dt;


                float velL =
                    (float)deltaL /
                    vel_dt;


                wheelVelRaw =
                    0.5f *
                    (velR + velL);


                wheelVel +=
                    vel_alpha *
                    (wheelVelRaw -
                     wheelVel);
            }


            encoderR_history[oldest] =
                encoderR;

            encoderL_history[oldest] =
                encoderL;

            time_history[oldest] =
                now;


            vel_index++;


            if (vel_index >= vel_window)
                vel_index = 0;
        }


        // ====================================================
        // SPEED LOOP
        // ====================================================

        speedError =
            -wheelVel;


        speedIntegral +=
            speedError *
            dt;


        set_pitch =
            kpv *
                speedError +
            kiv *
                speedIntegral;


        if (set_pitch > max_set_pitch)
            set_pitch =
                max_set_pitch;

        else if (set_pitch < -max_set_pitch)
            set_pitch =
                -max_set_pitch;


        // ====================================================
        // ANGLE LOOP
        // ====================================================

        float error =
            set_pitch -
            pitch;


        float p =
            kp *
            error;


        integral +=
            error *
            dt;


        float i =
            ki *
            integral;


        float u_raw =
            p +
            i +
            d;


        float u =
            u_raw;


        // ====================================================
        // KEYBOARD INPUT
        // ====================================================

        char c = 0;

        ssize_t nread;

        do
        {
            nread =
                read(
                    STDIN_FILENO,
                    &c,
                    1);


            if (nread != 1)
                break;


            // ================================================
            // LOG TOGGLE
            // ================================================

            if (c == 'l')
            {
                if (!logging)
                {
                    logging = true;

                    logIndex = 0;

                    printf("\n");
                    printf("============================================================\n");
                    printf("                       LOG STARTED\n");
                    printf("                 Press l again to stop\n");
                    printf("============================================================\n");

                    print_settings(
                        kp,
                        kd,
                        ki,
                        kpv,
                        kiv,
                        motor_min,
                        gweight,
                        offset_pitch,
                        max_d,
                        max_set_pitch,
                        vel_alpha,
                        vel_window);
                }

                else
                {
                    logging = false;

                    motor_set_speedR(0);
                    motor_set_speedL(0);

                    dump_log(
                        kp,
                        kd,
                        ki,
                        kpv,
                        kiv,
                        motor_min,
                        gweight,
                        offset_pitch,
                        max_d,
                        max_set_pitch,
                        vel_alpha,
                        vel_window);

                    log_pause = true;

                    break;
                }


                continue;
            }


            // ================================================
            // NO PARAMETER CHANGES WHILE LOGGING
            // ================================================

            if (logging)
                continue;


            bool changed =
                false;


            // ================================================
            // ANGLE KP
            // ================================================

            if (c == 'q')
            {
                kp += 0.10f;

                changed = true;
            }

            else if (c == 'a')
            {
                kp -= 0.10f;

                if (kp < 0.0f)
                    kp = 0.0f;

                changed = true;
            }


            // ================================================
            // ANGLE KD
            // ================================================

            else if (c == 'w')
            {
                kd += 0.10f;

                changed = true;
            }

            else if (c == 's')
            {
                kd -= 0.10f;

                if (kd < 0.0f)
                    kd = 0.0f;

                changed = true;
            }


            // ================================================
            // ANGLE KI
            // ================================================

            else if (c == 'e')
            {
                ki += 0.01f;

                changed = true;
            }

            else if (c == 'd')
            {
                ki -= 0.01f;

                if (ki < 0.0f)
                    ki = 0.0f;

                changed = true;
            }


            // ================================================
            // SPEED KP
            // ================================================

            else if (c == 'r')
            {
                kpv += 0.0001f;

                changed = true;
            }

            else if (c == 'f')
            {
                kpv -= 0.0001f;

                if (kpv < 0.0f)
                    kpv = 0.0f;

                changed = true;
            }


            // ================================================
            // SPEED KI
            // ================================================

            else if (c == 't')
            {
                kiv += 0.0001f;

                changed = true;
            }

            else if (c == 'g')
            {
                kiv -= 0.0001f;

                if (kiv < 0.0f)
                    kiv = 0.0f;

                changed = true;
            }


            // ================================================
            // MOTOR MIN
            // ================================================

            else if (c == 'y')
            {
                motor_min += 0.10f;

                if (motor_min > 99.0f)
                    motor_min = 99.0f;

                changed = true;
            }

            else if (c == 'h')
            {
                motor_min -= 0.10f;

                if (motor_min < 0.0f)
                    motor_min = 0.0f;

                changed = true;
            }


            // ================================================
            // GYRO WEIGHT
            // ================================================

            else if (c == 'u')
            {
                gweight += 0.05f;

                if (gweight > 1.0f)
                    gweight = 1.0f;

                changed = true;
            }

            else if (c == 'j')
            {
                gweight -= 0.05f;

                if (gweight < 0.0f)
                    gweight = 0.0f;

                changed = true;
            }


            // ================================================
            // PITCH OFFSET
            // ================================================

            else if (c == 'i')
            {
                offset_pitch += 0.05f;

                changed = true;
            }

            else if (c == 'k')
            {
                offset_pitch -= 0.05f;

                changed = true;
            }


            // ================================================
            // MAX D
            // ================================================

            else if (c == 'o')
            {
                max_d += 0.50f;

                changed = true;
            }

            else if (c == 'p')
            {
                max_d -= 0.50f;

                if (max_d < 0.0f)
                    max_d = 0.0f;

                changed = true;
            }


            // ================================================
            // MAX SET PITCH
            // ================================================

            else if (c == 'z')
            {
                max_set_pitch +=
                    0.10f;

                changed = true;
            }

            else if (c == 'x')
            {
                max_set_pitch -=
                    0.10f;

                if (max_set_pitch < 0.0f)
                    max_set_pitch = 0.0f;

                changed = true;
            }


            // ================================================
            // VELOCITY ALPHA
            // ================================================

            else if (c == 'c')
            {
                vel_alpha +=
                    0.05f;

                if (vel_alpha > 1.0f)
                    vel_alpha = 1.0f;

                changed = true;
            }

            else if (c == 'v')
            {
                vel_alpha -=
                    0.05f;

                if (vel_alpha < 0.05f)
                    vel_alpha = 0.05f;

                changed = true;
            }


            // ================================================
            // VELOCITY WINDOW
            // ================================================

            else if (c == 'b')
            {
                if (vel_window <
                    VEL_WINDOW_MAX)
                {
                    vel_window++;

                    vel_index = 0;
                    vel_samples = 0;

                    wheelVel = 0.0f;
                    wheelVelRaw = 0.0f;
                }

                changed = true;
            }

            else if (c == 'n')
            {
                if (vel_window > 2)
                {
                    vel_window--;

                    vel_index = 0;
                    vel_samples = 0;

                    wheelVel = 0.0f;
                    wheelVelRaw = 0.0f;
                }

                changed = true;
            }


            // ================================================
            // HELP
            // ================================================

            else if (c == '?')
            {
                print_controls();

                print_settings(
                    kp,
                    kd,
                    ki,
                    kpv,
                    kiv,
                    motor_min,
                    gweight,
                    offset_pitch,
                    max_d,
                    max_set_pitch,
                    vel_alpha,
                    vel_window);
            }


            // ================================================
            // PRINT NEW SETTINGS
            // ================================================

            if (changed)
            {
                print_settings(
                    kp,
                    kd,
                    ki,
                    kpv,
                    kiv,
                    motor_min,
                    gweight,
                    offset_pitch,
                    max_d,
                    max_set_pitch,
                    vel_alpha,
                    vel_window);
            }

        } while (nread == 1);


        // If stopping the log put us into pause mode,
        // immediately return to the top of the loop.
        if (log_pause)
        {
            motor_set_speedR(0);
            motor_set_speedL(0);

            continue;
        }


        // ====================================================
        // OUTPUT LIMIT
        // ====================================================

        if (u > 100.0f)
            u = 100.0f;

        else if (u < -100.0f)
            u = -100.0f;


        // ====================================================
        // MOTOR MIN REMAP
        // ====================================================

        if (u > 0.0f)
        {
            u =
                motor_min +
                (1.0f -
                 motor_min / 100.0f) *
                u;
        }

        else if (u < 0.0f)
        {
            u =
                -motor_min +
                (1.0f -
                 motor_min / 100.0f) *
                u;
        }


        // ====================================================
        // LOG SAMPLE
        // ====================================================

        if (logging &&
            logIndex < LOG_SIZE)
        {
            LogSample *s =
                &logBuffer[logIndex];


            s->time_us =
                now;


            s->pitch =
                pitch;

            s->set_pitch =
                set_pitch;


            s->gyro_raw =
                gyro;

            s->gyro_filtered =
                gyro_filtered;


            s->p =
                p;

            s->d =
                d;

            s->i =
                i;


            s->u_raw =
                u_raw;

            s->u_final =
                u;


            s->wheelVelRaw =
                wheelVelRaw;

            s->wheelVel =
                wheelVel;


            s->encoderR =
                encoderR;

            s->encoderL =
                encoderL;


            logIndex++;
        }


        // ====================================================
        // AUTO STOP IF LOG BUFFER FULL
        // ====================================================

        if (logging &&
            logIndex >= LOG_SIZE)
        {
            logging =
                false;


            motor_set_speedR(0);
            motor_set_speedL(0);


            printf(
                "\n"
                "LOG BUFFER FULL\n");


            dump_log(
                kp,
                kd,
                ki,
                kpv,
                kiv,
                motor_min,
                gweight,
                offset_pitch,
                max_d,
                max_set_pitch,
                vel_alpha,
                vel_window);


            log_pause = true;

            continue;
        }


        // ====================================================
        // MOTOR OUTPUT
        // ====================================================

        if (pitch > 11.0f ||
            pitch < -11.0f)
        {
            motor_set_speedR(0);
            motor_set_speedL(0);

            integral = 0.0f;
            speedIntegral = 0.0f;

            set_pitch = 0.0f;
        }

        else
        {
            motor_set_speedR(
                (int)u);

            motor_set_speedL(
                (int)u);
        }


        // ====================================================
        // LIVE DISPLAY
        // ====================================================

        if (!logging)
        {
            if (print_idx >= 10)
            {
                printf(
                    "P:%6.2f S:%6.2f U:%7.2f | "
                    "V:%8.1f VR:%8.1f | "
                    "KP:%4.1f KD:%4.1f KI:%5.3f | "
                    "KPV:%6.4f KIV:%6.4f | "
                    "MM:%4.1f GW:%4.2f OF:%5.2f MD:%4.1f | "
                    "MS:%3.1f VA:%4.2f VW:%2d\n",

                    pitch,
                    set_pitch,
                    u,

                    wheelVel,
                    wheelVelRaw,

                    kp,
                    kd,
                    ki,

                    kpv,
                    kiv,

                    motor_min,
                    gweight,
                    offset_pitch,
                    max_d,

                    max_set_pitch,
                    vel_alpha,
                    vel_window);


                print_idx = 0;
            }


            print_idx++;
        }


        vTaskDelay(
            pdMS_TO_TICKS(10));
    }
}

#endif


#ifdef BALANCE_ENCODER_TEST
#define MOTOR_MIN 2.6 //2.6
#define GWEIGHT 0.50 //0.5
#define FWEIGHT (1.0 - GWEIGHT)
#define OFFSET_PITCH -0.35

#define KP 6.7 //6.7
#define KD 0.9 //0.9
#define KI 0.03

#define KPV 0.0041 //0.0038
#define KIV 0.000

#define MAXD 15
#define MAX_SET_PITCH 2.0

void app_main(void)
{
    I2C_Init();
    printf("i2c done\n");
    BNO055_Init();
    printf("bno done\n");
    Orient_Init();
    printf("orient done\n");
    Motor_Init();
    printf("motor done\n");
    Encoder_Init();
    printf("encoder done\n");

    float error;
    float set_pitch = 0;
    float pitch;

    float integral = 0;

    float speedError = 0;
    float speedIntegral = 0;
    float wheelVel = 0;

    float kp = KP;
    float kd = KD;
    float ki = KI;

    float kpv = KPV;
    float kiv = KIV;

    int idx = 0;
    int idx2 = 0;

    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

    int64_t last_time = esp_timer_get_time();
    int64_t last_velocity_time = last_time;

    while (1)
    {
        static float gyro_filtered = 0;

        float gyro = Gyro_Y();

        gyro_filtered =
            FWEIGHT * gyro_filtered +
            GWEIGHT * gyro;

        float d = kd * gyro_filtered;

        if (d > MAXD)
            d = MAXD;
        else if (d < -MAXD)
            d = -MAXD;

        int64_t now = esp_timer_get_time();
        float dt = (now - last_time) / 1000000.0f;
        last_time = now;

        UpdatePitch(dt);
        pitch = GetPitch() - OFFSET_PITCH;

        if (idx2 >= 10)
        {
            float velocity_dt =
                (now - last_velocity_time) / 1000000.0f;

            last_velocity_time = now;

            update_velocity(velocity_dt);

            wheelVel =
                0.5f * (get_velocity() + get_velocity2());

            speedError = -wheelVel;

            speedIntegral += speedError * velocity_dt;

            set_pitch =
                kpv * speedError +
                kiv * speedIntegral;

            if (set_pitch > MAX_SET_PITCH)
                set_pitch = MAX_SET_PITCH;
            else if (set_pitch < -MAX_SET_PITCH)
                set_pitch = -MAX_SET_PITCH;

            idx2 = 0;
        }

        idx2++;

        error = set_pitch - pitch;

        float p = kp * error;

        integral += error * dt;
        float i = ki * integral;

        float u = p + i + d;

        int c = getchar();

        // Angle Kp
        if (c == 'q')
            kp += 0.1f;
        if (c == 'a')
            kp -= 0.1f;

        // Angle Kd
        if (c == 'w')
            kd += 0.10f;
        if (c == 's')
            kd -= 0.10f;

        // Angle Ki
        if (c == 'e')
            ki += 0.01f;
        if (c == 'd')
            ki -= 0.01f;

        // Speed Kp
        if (c == 'r')
            kpv += 0.0001f;
        if (c == 'f')
            kpv -= 0.0001f;

        // Speed Ki
        if (c == 't')
            kiv += 0.001f;
        if (c == 'g')
            kiv -= 0.001f;

        if (u > 100)
            u = 100;
        if (u < -100)
            u = -100;

        if (u > 0)
            u = MOTOR_MIN +
                (1 - MOTOR_MIN / 100.0f) * u;
        else if (u < 0)
            u = -MOTOR_MIN +
                (1 - MOTOR_MIN / 100.0f) * u;

        if (pitch > 11 || pitch < -11)
        {
            motor_set_speedR(0);
            motor_set_speedL(0);

            integral = 0;
            speedIntegral = 0;
            set_pitch = 0;
        }
        else
        {
            motor_set_speedR((int)u);
            motor_set_speedL((int)u);
        }

        if (idx == 10)
        {
            printf(
                "u:%7.2f | pitch:%6.2f | set:%6.2f | "
                "kp:%.2f kd:%.2f ki:%.3f | "
                "kpv:%.4f kiv:%.4f | "
                "p:%6.2f d:%6.2f i:%6.2f | "
                "vel:%8.2f | verr:%8.2f | vint:%8.2f\n",
                u,
                pitch,
                set_pitch,
                kp,
                kd,
                ki,
                kpv,
                kiv,
                p,
                d,
                i,
                wheelVel,
                speedError,
                speedIntegral);

            idx = 0;
        }

        idx++;

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
#endif

#ifdef BALANCE_TEST

#define MOTOR_MINR 56.0 //black casing
#define MOTOR_MINL 56.0 //blue casing
#define MOTOR_MIN 3.6 //3.6
#define GWEIGHT 1.0
#define FWEIGHT (1.0 - GWEIGHT)
#define OFFSET_PITCH -0.35


#define KP 8.8
#define KD 2.2
#define KI 0.23
#define MAXD 50 //5

void app_main(void)
{
    I2C_Init();
    printf("i2c done\n");
    BNO055_Init();
    printf("bno done\n");
    Orient_Init();
    printf("orient done\n");
    Motor_Init();
    printf("motor done\n");
    Encoder_Init();
    printf("encoder done\n");
    float error;
    float set_pitch = 0;
    float pitch;
    // float dt = 0.005; //200hz
    float integral = 0;
    float kp = KP;
    float kd = KD;
    float ki = KI;

    int idx = 0;

    // Make terminal input non-blocking
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    int64_t last_time = esp_timer_get_time();
    while (1)
    {

        static float gyro_filtered = 0;
        float gyro = Gyro_Y();
        gyro_filtered =
            FWEIGHT * gyro_filtered +
            GWEIGHT * gyro;
        float d = kd * gyro_filtered;
        
        
        if (d > MAXD)
            d = MAXD;
        else if (d < -MAXD)
            d = -MAXD;

        int64_t now = esp_timer_get_time();
        float dt = (now - last_time) / 1000000.0f;
        last_time = now;
        UpdatePitch(dt);
        pitch = GetPitch() - OFFSET_PITCH;
        error = set_pitch - pitch;
        float p = kp * error;
        // float d = kd * Gyro_Y();
        integral += error * dt;
        float i = ki * integral;
        float u = p + i + d;

        //
        int c = getchar();
        if (c == 'q')
            kp += 0.1f;
        if (c == 'a')
            kp -= 0.1f;

        if (c == 'w')
            kd += 0.10f;
        if (c == 's')
            kd -= 0.10f;
        // Ki
        if (c == 'e')
            ki += 0.01f;
        if (c == 'd')
            ki -= 0.01f;

        if (u > 100)
            u = 100;
        if (u < -100)
            u = -100;

        if(u>0)
            u = MOTOR_MIN + (1-MOTOR_MIN/100.0) * u;
        if(u<0)
            u = -MOTOR_MIN + (1-MOTOR_MIN/100.0) * u;

        if (pitch > 11 || pitch < -11)
        {
            motor_set_speedR(0);
            motor_set_speedL(0);
            integral = 0;
        }
        else
        {
            motor_set_speedR((int)u);
            motor_set_speedL((int)u);
        }

        if (idx == 10)
        {
            printf(
                "u:%8.2f | pitch:%7.2f | kp:%.2f | kd:%.2f | ki:%.3f | p:%7.2f | d:%7.2f | i:%7.2f\n",
                u, pitch, kp, kd, ki, p, d, i);
            idx = 0;
        }
        idx++;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

#endif

#ifdef MOTOR_MIN_TEST

#define MOTOR_MIN_TEST
#define MOTOR_MIN 6
#define MOTOR_MINR MOTOR_MIN
#define MOTOR_MINL MOTOR_MIN
void app_main(void)
{
    Motor_Init();
    motor_set_speedR(MOTOR_MINR);
    motor_set_speedL(MOTOR_MINL);
    while (1)
    {
    }
}

#endif

#ifdef ORIENTATION_TEST
void app_main(void)
{
    I2C_Init();
    BNO055_Init();
    Orient_Init();
    while (1)
    {
        UpdatePitch(0.005);
        float pitch = GetPitch();
        printf("Ptich: %f\n", pitch);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
#endif

#ifdef MOTOR_ENCODER_TEST
void app_main(void)
{
    Motor_Init();
    Encoder_Init();

    while (1)
    {
        update_velocity(1);
        printf("Velocity: %f\n", get_velocity());
        motor_set_speed(100);
        vTaskDelay(pdMS_TO_TICKS(1000));
        update_velocity(1);
        printf("Velocity: %f\n", get_velocity());
        /* motor_set_speed(0);
         vTaskDelay(pdMS_TO_TICKS(1000));
         update_velocity(1);
         printf("Velocity: %f\n", get_velocity());
         motor_set_speed(-100);
         vTaskDelay(pdMS_TO_TICKS(1000));
         update_velocity(1);
         printf("Velocity: %f\n", get_velocity());
         motor_set_speed(0);
         vTaskDelay(pdMS_TO_TICKS(1000));
         update_velocity(1);
         printf("Velocity: %f\n", get_velocity());*/
    }
}

#endif
#ifdef MOTOR_TEST
void app_main(void)
{
    Motor_Init();
    // motor_set_speed(-100);
    while (1)
    {
        motor_set_speed(50);

        vTaskDelay(pdMS_TO_TICKS(1000));
        motor_set_speed(100);
        vTaskDelay(pdMS_TO_TICKS(1000));

        motor_set_speed(0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        motor_set_speed(-50);
        vTaskDelay(pdMS_TO_TICKS(1000));

        motor_set_speed(-100);
        vTaskDelay(pdMS_TO_TICKS(1000));

        motor_set_speed(0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif
#ifdef ENCODER_TEST

void app_main(void)
{
    Encoder_Init();
    while (1)
    {
        printf("Count: %f\n", get_velocity());
        update_velocity(0.1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

#endif

#ifdef PITCH_TEST

#define NUM_SAMPLES 1000
#define GYRO_WEIGHT 0.98f
#define ACCEL_WEIGHT (1.0f - GYRO_WEIGHT)
#define PI 3.1415

void app_main(void)
{
    I2C_Init();
    BNO055_Init();
    // subtract gyro bias
    float bias = 0;
    float gyroy = 0;
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        gyroy = Gyro_Y();
        bias += gyroy;
    }
    bias = bias / NUM_SAMPLES;
    float angle_bias = 0, angle_nobias = 0;
    float filtered_angle = 0;
    float ax = Accel_X();
    float az = Accel_Z();
    float accelPitch = atan2f(ax, az) * (180.0f / PI);

    int64_t prevTime = esp_timer_get_time();
    while (1)
    {
        // get dt
        int64_t curr_time = esp_timer_get_time();
        float dt = (curr_time - prevTime) / 1000000.0f;
        prevTime = curr_time;
        gyroy = Gyro_Y();
        // update reading
        angle_bias += gyroy * dt;
        // printf("Angle without bias removal: %f\n", angle_bias);
        angle_nobias += (gyroy - bias) * dt;
        // printf("Angle with bias removal: %f\n", angle_nobias);
        //  complementary filter
        ax = Accel_X();
        az = Accel_Z();
        accelPitch = atan2f(ax, az) * (180.0f / PI);
        filtered_angle = GYRO_WEIGHT * (filtered_angle + (gyroy - bias) * dt) + ACCEL_WEIGHT * (accelPitch);
        printf("Filtered angle: %f\n", filtered_angle);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#endif

#ifdef BNO_TEST
#define PI 3.1415
void app_main(void)
{
    I2C_Init();
    BNO055_Init();
    int16_t gyrox = Gyro_X();

    while (1)

    {
        float gx = Gyro_X();
        float gy = Gyro_Y();
        float gz = Gyro_Z();
        printf(">GyroX:%f\n", gx);
        printf(">GyroY:%f\n", gy);
        printf(">GyroZ:%f\n", gz);

        float ax = Accel_X();
        float ay = Accel_Y();
        float az = Accel_Z();
        printf(">AccelX:%f\n", ax);
        printf(">AccelY:%f\n", ay);
        printf(">AccelZ:%f\n", az);

        float accelPitch = atan2f(ax, az) * (180.0f / PI);
        printf(">PitchEstimate:%f\n", accelPitch);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
#endif