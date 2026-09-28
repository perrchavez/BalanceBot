#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "I2C.h"
#include "BNO055.h"


// accelerometer
#define ACC_XREG 0x08
#define ACC_YREG 0x0A
#define ACC_ZREG 0x0C
// gyro
#define GYRO_XREG 0x14
#define GYRO_YREG 0x16
#define GYRO_ZREG 0x18

i2c_master_dev_handle_t bno; //bno device on i2c bus

i2c_device_config_t device_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7, //standard 7 bit address
    .device_address = BNO055_ADDR,
    .scl_speed_hz = 100000, //scl clock frequency
};

void BNO055_Init()
{
    I2C_AddDevice(&bno, &device_config);
    // Enter CONFIG mode
    uint8_t config[2] = {0x3D, 0x00};
    i2c_master_transmit(bno, config, 2, -1);
    vTaskDelay(pdMS_TO_TICKS(25));

    // Enter desired mode
    uint8_t mode[2] = {0x3D, BNO_MODE};
    i2c_master_transmit(bno, mode, 2, -1);
    vTaskDelay(pdMS_TO_TICKS(20));
}

float Gyro_X()
{
    uint8_t data[2];
    uint8_t reg = GYRO_XREG;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(bno, &reg, 1, data, 2, -1)); //gets data from sensor using I2C
    int16_t gyrox = (int16_t)((data[1] << 8) | data[0]);
    float gyrodps = gyrox / 16.0f;
    return gyrodps;
}

float Gyro_Y()
{
    uint8_t data[2];
    uint8_t reg = GYRO_YREG;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(bno, &reg, 1, data, 2, -1));
    int16_t gyroy = (int16_t)((data[1] << 8) | data[0]);
    float gyrodps = gyroy / 16.0f;
    return gyrodps;
}

float Gyro_Z()
{
    uint8_t data[2];
    uint8_t reg = GYRO_ZREG;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(bno, &reg, 1, data, 2, -1));
    int16_t gyroz = (int16_t)((data[1] << 8) | data[0]);
    float gyrodps = gyroz / 16.0f;
    return gyrodps;
}

static float ReadAccelAxis(uint8_t reg)
{
    uint8_t data[2];

    ESP_ERROR_CHECK(
        i2c_master_transmit_receive(bno,&reg,1,data,2,-1)
    );
    int16_t raw = (int16_t)((data[1] << 8) | data[0]);

    return raw / 100.0f;
}

float Accel_X(){
    return ReadAccelAxis(ACC_XREG);
}
float Accel_Y(){
    return ReadAccelAxis(ACC_YREG);
}
float Accel_Z(){
    return ReadAccelAxis(ACC_ZREG);
}
