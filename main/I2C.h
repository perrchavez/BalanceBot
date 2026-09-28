
#ifndef I2C_H
#define I2C_H
#include "driver/i2c_master.h"

#define I2C_SDA_IO   21 // SDA pin D21
#define I2C_SCL_IO   22 // SCL pin D22

/*
Initializes the I2C bus
*/
void I2C_Init();

/*
Adds device to bus
*/

void I2C_AddDevice(i2c_master_dev_handle_t *dev, i2c_device_config_t *config);

#endif