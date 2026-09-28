#include "I2C.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

static i2c_master_bus_handle_t bus;

void I2C_Init(){
    
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0, //selects i2c controller
        .sda_io_num = I2C_SDA_IO, //specifies sda pin
        .scl_io_num = I2C_SCL_IO, //specifies SCL pin
        .clk_source = I2C_CLK_SRC_DEFAULT, //selects default clock sourc
        .glitch_ignore_cnt = 7, //filter short glitches
        .flags.enable_internal_pullup = true, //pulls line high, devices pull low
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus)); //creates the bus
}

//adds devices to the i2c bus
void I2C_AddDevice(i2c_master_dev_handle_t *dev, i2c_device_config_t *config){
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, config, dev));
}
