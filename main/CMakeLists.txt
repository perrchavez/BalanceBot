idf_component_register(SRCS "main.c"
                       "I2C.c" 
                       "BNO055.c"
                       "Motor_Encoder.c"
                       "Orientation.c"
                       "Bluetooth.c"
                       "PID.c"
                   INCLUDE_DIRS "."
                   REQUIRES bt
                            nvs_flash
                            esp_driver_i2c
                            esp_driver_ledc
                            esp_driver_gpio)