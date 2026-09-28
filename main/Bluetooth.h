#pragma once

/*
    Ensure bluetooth is enabled in sdkconfig file, will not build without

    Example from sdkconfig: 
    # Bluetooth
    #
    CONFIG_BT_ENABLED=y
    # CONFIG_BT_BLUEDROID_ENABLED is not set
    CONFIG_BT_NIMBLE_ENABLED=y
    # CONFIG_BT_CONTROLLER_ONLY is not set
    CONFIG_BT_CONTROLLER_ENABLED=y
    # CONFIG_BT_CONTROLLER_DISABLED is not set
*/

#define SPEED_STEP 100.0f
#define TURN_STEP 150.0f

void Bluetooth_Init(void);