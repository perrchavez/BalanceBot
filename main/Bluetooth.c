#include "Bluetooth.h"
#include <stdio.h>
#include <string.h>
#include "PID.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gatt/ble_svc_gatt.h"

//function prototypes
static int callback(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
void ble_on_sync(void);
void ble_host_task(void *param);
// bluetooth address of esp32
static uint8_t own_addr_type;

static volatile float setSpeed = 0.0f;
static volatile float setTurn = 0.0f;

//service id creation
static const ble_uuid16_t service_uuid = BLE_UUID16_INIT(0xFFF0);
// command/characteristic id creation
static const ble_uuid16_t command_uuid = BLE_UUID16_INIT(0xFFF1);

// array containing the gatt service structs, describes the services/characteristics
static const struct ble_gatt_svc_def gatt_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY, //defining as the primary service to run
        .uuid = &service_uuid.u, //the service id
        .characteristics = (struct ble_gatt_chr_def[]) { //characteristics is struct within the service struct that calls the callback when written to
            {
                .uuid = &command_uuid.u,
                .access_cb = callback, // maps to callback function
                .flags = BLE_GATT_CHR_F_WRITE
            },
            {0}
        }
    },
    {0}
};

void Bluetooth_Init(){
    // enable flash for bluetooth stack storage
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nimble_port_init());
    //init the gatt services
    ble_svc_gatt_init();
    // counts the services in our gatt service array
    int rc = ble_gatts_count_cfg(gatt_services);

    if (rc != 0)
    {
        printf("Failed to count GATT services: %d\n", rc);
        return;
    }
    //adds the serices from the gatt service array
    rc = ble_gatts_add_svcs(gatt_services);

    if (rc != 0)
    {
        printf("Failed to add GATT services: %d\n", rc);
        return;
    }
    
    ble_hs_cfg.sync_cb = ble_on_sync;   // when ready call the ble_on_sync function
    nimble_port_freertos_init(ble_host_task);   // gives bluetooth its own freertos task
}


/**
 * Maps bluetooth commands to robot commands, called when characteristic is accessed
 * -add new commands here
 */
static int callback(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg
)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) // op is the operation within the context, checks if a char was written
    {
        uint8_t command = 0; // to store the char sent over bluetooth
        uint16_t length = 0;

        int rc = ble_hs_mbuf_to_flat(
            ctxt->om, //om is the buffer, the actual char sent
            &command, //store the sent char in command
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
        PID_SetSpeed(setSpeed);
        PID_SetTurn(setTurn);
    }

    return 0;
}

void start_advertising(void)
{
    struct ble_hs_adv_fields fields = {0}; // what info will be shown when advertising
    const char *name = "BalanceBot"; // if changed, also change the name within the mac_commands.py script

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP; //discover mode | not classis bluetooth device

    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params params = {0}; // advertisment behavior
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
    ble_hs_id_infer_auto(0, &own_addr_type);

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


