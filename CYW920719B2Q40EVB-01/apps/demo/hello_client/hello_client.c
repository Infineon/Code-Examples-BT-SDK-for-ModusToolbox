/*
 * Copyright 2019, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
*
* BLE Vendor Specific Client Device
*
* The Hello Client application is designed to connect and access services
* of the Hello Sensor device.  Hello Client can connect up to three
* Hello Sensor Device's.  Because handles of the all attributes of
* the Hello Sensor are well known, Hello Client does not perform GATT
* discovery, but uses them directly.  In addition to that Hello Client
* allows another master to connect, so the device will behave as a slave
* in one Bluetooth piconet and a master in another.  To accomplish that
* application can do both advertisements and scan.  Hello Client assumes
* that Hello Sensor advertises a special UUID and connects to the device
* which publishes it.
*
* Features demonstrated
*  - Registration with LE stack for various events
*  - Connection to a master and a slave
*  - As a master processing notifications from the server and
*    sending notifications to the client
*  - As a slave processing writes from the client and sending writes
*    to the server
*
* To demonstrate the app, work through the following steps.
* 1. Plug the WICED eval board into your computer
* 2. Build and download the application (to the WICED board)
* 3. Connect from some client application (for example LightBlue on iOS)
* 4. From the client application register for notifications
* 5. Make sure that your slave device (hello_sensor) is up and advertising
* 6. Push a button on the tag board for 6 seconds.  That will start
*    connection process.
* 7. Push a button on the hello_sensor to deliver notification through
*    hello_client device up to the client
* 8. Repeat the steps 5 to 7 for connecting to multiple hello_sensor
*     device's
*
*/
#include <string.h>
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "hello_sensor.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"
#include "wiced_transport.h"

#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif
#include "wiced_platform.h"
#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#include "wiced_hal_puart.h"
#include "wiced_timer.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define HCLIENT_APP_TIMEOUT_IN_SECONDS              1       /* Hello Client App Timer Timeout in seconds  */

#define HELLO_CLIENT_MAX_SLAVES                     3       /* Hello Client maximum number of slaves that can be connected */
#define HELLO_CLIENT_MAX_CONNECTIONS                4       /* Hello Client maximum number of connections including master/slave */

/* GPIO pins */
#ifdef CYW20706A2
#define HELLO_CLIENT_GPIO_BUTTON                     WICED_GPIO_BUTTON
#define HELLO_CLIENT_GPIO_SETTINGS                   WICED_GPIO_BUTTON_SETTINGS( GPIO_EN_INT_BOTH_EDGE )
#define HELLO_CLIENT_DEFAULT_STATE                   WICED_GPIO_BUTTON_DEFAULT_STATE
#define HELLO_CLIENT_BUTTON_PRESSED_VALUE            WICED_BUTTON_PRESSED_VALUE
#endif

#if defined(CYW20719B0) || defined(CYW20735B0)
#define HELLO_CLIENT_GPIO_BUTTON                     WICED_GPIO_PIN_BUTTON
#define HELLO_CLIENT_GPIO_SETTINGS                   ( GPIO_INPUT_ENABLE | GPIO_PULL_DOWN | GPIO_EN_INT_BOTH_EDGE )
#define HELLO_CLIENT_DEFAULT_STATE                   GPIO_PIN_OUTPUT_LOW
#define HELLO_CLIENT_BUTTON_PRESSED_VALUE            WICED_BUTTON_PRESSED_VALUE
#endif

#if defined(CYW20719B1) || defined(CYW20719B2) || defined(CYW20721B1) || defined(CYW20721B2) || defined(CYW20735B1) || defined(CYW20819A1)
#define HELLO_CLIENT_GPIO_BUTTON                     WICED_GPIO_PIN_BUTTON_1
#define HELLO_CLIENT_BUTTON_PRESSED_VALUE            wiced_platform_get_button_pressed_value(WICED_PLATFORM_BUTTON_1)
#endif


enum
{
    HANDLE_HCLIENT_GATT_SERVICE = 0x1,                         // GATT service handle

    HANDLE_HCLIENT_GAP_SERVICE = 0x14,                         // GAP service handle
        HANDLE_HCLIENT_GAP_SERVICE_CHAR_DEV_NAME,              // device name characteristic handle
        HANDLE_HCLIENT_GAP_SERVICE_CHAR_DEV_NAME_VAL,          // char value handle

        HANDLE_HCLIENT_GAP_SERVICE_CHAR_DEV_APPEARANCE,        // appearance characteristic handle
        HANDLE_HCLIENT_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,    // char value handle

    HANDLE_HELLO_CLIENT_SERVICE =  0x28,                       // Hello Client Service
        HANDLE_HELLO_CLIENT_SERVICE_CHAR_NOTIFY,               // notify characteristic handle
        HANDLE_HELLO_CLIENT_SERVICE_CHAR_NOTIFY_VAL,           // characteristic value handle
            HANDLE_HELLO_CLIENT_SERVICE_CHAR_CFG_DESC,         // characteristic client configuration descriptor handle

    HANDLE_HCLIENT_DEV_INFO_SERVICE = 0x40,                    // Device Information Service
        HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_MFR_NAME,         // manufacturer name characteristic handle
        HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,     // characteristic value handle

        HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_MODEL_NUM,        // model number characteristic handle
        HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,    // characteristic value handle

        HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_SYSTEM_ID,        // system ID characteristic handle
        HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,    // characteristic value handle

    HANDLE_HCLIENT_BATTERY_SERVICE = 0x60,                     // battery service handle
        HANDLE_HCLIENT_BATTERY_SERVICE_CHAR_LEVEL,             // battery level characteristic handle
        HANDLE_HCLIENT_BATTERY_SERVICE_CHAR_LEVEL_VAL          // characteristic value handle
};

/* UUID value of the Hello Client Service */
#define UUID_HELLO_CLIENT_SERVICE   0xef, 0x48, 0xa2, 0x32, 0x17, 0xc6, 0xa6, 0xbc, 0xfa, 0x44, 0x54, 0x7c, 0x0d, 0x90, 0x03, 0xdc

/* UUID value of the Hello Client Data Characteristic */
#define UUID_HELLO_CLIENT_DATA      0xc5, 0x42, 0x45, 0x3b, 0xd0, 0x74, 0x5b, 0x81, 0xf6, 0x4a, 0x26, 0x8f, 0xa5, 0xcf, 0x7a, 0xb7

/******************************************************************************
 *                          Type  Definitions
 ******************************************************************************/
/* structure to store GATT attributes for read/write operations */
typedef struct
{
    uint16_t    handle;
    uint16_t    attr_len;
    const void *p_attr;
} gatt_attribute_t;

/* Peer Info */
typedef struct
{
    uint16_t conn_id;                   // Connection Identifier
    uint8_t  role;                      // master or slave in the current connection
    uint8_t  addr_type;                 // peer address type
    uint8_t  transport;                 // peer connected transport
    uint8_t  peer_addr[BD_ADDR_LEN];    // Peer BD Address
} hello_client_peer_info_t;

/* Host information to be stored in NVRAM */
typedef struct
{
    BD_ADDR  bdaddr;                                // BD address of the bonded host
    uint16_t characteristic_client_configuration;   // Current value of the client configuration descriptor
} hclient_host_info_t;

/* Hello client application info */
typedef struct
{
    uint32_t                 app_timer_count;                         // App Timer Count
    uint16_t                 conn_id;                                 // Hold the slave connection id
    uint8_t                  num_connections;                         // Number of connections
    uint16_t                 master_conn_id;                          // Handle of the master connection
    uint8_t                  battery_level;                           // dummy battery level
    hclient_host_info_t      host_info;                               // NVRAM save area
    hello_client_peer_info_t peer_info[HELLO_CLIENT_MAX_CONNECTIONS]; // Peer Info
} hello_client_app_t;

/******************************************************************************
 *                            Variables Definitions
 ******************************************************************************/
/*
 * This is the GATT database for the Hello Client application.  Hello Client
 * can connect to hello sensor, and also provides service for
 * somebody to access.  The database defines services, characteristics and
 * descriptors supported by the application.  Each attribute in the database
 * has a handle, (characteristic has two, one for characteristic itself,
 * another for the value).  The handles are used by the peer to access
 * attributes, and can be used locally by application, for example to retrieve
 * data written by the peer.  Definition of characteristics and descriptors
 * has GATT Properties (read, write, notify...) but also has permissions which
 * identify if peer application is allowed to read or write into it.
 * Handles do not need to be sequential, but need to be in order.
 */
const uint8_t hello_client_gatt_database[]=
{
    // Handle 0x01: GATT service
    PRIMARY_SERVICE_UUID16( HANDLE_HCLIENT_GATT_SERVICE, UUID_SERVICE_GATT ),

    // Handle 0x14: GAP service
    PRIMARY_SERVICE_UUID16( HANDLE_HCLIENT_GAP_SERVICE, UUID_SERVICE_GAP ),

        CHARACTERISTIC_UUID16( HANDLE_HCLIENT_GAP_SERVICE_CHAR_DEV_NAME, HANDLE_HCLIENT_GAP_SERVICE_CHAR_DEV_NAME_VAL,
             UUID_CHARACTERISTIC_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        // Handle 0x17: characteristic Appearance, handle 0x18 characteristic value.
        // List of approved appearances is available at bluetooth.org.  Current
        // value is set to 0x200 - Generic Tag
        CHARACTERISTIC_UUID16( HANDLE_HCLIENT_GAP_SERVICE_CHAR_DEV_APPEARANCE, HANDLE_HCLIENT_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
             UUID_CHARACTERISTIC_APPEARANCE, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

    // Handle 0x28: Hello Client Service.
    // This is the main proprietary service of Hello Client application.  It has
    // a single characteristic which allows peer to write to and can be configured
    // to send indications to the peer.  Note that UUID of the vendor specific
    // service is 16 bytes, unlike standard Bluetooth UUIDs which are 2 bytes.
    // _UUID128 version of the macro should be used.
    PRIMARY_SERVICE_UUID128( HANDLE_HELLO_CLIENT_SERVICE, UUID_HELLO_CLIENT_SERVICE ),

        // Handle 0x29: characteristic Hello Notification, handle 0x2a characteristic value
        // we support both notification and indication.  Peer need to allow notifications
        // or indications by writing in the Characteristic Client Configuration Descriptor
        // (see handle 2b below).  Note that UUID of the vendor specific characteristic is
        // 16 bytes, unlike standard Bluetooth UUIDs which are 2 bytes.  _UUID128 version
        // of the macro should be used.
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_HELLO_CLIENT_SERVICE_CHAR_NOTIFY, HANDLE_HELLO_CLIENT_SERVICE_CHAR_NOTIFY_VAL,
             UUID_HELLO_CLIENT_DATA, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE |
             LEGATTDB_CHAR_PROP_WRITE_NO_RESPONSE | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE,
             LEGATTDB_PERM_READABLE  | LEGATTDB_PERM_WRITE_CMD  | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_VARIABLE_LENGTH ),

            // Handle 0x2b: Characteristic Client Configuration Descriptor.
            // This is standard GATT characteristic descriptor.  2 byte value 0 means that
            // message to the client is disabled.  Peer can write value 1 or 2 to enable
            // notifications or indications respectively.  Not _WRITABLE in the macro.  This
            // means that attribute can be written by the peer.
            CHAR_DESCRIPTOR_UUID16_WRITABLE( HANDLE_HELLO_CLIENT_SERVICE_CHAR_CFG_DESC,
                 UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_AUTH_READABLE | LEGATTDB_PERM_AUTH_WRITABLE),

    // Handle 0x40: Device Info service
    // Device Information service helps peer to identify manufacture or vendor
    // of the device.  It is required for some types of the devices (for example HID,
    // and medical, and optional for others.  There are a bunch of characteristics
    // available, out of which Hello Sensor implements 3.
    PRIMARY_SERVICE_UUID16( HANDLE_HCLIENT_DEV_INFO_SERVICE, UUID_SERVICE_DEVICE_INFORMATION ),

        // Handle 0x41: characteristic Manufacturer Name, handle 0x42 characteristic value
        CHARACTERISTIC_UUID16( HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_MFR_NAME, HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
             UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        // Handle 0x43: characteristic Model Number, handle 0x4 characteristic value
        CHARACTERISTIC_UUID16( HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_MODEL_NUM, HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,
             UUID_CHARACTERISTIC_MODEL_NUMBER_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        // Handle 0x45: characteristic System ID, handle 0x46 characteristic value
        CHARACTERISTIC_UUID16( HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_SYSTEM_ID, HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,
             UUID_CHARACTERISTIC_SYSTEM_ID, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

    // Handle 0x60: Battery service
    // This is an optional service which allows peer to read current battery level.
    PRIMARY_SERVICE_UUID16( HANDLE_HCLIENT_BATTERY_SERVICE, UUID_SERVICE_BATTERY ),

        // Handle 0x61: characteristic Battery Level, handle 0x62 characteristic value
        CHARACTERISTIC_UUID16( HANDLE_HCLIENT_BATTERY_SERVICE_CHAR_LEVEL, HANDLE_HCLIENT_BATTERY_SERVICE_CHAR_LEVEL_VAL,
             UUID_CHARACTERISTIC_BATTERY_LEVEL, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),
};

/* Holds the hello client app info */
hello_client_app_t g_hello_client;

char    hello_client_local_name[]           = "Hello Client";
uint8_t hello_client_appearance[2]          = { BIT16_TO_8( APPEARANCE_GENERIC_TAG ) };
uint8_t hello_client_notify_value[]         = "Hello Client";
char    hello_client_char_mfr_name_value[]  = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
char    hello_client_char_model_num_value[] = { '4', '3', '2', '1', 0, 0, 0, 0 };
uint8_t hello_client_char_system_id_value[] = { 0xef, 0x48, 0xa2, 0x32, 0x17, 0xc6, 0xa6, 0xbc };

/* GATT attribute lookup table                                */
/* (attributes externally referenced by GATT server database) */
const gatt_attribute_t hello_client_gattdb_attributes[] =
{
    { HANDLE_HCLIENT_GAP_SERVICE_CHAR_DEV_NAME_VAL,       sizeof( hello_client_local_name),          hello_client_local_name },
    { HANDLE_HCLIENT_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL, sizeof( hello_client_appearance),          hello_client_appearance },
    { HANDLE_HELLO_CLIENT_SERVICE_CHAR_CFG_DESC,          2,                                         &g_hello_client.host_info.characteristic_client_configuration },
    { HANDLE_HELLO_CLIENT_SERVICE_CHAR_NOTIFY_VAL,        sizeof(hello_client_notify_value),         hello_client_notify_value },
    { HANDLE_HCLIENT_BATTERY_SERVICE_CHAR_LEVEL_VAL,      1,                                         &g_hello_client.battery_level },
    { HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,  sizeof(hello_client_char_mfr_name_value),  hello_client_char_mfr_name_value },
    { HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL, sizeof(hello_client_char_model_num_value), hello_client_char_model_num_value },
    { HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL, sizeof(hello_client_char_system_id_value), hello_client_char_system_id_value },
};

/* transport configuration */
const wiced_transport_cfg_t  transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg =
    {
        .uart_cfg =
        {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
            .baud_rate =  HCI_UART_DEFAULT_BAUD
        },
    },
    .rx_buff_pool_cfg =
    {
        .buffer_size = 0,
        .buffer_count = 0
    },
    .p_status_handler = NULL,
    .p_data_handler = NULL,
    .p_tx_complete_cback = NULL
};

#define PAIRING_BUTTON_ALWAYS_ON 0
#define PAIRING_BUTTON_TOGGLE    1

/* Hello service UUID  */
const uint8_t hello_service[16] = {UUID_HELLO_SERVICE};

/* Variable to indicate if the scan has to be started.
 * Set to 1 if the user pushes and holds the button for more than 5 seconds */
uint8_t start_scan = 0;

#ifdef CYW43012C0
/* Application buffer pool. */
wiced_bt_buffer_pool_t*        p_hello_client_buffer_pool;
#endif

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];

wiced_timer_t hello_client_second_timer;

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
static void                     hello_client_app_init( void );
static wiced_result_t           hello_client_management_cback( wiced_bt_management_evt_t event,  wiced_bt_management_evt_data_t *p_event_data );
static wiced_bt_gatt_status_t   hello_client_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data );
static wiced_bt_gatt_status_t   hello_client_gatt_connection_up( wiced_bt_gatt_connection_status_t *p_conn_status );
static wiced_bt_gatt_status_t   hello_client_gatt_connection_down( wiced_bt_gatt_connection_status_t *p_conn_status );
static wiced_bt_gatt_status_t   hello_client_gatt_op_comp_cb( wiced_bt_gatt_operation_complete_t *p_data );
static wiced_bt_gatt_status_t   hello_client_gatt_req_cb( wiced_bt_gatt_attribute_request_t *p_data );
static wiced_bt_gatt_status_t   hello_client_gatt_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t *p_read_data );
static wiced_bt_gatt_status_t   hello_client_gatt_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data );
static void                     hello_client_set_advertisement_data( void );
static void                     hello_client_interrupt_handler(void* user_data, uint8_t value );
static void                     hello_client_app_timer( uint32_t arg );
static void                     hello_client_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data );
static void                     hello_client_smp_bond_result( BD_ADDR bda, uint8_t result );
static void                     hello_client_encryption_changed( wiced_result_t result, uint8_t* p_bd_addr );
static void                     hello_client_add_peer_info( uint16_t conn_id, uint8_t* p_bd_addr, uint8_t role , uint8_t transport, uint8_t address_type );
static void                     hello_client_remove_peer_info( uint16_t conn_id );
static hello_client_peer_info_t *hello_client_get_peer_information( uint16_t conn_id );
static wiced_bool_t             hello_client_save_link_keys( wiced_bt_device_link_keys_t *p_keys);
static wiced_bool_t             hello_client_read_link_keys( wiced_bt_device_link_keys_t *p_keys);
static void                     hello_client_load_keys_to_addr_resolution_db( void );
static void                     hello_client_process_data_from_slave( int len, uint8_t *data );
static void                     hello_client_gatt_enable_notification ( void );
static const gatt_attribute_t*  hello_client_get_attribute(uint16_t handle);
static wiced_bool_t             hello_client_is_device_bonded( wiced_bt_device_address_t bd_address );
static int                      hello_client_get_num_slaves(void);
static int                      hello_client_is_master( BD_ADDR bda );

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
#if defined(CYW20735B1 ) || defined(CYW20735B0)
void application_start( void )
#else
APPLICATION_START( )
#endif
{
    wiced_transport_init( &transport_cfg );

#ifdef WICED_BT_TRACE_ENABLE
    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
#if ( defined(CYW20706A2) || defined(CYW20735B0) || defined(CYW20719B0) || defined(CYW43012C0) )
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif

    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    // Note: WICED HCI must be configured to use this - see wiced_trasnport_init(), must
    // be called with wiced_transport_cfg_t.wiced_tranport_data_handler_t callback present
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

    // WICED BT Stack initialization and registering the managment callback
    // init complete
    wiced_bt_stack_init( hello_client_management_cback,
                    &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools );
}


/*
 * hello client bt/ble device and link management callbacks
 */
wiced_result_t hello_client_management_cback( wiced_bt_management_evt_t event,  wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t               result = WICED_BT_SUCCESS;
    wiced_result_t               add_db_result;

    WICED_BT_TRACE("hello_client_management_cback: %x\n", event );

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            hello_client_app_init( );
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            WICED_BT_TRACE("Numeric_value: %d \n", p_event_data->user_confirmation_request.numeric_value);
            wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS , p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            WICED_BT_TRACE("PassKey Notification. BDA %B, Key %d \n", p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_passkey_notification.bd_addr );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap      = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data          = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req          = BTM_LE_AUTH_REQ_SC_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size      = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys         = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys         = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            {
                wiced_bt_dev_ble_pairing_info_t * p_info = &p_event_data->pairing_complete.pairing_complete_info.ble;

                WICED_BT_TRACE( "Pairing Complete: %d\n", p_info->reason );

                hello_client_smp_bond_result( p_event_data->pairing_complete.bd_addr, p_info->reason );
            }
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            {
                wiced_bt_dev_encryption_status_t * p_status =
                    &p_event_data->encryption_status;

                WICED_BT_TRACE( "encryption status: bd ( %B ) res %d\n",
                        p_status->bd_addr,
                        p_status->result);

                hello_client_encryption_changed( p_status->result ,
                        p_status->bd_addr );
            }
            break;

        case BTM_SECURITY_REQUEST_EVT:
            /* Use the default security */
            wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            hello_client_save_link_keys( &p_event_data->paired_device_link_keys_update );
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            if ( hello_client_read_link_keys( &p_event_data->paired_device_link_keys_request ) )
            {
                WICED_BT_TRACE( "Key retrieval success\n" );
            }
            else
            {
                result = WICED_BT_ERROR;
                WICED_BT_TRACE( "Key retrieval failure\n" );
            }
            break;


        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* Request to store newly generated local identity keys to NVRAM */
            /* (sample app does not store keys to NVRAM) */

            break;


        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /* Request to restore local identity keys from NVRAM (requested during Bluetooth start up) */
            /* (sample app does not store keys to NVRAM).  New local identity keys will be generated */
            result = WICED_BT_NO_RESOURCES;
            break;

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
            WICED_BT_TRACE( "Scan State Change: %d\n", p_event_data->ble_scan_state_changed );
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            WICED_BT_TRACE( "Advertisement State Change: %d\n", p_event_data->ble_advert_state_changed);
            if ( p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_OFF )
            {
                if ( g_hello_client.master_conn_id == 0 )
                {
                    // Start the advertisement to enable slave connection
                    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
                    WICED_BT_TRACE( "wiced_bt_start_advertisements: %d\n", result );
                }
            }
            break;

        default:
            break;
    }

    return result;
}

/*
 *  Pass protocol traces up through the UART
 */
#ifdef ENABLE_HCI_TRACE
void hello_client_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    //send the trace
    wiced_transport_send_hci_trace( NULL, type, length, p_data  );
}
#endif

/*
 * WICED BT Init Complete.  This function is called when device initialization
 * has been completed.  Perform the App Initializations & Callback Registrations
 */
void hello_client_app_init( void )
{
    int                    index;
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t         result;

    WICED_BT_TRACE( "hello_client_app_init\n" );

    memset( &g_hello_client, 0, sizeof( g_hello_client ) );

#ifdef CYW43012C0
    p_hello_client_buffer_pool = wiced_bt_create_pool( 64, 5 );
    if (p_hello_client_buffer_pool == NULL)
        WICED_BT_TRACE("Err: wiced_bt_create_pool failed\n");
#endif // CYW43012C0

#ifdef CYW20706A2
    /* initialize common Bluetooth application logic */
    wiced_bt_app_init( );
#endif

#if defined(CYW20706A2) || defined(CYW20719B0) || defined(CYW20735B0)
    wiced_hal_gpio_register_pin_for_interrupt( HELLO_CLIENT_GPIO_BUTTON, hello_client_interrupt_handler, NULL );
    wiced_hal_gpio_configure_pin( HELLO_CLIENT_GPIO_BUTTON, HELLO_CLIENT_GPIO_SETTINGS , HELLO_CLIENT_DEFAULT_STATE );
#else
#ifndef CYW43012C0
    /* Configure buttons available on the platform */
    wiced_platform_register_button_callback( WICED_PLATFORM_BUTTON_1, hello_client_interrupt_handler, NULL, WICED_PLATFORM_BUTTON_BOTH_EDGE);
#endif
#endif
    // reset connection information
    g_hello_client.num_connections = 0;
    for ( index = 0; index < HELLO_CLIENT_MAX_CONNECTIONS; index++ )
    {
        g_hello_client.peer_info[index].conn_id = 0;
    }

    /* Register with stack to receive GATT related events */
    gatt_status = wiced_bt_gatt_register( hello_client_gatt_callback );

    WICED_BT_TRACE( "wiced_bt_gatt_register status %d \n", gatt_status );

    /*  GATT DB Initialization */
    gatt_status =  wiced_bt_gatt_db_init( hello_client_gatt_database, sizeof( hello_client_gatt_database ) );

    WICED_BT_TRACE( "wiced_bt_gatt_db_init %d \n", gatt_status );

#ifdef ENABLE_HCI_TRACE
    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace( hello_client_hci_trace_cback );
#endif

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* Load the address resolution DB with the keys stored in the NVRAM */
    hello_client_load_keys_to_addr_resolution_db();

    /* Set the advertising data and make the device discoverable */
    hello_client_set_advertisement_data( );

    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );

    WICED_BT_TRACE( "wiced_bt_start_advertisements %d\n", result );

    /* Starting the app timers , seconds timer and the ms timer  */
    if (wiced_init_timer(&hello_client_second_timer, hello_client_app_timer, 0, WICED_SECONDS_PERIODIC_TIMER) == WICED_SUCCESS)
    {
        wiced_start_timer( &hello_client_second_timer, HCLIENT_APP_TIMEOUT_IN_SECONDS );
    }
    UNUSED_VARIABLE(result);
    UNUSED_VARIABLE(gatt_status);
}

/*
 * Callback function is executed to process various GATT events
 */
wiced_bt_gatt_status_t hello_client_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_SUCCESS;

    WICED_BT_TRACE( "hello_client_gatt_callback event %d \n", event );

    switch( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
            if ( p_data->connection_status.connected )
            {
                result = hello_client_gatt_connection_up( &p_data->connection_status );
            }
            else
            {
                result = hello_client_gatt_connection_down( &p_data->connection_status );
            }
            break;

        case GATT_OPERATION_CPLT_EVT:
            result = hello_client_gatt_op_comp_cb( &p_data->operation_complete );
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            result = hello_client_gatt_req_cb( &p_data->attribute_request );
            break;

        default:
            break;
    }

    return result;
}

/* This function will be called on every connection establishment */
/* This function is invoked when connection is established */
wiced_bt_gatt_status_t hello_client_gatt_connection_up( wiced_bt_gatt_connection_status_t *p_conn_status )
{
    uint8_t dev_role;
    wiced_bt_dev_status_t status ;

    if ( g_hello_client.num_connections > HELLO_CLIENT_MAX_CONNECTIONS )
    {
        WICED_BT_TRACE("g_hello_client max connect limit!\n");
        wiced_bt_gatt_disconnect( p_conn_status->conn_id );
        return WICED_BT_GATT_SUCCESS;
    }

    // Keep number of active connections
    g_hello_client.num_connections++;

    wiced_bt_dev_get_role( p_conn_status->bd_addr, &dev_role, BT_TRANSPORT_LE );

    // Adding the peer info
    hello_client_add_peer_info( p_conn_status->conn_id, p_conn_status->bd_addr, dev_role , p_conn_status->transport, p_conn_status->addr_type );

    WICED_BT_TRACE( "hclient_connection_up Conn Id:%d Num conn:%d,Addr:<%B> role:%d\n ",
       p_conn_status->conn_id, g_hello_client.num_connections, p_conn_status->bd_addr, dev_role );

    // This application supports single connection to master (phone) and multiple connections to slaves (hello_sensors)
    if ( dev_role == HCI_ROLE_MASTER )
    {
        g_hello_client.conn_id = p_conn_status->conn_id;
        /* Configure to receive notification from server */
        hello_client_gatt_enable_notification( );
    }
    else // Connected as slave
    {
        // Update the connection handle to the master
        g_hello_client.master_conn_id = p_conn_status->conn_id;

        // Stop the advertisement
        status =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );
        WICED_BT_TRACE(" [%s] start adv status %d \n", __FUNCTION__, status);
    }
    UNUSED_VARIABLE(status);
    return WICED_BT_GATT_SUCCESS;
}

/* This function will be called when connection goes down */
wiced_bt_gatt_status_t hello_client_gatt_connection_down( wiced_bt_gatt_connection_status_t *p_conn_status )
{
    wiced_result_t status;

    WICED_BT_TRACE( "hello_client_connection_down %d <%B>\n", g_hello_client.num_connections, p_conn_status->bd_addr );

    /* Check if the device is there in the peer info table */
    if ( ( g_hello_client.num_connections ) && hello_client_get_peer_information( p_conn_status->conn_id ) )
    {
        // Decrement the number of  connections
        g_hello_client.num_connections--;
    }

    if ( p_conn_status->link_role == HCI_ROLE_SLAVE )
    {
        //Resetting the connection handle to the master
        g_hello_client.master_conn_id = 0;
    }

    //Remove the peer info
    hello_client_remove_peer_info( p_conn_status->conn_id );

     /*  Start the inquiry to search for other available slaves */
    if ( g_hello_client.num_connections < HELLO_CLIENT_MAX_CONNECTIONS )
    {
        uint8_t num_slaves = hello_client_get_num_slaves( );

        if ( g_hello_client.master_conn_id == 0 )
        {
            // Start the advertisement to enable slave connection
            status =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
            WICED_BT_TRACE( "wiced_bt_start_advertisements: %d\n", status );
        }
    }
    UNUSED_VARIABLE(status);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * GATT operation started by the client has been completed
 */
wiced_bt_gatt_status_t hello_client_gatt_op_comp_cb( wiced_bt_gatt_operation_complete_t *p_data )
{
    wiced_result_t              status;
    hello_client_peer_info_t    *p_peer_info = NULL;
    wiced_bt_ble_sec_action_type_t  encryption_type = BTM_BLE_SEC_ENCRYPT;

    WICED_BT_TRACE("hello_client_gatt_op_comp_cb conn %d op %d st %d\n", p_data->conn_id, p_data->op, p_data->status );

    switch ( p_data->op )
    {
    case GATTC_OPTYPE_READ:
        WICED_BT_TRACE( "read_rsp status:%d\n", p_data->status );
        break;

    case GATTC_OPTYPE_WRITE:
        WICED_BT_TRACE( "write_rsp status:%d\n", p_data->status );

        /* server puts authentication requirement. Encrypt the link */
        if( ( p_data->status == WICED_BT_GATT_INSUF_AUTHENTICATION ) && ( p_data->response_data.handle == HANDLE_HSENS_SERVICE_CHAR_CFG_DESC ) )
        {
            if ( ( p_peer_info = hello_client_get_peer_information( p_data->conn_id ) ) != NULL )
            {
                if ( hello_client_is_device_bonded(p_peer_info->peer_addr) )
                {
                    status = wiced_bt_dev_set_encryption( p_peer_info->peer_addr, p_peer_info->transport, &encryption_type );
                    WICED_BT_TRACE( "wiced_bt_dev_set_encryption %d \n", status );
                }
                else
                {
                    status = wiced_bt_dev_sec_bond( p_peer_info->peer_addr, p_peer_info->addr_type,
                                                        p_peer_info->transport,0, NULL );
                    WICED_BT_TRACE( "wiced_bt_dev_sec_bond %d \n", status );
                }
            }
        }
        break;

    case GATTC_OPTYPE_CONFIG:
        WICED_BT_TRACE( "peer mtu:%d\n", p_data->response_data.mtu );
        break;

    case GATTC_OPTYPE_NOTIFICATION:
        hello_client_process_data_from_slave( p_data->response_data.att_value.len, p_data->response_data.att_value.p_data );
        break;

    case GATTC_OPTYPE_INDICATION:
        hello_client_process_data_from_slave( p_data->response_data.att_value.len, p_data->response_data.att_value.p_data );
        wiced_bt_gatt_send_indication_confirm( p_data->conn_id, p_data->response_data.handle );
        break;
    }
    UNUSED_VARIABLE(status);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * This function handles notification/indication data received fromt the slave device
 */
void hello_client_process_data_from_slave( int len, uint8_t *data )
{
    WICED_BT_TRACE("hello_client_process_data_from_slave len:%d master conn_id:%d ccc:%d\n",
            len, g_hello_client.master_conn_id, g_hello_client.host_info.characteristic_client_configuration );

    // if master allows notifications, forward received data from the slave
    if ( ( g_hello_client.host_info.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION ) != 0 )
    {
        wiced_bt_gatt_send_notification( g_hello_client.master_conn_id, HANDLE_HELLO_CLIENT_SERVICE_CHAR_NOTIFY_VAL, len, data );
    }
    else if ( ( g_hello_client.host_info.characteristic_client_configuration & GATT_CLIENT_CONFIG_INDICATION ) != 0 )
    {
        wiced_bt_gatt_send_indication( g_hello_client.master_conn_id, HANDLE_HELLO_CLIENT_SERVICE_CHAR_NOTIFY_VAL, len, data );
    }
}

/*
 * Process various GATT requests received from the master
 */
wiced_bt_gatt_status_t hello_client_gatt_req_cb( wiced_bt_gatt_attribute_request_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

    WICED_BT_TRACE( "hello_sensor_gatts_req_cb. conn %d, type %d\n", p_data->conn_id, p_data->request_type );

    switch ( p_data->request_type )
    {
        case GATTS_REQ_TYPE_READ:
            result = hello_client_gatt_read_handler( p_data->conn_id, &(p_data->data.read_req ) );
            break;

        case GATTS_REQ_TYPE_WRITE:
            result = hello_client_gatt_write_handler( p_data->conn_id, &(p_data->data.write_req ) );
            break;

        case GATTS_REQ_TYPE_MTU:
            WICED_BT_TRACE( "peer mtu:%d\n", p_data->data.mtu );
            break;

        default:
            break;
    }

    return result;
}

/*
 * Process Read Request from master
 */
wiced_bt_gatt_status_t hello_client_gatt_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t *p_read_data )
{
    const gatt_attribute_t * puAttribute;
    const uint8_t *          from;
    int                      to_copy;

    WICED_BT_TRACE("read_hndlr conn %d hdl %x\n", conn_id, p_read_data->handle );

    puAttribute = hello_client_get_attribute( p_read_data->handle );
    if ( puAttribute == NULL )
    {
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    if ( p_read_data->offset >= puAttribute->attr_len )
    {
        return WICED_BT_GATT_INVALID_OFFSET;
    }

    to_copy = puAttribute->attr_len - p_read_data->offset;

    if ( to_copy >= *p_read_data->p_val_len )
    {
        to_copy = *p_read_data->p_val_len - 1;
    }

    from = ((uint8_t *)puAttribute->p_attr) + p_read_data->offset;
    *p_read_data->p_val_len = to_copy;

    memcpy( p_read_data->p_val, from, to_copy );

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process write request or command from peer device
 */
wiced_bt_gatt_status_t hello_client_gatt_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data )
{
    uint8_t *p_attr = p_data->p_val;

    WICED_BT_TRACE("write_handler: conn %d hdl %d prep %d off %d len %d \n ", conn_id, p_data->handle, p_data->is_prep, p_data->offset, p_data->val_len );

    if ( p_data->handle == HANDLE_HELLO_CLIENT_SERVICE_CHAR_CFG_DESC  )
    {
        g_hello_client.host_info.characteristic_client_configuration = p_attr[0] | ( p_attr[1] << 8 );
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Configure data to be sent in the advertisement packets
 */
void hello_client_set_advertisement_data( void )
{
    wiced_result_t result;
    wiced_bt_ble_advert_elem_t adv_elem[3];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t tx_power_level = 0; //0 db

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_TX_POWER;
    adv_elem[num_elem].len          = 1;
    adv_elem[num_elem].p_data       = &tx_power_level;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen(hello_client_local_name);;
    adv_elem[num_elem].p_data       = ( uint8_t* )hello_client_local_name;
    num_elem++;

    result = wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
    WICED_BT_TRACE(" [%s] wiced_bt_ble_set_raw_advertisement_data status %d \n", __FUNCTION__, result);
    UNUSED_VARIABLE(result);
}


static int hello_client_get_num_slaves(void)
{
    int num_slaves = g_hello_client.num_connections;

    if( g_hello_client.master_conn_id )
    {
        num_slaves = g_hello_client.num_connections - 1 ;
    }

    return num_slaves;
}

/* The function invoked on timeout of app seconds timer. */
void hello_client_app_timer( uint32_t arg )
{
    wiced_result_t               status;

    g_hello_client.app_timer_count++;
    WICED_BT_TRACE( "%d\n", g_hello_client.app_timer_count );

    if( start_scan && wiced_bt_ble_get_current_scan_state() == BTM_BLE_SCAN_TYPE_NONE )
    {
        status = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, hello_client_scan_result_cback );
        WICED_BT_TRACE( "wiced_bt_ble_scan: %d\n", status );
    }
    UNUSED_VARIABLE(status);
}

/*
 * Process SMP bonding result.  If we successfully paired with the
 * central device, save its BDADDR in the NVRAM and initialize
 * associated data
 */
void hello_client_smp_bond_result( BD_ADDR bda, uint8_t result )
{
    wiced_result_t status;

    /*  Start the inquiry to search for other available slaves */
    if ( g_hello_client.num_connections < HELLO_CLIENT_MAX_CONNECTIONS )
    {
        int num_slaves = hello_client_get_num_slaves();

        if ( g_hello_client.master_conn_id == 0 )
        {
            // Start the advertisement to enable slave connection
            status =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
            WICED_BT_TRACE( "wiced_bt_start_advertisements: %d\n", status );
        }

        if ( ( num_slaves < HELLO_CLIENT_MAX_SLAVES ) && ( start_scan ) )
        {
            if( wiced_bt_ble_get_current_scan_state() == BTM_BLE_SCAN_TYPE_NONE )
            {
                status = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, hello_client_scan_result_cback );
                WICED_BT_TRACE( "hello_client_smp_bond_result wiced_bt_ble_scan: %d\n", status );
            }
        }
    }
    UNUSED_VARIABLE(status);
}

/*
 * Process notification from the stack that encryption has been set.
 * If connected client is registered for notification or indication,
 * it is a good time to send it out
 */
void hello_client_encryption_changed( wiced_result_t result, uint8_t* p_bd_addr )
{
    WICED_BT_TRACE( "hello_client_encryption_changed %d", result );

    /* Bonding success */
    if( result == WICED_BT_SUCCESS )
    {
        // When we are connected as a master, we need to enable notifications from the slave
        // This needs to be done only once, because client configuration descriptor value
        // should be persistent across connections with bonded devices.
        if ( hello_client_is_master ( p_bd_addr) )
        {
            hello_client_gatt_enable_notification( );
        }
    }
}
#ifndef CYW43012C0
/* This function is invoked on button interrupt events */
void hello_client_interrupt_handler(void* user_data, uint8_t value )
{
    wiced_result_t  result;
    int             num_slaves = 0;
    static uint32_t button_pushed_time = 0;

    //WICED_BT_TRACE( "But1 %d, But2 %d, But3 %d \n", value & 0x01, ( value & 0x02 ) >> 1, ( value & 0x04 ) >> 2 );
    WICED_BT_TRACE( "hello_client_interrupt_handler, app timer :%d\n", g_hello_client.app_timer_count );

    if ( wiced_hal_gpio_get_pin_input_status(HELLO_CLIENT_GPIO_BUTTON) == HELLO_CLIENT_BUTTON_PRESSED_VALUE )
    {
        WICED_BT_TRACE( " Button pressed\n" );
        button_pushed_time = g_hello_client.app_timer_count;
    }
    else if ( button_pushed_time != 0 )
    {
        WICED_BT_TRACE( " Button released " );
        //Start the scan if the button is pressed for more than 5 seconds
        if ( g_hello_client.app_timer_count - button_pushed_time > 5 )
        {
			num_slaves = hello_client_get_num_slaves();
            WICED_BT_TRACE( " after more than 5s, connecting to next slave number %d\n", num_slaves+1 );

            if ( num_slaves < HELLO_CLIENT_MAX_SLAVES )
            {
                start_scan = 1;
                if( wiced_bt_ble_get_current_scan_state() == BTM_BLE_SCAN_TYPE_NONE )
                {
                    result = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, hello_client_scan_result_cback );
                    WICED_BT_TRACE( "\nhello_client_interrupt_handler wiced_bt_ble_scan: %d\n", result );
                }
            }
            else
            {
                WICED_BT_TRACE(" Scan Not Started. Connected to HELLO_CLIENT_MAX_SLAVES!! \n" );
            }
        }
        else
        {
            WICED_BT_TRACE( " before less than 5s, sending notifications\n" );
            wiced_bt_gatt_send_notification( g_hello_client.master_conn_id,
                    HANDLE_HELLO_CLIENT_SERVICE_CHAR_NOTIFY_VAL,
                    sizeof( hello_client_notify_value ), hello_client_notify_value );
        }
    }
}
#endif

/*
 * This function handles the scan results
 */
void hello_client_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
{
    wiced_result_t         status;
    wiced_bool_t           ret_status;
    uint8_t                length;
    uint8_t *              p_data;

    if ( p_scan_result )
    {
        // Advertisement data from hello_server should have Advertisement type SERVICE_UUID_128
        p_data = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE, &length );

        // Check if  the hello service uuid is there in the advertisement
        if ( ( p_data == NULL ) || ( length != LEN_UUID_128 ) || ( memcmp( p_data, hello_service, LEN_UUID_128 ) != 0 ) )
        {
            // wrong device
            return;
        }

        WICED_BT_TRACE(" Found Device : %B \n", p_scan_result->remote_bd_addr );
        start_scan = 0;
        /* Stop the scan since the desired device is found */
        status = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, hello_client_scan_result_cback );

        WICED_BT_TRACE( "scan off status %d\n", status );

        /* Initiate the connection */
        ret_status = wiced_bt_gatt_le_connect( p_scan_result->remote_bd_addr, p_scan_result->ble_addr_type, BLE_CONN_MODE_HIGH_DUTY, TRUE );

        WICED_BT_TRACE( "wiced_bt_gatt_connect status %d\n", ret_status );
    }
    else
    {
        WICED_BT_TRACE( "Scan completed:\n" );
    }
    UNUSED_VARIABLE(ret_status);
    UNUSED_VARIABLE(status);
}

/*
 * This function writes into peer's client configuration descriptor to enable notifications
 */
void hello_client_gatt_enable_notification ( void )
{
    wiced_bt_gatt_status_t status;
    uint16_t               u16 = GATT_CLIENT_CONFIG_NOTIFICATION;

    // Allocating a buffer to send the write request
#ifdef CYW43012C0
    wiced_bt_gatt_value_t *p_write = ( wiced_bt_gatt_value_t* )wiced_bt_get_buffer_from_pool( p_hello_client_buffer_pool );
#else
    wiced_bt_gatt_value_t *p_write = ( wiced_bt_gatt_value_t* )wiced_bt_get_buffer( sizeof( wiced_bt_gatt_value_t ) + 1 );
#endif

    if ( p_write )
    {
        p_write->handle   = HANDLE_HSENS_SERVICE_CHAR_CFG_DESC; /* hard coded server ccd */
        p_write->offset   = 0;
        p_write->len      = 2;
        p_write->auth_req = GATT_AUTH_REQ_NONE;
        p_write->value[0] = u16 & 0xff;
        p_write->value[1] = (u16 >> 8) & 0xff;

        // Register with the server to receive notification
        status = wiced_bt_gatt_send_write ( g_hello_client.conn_id, GATT_WRITE, p_write );

        WICED_BT_TRACE("wiced_bt_gatt_send_write \n", status);

        wiced_bt_free_buffer( p_write );
    }
    UNUSED_VARIABLE(status);
}

/*
 * This function adds the peer information to the table
 */
void hello_client_add_peer_info( uint16_t conn_id, uint8_t* p_bd_addr, uint8_t role , uint8_t transport, uint8_t address_type )
{
    int index;

    for ( index = 0; index < HELLO_CLIENT_MAX_CONNECTIONS; index++ )
    {
        if ( g_hello_client.peer_info[index].conn_id == 0 )
        {
            g_hello_client.peer_info[index].conn_id         = conn_id;
            g_hello_client.peer_info[index].role            = role;
            g_hello_client.peer_info[index].transport       = transport;
            g_hello_client.peer_info[index].addr_type       = address_type;
            memcpy( g_hello_client.peer_info[index].peer_addr, p_bd_addr, BD_ADDR_LEN );
            break;
        }
    }
}

/*
 * This function removes the peer information from the table
 */
void hello_client_remove_peer_info( uint16_t conn_id )
{
    int index;

    for ( index = 0; index < HELLO_CLIENT_MAX_CONNECTIONS; index++ )
    {
        if ( g_hello_client.peer_info[index].conn_id == conn_id )
        {
            g_hello_client.peer_info[index].conn_id = 0;
        }
    }
}

/*
 * This function gets the peer address from  the table
 */
hello_client_peer_info_t * hello_client_get_peer_information( uint16_t conn_id )
{
    int index;

    for ( index = 0; index < HELLO_CLIENT_MAX_CONNECTIONS; index++ )
    {
        if ( g_hello_client.peer_info[index].conn_id == conn_id )
        {
            return &g_hello_client.peer_info[index];
        }
    }
    return NULL;
}

/*
 * Find out if specific device is connected as a master
 */
static int hello_client_is_master( BD_ADDR bda )
{
    int index;

    for ( index = 0; index < HELLO_CLIENT_MAX_CONNECTIONS; index++ )
    {
        if ( g_hello_client.peer_info[index].conn_id != 0 )
        {
            if ( memcmp( g_hello_client.peer_info[index].peer_addr, bda, BD_ADDR_LEN) == 0 )
            {
                return ( g_hello_client.peer_info[index].role == HCI_ROLE_MASTER );
            }
        }
    }
    return FALSE;
}

/*
 * This function is called to save keys generated as a result of pairing or keys update
 */
wiced_bool_t hello_client_save_link_keys( wiced_bt_device_link_keys_t *p_keys)
{
    uint8_t                     bytes_written, bytes_read;
    wiced_bt_device_link_keys_t temp_keys;
    uint16_t                    id = 0, i;
    wiced_result_t              result;

    // search through all available NVRAM IDs.
    for ( i = WICED_NVRAM_VSID_START; i < WICED_NVRAM_VSID_END; i++ )
    {
        bytes_read = wiced_hal_read_nvram( i, sizeof( temp_keys ), (uint8_t *)&temp_keys, &result );

        WICED_BT_TRACE( "Read NVRAM at:%d bytes:%d result:%d\n", i, bytes_read, result );

        // if failed to read NVRAM, there is nothing saved at that location
        if ( ( result != WICED_SUCCESS ) || ( bytes_read != sizeof( temp_keys ) ) )
        {
            id = i;
            break;
        }
        else
        {
            if ( memcmp( temp_keys.bd_addr, p_keys->bd_addr, BD_ADDR_LEN ) == 0 )
            {
                // keys for this device have been saved, reuse the ID
                id = i;
                break;
            }
        }
    }
    if ( id == 0 )
    {
        // all NVRAM locations are already occupied.  Cann't save anything.
        WICED_BT_TRACE( "Failed to save NVRAM\n" );
        return WICED_FALSE;
    }
    WICED_BT_TRACE( "writing to id:%d\n", id );
    bytes_written = wiced_hal_write_nvram( id, sizeof( wiced_bt_device_link_keys_t ), (uint8_t *)p_keys, &result );
    WICED_BT_TRACE( "Saved %d bytes at id:%d %d\n", bytes_written, id );
    UNUSED_VARIABLE(bytes_written);
    return WICED_TRUE;
}

/*
 * This function is called to read keys for specific bdaddr
 */
wiced_bool_t hello_client_read_link_keys( wiced_bt_device_link_keys_t *p_keys)
{
    wiced_bt_device_link_keys_t temp_keys;
    uint8_t                     bytes_read;
    uint16_t                    i;
    wiced_result_t              result;

    // search through all available NVRAM IDs.
    for ( i = WICED_NVRAM_VSID_START; i < WICED_NVRAM_VSID_END; i++ )
    {
        bytes_read = wiced_hal_read_nvram( i, sizeof( temp_keys ), (uint8_t *)&temp_keys, &result );

        WICED_BT_TRACE(" [%s] read status %d bytes read %d \n", __FUNCTION__, result, bytes_read);

        // if failed to read NVRAM, there is nothing saved at that location
        if ( result == WICED_SUCCESS )
        {
            if ( memcmp( temp_keys.bd_addr, p_keys->bd_addr, BD_ADDR_LEN ) == 0 )
            {
                // keys for this device have been saved
                memcpy( &p_keys->key_data, &temp_keys.key_data, sizeof( temp_keys.key_data ) );
                return WICED_TRUE;
            }
        }
        else
        {
            break;
        }
    }
    UNUSED_VARIABLE(bytes_read);
    return WICED_FALSE;
}

void hello_client_load_keys_to_addr_resolution_db( void )
{
    uint8_t                     bytes_read;
    uint16_t                    i;
    wiced_result_t              result;
    wiced_bt_device_link_keys_t keys;

    // search through all available NVRAM IDs.
    for ( i = WICED_NVRAM_VSID_START; i < WICED_NVRAM_VSID_END; i++ )
    {
        bytes_read = wiced_hal_read_nvram( i, sizeof( keys ), (uint8_t *)&keys, &result );

        WICED_BT_TRACE(" [%s] read status %d bytes read %d \n", __FUNCTION__, result, bytes_read);

        // if failed to read NVRAM, there is nothing saved at that location
        if ( result == WICED_SUCCESS )
        {
#ifdef CYW20706A2
            result = wiced_bt_dev_add_device_to_address_resolution_db( &keys, keys.key_data.ble_addr_type );
#else
            result = wiced_bt_dev_add_device_to_address_resolution_db( &keys );
#endif

            WICED_BT_TRACE("Updated Addr Resolution DB:%d\n", result );
        }
        else
        {
            break;
        }
    }
    UNUSED_VARIABLE(bytes_read);
}

/*
 * Find attribute based on attribute handle
 */
const gatt_attribute_t * hello_client_get_attribute(uint16_t handle)
{
    const gatt_attribute_t * puAttributes = hello_client_gattdb_attributes;
    int   i;

    for ( i = 0; i < sizeof( hello_client_gattdb_attributes ) / sizeof( hello_client_gattdb_attributes[0] ); i++, puAttributes++ )
    {
        if ( puAttributes->handle == handle )
        {
            return puAttributes;
        }
    }
    WICED_BT_TRACE( "Attr handle:0x%x not found\n", handle );
    return NULL;
}

/* Check for device entry exists in NVRAM list */
wiced_bool_t hello_client_is_device_bonded( wiced_bt_device_address_t bd_address )
{
    wiced_bt_device_link_keys_t temp_keys;
    uint8_t                     bytes_read;
    uint16_t                    i;
    wiced_result_t              result;

    // search through all available NVRAM IDs.
    for ( i = WICED_NVRAM_VSID_START; i < WICED_NVRAM_VSID_END; i++ )
    {
        bytes_read = wiced_hal_read_nvram( i, sizeof( temp_keys ), (uint8_t *)&temp_keys, &result );

        WICED_BT_TRACE(" [%s] read status %d bytes read %d \n", __FUNCTION__, result, bytes_read);

        // if failed to read NVRAM, there is nothing saved at that location
        if ( result == WICED_SUCCESS )
        {
            if ( memcmp( temp_keys.bd_addr, bd_address, BD_ADDR_LEN ) == 0 )
            {
                return WICED_TRUE;
            }
        }
        else
        {
            break;
        }
    }
    UNUSED_VARIABLE(bytes_read);
    return WICED_FALSE;
}
