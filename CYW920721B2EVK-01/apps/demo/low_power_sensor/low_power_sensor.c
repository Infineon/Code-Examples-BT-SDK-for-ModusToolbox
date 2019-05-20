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
* Low power BLE Vendor Specific Device
*
* During initialization the application registers with LE stack to receive various
* notifications including bonding complete, connection status change,
* peer write and configure to enter to sleep on application idle time out(10 seconds).
* If no interaction from peer until 10 seconds of application start, device enter to sleep.
* Device can be wake up using GPIO (Please check device_wake_gpio_num
* to get to know  which pin configure as wake source) toggle or by intercating from peer.
* When device is successfully bonded, application saves
* peer's Bluetooth Device address to the NVRAM.  Bonded device can also
* write in to client configuration descriptor of the notification
* characteristic.  That is also saved in the NVRAM.  When user pushes the
* button(when device in wake up state), notification/indication is sent to the bonded and registered host.
*
*
* Features demonstrated
*  - Configuring for Shut Down Sleep and non Shut Down Sleep
*  - GATT database and Device configuration initialization
*  - Registration with LE stack for various events
*  - NVRAM read/write operation
*  - Configuring for sleep on app idle.
*  - Saving and restoring application context on entering and exiting from sleep.
*  - Sending data to the client
*  - Processing write requests from the client
 *
* To demonstrate the application, work through the following steps.
* 1. Plug the WICED eval board into your computer
* 2. Build and download the application (to the WICED board)
*    (a) To configure shutdown sleep, modify SLEEP_TYPE=SLEEP_TYPE_SHUTDOWN in application settings.
*    (b) To configure non shutdown sleep, modify SLEEP_TYPE=SLEEP_TYPE_NOT_SHUTDOWN in application settings.
*    (c) To configure sleep with transport, modify SLEEP_MODE=SLEEP_MODE_TRANSPORT in application settings.
*        Note: Device cannot sleep without connecting to transport. Hence expecting device should be connected
*        to transport after application boot.
*    (d) To configure sleep without transport, modify SLEEP_MODE=SLEEP_MODE_NO_TRANSPORT in application settings.
*    Note: Default configuration is sleep without transport and Shut Down Sleep.
*
* 3. starts advertisements.
* 4. If no one attempt to pair, hello sensor enter to sleep after idle timeout(10 seconds).
* 5. Pair with a client. Hello sensor should wake from sleep if it is in sleep and should allow pairing
*     Note: when app advertising in sleep mode, peer need to re attempt connection,
*     if fails first time
* 6. On the client side register for notifications
* 7. Push a button on the tag to send notifications to the client
* 8. if no activity on app, enters sleep on idle time out.
* 9. Wake the device.
*     (a) Wake using configured wake source (Please check device_wake_gpio_num
*         to get to know  which pin configure as wake source).
*         Initially wake source need to connect to ground. To wake the device, connect wake source to High.
*         to allow device to sleep again, connect wake source back to ground.
*         Note: In this sample, button configured using LHL GPIO. Hence device can be woken up using button press also. But cannot
*         guaranteed that on button press wake, interrupt handler get called, but guaranteed that device wake up. Correct button usage is
*         wake device using wake source then press button to send notification/to start ADV to get to connect etc.
*     (b) If device is in connected state and in sleep, peer interactions (for example GATT attribute Read/write) cause device wake.
*
*Note: When sleep configured with SLEEP_MODE_TRANSPORT, without connecting to transport(i.e. to WICED_HCI UART)
*      device does not allowed sleep. Expectation is after power up, transport should be connected immediately.
*/
#include "sparcommon.h"
#include "spar_utils.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_uuid.h"
#include "wiced_result.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"
#include "wiced_platform.h"
#include "wiced_sleep.h"
#include "low_power_sensor.h"
#include "wiced_transport.h"
#include "wiced_bt_l2c.h"
#include "wiced_timer.h"
#include "wiced_platform.h"
#ifdef  WICED_BT_TRACE_ENABLE
#include "wiced_bt_trace.h"
#endif

/*Enable shutdown sleep if configured for shutdown sleep */
#if (WICED_SLEEP_TYPE == SLEEP_TYPE_SHUTDOWN)
#define ENABLE_SHUTDOWN_SLEEP
#endif

/******************************************************************************
 *                                Constants
 ******************************************************************************/
//#define CONNECTION_INTERVAL  16   // 20ms
#define CONNECTION_INTERVAL  80  // 100ms
//#define CONNECTION_INTERVAL  160  // 200ms
//#define CONNECTION_INTERVAL  400  // 500ms

#define CYPRESS_COMPANY_ID          0x0131
#define WICED_LOW_POWER_SENSOR_ID   0x0010
#define MANUFACTURER_DATA_LENGTH    4

/******************************************************************************
 *                                Structures
 ******************************************************************************/
typedef struct
{
    BD_ADDR     remote_addr;              // remote peer device address
    uint16_t    conn_id;                  // connection ID referenced by the stack
    uint16_t    peer_mtu;                 // peer MTU
    uint8_t     num_to_write;             // num msgs to send, incr on each button intr
    uint8_t     flag_indication_sent;     // indicates waiting for ack/cfm
    uint8_t     battery_level;            // dummy battery level
    uint8_t     application_state;        //application state, see hello_sensor_mode_t
} hello_sensor_state_t;

#pragma pack(1)
/* Host information saved in  NVRAM */
typedef PACKED struct
{
    BD_ADDR  bdaddr;                                /* BD address of the bonded host */
    uint16_t  characteristic_client_configuration;  /* Current value of the client configuration descriptor */
} host_info_t;
#pragma pack()

typedef struct
{
    uint16_t handle;
    uint16_t attr_len;
    void     *p_attr;
} attribute_t;

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];


/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
/*
 * This is the GATT database for the Hello Sensor application.  It defines
 * services, characteristics and descriptors supported by the sensor.  Each
 * attribute in the database has a handle, (characteristic has two, one for
 * characteristic itself, another for the value).  The handles are used by
 * the peer to access attributes, and can be used locally by application for
 * example to retrieve data written by the peer.  Definition of characteristics
 * and descriptors has GATT Properties (read, write, notify...) but also has
 * permissions which identify if and how peer is allowed to read or write
 * into it.  All handles do not need to be sequential, but need to be in
 * ascending order.
 */
const uint8_t hello_sensor_gatt_database[]=
{
    // Declare mandatory GATT service
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_GATT_SERVICE, UUID_SERVICE_GATT ),

    // Declare mandatory GAP service. Device Name and Appearance are mandatory
    // characteristics of GAP service
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_GAP_SERVICE, UUID_SERVICE_GAP ),

    // Declare mandatory GAP service characteristic: Dev Name
        CHARACTERISTIC_UUID16( HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME, HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,
            UUID_CHARACTERISTIC_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

    // Declare mandatory GAP service characteristic: Appearance
        CHARACTERISTIC_UUID16( HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE, HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
            UUID_CHARACTERISTIC_APPEARANCE, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

    // Declare proprietary Hello Service with 128 byte UUID
    PRIMARY_SERVICE_UUID128( HANDLE_HSENS_SERVICE, UUID_HELLO_SERVICE ),

    // Declare characteristic used to notify/indicate change
        CHARACTERISTIC_UUID128( HANDLE_HSENS_SERVICE_CHAR_NOTIFY, HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL,
            UUID_HELLO_CHARACTERISTIC_NOTIFY, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE, LEGATTDB_PERM_READABLE ),

            // Declare client characteristic configuration descriptor
    // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection
            // for bonded devices.  Setting value to 1 tells this application to send notification
            // when value of the characteristic changes.  Value 2 is to allow indications.
            CHAR_DESCRIPTOR_UUID16_WRITABLE( HANDLE_HSENS_SERVICE_CHAR_CFG_DESC, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_AUTH_READABLE | LEGATTDB_PERM_AUTH_WRITABLE ),

    // Declare Device info service
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE, UUID_SERVICE_DEVICE_INFORMATION ),

    // Handle 0x4e: characteristic Manufacturer Name, handle 0x4f characteristic value
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
            UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

    // Handle 0x50: characteristic Model Number, handle 0x51 characteristic value
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,
            UUID_CHARACTERISTIC_MODEL_NUMBER_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

    // Handle 0x52: characteristic System ID, handle 0x53 characteristic value
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,
            UUID_CHARACTERISTIC_SYSTEM_ID, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

    // Declare Battery service
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_BATTERY_SERVICE, 0x180F ),

    // Handle 0x62: characteristic Battery Level, handle 0x63 characteristic value
        CHARACTERISTIC_UUID16( HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL, HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL,
            UUID_CHARACTERISTIC_BATTERY_LEVEL, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),
};

uint8_t hello_sensor_device_name[]          = "low_power_sensor";
uint8_t hello_sensor_appearance_name[2]     = { BIT16_TO_8(APPEARANCE_GENERIC_TAG) };
char    hello_sensor_char_notify_value[]    = { 'H', 'e', 'l', 'l', 'o', ' ', '0', };
char    hello_sensor_char_mfr_name_value[]  = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0, };
char    hello_sensor_char_model_num_value[] = { '1', '2', '3', '4',   0,   0,   0,   0 };
uint8_t hello_sensor_char_system_id_value[] = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71};

#ifdef ENABLE_SHUTDOWN_SLEEP
/* Required Data need to be ratined using retention RAM with shut down sleep */
PLACE_DATA_IN_RETENTION_RAM hello_sensor_state_t hello_sensor_state;
PLACE_DATA_IN_RETENTION_RAM char hello_sensor_char_notify_incremented_value;
#else
hello_sensor_state_t hello_sensor_state;
#endif

wiced_timer_t hello_sensor_idle_timer;

/* Holds the host info saved in the NVRAM */
host_info_t             hello_sensor_hostinfo;

/*sleep configuration*/
wiced_sleep_config_t    hello_sensor_sleep_config;
uint8_t                 hello_sensor_boot_mode;

attribute_t gauAttributes[] =
{
    { HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,       sizeof( hello_sensor_device_name ),         hello_sensor_device_name },
    { HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL, sizeof(hello_sensor_appearance_name),       hello_sensor_appearance_name },
    { HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL,             sizeof(hello_sensor_char_notify_value),     hello_sensor_char_notify_value },
    { HANDLE_HSENS_SERVICE_CHAR_CFG_DESC,               2,                                          (void*)&hello_sensor_hostinfo.characteristic_client_configuration },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,  sizeof(hello_sensor_char_mfr_name_value),   hello_sensor_char_mfr_name_value },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL, sizeof(hello_sensor_char_model_num_value),  hello_sensor_char_model_num_value },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL, sizeof(hello_sensor_char_system_id_value),  hello_sensor_char_system_id_value },
    { HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL,      1,                                          &hello_sensor_state.battery_level },
};

static wiced_result_t           hello_sensor_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static wiced_bt_gatt_status_t   hello_sensor_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
static void                     hello_sensor_set_advertisement_data(void);
static void                     hello_sensor_send_message( void );
static void                     hello_sensor_gatts_increment_notify_value( void );
static void                     hello_sensor_smp_bond_result( uint8_t result );
static void                     hello_sensor_encryption_changed( wiced_result_t result, uint8_t* bd_addr );
static void                     hello_sensor_interrupt_handler(void* user_data, uint8_t value );
static void                     hello_sensor_application_init( void );
static void                     hello_sensor_load_keys_for_address_resolution( void );
static void                     hello_sensor_idle_timer_handler(uint32_t data);
static uint32_t                 hello_sensor_sleep_handler(wiced_sleep_poll_type_t type );
static void                     hello_sensor_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data );
static wiced_bool_t             hello_sensor_is_device_bonded( wiced_bt_device_address_t bd_address );
static wiced_bool_t             hello_sensor_is_device_wake_pin_asserted (void);

/* transport configuration */
const wiced_transport_cfg_t transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg =
    {
        .uart_cfg =
        {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
            .baud_rate =  115200
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

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
/**
 * @addtogroup  low_power_sensor    low power sample application
 * @ingroup     low_power_sensor
 * @{
 */

/**
 * Function         APPLICATION_START
 *
 *                  Entry point to the application. Set device configuration and start BT
 *                  stack initialization.  The actual application initialization will happen
 *                  when stack reports that BT device is ready.
 *                  Note: When peer wake device by sending data, the assumption is device
 *                  should wake up and process peer request as soon as possible.
 *                  So we should not add more delay/code that introduce more delay in APPLICATION_START.

 *
 * @param[in]       void
 *
 * @return          void
 */

/** @} low_power_sensor */
#if defined(CYW20735B1)
void application_start( void )
#else
APPLICATION_START()
#endif
{
    wiced_bool_t        sleep_register_status;

    wiced_transport_init( &transport_cfg );

    //Set the debug uart to enable the debug traces
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);

    // Register call back and configuration with stack
    wiced_bt_stack_init( hello_sensor_management_cback ,
                         &wiced_bt_cfg_settings,
                         wiced_bt_cfg_buf_pools );

    /* configure to sleep if sensor is idle */
#if (WICED_SLEEP_MODE == SLEEP_MODE_NO_TRANSPORT)
    hello_sensor_sleep_config.sleep_mode             = WICED_SLEEP_MODE_NO_TRANSPORT;
#else
    hello_sensor_sleep_config.sleep_mode             = WICED_SLEEP_MODE_TRANSPORT;
#endif

#if defined (CYW20735B1)
    hello_sensor_sleep_config.device_wake_gpio_num   = WICED_P04;
#elif (defined(CYW20719B1) || defined(CYW20721B1))
    hello_sensor_sleep_config.device_wake_gpio_num   = WICED_P17;
#endif

    hello_sensor_sleep_config.device_wake_mode       = WICED_SLEEP_WAKE_ACTIVE_HIGH;
    hello_sensor_sleep_config.device_wake_source     = WICED_SLEEP_WAKE_SOURCE_GPIO;
    hello_sensor_sleep_config.host_wake_mode         = WICED_SLEEP_WAKE_ACTIVE_HIGH;
    hello_sensor_sleep_config.sleep_permit_handler   = hello_sensor_sleep_handler;

    wiced_sleep_configure( &hello_sensor_sleep_config );

    if( wiced_sleep_get_boot_mode(  ) == WICED_SLEEP_COLD_BOOT )
    {
        hello_sensor_boot_mode = WICED_SLEEP_COLD_BOOT;
        WICED_BT_TRACE( "Hello Sensor cold Start \n");
    }
    else
    {
        hello_sensor_boot_mode = WICED_SLEEP_FAST_BOOT;
        WICED_BT_TRACE( "Hello Sensor warm Start \n");
    }
}
wiced_bool_t hello_sensor_is_device_wake_pin_asserted ()
{
    uint32_t pin_status;

    pin_status = wiced_hal_gpio_get_pin_input_status(hello_sensor_sleep_config.device_wake_gpio_num);
    return (!(pin_status ^ hello_sensor_sleep_config.device_wake_mode));
}

uint32_t hello_sensor_sleep_handler(wiced_sleep_poll_type_t type )
{
    uint32_t ret = WICED_SLEEP_NOT_ALLOWED;

    WICED_BT_TRACE(".");

    /* Do not allow sleep if device wake pin is in asserted state */
    if (hello_sensor_is_device_wake_pin_asserted ())
        return WICED_SLEEP_NOT_ALLOWED;

    switch(type)
    {
        case WICED_SLEEP_POLL_SLEEP_PERMISSION:
            if( hello_sensor_state.application_state == HELLO_SENSOR_IDLE )
            {
#ifdef ENABLE_SHUTDOWN_SLEEP
                ret = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;
                WICED_BT_TRACE("$");
#else
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
#endif
            }
            break;

        case WICED_SLEEP_POLL_TIME_TO_SLEEP:
            if( hello_sensor_state.application_state != HELLO_SENSOR_IDLE )
            {
                ret = 0;
            }
            else
            {
                ret = WICED_SLEEP_MAX_TIME_TO_SLEEP;
            }
            break;
    }

    return ret;
}

/*
 * This function is executed in the BTM_ENABLED_EVT management callback.
 */
void hello_sensor_application_init( void )
{
    wiced_bt_gatt_status_t  gatt_status;
    wiced_result_t          result;

    WICED_BT_TRACE( "hello_sensor_application_init \n");

    hello_sensor_state.application_state = HELLO_SENSOR_NOT_IDLE;

    /* Configure buttons available on the platform */
    wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON_1, hello_sensor_interrupt_handler, NULL );
    wiced_hal_gpio_configure_pin( WICED_GPIO_PIN_BUTTON_1, WICED_GPIO_BUTTON_SETTINGS(GPIO_EN_INT_RISING_EDGE), GPIO_PIN_OUTPUT_LOW );

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(hello_sensor_gatts_callback);

    WICED_BT_TRACE("wiced_bt_gatt_register: %d\n", gatt_status);

    /*  Tell stack to use our GATT databse */
    gatt_status =  wiced_bt_gatt_db_init( hello_sensor_gatt_database, sizeof(hello_sensor_gatt_database) );
    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\n", gatt_status);

    /* Enable pairing */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    wiced_bt_dev_register_hci_trace( hello_sensor_hci_trace_cback );

    /* Load keys for adress resolution */
    hello_sensor_load_keys_for_address_resolution();

    /* start adv only in not connected state */
    if ( hello_sensor_state.conn_id == 0 )
    {
        /* Set the advertising params and make the device discoverable */
        hello_sensor_set_advertisement_data();

        result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
        WICED_BT_TRACE( "wiced_bt_start_advertisements %d\n", result );
    }

    /* Start idle timer to enter to sleep */
    if ( wiced_init_timer( &hello_sensor_idle_timer, hello_sensor_idle_timer_handler, 0, WICED_MILLI_SECONDS_TIMER )== WICED_SUCCESS )
    {
        if ( wiced_start_timer( &hello_sensor_idle_timer, HELLO_SENSOR_IDLE_TIMEOUT ) !=  WICED_SUCCESS )
        {
            WICED_BT_TRACE("idle timer start failure\n");
            return;
        }
    }
    else
    {
        WICED_BT_TRACE("idle timer init fail \n");
    }

    /* read the host info saved in NVRAM  */
    if ( wiced_sleep_get_boot_mode(  ) == WICED_SLEEP_FAST_BOOT )
    {
        wiced_hal_read_nvram( HELLO_SENSOR_VS_ID, sizeof(hello_sensor_hostinfo), (uint8_t*)&hello_sensor_hostinfo, &result );
#ifdef ENABLE_SHUTDOWN_SLEEP
        hello_sensor_char_notify_value[sizeof( hello_sensor_char_notify_value ) - 1] = hello_sensor_char_notify_incremented_value;
#endif
    }
}

/*
 * Setup advertisement data with 16 byte UUID and device name
 */
void hello_sensor_set_advertisement_data(void)
{
    wiced_bt_ble_advert_elem_t adv_elem[3];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t manufacturer_data[MANUFACTURER_DATA_LENGTH];

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    /* manufacturer data (vendor id, product id information)*/
    manufacturer_data[0]            = CYPRESS_COMPANY_ID & 0xff;
    manufacturer_data[1]            = ( CYPRESS_COMPANY_ID >> 8 ) & 0xff;
    manufacturer_data[2]            = WICED_LOW_POWER_SENSOR_ID & 0xff;
    manufacturer_data[3]            = ( WICED_LOW_POWER_SENSOR_ID >> 8 ) & 0xff;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_MANUFACTURER;
    adv_elem[num_elem].len          = MANUFACTURER_DATA_LENGTH;
    adv_elem[num_elem].p_data       = manufacturer_data;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen( (const char *)wiced_bt_cfg_settings.device_name );
    adv_elem[num_elem].p_data       = ( uint8_t* )wiced_bt_cfg_settings.device_name;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

void hello_sensor_idle_timer_handler(uint32_t data)
{
    WICED_BT_TRACE("hello sensor idle. \n");
    hello_sensor_state.application_state = HELLO_SENSOR_IDLE;
}

/*
 * Process SMP bonding result. If we successfully paired with the
 * central device, save its BDADDR in the NVRAM and initialize
 * associated data
 */
void hello_sensor_smp_bond_result( uint8_t result )
{
    wiced_result_t status;
    uint8_t written_byte = 0;
    WICED_BT_TRACE( "hello_sensor, bond result: %d\n", result );

    /* Bonding success */
    if ( result == WICED_BT_SUCCESS )
    {
        /* Pack the data to be stored into the hostinfo structure */
        memcpy( hello_sensor_hostinfo.bdaddr, hello_sensor_state.remote_addr, sizeof( BD_ADDR ) );
        hello_sensor_hostinfo.characteristic_client_configuration = 0;

        /* Write to NVRAM */
        written_byte = wiced_hal_write_nvram( HELLO_SENSOR_VS_ID, sizeof(hello_sensor_hostinfo), (uint8_t*)&hello_sensor_hostinfo, &status );
        WICED_BT_TRACE("NVRAM write: %d\n", written_byte);
    }

    UNUSED_VARIABLE(written_byte);
}


/*
 * Process notification from stack that encryption has been set. If connected
 * client is registered for notification or indication, it is a good time to
 * send it out
 */
void hello_sensor_encryption_changed( wiced_result_t result, uint8_t* bd_addr )
{
    WICED_BT_TRACE( "encryp change bd ( %B ) res: %d ", hello_sensor_hostinfo.bdaddr,  result);

    /* Connection has been encrypted meaning that we have correct/paired device
     * restore values in the database
     */
    wiced_hal_read_nvram( HELLO_SENSOR_VS_ID, sizeof(hello_sensor_hostinfo), (uint8_t*)&hello_sensor_hostinfo, &result );

    // If there are outstanding messages that we could not send out because
    // connection was not up and/or encrypted, send them now.  If we are sending
    // indications, we can send only one and need to wait for ack.
    while ( ( hello_sensor_state.num_to_write != 0 ) && !hello_sensor_state.flag_indication_sent )
    {
        hello_sensor_state.num_to_write--;
        hello_sensor_send_message();
    }

    // If configured to sleep after delivering data, start idle timer to sleep
    if ( !hello_sensor_state.flag_indication_sent )
    {
        /* Restart idle timer  */
        if( wiced_is_timer_in_use(&hello_sensor_idle_timer) )
        {
            wiced_stop_timer(&hello_sensor_idle_timer);
        }
        wiced_start_timer(&hello_sensor_idle_timer, HELLO_SENSOR_IDLE_TIMEOUT);
        WICED_BT_TRACE("idle timer re started \n");
    }
}

/*
 * Three Interrupt inputs (Buttons) can be handled here.
 * If the following value == 1, Button is pressed. Different than initial value.
 * If the following value == 0, Button is depressed. Same as initial value.
 * Button1 : ( value & 0x01 )
 * Button2 : (value&0x02)>>1
 * Button3 : (value&0x04)>>2
 */
void hello_sensor_interrupt_handler(void* user_data, uint8_t value )
{
    WICED_BT_TRACE( "hello_sensor_interrupt_handler \n");
    /* Increment the last byte of the hello sensor notify value */
    hello_sensor_gatts_increment_notify_value();

    /* Remember how many messages we need to send */
    hello_sensor_state.num_to_write++;

    /*
     * Connection up.
     * Send message if client registered to receive indication
     * or notification. After we send an indication wait for the ack
     * before we can send anything else
     */
    while ( ( hello_sensor_state.num_to_write != 0 ) && !hello_sensor_state.flag_indication_sent )
    {
        hello_sensor_state.num_to_write--;
        hello_sensor_send_message();
    }

    // if we sent all messages, start idle timer to sleep
    if ( !hello_sensor_state.flag_indication_sent )
    {
        /* Restart idle timer  */
        if( wiced_is_timer_in_use(&hello_sensor_idle_timer) )
        {
            wiced_stop_timer(&hello_sensor_idle_timer);
        }
        wiced_start_timer(&hello_sensor_idle_timer, HELLO_SENSOR_IDLE_TIMEOUT);
        WICED_BT_TRACE("idle timer re started \n");
    }
}

/*
 * hello_sensor bt/ble link management callback
 */
wiced_result_t hello_sensor_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                      result = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t    *p_status;
    wiced_bt_dev_ble_pairing_info_t     *p_info;
    wiced_bt_ble_advert_mode_t          *p_mode;
    wiced_bt_device_address_t           id_addr;
    uint8_t                             *p_keys;
    wiced_bt_device_address_t           null_bda={0,0,0,0,0,0};

    WICED_BT_TRACE("hello_sensor_management_cback: %x\n", event );

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            hello_sensor_application_init();
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req   = BTM_LE_AUTH_REQ_BOND | BTM_LE_AUTH_REQ_MITM;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_info =  &p_event_data->pairing_complete.pairing_complete_info.ble;
            WICED_BT_TRACE( "Pairing Complete: %d ", p_info->reason);
            hello_sensor_smp_bond_result( p_info->reason );
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /* save keys to NVRAM */
            p_keys = (uint8_t*)&p_event_data->paired_device_link_keys_update;
            /* do not overwrite if update is with invalid bd address */
            if (memcmp(p_keys, null_bda, BD_ADDR_LEN))
            {
                wiced_hal_write_nvram ( HELLO_SENSOR_PAIRED_KEYS_VS_ID, sizeof( wiced_bt_device_link_keys_t ), p_keys ,&result );
                WICED_BT_TRACE("keys save to NVRAM %B result: %d \n", p_keys, result);
            }
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* read keys from NVRAM */
            p_keys = (uint8_t *)&p_event_data->paired_device_link_keys_request;
            wiced_hal_read_nvram( HELLO_SENSOR_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), p_keys, &result );
            WICED_BT_TRACE("keys read from NVRAM %B result: %d \n", p_keys, result);
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* save keys to NVRAM */
            p_keys = (uint8_t*)&p_event_data->local_identity_keys_update;
            wiced_hal_write_nvram ( HELLO_SENSOR_LOCAL_KEYS_VS_ID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
            WICED_BT_TRACE("local keys save to NVRAM result: %d \n", result);
            break;

        case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /* read keys from NVRAM */
            p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
            wiced_hal_read_nvram( HELLO_SENSOR_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
            WICED_BT_TRACE("local keys read from NVRAM result: %d \n",  result);
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_status = &p_event_data->encryption_status;
            WICED_BT_TRACE( "Encryption Status Event: bd ( %B ) res %d", p_status->bd_addr, p_status->result);
            hello_sensor_encryption_changed( p_status->result, p_status->bd_addr );
        break;

        case BTM_SECURITY_REQUEST_EVT:
            wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            p_mode = &p_event_data->ble_advert_state_changed;
            WICED_BT_TRACE( "Advertisement State Change: %d\n", *p_mode);
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            WICED_BT_TRACE("connection update: status:%d conn interval: %d \n",
                p_event_data->ble_connection_param_update.status, p_event_data->ble_connection_param_update.conn_interval);
            break;

    default:
            break;
    }

    return result;
}


/*
 * Check if client has registered for notification/indication
 * and send message if appropriate
 */
void hello_sensor_send_message( void )
{
    WICED_BT_TRACE( "hello_sensor_send_message: CCC:%d\n", hello_sensor_hostinfo.characteristic_client_configuration );

    /* If client has not registered for indication or notification, no action */
    if ( hello_sensor_hostinfo.characteristic_client_configuration == 0 )
    {
        return;
    }
    else if ( hello_sensor_hostinfo.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION )
    {
        uint8_t *p_attr = (uint8_t*)hello_sensor_char_notify_value;

        wiced_bt_gatt_send_notification( hello_sensor_state.conn_id, HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL, sizeof(hello_sensor_char_notify_value), p_attr );
    }
    else
    {
        if ( !hello_sensor_state.flag_indication_sent )
        {
            uint8_t *p_attr = (uint8_t *)hello_sensor_char_notify_value;

            hello_sensor_state.flag_indication_sent = TRUE;

            wiced_bt_gatt_send_indication( hello_sensor_state.conn_id, HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL, sizeof(hello_sensor_char_notify_value), p_attr );
        }
    }
}

/*
 * Find attribute description by handle
 */
attribute_t * hello_sensor_get_attribute( uint16_t handle )
            {
    int i;
    for ( i = 0; i <  sizeof( gauAttributes ) / sizeof( gauAttributes[0] ); i++ )
    {
        if ( gauAttributes[i].handle == handle )
        {
            return ( &gauAttributes[i] );
        }
    }
    WICED_BT_TRACE( "attr not found:%x\n", handle );
    return NULL;
}


/*
 * Process Read request or command from peer device
 */
wiced_bt_gatt_status_t hello_sensor_gatts_req_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data )
{
    attribute_t *puAttribute;
    int          attr_len_to_copy;

    if ( ( puAttribute = hello_sensor_get_attribute(p_read_data->handle) ) == NULL)
    {
        WICED_BT_TRACE("read_hndlr attr not found hdl:%x\n", p_read_data->handle );
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Dummy battery value read increment */
    if( p_read_data->handle == HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL)
    {
        if ( hello_sensor_state.battery_level++ > 5)
        {
            hello_sensor_state.battery_level = 0;
        }
    }


    attr_len_to_copy = puAttribute->attr_len;

    WICED_BT_TRACE("read_hndlr conn_id:%d hdl:%x offset:%d len:%d\n", conn_id, p_read_data->handle, p_read_data->offset, attr_len_to_copy );

    if ( p_read_data->offset >= puAttribute->attr_len )
    {
        attr_len_to_copy = 0;
    }

    if ( attr_len_to_copy != 0 )
    {
        uint8_t *from;
        int      to_copy = attr_len_to_copy - p_read_data->offset;


        if ( to_copy > *p_read_data->p_val_len )
        {
            to_copy = *p_read_data->p_val_len;
        }

        from = ((uint8_t *)puAttribute->p_attr) + p_read_data->offset;
        *p_read_data->p_val_len = to_copy;

        memcpy( p_read_data->p_val, from, to_copy);
    }

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process write request or write command from peer device
 */
wiced_bt_gatt_status_t hello_sensor_gatts_req_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data )
{
    wiced_bt_gatt_status_t result    = WICED_BT_GATT_SUCCESS;
    uint8_t                *p_attr   = p_data->p_val;
    uint8_t                nv_update = WICED_FALSE;

    WICED_BT_TRACE("write_handler: conn_id:%d hdl:0x%x prep:%d offset:%d len:%d\n ", conn_id, p_data->handle, p_data->is_prep, p_data->offset, p_data->val_len );

    switch ( p_data->handle )
    {
        /* By writing into Characteristic Client Configuration descriptor
         * peer can enable or disable notification or indication */
        case HANDLE_HSENS_SERVICE_CHAR_CFG_DESC:
            if ( p_data->val_len != 2 )
            {
                return WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            hello_sensor_hostinfo.characteristic_client_configuration = p_attr[0] | ( p_attr[1] << 8 );
            nv_update = WICED_TRUE;
            break;

            default:
            result = WICED_BT_GATT_INVALID_HANDLE;
            break;
    }

    if ( nv_update )
    {
        wiced_result_t rc;
        int bytes_written = wiced_hal_write_nvram( HELLO_SENSOR_VS_ID, sizeof(hello_sensor_hostinfo), (uint8_t*)&hello_sensor_hostinfo, &rc );
        WICED_BT_TRACE("NVRAM write:%d rc:%d", bytes_written, rc);
    }

    return  result;
}

/*
 * Write Execute Procedure
 */
wiced_bt_gatt_status_t hello_sensor_gatts_req_write_exec_handler( uint16_t conn_id, wiced_bt_gatt_exec_flag_t exec_falg )
{
    WICED_BT_TRACE("write exec: flag:%d\n", exec_falg);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process MTU request from the peer
 */
wiced_bt_gatt_status_t hello_sensor_gatts_req_mtu_handler( uint16_t conn_id, uint16_t mtu)
{
    WICED_BT_TRACE("req_mtu: %d\n", mtu);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process indication confirm. If client wanted us to use indication instead of
 * notifications we have to wait for confirmation after every message sent.
 * For example if user pushed button twice very fast
 * we will send first message, then
 * wait for confirmation, then
 * send second message, then
 * wait for confirmation and
 * if configured start idle timer only after that.
 */
wiced_bt_gatt_status_t hello_sensor_gatts_req_conf_handler( uint16_t conn_id, uint16_t handle )
{
    WICED_BT_TRACE( "hello_sensor_indication_cfm, conn %d hdl %d\n", conn_id, handle );

    if ( !hello_sensor_state.flag_indication_sent )
    {
        WICED_BT_TRACE("Hello: Wrong Confirmation!");
        return WICED_BT_GATT_SUCCESS;
    }

    hello_sensor_state.flag_indication_sent = 0;

    /* We might need to send more indications */
    if ( hello_sensor_state.num_to_write )
    {
        hello_sensor_state.num_to_write--;
        hello_sensor_send_message();
    }
    /* if we sent all messages, start  idle timer to sleep */
    if ( !hello_sensor_state.flag_indication_sent )
    {
        /* Restart idle timer  */
        if( wiced_is_timer_in_use(&hello_sensor_idle_timer) )
        {
            wiced_stop_timer(&hello_sensor_idle_timer);
        }
        wiced_start_timer(&hello_sensor_idle_timer, HELLO_SENSOR_IDLE_TIMEOUT);
        WICED_BT_TRACE("idle timer re started \n");
    }

    return WICED_BT_GATT_SUCCESS;
}

/* This function is invoked when connection is established */
wiced_bt_gatt_status_t hello_sensor_gatts_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;

    WICED_BT_TRACE( "hello_sensor_conn_up %B id:%d\n:", p_status->bd_addr, p_status->conn_id);

    /* Update the connection handler.  Save address of the connected device. */
    hello_sensor_state.conn_id = p_status->conn_id;
    memcpy(hello_sensor_state.remote_addr, p_status->bd_addr, sizeof(BD_ADDR));

    /* Stop advertising */
    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );

    WICED_BT_TRACE( "Stopping Advertisements%d\n", result );

    /* Saving host info in NVRAM  */
    memcpy( hello_sensor_hostinfo.bdaddr, p_status->bd_addr, sizeof( BD_ADDR ) );
    hello_sensor_hostinfo.characteristic_client_configuration = 0;

    {
        uint8_t bytes_written = 0;
        bytes_written = wiced_hal_write_nvram( HELLO_SENSOR_VS_ID, sizeof(hello_sensor_hostinfo), (uint8_t*)&hello_sensor_hostinfo, &result );

        WICED_BT_TRACE("NVRAM write %d\n", bytes_written);
        UNUSED_VARIABLE(bytes_written);
    }

    wiced_bt_l2cap_update_ble_conn_params( p_status->bd_addr, CONNECTION_INTERVAL, CONNECTION_INTERVAL, 0, 512 );

    if ( !hello_sensor_is_device_bonded(p_status->bd_addr) )
    {
        result = wiced_bt_dev_sec_bond( p_status->bd_addr, p_status->addr_type, p_status->transport,0, NULL );
        WICED_BT_TRACE( "wiced_bt_dev_sec_bond %d \n", result );
    }
    else
    {
        wiced_bt_ble_sec_action_type_t  encryption_type = BTM_BLE_SEC_ENCRYPT;
        result = wiced_bt_dev_set_encryption (p_status->bd_addr, p_status->transport, &encryption_type);
        WICED_BT_TRACE( "wiced_bt_dev_set_encryption %d \n", result );
    }

    return WICED_BT_GATT_SUCCESS;
}

/*
 * This function is invoked when connection is lost
 */
wiced_bt_gatt_status_t hello_sensor_gatts_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;

    WICED_BT_TRACE( "connection_down %B conn_id:%d reason:%d\n", hello_sensor_state.remote_addr, p_status->conn_id, p_status->reason );

    /* Resetting the device info */
    memset( hello_sensor_state.remote_addr, 0, 6 );
    hello_sensor_state.conn_id = 0;

    /*
     * If we are configured to stay connected, disconnection was
     * caused by the peer, start low advertisements, so that peer
     * can reconnect when it wants to.
     */
    {
        result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
        WICED_BT_TRACE( "wiced_bt_start_advertisements %d\n", result );

        /*Start idle timer to sleep */
        if( wiced_is_timer_in_use(&hello_sensor_idle_timer) )
        {
            wiced_stop_timer(&hello_sensor_idle_timer);
        }
        wiced_start_timer(&hello_sensor_idle_timer, HELLO_SENSOR_IDLE_TIMEOUT);
        WICED_BT_TRACE("idle timer re started \n");
    }

    UNUSED_VARIABLE(result);

    return WICED_BT_SUCCESS;
}

/*
 * Connection up/down event
 */
wiced_bt_gatt_status_t hello_sensor_gatts_conn_status_cb( wiced_bt_gatt_connection_status_t *p_status )
{
    if ( p_status->connected )
    {
        return hello_sensor_gatts_connection_up( p_status );
    }

    return hello_sensor_gatts_connection_down( p_status );
}

/*
 * Process GATT request from the peer
 */
wiced_bt_gatt_status_t hello_sensor_gatts_req_cb( wiced_bt_gatt_attribute_request_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    WICED_BT_TRACE( "hello_sensor_gatts_req_cb. conn %d, type %d\n", p_data->conn_id, p_data->request_type );

    switch ( p_data->request_type )
    {
    case GATTS_REQ_TYPE_READ:
        result = hello_sensor_gatts_req_read_handler( p_data->conn_id, &(p_data->data.read_req) );
        break;

    case GATTS_REQ_TYPE_WRITE:
        result = hello_sensor_gatts_req_write_handler( p_data->conn_id, &(p_data->data.write_req) );
        break;

    case GATTS_REQ_TYPE_WRITE_EXEC:
        result = hello_sensor_gatts_req_write_exec_handler( p_data->conn_id, p_data->data.exec_write );
        break;

    case GATTS_REQ_TYPE_MTU:
        result = hello_sensor_gatts_req_mtu_handler( p_data->conn_id, p_data->data.mtu );
        break;

    case GATTS_REQ_TYPE_CONF:
        result = hello_sensor_gatts_req_conf_handler( p_data->conn_id, p_data->data.handle );
        break;

   default:
        break;
    }

    return result;
}

/*
 * Callback for various GATT events.  As this application performs only as a GATT server, some of the events are ommitted.
 */
wiced_bt_gatt_status_t hello_sensor_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = hello_sensor_gatts_conn_status_cb( &p_data->connection_status );
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = hello_sensor_gatts_req_cb( &p_data->attribute_request );
        break;

    default:
        break;
    }

    return result;
}

/*
 * Keep number of the button pushes in the last byte of the Hello message.
 * That will guarantee that if client reads it, it will have correct data.
 */
void hello_sensor_gatts_increment_notify_value( void )
{
    /* Getting the last byte */
    int last_byte = sizeof( hello_sensor_char_notify_value ) - 1 ;
    char c;

#ifdef ENABLE_SHUTDOWN_SLEEP
    c = hello_sensor_char_notify_incremented_value;
#else
    c = hello_sensor_char_notify_value[last_byte];
#endif

    c++;
    if ( (c < '0') || (c > '9') )
    {
        c = '0';
    }
    hello_sensor_char_notify_value[last_byte] = c;
#ifdef ENABLE_SHUTDOWN_SLEEP
    hello_sensor_char_notify_incremented_value = c;
#endif
}

static void hello_sensor_load_keys_for_address_resolution( void )
{
    wiced_bt_device_link_keys_t link_keys;
    wiced_result_t              result;
    uint8_t                     *p;

    memset( &link_keys, 0, sizeof(wiced_bt_device_link_keys_t));
    p = (uint8_t*)&link_keys;
    wiced_hal_read_nvram( HELLO_SENSOR_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), p, &result);

    if(result == WICED_BT_SUCCESS)
    {
        result = wiced_bt_dev_add_device_to_address_resolution_db ( &link_keys );
    }
    WICED_BT_TRACE("hello_sensor_load_keys_for_address_resolution %B result:%d \n", p, result );
}

/* Check for device entry exists in NVRAM list */
static wiced_bool_t hello_sensor_is_device_bonded( wiced_bt_device_address_t bd_address )
{
    wiced_bt_device_link_keys_t temp_keys;
    uint8_t                     bytes_read;
    wiced_result_t              result;

    bytes_read = wiced_hal_read_nvram( HELLO_SENSOR_PAIRED_KEYS_VS_ID, sizeof( temp_keys ), (uint8_t *)&temp_keys, &result );

    // if failed to read NVRAM, there is nothing saved at that location
    if ( bytes_read && (result == WICED_SUCCESS) )
    {
        if ( memcmp( temp_keys.bd_addr, bd_address, BD_ADDR_LEN ) == 0 )
        {
            return WICED_TRUE;
        }
    }
    return WICED_FALSE;
}

/*
 *  Pass protocol traces up through the UART
 */
static void hello_sensor_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    wiced_transport_send_hci_trace(NULL, type, length, p_data);
}
