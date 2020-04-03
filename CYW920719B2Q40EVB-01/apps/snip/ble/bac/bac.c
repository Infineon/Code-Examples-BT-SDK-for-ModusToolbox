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
* Battery Service Client
*
* The Battery Service Client application is designed to connect and access services of the Battery Server.
* Battery Client connects to a device advertising Battery Service UUID.

*
* Features demonstrated
*  - Registration with LE stack for various events.
*  - Read characteristic from a Battery Service Server device.
*  - Process the notifications received from the server.
*
* To demonstrate the app, work through the following steps.
* 1. Plug the WICED eval board into your computer
* 2. Build and download the application (to the WICED board)
* 3. On start of the application, push the button on the tag board and release with in 2 seconds,so that
*    Battery Client App scans and connects to the Battery Service Server, which would have
*    UUID_SERVICE_BATTERY in it's advertisements.
*    Note:- If no Battery Service Server device is found nearby for 90secs, then scan stops automatically.
*    To restart the scan, push the button on the tag board and release within 2 secs.
* 4. Upon successful Connection, the Battery Client App would discover all the characteristics/descriptors
*    of the server device.
* 5. Once the connection is established with the BLE peripheral (Battery Service found in the Peripheral),
*    the application can enable/disable for notifications, to receive the change in the battery level.
*    To enable/disable notifications from the server, push the button on the tag board and release after 5 secs.
* 6. To read the battery level of the server, push the button on the tag board and release between 2-4 secs.
*/

#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_gatt_util.h"
#include "wiced_bt_cfg.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"
#include "wiced_transport.h"
#include "wiced_platform.h"
#include "wiced_bt_bac.h"
#include "wiced_timer.h"
#include "wiced_hal_mia.h"
#include "wiced_hal_puart.h"
#include "string.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_gatt_util.h"

#ifdef  WICED_BT_TRACE_ENABLE
#include "wiced_bt_trace.h"
#endif

/******************************************************************************
 *                             External Definitions
 ******************************************************************************/
extern const wiced_bt_cfg_settings_t wiced_app_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_app_cfg_buf_pools[];

/******************************************************************************
 *                                Constants
 ******************************************************************************/

#define BAC_TRACE_DBG(format, ...)  WICED_BT_TRACE("[%s] " format, __FUNCTION__, ##__VA_ARGS__)
#define BAC_TRACE_ERR(format, ...)  WICED_BT_TRACE("Err: [%s] " format, __FUNCTION__, ##__VA_ARGS__)

/* App Timer Timeout in seconds  */
#define BATTC_APP_TIMEOUT_IN_SECONDS           1

#if !defined(CYW20819A1)
#define BUTTON_PRESSED                         WICED_BUTTON_PRESSED_VALUE
#endif

#ifdef CYW20706A2
#define APP_BUTTON                  WICED_GPIO_BUTTON
#define APP_BUTTON_SETTINGS         (WICED_GPIO_BUTTON_SETTINGS( GPIO_EN_INT_BOTH_EDGE ))
#define APP_BUTTON_DEFAULT_STATE    WICED_GPIO_BUTTON_DEFAULT_STATE
#define APP_LED                     WICED_PLATFORM_LED_1
#endif

#ifdef CYW20735B0
#define APP_BUTTON                  WICED_GPIO_PIN_BUTTON
#define APP_LED                     WICED_GPIO_PIN_LED1
#define APP_BUTTON_SETTINGS         ( GPIO_INPUT_ENABLE | GPIO_PULL_DOWN | GPIO_EN_INT_BOTH_EDGE )
#define APP_BUTTON_DEFAULT_STATE    GPIO_PIN_OUTPUT_LOW
#endif

#define BATTERY_CLIENT_LOCAL_KEYS_VS_ID         ( WICED_NVRAM_VSID_START )
#define BATTERY_CLIENT_PAIRED_KEYS_START_VS_ID  ( WICED_NVRAM_VSID_START + 1 )

/******************************************************************************
 *                                Structures
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
} battery_service_client_peer_info_t;

/* Battery Service client application info */
typedef struct
{
#define BAC_DISCOVERY_STATE_SERVICE     0
#define BAC_DISCOVERY_STATE_CHAR        1
    uint8_t discovery_state;

    // Current value of the client configuration descriptor for characteristic 'Report'
    uint16_t                    bac_s_handle;
    uint16_t                    bac_e_handle;
}battery_service_client_app_t;

/*****************************************************************************
 *                           Function Prototypes
 *****************************************************************************/
static wiced_result_t         battery_client_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static void                   battery_client_app_init();
static wiced_bt_gatt_status_t battery_client_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data );
static void                   battery_client_load_keys_to_addr_resolution_db();
static void                   battery_client_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data );
static wiced_bool_t           battery_client_save_link_keys( wiced_bt_device_link_keys_t *p_keys );
static wiced_bool_t           battery_client_read_link_keys(wiced_bt_device_link_keys_t *p_keys);
static wiced_bt_gatt_status_t battery_client_connection_up( wiced_bt_gatt_connection_status_t *p_conn_status );
static wiced_bt_gatt_status_t battery_client_connection_down( wiced_bt_gatt_connection_status_t *p_conn_status );
static wiced_bt_gatt_status_t battery_client_gatt_operation_complete( wiced_bt_gatt_operation_complete_t *p_data );
static void                   battery_client_process_read_rsp(wiced_bt_gatt_operation_complete_t *p_data);
static wiced_bt_gatt_status_t battery_client_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_data);
static wiced_bt_gatt_status_t battery_client_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data);
static void                   battery_client_interrupt_handler( void *user_data, uint8_t value );
static void                   battery_client_app_timer( uint32_t arg );
static wiced_bool_t           battery_client_is_device_bonded( wiced_bt_device_address_t bd_address );
static void                   battery_client_callback(wiced_bt_bac_event_t event, wiced_bt_bac_event_data_t *p_data);

/******************************************************************************
 *                           Variables Definitions
 ******************************************************************************/
battery_service_client_peer_info_t  battery_client_app_data;
battery_service_client_app_t battery_client_app_state;
uint32_t app_timer_count = 0;
wiced_bool_t is_enabled_notification = WICED_FALSE;
wiced_timer_t app_timer;

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

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
#ifndef CYW20735B0
APPLICATION_START( )
#else
void application_start( void )
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
    // be called with wiced_transport_cfg_t.wiced_trbasort_data_handler_t callback present
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

    WICED_BT_TRACE( "Battery Client Application Start\n" );

    wiced_bt_bac_init(battery_client_callback);

    memset(&battery_client_app_data, 0, sizeof(battery_client_app_data));
    memset(&battery_client_app_state, 0, sizeof(battery_client_app_state));

    // Register call back and configuration with stack
    wiced_bt_stack_init( battery_client_management_cback ,
                    &wiced_app_cfg_settings, wiced_app_cfg_buf_pools );
}

void battery_client_set_input_interrupt(void)
{
#if ( defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20735B1) || defined(CYW20819A1) || defined(CYW20721B2) || defined(CYW20719B2) )
    /* Configure buttons available on the platform */
    wiced_platform_register_button_callback( WICED_PLATFORM_BUTTON_1, battery_client_interrupt_handler, NULL, WICED_PLATFORM_BUTTON_BOTH_EDGE);
#else
    wiced_hal_gpio_register_pin_for_interrupt( APP_BUTTON, battery_client_interrupt_handler, NULL );
    wiced_hal_gpio_configure_pin( APP_BUTTON, APP_BUTTON_SETTINGS, APP_BUTTON_DEFAULT_STATE );
#endif

    /* Starting the app timer */
    if ( wiced_init_timer(&app_timer, battery_client_app_timer, 0, WICED_SECONDS_PERIODIC_TIMER) == WICED_SUCCESS)
    {
        if ( wiced_start_timer(&app_timer,BATTC_APP_TIMEOUT_IN_SECONDS) != WICED_SUCCESS)
        {
            WICED_BT_TRACE("Start timer FAILED!!");
        }
    }
}

/* The function invoked on timeout of app seconds timer. */
void battery_client_app_timer( uint32_t arg )
{
    app_timer_count++;
    if ((app_timer_count % 10) == 0)
        WICED_BT_TRACE("%d \n", app_timer_count);
}

void battery_client_interrupt_handler( void *user_data, uint8_t value )
{
    static uint32_t button_pushed_time = 0;
    wiced_result_t result;
    wiced_bt_gatt_status_t status;
#if ( defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20735B1) || defined(CYW20819A1) || defined(CYW20721B2) || defined(CYW20719B2) )
    if ( wiced_hal_gpio_get_pin_input_status(WICED_GPIO_PIN_BUTTON_1) == wiced_platform_get_button_pressed_value(WICED_PLATFORM_BUTTON_1) )
#else
    if ( wiced_hal_gpio_get_pin_input_status(APP_BUTTON) == BUTTON_PRESSED )
#endif
    {
        button_pushed_time = app_timer_count;
    }
    else if ( button_pushed_time != 0 )
    {
        if ( app_timer_count - button_pushed_time > 5 )
        {
             if(!is_enabled_notification)
             {
                status = wiced_bt_bac_enable_notification( battery_client_app_data.conn_id );
                WICED_BT_TRACE("Enabling Notification: %x \n",status);
             }
             else
             {
                status = wiced_bt_bac_disable_notification( battery_client_app_data.conn_id );
                WICED_BT_TRACE("Disabling Notification: %x \n",status);
             }
        }
        else if( (app_timer_count - button_pushed_time > 2) && (app_timer_count - button_pushed_time < 4) )
        {
            wiced_bt_bac_read_battery_level( battery_client_app_data.conn_id );
        }
        else if( app_timer_count - button_pushed_time < 2 )
        {
            /*start scan if not connected and no scan in progress*/
            if (( battery_client_app_data.conn_id == 0 ) && (wiced_bt_ble_get_current_scan_state() == BTM_BLE_SCAN_TYPE_NONE))
            {
                result = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, battery_client_scan_result_cback );
                WICED_BT_TRACE("wiced_bt_ble_scan: %d \n", result);
            }
        }
    }
}

/*
 *  Pass protocol traces up through the UART
 */
#ifdef ENABLE_HCI_TRACE
void battery_client_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    //send the trace
    wiced_transport_send_hci_trace( NULL, type, length, p_data  );
}
#endif

static void battery_client_app_init()
{
    wiced_bt_gatt_status_t gatt_status;

#if !defined(CYW20735B1) && !defined(CYW20819A1) && !defined(CYW20719B2) && !defined(CYW20721B2)
    /* Initialize wiced app */
    wiced_bt_app_init();
#endif

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(battery_client_gatts_callback);

    WICED_BT_TRACE("wiced_bt_gatt_register: %d\n", gatt_status);

#ifdef ENABLE_HCI_TRACE
    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace( battery_client_hci_trace_cback );
#endif

    /* Load the address resolution DB with the keys stored in the NVRAM */
    battery_client_load_keys_to_addr_resolution_db();
}

static wiced_result_t battery_client_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                    result = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t *p_status;
    wiced_bt_dev_ble_pairing_info_t  *p_info;
    wiced_bt_ble_advert_mode_t       *p_mode;
    wiced_bt_device_link_keys_t       paired_device_link_keys_request;
    uint8_t                           bytes_written, bytes_read;
    uint8_t                          *p_keys;

    WICED_BT_TRACE("battery_client_management_cback:%d\n", event);

    switch(event)
    {
    /* Bluetooth stack enabled */
    case BTM_ENABLED_EVT:
        battery_client_app_init();
        battery_client_set_input_interrupt();
        break;

    case BTM_DISABLED_EVT:
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_SC_BOND;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        WICED_BT_TRACE("Pairing Complete: %d\n", p_event_data->pairing_complete.pairing_complete_info.ble.reason);
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        WICED_BT_TRACE("Encryption Status Event: bd (%B) res %d\n", p_event_data->encryption_status.bd_addr, p_event_data->encryption_status.result);
        if (p_event_data->encryption_status.result == WICED_BT_SUCCESS)
        {
            wiced_bt_bac_enable_notification( battery_client_app_data.conn_id );
        }
        break;

    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        battery_client_save_link_keys(&p_event_data->paired_device_link_keys_update);
        break;

     case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        if (battery_client_read_link_keys(&p_event_data->paired_device_link_keys_request))
        {
            result = WICED_BT_SUCCESS;
            WICED_BT_TRACE("Key retrieval success\n");
        }
        else
        {
            result = WICED_BT_ERROR;
            WICED_BT_TRACE("Key retrieval failure\n");
        }
        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        /* save keys to NVRAM */
        p_keys = (uint8_t*)&p_event_data->local_identity_keys_update;
        wiced_hal_write_nvram ( BATTERY_CLIENT_LOCAL_KEYS_VS_ID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
        WICED_BT_TRACE("local keys save to NVRAM result: %d \n", result);
        break;


    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /* read keys from NVRAM */
        p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
        wiced_hal_read_nvram( BATTERY_CLIENT_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
        WICED_BT_TRACE("local keys read from NVRAM result: %d \n",  result);
        break;

    case BTM_BLE_SCAN_STATE_CHANGED_EVT:
        WICED_BT_TRACE( "Scan State Change: %d\n", p_event_data->ble_scan_state_changed );
        break;

    default:
        break;
    }
    return result;
}

static wiced_bt_gatt_status_t battery_client_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch(event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            if (p_data->connection_status.connected)
            {
                battery_client_connection_up(&p_data->connection_status);
            }
            else
            {
                battery_client_connection_down(&p_data->connection_status);
            }
            break;

        case GATT_DISCOVERY_RESULT_EVT:
            WICED_BT_TRACE("discovery result event\n");
            result = battery_client_gatt_discovery_result(&p_data->discovery_result);
            break;

        case GATT_DISCOVERY_CPLT_EVT:
            WICED_BT_TRACE("discovery complete event\n");
            result = battery_client_gatt_discovery_complete(&p_data->discovery_complete);
            break;

        case GATT_OPERATION_CPLT_EVT:
            result = battery_client_gatt_operation_complete(&p_data->operation_complete);
            break;

        default:
            break;
    }

    return result;
}

/*
 * Pass read response to appropriate client based on the attribute handle
 */
static void battery_client_process_read_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
    WICED_BT_TRACE("read response handle:%04x\n", p_data->response_data.handle);

    // Verify that read response is for our service
    if ((p_data->response_data.handle >= battery_client_app_state.bac_s_handle) &&
        (p_data->response_data.handle <= battery_client_app_state.bac_e_handle))
    {
        wiced_bt_bac_read_rsp(p_data);
    }
}

/*
 * Pass notification to appropriate client based on the attribute handle
 */
static void battery_client_notification_handler(wiced_bt_gatt_operation_complete_t *p_data)
{
    WICED_BT_TRACE("notification handle:%04x\n", p_data->response_data.att_value.handle);

    if ((p_data->response_data.handle >= battery_client_app_state.bac_s_handle) &&
        (p_data->response_data.handle <= battery_client_app_state.bac_e_handle))
    {
        wiced_bt_bac_process_notification(p_data);
    }
}

/*
 * GATT operation started by the client has been completed
 */
static wiced_bt_gatt_status_t battery_client_gatt_operation_complete( wiced_bt_gatt_operation_complete_t *p_data )
{
    wiced_result_t              status;
    wiced_bt_ble_sec_action_type_t  encryption_type = BTM_BLE_SEC_ENCRYPT;

    WICED_BT_TRACE("battery_client_gatt_operation_complete conn %d op %d st %d\n", p_data->conn_id, p_data->op, p_data->status );

    switch ( p_data->op )
    {
        case GATTC_OPTYPE_READ:
            WICED_BT_TRACE( "read_rsp status:%d\n", p_data->status );
            battery_client_process_read_rsp(p_data);
            break;

        case GATTC_OPTYPE_WRITE:
            WICED_BT_TRACE( "write_rsp status:%d desc_handle:%x \n", p_data->status,p_data->response_data.handle );
            if(p_data->status == WICED_BT_GATT_SUCCESS)
            {
                if (!is_enabled_notification)
                {
                    is_enabled_notification = WICED_TRUE;
                }
                else
                {
                    is_enabled_notification = WICED_FALSE;
                }
            }
            break;

        case GATTC_OPTYPE_CONFIG:
            WICED_BT_TRACE( "peer mtu:%d\n", p_data->response_data.mtu );
            break;

        case GATTC_OPTYPE_NOTIFICATION:
            WICED_BT_TRACE( "notification status:%d\n", p_data->status );
            battery_client_notification_handler( p_data );
            break;

        case GATTC_OPTYPE_INDICATION:
            break;
    }

    /* server puts authentication requirement. Encrypt the link */
    if ( p_data->status == WICED_BT_GATT_INSUF_AUTHENTICATION )
    {
        BAC_TRACE_DBG("Insufficient Authentication\n");
        if ( battery_client_is_device_bonded(battery_client_app_data.peer_addr) )
        {
            BAC_TRACE_DBG("Authentified. Start Encryption\n");
            status = wiced_bt_dev_set_encryption( battery_client_app_data.peer_addr,
                    battery_client_app_data.transport, &encryption_type );
            WICED_BT_TRACE( "wiced_bt_dev_set_encryption %d \n", status );
        }
        else
        {
            BAC_TRACE_DBG("Start Authentification/Pairing\n");
            status = wiced_bt_dev_sec_bond( battery_client_app_data.peer_addr,
                    battery_client_app_data.addr_type, battery_client_app_data.transport,0, NULL );
            WICED_BT_TRACE( "wiced_bt_dev_sec_bond %d \n", status );
        }
    }

    return WICED_BT_GATT_SUCCESS;
}

void battery_client_add_peer_info( uint16_t conn_id, uint8_t* p_bd_addr, uint8_t role , uint8_t transport, uint8_t address_type )
{
    battery_client_app_data.addr_type = address_type;
    battery_client_app_data.conn_id = conn_id;
    memcpy(battery_client_app_data.peer_addr,p_bd_addr,BD_ADDR_LEN);
    battery_client_app_data.role = role;
    battery_client_app_data.transport = transport;
}

static wiced_bt_gatt_status_t battery_client_connection_up( wiced_bt_gatt_connection_status_t *p_conn_status )
{
    uint8_t dev_role;
    wiced_bt_dev_status_t status ;
    wiced_bt_ble_sec_action_type_t  encryption_type = BTM_BLE_SEC_ENCRYPT;

    wiced_bt_dev_get_role( p_conn_status->bd_addr, &dev_role, BT_TRANSPORT_LE );

    // Adding the peer info
    battery_client_add_peer_info( p_conn_status->conn_id, p_conn_status->bd_addr, dev_role , p_conn_status->transport, p_conn_status->addr_type );

    WICED_BT_TRACE( "battery client_connection_up Conn Id:%d Addr:<%B> role:%d\n ",
       p_conn_status->conn_id, p_conn_status->bd_addr, dev_role );

    // need to notify BAS library that the connection is up
    wiced_bt_bac_client_connection_up( p_conn_status );

        /* Initialize WICED BT bac library Start discovery */
    battery_client_app_state.discovery_state = BAC_DISCOVERY_STATE_SERVICE;
    battery_client_app_state.bac_s_handle = 0;
    battery_client_app_state.bac_e_handle = 0;

   // perform primary service search
    status = wiced_bt_util_send_gatt_discover( p_conn_status->conn_id, GATT_DISCOVER_SERVICES_ALL, UUID_ATTRIBUTE_PRIMARY_SERVICE, 1, 0xffff);
    WICED_BT_TRACE("start discover status:%d\n", status);

    return WICED_BT_GATT_SUCCESS;
}

static wiced_bt_gatt_status_t battery_client_connection_down( wiced_bt_gatt_connection_status_t *p_conn_status )
{
    WICED_BT_TRACE("battery client connection down\n");
    battery_client_app_data.conn_id = 0;

    battery_client_app_state.discovery_state = BAC_DISCOVERY_STATE_SERVICE;
    battery_client_app_state.bac_s_handle = 0;
    battery_client_app_state.bac_e_handle = 0;

    wiced_bt_bac_client_connection_down( p_conn_status );

    return WICED_BT_GATT_SUCCESS;
}

/*
 * This function handles the scan results and attempt to connect to Battery Service Server.
 */
static void battery_client_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
{
    wiced_result_t          status;
    wiced_bool_t            ret_status;
    uint8_t                 length;
    uint8_t *               p_data;
    uint16_t                service_uuid16=0;

    if ( p_scan_result )
    {
        // Search for SERVICE_UUID_16 element in the Advertisement data received.Check for both
        // complete and partial list
        p_data = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE, &length );
        if ( p_data == NULL )
        {
            p_data = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_16SRV_PARTIAL, &length );
            if (p_data == NULL)
                return;     // No UUID_16 element
        }

        while (length >= LEN_UUID_16)
        {
            STREAM_TO_UINT16(service_uuid16, p_data);
            if (service_uuid16 == UUID_SERVICE_BATTERY)
            {
                // UUID16 Battery Service found
                break;
            }
            length -= LEN_UUID_16;
        }

        if (service_uuid16 != UUID_SERVICE_BATTERY)
        {
            // UUID16 Battery Service not found. Ignore device
            return;
        }

        WICED_BT_TRACE("Battery Server Device found: %B addr_type:%d\n",
                p_scan_result->remote_bd_addr, p_scan_result->ble_addr_type);

        /* Stop the scan since the desired device is found */
        status = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, battery_client_scan_result_cback );
        WICED_BT_TRACE( "scan off status %d\n", status );

        /* Initiate the connection */
        ret_status = wiced_bt_gatt_le_connect( p_scan_result->remote_bd_addr, p_scan_result->ble_addr_type, BLE_CONN_MODE_HIGH_DUTY, TRUE );
        WICED_BT_TRACE( "wiced_bt_gatt_connect status %d\n", ret_status );
    }
    else
    {
        WICED_BT_TRACE( "Scan completed:\n" );
    }
}

static wiced_bt_gatt_status_t battery_client_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_data)
{
    WICED_BT_TRACE("[%s] conn %d type %d state 0x%02x\n", __FUNCTION__, p_data->conn_id, p_data->discovery_type, battery_client_app_state.discovery_state);

    switch (battery_client_app_state.discovery_state)
    {
        case BAC_DISCOVERY_STATE_CHAR:
            wiced_bt_bac_discovery_result(p_data);
            break;

        default:
            if (p_data->discovery_type  == GATT_DISCOVER_SERVICES_ALL)
            {
                if (p_data->discovery_data.group_value.service_type.len == LEN_UUID_16 )
                {
                    WICED_BT_TRACE("uuid:%04x start_handle:%04x end_handle:%04x\n",
                            p_data->discovery_data.group_value.service_type.uu.uuid16,
                            p_data->discovery_data.group_value.s_handle,
                            p_data->discovery_data.group_value.e_handle);
                    if( p_data->discovery_data.group_value.service_type.uu.uuid16 == UUID_SERVICE_BATTERY )
                    {
                        WICED_BT_TRACE("Battery Service found s:%04x e:%04x\n",
                                p_data->discovery_data.group_value.s_handle,
                                p_data->discovery_data.group_value.e_handle);
                        battery_client_app_state.bac_s_handle = p_data->discovery_data.group_value.s_handle;
                        battery_client_app_state.bac_e_handle = p_data->discovery_data.group_value.e_handle;
                    }
                }
            }
            else
            {
                WICED_BT_TRACE("!!!! invalid op:%d\n", p_data->discovery_type);
            }
    }
    return WICED_BT_GATT_SUCCESS;
}

static wiced_bt_gatt_status_t battery_client_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data)
{
    wiced_result_t result;

    WICED_BT_TRACE("[%s] conn %d type %d state %d\n", __FUNCTION__, p_data->conn_id, p_data->disc_type, battery_client_app_state.discovery_state);

    switch (battery_client_app_state.discovery_state)
    {
    case BAC_DISCOVERY_STATE_CHAR:
        wiced_bt_bac_client_discovery_complete(p_data);
        break;

    default:
        if (p_data->disc_type == GATT_DISCOVER_SERVICES_ALL)
        {
            WICED_BT_TRACE("bac:%04x-%04x\n", battery_client_app_state.bac_s_handle, battery_client_app_state.bac_e_handle);

            /* If bac Service found tell WICED BT bac library to start its discovery */
            if ((battery_client_app_state.bac_s_handle != 0) && (battery_client_app_state.bac_e_handle != 0))
            {
                battery_client_app_state.discovery_state = BAC_DISCOVERY_STATE_CHAR;
                if (wiced_bt_bac_discover(battery_client_app_data.conn_id, battery_client_app_state.bac_s_handle, battery_client_app_state.bac_e_handle))
                    break;
            }
        }
        else
        {
            WICED_BT_TRACE("!!!! invalid op:%d\n", p_data->disc_type);
        }
    }
    return WICED_BT_GATT_SUCCESS;
}

static void battery_client_callback(wiced_bt_bac_event_t event, wiced_bt_bac_event_data_t *p_data)
{
    wiced_bt_gatt_status_t status;

    switch (event)
    {
    case WICED_BT_BAC_EVENT_DISCOVERY_COMPLETE:
        BAC_TRACE_DBG("Discovery Complete conn_id:%d status:%d notif:%d\n",
                p_data->discovery.conn_id, p_data->discovery.status,
                p_data->discovery.notification_supported);

        /* If Battery Service successfully discovered */
        if (p_data->discovery.status == WICED_BT_GATT_SUCCESS)
        {
            /* if the Battery Service supports (optional) Notification, enable Notifications. */
            if (p_data->discovery.notification_supported)
            {
                /* Enable Battery Level Notification */
                BAC_TRACE_DBG("Enable Battery Level Notification\n");
                status = wiced_bt_bac_enable_notification(p_data->discovery.conn_id);
                if (status != WICED_BT_GATT_SUCCESS)
                    BAC_TRACE_ERR("wiced_bt_bac_enable_notification failed status:%d\n", status);
            }
            else
            {
                /* Notification not supported by Server, just Read the Current Battery Level */
                BAC_TRACE_DBG("Read Battery Level\n");
                status = wiced_bt_bac_read_battery_level(p_data->discovery.conn_id);
                if (status != WICED_BT_GATT_SUCCESS)
                    BAC_TRACE_ERR("wiced_bt_bac_read_battery_level failed status:%d\n", status);
            }
        }
        break;

    case WICED_BT_BAC_EVENT_BATTERY_LEVEL_RSP:
        BAC_TRACE_DBG("Battery Level Rsp conn_id:%d status:%d level:%d\n",
                p_data->battery_level_rsp.conn_id, p_data->battery_level_rsp.status,
                p_data->battery_level_rsp.battery_level);
        break;

    case WICED_BT_BAC_EVENT_BATTERY_LEVEL_NOTIFICATION:
        BAC_TRACE_DBG("Battery Level Notif conn_id:%d level:%d\n",
                p_data->battery_level_notification.conn_id,
                p_data->battery_level_notification.battery_level);
        break;

    default:
        WICED_BT_TRACE("Unknown BAC Event:%d\n", event);
        break;
    }
}

static void battery_client_load_keys_to_addr_resolution_db()
{
    uint8_t                     bytes_read;
    wiced_result_t              result;
    wiced_bt_device_link_keys_t keys;
    uint16_t                    i;

    for ( i = BATTERY_CLIENT_PAIRED_KEYS_START_VS_ID; i < WICED_NVRAM_VSID_END; i++ )
    {
        bytes_read = wiced_hal_read_nvram(i, sizeof(keys), (uint8_t *)&keys, &result);

        WICED_BT_TRACE("[%s] read status %d bytes read %d \n", __FUNCTION__, result, bytes_read);

        // if failed to read NVRAM, there is nothing saved at that location
        if ((result == WICED_SUCCESS) && (bytes_read == sizeof(wiced_bt_device_link_keys_t)))
        {
#ifdef CYW20706A2
            result = wiced_bt_dev_add_device_to_address_resolution_db(&keys, keys.key_data.ble_addr_type);
#else
            result = wiced_bt_dev_add_device_to_address_resolution_db(&keys);
#endif
        }
        else
        {
            break;
        }
    }
}

static wiced_bool_t battery_client_save_link_keys( wiced_bt_device_link_keys_t *p_keys )
{
    uint8_t                     bytes_written, bytes_read;
    wiced_bt_device_link_keys_t temp_keys;
    uint16_t                    id = 0;
    uint32_t i;
    wiced_result_t              result;

    // search through all available NVRAM IDs.
    for ( i = BATTERY_CLIENT_PAIRED_KEYS_START_VS_ID; i < WICED_NVRAM_VSID_END; i++ )
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
    return WICED_TRUE;
}

static wiced_bool_t battery_client_read_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    wiced_bt_device_link_keys_t temp_keys;
    uint8_t                     bytes_read;
    uint16_t                    i;
    wiced_result_t              result;

    // search through all available NVRAM IDs.
    for ( i = BATTERY_CLIENT_PAIRED_KEYS_START_VS_ID; i < WICED_NVRAM_VSID_END; i++ )
    {
        bytes_read = wiced_hal_read_nvram( i, sizeof( temp_keys ), (uint8_t *)&temp_keys, &result );

        WICED_BT_TRACE("[%s] read status %d bytes read %d \n", __FUNCTION__, result, bytes_read);

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
    return WICED_FALSE;
}

/* Check for device entry exists in NVRAM list */
static wiced_bool_t battery_client_is_device_bonded( wiced_bt_device_address_t bd_address )
{
    wiced_bt_device_link_keys_t temp_keys;
    uint8_t                     bytes_read;
    uint16_t                    i;
    wiced_result_t              result;

    // search through all available NVRAM IDs.
    for ( i = BATTERY_CLIENT_PAIRED_KEYS_START_VS_ID; i < WICED_NVRAM_VSID_END; i++ )
    {
        bytes_read = wiced_hal_read_nvram( i, sizeof( temp_keys ), (uint8_t *)&temp_keys, &result );

        WICED_BT_TRACE("[%s] read status %d bytes read %d \n", __FUNCTION__, result, bytes_read);

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
    return WICED_FALSE;
}
