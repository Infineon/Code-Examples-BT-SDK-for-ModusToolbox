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
* Heart Rate Client (HRC) snippet application
*
* The HRC snippet application shows how to initialize and use WICED BT Heart Rate
* Client library. This snippet implements GAP central role
*
* On initialization, the application starts scanning and connects to a nearby peripheral that advertises
* Heart Rate Service UUID. After connecting successfully, application calls HRC library
* to start GATT discovery for HRS characteristics and descriptors. The library then issues
* callbacks to notify status of the discovery operation. On successful discovery,
* application configures HRS to send heart rate notifications. User can use application button
* to un-resgister and re-register to notifications and to reset energy expended value.
* Application tracks the duration of button pressed using a timer and performs the actions as explained below.
*
* Features demonstrated
*  - Initialize and use WICED BT HRC library
*
* To demonstrate the app, work through the following steps.
* 1. Plug the WICED eval board into your computer
* 2. Build and download the application (to the WICED board)
* 3. On start of the application, push the button on the tag board and release with in 2 seconds, so that
*    Heart Rate Client scans and connects to the Heart Rate Server which would have
*    UUID_SERVICE_HEART_RATE in it's advertisements.
*    Note:- If no Heart Rate server device is found nearby for 90secs, then scan stops automatically.
*    To restart the scan, push the button on the tag board and release within 2 secs.
* 4. Once connection established with BLE peripheral and heart rate service found in BLE peripheral,
*    automatically application receive heart rate notification from server on every 1 minute.
* 5. To start or stop notifications, user should press the button and release between 2 to 4 seconds.
* 6. During notifications if user finds energy expended value reched maximum value, i.e. 0xffff and want to reset
*    the value, user should press the application button and release after 5 seconds.
*/

#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_gatt_util.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_uuid.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif
#include "wiced_timer.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"
#include "wiced_result.h"
#include "wiced_platform.h"
#include "wiced_transport.h"
#include "wiced_bt_hrp.h"
#include "wiced_bt_hrc.h"
#include "wiced_hal_puart.h"
#include "string.h"
#include "wiced_bt_stack.h"


/******************************************************
 *                      Constants
 ******************************************************/

extern const wiced_bt_cfg_settings_t wiced_app_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_app_cfg_buf_pools[];

#ifdef CYW20706A2
#define APP_BUTTON                  WICED_GPIO_BUTTON
#define APP_BUTTON_SETTINGS         (WICED_GPIO_BUTTON_SETTINGS( GPIO_EN_INT_BOTH_EDGE ))
#define APP_BUTTON_DEFAULT_STATE    WICED_GPIO_BUTTON_DEFAULT_STATE
#define APP_LED                     WICED_PLATFORM_LED_1
#endif

#if ( defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20719B2) || defined(CYW20721B2) || defined(CYW20735B1) || defined(CYW20819A1) )
#define APP_LED                     WICED_GPIO_PIN_LED_1
#endif

#ifdef CYW20735B0
#define APP_BUTTON                  WICED_GPIO_PIN_BUTTON
#define APP_LED                     WICED_GPIO_PIN_LED1
#define APP_BUTTON_SETTINGS         ( GPIO_INPUT_ENABLE | GPIO_PULL_DOWN | GPIO_EN_INT_BOTH_EDGE )
#define APP_BUTTON_DEFAULT_STATE    GPIO_PIN_OUTPUT_LOW
#endif

#define HRC_LOCAL_KEYS_VS_ID     (WICED_NVRAM_VSID_START)
#define HRC_PAIRED_KEYS_VS_ID    (WICED_NVRAM_VSID_START + 1)

/******************************************************
 *                     Structures
 ******************************************************/

/******************************************************
 *               Function Prototypes
 ******************************************************/
static wiced_result_t         hrc_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static void                   hrc_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data );
static void                   hrc_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status);
static void                   hrc_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status);
static void                   hrc_process_pairing_complete(uint8_t result);
static wiced_bt_gatt_status_t hrc_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
static wiced_bt_gatt_status_t hrc_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_data);
static wiced_bt_gatt_status_t hrc_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data);
static wiced_bt_gatt_status_t hrc_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data);

static void                   hrc_callback(wiced_bt_hrc_event_t event, wiced_bt_hrc_event_data_t *p_data);
static void                   hrc_interrupt_handler(void* user_data, uint8_t value );
static void                   hrc_timeout( uint32_t count );
static void                   hrc_load_keys_to_addr_resolution_db(void);
static wiced_bool_t           hrc_save_link_keys(wiced_bt_device_link_keys_t *p_keys);
static wiced_bool_t           hrc_read_link_keys(wiced_bt_device_link_keys_t *p_keys);
static void                   hrc_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);

/******************************************************
 *               Variables Definitions
 ******************************************************/

typedef struct
{
    uint16_t                    conn_id;

#define HRC_DISCOVERY_STATE_SERVICE     0
#define HRC_DISCOVERY_STATE_HRS         1

    uint8_t                     discovery_state;

    uint8_t                     started;

    uint32_t                    timeout;

    uint16_t                    hear_rate_service_s_handle; // Heart Rate Service start handle
    uint16_t                    hear_rate_service_e_handle; // Heart Rate Service end handle

    BD_ADDR                     remote_addr;    //address of currently connected peer
    wiced_bt_ble_address_type_t addr_type;
} hrc_app_cb_t;

hrc_app_cb_t hrc_app_cb;

const wiced_transport_cfg_t transport_cfg =
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

wiced_transport_buffer_pool_t*  host_trans_pool;
wiced_timer_t app_timer;

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 * Entry point to the application. Set device configuration and start BT
 * stack initialization.  The actual application initialization will happen
 * when stack reports that BT device is ready
 */
#ifndef CYW20735B0
APPLICATION_START()
#else
void application_start( void )
#endif
{
    wiced_result_t result;

    wiced_transport_init(&transport_cfg);

    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

    // Set to PUART to see traces on peripheral uart(puart)
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
    // wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);

    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy
    // Note: WICED HCI must be configured to use this - see wiced_trasnport_init(), must
    // be called with wiced_transport_cfg_t.wiced_tranport_data_handler_t callback present
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
#if ( defined(CYW20706A2) || defined(CYW20735B0) || defined(CYW20719B0) || defined(CYW43012C0) )
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif

    WICED_BT_TRACE("HRC APP START\n");

    memset(&hrc_app_cb, 0, sizeof(hrc_app_cb_t));

    /* Initialize Stack and Register Management Callback */
    wiced_bt_stack_init(hrc_management_callback, &wiced_app_cfg_settings, wiced_app_cfg_buf_pools);
}

/*
 * Application initialization is executed after BT stack initialization is completed.
 */
void hrc_application_init(void)
{
    wiced_bt_gatt_status_t gatt_status;

#if !defined(CYW20735B1) && !defined(CYW20819A1) && !defined(CYW20719B2) && !defined(CYW20721B2)
    /* Initialize wiced app */
    wiced_bt_app_init();
#endif
    /* Configure LED PIN as input and initial outvalue as high */
    wiced_hal_gpio_configure_pin( APP_LED, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH );

#if ( defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20735B1) || defined(CYW20819A1) || defined(CYW20721B2) || defined(CYW20719B2) )
    /* Configure buttons available on the platform */
    wiced_platform_register_button_callback( WICED_PLATFORM_BUTTON_1, hrc_interrupt_handler, NULL, WICED_PLATFORM_BUTTON_BOTH_EDGE);
#else
    wiced_hal_gpio_register_pin_for_interrupt( APP_BUTTON, hrc_interrupt_handler, NULL );
    wiced_hal_gpio_configure_pin( APP_BUTTON, APP_BUTTON_SETTINGS, APP_BUTTON_DEFAULT_STATE );
#endif

    wiced_bt_hrc_init(&hrc_callback);

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(hrc_gatts_callback);

    WICED_BT_TRACE("wiced_bt_gatt_register: %d\n", gatt_status);

    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace(hrc_trace_callback);

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* Load the address resolution DB with the keys stored in the NVRAM */
    hrc_load_keys_to_addr_resolution_db();

    /* Starting the periodic seconds timer */
    if ( wiced_init_timer(&app_timer,hrc_timeout,0,WICED_SECONDS_PERIODIC_TIMER) == WICED_SUCCESS)
    {
        if ( wiced_start_timer(&app_timer,1) != WICED_SUCCESS )
        {
            WICED_BT_TRACE("APP START timer FAILED \n");
        }
    }
    else
    {
        WICED_BT_TRACE("APP INIT timer FAILED \n");
    }
}

/*
 * Application link management callback
 */
wiced_result_t hrc_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                    result = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t *p_status;
    wiced_bt_dev_ble_pairing_info_t  *p_info;
    wiced_bt_ble_advert_mode_t       *p_mode;
    wiced_bt_device_link_keys_t       paired_device_link_keys_request;
    uint8_t                           bytes_written, bytes_read;
    uint8_t                           *p_keys;

    WICED_BT_TRACE("hrc_management_cback:%d\n", event);

    switch(event)
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
        hrc_application_init();
        break;

    case BTM_DISABLED_EVT:
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_SC_BOND;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        WICED_BT_TRACE("Pairing Complete: %d\n", p_event_data->pairing_complete.pairing_complete_info.ble.reason);
        hrc_process_pairing_complete(p_event_data->pairing_complete.pairing_complete_info.ble.reason);
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        WICED_BT_TRACE("Encryption Status Event: bd (%B) res %d\n", p_event_data->encryption_status.bd_addr, p_event_data->encryption_status.result);
        break;

    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        hrc_save_link_keys(&p_event_data->paired_device_link_keys_update);
        break;

     case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        if (hrc_read_link_keys(&p_event_data->paired_device_link_keys_request))
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
         wiced_hal_write_nvram ( HRC_LOCAL_KEYS_VS_ID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
         WICED_BT_TRACE("local keys save to NVRAM result: %d \n", result);
         break;

     case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
         /* read keys from NVRAM */
         p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
         wiced_hal_read_nvram( HRC_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
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

/*
 * This function process the scan results and attempt to connect to Heart Rate server
 */
void hrc_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
{
    wiced_result_t          status;
    wiced_bool_t            ret_status;
    uint8_t                 length;
    uint8_t *               p_data;
    uint16_t                service_uuid16=0;
    if ( p_scan_result )
    {
        /* Search for SERVICE_UUID_16 element in the Advertisement data received.Check for both
        complete and partial list */
        p_data = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE, &length );
        if (p_data == NULL)
        {
            p_data = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_16SRV_PARTIAL, &length );
            if (p_data == NULL)
                return;     // No UUID_16 element
        }

        while (length >= LEN_UUID_16)
        {
            STREAM_TO_UINT16(service_uuid16, p_data);
            if (service_uuid16 == UUID_SERVICE_HEART_RATE)
            {
                // UUID16 Heart rate service found
                break;
            }
            length -= LEN_UUID_16;
        }

        if (service_uuid16 != UUID_SERVICE_HEART_RATE)
        {
            // UUID16 Heart rate service not found. Ignore device
            return;
        }

        WICED_BT_TRACE(" Found Device : %B \n", p_scan_result->remote_bd_addr );

        /* Stop the scan since the desired device is found */
        status = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, hrc_scan_result_cback );

        WICED_BT_TRACE( "scan off status %d\n", status );

        /* Initiate the connection */
        ret_status = wiced_bt_gatt_le_connect( p_scan_result->remote_bd_addr, p_scan_result->ble_addr_type, BLE_CONN_MODE_HIGH_DUTY, TRUE );
        WICED_BT_TRACE( "wiced_bt_gatt_le_connect status %d\n", ret_status );
    }
    else
    {
        WICED_BT_TRACE( "Scan completed:\n" );
    }
}

/*
 * Callback for various GATT events.  As this application performs only as a GATT client, some of the events are omitted.
 */
wiced_bt_gatt_status_t hrc_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        if (p_data->connection_status.connected)
        {
            hrc_connection_up(&p_data->connection_status);
        }
        else
        {
            hrc_connection_down(&p_data->connection_status);
        }
        break;

    case GATT_DISCOVERY_RESULT_EVT:
        result = hrc_gatt_discovery_result(&p_data->discovery_result);
        break;

    case GATT_DISCOVERY_CPLT_EVT:
        result = hrc_gatt_discovery_complete(&p_data->discovery_complete);
        break;

    case GATT_OPERATION_CPLT_EVT:
        result = hrc_gatt_operation_complete(&p_data->operation_complete);
        break;

    default:
        break;
    }

    return result;
}

/*
 * This function will be called when a connection is established
 */
void hrc_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t  status;

    WICED_BT_TRACE("%s\n", __FUNCTION__);

    hrc_app_cb.conn_id   = p_conn_status->conn_id;

    // save address of the connected device.
    memcpy(hrc_app_cb.remote_addr, p_conn_status->bd_addr, sizeof(hrc_app_cb.remote_addr));
    hrc_app_cb.addr_type = p_conn_status->addr_type;

    // tell library that connection is up
    wiced_bt_hrc_connection_up(p_conn_status->conn_id);

    /* Initialize heart Rate Service discovery */
    hrc_app_cb.discovery_state            = HRC_DISCOVERY_STATE_SERVICE;
    hrc_app_cb.hear_rate_service_s_handle = 0;
    hrc_app_cb.hear_rate_service_e_handle = 0;

   // perform primary service search
    status = wiced_bt_util_send_gatt_discover(hrc_app_cb.conn_id, GATT_DISCOVER_SERVICES_ALL, UUID_ATTRIBUTE_PRIMARY_SERVICE, 1, 0xffff);
    WICED_BT_TRACE("start discover status:%d\n", status);
}

/*
 * This function will be called when connection goes down
 */
void hrc_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    WICED_BT_TRACE("%s\n", __FUNCTION__);

    hrc_app_cb.conn_id         = 0;
    hrc_app_cb.discovery_state = HRC_DISCOVERY_STATE_SERVICE;
    hrc_app_cb.hear_rate_service_s_handle    = 0;
    hrc_app_cb.hear_rate_service_e_handle    = 0;

    /* Turn off energy expended reset indication.*/
    wiced_hal_gpio_set_pin_output( APP_LED, GPIO_PIN_OUTPUT_HIGH);

    // tell library that connection is down
    wiced_bt_hrc_connection_down(p_conn_status->conn_id);
}

/*
 * Process pairing complete event from the stack
 */
void hrc_process_pairing_complete(uint8_t result)
{
    wiced_bt_gatt_status_t status;

    if (result == WICED_SUCCESS)
    {
        // if we started bonding because we could not start client, do it now
        if ( (hrc_app_cb.hear_rate_service_s_handle != 0) && (hrc_app_cb.hear_rate_service_e_handle != 0))
        {
            status = wiced_bt_hrc_start(hrc_app_cb.conn_id);
            WICED_BT_TRACE("HRC start %d\n", status);
        }
    }
}

/*
 * Process discovery results from the stack
 */
wiced_bt_gatt_status_t hrc_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_data)
{
    uint16_t HEART_RATE_SERVICE_UUID = UUID_SERVICE_HEART_RATE;

    WICED_BT_TRACE("[%s] conn %d type %d state 0x%02x\n", __FUNCTION__, p_data->conn_id, p_data->discovery_type, hrc_app_cb.discovery_state);

    switch (hrc_app_cb.discovery_state)
    {
    case HRC_DISCOVERY_STATE_HRS:
        wiced_bt_hrc_discovery_result(p_data);
        break;

    default:
        if (p_data->discovery_type  == GATT_DISCOVER_SERVICES_ALL)
        {
            if (p_data->discovery_data.group_value.service_type.len == 2)
            {
                WICED_BT_TRACE("s:%04x e:%04x uuid:%x\n", p_data->discovery_data.group_value.s_handle, p_data->discovery_data.group_value.e_handle, p_data->discovery_data.group_value.service_type.uu.uuid16);
                if (memcmp(&p_data->discovery_data.group_value.service_type.uu, &HEART_RATE_SERVICE_UUID, 2) == 0)
                {
                    WICED_BT_TRACE("Heart Rate Service found s:%04x e:%04x\n",
                            p_data->discovery_data.group_value.s_handle,
                            p_data->discovery_data.group_value.e_handle);
                    hrc_app_cb.hear_rate_service_s_handle = p_data->discovery_data.group_value.s_handle;
                    hrc_app_cb.hear_rate_service_e_handle = p_data->discovery_data.group_value.e_handle;
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

/*
 * Process discovery complete from the stack
 */
wiced_bt_gatt_status_t hrc_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data)
{
    wiced_result_t result;

    WICED_BT_TRACE("[%s] conn %d type %d state %d\n", __FUNCTION__, p_data->conn_id, p_data->disc_type, hrc_app_cb.discovery_state);

    switch (hrc_app_cb.discovery_state)
    {
    case HRC_DISCOVERY_STATE_HRS:
        wiced_bt_hrc_discovery_complete(p_data);
        break;

    default:
        if (p_data->disc_type == GATT_DISCOVER_SERVICES_ALL)
        {
            WICED_BT_TRACE("HRC:%04x-%04x\n", hrc_app_cb.hear_rate_service_s_handle, hrc_app_cb.hear_rate_service_e_handle);

            /* If Heart Rate Service found tell WICED BT HRC library to start its discovery */
            if ((hrc_app_cb.hear_rate_service_s_handle != 0) && (hrc_app_cb.hear_rate_service_e_handle != 0))
            {
                hrc_app_cb.discovery_state = HRC_DISCOVERY_STATE_HRS;
                if (wiced_bt_hrc_discover(hrc_app_cb.conn_id, hrc_app_cb.hear_rate_service_s_handle, hrc_app_cb.hear_rate_service_e_handle))
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

/*
 * Process GATT operation complete from the stack
 */
wiced_bt_gatt_status_t hrc_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data)
{
    switch (p_data->op)
    {
    case GATTC_OPTYPE_WRITE:
    case GATTC_OPTYPE_NOTIFICATION:
        wiced_bt_hrc_gatt_op_complete(p_data);
        break;

    case GATTC_OPTYPE_CONFIG:
    case GATTC_OPTYPE_READ:
    case GATTC_OPTYPE_INDICATION:
        WICED_BT_TRACE("This app does not support op:%d\n", p_data->op);
        break;
    }
    return WICED_BT_GATT_SUCCESS;
}

static void hrc_callback(wiced_bt_hrc_event_t event, wiced_bt_hrc_event_data_t *p_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    switch(event)
    {
    case WICED_BT_HRC_EVENT_DISCOVERY:               /**< HRC Discovery event */
        status = p_data->discovery.status;
        WICED_BT_TRACE("%s Discovery status:%d\n", __FUNCTION__, status);
        // This app automatically starts the client
        if (status == WICED_BT_GATT_SUCCESS)
        {
            wiced_bt_hrc_start(hrc_app_cb.conn_id);
        }
        else
        {
            // Disconnect. In the snippet, no point maintain connection without heart rate service
            status = wiced_bt_gatt_disconnect(p_data->discovery.conn_id);
            WICED_BT_TRACE("wiced_bt_gatt_discnnect %d\n", status);
        }
        break;

    case WICED_BT_HRC_EVENT_START:                   /**< HRC Start event */
        status = p_data->start.status;
        WICED_BT_TRACE("%s Start status:%d\n", __FUNCTION__, status);
        if (status == WICED_BT_GATT_SUCCESS)
        {
            hrc_app_cb.started = WICED_TRUE;
        }
        break;

    case WICED_BT_HRC_EVENT_STOP:                    /**< HRC Stop event */
        status = p_data->stop.status;
        WICED_BT_TRACE("%s Stop status:%d\n", __FUNCTION__, status);
        if (status == WICED_BT_GATT_SUCCESS)
        {
            hrc_app_cb.started = WICED_FALSE;
        }
        break;

    case WICED_BT_HRC_EVENT_RESET_ENERGY_EXPENDED:   /**< HRC Reset Energy Expended event */
        status = p_data->reset_energy_expended.status;
        WICED_BT_TRACE("%s Reset Energy Expended status:%d\n", __FUNCTION__, status);
        if(status == WICED_BT_GATT_SUCCESS)
        {
            /* Turn off LED to indicate energy expended got reset */
            wiced_hal_gpio_set_pin_output( APP_LED, GPIO_PIN_OUTPUT_HIGH);
        }
        break;

    case WICED_BT_HRC_EVENT_NOTIFICATION_DATA:/**< HRC Notification event */
        /* server remembers previous configuration and enables notification. so client
        has to set the flag when receive notification */
        hrc_app_cb.started = WICED_TRUE;
        if (p_data->notification_data.energy_expended_present)
        {
            WICED_BT_TRACE( "%s heart_rate:%d energy_expended:%04x\n", __FUNCTION__,
            p_data->notification_data.heart_rate, p_data->notification_data.energy_expended );
        }
        else
        {
            WICED_BT_TRACE( "%s heart_rate:%d\n", __FUNCTION__, p_data->notification_data.heart_rate);
        }

        /* Turn on LED to indicate  energy expended reset is required */
        if ( p_data->notification_data.energy_expended == 0xffff )
        {
            wiced_hal_gpio_set_pin_output( APP_LED, GPIO_PIN_OUTPUT_LOW);
        }
        break;

    default:
        WICED_BT_TRACE( "%s unknown event:%d\n", __FUNCTION__, event);
        break;
    }

    /* If the peer device indicates an Insufficient Authentication */
    if (status == WICED_BT_GATT_INSUF_AUTHENTICATION)
    {
        wiced_bt_device_link_keys_t link_keys;

        /* Check if this device was already paired (LinkKey present) */
        if (hrc_read_link_keys(&link_keys))
        {
            /* Already Paired, Start Encryption */
            wiced_bt_ble_sec_action_type_t sec_act = BTM_BLE_SEC_ENCRYPT;
            status = wiced_bt_dev_set_encryption(hrc_app_cb.remote_addr, BT_TRANSPORT_LE, &sec_act);
            WICED_BT_TRACE("start encrypt result:%d\n", status);
        }
        else
        {
            /* Device Not Paired, Start Pairing (aka Bonding) */
            status = wiced_bt_dev_sec_bond(hrc_app_cb.remote_addr, hrc_app_cb.addr_type, BT_TRANSPORT_LE, 0, NULL);
            WICED_BT_TRACE("start bond result:%d\n", status);
        }
    }
}

/* This function is invoked on button interrupt events */
void hrc_interrupt_handler(void* user_data, uint8_t value )
{
    wiced_result_t  result;
    wiced_bt_gatt_status_t status;
    int             num_slaves;
    static uint32_t button_pushed_time = 0;

    //WICED_BT_TRACE( "But1 %d, But2 %d, But3 %d \n", value & 0x01, ( value & 0x02 ) >> 1, ( value & 0x04 ) >> 2 );
    WICED_BT_TRACE( "hrc_interrupt_handler, app timer :%d\n", hrc_app_cb.timeout );

#if ( defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20735B1) || defined(CYW20819A1) || defined(CYW20721B2) || defined(CYW20719B2) )
    if ( wiced_hal_gpio_get_pin_input_status(WICED_GPIO_PIN_BUTTON_1) == wiced_platform_get_button_pressed_value(WICED_PLATFORM_BUTTON_1) )
#else
    if ( wiced_hal_gpio_get_pin_input_status(APP_BUTTON) == WICED_BUTTON_PRESSED_VALUE )
#endif
    {
        WICED_BT_TRACE( " Button pressed\n" );
        button_pushed_time = hrc_app_cb.timeout;
    }
    else if ( button_pushed_time != 0 )
    {
        WICED_BT_TRACE( " Button released " );
        if (hrc_app_cb.conn_id && (hrc_app_cb.hear_rate_service_s_handle != 0) && (hrc_app_cb.hear_rate_service_e_handle != 0) )
        {
            if ( hrc_app_cb.timeout - button_pushed_time > 5 )
            {
                WICED_BT_TRACE( " after more than 5s. Reset energy expended " );
                status = wiced_bt_hrc_reset_energy_expended( hrc_app_cb.conn_id );
                WICED_BT_TRACE( " %d \n", status );
            }
            else if ((hrc_app_cb.timeout - button_pushed_time > 2) &&
                     (hrc_app_cb.timeout - button_pushed_time < 4)
                    )
            {
                WICED_BT_TRACE( " between 2 to 4 seconds " );
                if (hrc_app_cb.started)
                {
                    status = wiced_bt_hrc_stop( hrc_app_cb.conn_id );
                    WICED_BT_TRACE( " Stop Heart %d \n", status );
                }
                else
                {
                    status = wiced_bt_hrc_start( hrc_app_cb.conn_id );
                    WICED_BT_TRACE( " Start Heart %d \n", status );
                }
            }
        }
        else if (hrc_app_cb.timeout - button_pushed_time < 2)
        {
            WICED_BT_TRACE( " with in 2 seconds. " );
            /*start scan if not connected and no scan in progress*/
            if ((hrc_app_cb.conn_id == 0) && (wiced_bt_ble_get_current_scan_state() == BTM_BLE_SCAN_TYPE_NONE))
            {
                result = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, hrc_scan_result_cback );
                WICED_BT_TRACE("wiced_bt_ble_scan: %d \n", result);
            }
        }
    }
}

/*
 * The function invoked on timeout of app seconds timer.
 */
void hrc_timeout( uint32_t count )
{
    hrc_app_cb.timeout++;
    WICED_BT_TRACE(" %d\n", hrc_app_cb.timeout);
}

/*
 * Read keys from the NVRAM and update address resolution database
 */
void hrc_load_keys_to_addr_resolution_db(void)
{
    uint8_t                     bytes_read;
    wiced_result_t              result;
    wiced_bt_device_link_keys_t keys;

    bytes_read = wiced_hal_read_nvram(HRC_PAIRED_KEYS_VS_ID, sizeof(keys), (uint8_t *)&keys, &result);

    WICED_BT_TRACE(" [%s] read status %d bytes read %d \n", __FUNCTION__, result, bytes_read);

    // if failed to read NVRAM, there is nothing saved at that location
    if (result == WICED_SUCCESS)
    {
#ifdef CYW20706A2
        result = wiced_bt_dev_add_device_to_address_resolution_db(&keys, keys.key_data.ble_addr_type);
#else
        result = wiced_bt_dev_add_device_to_address_resolution_db(&keys);
#endif
    }
}

/*
 * This function is called to save keys generated as a result of pairing or keys update
 */
wiced_bool_t hrc_save_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    uint8_t          bytes_written;
    wiced_result_t   result;

    bytes_written = wiced_hal_write_nvram(HRC_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), (uint8_t *)p_keys, &result);
    WICED_BT_TRACE("Saved %d bytes at id:%d\n", bytes_written, HRC_PAIRED_KEYS_VS_ID);
    return (bytes_written == sizeof (wiced_bt_device_link_keys_t));
}

/*
 * This function is called to read keys for specific bdaddr
 */
wiced_bool_t hrc_read_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    uint8_t         bytes_read;
    wiced_result_t  result;

    bytes_read = wiced_hal_read_nvram(HRC_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), (uint8_t *)p_keys, &result);
    WICED_BT_TRACE("read %d bytes at id:%d\n", bytes_read, HRC_PAIRED_KEYS_VS_ID);
    return (bytes_read == sizeof (wiced_bt_device_link_keys_t));
}

/*
 *  Pass protocol traces up over the transport
 */
void hrc_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    wiced_transport_send_hci_trace(NULL, type, length, p_data);
}
