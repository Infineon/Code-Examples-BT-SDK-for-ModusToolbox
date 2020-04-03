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
* Alert Notification Server (ANS) snippet application
*
* The ANS snippet application shows how to initialize and use WICED BT Alert
* Notification Server library. ANS implemented in GAP central mode and provides ANS services
* to ANC using WICED BT ANS library.
*
* On ClientControl Connect, ANS automatically does Scan and connect to Alert Notification Client (ANC),if
* ANC is advertising with in the range and ANC includes its name (i.e "ANC") in its ADV.
* if ANC does not found with in 90 seconds, Scan stopped automatically. User has to push application
* button to restart the scan to connect to ANC whenever ANC ready for connection.
* User need to use ClientControl ANS GUI option to generate the alerts and control the alerts.
* Application currently support simple alerts, email and SMS/MMS alert categories and notifies this
* information to ClientControl whenever transport connected.
* User can inturn enable subset of supported new alerts and unread alerts when ANS not connected to ANC.
* After connection to ANC, ANS sends new alerts and unread alerts to ANC based on ANC configuration.
* Below explains how new alerts and unread alerts get generated.
*   ClientControl enables the GUI option to generate the alert when ANS in connection with ANC.
*   When user Generate the Alert using GUI, ANC receives the new alert and unread alert.
*   User can clear alert count using GUI Clear Alert button.
* Application calls ANS library APIS based on user requests (i.e. on updating supported alert categories,
* generate and clear the alert requests).
* Read/Write requests received from ANC passed to WICED BT ANS library to update the ANC
* configuration (for example to start or stop new alerts/unread alerts and to configure to send
* only requested alert categories.)
*
* To test this snippet app use ClientControl application -
* \apps\host\ClientControl\<OS>\ClientControl
*
* Features demonstrated
*  - Initialize and use WICED BT ANS library
*  - GATTDB with Alert notification service and characteristics

* To demonstrate the app, work through the following steps.
* 1.Plug the WICED eval board into your computer
* 2.Build and download the application (to the WICED board)
* 3.Start tracing to monitor the activity (see Kit Guide for details)
* 4 Use ClientControl to update supported alerts.
* 5 Pair with ANC.
* 6.Send alerts to ANC using ClientControl.
*/

#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_gatt_util.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"
#include "wiced_result.h"
#include "hci_control_api.h"
#include "GeneratedSource/cycfg_gatt_db.h"

#include "wiced_bt_anp.h"
#include "wiced_bt_ans.h"
#include "string.h"
#include "wiced_bt_stack.h"
#include "wiced_transport.h"
#include "wiced_hal_puart.h"

#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#include "wiced_bt_app_hal_common.h"
#endif

#include "wiced_platform.h"

#ifdef CYW20706A2
#define APP_BUTTON_SETTINGS       WICED_GPIO_BUTTON_SETTINGS( GPIO_EN_INT_RISING_EDGE )
#define APP_BUTTON_PRESSED_VALUE  WICED_GPIO_BUTTON_DEFAULT_STATE
#endif

#if (defined (CYW20735B0) || defined(CYW20719B0))
#define APP_BUTTON_SETTINGS         WICED_GPIO_BUTTON_SETTINGS
#define APP_BUTTON_PRESSED_VALUE    WICED_BUTTON_PRESSED_VALUE
#endif

#define ANS_LOCAL_KEYS_NVRAM_ID         WICED_NVRAM_VSID_START
#define ANS_PAIRED_KEYS_NVRAM_ID       (WICED_NVRAM_VSID_START + 1)

/******************************************************
 *                      Constants
 ******************************************************/
extern const wiced_bt_cfg_settings_t wiced_app_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_app_cfg_buf_pools[];

/******************************************************
 *                     Structures
 ******************************************************/

/******************************************************
 *               Function Prototypes
 ******************************************************/
static wiced_result_t         ans_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static void                   ans_configure_button (void);
static void                   ans_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data );
static void                   ans_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status);
static void                   ans_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status);
static wiced_bt_gatt_status_t ans_gatts_req_callback(wiced_bt_gatt_attribute_request_t *p_data);
static wiced_bt_gatt_status_t ans_gatts_req_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data);
static wiced_bt_gatt_status_t ans_gatts_req_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t * p_data);
static wiced_bt_gatt_status_t ans_gatts_req_write_exec_handler(uint16_t conn_id, wiced_bt_gatt_exec_flag_t exec_falg);
static wiced_bt_gatt_status_t ans_gatts_req_mtu_handler(uint16_t conn_id, uint16_t mtu);
static wiced_bt_gatt_status_t ans_gatts_req_conf_handler(uint16_t conn_id, uint16_t handle);
static wiced_bt_gatt_status_t ans_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
static void                   ans_interrupt_handler(void* user_data, uint8_t value );
static void                   ans_load_keys_to_addr_resolution_db(void);
static wiced_bool_t           ans_save_link_keys(wiced_bt_device_link_keys_t *p_keys);
static wiced_bool_t           ans_read_link_keys(wiced_bt_device_link_keys_t *p_keys);
static void                   ans_send_connection_status_event( wiced_bool_t connected);
static uint8_t                ans_handle_set_supported_new_alert_categories(uint16_t conn_id, uint8_t *p_data, uint16_t length);
static uint8_t                ans_handle_set_supported_unread_alert_categories(uint16_t conn_id, uint8_t *p_data, uint16_t length);
static uint8_t                ans_handle_generate_alert( uint16_t conn_id, uint8_t *p_data, uint8_t len);
static uint8_t                ans_handle_clear_alert( uint16_t conn_id, uint8_t *p_data, uint8_t len);
static void                   ans_handle_get_version(void);
static void                   ans_transport_status( wiced_transport_type_t type );
static uint32_t               ans_proc_rx_hci_cmd(uint8_t *p_data, uint32_t length);
static void                   ans_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);

#ifdef ANS_UNIT_TESTING
static void                   ans_app_timeout( uint32_t arg );
#endif

/******************************************************
 *               Variables Definitions
 ******************************************************/

typedef struct
{
    uint16_t                                conn_id;
    wiced_bt_anp_alert_category_enable_t    current_enabled_alert_cat;
} ans_app_cb_t;

ans_app_cb_t ans_app_cb;

#define ANS_CLIENT_NAME         "ANC"
const char *p_ans_client_name = ANS_CLIENT_NAME;

//#define HCI_TRACE_OVER_TRANSPORT        1 /* Uncomment to enable HCI trace */
#define TRANS_UART_BUFFER_SIZE          1024
#define ANS_TRANS_MAX_BUFFERS           2
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
        .buffer_size  = TRANS_UART_BUFFER_SIZE,
        .buffer_count = 1
    },
    .p_status_handler       = ans_transport_status,
    .p_data_handler         = ans_proc_rx_hci_cmd,
    .p_tx_complete_cback    = NULL
};
wiced_transport_buffer_pool_t*  host_trans_pool;


/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 * Entry point to the application. Set device configuration and start BT
 * stack initialization.  The actual application initialization will happen
 * when stack reports that BT device is ready
 */
#if (defined (CYW20735B0) || defined(CYW20735B1))
void application_start( void )
#else
APPLICATION_START()
#endif
{
    wiced_result_t result;
    wiced_transport_init(&transport_cfg);

    // create special pool for sending data to the MCU
    host_trans_pool = wiced_transport_create_buffer_pool(TRANS_UART_BUFFER_SIZE, ANS_TRANS_MAX_BUFFERS);

#ifdef WICED_BT_TRACE_ENABLE
    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

#ifdef CYW43012C0
    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_WICED_UART );
#else
    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
#if ( defined(CYW20706A2) || defined(CYW20735B0) || defined(CYW20719B0) || defined(CYW43012C0) )
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif
#endif
    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif //WICED_BT_TRACE_ENABLE

    WICED_BT_TRACE("ANS APP START\n");

    memset(&ans_app_cb, 0, sizeof(ans_app_cb));

    /* Initialize Stack and Register Management Callback */
    wiced_bt_stack_init(ans_management_callback, &wiced_app_cfg_settings, wiced_app_cfg_buf_pools);
}

/*
 * ANS application initialization is executed after BT stack initialization is completed.
 */
void ans_application_init(void)
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t         result;
    wiced_bt_ans_gatt_handles_t gatt_handles =
    {
        .new_alert =
        {
            .supported_category = HDLC_ANS_SUPPORTED_NEW_ALERT_CATEGORY_VALUE,
            .value = HDLC_ANS_NEW_ALERT_VALUE,
            .configuration = HDLD_ANS_NEW_ALERT_CLIENT_CHAR_CONFIG,
        },
        .unread_alert =
        {
            .supported_category = HDLC_ANS_SUPPORTED_UNREAD_ALERT_CATEGORY_VALUE,
            .value = HDLC_ANS_UNREAD_ALERT_STATUS_VALUE,
            .configuration = HDLD_ANS_UNREAD_ALERT_STATUS_CLIENT_CHAR_CONFIG,
        },
        .notification_control = HDLC_ANS_ALERT_NOTIFICATION_CONTROL_POINT_VALUE,
    };

#if !defined(CYW20735B1) && !defined(CYW20819A1) && !defined(CYW20719B2) && !defined(CYW20721B2)
    /* Initialize wiced app */
    wiced_bt_app_init();
#endif

#ifndef CYW43012C0
    /* Configure buttons available on the platform */
    ans_configure_button();
#endif

    /* Initialize WICED BT ANS library */
    result = wiced_bt_ans_init(&gatt_handles);
    if (result != WICED_BT_SUCCESS)
        WICED_BT_TRACE("Err: wiced_bt_ans_init failed status:%d\n", result);

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(ans_gatts_callback);

    WICED_BT_TRACE("wiced_bt_gatt_register: %d\n", gatt_status);

    /*  Tell stack to use our GATT databse */
    gatt_status =  wiced_bt_gatt_db_init(gatt_database, gatt_database_len);

    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\n", gatt_status);

#ifdef HCI_TRACE_OVER_TRANSPORT
    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace(ans_trace_callback);
#endif

#if defined(CYW20706A2) || defined(CYW20735B0)
    /* Enable privacy to advertise with RPA */
    wiced_bt_ble_enable_privacy ( WICED_TRUE );
#endif

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* Load the address resolution DB with the keys stored in the NVRAM */
    ans_load_keys_to_addr_resolution_db();

    /* Currently application demonstrates, simple alerts, email and SMS or MMS categories*/
    ans_app_cb.current_enabled_alert_cat = ANP_ALERT_CATEGORY_ENABLE_SIMPLE_ALERT|ANP_ALERT_CATEGORY_ENABLE_EMAIL|ANP_ALERT_CATEGORY_ENABLE_SMS_OR_MMS;

    /* tell to ANS library on current supported categories */
    wiced_bt_ans_set_supported_new_alert_categories(0, ans_app_cb.current_enabled_alert_cat);
    wiced_bt_ans_set_supported_unread_alert_categories(0, ans_app_cb.current_enabled_alert_cat);

#ifdef ANS_UNIT_TESTING
    /* Start scan to find ANS Client */
    result = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, ans_scan_result_cback );
    WICED_BT_TRACE("wiced_bt_ble_scan: %d \n", result);

    wiced_bt_app_start_timer( 1, 0, ans_app_timeout, NULL );
#endif
}

/*
 * ANS link management callback
 */
wiced_result_t ans_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                      result = WICED_BT_SUCCESS;
    uint8_t                             *p_keys;

    WICED_BT_TRACE("ans_management_cback:%d\n", event);

    switch(event)
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
        ans_application_init();
        break;

    case BTM_DISABLED_EVT:
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_SC_BOND;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        WICED_BT_TRACE("Pairing Complete: %d", p_event_data->pairing_complete.pairing_complete_info.ble.reason);
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        WICED_BT_TRACE("Encryption Status Event: bd (%B) res %d", p_event_data->encryption_status.bd_addr, p_event_data->encryption_status.result);
        break;

    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        ans_save_link_keys(&p_event_data->paired_device_link_keys_update);
        break;

     case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        if (ans_read_link_keys(&p_event_data->paired_device_link_keys_request))
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
        wiced_hal_write_nvram ( ANS_LOCAL_KEYS_NVRAM_ID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
        WICED_BT_TRACE("local keys save to NVRAM result: %d \n", result);
        break;

    case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /* read keys from NVRAM */
        p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
        wiced_hal_read_nvram( ANS_LOCAL_KEYS_NVRAM_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
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
 * This function configure for button interrupt.
 */
#ifndef CYW43012C0
void ans_configure_button (void)
{
#if defined(CYW20706A2) || defined(CYW20719B0)
    wiced_bt_app_init();
#endif

    /* Configure buttons available on the platform (pin should be configured before registering interrupt handler ) */
#if defined(CYW20735B0) || defined(CYW20719B0) || defined(CYW20706A2)
    wiced_hal_gpio_configure_pin( WICED_GPIO_PIN_BUTTON, APP_BUTTON_SETTINGS, APP_BUTTON_PRESSED_VALUE );
    wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON, ans_interrupt_handler, NULL );
#else

#ifndef ANS_UNIT_TESTING
    /* Configure buttons available on the platform */
    wiced_platform_register_button_callback( WICED_PLATFORM_BUTTON_1, ans_interrupt_handler, NULL, WICED_PLATFORM_BUTTON_RISING_EDGE);
#else
    /* Configure buttons available on the platform */
    wiced_platform_register_button_callback( WICED_PLATFORM_BUTTON_1, ans_interrupt_handler, NULL, WICED_PLATFORM_BUTTON_BOTH_EDGE);
#endif

#endif
}
#endif

/*
 * This function handles the scan results and attempt to connect to ANS client.
 */
void ans_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
{
    wiced_result_t         status;
    wiced_bool_t           ret_status;
    uint8_t                length;
    uint8_t *              p_data;

    if ( p_scan_result )
    {
        // Advertisement data from clinet should have client complete name.
        p_data = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_NAME_COMPLETE, &length );

        // Check if  the client name is there in the advertisement.
        if ( ( p_data == NULL ) || ( length != strlen(p_ans_client_name) ) || ( memcmp( p_data, p_ans_client_name, strlen(p_ans_client_name) ) != 0 ) )
        {
            // wrong device
            return;
        }

        WICED_BT_TRACE(" Found ANS client : %B \n", p_scan_result->remote_bd_addr );

        /* Stop the scan since the desired device is found */
        status = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, ans_scan_result_cback );

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

/*
 * Callback for various GATT events.  As this application performs only as a GATT server, some of the events are omitted.
 */
wiced_bt_gatt_status_t ans_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        if (p_data->connection_status.connected)
        {
            ans_connection_up(&p_data->connection_status);
        }
        else
        {
            ans_connection_down(&p_data->connection_status);
        }
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = ans_gatts_req_callback(&p_data->attribute_request);
        break;

    default:
        break;
    }

    return result;
}

/*
 * This function will be called when a connection is established
 */
void ans_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t  status;
    wiced_bt_device_link_keys_t keys;
    wiced_result_t result;
    wiced_bt_ble_sec_action_type_t sec_act = BTM_BLE_SEC_ENCRYPT;

    WICED_BT_TRACE("%s\n", __FUNCTION__);

    ans_app_cb.conn_id   = p_conn_status->conn_id;

    // Need to notify ANP Server library that the connection is up
    wiced_bt_ans_connection_up(p_conn_status->conn_id);

    /* if the peer already paired with us initiate encryption instead waiting client to
    initiate*/
    if (ans_read_link_keys(&keys))
    {
        if (!memcmp(p_conn_status->bd_addr, keys.bd_addr, 6))
        {
            result = wiced_bt_dev_set_encryption(keys.bd_addr, BT_TRANSPORT_LE, &sec_act);
            WICED_BT_TRACE("Start Encryption %B %d \n", keys.bd_addr, result);
        }
    }

    /* Send ANS up status to transport to enable ANS services to the user */
    ans_send_connection_status_event( WICED_TRUE);
}

/*
 * This function will be called when connection goes down
 */
void ans_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    WICED_BT_TRACE("%s\n", __FUNCTION__);

    // tell library that connection is down
    wiced_bt_ans_connection_down(p_conn_status->conn_id);

    ans_app_cb.conn_id           = 0;

    /* Send ANS down status to transport to disable ANS services to the user */
    ans_send_connection_status_event(  WICED_FALSE );
}

/*
 * Process GATT request from the peer. Even our main goal is to be a GATT client for the
 * ANCS service, we need to support mandatory GATT procedures.
 */
wiced_bt_gatt_status_t ans_gatts_req_callback(wiced_bt_gatt_attribute_request_t *p_data)
{
    wiced_result_t result = WICED_BT_GATT_INVALID_PDU;

    WICED_BT_TRACE("ans_gatts_req_callback. conn %d, type %d\n", p_data->conn_id, p_data->request_type);

    switch (p_data->request_type)
    {
    case GATTS_REQ_TYPE_READ:
        result = ans_gatts_req_read_handler(p_data->conn_id, &(p_data->data.read_req));
        break;
    case GATTS_REQ_TYPE_WRITE:
        result = ans_gatts_req_write_handler(p_data->conn_id, &(p_data->data.write_req));
        break;
    case GATTS_REQ_TYPE_MTU:
        result = ans_gatts_req_mtu_handler(p_data->conn_id, p_data->data.mtu);
        break;
   default:
        break;
    }

    return result;
}

/*
 * Process Read request or command from peer device
 */
wiced_bt_gatt_status_t ans_gatts_req_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_data)
{
    int          i, attr_len_to_copy;

    /* ANP server library takes care reading of ANS service characteristics */
    if ( (p_data->handle >= HDLS_ANS) && ( p_data->handle <= HDLC_ANS_ALERT_NOTIFICATION_CONTROL_POINT_VALUE ) )
    {
        return wiced_bt_ans_process_gatt_read_req (conn_id, p_data);
    }

    /* All other GATT services read handled by application */

    // Check for a matching handle entry
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == p_data->handle)
        {
            break;
        }
    }
    if (i == app_gatt_db_ext_attr_tbl_size)
    {
        WICED_BT_TRACE("read_hndlr attr not found hdl:%x\n", p_data->handle);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    attr_len_to_copy = app_gatt_db_ext_attr_tbl[i].cur_len;

    WICED_BT_TRACE("read_hndlr conn_id:%d hdl:%x offset:%d len:%d\n", conn_id, p_data->handle, p_data->offset, attr_len_to_copy);

    if (p_data->offset >= app_gatt_db_ext_attr_tbl[i].cur_len)
    {
        attr_len_to_copy = 0;
    }

    if (attr_len_to_copy != 0)
    {
        uint8_t *from;
        int      to_copy = attr_len_to_copy - p_data->offset;


        if (to_copy > *p_data->p_val_len)
        {
            to_copy = *p_data->p_val_len;
        }

        from = app_gatt_db_ext_attr_tbl[i].p_data + p_data->offset;
        *p_data->p_val_len = to_copy;

        memcpy(p_data->p_val, from, to_copy);
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process write request or write command from peer device
 */
wiced_bt_gatt_status_t ans_gatts_req_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t * p_data)
{
    WICED_BT_TRACE("write_handler: conn_id:%d hdl:0x%x prep:%d offset:%d len:%d \n ", conn_id, p_data->handle, p_data->is_prep, p_data->offset, p_data->val_len);

    /* ANP server library takes care writing to ANS service characteristics */
    if ( (p_data->handle >= HDLS_ANS) && ( p_data->handle <= HDLC_ANS_ALERT_NOTIFICATION_CONTROL_POINT_VALUE ) )
    {
        return wiced_bt_ans_process_gatt_write_req(conn_id, p_data);
    }

    /* This snippet does not have any other services to support the GATT write */
    return WICED_BT_GATT_INVALID_HANDLE;
}

/*
 * Write Execute Procedure
 */
wiced_bt_gatt_status_t ans_gatts_req_write_exec_handler(uint16_t conn_id, wiced_bt_gatt_exec_flag_t exec_falg)
{
    WICED_BT_TRACE("write exec: flag:%d\n", exec_falg);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process MTU request from the peer
 */
wiced_bt_gatt_status_t ans_gatts_req_mtu_handler(uint16_t conn_id, uint16_t mtu)
{
    WICED_BT_TRACE("req_mtu: %d\n", mtu);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process indication confirm from the peer.
 */
wiced_bt_gatt_status_t ans_gatts_req_conf_handler(uint16_t conn_id, uint16_t handle)
{
    WICED_BT_TRACE("indication_cfm, conn %d hdl %d\n", conn_id, handle);
    return WICED_BT_GATT_SUCCESS;
}

/* Send ANS enabled event on transport detection */
void ans_transport_status( wiced_transport_type_t type )
{
     WICED_BT_TRACE("HCI_CONTROL_ANS_EVENT_ANS_ENABLED\n");

    /* Now onwards redirect all traces to BT Spy log */
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_WICED_UART );

    /* tell to ClientControl on current supported categories. User can enable and and disable in the supported if required */
    wiced_transport_send_data( HCI_CONTROL_ANS_EVENT_ANS_ENABLED, (uint8_t *)&ans_app_cb.current_enabled_alert_cat, 2 );
}

void ans_start_scan (void)
{
    wiced_result_t  result;
    /* Start scan to find ANS Client */
    if( (wiced_bt_ble_get_current_scan_state() == BTM_BLE_SCAN_TYPE_NONE) &&
        (ans_app_cb.conn_id == 0))
    {
         result = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, ans_scan_result_cback );
         WICED_BT_TRACE("wiced_bt_ble_scan: %d \n", result);
    }
}

/*
 * Handle received command over UART. Please refer to the WICED HCI Control
 * Protocol for details on the protocol.  The function converts from the WICED
 * HCI remote control commands to the commands expected by the ANS.
 */
uint32_t  ans_proc_rx_hci_cmd(uint8_t *p_buffer, uint32_t length)
{
    uint16_t                opcode;
    uint8_t*                p_data = p_buffer;
    uint16_t                payload_len;
    uint8_t                 status = HCI_CONTROL_STATUS_SUCCESS;
    wiced_bt_gatt_status_t  gatt_status = WICED_BT_GATT_SUCCESS;

    // WICED_BT_TRACE("hci_control_proc_rx_cmd:%d\n", length);

    if (!p_data)
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    //Expected minimum 4 byte as the wiced header
    if (length < 4)
    {
        WICED_BT_TRACE("invalid params\n");
        wiced_transport_free_buffer(p_data);
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    STREAM_TO_UINT16(opcode, p_data);       // Get OpCode
    STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length

    WICED_BT_TRACE("cmd_opcode 0x%04x\n", opcode);

    switch (opcode)
    {
        case HCI_CONTROL_ANS_COMMAND_SET_SUPPORTED_NEW_ALERT_CATEGORIES:
            WICED_BT_TRACE("Set Supported New Alert Categories\n");
            status = ans_handle_set_supported_new_alert_categories(ans_app_cb.conn_id, p_data, payload_len);
            break;

        case HCI_CONTROL_ANS_COMMAND_SET_SUPPORTED_UNREAD_ALERT_CATEGORIES:
            WICED_BT_TRACE("Set Supported Unread Alert Categories\n");
            status = ans_handle_set_supported_unread_alert_categories(ans_app_cb.conn_id, p_data, payload_len);
            break;

        case HCI_CONTROL_ANS_COMMAND_GENERATE_ALERT:
            WICED_BT_TRACE("Generate Alert\n");
            status = ans_handle_generate_alert(ans_app_cb.conn_id, p_data, payload_len);
            break;

        case HCI_CONTROL_ANS_COMMAND_CLEAR_ALERT:
            WICED_BT_TRACE("Clear Alert\n");
            status = ans_handle_clear_alert(ans_app_cb.conn_id, p_data, payload_len);
            break;

        case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
            ans_handle_get_version();
            ans_transport_status(0);
            ans_start_scan ();
            break;

        default:
            WICED_BT_TRACE("ignored opcode:0x%04x\n", opcode);
            status = HCI_CONTROL_STATUS_UNKNOWN_COMMAND;
            break;
    }

    wiced_transport_send_data(HCI_CONTROL_ANS_EVENT_COMMAND_STATUS, &status, 1);

    // Freeing the buffer in which data is received
    wiced_transport_free_buffer(p_buffer);
    return HCI_CONTROL_STATUS_SUCCESS;
}

#ifdef ANS_UNIT_TESTING
uint32_t ans_app_timer_count=0;
void ans_app_timeout( uint32_t arg )
{
    ans_app_timer_count++;
    WICED_BT_TRACE("%d \n", ans_app_timer_count);
}
void test_ans(void)
{
    wiced_result_t  result;
    uint8_t cat = 0;

    static uint32_t button_pushed_time = 0;
    static uint32_t tc=0;

    if ( wiced_hal_gpio_get_pin_input_status(APP_BUTTON) == WICED_BUTTON_PRESSED_VALUE )
    {
        WICED_BT_TRACE( " Button pressed\n" );
        button_pushed_time = ans_app_timer_count;
    }
    else if ( button_pushed_time != 0 )
    {
        //calculate button pressed time
        uint32_t pt = ans_app_timer_count - button_pushed_time;

        WICED_BT_TRACE( " Button released \n" );

        if (pt > 6)
        {
            //clear the alerts
            cat = ANP_ALERT_CATEGORY_ID_SIMPLE_ALERT;
            ans_handle_clear_alert(0, &cat, 1);
            cat = ANP_ALERT_CATEGORY_ID_EMAIL;
            ans_handle_clear_alert(0, &cat, 1);
            cat = ANP_ALERT_CATEGORY_ID_SMS_OR_MMS;
            ans_handle_clear_alert(0, &cat, 1);
        }
        else if (pt > 4)
        {
            //Generate alert
            tc++;

            if (tc == 1)
            {
                cat = ANP_ALERT_CATEGORY_ID_SIMPLE_ALERT;
                ans_handle_generate_alert(ans_app_cb.conn_id, &cat, 1);

            }
            else if (tc ==2)
            {
                cat = ANP_ALERT_CATEGORY_ID_EMAIL;
                ans_handle_generate_alert(ans_app_cb.conn_id, &cat, 1);
            }
            else if (tc ==3)
            {
                cat = ANP_ALERT_CATEGORY_ID_SMS_OR_MMS;
                ans_handle_generate_alert(ans_app_cb.conn_id, &cat, 1);

                tc = 0; // To repeat the sequence
            }
        }
        else if(pt > 2)
        {
            uint16_t sup_a = ANP_ALERT_CATEGORY_ENABLE_EMAIL;
            ans_handle_set_supported_new_alert_categories(0, (uint8_t*)&sup_a, 2);
            ans_handle_set_supported_unread_alert_categories(0, (uint8_t*)&sup_a, 2);
        }
    }
}
#endif

/* This function is invoked on button interrupt events */
void ans_interrupt_handler(void* user_data, uint8_t value )
{
    wiced_result_t  result;

#ifdef ANS_UNIT_TESTING
    test_ans();
#endif

    /*start scan if not connected and no scan in progress*/
    if ((ans_app_cb.conn_id == 0) && (wiced_bt_ble_get_current_scan_state() == BTM_BLE_SCAN_TYPE_NONE))
    {
        result = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, ans_scan_result_cback );
        WICED_BT_TRACE("wiced_bt_ble_scan: %d \n", result);
    }
}

/*
 * Read keys from the NVRAM and update address resolution database
 */
void ans_load_keys_to_addr_resolution_db(void)
{
    uint8_t                     bytes_read;
    wiced_result_t              result;
    wiced_bt_device_link_keys_t keys;

    bytes_read = wiced_hal_read_nvram(ANS_PAIRED_KEYS_NVRAM_ID, sizeof(keys), (uint8_t *)&keys, &result);

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
wiced_bool_t ans_save_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    uint8_t          bytes_written;
    wiced_result_t   result;

    bytes_written = wiced_hal_write_nvram(ANS_PAIRED_KEYS_NVRAM_ID, sizeof(wiced_bt_device_link_keys_t), (uint8_t *)p_keys, &result);
    WICED_BT_TRACE("Saved %d bytes at id:%d \n", bytes_written, ANS_PAIRED_KEYS_NVRAM_ID);
    return (bytes_written == sizeof (wiced_bt_device_link_keys_t));
}

/*
 * This function is called to read keys for specific bdaddr
 */
wiced_bool_t ans_read_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    uint8_t         bytes_read;
    wiced_result_t  result;

    bytes_read = wiced_hal_read_nvram(ANS_PAIRED_KEYS_NVRAM_ID, sizeof(wiced_bt_device_link_keys_t), (uint8_t *)p_keys, &result);
    WICED_BT_TRACE("read %d bytes at id:%d \n", bytes_read, ANS_PAIRED_KEYS_NVRAM_ID);
    return (bytes_read == sizeof (wiced_bt_device_link_keys_t));
}

#ifdef HCI_TRACE_OVER_TRANSPORT
/*
 *  Pass protocol traces up over the transport
 */
void ans_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    wiced_transport_send_hci_trace(host_trans_pool, type, length, p_data);
}
#endif

/*
 *  transfer connection event to uart
 */
void ans_send_connection_status_event( wiced_bool_t connected)
{
    if ( connected )
    {
        wiced_transport_send_data ( HCI_CONTROL_ANS_EVENT_CONNECTION_UP, NULL, 0 );
    }
    else
    {
        wiced_transport_send_data ( HCI_CONTROL_ANS_EVENT_CONNECTION_DOWN, NULL, 0 );
    }
}

uint8_t ans_handle_set_supported_new_alert_categories(uint16_t conn_id, uint8_t *p_data, uint16_t length)
{
    uint16_t    supported_new_alert_cat = 0;
    uint8_t     status = HCI_CONTROL_STATUS_SUCCESS;

    if (ans_app_cb.conn_id != 0)
    {
        WICED_BT_TRACE("request not supported: ANS connected with ANC \n");
        return HCI_CONTROL_STATUS_WRONG_STATE;
    }

    if (length == 2)
    {
        STREAM_TO_UINT16(supported_new_alert_cat, p_data);

        /* Make sure user sets choice only in supported categories */
        supported_new_alert_cat &= ans_app_cb.current_enabled_alert_cat;
        wiced_bt_ans_set_supported_new_alert_categories(conn_id, supported_new_alert_cat);
    }
    else
    {
        status = HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    return status;
}

uint8_t ans_handle_set_supported_unread_alert_categories(uint16_t conn_id, uint8_t *p_data, uint16_t length)
{
    uint16_t    supported_unread_alert_cat = 0;
    uint8_t     status = HCI_CONTROL_STATUS_SUCCESS;

    if (ans_app_cb.conn_id != 0)
    {
        WICED_BT_TRACE("request not supported: ANS connected with ANC \n");
        return HCI_CONTROL_STATUS_WRONG_STATE;
    }

    if (length == 2)
    {
        STREAM_TO_UINT16(supported_unread_alert_cat, p_data);

        /* Make sure user sets choice only in supported categories */
        supported_unread_alert_cat &= ans_app_cb.current_enabled_alert_cat;
        wiced_bt_ans_set_supported_unread_alert_categories(conn_id, supported_unread_alert_cat);
    }
    else
    {
        status = HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    return status;

}

uint8_t ans_handle_generate_alert( uint16_t conn_id, uint8_t *p_data, uint8_t len)
{
    uint8_t                 status = HCI_CONTROL_STATUS_FAILED;
    wiced_bt_gatt_status_t  gatt_status;

    if (ans_app_cb.conn_id == 0)
    {
        WICED_BT_TRACE("ans_handle_generate_alert: Service not connected \n");
        return HCI_CONTROL_STATUS_NOT_CONNECTED;
    }

    if (len == 1)
    {
        gatt_status = wiced_bt_ans_process_and_send_new_alert( conn_id, *p_data);
        if (gatt_status == WICED_BT_GATT_SUCCESS )
        {
            gatt_status = wiced_bt_ans_process_and_send_unread_alert (conn_id, *p_data);
            if (gatt_status != WICED_BT_GATT_SUCCESS)
            {
                WICED_BT_TRACE("ans_handle_generate_alert: unread alert send error %d \n", gatt_status);
            }
            else
            {
                status = HCI_CONTROL_STATUS_SUCCESS;
            }
        }
        else
        {
            WICED_BT_TRACE("ans_handle_generate_alert: new alert send error %d \n", gatt_status);
        }
    }
    else
    {
        status = HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    return status;
}

uint8_t ans_handle_clear_alert( uint16_t conn_id, uint8_t *p_data, uint8_t len)
{
    uint8_t status = HCI_CONTROL_STATUS_SUCCESS;

    if (len == 1)
    {
        if (wiced_bt_ans_clear_alerts(conn_id, *p_data) != WICED_TRUE)
        {
            status = HCI_CONTROL_STATUS_FAILED;
        }
    }
    else
    {
        status = HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    return status;
}

/*
 * Handle the GET_VERSION command
 */
void ans_handle_get_version(void)
{
    uint8_t   tx_buf[20];
    uint8_t   cmd = 0;
    uint32_t  chip = CHIP;

    tx_buf[cmd++] = WICED_SDK_MAJOR_VER;
    tx_buf[cmd++] = WICED_SDK_MINOR_VER;
    tx_buf[cmd++] = WICED_SDK_REV_NUMBER;
    tx_buf[cmd++] = WICED_SDK_BUILD_NUMBER & 0xFF;
    tx_buf[cmd++] = (WICED_SDK_BUILD_NUMBER>>8) & 0xFF;
    tx_buf[cmd++] = chip & 0xFF;
    tx_buf[cmd++] = (chip>>8) & 0xFF;
    tx_buf[cmd++] = (chip>>24) & 0xFF;
    tx_buf[cmd++] = 0; // not used

    /* Send MCU app the supported features */
    tx_buf[cmd++] = HCI_CONTROL_GROUP_ANS;

    wiced_transport_send_data(HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd);
}
