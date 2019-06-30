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
 * Alert Notification Client(ANC) Snippet Application
 *
 * The ANC snippet application shows how to initialize and use WICED Alert Notification Client Library.
 * ANC is implemented in GAP peripheral role and configures Alert Notification Server(ANS)
 * to receive Alerts based on user requests using ClientControl(CC) Alert GUI.
 *
 * On application init, ANC starts advertisements. The advertisement data would include "ANC".
 * The ANS would scan for this name and connects itself automatically when found.
 * Once connection is established ANC UI would be enabled.
 * User has to perform the following steps after connection:
 * 1) To get to know the Server Supported New Alerts and Unread Alerts using New Alert and Unread Alert radio buttons,
 *    Read Alerts button has to be clicked. Once this operation is completed successfully,
 *    ANC GUI shows server supported New Alert and Unread Alert categories.
 *    NOTE: For now, The App is designed to support Simple ALert, SMS/MMS and Email.The remaining ALert types will not be highlighted.
 * 2) After ANC knows the Server Supported Alerts, ANC should configure for Alert notifications as mentioned below:
 *    2.1 -- Irrespective of Alert type, all New/Unread Alerts should be enabled by using EnableNewAlerts and EnableUnreadAlerts buttons.
 *    2.2 -- Using Control Alerts Button User can selectively Enable/disable Alerts or "Notify immediately all the pending Alerts".
 *           This can be done based on selection of function from drop down which is present right above the Control Alerts button.
 *           In this use case the functionality for New Alerts or Unread Alerts is based on which Radio Button is enabled.
 * 3) Once configured for Alert Notifications as mentioned above, Alerts generated from Server(i.e ANS) can be monitored through traces.
 *
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
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"
#include "wiced_result.h"
#include "hci_control_api.h"
#include "wiced_app_cfg.h"
#include "wiced_platform.h"
#include "wiced_bt_anc.h"
#include "string.h"
#include "wiced_bt_stack.h"

#define HCI_TRACE_OVER_TRANSPORT    1   // Send trace messages over WICED HCI interface
//#define TEST_HCI_CONTROL            1   // Use WICED HCI interface for test purposes

#if defined WICED_BT_TRACE_ENABLE || defined TEST_HCI_CONTROL || defined HCI_TRACE_OVER_TRANSPORT
#include "wiced_transport.h"
#endif

#ifndef TEST_HCI_CONTROL
#ifdef CYW20706A2
#define APP_BUTTON                  WICED_GPIO_BUTTON
#define APP_BUTTON_SETTINGS         (WICED_GPIO_BUTTON_SETTINGS( GPIO_EN_INT_BOTH_EDGE ))
#define APP_BUTTON_DEFAULT_STATE    WICED_GPIO_BUTTON_DEFAULT_STATE
#endif

#if ( defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20735B1) || defined(CYW20819A1) )
#define APP_BUTTON                  WICED_GPIO_PIN_BUTTON_1
#endif

#ifdef CYW20735B0
#define APP_BUTTON                  WICED_GPIO_PIN_BUTTON
#endif

#if ( defined(CYW20719B0) || defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20735B1) || defined(CYW20735B0) )
#define APP_BUTTON_SETTINGS         ( GPIO_INPUT_ENABLE | GPIO_PULL_DOWN | GPIO_EN_INT_BOTH_EDGE )
#define APP_BUTTON_DEFAULT_STATE    GPIO_PIN_OUTPUT_LOW
#endif
#endif

#define ANC_LOCAL_KEYS_NVRAM_ID                 WICED_NVRAM_VSID_START
#define ANC_PAIRED_KEYS_NVRAM_ID                (WICED_NVRAM_VSID_START+1)

/******************************************************************************************
 *                                      Constants
 *****************************************************************************************/

/******************************************************************************************
 *                                     Structures
 ******************************************************************************************/

/******************************************************************************************
 *                                 Function Prototypes
 *****************************************************************************************/
static void                   anc_callback(wiced_bt_anc_event_t event, wiced_bt_anc_event_data_t *p_data);
static wiced_result_t         anc_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static wiced_bt_gatt_status_t anc_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
static void                   anc_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status);
static void                   anc_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status);
static void                   anc_set_advertisement_data();
static wiced_bt_gatt_status_t anc_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data);
static wiced_bt_gatt_status_t anc_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_data);
static wiced_bt_gatt_status_t anc_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data);
static void                   anc_start_pair(void);
static void                   anc_load_keys_to_addr_resolution_db(void);
static wiced_bool_t           anc_save_link_keys(wiced_bt_device_link_keys_t *p_keys);
static wiced_bool_t           anc_read_link_keys(wiced_bt_device_link_keys_t *p_keys);
static void                   anc_process_write_rsp(wiced_bt_gatt_operation_complete_t *p_data);
static void                   anc_process_read_rsp(wiced_bt_gatt_operation_complete_t *p_data);
static void                   anc_notification_handler(wiced_bt_gatt_operation_complete_t *p_data);
static void                   anc_trigger_pending_action ( void );
static void                   clear_anc_pending_cmd_context ( void );
static void                   hci_control_send_anc_enabled( void );
static void                   hci_control_send_anc_disabled( void );
static void                   hci_control_send_supported_new_alerts( uint8_t status, uint16_t conn_id, uint8_t *p_data );
static void                   hci_control_send_supported_unread_alerts( uint8_t status, uint16_t conn_id, uint8_t *p_data );
static void                   hci_control_send_control_alerts_result( uint8_t status, uint16_t conn_id, uint8_t cmd_id, uint8_t category_id );
static void                   hci_control_send_enable_new_alerts_result( uint8_t status, uint16_t conn_id );
static void                   hci_control_send_disable_new_alerts_result( uint8_t status, uint16_t conn_id );
static void                   hci_control_send_enable_unread_alerts_result(uint8_t status, uint16_t conn_id);
static void                   hci_control_send_disable_unread_alerts_result( uint8_t status, uint16_t conn_id );
static const char *           alert_type_name (wiced_bt_anp_alert_category_id_t id);
#ifndef TEST_HCI_CONTROL
static void                   anc_interrupt_handler(void* user_data, uint8_t value );
static void                   anc_app_timeout( uint32_t arg );
#else
static void                   anc_handle_get_version(void);
#endif
/*******************************************************************************************
 *                                 Variables Definitions
 ******************************************************************************************/
typedef struct
{
    uint16_t conn_id;

#define ANC_DISCOVERY_STATE_SERVICE    0
#define ANC_DISCOVERY_STATE_ANC        1
    uint8_t discovery_state;

    // Current value of the client configuration descriptor for characteristic 'Report'
    uint16_t                    anc_s_handle;
    uint16_t                    anc_e_handle;

    BD_ADDR                     remote_addr;   //address of currently connected client
    wiced_bt_ble_address_type_t addr_type;
}anc_app_state_t;

anc_app_state_t anc_app_state;


/* context of command that is failed due to gatt insufficient authentication.
Need to re trigger the command after anc establish authentication with ans */
uint8_t    anc_pending_cmd_context[4] = {0};

#if defined WICED_BT_TRACE_ENABLE || defined HCI_TRACE_OVER_TRANSPORT || defined TEST_HCI_CONTROL

#define TRANS_UART_BUFFER_SIZE          1024
#define ANC_TRANS_MAX_BUFFERS          2

#ifdef TEST_HCI_CONTROL
static uint32_t  anc_proc_rx_hci_cmd(uint8_t *p_data, uint32_t length);
void anc_transport_status( wiced_transport_type_t type );
#endif

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
#ifdef TEST_HCI_CONTROL
    .p_status_handler = anc_transport_status,
    .p_data_handler = anc_proc_rx_hci_cmd,
#else
    .p_status_handler = NULL,
    .p_data_handler = NULL,
#endif
    .p_tx_complete_cback = NULL
};

wiced_transport_buffer_pool_t*  host_trans_pool;
static void anc_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);
#endif

#ifndef TEST_HCI_CONTROL
uint32_t anc_app_timer_count=0;
static uint8_t c = 0;
#endif


/***********************************************************************************************
 *                                  Function Definitions
 **********************************************************************************************/
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

#if defined WICED_BT_TRACE_ENABLE || defined HCI_TRACE_OVER_TRANSPORT
    wiced_transport_init(&transport_cfg);

//    // create special pool for sending data to the MCU
    host_trans_pool = wiced_transport_create_buffer_pool(TRANS_UART_BUFFER_SIZE, ANC_TRANS_MAX_BUFFERS);

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
    // be called with wiced_transport_cfg_t.wiced_trancort_data_handler_t callback present
#ifdef TEST_HCI_CONTROL
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#else
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
#endif

#endif
    WICED_BT_TRACE("ANC APP START\n");

    memset(&anc_app_state, 0, sizeof(anc_app_state));
    memset(anc_pending_cmd_context, 0, sizeof(anc_pending_cmd_context));

    /* Initialize Stack and Register Management Callback */
    wiced_bt_stack_init(anc_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);
}

void anc_application_init()
{
    wiced_bt_gatt_status_t gatt_status;

#if !defined(CYW20735B1) && !defined(CYW20819A1) && !defined(CYW20719B2) && !defined(CYW20721B2)
    /* Initialize wiced app */
    wiced_bt_app_init();
#endif

#ifndef TEST_HCI_CONTROL
#if defined(CYW20819A1)
    wiced_platform_register_button_callback( APP_BUTTON, anc_interrupt_handler, NULL, WICED_PLATFORM_BUTTON_BOTH_EDGE);
#else
    /* Configure buttons available on the platform */
    wiced_hal_gpio_register_pin_for_interrupt( APP_BUTTON, anc_interrupt_handler, NULL );
    wiced_hal_gpio_configure_pin( APP_BUTTON, APP_BUTTON_SETTINGS, APP_BUTTON_DEFAULT_STATE );
#endif
#endif

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(anc_gatts_callback);

    WICED_BT_TRACE("wiced_bt_gatt_register: %d\n", gatt_status);

#ifdef WICED_BT_TRACE_ENABLE
    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace(anc_trace_callback);
#endif

#if defined(CYW20706A2) || defined(CYW20735B0)
    /* Enable privacy to advertise with RPA */
    wiced_bt_ble_enable_privacy ( WICED_TRUE );
#endif

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* Load the address resolution DB with the keys stored in the NVRAM */
    anc_load_keys_to_addr_resolution_db();

    /* Set the advertising params and make the device discoverable */
    anc_set_advertisement_data();

#ifndef TEST_HCI_CONTROL
    gatt_status =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    WICED_BT_TRACE("wiced_bt_start_advertisements %d\n", gatt_status);
    wiced_bt_app_start_timer( 1, 0, anc_app_timeout, NULL );
#endif
}

static void anc_set_advertisement_data()
{
    wiced_bt_ble_advert_elem_t adv_elem[3];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t power = 0;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((char *)wiced_bt_cfg_settings.device_name);
    adv_elem[num_elem].p_data       = (uint8_t*)wiced_bt_cfg_settings.device_name;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

static wiced_result_t anc_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                  result = WICED_BT_SUCCESS;
    wiced_bt_ble_advert_mode_t      *p_mode;
    uint8_t                         *p_keys;

    WICED_BT_TRACE("anc_management_callback:%d\n", event);

    switch(event)
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
        wiced_bt_anc_init(&anc_callback);
        anc_application_init();
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
        if (p_event_data->encryption_status.result == WICED_BT_SUCCESS)
            anc_trigger_pending_action();
        else /* pending command no more valid to send if authentication fails */
            clear_anc_pending_cmd_context();
        break;

    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        anc_save_link_keys(&p_event_data->paired_device_link_keys_update);
        break;

     case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        if (anc_read_link_keys(&p_event_data->paired_device_link_keys_request))
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
        wiced_hal_write_nvram ( ANC_LOCAL_KEYS_NVRAM_ID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
        WICED_BT_TRACE("local keys save to NVRAM result: %d \n", result);
        break;

    case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /* read keys from NVRAM */
        p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
        wiced_hal_read_nvram( ANC_LOCAL_KEYS_NVRAM_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
        WICED_BT_TRACE("local keys read from NVRAM result: %d \n",  result);
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        p_mode = &p_event_data->ble_advert_state_changed;
        WICED_BT_TRACE( "Advertisement State Change: %d\n", *p_mode);
        break;

    default:
        break;
    }
    return result;
}

/*
 * Callback for various GATT events.  As this application performs only as a GATT Client, some of the events are omitted.
 */
static wiced_bt_gatt_status_t anc_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        if (p_data->connection_status.connected)
        {
            anc_connection_up(&p_data->connection_status);
        }
        else
        {
            anc_connection_down(&p_data->connection_status);
        }
        break;

    case GATT_OPERATION_CPLT_EVT:
        result = anc_gatt_operation_complete(&p_data->operation_complete);
        break;

    case GATT_DISCOVERY_RESULT_EVT:
        result = anc_gatt_discovery_result(&p_data->discovery_result);
        break;

    case GATT_DISCOVERY_CPLT_EVT:
        result = anc_gatt_discovery_complete(&p_data->discovery_complete);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
// Not supported for this as this acts as a client role
        break;

    default:
        break;
    }

    return result;
}

/*
 * This function will be called when a connection is established
 */
static void anc_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t  status;

    WICED_BT_TRACE("%s\n", __FUNCTION__);

    anc_app_state.conn_id = p_conn_status->conn_id;

    // save address of the connected device.
    memcpy(anc_app_state.remote_addr, p_conn_status->bd_addr, sizeof(anc_app_state.remote_addr));
    anc_app_state.addr_type = p_conn_status->addr_type;

    // need to notify ANC library that the connection is up
    wiced_bt_anc_client_connection_up(p_conn_status);

    /* Initialize WICED BT ANC library Start discovery */
    anc_app_state.discovery_state = ANC_DISCOVERY_STATE_SERVICE;
    anc_app_state.anc_s_handle = 0;
    anc_app_state.anc_e_handle = 0;

   // perform primary service search
    status = wiced_bt_util_send_gatt_discover(anc_app_state.conn_id, GATT_DISCOVER_SERVICES_ALL, UUID_ATTRIBUTE_PRIMARY_SERVICE, 1, 0xffff);
    WICED_BT_TRACE("start discover status:%d\n", status);
}

static void anc_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    WICED_BT_TRACE("%s\n", __FUNCTION__);

    anc_app_state.conn_id         = 0;
    anc_app_state.anc_s_handle   = 0;
    anc_app_state.anc_e_handle   = 0;
    anc_app_state.discovery_state = ANC_DISCOVERY_STATE_SERVICE;
    /* pending command no more valid now */
    clear_anc_pending_cmd_context();

    memset(anc_app_state.remote_addr, 0, sizeof(wiced_bt_device_address_t));
    // tell library that connection is down
    wiced_bt_anc_client_connection_down(p_conn_status);

    hci_control_send_anc_disabled();
}

/*
 * GATT operation started by the client has been completed
 */
static wiced_bt_gatt_status_t anc_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data)
{
    switch (p_data->op)
    {
    case GATTC_OPTYPE_WRITE:
        anc_process_write_rsp(p_data);
        break;

    case GATTC_OPTYPE_CONFIG:
        WICED_BT_TRACE("This app does not support op:%d\n", p_data->op);
        break;

    case GATTC_OPTYPE_NOTIFICATION:
        anc_notification_handler(p_data);
        break;

    case GATTC_OPTYPE_READ:
        anc_process_read_rsp(p_data);
        break;

    case GATTC_OPTYPE_INDICATION:
        WICED_BT_TRACE("This app does not support op:%d\n", p_data->op);
        break;
    }
    return WICED_BT_GATT_SUCCESS;
}

static wiced_bt_gatt_status_t anc_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_data)
{
    WICED_BT_TRACE("[%s] conn %d type %d state 0x%02x\n", __FUNCTION__, p_data->conn_id, p_data->discovery_type, anc_app_state.discovery_state);
    uint16_t alert_service_uuid = UUID_SERVICE_ALERT_NOTIFICATION;
    switch (anc_app_state.discovery_state)
    {
    case ANC_DISCOVERY_STATE_ANC:
        wiced_bt_anc_discovery_result(p_data);
        break;

    default:
        if (p_data->discovery_type  == GATT_DISCOVER_SERVICES_ALL)
        {
            if (p_data->discovery_data.group_value.service_type.len == LEN_UUID_16 )
            {
                WICED_BT_TRACE("%04x e:%04x uuid\n", p_data->discovery_data.group_value.s_handle, p_data->discovery_data.group_value.e_handle);
                if (memcmp(&p_data->discovery_data.group_value.service_type.uu, &alert_service_uuid, LEN_UUID_16) == 0)
                {
                    WICED_BT_TRACE("ANC Service found s:%04x e:%04x\n",
                            p_data->discovery_data.group_value.s_handle,
                            p_data->discovery_data.group_value.e_handle);
                    anc_app_state.anc_s_handle = p_data->discovery_data.group_value.s_handle;
                    anc_app_state.anc_e_handle = p_data->discovery_data.group_value.e_handle;
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

static wiced_bt_gatt_status_t anc_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data)
{
    wiced_result_t result;

    WICED_BT_TRACE("[%s] conn %d type %d state %d\n", __FUNCTION__, p_data->conn_id, p_data->disc_type, anc_app_state.discovery_state);

    switch (anc_app_state.discovery_state)
    {
    case ANC_DISCOVERY_STATE_ANC:
        wiced_bt_anc_client_discovery_complete(p_data);
        break;

    default:
        if (p_data->disc_type == GATT_DISCOVER_SERVICES_ALL)
        {
            WICED_BT_TRACE("ANC:%04x-%04x\n", anc_app_state.anc_s_handle, anc_app_state.anc_e_handle);

            /* If anc Service found tell WICED BT anc library to start its discovery */
            if ((anc_app_state.anc_s_handle != 0) && (anc_app_state.anc_e_handle != 0))
            {
                anc_app_state.discovery_state = ANC_DISCOVERY_STATE_ANC;
                if (wiced_bt_anc_discover(anc_app_state.conn_id, anc_app_state.anc_s_handle, anc_app_state.anc_e_handle))
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
 * Pass write response to appropriate client based on the attribute handle
 */
static void anc_process_write_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
    WICED_BT_TRACE("anc_process_write_rsp handle:%04x\n", p_data->response_data.handle);

    // Verify that write response is for our service
    if ((p_data->response_data.handle >= anc_app_state.anc_s_handle) &&
        (p_data->response_data.handle <= anc_app_state.anc_e_handle))
    {
        wiced_bt_anc_write_rsp(p_data);
    }
}

/*
 * Pass read response to appropriate client based on the attribute handle
 */
static void anc_process_read_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
    WICED_BT_TRACE("anc_process_read_rsp handle:%04x\n", p_data->response_data.handle);

    // Verify that write response is for our service
    if ((p_data->response_data.handle >= anc_app_state.anc_s_handle) &&
        (p_data->response_data.handle <= anc_app_state.anc_e_handle))
    {
        wiced_bt_anc_read_rsp(p_data);
    }
}

/*
 * Pass notification to appropriate client based on the attribute handle
 */
static void anc_notification_handler(wiced_bt_gatt_operation_complete_t *p_data)
{
    WICED_BT_TRACE("notification handle:%04x\n", p_data->response_data.att_value.handle);

    // Verify that notification is for ANCS service and if true pass to the library for processing
    if ((p_data->response_data.att_value.handle >= anc_app_state.anc_s_handle) &&
        (p_data->response_data.att_value.handle < anc_app_state.anc_e_handle))
    {
        wiced_bt_anc_client_process_notification(p_data);
    }
}

static void clear_anc_pending_cmd_context (void)
{
    WICED_BT_TRACE ("%s \n", __FUNCTION__);
    memset(anc_pending_cmd_context, 0, sizeof(anc_pending_cmd_context));
}

static void anc_trigger_pending_action (void)
{
    wiced_bt_gatt_status_t  gatt_status = WICED_BT_GATT_SUCCESS;
    if (!anc_pending_cmd_context[0])
    {
        WICED_BT_TRACE(" anc_trigger_pending_action No commands pending! \n");
        return;
    }

    switch (( HCI_CONTROL_GROUP_ANC << 8 ) | anc_pending_cmd_context[0])
    {
    case HCI_CONTROL_ANC_COMMAND_READ_SERVER_SUPPORTED_NEW_ALERTS:
        gatt_status = wiced_bt_anc_read_server_supported_new_alerts( anc_app_state.conn_id );
        break;

    case HCI_CONTROL_ANC_COMMAND_READ_SERVER_SUPPORTED_UNREAD_ALERTS:
        gatt_status = wiced_bt_anc_read_server_supported_unread_alerts(anc_app_state.conn_id);
        break;

    case HCI_CONTROL_ANC_COMMAND_CONTROL_ALERTS:
        gatt_status = wiced_bt_anc_control_required_alerts( anc_app_state.conn_id, anc_pending_cmd_context[1], anc_pending_cmd_context[2] );
        break;

    case HCI_CONTROL_ANC_COMMAND_ENABLE_NEW_ALERTS:
        gatt_status = wiced_bt_anc_enable_new_alerts( anc_app_state.conn_id );
        break;

    case HCI_CONTROL_ANC_COMMAND_ENABLE_UNREAD_ALERTS:
        gatt_status = wiced_bt_anc_enable_unread_alerts( anc_app_state.conn_id );
        break;

    case HCI_CONTROL_ANC_COMMAND_DISABLE_NEW_ALERTS:
        gatt_status = wiced_bt_anc_disable_new_alerts( anc_app_state.conn_id );
        break;

    case HCI_CONTROL_ANC_COMMAND_DISABLE_UNREAD_ALERTS:
        gatt_status = wiced_bt_anc_disable_unread_alerts( anc_app_state.conn_id );
        break;
    default:
        WICED_BT_TRACE("unkknown pending HCI command \n");
        break;
    }

    if (gatt_status != WICED_BT_GATT_SUCCESS)
    {
        WICED_BT_TRACE("anc_trigger_pending_action %d \n", gatt_status);
    }

    /* should not keep trying if fail in sending pending command */
    clear_anc_pending_cmd_context();
}

static void anc_callback(wiced_bt_anc_event_t event, wiced_bt_anc_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

    switch(event)
    {
        case WICED_BT_ANC_DISCOVER_RESULT:
            WICED_BT_TRACE( "ANC discover result: %d ", p_data->discovery_result.status );
            result = p_data->discovery_result.status;
            if (result == WICED_BT_GATT_SUCCESS)
            {
                hci_control_send_anc_enabled();
            }
            break;

        case WICED_BT_ANC_READ_SUPPORTED_NEW_ALERTS_RESULT:
            WICED_BT_TRACE( "ANC read supported new alerts: %d ", p_data->supported_new_alerts_result.status );
            result = p_data->supported_new_alerts_result.status;
            if (result != WICED_BT_GATT_INSUF_AUTHENTICATION)
            {
                hci_control_send_supported_new_alerts( p_data->supported_new_alerts_result.status,
                        p_data->supported_new_alerts_result.conn_id,
                        (uint8_t*)&p_data->supported_new_alerts_result.supported_alerts );
            }
            break;

        case WICED_BT_ANC_READ_SUPPORTED_UNREAD_ALERTS_RESULT:
            WICED_BT_TRACE( "ANC read supported unread alerts: %d ", p_data->supported_unread_alerts_result.status );
            result = p_data->supported_unread_alerts_result.status;
            if (p_data->supported_unread_alerts_result.status != WICED_BT_GATT_INSUF_AUTHENTICATION)
            {
                hci_control_send_supported_unread_alerts( p_data->supported_unread_alerts_result.status,
                        p_data->supported_unread_alerts_result.conn_id,
                        (uint8_t*)&p_data->supported_unread_alerts_result.supported_alerts);
            }
            break;

        case WICED_BT_ANC_CONTROL_ALERTS_RESULT:
            result = p_data->control_alerts_result.status;
            WICED_BT_TRACE( "ANC control alerts result: %d ", p_data->control_alerts_result.status );
            if (result != WICED_BT_GATT_INSUF_AUTHENTICATION)
            {
                hci_control_send_control_alerts_result( p_data->control_alerts_result.status,
                        p_data->control_alerts_result.conn_id,
                        p_data->control_alerts_result.control_point_cmd_id,
                        p_data->control_alerts_result.category_id);
            }
            break;

        case WICED_BT_ANC_ENABLE_NEW_ALERTS_RESULT:
            WICED_BT_TRACE( "ANC enable new alerts result: %d ", p_data->enable_disable_alerts_result.status );
            result = p_data->enable_disable_alerts_result.status;
            if (result != WICED_BT_GATT_INSUF_AUTHENTICATION)
            {
                hci_control_send_enable_new_alerts_result( p_data->enable_disable_alerts_result.status,
                        p_data->enable_disable_alerts_result.conn_id );
            }
            break;

        case WICED_BT_ANC_DISABLE_NEW_ALERTS_RESULT:
            WICED_BT_TRACE( "ANC disable new alerts result: %d ", p_data->enable_disable_alerts_result.status );
            result = p_data->enable_disable_alerts_result.status;
            if (result != WICED_BT_GATT_INSUF_AUTHENTICATION)
            {
                hci_control_send_disable_new_alerts_result( p_data->enable_disable_alerts_result.status,
                        p_data->enable_disable_alerts_result.conn_id );
            }
            break;

        case WICED_BT_ANC_ENABLE_UNREAD_ALERTS_RESULT:
            WICED_BT_TRACE( "ANC enable unread alerts result: %d ", p_data->enable_disable_alerts_result.status );
            result = p_data->enable_disable_alerts_result.status;
            if (result != WICED_BT_GATT_INSUF_AUTHENTICATION)
            {
                hci_control_send_enable_unread_alerts_result( p_data->enable_disable_alerts_result.status,
                        p_data->enable_disable_alerts_result.conn_id );
            }
            break;

        case WICED_BT_ANC_DISABLE_UNREAD_ALERTS_RESULT:
            WICED_BT_TRACE( "ANC enable unread alerts result: %d ", p_data->enable_disable_alerts_result.status );
            result = p_data->enable_disable_alerts_result.status;
            if (result != WICED_BT_GATT_INSUF_AUTHENTICATION)
            {
                hci_control_send_disable_unread_alerts_result( p_data->enable_disable_alerts_result.status,
                        p_data->enable_disable_alerts_result.conn_id );
            }
            break;

        case WICED_BT_ANC_EVENT_NEW_ALERT_NOTIFICATION:
            WICED_BT_TRACE("New Alert type:%s Count:%d Last Alert Data:%s\n",
                alert_type_name(p_data->new_alert_notification.new_alert_type),
                p_data->new_alert_notification.new_alert_count,
                p_data->new_alert_notification.p_last_alert_data);
            break;

        case WICED_BT_ANC_EVENT_UNREAD_ALERT_NOTIFICATION:
            WICED_BT_TRACE("Unread Alert type: %s Count: %d \n",
                alert_type_name(p_data->unread_alert_notification.unread_alert_type),
                p_data->unread_alert_notification.unread_count);
            break;

        default:
            break;
    }
    if ( result == WICED_BT_GATT_INSUF_AUTHENTICATION )
    {
#ifndef TEST_HCI_CONTROL
        c=0;
#endif
        anc_start_pair();
    }
    else
    {
        /* pending command no more valid other than authentication failure cases */
        clear_anc_pending_cmd_context();
    }
}

/* Returns Alert name for the given alert type */
const char *alert_type_name (wiced_bt_anp_alert_category_id_t id)
{
    switch (id)
    {
    case ANP_ALERT_CATEGORY_ID_SIMPLE_ALERT:
        return "simple_alert";
    break;

    case ANP_ALERT_CATEGORY_ID_EMAIL:
        return "Email";
    break;

    case ANP_ALERT_CATEGORY_ID_NEWS:
        return "News";
    break;

    case ANP_ALERT_CATEGORY_ID_CALL:
        return "Call";
    break;

    case ANP_ALERT_CATEGORY_ID_MISSED_CALL:
        return "Missed Call";
    break;

    case ANP_ALERT_CATEGORY_ID_SMS_OR_MMS:
        return "SMS/MMS";
    break;

    case ANP_ALERT_CATEGORY_ID_VOICE_MAIL:
        return "Voice Mail";
    break;

    case ANP_ALERT_CATEGORY_ID_SCHEDULE_ALERT:
        return "Scheduled Alert";
    break;

    case ANP_ALERT_CATEGORY_ID_HIGH_PRI_ALERT:
        return "High Pri Alert";
    break;

    case ANP_ALERT_CATEGORY_ID_INSTANT_MESSAGE:
        return "Instant Message";
    break;
    }

    return NULL;
}


#ifdef TEST_HCI_CONTROL

/* transport status */
void anc_transport_status( wiced_transport_type_t type )
{
    WICED_BT_TRACE(" anc_transport connected type: %d ", type);
}

void ans_start_advertisements (void)
{
    /* Start adv on opening transport */
    wiced_result_t status =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    WICED_BT_TRACE("wiced_bt_start_advertisements %d\n", status);
}

/*
 * Handle received command over UART. Please refer to the WICED HCI Control
 * Protocol for details on the protocol.  The function converts from the WICED
 * HCI remote control commands to the commands expected by the AMS.
 */
uint32_t  anc_proc_rx_hci_cmd(uint8_t *p_buffer, uint32_t length)
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
    if (anc_app_state.conn_id == 0)
    {
        STREAM_TO_UINT16(opcode, p_data);       // Get OpCode
        if(opcode == HCI_CONTROL_MISC_COMMAND_GET_VERSION)
        {
            WICED_BT_TRACE("HCI_CONTROL_MISC_COMMAND_GET_VERSION\n");
            anc_handle_get_version();
            ans_start_advertisements();
        }
        else
        {
            WICED_BT_TRACE("no connection\n");
            status = HCI_CONTROL_STATUS_NOT_CONNECTED;
        }
    }
    else if (anc_pending_cmd_context[0])
    {
        /* previous command is not yet completed. Hold on until complete */
        WICED_BT_TRACE("ANC busy with previous command \n");
        status = HCI_CONTROL_STATUS_DISALLOWED;
    }
    else
    {
        STREAM_TO_UINT16(opcode, p_data);       // Get OpCode
        STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length
        WICED_BT_TRACE("cmd_opcode 0x%02x payload_len %d \n", opcode, payload_len);

        anc_pending_cmd_context[0] = opcode & 0xff;
        switch (opcode)
        {
        case HCI_CONTROL_ANC_COMMAND_READ_SERVER_SUPPORTED_NEW_ALERTS:
            gatt_status = wiced_bt_anc_read_server_supported_new_alerts( anc_app_state.conn_id );
            break;

        case HCI_CONTROL_ANC_COMMAND_READ_SERVER_SUPPORTED_UNREAD_ALERTS:
            gatt_status = wiced_bt_anc_read_server_supported_unread_alerts(anc_app_state.conn_id);
            break;

        case HCI_CONTROL_ANC_COMMAND_CONTROL_ALERTS:
            anc_pending_cmd_context[1] = p_data[0];
            anc_pending_cmd_context[2] = p_data[1];
            gatt_status = wiced_bt_anc_control_required_alerts( anc_app_state.conn_id, p_data[0], p_data[1] );
            break;

        case HCI_CONTROL_ANC_COMMAND_ENABLE_NEW_ALERTS:
            gatt_status = wiced_bt_anc_enable_new_alerts( anc_app_state.conn_id );
            break;

        case HCI_CONTROL_ANC_COMMAND_ENABLE_UNREAD_ALERTS:
            gatt_status = wiced_bt_anc_enable_unread_alerts( anc_app_state.conn_id );
            break;

        case HCI_CONTROL_ANC_COMMAND_DISABLE_NEW_ALERTS:
            gatt_status = wiced_bt_anc_disable_new_alerts( anc_app_state.conn_id );
            break;

        case HCI_CONTROL_ANC_COMMAND_DISABLE_UNREAD_ALERTS:
            gatt_status = wiced_bt_anc_disable_unread_alerts( anc_app_state.conn_id );
            break;

       case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
            anc_handle_get_version();
            break;

        default:
            WICED_BT_TRACE("ignored\n", opcode);
            status = HCI_CONTROL_STATUS_UNKNOWN_COMMAND;
            break;
        }
    }

    if (gatt_status != WICED_BT_GATT_SUCCESS)
    {
        /* pending command no more valid other than authentication failure cases */
        clear_anc_pending_cmd_context();
        status = HCI_CONTROL_STATUS_FAILED;
    }
    wiced_transport_send_data(HCI_CONTROL_ANC_EVENT_COMMAND_STATUS, &status, 1);

    // Freeing the buffer in which data is received
    wiced_transport_free_buffer(p_buffer);
    return HCI_CONTROL_STATUS_SUCCESS;
}
#endif

void anc_start_pair(void)
{
    wiced_result_t rc;

    rc = wiced_bt_dev_sec_bond(anc_app_state.remote_addr, anc_app_state.addr_type, BT_TRANSPORT_LE, 0, NULL);
    WICED_BT_TRACE("start bond result:%d\n", rc);
}

/*
 * Read keys from the NVRAM and update address resolution database
 */
void anc_load_keys_to_addr_resolution_db(void)
{
    uint8_t                     bytes_read;
    wiced_result_t              result;
    wiced_bt_device_link_keys_t keys;

    bytes_read = wiced_hal_read_nvram(ANC_PAIRED_KEYS_NVRAM_ID, sizeof(keys), (uint8_t *)&keys, &result);

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
wiced_bool_t anc_save_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    uint8_t          bytes_written;
    wiced_result_t   result;

    bytes_written = wiced_hal_write_nvram(ANC_PAIRED_KEYS_NVRAM_ID, sizeof(wiced_bt_device_link_keys_t), (uint8_t *)p_keys, &result);
    WICED_BT_TRACE("Saved %d bytes at id:%d \n", bytes_written, ANC_PAIRED_KEYS_NVRAM_ID);
    return (bytes_written == sizeof (wiced_bt_device_link_keys_t));
}

/*
 * This function is called to read keys for specific bdaddr
 */
wiced_bool_t anc_read_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    uint8_t         bytes_read;
    wiced_result_t  result;

    bytes_read = wiced_hal_read_nvram(ANC_PAIRED_KEYS_NVRAM_ID, sizeof(wiced_bt_device_link_keys_t), (uint8_t *)p_keys, &result);
    WICED_BT_TRACE("read %d bytes at id:%d \n", bytes_read, ANC_PAIRED_KEYS_NVRAM_ID);
    return (bytes_read == sizeof (wiced_bt_device_link_keys_t));
}

#ifdef HCI_TRACE_OVER_TRANSPORT
/*
 *  Pass protocol traces up over the transport
 */
void anc_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    wiced_transport_send_hci_trace(host_trans_pool, type, length, p_data);
}
#endif

/*
 * This utility copies a character string to another
 */
char *utl_strcpy(char *p_dst, char *p_src)
{
    register char *pd = p_dst;
    register char *ps = p_src;

    while (*ps)
        *pd++ = *ps++;

    *pd++ = 0;

    return (p_dst);
}

static void hci_control_send_anc_enabled( void )
{
    WICED_BT_TRACE( "[%s] \n", __FUNCTION__ );

#ifdef TEST_HCI_CONTROL
    wiced_transport_send_data(HCI_CONTROL_ANC_EVENT_ANC_ENABLED, NULL, 0);
#endif
}

static void hci_control_send_anc_disabled( void )
{
    WICED_BT_TRACE( "[%s] \n", __FUNCTION__ );

#ifdef TEST_HCI_CONTROL
    wiced_transport_send_data(HCI_CONTROL_ANC_EVENT_ANC_DISABLED, NULL, 0);
#endif
}
static void hci_control_send_supported_new_alerts( uint8_t status, uint16_t conn_id, uint8_t *p_data )
{
    uint8_t event_data[3];

    WICED_BT_TRACE( "[%s] conn_id %04x\n", __FUNCTION__, conn_id );

    //Build event payload
    event_data[0] = status;
    event_data[1] = p_data[0];
    event_data[2] = p_data[1];
#ifdef TEST_HCI_CONTROL
    wiced_transport_send_data(HCI_CONTROL_ANC_EVENT_SERVER_SUPPORTED_NEW_ALERTS, event_data, 3);
#else
    WICED_BT_TRACE(" %x \n", (p_data[0] + (p_data[1]<<8)));
#endif
}

static void hci_control_send_supported_unread_alerts( uint8_t status, uint16_t conn_id, uint8_t *p_data )
{
    uint8_t event_data[3];

    WICED_BT_TRACE( "[%s] conn_id %04x\n", __FUNCTION__, conn_id );

    //Build event payload
    event_data[0] = status;
    event_data[1] = p_data[0];
    event_data[2] = p_data[1];
#ifdef TEST_HCI_CONTROL
    wiced_transport_send_data(HCI_CONTROL_ANC_EVENT_SERVER_SUPPORTED_UNREAD_ALERTS, event_data, 3);
#else
    WICED_BT_TRACE(" %x \n", (p_data[0] + (p_data[1]<<8)));
#endif
}

static void hci_control_send_control_alerts_result( uint8_t status, uint16_t conn_id, uint8_t cmd_id, uint8_t category_id )
{
    uint8_t event_data[3];

    WICED_BT_TRACE( "[%s] conn_id %04x\n", __FUNCTION__, conn_id );

    //Build event payload
    event_data[0] = status;
    event_data[1] = cmd_id;
    event_data[2] = category_id;
#ifdef TEST_HCI_CONTROL
    wiced_transport_send_data(HCI_CONTROL_ANC_EVENT_CONTROL_ALERTS, event_data, 3);
#endif
}

static void hci_control_send_enable_new_alerts_result( uint8_t status, uint16_t conn_id )
{
    WICED_BT_TRACE( "[%s] conn_id %04x\n", __FUNCTION__, conn_id );

#ifdef TEST_HCI_CONTROL
    wiced_transport_send_data(HCI_CONTROL_ANC_EVENT_ENABLE_NEW_ALERTS, &status, 1);
#endif
}

static void hci_control_send_disable_new_alerts_result( uint8_t status, uint16_t conn_id )
{
    WICED_BT_TRACE( "[%s] conn_id %04x\n", __FUNCTION__, conn_id );

#ifdef TEST_HCI_CONTROL
    wiced_transport_send_data(HCI_CONTROL_ANC_EVENT_DISABLE_NEW_ALERTS, &status, 1);
#endif
}

static void hci_control_send_enable_unread_alerts_result(uint8_t status, uint16_t conn_id)
{
    WICED_BT_TRACE( "[%s] conn_id %04x\n", __FUNCTION__, conn_id );
#ifdef TEST_HCI_CONTROL
    wiced_transport_send_data(HCI_CONTROL_ANC_EVENT_ENABLE_UNREAD_ALERTS, &status, 1);
#endif
}
static void hci_control_send_disable_unread_alerts_result(uint8_t status, uint16_t conn_id)
{
    WICED_BT_TRACE( "[%s] conn_id %04x\n", __FUNCTION__, conn_id );

#ifdef TEST_HCI_CONTROL
    wiced_transport_send_data(HCI_CONTROL_ANC_EVENT_DISABLE_UNREAD_ALERTS, &status, 1);
#endif
}

#ifndef TEST_HCI_CONTROL
void anc_app_timeout( uint32_t arg )
{
    anc_app_timer_count++;
    WICED_BT_TRACE("%d \n", anc_app_timer_count);
}

wiced_bool_t test_enable_category(uint8_t cat)
{
    wiced_bt_gatt_status_t status;

    wiced_bt_anp_alert_control_cmd_id_t cmd = ANP_ALERT_CONTROL_CMD_ENABLE_NEW_ALERTS;

    if (c == 0)
    {
        status = wiced_bt_anc_control_required_alerts(anc_app_state.conn_id, cmd, cat);
        WICED_BT_TRACE("test_enable_category email(new alerts):%x \n", status);
        if (!status) c++;
    }
    else
    {
        cmd = ANP_ALERT_CONTROL_CMD_ENABLE_UNREAD_STATUS;
        status = wiced_bt_anc_control_required_alerts(anc_app_state.conn_id, cmd, cat);
        WICED_BT_TRACE("test_enable_category email(unread alerts):%x \n", status);
        if (!status)
        {
            c = 0;
            return WICED_TRUE;
        }
    }

    return WICED_FALSE;
}

wiced_bool_t test_enable_all_category(void)
{
    wiced_bt_gatt_status_t status;

    wiced_bt_anp_alert_control_cmd_id_t cmd = ANP_ALERT_CONTROL_CMD_ENABLE_NEW_ALERTS;
    wiced_bt_anp_alert_category_id_t    cat = ANP_ALERT_CATEGORY_ID_ALL_CONFIGURED;

    if (c == 0)
    {
        status = wiced_bt_anc_control_required_alerts(anc_app_state.conn_id, cmd, cat);
        WICED_BT_TRACE("test_enable_all_category , new alerts:%x \n", status);
        if (!status)c++;
    }
    else
    {
        cmd = ANP_ALERT_CONTROL_CMD_ENABLE_UNREAD_STATUS;
        status = wiced_bt_anc_control_required_alerts(anc_app_state.conn_id, cmd, cat);
        WICED_BT_TRACE("test_enable_all_category , unread alerts:%x \n", status);
        if (!status)
        {
            c=0;
            return WICED_TRUE;
        }
    }
    return WICED_FALSE;
}

wiced_bool_t test_disable_category(uint8_t cat)
{
    wiced_bt_gatt_status_t status;

    wiced_bt_anp_alert_control_cmd_id_t cmd = ANP_ALERT_CONTROL_CMD_DISABLE_NEW_ALERTS;

    if (c == 0)
    {
        status = wiced_bt_anc_control_required_alerts(anc_app_state.conn_id, cmd, cat);
        WICED_BT_TRACE("test_disable_category simple alert(new alerts):%x \n", status);
        if (!status)c++;
    }
    else
    {
        cmd = ANP_ALERT_CONTROL_CMD_DISABLE_UNREAD_ALERTS;
        status = wiced_bt_anc_control_required_alerts(anc_app_state.conn_id, cmd, cat);
        WICED_BT_TRACE("test_disable_category simple alert(unread alerts):%x \n", status);
        if (!status)
        {
            c = 0;
            return WICED_TRUE;
        }
    }
    return WICED_FALSE;
}

wiced_bool_t test_disable_all_category(void)
{
    wiced_bt_gatt_status_t status;

    wiced_bt_anp_alert_control_cmd_id_t cmd = ANP_ALERT_CONTROL_CMD_DISABLE_NEW_ALERTS;
    wiced_bt_anp_alert_category_id_t    cat = ANP_ALERT_CATEGORY_ID_ALL_CONFIGURED;
    if (c == 0)
    {
        status = wiced_bt_anc_control_required_alerts(anc_app_state.conn_id, cmd, cat);
        WICED_BT_TRACE("test_disable_all_category , new alerts:%x \n", status);
        if (!status) c++;
    }
    else
    {
        cmd = ANP_ALERT_CONTROL_CMD_DISABLE_UNREAD_ALERTS;
        status = wiced_bt_anc_control_required_alerts(anc_app_state.conn_id, cmd, cat);
        WICED_BT_TRACE("test_disable_all_category , unread alerts:%x \n", status);
        if (!status)
        {
            c = 0;
            return WICED_TRUE;
        }
    }
    return WICED_FALSE;
}

wiced_bool_t test_notify_immediete_category(uint8_t cat)
{
    wiced_bt_gatt_status_t status;

    wiced_bt_anp_alert_control_cmd_id_t cmd = ANP_ALERT_CONTROL_CMD_NOTIFY_NEW_ALERTS_IMMEDIATE;

    if (c == 0)
    {
        status = wiced_bt_anc_control_required_alerts(anc_app_state.conn_id, cmd, cat);
        WICED_BT_TRACE("test_notify_immediete_category , SMS/MMS(new alerts):%x \n", status);
        if(!status) c++;
    }
    else
    {
        cmd = ANP_ALERT_CONTROL_CMD_NOTIFY_UNREAD_ALERTS_IMMEDIATE;
        status = wiced_bt_anc_control_required_alerts(anc_app_state.conn_id, cmd, cat);
        WICED_BT_TRACE("test_notify_immediete_category , SMS/MMS(unread alerts):%x \n", status);
        if (!status)
        {
            c = 0;
            return WICED_TRUE;
        }
    }
    return WICED_FALSE;
}

wiced_bool_t test_notify_immediete_all_category(void)
{
    wiced_bt_gatt_status_t status;

    wiced_bt_anp_alert_control_cmd_id_t cmd = ANP_ALERT_CONTROL_CMD_NOTIFY_NEW_ALERTS_IMMEDIATE;
    wiced_bt_anp_alert_category_id_t    cat = ANP_ALERT_CATEGORY_ID_ALL_CONFIGURED;

    if (c == 0)
    {
        status = wiced_bt_anc_control_required_alerts(anc_app_state.conn_id, cmd, cat);
        WICED_BT_TRACE("test_notify_immediete_all_category , new alerts:%x \n", status);
        if (!status) c++;
    }
    else
    {
        cmd = ANP_ALERT_CONTROL_CMD_NOTIFY_UNREAD_ALERTS_IMMEDIATE;
        status = wiced_bt_anc_control_required_alerts(anc_app_state.conn_id, cmd, cat);
        WICED_BT_TRACE("test_notify_immediete_all_category , unread alerts:%x \n", status);
        if (!status)
        {
            c = 0;
            return WICED_TRUE;
        }
    }
    return WICED_FALSE;
}

void anc_interrupt_handler(void* user_data, uint8_t value )
{
    wiced_result_t  result;
    uint8_t cat = 0;

    static uint32_t button_pushed_time = 0;
    static uint32_t tc=0;
    static uint32_t enable=1;
    static uint8_t tc2 = 0;

#if defined(CYW20819A1)
    if ( wiced_hal_gpio_get_pin_input_status(APP_BUTTON) == wiced_platform_get_button_pressed_value(WICED_PLATFORM_BUTTON_1))

#else
    if ( wiced_hal_gpio_get_pin_input_status(APP_BUTTON) == WICED_BUTTON_PRESSED_VALUE )
#endif
    {
        WICED_BT_TRACE( " Button pressed\n" );
        button_pushed_time = anc_app_timer_count;
    }
    else if ( button_pushed_time != 0 )
    {
        //calculate button pressed time
        uint32_t pt = anc_app_timer_count - button_pushed_time;

        WICED_BT_TRACE( " Button released " );

        if (pt > 2)
        {
            /* In below to move tc to next value(i.e to increment tc), user has to press button more than two seconds, 2 times(first time new alerts, second time unread alerts) */
            if (tc == 0)
            {
                if (test_enable_category(ANP_ALERT_CATEGORY_ID_EMAIL)) /* After this step, Gneerate Alert in server and makesure you receive only email. */
                    tc++;
            }
            else if (tc == 1)
            {
                if ( test_enable_all_category()) /* After this step, Generate Alerts in server and makesure you receive all new alerts and unread alerts */
                    tc++;
            }                                    /* Before move to next step, once verified all alerts recevd from server, we can disable CCCD(press button less than 2 seconds 2 times) and make sure we did not recv alerts from server. After that enable CCCD (press less than 2 second s2 times) */
            else if(tc == 2)
            {
                if (test_disable_category(ANP_ALERT_CATEGORY_ID_SIMPLE_ALERT)) /* After this step, Gneerate Alerts in server and makesure you did not recvd simple alerts */
                    tc++;
            }
            else if (tc == 3)
            {
                if (test_enable_category(ANP_ALERT_CATEGORY_ID_SIMPLE_ALERT)) /* After this step, Gneerate Alerts in server and makesure you should recv simple alerts */
                    tc++;
            }
            else if(tc == 4)
            {
                if ( test_disable_all_category()) /* After this step, Gneerate Alerts in server and makesure you should not recv alerts */
                    tc++;
            }
            else if (tc == 5)
            {
                if ( test_notify_immediete_category(ANP_ALERT_CATEGORY_ID_SIMPLE_ALERT)) /* After this step, makesure you should not receive simple alert, because it is not enabled */
                    tc++;
            }
            else if(tc == 6)
            {
                if ( test_enable_all_category()) /* After this step, no user action required at server side */
                    tc++;
            }
            else if (tc == 7)
            {
                if (test_notify_immediete_all_category()) /* After this step, all pending new alerts and unread alerts should be received  No user action required at server side */
                    tc++;
            }
            else if (tc == 8)
            {
                if (test_notify_immediete_all_category()) // To make sure server should not attempt to notify again, becase we received all pending notiifcations in tc == 7 condition
                    tc++;
            }
            else if (tc == 9)
            {
                if (test_disable_all_category())
                    tc = 0; // to repeat above 6 steps
            }
        }
        else
        {
            if (anc_app_state.conn_id)
            {
                wiced_bt_gatt_status_t status;
                if (tc2 < 2)
                {
                        if (tc2 == 0)
                        {
                            status = wiced_bt_anc_read_server_supported_new_alerts( anc_app_state.conn_id );
                            WICED_BT_TRACE("read server supported na: %x \n", status);
                            tc2++;
                        }
                        else
                        {
                            status = wiced_bt_anc_read_server_supported_unread_alerts(anc_app_state.conn_id);
                            WICED_BT_TRACE("read server supported ua: %x \n", status);
                            if (status == WICED_BT_GATT_SUCCESS)
                            {
                                tc2++;
                            }
                        }
                }
                else if (tc2 >= 2)
                {
                    if (enable)
                    {
                        if (tc2 == 2)
                        {
                            status = wiced_bt_anc_enable_new_alerts( anc_app_state.conn_id );
                            WICED_BT_TRACE("enable new alerts:%x \n", status);
                            tc2++;
                        }
                        else
                        {
                            status = wiced_bt_anc_enable_unread_alerts( anc_app_state.conn_id );
                            WICED_BT_TRACE("enable unread alerts:%x \n", status);
                            if (status == WICED_BT_GATT_SUCCESS){
                                enable = 0;
                                tc2 = 2;
                            }
                        }
                    }
                    else
                    {
                        if (tc2 == 2)
                        {
                            status = wiced_bt_anc_disable_new_alerts( anc_app_state.conn_id );
                            WICED_BT_TRACE("disable new alerts:%x \n", status);
                            tc2++;
                        }
                        else
                        {
                            status = wiced_bt_anc_disable_unread_alerts( anc_app_state.conn_id );
                            WICED_BT_TRACE("disable unread alerts:%x \n", status);
                            if (status == WICED_BT_GATT_SUCCESS)
                            {
                                enable = 1;
                                tc2=2;
                            }
                        }
                    }
                }
            }
            else
            {
                //start ADV to reconnect, if not connected and ADV is off
                if (wiced_bt_ble_get_current_advert_mode() == BTM_BLE_ADVERT_OFF)
                {
                    wiced_result_t result;
                    result = wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
                    WICED_BT_TRACE( "wiced_bt_start_advertisements:%d\n", result );
                }
            }
        }
    }
}
#endif

#ifdef TEST_HCI_CONTROL
/*
 * Handle the GET_VERSION command
 */
void anc_handle_get_version(void)
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
    tx_buf[cmd++] = HCI_CONTROL_GROUP_ANC;

    wiced_transport_send_data(HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd);

}
#endif
