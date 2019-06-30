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
* Heart Rate Server (HRS) snippet application
*
* The HRS snippet application shows how to initialize and use WICED BT Heart Rate
* Server library.This snippet implements GAP peripheral role. On application start,
* calls HRS library APIs to register callbacks for receiving HRC requests. HRS allows a client
* to register/un-register for notifications and to reset Energy expended values in the server.

* Once application receive BTM_ENABLED event, it loads bonded keys and enters
* Limited GAP ADV mode.i.e. enters high duty GAP ADV mode for 30 seconds, after which it
* switches to Low duty GAP ADV mode and remains in this mode for 30 seconds. If client(HRC)
* does not attempt connection with in timeout, application exits from ADV mode.
* Application includes Heart Rate Service UUID in its ADV data to get connected by HRC.
* User has to push the button once to enter to Limited GAP ADV mode whenever HRC required to connect.
* After connection establishes with HRC, application calls Library APIs to inform GATT connection status,
* to process HRC GATT read, write requests for Heart Rate notifications.
* Application is notified by library whenever HRC configures heart rate notifications.
* Application stores bonded HRC address and notification configuration in NVRAM. This information is
* used in reconnection, after successful encryption, to start heart rate notifications automatically.
* Application sends heart rate notifications to HRC on every 1 minute, until HRC stops.

* Features demonstrated
*  - Initialize and use WICED BT HRS library
*  - GATTDB with Heart Rate notification service and characteristics, Device Information Service and characteristics
*
* To demonstrate the app, work through the following steps.
* 1. Plug the WICED eval board into your computer
* 2. Build and download the application (to the WICED board)
* 3. Start tracing to monitor the activity (see Kit Guide for details)
* 4. Application automatically enter to Limited GAP ADV mode.
* 4. After connection with HRC, based on HRC request, start, stop Heart Rate notifications and
*    resets energy expended value accumulated during Heart Rate Notification since last HRC Energy expended reset request.
* 5. If heart rate client not connected and HRS exited GAP ADV mode, push the application button to enter Limited GAP ADV mode.
*/

#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_gatt_util.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_uuid.h"
#if !defined(CYW20735B1) && !defined(CYW20819A1) && !defined(CYW20719B2) && !defined(CYW20721B2)
#include "wiced_bt_app_common.h"
#endif
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"
#include "wiced_result.h"
#include "GeneratedSource/cycfg_gatt_db.h"
#include "wiced_transport.h"
#include "wiced_timer.h"
#include "wiced_bt_hrp.h"
#include "wiced_bt_hrs.h"
#include "wiced_bt_uuid.h"
#include "wiced_hal_puart.h"
#include "string.h"
#include "wiced_bt_stack.h"

/******************************************************
 *                      Constants
 ******************************************************/
extern const wiced_bt_cfg_settings_t wiced_app_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_app_cfg_buf_pools[];

#ifdef CYW20706A2
#define APP_BUTTON_SETTINGS         (WICED_GPIO_BUTTON_SETTINGS( GPIO_EN_INT_RISING_EDGE ))
#define APP_BUTTON_DEFAULT_STATE    WICED_GPIO_BUTTON_DEFAULT_STATE
#endif
#if ( defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20735B1) || defined(CYW20819A1) )
#define APP_BUTTON                  WICED_GPIO_PIN_BUTTON_1
#endif

#if ( defined(CYW20735B0) || defined(CYW20706A2) )
#define APP_BUTTON                  WICED_GPIO_PIN_BUTTON
#endif

#if ( defined(CYW20735B0) || defined(CYW20719B0) )
#define APP_BUTTON_SETTINGS         WICED_GPIO_BUTTON_SETTINGS
#define APP_BUTTON_DEFAULT_STATE    GPIO_PIN_OUTPUT_LOW
#endif

#define HRS_HOST_INFO_VS_ID      WICED_NVRAM_VSID_START
#define HRS_LOCAL_KEYS_VS_ID     (WICED_NVRAM_VSID_START + 1)
#define HRS_PAIRED_KEYS_VS_ID    (WICED_NVRAM_VSID_START + 2)

/******************************************************
 *                     Structures
 ******************************************************/

/******************************************************
 *               Function Prototypes
 ******************************************************/
static wiced_result_t         hrs_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static void                   hrs_set_advertisement_data(void);
static void                   hrs_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status);
static void                   hrs_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status);
static wiced_bt_gatt_status_t hrs_gatts_req_callback(wiced_bt_gatt_attribute_request_t *p_data);
static wiced_bt_gatt_status_t hrs_gatts_req_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data);
static wiced_bt_gatt_status_t hrs_gatts_req_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t * p_data);
static wiced_bt_gatt_status_t hrs_gatts_req_write_exec_handler(uint16_t conn_id, wiced_bt_gatt_exec_flag_t exec_falg);
static wiced_bt_gatt_status_t hrs_gatts_req_mtu_handler(uint16_t conn_id, uint16_t mtu);
static wiced_bt_gatt_status_t hrs_gatts_req_conf_handler(uint16_t conn_id, uint16_t handle);
static wiced_bt_gatt_status_t hrs_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
static void                   hrs_interrupt_handler(void* data, uint8_t pin );
static void                   hrs_process_pairing_complete(uint8_t reason);
static void                   hrs_process_encryption_status(wiced_result_t result);
static void                   hrs_load_keys_to_addr_resolution_db(void);
static wiced_bool_t           hrs_save_link_keys(wiced_bt_device_link_keys_t *p_keys);
static wiced_bool_t           hrs_read_link_keys(wiced_bt_device_link_keys_t *p_keys);
static void                   hrs_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);
static void                   heart_rate_notify_timeout(uint32_t count);
static void                   hrs_event_cback(wiced_bt_hrs_event_t event_type, wiced_bt_hrs_event_data_t *p_data);
static void                   hrs_interrput_config (void);

/******************************************************
 *               Variables Definitions
 ******************************************************/
#pragma pack(1)
// host information for NVRAM
typedef PACKED struct
{
    // BD address of the bonded host
    BD_ADDR     bd_addr;                     /* bonded client address */
    uint8_t     heart_rate_notifications_enabled;       /* bonded client heart rate notifications registered state */
} HOSTINFO;

typedef struct
{
    wiced_bt_device_address_t   peer_addr;  /* peer address */
    uint16_t                    conn_id;
    uint16_t                    energy_expended; /*Kilo jouls.Max value is 65535 */
    wiced_timer_t               heart_rate_notify_timer;
} hrs_app_cb_t;
#pragma pack()

hrs_app_cb_t    hrs_app_cb;
HOSTINFO        hrs_host_info;

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

#if defined WICED_BT_TRACE_ENABLE || defined HCI_TRACE_OVER_TRANSPORT
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
#endif
    WICED_BT_TRACE("HRS APP START\n");

    memset(&hrs_app_cb, 0, sizeof(hrs_app_cb));

    /* Initialize Stack and Register Management Callback */
    wiced_bt_stack_init(hrs_management_callback, &wiced_app_cfg_settings, wiced_app_cfg_buf_pools);
}

/*
 * HRS application initialization is executed after BT stack initialization is completed.
 */
void hrs_application_init(void)
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t         result;
    wiced_bt_hrs_handles_t hrs_handles =
    {
            .value = HDLC_HRS_HEART_RATE_MEASUREMENT_VALUE,     /* Measurement Value handle */
             .configuration = HDLD_HRS_HEART_RATE_MEASUREMENT_CLIENT_CHAR_CONFIG, /* Measurement Configuration Value handle */
            .control = HDLC_HRS_HEART_RATE_CONTROL_POINT_VALUE, /* Reset Energy Expended Control Value handle */
            .location = HDLC_HRS_BODY_SENSOR_LOCATION_VALUE,    /* Body Sensor Location Value handle */
    };

    hrs_interrput_config();

    /* Initialize Heart Rate Server Library */
    result = wiced_bt_hrs_init (hrs_event_cback, &hrs_handles);
    if (result != WICED_BT_SUCCESS)
        WICED_BT_TRACE("Err: wiced_bt_hrs_init failed status:%d\n", result);

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(hrs_gatts_callback);
    WICED_BT_TRACE("wiced_bt_gatt_register: %d\n", gatt_status);

    /*  Tell stack to use our GATT databse */
    gatt_status =  wiced_bt_gatt_db_init(gatt_database, gatt_database_len);
    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\n", gatt_status);

    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace(hrs_trace_callback);

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* Load the address resolution DB with the keys stored in the NVRAM */
    hrs_load_keys_to_addr_resolution_db();

    /* Set the advertising params and make the device discoverable */
    hrs_set_advertisement_data();

    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
    WICED_BT_TRACE( "wiced_bt_start_advertisements %d\n", result );

    /* Initialize periodic timer */
    wiced_init_timer (&hrs_app_cb.heart_rate_notify_timer, heart_rate_notify_timeout, 0, WICED_SECONDS_PERIODIC_TIMER);
}

#ifndef CYW43012C0
static void hrs_interrput_config (void)
{
#if !defined(CYW20735B1) && !defined(CYW20819A1) && !defined(CYW20719B2) && !defined(CYW20721B2)
    wiced_bt_app_init();
#endif

    /* Configure buttons available on the platform (pin should be configured before registering interrupt handler ) */
#if ( defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20735B1) || defined(CYW20819A1) || defined(CYW20721B2) || defined(CYW20719B2) )
    /* Configure buttons available on the platform */
    wiced_platform_register_button_callback( WICED_PLATFORM_BUTTON_1, hrs_interrupt_handler, NULL, WICED_PLATFORM_BUTTON_BOTH_EDGE);
#else
    wiced_hal_gpio_configure_pin( APP_BUTTON, APP_BUTTON_SETTINGS, APP_BUTTON_DEFAULT_STATE );
    wiced_hal_gpio_register_pin_for_interrupt( APP_BUTTON, hrs_interrupt_handler, NULL );
#endif
}
#endif

/*
 * ANCS Client link management callback
 */
wiced_result_t hrs_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                    result = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t *p_status;
    wiced_bt_dev_ble_pairing_info_t  *p_info;
    wiced_bt_ble_advert_mode_t       *p_mode;
    wiced_bt_device_link_keys_t       paired_device_link_keys_request;
    uint8_t                           bytes_written, bytes_read;
    uint8_t                           *p_keys;

    WICED_BT_TRACE("hrs_management_cback:%d\n", event);

    switch(event)
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
        hrs_application_init();
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
        hrs_process_pairing_complete(p_event_data->pairing_complete.pairing_complete_info.ble.reason);
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        WICED_BT_TRACE("Encryption Status Event: bd (%B) res %d\n", p_event_data->encryption_status.bd_addr, p_event_data->encryption_status.result);
        hrs_process_encryption_status(p_event_data->encryption_status.result);
        break;

    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        hrs_save_link_keys(&p_event_data->paired_device_link_keys_update);
        break;

     case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        if (hrs_read_link_keys(&p_event_data->paired_device_link_keys_request))
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
         wiced_hal_write_nvram ( HRS_LOCAL_KEYS_VS_ID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
         WICED_BT_TRACE("local keys save to NVRAM result: %d \n", result);
         break;

     case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
         /* read keys from NVRAM */
         p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
         wiced_hal_read_nvram( HRS_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
         WICED_BT_TRACE("local keys read from NVRAM result: %d \n",  result);
         break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        p_mode = &p_event_data->ble_advert_state_changed;
        WICED_BT_TRACE( "Advertisement State Change: %d\n", *p_mode);
        if ( *p_mode == BTM_BLE_ADVERT_OFF )
        {
            WICED_BT_TRACE("HRS ADV stopped\n");
        }
        break;

    default:
        break;
    }
    return result;
}

/*
 * Setup advertisement data with 16 byte UUID and device name
 */
void hrs_set_advertisement_data(void)
{
    wiced_bt_ble_advert_elem_t adv_elem[3];
    uint8_t    num_elem              = 0;
    uint8_t    flag                  = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t    hrs_local_name[]      = "HRS" ; //Alternate way to declare {'H', 'R', 'S', 0x00, 0x00};
    uint16_t   hrs_uuid              = UUID_SERVICE_HEART_RATE;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE;
    adv_elem[num_elem].len          = LEN_UUID_16;
    adv_elem[num_elem].p_data       = ( uint8_t* )&hrs_uuid;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((const char *)hrs_local_name);
    adv_elem[num_elem].p_data       = ( uint8_t* )hrs_local_name;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

/*
 * Callback for various GATT events.  As this application performs only as a GATT server, some of the events are omitted.
 */
wiced_bt_gatt_status_t hrs_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        if (p_data->connection_status.connected)
        {
            hrs_connection_up(&p_data->connection_status);
        }
        else
        {
            hrs_connection_down(&p_data->connection_status);
        }
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = hrs_gatts_req_callback(&p_data->attribute_request);
        break;

    default:
        break;
    }

    return result;
}

/*
 * This function will be called when a connection is established
 */
void hrs_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t  status;

    WICED_BT_TRACE("%s\n", __FUNCTION__);

    hrs_app_cb.conn_id   = p_conn_status->conn_id;
    memcpy(hrs_app_cb.peer_addr, p_conn_status->bd_addr, BD_ADDR_LEN);
    // Need to notify ANP Server library that the connection is up
    wiced_bt_hrs_connection_up(p_conn_status->conn_id);
}

/*
 * This function will be called when connection goes down
 */
void hrs_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_dev_status_t status;

    WICED_BT_TRACE("%s\n", __FUNCTION__);

    // tell library that connection is down
    wiced_bt_hrs_connection_down(p_conn_status->conn_id);

    memset(hrs_app_cb.peer_addr, 0, sizeof(hrs_app_cb.peer_addr));
    hrs_app_cb.conn_id           = 0;
    hrs_app_cb.energy_expended   = 0;
    wiced_stop_timer(&hrs_app_cb.heart_rate_notify_timer);

    /* Restart Advertissement to allow Client to reconenct */
    status =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
    WICED_BT_TRACE( "wiced_bt_start_advertisements %d\n", status );
}

/*
 * Process GATT request from the peer. Even our main goal is to be a GATT client for the
 * ANCS service, we need to support mandatory GATT procedures.
 */
wiced_bt_gatt_status_t hrs_gatts_req_callback(wiced_bt_gatt_attribute_request_t *p_data)
{
    wiced_result_t result = WICED_BT_GATT_INVALID_PDU;

    WICED_BT_TRACE("hrs_gatts_req_cb. conn %d, type %d\n", p_data->conn_id, p_data->request_type);

    switch (p_data->request_type)
    {
    case GATTS_REQ_TYPE_READ:
        result = hrs_gatts_req_read_handler(p_data->conn_id, &(p_data->data.read_req));
        break;

    case GATTS_REQ_TYPE_WRITE:
        result = hrs_gatts_req_write_handler(p_data->conn_id, &(p_data->data.write_req));
        break;

    case GATTS_REQ_TYPE_MTU:
        result = hrs_gatts_req_mtu_handler(p_data->conn_id, p_data->data.mtu);
        break;

   default:
        break;
    }

    return result;
}

/*
 * Process Read request or command from peer device
 */
wiced_bt_gatt_status_t hrs_gatts_req_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_data)
{
    int          i, attr_len_to_copy;

    /* WICED BT HRS library takes care HRS service characteristics read requests*/
    if ( (p_data->handle >= HDLS_HRS) && ( p_data->handle <= HDLC_HRS_HEART_RATE_CONTROL_POINT_VALUE ) )
    {
        return wiced_bt_hrs_process_client_read_req (conn_id, p_data);
    }

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
wiced_bt_gatt_status_t hrs_gatts_req_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t * p_data)
{
    WICED_BT_TRACE("write_handler: conn_id:%d hdl:0x%x prep:%d offset:%d len:%d\n ", conn_id, p_data->handle, p_data->is_prep, p_data->offset, p_data->val_len);

    if ( (p_data->handle >= HDLS_HRS) && ( p_data->handle <= HDLC_HRS_HEART_RATE_CONTROL_POINT_VALUE ) )
    {
        return wiced_bt_hrs_process_client_write_req(conn_id, p_data);
    }

    /* No other writable characteristics implemented in the snippet */
    return WICED_BT_GATT_WRITE_NOT_PERMIT;
}

/*
 * Write Execute Procedure
 */
wiced_bt_gatt_status_t hrs_gatts_req_write_exec_handler(uint16_t conn_id, wiced_bt_gatt_exec_flag_t exec_falg)
{
    WICED_BT_TRACE("write exec: flag:%d\n", exec_falg);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process MTU request from the peer
 */
wiced_bt_gatt_status_t hrs_gatts_req_mtu_handler(uint16_t conn_id, uint16_t mtu)
{
    WICED_BT_TRACE("req_mtu: %d\n", mtu);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process indication confirm from the peer.
 */
wiced_bt_gatt_status_t hrs_gatts_req_conf_handler(uint16_t conn_id, uint16_t handle)
{
    WICED_BT_TRACE("indication_cfm, conn %d hdl %d\n", conn_id, handle);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Button Interrupt can be handled here.
 */
void hrs_interrupt_handler(void* data, uint8_t pin )
{
    WICED_BT_TRACE("[hrs_interrupt_handler] \n");
    /* If connection is down, start high duty advertisements, so client can connect */
    if ( (hrs_app_cb.conn_id == 0) && (wiced_bt_ble_get_current_advert_mode() == BTM_BLE_ADVERT_OFF) )
    {
        wiced_result_t result;

        WICED_BT_TRACE( "ADV start high\n");

        result = wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );

        WICED_BT_TRACE( "wiced_bt_start_advertisements:%d\n", result );
        return;
    }
}

/*
 * Updates bonded client address to NVRAM
 */
void hrs_process_pairing_complete(uint8_t reason)
{
    wiced_result_t result;

    if (reason == WICED_BT_SUCCESS)
    {
        memset(&hrs_host_info, 0, sizeof(hrs_host_info));
        memcpy(hrs_host_info.bd_addr, hrs_app_cb.peer_addr, BD_ADDR_LEN);
        wiced_hal_write_nvram( HRS_HOST_INFO_VS_ID, sizeof(hrs_host_info), (uint8_t*)&hrs_host_info, &result );
    }
}

/*
 * Sends Heart Rate notifications, if authenticated client configured for notifications in previous connection.
 */
void hrs_process_encryption_status(wiced_result_t result)
{
    if (result == WICED_BT_SUCCESS)
    {
        wiced_hal_read_nvram( HRS_HOST_INFO_VS_ID, sizeof(hrs_host_info), (uint8_t*)&hrs_host_info, &result );
        if (result == WICED_SUCCESS)
        {
            if ( (memcmp(hrs_host_info.bd_addr, hrs_app_cb.peer_addr, BD_ADDR_LEN) == 0) )
            {
                wiced_bt_hrs_set_previous_connection_client_notification_configuration(hrs_app_cb.conn_id, hrs_host_info.heart_rate_notifications_enabled);
                if( hrs_host_info.heart_rate_notifications_enabled )
                {
                    wiced_start_timer(&hrs_app_cb.heart_rate_notify_timer, 60);
                }
            }
        }
    }
}

/*
 * Read keys from the NVRAM and update address resolution database
 */
void hrs_load_keys_to_addr_resolution_db(void)
{
    uint8_t                     bytes_read;
    wiced_result_t              result;
    wiced_bt_device_link_keys_t keys;

    bytes_read = wiced_hal_read_nvram(HRS_PAIRED_KEYS_VS_ID, sizeof(keys), (uint8_t *)&keys, &result);

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
wiced_bool_t hrs_save_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    uint8_t          bytes_written;
    wiced_result_t   result;

    bytes_written = wiced_hal_write_nvram(HRS_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), (uint8_t *)p_keys, &result);
    WICED_BT_TRACE("Saved %d bytes at id:%d\n", bytes_written, HRS_PAIRED_KEYS_VS_ID);
    return (bytes_written == sizeof (wiced_bt_device_link_keys_t));
}

/*
 * This function is called to read keys for specific bdaddr
 */
wiced_bool_t hrs_read_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    uint8_t         bytes_read;
    wiced_result_t  result;

    bytes_read = wiced_hal_read_nvram(HRS_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), (uint8_t *)p_keys, &result);
    WICED_BT_TRACE("read %d bytes at id:%d %d\n", bytes_read, HRS_PAIRED_KEYS_VS_ID);
    return (bytes_read == sizeof (wiced_bt_device_link_keys_t));
}


/*
 *  Pass protocol traces up over the transport
 */
void hrs_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    wiced_transport_send_hci_trace(NULL, type, length, p_data);
}

void heart_rate_notify_timeout(uint32_t arg)
{
    wiced_bt_gatt_status_t status;
    static uint32_t count = 0;
    wiced_bt_hrs_notification_data_t  heart_rate_notification;

    memset(&heart_rate_notification, 0, sizeof(heart_rate_notification));

#define HEART_BEAT_PER_MINUTE                   72 //typical value
    heart_rate_notification.heart_rate  = HEART_BEAT_PER_MINUTE;

    if (hrs_app_cb.energy_expended == 0)
    {
        count = 0;
    }

    /* This value get reset only on WICED_BT_HRS_RESET_ENERGY_EXPENDED_VALUE */
    if (hrs_app_cb.energy_expended != 0xffff)
        hrs_app_cb.energy_expended += 0x1111; // 0xf * 0x1111 = 0xffff

    /* Typically once on every 10 heart rate measurements energy expended value get notified */
    if ( !(++count % 10) )
    {
        heart_rate_notification.energy_expended_present = 1;
        heart_rate_notification.energy_expended         = hrs_app_cb.energy_expended;
    }

    //Todo Add RR intervals once wiced bt HRS supports RR intervals

    status = wiced_bt_hrs_send_heart_rate( hrs_app_cb.conn_id, &heart_rate_notification );
    WICED_BT_TRACE("notify heart rate %d \n", status);
}

void hrs_event_cback(wiced_bt_hrs_event_t event, wiced_bt_hrs_event_data_t *p_data)
{
    wiced_result_t  result;
    uint8_t         nvram_write = 0;

    switch(event)
    {
    case WICED_BT_HRS_EVENT_HEART_RATE_NOTIFICATIONS_ENABLED:
        WICED_BT_TRACE("[%s]Notification Enabled\n", __FUNCTION__);
        /* on every 1 minute heart rate value notified to client*/
        nvram_write = 1;
        hrs_host_info.heart_rate_notifications_enabled = 1;
        wiced_start_timer(&hrs_app_cb.heart_rate_notify_timer, 60);
        break;

    case WICED_BT_HRS_EVENT_HEART_RATE_NOTIFICATIONS_DISABLED:
        WICED_BT_TRACE("[%s]Notification Disabled\n", __FUNCTION__);
        /* stop notifications */
        nvram_write = 1;
        hrs_host_info.heart_rate_notifications_enabled = 0;
        wiced_stop_timer(&hrs_app_cb.heart_rate_notify_timer);
        break;

    case WICED_BT_HRS_RESET_ENERGY_EXPENDED_VALUE:
        WICED_BT_TRACE("[%s]Reset Energy Expended\n", __FUNCTION__);
        hrs_app_cb.energy_expended = 0;
        break;

    default:
        WICED_BT_TRACE("[%s] unknown event:%d\n", __FUNCTION__, event);
        return;
    }

    if (nvram_write)
    {
        wiced_hal_write_nvram( HRS_HOST_INFO_VS_ID, sizeof(hrs_host_info), (uint8_t*)&hrs_host_info, &result );
    }
}
