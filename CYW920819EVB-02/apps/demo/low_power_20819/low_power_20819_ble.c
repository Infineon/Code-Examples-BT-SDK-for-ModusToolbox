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

/*
 * File name: low_power_20819_ble.c
 *
 * Description: This file defines various functions to handle BLE related
 *              functionality as well as button callbacks.
 *
 */
#include "wiced.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_hal_gpio.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"
#include "GeneratedSource/cycfg_gatt_db.h"
#include "wiced_bt_cfg.h"
#include "wiced_sleep.h"
#include "wiced_bt_l2c.h"
#include "wiced_rtc.h"
#include "wiced_timer.h"

/*******************************************************************
 * Constant Definitions
 ******************************************************************/

/* enum for system state */
enum
{
    SLEEP_WITHOUT_BLE,
    SLEEP_WITH_ADV,
    SLEEP_WITH_CONNECTION,
}application_state;

#define NOTFICATION_TIME_MS    5000     /* 5 seconds interval for notification if
                                           notifications enabled by client*/

#define BATTERY_LEVEL_FULL     100      /* Value of battery when full */
#define BATTERY_LEVEL_EMPTY    0        /* Value of battery when empty */

#define CONNECTION_INTERVAL    80       /* Connection interval of 100 ms */
#define SLAVE_LATENCY          0        /* Slave latency of 0 */
#define SUPERVISION_TIMEOUT    300      /* link Supervision timeout of 3 seconds */

#define DUMMY                  0        /* Dummy value to pass to timer init function
                                           and connection ID when no connection present*/

/**************************************************************************************
 *                                Variable Definitions
 *************************************************************************************/

uint8_t low_power_20819_current_state = SLEEP_WITHOUT_BLE;     /* Maintains the current system state */
uint8_t low_power_20819_conn_id       = DUMMY;                 /* Maintains the connection id of the current connection */
wiced_timer_t  low_power_20819_notification_timer_handle;      /* Notification timer handle */

extern uint8_t BT_LOCAL_NAME[];
/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static void                   low_power_20819_app_init               ( void );

/* GATT Registration Callbacks */
static wiced_bt_gatt_status_t low_power_20819_gatt_write_handler          ( wiced_bt_gatt_write_t *p_write_req, uint16_t conn_id );
static wiced_bt_gatt_status_t low_power_20819_gatt_read_handler           ( wiced_bt_gatt_read_t *p_read_req, uint16_t conn_id );
static wiced_bt_gatt_status_t low_power_20819_gatt_connect_callback       ( wiced_bt_gatt_connection_status_t *p_conn_status );
static wiced_bt_gatt_status_t low_power_20819_gatt_server_callback        ( uint16_t conn_id, wiced_bt_gatt_request_type_t type, wiced_bt_gatt_request_data_t *p_data );
static wiced_bt_gatt_status_t low_power_20819_gatt_event_handler          ( wiced_bt_gatt_evt_t  event, wiced_bt_gatt_event_data_t *p_event_data );
static void                   low_power_20819_timeout                ( uint32_t count );
static void                   low_power_20819_set_advertisement_data (void);
static void                   button_cb                              (void* user_data, uint8_t value );

extern void low_power_20819_enter_hid_off (void);
/*******************************************************************
 * Function Definitions
 ******************************************************************/

/*********************************************************************
* Function Name: void low_power_20819_app_init(void)
**********************************************************************
* Summary:
*   This function initializes/registers button interrupt, initializes
*   notification timer and register gatt database.
*
* Parameters:
*   None
*
* Return:
*   None
*
**********************************************************************/
void low_power_20819_app_init(void)
{

    wiced_real_time_clock_t curr_time;

    /* Register for button callback */
    wiced_platform_register_button_callback(WICED_GPIO_PIN_BUTTON, button_cb, NULL, GPIO_EN_INT_FALLING_EDGE);

    /* Initialize RTC */
    wiced_rtc_init();

    /* Get current time and print */
    wiced_rtc_get_raw_clock(&curr_time);

    WICED_BT_TRACE("Current Raw RTC clock: %d\r\n", curr_time.wiced_rtc64);

    /* Initialize Notification timer */
    if(WICED_BT_SUCCESS != wiced_init_timer(&low_power_20819_notification_timer_handle, &low_power_20819_timeout, DUMMY, WICED_MILLI_SECONDS_PERIODIC_TIMER))
    {
        WICED_BT_TRACE("Notification timer failed\r\n");
    }

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, FALSE);

    /* Register with stack to receive GATT callback */
    if(WICED_BT_GATT_SUCCESS != wiced_bt_gatt_register(low_power_20819_gatt_event_handler))
    {
        WICED_BT_TRACE("GATT callback registration failed\r\n");
    }

    /* Initialize GATT Database */
    if(WICED_BT_GATT_SUCCESS != wiced_bt_gatt_db_init( gatt_database, gatt_database_len ))
    {
        WICED_BT_TRACE("GATT DB init failed\r\n");
    }
}

/*********************************************************************
* Function Name: void low_power_20819_set_advertisement_data(void)
**********************************************************************
* Summary:
*   Setup advertisement data with device name.
*
* Parameters:
*   None
*
* Return:
*   None
*
**********************************************************************/
void low_power_20819_set_advertisement_data( void )
{
    wiced_bt_ble_advert_elem_t adv_elem[2] = { 0 };
    uint8_t adv_flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t num_elem = 0;

    /* Advertisement Element for Flags */
    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len = sizeof(uint8_t);
    adv_elem[num_elem].p_data = &adv_flag;
    num_elem++;

    /* Advertisement Element for Name */
    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len = strlen((const char*)BT_LOCAL_NAME);
    adv_elem[num_elem].p_data = BT_LOCAL_NAME;
    num_elem++;

    /* Set Raw Advertisement Data */
    if(WICED_BT_SUCCESS != wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem))
    {
        WICED_BT_TRACE("Set ADV data failed\r\n");
    }
}

/****************************************************************************
* Function Name: void low_power_20819_timeout(uint32_t count)
*****************************************************************************
* Summary:
*   Five second timer callback. Send dummy battery value if connected and
*   notification enabled.
*
* Parameters:
*   uint32_t count: Variable registered by wiced_init_timer to pass to
*                    timer callback. Not used here.
*
* Return:
*   None
*
****************************************************************************/
void low_power_20819_timeout( uint32_t count )
{
    /* Check if device is connected and notifications are enabled and send notification */
    if((GATT_CLIENT_CONFIG_NOTIFICATION == app_bas_battery_level_client_char_config[0]) && (0 != low_power_20819_conn_id))
    {
       WICED_BT_TRACE("\r\nSending Notification\r\n");
       if(BATTERY_LEVEL_FULL > app_bas_battery_level[0])
       {
           app_bas_battery_level[0]++;
       }
       else
       {
           app_bas_battery_level[0] = BATTERY_LEVEL_EMPTY;
       }

       if(WICED_BT_GATT_SUCCESS != wiced_bt_gatt_send_notification(low_power_20819_conn_id, HDLC_BAS_BATTERY_LEVEL_VALUE, app_gatt_db_ext_attr_tbl[2].max_len, app_gatt_db_ext_attr_tbl[2].p_data))
       {
           WICED_BT_TRACE("Sending sensor value notification failed\r\n");
       }
    }
}

/**********************************************************************************************
* Function Name: wiced_bt_dev_status_t low_power_20819_bt_management_callback( wiced_bt_management_evt_t event,
                                                wiced_bt_management_evt_data_t *p_event_data )
***********************************************************************************************
* Summary:
*   This is a Bluetooth management event handler function to receive events from
*   bluetooth stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : Bluetooth event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to bluetooth event data
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
***********************************************************************************************/
wiced_bt_dev_status_t low_power_20819_bt_management_callback( wiced_bt_management_evt_t event,
                                                wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_dev_status_t status = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = { 0 };
    wiced_bt_dev_ble_pairing_info_t *p_ble_info = NULL;
    wiced_bt_ble_advert_mode_t *p_adv_mode = NULL;

    switch (event)
    {
    case BTM_ENABLED_EVT:
        /* Bluetooth Controller and Host Stack Enabled */

        WICED_BT_TRACE("Bluetooth Enabled (%s)\r\n",
                ((WICED_BT_SUCCESS == p_event_data->enabled.status) ? "success" : "failure"));

        if (WICED_BT_SUCCESS == p_event_data->enabled.status)
        {
            /* Bluetooth is enabled */
            wiced_bt_dev_read_local_addr(bda);
            WICED_BT_TRACE("Local Bluetooth Address: [%B]\r\n", bda);

            /* Perform application-specific initialization */
            low_power_20819_app_init();
        }
        break;
    case BTM_DISABLED_EVT:
        /* Bluetooth Controller and Host Stack Disabled */
        WICED_BT_TRACE("Bluetooth Disabled\r\n");
        break;
    case BTM_SECURITY_REQUEST_EVT:
        /* Security Request */
        WICED_BT_TRACE("Security Request\r\n");
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
        break;
    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        /* Request for Pairing IO Capabilities (BLE) */
        WICED_BT_TRACE("BLE Pairing IO Capabilities Request\r\n");
        /* No IO Capabilities on this Platform */
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC_ONLY;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys = 0;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
        break;
    case BTM_PAIRING_COMPLETE_EVT:
        /* Pairing is Complete */
        p_ble_info = &p_event_data->pairing_complete.pairing_complete_info.ble;
        WICED_BT_TRACE("Pairing Complete %d.\r\n", p_ble_info->reason);
        break;
    case BTM_ENCRYPTION_STATUS_EVT:
        /* Encryption Status Change */
        WICED_BT_TRACE("Encryption Status event: bd ( %B ) res %d\r\n", p_event_data->encryption_status.bd_addr, p_event_data->encryption_status.result);
        break;
    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        /* Paired Device Link Keys Request */
        WICED_BT_TRACE("Paired Device Link Request Keys Event\r\n");
        status = WICED_BT_ERROR;
        break;
    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        /* Local identity Keys Update */
        WICED_BT_TRACE("Local Identity Update Keys Event\r\n");
        status = WICED_BT_ERROR;
        break;
    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /* Local identity Keys Request */
        WICED_BT_TRACE("Local Identity Request Keys Event\r\n");
        status = WICED_BT_ERROR;
        break;
    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        /* Advertisement State Changed */
        p_adv_mode = &p_event_data->ble_advert_state_changed;
        WICED_BT_TRACE("Advertisement State Change: %d\r\n", *p_adv_mode);
        if ( BTM_BLE_ADVERT_OFF == *p_adv_mode )
        {
            WICED_BT_TRACE("Advertisement stopped\r\n");
        }
        break;
    case BTM_BLE_CONNECTION_PARAM_UPDATE:
        /* Connection parameters updated */
        if(WICED_SUCCESS == p_event_data->ble_connection_param_update.status)
        {
            WICED_BT_TRACE("\r\nNew connection parameters:\r\nConnection interval = %d intervals\r\nConnection latency = %d\r\nSupervision timeout = %d\r\n",
                    ((p_event_data->ble_connection_param_update.conn_interval)),
                    p_event_data->ble_connection_param_update.conn_latency,
                    ((p_event_data->ble_connection_param_update.supervision_timeout)));
        }
        else
        {
            WICED_BT_TRACE("Connection parameters update failed\r\n");
        }
        break;
    default:
        WICED_BT_TRACE("Unhandled Bluetooth Management Event: 0x%x (%d)\r\n", event, event);
        break;
    }

    return status;
}

/**********************************************************************************************
* Function Name: wiced_bt_gatt_status_t low_power_20819_get_value( uint16_t attr_handle,
*                       uint16_t conn_id, uint8_t *p_val, uint16_t max_len, uint16_t *p_len )
***********************************************************************************************
* Summary:
*   This function searches the GATT DB for the specified handle passes on the value
*
* Parameters:
*   uint16_t attr_handle                        : Attribute handle to search for
*   uint16_t conn_id                            : Connection id of the current connection
*   uint8_t *p_val                              : Pointer to store the retrieved value
*   uint16_t max_len                            : Maximum length of the of the supplied buffer
*   uint16_t *p_len                             : Pointer to pass the length of the data passed
*
* Return:
*  wiced_bt_gatt_status_t: Error code from wiced_bt_gatt_status_t
*
***********************************************************************************************/
wiced_bt_gatt_status_t low_power_20819_get_value( uint16_t attr_handle,
                      uint16_t conn_id, uint8_t *p_val, uint16_t max_len, uint16_t *p_len )
{
    int i = 0;
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    /* Check for a matching entry in the GATT DB */
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            /* Detected a matching handle in GATT DB */
            isHandleInTable = WICED_TRUE;
            if (app_gatt_db_ext_attr_tbl[i].cur_len <= max_len)
            {
                /* Value fits within the supplied buffer; copy over the value */
                *p_len = app_gatt_db_ext_attr_tbl[i].cur_len;
                memcpy(p_val, app_gatt_db_ext_attr_tbl[i].p_data, app_gatt_db_ext_attr_tbl[i].cur_len);
                res = WICED_BT_GATT_SUCCESS;
            }
            else
            {
                /* Length of the buffer smaller than the attribute value length */
                res = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!isHandleInTable)
    {
        WICED_BT_TRACE("Read Request to Invalid Handle: 0x%x\r\n", attr_handle);
        res = WICED_BT_GATT_INVALID_HANDLE;
    }

    return res;
}

/*************************************************************************************
* Function Name: wiced_bt_gatt_status_t low_power_20819_gatt_write_handler(
*                wiced_bt_gatt_write_t *p_write_req, uint16_t conn_id)
**************************************************************************************
* Summary: GATT attribute write function.
*          Process write request or write command from peer device.
*
* Parameters:
*   uint16_t conn_id                : GATT connection id
*   wiced_bt_gatt_write_t * p_data  : GATT write attribute handle
*
* Return:
*   wiced_bt_gatt_status_t: WICED_BT_GATT_SUCCESS or WICED_BT_GATT_INVALID_HANDLE or
*                           WICED_BT_GATT_INVALID_ATTR_LEN
*
*************************************************************************************/
wiced_bt_gatt_status_t low_power_20819_gatt_write_handler( wiced_bt_gatt_write_t *p_write_req, uint16_t conn_id )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;
    uint8_t                *p_attr   = p_write_req->p_val;
    uint16_t                old_cccd = 0; /* Variable to store the old CCCD value */

    if(HDLD_BAS_BATTERY_LEVEL_CLIENT_CHAR_CONFIG == p_write_req->handle)
    {
        if (app_bas_battery_level_client_char_config_len != p_write_req->val_len)
        {
            return WICED_BT_GATT_INVALID_ATTR_LEN;
        }

        old_cccd = (app_bas_battery_level_client_char_config[0]);
        app_bas_battery_level_client_char_config[0] = p_attr[0];
        app_bas_battery_level_client_char_config[1] = ( p_attr[1] << 8 );

        /* Start timer only the notification is being enabled for the first time here */
        if(GATT_CLIENT_CONFIG_NOTIFICATION == app_bas_battery_level_client_char_config[0] &&
                GATT_CLIENT_CONFIG_NOTIFICATION != old_cccd)
        {
            if(WICED_BT_SUCCESS != wiced_start_timer(&low_power_20819_notification_timer_handle, NOTFICATION_TIME_MS))
            {
                WICED_BT_TRACE("Notification timer start failed\r\n");
            }
        }
        else if(GATT_CLIENT_CONFIG_NONE == app_bas_battery_level_client_char_config[0])
        {
            if(WICED_BT_SUCCESS != wiced_stop_timer(&low_power_20819_notification_timer_handle))
            {
                WICED_BT_TRACE("Notification timer stop failed\r\n");
            }
        }
    }

    return WICED_BT_GATT_SUCCESS;
}

/*************************************************************************************************
* Function Name: wiced_bt_gatt_status_t
*                low_power_20819_gatt_read_handler(wiced_bt_gatt_read_t *p_read_req, uint16_t conn_id)
**************************************************************************************************
* Summary: GATT attribute read function.
*          Process read command from peer device.
*
* Parameters:
*   wiced_bt_gatt_read_t * p_read_req  : GATT read attribute handle
*   uint16_t conn_id                    : GATT connection id
*
* Return:
*   wiced_bt_gatt_status_t: WICED_BT_GATT_SUCCESS or WICED_BT_GATT_INVALID_HANDLE
*
*************************************************************************************************/
wiced_bt_gatt_status_t low_power_20819_gatt_read_handler( wiced_bt_gatt_read_t *p_read_req, uint16_t conn_id )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    /* Attempt to perform the Read Request */
    status = low_power_20819_get_value(p_read_req->handle, conn_id, p_read_req->p_val, *p_read_req->p_val_len, p_read_req->p_val_len);

    return status;
}

/*************************************************************************************
* Function Name: wiced_bt_gatt_status_t low_power_20819_gatt_connect_callback
*                       ( wiced_bt_gatt_connection_status_t *p_conn_status )
**************************************************************************************
* Summary: This function is invoked on GATT connection status change
*
* Parameters:
*   wiced_bt_gatt_connection_status_t *p_status: GATT connection status handle
*
* Return:
*   wiced_bt_gatt_status_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*************************************************************************************/
wiced_bt_gatt_status_t low_power_20819_gatt_connect_callback( wiced_bt_gatt_connection_status_t *p_conn_status )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    if ( NULL != p_conn_status )
    {
        if ( p_conn_status->connected )
        {
            /* Device got connected */
            WICED_BT_TRACE("Connected : BDA '%B', Connection ID '%d'\r\n", p_conn_status->bd_addr, p_conn_status->conn_id );
            low_power_20819_current_state = SLEEP_WITH_CONNECTION;
            low_power_20819_conn_id = p_conn_status->conn_id;

            /* Update connection parameters to 100 ms */
            if(TRUE != wiced_bt_l2cap_update_ble_conn_params( p_conn_status->bd_addr, CONNECTION_INTERVAL, CONNECTION_INTERVAL,
                                                    SLAVE_LATENCY, SUPERVISION_TIMEOUT ))
            {
                WICED_BT_TRACE("Connection parameters update failed\r\n");
            }
        }
        else
        {
            /* Device got disconnected */
            WICED_BT_TRACE("Disconnected : BDA '%B', Connection ID '%d', Reason '%d'\r\n", p_conn_status->bd_addr, p_conn_status->conn_id, p_conn_status->reason );
            low_power_20819_conn_id = DUMMY;

            /*Enter HID-Off */
            low_power_20819_enter_hid_off();
        }
        status = WICED_BT_GATT_SUCCESS;
    }

    return status;
}

/*************************************************************************************
 * Function Name: wiced_bt_gatt_status_t low_power_20819_gatt_server_callback( uint16_t conn_id,
 *            wiced_bt_gatt_request_type_t type, wiced_bt_gatt_request_data_t *p_data )
 **************************************************************************************
 * Summary: Process GATT Read/Write request from the peer.
 *
 * Parameters:
 *   uint16_t conn_id                           : Connection id of the current connection
 *   wiced_bt_gatt_request_type_t type          : Type of GATT request
 *   wiced_bt_gatt_attribute_request_t *p_data  : GATT request information handle.
 *
 * Return:
 *   wiced_bt_gatt_status_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
 *
 *************************************************************************************/
wiced_bt_gatt_status_t low_power_20819_gatt_server_callback( uint16_t conn_id,
        wiced_bt_gatt_request_type_t type, wiced_bt_gatt_request_data_t *p_data )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    switch ( type )
    {
    case GATTS_REQ_TYPE_READ:
        status = low_power_20819_gatt_read_handler( &p_data->read_req, conn_id );
        break;
    case GATTS_REQ_TYPE_WRITE:
        status = low_power_20819_gatt_write_handler( &p_data->write_req, conn_id );
        break;
    default:
        break;
    }

    return status;
}

/**************************************************************************************
 * Function Name: wiced_bt_gatt_status_t low_power_20819_gatt_event_handler(wiced_bt_gatt_evt_t event,
 *                                                  wiced_bt_gatt_event_data_t *p_event_data)
 **************************************************************************************
 * Summary: Callback for various GATT events.
 *
 * Parameters:
 *   wiced_bt_gatt_evt_t event                : GATT event code.
 *   wiced_bt_gatt_event_data_t *p_event_data : GATT event information handle.
 *
 * Return:
 *   wiced_bt_gatt_status_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
 *
 *************************************************************************************/
wiced_bt_gatt_status_t low_power_20819_gatt_event_handler( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_bt_gatt_connection_status_t *p_conn_status = NULL;
    wiced_bt_gatt_attribute_request_t *p_attr_req = NULL;

    switch ( event )
    {
    case GATT_CONNECTION_STATUS_EVT:
        status = low_power_20819_gatt_connect_callback( &p_event_data->connection_status );
        break;
    case GATT_ATTRIBUTE_REQUEST_EVT:
        p_attr_req = &p_event_data->attribute_request;
        status = low_power_20819_gatt_server_callback( p_attr_req->conn_id, p_attr_req->request_type, &p_attr_req->data );
        break;
    default:
        WICED_BT_TRACE("Unhandled GATT Event: 0x%x (%d)\r\n", event, event);
        status = WICED_BT_GATT_SUCCESS;
        break;
    }

    return status;
}

/**************************************************************************************
 * Function Name: void button_cb (void* user_data, uint8_t value )
 **************************************************************************************
 * Summary: Callback on button press. It checks the current state of the system and takes
 *          appropriate action
 *
 * Parameters:
 *   void* user_data                          : Not used
 *   uint8_t value                            : Not used
 *
 * Return:
 *   None
 *
 *************************************************************************************/
void button_cb (void* user_data, uint8_t value )
{

    wiced_bt_gatt_status_t disconnect_status;

    WICED_BT_TRACE("\n\rButton CB\r\n");

    switch(low_power_20819_current_state)
    {
    case SLEEP_WITHOUT_BLE:
        /* If the device is sleeping without any activity then start ADV */
        WICED_BT_TRACE("Starting ADV\r\n");
        /* Set Advertisement Data */
        low_power_20819_set_advertisement_data();

        /* Start Undirected LE Advertisements
         * The corresponding parameters are contained in 'wiced_bt_cfg.c' */
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);

        low_power_20819_current_state = SLEEP_WITH_ADV;
        break;

    case SLEEP_WITH_CONNECTION:

        /* If the device is in connection, disconnect and got to HID-Off */
        /* Disconnect */
        disconnect_status = wiced_bt_gatt_disconnect(low_power_20819_conn_id);

        if(WICED_BT_GATT_SUCCESS == disconnect_status)
        {
            WICED_BT_TRACE("Disconnecting\n\r");
        }
        break;
    default:
        break;
    }
}
