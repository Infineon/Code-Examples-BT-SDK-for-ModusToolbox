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

/*******************************************************************************
* File Name: app_bt_event_handler.c
* Version: 1.0
*
* Description:
*   Source file for handling Bluetooth stack events at the application level
*
*******************************************************************************/

/*******************************************************************************
*        Header Files
*******************************************************************************/
#include "app_bt_event_handler.h"
#include "app_user_interface.h"
#include "GeneratedSource/cycfg_gatt_db.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"

/*******************************************************************************
*        Variable Definitions
*******************************************************************************/
uint16_t bt_connection_id = 0;
app_bt_adv_conn_mode_t app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_OFF;

/*******************************************************************************
*        External Variable Declarations
*******************************************************************************/
extern uint8_t BT_LOCAL_NAME[];

/*******************************************************************************
*        Function Prototypes
*******************************************************************************/
static void                   ble_app_init               (void);
static void                   ble_app_set_advertisement_data (void);

/* GATT Event Callback Functions */
static wiced_bt_gatt_status_t ble_app_write_handler          (wiced_bt_gatt_write_t *p_write_req, uint16_t conn_id);
static wiced_bt_gatt_status_t ble_app_read_handler           (wiced_bt_gatt_read_t *p_read_req, uint16_t conn_id);
static wiced_bt_gatt_status_t ble_app_connect_callback       (wiced_bt_gatt_connection_status_t *p_conn_status);
static wiced_bt_gatt_status_t ble_app_server_callback        (uint16_t conn_id, wiced_bt_gatt_request_type_t type, wiced_bt_gatt_request_data_t *p_data);
static wiced_bt_gatt_status_t ble_app_gatt_event_handler     (wiced_bt_gatt_evt_t  event, wiced_bt_gatt_event_data_t *p_event_data);

/*******************************************************************************
*        Function Definitions
*******************************************************************************/

/**************************************************************************************************
* Function Name: app_bt_management_callback()
***************************************************************************************************
* Summary:
*   This is a Bluetooth stack event handler function to receive management events from
*   the BLE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : BLE event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to BLE management event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*************************************************************************************************/
wiced_result_t app_bt_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t status = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = { 0 };
    wiced_bt_dev_ble_pairing_info_t *p_ble_info = NULL;
    wiced_bt_ble_advert_mode_t *p_adv_mode = NULL;

    switch (event)
    {
        case BTM_ENABLED_EVT:

            /* Bluetooth Controller and Host Stack Enabled */
            WICED_BT_TRACE("Bluetooth Enabled (%s)\n\r",
                    ((WICED_BT_SUCCESS == p_event_data->enabled.status) ? "success" : "failure"));

            if (WICED_BT_SUCCESS == p_event_data->enabled.status)
            {
                /* Bluetooth is enabled */
                wiced_bt_dev_read_local_addr(bda);
                WICED_BT_TRACE("Local Bluetooth Address: [%B]\n\r", bda);

                /* Perform application-specific initialization */
                ble_app_init();
            }

            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:

            /* Advertisement State Changed */
            p_adv_mode = &p_event_data->ble_advert_state_changed;
            WICED_BT_TRACE("Advertisement State Change: %d\n\r", *p_adv_mode);

            if (BTM_BLE_ADVERT_OFF == *p_adv_mode)
            {
                /* Advertisement Stopped */
                WICED_BT_TRACE("Advertisement stopped\n\r");

                /* Check connection status after advertisement stops */
                if(bt_connection_id == 0)
                {
                    app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_OFF;
                }
                else
                {
                    app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_ON;
                }
            }
            else
            {
                /* Advertisement Started */
                WICED_BT_TRACE("Advertisement started\n\r");
                app_bt_adv_conn_state = APP_BT_ADV_ON_CONN_OFF;
            }

            /* Update Advertisement LED to reflect the updated state */
            adv_led_update();

            break;

        default:
            WICED_BT_TRACE("Unhandled Bluetooth Management Event: 0x%x (%d)\n\r", event, event);
            break;
    }

    return status;
}

/*
 * This function is executed in the BTM_ENABLED_EVT management callback.
 */

/**************************************************************************************************
* Function Name: ble_app_init()
***************************************************************************************************
* Summary:
*   This function handles application level initialization tasks and is called from the BT
*   management callback once the BLE stack enabled event (BTM_ENABLED_EVT) is triggered
*
* Parameters:
*   None
*
* Return:
*  None
*
*************************************************************************************************/
static void ble_app_init(void)
{
    /* User interface initialization for LEDs, buttons */
    app_user_interface_init();

    /* Disable pairing for this application */
    wiced_bt_set_pairable_mode(WICED_FALSE, 0);

    /* Set Advertisement Data */
    ble_app_set_advertisement_data();

    /* Register with BT stack to receive GATT callback */
    wiced_bt_gatt_register(ble_app_gatt_event_handler);

    /* Initialize GATT Database */
    wiced_bt_gatt_db_init(gatt_database, gatt_database_len);

    /* Start Undirected LE Advertisements on device startup.
     * The corresponding parameters are contained in 'app_bt_cfg.c' */
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
}

/**************************************************************************************************
* Function Name: ble_app_gatt_event_handler()
***************************************************************************************************
* Summary:
*   This function handles GATT events from the BT stack.
*
* Parameters:
*   wiced_bt_gatt_evt_t event                   : BLE GATT event code of one byte length
*   wiced_bt_gatt_event_data_t *p_event_data    : Pointer to BLE GATT event structures
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
static wiced_bt_gatt_status_t ble_app_gatt_event_handler(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_bt_gatt_connection_status_t *p_conn_status = NULL;
    wiced_bt_gatt_attribute_request_t *p_attr_req = NULL;

    /* Call the appropriate callback function based on the GATT event type, and pass the relevant event
     * parameters to the callback function */
    switch ( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
            status = ble_app_connect_callback( &p_event_data->connection_status );
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            p_attr_req = &p_event_data->attribute_request;
            status = ble_app_server_callback( p_attr_req->conn_id, p_attr_req->request_type, &p_attr_req->data );
            break;

        default:
            status = WICED_BT_GATT_SUCCESS;
            break;
    }

    return status;
}

/**************************************************************************************************
* Function Name: ble_app_set_advertisement_data()
***************************************************************************************************
* Summary:
*   This function configures the advertisement packet data
*
* Parameters:
*   None
*
* Return:
*   None
*
**************************************************************************************************/
static void ble_app_set_advertisement_data(void)
{
    wiced_bt_ble_advert_elem_t adv_elem[3] = { 0 };
    uint8_t adv_flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t adv_appearance[] = { BIT16_TO_8( APPEARANCE_GENERIC_KEYRING ) };
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

    /* Advertisement Element for Appearance */
    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_APPEARANCE;
    adv_elem[num_elem].len = sizeof(adv_appearance);
    adv_elem[num_elem].p_data = adv_appearance;
    num_elem++;

    /* Set Raw Advertisement Data */
    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

/**************************************************************************************************
* Function Name: ble_app_get_value()
***************************************************************************************************
* Summary:
*   This function handles reading of the attribute value from the GATT database and passing the
*   data to the BT stack. The value read from the GATT database is stored in a buffer whose
*   starting address is passed as one of the function parameters
*
* Parameters:
*   uint16_t attr_handle                    : Attribute handle for read operation
*   uint16_t conn_id                        : Connection ID
*   uint8_t *p_val                          : Pointer to the buffer to store read data
*   uint16_t max_len                        : Maximum buffer length available to store the read data
*   uint16_t *p_len                         : Actual length of data copied to the buffer
*
* Return:
*   wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
static wiced_bt_gatt_status_t ble_app_get_value(uint16_t attr_handle, uint16_t conn_id, uint8_t *p_val, uint16_t max_len, uint16_t *p_len)
{
    int i = 0;
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    /* Check for a matching handle entry */
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            /* Detected a matching handle in external lookup table */
            isHandleInTable = WICED_TRUE;

            /* Check if the buffer has space to store the data */
            if (app_gatt_db_ext_attr_tbl[i].cur_len <= max_len)
            {
                /* Value fits within the supplied buffer; copy over the value */
                *p_len = app_gatt_db_ext_attr_tbl[i].cur_len;
                memcpy(p_val, app_gatt_db_ext_attr_tbl[i].p_data, app_gatt_db_ext_attr_tbl[i].cur_len);
                res = WICED_BT_GATT_SUCCESS;

                /* Add code for any action required when a particular attribute is read */
                switch ( attr_handle )
                {
                    case HDLC_GAP_DEVICE_NAME_VALUE:
                        break;
                    case HDLC_GAP_APPEARANCE_VALUE:
                        break;
                }
            }
            else
            {
                /* Value to read will not fit within the buffer */
                res = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!isHandleInTable)
    {
        /* Add code to read value for handles not contained within generated lookup table.
         * This is a custom logic that depends on the application, and is not used in the
         * current application. If the value for the current handle is successfully read in the
         * below code snippet, then set the result using:
         * res = WICED_BT_GATT_SUCCESS; */
        switch ( attr_handle )
        {
            default:
                /* The read operation was not performed for the indicated handle */
                WICED_BT_TRACE("Read Request to Invalid Handle: 0x%x\n\r", attr_handle);
                res = WICED_BT_GATT_READ_NOT_PERMIT;
                break;
        }
    }

    return res;
}

/**************************************************************************************************
* Function Name: ble_app_set_value()
***************************************************************************************************
* Summary:
*   This function handles writing to the attribute handle in the GATT database using the
*   data passed from the BT stack. The value to write is stored in a buffer
*   whose starting address is passed as one of the function parameters
*
* Parameters:
*   uint16_t attr_handle                    : Attribute handle for write operation
*   uint16_t conn_id                        : Connection ID
*   uint8_t *p_val                          : Pointer to the buffer that stores the data to be written
*   uint16_t len                            : Length of data to be written
*
* Return:
*   wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
static wiced_bt_gatt_status_t ble_app_set_value(uint16_t attr_handle, uint16_t conn_id, uint8_t *p_val, uint16_t len)
{
    int i = 0;
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bool_t validLen = WICED_FALSE;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    /* Check for a matching handle entry */
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            /* Detected a matching handle in external lookup table */
            isHandleInTable = WICED_TRUE;

            /* Check if the buffer has space to store the data */
            validLen = (app_gatt_db_ext_attr_tbl[i].max_len >= len);

            if (validLen)
            {
                /* Value fits within the supplied buffer; copy over the value */
                app_gatt_db_ext_attr_tbl[i].cur_len = len;
                memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                res = WICED_BT_GATT_SUCCESS;

                /* Add code for any action required when this attribute is written.
                 * In this case, we update the IAS led based on the IAS alert
                 * level characteristic value */

                switch ( attr_handle )
                {
                    case HDLC_IAS_ALERT_LEVEL_VALUE:
                        WICED_BT_TRACE("Alert Level = %d\n\r", app_ias_alert_level[0]);
                        ias_led_update();
                        break;
                }
            }
            else
            {
                /* Value to write does not meet size constraints */
                res = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!isHandleInTable)
    {
        /* TODO: Add code to read value for handles not contained within generated lookup table.
         * This is a custom logic that depends on the application, and is not used in the
         * current application. If the value for the current handle is successfully written in the
         * below code snippet, then set the result using:
         * res = WICED_BT_GATT_SUCCESS; */
        switch ( attr_handle )
        {
            default:
                /* The write operation was not performed for the indicated handle */
                WICED_BT_TRACE("Write Request to Invalid Handle: 0x%x\n\r", attr_handle);
                res = WICED_BT_GATT_WRITE_NOT_PERMIT;
                break;
        }
    }

    return res;
}

/**************************************************************************************************
* Function Name: ble_app_write_handler()
***************************************************************************************************
* Summary:
*   This function handles Write Requests received from the client device
*
* Parameters:
*   wiced_bt_gatt_write_t *p_write_req          : Pointer that contains details of Write Request
*                                                 including the attribute handle
*   uint16_t conn_id                            : Connection ID
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
static wiced_bt_gatt_status_t ble_app_write_handler(wiced_bt_gatt_write_t *p_write_req, uint16_t conn_id)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    /* Attempt to perform the Write Request */
    status = ble_app_set_value(p_write_req->handle, conn_id, p_write_req->p_val, p_write_req->val_len);

    return status;
}

/**************************************************************************************************
* Function Name: ble_app_read_handler()
***************************************************************************************************
* Summary:
*   This function handles Read Requests received from the client device
*
* Parameters:
*   wiced_bt_gatt_write_t *p_read_req           : Pointer that contains details of Read Request
*                                                 including the attribute handle
*   uint16_t conn_id                            : Connection ID
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
static wiced_bt_gatt_status_t ble_app_read_handler( wiced_bt_gatt_read_t *p_read_req, uint16_t conn_id )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    /* Attempt to perform the Read Request */
    status = ble_app_get_value(p_read_req->handle, conn_id, p_read_req->p_val, *p_read_req->p_val_len, p_read_req->p_val_len);

    return status;
}

/**************************************************************************************************
* Function Name: ble_app_connect_callback()
***************************************************************************************************
* Summary:
*   This callback function handles connection status changes.
*
* Parameters:
*   wiced_bt_gatt_connection_status_t *p_conn_status  : Pointer to data that has connection details
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
static wiced_bt_gatt_status_t ble_app_connect_callback(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    if ( NULL != p_conn_status )
    {
        if ( p_conn_status->connected )
        {
            /* Device has connected */
            WICED_BT_TRACE("Connected : BDA '%B', Connection ID '%d'\n\r", p_conn_status->bd_addr, p_conn_status->conn_id );

            /* Store the connection ID */
            bt_connection_id = p_conn_status->conn_id;

            /* Update the adv/conn state */
            app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_ON;
        }
        else
        {
            /* Device has disconnected */
            WICED_BT_TRACE("Disconnected : BDA '%B', Connection ID '%d', Reason '%d'\n\r", p_conn_status->bd_addr, p_conn_status->conn_id, p_conn_status->reason );

            /* Set the connection id to zero to indicate disconnected state */
            bt_connection_id = 0;

            /* Restart the advertisements */
            wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);

            /* Update the adv/conn state */
            app_bt_adv_conn_state = APP_BT_ADV_ON_CONN_OFF;

            /* Turn Off the IAS LED on a disconnection */
            ias_led_update();
        }

        /* Update the advertisement LED to reflect updated state */
        adv_led_update();

        status = WICED_BT_GATT_SUCCESS;
    }

    return status;
}

/**************************************************************************************************
* Function Name: ble_app_server_callback()
***************************************************************************************************
* Summary:
*   This function handles GATT server events from the BT stack.
*
* Parameters:
*   uint16_t conn_id                            : Connection ID
*   wiced_bt_gatt_request_type_t type           : Type of GATT server event
*   wiced_bt_gatt_request_data_t *p_data        : Pointer to GATT server event data
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
static wiced_bt_gatt_status_t ble_app_server_callback(uint16_t conn_id, wiced_bt_gatt_request_type_t type, wiced_bt_gatt_request_data_t *p_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    switch ( type )
    {
        case GATTS_REQ_TYPE_READ:
            /* Attribute read request */
            status = ble_app_read_handler( &p_data->read_req, conn_id );
            break;
        case GATTS_REQ_TYPE_WRITE:
            /* Attribute write request */
            status = ble_app_write_handler( &p_data->write_req, conn_id );
            break;
    }

    return status;
}

/* [] END OF FILE */
