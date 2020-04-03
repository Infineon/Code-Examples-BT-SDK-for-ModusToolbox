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


/* @file
 * thermistor_gatt_handler.c
 *
 * @brief
 * This file consists of the utility functions that will help debugging and
 * developing the applications easier with much more meaningful information.
 *
 */


/* *******************************************************************
 *                              INCLUDES
 * *******************************************************************/
#include "thermistor_gatt_handler.h"
#include "thermistor_util_functions.h"
#include "GeneratedSource/cycfg_gatt_db.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_ota_firmware_upgrade.h"

/* *******************************************************************
 *                              FUNCTION DEFINITIONS
 * *******************************************************************/
/*
 Function Name:
 thermistor_event_handler

 Function Description:
 @brief  This Function handles the all the GATT events - GATT Event Handler

 @param event            BLE GATT event type
 @param p_event_data     Pointer to BLE GATT event data

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
wiced_bt_gatt_status_t
thermistor_event_handler(wiced_bt_gatt_evt_t event,
                         wiced_bt_gatt_event_data_t *p_event_data)
{

    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_bt_gatt_connection_status_t *p_conn_status = NULL;
    wiced_bt_gatt_attribute_request_t *p_attr_req = NULL;

    switch (event)
    {

    case GATT_CONNECTION_STATUS_EVT:
        status = thermistor_connect_callback(&p_event_data->connection_status);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        p_attr_req = &p_event_data->attribute_request;
        status = thermistor_server_callback(p_attr_req->conn_id,
                                            p_attr_req->request_type,
                                            &p_attr_req->data);
        break;

    default:
        WICED_BT_TRACE("Other than GATT_CONNECTION_STATUS_EVT and "
                       "GATT_ATTRIBUTE_REQUEST_EVT\r\n");
        status = WICED_BT_GATT_SUCCESS;
        break;

    }

    return status;
}


/*
 Function Name:
 thermistor_connect_callback

 Function Description:
 @brief  The callback function is invoked when GATT_CONNECTION_STATUS_EVT occurs
         in GATT Event handler function

 @param p_conn_status     Pointer to BLE GATT connection status

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
wiced_bt_gatt_status_t
thermistor_connect_callback(wiced_bt_gatt_connection_status_t *p_conn_status)
{

    wiced_result_t gatt_status = WICED_BT_GATT_ERROR;

    if ((p_conn_status->connected) && (thermistor_conn_id == 0))
    {
        /* Device has connected */
        WICED_BT_TRACE("\r\nConnected to BDA: '%B', Connection ID: '%d'\r\n",
                        p_conn_status->bd_addr,
                        p_conn_status->conn_id);
        wiced_hal_gpio_set_pin_output(CONNECTION_LED, GPIO_PIN_OUTPUT_LOW);
        thermistor_conn_id  = p_conn_status->conn_id;
        gatt_status         = wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF,
                                                            BLE_ADDR_PUBLIC,
                                                            NULL);

        /* Setting 2M PHY BLE connection if the peer device supports */
        if (WICED_SUCCESS != set_ble_2m_phy(p_conn_status))
        {
            WICED_BT_TRACE("\r\nUnable to configure "
                           "preferred PHY as LE 2M PHY\r\n");
        }
    }
    else
    {
        /* Device has disconnected */
        WICED_BT_TRACE("\r\nDisconnected : BDA '%B', Connection ID '%d'\r\n",
                        p_conn_status->bd_addr, p_conn_status->conn_id);
        WICED_BT_TRACE("\r\nReason for disconnection: \t");
        WICED_BT_TRACE(gatt_disconn_reason_name(p_conn_status->reason));
        WICED_BT_TRACE("\r\n");
        wiced_hal_gpio_set_pin_output(CONNECTION_LED, GPIO_PIN_OUTPUT_HIGH);

        /* Handle the disconnection */
        thermistor_conn_id  = 0;

        /*
         * Reset the CCCD value so that on a reconnect CCCD (notifications)
         * will be off
         */
        app_ess_temperature_client_char_config[0] = 0;
        gatt_status = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                                    BLE_ADDR_PUBLIC,
                                                    NULL);
    }

    /* Pass connection up/down event to the OTA FW upgrade library */
    wiced_ota_fw_upgrade_connection_status_event(p_conn_status);

    return gatt_status;
}

/*
 Function Name:
 thermistor_server_callback

 Function Description:
 @brief  The callback function is invoked when GATT_ATTRIBUTE_REQUEST_EVT occurs
         in GATT Event handler function. GATT Server Event Callback function.

 @param conn_id  Connection ID from GATT Connection event
 @param type     GATT Request type
 @param p_data   Pointer to BLE GATT request data

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
wiced_bt_gatt_status_t
thermistor_server_callback(uint16_t conn_id,
                           wiced_bt_gatt_request_type_t type,
                           wiced_bt_gatt_request_data_t *p_data)
{

    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    switch (type)
    {

    case GATTS_REQ_TYPE_READ:
        status = thermistor_read_handler(&p_data->read_req, conn_id);
        break;

    case GATTS_REQ_TYPE_WRITE:
        status = thermistor_write_handler(&p_data->write_req, conn_id);
        break;

    case GATTS_REQ_TYPE_CONF:
        status = thermistor_indication_cfm_handler(conn_id, p_data->handle);
        break;
    }

    return status;
}

/*
 Function Name:
 thermistor_write_handler

 Function Description:
 @brief  The function is invoked when GATTS_REQ_TYPE_WRITE is received from the
         client device and is invoked GATT Server Event Callback function. This
         handles "Write Requests" received from Client device.

 @param p_write_req   Pointer to BLE GATT write request
 @param conn_id  Connection ID from GATT Connection event

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
wiced_bt_gatt_status_t
thermistor_write_handler(wiced_bt_gatt_write_t *p_write_req,
                         uint16_t conn_id)
{

    switch(p_write_req->handle)
    {
    /*
     * If write request is for the OTA FW upgrade service, pass it to the
     * library to process
     */
        case HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT:
        case HANDLE_OTA_FW_UPGRADE_CONTROL_POINT:
        case HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR:
        case HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_DATA:
        case HANDLE_OTA_FW_UPGRADE_DATA:
        case HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_APP_INFO:
        case HANDLE_OTA_FW_UPGRADE_APP_INFO:
                return wiced_ota_fw_upgrade_write_handler(conn_id, p_write_req);
                break;
        default:
            /* Handle normal (non-OTA) indication confirmation requests here */
            /* Attempt to perform the Write Request */
            return thermistor_set_value(p_write_req->handle,
                                        conn_id,
                                        p_write_req->p_val,
                                        p_write_req->val_len);
            break;
    }
}

/*
 Function Name:
 thermistor_read_handler

 Function Description:
 @brief  The function is invoked when GATTS_REQ_TYPE_READ is received from the
         client device and is invoked by GATT Server Event Callback function.
         This handles "Read Requests" received from Client device

 @param p_read_req   Pointer to BLE GATT read request
 @param conn_id  Connection ID from GATT Connection event

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
wiced_bt_gatt_status_t
thermistor_read_handler(wiced_bt_gatt_read_t *p_read_req,
                        uint16_t conn_id)
{

    switch(p_read_req->handle)
    {
    /*
     * If read request is for the OTA FW upgrade service, pass it to the
     * library to process
     */
        case HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT:
        case HANDLE_OTA_FW_UPGRADE_CONTROL_POINT:
        case HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR:
        case HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_DATA:
        case HANDLE_OTA_FW_UPGRADE_DATA:
        case HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_APP_INFO:
        case HANDLE_OTA_FW_UPGRADE_APP_INFO:
                return wiced_ota_fw_upgrade_read_handler(conn_id, p_read_req);
                break;
        default:
             /* Handle normal (non-OTA) indication confirmation requests here */
             /* Attempt to perform the Read Request */
                return thermistor_get_value(p_read_req->handle,
                                            conn_id,
                                            p_read_req->p_val,
                                            *p_read_req->p_val_len,
                                            p_read_req->p_val_len);
                break;
    }

}

/*
 Function Name:
 thermistor_indication_cfm_handler

 Function Description:
 @brief  The function is invoked when GATTS_REQ_TYPE_CONF is received from the
         client device and is invoked by GATT Server Event Callback function.
         This handles "Confirmation Requests" received from Client device.

 @param conn_id  Connection ID from GATT Connection event
 @param handle   Connection handle

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
/*
 * Process indication_confirm from peer device
 */
wiced_bt_gatt_status_t
thermistor_indication_cfm_handler(uint16_t conn_id, uint16_t handle)
{

    switch(handle)
    {
    /*
     * If confirmation request is for the OTA FW upgrade service, pass it to the
     * library to process
     */
        case HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT:
        case HANDLE_OTA_FW_UPGRADE_CONTROL_POINT:
        case HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR:
        case HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_DATA:
        case HANDLE_OTA_FW_UPGRADE_DATA:
        case HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_APP_INFO:
        case HANDLE_OTA_FW_UPGRADE_APP_INFO:
             return wiced_ota_fw_upgrade_indication_cfm_handler(conn_id, handle);
             break;
        default:
             /* Handle normal (non-OTA) indication confirmation requests here */
             return WICED_BT_GATT_INVALID_HANDLE;
    }

}

/*
 Function Name:
 thermistor_get_value

 Function Description:
 @brief  The function is invoked by thermistor_read_handler to get a Value from
         GATT DB.

 @param attr_handle  GATT attribute handle
 @param conn_id      Connection ID from GATT Connection event
 @param p_val        Pointer to BLE GATT read request value
 @param len          Maximum length of GATT read request
 @param p_len        Pointer to BLE GATT read request length

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
wiced_bt_gatt_status_t
thermistor_get_value(uint16_t attr_handle,
                     uint16_t conn_id,
                     uint8_t *p_val,
                     uint16_t len,
                     uint16_t *p_len)
{

    int i = 0;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        /* Check for a matching handle entry */
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            /* Detected a matching handle in the external lookup table */
            if (app_gatt_db_ext_attr_tbl[i].cur_len <= len)
            {
                /* Value fits within the supplied buffer; copy over the value */
                *p_len = app_gatt_db_ext_attr_tbl[i].cur_len;
                memcpy(p_val,
                       app_gatt_db_ext_attr_tbl[i].p_data,
                       app_gatt_db_ext_attr_tbl[i].cur_len);

                res = WICED_BT_GATT_SUCCESS;
            }
            else
            {
                /* Value to read will not fit within the buffer */
                res = WICED_BT_GATT_INVALID_ATTR_LEN;
                WICED_BT_TRACE("Invalid attribute length\r\n");
            }
            break;
        }
        else
        {
            res = WICED_BT_GATT_INVALID_HANDLE;
        }
    }

    return res;
}

/*
 Function Name:
 thermistor_set_value

 Function Description:
 @brief  The function is invoked by thermistor_write_handler to set a value
         to GATT DB.

 @param attr_handle  GATT attribute handle
 @param conn_id      Connection ID from GATT Connection event
 @param p_val        Pointer to BLE GATT write request value
 @param len          length of GATT write request

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
wiced_bt_gatt_status_t thermistor_set_value(uint16_t attr_handle,
                                            uint16_t conn_id,
                                            uint8_t *p_val,
                                            uint16_t len)
{
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

      /* Check for a matching handle entry */
      if (HDLD_ESS_TEMPERATURE_CLIENT_CHAR_CONFIG == attr_handle)
      {
          /* Verify that size constraints have been met */
          if (app_ess_temperature_client_char_config_len >= len)
          {
              /* Value fits within the supplied buffer; copy over the value */
              memcpy(app_ess_temperature_client_char_config,
                     p_val,
                     len);

              res = WICED_BT_GATT_SUCCESS;
          }
          else
          {
              /* Value to write does not meet size constraints */
              res = WICED_BT_GATT_INVALID_ATTR_LEN;
          }
      }

  return res;
}

/*
 Function Name:
 set_ble_2m_phy

 Function Description:
 @brief  This function configures the PHY layer to support BLE 2Mbps feature.
         The function is invoked by thermistor_connect_callback when the device
         is connected.

 @param p_conn_status   wiced_bt_gatt_connection_status_t to set the PHY
                        preferences for specific BD_ADDR

 @return wiced_bt_dev_status_t      PHY 2M success or failure status
 */
wiced_bt_dev_status_t
set_ble_2m_phy(wiced_bt_gatt_connection_status_t *p_conn_status)
{
        wiced_bt_ble_phy_preferences_t phy_preferences;
        wiced_bt_dev_status_t result;

        memcpy(phy_preferences.remote_bd_addr,
               p_conn_status->bd_addr,
               BD_ADDR_LEN);

        phy_preferences.rx_phys = phy_preferences.tx_phys = BTM_BLE_PREFER_2M_PHY;

        result = wiced_bt_ble_set_phy(&phy_preferences);

        return result;
}
