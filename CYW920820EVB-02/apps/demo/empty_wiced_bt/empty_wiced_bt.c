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
* WICED Bluetooth Template App
*
* This application is provided as a starting place for adding new code and
* functionality. As is, the app only outputs a trace message on init.
*
* Features demonstrated: none
*
* To use the app, work through the following steps.
* 1. Plug the eval board into your computer
* 2. Open a terminal such as "putty" to receive messages from the puart.
* 3. Build and download the application (to the eval board)
* 4. Observe the puart terminal to see the WICED_BT_TRACE message.
* 5. Add new code and functionality.
*/

#include "app_bt_cfg.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "GeneratedSource/cycfg_gatt_db.h"

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static wiced_bt_dev_status_t  app_bt_management_callback    ( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
wiced_result_t app_set_advertisement_data(void);
wiced_bt_gatt_status_t app_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data );

/*******************************************************************************
* Function Name: void application_start(void)
********************************************************************************
* Summary: Entry point to the application. Initialize transport configuration
*          and register BLE management event callback. The actual application
*          initialization will happen when stack reports that BT device is ready
*
* Parameters:
*   None
*
* Return:
*  None
*
********************************************************************************/
void application_start(void)
{
    #if ((defined WICED_BT_TRACE_ENABLE) || (defined HCI_TRACE_OVER_TRANSPORT))
        /* Select Debug UART setting to see debug traces on the appropriate port */
        wiced_set_debug_uart(  WICED_ROUTE_DEBUG_TO_PUART );
    #endif

    WICED_BT_TRACE("**** App Start **** \n\r");

    /* TODO your app init code */

    /* Initialize Stack and Register Management Callback */
    wiced_bt_stack_init(app_bt_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);
}

/**************************************************************************************************
* Function Name: wiced_bt_dev_status_t app_bt_management_callback(wiced_bt_management_evt_t event,
*                                                  wiced_bt_management_evt_data_t *p_event_data)
***********************************************************************************************    ***
* Summary:
*   This is a Bluetooth stack management event handler function to receive events from
*   BLE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : BLE event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to BLE management event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
***********************************************************************************************/

/* Bluetooth Management Event Handler */
wiced_result_t app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t status = WICED_BT_SUCCESS;
    // wiced_bt_gatt_status_t gatt_status;

    switch (event)
    {
    case BTM_ENABLED_EVT:
        /* Bluetooth Controller and Host Stack Enabled */
        WICED_BT_TRACE("Bluetooth Enabled (%s)\n",
                ((WICED_BT_SUCCESS == p_event_data->enabled.status) ? "success" : "failure"));

        /* TODO - register for GATT callbacks if needed by your application */
        // gatt_status = wiced_bt_gatt_register( app_gatt_callback );


        /* TODO - initialize GATT database created by Bluetooth Configurator here if needed by your application */
        // gatt_status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len);
        // WICED_BT_TRACE("GATT status:d\n", gatt_status);

        /* TODO - set advertisement data for your app */
        // app_set_advertisement_data();

        /* TODO - start advertisement */
        // wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, BLE_ADDR_PUBLIC, NULL));

        /* TODO - further initialization here */

        break;

    /* TODO - Handle Bluetooth Management Event
    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT: // IO capabilities request
         break;

     case BTM_PAIRING_COMPLETE_EVT: // Pairing Complete event
         break;

     case BTM_ENCRYPTION_STATUS_EVT: // Encryption Status Event
         break;

     case BTM_SECURITY_REQUEST_EVT: // security accesss
         break;

     case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT: // save link keys with app
         break;

      case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT: // retrieval saved link keys
         break;

     case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT: // save keys to NVRAM
         break;

     case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT: // read keys from NVRAM
         break;

     case BTM_BLE_SCAN_STATE_CHANGED_EVT: // Scan State Change
         break;
    */

    default:
        WICED_BT_TRACE("Unhandled Bluetooth Management Event: 0x%x (%d)\n", event, event);
        break;
    }

    return status;
}

/* TODO Set advertisement data for your application
wiced_result_t app_set_advertisement_data(void)
{

    wiced_bt_ble_advert_elem_t  adv_elem[3];
    wiced_result_t              result;
    uint8_t         num_elem                = 0;
    uint8_t         flag                    = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;


    adv_elem[num_elem].advert_type          = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len                  = sizeof(uint8_t);
    adv_elem[num_elem].p_data               = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type          = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len                  = strlen((const char *) wiced_bt_cfg_settings.device_name);
    adv_elem[num_elem].p_data               = (uint8_t *) wiced_bt_cfg_settings.device_name;
    num_elem++;

    result = wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);

    return result;
}
*/

/* TODO Handle GATT event callbacks if needed by your app
wiced_bt_gatt_status_t app_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    wiced_bt_gatt_attribute_request_t *p_attr_req = &p_event_data->attribute_request;
    wiced_bt_gatt_connection_status_t *p_conn_status = &p_data->connection_status;

    switch( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
        {
            if (p_conn_status->connected) // Device has connected
            {
            }
            else // Device has disconnected
            {
            }
        }
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
        {
            switch (p_attr_req->request_type)
            {
            case GATTS_REQ_TYPE_READ: // read request
               break;

            case GATTS_REQ_TYPE_WRITE: // write request
               break;

            case GATTS_REQ_TYPE_CONF: // confirm request
               break;
            }
        }
            break;

        default:
            break;
    }
    return status;
}
*/
