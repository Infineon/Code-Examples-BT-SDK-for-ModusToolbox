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
 * Entry point to LE remote control application.
 *
 */
#include "wiced_bt_trace.h"
#include "wiced_hidd_lib.h"
#include "blehidhci.h"
#include "ble_remote.h"

extern void sfi_allow_deep_sleep(void);

#ifdef TESTING_USING_HCI
static hci_rpt_db_t hci_rpt_db[] =
{
   // rpt_buf,             rpt_type,                    rpt_id,              length (exclude rpt_id)
   {bleremote_key_std_rpt, WICED_HID_REPORT_TYPE_INPUT, STD_KB_REPORT_ID,    KEYRPT_LEN},
   {bleremote_bitmap_rpt,  WICED_HID_REPORT_TYPE_INPUT, BITMAPPED_REPORT_ID, KEYRPT_NUM_BYTES_IN_BIT_MAPPED_REPORT},
};
#define HCI_CONTROL_RPT_CNT (sizeof(hci_rpt_db)/sizeof(hci_rpt_db_t))
#endif
/******************************************************************************
 *                          Function Definitions
******************************************************************************/
wiced_result_t blehid_app_init(void)
{
    /*  GATT DB Initialization  */
    if ( blehid_gatts_init( blehid_db_data, blehid_db_size, NULL, NULL ) != WICED_BT_SUCCESS )
    {
        return WICED_BT_ERROR;
    }

    /* general hid app init */
    wiced_hidd_app_init(BT_DEVICE_TYPE_BLE);

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    //start bleremote app
    bleremoteapp_create();

    return WICED_BT_SUCCESS;
}

#ifdef HCI_TRACES_ENABLED
void myapp_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    WICED_BT_TRACE( "\nHCI event type %d len %d\n", type,length );
    wiced_trace_array(  p_data, length );

}
#endif

/*
 * bleremote ble link management callbacks
 */
wiced_result_t bleremote_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_device_address_t         bda = { 0 };

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
#ifdef HCI_TRACES_EANBLED
            /* Register callback for receiving hci traces */
            wiced_bt_dev_register_hci_trace( myapp_hci_trace_cback );
#else
            hci_control_le_enable_trace();
#endif
            wiced_bt_dev_read_local_addr(bda);
            WICED_BT_TRACE("Address: [%B]\n", bda);
            blehid_app_init();
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            WICED_BT_TRACE("BTM_PAIRING_COMPLETE_EVT ");
            if(p_event_data->pairing_complete.transport == BT_TRANSPORT_LE)
            {
                wiced_bt_dev_ble_pairing_info_t * p_info;
                p_info =  &p_event_data->pairing_complete.pairing_complete_info.ble;
                WICED_BT_TRACE( "LE Pairing Complete: %x\n",p_info->reason);

                //bonding successful
                if (!p_info->reason )
                {
                    WICED_BT_TRACE( "BONDED successful\n");
                    if (!wiced_blehidd_is_device_bonded())
                    {
                        WICED_BT_TRACE( "set device bonded flag\n");
                        wiced_blehidd_set_device_bonded_flag(WICED_TRUE);
                    }

#ifdef CONNECTED_ADVERTISING_SUPPORTED
                    //If there is a connection existing, delete pairing information and disconnect existing connection
                    if (ble_hidd_link.second_conn_state == BLEHIDLINK_2ND_CONNECTION_PENDING)
                    {
                        ble_hidd_link.second_conn_state = BLEHIDLINK_2ND_CONNECTION_NOT_ALLOWED;

                        if (wiced_ble_hidd_host_info_is_bonded())
                        {
                            uint8_t *bonded_bdadr = (uint8_t *)wiced_ble_hidd_host_info_get_bdaddr();

                            WICED_BT_TRACE("remove bonded device : %B\n", bonded_bdadr);
                            wiced_bt_dev_delete_bonded_device(bonded_bdadr);
                        }

                        WICED_BT_TRACE("Removing all bonded info\n");
                        wiced_ble_hidd_host_info_delete_all();

                        //disconnect existing connection
                        wiced_bt_gatt_disconnect(ble_hidd_link.existing_connection_gatts_conn_id);
                    }
#endif

                    //SMP result callback: successful
                    wiced_ble_hidd_host_info_add_first(ble_hidd_link.gatts_peer_addr, ble_hidd_link.gatts_peer_addr_type, &blehostlist_link_keys, blehostlist_flags);
                }
                else
                {
                    //SMP result callback: failed
                    WICED_BT_TRACE( " BONDED failed\n");
#ifdef CONNECTED_ADVERTISING_SUPPORTED
                    //If this is from the new connection
                    if (ble_hidd_link.second_conn_state == BLEHIDLINK_2ND_CONNECTION_PENDING)
                    {
                        uint16_t temp_gatts_conn_id = ble_hidd_link.gatts_conn_id;

                        WICED_BT_TRACE("delete the new connection: %d\n", temp_gatts_conn_id);

                        ble_hidd_link.second_conn_state = BLEHIDLINK_2ND_CONNECTION_NOT_ALLOWED;

                        //recover current connection gatt connection id
                        ble_hidd_link.gatts_conn_id = ble_hidd_link.existing_connection_gatts_conn_id;

                        //disconnect new connection
                        wiced_bt_gatt_disconnect(temp_gatts_conn_id);

                        //restore embeded controller info for the LE link (peer device info, bonded, encrypted, connection parameters etc.)
                        memcpy(&emConInfo_devInfo, &ble_hidd_link.existing_emconinfo, sizeof(EMCONINFO_DEVINFO));
                    }
                    else
#endif
                    wiced_ble_hidd_host_info_delete(ble_hidd_link.gatts_peer_addr, ble_hidd_link.gatts_peer_addr_type);
                }
            }
            hci_control_le_handle_event(event, p_event_data);
            break;

        default:
            // we didn't handle this event, let default library handler to deal with it.
            result = WICED_NOT_FOUND;
            break;
    }
    return result;
}

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
void application_start( void )
{
    sfi_allow_deep_sleep();

    //restore content from AON memory
    bleremoteapp_aon_restore();

    wiced_ble_hidd_start("ble_remote", blehid_app_init, bleremote_management_cback, &wiced_bt_hid_cfg_settings, wiced_bt_hid_cfg_buf_pools);
    hci_control_le_init(HCI_CONTROL_RPT_CNT, hci_rpt_db);
}
