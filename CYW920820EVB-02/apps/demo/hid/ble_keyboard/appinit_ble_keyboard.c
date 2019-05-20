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
 * Entry point to LE keyboard application.
 *
 */
#include "sparcommon.h"
#include "gki_target.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_gpio.h"
#include "wiced_platform.h"
#include "wiced_hal_puart.h"
#include "wiced_hal_wdog.h"
#include "wiced_result.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"
#include "wiced_platform.h"
#include "wiced_memory.h"

#include "wiced_hal_nvram.h"
#include "wiced_hidd_lib.h"
#include "ble_keyboard.h"
#include "blehidhci.h"
#include "blehidgatts.h"

//Local identity key ID
#define  VS_LOCAL_IDENTITY_KEYS_ID WICED_NVRAM_VSID_START

wiced_bt_ble_advert_mode_t app_adv_mode = BTM_BLE_ADVERT_OFF;

extern const uint8_t blehid_db_data[];
extern const uint16_t blehid_db_size;
extern const wiced_bt_cfg_settings_t wiced_bt_hid_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_hid_cfg_buf_pools[];
extern wiced_bt_device_link_keys_t  blehostlist_link_keys;
extern uint16_t blehostlist_flags;

extern wiced_bt_gatt_status_t blekb_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);

#ifdef TESTING_USING_HCI
extern uint8_t blekb_key_std_rpt[];
extern uint8_t blekb_bitmap_rpt[];
extern uint8_t blekb_func_lock_rpt;
static hci_rpt_db_t hci_rpt_db[] =
{
   // rpt_buf,             rpt_type,                    rpt_id,              length (exclude rpt_id)
   {blekb_key_std_rpt,     WICED_HID_REPORT_TYPE_INPUT, STD_KB_REPORT_ID,    KEYRPT_MAX_KEYS_IN_STD_REPORT+2},
   {blekb_bitmap_rpt,      WICED_HID_REPORT_TYPE_INPUT, BITMAPPED_REPORT_ID, KEYRPT_NUM_BYTES_IN_BIT_MAPPED_REPORT},
   {&blekb_func_lock_rpt,  WICED_HID_REPORT_TYPE_INPUT, FUNC_LOCK_REPORT_ID, 1},
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

    //start blekb app
    blekbapp_create();

    return WICED_BT_SUCCESS;
}

/*
 * blekb bt/ble link management callbacks
 * */
wiced_result_t blekb_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_device_link_keys_t *pLinkKeys;
    wiced_bt_ble_advert_mode_t * p_mode;
    uint8_t *p_keys;

    WICED_BT_TRACE("bt_stack_management_cback event%d\n", event);

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            blehid_app_init();
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT\n");
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC_ONLY|BTM_LE_AUTH_REQ_BOND;              /* LE sec bonding */
            p_event_data->pairing_io_capabilities_ble_request.max_key_size = 16;
            p_event_data->pairing_io_capabilities_ble_request.init_keys = 0x0F; //(BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_PLK);
            p_event_data->pairing_io_capabilities_ble_request.resp_keys = 0x0F; //(BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_PLK);
            break;

        case BTM_PAIRING_COMPLETE_EVT:
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

                    //SMP result callback: successful
                    wiced_ble_hidd_host_info_add_first(ble_hidd_link.gatts_peer_addr, ble_hidd_link.gatts_peer_addr_type, &blehostlist_link_keys, blehostlist_flags);
                }
                else
                {
                    WICED_BT_TRACE( " BONDED failed\n");
                    //SMP result callback: failed
                    wiced_ble_hidd_host_info_delete(ble_hidd_link.gatts_peer_addr, ble_hidd_link.gatts_peer_addr_type);
                }
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT %B\n", (uint8_t *)&p_event_data->paired_device_link_keys_update);
            memcpy(&blehostlist_link_keys, &p_event_data->paired_device_link_keys_update, sizeof(wiced_bt_device_link_keys_t));
            wiced_trace_array((uint8_t *)(&(blehostlist_link_keys.key_data)), BTM_SECURITY_KEY_DATA_LEN);
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            pLinkKeys = (wiced_bt_device_link_keys_t *)wiced_ble_hidd_host_info_get_link_keys();
            WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT\n");
            if (pLinkKeys && !memcmp(pLinkKeys->bd_addr, p_event_data->paired_device_link_keys_request.bd_addr, BD_ADDR_LEN))
            {
                memcpy(&p_event_data->paired_device_link_keys_request, pLinkKeys, sizeof(wiced_bt_device_link_keys_t));
            }
            else
            {
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_SECURITY_REQUEST_EVT:
             /* Use the default security */
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr,  WICED_BT_SUCCESS);
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            WICED_BT_TRACE("BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT\n");
            /* save keys to NVRAM */
            p_keys = (uint8_t *)&p_event_data->local_identity_keys_update;
            wiced_hal_write_nvram ( VS_LOCAL_IDENTITY_KEYS_ID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
            WICED_BT_TRACE("local keys save to NVRAM result: %d \n", result);
            break;

        case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            WICED_BT_TRACE("BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT\n");
            /* read keys from NVRAM */
            p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
            wiced_hal_read_nvram( VS_LOCAL_IDENTITY_KEYS_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
            WICED_BT_TRACE("local keys read from NVRAM result: %d\n",  result);
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            p_mode = &p_event_data->ble_advert_state_changed;

            WICED_BT_TRACE( "Advertisement State Change: %d\n", *p_mode);

            //if high duty cycle directed advertising stops
            if ( (app_adv_mode == BTM_BLE_ADVERT_DIRECTED_HIGH) &&
                     (*p_mode == BTM_BLE_ADVERT_DIRECTED_LOW))
            {
                app_adv_mode = *p_mode;
                wiced_ble_hidd_link_directed_adv_stop();
            }
            // btstack will switch to low adv mode automatically when high adv mode timeout,
            // for HIDD, we want to stop adv instead
            else if ((app_adv_mode == BTM_BLE_ADVERT_UNDIRECTED_HIGH) &&
                         (*p_mode == BTM_BLE_ADVERT_UNDIRECTED_LOW))
            {
                app_adv_mode = *p_mode;
                wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
            }
            else if ((app_adv_mode == BTM_BLE_ADVERT_UNDIRECTED_LOW) &&
                         (*p_mode == BTM_BLE_ADVERT_OFF))
            {
                app_adv_mode = *p_mode;
                wiced_ble_hidd_link_adv_stop();
            }
            else
            {
                app_adv_mode = *p_mode;
            }
            break;

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
                WICED_BT_TRACE( "Scan State Change: %d\n", p_event_data->ble_scan_state_changed );
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            WICED_BT_TRACE("BTM_ENCRYPTION_STATUS_EVT");
            if (p_event_data->encryption_status.result == WICED_SUCCESS)
            {
                WICED_BT_TRACE(" link encrypted\n");
                wiced_blehidd_set_link_encrypted_flag(WICED_TRUE);
            }
            else
            {
                WICED_BT_TRACE(" Encryption failed:%d\n", p_event_data->encryption_status.result);
            }
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            WICED_BT_TRACE(" BTM_BLE_CONNECTION_PARAM_UPDATE status:%d\n", p_event_data->ble_connection_param_update.status);
            if (!p_event_data->ble_connection_param_update.status)
            {
                wiced_ble_hidd_link_conn_update_complete();
            }
            break;

        default:
            WICED_BT_TRACE("\n!!!unprocessing blekb_management_cback: %d!!!\n", event );
            break;
    }

    hci_control_le_handle_event(event, p_event_data);

    return result;
}

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
void application_start( void )
{
#ifdef WICED_BT_TRACE_ENABLE
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
#endif
    WICED_BT_TRACE("kbApp start\n");

    //restore content from AON memory when wake up from shutdown sleep (SDS)
    kbapp_aon_restore();

    wiced_bt_stack_init (blekb_management_cback,
        &wiced_bt_hid_cfg_settings,
        wiced_bt_hid_cfg_buf_pools);

    hci_control_le_init(HCI_CONTROL_RPT_CNT, hci_rpt_db);
}

