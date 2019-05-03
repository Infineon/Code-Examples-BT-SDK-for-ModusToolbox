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
 * GATT callback function and handlers
 *
 */

#include "wiced_bt_trace.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_gpio.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#include "wiced_bt_app_hal_common.h"
#endif
#include "wiced_hal_nvram.h"
#include "wiced_result.h"
#include "ble_keyboard_gatts.h"
#include "blehidlink.h"

#ifdef TESTING_USING_HCI
#include "wiced_transport.h"
#include "hci_control_api.h"
#endif

#ifdef OTA_FIRMWARE_UPGRADE
#include "wiced_bt_ota_firmware_upgrade.h"

#ifdef OTA_SECURE_FIRMWARE_UPGRADE
#include "bt_types.h"
#include "p_256_multprecision.h"
#include "p_256_ecc_pp.h"

// If secure version of the OTA firmware upgrade is used, the app should be linked with the ecdsa256_pub.c
// which exports the public key
extern Point    ecdsa256_public_key;
#endif
static uint8_t  ota_fw_upgrade_initialized = WICED_FALSE;

extern void     blekbapp_ota_fw_upgrade_status(uint8_t status);
#endif

extern const attribute_t blehid_gattAttributes[];
extern const uint16_t blehid_gattAttributes_size;
extern const wiced_bt_cfg_settings_t wiced_bt_hid_cfg_settings;

#ifdef PTS_HIDS_CONFORMANCE_TC_CW_BV_03_C
uint16_t last_written_gatt_attr_handle;
uint16_t last_written_gatt_attr_length;
#endif

const attribute_t * blehid_get_attribute(uint16_t handle)
{
    const attribute_t * puAttributes = blehid_gattAttributes;
    uint16_t limit = blehid_gattAttributes_size;

    while(limit--)
    {
        if(puAttributes->handle == handle)
            break;

        puAttributes++;
    }
    //WICED_BT_TRACE("Search %d attr %d %x\n", handle, limit, puAttributes);

    if(limit < 0)
    {
        //WICED_BT_TRACE("\b attribute not FOUND!!!");
        return NULL;
    }

    return puAttributes;
}

/*
 * Process Read request or command from peer device
 * */
wiced_bt_gatt_status_t blekb_gatts_req_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data )
{
    const attribute_t * puAttribute;
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;
    uint16_t attr_len_to_copy;

    if(!p_read_data)
    {
        return WICED_BT_GATT_ERROR;
    }

#ifdef OTA_FIRMWARE_UPGRADE
    // if read request is for the OTA FW upgrade service, pass it to the library to process
    if (p_read_data->handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        return wiced_ota_fw_upgrade_read_handler(conn_id, p_read_data);
    }
#endif
    //WICED_BT_TRACE("read_hndlr conn %d hdl 0x%x\n", conn_id, p_read_data->handle );

    puAttribute = blehid_get_attribute(p_read_data->handle);
    if(!puAttribute)
    {
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    attr_len_to_copy = puAttribute->attr_len;

#ifdef PTS_HIDS_CONFORMANCE_TC_CW_BV_03_C
    if( puAttribute->handle == last_written_gatt_attr_handle )
    {
        attr_len_to_copy = last_written_gatt_attr_length;
    }
#endif

    //WICED_BT_TRACE("attr_len_to_copy: %d offset: %d\n", attr_len_to_copy, p_read_data->offset);

    if(p_read_data->offset >= puAttribute->attr_len)
    {
        attr_len_to_copy = 0;
        result = WICED_BT_GATT_INVALID_OFFSET;
        WICED_BT_TRACE("WICED_BT_GATT_INVALID_OFFSET");
    }

    if(attr_len_to_copy)
    {
        const uint8_t * from;
        uint16_t mtu;
        uint16_t to_copy = attr_len_to_copy - p_read_data->offset;

        mtu = wiced_blehidd_get_att_mtu_size(ble_hidd_link.gatts_peer_addr);

        if(to_copy >= mtu)
        {
            to_copy = mtu - 1;
        }

        from = ((uint8_t *)puAttribute->p_attr) + p_read_data->offset;
        *p_read_data->p_val_len = to_copy;

        memcpy( p_read_data->p_val, from, to_copy);
    }

    return result;
}

/*
 * Process write request or command from peer device
 * */
wiced_bt_gatt_status_t blekb_gatts_req_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data )
{
    attribute_t *puAttribute;

    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

    // NOTE: the gatt connection id is not the connection handler in the controller.
    if( conn_id != ble_hidd_link.gatts_conn_id)
    {
         return WICED_BT_GATT_ERROR;
    }

#ifdef OTA_FIRMWARE_UPGRADE
    // if write request is for the OTA FW upgrade service, pass it to the library to process
    if (p_data->handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        if (!ota_fw_upgrade_initialized)
        {
            /* OTA Firmware upgrade Initialization */
#ifdef OTA_SECURE_FIRMWARE_UPGRADE
            if (wiced_ota_fw_upgrade_init(&ecdsa256_public_key, blekbapp_ota_fw_upgrade_status, NULL) == WICED_FALSE)
#else
            if (wiced_ota_fw_upgrade_init(NULL, blekbapp_ota_fw_upgrade_status, NULL) == WICED_FALSE)
#endif
            {
                WICED_BT_TRACE("OTA upgrade Init failure!!! \n");
                return WICED_BT_GATT_ERR_UNLIKELY;
            }
            ota_fw_upgrade_initialized = WICED_TRUE;
        }
        result = wiced_ota_fw_upgrade_write_handler(conn_id, p_data);
    }
    else
#endif
    {
        //WICED_BT_TRACE("write_handler: conn %d hdl %x prep %d off %d len %d \n ", conn_id, p_data->handle, p_data->is_prep, p_data->offset,p_data->val_len );

        puAttribute = (attribute_t *)blehid_get_attribute(p_data->handle);

        if(puAttribute)
        {
            if(p_data->offset > puAttribute->attr_len)
            {
                result = WICED_BT_GATT_INVALID_OFFSET;
            }
            else if((p_data->val_len + p_data->offset) > puAttribute->attr_len)
            {
                result = WICED_BT_GATT_INVALID_ATTR_LEN;
            }

#ifdef PTS_HIDS_CONFORMANCE_TC_CW_BV_03_C
            last_written_gatt_attr_handle = puAttribute->handle;
            last_written_gatt_attr_length = p_data->val_len;
#endif

            result = wiced_blehidd_write_handler(p_data);
        }
        else
        {
            //result = wiced_bt_gatt_legattdb_dispatchWriteCb(p_data);
            result = WICED_BT_GATT_INVALID_HANDLE;
        }
    }

    // Whenever there is an activity, restart the idle timer
    if (ble_hidd_link.conn_idle_timeout)
    {
        osapi_activateTimer( &ble_hidd_link.conn_idle_timer, ble_hidd_link.conn_idle_timeout * 1000000UL); //timout in micro seconds.
        ble_hidd_link.osapi_app_timer_start_instant = clock_SystemTimeMicroseconds64();
        ble_hidd_link.osapi_app_timer_running |= BLEHIDLINK_CONNECTION_IDLE_TIMER;
        ble_hidd_link.osapi_app_timer_running |= 1;
    }

    return result;
}

/* This function is invoked when connection is established */
wiced_bt_gatt_status_t blekb_gatts_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    WICED_BT_TRACE( "blekb_conn_up, id %d: %B\n",  p_status->conn_id, p_status->bd_addr);

    //configure ATT MTU size with peer device
    wiced_bt_gatt_configure_mtu(p_status->conn_id, wiced_bt_hid_cfg_settings.gatt_cfg.max_mtu_size);

    ble_hidd_link.gatts_conn_id = p_status->conn_id;
    ble_hidd_link.gatts_peer_addr_type = p_status->addr_type;
    memcpy(ble_hidd_link.gatts_peer_addr, p_status->bd_addr, BD_ADDR_LEN);

    wiced_ble_hidd_link_connected();

#ifdef OTA_FIRMWARE_UPGRADE
    // Pass connection up/down event to the OTA FW upgrade library
    wiced_ota_fw_upgrade_connection_status_event(p_status);
#endif

#ifdef TESTING_USING_HCI
    wiced_transport_send_data( HCI_CONTROL_HID_EVENT_OPENED, NULL, 0 );
#endif

    return WICED_BT_GATT_SUCCESS;
}

/*
 * This function is invoked when connection is lost
 * */
wiced_bt_gatt_status_t blekb_gatts_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    WICED_BT_TRACE( "connection_down conn %d reason %d\n",  p_status->conn_id, p_status->reason );
    ble_hidd_link.gatts_conn_id = 0;
    wiced_ble_hidd_link_disconnected();
    return WICED_BT_GATT_SUCCESS;
}

wiced_bt_gatt_status_t blekb_gatts_conn_status_cb( wiced_bt_gatt_connection_status_t *p_status )
{
    if(p_status->connected)
    {
        return blekb_gatts_connection_up( p_status );
    }
    else
    {
        return blekb_gatts_connection_down( p_status );
    }
}

/*
 * Process indication confirm. If client wanted us to use indication instead of
 * notifications we have to wait for confirmation after every message sent.
 * For example if user pushed button twice very fast
 * we will send first message, then
 * wait for confirmation, then
 * send second message, then
 * wait for confirmation and
 * if configured start idle timer only after that.
 */
wiced_bt_gatt_status_t blekb_gatts_req_conf_handler( uint16_t conn_id, uint16_t handle )
{
    WICED_BT_TRACE( "blekb_indication_cfm, conn %d hdl %d\n", conn_id, handle );

#ifdef OTA_FIRMWARE_UPGRADE
    // if indication confirmation is for the OTA FW upgrade service, pass it to the library to process
    if (handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        return wiced_ota_fw_upgrade_indication_cfm_handler(conn_id, handle);
    }
#endif

    return WICED_BT_GATT_SUCCESS;

}

wiced_bt_gatt_status_t blekb_gatts_req_cb( wiced_bt_gatt_attribute_request_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

    //WICED_BT_TRACE( "_blekb_gatts_req_cb. conn %d, type %d\n", p_data->conn_id, p_data->request_type );
    switch ( p_data->request_type )
    {
        case GATTS_REQ_TYPE_READ:
            result = blekb_gatts_req_read_handler(p_data->conn_id, &(p_data->data.read_req));
            break;

        case GATTS_REQ_TYPE_WRITE:
        case GATTS_REQ_TYPE_PREP_WRITE:
            result = blekb_gatts_req_write_handler(p_data->conn_id, &(p_data->data.write_req));
             break;

        case GATTS_REQ_TYPE_MTU:
            WICED_BT_TRACE("GATTS_REQ_TYPE_MTU\n");
            break;

       case GATTS_REQ_TYPE_CONF:
           result = blekb_gatts_req_conf_handler( p_data->conn_id, p_data->data.handle );
           break;

       default:
            WICED_BT_TRACE("Please check this blekb_gatts_req_cb!!!\n");
            break;
    }

    //Start a timer to make sure the packet is sent over the air before enter SDS
    if (wiced_is_timer_in_use(&ble_hidd_link.allowSDS_timer))
    {
        wiced_stop_timer(&ble_hidd_link.allowSDS_timer);
    }
    wiced_start_timer(&ble_hidd_link.allowSDS_timer,1000);// 1 second. timeout in ms
    ble_hidd_link.allowSDS = 0;

    return result;
}

wiced_bt_gatt_status_t blekb_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

    //WICED_BT_TRACE("\nblekb_gatts_callback event: 0x%x\n", event);

    switch(event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            result = blekb_gatts_conn_status_cb(&p_data->connection_status);
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            result = blekb_gatts_req_cb(&p_data->attribute_request);
            break;

        case GATT_CONGESTION_EVT:
            WICED_BT_TRACE("congested:%d\n", p_data->congestion.congested);
            break;

        default:
            WICED_BT_TRACE("Please check this unhandled event!!!:0x%x\n", event);
            break;
    }

    return result;
}
