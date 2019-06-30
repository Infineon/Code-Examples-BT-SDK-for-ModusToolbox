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
 *
 * This file shows how to create a device which implements mesh on/off client model.
 */
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_bt_trace.h"
#ifdef HCI_CONTROL
#include "wiced_transport.h"
#include "hci_control_api.h"
#endif
#include "wiced_bt_mesh_provision.h"

#include "wiced_bt_cfg.h"
extern wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

/******************************************************
 *          Constants
 ******************************************************/
#define MESH_PID                0x3016
#define MESH_VID                0x0002
#define MESH_FWID               0x3016000101010001
#define MESH_CACHE_REPLAY_SIZE  0x0008


/******************************************************
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
// Forward declarations
static void     mesh_app_init(wiced_bool_t is_provisioned);
static uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
extern void mesh_application_init(void);
extern void mesh_set_node_info(uint8_t* data, uint8_t len, uint8_t idx);
void mesh_proxy_client_process_filter_status(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/******************************************************
 *          Variables Definitions
 ******************************************************/
//static uint8_t mesh_element_index;

uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME] = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER]     = { '1', '2', '3', '4', 0, 0, 0, 0 };
uint8_t mesh_system_id[8]                                                  = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

wiced_bt_mesh_core_config_model_t   mesh_element1_models[] =
{
        WICED_BT_MESH_DEVICE,
        WICED_BT_MESH_MODEL_REMOTE_PROVISION_SERVER,
};
#define MESH_APP_NUM_MODELS  (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t))


wiced_bt_mesh_core_config_element_t mesh_elements[] =
{
    {
        .location = MESH_ELEM_LOC_MAIN,                                 // location description as defined in the GATT Bluetooth Namespace Descriptors section of the Bluetooth SIG Assigned Numbers
        .default_transition_time = MESH_DEFAULT_TRANSITION_TIME_IN_MS,  // Default transition time for models of the element in milliseconds
        .onpowerup_state = WICED_BT_MESH_ON_POWER_UP_STATE_RESTORE,     // Default element behavior on power up
        .default_level = 0,                                             // Default value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_min = 1,                                                 // Minimum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_max = 0xffff,                                            // Maximum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .move_rollover = 0,                                             // If true when level gets to range_max during move operation, it switches to min, otherwise move stops.
        .properties_num = 0,                                            // Number of properties in the array models
        .properties = NULL,                                             // Array of properties in the element.
        .sensors_num = 0,                                               // Number of sensors in the sensor array
        .sensors = NULL,                                                // Array of sensors of that element
        .models_num = MESH_APP_NUM_MODELS,                              // Number of models in the array models
        .models = mesh_element1_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
};

wiced_bt_mesh_core_config_t  mesh_config =
{
    .company_id         = MESH_COMPANY_ID_CYPRESS,                  // Company identifier assigned by the Bluetooth SIG
    .product_id         = MESH_PID,                                 // Vendor-assigned product identifier
    .vendor_id          = MESH_VID,                                 // Vendor-assigned product version identifier
    .firmware_id        = MESH_FWID,                                // Vendor-assigned firmware version identifier
    .replay_cache_size  = MESH_CACHE_REPLAY_SIZE,                   // minimum number of replay protection list entries in a device
    .features           = WICED_BT_MESH_CORE_FEATURE_BIT_FRIEND | WICED_BT_MESH_CORE_FEATURE_BIT_RELAY | WICED_BT_MESH_CORE_FEATURE_BIT_GATT_PROXY_SERVER,   // Supports Friend, Relay and GATT Proxy
    .friend_cfg         =                                           // Configuration of the Friend Feature(Receive Window in Ms, messages cache)
    {
        .receive_window        = 200,
        .cache_buf_len         = 300,                               // Length of the buffer for the cache
        .max_lpn_num           = 4                                  // Max number of Low Power Nodes with established friendship. Must be > 0 if Friend feature is supported.
    },
    .low_power          =                                           // Configuration of the Low Power Feature
    {
        .rssi_factor           = 0,                                 // contribution of the RSSI measured by the Friend node used in Friend Offer Delay calculations.
        .receive_window_factor = 0,                                 // contribution of the supported Receive Window used in Friend Offer Delay calculations.
        .min_cache_size_log    = 0,                                 // minimum number of messages that the Friend node can store in its Friend Cache.
        .receive_delay         = 0,                                 // Receive delay in 1 ms units to be requested by the Low Power node.
        .poll_timeout          = 0                                  // Poll timeout in 100ms unite to bt requested by the Low Power node.
    },
    .gatt_client_only          = WICED_FALSE,                       // Can connect to mesh over GATT or ADV
    .elements_num  = (uint8_t)(sizeof(mesh_elements) / sizeof(mesh_elements[0])),   // number of elements on this device
    .elements      = mesh_elements                                  // Array of elements for this device
};

/*
 * Mesh application library will call into application functions if provided by the application.
 */
wiced_bt_mesh_app_func_table_t wiced_bt_mesh_app_func_table =
{
    mesh_app_init,          // application initialization
    NULL,                   // Default SDK platform button processing
    NULL,                   // GATT connection status
    NULL,                   // attention processing
    NULL,                   // notify period set
    mesh_app_proc_rx_cmd,   // WICED HCI command
    NULL,                   // LPN sleep
    NULL                    // factory reset
};


/******************************************************
 *               Function Definitions
 ******************************************************/
void mesh_app_init(wiced_bool_t is_provisioned)
{
    WICED_BT_TRACE("%s \n" , __FUNCTION__);
#if 0
    extern uint8_t wiced_bt_mesh_model_trace_enabled;
    wiced_bt_mesh_model_trace_enabled = WICED_TRUE;
#endif
#if 0
#include "fid_app.h"
    // enable core trace
    extern void wiced_bt_mesh_core_set_trace_level(uint32_t fids_mask, uint8_t level);

    wiced_bt_mesh_core_set_trace_level(0xffffffff, 4);      //(ALL, TRACE_DEBUG)
    wiced_bt_mesh_core_set_trace_level((1 << FID_MESH_APP__CORE_AES_CCM_C), 0);
    wiced_bt_mesh_core_set_trace_level((1 << FID_MESH_APP__MESH_UTIL_C), 3);
#endif

    wiced_bt_cfg_settings.device_name = (uint8_t *)"Mesh Gateway";
    wiced_bt_cfg_settings.gatt_cfg.appearance = APPEARANCE_GENERIC_TAG;
    // Adv Data is fixed. Spec allows to put URI, Name, Appearance and Tx Power in the Scan Response Data.
    if (!is_provisioned)
    {
        wiced_bt_ble_advert_elem_t  adv_elem[3];
        uint8_t                     buf[2];
        uint8_t                     num_elem = 0;
        adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
        adv_elem[num_elem].len = (uint16_t)strlen((const char*)wiced_bt_cfg_settings.device_name);
        adv_elem[num_elem].p_data = wiced_bt_cfg_settings.device_name;
        num_elem++;

        adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_APPEARANCE;
        adv_elem[num_elem].len = 2;
        buf[0] = (uint8_t)wiced_bt_cfg_settings.gatt_cfg.appearance;
        buf[1] = (uint8_t)(wiced_bt_cfg_settings.gatt_cfg.appearance >> 8);
        adv_elem[num_elem].p_data = buf;
        num_elem++;

        wiced_bt_mesh_set_raw_scan_response_data(num_elem, adv_elem);
    }

    wiced_bt_mesh_remote_provisioning_server_init();
}


/*
 * Process the Filter Status message from a Proxy Server with the status of the proxy filter
 */
void mesh_proxy_client_process_filter_status(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len)
{
//    wiced_bt_mesh_proxy_filter_status_data_t status_data = { 0 };

    if (data_len != 3)
    {
        WICED_BT_TRACE("filter status bad len\n");
        wiced_bt_mesh_release_event(p_event);
        return;
    }

    // If we are still retransmitting cancel transmit and release mesh event
    // For proxy it is a special case because event->dst is FFFF
//    wiced_bt_mesh_models_utils_ack_received(&mesh_proxy_state.p_out_event, 0xFFFF);
//
//    BE_STREAM_TO_UINT8(status_data.type, p_data);
//    BE_STREAM_TO_UINT16(status_data.list_size, p_data);

    WICED_BT_TRACE("mesh_proxy_client_process_filter_status !!!!");

//
//    if (mesh_proxy_state.p_parent_callback)
//        (*mesh_proxy_state.p_parent_callback)(WICED_BT_MESH_PROXY_FILTER_STATUS, p_event, &status_data);
//    else
//        wiced_bt_mesh_release_event(p_event);
//    uint8_t *p_buffer = wiced_transport_allocate_buffer(host_trans_pool);
//       uint8_t *p = p_buffer;
//
//       mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_COMMAND_STATUS, p_buffer, (uint16_t)(p - p_buffer));

}

#if 0
uint32_t mesh_app_proc_rx_cmd(uint16_t cmd_opcode, uint8_t *p_data, uint32_t data_len)
{

    uint8_t res = 0;
    uint8_t index;
    WICED_BT_TRACE( "mesh_app_proc_rx_cmd cmd_opcode : %d\n", cmd_opcode );
    switch ( cmd_opcode )
    {
        case HCI_CONTROL_MESH_COMMAND_START:
            mesh_application_init();
            wiced_transport_send_data ( HCI_CONTROL_MESH_EVENT_MESH_STATUS, &res, 1);
        break;

        case HCI_CONTROL_MESH_COMMAND_SEND_PROXY_DATA:
            WICED_BT_TRACE("\n HCI_CONTROL_MESH_COMMAND_SEND_PROXY_DATA ");
            wiced_bt_mesh_core_proxy_packet(p_data, data_len);
        break;

        case HCI_CONTROL_MESH_COMMAND_PUSH_NVRAM_DATA:
            index = p_data[0];
            mesh_set_node_info(p_data+1, data_len-1,  index);
            break;

        default:
            WICED_BT_TRACE("Unknown MESH opcode0x:%04X\n", cmd_opcode);
            break;
    }
    return 1;
}
#else
uint32_t mesh_app_proc_rx_cmd(uint16_t cmd_opcode, uint8_t *p_data, uint32_t data_len)
{
    return 1;
}

#endif

#if MESH_CHIP == 20703
// Core initialization initializes the RTC calling wiced_bt_mesh_core_rtc_init() defined in app. It just should call rtc_init().
// For that chip rtc_init should be called from application. Otherwise there is build error at link phase
void wiced_bt_mesh_core_rtc_init(void)
{
    rtc_init();
}
#endif

// Temporary dummy implementation of some functions to exclude fw_upgrade_lib.a from the build
#if defined(CYW43012C0)
#include "wiced_bt_ota_firmware_upgrade.h"
wiced_bool_t wiced_ota_fw_upgrade_init(void *public_key, wiced_ota_firmware_upgrade_status_callback_t *p_status_callback, wiced_ota_firmware_upgrade_send_data_callback_t *p_send_data_callback)
{
    return WICED_FALSE;
}
wiced_bt_gatt_status_t wiced_ota_fw_upgrade_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t *p_write_data)
{
    return 0;
}
wiced_bt_gatt_status_t wiced_ota_fw_upgrade_indication_cfm_handler(uint16_t conn_id, uint16_t handle)
{
    return 0;
}
wiced_bt_gatt_status_t wiced_ota_fw_upgrade_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data)
{
    return 0;
}
void wiced_ota_fw_upgrade_connection_status_event(wiced_bt_gatt_connection_status_t *p_status)
{
}
#endif
