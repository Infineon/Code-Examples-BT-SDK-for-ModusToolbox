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
 * This file shows how to create a device which implements onoff server.
 */
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_mesh_app.h"

#ifdef HCI_CONTROL
#include "wiced_transport.h"
#include "hci_control_api.h"
#endif

#include "wiced_bt_cfg.h"
extern wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

#define NUM_ONOFF_SERVERS 1

// Needed to pass some PTS tests which require vendor model
//#define MESH_VENDOR_TST_COMPANY_ID  0x131
//#define MESH_VENDOR_TST_MODEL_ID    1

// Uncomment following line to enable Mesh DFU
// #define MESH_DFU
/******************************************************
 *          Constants
 ******************************************************/
#define MESH_PID                0x3016
#define MESH_VID                0x0001
#define MESH_FWID               0x3016000101010001
#define MESH_CACHE_REPLAY_SIZE  0x0008

/******************************************************
 *          Structures
 ******************************************************/
typedef struct
{
    uint8_t  present_state;
    uint8_t  target_state;
} mesh_onoff_server_t;

/******************************************************
 *          Function Prototypes
 ******************************************************/
static void mesh_app_init(wiced_bool_t is_provisioned);
static uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
static void mesh_onoff_server_message_handler(uint8_t element_idx, uint16_t event, void *p_data);
static void mesh_onoff_server_send_state_change(uint8_t element_idx, uint8_t onoff);
static void mesh_onoff_server_process_set(uint8_t element_idx, wiced_bt_mesh_onoff_status_data_t *p_data);

#ifdef HCI_CONTROL
static void mesh_onoff_hci_event_send_set(uint8_t element_idx, wiced_bt_mesh_onoff_status_data_t *p_data);
#endif

/******************************************************
 *          Variables Definitions
 ******************************************************/
uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME] = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER]     = { '1', '2', '3', '4', 0, 0, 0, 0 };
uint8_t mesh_system_id[8]                                                  = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

wiced_bt_mesh_core_config_model_t   mesh_element1_models[] =
{
    WICED_BT_MESH_DEVICE,
#ifdef MESH_DFU
    WICED_BT_MESH_MODEL_FW_DISTRIBUTION_SERVER,
    WICED_BT_MESH_MODEL_FW_UPDATE_SERVER,
#endif
    WICED_BT_MESH_MODEL_ONOFF_SERVER,
#ifdef MESH_VENDOR_TST_MODEL_ID
    { MESH_VENDOR_TST_COMPANY_ID, MESH_VENDOR_TST_MODEL_ID, NULL, NULL, NULL },
#endif
};
#if NUM_ONOFF_SERVERS > 1
wiced_bt_mesh_core_config_model_t   mesh_element2_models[] =
{
    WICED_BT_MESH_MODEL_ONOFF_SERVER,
};
#endif
#if NUM_ONOFF_SERVERS > 2
wiced_bt_mesh_core_config_model_t   mesh_element3_models[] =
{
    WICED_BT_MESH_MODEL_ONOFF_SERVER,
};
#endif
#if NUM_ONOFF_SERVERS > 3
wiced_bt_mesh_core_config_model_t   mesh_element4_models[] =
{
    WICED_BT_MESH_MODEL_ONOFF_SERVER,
};
#endif

#define MESH_ONOFF_SERVER_ELEMENT_INDEX   0

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
        .models_num = (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t)),    // Number of models in the array models
        .models = mesh_element1_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
#if NUM_ONOFF_SERVERS > 1
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
        .models_num = (sizeof(mesh_element2_models) / sizeof(wiced_bt_mesh_core_config_model_t)),                              // Number of models in the array models
        .models = mesh_element2_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
#endif
#if NUM_ONOFF_SERVERS > 2
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
        .models_num = (sizeof(mesh_element3_models) / sizeof(wiced_bt_mesh_core_config_model_t)),                              // Number of models in the array models
        .models = mesh_element3_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
#endif
#if NUM_ONOFF_SERVERS > 3
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
        .models_num = (sizeof(mesh_element4_models) / sizeof(wiced_bt_mesh_core_config_model_t)),                              // Number of models in the array models
        .models = mesh_element4_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
#endif
};

wiced_bt_mesh_core_config_t  mesh_config =
{
    .company_id         = MESH_COMPANY_ID_CYPRESS,                  // Company identifier assigned by the Bluetooth SIG
    .product_id         = MESH_PID,                                 // Vendor-assigned product identifier
    .vendor_id          = MESH_VID,                                 // Vendor-assigned product version identifier
    .firmware_id        = MESH_FWID,                                // Vendor-assigned firmware version identifier
    .replay_cache_size  = MESH_CACHE_REPLAY_SIZE,                   // Number of replay protection entries, i.e. maximum number of mesh devices that can send application messages to this device.
#if defined(LOW_POWER_NODE) && (LOW_POWER_NODE == 1)
    .features           = WICED_BT_MESH_CORE_FEATURE_BIT_LOW_POWER, // A bit field indicating the device features. In Low Power mode no Relay, no Proxy and no Friend
    .friend_cfg         =                                           // Empty Configuration of the Friend Feature
    {
        .receive_window = 0,                                        // Receive Window value in milliseconds supported by the Friend node.
        .cache_buf_len  = 0,                                        // Length of the buffer for the cache
        .max_lpn_num    = 0                                         // Max number of Low Power Nodes with established friendship. Must be > 0 if Friend feature is supported.
    },
    .low_power          =                                           // Configuration of the Low Power Feature
    {
        .rssi_factor           = 2,                                 // contribution of the RSSI measured by the Friend node used in Friend Offer Delay calculations.
        .receive_window_factor = 2,                                 // contribution of the supported Receive Window used in Friend Offer Delay calculations.
        .min_cache_size_log    = 3,                                 // minimum number of messages that the Friend node can store in its Friend Cache.
        .receive_delay         = 100,                               // Receive delay in 1 ms units to be requested by the Low Power node.
        .poll_timeout          = 200                                // Poll timeout in 100ms units to be requested by the Low Power node.
    },
#else
    .features = WICED_BT_MESH_CORE_FEATURE_BIT_FRIEND | WICED_BT_MESH_CORE_FEATURE_BIT_RELAY | WICED_BT_MESH_CORE_FEATURE_BIT_GATT_PROXY_SERVER,   // Supports Friend, Relay and GATT Proxy
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
        .poll_timeout          = 0                                  // Poll timeout in 100ms units to be requested by the Low Power node.
    },
#endif
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

// Application state
mesh_onoff_server_t app_state;

/******************************************************
 *               Function Definitions
 ******************************************************/
void mesh_app_init(wiced_bool_t is_provisioned)
{
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

    wiced_bt_cfg_settings.device_name = (uint8_t *)"OnOff Server";
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

    memset (&app_state, 0, sizeof(app_state));

#if REMOTE_PROVISIONG_SERVER_SUPPORTED
    wiced_bt_mesh_remote_provisioning_server_init();
#endif

    wiced_bt_mesh_model_onoff_server_init(MESH_ONOFF_SERVER_ELEMENT_INDEX, mesh_onoff_server_message_handler, is_provisioned);

    wiced_bt_mesh_model_fw_update_server_init(MESH_ONOFF_SERVER_ELEMENT_INDEX, NULL, is_provisioned);
    wiced_bt_mesh_model_fw_distribution_server_init();

#if NUM_ONOFF_SERVERS > 1
    wiced_bt_mesh_model_onoff_server_init(1, mesh_onoff_server_message_handler, is_provisioned);
#endif
#if NUM_ONOFF_SERVERS > 2
    wiced_bt_mesh_model_onoff_server_init(2, mesh_onoff_server_message_handler, is_provisioned);
#endif
#if NUM_ONOFF_SERVERS > 3
    wiced_bt_mesh_model_onoff_server_init(3, mesh_onoff_server_message_handler, is_provisioned);
#endif
}

/*
 * Process event received from the OnOff Client.
 */
void mesh_onoff_server_message_handler(uint8_t element_idx, uint16_t event, void *p_data)
{
    switch (event)
    {
    case WICED_BT_MESH_ONOFF_SET:
        mesh_onoff_server_process_set(element_idx, (wiced_bt_mesh_onoff_status_data_t *)p_data);
        break;

    default:
        WICED_BT_TRACE("unknown\n");
    }
}

/*
 * In 2 chip solutions MCU can send a command that On/Off state has changed.
 */
uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
    uint8_t element_idx;

    WICED_BT_TRACE("onoff rx cmd_opcode 0x%02x\n", opcode);

    switch (opcode)
    {
#ifdef HCI_CONTROL
    case HCI_CONTROL_MESH_COMMAND_ONOFF_SET:
        element_idx = wiced_bt_mesh_get_element_idx_from_wiced_hci(&p_data, &length);
        mesh_onoff_server_send_state_change(element_idx, *p_data);
        break;
#endif

    default:
        WICED_BT_TRACE("unknown\n");
        return WICED_FALSE;
    }
    return WICED_TRUE;
}

/*
 * This function is called when command to change state is received over mesh.
 */
void mesh_onoff_server_process_set(uint8_t element_idx, wiced_bt_mesh_onoff_status_data_t *p_status)
{
    WICED_BT_TRACE("onoff srv set onoff: present:%d target:%d remaining:%d\n", p_status->present_onoff, p_status->target_onoff, p_status->remaining_time);
#if defined HCI_CONTROL
    mesh_onoff_hci_event_send_set(element_idx, p_status);
#endif
}

/*
 * This function shall be called when On/Off state has been changed locally
 */
void mesh_onoff_server_send_state_change(uint8_t element_idx, uint8_t onoff)
{
    wiced_bt_mesh_model_onoff_changed(element_idx, onoff);
}

#ifdef HCI_CONTROL
/*
 * Send OnOff Set event over transport
 */
void mesh_onoff_hci_event_send_set(uint8_t element_idx, wiced_bt_mesh_onoff_status_data_t *p_data)
{
    wiced_bt_mesh_hci_event_t *p_hci_event = wiced_bt_mesh_alloc_hci_event(element_idx);
    if (p_hci_event)
    {
        uint8_t *p = p_hci_event->data;

        UINT8_TO_STREAM(p, p_data->present_onoff);
        UINT8_TO_STREAM(p, p_data->target_onoff);
        UINT32_TO_STREAM(p, p_data->remaining_time);

        mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_ONOFF_SET, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
    }
}

#endif
