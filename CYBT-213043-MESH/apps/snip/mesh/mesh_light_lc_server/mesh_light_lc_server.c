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
 * This file shows how to create a device which publishes Generic Power Level Server.
 *
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

/******************************************************
 *          Constants
 ******************************************************/
#define MESH_PID                0x300D
#define MESH_VID                0x0002
#define MESH_FWID               0x300D000101010001
#define MESH_CACHE_REPLAY_SIZE  0x0008

/******************************************************
 *          Structures
 ******************************************************/
typedef struct
{
#define MESH_TRANSITION_STATE_IDLE         0
#define MESH_TRANSITION_STATE_DELAY        1
#define MESH_TRANSITION_STATE_TRANSITION   2
    uint8_t              transition_state;
    int                  lightness_present;
    int                  lightness_target;
    int                  start_level;
    uint16_t             default_level;
    uint16_t             lightness_min;
    uint16_t             lightness_max;
    uint32_t             transition_start_time;
    uint32_t             transition_remaining_time;
    uint32_t             transition_time;
    uint16_t             delay;
} mesh_light_lc_server_t;

/******************************************************
 *          Function Prototypes
 ******************************************************/
static void mesh_app_init(wiced_bool_t is_provisioned);
static uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_server_message_handler(uint8_t element_idx, uint16_t event, void *p_data);
static void mesh_light_lc_server_status_changed(uint8_t *p_data, uint32_t length);
static void mesh_light_lightness_server_status_changed(uint8_t element_idx, uint8_t *p_data, uint32_t length);
static void mesh_light_lightness_process_set_level(uint8_t element_idx, wiced_bt_mesh_light_lightness_status_data_t *p_data);
static void mesh_light_lc_server_mode_set(uint8_t element_idx, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_server_occupancy_mode_set(uint8_t element_idx, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_server_occupancy_detected(uint8_t element_idx, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_server_onoff_set(uint8_t element_idx, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_server_property_set(uint8_t element_idx, uint8_t *p_data, uint32_t length);

#ifdef HCI_CONTROL
static void mesh_light_lc_hci_event_send_lightness_set(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lightness_set_t *p_data);
#endif

/******************************************************
 *          Variables Definitions
 ******************************************************/
uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME] = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER]     = { '1', '2', '3', '4', 0, 0, 0, 0 };
uint8_t mesh_system_id[8]                                                  = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

uint32_t mesh_light_lc_ambinet_lux_level_on         = 65535;
uint32_t mesh_light_lc_ambinet_lux_level_prolong    = 1000;
uint32_t mesh_light_lc_ambinet_lux_level_standby    = 0;
uint16_t mesh_light_lc_lightness_on                 = 65535;
uint16_t mesh_light_lc_lightness_prolong            = 32767;
uint16_t mesh_light_lc_lightness_standby            = 0;
uint8_t  mesh_light_lc_regulator_accuracy           = 50;
uint8_t  mesh_light_lc_regulator_kid                = 1;
uint8_t  mesh_light_lc_regulator_kiu                = 1;
uint8_t  mesh_light_lc_regulator_kpd                = 1;
uint8_t  mesh_light_lc_regulator_kpu                = 1;
uint32_t mesh_light_lc_time_fade                    = 0; // 1000;
uint32_t mesh_light_lc_time_fade_on                 = 0; // 1000;
uint32_t mesh_light_lc_time_fade_standby_auto       = 0; // 2000;
uint32_t mesh_light_lc_time_fade_standby_manual     = 0; // 1000;
uint32_t mesh_light_lc_time_occupany_delay          = 100;
uint32_t mesh_light_lc_time_prolong                 = 0; // 2000;
uint32_t mesh_light_lc_time_run_on                  = 3000;

wiced_bt_mesh_core_config_model_t   mesh_element1_models[] =
{
    WICED_BT_MESH_DEVICE,
    WICED_BT_MESH_MODEL_LIGHT_LIGHTNESS_SERVER,
};
wiced_bt_mesh_core_config_model_t   mesh_element2_models[] =
{
    WICED_BT_MESH_MODEL_LIGHT_LC_SERVER,
};

#define _USER_ WICED_BT_MESH_PROPERTY_TYPE_USER
#define _RW_   (WICED_BT_MESH_PROPERTY_ID_READABLE | WICED_BT_MESH_PROPERTY_ID_WRITABLE)

wiced_bt_mesh_core_config_property_t mesh_element2_properties[] =
{
    { WICED_BT_MESH_PROPERTY_AMBIENT_LUX_LEVEL_ON     , _USER_, _RW_, WICED_BT_MESH_PROPERTY_LEN_AMBIENT_LUX_LEVEL_ON,      (uint8_t *)&mesh_light_lc_ambinet_lux_level_on      },
    { WICED_BT_MESH_PROPERTY_AMBIENT_LUX_LEVEL_PROLONG, _USER_, _RW_, WICED_BT_MESH_PROPERTY_LEN_AMBIENT_LUX_LEVEL_PROLONG, (uint8_t *)&mesh_light_lc_ambinet_lux_level_prolong },
    { WICED_BT_MESH_PROPERTY_AMBIENT_LUX_LEVEL_STANDBY, _USER_, _RW_, WICED_BT_MESH_PROPERTY_LEN_AMBIENT_LUX_LEVEL_STANDBY, (uint8_t *)&mesh_light_lc_ambinet_lux_level_standby },
    { WICED_BT_MESH_PROPERTY_LIGHTNESS_ON             , _USER_, _RW_, WICED_BT_MESH_PROPERTY_LEN_LIGHTNESS_ON,              (uint8_t *)&mesh_light_lc_lightness_on              },
    { WICED_BT_MESH_PROPERTY_LIGHTNESS_PROLONG        , _USER_, _RW_, WICED_BT_MESH_PROPERTY_LEN_LIGHTNESS_PROLONG,         (uint8_t *)&mesh_light_lc_lightness_prolong         },
    { WICED_BT_MESH_PROPERTY_LIGHTNESS_STANDBY        , _USER_, _RW_, WICED_BT_MESH_PROPERTY_LEN_LIGHTNESS_STANDBY,         (uint8_t *)&mesh_light_lc_lightness_standby         },
    { WICED_BT_MESH_PROPERTY_REGULATOR_ACCURACY       , _USER_, _RW_, WICED_BT_MESH_PROPERTY_LEN_REGULATOR_ACCURACY,        (uint8_t *)&mesh_light_lc_regulator_accuracy        },
    { WICED_BT_MESH_PROPERTY_REGULATOR_KID            , _USER_, _RW_, WICED_BT_MESH_PROPERTY_LEN_REGULATOR_KID,             (uint8_t *)&mesh_light_lc_regulator_kid             },
    { WICED_BT_MESH_PROPERTY_REGULATOR_KIU            , _USER_, _RW_, WICED_BT_MESH_PROPERTY_LEN_REGULATOR_KIU,             (uint8_t *)&mesh_light_lc_regulator_kiu             },
    { WICED_BT_MESH_PROPERTY_REGULATOR_KPD            , _USER_, _RW_, WICED_BT_MESH_PROPERTY_LEN_REGULATOR_KPD,             (uint8_t *)&mesh_light_lc_regulator_kpd             },
    { WICED_BT_MESH_PROPERTY_REGULATOR_KPU            , _USER_, _RW_, WICED_BT_MESH_PROPERTY_LEN_REGULATOR_KPU,             (uint8_t *)&mesh_light_lc_regulator_kpu             },
    { WICED_BT_MESH_PROPERTY_TIME_FADE                , _USER_, _RW_, WICED_BT_MESH_PROPERTY_LEN_TIME_FADE,                 (uint8_t *)&mesh_light_lc_time_fade                 },
    { WICED_BT_MESH_PROPERTY_TIME_FADE_ON             , _USER_, _RW_, WICED_BT_MESH_PROPERTY_LEN_TIME_FADE_ON,              (uint8_t *)&mesh_light_lc_time_fade_on              },
    { WICED_BT_MESH_PROPERTY_TIME_FADE_STANDBY_AUTO   , _USER_, _RW_, WICED_BT_MESH_PROPERTY_LEN_TIME_FADE_STANDBY_AUTO,    (uint8_t *)&mesh_light_lc_time_fade_standby_auto    },
    { WICED_BT_MESH_PROPERTY_TIME_FADE_STANDBY_MANUAL , _USER_, _RW_, WICED_BT_MESH_PROPERTY_LEN_TIME_FADE_STANDBY_MANUAL,  (uint8_t *)&mesh_light_lc_time_fade_standby_manual  },
    { WICED_BT_MESH_PROPERTY_TIME_OCCUPANCY_DELAY     , _USER_, _RW_, WICED_BT_MESH_PROPERTY_LEN_TIME_OCCUPANCY_DELAY,      (uint8_t *)&mesh_light_lc_time_occupany_delay       },
    { WICED_BT_MESH_PROPERTY_TIME_PROLONG             , _USER_, _RW_, WICED_BT_MESH_PROPERTY_LEN_TIME_PROLONG,              (uint8_t *)&mesh_light_lc_time_prolong              },
    { WICED_BT_MESH_PROPERTY_TIME_RUN_ON              , _USER_, _RW_, WICED_BT_MESH_PROPERTY_LEN_TIME_RUN_ON,               (uint8_t *)&mesh_light_lc_time_run_on               },
};
#define MESH_APP_NUM_PROPERTIES (sizeof(mesh_element2_properties) / sizeof(wiced_bt_mesh_core_config_property_t))

#define MESH_LIGHT_LC_SERVER_ELEMENT_INDEX      0
#define MESH_LIGHT_LC_CONTROL_ELEMENT_INDEX     1

wiced_bt_mesh_core_config_element_t mesh_elements[] =
{
    {
        .location = MESH_ELEM_LOC_MAIN,                                 // location description as defined in the GATT Bluetooth Namespace Descriptors section of the Bluetooth SIG Assigned Numbers
        .default_transition_time = MESH_DEFAULT_TRANSITION_TIME_IN_MS,  // Default transition time for models of the element in milliseconds
        .onpowerup_state = WICED_BT_MESH_ON_POWER_UP_STATE_RESTORE,     // Default element behavior on power up
        .default_level = 0xffff,                                        // Default value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_min = 1,                                                 // Minimum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_max = 0xffff,                                            // Maximum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .move_rollover = 0,                                             // If true when level gets to range_max during move operation, it switches to min, otherwise move stops.
        .properties_num = 0,                                            // Number of properties in the array models
        .properties = NULL,                                             // Array of properties in the element.
        .sensors_num = 0,                                               // Number of sensors in the sensor array
        .sensors = NULL,                                                // Array of sensors of that element
        .models_num = sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t),  // Number of models in the array models
        .models = mesh_element1_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
    {
        .location = MESH_ELEM_LOC_MAIN,                                 // location description as defined in the GATT Bluetooth Namespace Descriptors section of the Bluetooth SIG Assigned Numbers
        .default_transition_time = MESH_DEFAULT_TRANSITION_TIME_IN_MS,  // Default transition time for models of the element in milliseconds
        .onpowerup_state = WICED_BT_MESH_ON_POWER_UP_STATE_RESTORE,     // Default element behavior on power up
        .default_level = 1,                                             // Default value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_min = 1,                                                 // Minimum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_max = 0xffff,                                            // Maximum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .move_rollover = 0,                                             // If true when level gets to range_max during move operation, it switches to min, otherwise move stops.
        .properties_num = MESH_APP_NUM_PROPERTIES,                      // Number of properties in the array models
        .properties = mesh_element2_properties,                         // Array of properties in the element.
        .sensors_num = 0,                                               // Number of sensors in the sensor array
        .sensors = NULL,                                                // Array of sensors of that element
        .models_num = sizeof(mesh_element2_models) / sizeof(wiced_bt_mesh_core_config_model_t),                              // Number of models in the array models
        .models = mesh_element2_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
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
        .poll_timeout          = 36000                              // Poll timeout in 100ms units to be requested by the Low Power node.
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
mesh_light_lc_server_t app_state;

/******************************************************
 *               Function Definitions
 ******************************************************/
void mesh_app_init(wiced_bool_t is_provisioned)
{
#if 0
    extern uint8_t wiced_bt_mesh_model_trace_enabled;
    wiced_bt_mesh_model_trace_enabled = WICED_TRUE;
#endif

    wiced_bt_cfg_settings.device_name = (uint8_t *)"Smart Light";
    wiced_bt_cfg_settings.gatt_cfg.appearance = APPEARANCE_LIGHT_CEILING;
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
    wiced_bt_mesh_model_light_lc_server_init(MESH_LIGHT_LC_SERVER_ELEMENT_INDEX, mesh_light_lc_server_message_handler, is_provisioned);
}

/*
 * Process event received from the models library.
 */
void mesh_light_lc_server_message_handler(uint8_t element_idx, uint16_t event, void *p_data)
{
#if defined HCI_CONTROL
    wiced_bt_mesh_hci_event_t *p_hci_event;
#endif
    WICED_BT_TRACE("light lc srv msg:%d\n", event);

    switch (event)
    {
    case WICED_BT_MESH_LIGHT_LIGHTNESS_SET:
#if defined HCI_CONTROL
//        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
//            mesh_light_lc_hci_event_send_lightness_set(p_hci_event, (wiced_bt_mesh_light_lightness_set_t *)p_data);
#endif
        mesh_light_lightness_process_set_level(element_idx, (wiced_bt_mesh_light_lightness_status_data_t *)p_data);
        break;

    default:
        WICED_BT_TRACE("unknown\n");
        break;
    }
}

/*
 * In 2 chip solutions MCU can send commands to indicate that Level has changed.
 */
uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
#ifdef HCI_CONTROL
    uint8_t element_idx;
    WICED_BT_TRACE("[%s] cmd_opcode 0x%04x\n", __FUNCTION__, opcode);

    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LIGHTNESS_SET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_MODE_SET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_OCCUPANCY_MODE_SET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_OCCUPANCY_SET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_ONOFF_SET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_PROPERTY_SET:
        break;

    default:
        WICED_BT_TRACE("unknown\n");
        return WICED_FALSE;
    }
    element_idx = wiced_bt_mesh_get_element_idx_from_wiced_hci(&p_data, &length);
    if (element_idx != MESH_LIGHT_LC_SERVER_ELEMENT_INDEX)
    {
        WICED_BT_TRACE("[%s] cmd_opcode 0x%04x invalid element_idx:%d\n", __FUNCTION__, opcode, element_idx);
        return WICED_TRUE;
    }
    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LIGHTNESS_SET:
        mesh_light_lightness_server_status_changed(MESH_LIGHT_LC_SERVER_ELEMENT_INDEX, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_MODE_SET:
        mesh_light_lc_server_mode_set(MESH_LIGHT_LC_CONTROL_ELEMENT_INDEX, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_OCCUPANCY_MODE_SET:
        mesh_light_lc_server_occupancy_mode_set(MESH_LIGHT_LC_CONTROL_ELEMENT_INDEX, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_OCCUPANCY_SET:
        mesh_light_lc_server_occupancy_detected(MESH_LIGHT_LC_CONTROL_ELEMENT_INDEX, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_ONOFF_SET:
        mesh_light_lc_server_onoff_set(MESH_LIGHT_LC_CONTROL_ELEMENT_INDEX, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_PROPERTY_SET:
        mesh_light_lc_server_property_set(MESH_LIGHT_LC_CONTROL_ELEMENT_INDEX, p_data, length);
        break;

    default:
        WICED_BT_TRACE("unknown\n");
        break;
    }
#endif
    return WICED_TRUE;
}

/*
 * Send Light Lightness Status event
 */
void mesh_light_lightness_server_status_changed(uint8_t element_idx, uint8_t *p_data, uint32_t length)
{
//    uint16_t lightness = p_data[0] + (p_data[1] << 8);
//    wiced_bt_mesh_model_lc_lightness_changed(element_idx, lightness);
}

/*
 * Command from the level client is received to set the new level
 */
void mesh_light_lightness_process_set_level(uint8_t element_idx, wiced_bt_mesh_light_lightness_status_data_t *p_status)
{
    WICED_BT_TRACE("mesh light srv set level lightness:%d remaining_time:%d\n",
            p_status->present, p_status->remaining_time);
}

/*
 * Light LC Mode changed notification
 */
void mesh_light_lc_server_mode_set(uint8_t element_idx, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_light_lc_mode_set_data_t set_data;

    STREAM_TO_UINT8(set_data.mode, p_data);

    wiced_bt_mesh_model_light_lc_mode_changed(element_idx, &set_data);
}

/*
 * Light LC Occupancy Mode changed notification
 */
void mesh_light_lc_server_occupancy_mode_set(uint8_t element_idx, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_light_lc_occupancy_mode_set_data_t set_data;

    STREAM_TO_UINT8(set_data.mode, p_data);

    wiced_bt_mesh_model_light_lc_occupancy_mode_changed(element_idx, &set_data);
}

/*
 * Light LC Occupancy Detected notification
 */
void mesh_light_lc_server_occupancy_detected(uint8_t element_idx, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_light_lc_occupancy_detected(element_idx);
}

/*
 * Light LC OnOff changed notification
 */
void mesh_light_lc_server_onoff_set(uint8_t element_idx, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_onoff_set_data_t set_data = { 0 };

    STREAM_TO_UINT8(set_data.onoff, p_data);

    wiced_bt_mesh_model_light_lc_onoff_changed(element_idx, &set_data);
}

/*
 * Light LC Property changed notification
 */
void mesh_light_lc_server_property_set(uint8_t element_idx, uint8_t *p_data, uint32_t length)
{
    uint16_t property_id;
    uint8_t  property_value[4];
    int i;

    STREAM_TO_UINT16(property_id, p_data);
    length -= 2;
    memcpy(property_value, p_data, length);

    // If value of the User Property ID field that does not identify any existing User Property, Send Reply without bothering the app.
    for (i = 0; i < MESH_APP_NUM_PROPERTIES; i++)
    {
        if ((mesh_element2_properties[i].id == property_id) &&
            (mesh_element2_properties[i].max_len == length))
        {
            switch (length)
            {
            case 1:
                mesh_element2_properties[i].value[0] = property_value[0];
                break;
            case 2:
                mesh_element2_properties[i].value[0] = property_value[1];
                mesh_element2_properties[i].value[1] = property_value[0];
                break;
            case 3:
                mesh_element2_properties[i].value[0] = property_value[2];
                mesh_element2_properties[i].value[1] = property_value[1];
                mesh_element2_properties[i].value[2] = property_value[0];
                break;
            case 4:
                mesh_element2_properties[i].value[0] = property_value[3];
                mesh_element2_properties[i].value[1] = property_value[2];
                mesh_element2_properties[i].value[2] = property_value[1];
                mesh_element2_properties[i].value[3] = property_value[0];
                break;
            }
            // ToDo either need to save to NVRAM here, or tell library that mesh_config has changed
            return;
        }
    }
    WICED_BT_TRACE("not found\n");
}

#ifdef HCI_CONTROL
/*
 * Send Light LC Set lightness event over transport
 */
void mesh_light_lc_hci_event_send_lightness_set(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lightness_set_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, p_data->lightness_actual);
    UINT32_TO_STREAM(p, p_data->transition_time);
    UINT16_TO_STREAM(p, p_data->delay);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LIGHT_LIGHTNESS_SET, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

#endif
