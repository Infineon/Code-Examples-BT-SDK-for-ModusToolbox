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
 * This file shows how to create a device which implements mesh user sensor client.
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
#define MESH_PID                0x3021
#define MESH_VID                0x0002
#define MESH_FWID               0x3021000101010001
#define MESH_CACHE_REPLAY_SIZE  0x0008

#define SETTING_PROPERTY_ID                         (0x2001)

/******************************************************
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
static void mesh_app_init(wiced_bool_t is_provisioned);
static uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
static void mesh_sensor_client_message_handler(uint8_t element_idx, uint16_t addr, uint16_t event, void *p_data);
static void mesh_sensor_descriptor_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_sensor_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_sensor_column_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_sensor_cadence_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_sensor_series_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_sensor_cadence_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_sensor_setting_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_sensor_setting_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_sensor_settings_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_sensor_series_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_series_status_data_t *p_data);
static void mesh_sensor_desc_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_descriptor_status_data_t *p_data);
static void mesh_sensor_data_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_status_data_t *p_data);
static void mesh_sensor_column_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_column_status_data_t *p_data);
static void mesh_sensor_cadence_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_cadence_status_data_t *cadence_status );
static void mesh_sensor_setting_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_setting_status_data_t *p_data);
static void mesh_sensor_settings_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_settings_status_data_t *p_data);

/******************************************************
 *          Variables Definitions
 ******************************************************/
uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME] = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER]     = { '1', '2', '3', '4', 0, 0, 0, 0 };
uint8_t mesh_system_id[8]                                                  = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

wiced_bt_mesh_core_config_model_t   mesh_element1_models[] =
{
        WICED_BT_MESH_DEVICE,
        WICED_BT_MESH_MODEL_SENSOR_CLIENT,
};
#define MESH_APP_NUM_MODELS  (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t))

#define MESH_SENSOR_CLIENT_ELEMENT_INDEX   0

wiced_bt_mesh_core_config_property_t mesh_element1_properties[] =
{
    { WICED_BT_MESH_PROPERTY_MOTION_SENSED,               0, 0, 1, NULL},
    { WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_LIGHT_LEVEL, 0, 0, 3, NULL},
    { WICED_BT_MESH_PROPERTY_TOTAL_LIGHT_EXPOSURE_TIME,   0, 0, 3, NULL},
    { SETTING_PROPERTY_ID,                                0, 0, 2, NULL},
};

wiced_bt_mesh_core_config_element_t mesh_elements[] =
{
    {
        .location = MESH_ELEM_LOC_MAIN,                                 // location description as defined in the GATT Bluetooth Namespace Descriptors section of the Bluetooth SIG Assigned Numbers
        .default_transition_time = MESH_DEFAULT_TRANSITION_TIME_IN_MS,  // Default transition time for models of the element in milliseconds
        .onpowerup_state = WICED_BT_MESH_ON_POWER_UP_STATE_RESTORE,     // Default element behavior on power up
        .properties_num = 4,                                            // Number of properties in the array models
        .properties = mesh_element1_properties,                         // Array of properties in the element.
        .sensors_num = 0,                                               // Number of sensors in the sensor array
        .sensors = NULL,                                                // Array of sensors of that element
        .models_num = MESH_APP_NUM_MODELS,                              // Number of models in the array models
        .models = mesh_element1_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
};

uint8_t mesh_num_elements = 1;

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

/******************************************************
 *               Function Definitions
 ******************************************************/
void mesh_app_init(wiced_bool_t is_provisioned)
{
#if 1
    extern uint8_t wiced_bt_mesh_model_trace_enabled;
    wiced_bt_mesh_model_trace_enabled = WICED_TRUE;
#endif

    wiced_bt_cfg_settings.device_name = (uint8_t *)"Sensor Client";
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

    // register with the library to receive parsed data
    wiced_bt_mesh_model_sensor_client_init(MESH_SENSOR_CLIENT_ELEMENT_INDEX, mesh_sensor_client_message_handler, is_provisioned);
}

/*
 * Process event received from the sensor Server.
 */
void mesh_sensor_client_message_handler(uint8_t element_idx, uint16_t addr, uint16_t event, void *p_data)
{
#if defined HCI_CONTROL
    wiced_bt_mesh_hci_event_t *p_hci_event;
#endif
    WICED_BT_TRACE("sensor clt msg:%d\n", event);

    switch (event)
    {
    case WICED_BT_MESH_TX_COMPLETE:
        WICED_BT_TRACE("tx complete\n");
        break;

#if defined HCI_CONTROL
    case WICED_BT_MESH_SENSOR_DESCRIPTOR_STATUS:
        if ((p_hci_event = wiced_bt_mesh_alloc_hci_event(element_idx)) != NULL)
        {
            p_hci_event->src = addr;
            mesh_sensor_desc_hci_event_send(p_hci_event, (wiced_bt_mesh_sensor_descriptor_status_data_t *)p_data);
        }
        break;

    case WICED_BT_MESH_SENSOR_STATUS:
        if ((p_hci_event = wiced_bt_mesh_alloc_hci_event(element_idx)) != NULL)
        {
            p_hci_event->src = addr;
            mesh_sensor_data_hci_event_send(p_hci_event, (wiced_bt_mesh_sensor_status_data_t *)p_data);
        }
        break;

    case WICED_BT_MESH_SENSOR_COLUMN_STATUS:
        if ((p_hci_event = wiced_bt_mesh_alloc_hci_event(element_idx)) != NULL)
        {
            p_hci_event->src = addr;
            mesh_sensor_column_hci_event_send(p_hci_event, (wiced_bt_mesh_sensor_column_status_data_t *)p_data);
        }
        break;

    case WICED_BT_MESH_SENSOR_SERIES_STATUS:
        if ((p_hci_event = wiced_bt_mesh_alloc_hci_event(element_idx)) != NULL)
        {
            p_hci_event->src = addr;
            mesh_sensor_series_hci_event_send(p_hci_event, (wiced_bt_mesh_sensor_series_status_data_t *)p_data);
        }
        break;

    case WICED_BT_MESH_SENSOR_CADENCE_STATUS:
        if ((p_hci_event = wiced_bt_mesh_alloc_hci_event(element_idx)) != NULL)
        {
            p_hci_event->src = addr;
            mesh_sensor_cadence_hci_event_send(p_hci_event, (wiced_bt_mesh_sensor_cadence_status_data_t *)p_data);
        }
        break;

    case WICED_BT_MESH_SENSOR_SETTINGS_STATUS:
        if ((p_hci_event = wiced_bt_mesh_alloc_hci_event(element_idx)) != NULL)
        {
            p_hci_event->src = addr;
            mesh_sensor_settings_hci_event_send(p_hci_event, (wiced_bt_mesh_sensor_settings_status_data_t *)p_data);
        }
        break;

    case WICED_BT_MESH_SENSOR_SETTING_STATUS:
        if ((p_hci_event = wiced_bt_mesh_alloc_hci_event(element_idx)) != NULL)
        {
            p_hci_event->src = addr;
            mesh_sensor_setting_hci_event_send(p_hci_event, (wiced_bt_mesh_sensor_setting_status_data_t *)p_data);
        }
        break;
#endif
    default:
        WICED_BT_TRACE("not processed\n");
        break;
    }
}


/*
 * In 2 chip solutions MCU can send commands to change sensor state.
 */
uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
#ifdef HCI_CONTROL
    wiced_bt_mesh_event_t *p_event;

    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_SENSOR_DESCRIPTOR_GET:
    case HCI_CONTROL_MESH_COMMAND_SENSOR_GET:
    case HCI_CONTROL_MESH_COMMAND_SENSOR_COLUMN_GET:
    case HCI_CONTROL_MESH_COMMAND_SENSOR_SERIES_GET:
    case HCI_CONTROL_MESH_COMMAND_SENSOR_CADENCE_GET:
    case HCI_CONTROL_MESH_COMMAND_SENSOR_CADENCE_SET:
    case HCI_CONTROL_MESH_COMMAND_SENSOR_SETTING_GET:
    case HCI_CONTROL_MESH_COMMAND_SENSOR_SETTING_SET:
    case HCI_CONTROL_MESH_COMMAND_SENSOR_SETTINGS_GET:
        break;

    default:
        return WICED_FALSE;
    }
    p_event = wiced_bt_mesh_create_event_from_wiced_hci(opcode, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SENSOR_CLNT, &p_data, &length);
    if (p_event == NULL)
    {
        WICED_BT_TRACE("bad hdr\n");
        return WICED_TRUE;
    }
    switch (opcode)
    {
    //sensor client messages
    case HCI_CONTROL_MESH_COMMAND_SENSOR_DESCRIPTOR_GET:
        mesh_sensor_descriptor_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_SENSOR_GET:
        mesh_sensor_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_SENSOR_COLUMN_GET:
        mesh_sensor_column_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_SENSOR_SERIES_GET:
        mesh_sensor_series_get(p_event, p_data, length);
        break;

    //sensor setup server messages
    case HCI_CONTROL_MESH_COMMAND_SENSOR_CADENCE_GET:
        mesh_sensor_cadence_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_SENSOR_CADENCE_SET:
        mesh_sensor_cadence_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_SENSOR_SETTING_GET:
        mesh_sensor_setting_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_SENSOR_SETTING_SET:
        mesh_sensor_setting_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_SENSOR_SETTINGS_GET:
        mesh_sensor_settings_get(p_event, p_data, length);
        break;
    }
#endif

    return WICED_TRUE;
}

/*
 * Send sensor descriptor get command
 */
void mesh_sensor_descriptor_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_sensor_get_t get_data;

    WICED_BT_TRACE("mesh_sensor_descriptor_get\n");

    if (length == 2)
    {
        STREAM_TO_UINT16(get_data.property_id, p_data);
    }
    else
    {
        get_data.property_id = 0;
    }
    wiced_bt_mesh_model_sensor_client_descriptor_send_get(p_event, &get_data);
}

/*
 * Send sensor get command
 */
void mesh_sensor_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_sensor_get_t get_data;

    WICED_BT_TRACE("mesh_sensor_get\n");

    if (length == 2)
    {
        STREAM_TO_UINT16(get_data.property_id, p_data);
    }
    else
    {
        get_data.property_id = 0;
    }
    wiced_bt_mesh_model_sensor_client_sensor_send_get(p_event, &get_data);
}

/*
 * Send column get command
 */
void mesh_sensor_column_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_sensor_column_get_data_t get_data;
    int i;

    STREAM_TO_UINT16(get_data.property_id, p_data);
    STREAM_TO_UINT8(get_data.prop_value_len, p_data);
    STREAM_TO_ARRAY(get_data.raw_valuex, p_data, get_data.prop_value_len);

    WICED_BT_TRACE("sensor column get\n");
    for(i = 0; i < get_data.prop_value_len; i++)
        WICED_BT_TRACE(" %02x", get_data.raw_valuex[i]);
    WICED_BT_TRACE("\n");

    wiced_bt_mesh_model_sensor_client_sensor_column_send_get(p_event, &get_data);
}

/*
 * Send series get command
 */
void mesh_sensor_series_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_sensor_series_get_data_t get_data;
    uint8_t header;
	int i;

    memset(&get_data, 0, sizeof(wiced_bt_mesh_sensor_series_get_data_t));

    header = 2 * sizeof(uint16_t);

    WICED_BT_TRACE("mesh_sensor_series_get\n");
    if ((length - header) == 2)
    {
        WICED_BT_TRACE("\n property id only\n");
        STREAM_TO_UINT16(get_data.property_id, p_data);
        get_data.start_index = 0x00;
        get_data.end_index = 0xFF;
    }
    else
    {
        WICED_BT_TRACE("\n property id \n");
        STREAM_TO_UINT16(get_data.property_id, p_data);
        STREAM_TO_UINT8(get_data.prop_value_len, p_data);
        STREAM_TO_ARRAY(get_data.raw_valuex1, p_data, get_data.prop_value_len);
        STREAM_TO_ARRAY(get_data.raw_valuex2, p_data, get_data.prop_value_len);
    }
    for(i = 0; i < get_data.prop_value_len; i++)
        WICED_BT_TRACE(" %02x",get_data.raw_valuex1[i]);
    WICED_BT_TRACE("\n");

    for(i = 0; i < get_data.prop_value_len; i++)
        WICED_BT_TRACE(" %02x",get_data.raw_valuex2[i]);
    WICED_BT_TRACE("\n");

    wiced_bt_mesh_model_sensor_client_sensor_series_send_get(p_event, &get_data);
}

/*
 * Send cadence get command
 */
void mesh_sensor_cadence_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_sensor_get_t get_data;

    if (length == 2)
    {
        STREAM_TO_UINT16(get_data.property_id, p_data);
    }
    else
    {
        get_data.property_id = 0;
    }
    wiced_bt_mesh_model_sensor_client_sensor_cadence_send_get(p_event, &get_data);
}

/*
 * Send cadence set command
 */

void mesh_sensor_cadence_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t data_len)
{
    wiced_bt_mesh_sensor_cadence_set_data_t cadence_set;

    WICED_BT_TRACE("sensor cadence set\n");

    STREAM_TO_UINT16(cadence_set.property_id, p_data);
    STREAM_TO_UINT8(cadence_set.prop_value_len, p_data);
    STREAM_TO_UINT16(cadence_set.cadence_data.fast_cadence_period_divisor, p_data);
    STREAM_TO_UINT8(cadence_set.cadence_data.trigger_type, p_data);
    STREAM_TO_UINT32(cadence_set.cadence_data.trigger_delta_down, p_data);
    STREAM_TO_UINT32(cadence_set.cadence_data.trigger_delta_up, p_data);
    STREAM_TO_UINT32(cadence_set.cadence_data.min_interval, p_data);
    STREAM_TO_UINT32(cadence_set.cadence_data.fast_cadence_low, p_data);
    STREAM_TO_UINT32(cadence_set.cadence_data.fast_cadence_high, p_data);

    wiced_bt_mesh_model_sensor_client_sensor_cadence_send_set(p_event, &cadence_set);
}

/*
 * Send setting get command
 */
void mesh_sensor_setting_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_sensor_setting_get_data_t get_data;

    WICED_BT_TRACE("sensor setting get\n");

    STREAM_TO_UINT16(get_data.property_id, p_data);
    STREAM_TO_UINT16(get_data.setting_property_id, p_data);

    wiced_bt_mesh_model_sensor_client_sensor_setting_send_get(p_event, &get_data);
}

/*
 * Send setting set command
 */
void mesh_sensor_setting_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_sensor_setting_set_data_t set_data;

    WICED_BT_TRACE("sensor setting set\n");

    memset(&set_data, 0, sizeof(set_data));

    STREAM_TO_UINT16(set_data.property_id, p_data);
    STREAM_TO_UINT16(set_data.setting_property_id, p_data);
    STREAM_TO_UINT8(set_data.prop_value_len, p_data);
    STREAM_TO_ARRAY(set_data.setting_raw_val, p_data, set_data.prop_value_len);

    wiced_bt_mesh_model_sensor_client_sensor_setting_send_set(p_event, &set_data);
}

/*
 * Send settings get command
 */
void mesh_sensor_settings_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_sensor_get_t get_data;

    STREAM_TO_UINT16(get_data.property_id, p_data);

    WICED_BT_TRACE("sensor settings get prop:%04x\n", get_data.property_id);

    wiced_bt_mesh_model_sensor_client_sensor_settings_send_get(p_event, &get_data);
}

#ifdef HCI_CONTROL
/*
 * Send Descriptor Status event over transport
 */
void mesh_sensor_desc_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_descriptor_status_data_t *p_data)
{
    int i;
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("mesh_sensor_desc_hci_event_send: num descriptors %x\n", p_data->num_descriptors);
    if (p_data->num_descriptors != 0)
    {
        for (i = 0; i < p_data->num_descriptors; i++)
        {
            WICED_BT_TRACE("property_id :  %x\n", p_data->descriptor_list[i].property_id);
            WICED_BT_TRACE("positive_tolerance :  %x\n", p_data->descriptor_list[i].positive_tolerance);
            WICED_BT_TRACE("negative_tolerance :  %x\n", p_data->descriptor_list[i].negative_tolerance);
            WICED_BT_TRACE("sampling_function :  %x\n", p_data->descriptor_list[i].sampling_function);
            WICED_BT_TRACE("measurement_period :  %x\n", p_data->descriptor_list[i].measurement_period);
            WICED_BT_TRACE("update_interval :  %x\n", p_data->descriptor_list[i].update_interval);
            UINT16_TO_STREAM(p, p_data->descriptor_list[i].property_id);
            UINT16_TO_STREAM(p, p_data->descriptor_list[i].positive_tolerance);
            UINT16_TO_STREAM(p, p_data->descriptor_list[i].negative_tolerance);
            UINT8_TO_STREAM(p, p_data->descriptor_list[i].sampling_function);
            UINT8_TO_STREAM(p, p_data->descriptor_list[i].measurement_period);
            UINT8_TO_STREAM(p, p_data->descriptor_list[i].update_interval);
        }
    }
    else
    {
        WICED_BT_TRACE("mesh_sensor_desc_get : no descriptor present for property ID\n");
    }

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_SENSOR_DESCRIPTOR_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send sensor Status event over transport
 */
void mesh_sensor_data_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;
    int i, j;

    WICED_BT_TRACE("property_id:%04x\n", p_data->property_id);
    WICED_BT_TRACE("prop_value_len:%d\n", p_data->prop_value_len);
    WICED_BT_TRACE("Raw val");

    for (j = 0; j < p_data->prop_value_len; j++)
        WICED_BT_TRACE(" %02x", p_data->raw_value[j]);
    WICED_BT_TRACE("\n");

    UINT16_TO_STREAM(p, p_data->property_id);
    UINT8_TO_STREAM(p, p_data->prop_value_len);
    memcpy(p, p_data->raw_value, p_data->prop_value_len);
    p = p + p_data->prop_value_len;

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_SENSOR_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send sensor column status event over transport
 */
void mesh_sensor_column_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_column_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;
    int i;

    UINT16_TO_STREAM(p, p_data->property_id);
    UINT8_TO_STREAM(p, p_data->prop_value_len);

    ARRAY_TO_STREAM(p, p_data->column_data.raw_valuex, p_data->prop_value_len);
    ARRAY_TO_STREAM(p, p_data->column_data.column_width, p_data->prop_value_len);
    ARRAY_TO_STREAM(p, p_data->column_data.raw_valuey, p_data->prop_value_len);

    WICED_BT_TRACE(" property_id:%x\n",p_data->property_id);
    WICED_BT_TRACE(" prop_value_len:%x\n",p_data->prop_value_len);
    WICED_BT_TRACE("\n -----RAW VAL X----------- \n");
    for (i = 0; i < p_data->prop_value_len; i++)
        WICED_BT_TRACE(" %x ",p_data->column_data.raw_valuex[i]);
    WICED_BT_TRACE("\n -----COL WIDTH----------- \n");
    for (i = 0; i < p_data->prop_value_len; i++)
           WICED_BT_TRACE(" %x ",p_data->column_data.column_width[i]);
    WICED_BT_TRACE("\n -----RAW VAL Y----------- \n");
    for (i = 0; i < p_data->prop_value_len; i++)
           WICED_BT_TRACE(" %x ",p_data->column_data.raw_valuey[i]);
    WICED_BT_TRACE("\n ------------------------ \n");
    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_SENSOR_COLUMN_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send sensor series status event over transport
 */
void mesh_sensor_series_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_series_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;
    int i, j;

    WICED_BT_TRACE(" property_id:%x val_len:%x no_columns:%x\n",p_data->property_id, p_data->prop_value_len, p_data->no_of_columns);

    UINT16_TO_STREAM(p, p_data->property_id);
    UINT8_TO_STREAM(p, p_data->prop_value_len);
    UINT8_TO_STREAM(p, p_data->no_of_columns);

    for (i = 0; i < p_data->no_of_columns; i++)
    {
        ARRAY_TO_STREAM(p, p_data->column_list->raw_valuex, p_data->prop_value_len);
        ARRAY_TO_STREAM(p, p_data->column_list->column_width, p_data->prop_value_len);
        ARRAY_TO_STREAM(p, p_data->column_list->raw_valuey, p_data->prop_value_len);

        WICED_BT_TRACE("\n -----RAW VAL X\n");
        for (j=0; j <  p_data->prop_value_len; j++)
            WICED_BT_TRACE(" %x ", p_data->column_list[i].raw_valuex[j]);
        WICED_BT_TRACE("\n -----COL WIDTH\n");
        for (j=0; j < p_data->prop_value_len; j++)
               WICED_BT_TRACE(" %x ", p_data->column_list[i].column_width[j]);
        WICED_BT_TRACE("\n -----RAW VAL Y\n");
        for (j=0; j < p_data->prop_value_len; j++)
               WICED_BT_TRACE(" %x ", p_data->column_list[i].raw_valuey[j]);
    }
    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_SENSOR_SERIES_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send sensor cadence status event over transport
 */

void mesh_sensor_cadence_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_cadence_status_data_t *cadence_status)
{
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, cadence_status->property_id);
    WICED_BT_TRACE(" property_id:%x\n",cadence_status->property_id);

    if (cadence_status->is_data_present)
    {
        WICED_BT_TRACE("fast_cadence_period_divisor:%x\n", cadence_status->cadence_data.fast_cadence_period_divisor);
        WICED_BT_TRACE("trigger_type:%x\n", cadence_status->cadence_data.trigger_type);
        WICED_BT_TRACE("trigger_delta_down:%d\n", cadence_status->cadence_data.trigger_delta_down);
        WICED_BT_TRACE("trigger_delta_up:%d\n", cadence_status->cadence_data.trigger_delta_up);
        WICED_BT_TRACE("min interval:%x\n", cadence_status->cadence_data.min_interval);
        WICED_BT_TRACE("fast_cadence_high:%d\n", cadence_status->cadence_data.fast_cadence_high);
        WICED_BT_TRACE("fast_cadence_low:%d\n", cadence_status->cadence_data.fast_cadence_low);

        UINT16_TO_STREAM(p, cadence_status->cadence_data.fast_cadence_period_divisor);
        UINT8_TO_STREAM(p, cadence_status->cadence_data.trigger_type);
        UINT32_TO_STREAM(p, cadence_status->cadence_data.trigger_delta_down);
        UINT32_TO_STREAM(p, cadence_status->cadence_data.trigger_delta_up);
        UINT32_TO_STREAM(p, cadence_status->cadence_data.min_interval);
        UINT32_TO_STREAM(p, cadence_status->cadence_data.fast_cadence_low);
        UINT32_TO_STREAM(p, cadence_status->cadence_data.fast_cadence_high);
    }
    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_SENSOR_CADENCE_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send sensor setting status event over transport
 */
void mesh_sensor_setting_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_setting_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;
    int i;

    UINT16_TO_STREAM(p, p_data->property_id);
    UINT8_TO_STREAM(p, p_data->setting.setting_property_id);
    UINT8_TO_STREAM(p, p_data->setting.access);
    UINT8_TO_STREAM(p, p_data->setting.value_len);
    ARRAY_TO_STREAM(p, p_data->setting.val, p_data->setting.value_len);

    WICED_BT_TRACE(" property_id:%x\n", p_data->property_id);
    WICED_BT_TRACE(" setting_property_id:%x\n",p_data->setting.setting_property_id);
    WICED_BT_TRACE(" access:%x\n",p_data->setting.access);
    WICED_BT_TRACE(" value_len:%x\n", p_data->setting.value_len);
    for (i = 0; i < p_data->setting.value_len; i++)
        WICED_BT_TRACE(" %x ",p_data->setting.val[i]);

    WICED_BT_TRACE("\n----------\n");
    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_SENSOR_SETTING_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send sensor settings status event over transport
 */
void mesh_sensor_settings_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_settings_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;
    int i;

    UINT16_TO_STREAM(p, p_data->property_id);
    WICED_BT_TRACE(" property_id:%x\n", p_data->property_id);

    for (i = 0; i < p_data->num_setting_property_id; i++)
    {
        UINT16_TO_STREAM(p, p_data->setting_property_id_list[i]);
        WICED_BT_TRACE(" %x ", p_data->setting_property_id_list[i]);
    }
    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_SENSOR_SETTINGS_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

#endif
