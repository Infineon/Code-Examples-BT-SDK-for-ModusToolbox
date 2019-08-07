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
 * This file shows how to create a device which implements mesh Mesh Light LC Client.
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
#define MESH_PID                0x300C
#define MESH_VID                0x0002
#define MESH_FWID               0x300C000101010001
#define MESH_CACHE_REPLAY_SIZE  0x0008

/******************************************************
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
static void mesh_app_init(wiced_bool_t is_provisioned);
static uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void mesh_light_lc_client_mode_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_client_mode_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_client_occupancy_mode_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_client_occupancy_mode_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_client_onoff_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_client_onoff_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_client_property_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_client_property_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_hci_event_send_mode_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lc_mode_set_data_t *p_data);
static void mesh_light_lc_hci_event_send_occupancy_mode_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lc_occupancy_mode_set_data_t *p_data);
static void mesh_light_lc_hci_event_send_light_onoff_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lc_light_onoff_status_data_t *p_data);
static void mesh_light_lc_hci_event_send_property_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lc_property_status_data_t *p_data);

/******************************************************
 *          Variables Definitions
 ******************************************************/

uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME] = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER]     = { '1', '2', '3', '4', 0, 0, 0, 0 };
uint8_t mesh_system_id[8]                                                  = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

wiced_bt_mesh_core_config_model_t   mesh_element1_models[] =
{
    WICED_BT_MESH_DEVICE,
    WICED_BT_MESH_MODEL_LIGHT_LC_CLIENT,
};
#define MESH_APP_NUM_MODELS  (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t))

#define MESH_LIGHT_LC_CLIENT_ELEMENT_INDEX   0

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
    wiced_bt_cfg_settings.device_name = (uint8_t *)"Light Control";
    wiced_bt_cfg_settings.gatt_cfg.appearance = APPEARANCE_CONTROL_DEVICE_TOUCH_PANEL;
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

    wiced_bt_mesh_model_light_lc_client_init(MESH_LIGHT_LC_CLIENT_ELEMENT_INDEX, mesh_light_lc_client_message_handler, is_provisioned);
}

/*
 * Process event received from the Light LC Server.
 */
void mesh_light_lc_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
    wiced_bt_mesh_light_lc_mode_set_data_t *p_mode;
    wiced_bt_mesh_light_lc_occupancy_mode_set_data_t *p_occupancy_mode;
    wiced_bt_mesh_light_lc_light_onoff_status_data_t *p_onoff_status;
    wiced_bt_mesh_light_lc_property_status_data_t *p_property_status;
#if defined HCI_CONTROL
    wiced_bt_mesh_hci_event_t *p_hci_event;
#endif

    WICED_BT_TRACE("light lc clnt msg:%d\n", event);

    switch (event)
    {
    case WICED_BT_MESH_TX_COMPLETE:
        WICED_BT_TRACE("tx complete status:%d\n", p_event->tx_status);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            wiced_bt_mesh_send_hci_tx_complete(p_hci_event, p_event);
#endif
        break;

    case WICED_BT_MESH_LIGHT_LC_MODE_STATUS:
        p_mode = (wiced_bt_mesh_light_lc_mode_set_data_t *)p_data;
        WICED_BT_TRACE("light lc mode:%d\n", p_mode->mode);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_light_lc_hci_event_send_mode_status(p_hci_event, p_mode);
#endif
        break;

    case WICED_BT_MESH_LIGHT_LC_OCCUPANCY_MODE_STATUS:
        p_occupancy_mode = (wiced_bt_mesh_light_lc_occupancy_mode_set_data_t *)p_data;
        WICED_BT_TRACE("light lc occupany mode:%d\n", p_occupancy_mode->mode);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_light_lc_hci_event_send_occupancy_mode_status(p_hci_event, p_occupancy_mode);
#endif
        break;

    case WICED_BT_MESH_LIGHT_LC_LIGHT_ONOFF_STATUS:
        p_onoff_status = (wiced_bt_mesh_light_lc_light_onoff_status_data_t *)p_data;
        WICED_BT_TRACE("light onoff status present:%d target:%d remain time:%d\n",
            p_onoff_status->present_onoff, p_onoff_status->target_onoff, p_onoff_status->remaining_time);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_light_lc_hci_event_send_light_onoff_status(p_hci_event, p_onoff_status);
#endif
        break;

    case WICED_BT_MESH_LIGHT_LC_PROPERTY_STATUS:
        p_property_status = (wiced_bt_mesh_light_lc_property_status_data_t *)p_data;
        WICED_BT_TRACE("light property status ID:%d len:%d\n", p_property_status->id, p_property_status->len);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_light_lc_hci_event_send_property_status(p_hci_event, p_property_status);
#endif
        break;

    default:
        break;
    }
    wiced_bt_mesh_release_event(p_event);
}

/*
 * In 2 chip solutions MCU can send commands to change onoff state.
 */
uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
#ifdef HCI_CONTROL
    wiced_bt_mesh_event_t *p_event;

    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_MODE_GET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_MODE_SET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_OCCUPANCY_MODE_GET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_OCCUPANCY_MODE_SET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_ONOFF_GET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_ONOFF_SET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_PROPERTY_GET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_PROPERTY_SET:
        break;

    default:
        return WICED_FALSE;
    }
    p_event = wiced_bt_mesh_create_event_from_wiced_hci(opcode, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LC_CLNT, &p_data, &length);
    if (p_event == NULL)
    {
        WICED_BT_TRACE("bad hdr\n");
        return WICED_TRUE;
    }
    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_MODE_GET:
        mesh_light_lc_client_mode_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_MODE_SET:
        mesh_light_lc_client_mode_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_OCCUPANCY_MODE_GET:
        mesh_light_lc_client_occupancy_mode_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_OCCUPANCY_MODE_SET:
        mesh_light_lc_client_occupancy_mode_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_ONOFF_GET:
        mesh_light_lc_client_onoff_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_ONOFF_SET:
        mesh_light_lc_client_onoff_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_PROPERTY_GET:
        mesh_light_lc_client_property_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_PROPERTY_SET:
        mesh_light_lc_client_property_set(p_event, p_data, length);
        break;
    }
#endif
    return WICED_TRUE;
}

/*
 * Send Light LC Mode Get command
 */
void mesh_light_lc_client_mode_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_light_lc_client_send_mode_get(p_event);
}

/*
 * Send Light LC Mode set command
 */
void mesh_light_lc_client_mode_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_light_lc_mode_set_data_t set_data;

    STREAM_TO_UINT8(set_data.mode, p_data);

    wiced_bt_mesh_model_light_lc_client_send_mode_set(p_event, &set_data);
}

/*
 * Send Light Occupancy Mode get command
 */
void mesh_light_lc_client_occupancy_mode_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_light_lc_client_send_occupancy_mode_get(p_event);
}

/*
 * Send Light LC Occupancy Mode set command
 */
void mesh_light_lc_client_occupancy_mode_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_light_lc_occupancy_mode_set_data_t set_data;

    STREAM_TO_UINT8(set_data.mode, p_data);

    wiced_bt_mesh_model_light_lc_client_send_occupancy_mode_set(p_event, &set_data);
}

/*
 * Send Light LC OnOff get command
 */
void mesh_light_lc_client_onoff_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_light_lc_client_send_light_onoff_get(p_event);
}

/*
 * Send Light LC OnOff set command
 */
void mesh_light_lc_client_onoff_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_light_lc_light_onoff_set_data_t set_data;

    STREAM_TO_UINT8(set_data.light_onoff, p_data);
    STREAM_TO_UINT32(set_data.transition_time, p_data);
    STREAM_TO_UINT16(set_data.delay, p_data);

    wiced_bt_mesh_model_light_lc_client_send_light_onoff_set(p_event, &set_data);
}

/*
 * Send Light LC Property Get command
 */
void mesh_light_lc_client_property_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_light_lc_property_get_data_t get_data;

    STREAM_TO_UINT16(get_data.id, p_data);

    wiced_bt_mesh_model_light_lc_client_send_property_get(p_event, &get_data);
}

/*
 * Send Light LC Property set command
 */
void mesh_light_lc_client_property_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_light_lc_property_set_data_t set_data;

    STREAM_TO_UINT16(set_data.id, p_data);

    set_data.len = length - 2;
    memcpy(set_data.value, p_data, set_data.len);

    wiced_bt_mesh_model_light_lc_client_send_property_set(p_event, &set_data);
}


#ifdef HCI_CONTROL
/*
 * Send Light LC Mode Status event over transport
 */
void mesh_light_lc_hci_event_send_mode_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lc_mode_set_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT8_TO_STREAM(p, p_data->mode);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LIGHT_LC_MODE_CLIENT_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Light LC Occupancy Mode Status event over transport
 */
void mesh_light_lc_hci_event_send_occupancy_mode_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lc_occupancy_mode_set_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT8_TO_STREAM(p, p_data->mode);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LIGHT_LC_OCCUPANCY_MODE_CLIENT_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Light LC Occupancy Light OnOff Status event over transport
 */
void mesh_light_lc_hci_event_send_light_onoff_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lc_light_onoff_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT8_TO_STREAM(p, p_data->present_onoff);
    UINT8_TO_STREAM(p, p_data->target_onoff);
    UINT32_TO_STREAM(p, p_data->remaining_time);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LIGHT_LC_ONOFF_CLIENT_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Light LC Occupancy Property Status event over transport
 */
void mesh_light_lc_hci_event_send_property_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lc_property_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, p_data->id);
    memcpy(p, p_data->value, p_data->len);
    p += p_data->len;

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LIGHT_LC_PROPERTY_CLIENT_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

#endif
