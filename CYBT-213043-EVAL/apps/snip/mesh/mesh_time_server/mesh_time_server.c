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
 * This file shows how to create a device which implements mesh user time client.
 */
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "rtc.h"
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
#define MESH_PID                0x3024
#define MESH_VID                0x0002
#define MESH_FWID               0x3024000101010001
#define MESH_CACHE_REPLAY_SIZE  0x0008

/******************************************************
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
static void mesh_app_init(wiced_bool_t is_provisioned);
static uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
static void mesh_time_server_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void mesh_time_get(uint8_t *p_data, uint32_t length);
static void mesh_time_set(uint8_t *p_data, uint32_t length);
static void mesh_time_send_status(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_time_set_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, RtcTime *p_time);

/******************************************************
 *          Variables Definitions
 ******************************************************/
uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME] = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER]     = { '1', '2', '3', '4', 0, 0, 0, 0 };
uint8_t mesh_system_id[8]                                                  = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

wiced_bt_mesh_core_config_model_t   mesh_element1_models[] =
{
        WICED_BT_MESH_DEVICE,
        WICED_BT_MESH_MODEL_TIME_SERVER,
};
#define MESH_APP_NUM_MODELS  (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t))

#define MESH_TIME_SERVER_ELEMENT_INDEX   0

wiced_bt_mesh_core_config_element_t mesh_elements[] =
{
    {
        .location = MESH_ELEM_LOC_MAIN,                                 // location description as defined in the GATT Bluetooth Namespace Descriptors section of the Bluetooth SIG Assigned Numbers
        .default_transition_time = MESH_DEFAULT_TRANSITION_TIME_IN_MS,  // Default transition time for models of the element in milliseconds
        .onpowerup_state = WICED_BT_MESH_ON_POWER_UP_STATE_RESTORE,     // Default element behavior on power up
        .properties_num = 4,                                            // Number of properties in the array models
        .properties = NULL,                                             // Array of properties in the element.
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
#if LOW_POWER_NODE
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
    .features           = WICED_BT_MESH_CORE_FEATURE_BIT_FRIEND | WICED_BT_MESH_CORE_FEATURE_BIT_RELAY | WICED_BT_MESH_CORE_FEATURE_BIT_GATT_PROXY_SERVER,   // In Friend mode support friend, relay
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
#if 0
    extern uint8_t wiced_bt_mesh_model_trace_enabled;
    wiced_bt_mesh_model_trace_enabled = WICED_TRUE;

    // enable core trace
    extern void wiced_bt_mesh_core_set_trace_level(uint32_t fids_mask, uint8_t level);
    wiced_bt_mesh_core_set_trace_level(0xffffffff, 4);      //(ALL, TRACE_DEBUG)
    wiced_bt_mesh_core_set_trace_level(0x4, 3);             //(CORE_AES_CCM_C, TRACE_INFO)
#endif

    wiced_bt_cfg_settings.device_name = (uint8_t *)"Time Server";
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
    wiced_bt_mesh_model_time_server_init(mesh_time_server_message_handler, is_provisioned);
}

/*
 * Process event received from the time client.
 */
void mesh_time_server_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
#if defined HCI_CONTROL
    wiced_bt_mesh_hci_event_t *p_hci_event;
#endif
    WICED_BT_TRACE("time srv msg:%d\n", event);

    RtcTime *p_time;
    switch (event)
    {
    case WICED_BT_MESH_TIME_SET:
    {
        p_time = (RtcTime*)p_data;
        if (p_event)
        {
#if defined HCI_CONTROL
            p_hci_event = wiced_bt_mesh_create_hci_event(p_event);
            if (p_hci_event != NULL)
            {
                mesh_time_set_hci_event_send(p_hci_event, p_time);
            }
#endif
        }
        break;
    }
    default:
        WICED_BT_TRACE("unknown\n");
    break;
    }
}


/*
 * In 2 chip solutions MCU can send commands to change time state.
 */
uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_event_t *p_event;

    WICED_BT_TRACE("[%s] cmd_opcode 0x%02x\n", __FUNCTION__, opcode);

    switch (opcode)
    {
//    case HCI_CONTROL_MESH_COMMAND_TIME_SERVER_SET:
//           mesh_time_set(p_data, length);
//           break;

    case HCI_CONTROL_MESH_COMMAND_TIME_SET:
        p_event = wiced_bt_mesh_create_event_from_wiced_hci(opcode, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, &p_data, &length);
        if (p_event == NULL)
        {
            WICED_BT_TRACE("bad hdr\n");
            return WICED_TRUE;
        }
        mesh_time_send_status(p_event, p_data, length);
        break;

    default:
        WICED_BT_TRACE("unknown\n");
        return WICED_FALSE;
    }
    return WICED_TRUE;
}

#if 0
/*
 * Send time set command
 */
void mesh_time_set(uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_time_state_msg_t set_data;
    WICED_BT_TRACE("mesh_time_set\n");

    STREAM_TO_UINT40(set_data.tai_seconds, p_data);
    STREAM_TO_UINT8(set_data.subsecond, p_data);
    STREAM_TO_UINT8(set_data.uncertainty, p_data);

    set_data.time_authority = WICED_TRUE;
    STREAM_TO_UINT16(set_data.tai_utc_delta_current, p_data);
    STREAM_TO_UINT8(set_data.time_zone_offset_current, p_data);

    wiced_bt_mesh_model_time_sever_time_set(&set_data);
}
*/

/*
 * Send time zone set command
 */
void mesh_time_zone_set(uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_time_zone_set_t set_data;
    WICED_BT_TRACE("mesh_time_zone_set\n");

    STREAM_TO_UINT8(set_data.time_zone_offset_new, p_data);
    STREAM_TO_UINT40(set_data.tai_of_zone_change, p_data);
    wiced_bt_mesh_model_time_server_time_zone_set(&set_data);
}

/*
 * Send time tai_utc delta set command
 */
void mesh_time_tai_utc_delta_set(uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_time_tai_utc_delta_set_t set_data;
    WICED_BT_TRACE("mesh_time_tai_utc_delta_set\n");

    STREAM_TO_UINT16(set_data.tai_utc_delta_new, p_data);
    STREAM_TO_UINT40(set_data.tai_of_delta_change, p_data);

    wiced_bt_mesh_model_time_server_tai_utc_delta_set(&set_data);
}
#endif

void mesh_time_send_status(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t data_len)
{
    wiced_bt_mesh_time_state_msg_t status;

    WICED_BT_TRACE("mesh_time_status_send len:%d\n", data_len);

    if (data_len == 9)
    {
        memset(&status, 0, sizeof(wiced_bt_mesh_time_state_msg_t));
        wiced_bt_mesh_model_time_server_status_send(p_event, &status);
    }
    else
    {
        STREAM_TO_UINT40(status.tai_seconds, p_data);
        STREAM_TO_UINT8(status.subsecond, p_data);
        STREAM_TO_UINT8(status.uncertainty, p_data);
        STREAM_TO_UINT8(status.time_authority, p_data);
        STREAM_TO_UINT16(status.tai_utc_delta_current, p_data);
        STREAM_TO_UINT8(status.time_zone_offset_current, p_data);
        wiced_bt_mesh_model_time_server_status_send(p_event, &status);
    }

}

#ifdef HCI_CONTROL
/*
 * Send Time set event over transport
 */
void mesh_time_set_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, RtcTime *p_time)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("mesh_time_set_hci_event_send year %d month:%d day:%d hour:%d min:%d sec:%d", p_time->year, p_time->month, p_time->day, p_time->hour, p_time->minute, p_time->second);

    UINT16_TO_STREAM(p, p_time->year);
    UINT16_TO_STREAM(p, p_time->month);
    UINT16_TO_STREAM(p, p_time->day);
    UINT16_TO_STREAM(p, p_time->hour);
    UINT16_TO_STREAM(p, p_time->minute);
    UINT16_TO_STREAM(p, p_time->second);
    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_TIME_SET, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));

}

#endif

#ifdef CYW20706A2
// Core initialization initializes the RTC calling wiced_bt_mesh_core_rtc_init() defined in app. It just should call rtc_init().
// For that chip rtc_init should be called from application. Otherwise there is build error at link phase
void wiced_bt_mesh_core_rtc_init(void)
{
    rtc_init();
}
#endif
