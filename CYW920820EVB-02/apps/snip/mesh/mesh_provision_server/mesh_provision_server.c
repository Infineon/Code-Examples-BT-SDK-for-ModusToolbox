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
 * This file shows various methods that device can support for provisioning.
 * It works along with the ClientControl application running on the MCU.  MCU can
 * configure device to use or not to use static public key and various authentication
 * methods.  The real implementation may use portions of this app after figuring
 * out what options are available in the hardware.  For example if application is
 * a bulb and can blink, the appropriate method and not use other methods.
 */
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_timer.h"
#include "wiced_hal_rand.h"
#include "wiced_bt_mesh_provision.h"
#include "wiced_bt_mesh_app.h"

#ifdef HCI_CONTROL
#include "wiced_transport.h"
#include "hci_control_api.h"
#endif

#include "wiced_bt_cfg.h"
extern wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

// Needed to pass some PTS tests which requre vendor model
#define MESH_VENDOR_TST_COMPANY_ID      0x131
#define MESH_VENDOR_TST_MODEL_ID        1

#define MESH_ONOFF_TICK_IN_MS           100

/******************************************************
 *          Constants
 ******************************************************/
#define MESH_PID                        0x301E
#define MESH_VID                        0x0002
#define MESH_FWID                       0x301E000101010001
#define MESH_CACHE_REPLAY_SIZE          0x0008

/******************************************************
 *          Structures
 ******************************************************/
typedef struct
{
    uint8_t static_oob[16];
} mesh_provision_server_t;

/******************************************************
 *          Function Prototypes
 ******************************************************/
static void mesh_app_init(wiced_bool_t is_provisioned);
static uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
static void mesh_provision_server_message_handler(uint16_t event, void *p_data);
static void mesh_provision_server_oob_configure(uint8_t *p_data, uint32_t length);
static void mesh_provision_server_process_oob_get(wiced_bt_mesh_provision_device_oob_request_data_t *p_data);
static void mesh_provisioner_hci_event_device_get_oob_data_send(wiced_bt_mesh_provision_device_oob_request_data_t *p_oob_data);
static uint8_t mesh_provisioner_process_oob_value(uint8_t *p_data, uint32_t length);

#ifdef HCI_CONTROL
//static void mesh_provision_hci_event_send_set(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_provision_set_data_t *p_data);
#endif

/******************************************************
 *          Variables Definitions
 ******************************************************/
// Use hardcoded PTS default priv key. In real app it will be generated once and written into OTP memory
uint8_t pb_priv_key[WICED_BT_MESH_PROVISION_PRIV_KEY_LEN] = { 0x52, 0x9A, 0xA0, 0x67, 0x0D, 0x72, 0xCD, 0x64, 0x97, 0x50, 0x2E, 0xD4, 0x73, 0x50, 0x2B, 0x03, 0x7E, 0x88, 0x03, 0xB5, 0xC6, 0x08, 0x29, 0xA5, 0xA3, 0xCA, 0xA2, 0x19, 0x50, 0x55, 0x30, 0xBA };

static void mesh_provisioner_hci_send_status(uint8_t status);

// Application state
mesh_provision_server_t app_state =
{
    .static_oob = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08},
};
uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME] = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER]     = { '1', '2', '3', '4', 0, 0, 0, 0 };
uint8_t mesh_system_id[8]                                                  = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

extern wiced_transport_buffer_pool_t* host_trans_pool;

wiced_bt_mesh_core_config_model_t   mesh_element1_models[] =
{
    WICED_BT_MESH_DEVICE,
};
#define MESH_APP_NUM_MODELS  (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t))

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
    wiced_bt_mesh_provision_capabilities_data_t config;

    wiced_bt_cfg_settings.device_name = (uint8_t *)"Provision Server";
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

    // call app library to provide private key and register event handler
    wiced_bt_mesh_app_provision_server_init(pb_priv_key, mesh_provision_server_message_handler);

    config.pub_key_type      = 1;
    config.static_oob_type   = 1;
    config.output_oob_action = (1 << WICED_BT_MESH_PROVISION_OUT_OOB_ACT_DISP_NUM);
    config.output_oob_size   = 1;
    config.input_oob_action  = (1 << WICED_BT_MESH_PROVISION_IN_OOB_ACT_ENTER_NUM);
    config.input_oob_size    = 1;
    wiced_bt_mesh_app_provision_server_configure(&config);

}

/*
 * Process event received from the OnOff Client.
 */
void mesh_provision_server_message_handler(uint16_t event, void *p_data)
{
    WICED_BT_TRACE("provision srv msg:%d\n", event);

    switch (event)
    {
    case WICED_BT_MESH_PROVISION_STARTED:
        // mesh_provisioner_hci_event_provision_started_send((wiced_bt_mesh_provision_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_PROVISION_END:
        // mesh_provisioner_hci_event_provision_end_send((wiced_bt_mesh_provision_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_PROVISION_DEVICE_CAPABILITIES:
        // mesh_provisioner_hci_event_device_capabilities_send((wiced_bt_mesh_provision_device_capabilities_data_t *)p_data);
        break;

    case WICED_BT_MESH_PROVISION_GET_OOB_DATA:
        mesh_provision_server_process_oob_get((wiced_bt_mesh_provision_device_oob_request_data_t *)p_data);
        break;

    default:
        WICED_BT_TRACE("unknown\n");
    }
}

/*
 * In 2 chip solutions MCU can send commands to change provision state.
 */
uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
    uint8_t status = HCI_CONTROL_MESH_STATUS_ERROR;

    WICED_BT_TRACE("[%s] cmd_opcode 0x%02x\n", __FUNCTION__, opcode);

    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_PROVISION_OOB_CONFIGURE:
        mesh_provision_server_oob_configure(p_data, length);
        status = HCI_CONTROL_MESH_STATUS_SUCCESS;
        break;

    case HCI_CONTROL_MESH_COMMAND_PROVISION_OOB_VALUE:
        wiced_bt_mesh_skip_wiced_hci_hdr(&p_data, &length);
        status = mesh_provisioner_process_oob_value(p_data, length);
        break;

    default:
        WICED_BT_TRACE("unknown\n");
        break;
    }
    mesh_provisioner_hci_send_status(status);
    return 0;
}

/*
 * Send OnOff Status event
 */
void mesh_provision_server_oob_configure(uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_provision_capabilities_data_t config;

    STREAM_TO_UINT8(config.pub_key_type, p_data);
    STREAM_TO_UINT8(config.static_oob_type, p_data);
    STREAM_TO_UINT16(config.output_oob_action, p_data);
    STREAM_TO_UINT8(config.output_oob_size, p_data);
    STREAM_TO_UINT16(config.input_oob_action, p_data);
    STREAM_TO_UINT8(config.input_oob_size, p_data);
    STREAM_TO_ARRAY(app_state.static_oob, p_data, sizeof(config.static_oob));
    wiced_bt_mesh_app_provision_server_configure(&config);
}

void mesh_provision_server_process_oob_get(wiced_bt_mesh_provision_device_oob_request_data_t *p_data)
{
    wiced_bt_mesh_provision_oob_value_data_t oob;
    int i;
    uint32_t max_value;
    uint32_t rand = wiced_hal_rand_gen_num();

    WICED_BT_TRACE("\n\n#####OOB get: type:%d size:%d action:%d\n", p_data->type, p_data->size, p_data->action);
    if (p_data->type == WICED_BT_MESH_PROVISION_GET_OOB_TYPE_DISPLAY_OUTPUT)
    {
        // sanity check.  max should be less or equal to 8
        oob.data_size = (p_data->size > 8) ? 8 : p_data->size;

        // Generate output depending on the action
        switch (p_data->action)
        {
        case WICED_BT_MESH_PROVISION_OUT_OOB_ACT_DISP_NUM:
            for (i = 0; i < oob.data_size; i++)
            {
                uint32_t rand = wiced_hal_rand_gen_num();
                oob.data[i] = (rand % 9);
                WICED_BT_TRACE("%d %d", rand, oob.data[i]);
            }
            WICED_BT_TRACE("\n");
            break;

        case WICED_BT_MESH_PROVISION_OUT_OOB_ACT_DISP_ALPH:
            for (i = 0; i < oob.data_size; )
            {
                oob.data[i] = (wiced_hal_rand_gen_num() % 'Z');
                if (((oob.data[i] > '0') && (oob.data[i] <= '9')) ||
                    ((oob.data[i] > 'A') && (oob.data[i] <= 'Z')))
                {
                    WICED_BT_TRACE("%c", oob.data[i]);
                    i++;
                }
            }
            WICED_BT_TRACE("\n", oob.data[i]);
            break;
        default:
        //case WICED_BT_MESH_PROVISION_OUT_OOB_ACT_BLINK:
        //case WICED_BT_MESH_PROVISION_OUT_OOB_ACT_BEEP:
        //case WICED_BT_MESH_PROVISION_OUT_OOB_ACT_VIBRATE:
            oob.data[0] = (wiced_hal_rand_gen_num() % 8) + 1;
            WICED_BT_TRACE("%d\n", oob.data[0]);
            oob.data_size = 1;
            break;
        }
        // oob.conn_id = p_data->conn_id;
        wiced_bt_mesh_provision_set_oob(&oob);
    }
    else if (p_data->type == WICED_BT_MESH_PROVISION_GET_OOB_TYPE_ENTER_INPUT)
    {
        mesh_provisioner_hci_event_device_get_oob_data_send(p_data);
    }
    else if (p_data->type == WICED_BT_MESH_PROVISION_GET_OOB_TYPE_GET_STATIC)
    {
        // oob.conn_id = p_data->conn_id;
        memcpy(oob.data, app_state.static_oob, sizeof(app_state.static_oob));
        oob.data_size = sizeof(app_state.static_oob);
        wiced_bt_mesh_provision_set_oob(&oob);
    }
}

/*
 * Send to the MCU Out of Band data request
 */
void mesh_provisioner_hci_event_device_get_oob_data_send(wiced_bt_mesh_provision_device_oob_request_data_t *p_oob_data)
{
    uint8_t *p_buffer = wiced_transport_allocate_buffer(host_trans_pool);
    uint8_t *p = p_buffer;

    WICED_BT_TRACE("mesh prov oob req type:%d\n", /*p_oob_data->conn_id, */p_oob_data->type);

    UINT16_TO_STREAM(p, p_oob_data->provisioner_addr);
    UINT8_TO_STREAM(p, p_oob_data->type);
    UINT8_TO_STREAM(p, p_oob_data->size);
    UINT8_TO_STREAM(p, p_oob_data->action);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_PROVISION_OOB_DATA, p_buffer, (uint16_t)(p - p_buffer));
}

void mesh_provisioner_hci_send_status(uint8_t status)
{
    uint8_t *p_buffer = wiced_transport_allocate_buffer(host_trans_pool);
    uint8_t *p = p_buffer;

    UINT8_TO_STREAM(p, status);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_COMMAND_STATUS, p_buffer, (uint16_t)(p - p_buffer));
}

/*
 * Process command from MCU to with OOB value to be used in the link calculation
 */
uint8_t mesh_provisioner_process_oob_value(uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_provision_oob_value_data_t oob;

    STREAM_TO_UINT8(oob.data_size, p_data);
    STREAM_TO_ARRAY(oob.data, p_data, length);

    return wiced_bt_mesh_provision_set_oob(&oob) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}
