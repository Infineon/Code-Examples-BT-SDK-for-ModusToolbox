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
 * This file shows how to create a device which publishes user_property level.
 */
#include "wiced_bt_uuid.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_timer.h"
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
#define MESH_PID                0x3122
#define MESH_VID                0x0002
#define MESH_FWID               0x3022000101010001
#define MESH_CACHE_REPLAY_SIZE  0x0008

#define MESH_TEMP_SENSOR_PROPERTY_ID                    WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_TEMPERATURE
#define MESH_TEMP_SENSOR_VALUE_LEN                      WICED_BT_MESH_PROPERTY_LEN_PRESENT_AMBIENT_TEMPERATURE

// The onboard thermistor hardware has a positive and negative tolerance of 1%
#define MESH_TEMPERATURE_SENSOR_POSITIVE_TOLERANCE      CONVERT_TOLERANCE_PERCENTAGE_TO_MESH(1)
#define MESH_TEMPERATURE_SENSOR_NEGATIVE_TOLERANCE      CONVERT_TOLERANCE_PERCENTAGE_TO_MESH(1)

#define MESH_TEMPERATURE_SENSOR_SAMPLING_FUNCTION       WICED_BT_MESH_SENSOR_SAMPLING_FUNCTION_UNKNOWN
#define MESH_TEMPERATURE_SENSOR_MEASUREMENT_PERIOD      WICED_BT_MESH_SENSOR_VAL_UNKNOWN
#define MESH_TEMPERATURE_SENSOR_UPDATE_INTERVAL         WICED_BT_MESH_SENSOR_VAL_UNKNOWN

/******************************************************
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
static void mesh_app_init(wiced_bool_t is_provisioned);
static wiced_bool_t mesh_app_notify_period_set(uint8_t element_idx, uint16_t company_id, uint16_t model_id, uint32_t period);
static uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
static void mesh_sensor_server_restart_timer(wiced_bt_mesh_core_config_sensor_t *p_sensor);
static void mesh_sensor_server_report_handler(uint16_t event, uint8_t element_idx, void *p_get, void *p_ref_data);
static void mesh_sensor_server_config_change_handler(uint8_t element_idx, uint16_t event, uint16_t property_id, uint16_t setting_prop_id);
static void mesh_sensor_server_status_changed(uint8_t element_idx, uint8_t *p_data, uint32_t length);
static void mesh_sensor_server_send_column_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_sensor_column_get_data_t *p_get_column);
static void mesh_sensor_server_send_series_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_sensor_series_get_data_t* data);
static void mesh_sensor_server_process_cadence_changed(uint8_t element_idx, uint16_t property_id);
static void mesh_sensor_server_process_setting_changed(uint8_t element_idx, uint16_t property_id, uint16_t setting_property_id);
static void mesh_sensor_publish_timer_callback(TIMER_PARAM_TYPE arg);

#ifdef HCI_CONTROL
static void mesh_sensor_hci_event_send_cadence_set(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_cadence_set_data_t *p_set);
static void mesh_sensor_hci_event_send_setting_set(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_setting_set_data_t *p_set);
#endif

/******************************************************
 *          Variables Definitions
 ******************************************************/
uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME] = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER]     = { '1', '2', '3', '4', 0, 0, 0, 0 };
uint8_t mesh_system_id[8]                                                  = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

// Present Ambient Temperature property uses Temperature 8 format, i.e. 0.5 degree Celsius.
int8_t        mesh_sensor_current_temperature = 42; // 21 degree Celsius
int8_t        mesh_sensor_sent_value = 0;           //
uint32_t      mesh_sensor_sent_time;                // time stamp when temperature was published
uint32_t      mesh_sensor_publish_period = 0;       // publish period in msec
uint32_t      mesh_sensor_fast_publish_period = 0;  // publish period in msec when values are outside of limit
wiced_timer_t mesh_sensor_cadence_timer;

// Optional setting for the temperature sensor, the Total Device Runtime, in Time Hour 24 format
uint8_t mesh_temperature_sensor_setting0_val[] = { 0x01, 0x00, 0x00 };

// first set of series and column values
uint8_t raw_valuex_0[]                         = { 0x00, 0x01 };
uint8_t column_width0[]                        = { 0x01, 0x10 };
uint8_t raw_valuey0[]                          = { 0x00, 0x10 };

// second set of series and column values
uint8_t raw_valuex_1[]                         = { 0x00, 0x10 };
uint8_t column_width1[]                        = { 0x01, 0x10 };
uint8_t raw_valuey1[]                          = { 0x00, 0x20 };

wiced_bt_mesh_core_config_model_t mesh_element1_models[] =
{
    WICED_BT_MESH_DEVICE,
    WICED_BT_MESH_MODEL_SENSOR_SERVER,
};
#define MESH_APP_NUM_MODELS  (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t))

wiced_bt_mesh_sensor_config_setting_t sensor_settings[] =
{
    {
        .setting_property_id = WICED_BT_MESH_PROPERTY_TOTAL_DEVICE_RUNTIME,
        .access              = WICED_BT_MESH_SENSOR_SETTING_READABLE_AND_WRITABLE,
        .value_len           = 3,
        .val                 = mesh_temperature_sensor_setting0_val
    },
};


wiced_bt_mesh_sensor_config_column_data_t   mesh_temperature_sensor_columns[] =
{
    {
        .raw_valuex = raw_valuex_0,
        .column_width = column_width0,
        .raw_valuey = raw_valuey0,
    },
    {
        .raw_valuex = raw_valuex_1,
        .column_width = column_width1,
        .raw_valuey = raw_valuey1
    },
};

wiced_bt_mesh_core_config_sensor_t mesh_element1_sensors[] =
{
    {
        .property_id = WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_TEMPERATURE,
        .prop_value_len = WICED_BT_MESH_PROPERTY_LEN_PRESENT_AMBIENT_TEMPERATURE,
        .descriptor =
        {
            .positive_tolerance = MESH_TEMPERATURE_SENSOR_POSITIVE_TOLERANCE,
            .negative_tolerance = MESH_TEMPERATURE_SENSOR_NEGATIVE_TOLERANCE,
            .sampling_function  = MESH_TEMPERATURE_SENSOR_SAMPLING_FUNCTION,
            .measurement_period = MESH_TEMPERATURE_SENSOR_MEASUREMENT_PERIOD,
            .update_interval    = MESH_TEMPERATURE_SENSOR_UPDATE_INTERVAL,
        },
        .data = (uint8_t *)&mesh_sensor_current_temperature,
        .cadence =
        {
            // Value 1 indicates that cadence does not change depending on the measurements
            .fast_cadence_period_divisor = 1,            // Value of the divisor
            .trigger_type_percentage     = WICED_FALSE,
            .trigger_delta_down          = 0,
            .trigger_delta_up            = 0,
            .min_interval                = (1 << 0x0C),  // ~4 seconds
            .fast_cadence_low            = 0,
            .fast_cadence_high           = 0,
        },
        .num_series     = sizeof(mesh_temperature_sensor_columns)/sizeof(mesh_temperature_sensor_columns[0]),
        .series_columns = mesh_temperature_sensor_columns,
        .num_settings   = sizeof(sensor_settings)/sizeof(sensor_settings[0]),
        .settings       = sensor_settings,
    },
};


#define MESH_APP_NUM_PROPERTIES (sizeof(mesh_element1_properties) / sizeof(wiced_bt_mesh_core_config_property_t))

#define MESH_SENSOR_SERVER_ELEMENT_INDEX   0
#define MESH_TEMPERATURE_SENSOR_INDEX      0

wiced_bt_mesh_core_config_element_t mesh_elements[] =
{
    {
        .location = MESH_ELEM_LOC_MAIN,                                  // location description as defined in the GATT Bluetooth Namespace Descriptors section of the Bluetooth SIG Assigned Numbers
        .default_transition_time = MESH_DEFAULT_TRANSITION_TIME_IN_MS,   // Default transition time for models of the element in milliseconds
        .onpowerup_state = WICED_BT_MESH_ON_POWER_UP_STATE_RESTORE,      // Default element behavior on power up
        .default_level = 0,                                              // Default value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_min = 1,                                                  // Minimum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_max = 0xffff,                                             // Maximum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .move_rollover = 0,                                              // If true when level gets to range_max during move operation, it switches to min, otherwise move stops.
        .properties_num = 0,                                             // Number of properties in the array models
        .properties = NULL,                                              // Array of properties in the element.
        .sensors_num = 1,                                                // Number of properties in the array models
        .sensors = mesh_element1_sensors,                                // Array of properties in the element.
        .models_num = MESH_APP_NUM_MODELS,                               // Number of models in the array models
        .models = mesh_element1_models,                                  // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
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
    mesh_app_init,              // application initialization
    NULL,                       // Default SDK platform button processing
    NULL,                       // GATT connection status
    NULL,                       // attention processing
    mesh_app_notify_period_set, // notify period set
    mesh_app_proc_rx_cmd,       // WICED HCI command
    NULL,                   // LPN sleep
    NULL                        // factory reset
};

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
//    wiced_bt_mesh_core_set_trace_level((1 << FID_MESH_APP__PROVISIONING_C), 0);
//    wiced_bt_mesh_core_set_trace_level((1 << FID_MESH_APP__PB_TRANSPORT_C), 0);
#endif

    wiced_bt_cfg_settings.device_name = (uint8_t *)"Sensor";
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

    if (!is_provisioned)
        return;

    // initialize the cadence timer.  Need a timer for each element because each sensor model can be
    // configured for different publication period.  This app has only one sensor.
    wiced_init_timer(&mesh_sensor_cadence_timer, &mesh_sensor_publish_timer_callback, (TIMER_PARAM_TYPE)&mesh_config.elements[MESH_SENSOR_SERVER_ELEMENT_INDEX].sensors[MESH_TEMPERATURE_SENSOR_INDEX], WICED_MILLI_SECONDS_TIMER);

    wiced_bt_mesh_model_sensor_server_init(MESH_SENSOR_SERVER_ELEMENT_INDEX, mesh_sensor_server_report_handler, mesh_sensor_server_config_change_handler, is_provisioned);
}

/*
 * New publication period is set. If it is for the sensor model, this application should take care of it.
 * The period may need to be adjusted based on the divisor.
 */
wiced_bool_t mesh_app_notify_period_set(uint8_t element_idx, uint16_t company_id, uint16_t model_id, uint32_t period)
{
    if ((element_idx != MESH_TEMPERATURE_SENSOR_INDEX) || (company_id != MESH_COMPANY_ID_BT_SIG) || (model_id != WICED_BT_MESH_CORE_MODEL_ID_SENSOR_SRV))
    {
        return WICED_FALSE;
    }
    mesh_sensor_publish_period = period;
    WICED_BT_TRACE("Sensor data send period:%dms\n", mesh_sensor_publish_period);
    mesh_sensor_server_restart_timer(&mesh_config.elements[element_idx].sensors[MESH_TEMPERATURE_SENSOR_INDEX]);
    return WICED_TRUE;
}

/*
 * Start periodic timer depending on the publication period, fast cadence divisor and minimum interval
 */
void mesh_sensor_server_restart_timer(wiced_bt_mesh_core_config_sensor_t *p_sensor)
{
    // If there are no specific cadence settings, publish every publish period.
    uint32_t timeout = mesh_sensor_publish_period;

    wiced_stop_timer(&mesh_sensor_cadence_timer);
    if (mesh_sensor_publish_period == 0)
    {
        // The thermistor is not interrupt driven.  If client configured sensor to send notification when
        // the value changes, we will need to check periodically if the condition has been satisfied.
        // The cadence.min_interval can be used because we do not need to send data more often than that.
        if ((p_sensor->cadence.min_interval != 0) &&
            ((p_sensor->cadence.trigger_delta_up != 0) || (p_sensor->cadence.trigger_delta_down != 0)))
        {
            timeout = p_sensor->cadence.min_interval;
        }
        else
        {
            WICED_BT_TRACE("sensor restart timer period:%d\n", mesh_sensor_publish_period);
            return;
        }
    }
    else
    {
        // If fast cadence period divisor is set, we need to check temperature more
        // often than publication period.  Publish if measurement is in specified range
        if (p_sensor->cadence.fast_cadence_period_divisor > 1)
        {
            mesh_sensor_fast_publish_period = mesh_sensor_publish_period / p_sensor->cadence.fast_cadence_period_divisor;
            timeout = mesh_sensor_fast_publish_period;
        }
        else
        {
            mesh_sensor_fast_publish_period = 0;
        }
        // The thermistor is not interrupt driven.  If client configured sensor to send notification when
        // the value changes, we may need to check value more often not to miss the trigger.
        // The cadence.min_interval can be used because we do not need to send data more often than that.
        if ((p_sensor->cadence.min_interval < timeout) &&
            ((p_sensor->cadence.trigger_delta_up != 0) || (p_sensor->cadence.trigger_delta_down != 0)))
        {
            timeout = p_sensor->cadence.min_interval;
        }
    }
    WICED_BT_TRACE("sensor restart timer:%d\n", timeout);
    wiced_start_timer(&mesh_sensor_cadence_timer, timeout);
}

/*
 * Process the configuration changes set by the Sensor Client.
 */
//void mesh_sensor_server_config_change_handler(uint8_t element_idx, uint16_t event, void *p_data)
void mesh_sensor_server_config_change_handler(uint8_t element_idx, uint16_t event, uint16_t property_id, uint16_t setting_property_id)
{
#if defined HCI_CONTROL
    wiced_bt_mesh_hci_event_t *p_hci_event;
#endif
    WICED_BT_TRACE("mesh_sensor_server_config_change_handler msg: %d\n", event);

    switch (event)
    {

    case WICED_BT_MESH_SENSOR_CADENCE_SET:
#if defined HCI_CONTROL
//        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
//            mesh_sensor_hci_event_send_cadence_set(p_hci_event, (wiced_bt_mesh_sensor_cadence_set_data_t *)p_data);
#endif
        mesh_sensor_server_process_cadence_changed(element_idx, property_id);
        break;

    case WICED_BT_MESH_SENSOR_SETTING_SET:
#if defined HCI_CONTROL
//        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
//            mesh_sensor_hci_event_send_setting_set(p_hci_event, (wiced_bt_mesh_sensor_setting_set_data_t *)p_data);
#endif
        mesh_sensor_server_process_setting_changed(element_idx, property_id, setting_property_id);
        break;
    }
}

/*
 * Process get request from Sensor Client and respond with sensor data
 */
void mesh_sensor_server_report_handler(uint16_t event, uint8_t element_idx, void *p_get, void *p_ref_data)
{
    wiced_bt_mesh_sensor_get_t *p_sensor_get = (wiced_bt_mesh_sensor_get_t *)p_get;
    WICED_BT_TRACE("mesh_sensor_server_report_handler msg: %d\n", event);

    switch (event)
    {
    case WICED_BT_MESH_SENSOR_GET:
        // tell mesh models library that data is ready to be shipped out, the library will get data from mesh_config
        wiced_bt_mesh_model_sensor_server_data(element_idx, p_sensor_get->property_id, p_ref_data);
        break;

    case WICED_BT_MESH_SENSOR_COLUMN_GET:
        mesh_sensor_server_send_column_status(wiced_bt_mesh_create_reply_event(p_ref_data), (wiced_bt_mesh_sensor_column_get_data_t *)p_get);
        break;

    case WICED_BT_MESH_SENSOR_SERIES_GET:
        mesh_sensor_server_send_series_status(wiced_bt_mesh_create_reply_event(p_ref_data), (wiced_bt_mesh_sensor_series_get_data_t *)p_get);
        break;

    default:
        WICED_BT_TRACE("unknown\n");
        break;
    }
}

/*
 * Process cadence change
 */
void mesh_sensor_server_process_cadence_changed(uint8_t element_idx, uint16_t property_id)
{
    wiced_bt_mesh_core_config_sensor_t *p_sensor;
    p_sensor = &mesh_config.elements[element_idx].sensors[MESH_TEMPERATURE_SENSOR_INDEX];

    WICED_BT_TRACE("cadence changed property id:%04x\n", property_id);
    WICED_BT_TRACE("Fast cadence period divisor:%d\n", p_sensor->cadence.fast_cadence_period_divisor);
    WICED_BT_TRACE("Is trigger type percent:%d\n", p_sensor->cadence.trigger_type_percentage);
    WICED_BT_TRACE("Trigger delta up:%d\n", p_sensor->cadence.trigger_delta_up);
    WICED_BT_TRACE("Trigger delta down:%d\n", p_sensor->cadence.trigger_delta_down);
    WICED_BT_TRACE("Min Interval:%d\n", p_sensor->cadence.min_interval);
    WICED_BT_TRACE("Fast cadence low:%d\n", p_sensor->cadence.fast_cadence_low);
    WICED_BT_TRACE("Fast cadence high:%d\n", p_sensor->cadence.fast_cadence_high);

    mesh_sensor_server_restart_timer(p_sensor);
}

/*
 * Publication timer callback.  Need to send data if publish period expired, or
 * if value has changed more than specified in the triggers, or if value is in range
 * of fast cadence values.
 */
void mesh_sensor_publish_timer_callback(TIMER_PARAM_TYPE arg)
{
    wiced_bt_mesh_event_t *p_event;
    wiced_bt_mesh_core_config_sensor_t *p_sensor = (wiced_bt_mesh_core_config_sensor_t *)arg;
    wiced_bool_t pub_needed = WICED_FALSE;
    uint32_t cur_time = wiced_bt_mesh_core_get_tick_count();

    if ((cur_time - mesh_sensor_sent_time) < p_sensor->cadence.min_interval)
    {
        WICED_BT_TRACE("time since last pub:%d interval:%d\n", cur_time - mesh_sensor_sent_time, p_sensor->cadence.min_interval);
        wiced_start_timer(&mesh_sensor_cadence_timer, p_sensor->cadence.min_interval - cur_time + mesh_sensor_sent_time);
    }
    else
    {
        // check if publication timer expired
        if ((mesh_sensor_publish_period != 0) && (cur_time - mesh_sensor_sent_time >= mesh_sensor_publish_period))
        {
            WICED_BT_TRACE("Pub needed period\n");
            pub_needed = WICED_TRUE;
        }
        // still need to send if publication timer has not expired, but triggers are configured, and value
        // changed too much
        if (!pub_needed && ((p_sensor->cadence.trigger_delta_up != 0) || (p_sensor->cadence.trigger_delta_down != 0)))
        {
            if (!p_sensor->cadence.trigger_type_percentage)
            {
                WICED_BT_TRACE("Native cur value:%d sent:%d delta:%d/%d\n",
                        mesh_sensor_current_temperature, mesh_sensor_sent_value, p_sensor->cadence.trigger_delta_up, p_sensor->cadence.trigger_delta_down);

                if (((p_sensor->cadence.trigger_delta_up != 0)   && (mesh_sensor_current_temperature >= (mesh_sensor_sent_value + p_sensor->cadence.trigger_delta_up))) ||
                    ((p_sensor->cadence.trigger_delta_down != 0) && (mesh_sensor_current_temperature <= (mesh_sensor_sent_value - p_sensor->cadence.trigger_delta_down))))
                {
                    WICED_BT_TRACE("Pub needed native value\n");
                    pub_needed = WICED_TRUE;
                }
            }
            else
            {
                // need to calculate percentage of the increase or decrease.  The deltas are in 0.01%.
                if ((p_sensor->cadence.trigger_delta_up != 0) && (mesh_sensor_current_temperature > mesh_sensor_sent_value))
                {
                    WICED_BT_TRACE("Delta up:%d\n", ((uint32_t)(mesh_sensor_current_temperature - mesh_sensor_sent_value) * 10000 / mesh_sensor_current_temperature));
                    if (((uint32_t)(mesh_sensor_current_temperature - mesh_sensor_sent_value) * 10000 / mesh_sensor_current_temperature) > p_sensor->cadence.trigger_delta_up)
                    {
                        WICED_BT_TRACE("Pub needed percent delta up:%d\n", ((mesh_sensor_current_temperature - mesh_sensor_sent_value) * 10000 / mesh_sensor_current_temperature));
                        pub_needed = WICED_TRUE;
                    }
                }
                else if ((p_sensor->cadence.trigger_delta_down != 0) && (mesh_sensor_current_temperature < mesh_sensor_sent_value))
                {
                    WICED_BT_TRACE("Delta down:%d\n", ((uint32_t)(mesh_sensor_sent_value - mesh_sensor_current_temperature) * 10000 / mesh_sensor_current_temperature));
                    if (((uint32_t)(mesh_sensor_sent_value - mesh_sensor_current_temperature) * 10000 / mesh_sensor_current_temperature) > p_sensor->cadence.trigger_delta_down)
                    {
                        WICED_BT_TRACE("Pub needed percent delta down:%d\n", ((mesh_sensor_sent_value - mesh_sensor_current_temperature) * 10000 / mesh_sensor_current_temperature));
                        pub_needed = WICED_TRUE;
                    }
                }
            }
        }
        // may still need to send if fast publication is configured
        if (!pub_needed && (mesh_sensor_fast_publish_period != 0))
        {
            // check if fast publish period expired
            if (cur_time - mesh_sensor_sent_time >= mesh_sensor_fast_publish_period)
            {
                // if cadence high is more than cadence low, to publish, the value should be in range
                if (p_sensor->cadence.fast_cadence_high >= p_sensor->cadence.fast_cadence_low)
                {
                    if ((mesh_sensor_current_temperature >= p_sensor->cadence.fast_cadence_low) &&
                        (mesh_sensor_current_temperature <= p_sensor->cadence.fast_cadence_high))
                    {
                        WICED_BT_TRACE("Pub needed in range\n");
                        pub_needed = WICED_TRUE;
                    }
                }
                else if (p_sensor->cadence.fast_cadence_high < p_sensor->cadence.fast_cadence_low)
                {
                    if ((mesh_sensor_current_temperature > p_sensor->cadence.fast_cadence_low) ||
                        (mesh_sensor_current_temperature < p_sensor->cadence.fast_cadence_high))
                    {
                        WICED_BT_TRACE("Pub needed out of range\n");
                        pub_needed = WICED_TRUE;
                    }
                }
            }
        }
        /*
        if (!pub_needed)
        {
           if (((p_sensor->cadence.trigger_delta_up == 0) && (mesh_sensor_current_temperature > mesh_sensor_sent_value)) ||
               ((p_sensor->cadence.trigger_delta_down == 0) && (mesh_sensor_current_temperature < mesh_sensor_sent_value)))
            {
               WICED_BT_TRACE("Pub needed new value no deltas\n");
               pub_needed = WICED_TRUE;
            }
        }
        */
        if (pub_needed)
        {
            mesh_sensor_sent_value  = mesh_sensor_current_temperature;
            mesh_sensor_sent_time   = cur_time;

            WICED_BT_TRACE("Pub value:%d time:%d\n", mesh_sensor_sent_value, mesh_sensor_sent_time);
            wiced_bt_mesh_model_sensor_server_data(MESH_SENSOR_SERVER_ELEMENT_INDEX, WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_TEMPERATURE, NULL);
        }
        mesh_sensor_server_restart_timer(p_sensor);
    }
}


/*
 * Send Sensor Series Status message to the Sensor Client
 */
void mesh_sensor_server_send_series_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_sensor_series_get_data_t* data)
{
    uint8_t i,j,k=0;
    uint8_t element_idx = p_event->element_idx;
    wiced_bt_mesh_sensor_series_status_data_t series_data;
    uint8_t end_idx;
    wiced_bool_t copy_flag = WICED_FALSE;

    series_data.property_id = data->property_id;

    for (i = 0; i < mesh_config.elements[element_idx].sensors_num; i++)
    {
        if (mesh_config.elements[element_idx].sensors[i].property_id == data->property_id)
        {
            series_data.prop_value_len = mesh_config.elements[element_idx].sensors[i].prop_value_len;

            end_idx = (data->end_index > mesh_config.elements[element_idx].sensors[i].num_series) ?
                            mesh_config.elements[element_idx].sensors[i].num_series : data->end_index;
            for (j = data->start_index; j < end_idx; j++, k++)
            {
                memcpy(series_data.column_list[k].raw_valuex, mesh_config.elements[element_idx].sensors[i].series_columns[j].raw_valuex,  series_data.prop_value_len);
                memcpy(series_data.column_list[k].raw_valuey, mesh_config.elements[element_idx].sensors[i].series_columns[j].raw_valuey,  series_data.prop_value_len);
                memcpy(series_data.column_list[k].column_width, mesh_config.elements[element_idx].sensors[i].series_columns[j].column_width,  series_data.prop_value_len);
            }
            series_data.no_of_columns = k;
            wiced_bt_mesh_model_sensor_server_series_status_send(p_event, &series_data);
            return;
        }
    }
}

/*
 * Send Sensor Column Status message to the Sensor Client
 */
void mesh_sensor_server_send_column_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_sensor_column_get_data_t *p_get_column)
{
    wiced_bt_mesh_sensor_column_status_data_t column_status;
    uint8_t element_idx = p_event->element_idx;
    uint8_t num_sensor = mesh_config.elements[element_idx].sensors_num;
    uint16_t property_id;
    uint16_t prop_val_len;
    uint8_t i;
    uint8_t j;

    WICED_BT_TRACE("%s\n", __FUNCTION__);

    for (i = 0; i < num_sensor; i++)
    {
        if (mesh_config.elements[element_idx].sensors[i].property_id == p_get_column->property_id)
        {
            property_id  = mesh_config.elements[element_idx].sensors[i].property_id;
            prop_val_len = mesh_config.elements[element_idx].sensors[i].prop_value_len;
            for (j = 0; j < mesh_config.elements[element_idx].sensors[i].num_series; j++)
            {
                if (memcmp(p_get_column->raw_valuex, mesh_config.elements[element_idx].sensors[i].series_columns[j].raw_valuex, prop_val_len) == 0)
                {
                    column_status.property_id = property_id;
                    column_status.prop_value_len = prop_val_len;
                    column_status.is_column_present = WICED_TRUE;
                    memcpy(column_status.column_data.raw_valuex, mesh_config.elements[element_idx].sensors[i].series_columns[j].raw_valuex, prop_val_len);
                    memcpy(column_status.column_data.raw_valuey, mesh_config.elements[element_idx].sensors[i].series_columns[j].raw_valuey, prop_val_len);
                    memcpy(column_status.column_data.column_width, mesh_config.elements[element_idx].sensors[i].series_columns[j].column_width, prop_val_len);
                }
            }
            wiced_bt_mesh_model_sensor_server_column_status_send(p_event, &column_status);
        }
    }
}


/*
 * Process setting change.  Library already copied the new value to the mesh_config.  Add additional processing here if needed.
 */
void mesh_sensor_server_process_setting_changed(uint8_t element_idx, uint16_t property_id, uint16_t setting_property_id)
{
    WICED_BT_TRACE("setting changed, prop_id:%x, setting prop_id:%x\n", property_id, setting_property_id);
}


/*
 * Send Sensor Status event
 */
void mesh_sensor_server_status_changed(uint8_t element_idx, uint8_t *p_data, uint32_t length)
{
    uint16_t property_id;
    uint16_t prop_value_len;
    uint32_t cur_time = wiced_bt_mesh_core_get_tick_count();
    wiced_bt_mesh_core_config_sensor_t *p_sensor;

    STREAM_TO_UINT16(property_id, p_data);
    STREAM_TO_UINT16(prop_value_len, p_data);

    if ((length >= 5) && (element_idx == 0) && (property_id == MESH_TEMP_SENSOR_PROPERTY_ID) && (prop_value_len == 1))
    {
        mesh_sensor_current_temperature = p_data[0];
        WICED_BT_TRACE("new temp:%d\n", mesh_sensor_current_temperature);

        p_sensor = &mesh_config.elements[element_idx].sensors[MESH_TEMPERATURE_SENSOR_INDEX];

        // Cannot send pubs more often than cadence.min_interval
        if ((cur_time - mesh_sensor_sent_time) < p_sensor->cadence.min_interval)
        {
            WICED_BT_TRACE("Not enough time since last pub\n");

            // if timer is running, the value will be sent, when needed, otherwise, start the time.
            wiced_start_timer(&mesh_sensor_cadence_timer, p_sensor->cadence.min_interval + mesh_sensor_sent_time - cur_time);
        }
        else
        {
            // the timer callback function sends value change notification if it is appropriate
            mesh_sensor_publish_timer_callback((TIMER_PARAM_TYPE)p_sensor);
        }
    }
    else
    {
        WICED_BT_TRACE("sensor server bad params idx:%d prop:%04x len:%d\n", element_idx, property_id, prop_value_len);
    }
}

/*
 * In 2 chip solutions MCU can send commands to change user_property state.
 */
uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
#ifdef HCI_CONTROL
    uint8_t element_idx;
    element_idx = wiced_bt_mesh_get_element_idx_from_wiced_hci(&p_data, &length);
    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_SENSOR_SET:
        mesh_sensor_server_status_changed(element_idx, p_data, length);
        break;
    }
#endif
    return WICED_TRUE;
}


#ifdef HCI_CONTROL
/*
 * Send Sensor Cadence Set event over transport
 */
void mesh_sensor_hci_event_send_cadence_set(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_cadence_set_data_t *p_set)
{
    uint8_t *p = p_hci_event->data;
    uint8_t flag_trigger_type;

    WICED_BT_TRACE("mesh_sensor_hci_event_send_cadence_set:\n");

    UINT16_TO_STREAM(p, p_set->property_id);
    UINT8_TO_STREAM(p, p_set->prop_value_len);
    UINT16_TO_STREAM(p, p_set->cadence_data.fast_cadence_period_divisor);
    UINT8_TO_STREAM(p, p_set->cadence_data.trigger_type ? 0x01 : 0x00);
    UINT32_TO_STREAM(p, p_set->cadence_data.trigger_delta_down);
    UINT32_TO_STREAM(p, p_set->cadence_data.trigger_delta_up);
    UINT32_TO_STREAM(p, p_set->cadence_data.min_interval);
    UINT32_TO_STREAM(p, p_set->cadence_data.fast_cadence_low);
    UINT32_TO_STREAM(p, p_set->cadence_data.fast_cadence_high);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_SENSOR_CADENCE_SET, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Sensor Setting Set event over transport
 */
void mesh_sensor_hci_event_send_setting_set(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_setting_set_data_t *p_set)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("mesh_sensor_hci_event_send_setting_get:\n");

    UINT16_TO_STREAM(p, p_set->property_id);
    UINT16_TO_STREAM(p, p_set->setting_property_id);
    UINT8_TO_STREAM(p, p_set->prop_value_len);
    ARRAY_TO_STREAM(p, p_set->setting_raw_val, p_set->prop_value_len);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_SENSOR_SETTING_SET, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

#endif
