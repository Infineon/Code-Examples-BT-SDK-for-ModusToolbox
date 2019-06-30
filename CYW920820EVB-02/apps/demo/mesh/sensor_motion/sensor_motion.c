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
 * This demo application shows an implementation of a motion sensor.
 * The app is based on the snip/mesh/mesh_sensor_server sample which
 * implements generic BLE Mesh Sensor Server model.
 *
 * Features demonstrated
 * - Configuring and receiving interrupts from the PIR motion sensor
 * - Publishing motion data over BLE mesh
 *
 * See chip specific readme.txt for more information about the BT SDK.
 *
 * To demonstrate the app, work through the following steps.
 * 1. Build and download the application to 213043 mesh kit.
 * 2. Use Android MeshController or Windows Mesh Client and provision the motion sensor
 * 3. After successful provisioning, user can use the Android MeshController/ Windows Mesh
 *    Client to configure the below parameters of the sensor:
 *         When sensor is provisioned, it is configured to publish data to a group, if current
 *         group was configured, or to "all-nodes", if there were no group. Modify publication
 *         to configuration to publish to "all-nodes".  Also set up the sensor to publish data
 *         with period 320000msec(5 minutes). The default configuration of the sensor is to
 *         publish data with publish period when presence is not detected. When presence is
 *         detected the period is divided by 32, i.e. presence detected message is sent every
 *         10 seconds. In addition to that the message will be sent as soon as if there were no
 *         presence for more than 10 seconds.
 * 4. Wave your hand in front of the CYBT-213043-MESH board to show some motion.
 */
#include "wiced_bt_uuid.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_timer.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_hal_nvram.h"
#include "wiced_platform.h"
#include "e93196.h"
#include "wiced_sleep.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_mia.h"
extern wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

/******************************************************
 *          Constants
 ******************************************************/
#define MESH_PID                0x3123
#define MESH_VID                0x0002
#define MESH_FWID               0x3123000101010001

#define MESH_CACHE_REPLAY_SIZE  0x0008

#define MESH_SENSOR_PROPERTY_ID                         WICED_BT_MESH_PROPERTY_PRESENCE_DETECTED
#define MESH_SENSOR_VALUE_LEN                           WICED_BT_MESH_PROPERTY_LEN_PRESENCE_DETECTED

#define MESH_MOTION_SENSOR_POSITIVE_TOLERANCE           WICED_BT_MESH_SENSOR_TOLERANCE_UNSPECIFIED
#define MESH_MOTION_SENSOR_NEGATIVE_TOLERANCE           WICED_BT_MESH_SENSOR_TOLERANCE_UNSPECIFIED

#define MESH_MOTION_SENSOR_SAMPLING_FUNCTION            WICED_BT_MESH_SENSOR_SAMPLING_FUNCTION_UNKNOWN
#define MESH_MOTION_SENSOR_MEASUREMENT_PERIOD           WICED_BT_MESH_SENSOR_VAL_UNKNOWN
#define MESH_MOTION_SENSOR_UPDATE_INTERVAL              WICED_BT_MESH_SENSOR_VAL_UNKNOWN

#define MESH_MOTION_SENSOR_CADENCE_VSID_START           WICED_NVRAM_VSID_START

// After presence is detected, interrupts are disabled for 5 seconds
#define MESH_PRESENCE_DETECTED_BLIND_TIME               5

/******************************************************
 *          Structures
 ******************************************************/
e93196_usr_cfg_t e93196_usr_cfg =
{
    .doci_pin           = WICED_P05,                                /* Interrupt/Data output Clock input configure pin          */
    .serin_pin          = WICED_P17,                                /* Serial Input configure pin                               */
    .e93196_init_reg    =
    {
        .sensitivity    = 0x10,                                     /* [24:17]sensitivity,   [Register Value] * 6.5uV           */
        .blind_time     = MESH_PRESENCE_DETECTED_BLIND_TIME * 2,    /* [16:13]blind time,    [Register Value] * 0.5s, max is 8s */
        .pulse_cnt      = 0x01,                                     /* [12:11]pulse count                                       */
        .window_time    = 0x01,                                     /* [10:9]window time                                        */
        .move_dete_en   = 0x01,                                     /* [8]move detect enable                                    */
        .int_src        = 0x00,                                     /* [7]irq source                                            */
        .adc_filter     = 0x01,                                     /* [6:5]ADC filter                                          */
        .power_en       = 0x00,                                     /* [4]power enable                                          */
        .self_test_en   = 0x00,                                     /* [3]selftest                                              */
        .capa           = 0x00,                                     /* [2]selftest capacity                                     */
        .test_mode      = 0x00,                                     /* [1:0]reserved                                            */
    }
};

/******************************************************
 *          Function Prototypes
 ******************************************************/
static void         mesh_app_init(wiced_bool_t is_provisioned);
static wiced_bool_t mesh_app_notify_period_set(uint8_t element_idx, uint16_t company_id, uint16_t model_id, uint32_t period);

static void         mesh_sensor_server_restart_timer(wiced_bt_mesh_core_config_sensor_t *p_sensor);
static void         mesh_sensor_server_report_handler(uint16_t event, uint8_t element_idx, void *p_get_data, void *p_ref_data);
static void         mesh_sensor_server_config_change_handler(uint8_t element_idx, uint16_t event, uint16_t property_id, uint16_t setting_property_id);
static void         mesh_sensor_server_process_cadence_changed(uint8_t element_idx, uint16_t property_id);
static void         mesh_sensor_server_process_setting_changed(uint8_t element_idx, uint16_t property_id, uint16_t setting_property_id);
static void         mesh_sensor_publish_timer_callback(TIMER_PARAM_TYPE arg);
static void         e93196_int_proc(void *data, uint8_t port_pin);
static void         mesh_sensor_presence_detected_timer_callback(TIMER_PARAM_TYPE arg);
static int32_t      mesh_sensor_get_current_value(void);
static void         mesh_app_factory_reset(void);

#ifdef HCI_CONTROL
static void         mesh_sensor_hci_event_send_cadence_set(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_cadence_set_data_t *p_set);
static void         mesh_sensor_hci_event_send_setting_set(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_setting_set_data_t *p_set);
#endif

#if defined(LOW_POWER_NODE) && (LOW_POWER_NODE == 1)
void mesh_sensor_motion_lpn_sleep(uint32_t max_sleep_duration);
#endif
/******************************************************
 *          Variables Definitions
 ******************************************************/
uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME]          = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER]              = { '1', '2', '3', '4', 0, 0, 0, 0 };
uint8_t mesh_prop_fw_version[WICED_BT_MESH_PROPERTY_LEN_DEVICE_FIRMWARE_REVISION] =   { '0', '6', '.', '0', '2', '.', '0', '5' }; // this is overwritten during init
uint8_t mesh_system_id[8]                                                           = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

int32_t       mesh_sensor_sent_value = 0;          //
int32_t       mesh_sensor_current_value = 0;
uint32_t      mesh_sensor_sent_time;               // time stamp when data was published
uint32_t      mesh_sensor_publish_period = 320000; // 0;  // publish "no presence" every ~5 minutes, with fast cadence 32. This is reset to 0 after provisioning.  Set here for testing.
                                                   // we will publish "presence" every 10 seconds.
uint32_t      mesh_sensor_fast_publish_period = 0; // publish period in msec when values are outside of limit
wiced_timer_t mesh_sensor_cadence_timer;
wiced_timer_t mesh_sensor_presence_detected_timer;
wiced_bool_t  presence_detected = WICED_FALSE;

// We define optional setting for the motion sensor, the Motion Threshold. Default is 80%.
uint8_t       mesh_motion_sensor_threshold_val = 0x50;

wiced_bt_mesh_core_config_model_t mesh_element1_models[] =
{
    WICED_BT_MESH_DEVICE,
    WICED_BT_MESH_MODEL_SENSOR_SERVER,
};
#define MESH_APP_NUM_MODELS  (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t))

wiced_bt_mesh_core_config_sensor_t mesh_element1_sensors[] =
{
    {
        .property_id    = MESH_SENSOR_PROPERTY_ID,
        .prop_value_len = MESH_SENSOR_VALUE_LEN,
        .descriptor =
        {
            .positive_tolerance = MESH_MOTION_SENSOR_POSITIVE_TOLERANCE,
            .negative_tolerance = MESH_MOTION_SENSOR_NEGATIVE_TOLERANCE,
            .sampling_function  = MESH_MOTION_SENSOR_SAMPLING_FUNCTION,
            .measurement_period = MESH_MOTION_SENSOR_MEASUREMENT_PERIOD,
            .update_interval    = MESH_MOTION_SENSOR_UPDATE_INTERVAL,
        },
        .data = (uint8_t*)&mesh_sensor_sent_value,
        .cadence =
        {
            // Value 0 indicates that cadence does not change depending on the measurements
            .fast_cadence_period_divisor = 32,          // Recommended publish period is 320sec, 32 will make fast period 10sec
            .trigger_type_percentage     = WICED_FALSE, // The Property is Bool, does not make sense to use percentage
            .trigger_delta_down          = 0,           // This will not cause message when presence changes from 1 to 0
            .trigger_delta_up            = 1,           // This will cause immediate message when presence changes from 0 to 1
            .min_interval                = (1 << 10),   // Milliseconds. Conversion to SPEC values is done by the mesh models library
            .fast_cadence_low            = 1,           // If fast_cadence_low is greater than fast_cadence_high and the measured value is either is lower
                                                        // than fast_cadence_high or higher than fast_cadence_low, then the message shall be published
                                                        // with publish period (equals to mesh_sensor_publish_period divided by fast_cadence_divisor_period)
            .fast_cadence_high           = 0,           // is more or equal cadence_low or less then cadence_high. This is what we need.
        },
        .num_series     = 0,
        .series_columns = NULL,
        .num_settings   = 0,
        .settings       = NULL,
    },
};


#define MESH_APP_NUM_PROPERTIES (sizeof(mesh_element1_properties) / sizeof(wiced_bt_mesh_core_config_property_t))

#define MESH_SENSOR_SERVER_ELEMENT_INDEX    0
#define MESH_MOTION_SENSOR_INDEX            0

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
        .poll_timeout          = 600                                // Poll timeout in 100ms units to be requested by the Low Power node.
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
    mesh_app_init,                  // application initialization
    NULL,                           // Default SDK platform button processing
    NULL,                           // GATT connection status
    NULL,                           // attention processing
    mesh_app_notify_period_set,     // notify period set
    NULL,                           // WICED HCI command
#if defined(LOW_POWER_NODE) && (LOW_POWER_NODE == 1)
    mesh_sensor_motion_lpn_sleep,   // LPN sleep
#else
    NULL,
#endif
    mesh_app_factory_reset          // factory reset
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
    wiced_result_t result;
    wiced_bt_mesh_core_config_sensor_t *p_sensor;

    /* This means that device came out of HID off mode and it is not a power cycle */
    if(wiced_hal_mia_is_reset_reason_por())
    {
        WICED_BT_TRACE("start reason: reset\n");
    }
    else
    {
#if CYW20819A1
        if(wiced_hal_mia_is_reset_reason_hid_timeout())
        {
            WICED_BT_TRACE("Wake from HID off: timed wake\n");
        }
        else
#endif
        {
            /* We woke up from hid-off, need to check if we woke because of GPIO */
            WICED_BT_TRACE("Wake from HID off, interrupt:%d\n",
                wiced_hal_gpio_get_pin_interrupt_status(e93196_usr_cfg.doci_pin));
        }
    }

#if defined(LOW_POWER_NODE) && (LOW_POWER_NODE == 1)
    wiced_bt_cfg_settings.device_name = (uint8_t *)"Motion Sensor LPN";
#else
    wiced_bt_cfg_settings.device_name = (uint8_t *)"Motion Sensor";
#endif

    wiced_bt_cfg_settings.gatt_cfg.appearance = APPEARANCE_SENSOR_MOTION;

    mesh_prop_fw_version[0] = 0x30 + (WICED_SDK_MAJOR_VER / 10);
    mesh_prop_fw_version[1] = 0x30 + (WICED_SDK_MAJOR_VER % 10);
    mesh_prop_fw_version[2] = 0x30 + (WICED_SDK_MINOR_VER / 10);
    mesh_prop_fw_version[3] = 0x30 + (WICED_SDK_MINOR_VER % 10);
    mesh_prop_fw_version[4] = 0x30 + (WICED_SDK_REV_NUMBER / 10);
    mesh_prop_fw_version[5] = 0x30 + (WICED_SDK_REV_NUMBER % 10);
    mesh_prop_fw_version[6] = 0x30 + (WICED_SDK_BUILD_NUMBER / 10);
    mesh_prop_fw_version[7] = 0x30 + (WICED_SDK_BUILD_NUMBER % 10);

    // Adv Data is fixed. Spec allows to put URI, Name, Appearance and Tx Power in the Scan Response Data.
    if (!is_provisioned)
    {
        wiced_bt_ble_advert_elem_t  adv_elem[3];
        uint8_t                     buf[2];
        uint8_t                     num_elem = 0;

        adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
        adv_elem[num_elem].len         = (uint16_t)strlen((const char*)wiced_bt_cfg_settings.device_name);
        adv_elem[num_elem].p_data      = wiced_bt_cfg_settings.device_name;
        num_elem++;

        adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_APPEARANCE;
        adv_elem[num_elem].len         = 2;
        buf[0]                         = (uint8_t)wiced_bt_cfg_settings.gatt_cfg.appearance;
        buf[1]                         = (uint8_t)(wiced_bt_cfg_settings.gatt_cfg.appearance >> 8);
        adv_elem[num_elem].p_data      = buf;
        num_elem++;

        wiced_bt_mesh_set_raw_scan_response_data(num_elem, adv_elem);

        wiced_bt_mesh_model_sensor_server_init(MESH_SENSOR_SERVER_ELEMENT_INDEX, mesh_sensor_server_report_handler, mesh_sensor_server_config_change_handler, is_provisioned);
        return;
    }

    p_sensor = &mesh_config.elements[MESH_SENSOR_SERVER_ELEMENT_INDEX].sensors[MESH_MOTION_SENSOR_INDEX];

    e93196_init(&e93196_usr_cfg, e93196_int_proc, NULL);

    // initialize the cadence timer.  Need a timer for each element because each sensor model can be
    // configured for different publication period.  This app has only one sensor.
    wiced_init_timer(&mesh_sensor_cadence_timer, &mesh_sensor_publish_timer_callback, (TIMER_PARAM_TYPE)&mesh_config.elements[MESH_SENSOR_SERVER_ELEMENT_INDEX].sensors[MESH_MOTION_SENSOR_INDEX], WICED_MILLI_SECONDS_TIMER);

    wiced_init_timer(&mesh_sensor_presence_detected_timer, mesh_sensor_presence_detected_timer_callback, (TIMER_PARAM_TYPE)&mesh_config.elements[MESH_SENSOR_SERVER_ELEMENT_INDEX].sensors[MESH_MOTION_SENSOR_INDEX], WICED_SECONDS_TIMER);

    //restore the cadence from NVRAM
    wiced_hal_read_nvram( MESH_MOTION_SENSOR_CADENCE_VSID_START, sizeof(wiced_bt_mesh_sensor_config_cadence_t), (uint8_t*)(&p_sensor->cadence), &result);

    wiced_bt_mesh_model_sensor_server_init(MESH_SENSOR_SERVER_ELEMENT_INDEX, mesh_sensor_server_report_handler, mesh_sensor_server_config_change_handler, is_provisioned);
}

/*
 * New publication period is set. If it is for the sensor model, this application should take care of it.
 * The period may need to be adjusted based on the divisor.
 */
wiced_bool_t mesh_app_notify_period_set(uint8_t element_idx, uint16_t company_id, uint16_t model_id, uint32_t period)
{
    if ((element_idx != MESH_MOTION_SENSOR_INDEX) || (company_id != MESH_COMPANY_ID_BT_SIG) || (model_id != WICED_BT_MESH_CORE_MODEL_ID_SENSOR_SRV))
    {
        return WICED_FALSE;
    }
    mesh_sensor_publish_period = period;
    WICED_BT_TRACE("Sensor data send period:%dms\n", mesh_sensor_publish_period);
    mesh_sensor_server_restart_timer(&mesh_config.elements[element_idx].sensors[MESH_MOTION_SENSOR_INDEX]);
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
        WICED_BT_TRACE("sensor restart timer period:%d\n", mesh_sensor_publish_period);
        return;
    }
    // If fast cadence period divisor is set, we need to check data more
    // often than publication period.  Publish if measurement is in specified range
    if (p_sensor->cadence.fast_cadence_period_divisor > 1)
    {
        mesh_sensor_fast_publish_period = mesh_sensor_publish_period / p_sensor->cadence.fast_cadence_period_divisor;
        timeout = mesh_sensor_fast_publish_period;
        WICED_BT_TRACE("sensor fast cadence:%d\n", mesh_sensor_fast_publish_period);
    }
    else
    {
        mesh_sensor_fast_publish_period = 0;
    }
    // should not send data more often than min_interval
    if ((p_sensor->cadence.min_interval != 0) && (p_sensor->cadence.min_interval > timeout) &&
        ((p_sensor->cadence.trigger_delta_up != 0) || (p_sensor->cadence.trigger_delta_down != 0)))
    {
        timeout = p_sensor->cadence.min_interval;
        WICED_BT_TRACE("sensor min interval:%d\n", timeout);
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
        mesh_sensor_sent_value = presence_detected;
        wiced_bt_mesh_model_sensor_server_data(element_idx, p_sensor_get->property_id, p_ref_data);
        break;

    case WICED_BT_MESH_SENSOR_COLUMN_GET:
        break;

    case WICED_BT_MESH_SENSOR_SERIES_GET:
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
    uint8_t written_byte = 0;
    wiced_result_t status;
    p_sensor = &mesh_config.elements[element_idx].sensors[MESH_MOTION_SENSOR_INDEX];

    WICED_BT_TRACE("cadence changed property id:%04x\n", property_id);
    WICED_BT_TRACE("Fast cadence period divisor:%d\n", p_sensor->cadence.fast_cadence_period_divisor);
    WICED_BT_TRACE("Is trigger type percent:%d\n", p_sensor->cadence.trigger_type_percentage);
    WICED_BT_TRACE("Trigger delta up:%d\n", p_sensor->cadence.trigger_delta_up);
    WICED_BT_TRACE("Trigger delta down:%d\n", p_sensor->cadence.trigger_delta_down);
    WICED_BT_TRACE("Min Interval:%d\n", p_sensor->cadence.min_interval);
    WICED_BT_TRACE("Fast cadence low:%d\n", p_sensor->cadence.fast_cadence_low);
    WICED_BT_TRACE("Fast cadence high:%d\n", p_sensor->cadence.fast_cadence_high);

    /* save cadence to NVRAM */
    written_byte = wiced_hal_write_nvram( MESH_MOTION_SENSOR_CADENCE_VSID_START, sizeof(wiced_bt_mesh_sensor_config_cadence_t), (uint8_t*)(&p_sensor->cadence), &status);
    WICED_BT_TRACE("NVRAM write: %d\n", written_byte);

    mesh_sensor_server_restart_timer(p_sensor);
}

/*
 * Publication timer callback.  Need to send data if publish period expired, or
 * if value has changed more than specified in the triggers, or if value is in range
 * of fast cadence values and fast cadence interval expired.
 */
void mesh_sensor_publish_timer_callback(TIMER_PARAM_TYPE arg)
{
    wiced_bt_mesh_event_t *p_event;
    wiced_bt_mesh_core_config_sensor_t *p_sensor = (wiced_bt_mesh_core_config_sensor_t *)arg;
    wiced_bool_t pub_needed = WICED_FALSE;
    uint32_t cur_time = wiced_bt_mesh_core_get_tick_count();

    mesh_sensor_current_value = mesh_sensor_get_current_value();

    if ((p_sensor->cadence.min_interval != 0) && ((cur_time - mesh_sensor_sent_time) < p_sensor->cadence.min_interval))
    {
        WICED_BT_TRACE("time since last pub:%d less then cadence interval:%d\n", cur_time - mesh_sensor_sent_time, p_sensor->cadence.min_interval);
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
                        mesh_sensor_current_value, mesh_sensor_sent_value, p_sensor->cadence.trigger_delta_up, p_sensor->cadence.trigger_delta_down);

                if (((p_sensor->cadence.trigger_delta_up != 0)   && (mesh_sensor_current_value >= (mesh_sensor_sent_value + p_sensor->cadence.trigger_delta_up)))
                 || ((p_sensor->cadence.trigger_delta_down != 0) && (mesh_sensor_current_value <= (mesh_sensor_sent_value - p_sensor->cadence.trigger_delta_down))))
                {
                    WICED_BT_TRACE("Pub needed native value\n");
                    pub_needed = WICED_TRUE;
                }
            }
            else
            {
                // need to calculate percentage of the increase or decrease. The deltas are in 0.01%.
                if (mesh_sensor_current_value > mesh_sensor_sent_value)
                {
                    WICED_BT_TRACE("Delta up:%d\n", ((uint32_t)(mesh_sensor_current_value - mesh_sensor_sent_value) * 10000 / mesh_sensor_current_value));
                    if (((uint32_t)(mesh_sensor_current_value - mesh_sensor_sent_value) * 10000 / mesh_sensor_current_value) > p_sensor->cadence.trigger_delta_up)
                    {
                        WICED_BT_TRACE("Pub needed percent delta up:%d\n", ((mesh_sensor_current_value - mesh_sensor_sent_value) * 10000 / mesh_sensor_current_value));
                        pub_needed = WICED_TRUE;
                    }
                }
                else
                {
                    WICED_BT_TRACE("Delta down:%d\n", ((uint32_t)(mesh_sensor_sent_value - mesh_sensor_current_value) * 10000 / mesh_sensor_current_value));
                    if (((uint32_t)(mesh_sensor_sent_value - mesh_sensor_current_value) * 10000 / mesh_sensor_current_value) > p_sensor->cadence.trigger_delta_down)
                    {
                        WICED_BT_TRACE("Pub needed percent delta down:%d\n", ((mesh_sensor_sent_value - mesh_sensor_current_value) * 10000 / mesh_sensor_current_value));
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
                if (p_sensor->cadence.fast_cadence_high > p_sensor->cadence.fast_cadence_low)
                {
                    if ((mesh_sensor_current_value > p_sensor->cadence.fast_cadence_low) &&
                        (mesh_sensor_current_value <= p_sensor->cadence.fast_cadence_high))
                    {
                        WICED_BT_TRACE("Pub needed in range\n");
                        pub_needed = WICED_TRUE;
                    }
                }
                else if (p_sensor->cadence.fast_cadence_high < p_sensor->cadence.fast_cadence_low)
                {
                    if ((mesh_sensor_current_value >= p_sensor->cadence.fast_cadence_low) ||
                        (mesh_sensor_current_value < p_sensor->cadence.fast_cadence_high))
                    {
                        WICED_BT_TRACE("Pub needed out of range\n");
                        pub_needed = WICED_TRUE;
                    }
                }
            }
        }
        // We will still send publication if Deltas are not set, but measured value has changed.
        if (!pub_needed && (p_sensor->cadence.trigger_delta_up == 0) && (p_sensor->cadence.trigger_delta_down == 0))
        {
            if (((p_sensor->cadence.trigger_delta_up == 0)   && (mesh_sensor_current_value > mesh_sensor_sent_value)) ||
                ((p_sensor->cadence.trigger_delta_down == 0) && (mesh_sensor_current_value < mesh_sensor_sent_value)))
            {
               WICED_BT_TRACE("Pub needed new value no deltas\n");
               pub_needed = WICED_TRUE;
            }
        }
        if (pub_needed)
        {
            mesh_sensor_sent_value  = mesh_sensor_current_value;
            mesh_sensor_sent_time   = cur_time;

            WICED_BT_TRACE("*** Pub value:%d time:%d\n", mesh_sensor_sent_value, mesh_sensor_sent_time);
            wiced_bt_mesh_model_sensor_server_data(MESH_SENSOR_SERVER_ELEMENT_INDEX, MESH_SENSOR_PROPERTY_ID, NULL);
        }
    }
    mesh_sensor_server_restart_timer(p_sensor);
}

/*
 * Process setting change
 */
void mesh_sensor_server_process_setting_changed(uint8_t element_idx, uint16_t property_id, uint16_t setting_property_id)
{
    WICED_BT_TRACE("settings changed sensor prop id:%x, setting prop id:%x\n", property_id, setting_property_id);
}

void e93196_int_proc(void* data, uint8_t port_pin)
{
    WICED_BT_TRACE("presence detected TRUE\n");
    e93196_int_clean(port_pin);

    // We disable interrupts for MESH_PRESENCE_DETECTED_BLIND_TIME.  If interrupt does not happen within
    // MESH_PRESENCE_DETECTED_BLIND_TIME * 2, we assume that there is no presence anymore
    wiced_start_timer(&mesh_sensor_presence_detected_timer, 2 * MESH_PRESENCE_DETECTED_BLIND_TIME);

    if (!presence_detected)
    {
        presence_detected = WICED_TRUE;
        mesh_sensor_publish_timer_callback((TIMER_PARAM_TYPE)&mesh_config.elements[MESH_SENSOR_SERVER_ELEMENT_INDEX].sensors[MESH_MOTION_SENSOR_INDEX]);
    }
}

void mesh_sensor_presence_detected_timer_callback(TIMER_PARAM_TYPE arg)
{
    WICED_BT_TRACE("presence detected timeout\n");
    presence_detected = WICED_FALSE;
    mesh_sensor_publish_timer_callback((TIMER_PARAM_TYPE)&mesh_config.elements[MESH_SENSOR_SERVER_ELEMENT_INDEX].sensors[MESH_MOTION_SENSOR_INDEX]);
}

int32_t mesh_sensor_get_current_value(void)
{
    return presence_detected;
}

/*
 * Application is notified that factory reset is executed.
 */
void mesh_app_factory_reset(void)
{
    wiced_hal_delete_nvram(MESH_MOTION_SENSOR_CADENCE_VSID_START, NULL);
}

#if defined(LOW_POWER_NODE) && (LOW_POWER_NODE == 1)
void mesh_sensor_motion_lpn_sleep(uint32_t max_sleep_duration)
{
    WICED_BT_TRACE("Entering HID-OFF for max_sleep_duration: %ds\r\n", max_sleep_duration/1000);
    wiced_sleep_enter_hid_off(max_sleep_duration, e93196_usr_cfg.doci_pin, WICED_GPIO_ACTIVE_HIGH);
    WICED_BT_TRACE("Entering HID-Off failed\n\r");
}
#endif
