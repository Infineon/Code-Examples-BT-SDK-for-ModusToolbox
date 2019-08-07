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
 * This demo application shows a OnOff switch implementation using WICED EVK.
 * The app is based on the snip/mesh/mesh_onoff_client which implements
 * BLE Mesh Generic OnOff Client model.
 * Normally a switch has 2 buttons to turn the light (or any other device) on and off.
 * This application performs on and off functionality using a single button
 * available on the EVK.  On a first button push the On command is sent, on
 * consecutive the Off command is sent.
 *
 * By default application does not support Relay, Proxy or Friend features
 * The application can be compiled to support a Low Power Node feature by adding
 * #define LOW_POWER_NODE 1
 *
 * Features demonstrated
 *  - Button usage on the EVK
 *  - Controlling of a BLE light bulb using BLE Mesh On/Off messages
 *
 * To demonstrate the app, work through the following steps.
 * 1. Build and download the application (to the WICED board)
 * 2. Build and download a light application (to another WICED board)
 *    (for example apps/demo/light/light_dimmable project)
 * 3. Use Mesh Client or Client Control to provision a light bulb and an on_off_switch
 * 4. Configure on_off_switch to control the light bulb by configuring publication.
 *    (note that if the bulb and the on_off_switch were provisioned in the same group,
 *    the on_off_switch will be automatically configured to send messages to the group
 *    and this step can be skipped.
 * 5. Push/release the application button on the on_off_switch.  The LED on the light
 *    side should turn on.
 * 6. Push/release the application button on the on_off_switch.  The LED on the light
 *    side should turn off.
 */
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_hal_gpio.h"
#include "wiced_platform.h"

#ifdef HCI_CONTROL
#include "wiced_transport.h"
#include "hci_control_api.h"
#endif

#include "wiced_bt_cfg.h"
extern wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

/******************************************************
 *          Constants
 ******************************************************/
#define MESH_PID                0x3025
#define MESH_VID                0x0002
#define MESH_FWID               0x3025000101010001
#define MESH_CACHE_REPLAY_SIZE  0x0008

extern wiced_platform_button_config_t platform_button[];
/******************************************************
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
static void mesh_app_init(wiced_bool_t is_provisioned);
static void mesh_app_hardware_init(void);
static void button_interrupt_handler(void* user_data, uint8_t pin);
static void process_button_push(uint8_t element_idx);

/******************************************************
 *          Variables Definitions
 ******************************************************/

uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME] = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER]     = { '1', '2', '3', '4', 0, 0, 0, 0 };
uint8_t mesh_system_id[8]                                                  = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

wiced_bt_mesh_core_config_model_t   mesh_element1_models[] =
{
    WICED_BT_MESH_DEVICE,
    WICED_BT_MESH_MODEL_ONOFF_CLIENT,
};
#define MESH_APP_NUM_MODELS  (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t))

#define ONOFF_SWITCH_ELEMENT_INDEX   0

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
        .receive_delay         = 100,                               // Receive delay in 1ms units to be requested by the Low Power node.
        .poll_timeout          = 36000                              // Poll timeout in 100ms units to be requested by the Low Power node.
    },
#else
    .features           = 0,                                        // no, support for proxy, friend, or relay
    .friend_cfg         =                                           // Empty Configuration of the Friend Feature
    {
        .receive_window        = 0,                                 // Receive Window value in milliseconds supported by the Friend node.
        .cache_buf_len         = 0,                                 // Length of the buffer for the cache
        .max_lpn_num           = 0                                  // Max number of Low Power Nodes with established friendship. Must be > 0 if Friend feature is supported.
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
    mesh_app_hardware_init, // hardware initialization
    NULL,                   // GATT connection status
    NULL,                   // attention processing
    NULL,                   // notify period set
    NULL,                   // WICED HCI command
    NULL,                   // LPN sleep
    NULL                    // factory reset
};

uint64_t      button_pushed_time = 0;
uint32_t      button_previous_value;

/******************************************************
 *               Function Definitions
 ******************************************************/
void mesh_app_init(wiced_bool_t is_provisioned)
{
#if 0
    extern uint8_t wiced_bt_mesh_model_trace_enabled;
    wiced_bt_mesh_model_trace_enabled = WICED_TRUE;
#endif
    wiced_bt_cfg_settings.device_name = (uint8_t *)"Switch";
    wiced_bt_cfg_settings.gatt_cfg.appearance = APPEARANCE_CONTROL_DEVICE_SLIDER;

#if defined(LOW_POWER_NODE) && (LOW_POWER_NODE == 1)
    WICED_BT_TRACE("LPN Switch init provisioned:%d\n", is_provisioned);
#else
    WICED_BT_TRACE("Switch init provisioned:%d\n", is_provisioned);
#endif

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
    }

    // This application does not check result of the transmission or status event from the
    // target device.  Initialize OnOff client library not registering the callback.
    wiced_bt_mesh_model_onoff_client_init(ONOFF_SWITCH_ELEMENT_INDEX, NULL, is_provisioned);
}

void mesh_app_hardware_init(void)
{
    /* Configure buttons available on the platform */
#if defined(CYW20706A2)
    wiced_hal_gpio_configure_pin(WICED_GPIO_BUTTON, WICED_GPIO_BUTTON_SETTINGS(GPIO_EN_INT_BOTH_EDGE), WICED_GPIO_BUTTON_DEFAULT_STATE);
    wiced_hal_gpio_register_pin_for_interrupt(WICED_GPIO_BUTTON, button_interrupt_handler, NULL);
#elif (defined(CYW20735B0) || defined(CYW20719B0) || defined(CYW20721B0))
    wiced_hal_gpio_register_pin_for_interrupt(WICED_GPIO_PIN_BUTTON, button_interrupt_handler, NULL);
    wiced_hal_gpio_configure_pin(WICED_GPIO_PIN_BUTTON, WICED_GPIO_BUTTON_SETTINGS, GPIO_PIN_OUTPUT_LOW);
#else
    wiced_platform_register_button_callback(WICED_PLATFORM_BUTTON_1, button_interrupt_handler, NULL, GPIO_EN_INT_BOTH_EDGE);
    button_previous_value = platform_button[WICED_PLATFORM_BUTTON_1].default_state;
#endif
}

/*
 * Process interrupts from the button.
 */
void button_interrupt_handler(void* user_data, uint8_t pin)
{
    uint32_t value = wiced_hal_gpio_get_pin_input_status(pin);
    uint32_t current_time = wiced_bt_mesh_core_get_tick_count();
    uint32_t button_pushed_duration;

    if (value == button_previous_value)
    {
        WICED_BT_TRACE("interrupt_handler: duplicate pin:%d value:%d current_time:%d\n", pin, value, current_time);
        return;
    }
    button_previous_value = value;

    WICED_BT_TRACE("interrupt_handler: pin:%d value:%d current_time:%d\n", pin, value, current_time);

    if (value == platform_button[WICED_PLATFORM_BUTTON_1].button_pressed_value)
    {
        button_pushed_time = current_time;
        return;
    }
    // button is released
    button_pushed_duration = current_time - button_pushed_time;
    if (button_pushed_duration < 15000)
    {
        process_button_push(ONOFF_SWITCH_ELEMENT_INDEX);
    }
    else
    {
        // More than 15 seconds means factory reset
        mesh_application_factory_reset();
    }
}

void process_button_push(uint8_t element_idx)
{
    static uint8_t onoff = 0;
    wiced_bt_mesh_onoff_set_data_t set_data;

    onoff ^= 1;

    set_data.onoff           = onoff;
    set_data.transition_time = WICED_BT_MESH_TRANSITION_TIME_DEFAULT;
    set_data.delay           = 0;

    wiced_bt_mesh_model_onoff_client_set(element_idx, &set_data);
}
