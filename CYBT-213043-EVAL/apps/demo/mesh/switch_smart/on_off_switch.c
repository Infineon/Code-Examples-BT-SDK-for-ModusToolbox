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
 * Application will toggle between On and Off on a button push.  If provisioner
 * sets up a bulb to send status to the switch and this application receives the
 * on/off status, it will send On when the switch is off, and it will send Off when
 * the switch is on.
 */
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_hal_gpio.h"
#include "wiced_platform.h"
#include "wiced_sleep.h"

/******************************************************
 *          Constants
 ******************************************************/
/******************************************************
 *          Structures
 ******************************************************/
/******************************************************
 *          Function Prototypes
 ******************************************************/
static void process_button_push(uint8_t element_idx);
static void mesh_onoff_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void button_interrupt_handler(void* user_data, uint8_t pin);

/******************************************************
 *          Variables Definitions
 ******************************************************/
extern wiced_platform_button_config_t platform_button[];

uint64_t      button_pushed_time = 0;
uint32_t      button_previous_value;
uint8_t       onoff_element_idx = 0;

/******************************************************
 *               Function Definitions
 ******************************************************/
void mesh_switch_init(uint8_t element_idx, wiced_bool_t is_provisioned)
{
    onoff_element_idx = element_idx;

    // Initialize OnOff client library to receive Status and LC client library to send messages.
    wiced_bt_mesh_model_onoff_client_init(element_idx, mesh_onoff_client_message_handler, is_provisioned);
    wiced_bt_mesh_model_light_lc_client_init(element_idx, mesh_onoff_client_message_handler, is_provisioned);
}

void mesh_switch_hardware_init(uint8_t element_idx)
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

    uint32_t button_pressed_value = wiced_platform_get_button_pressed_value(WICED_PLATFORM_BUTTON_1);
    uint32_t value = wiced_hal_gpio_get_pin_input_status(26);

    WICED_BT_TRACE("hw init button default:%d pressed:%d input:%d\n", button_previous_value, button_pressed_value, value);
    if (button_pressed_value == value)
        return;

    wiced_bt_mesh_model_onoff_client_get(element_idx);
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
        process_button_push(onoff_element_idx);
    }
    else
    {
        // More than 15 seconds means factory reset
        mesh_application_factory_reset();
    }
}

static uint8_t onoff_state = 0;

void process_button_push(uint8_t element_idx)
{
    wiced_bt_mesh_light_lc_light_onoff_set_data_t set_data;

    wiced_bt_mesh_event_t *p_event = wiced_bt_mesh_create_event(element_idx, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LC_CLNT, 0, 0);
    if (p_event == NULL)
        return;

    if (WICED_BT_MESH_IS_UNICAST_ADDR(p_event->dst))
    {
        p_event->reply = WICED_TRUE;
    }

    set_data.light_onoff     = (onoff_state == 1) ? 0 : 1;
    set_data.transition_time = WICED_BT_MESH_TRANSITION_TIME_DEFAULT;
    set_data.delay           = 0;

    WICED_BT_TRACE("onoff set target:%d\n", set_data.light_onoff);

    wiced_bt_mesh_model_light_lc_client_send_light_onoff_set(p_event, &set_data);

    // If we do not receive status from the bulb, we will just toggle between on and off.
    // If we receive status, the state will be updated.  For example, if the bulb is the
    // light controller, it may execute state machine and go off on timeout.  Then this
    // application will receive the state Off and will send On again on the button push.
    onoff_state ^= 1;
}

/*
 * Process event received from the onoff Server.
 */
void mesh_onoff_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
    if (event == WICED_BT_MESH_LIGHT_LC_LIGHT_ONOFF_STATUS)
    {
        wiced_bt_mesh_light_lc_light_onoff_status_data_t *p_onoff_status = (wiced_bt_mesh_light_lc_light_onoff_status_data_t *)p_data;
        WICED_BT_TRACE("lc onoff status present:%d target:%d time:%d\n", p_onoff_status->present_onoff, p_onoff_status->target_onoff, p_onoff_status->remaining_time);
        onoff_state = p_onoff_status->target_onoff;
    }
    else if (event == WICED_BT_MESH_ONOFF_STATUS)
    {
        wiced_bt_mesh_onoff_status_data_t *p_onoff_status = (wiced_bt_mesh_onoff_status_data_t *)p_data;
        WICED_BT_TRACE("onoff status present:%d target:%d time:%d\n", p_onoff_status->present_onoff, p_onoff_status->target_onoff, p_onoff_status->remaining_time);
        onoff_state = p_onoff_status->target_onoff;
    }
    else
    {
        WICED_BT_TRACE("message ignored:%d\n", event);

    }
    wiced_bt_mesh_release_event(p_event);
}
