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
 * Button control functionality for a dimmer
 *
 */

#include "sparcommon.h"

#include "wiced_bt_mesh_models.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_aclk.h"
#include "wiced_platform.h"
#include "button_control.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define NUM_STEPS    9
static uint16_t button_level_step[NUM_STEPS] =
{
    0x8000, 0xa000, 0xC000, 0xE000, 0x0000, 0x2000, 0x4000, 0x6000, 0x7FFF,
};

extern wiced_platform_button_config_t platform_button[];
/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/
static void button_interrupt_handler(void* user_data, uint8_t value);
static void button_set_level(wiced_bool_t is_instant, wiced_bool_t is_final);
static void button_timer_callback(uint32_t arg);

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
uint8_t       button_step = 0;
uint64_t      button_pushed_time = 0;
wiced_bool_t  button_direction = WICED_FALSE;
wiced_bool_t  button_level_moving = WICED_FALSE;
uint32_t      transation_start_time = 0;
wiced_timer_t button_timer;
uint32_t      button_previous_value;
/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
void button_control_init(void)
{
    wiced_init_timer(&button_timer, &button_timer_callback, 0, WICED_MILLI_SECONDS_TIMER);
    button_previous_value = platform_button[WICED_PLATFORM_BUTTON_1].default_state;
}

void button_hardware_init(void)
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

        // if button is not released within 500ms, we will start sending move events
        wiced_start_timer(&button_timer, 500);
        return;
    }
    wiced_stop_timer(&button_timer);

    // button is released
    button_pushed_duration = current_time - button_pushed_time;
    if (button_pushed_duration < 500)
    {
        button_level_moving = WICED_FALSE;

        if (button_step == 0)
            button_step = NUM_STEPS - 1;
        else if (button_step == NUM_STEPS - 1)
            button_step = 0;
        else
            button_step = (button_direction ? NUM_STEPS - 1 : 0);
        button_set_level(WICED_TRUE, WICED_TRUE);
        return;
    }
    else if (button_pushed_duration < 15000)
    {
        // we were moving the level and button is released.
        // set message with ack
        if ((button_step != NUM_STEPS - 1) && (button_step != 0))
            button_set_level(WICED_FALSE, WICED_TRUE);
        return;
    }
    // More than 15 seconds means factory reset
    mesh_application_factory_reset();
}

void button_timer_callback(uint32_t arg)
{
    if (!button_level_moving)
    {
        if (button_step == 0)
        {
            button_direction = WICED_TRUE;
            button_step++;
        }
        else if (button_step == NUM_STEPS - 1)
        {
            button_direction = WICED_FALSE;
            button_step--;
        }
        else
        {
            if (button_direction)
            {
                if (button_step != NUM_STEPS - 1)
                    button_step++;
            }
            else
            {
                if (button_step != 0)
                    button_step--;
            }
        }
    }
    else
    {
        if (button_direction)
        {
            if (button_step != NUM_STEPS - 1)
                button_step++;
        }
        else
        {
            if (button_step != 0)
                button_step--;
        }
    }
    if ((button_step != NUM_STEPS - 1) && (button_step != 0))
    {
        button_level_moving = WICED_TRUE;
        wiced_start_timer(&button_timer, 500);
        button_set_level(WICED_FALSE, WICED_FALSE);
    }
    else
    {
        button_level_moving = WICED_FALSE;
        button_set_level(WICED_FALSE, WICED_TRUE);
    }
}

/*
 * This function tells peer to set the level. Instant transition means that the button
 * is pushed and release, otherwise the button is being pushed, so the transition
 * time is set to 500ms which is the duration between 2 consecutive commands. I.e. we
 * send command to go to the next level every 500ms and duration is 500ms, which should
 * make the transition go smooth.  The final parameter indicates that the transition is
 * complete, so we ask peer to send the ack.
 */
void button_set_level(wiced_bool_t is_instant, wiced_bool_t is_final)
{
    wiced_bt_mesh_level_set_level_t set_data;

    set_data.level = button_level_step[button_step];
    set_data.transition_time = is_instant ? 100 : 500;
    set_data.delay = 0;

    WICED_BT_TRACE("Set level:%d transition time:%d final:%d\n", set_data.level, set_data.transition_time, is_final);

    wiced_bt_mesh_model_level_client_set(0, is_final, &set_data);
}
