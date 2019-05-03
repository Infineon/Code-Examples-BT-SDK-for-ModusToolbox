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
 * WICED PWM sample application.
 *
 * This application demonstrates how to configure and use
 * PWM in WICED Eval. boards to control the brightness of the
 * LED's onboard
 *
 * Features demonstrated
 * - PWM WICED API's
 *
 * Application Instructions
 *  To demonstrate the app, work through the following steps.
 * 1. Plug the CYW920819EVB-02 evaluation board to your computer.
 * 2. Build and download the application. (See CYW920819EVB-02 User Guide)
 * 3. Use Terminal emulation tools like Teraterm or Putty to view the
 *   trace messages(See Kit User Guide).
 * 4. The user can notice the LED breathing effect on the LEDs
 */

#include "sparcommon.h"
#include "wiced_hal_pwm.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_aclk.h"
#include "wiced_platform.h"
#include "wiced_timer.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define PWM_CHANNEL         PWM0

#define PWM_INP_CLK_IN_HZ   (32*1000)
#define PWM_FREQ_IN_HZ      (10000)
#define PWM_DUTY_CYCLE      (40)

/******************************************************************************
 *                                Structures
 ******************************************************************************/

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
static wiced_bool_t pwm_running = WICED_FALSE;

static wiced_timer_t seconds_timer;               /* wiced bt app seconds timer */
/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/
void pwm_sample_app_init(void);

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

void pwm_sample_app_button_interrupt_handler(void* data, uint8_t pin)
{
    /* Toggle the PWM on/off */
    if (pwm_running)
    {
        WICED_BT_TRACE("Stopping PWM... \n\r");
        wiced_hal_pwm_disable(PWM_CHANNEL);
        pwm_running = WICED_FALSE;
    }
    else
    {
        WICED_BT_TRACE("Starting PWM... \n\r");
        wiced_hal_pwm_enable(PWM_CHANNEL);
        pwm_running = WICED_TRUE;
    }
}

/* The function invoked on timeout of app seconds timer. */
void seconds_app_timer_cb( uint32_t arg )
{
    pwm_config_t pwm_config;
    static uint32_t idx=0;

    /* vary the PWM duty cycle in steps of 5% to control the brightness of the onboard LED */
    uint32_t duty_cycle[] = { 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65,
            70, 75, 80, 85, 90, 95, 100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50,
            45, 40, 35, 30, 25, 20, 15, 10, 5 };

    if( (sizeof(duty_cycle)/sizeof(uint32_t) == idx) )
    {
        idx = 0;
    }

    wiced_hal_pwm_get_params(PWM_INP_CLK_IN_HZ,
                             duty_cycle[idx++],
                             PWM_FREQ_IN_HZ,
                             &pwm_config);

    wiced_hal_pwm_change_values(PWM_CHANNEL,
                                pwm_config.toggle_count,
                                pwm_config.init_count);
}

void pwm_sample_app_init(void)
{
    pwm_config_t pwm_config;

    /* Configure buttons available on the platform (pin should be configured
     * before registering interrupt handler )
     */
    wiced_platform_register_button_callback(WICED_PLATFORM_BUTTON_1,
                                            pwm_sample_app_button_interrupt_handler,
                                            NULL,
                                            WICED_PLATFORM_BUTTON_RISING_EDGE);

    /* configure PWM */
    wiced_hal_aclk_enable(PWM_INP_CLK_IN_HZ, ACLK1, ACLK_FREQ_24_MHZ);

#ifdef CYW20819A1
    wiced_hal_gpio_select_function(WICED_P26, WICED_PWM0);
#else
    wiced_hal_pwm_configure_pin(WICED_GPIO_PIN_LED_2,
                                PWM_CHANNEL);
#endif

    wiced_hal_pwm_get_params(PWM_INP_CLK_IN_HZ,
                             PWM_DUTY_CYCLE,
                             PWM_FREQ_IN_HZ,
                             &pwm_config);

    wiced_hal_pwm_start(PWM_CHANNEL,
                        PMU_CLK,
                        pwm_config.toggle_count,
                        pwm_config.init_count,
                        0);
    pwm_running = WICED_TRUE;

    if(WICED_SUCCESS == wiced_init_timer(&seconds_timer,
                                          &seconds_app_timer_cb,
                                          0,
                                          WICED_MILLI_SECONDS_PERIODIC_TIMER ))
    {
        if(WICED_SUCCESS != wiced_start_timer(&seconds_timer, 100 ))
        {
            WICED_BT_TRACE("Seconds Timer Error\n\r");
        }
    }
}

/*
 *  Management callback receives various notifications from the stack
 */
wiced_result_t app_management_callback(wiced_bt_management_evt_t event,
                                  wiced_bt_management_evt_data_t *p_event_data)
{

    wiced_result_t result = WICED_BT_SUCCESS;

    WICED_BT_TRACE("app_management_callback %d\n", event);

    switch (event)
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            pwm_sample_app_init();
        break;

        default:
            result = WICED_BT_USE_DEFAULT_SECURITY;
        break;
    }
    return result;
}

/*
 *  Entry point to the application.
 */
void application_start(void)
{

#ifdef WICED_BT_TRACE_ENABLE
     wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
#endif

     WICED_BT_TRACE( "\n--------------------------------------------------------- \n"
                     "              PWM Sample Application \n"
                     "---------------------------------------------------------\n");

     wiced_bt_stack_init(app_management_callback, NULL, NULL);
}

