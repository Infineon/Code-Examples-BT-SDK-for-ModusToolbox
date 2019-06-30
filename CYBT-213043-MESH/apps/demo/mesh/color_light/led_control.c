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
 * Led control functionality
 *
 */

#include "sparcommon.h"

#include "wiced_hal_pwm.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_aclk.h"
#include "wiced_bt_trace.h"
#include "wiced_platform.h"
#include "led_control.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define PWM_CHANNEL0         PWM0
#define PWM_CHANNEL1         PWM1
#define PWM_CHANNEL2         PWM2

#define PWM_INP_CLK_IN_HZ   (512*1000)
#define PWM_FREQ_IN_HZ      (10000)

/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
#if ( !defined(CYW20719B1) && !defined(CYW20819A1) && !defined(CYW20735B1) )
#define WICED_GPIO_PIN_LED_2 1
#endif
wiced_bt_gpio_numbers_t led_pin = WICED_GPIO_PIN_LED_2;

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/*
 * Initialize LED control
 */
void led_control_init(uint8_t control_type)
{
    pwm_config_t pwm_config;
    uint8_t duty_cycle = 0;

    if (control_type == LED_CONTROL_TYPE_ONOFF)
        return;

    else if (control_type == LED_CONTROL_TYPE_LEVEL)
    {
        /* configure PWM */
#ifdef CYW20719B1
        wiced_hal_pwm_configure_pin(led_pin, PWM_CHANNEL0);
#endif

#if ( defined(CYW20819A1) || defined(CYW20735B1) )
        wiced_hal_gpio_select_function(WICED_GPIO_PIN_LED_2, WICED_PWM0);
#endif
        wiced_hal_aclk_enable(PWM_INP_CLK_IN_HZ, ACLK1, ACLK_FREQ_24_MHZ);
        wiced_hal_pwm_get_params(PWM_INP_CLK_IN_HZ, 0, PWM_FREQ_IN_HZ, &pwm_config);
        wiced_hal_pwm_start(PWM_CHANNEL0, PMU_CLK, pwm_config.toggle_count, pwm_config.init_count, 1);
    }
    else if (control_type == LED_CONTROL_TYPE_COLOR)
    {
        wiced_hal_gpio_select_function(LED_RED, WICED_PWM0);
        wiced_hal_gpio_select_function(LED_GREEN, WICED_PWM1);
        wiced_hal_gpio_select_function(LED_BLUE, WICED_PWM2);

        //start clock that will go into PWM block.
        wiced_hal_aclk_enable(PWM_INP_CLK_IN_HZ, ACLK1, ACLK_FREQ_24_MHZ);

        //convert duty cycle and frequency to toggle and init values
        wiced_hal_pwm_get_params(PWM_INP_CLK_IN_HZ, 0, PWM_FREQ_IN_HZ, &pwm_config);

        //start PWM with proper clock in and toggle/init values derived above
        wiced_hal_pwm_start(PWM_CHANNEL0, PMU_CLK, pwm_config.toggle_count, pwm_config.init_count, 1);
        wiced_hal_pwm_start(PWM_CHANNEL1, PMU_CLK, pwm_config.toggle_count, pwm_config.init_count, 1);
        wiced_hal_pwm_start(PWM_CHANNEL2, PMU_CLK, pwm_config.toggle_count, pwm_config.init_count, 1);
    }
}

/*
 * Set LED brightness level 0 to 100%
 */
void led_control_set_brighness_level(uint8_t brightness_level)
{
    pwm_config_t pwm_config;

    WICED_BT_TRACE("set brightness:%d\n", brightness_level);

    // ToDo.  For some reason, setting brightness to 100% does not work well on 20719B1 platform. For now just use 99% instead of 100.
    if (brightness_level == 100)
        brightness_level = 99;

    wiced_hal_pwm_get_params(PWM_INP_CLK_IN_HZ, brightness_level, PWM_FREQ_IN_HZ, &pwm_config);
    wiced_hal_pwm_change_values(PWM_CHANNEL0, pwm_config.toggle_count, pwm_config.init_count);
}

/*
 * Set LED brightness level 0 to 100% according to different channels.
 */
void led_color_set_brightness_level(uint8_t channel, uint8_t brightness_level)
{
    pwm_config_t pwm_config;

    WICED_BT_TRACE("set brightness:%d\n", brightness_level);

    // ToDo.  For some reason, setting brightness to 100% does not work well on 20719B1 platform. For now just use 99% instead of 100.
    if (brightness_level == 100)
        brightness_level = 99;

    wiced_hal_pwm_get_params(PWM_INP_CLK_IN_HZ, brightness_level, PWM_FREQ_IN_HZ, &pwm_config);
    wiced_hal_pwm_change_values(channel, pwm_config.toggle_count, pwm_config.init_count);
}

/*
 * Control the LED light on the evaluation board with RGB color data.
 */
void rgb_light_control(int red, int green, int blue)
{
    uint8_t red_brightness_level = red * 100 / RGB_COLOR_MAX_RED;
    uint8_t green_brightness_level = green * 100 / RGB_COLOR_MAX_RED;
    uint8_t blue_brightness_level = blue * 100 / RGB_COLOR_MAX_RED;

    led_color_set_brightness_level(PWM_CHANNEL0, red_brightness_level);
    led_color_set_brightness_level(PWM_CHANNEL1, green_brightness_level);
    led_color_set_brightness_level(PWM_CHANNEL2, blue_brightness_level);
}

/*
 * Control the LED light on the evaluation board with HSL color system data.
 */
void led_control_set_color(int hue, int saturation, int lightness)
{
    int red = 0;
    int green = 0;
    int blue = 0;

    if (lightness > 0)
    {
        //when lightness < 2, the lightness is not visible.
        lightness = (lightness < 2) ? 2 : lightness;
        hsl_2_rgb(hue, saturation, lightness, &red, &green, &blue);
    }
    rgb_light_control(red, green, blue);
}


/*
 * Turn LED on or off
 */
void led_control_set_onoff(uint8_t onoff_value)
{
    WICED_BT_TRACE("set onoff:%d\n", onoff_value);

    if (onoff_value == 1)           // led is on
    {
        wiced_hal_gpio_configure_pin(led_pin, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);
    }
    else if (onoff_value == 0)      // led is off
    {
        wiced_hal_gpio_configure_pin(led_pin, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);
    }
}
