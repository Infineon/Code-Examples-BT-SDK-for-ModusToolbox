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

/*******************************************************************************
* File Name: app_user_interface.c
* Version: 1.0
*
* Description:
*   Source file for application user interface (LEDs, Buttons) related
*   functionality
*
*******************************************************************************/

/*******************************************************************************
*        Header Files
*******************************************************************************/
#include "app_bt_event_handler.h"
#include "app_user_interface.h"
#include "wiced_timer.h"
#include "wiced_platform.h"
#include "wiced_hal_gpio.h"
#include "GeneratedSource/cycfg_gatt_db.h"

/*******************************************************************************
*        Variable Definitions
*******************************************************************************/
static wiced_timer_t adv_led_timer, ias_led_timer;
static wiced_bool_t adv_timer_stopped_flag = WICED_TRUE;
static wiced_bool_t ias_timer_stopped_flag = WICED_TRUE;

/*******************************************************************************
*        Function Prototypes
*******************************************************************************/
static void ias_led_timer_cb(uint32_t arg);
static void adv_led_timer_cb(uint32_t arg);

/*******************************************************************************
*        Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: app_user_interface_init()
********************************************************************************
*
* Summary:
*   This function initializes the application user interface related
*   functionality
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void app_user_interface_init(void)
{
    /* Initialize the timers used for advertising state LED, and IAS alert
     * level LED */
    wiced_init_timer(&adv_led_timer, adv_led_timer_cb, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER);
    wiced_init_timer(&ias_led_timer, ias_led_timer_cb, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER);
}

/*******************************************************************************
* Function Name: adv_led_update()
********************************************************************************
*
* Summary:
*   This function updates the advertising LED state based on BLE advertising/
*   connection state
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void adv_led_update(void)
{
    /* Stop the advertising led timer */
    wiced_stop_timer(&adv_led_timer);

    /* Set the stop flag to prevent any pending timer callback to change LED
     * state */
    adv_timer_stopped_flag = WICED_TRUE;

    /* Set LED state based on BLE advertising/connection state.
     * LED OFF for no advertisement/connection, LED blinking for advertisement
     * state, and LED ON for connected state  */
    switch(app_bt_adv_conn_state)
    {
        case APP_BT_ADV_OFF_CONN_OFF:
            wiced_hal_gpio_set_pin_output(ADV_LED_GPIO, LED_OFF);
            break;

        case APP_BT_ADV_ON_CONN_OFF:
            wiced_start_timer(&adv_led_timer, ADV_LED_UPDATE_RATE_MS);
            adv_timer_stopped_flag = WICED_FALSE;
            break;

        case APP_BT_ADV_OFF_CONN_ON:
            wiced_hal_gpio_set_pin_output(ADV_LED_GPIO, LED_ON);
            break;

        default:
            /* LED OFF for unexpected states */
            wiced_hal_gpio_set_pin_output(ADV_LED_GPIO, LED_OFF);
            break;
    }
}

/*******************************************************************************
* Function Name: ias_led_update()
********************************************************************************
*
* Summary:
*   This function updates the IAS alert level LED state based on BLE
*   advertising/connection state
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void ias_led_update(void)
{
    /* Stop the IAS led timer */
    wiced_stop_timer(&ias_led_timer);

    /* Set the stop flag to prevent any pending timer callback to change LED
     * state */
    ias_timer_stopped_flag = WICED_TRUE;

    /* Update LED based on IAS alert level only when the device is connected */
    if(app_bt_adv_conn_state == APP_BT_ADV_OFF_CONN_ON)
    {
        /* Set LED state based on IAS alert level. LED OFF for low level,
         * LED blinking for mid level, and LED ON for high level  */
        switch(app_ias_alert_level[0])
        {
            case IAS_ALERT_LEVEL_LOW:
                wiced_hal_gpio_set_pin_output(IAS_LED_GPIO, LED_OFF);
                break;

            case IAS_ALERT_LEVEL_MID:
                ias_timer_stopped_flag = WICED_FALSE;
                wiced_start_timer(&ias_led_timer, IAS_LED_UPDATE_RATE_MS);
                break;

            case IAS_ALERT_LEVEL_HIGH:
                wiced_hal_gpio_set_pin_output(IAS_LED_GPIO, LED_ON);
                break;

            default:
                /* Consider any other level as High alert level */
                wiced_hal_gpio_set_pin_output(IAS_LED_GPIO, LED_ON);
                break;
        }
    }
    else
    {
        /* In case of disconnection, turn off the IAS LED */
        wiced_hal_gpio_set_pin_output(IAS_LED_GPIO, LED_OFF);
    }
}

/*******************************************************************************
* Function Name: adv_led_timer_cb()
********************************************************************************
*
* Summary:
*   This timer callback function toggles the state of the advertising LED and is
*   used to indicate the device is advertising
*
* Parameters:
*   uint32_t arg - The argument parameter is not used in this callback
*
* Return:
*   None
*
*******************************************************************************/
static void adv_led_timer_cb(uint32_t arg)
{
    /* The timer stopped flag is checked to prevent any pending timer
     * callback from changing LED state after the timer is stopped */
    if(!adv_timer_stopped_flag)
    {
        if(wiced_hal_gpio_get_pin_output(ADV_LED_GPIO) == LED_OFF)
        {
            wiced_hal_gpio_set_pin_output(ADV_LED_GPIO, LED_ON);
        }
        else
        {
            wiced_hal_gpio_set_pin_output(ADV_LED_GPIO, LED_OFF);
        }
    }
}

/*******************************************************************************
* Function Name: ias_led_timer_cb()
********************************************************************************
*
* Summary:
*   This timer callback function toggles the state of the IAS LED and is
*   used to indicate the alert level is mid level
*
* Parameters:
*   uint32_t arg - The argument parameter is not used in this callback
*
* Return:
*   None
*
*******************************************************************************/
static void ias_led_timer_cb(uint32_t arg)
{
    /* The timer stopped flag is checked to prevent any pending timer
     * callback from changing LED state after the timer is stopped */
    if(!ias_timer_stopped_flag)
    {
        if(wiced_hal_gpio_get_pin_output(IAS_LED_GPIO) == LED_OFF)
        {
            wiced_hal_gpio_set_pin_output(IAS_LED_GPIO, LED_ON);
        }
        else
        {
            wiced_hal_gpio_set_pin_output(IAS_LED_GPIO, LED_OFF);
        }
    }
}

/* [] END OF FILE */
