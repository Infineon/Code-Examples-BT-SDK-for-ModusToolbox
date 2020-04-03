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
* File Name: app_user_interface.h
* Version: 1.0
*
* Description:
*   Header file for application user interface (LEDs, Buttons) related
*   functionality
*
*******************************************************************************/

#ifndef APP_USER_INTERFACE_H_
#define APP_USER_INTERFACE_H_

/*******************************************************************************
*        Header Files
*******************************************************************************/
#include "wiced_platform.h"
#include "GeneratedSource/cycfg_pins.h"

/*******************************************************************************
*        Macro Definitions
*******************************************************************************/
/* LED pin assignments for advertising event and IAS alert levels */
#define ADV_LED_GPIO                    WICED_GET_PIN_FOR_LED(WICED_PLATFORM_LED_1)
#define IAS_LED_GPIO                    WICED_GET_PIN_FOR_LED(WICED_PLATFORM_LED_2)

/* Update rate of LED's in milliseconds when blinking */
#define ADV_LED_UPDATE_RATE_MS          250
#define IAS_LED_UPDATE_RATE_MS          250

/* LED's on the kit are active low */
#define LED_ON                          0
#define LED_OFF                         1

/* IAS Alert Levels */
#define IAS_ALERT_LEVEL_LOW             0u
#define IAS_ALERT_LEVEL_MID             1u
#define IAS_ALERT_LEVEL_HIGH            2u

/*******************************************************************************
*        Function Prototypes
*******************************************************************************/
void app_user_interface_init(void);
void adv_led_update(void);
void ias_led_update(void);

#endif /* APP_USER_INTERFACE_H_ */

/* [] END OF FILE */
