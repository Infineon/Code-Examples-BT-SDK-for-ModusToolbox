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

#ifndef __APP_DEFS_H__
#define __APP_DEFS_H__

#include "hidevent.h"
#include "wiced_hal_gpio.h"


enum {
   GPIO_00,
   GPIO_01,
   GPIO_02,
   GPIO_03,
   GPIO_04,
   GPIO_05,
   GPIO_06,
   GPIO_07,
   GPIO_08,
   GPIO_09,
   GPIO_10,
   GPIO_11,
   GPIO_12,
   GPIO_13,
   GPIO_14,
   GPIO_15,
   GPIO_16,
   GPIO_17,
   GPIO_18,
   GPIO_19,
   GPIO_20,
   GPIO_21,
   GPIO_22,
   GPIO_23,
   GPIO_24,
   GPIO_25,
   GPIO_26,
   GPIO_27,
   GPIO_28,
   GPIO_29,
   GPIO_30,
   GPIO_31,
   GPIO_32,
   GPIO_33,
   GPIO_34,
   GPIO_35,
   GPIO_36,
   GPIO_37,
   GPIO_38,
   GPIO_39,
};
#define GPIO_PORT(n) (n / GPIO_MAX_PINS_PER_PORT)
#define GPIO_PIN(n)  (n % GPIO_MAX_PINS_PER_PORT)

#define GPIO_OUT(p, v) wiced_hal_gpio_set_pin_output(p,v)


// Hardware GPIO pin assignment
//----------------------------  -------
#define GPIO_KEY_R0             GPIO_00
#define GPIO_KEY_R1             GPIO_01
#define GPIO_KEY_R2             GPIO_02
#define GPIO_KEY_R3             GPIO_03
#define GPIO_KEY_R4             GPIO_04
#define GPIO_SDA                GPIO_05
#define GPIO_BUZ_PWM            GPIO_06
#define GPIO_SCL                GPIO_07
#define GPIO_KEY_C0             GPIO_08
#define GPIO_KEY_C1             GPIO_09
#define GPIO_KEY_C2             GPIO_10
#define GPIO_KEY_C3             GPIO_11
#define GPIO_RSTN_TP            GPIO_12
  #define GPIO_TOUCHPAD_OFF           0
  #define GPIO_TOUCHPAD_ON            1
#define GPIO_ATTN_TP            GPIO_13
#define GPIO_MOTION_INT         GPIO_14
#define GPIO_XTAL               GPIO_15
#define GPIO_NC16               GPIO_16
#define GPIO_NC17               GPIO_17
#define GPIO_NC16               GPIO_16
#define GPIO_NC17               GPIO_17
#define GPIO_NC18               GPIO_18
#define GPIO_NC19               GPIO_19
#define GPIO_NC20               GPIO_20
#define GPIO_NC21               GPIO_21
#define GPIO_NC22               GPIO_22
#define GPIO_NC23               GPIO_23
#define GPIO_NC24               GPIO_24
#define GPIO_NC25               GPIO_25
#define GPIO_NC26               GPIO_26
#define GPIO_PORT_LED           GPIO_27
  #define GPIO_Led_ON                 0
  #define GPIO_Led_OFF                1
#define GPIO_NC28               GPIO_28
#define GPIO_NC29               GPIO_29
#define GPIO_NC33               GPIO_33
#define GPIO_NC34               GPIO_34
#define GPIO_NC35               GPIO_35
#define GPIO_VBAT               GPIO_36
#define GPIO_NC37               GPIO_37
#define GPIO_IRTX               GPIO_38
#define GPIO_NC39               GPIO_39

/***********************************************************************************************************************************************/
// report id defines
/***********************************************************************************************************************************************/
// App input report defines
enum {
    RPT_ID_MOUSE              = 8, //BLTH03513877 changed from 2 to 8 because keyboard bit-mapped report is 2 and causes conflict
    RPT_ID_IN_ABS_XY          = 0x20,
    RPT_ID_VOICE_CTL          = 0xf8,
#define RPT_ID_VOICE_CTL  0xF8
#ifdef USE_MOTION_AS_AIR_MOUSE
#define MOTION_REPORT_ID     RPT_ID_MOUSE
#else
#define MOTION_REPORT_ID     0x08
#endif
};
/***********************************************************************************************************************************************/
// App event defines
/***********************************************************************************************************************************************/

enum
{
    NO_EVENTS = 0,
    HID_EVENT_AVAILABLE = 1,
    HID_EVENT_APP = 0xd0,                   // HID Event app extension starts from 0xd0
    HID_EVENT_HID_RPT = HID_EVENT_APP,      // 0xd0 User defeined hid report event
    HID_EVENT_AUDIO_RPT,
    // from evoke
    HID_EVENT_HID_INFO,
    HID_EVENT_TP_FINGER_STATUS_CHANGE,
    HID_EVENT_TP_INFO,                      // information data, no need to send as report
    HID_EVENT_TP_DATA,                      // information data, no need to send as report
};



#endif
