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
 * Interrupt Interface definitions
 *
 */

#ifdef SUPPORT_MOTION

#ifndef __INTR_H__
#define __INTR_H__
#include "brcm_fw_types.h"
#include "wiced.h"

enum
{
    INTR_LVL_LOW,
    INTR_LVL_HIGH,
};

typedef struct
{
    uint8_t intrGPIO;
    uint8_t activeLogic;
    uint16_t intLevelCfg;        // A 16-bit value will be used for interrupt register configuration.
                                 // For example, user can define Interrupt enabling type: GPIO_EN_INT_RISING_EDGE or GPIO_EN_INT_BOTH_EDGE --> see "wiced_hal_gpio.h"
    uint8_t enabled;
} Intr_State;


void Interrupt_init(Intr_State * state, void (*callBack)(void*, uint8_t), void *userdata, uint8_t gpio, uint8_t activeLogic, uint16_t config);
void Interrupt_setInterruptEnable(Intr_State * state, uint8_t en);
void Interrupt_clearInterrupt(Intr_State * state) ;
uint8_t Interrupt_isInterruptPending(Intr_State * state);
uint8_t Interrupt_isInterruptEnabled(Intr_State * state);
uint8_t Interrupt_isInterruptPinActive(Intr_State * state);

#endif

#endif // SUPPORT_MOTION
