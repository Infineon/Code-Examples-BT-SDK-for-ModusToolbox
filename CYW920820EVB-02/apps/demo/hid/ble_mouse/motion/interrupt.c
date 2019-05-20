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
 * Interrupt Interface
 *
 */

#ifdef SUPPORT_MOTION

#include "interrupt.h"
#include "wiced_hal_gpio.h"


//////////////////////////////////////////////////////////////////
/// Enable Interrupt
//////////////////////////////////////////////////////////////////
void Interrupt_enableInterrupt(Intr_State * state)
{
    // configure initerrupt line
    uint16_t config = state->intLevelCfg | GPIO_PULL_UP;  // assume interrupt is active low
    if (state->activeLogic == INTR_LVL_HIGH)
    {
        config = state->intLevelCfg | GPIO_PULL_DOWN;
    }

    Interrupt_clearInterrupt(state);
    wiced_hal_gpio_configure_pin(state->intrGPIO, config, !state->activeLogic);

    //store in AON memory to be reinforced in slimboot
    wiced_hal_gpio_slimboot_reenforce_cfg(state->intrGPIO, config);
}

//////////////////////////////////////////////////////////////////
/// Disable interrupt
//////////////////////////////////////////////////////////////////
void  Interrupt_disableInterrupt(Intr_State * state)
{
    // configure initerrupt line
    uint16_t config = GPIO_PULL_UP;  // assume interrupt is active low
    if (state->activeLogic == INTR_LVL_HIGH)
    {
        config = GPIO_PULL_DOWN;
    }

    wiced_hal_gpio_configure_pin(state->intrGPIO, config, !state->activeLogic);
    Interrupt_clearInterrupt(state);

    //store in AON memory to be reinforced in slimboot
    wiced_hal_gpio_slimboot_reenforce_cfg(state->intrGPIO, config);
}

///////////////////////////////////////////////////////////////
/// Enable or disable interrupt
///////////////////////////////////////////////////////////////
void Interrupt_setInterruptEnable(Intr_State * state, uint8_t en)
{
    if (en != state->enabled)
    {
        state->enabled = en;
        if (en)
            Interrupt_enableInterrupt(state);
        else
            Interrupt_disableInterrupt(state);
    }
}

////////////////////////////////////////////////////////////////
/// Clear interrupt
////////////////////////////////////////////////////////////////
void Interrupt_clearInterrupt(Intr_State * state)
{
    wiced_hal_gpio_clear_pin_interrupt_status(state->intrGPIO);
}

////////////////////////////////////////////////////////////////
/// return hardware interrupt pending status
////////////////////////////////////////////////////////////////
uint8_t Interrupt_isInterruptPending(Intr_State * state)
{
    return wiced_hal_gpio_get_pin_interrupt_status(state->intrGPIO);
}

///////////////////////////////////////////////////////////////
/// Retuns interrupt enabling status
///////////////////////////////////////////////////////////////
uint8_t Interrupt_isInterruptEnabled(Intr_State * state)
{
    return state->enabled;
}

/////////////////////////////////////////////////////////////////
/// Returns whether the intr pin is active
/////////////////////////////////////////////////////////////////
uint8_t Interrupt_isInterruptPinActive(Intr_State * state)
{
    return (wiced_hal_gpio_get_pin_input_status(state->intrGPIO) ? INTR_LVL_HIGH : INTR_LVL_LOW) == state->activeLogic;
}

/////////////////////////////////////////////////////////////////
/// initerrupt initialization
/////////////////////////////////////////////////////////////////
void Interrupt_registerForInterrupt(Intr_State * state, void (*callBack)(void*, uint8_t), void *userdata)
{
    if (NULL == callBack)
    {
        return;
    }

    wiced_hal_gpio_register_pin_for_interrupt(state->intrGPIO, callBack, userdata);

    Interrupt_clearInterrupt(state);
    Interrupt_disableInterrupt(state);
}

/////////////////////////////////////////////////////////////////////////////////
/// Interrupt initialization
/////////////////////////////////////////////////////////////////////////////////
void Interrupt_init(Intr_State * state, void (*callBack)(void*, uint8_t), void *userdata, uint8_t gpio, uint8_t activeLogic, uint16_t config)
{
    state->activeLogic = activeLogic;
    state->intLevelCfg = config;
    state->intrGPIO = gpio;

    Interrupt_registerForInterrupt(state, callBack, userdata);
}


#endif // SUPPORT_MOTION
