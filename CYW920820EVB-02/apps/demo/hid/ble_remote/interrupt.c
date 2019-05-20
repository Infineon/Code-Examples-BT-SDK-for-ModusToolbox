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

#include "interrupt.h"
#include "wiced_hal_gpio.h"

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
// private class implementation
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
/// Enable Interrupt
//////////////////////////////////////////////////////////////////
static void Intr_enableInterrupt(Intr_State * state)
{
    // configure initerrupt line
    uint16_t config = state->intLevelCfg | GPIO_PULL_UP;  // assume interrupt is active low
    if (state->activeLogic == INTR_LVL_HIGH)
    {
        config = state->intLevelCfg | GPIO_PULL_DOWN;
    }

    intrVtblPtr->clearInterrupt(state);
    wiced_hal_gpio_configure_pin(state->intrGPIO, config, !state->activeLogic);

    //store in AON memory to be reinforced in slimboot
    wiced_hal_gpio_slimboot_reenforce_cfg(state->intrGPIO, config);
}

//////////////////////////////////////////////////////////////////
/// Disable interrupt
//////////////////////////////////////////////////////////////////
static void Intr_disableInterrupt(Intr_State * state)
{
    // configure initerrupt line
    uint16_t config = GPIO_PULL_UP;  // assume interrupt is active low
    if (state->activeLogic == INTR_LVL_HIGH)
    {
        config = GPIO_PULL_DOWN;
    }

    wiced_hal_gpio_configure_pin(state->intrGPIO, config, !state->activeLogic);
    intrVtblPtr->clearInterrupt(state);

    //store in AON memory to be reinforced in slimboot
    wiced_hal_gpio_slimboot_reenforce_cfg(state->intrGPIO, config);
}

///////////////////////////////////////////////////////////////
/// Enable or disable interrupt
///////////////////////////////////////////////////////////////
static void Intr_setInterruptEnable(Intr_State * state, uint8_t en)
{
    if (en != state->enabled)
    {
        state->enabled = en;
        if (en)
            intrVtblPtr->enableInterrupt(state);
        else
            intrVtblPtr->disableInterrupt(state);
    }
}

////////////////////////////////////////////////////////////////
/// Clear interrupt
////////////////////////////////////////////////////////////////
static void Intr_clearInterrupt(Intr_State * state)
{
    wiced_hal_gpio_clear_pin_interrupt_status(state->intrGPIO);
}

////////////////////////////////////////////////////////////////
/// return hardware interrupt pending status
////////////////////////////////////////////////////////////////
static uint8_t Intr_isInterruptPending(Intr_State * state)
{
    return wiced_hal_gpio_get_pin_interrupt_status(state->intrGPIO);
}

///////////////////////////////////////////////////////////////
/// Retuns interrupt enabling status
///////////////////////////////////////////////////////////////
static uint8_t Intr_isInterruptEnabled(Intr_State * state)
{
    return state->enabled;
}

/////////////////////////////////////////////////////////////////
/// Returns whether the intr pin is active
/////////////////////////////////////////////////////////////////
static uint8_t Intr_isInterruptPinActive(Intr_State * state)
{
    return (wiced_hal_gpio_get_pin_input_status(state->intrGPIO) ? INTR_LVL_HIGH : INTR_LVL_LOW) == state->activeLogic;
}

/////////////////////////////////////////////////////////////////
/// initerrupt initialization
/////////////////////////////////////////////////////////////////
static void Intr_registerForInterrupt(Intr_State * state, void (*callBack)(void*, uint8_t), void * objPtr)
{
    if (NULL == callBack)
    {
        return;
    }

    wiced_hal_gpio_register_pin_for_interrupt(state->intrGPIO, callBack, objPtr);

    intrVtblPtr->clearInterrupt(state);
    intrVtblPtr->disableInterrupt(state);
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
//          Contructors
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
static Intr_vtbl intrVtbl = {
    Intr_enableInterrupt,
    Intr_disableInterrupt,
    Intr_setInterruptEnable,
    Intr_clearInterrupt,
    Intr_isInterruptPending,
    Intr_isInterruptEnabled,
    Intr_isInterruptPinActive,
    Intr_registerForInterrupt,
};

Intr_vtbl * intrVtblPtr = &intrVtbl;

/////////////////////////////////////////////////////////////
/// Incomplete Intr Contructor
/////////////////////////////////////////////////////////////
void Intr_CIntr_Incomplete(Intr_State * state, uint8_t gpio, uint8_t activeLogic, uint16_t config)
{
    state->activeLogic = activeLogic;
    state->intLevelCfg = config;
    state->intrGPIO = gpio;
    intrVtblPtr->disableInterrupt(state);        // disables interrupt since we don't have callback hooked up yet.
}

//////////////////////////////////////////////////////////////
/// Incompelete contructor
//////////////////////////////////////////////////////////////
void Intr_CIntr(Intr_State * state, void (*callBack)(void*, uint8_t), void * objPtr, uint8_t gpio, uint8_t activeLogic, uint16_t config)
{
    Intr_CIntr_Incomplete(state, gpio, activeLogic, config);
    intrVtblPtr->registerForInterrupt(state, callBack, objPtr);
}

