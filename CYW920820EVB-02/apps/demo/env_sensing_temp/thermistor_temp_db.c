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
 * thermistor_temp_db.c
 *
 *  @brief
 * This file consists of the resistance to temperature lookup by finding the slope of the R Vs T curve
 *
 */

/****************************************************************************
 *                              INCLUDES
 ***************************************************************************/
#include "thermistor_temp_db.h"

/****************************************************************************
 *                              FUNCTION DEFINITIONS
 ***************************************************************************/
/*
Function name:
    r_t_look_up

Function Description:
@brief    function to map resistance to temperature using R Vs T look-up table

@param table            Pointer to Resistance Vs Temperature look-up table
@param therm_resist     Resistance of thermistor

@return Temperature in celsius as int16_t type
*/

static int16_t r_t_look_up(const r_t_look_up_table* table, int32_t therm_resist)
{

    int i;
    /* The R Vs T table uses Resistance in ohms multiplied by 10 */
    int32_t r = therm_resist * 10;
    int32_t r1, r2;
    int32_t t1, t2, t;

    if (r > (int32_t)r_t_centre[0].resistance_ohms)
    {
        return r_t_centre[0].temp_celsius;
    }

    for (i = 1; i < TABLE_SIZE - 1; i++)
    {
        /* Find the two points in the table to use */
        if (r > (int32_t)r_t_centre[i].resistance_ohms)
        {
            t1 = r_t_centre[i - 1].temp_celsius;
            t2 = r_t_centre[i].temp_celsius;
            r1 = r_t_centre[i - 1].resistance_ohms;
            r2 = r_t_centre[i].resistance_ohms;

            /*
             * The temperature is calculated by adding the temperature at
             * previous point with the slope between the previous point and
             * next point.
             */
            t = t1 + ((((t2 - t1) * (r - r1)) + ((r2 - r1) / 2)) / (r2 - r1));

            return t;
        }
    }

    return r_t_centre[TABLE_SIZE - 1].temp_celsius;
}

/*
 Function name:
 get_temp_in_celsius

 Function Description:
 @brief  This function takes in ADC output from VDDIO and voltage divider to calculate the temperature in celsius.
         The function returns temperature with 2 decimal points of accuracy as expected by Environmental Sensing Profile.

 @param  vref    voltage in millivolts measured from the VDDIO
 @param  vadc    voltage in millivolts measured from the DC Channel

 @return    temperature in celsius multiplied by 100. This to provide 2 fractional positions of resolution.
 */
int16_t get_temp_in_celsius(uint32_t vref, uint32_t vadc)
{

    volatile int16_t    temp_in_celsius     =   0;
    volatile int32_t    r_thermistor        =   0;

    r_thermistor = ((vref - vadc) * BALANCE_RESISTANCE) / vadc;
    temp_in_celsius = r_t_look_up(r_t_centre, r_thermistor);

    return (temp_in_celsius);
}
