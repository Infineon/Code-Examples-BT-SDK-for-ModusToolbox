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
 * This file defines the driver for PAW3805EK-CJV1: Track-On-Glass Mouse Sensor.
 *
 */

#ifdef SUPPORT_MOTION

#ifndef __PAW3805_OPTICALSENSOR_H__
#define __PAW3805_OPTICALSENSOR_H__

#include "wiced.h"
#include "data_types.h"

typedef struct
{
    uint8_t       rw;         // read or write
    uint8_t       regoffset;   // register offset
    uint8_t       value;       // write value
}PAWSensorRegSeq;

//////////////////////////////////
/// Optical Sensor Config for PAW optical sensor
//////////////////////////////////
typedef struct
    {
    /// the GPIO pin the CS line is connected to
    uint8_t cs_gpio;

    /// the GPIO pin the motion line is connected to
    uint8_t motion_gpio;

    /// SPI speed to use for the 7050.
    uint32_t spiSpeed;

} PAWsensor_Config;

enum
{
    // This bit must be set in the address byte of every write to
    WRITE_CMD_BIT  = 0x80,

    CS_ASSERT       = 0,    // low active
    CS_DEASSERT     = 1,
};

enum PAWSENSOR_ACT_PROCEDURE_ID
{
    /// perform spi read and compare
    PAWSENSOR_READCOMPARE            =  1  ,

    /// just read the sensor register via spi interferace, ignore the read result
    PAWSENSOR_READONLY               =  2  ,

    /// write the sensor register
    PAWSENSOR_WRITE                  =  3  ,

    /// compare with previoue read value,
    /// the regoffset will become mask
    PAWSENSOR_COMPARE                  =  4  ,
};

void PAW3805_init(void (*userfn)(void*, uint8_t), void* userdata);
void PAW3805_enable_Interrupt(wiced_bool_t enabled);
void PAW3805_getMotion(int16_t *x, int16_t *y);
void PAW3805_flushMotion(void);
wiced_bool_t PAW3805_isActive(void);
void PAW3805_powerdown(void);
void PAW3805_enable_deep_sleep_mode(void);

#endif

#endif // SUPPORT_MOTION
