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
 * I2C Interface
 *
 */

#include "i2cDevice.h"
#include "ble_remote.h"
#include "ble_remote_gatts.h"
#include "wiced_bt_trace.h"

#define iocfg_p5_adr                                   0x00338214                                                   // lhl_adr_base + 0x00000214
#define iocfg_p5                                       (*(volatile unsigned int *)iocfg_p5_adr)
#define iocfg_p7_adr                                   0x0033821c                                                   // lhl_adr_base + 0x0000021c
#define iocfg_p7                                       (*(volatile unsigned int *)iocfg_p7_adr)

uint8_t ReadByte(uint8_t reg_address);
uint8_t ReadBlock(uint8_t reg_address, uint16_t length, uint8_t *data);
uint8_t WriteByte(uint8_t reg_address, uint8_t data);
uint8_t WriteBlock(uint16_t length, uint8_t *dataPtr);

static I2C_t theI2c =
{
    ReadByte,
    ReadBlock,
    WriteByte,
    WriteBlock,
};

static I2C_t * i2c = &theI2c;
static I2CDev_t theDev;
static I2CDev_t * pDev = &theDev;

static uint8_t muxSet = 0;
////////////////////////////////////////////////////////////////////////////////
/// super mux I2C configured for GPIO pins
////////////////////////////////////////////////////////////////////////////////
void i2c_setMux()
{
    if (!muxSet)
    {
        //configure P7 for I2C SCL
        //configure P5 for I2C SDA
        iocfg_p5 = 0;
        iocfg_p7 = 0;
        wiced_hal_i2c_select_pads(7, 5);
    }
}
////////////////////////////////////////////////////////////////////////////////
/// Create a i2c device object
////////////////////////////////////////////////////////////////////////////////
I2C_t * I2CDev(uint8_t speed, uint8_t addr)
{
    i2c_setMux();

    //enable I2C clock
    wiced_hal_i2c_init();

    // if the speed is default 100HHZ, most likely this is the first time we create the instance.
    // We change it to 400 as default speed.
    if (wiced_hal_i2c_get_speed() == I2CM_SPEED_100KHZ)
    {
        wiced_hal_i2c_set_speed(I2CM_SPEED_400KHZ);
    }
    pDev->devAddr   = addr & 0xfe;
    pDev->orgSpeed  = wiced_hal_i2c_get_speed();
    pDev->mySpeed   = speed;
    return i2c;
}

////////////////////////////////////////////////////////////////////////////////
/// Read a byte of data from the LIS331
///   sub_address is the register address.
///   Return the data read.
////////////////////////////////////////////////////////////////////////////////
uint8_t ReadByte(uint8_t reg_address)
{
    uint8_t data;
    wiced_hal_i2c_set_speed(pDev->mySpeed);
    wiced_hal_i2c_combined_read(&data, 1, &reg_address, 1, pDev->devAddr);
    wiced_hal_i2c_set_speed(pDev->orgSpeed);
    return data;
}

////////////////////////////////////////////////////////////////////////////////
/// Read a byte of data
///   sub_address is the register address.
///   Returns TRUE if the read is successful.
///           FALSE if the read has failed.
////////////////////////////////////////////////////////////////////////////////
uint8_t ReadBlock(uint8_t reg_address, uint16_t length, uint8_t *data)
{
    uint8_t status = I2CM_OP_FAILED;
    if (length)
    {
        wiced_hal_i2c_set_speed(pDev->mySpeed);
        status = wiced_hal_i2c_combined_read(data, length, &reg_address, 1, pDev->devAddr);
        wiced_hal_i2c_set_speed(pDev->orgSpeed);
    }
    return status == I2CM_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
/// Write a byte of data to the device
///   sub_address is the register address.
///   data is the byte to write
///   Returns TRUE if the read is successful.
///           FALSE if the read has failed.
////////////////////////////////////////////////////////////////////////////////
uint8_t WriteByte(uint8_t reg_address, uint8_t data)
{
    uint8_t status;
    uint8_t data_array[2];

    wiced_hal_i2c_set_speed(pDev->mySpeed);
    data_array[0] = reg_address;
    data_array[1] = data;
    status = wiced_hal_i2c_write(data_array, 2, pDev->devAddr);
    wiced_hal_i2c_set_speed(pDev->orgSpeed);
    return status == I2CM_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
/// Write a byte of data from the LIS331
///   sub_address is the register address.
///   data is the byte to write
///   Returns TRUE if the read is successful.
///           FALSE if the read has failed.
////////////////////////////////////////////////////////////////////////////////
uint8_t WriteBlock(uint16_t length, uint8_t *dataPtr)
{
    uint8_t status;

    wiced_hal_i2c_set_speed(pDev->mySpeed);
    status = wiced_hal_i2c_write(dataPtr, length, pDev->devAddr);
    wiced_hal_i2c_set_speed(pDev->orgSpeed);
    return status == I2CM_SUCCESS;
}
