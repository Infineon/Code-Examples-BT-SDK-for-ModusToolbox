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
 * This file implements the PAW3805EK-CJV1: Track-On-Glass Mouse Sensor.
 *
 */

#ifdef SUPPORT_MOTION

#include "PAW3805_opticalsensor.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_pspi.h"
#include "wiced_hal_mia.h"
#include "wiced_bt_trace.h"
#include "wiced_hidd_lib.h"
#include "wiced_timer.h"
#include "wiced_gki.h"
#include "interrupt.h"

PAWsensor_Config  PAW_sensor_config =
{
    // the CS line is connected to P2
    // uint8_t cs_gpio;
    2,

    // the motion line is connected to P16
    // uint8_t motion_gpio;
    16,

    // SPI speed to use for the PAW.
    //uint32_t spiSpeed;
    1000000,
};

//
// Product ID for PAW3805EK-CJV1
//
// Product_ID1 (0x00) = 0x31
// Product_ID2 (0x01) = 0x61
//
//
PAWSensorRegSeq PAW_IDCheckSequence[]=
{
    {PAWSENSOR_READCOMPARE,    0x00, 0x31},
    {PAWSENSOR_READCOMPARE,    0x01, 0x61},
};
#define PAW_IDCHECK_NO    (sizeof(PAW_IDCheckSequence)/sizeof(PAWSensorRegSeq))

//
// Software Reset for PAW3805EK-CJV1
//
// Configuration (0x06) = 0x80 : full chip reset (to reset all sensor's internal registers and states)
//
//
PAWSensorRegSeq PAW_resetSequence[]=
{
    {PAWSENSOR_WRITE,    0x06, 0x80},
};
#define PAW_RESET_NO    (sizeof(PAW_resetSequence)/sizeof(PAWSensorRegSeq))

//
// Power down for PAW3805EK-CJV1
//
// Configuration (0x06) = 0x08 : power down mode for lowest power consumption
//
//
PAWSensorRegSeq PAW_powerdownSequence[]=
{
    {PAWSENSOR_WRITE,    0x06, 0x08},
};
#define PAW_POWERDOWN_NO    (sizeof(PAW_powerdownSequence)/sizeof(PAWSensorRegSeq))

//
// Sleep3 mode enable for PAW3805EK-CJV1
//
// Configuration (0x06) = 0x20 : Enable Sleep3 mode for power saving
//
//
PAWSensorRegSeq PAW_sleep3Sequence[]=
{
    {PAWSENSOR_WRITE,    0x06, 0x30},
};
#define PAW_SLEEP3_NO    (sizeof(PAW_sleep3Sequence)/sizeof(PAWSensorRegSeq))

//NOTE: QUICK_BURST is DISABLED by default. Due to not able to get XY data.
#ifdef QUICK_BURST
//
// enable Quick Burst for PAW3805EK-CJV1
//
// Write_Protect (0x09) = 0x5A: disable write protect first
// MFIO_Config (0x26) = 0x10: configure to Quick Burst
// Write_Protect (0x09) = 0x00: enable write protect
//
PAWSensorRegSeq PAW_enableQBSequence[]=
{
    {PAWSENSOR_WRITE,    0x09, 0x5A},
    {PAWSENSOR_WRITE,    0x26, 0x10},
    {PAWSENSOR_WRITE,    0x09, 0x0},
};

#define PAW_ENABLE_QB_NO    (sizeof(PAW_enableQBSequence)/sizeof(PAWSensorRegSeq))

extern void bleapputils_delayUs(uint32_t delay);
#endif

Intr_State   PAWIntr;
wiced_timer_t PAW_reset_timer;
wiced_bool_t  PAWdeviceFound=WICED_FALSE;
wiced_bool_t  PAWActive=WICED_FALSE;


uint8_t PAW3805_readReg(uint8_t regAddress);
void PAW3805_writeReg(uint8_t regAddress, uint8_t val);
void PAW3805_burstRead(uint8_t  *buf, uint8_t bytesToRead);
wiced_bool_t PAW3805_verifyProductId(void);
void PAW3805_reset(void);

/////////////////////////////////////////////////////////////////////
/// walk the register settings list point by regSegList
///
/// \param *regSeqList - point to the PAWSensorRegSeq structure list.
/// \param regSeqNo - size of the regSeglist
/// \return WICED_TRUE if sucess, WICED_FALSE otherwise
///
/////////////////////////////////////////////////////////////////////
wiced_bool_t PAW3805_walkRegSequence(const PAWSensorRegSeq *regSeqList, uint8_t regSeqNo)
{
    uint8_t     i, value=0;
    wiced_bool_t retresult = WICED_TRUE;

    if ( ( regSeqList == NULL ) || ( regSeqNo == 0 ) )
    {
        return WICED_FALSE;
    }

    //
    // walk the entire sequence list and do action
    //
    for ( i=0 ; i < regSeqNo ; i++ )
    {
        //
        // read and compare
        //
        if ( regSeqList[i].rw ==PAWSENSOR_READCOMPARE )
        {
            value = PAW3805_readReg(regSeqList[i].regoffset);

            if ( value != regSeqList[i].value )
            {
                //return false;
                retresult = WICED_FALSE;
            }
        }
        //
        // read only
        //
        else if ( regSeqList[i].rw == PAWSENSOR_READONLY )
        {
            value = PAW3805_readReg(regSeqList[i].regoffset);

        }
        // mask compare with previous read value
        else if ( regSeqList[i].rw == PAWSENSOR_COMPARE )
        {

            if ( (value&regSeqList[i].regoffset) != regSeqList[i].value )
            {
                retresult = WICED_FALSE;
            }
        }
        //
        // write register
        //
        else if ( regSeqList[i].rw == PAWSENSOR_WRITE )
        {
            PAW3805_writeReg(regSeqList[i].regoffset , regSeqList[i].value );
        }
    }

    return retresult;
}

////////////////////////////////////////////////////////////////////////////////
/// This function is the timeout handler for reset timer
////////////////////////////////////////////////////////////////////////////////
void PAW3805_reset_timeout( uint32_t arg )
{
    WICED_BT_TRACE("reset timer timeout\n");

    PAWdeviceFound = PAW3805_verifyProductId();
}

//////////////////////////////////////////////////////////////////////////////////////////
/// Initialize the PAW optical sensor.
///////////////////////////////////////////////////////////////////////////////////////
void PAW3805_init(void (*userfn)(void*, uint8_t), void* userdata)
{
    //initialize reset timer
    wiced_init_timer( &PAW_reset_timer, PAW3805_reset_timeout, 0, WICED_MILLI_SECONDS_TIMER );

    /*********************************
        SPI configuration and intialization START
      **********************************/
    //configure P3 as SPIFFY1 CLK, P4 as SPIFFY1 MOSI, P5 as SPIFFY1 MISO
    wiced_hal_gpio_select_function(WICED_P03, WICED_SPI_1_CLK);
    wiced_hal_gpio_select_function(WICED_P04, WICED_SPI_1_MOSI);
    wiced_hal_gpio_select_function(WICED_P05, WICED_SPI_1_MISO);

    wiced_hal_pspi_init(SPI1,
                        SPI_MASTER,
                        0,    //ignore, it is taken care by above SPIFFY1 MOSI, SPIFFY1 MISO configurations
                        0,    //ignore, it is taken care by above SPIFFY1 CLK, SPIFFY1 MOSI, SPIFFY1 MISO configurations
                        PAW_sensor_config.spiSpeed,
                        SPI_MSB_FIRST,
                        SPI_SS_ACTIVE_LOW,
                        SPI_MODE_3,
                        PAW_sensor_config.cs_gpio);

    //maintain CS pin output during SDS
    wiced_hal_gpio_slimboot_reenforce_cfg(PAW_sensor_config.cs_gpio, GPIO_OUTPUT_ENABLE);
    /*********************************
        SPI configuration and intialization END
      **********************************/

    PAWdeviceFound = PAW3805_verifyProductId();

    if (PAWdeviceFound)
    {
        WICED_BT_TRACE("PAW productID match.\n");

        //register motion interrupt
        Interrupt_init(&PAWIntr, userfn, userdata, PAW_sensor_config.motion_gpio, INTR_LVL_LOW, GPIO_EN_INT_LEVEL_LOW);

        if (wiced_hal_mia_is_reset_reason_por())
        {
#ifdef QUICK_BURST
            //enable Quick Burst
            PAW3805_walkRegSequence(PAW_enableQBSequence,PAW_ENABLE_QB_NO);

            //set P38_MFIO output enable
            wiced_hal_gpio_configure_pin(38, GPIO_OUTPUT_ENABLE, 0);
            wiced_hal_gpio_set_pin_output(38, 0);
#endif
        }
    }
    else
    {
        WICED_BT_TRACE("PAW productID NOT match!!!\n");
        //PAW3805_reset();
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
/// enable/disable motion interrupt
///////////////////////////////////////////////////////////////////////////////////////
void PAW3805_enable_Interrupt(wiced_bool_t enabled)
{
    Interrupt_setInterruptEnable(&PAWIntr, (enabled ? 1:0));
}

////////////////////////////////////////////////////////////////////////////////////////
/// This function reads the specified register over the SPI interface and returns
/// its value
/// \param
///    regAddress - address to read
/// \return
///    register value read from sensor
////////////////////////////////////////////////////////////////////////////////////////
uint8_t PAW3805_readReg(uint8_t regAddress)
{
    uint8_t val;

    // Assert CS.
    wiced_hal_gpio_set_pin_output(PAW_sensor_config.cs_gpio, CS_ASSERT);

    // Write the register address over the SPI
    wiced_hal_pspi_tx_data(SPI1, 1 ,&regAddress);

    // Now get the response from the sensor
    wiced_hal_pspi_rx_data(SPI1, 1, &val);

    // Deassert  CS
    wiced_hal_gpio_set_pin_output(PAW_sensor_config.cs_gpio, CS_DEASSERT);

    return val;
}


///////////////////////////////////////////////////////////////////////////
/// This function writes the given value to the specified sensor register over the
/// SPI interface
/// \param
///    regAddress - address to write to
/// \param
///    val - value to write
///////////////////////////////////////////////////////////////////////////
void PAW3805_writeReg(uint8_t regAddress, uint8_t val)
{
    uint8_t buf[2];

    // Create spi command. Ensure write bit is set in the address byte
    buf[0] = regAddress | WRITE_CMD_BIT;
    buf[1] = val;

    // Assert CS.
    wiced_hal_gpio_set_pin_output(PAW_sensor_config.cs_gpio, CS_ASSERT);

    // Get the SPI interface to do the job
    wiced_hal_pspi_tx_data(SPI1, 2,(uint8_t *)buf);

    // Deassert CS
    wiced_hal_gpio_set_pin_output(PAW_sensor_config.cs_gpio, CS_DEASSERT);
}

////////////////////////////////////////////////////////////////////////////////
/// This function performs a burst read of the sensor registers
/// \param
///    buf - where to store the burst report
/// \param
///    bytesToRead - number of consecutive registers to read
////////////////////////////////////////////////////////////////////////////////
void PAW3805_burstRead(uint8_t  *buf, uint8_t bytesToRead)
{
    // Assert CS.
    wiced_hal_gpio_set_pin_output(PAW_sensor_config.cs_gpio, CS_ASSERT);


    //set P38_MFIO to high for 3 us
    wiced_hal_gpio_set_pin_output(38, 1);
    wiced_gki_delay_us(4);
    wiced_hal_gpio_set_pin_output(38, 0);

    // Now read the burst report
    wiced_hal_pspi_rx_data(SPI1, bytesToRead, buf);


    // Deassert CS
    wiced_hal_gpio_set_pin_output(PAW_sensor_config.cs_gpio, CS_DEASSERT);
}

////////////////////////////////////////////////////////////////////////////////
/// Reads motion data.
///
/// \param
///    x - pointer to location where x movement is returned
/// \param
///    y - pointer to location where y movement is returned
////////////////////////////////////////////////////////////////////////////////
void PAW3805_getMotion(int16_t *x, int16_t *y)
{
    static uint8_t count=0;
    uint8_t data[4]={0, };

    // If we have a sensor connected and MOTION interrupt is active
    if (PAWdeviceFound && Interrupt_isInterruptPinActive(&PAWIntr))
    {
        //disallow slave latency to ensure smooth operation
        count = 0;
        wiced_blehidd_allow_slave_latency(FALSE);
        PAWActive = WICED_TRUE;

        //if motion data is valid
        while (PAW3805_readReg(0x02) & 0x80)
        {
#ifndef QUICK_BURST
            data[0] = PAW3805_readReg(0x03);
            data[1] = PAW3805_readReg(0x04);
            data[2] = PAW3805_readReg(0x11);
            data[3] = PAW3805_readReg(0x12);

#else
            PAW3805_burstRead(data, 4);

            WICED_BT_TRACE("data[0]:0x%x, data[1]:0x%x, data[2]:0x%x, data[3]:0x%x\n", data[0], data[1], data[2], data[3]);
#endif
            // Accumulate X and Y motion
            *x += (int16_t)((data[2] << 8) | data[0]);
            *y += (int16_t)((data[3] << 8) | data[1]);

        }

    }
    else
    {
        count++;
    }

    if (count == 10)
    {
        //allow slave latency if no motion for a while
        wiced_blehidd_allow_slave_latency(TRUE);
        PAWActive = WICED_FALSE;

        //re-enable motion INTERRUPT
        PAW3805_enable_Interrupt(WICED_TRUE);
    }

}

////////////////////////////////////////////////////////////////////////////////
/// flush motion data so that MOTION interrupt will be clear
////////////////////////////////////////////////////////////////////////////////
void PAW3805_flushMotion(void)
{
    uint8_t data[4]={0, };

    // If we have a sensor connected and MOTION interrupt is active
    if (PAWdeviceFound && Interrupt_isInterruptPinActive(&PAWIntr))
    {
        //if motion data is valid
        while (PAW3805_readReg(0x02) & 0x80)
        {
#ifndef QUICK_BURST
            data[0] = PAW3805_readReg(0x03);
            data[1] = PAW3805_readReg(0x04);
            data[2] = PAW3805_readReg(0x11);
            data[3] = PAW3805_readReg(0x12);
            UNUSED_VARIABLE(data);

#else
            PAW3805_burstRead(data, 4);

            WICED_BT_TRACE("data[0]:0x%x, data[1]:0x%x, data[2]:0x%x, data[3]:0x%x\n", data[0], data[1], data[2], data[3]);
#endif
        }

    }

    wiced_blehidd_allow_slave_latency(TRUE);
    PAWActive = WICED_FALSE;

}


///////////////////////////////////////////////////////////////////////////
/// This function verifies that a sensor is attached to the SPI interface
/// \return
///    - WICED_TRUE if a sensore is detected
///    - WICED_FALSE otherwise
///////////////////////////////////////////////////////////////////////////
wiced_bool_t PAW3805_verifyProductId(void)
{
    return PAW3805_walkRegSequence(PAW_IDCheckSequence,PAW_IDCHECK_NO);
}

///////////////////////////////////////////////////////////////////////////
/// Reset sensor and wait 4ms
///////////////////////////////////////////////////////////////////////////
void PAW3805_reset(void)
{
    PAW3805_walkRegSequence(PAW_resetSequence,PAW_RESET_NO);

    wiced_start_timer(&PAW_reset_timer,4); //timeout in 4 ms
}

///////////////////////////////////////////////////////////////////////////
/// power down sensor
///////////////////////////////////////////////////////////////////////////
void PAW3805_powerdown(void)
{
    //disable interrupt
    PAW3805_enable_Interrupt(WICED_FALSE);

    PAW3805_walkRegSequence(PAW_powerdownSequence,PAW_POWERDOWN_NO);
}

////////////////////////////////////////////////////////////////////////////////
/// enable sleep3 mode for PAW3805. sleep1 and sleep2 are enabled by default
////////////////////////////////////////////////////////////////////////////////
void PAW3805_enable_deep_sleep_mode(void)
{
    PAW3805_walkRegSequence(PAW_sleep3Sequence,PAW_SLEEP3_NO);
}

///////////////////////////////////////////////////////////////////////////
/// Check if PAW3805 is active
///////////////////////////////////////////////////////////////////////////
wiced_bool_t PAW3805_isActive(void)
{
  return ((Interrupt_isInterruptPinActive(&PAWIntr) || wiced_is_timer_in_use(&PAW_reset_timer) || PAWActive) ? WICED_TRUE : WICED_FALSE);
}

#endif // SUPPORT_MOTION
