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

#ifdef SUPPORT_TOUCHPAD

#include "brcm_fw_types.h"
#include "IQS5xx.h"
#include "../interrupt.h"
#include "appDefs.h"
#include "wiced_bt_trace.h"
#include "ble_remote.h"

// include "localdbg.h" to enable debug code
//#include "localdbg.h"

static uint8_t initData[] = {
// AZP525 Touchpad register initialization data
// LEN    REG     2     3     4     5     6     7     8     9     a     b     c     d     e     f    10    11    12    13    14    15
// ---   ----  ----  ----  ----  ----  ----  ----  ----  ----  ----  ----  ----  ----  ----  ----  ----  ----  ----  ----  ----  ----
   0x02, 0x10, 0x80,                                          // Device reset
   0x08, 0x15, 0x05, 0x04, 0x04, 0x04, 0x85, 0x00, 0x0F,      // Channel setup
   0x09, 0x17, 0x00, 0x0F, 0x00, 0x0F, 0x00, 0x0F, 0x00, 0x0F,// Active channels
   0x0c, 0x11, 0x0e, 0x8c, 0x07, 0x0a, 0x00, 0x64, 0x16, 0xeb, 0x07, 0x01, 0x2c, // Threshold settings
   0x15, 0x41, 0x14, 0x28, 0x28, 0x19, 0x0a, 0x28, 0x2d, 0x2d, 0x2d, 0x07, 0x28, 0x32, 0x32, 0x28, 0x07, 0x14, 0x28, 0x28, 0x19, 0x0a, // 5(rx)*4(tx)=20 touchThresholdSettings
   0x05, 0x43, 0x0e, 0x0e, 0x0e, 0x0e,                        // Slider ATICSettings
   0x03, 0x18, 0x44, 0x44,                                    // Debounce setting
   0x04, 0x40, 0x80, 0x80, 0x09,                              // Track management
   0x0d, 0x13, 0x10, 0x01, 0x26, 0x10, 0x80, 0x03, 0x0f, 0x14, 0x4b, 0x0a, 0x14, 0x32, // filter settings
//   0x06, 0x14, 0xFF, 0xFF, 0x08, 0x07, 0x04,                  // Timing Set
   0x06, 0x14, 0xFF, 0xFF, 0x04, 0x07, 0x01,                  // Timing Set
   0x02, 0x40, 0x20,                                          // Track management PM_ATI_LOAD_FROM_NVM
   0x15, 0x42, 0x50, 0x46, 0x46, 0x50, 0x64, 0x4b, 0x3c, 0x3c, 0x4b, 0x64, 0x4b, 0x3c, 0x3c, 0x4b, 0x64, 0x5a, 0x46, 0x46, 0x5a, 0x64, // ATITargetSettings
   0x02, 0x10, 0x08,                                          // CTRL normal mode
   0x07, 0x12, 0x02, 0x58, 0x03, 0x02, 0xee, 0x06,            // ATISettings
   0x02, 0x10, 0x04,                                          // CTRL AUTO_ATI
   0x02, 0x40, 0x40,                                          // TRACKPAD_MANAGEMENT_REG   save to nvm
   0x02, 0x10, 0x08,                                          // CTRL prox mode
   0x04, 0x24, 0x03, 0xe8, 0x19,                              // CTRL prox mode ATI settings
   0x02, 0x10, 0x0c,                                          // CTRL PROX_MODE | AUTO_ATI
   0x02, 0x40, 0x10,                                          // TRACKPAD_MANAGEMENT_REG   PM_ATI_SAVE_TO_NVM nvm
   0x03, 0x10, 0x21, 0x02,                                    // CTRL Set single-xy, event mode, low pwr, no snap
   0x00                                                       // end of cfg, must be 0x00
};

#define INVALID_TP 0x55aa

static uint8_t wakeUp();
static uint8_t writeData(uint8_t * data);
static uint8_t waitForRDY_LowHigh(uint16_t wLo, uint16_t wHi);
uint8_t init();
uint8_t shutdown();
void hwReset();
uint8_t readFirmwareVersion(uint8_t * buff);
uint8_t waitForRDY(uint8_t logic, uint32_t waitMs);
uint8_t getInfo();
uint8_t proximityRpt();
uint8_t fingerCount();
uint8_t readXYData();
void clearData();
void clearFingerData(XY_Data_finger_t * finger);
AbsXYReport * getRpt();

static TPDrv_t   theTpDrv = {
    readFirmwareVersion,
    init,
    shutdown,
    hwReset,
    wakeUp,
    waitForRDY,
    getInfo,
    proximityRpt,
    fingerCount,
    readXYData,
    clearData,
    clearFingerData,	
    getRpt,
};

static TPDrv_t * tpDrv = &theTpDrv;
static I2C_t *                i2c;
//static i2cBitbang *         i2cBB;
static cmd_Version_Info_t   ver;

static IQS5xx_t             theIqs5xx;
static IQS5xx_t *           pDev = &theIqs5xx;

////////////////////////////////////////////////////////////////////////////////
/// Create an IQS5xx object.
/// \param rdy: GPIO port for RDY interrupt
/// \return pointer to object constructed.
////////////////////////////////////////////////////////////////////////////////
TPDrv_t * IQS5xx(uint8_t rdy)
{
    i2c   = I2CDev(I2CM_SPEED_400KHZ, TP_I2C_ADDRESS);
    //i2cBB = i2cBitbang(GPIO_SCL, GPIO_SDA, TPBL_I2C_ADDRESS);
    pDev->rdyGPIO = rdy;
    pDev->rpt.count = 0;
    pDev->rpt.reportID = RPT_ID_IN_ABS_XY;

    return tpDrv;
}

////////////////////////////////////////////////////////////////////////////////
/// HW reset
////////////////////////////////////////////////////////////////////////////////
void hwReset()
{
    // toggle reset pin for hardware reset
    wiced_hal_gpio_configure_pin(GPIO_RSTN_TP, GPIO_OUTPUT_ENABLE, GPIO_TOUCHPAD_OFF);
    TP_RSTN_ReleaseReset();
}


////////////////////////////////////////////////////////////////////////////////
/// get the Info filed of the TP data from the chip
////////////////////////////////////////////////////////////////////////////////
uint8_t getInfo()
{
    return  pDev->rpt.xyData.info;
}

////////////////////////////////////////////////////////////////////////////////
/// Returns whether the data interrupt is due to a proximity
////////////////////////////////////////////////////////////////////////////////
uint8_t proximityRpt()
{
    return !(pDev->rpt.xyData.info);
}

////////////////////////////////////////////////////////////////////////////////
/// regturn finger count
////////////////////////////////////////////////////////////////////////////////
uint8_t fingerCount()
{
    return  pDev->rpt.xyData.info & NO_OF_FINGER_MASK;
}

////////////////////////////////////////////////////////////////////////////////
/// get a pointer to the most recent report
////////////////////////////////////////////////////////////////////////////////
AbsXYReport * getRpt()
{
    return &(pDev->rpt);
}

////////////////////////////////////////////////////////////////////////////////
/// clear the TP data last retrieved
////////////////////////////////////////////////////////////////////////////////
void clearData()
{
    pDev->rpt.xyData.info = 0;
}


////////////////////////////////////////////////////////////////////////////////
/// Read XY data from the TP chip
///   sub_address is the register address.
///   Returns TRUE if the read is successful.
///           FALSE if the read has failed.
////////////////////////////////////////////////////////////////////////////////
uint8_t readXYData()
{
    return wakeUp() && i2c->ReadBlock(XY_DATA_REG, sizeof(Cmd_XY_Data_t), (uint8_t *) &pDev->rpt.xyData);
}

////////////////////////////////////////////////////////////////////////////////
/// clear the finger data to one that indicates no finger detected
/// \param finger: The findger data to be cleared
////////////////////////////////////////////////////////////////////////////////
void clearFingerData(XY_Data_finger_t * finger)
{
    memset(finger, 0xff, sizeof(XY_Data_finger_t));
    finger->tagId = finger->Strength_H = finger->Strength_L = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Wait for RDY line to go to a specified level.
/// \param logic: the logic level of the RDY line
/// \param waitMs: msec to wait for
/// \return Whether line has gone to the desired level within the wait time
////////////////////////////////////////////////////////////////////////////////
uint8_t waitForRDY(uint8_t logic, uint32_t waitMs)
{
    // convert ms to loop count
    #define MS_LOOP_CNT 816
    uint32_t cnt = waitMs * MS_LOOP_CNT;

    while ((wiced_hal_gpio_get_pin_input_status(pDev->rdyGPIO) ? INTR_LVL_HIGH : INTR_LVL_LOW)  != logic)
    {
        if (!--cnt)
        {
            return 0;
        }
    }
    return 1;
}

////////////////////////////////////////////////////////////////////////////////
// Wait for RDY line goes low and goes back high again
////////////////////////////////////////////////////////////////////////////////
static uint8_t waitForRDY_LowHigh(uint16_t wLo, uint16_t wHi)
{
    return waitForRDY(INTR_LVL_LOW, wLo) && waitForRDY(INTR_LVL_HIGH, wHi);
}

////////////////////////////////////////////////////////////////////////////////
// Write multiple set of data to TP chip.
// Each set of data consists of one byte Len and Len bytes of data.
// Data sets ends with a single byte of 0.
// RDY line must be high when calling this routine
/// \param ptr: pointer to the data sets to be written.
/// \return Whether all data has been written successfully.
////////////////////////////////////////////////////////////////////////////////
static uint8_t writeData(uint8_t * ptr)
{
    uint8_t len = *ptr++;

    while (len)
    {
        if (!i2c->WriteBlock(len, ptr))
        {
            break;
        }

        ptr += len;
        len = *ptr++;

        if (len && !waitForRDY_LowHigh(100, 500))
        {
            break;
        }
    }
    return !len;
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize the TP chip by writing the init/config data sets to the chip.
/// \return Whether epointer to object constructed.
////////////////////////////////////////////////////////////////////////////////
uint8_t init()
{
    if (!waitForRDY(INTR_LVL_HIGH, 100))
    {
        return 0;
    }

    return writeData(initData);
}

////////////////////////////////////////////////////////////////////////////////
/// Shutdown the TP chip
/// \return Whether all shutdown cmd sequence is successfully written out.
////////////////////////////////////////////////////////////////////////////////
uint8_t shutdown()
{
    uint8_t shutDownCmd11[] = {0x11, 0x0A, 0x05, 0x07, 0xEF, 0x00, 0x64, 0x0A, 0x05, 0x07};
    uint8_t shutDownCmd14[] = {0x14, 0x50, 0x64, 0x08, 0x0D, 0x03};

    return (wakeUp() && i2c->WriteBlock(sizeof(shutDownCmd11), shutDownCmd11) && waitForRDY(INTR_LVL_LOW, 50) &&
            wakeUp() && i2c->WriteBlock(sizeof(shutDownCmd14), shutDownCmd14));
}

////////////////////////////////////////////////////////////////////////////////
/// Wakeup TP chip by writing read cmd to the chip.
/// \return Whether chip has woken up within a time limit.
////////////////////////////////////////////////////////////////////////////////
static uint8_t wakeUp()
{
    uint16_t waitCnt;
    uint8_t data;
    // if RDY already high, no need to wake it up
    if (wiced_hal_gpio_get_pin_input_status(pDev->rdyGPIO))
        return 1;

    waitCnt = 200;
    // wait until touchpad responds
    while (!i2c->ReadBlock(WAKEUP_ADDR, 1, &data) && waitCnt)
    {
        waitCnt--;
    }

    // if device responds
    if (waitCnt)
    {
        //return waitForRDY(INTR_LVL_HIGH, 20);
        return waitForRDY(INTR_LVL_HIGH, 80);
    }
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Read TP chip's FW versikon.
/// \param buff: buffer to contain fw version, must be at least 2 byte long
/// \return Whether read is successful
////////////////////////////////////////////////////////////////////////////////
uint8_t readFirmwareVersion(uint8_t * buff)
{
    if (wakeUp() && i2c->ReadBlock(VERSION_INFO_REG, sizeof(cmd_Version_Info_t),(uint8_t *) &ver))
    {
        buff[0] = ver.major;
        buff[1] = ver.minor;
        return 1;
    }
    return 0;
}

#endif // SUPPORT_TOUCHPAD
