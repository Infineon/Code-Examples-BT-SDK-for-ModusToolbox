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
 * This file implements Infrared (IR) Transmit feature
 *
 */
#ifdef SUPPORT_IR

#include "clock_timer.h"
#include "irtxdriver.h"
#include "aclk.h"
#include "bleapp_appirtx.h"

#define cr_mia_clk_cfg_adr                             0x00320230                                                   // pmu_adr_base + 0x00000230
#define cr_mia_clk_cfg                                 (*(volatile unsigned int *)cr_mia_clk_cfg_adr)

#define    MIA_CLK_1M_NATIVE_MASK      0x1000
#define    MIA_CLK_1M_NATIVE_DISABLE   0x0000  // default
#define    MIA_CLK_1M_NATIVE_ENABLE    0x1000

#define APPIR_HIGH             0x8000      // bit15=1, bit 15 set to one to indicate logic high in data duation
#define APPIR_LOW              0            // bit15=0, bit 15 set to zero to indicate logic low in data duation

#define APPIRTX_DATA_SIZE  128

enum
{
    APPIRTX_IDLE = 0,
    APPIRTX_STARTED,
    APPIRTX_REPEAT,
    APPIRTX_END
};

uint32_t appir_cycle = 108000;   // 107.900 ms for the entire transaction
uint16_t appir_carrier_freq = 38000;
uint16_t appir_one = 1680;       // 1.68 ms for one to stay low
uint16_t appir_zero = 560;       // 560 us for zero to stay low
uint16_t appir_separator = 560;  // 560 us for the separator
uint32_t appirtx_mia_clock_1M_restore;

IR_TX_CLOCK_SETTING irTxClkSetting;
uint16_t   ir_data[APPIRTX_DATA_SIZE];
uint32_t  irIndex;
uint8_t irTxStarted;
uint8_t repeatFlag;
uint8_t  repeatCount;

OSAPI_TIMER appirtx_timer;

void appirtx_init(void);
void appirtx_sendIR(uint8_t code, uint8_t repeat, uint8_t repeatcount);
void appirtx_stopRepeat(void);
void appirtx_irTxTimerCallback(INT32 args, UINT32 overTimeInUs);
void appirtx_writebyte(uint8_t code);
uint8_t appirtx_isActive(void);
void appirtx_irBtClockEnable(uint8_t enable);

tIrTxDriverVtbl irTxDriverVtbl =
{
    appirtx_sendIR,
    appirtx_isActive,
    appirtx_stopRepeat,
};

///////////////////////////////////////////////////////////////////////////
///   Initialize the appIRtx driver
////////////////////////////////////////////////////////////////////////////
tIrTxDriverVtbl* appIRtx_appIRtx(uint8_t port, uint8_t pin)
{
    irtx_init();
    osapi_createTimer(&appirtx_timer, appirtx_irTxTimerCallback, 0);
    irtx_setIrTxPortPin(port, pin);

    irTxStarted = APPIRTX_IDLE;

    return &irTxDriverVtbl;
}

////////////////////////////////////////////////////////////////
///  This function initializes the appIRtx Driver.
////////////////////////////////////////////////////////////////
void appirtx_init(void)
{
    irTxClkSetting.clockSrc     = ACLK0;
    // Need to use 24MHz couck source, 1MHz cannot make 37.9kHz
    irTxClkSetting.clockSrcFreq = ACLK_FREQ_24_MHZ;

    irTxClkSetting.extendedSettings = 0;
    irTxClkSetting.pwmDutyCycleHighCount = 1;
    irTxClkSetting.pwmDutyCycleLowCount = 1;

    irTxClkSetting.invertOutput = FALSE;
    irTxClkSetting.modulateFreq = appir_carrier_freq;

    irTxStarted = APPIRTX_IDLE;
}

////////////////////////////////////////////////////////////////
///  This function indicates if IR Tx is active or idle.
///  Return: TRUE - active; FALSE -idle
////////////////////////////////////////////////////////////////
uint8_t appirtx_isActive(void)
{
    return (irTxStarted ? TRUE : FALSE);
}

////////////////////////////////////////////////////////////////
///  This function set the repeatFlag to false.
////////////////////////////////////////////////////////////////
void appirtx_stopRepeat(void)
{
    repeatFlag = 0;
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
void appirtx_sendIR(uint8_t code, uint8_t repeat, uint8_t repeatcount)
{
    // IR lead signal length
    uint16_t irLeadLength[2] = { 9000, 4500 };

    //previoud IR sending is not finished yet.
    if (irTxStarted)
        return;

    // something is wrong. reset IR.
    if( !irtx_isAvailable() )
    {
        irtx_abortCurrentTransaction();
    }

    appirtx_init();

    irIndex = 0;

    ir_data[irIndex++] = APPIR_HIGH + irLeadLength[0];
    ir_data[irIndex++] = APPIR_LOW  + irLeadLength[1];
    ir_data[irIndex++] = APPIR_HIGH + appir_separator;      // separator .56ms

    repeatFlag = repeat;
    repeatCount = repeatcount;

    appirtx_writebyte(code);
    appirtx_writebyte(~code);

    ir_data[irIndex++] = APPIR_LOW+ appir_separator;  // bring logic low

    appirtx_irBtClockEnable(TRUE);
    irTxStarted = APPIRTX_STARTED;
    appirtx_irTxTimerCallback(0, 0);
}


/////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////
void appirtx_writebyte(uint8_t code)
{
    uint8_t bit=1;

    while (bit)
    {
        ir_data[irIndex++] = APPIR_HIGH+ appir_separator; // separator .56ms
        if (bit & code)
        {
            ir_data[irIndex++] = APPIR_LOW+ appir_one; // stay low for 1.69 ms for logic one
        }
        else
        {
            ir_data[irIndex++] = APPIR_LOW+ appir_zero; // stay low for 0.56 ms for logic zero
        }
        bit <<=1;
    }
}

/////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////
void appirtx_irTxTimerCallback(INT32 args, UINT32 overTimeInUs)
{
    switch (irTxStarted)
    {
        case APPIRTX_STARTED:
            irtx_sendData(ir_data, irIndex, irTxClkSetting);
            if( repeatFlag || repeatCount)
            {
                irTxStarted = APPIRTX_REPEAT;

                osapi_activateTimer( &appirtx_timer, appir_cycle);
            }
            else
            {
                irTxStarted = APPIRTX_IDLE;
            }
            break;

        case APPIRTX_REPEAT:
            if (repeatCount)
            {
                repeatCount--;

                irtx_sendData(ir_data, irIndex, irTxClkSetting);

                if (!repeatCount && !repeatFlag)
                {
                    irTxStarted = APPIRTX_END;
                }
                osapi_activateTimer( &appirtx_timer, appir_cycle);
            }
            else if (repeatFlag)
            {
                irtx_sendData(ir_data, irIndex, irTxClkSetting);

                osapi_activateTimer( &appirtx_timer, appir_cycle);
            }
            else
            {
                irTxStarted = APPIRTX_IDLE;
            }

            break;

        case APPIRTX_END:
            irTxStarted = APPIRTX_IDLE;
            break;

        default:
            break;
    }

    if(irTxStarted == APPIRTX_IDLE)
    {
        appirtx_irBtClockEnable(FALSE);
    }
}

extern void bcs_pmuWaitForBtClock(void);
extern void bcs_pmuReleaseBtClock(void);
/////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////
void appirtx_irBtClockEnable(uint8_t enable)
{
    if(enable)
    {
        appirtx_mia_clock_1M_restore = cr_mia_clk_cfg & MIA_CLK_1M_NATIVE_MASK;
        cr_mia_clk_cfg |= MIA_CLK_1M_NATIVE_ENABLE;
        bcs_pmuWaitForBtClock();
    }
    else
    {
        cr_mia_clk_cfg &= appirtx_mia_clock_1M_restore | ~MIA_CLK_1M_NATIVE_ENABLE;
        bcs_pmuReleaseBtClock();
    }

}

#endif // SUPPORT_IR
