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
 * TouchPad interface and definitions
 *
 */

#ifdef SUPPORT_TOUCHPAD

#ifndef __TOUCHPAD_H__
#define __TOUCHPAD_H__

#include "../interrupt.h"
#include "appDefs.h"
#include "IQS5xx.h"
#include "wiced.h"

#define TP_INTERRUPT_GPIO       GPIO_ATTN_TP
#define TOUCHPAD_FIFO_CNT       46
#define MAX_TP_EVENT_DELAY_IN_MS    100
#define STUCK_FINGER_TIMEOUT_mS     200 // 200ms
enum {
    ZONE_CENTER = 0,
    ZONE_RIGHT,
    ZONE_LEFT,
    ZONE_DOWN,
    ZONE_UP,
    ZONE_MAX_COUNT,
    ZONE_UNDEFINED = ZONE_MAX_COUNT,
};

enum {
    CLICK_NONE,
    CLICK_LEFT,
    CLICK_RIGHT,
    CLICK_THRESHOLD = 0x100,
};

#pragma pack(1)
typedef PACKED struct {
    uint16_t x, y;
} Pos_t, * PosPtr;

typedef PACKED struct {
    Pos_t p1, p2;
} Area_t, * AreaPtr;

#define SIZE_OF_TP_CFG_DATA   (ZONE_MAX_COUNT*8+MAX_INIT_DATA_SIZE)
typedef PACKED struct
{
    /// Base event info
    HidEvent eventInfo;

    /// user data
    void * userDataPtr;
} HidEventTouchpad;
#pragma pack()


typedef struct
{
    void  (*initialize)();  // should be called only if POR
    void  (*reInitialize)();
    void  (*shutdown)();
    void  (*clearEvent)();
    void  (*setEnable)(uint8_t en);
    uint8_t  (*getInfo)();
    uint8_t  (*isActive)();
    uint8_t  (*isFunctional)();
    uint8_t (*getPinActive)();
    uint8_t  (*fingerCount)();
    uint8_t  (*waitForEvent)(uint32_t waitInMs);
    uint8_t  (*getZone)();
    uint8_t  (*getLeftRight)();
    uint8_t  (*wakeUp)();
    uint8_t  (*readFirmwareVersion)(uint8_t * buff);
    uint8_t  (*proximityRpt)();
    AbsXYRptPtr (*getAbsFingerUpRpt)();

    uint8_t  (*pollActivity)(HidEventTouchpad* evtPtr, uint8_t ignore);
} TouchPadIf_t;

TouchPadIf_t * TouchPad(void (*callBack)(void*, uint8_t), void * parent);


typedef struct
{
    uint8_t verifiedWorking;
    uint8_t tpSuspend;
    AbsXYReport tpData[TOUCHPAD_FIFO_CNT];
    uint8_t        fifoIndex;
#ifdef HANDLE_STUCK_FINGER
    wiced_timer_t stuckFingerTimer;
    uint8_t          fingerStuckTimerOn; // local varible to save calls to wiced_xxx_timer()
    uint8_t          tpActive;
#endif
} TouchPad_t;
#endif

#endif // SUPPORT_TOUCHPAD
