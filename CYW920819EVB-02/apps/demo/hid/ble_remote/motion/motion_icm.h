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
 * Motion Sensor Interface definitions
 *
 */

#ifdef SUPPORT_MOTION

#ifndef __MOTION_ICM_H__
#define __MOTION_ICM_H__
#include "hidevent.h"
#include "../interrupt.h"                  // Use Interrupt class
#include "obj_secret/AIR_MOTION_Lib.h"  // We use air mouse liberary

#define MOTION_ICM_FIFO_CNT 50

#define SENSOR_DISABLED 0
#define SENSOR_ENABLED  1
#define SENSOR_SHUTDOWN 2
typedef uint8_t MotionSensorICM_mode_t;

#pragma pack(1)
#ifdef USE_MOTION_AS_AIR_MOUSE
/// Data mouse with 8 bit X/Y
typedef PACKED struct
{
//    uint8_t    reportID;   // this does not need to be sent
    /// Button state in bitmap
    uint8_t    buttonState;
    /// X motion 8 bits
    uint8_t    xMotion;
    /// Y motion 8 bits
    uint8_t    yMotion;
    /// The scroll wheel motion
    int8_t    scroll;
} MotionSensorICM_Mouse8bitXY_t;

#define MOTIONRPT_MAX_DATA            sizeof(MotionSensorICM_Mouse8bitXY_t)
#define SENSOR_REPORT_DATA(p)         (uint8_t*)&((p)->buttonState)
#define SENSOR_REPORT_DATA_SIZE(p)    (sizeof(MotionSensorICM_Mouse8bitXY_t) - sizeof((p)->reportID))  // repportID is not sent in LE
#else
typedef PACKED struct
{
    int16_t X;
    int16_t Y;
    int16_t Z;
} SensorAxis_t;

#define DUMMY_MOTION_DATA_SIZE       0
typedef PACKED struct
{
    SensorAxis_t GyroSamples;
    SensorAxis_t AccSamples;
#if DUMMY_MOTION_DATA_SIZE != 0 // dummy data to increase LE payload
    uint8_t dummyData[DUMMY_MOTION_DATA_SIZE];
#endif
} IcmRawMotionData_t;
#define MOTIONRPT_MAX_DATA    sizeof(IcmRawMotionData_t)
#endif
#pragma pack()

///////////////////////////////////////////////////////////////////////////////////////////////
// I have 2 base classes, Intr & pheriheral
///////////////////////////////////////////////////////////////////////////////////////////////
typedef struct MotionSensorICM_t
{
    void  (*enableInterrupt)     ();
    void  (*disableInterrupt)    ();
    void  (*setInterruptEnable)  (uint8_t en);
    void  (*clearInterrupt)      ();
    uint8_t (*isInterruptPending)  ();
    uint8_t (*isInterruptEnabled)  ();
    uint8_t (*isInterruptPinActive)();
    void  (*registerForInterrupt)(void (*userfn)(void*, uint8_t), void * objPtr);
    uint8_t (*pollActivity) (HidEventUserDefine *eventPtr);
    void  (*enable)    (uint8_t active);
    uint8_t (*isEnabled)     ();
    uint8_t (*isActive)     ();
    void  (*initialize)   ();
    void  (*shutdown)     ();
#ifdef USE_MOTION_AS_AIR_MOUSE
    void  (*setClickState)(BYTE newClickState);
#endif
} MotionSensorICM_c;


// Contructor
MotionSensorICM_c * CMotionSensorICM_CMotionSensorICM(void (*callBack)(void*, uint8_t), void * objPtr, uint8_t gpio, uint8_t activeLogic, uint16_t config);


#endif

#endif // SUPPORT_MOTION
