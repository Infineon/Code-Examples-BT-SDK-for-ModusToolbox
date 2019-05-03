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
 * BLE Mouse
 *
 * This file provides definitions and function prototypes for BLE mouse
 * device
 *
 */

#ifndef _BLEMOUSE_H_
#define _BLEMOUSE_H_

#include "wiced_hidd_lib.h"
#include "blehidlink.h"
#include "blehostlist.h"

#define MOUSE_REPORT_ID     2
#define BATTERY_REPORT_ID   3
#define MOUSE_REPORT_SIZE   5

//defined the maximum number of different client configuration notifications
#define MAX_NUM_CLIENT_CONFIG_NOTIF     8

#define MOUSEAPP_CLIENT_CONFIG_NOTIF_BOOT_RPT       (0x01)
#define MOUSEAPP_CLIENT_CONFIG_NOTIF_STD_RPT        (0x02)
#define MOUSEAPP_CLIENT_CONFIG_NOTIF_BATTERY_RPT    (0x04)
#define MOUSEAPP_CLIENT_CONFIG_NOTIF_NONE           (0)


#pragma pack(1)

/// Mouse App config
typedef PACKED struct
{
    /// Mouse motion report ID
    uint8_t motionReportID;

    /// Mouse motion report size in boot mode
    uint8_t motionReportBootModeSize;

    /// Mouse motion report size in report mode
    uint8_t motionReportReportModeSize;

    /// Connect button mask.
    uint16_t connectButtonMask;

    /// Certain high resolution sensors keep track of more motion than can fit in a byte
    /// but only report it one byte at a time. Such sensors may have to be polled
    /// multiple times to get all the motion. This specifies how many times the
    /// a sensor can be polled. Note that this is a max. value. The sensor may
    /// be polled fewer times if reports no motion (either through a motion line
    /// or because the last motion value is 0).
    uint8_t maxNumXYReadsPerPoll;

    /// Flag indicating whether X/Y from the sensor should be swapped
    uint8_t swapXY;

    /// Flags indicating X data from the sensor should be negated.
    /// Note that X negation is done after swapping.
    uint8_t negateX;
    /// Flags indicating Y data from the sensor should be negated.
    /// Note that Y negation is done after swapping.
    uint8_t negateY;

    /// Flag indicating that scroll data should be negated
    uint8_t negateScroll;

    /// Scale values for X motion data. Should be set to zero if no scaling is desired
    /// Scaling always divides the input data, i.e. its a shift right.
    uint8_t xScale;
    /// Scale values for Y motion data. Should be set to zero if no scaling is desired
    /// Scaling always divides the input data, i.e. its a shift right.
    uint8_t yScale;
    /// Scale values for scroll wheel data  . Should be set to zero if no scaling is desired
    /// Scaling always divides the input data, i.e. its a shift right.
    uint8_t scrollScale;

    /// Maximum number of ticks for which fractional motion data is kept, i.e. if no
    /// additional motion is detected, remaining fractional data is discarded. If set
    /// to 0, data is never discarded
    uint8_t pollsToKeepFracXYData;

    /// Maximum number of ticks for which fractional scroll wheel motion data is kept,
    /// i.e. if no additional motion is detected, remaining fractional data is discarded.
    /// If set to 0, data is never discarded. If scroll scaling is not used, should be set to
    /// 0 to improve execution efficiency.
    uint8_t pollsToKeepFracScrollData;

    /// Flag to enable report combining. If enabled, multiple motion events of the same type
    /// will be combined by the FW into a single report. Note that this only works for motion/scroll
    /// data and will not combine motion and scroll events together. Also note that events detected
    /// in the same poll period are always combined into a single mouse report
    /// regardless of the value of this flag.
    uint8_t eventCombining;

    /// Size of each element in the event queue. Note: This has to be at least as large as the
    /// largest event that the app will handle
    uint8_t maxEventSize;

    /// Maximum number of events that the event queue can hold.
    uint8_t maxEventNum;
}MouseAppConfig;


/// Boot mode mouse report
typedef struct
{
    /// HID report ID
    uint8_t    reportID;
    /// Button state in bitmap
    uint8_t    buttonState;
    /// The accumulcatd X motion of sensor
    int8_t    xMotion;
    /// The accumulcatd Y motion of sensor
    int8_t    yMotion;
    /// The scroll wheel motion
    int8_t    scroll;
    /// The pan movement. It is not used by most of mice
    int8_t    pan;
}BootModeReport;


/// Report mode mouse report with 12 bit X/Y
typedef struct
{
    /// HID report ID
    uint8_t    reportID;
    /// Button state in bitmap
    uint8_t    buttonState;

    /// X motion lower 8 bits
    uint8_t    xMotionLow;

    /// X motion upper 4 bits
    uint8_t    xMotionHigh:4;

    /// Y motion lower 4 bits
    uint8_t    yMotionLow:4;

    /// Y motion upper 8 bits
    uint8_t    yMotionHigh;

    /// The scroll wheel motion
    int8_t    scroll;

    /// Reserved area for extending the report mode report
    uint8_t    reserved[15];
}ReportModeReport;

/// Battery key report structure
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    uint8_t    level[1];
}MouseBatteryReport;
#pragma pack()

enum
{
    /// Max X/Y value in report mode
    MAX_REPORT_X_Y = 2047,

    /// Max X/Y value in boot mode. Note that this value is divided by two before being
    /// reported
    MAX_BOOT_X_Y = 254,

    /// Max scroll value
    MAX_SCROLL = 127
};

enum
{
    /// Pin code size
    PIN_SIZE = 4
};


/// Connect button state
typedef enum
{
    CONNECT_BUTTON_UP,
    CONNECT_BUTTON_DOWN
}ConnectButtonPosition;


void blemouseapp_create(void);
void mouseapp_init(void);
void mouseapp_shutdown(void);
void mouseapp_stateChangeNotification(uint32_t newState);
void mouseapp_pollReportUserActivity(void);
int16_t mouseapp_scaleValue(int16_t *val, uint8_t scaleFactor);
void mouseapp_pollActivityXYSensor(void);
void mouseapp_pollActivityScroll(void);
uint8_t mouseapp_pollActivityUser(void);
void mouseapp_generateAndTxReports(void);
void mouseapp_userKeyPressDetected(void *MApp);
void mouseapp_userScrollDetected(void* unused);
void mouseapp_userMotionXYDetected(void* unused, uint8_t port);
void mouseapp_generateReportSet(void);
int16_t mouseapp_limitRange(int16_t input, int16_t limit);
void mouseapp_batRptSend(void);
void mouseapp_createAndTxBootModeReport(void);
void mouseapp_createAndTxReportModeReport(void);
void mouseapp_createAndTxReport(void);
void mouseapp_txReportSet(void);
void mouseapp_pollActivityButton(void);
void mouseapp_connectButtonHandler(ConnectButtonPosition connectButtonPosition);
void mouseapp_connectButtonPressed(void);
void mouseapp_clientConfWriteBatteryRpt(wiced_hidd_report_type_t reportType,
                                     uint8_t reportId,
                                     void *payload,
                                     uint16_t payloadSize);
void mouseapp_ctrlPointWrite(wiced_hidd_report_type_t reportType,
                             uint8_t reportId,
                             void *payload,
                             uint16_t payloadSize);
void mouseapp_setProtocol(wiced_hidd_report_type_t reportType,
                          uint8_t reportId,
                          void *payload,
                          uint16_t payloadSize);
void mouseapp_clientConfWriteRptStd(wiced_hidd_report_type_t reportType,
                                    uint8_t reportId,
                                    void *payload,
                                    uint16_t payloadSize);
void mouseapp_clientConfWriteBootMode(wiced_hidd_report_type_t reportType,
                                   uint8_t reportId,
                                   void *payload,
                                      uint16_t payloadSize);
void mouseapp_updateClientConfFlags(uint16_t enable, uint16_t featureBit);
void mouseapp_updateGattMapWithNotifications(uint16_t flags);
void mouseapp_ResetGattDB(void);
void mouseapp_clearAllReports(void);
void mouseapp_flushUserInput(void);
void mouseapp_batLevelChangeNotification(uint32_t newLevel);
uint32_t mouseapp_sleep_handler(wiced_sleep_poll_type_t type );
void mouseapp_aon_restore(void);

//////////////////////////////////////////////////////////////////////////////
//                      public data type definition.
//////////////////////////////////////////////////////////////////////////////
#define MOTION_FIFO_CNT     100
typedef struct
{
/// Current accumulated motion values
    int16_t   mouseapp_xMotion;
    int16_t   mouseapp_yMotion;
    int16_t   mouseapp_scroll;

/// Fractional motion/scroll data. Only valid if scaling is in use.
    int16_t   mouseapp_xFractional;
    int16_t   mouseapp_yFractional;
    int16_t   mouseapp_scrollFractional;

/// Tick count of how many polls have gone by since motion data was updated
    uint8_t    mouseapp_pollsSinceXYMotion;
    uint8_t    mouseapp_pollsSinceScroll;

/// Flag to indicate that new reportable data is available.
    uint8_t    mouseapp_reportableDataInReportSet;

/// Boot mode motion report
    BootModeReport mouseapp_bootModeReport;

/// Report mode motion report
    ReportModeReport mouseapp_reportModeReport;

/// Battery level report
    MouseBatteryReport     mouseapp_batRpt;

// Mouse events

/// Button events
    HidEventButtonStateChange mouseapp_buttonEvent;

/// XY motion events
    HidEventMotionXY mouseapp_xyMotionEvent[MOTION_FIFO_CNT];

/// Scroll events
    HidEventMotionSingleAxis mouseapp_scrollEvent;

// Pin code entry parameter

/// Pin code buffer
    uint8_t mouseapp_pinCodeBuffer[PIN_SIZE];

    uint8_t mouseapp_pollSeqn;

/// The event queue for use by mouse app.
    wiced_hidd_app_event_queue_t mouseappEventQueue;

    /// Size of the Battery report.
    uint8_t mouseapp_batRptSize;

    uint8_t motion_fifo_in;

    uint8_t keyInterrupt_On;
    uint8_t allowSDS;

} tMouseAppState;

#endif
