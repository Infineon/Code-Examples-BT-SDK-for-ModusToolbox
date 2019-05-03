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
 * BLE Keyboard
 *
 * This file provides definitions and function prototypes for BLE keyboard
 * device
 *
 */

#ifndef _BLEKB_H_
#define _BLEKB_H_

#include "wiced_hidd_lib.h"
#include "blehidlink.h"
#include "blehostlist.h"

/// Maximum number of keys supported by our HID
//#define KB_MAX_KEYS 144     // 8x18=144 ( 8x20 = 160 max )
#define KB_MAX_KEYS 56     // 7x8=56 ( 8x20 = 160 max )

#define KB_MAX_FUNC_LOCK_DEP_KEYS 24

/*******************************************************************************
* Types and Defines
*******************************************************************************/
#define NUM_KEYSCAN_ROWS    7    // Num of Rows in keyscan matrix
#define NUM_KEYSCAN_COLS    8    // Num of Cols in keyscan matrix

#define STD_KB_REPORT_ID            1
#define BITMAPPED_REPORT_ID         2
#define BATTERY_REPORT_ID           3
#define SLEEP_REPORT_ID             4
#define FUNC_LOCK_REPORT_ID         5
#define SCROLL_REPORT_ID            6


/// Maximum number of keys in a standard key report. Technically the report is
/// limited to 6 keys. A BLE ATT can hold 23 bytes. We'll
/// only use 6. The length of a non-boot mode report will be set through the config
/// record
#define KEYRPT_MAX_KEYS_IN_STD_REPORT    6
#define KEYRPT_BIT_MODIFIER_LEN          2
#define KEYRPT_LEN (KEYRPT_MAX_KEYS_IN_STD_REPORT+KEYRPT_BIT_MODIFIER_LEN)

/// Maximum number of bytes in the bit-mapped key report structure.
/// A BLE ATT can hold 23 bytes.
#define KEYRPT_NUM_BYTES_IN_BIT_MAPPED_REPORT   11

/// 11 bytes allow 88 keys in the bit mapped key report
#define KEYRPT_NUM_KEYS_IN_BIT_MAPPED_REPORT    ((KEYRPT_NUM_BYTES_IN_BIT_MAPPED_REPORT)*8)

/// Reserved bytes in motion report
#define KEYRPT_NUM_RESERVED_BYTES_IN_MOTION_REPORT  5

/// Rollover code
#define KEYRPT_CODE_ROLLOVER        0x01

//defined the maximum number of different client configuration notifications
#define MAX_NUM_CLIENT_CONFIG_NOTIF     16
//bit mask for different client configuration notifications
#define KBAPP_CLIENT_CONFIG_NOTIF_BOOT_RPT          (0x01)
#define KBAPP_CLIENT_CONFIG_NOTIF_STD_RPT           (0x02)
#define KBAPP_CLIENT_CONFIG_NOTIF_BIT_MAPPED_RPT    (0x04)
#define KBAPP_CLIENT_CONFIG_NOTIF_SLP_RPT           (0x08)
#define KBAPP_CLIENT_CONFIG_NOTIF_FUNC_LOCK_RPT     (0x10)
#define KBAPP_CLIENT_CONFIG_NOTIF_BATTERY_RPT       (0x20)
#define KBAPP_CLIENT_CONFIG_NOTIF_SCROLL_RPT        (0x40)

#define KBAPP_CLIENT_CONFIG_NOTIF_NONE              (0)

#ifdef LED_USE_PWM
#define BLEKB_PWM_LED1 28
#define BLEKB_PWM_LED2 29
#define BLEKB_PWM_LED_BASE 26
#define BLEKB_PWM_STEPS 500 //max 0x3ff=1023
#else
#define BLEKB_LED_CAPS 26
#define BLEKB_LED_BLUE 27
#define BLEKB_LED_GREEN 28
#define BLEKB_LED_RED 29
#endif

#pragma pack(1)
/// Keyboard application configuration
typedef struct
{
    /// ID of the standard report
    uint8_t stdRptID;

    /// Maximum number of keys in standard key report. Should be set to 6
    uint8_t maxKeysInStdRpt;

    /// Report ID for the bit mapped report
    uint8_t bitReportID;

    /// Number of bit mapped keys. Size of the bit report is automatically calculated from this value
    /// according to the following formula:
    ///     report size = ((num bit mapped keys) + 7)/8
    uint8_t numBitMappedKeys;

    /// Report ID for the bit mapped report
    uint8_t sleepReportID;

    /// Report ID of the pin entry report
    uint8_t pinReportID;

    /// Report ID of the LED (output) report
    uint8_t ledReportID;

    /// Default LED state. Note that the default implementation does not tie the LED value to physical LEDs
    uint8_t defaultLedState;

    /// Scan code of the connect button
    uint8_t connectButtonScanIndex;

    /// After an error has occurred, events from multiple poll cycles are combined to ensure that transient
    /// events are not generated. The count below specifies the recovery period in poll cycles.
    uint8_t recoveryPollCount;

    /// HW fifo threshold to stop generating idle rate reports. Idle rate report will be generated
    /// as long as the number of packets in the HW fifo is below this number
    uint8_t hwFifoThresholdForIdleRateReports;

    /// This parameter defines the rate at which a rollover report is generated when an error state (ghost or overflow) is
    /// maintained for long periods of time. The rate is in BT clock periods. If set to 0, it disables regeneration of
    /// the rollover report.
    uint16_t repeatRateInBTClocksForRolloverRpt;

    /// Rollover reports will only be repeated as long as the number of packets in the HW fifo is less than this threshold
    uint8_t hwFifoThresholdForRolloverRepeats;

    /// Report ID for func-lock reports
    uint8_t funcLockReportID;

    /// Default func lock state
    uint8_t defaultFuncLockState;

    /// Scroll report ID
    uint8_t scrollReportID;

    /// Length of scroll report
    uint8_t scrollReportLen;

    /// Negate scroll data.
    uint8_t negateScroll;

    /// Scale values for scroll wheel data. Should be set to zero if no scaling is desired
    /// Scaling always divides the input data, i.e. its a shift right.
    uint8_t scrollScale;

    /// Maximum number of ticks for which fractional scroll wheel motion data is kept,
    /// i.e. if no additional motion is detected, remaining fractional data is discarded.
    /// If set to 0, data is never discarded. If scroll scaling is not used, should be set to
    /// 0 to improve execution efficiency.
    uint8_t pollsToKeepFracScrollData;

    /// Flag indicating whether multiple scroll events should be combined into a single report
    /// Note that this will not combine any other type of event with scroll info
    uint8_t scrollCombining;

    /// Size of each element in the event queue. Note: This has to be at least as large as the
    /// largest event that the app will handle
    uint8_t maxEventSize;

    /// Maximum number of events that the event queue can hold.
    uint8_t maxEventNum;
}KbAppConfig;

/// Keyboard Key Config
typedef struct
{
    /// Type of key, e.g. std key, modifier key, etc.
    uint8_t    type;

    /// Translation code. The actual value depend on the key type.
    ///     - For modifier keys, it  is a bit mask for the reported key
    ///     - For std key, it is the usage provided in the std key report
    ///     - For bit mapped keys, it is the row/col of the associated bit in the bit mapped report
    uint8_t    translationValue;
}KbKeyConfig;
#pragma pack()

/*** ALL EXTERNAL REPORTS MUST BE PACKED!
 *** (and don't forget the pack() afterwards) **/
#pragma pack(1)

/// Standard key report structure
typedef PACKED struct
{
    /// Set to the value specified in the config record. 1 is recommended for boot-mode support
    uint8_t    reportID;

    /// Modifier keys
    uint8_t    modifierKeys;

    /// Reserved (OEM). Normally set to 0 unless changed by application code.
    uint8_t    reserved;

    /// Key array.
    uint8_t    keyCodes[KEYRPT_MAX_KEYS_IN_STD_REPORT];
}KeyboardStandardReport;

/// Battery key report structure
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    uint8_t    level[1];
}KeyboardBatteryReport;

/// Bit mapped key report structure
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    /// Bit mapped keys
    uint8_t    bitMappedKeys[KEYRPT_NUM_BYTES_IN_BIT_MAPPED_REPORT];
}KeyboardBitMappedReport;


/// Sleep report structure. Sent when sleep key press is detected.
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    /// Set to the value specified in the config record.
    uint8_t    sleepVal;
}KeyboardSleepReport;


/// Func lock report struct. Sent when the func lock key is pressed
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    /// Set to the value specified in the config record.
    uint8_t    status;
}KeyboardFuncLockReport;


/// Keyboard output report. Sets the LED state
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    /// State of various LEDs
    uint8_t    ledStates;
}KeyboardLedReport;


/// Scroll report structure. This can scroll wheels, track balls, and volume knobs.
/// This report allows for 3 axis, but most devices will only use one or 2
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    /// Motion along axis 0
    int16_t    motionAxis0;

    /// Motion along axis 1
    int16_t    motionAxis1;

    /// Motion along axis 2
    int16_t    motionAxis2;

    /// Reserved bytes. Can be used by application to extend motion report
    uint8_t    reserved[KEYRPT_NUM_RESERVED_BYTES_IN_MOTION_REPORT];
}KeyboardMotionReport;

/// Func-lock dependent key translation codes
typedef PACKED struct
{
    /// Bit mapped report handler translation code
    uint8_t bitRptCode;

    /// Standard report handler translation code
    uint8_t stdRptCode;
}KbFuncLockDepKeyTransTab;

#pragma pack()

/// Key types. Used to direct key codes to the relevant key processing function
enum KeyType
{
    /// Represents no key. This should not occur normally
    KEY_TYPE_NONE=0,

    /// Represents a standard key. The associated translation code represents the reported value
    /// of this key
    KEY_TYPE_STD=1,

    /// Represents a modifier key. The associated translation value indicates which bit
    /// in the modifier key mask is controlled by this key
    KEY_TYPE_MODIFIER=2,

    /// Represents a bit mapped key in the bit mapped report. The associated translation value
    /// provides the row col of the bit which represents this key
    KEY_TYPE_BIT_MAPPED=3,

    /// The sleep key
    KEY_TYPE_SLEEP=4,

    /// The function lock key
    KEY_TYPE_FUNC_LOCK=5,

    /// Function lock dependent keys. These keys act like bit mapped keys when function lock is on
    /// and standard keys when function lock is off or the keyboard is in boot mode.
    KEY_TYPE_FUNC_LOCK_DEP=6,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_USER_DEF_0=16,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_USER_DEF_1=17,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_USER_DEF_2=18,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_USER_DEF_3=19,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_USER_DEF_4=20,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_USER_DEF_5=21,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_USER_DEF_6=22,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_USER_DEF_7=23
};


/// KB USB usages
enum UsbUsage
{
    USB_USAGE_NO_EVENT=0,
    USB_USAGE_ROLLOVER=1,
    USB_USAGE_POST_FAIL=2,
    USB_USAGE_UNDEFINED_ERROR=3,

    USB_USAGE_A=4,
    USB_USAGE_B=5,
    USB_USAGE_C=6,
    USB_USAGE_D=7,
    USB_USAGE_E=8,
    USB_USAGE_F=9,
    USB_USAGE_G=10,
    USB_USAGE_H=11,
    USB_USAGE_I=12,
    USB_USAGE_J=13,
    USB_USAGE_K=14,
    USB_USAGE_L=15,
    USB_USAGE_M=16,
    USB_USAGE_N=17,
    USB_USAGE_O=18,
    USB_USAGE_P=19,
    USB_USAGE_Q=20,
    USB_USAGE_R=21,
    USB_USAGE_S=22,
    USB_USAGE_T=23,
    USB_USAGE_U=24,
    USB_USAGE_V=25,
    USB_USAGE_W=26,
    USB_USAGE_X=27,
    USB_USAGE_Y=28,
    USB_USAGE_Z=29,

    USB_USAGE_1=30,
    USB_USAGE_2=31,
    USB_USAGE_3=32,
    USB_USAGE_4=33,
    USB_USAGE_5=34,
    USB_USAGE_6=35,
    USB_USAGE_7=36,
    USB_USAGE_8=37,
    USB_USAGE_9=38,
    USB_USAGE_0=39,

    USB_USAGE_ENTER=40,
    USB_USAGE_ESCAPE=41,
    USB_USAGE_BACKSPACE=42,
    USB_USAGE_TAB=43,
    USB_USAGE_SPACEBAR=44,
    USB_USAGE_MINUS=45,
    USB_USAGE_EQUAL=46,
    USB_USAGE_LEFT_BRACKET=47,
    USB_USAGE_RIGHT_BRACKET=48,
    USB_USAGE_BACK_SLASH=49,

    USB_USAGE_NON_US_HASH=50,
    USB_USAGE_SEMICOLON=51,
    USB_USAGE_QUOTE=52,
    USB_USAGE_ACCENT=53,
    USB_USAGE_COMMA=54,
    USB_USAGE_STOP_AND_GREATER=55,
    USB_USAGE_SLASH=56,
    USB_USAGE_CAPS_LOCK=57,
    USB_USAGE_F1=58,
    USB_USAGE_F2=59,

    USB_USAGE_F3=60,
    USB_USAGE_F4=61,
    USB_USAGE_F5=62,
    USB_USAGE_F6=63,
    USB_USAGE_F7=64,
    USB_USAGE_F8=65,
    USB_USAGE_F9=66,
    USB_USAGE_F10=67,
    USB_USAGE_F11=68,
    USB_USAGE_F12=69,

    USB_USAGE_PRINT_SCREEN=70,
    USB_USAGE_SCROLL_LOCK=71,
    USB_USAGE_PAUSE=72,
    USB_USAGE_INSERT=73,
    USB_USAGE_HOME=74,
    USB_USAGE_PAGE_UP=75,
    USB_USAGE_DELETE=76,
    USB_USAGE_END=77,
    USB_USAGE_PAGE_DOWN=78,
    USB_USAGE_RIGHT_ARROW=79,

    USB_USAGE_LEFT_ARROW=80,
    USB_USAGE_DOWN_ARROW=81,
    USB_USAGE_UP_ARROW=82,
    USB_USAGE_NUM_LOCK=83,
    USB_USAGE_KP_SLASH=84,
    USB_USAGE_KP_ASTERISK=85,
    USB_USAGE_KP_MINUS=86,
    USB_USAGE_KP_PLUS=87,
    USB_USAGE_KP_ENTER=88,
    USB_USAGE_KP_1=89,

    USB_USAGE_KP_2=90,
    USB_USAGE_KP_3=91,
    USB_USAGE_KP_4=92,
    USB_USAGE_KP_5=93,
    USB_USAGE_KP_6=94,
    USB_USAGE_KP_7=95,
    USB_USAGE_KP_8=96,
    USB_USAGE_KP_9=97,
    USB_USAGE_KP_0=98,
    USB_USAGE_KP_DOT=99,

    USB_USAGE_NON_US_BACK_SLASH=100,
    USB_USAGE_APPLICATION=101,
    USB_USAGE_POWER=102,
    USB_USAGE_KP_EQUAL=103,
    USB_USAGE_F13=104,
    USB_USAGE_F14=105,
    USB_USAGE_F15=106,
    USB_USAGE_F16=107,
    USB_USAGE_F17=108,
    USB_USAGE_F18=109,

    USB_USAGE_F19=110,
    USB_USAGE_F20=111,
    USB_USAGE_F21=112,
    USB_USAGE_F22=113,
    USB_USAGE_F23=114,
    USB_USAGE_F24=115,
    USB_USAGE_EXECUTE=116,
    USB_USAGE_HELP=117,
    USB_USAGE_MENU=118,
    USB_USAGE_SELECT=119,

    USB_USAGE_STOP=120,
    USB_USAGE_AGAIN=121,
    USB_USAGE_UNDO=122,
    USB_USAGE_CUT=123,
    USB_USAGE_COPY=124,
    USB_USAGE_PASTE=125,
    USB_USAGE_FIND=126,
    USB_USAGE_MUTE=127,
    USB_USAGE_VOL_UP=128,
    USB_USAGE_VOL_DOWN=129,

    USB_USAGE_LOCKING_CAPS_LOCK=130,
    USB_USAGE_LOCKING_NUM_LOCK=131,
    USB_USAGE_LOCKING_SCROLL_LOCK=132,
    USB_USAGE_KP_COMMA=133,
    USB_USAGE_KP_EQUAL_AS400=134,
    USB_USAGE_INTL_1=135,
    USB_USAGE_INTL_2=136,
    USB_USAGE_INTL_3=137,
    USB_USAGE_INTL_4=138,
    USB_USAGE_INTL_5=139,

    USB_USAGE_INTL_6=140,
    USB_USAGE_INTL_7=141,
    USB_USAGE_INTL_8=142,
    USB_USAGE_INTL_9=143,
    USB_USAGE_LANG_1=144,
    USB_USAGE_LANG_2=145,
    USB_USAGE_LANG_3=146,
    USB_USAGE_LANG_4=147,
    USB_USAGE_LANG_5=148,
    USB_USAGE_LANG_6=149,

    USB_USAGE_LANG_7=150,
    USB_USAGE_LANG_8=151,
    USB_USAGE_LANG_9=152,
    USB_USAGE_ALT_ERASE=153,
    USB_USAGE_SYS_REQ=154,
    USB_USAGE_CANCEL=155,
    USB_USAGE_CLEAR=156,
    USB_USAGE_PRIOR=157,
    USB_USAGE_RETURN=158,
    USB_USAGE_SEPARATOR=159,

    USB_USAGE_OUT=160,
    USB_USAGE_OPER=161,
    USB_USAGE_CLEAR_AGAIN=162,
    USB_USAGE_CRSEL=163,
    USB_USAGE_EXSEL=164,

    // Reserved 165-175

    USB_USAGE_KP_00=176,
    USB_USAGE_KP_000=177,
    USB_USAGE_THOUSANDS_SEPERATOR=178,
    USB_USAGE_DECIMAL_SEPERATOR=179,

    USB_USAGE_CURRENCY_UNIT=180,
    USB_USAGE_CURRENCY_SUB_UNIT=181,
    USB_USAGE_KP_LEFT_PAREN=182,
    USB_USAGE_KP_RIGHT_PAREN=183,
    USB_USAGE_KP_LEFT_CURLY_BRACE=184,
    USB_USAGE_KP_RIGHT_CURLY_BRACE=185,
    USB_USAGE_KP_TAB=186,
    USB_USAGE_KP_BACKSPACE=187,
    USB_USAGE_KP_A=188,
    USB_USAGE_KP_B=189,

    USB_USAGE_KP_C=190,
    USB_USAGE_KP_D=191,
    USB_USAGE_KP_E=192,
    USB_USAGE_KP_F=193,
    USB_USAGE_KP_XOR=194,
    USB_USAGE_KP_CARET=195,
    USB_USAGE_KP_PERCENT=196,
    USB_USAGE_KP_LESS_THAN=197,
    USB_USAGE_KP_GREATER_THAN=198,
    USB_USAGE_KP_AMPERSAND=199,

    USB_USAGE_KP_DOUBLE_AMPERSAND=200,
    USB_USAGE_KP_VERTICAL_BAR=201,
    USB_USAGE_KP_DOUBLE_VERTICAL_BAR=202,
    USB_USAGE_KP_COLON=203,
    USB_USAGE_KP_HASH=204,
    USB_USAGE_KP_SPACE=205,
    USB_USAGE_KP_AT=206,
    USB_USAGE_KP_EXCLAMATION=207,
    USB_USAGE_KP_MEM_STORE=208,
    USB_USAGE_KP_MEM_RECALL=209,

    USB_USAGE_KP_MEM_CLEAR=210,
    USB_USAGE_KP_MEM_ADD=211,
    USB_USAGE_KP_MEM_SUBTRACT=212,
    USB_USAGE_KP_MEM_MULTIPLY=213,
    USB_USAGE_KP_MEM_DIVIDE=214,
    USB_USAGE_KP_PLUS_MINUS=215,
    USB_USAGE_KP_CLEAR=216,
    USB_USAGE_KP_CLEAR_ENTRY=217,
    USB_USAGE_KP_BINARY=218,
    USB_USAGE_KP_OCTAL=219,

    USB_USAGE_KP_DECIMAL=220,
    USB_USAGE_KP_HEX=221,
    // 222-223 reserved
    USB_USAGE_LEFT_CTL=224,
    USB_USAGE_LEFT_SHIFT=225,
    USB_USAGE_LEFT_ALT=226,
    USB_USAGE_LEFT_GUI=227,
    USB_USAGE_RIGHT_CTL=228,
    USB_USAGE_RIGHT_SHIFT=229,

    USB_USAGE_RIGHT_ALT=230,
    USB_USAGE_RIGHT_GUI=231
};

/// Modifier keys bit masks
enum
{
    USB_MODKEY_MASK_LEFT_CTL=0x01,
    USB_MODKEY_MASK_LEFT_SHIFT=0x02,
    USB_MODKEY_MASK_LEFT_ALT=0x04,
    USB_MODKEY_MASK_LEFT_GUI=0x08,
    USB_MODKEY_MASK_RIGHT_CTL=0x10,
    USB_MODKEY_MASK_RIGHT_SHIFT=0x20,
    USB_MODKEY_MASK_RIGHT_ALT=0x40,
    USB_MODKEY_MASK_RIGHT_GUI=0x80
};



/// Func lock state
enum FuncLockState
{
    /// Func lock is off
    FUNC_LOCK_STATE_OFF,

    /// Func lock is on
    FUNC_LOCK_STATE_ON
};


/// Func lock key state
typedef enum
{
    /// Func lock key is up
    FUNC_LOCK_KEY_UP,

    /// Func lock key is down
    FUNC_LOCK_KEY_DOWN
}FuncLockKeyPosition;


// bit define in SCROLL REPORT
enum
{
    SCROLL_REPORT_VOLUME_UP   =  0x01,
    SCROLL_REPORT_VOLUME_DOWN =  0x02,
};

/// Structure handling func-lock related data
typedef struct
{
    /// Current func-lock state. 0 is off, 1 is on
    uint8_t state;

    /// Current func-lock key state. 0 is up, 1 is down
    FuncLockKeyPosition kepPosition;

    /// Flag indicating whether func-lock should be toggled when the func-lock key goes up
    uint8_t toggleStateOnKeyUp;
}FuncLockInfo;

/// Connect button state
typedef enum
{
    CONNECT_BUTTON_UP,
    CONNECT_BUTTON_DOWN
}ConnectButtonPosition;


void blekbapp_create(void);
void kbapp_init(void);
void kbapp_shutdown(void);
void kbapp_pollReportUserActivity(void);
uint8_t kbapp_pollActivityUser(void);
void kbapp_pollActivityKey(void);
void kbapp_generateAndTxReports(void);
void kbapp_procEvtUserDefined(void);
void kbapp_stdErrResp(void);
void kbapp_stdRptRolloverSend(void);
void kbapp_procEvtKey(void);
void kbapp_procEvtScroll(void);
void kbapp_procEvtUserDefinedKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode);
void kbapp_stdErrRespWithFwHwReset(void);
void kbapp_procErrKeyscan(void);
void kbapp_procErrEvtQueue(void);
void kbapp_stdRptProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode);
void kbapp_stdRptProcEvtKeyDown(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode);
void kbapp_stdRptProcEvtKeyUp(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode);
void kbapp_stdRptProcOverflow(void);
void kbapp_stdRptProcEvtModKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode);
void kbapp_funcLockProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode);
void kbapp_funcLockProcEvtDepKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t funcLockDepKeyTableIndex);
void kbapp_slpRptProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t slpBitMask);
void kbapp_bitRptProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t rowCol);
void kbapp_txModifiedKeyReports(void);
void kbapp_scrollRptSend(void);
void kbapp_funcLockRptSend(void);
void kbapp_slpRptSend(void);
void kbapp_bitRptSend(void);
void kbapp_batRptSend(void);
void kbapp_stdRptSend(void);
void kbapp_setIdleRate(uint8_t idleRateIn4msUnits);

void kbapp_userKeyPressDetected(void* unused);
void kbapp_userScrollDetected(void* unused);
void kbapp_stateChangeNotification(uint32_t newState);
void kbapp_batLevelChangeNotification(uint32_t newLevel);
void kbapp_clearAllReports(void);
void kbapp_flushUserInput(void);
void kbapp_ledRptInit(void);
void kbapp_funcLockRptInit(void);
void kbapp_stdRptRolloverInit(void);
void kbapp_stdRptClear(void);
void kbapp_slpRptClear(void);
void kbapp_bitRptClear(void);
void kbapp_scrollRptClear(void);
void kbapp_funcLockToggle(void);

void kbapp_connectButtonHandler(ConnectButtonPosition connectButtonPosition);
void kbapp_connectButtonPressed(void);

void kbapp_setReport(wiced_hidd_report_type_t reportType,
                     uint8_t reportId,
                     void *payload,
                     uint16_t payloadSize);
void kbapp_ctrlPointWrite(wiced_hidd_report_type_t reportType,
                          uint8_t reportId,
                          void *payload,
                          uint16_t payloadSize);
void kbapp_clientConfWriteRptStd(wiced_hidd_report_type_t reportType,
                     uint8_t reportId,
                     void *payload,
                     uint16_t payloadSize);
void kbapp_clientConfWriteRptBitMapped(wiced_hidd_report_type_t reportType,
                     uint8_t reportId,
                     void *payload,
                     uint16_t payloadSize);
void kbapp_clientConfWriteRptSlp(wiced_hidd_report_type_t reportType,
                     uint8_t reportId,
                     void *payload,
                     uint16_t payloadSize);
void kbapp_clientConfWriteRptFuncLock(wiced_hidd_report_type_t reportType,
                     uint8_t reportId,
                     void *payload,
                     uint16_t payloadSize);
void kbapp_clientConfWriteScroll(wiced_hidd_report_type_t reportType,
                                      uint8_t reportId,
                                      void *payload,
                                      uint16_t payloadSize);
void kbapp_clientConfWriteBootMode(wiced_hidd_report_type_t reportType,
                                   uint8_t reportId,
                                   void *payload,
                                   uint16_t payloadSize);
void kbapp_setProtocol(wiced_hidd_report_type_t reportType,
                                   uint8_t reportId,
                                   void *payload,
                       uint16_t payloadSize);
void kbapp_clientConfWriteBatteryRpt(wiced_hidd_report_type_t reportType,
                                     uint8_t reportId,
                                     void *payload,
                                     uint16_t payloadSize);

void kbapp_updateClientConfFlags(uint16_t enable, uint16_t featureBit);
void kbapp_updateGattMapWithNotifications(uint16_t flags);
void kbapp_ResetGattDB(void);

void kbapp_pollActivityScroll(void);
int16_t kbapp_scaleValue(int16_t *val, uint8_t scaleFactor);
void kbapp_LED_init(void);
void kbapp_LED_on(uint8_t gpio);
void kbapp_LED_off(uint8_t gpio);
uint32_t kbapp_sleep_handler(wiced_sleep_poll_type_t type );
void kbapp_aon_restore(void);

//////////////////////////////////////////////////////////////////////////////
//                      public data type definition.
//////////////////////////////////////////////////////////////////////////////
typedef struct
{
/// Standard key report
    KeyboardStandardReport  stdRpt;

/// Battery level report
    KeyboardBatteryReport     batRpt;

/// Standard rollover report
    KeyboardStandardReport rolloverRpt;

/// Output LED report.
    KeyboardLedReport ledReport;

/// Bit mapped key report
    KeyboardBitMappedReport bitMappedReport;

/// Sleep key report
    KeyboardSleepReport slpRpt;

/// Keyboard scroll report
    KeyboardMotionReport scrollReport;

/// Func-lock report
    KeyboardFuncLockReport funcLockRpt;

// Report change flags

/// Flag indicating that the std report has been changed since it was last sent
    uint8_t stdRptChanged;

/// Flag indicating that the bit mapped report has been changed since it was last sent
    uint8_t bitRptChanged;

/// Flag indicating that the sleep report has changed since it was last sent
    uint8_t slpRptChanged;

/// Flag indicating that the scroll report has changed since it was last sent
    uint8_t scrollRptChanged;

/// Flag indicating that the func-lock report has changed since it was last sent
    uint8_t funcLockRptChanged;


// Additional report related attributes

/// Number of keys in the current standard report
    uint8_t keysInStdRpt;

/// Number of down modifier keys in the standard report
    uint8_t modKeysInStdRpt;

/// Size of the standard report. Arrived at by adding 4 bytes (report header, report ID,
/// modifier byte, and reserved byte) to the maximum number of keys in a standard report
    uint8_t stdRptSize;

/// Size of the Battery report.
    uint8_t batRptSize;

///  Idle rate in 4ms units
    uint8_t idleRate;

///  Idle rate in BT clocks
    uint32_t idleRateInBtClocks;

/// Native BT clock value when the standard report was transmitted. Used for idle rate
    uint32_t stdRptTxInstant;

/// Number of keys in the current bit mapped report
    uint8_t keysInBitRpt;

/// Size of the bit mapped report. Includes header and report ID
    uint8_t bitReportSize;

/// Func-lock information
    FuncLockInfo funcLockInfo;

/// Number of polls cycles left in the recovery period
    uint8_t recoveryInProgress;

// Event structures

/// Temporary used for creating key events
    HidEventKey keyEvent;

/// Temporary used for creating scroll events
    HidEventMotionSingleAxis scrollEvent;


// Scroll related attributes

/// Fractional scroll data. Only valid if scaling is in use.
    int16_t scrollFractional;

/// Tick count of how many polls have gone by since scroll data was updated
    uint8_t pollsSinceScroll;

/// The event queue for use by KB app.
    wiced_hidd_app_event_queue_t eventQueue;

    uint8_t pollSeqn;
    uint8_t keyInterrupt_On;

    uint8_t allowSDS;
} tKbAppState;

#endif
