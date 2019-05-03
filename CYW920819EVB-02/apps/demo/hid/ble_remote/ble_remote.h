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
 * BLE Remote control
 *
 * This file provides definitions and function prototypes for BLE remote control
 * device
 *
 */

#ifndef __BLEREMOTE_H__
#define __BLEREMOTE_H__
#ifdef SUPPORT_IR
#include "ir/bleapp_appirtx.h"
#endif
#ifdef ENABLE_ADC_AUDIO_ENHANCEMENTS
#include "adc.h"
#endif
#include "interrupt.h"
#ifdef SUPPORT_MOTION
#include "motion/motion_icm.h"
#endif
#include "wiced_hidd_micaudio.h"
#include "wiced_hidd_lib.h"
#include "blehidlink.h"
#include "blehostlist.h"

/*******************************************************************************
* Types and Defines
*******************************************************************************/

#define NUM_KEYSCAN_ROWS    5  // Num of Rows in keyscan matrix
#define NUM_KEYSCAN_COLS    4  // Num of Cols in keyscan matrix

#define STD_KB_REPORT_ID            1
#define BITMAPPED_REPORT_ID         2
#define BATTERY_REPORT_ID           3

/// Maximum number of keys supported by our HID
#define KB_MAX_KEYS 160

/// Maximum number of remote reports supported by our HID
#define REMOTE_MAX_USER_DEFINED_RPT_TYPE 8

/// Maximum size of remote translation code
#define REMOTE_MAX_TRANSLATION_CODE_SIZE 2

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


#define REMOTERPT_MAX_BYTES_IN_REPORT 8

/// Rollover code
#define KEYRPT_CODE_ROLLOVER        0x01

//defined the maximum number of different client configuration notifications
#define MAX_NUM_CLIENT_CONFIG_NOTIF     16
//bit mask for different client configuration notifications
#define KBAPP_CLIENT_CONFIG_NOTIF_STD_RPT           (0x02)
#define KBAPP_CLIENT_CONFIG_NOTIF_BIT_MAPPED_RPT    (0x04)
#define KBAPP_CLIENT_CONFIG_NOTIF_MOTION_RPT        (0x08)
#define KBAPP_CLIENT_CONFIG_NOTIF_USER_DEFINED_KEY_RPT      (0x10)
#define KBAPP_CLIENT_CONFIG_NOTIF_BATTERY_RPT       (0x20)
#define KBAPP_CLIENT_CONFIG_NOTIF_VOICE_RPT         (0x40)
#define KBAPP_CLIENT_CONFIG_NOTIF_VOICE_CTRL_RPT    (0x80)
#ifdef SUPPORT_TOUCHPAD
#define KBAPP_CLIENT_CONFIG_NOTIF_TOUCHPAD_RPT      (0x0100)
#endif

#define KBAPP_CLIENT_CONFIG_NOTIF_NONE              (0)

#pragma pack(1)
/// Keyboard application configuration
typedef PACKED struct
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
typedef PACKED struct
{
    /// Type of key, e.g. std key, modifier key, etc.
    uint8_t    type;

    /// Translation code. The actual value depend on the key type.
    ///     - For modifier keys, it  is a bit mask for the reported key
    ///     - For std key, it is the usage provided in the std key report
    ///     - For bit mapped keys, it is the row/col of the associated bit in the bit mapped report
    uint8_t    translationValue;
}KbKeyConfig;

/// Remote user defined report configuration
typedef PACKED struct
{
    /// Remote user defined report ID
    uint8_t rptID;

    /// Size of the key translation code
    uint8_t translationCodeSize;
}RemoteUserDefinedReportConfig;

/// Remote Key Translation Code
typedef PACKED struct
{
    uint8_t translationValue[REMOTE_MAX_TRANSLATION_CODE_SIZE];
}RemoteKeyTranslationCode;

/// Remote application configuration
typedef PACKED struct
{
    /// the default lpm index for mode "HIGH"
    uint8_t default_lpm_idx;

    /// the lpm index for motion
    uint8_t motion_lpm_idx;

    /// the lpm index for voice
    uint8_t voice_lpm_idx;

    /// report ID for motion data
    uint8_t motionRptID;

    /// Scan code of the IR button
    uint8_t IR_ButtonScanIndex;

    /// Scan code of the Motion START button
    uint8_t MotionStart_ButtonScanIndex;

    /// Scan code of the Motion STOP button
    uint8_t MotionStop_ButtonScanIndex;

    /// Scan code of the Voice button
    uint8_t Voice_ButtonScanIndex;

    /// delay sending audio time period in ms
    uint16_t  audio_delay;

    /// audio mode
    uint8_t    audio_mode;

    /// gain of the audio codec
    uint8_t    audio_gain;

    /// boost of the audio codec
    uint8_t    audio_boost;

    /// Maximum number of data bytes in remote report
    uint8_t maxBytesInRemoteRpt;

    /// Number of different typpes of remote reports
    uint8_t numOfRemoteRpt;

    /// Maximum sample number read in one slot callback
    uint8_t maxSampleInOneSlot;

    /// Remote user defined report configuration
    RemoteUserDefinedReportConfig remoteUserDefinedReportConfig[REMOTE_MAX_USER_DEFINED_RPT_TYPE];

    /// Remote Key Translation Code
    RemoteKeyTranslationCode remoteKeyTranslationCode[KB_MAX_KEYS];
}RemoteAppConfig;
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


/// Keyboard output report. Sets the LED state
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    /// State of various LEDs
    uint8_t    ledStates;
}KeyboardLedReport;


/// Remote report structure
typedef struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    /// Key array.
    uint8_t    keyCodes[REMOTERPT_MAX_BYTES_IN_REPORT];
}RemoteReport;
#pragma pack()


#ifdef SUPPORT_TOUCHPAD
#include "touchpad/touchPad.h"

#define NUM_ROWS                    5
#define NUM_COLS                    5  // 4 actual keyscan column + 1 extra column for touch pad virtual keys = 5 columns
#define TOUCHPAD_BUTTON_KEYINDEX    4 // ENTER
#define VKEY_INDEX_CENTER           TOUCHPAD_BUTTON_KEYINDEX
#define VKEY_INDEX_RIGHT            (NUM_ROWS * (NUM_COLS-1))
#define VKEY_INDEX_LEFT             (VKEY_INDEX_RIGHT+1)
#define VKEY_INDEX_DOWN             (VKEY_INDEX_RIGHT+2)
#define VKEY_INDEX_UP               (VKEY_INDEX_RIGHT+3)
#endif /* SUPPORT_TOUCHPAD */



#ifdef SUPPORTING_FINDME
#include "pwm.h"
#define FINDME_ALERT_TYPE        ALERT_BUZ_LED  // ALERT_LED, ALERT_BUZ

#define LED_ON  0
#define LED_OFF 1


// findMe BUZ alert config
typedef struct
{
    uint8_t freq;         // pwm freq
    uint16_t init_value;  // pwm init value
    uint16_t toggle_val;  // pwm toggle value
    uint16_t buz_on_ms;   // buz on duration
    uint16_t buz_off_ms;  // buz on duration
    uint16_t repeat_num;  // repeat num
}AppBuzAlertConfig;

// findMe LED alert config
typedef struct
{
    uint16_t led_on_ms;    // led on duration
    uint16_t led_off_ms;   // led on duration
    uint16_t repeat_num;   // repeat num
}AppLedAlertConfig;


//Find me Alert mode
enum ble_findme_alert_level
{
    NO_ALERT                        = 0,
    MILD_ALERT                      = 1,
    HIGH_ALERT                      = 2,
    UNDIRECTED_DISCOVERABLE_ALERT   = 3,
};

// Find me Alert type
#define ALERT_NONE    0x00
#define ALERT_BUZ     0x01
#define ALERT_LED     0x02

#define ALERT_BUZ_LED (ALERT_BUZ | ALERT_LED)
// valid id's are 0 thru 5, corresponding to P26 thru p31
typedef enum {
    BUZ_ID0 = PWM0,
    BUZ_ID1 = PWM1,
    BUZ_ID2 = PWM2,
    BUZ_ID3 = PWM3,
    BUZ_ID4 = PWM4,
    BUZ_ID5 = PWM5,
} tBuzId;
#define FINDME_BUZ_PWM_ID        BUZ_ID2
// app Alert ID
enum
{
    APP_ALERT_PATTERN_MILD_ID     = 0x00,
    APP_ALERT_PATTERN_HIGH_ID     = 0x01,
    APP_ALERT_PATTERN_MAX_ID      = 0x02,
};

typedef struct
{
    uint8_t activeAlterLevel;   // active immediate alert level.
    uint8_t alertType;          // alert type(LED or BUZ or both)

    // buz alert state
    tBuzId buz_id;             // id for buz
    uint8_t buz_alert_active;   // buz alert is on playing
    uint8_t buz_on;             // app buz on state
    uint8_t buz_pattern_id;     // active buz pattern ID (mild or High)
    uint16_t buz_repeat;        // app buz repeat num
    uint16_t buz_timeout_sec;   // buz timeout in sec part
    uint16_t buz_timeout_ms;    // buz timeout in ms part
    uint16_t buz_timer_call_per_sec; // buz tick divider for ms timer

    // led alert state
    uint8_t led_alert_active;   // app led alert  is on playing
    uint8_t led_on;             // app led on state
    uint8_t led_pattern_id;     // active pattern ID (mild or High)
    uint16_t led_repeat;        // app led repeat num
    uint16_t led_timeout_sec;   // led timeout in sec part
    uint16_t led_timeout_ms;    // led timeout in ms part
    uint16_t led_timer_call_per_sec; // led tick divider for ms timer
}tAppFindmeState;

typedef struct
{
    // Alert Buz config
    AppBuzAlertConfig alertBuzCfg[APP_ALERT_PATTERN_MAX_ID];

    // Alert Led config
    AppLedAlertConfig alertLedCfg[APP_ALERT_PATTERN_MAX_ID];
}tAppAlertConfig;

#endif

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


typedef struct
{
    /// Standard key report
    KeyboardStandardReport  stdRpt;

    /// Battery level report
    KeyboardBatteryReport     batRpt;

    /// Standard rollover report
    KeyboardStandardReport rolloverRpt;

    /// Output LED report. Maintained for GET_REPORT
    KeyboardLedReport ledReport;

    /// Bit mapped key report
    KeyboardBitMappedReport bitMappedReport;

    /// Remote reports
    RemoteReport remoteRpt[REMOTE_MAX_USER_DEFINED_RPT_TYPE];


    // Report change flags

    /// Flag indicating that the std report has been changed since it was last sent
    uint8_t stdRptChanged;

    /// Flag indicating that the bit mapped report has been changed since it was last sent
    uint8_t bitRptChanged;

    /// Flag indicating that the remote reports have been changed since they were last sent
    uint8_t remoteRptChanged[REMOTE_MAX_USER_DEFINED_RPT_TYPE];

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

    /// Number of keys in the current bit mapped report
    uint8_t keysInBitRpt;

    /// Size of the bit mapped report. Includes header and report ID
    uint8_t bitReportSize;

    /// Number of keys in the current remote reports
    uint8_t bytesInRemoteRpt[REMOTE_MAX_USER_DEFINED_RPT_TYPE];

    /// Size of the remote reports
    uint8_t remoteRptSize[REMOTE_MAX_USER_DEFINED_RPT_TYPE];

    /// Number of polls cycles left in the recovery period
    uint8_t recoveryInProgress;

    // Event structures

    /// Temporary used for creating key events
    HidEventKey kbKeyEvent;

    /// null Event.
    HidEventAny eventNULL;

    /// Temporary used for events
    HidEventUserDefine  voiceEvent;
    HidEventUserDefine  voiceCtrlEvent;
#ifdef SUPPORT_TOUCHPAD
    HidEventTouchpad    touchpadEvent;
#endif
/// The event queue for use by app.
    wiced_hidd_app_event_queue_t appEventQueue;



#if SUPPORT_AUDIO
    uint8_t audioStopEventInQueue;  //indicate if WICED_HIDD_RC_MIC_STOP_REQ event is in the event queue
    uint8_t micStopEventInQueue;    //indicate if WICED_HIDD_MIC_STOP event is in the event queue
    uint8_t audioPacketInQueue;
    uint8_t audiobutton_pressed;
#endif

    uint8_t codecSettingMsg_type;
    uint8_t codecSettingMsg_dataCnt;
    uint8_t codecSettingMsg_dataBuffer[6];

    uint8_t pollSeqn;
    uint8_t keyInterrupt_On;
    uint8_t allowSDS;


} tRemoteAppState;

/// Connect button state
typedef enum
{
    CONNECT_BUTTON_UP,
    CONNECT_BUTTON_DOWN
}ConnectButtonPosition;


void bleremoteapp_create(void);
void bleremoteapp_pre_init(void);
void bleremoteapp_init(void);
void bleremoteapp_shutdown(void);
void bleremoteapp_pollReportUserActivity(void);
uint8_t bleremoteapp_pollActivityUser(void);
void bleremoteapp_pollActivityKey(void);

void bleremoteapp_flushUserInput(void);
void bleremoteapp_stdErrResp(void);
void bleremoteapp_procErrKeyscan(void);
void bleremoteapp_procErrEvtQueue(void);

void bleremoteapp_stdRptRolloverSend(void);
void bleremoteapp_bitRptSend(void);
void bleremoteapp_batRptSend(void);
void bleremoteapp_stdRptSend(void);
void bleremoteapp_remoteRptSend(uint8_t rptIndex);

void bleremoteapp_procEvtKey(void);
void bleremoteapp_stdRptProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode);
void bleremoteapp_stdRptProcEvtKeyDown(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode);
void bleremoteapp_stdRptProcEvtKeyUp(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode);
void bleremoteapp_stdRptProcOverflow(void);
void bleremoteapp_stdRptProcEvtModKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode);
void bleremoteapp_bitRptProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t rowCol);

uint8_t bleremoteapp_findKeyInRemoteRpt(uint8_t rptIndex, uint8_t keyCode, uint8_t translationCodeSize);
uint8_t bleremoteapp_addKeytoRemoteRpt(uint8_t rptIndex, uint8_t keyCode, uint8_t translationCodeSize);
void bleremoteapp_removeKeyfromRemoteRpt(uint8_t rptIndex, uint8_t keyCode, uint8_t translationCodeSize);
void bleremoteapp_remoteRptProcEvtKeyDown(uint8_t upDownFlag, uint8_t keyCode, uint8_t rptID, uint8_t translationCodeSize);
void bleremoteapp_remoteRptProcEvtKeyUp(uint8_t upDownFlag, uint8_t keyCode, uint8_t rptID, uint8_t translationCodeSize);
void bleremoteapp_remoteRptProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t rptID, uint8_t translationCodeSize);
void bleremoteapp_procEvtUserDefinedKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode);

void bleremoteapp_appActivityDetected(void *remApp);
void bleremoteapp_userKeyPressDetected(void* unused);
void bleremoteapp_motionsensorActivityDetected(void* unused, uint8_t unsued);


void bleremoteapp_batLevelChangeNotification(uint32_t newLevel);

void bleremoteapp_connectButtonHandler(ConnectButtonPosition connectButtonPosition);

void bleremoteapp_ledRptInit(void);
void bleremoteapp_stdRptRolloverInit(void);
void bleremoteapp_stdRptClear(void);
void bleremoteapp_bitRptClear(void);
void bleremoteapp_remoteRptClear(void);
void bleremoteapp_clearAllReports(void);

//audio
void bleremoteapp_pollActivityVoice(void);
void bleremoteApp_procEvtVoice(void);
void bleremoteapp_procEvtVoiceCtrl(uint8_t eventType);
void bleremoteapp_voiceModeSend(void);
void bleremoteapp_voiceReadCodecSetting(void);
void bleremoteapp_voiceWriteCodecSetting(void);


//motion
void bleremoteapp_motionInterruptHandler(void);
uint8_t bleremoteapp_pollActivitySensor(void);
void bleremoteapp_procEvtMotion(void);
#ifdef SUPPORT_TOUCHPAD
void bleremoteapp_procEvtTouchpad(void);
void   gpioActivityDetected(void * appData, uint8_t portPin);
uint8_t   pollTouchpadActivity(void);
uint8_t  handleTouchpadVirtualKey(HidEventKey * ke);
#endif



void bleremoteapp_txModifiedKeyReports(void);
void bleremoteapp_procEvtUserDefined(void);
void bleremoteapp_transportStateChangeNotification(uint32_t newState);

void bleremoteapp_setReport(wiced_hidd_report_type_t reportType,uint8_t reportId,void *payload,uint16_t payloadSize);


//gatt callback of client write (HID READ)
void bleremoteapp_ctrlPointWrite(wiced_hidd_report_type_t reportType, uint8_t reportId, void *payload, uint16_t payloadSize);
void bleremoteapp_clientConfWriteRptStd(wiced_hidd_report_type_t reportType, uint8_t reportId, void *payload, uint16_t payloadSize);
void bleremoteapp_clientConfWriteRptBitMapped(wiced_hidd_report_type_t reportType, uint8_t reportId, void *payload, uint16_t payloadSize);
void bleremoteapp_clientConfWriteBatteryRpt(wiced_hidd_report_type_t reportType, uint8_t reportId, void *payload, uint16_t payloadSize);
void bleremoteapp_clientConfWriteRptMotion(wiced_hidd_report_type_t reportType, uint8_t reportId, void *payload, uint16_t payloadSize);
void bleremoteapp_clientConfWriteRptUserDefinedKey(wiced_hidd_report_type_t reportType, uint8_t reportId, void *payload, uint16_t payloadSize);
void bleremoteapp_clientConfWriteRptVoice(wiced_hidd_report_type_t reportType, uint8_t reportId, void *payload, uint16_t payloadSize);
void bleremoteapp_clientConfWriteRptVoiceCtrl(wiced_hidd_report_type_t reportType, uint8_t reportId, void *payload, uint16_t payloadSize);
#ifdef SUPPORT_TOUCHPAD
void bleremoteapp_clientConfWriteRptTouchpad(wiced_hidd_report_type_t reportType, uint8_t reportId, void *payload, uint16_t payloadSize);
#endif
void bleremoteapp_updateClientConfFlags(uint16_t enable, uint16_t featureBit);
void bleremoteapp_updateGattMapWithNotifications(uint16_t flags);

uint32_t bleremoteapp_sleep_handler(wiced_sleep_poll_type_t type );
void bleremoteapp_aon_restore(void);

#ifdef SUPPORTING_FINDME
void bleremoteapp_findme_init(void);
uint8_t bleremoteapp_isAlertIdle(void);
void bleremoteapp_alertBuz_timeout(uint32_t unused);
void bleremoteapp_StartAlertBuzTimer(uint16_t timeout_ms);
void bleremoteapp_StopAlertBuzTimer(void);
void bleremoteapp_alertBuzFreq(uint8_t freq, uint16_t init_value, uint16_t toggle_val);
void bleremoteapp_alertBuzOn(uint8_t pwm_id);
void bleremoteapp_alertBuzOff(uint8_t pwm_id);
void bleremoteapp_alertBuzPlay(uint8_t pattern_id);
void bleremoteapp_alertBuzStop(void);
void bleremoteapp_alertLed_timeout(uint32_t unused);
void bleremoteapp_StartAlertLedTimer(uint16_t timeout_ms);
void bleremoteapp_StopAlertLedTimer(void);
void bleremoteapp_alertLedOn(void);
void bleremoteapp_alertLedOff(void);
void bleremoteapp_alertLedPlay(uint8_t pattern_id);
void bleremoteapp_alertLedStop(void);
int bleremoteapp_findme_writeCb(void *p);
#endif





#endif
