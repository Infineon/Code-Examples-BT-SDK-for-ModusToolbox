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
* The BLE Keyboard application is a single chip SoC.  It provides a turnkey solution
* using on-chip keyscan HW component and is compliant with HID over GATT Profile (HOGP).
*
* During initialization the app registers with LE stack, WICED HID Device Library and
* keyscan HW to receive various notifications including bonding complete, connection
* status change, peer GATT request/commands and interrupts for key pressed/released.
* Press any key will start LE advertising. When device is successfully bonded, the app
* saves bonded host's information in the NVRAM.
* When user presses/releases key, a key report will be sent to the host.
* On connection up or battery level changed, a battery report will be sent to the host.
* When battery level is below shutdown voltage, device will do critical shutdown.
* Host can send LED report to the device to control LED.
*
* Features demonstrated
*  - GATT database and Device configuration initialization
*  - Registration with LE stack for various events
*  - Sending HID reports to the host
*  - Processing write requests from the host
*  - Low power management
*  - Over the air firmware update (OTAFWU)
*
* To demonstrate the app, walk through the following steps.
* 1. Plug the keyboard HW into your computer
* 2. Build and download the application
* 3. Unplug the keyboard HW from your computer and power cycle the keyboard HW
* 4. Press any key to start LE advertising, then pair with a PC or Tablet
* 5. Once connected, it becomes the keyboard of the PC or Tablet.
*
* In case you don't have the right hardware, eval_keyboard, which is required to support the 8*18
* key matrix used in the BLE keyboard application, you will need to modify the key matrix to match your hardware.

* You can also use the WICED board to simulate keyboard by using ClientControl tool
* test the basic BLE functions.
* NOTE: To use Client Control, make sure you use "TESTING_USING_HCI=1" in application settings.
* In ModusToolbox, select right click on app and select 'Change Application Settings'
*
* 1. Plug the hardware into your computer
* 2. Build and download the application
* 3. Run ClientControl.exe.
* 4. Choose 3M as Baudrate and select the serial port in ClientControl tool window.
* 5. Press Reset button on the board and open the port.
* 6. Press "Enter Pairing Mode"or "Connect" to start LE advertising, then pair with a PC or Tablet
* 7. Once connected, it becomes the keyboard of the PC or Tablet.
*  - Select Interrupt channel, Input report, enter the contents of the report
*    and click on the Send button, to send the report.  For example to send
*    key down event when key '1' is pushed, report should be
*    01 00 00 1e 00 00 00 00 00.  All keys up 01 00 00 00 00 00 00 00 00.
*    Please make sure you always send a key up report following key down report.
*/
#include "spar_utils.h"
#include "gki_target.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_mia.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_keyscan.h"
//#include "scrolldriver.h"
#include "ble_keyboard_gatts.h"
#include "ble_keyboard.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_sdp.h"
#include "wiced_hal_batmon.h"
#include "wiced_hal_adc.h"
#include "wiced_timer.h"
#include "wiced_memory.h"
#ifdef LED_USE_PWM
#include "wiced_hal_aclk.h"
#include "wiced_hal_pwm.h"
#endif
//////////////////////////////////////////////////////////////////////////////
//                      local interface declaration
//////////////////////////////////////////////////////////////////////////////

#ifdef OTA_FIRMWARE_UPGRADE
#include "wiced_bt_ota_firmware_upgrade.h"

#define OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT         512
typedef struct
{
// device states during OTA FW upgrade
#define OTA_STATE_IDLE                   0
#define OTA_STATE_READY_FOR_DOWNLOAD     1
#define OTA_STATE_DATA_TRANSFER          2
#define OTA_STATE_VERIFICATION           3
#define OTA_STATE_VERIFIED               4
#define OTA_STATE_ABORTED                5
    int32_t         state;
    uint8_t         bdaddr[6];               // BDADDR of connected device
    uint16_t        client_configuration;    // characteristic client configuration descriptor
    uint8_t         status;                  // Current status
    uint16_t        current_offset;          // Offset in the image to store the data
    int32_t         total_len;               // Total length expected from the host
    int32_t         current_block_offset;
    int32_t         total_offset;
    uint32_t        crc32;
    uint32_t        recv_crc32;
    uint8_t         indication_sent;
    wiced_timer_t   reset_timer;
    uint8_t         read_buffer[OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT];
} ota_fw_upgrade_state_t;

extern ota_fw_upgrade_state_t   ota_fw_upgrade_state;
extern uint8_t  ota_fw_upgrade_initialized;

wiced_bool_t wiced_ota_fw_upgrade_is_active(void);
#endif

tKbAppState ble_keyboard_application_state = {0, };
tKbAppState *kbAppState = &ble_keyboard_application_state;


uint16_t  characteristic_client_configuration[MAX_NUM_CLIENT_CONFIG_NOTIF] = {0,};
uint8_t   kbapp_protocol = PROTOCOL_REPORT;
uint8_t   battery_level = 100;

uint8_t blekb_key_std_rpt[KEYRPT_LEN] = {0, };       //map to (&(kbAppState->stdRpt.modifierKeys))[kbAppState->stdRptSize]
uint8_t blekb_bitmap_rpt[KEYRPT_NUM_BYTES_IN_BIT_MAPPED_REPORT] = {0, };  //map to kbAppState->bitMappedReport.bitMappedKeys[]
uint8_t blekb_kb_output_rpt = 0;
uint8_t blekb_sleep_rpt = 0;
uint8_t blekb_scroll_rpt = 0;
uint8_t blekb_func_lock_rpt =0;
uint8_t blekb_connection_ctrl_rpt = 0;

uint8_t firstTransportStateChangeNotification = 1;
wiced_timer_t blekb_allow_sleep_timer;
wiced_timer_t blekb_conn_param_update_timer;

PLACE_DATA_IN_RETENTION_RAM uint8_t  kbapp_funcLock_state; // function lock state

extern KbAppConfig kbAppConfig;
extern KbKeyConfig kbKeyConfig[];
extern uint8_t kbKeyConfig_size;

extern wiced_bt_cfg_settings_t wiced_bt_hid_cfg_settings;
extern uint16_t blehostlist_flags;
extern wiced_bool_t blehidlink_connection_param_updated;

wiced_blehidd_report_gatt_characteristic_t reportModeGattMap[] =
{
    // STD keyboard Input report
    {STD_KB_REPORT_ID   ,WICED_HID_REPORT_TYPE_INPUT ,HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_STD_INPUT_VAL, FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_STD_RPT},
    // Std output report
    {STD_KB_REPORT_ID   ,WICED_HID_REPORT_TYPE_OUTPUT,HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_STD_OUTPUT_VAL,FALSE,kbapp_setReport, KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    // Battery Input report
    {BATTERY_REPORT_ID  ,WICED_HID_REPORT_TYPE_INPUT ,HANDLE_BLEKB_BATTERY_SERVICE_CHAR_LEVEL_VAL,       FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_BATTERY_RPT},
    //Bitmapped report
    {BITMAPPED_REPORT_ID,WICED_HID_REPORT_TYPE_INPUT ,HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_BITMAP_VAL,    FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_BIT_MAPPED_RPT},
    //sleep report
    {SLEEP_REPORT_ID    ,WICED_HID_REPORT_TYPE_INPUT ,HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_SLEEP_VAL,     FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_SLP_RPT},
    //func lock report
    {FUNC_LOCK_REPORT_ID,WICED_HID_REPORT_TYPE_INPUT ,HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_FUNC_LOCK_VAL, FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_FUNC_LOCK_RPT},
    //scroll report
    {SCROLL_REPORT_ID   ,WICED_HID_REPORT_TYPE_INPUT ,HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_SCROLL_VAL,    FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_SCROLL_RPT},

    //connection control feature
    {0xCC, WICED_HID_REPORT_TYPE_FEATURE,HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL_VAL,FALSE,kbapp_setReport     ,KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, WICED_HID_REPORT_TYPE_OTHER,HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_HID_CTRL_POINT_VAL, FALSE,kbapp_ctrlPointWrite,KBAPP_CLIENT_CONFIG_NOTIF_NONE},

    {0xFF, WICED_HID_REPORT_TYPE_OTHER,HANDLE_BLEKB_LE_HID_SERVICE_PROTO_MODE_VAL,                   FALSE, kbapp_setProtocol                ,KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, WICED_HID_CLIENT_CHAR_CONF, HANDLE_BLEKB_BATTERY_SERVICE_CHAR_CFG_DESCR,                  FALSE, kbapp_clientConfWriteBatteryRpt  ,KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, WICED_HID_CLIENT_CHAR_CONF, HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_STD_INPUT_CHAR_CFG_DESCR, FALSE, kbapp_clientConfWriteRptStd      ,KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, WICED_HID_CLIENT_CHAR_CONF, HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_BITMAP_CHAR_CFG_DESCR,    FALSE, kbapp_clientConfWriteRptBitMapped,KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, WICED_HID_CLIENT_CHAR_CONF, HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_SLEEP_CHAR_CFG_DESCR,     FALSE, kbapp_clientConfWriteRptSlp      ,KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, WICED_HID_CLIENT_CHAR_CONF, HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_FUNC_LOCK_CHAR_CFG_DESCR, FALSE, kbapp_clientConfWriteRptFuncLock ,KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, WICED_HID_CLIENT_CHAR_CONF, HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_SCROLL_CHAR_CFG_DESCR,    FALSE, kbapp_clientConfWriteScroll      ,KBAPP_CLIENT_CONFIG_NOTIF_NONE},

    //Boot keyboard input client conf write
    {0xFF, WICED_HID_CLIENT_CHAR_CONF, HANDLE_BLEKB_LE_HID_SERVICE_HID_BT_KB_INPUT_CHAR_CFG_DESCR,   FALSE, kbapp_clientConfWriteBootMode,    KBAPP_CLIENT_CONFIG_NOTIF_NONE},
};

wiced_blehidd_report_gatt_characteristic_t bootModeGattMap[] =
{
    //Boot keyboard Input report
    {STD_KB_REPORT_ID, WICED_HID_REPORT_TYPE_INPUT, HANDLE_BLEKB_LE_HID_SERVICE_HID_BT_KB_INPUT_VAL,            TRUE, NULL,                           KBAPP_CLIENT_CONFIG_NOTIF_BOOT_RPT},
    //Boot keyboard output report
    {STD_KB_REPORT_ID, WICED_HID_REPORT_TYPE_OUTPUT,HANDLE_BLEKB_LE_HID_SERVICE_HID_BT_KB_OUTPUT_VAL,           FALSE, kbapp_setReport,               KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    //Boot keyboard client conf write
    {0xFF, WICED_HID_CLIENT_CHAR_CONF,     HANDLE_BLEKB_LE_HID_SERVICE_HID_BT_KB_INPUT_CHAR_CFG_DESCR, FALSE, kbapp_clientConfWriteBootMode, KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, WICED_HID_REPORT_TYPE_OTHER,    HANDLE_BLEKB_LE_HID_SERVICE_PROTO_MODE_VAL,                 FALSE, kbapp_setProtocol,             KBAPP_CLIENT_CONFIG_NOTIF_NONE},
};

/// Translation table for func-lock dependent keys.
KbFuncLockDepKeyTransTab kbFuncLockDepKeyTransTab[KB_MAX_FUNC_LOCK_DEP_KEYS] =
{
    // Home/F1
    {0x03, USB_USAGE_F1},
    // Lock/F2
    {0x05, USB_USAGE_F2},
    // Siri/F3
    {0x08, USB_USAGE_F3},
    // Search/F4
    {0x06, USB_USAGE_F4},

    // Language/F5
    {0x09, USB_USAGE_F5},
    // Eject/F6
    {0x0D, USB_USAGE_F6},
    // Previous Track/F7
    {0x0B, USB_USAGE_F7},
    // Play-Pause/F8
    {0x0E, USB_USAGE_F8},

    // Next Track/F9
    {0x0C, USB_USAGE_F9},
    // Mute/F10
    {0x11, USB_USAGE_F10},
    // Vol-Down/F11
    {0x10, USB_USAGE_F11},
    // Vol-Up/F12
    {0x0F, USB_USAGE_F12},

    // Power/Power
    {0x0, USB_USAGE_POWER}
};

#ifdef OTA_FIRMWARE_UPGRADE
void blekbapp_ota_fw_upgrade_status(uint8_t status);
#endif


/////////////////////////////////////////////////////////////////////////////////////////////
/// set up LE Advertising data
/////////////////////////////////////////////////////////////////////////////////////////////
void kbapp_setUpAdvData(void)
{
    wiced_bt_ble_advert_elem_t kbapp_adv_elem[4];
    uint8_t kbapp_adv_flag = BTM_BLE_LIMITED_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint16_t kbapp_adv_appearance = APPEARANCE_HID_KEYBOARD;
    uint16_t kbapp_adv_service = UUID_SERVCLASS_LE_HID;

    // flag
    kbapp_adv_elem[0].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    kbapp_adv_elem[0].len          = sizeof(uint8_t);
    kbapp_adv_elem[0].p_data       = &kbapp_adv_flag;

    // Appearance
    kbapp_adv_elem[1].advert_type  = BTM_BLE_ADVERT_TYPE_APPEARANCE;
    kbapp_adv_elem[1].len          = sizeof(uint16_t);
    kbapp_adv_elem[1].p_data       = (uint8_t *)&kbapp_adv_appearance;

    //16 bits Service: UUID_SERVCLASS_LE_HID
    kbapp_adv_elem[2].advert_type  = BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE;
    kbapp_adv_elem[2].len          = sizeof(uint16_t);
    kbapp_adv_elem[2].p_data       = (uint8_t *)&kbapp_adv_service;

    //dev name
    kbapp_adv_elem[3].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    kbapp_adv_elem[3].len          = strlen(dev_local_name);
    kbapp_adv_elem[3].p_data       = (uint8_t *)dev_local_name;

    wiced_bt_ble_set_raw_advertisement_data(4,  kbapp_adv_elem);
}

////////////////////////////////////////////////////////////////////////////////
/// This function is the timeout handler for allow_sleep_timer
////////////////////////////////////////////////////////////////////////////////
void kbapp_allowsleep_timeout( uint32_t arg )
{
    WICED_BT_TRACE("allow SDS\n");

    kbAppState->allowSDS = WICED_TRUE;
}

////////////////////////////////////////////////////////////////////////////////
/// This function is the timeout handler for conn_param_update_timer
////////////////////////////////////////////////////////////////////////////////
void kbapp_connparamupdate_timeout( uint32_t arg )
{
    //request connection param update if it not requested before
    if (!blehidlink_connection_param_updated
#ifdef OTA_FIRMWARE_UPGRADE
        // if we are not in the middle of OTAFWU
        && !wiced_ota_fw_upgrade_is_active()
#endif
        )
    {
#ifdef ASSYM_SLAVE_LATENCY
        //if actual slavelatency is smaller than desired slave latency, set asymmetric slave latency in the slave side
        if (wiced_blehidd_get_connection_interval()*(wiced_blehidd_get_slave_latency() + 1) <
             ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_INTERVAL_MIN] * (ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_SLAVE_LATENCY] + 1))
        {
            wiced_ble_hidd_link_set_slave_latency(ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_INTERVAL_MIN]*(ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_SLAVE_LATENCY]+1)*5/4);
        }
#else
        wiced_ble_hidd_link_conn_param_update();
#endif
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////
/// This function will be called from blehid_app_init() during start up.
/////////////////////////////////////////////////////////////////////////////////////////////
void blekbapp_create(void)
{
    WICED_BT_TRACE("KBCreated\n");

    //battery monitoring configuraion
    wiced_hal_batmon_config(ADC_INPUT_VDDIO,    // ADC input pin
                            3000,               // Period in millisecs between battery measurements
                            8,                  // Number of measurements averaged for a report, max 16
                            3200,               // The full battery voltage in mili-volts
                            1800,               // The voltage at which the batteries are considered drained (in milli-volts)
                            1700,               // System should shutdown if it detects battery voltage at or below this value (in milli-volts)
                            100,                // battery report max level
                            BATTERY_REPORT_ID,  // battery report ID
                            1,                  // battery report length
                            1);                 // Flag indicating that a battery report should be sent when a connection is established



#ifndef  TESTING_USING_HCI
    wiced_hal_keyscan_configure(NUM_KEYSCAN_ROWS, NUM_KEYSCAN_COLS);
    wiced_hal_keyscan_init();
#endif

#ifdef __BLEKB_SCROLL_REPORT__
    quadratureConfig.port0PinsUsedAsQuadratureInput=0;
    quadratureConfig.configureP26AsQOC0=0;
    quadratureConfig.ledEnableDisableControls=0;
    quadratureConfig.scanPeriod=0xff00;

    quadratureConfig.togglecountLed0=0xfff0;
    quadratureConfig.togglecountLed1=0xfff0;
    quadratureConfig.togglecountLed2=0xfff0;
    quadratureConfig.togglecountLed3=0xfff0;

    quadratureConfig.sampleInstantX=0xfff8;
    quadratureConfig.sampleInstantY=0xfff8;
    quadratureConfig.sampleInstantZ=0xfff8;

    quadratureConfig.channelEnableAndSamplingRate=0x88;

    quadratureConfig.pollXAxis=0;
    quadratureConfig.pollYAxis=1;
    quadratureConfig.pollZAxis=0;

    scroll_init();
#endif

    kbapp_LED_init();

    wiced_hidd_event_queue_init(&kbAppState->eventQueue, (uint8_t *)wiced_memory_permanent_allocate(kbAppConfig.maxEventNum * kbAppConfig.maxEventSize),
                    kbAppConfig.maxEventSize, kbAppConfig.maxEventNum);


    kbapp_init();

    WICED_BT_TRACE("Free RAM bytes=%d bytes\n", wiced_memory_get_free_bytes());
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// This function will be called from blekbapp_create() during start up.
/////////////////////////////////////////////////////////////////////////////////////////////
void kbapp_init(void)
{
    wiced_ble_hidd_link_set_preferred_conn_params(wiced_bt_hid_cfg_settings.ble_scan_cfg.conn_min_interval,        // 18*1.25=22.5ms
                                        wiced_bt_hid_cfg_settings.ble_scan_cfg.conn_max_interval,        // 18*1.25=22.5ms
                                        wiced_bt_hid_cfg_settings.ble_scan_cfg.conn_latency,             //  21. i.e.  495ms slave latency
                                        wiced_bt_hid_cfg_settings.ble_scan_cfg.conn_supervision_timeout);//600 * 10=600ms=6 seconds

    kbapp_setUpAdvData();

    //timer to allow ShutDown Sleep (SDS)
    wiced_init_timer( &blekb_allow_sleep_timer, kbapp_allowsleep_timeout, 0, WICED_MILLI_SECONDS_TIMER );

    //timer to request connection param update
    wiced_init_timer( &blekb_conn_param_update_timer, kbapp_connparamupdate_timeout, 0, WICED_MILLI_SECONDS_TIMER );

    // Determine the size of the standard report. Report ID will not be sent.
    kbAppState->stdRptSize = kbAppConfig.maxKeysInStdRpt +
        (sizeof(KeyboardStandardReport) - sizeof(kbAppState->stdRpt.keyCodes)) - 1;

    // Determine the size of the Battery Report. Report ID will not be sent.
    kbAppState->batRpt.reportID = BATTERY_REPORT_ID;
    kbAppState->batRptSize = sizeof(kbAppState->batRpt.level);

    // Determine the size of the bit mapped report.Report ID will not be sent.
    // and round up to the next largest integer
    kbAppState->bitReportSize = (kbAppConfig.numBitMappedKeys + 7)/8;

#ifndef  TESTING_USING_HCI
    wiced_hal_keyscan_register_for_event_notification(kbapp_userKeyPressDetected, NULL);
#endif
#ifdef __BLEKB_SCROLL_REPORT__
    wiced_hal_quadrature_register_for_event_notification(kbapp_userScrollDetected, NULL);
#endif

     // Set initial func-lock state for power on reset
    if (wiced_hal_mia_is_reset_reason_por())
    {
        kbapp_funcLock_state = kbAppState->funcLockInfo.state = kbAppConfig.defaultFuncLockState;
    }


    // Set func lock key as up
    kbAppState->funcLockInfo.kepPosition = FUNC_LOCK_KEY_UP;

    // The following flag applies when func-lock is used in combo with another key. Start it off as FALSE
    kbAppState->funcLockInfo.toggleStateOnKeyUp = WICED_FALSE;

    // We are not in recovery
    kbAppState->recoveryInProgress = 0;

    // Initialize temporaries used for events
    kbAppState->keyEvent.eventInfo.eventType = HID_EVENT_KEY_STATE_CHANGE;
    kbAppState->scrollEvent.eventInfo.eventType = HID_EVENT_MOTION_AXIS_0;

    // No fractional scroll present
    kbAppState->scrollFractional = 0;

    // Reset the scroll discard counter
    kbAppState->pollsSinceScroll = 0;

    kbapp_stdRptRolloverInit();

    kbapp_ledRptInit();

    kbapp_funcLockRptInit();

    kbapp_clearAllReports();

    //add battery observer
    wiced_hal_batmon_add_battery_observer(kbapp_batLevelChangeNotification);

    //register App low battery shut down handler
    wiced_hal_batmon_register_low_battery_shutdown_cb(kbapp_shutdown);

    wiced_ble_hidd_link_add_state_observer(kbapp_stateChangeNotification);

    wiced_ble_hidd_link_register_poll_callback(kbapp_pollReportUserActivity);

    wiced_ble_hidd_link_register_sleep_permit_handler(kbapp_sleep_handler);

    wiced_blehidd_register_report_table(reportModeGattMap, sizeof(reportModeGattMap)/sizeof(reportModeGattMap[0]));

#ifdef __BLEKB_SCROLL_REPORT__
    wiced_hal_mia_notificationRegisterQuad();
#endif

    wiced_ble_hidd_link_init();

    wiced_hal_mia_enable_mia_interrupt(TRUE);
    wiced_hal_mia_enable_lhl_interrupt(TRUE);//GPIO interrupt
}

////////////////////////////////////////////////////////////////////////////////
/// This function is called when battery voltage drops below the configured threshold.
////////////////////////////////////////////////////////////////////////////////
void kbapp_shutdown(void)
{
    WICED_BT_TRACE("kbapp_shutdown\n");

    kbapp_flushUserInput();

#ifdef __BLEKB_SCROLL_REPORT__
    // Disable the scroll HW
    scroll_turnOff();
#endif

#ifndef  TESTING_USING_HCI
    // Disable key detection
    wiced_hal_keyscan_turnOff();
#endif

    if(wiced_ble_hidd_link_is_connected())
        wiced_ble_hidd_link_disconnect();

    // Disable Interrupts
    wiced_hal_mia_enable_mia_interrupt(FALSE);
    wiced_hal_mia_enable_lhl_interrupt(FALSE);

}

////////////////////////////////////////////////////////////////////////////////
/// This function will poll user activities and send reports
////////////////////////////////////////////////////////////////////////////////
void kbapp_pollReportUserActivity(void)
{
    uint8_t activitiesDetectedInLastPoll;

    kbAppState->pollSeqn++;

    if((kbAppState->pollSeqn % 64) == 0)
    {
        WICED_BT_TRACE(".");
    }

    activitiesDetectedInLastPoll = kbapp_pollActivityUser();

    // If there was an activity and the transport is not connected
    if (activitiesDetectedInLastPoll != BLEHIDLINK_ACTIVITY_NONE &&
        !wiced_ble_hidd_link_is_connected())
    {
        // ask the transport to connect.
        wiced_ble_hidd_link_connect();
    }

    if(wiced_ble_hidd_link_is_connected())
    {
        // Generate a report
        if(wiced_bt_hid_cfg_settings.security_requirement_mask)
        {
            if (wiced_blehidd_is_link_encrypted())
            {
                kbapp_generateAndTxReports();
            }
        }
        else
        {
            kbapp_generateAndTxReports();
        }

#ifdef OTA_FIRMWARE_UPGRADE
        if (!wiced_ota_fw_upgrade_is_active())
#endif
        {
            wiced_hal_batmon_poll_monitor();  // Poll the battery monitor
        }
    }

}

////////////////////////////////////////////////////////////////////////////////
/// This function will poll HW for user activies
////////////////////////////////////////////////////////////////////////////////
uint8_t kbapp_pollActivityUser(void)
{
    // Poll the hardware for events
    wiced_hal_mia_pollHardware();

    // Poll and queue key activity
    kbapp_pollActivityKey();

#ifdef __BLEKB_SCROLL_REPORT__
    // Poll and queue scroll activity
    kbapp_pollActivityScroll();
#endif

    // For all other cases, return value indicating whether any event is pending or
    return (wiced_hidd_event_queue_get_num_elements(&kbAppState->eventQueue) ? BLEHIDLINK_ACTIVITY_REPORTABLE : BLEHIDLINK_ACTIVITY_NONE) |
               ((kbAppState->modKeysInStdRpt || kbAppState->keysInStdRpt || kbAppState->keysInBitRpt || kbAppState->slpRpt.sleepVal)?
                BLEHIDLINK_ACTIVITY_NON_REPORTABLE : BLEHIDLINK_ACTIVITY_NONE);
}

/////////////////////////////////////////////////////////////////////////////////
/// This function polls for key activity and queues any key events in the
/// FW event queue. Events from the keyscan driver are processed until the driver
/// runs out of events. Connect button events are seperated out and handled here
/// since we don't want them to go through the normal event queue. If necessary,
/// the end of scan cycle event after the connect button is supressed. Also
/// note that connect button events are supressed during recovery to eliminate
/// spurious connect button events.
/////////////////////////////////////////////////////////////////////////////////
void kbapp_pollActivityKey(void)
{
#ifndef  TESTING_USING_HCI
    uint8_t suppressEndScanCycleAfterConnectButton;

    // Assume that end-of-cycle event suppression is on
    suppressEndScanCycleAfterConnectButton = TRUE;

    // Process all key events from the keyscan driver
    while (wiced_hal_keyscan_get_next_event(&kbAppState->keyEvent.keyEvent))
    {
        //WICED_BT_TRACE("\nkc_: %d, U/D: %d", kbAppState->keyEvent.keyEvent.keyCode, kbAppState->keyEvent.keyEvent.upDownFlag);
        // Check for connect button
        if (kbAppState->keyEvent.keyEvent.keyCode == kbAppConfig.connectButtonScanIndex)
        {
            // Ignore connect button in recovery
            if (!kbAppState->recoveryInProgress)
            {
                // Pass current connect button state to connect button handler
                kbapp_connectButtonHandler(
                    ((kbAppState->keyEvent.keyEvent.upDownFlag == KEY_DOWN)?
                     CONNECT_BUTTON_DOWN:CONNECT_BUTTON_UP));
            }
        }
        else
        {
            // Check if this is an end-of-scan cycle event
            if (kbAppState->keyEvent.keyEvent.keyCode == END_OF_SCAN_CYCLE)
            {
                // Yes. Queue it if it need not be suppressed
                if (!suppressEndScanCycleAfterConnectButton)
                {
                    wiced_hidd_event_queue_add_event_with_overflow(&kbAppState->eventQueue, &kbAppState->keyEvent.eventInfo, sizeof(kbAppState->keyEvent), kbAppState->pollSeqn);
                }

                // Enable end-of-scan cycle supression since this is the start of a new cycle
                suppressEndScanCycleAfterConnectButton = TRUE;
            }
            else
            {
                // No. Queue the key event
                wiced_hidd_event_queue_add_event_with_overflow(&kbAppState->eventQueue, &kbAppState->keyEvent.eventInfo, sizeof(kbAppState->keyEvent), kbAppState->pollSeqn);

                // Disable end-of-scan cycle supression
                suppressEndScanCycleAfterConnectButton = FALSE;
            }
        }
    }
#endif
}

/////////////////////////////////////////////////////////////////////////////////
/// This function polls the scroll interface to get any newly detected
/// scroll count. It negates the data and performs any scaling if configured to do so.
/// If configured to do so, it discards any fractional value after the configured
/// number of polls. If any non-fractional scroll activity is accumulated,
/// it queues a scroll event.
/////////////////////////////////////////////////////////////////////////////////
void kbapp_pollActivityScroll(void)
{
#ifdef __BLEKB_SCROLL_REPORT__
    int16_t scrollCurrent = scroll_getCount();

    // Check for scroll
    if (scrollCurrent)
    {
        // Negate scroll value if enabled
        if (kbAppConfig.negateScroll)
        {
            scrollCurrent = -scrollCurrent;
        }

        // Check if scroll scaling is enabled
        if (kbAppConfig.scrollScale)
        {
            // Yes. Add the current scroll count to the fractional count
            kbAppState->scrollFractional += scrollCurrent;

            // Scale and adjust accumulated scroll value. Fractional value will be
            // left in the factional part. Place the whole number in the scroll
            // event
            kbAppState->scrollEvent.motion =
                kbapp_scaleValue(&kbAppState->scrollFractional, kbAppConfig.scrollScale);

            // Reset the scroll discard counter
            kbAppState->pollsSinceScroll = 0;
        }
        else
        {
            // No scaling is required. Put the data in the scroll event
            kbAppState->scrollEvent.motion = scrollCurrent;
        }

        // Queue scroll event with the proper seqn
        wiced_hidd_event_queue_add_event_with_overflow(&kbAppState->eventQueue,
                                          &kbAppState->scrollEvent.eventInfo, sizeof(kbAppState->scrollEvent), kbAppState->pollSeqn);
    }
    else
    {
        // If scroll scaling timeout is not infinite, bump up the
        // inactivity counter and check if we have crossed the threshold.
        if (kbAppConfig.pollsToKeepFracScrollData &&
            ++kbAppState->pollsSinceScroll >= kbAppConfig.pollsToKeepFracScrollData)
        {
            // We have. Discard any fractional scroll data
            kbAppState->scrollFractional = 0;

            // Reset the scroll discard counter
            kbAppState->pollsSinceScroll = 0;
        }
    }
#endif
}

/////////////////////////////////////////////////////////////////////////////////
///   This function scales (divides by a power of 2) a value, returns the quotient
/// and leaves the remainder in the value. It handles positive and negative
/// numbers
///
/// \param val -Pointer to value. It outputs the remainder value
/// \param scaleFactor -Number of bits to scale by (shift right)
///
///
/// \return
///   The whole number after the scaling.
/////////////////////////////////////////////////////////////////////////////////
int16_t kbapp_scaleValue(int16_t * val, uint8_t scaleFactor)
{
    int16_t result;

    // Get the mod of the value
    if (*val < 0)
    {
        result = - *val;
    }
    else
    {
        result = *val;
    }

    // Now scale it by the given amount
    result >>= scaleFactor;

    // Check if we have anything left
    if (result)
    {
        // Yes. Now we have to adjust the sign of the result
        if (*val < 0)
        {
            // So we had a negative value. Adjust result accordingly
            result = -result;
        }

        // Now adjust the actual value
        *val -= (result << scaleFactor);
    }

    // Return the scaled value
    return result;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function performs connection button processing. It should be called with
/// the current state of the connect button. This function generates a
/// become discoverable event to the BT Transport if the connect button is
/// held for the configured duration. The configured duration may be 0
/// in which case an instantaneous press of the button causes the device
/// to become discoverable. Once a "become disoverable" event has
/// been generated, not further events will be generated until after the
/// button has been released
///
/// \param connectButtonPosition current position of the connect button, up or down
/////////////////////////////////////////////////////////////////////////////////
void kbapp_connectButtonHandler(ConnectButtonPosition connectButtonPosition)
{
    // The connect button was not pressed. Check if it is now pressed
    if (connectButtonPosition == CONNECT_BUTTON_DOWN)
    {
        WICED_BT_TRACE("Connect Btn Pressed\n");
        kbapp_connectButtonPressed();
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This method handles connect button pressed events. It performs the following
/// actions in order:
///  - If we are configured to generate a VC unplug on connect button press
///    it generates a VC unplug to the BT transport
///  - If we are configured to become discoverable on connect button press
///    it tells the BT transport to become discoverable
/////////////////////////////////////////////////////////////////////////////////
void kbapp_connectButtonPressed(void)
{
    wiced_ble_hidd_link_virtual_cable_unplug();
}

/////////////////////////////////////////////////////////////////////////////////
/// Set the idle rate. Converts the idle rate to BT clocks and saves the value
/// for later use.
/// \param idleRateIn4msUnits 0 means infinite idle rate
/////////////////////////////////////////////////////////////////////////////////
void kbapp_setIdleRate(uint8_t idleRateIn4msUnits)
{
    // Save the idle rate in units of 4 ms
    kbAppState->idleRate = idleRateIn4msUnits;

    // Convert to BT clocks for later use. Formula is ((Rate in 4 ms)*192)/15
    kbAppState->idleRateInBtClocks = idleRateIn4msUnits*192;
    kbAppState->idleRateInBtClocks /= 15;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function performs idle rate processing for the standard keyboard report.
/// It will transmit the old standard report under the following conditions:
///     - Idle rate is non-zero
///     - We are not in the middle of a recovery
///     - At least one key is down in the standard report, either normal key
///       or a modifier key.
///     - No events are pending
///     - The active transport is willing to accept a report
///     - Required time has elapsed since the last time the standard report was sent
/////////////////////////////////////////////////////////////////////////////////
void kbapp_idleRateProc(void)
{
    // Send the standard report again if the above criteria is satisfied
    if (kbAppState->idleRate &&
        !kbAppState->recoveryInProgress &&
        (kbAppState->keysInStdRpt || kbAppState->modKeysInStdRpt) &&
        !wiced_hidd_event_queue_get_num_elements(&kbAppState->eventQueue) &&
        (wiced_bt_buffer_poolutilization(HCI_ACL_POOL_ID) < 80) &&
        (wiced_hidd_get_bt_clocks_since(kbAppState->stdRptTxInstant) >= kbAppState->idleRateInBtClocks))
    {
        kbapp_stdRptSend();
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function provides an implementation for the generateAndTxReports() function
/// defined by the HID application. This function is only called when the active transport
/// is connected. This function performs the following actions:
///  - When pin code entry is in progress, the behavior of this function is changed.
///    It only checks and transmits the pin code report; normal event processing is
///    suspended.
///  - If the number of packets in the hardware fifo is less than the report generation
///    threshold and the event queue is not empty, this function will process events
///    by calling the event rpocessing functions, e.g. kbapp_procEvtKey(),
///    kbapp_procEvtScroll()
///  - This function also tracks the recovery period after an error. If
///    the recovery count is non-zero, it is decremented as long as there is room
///    for one report in the transport
/////////////////////////////////////////////////////////////////////////////////
void kbapp_generateAndTxReports(void)
{
    HidEvent *curEvent;

    // If we are recovering from an error, decrement the recovery count as long as the transport
    // has room. Avoid the case where no event processing is done during recovery because
    // transport is full, as the failure might be a non-responding transport.
    if (kbAppState->recoveryInProgress)
    {
        // If recovery is complete, transmit any modified reports that we have been hoarding
        if (!--kbAppState->recoveryInProgress)
        {
            kbapp_txModifiedKeyReports();
        }
    }
    // Continue report generation as long as the transport has room and we have events to process
    while ((wiced_bt_buffer_poolutilization (HCI_ACL_POOL_ID) < 80) &&
           ((curEvent = (HidEvent *)wiced_hidd_event_queue_get_current_element(&kbAppState->eventQueue)) != NULL))
    {
        // Further processing depends on the event type
        switch (curEvent->eventType)
        {
            case HID_EVENT_KEY_STATE_CHANGE:
                    kbapp_procEvtKey();
                    break;
            case HID_EVENT_MOTION_AXIS_0:
                    kbapp_procEvtScroll();
                    break;
            case HID_EVENT_EVENT_FIFO_OVERFLOW:
                    // Call event queue error handler
                    kbapp_procErrEvtQueue();
                    break;
            default:
                    kbapp_procEvtUserDefined();
                    break;
        }

        // The current event should be deleted by the event processing function.
        // Additional events may also be consumed but we don't care about that
    }

    // Do idle rate processing
    kbapp_idleRateProc();
}

void kbapp_procEvtUserDefinedKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode)
{
    //doing nothing
}

void kbapp_procEvtUserDefined(void)
{
    // Deliberately nothing
}

/////////////////////////////////////////////////////////////////////////////////
/// This function processes keys from the event queue until the end of scan cycle event is seen.
/// During this process it accumulates changes to key reports. Once the end-of-scan-cycle
/// event is seen, it generates any modified reports. If any errors are detected
/// during the processing of events it calls one of the error handlers. Note that
/// if this function is called, there should be at least one event in the event queue
/////////////////////////////////////////////////////////////////////////////////
void kbapp_procEvtKey(void)
{
    uint8_t keyCode=KB_MAX_KEYS;
    uint8_t upDownFlag;
    uint8_t keyType;
    uint8_t keyTranslationCode;
    HidEventKey *keyEvent;

    // Process events until we get an end-of cycle event
    // or an error (which doubles as an end-of-scan cycle event)
    // or we run out of events
    do
    {
        // Grab the next event. The first time we enter this loop we will have at least one
        // event. Subsequent iterations may run out of events
        if ((keyEvent = (HidEventKey *)wiced_hidd_event_queue_get_current_element(&kbAppState->eventQueue)) != NULL)
        {
            // Verify that the next event is a key event. Note that
            // an end of cycle key event is always present except when the event fifo overflows
            // We can assume that we have an overflow if the next event is not a key event
            if (keyEvent->eventInfo.eventType == HID_EVENT_KEY_STATE_CHANGE)
            {
                // Get the current key event and up/down flag
                upDownFlag = keyEvent->keyEvent.upDownFlag;
                keyCode = keyEvent->keyEvent.keyCode;

                // Check if we have a valid key
                if (keyCode < kbKeyConfig_size)  //(keyCode < KB_MAX_KEYS)
                {
                    // This is a normal key event. Translate it to event type and tranlation code
                    keyType = kbKeyConfig[keyCode].type;
                    keyTranslationCode = kbKeyConfig[keyCode].translationValue;

                    // Depending on the key type, call the appropriate function for handling
                    // Pass unknown key types to user function
                    switch(keyType)
                    {
                        case KEY_TYPE_STD:
                            kbapp_stdRptProcEvtKey(upDownFlag, keyCode, keyTranslationCode);
                            break;
                        case KEY_TYPE_MODIFIER:
                            kbapp_stdRptProcEvtModKey(upDownFlag, keyCode, keyTranslationCode);
                            break;
                        case KEY_TYPE_BIT_MAPPED:
                            kbapp_bitRptProcEvtKey(upDownFlag, keyCode, keyTranslationCode);
                            break;
                        case KEY_TYPE_SLEEP:
                            kbapp_slpRptProcEvtKey(upDownFlag, keyCode, keyTranslationCode);
                            break;
                        case KEY_TYPE_FUNC_LOCK:
                            kbapp_funcLockProcEvtKey(upDownFlag, keyCode, keyTranslationCode);
                            break;
                        case KEY_TYPE_FUNC_LOCK_DEP:
                            kbapp_funcLockProcEvtDepKey(upDownFlag, keyCode, keyTranslationCode);
                            break;
                        case KEY_TYPE_NONE:
                            //do nothing
                            break;
                        default:
                            kbapp_procEvtUserDefinedKey(upDownFlag, keyCode, keyTranslationCode);
                            break;
                    }
                }
                // Check if we have an end of scan cycle event
                else if (keyCode == END_OF_SCAN_CYCLE)
                {
                    kbapp_txModifiedKeyReports();
                }
                else
                {
                    WICED_BT_TRACE("\nkc: %d ", keyCode);
                    // Call error handler for all other events
                    kbapp_procErrKeyscan();
                    break;
                }
            }
            else
            {
                // We probably have event queue overflow. Call the event queue error handler
                kbapp_procErrEvtQueue();
                break;
            }

            // Delete the current event since we have consumed it
            wiced_hidd_event_queue_remove_current_element(&kbAppState->eventQueue);
        }
        else
        {
            // We ran out of events before we saw an end-of-scan-cycle event
            // Call error handler and manually exit the loop
            kbapp_procErrEvtQueue();
            break;
        }
    } while (keyCode < KB_MAX_KEYS);
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles func lock key events. Func-lock events are ignored
/// during recovery and in boot mode. On func-lock down, it performs
/// the following actions:
///     - It toggles the func lock state and clears the toggleStateOnKeyUp flag
///       By default, func lock state will not be toggled when the key goes up
///       unless this flag is cleared. Typically, this flags is set if
///       a func-lock dependent key is detected while func-lock is down
///     - It updates the func lock report with the current func-lock state
///       but does not send it
/// On func-lock up, it performs the following actions:
///     - If the toggleStateOnKeyUp flag is set, it toggles func-lock state
///       and updates the func lock report with the new state and event flag.
///       It does not send the report
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param translationCode associated with the func-lock key. Unused
/////////////////////////////////////////////////////////////////////////////////
void kbapp_funcLockProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode)
{
    // Process the event only if we are not in recovery
    if (!kbAppState->recoveryInProgress && kbapp_protocol == PROTOCOL_REPORT)
    {
        // Check if this is a down key or up key
        if (upDownFlag == KEY_DOWN)
        {
            // Only process further if we think the func-lock key state
            // is up
            if (kbAppState->funcLockInfo.kepPosition == FUNC_LOCK_KEY_UP)
            {
                // Flag that the func lock key is down
                kbAppState->funcLockInfo.kepPosition = FUNC_LOCK_KEY_DOWN;

                // Toggle the func lock state and update the func lock report
                kbapp_funcLockToggle();

                // Clear the toggleStateOnKeyUp flag.
                kbAppState->funcLockInfo.toggleStateOnKeyUp = FALSE;
            }
        }
        else
        {
            // Key up. Only process further if we think the func-lock key state
            // is down
            if (kbAppState->funcLockInfo.kepPosition == FUNC_LOCK_KEY_DOWN)
            {
                // Flag that the func lock key is up
                kbAppState->funcLockInfo.kepPosition = FUNC_LOCK_KEY_UP;

                // Check if we need to toggle func-lock
                if (kbAppState->funcLockInfo.toggleStateOnKeyUp)
                {
                    // Toggle the func lock state and update the func lock report
                    kbapp_funcLockToggle();
                }
            }
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function handles sleep key events. It updates the sleep report with
/// the new value of the sleep bit.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param slpBitMask location of the sleep bit in the sleep report
/////////////////////////////////////////////////////////////////////////////////
void kbapp_slpRptProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t slpBitMask)
{
    // Check if this is a down key or up key
    if (upDownFlag == KEY_DOWN)
    {
        // Key down. Update report only if the key state has changed
        if (!(kbAppState->slpRpt.sleepVal & slpBitMask))
        {
            // Mark the appropriate key as down in the sleep report
            kbAppState->slpRpt.sleepVal |= slpBitMask;

            // Flag that the sleep report has changed
            kbAppState->slpRptChanged = TRUE;
        }
    }
    else
    {
        // Key up. Update report only if the key state has changed
        if (kbAppState->slpRpt.sleepVal & slpBitMask)
        {
            // Mark the appropriate key as up in the sleep report
            kbAppState->slpRpt.sleepVal &= ~slpBitMask;

            // Flag that the sleep report has changed
            kbAppState->slpRptChanged = TRUE;
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function handles bit mapped key events. It updates the bit associated
/// with the key in the bit mapped key report.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param rowCol row/col of the associated bit in the report. The col is in the
///               last 3 bits and defines the bit offset while the row defines
///               the byte offset in the bit mapped report array.
/////////////////////////////////////////////////////////////////////////////////
void kbapp_bitRptProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t rowCol)
{
    uint8_t row, col, keyMask;

    // Bit translation table
    const uint8_t bitOffsetToByteValue[8] =
    {
        0x01, 0x02, 0x04, 0x08,
        0x10, 0x20, 0x40, 0x80
    };

    // Only process the key if it is in range. Since the row/col value comes from the user
    // we don't want a bad index to crash the system
    if (rowCol < kbAppConfig.numBitMappedKeys)
    {
        // Extract the row/col from the input argument
        row = (rowCol >> 3);
        col = (rowCol & 0x07);

        // Convert col to bit mask
        keyMask = bitOffsetToByteValue[col];

        // Check if this is a down key or up key
        if (upDownFlag == KEY_DOWN)
        {
            // Key down. Update the report only if the state of the key changed
            if (!(kbAppState->bitMappedReport.bitMappedKeys[row] & keyMask))
            {
                // The following code is funky because compiler will not allow a simple expression
                kbAppState->bitMappedReport.bitMappedKeys[row] |= keyMask;

                // Increment the number of keys in the bit report
                kbAppState->keysInBitRpt++;

                // Flag that the bit report has changed
                kbAppState->bitRptChanged = TRUE;
            }
        }
        else
        {
            // Key up.  Update the report only if the state of the key changed
            if (kbAppState->bitMappedReport.bitMappedKeys[row] & keyMask)
            {
                // The following code is funky because compiler will not allow a simple expression

                kbAppState->bitMappedReport.bitMappedKeys[row] &= ~keyMask;

                // Decrement the number of keys in the bit report
                kbAppState->keysInBitRpt--;

                // Flag that the bit report has changed
                kbAppState->bitRptChanged = TRUE;
            }
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function retrieves the scroll event, combines it with
/// other scroll events if configured to do so and then generates reports as
/// necessary
/////////////////////////////////////////////////////////////////////////////////
void kbapp_procEvtScroll(void)
{
    HidEventMotionSingleAxis *scrollEvent;

    // Clear the scroll count
    kbAppState->scrollReport.motionAxis0 = 0;

    // Go through all scroll events
    while (((scrollEvent = (HidEventMotionSingleAxis *)wiced_hidd_event_queue_get_current_element(&kbAppState->eventQueue))!= NULL) &&
           (scrollEvent->eventInfo.eventType == HID_EVENT_MOTION_AXIS_0))
    {
        // Add new scroll value to the scroll report
        kbAppState->scrollReport.motionAxis0 += scrollEvent->motion;

        // We are done with this event. Delete it
        wiced_hidd_event_queue_remove_current_element(&kbAppState->eventQueue);

        // If report combining is not enabled, get out
        if (!kbAppConfig.scrollCombining)
        {
            break;
        }
    }

    // If the accumulated motion is non-zero, flag that the scroll report has not been sent
    if (kbAppState->scrollReport.motionAxis0)
    {
        kbAppState->scrollRptChanged = TRUE;
    }

    // Now transmit modified reports. This will generate and transmit scroll report when appropriate
    kbapp_txModifiedKeyReports();
}

////////////////////////////////////////////////////////////////////////////////
/// This function provides a standard response identical to kbAppStdErrorResponse.
/// In addition, it also performs the following actions:
///     - All pending events are flushed
///     - The keyscan HW is reset
/// This function is typically used when the FW itself is (or involved in) in error.
/// In such cases the FW no longer has the correct state of anything and we
/// must resort to a total reset
////////////////////////////////////////////////////////////////////////////////
void kbapp_stdErrRespWithFwHwReset(void)
{
    // Provide standard error response
    kbapp_stdErrResp();

    // Flush the event fifo
    wiced_hidd_event_queue_flush(&kbAppState->eventQueue);

#ifndef  TESTING_USING_HCI
    // Reset the keyscan HW
    wiced_hal_keyscan_reset();

    // Configure GPIOs for keyscan operation
    wiced_hal_keyscan_config_gpios();
#endif
}

////////////////////////////////////////////////////////////////////////////////
/// This function handles error events reported by the keyscan HW. Typically
/// these would be ghost events. This function calls stdErrResp() to
/// handle the error.
////////////////////////////////////////////////////////////////////////////////
void kbapp_procErrKeyscan(void)
{
    WICED_BT_TRACE("KSErr\n");
    kbapp_stdErrRespWithFwHwReset();
}

////////////////////////////////////////////////////////////////////////////////
/// This function handles event queue errors. This includes event queue overflow
/// unexpected events, missing expected events, and events in unexpected order.
/// This function does a FW/HW reset via stdErrRespWithFwHwReset in an attempt
/// to address the problem. A user defined implementation should at least
/// remove the first element in the queue if this event is an overflow event
////////////////////////////////////////////////////////////////////////////////
void kbapp_procErrEvtQueue(void)
{
    WICED_BT_TRACE("KSQerr\n");
    kbapp_stdErrRespWithFwHwReset();
}

////////////////////////////////////////////////////////////////////////////////
/// This function provides a standard response for errors. The response is:
///     - A rollover report is sent to the host if we are not already recovering from an error
///     - All reports are cleared and marked as modified. They will be sent once
///       we have recovered from the error.
///     - Marks the func-lock key as up but dow not toggle its state even if
///       the associated toggle flag is set. This allows for proper reconstruction
///       of the keyboard state including func-lock dependent keys after the recovery
///     - The recovery poll count is also set to the configured value.
///     - Connect button state is cleared since we don't know if the connect button press
///       is valid
////////////////////////////////////////////////////////////////////////////////
void kbapp_stdErrResp(void)
{
    // Clear all reports unconditionally
    kbapp_clearAllReports();

    // Mark the func-lock key as up.
    kbAppState->funcLockInfo.kepPosition = FUNC_LOCK_KEY_UP;

    // Send a rollover report if
    //   - we are not already in the middle of a recovery OR
    //   - the rollover report generation period is non-zero and we have exceeded that period
    if (!kbAppState->recoveryInProgress)
    {
        // Send rollover report
        kbapp_stdRptRolloverSend();
    }

    // Reset recovery timeout
    kbAppState->recoveryInProgress = kbAppConfig.recoveryPollCount;

    // Mark all reports as not sent. This ensures that all reports will be sent once
    // the recovery is complete
    kbAppState->slpRptChanged = kbAppState->bitRptChanged = kbAppState->stdRptChanged = TRUE;

    // Assume connect button is now up
//    kbapp_connectButtonHandler(CONNECT_BUTTON_UP);
}

/////////////////////////////////////////////////////////////////////////////////
/// This function sends a rollover report. It assumes that the rollover report
/// has already been initialized. It also snaps the current BT clock and
/// places it in keyboardAppData.rolloverRptTxInstant. This
/// can be used to reapeat the rollover report if desired
/////////////////////////////////////////////////////////////////////////////////
void kbapp_stdRptRolloverSend(void)
{
    // Tx rollover report
    WICED_BT_TRACE("RollOverRpt\n");

    //set gatt attribute value here before sending the report
    memcpy(blekb_key_std_rpt, &(kbAppState->rolloverRpt.modifierKeys), kbAppState->stdRptSize);

    wiced_ble_hidd_link_send_report(kbAppState->rolloverRpt.reportID,WICED_HID_REPORT_TYPE_INPUT,
            &(kbAppState->rolloverRpt.modifierKeys),kbAppState->stdRptSize);
    // Snap the current BT clock for idle rate
    // rolloverRptTxInstant = hiddcfa_currentNativeBtClk();
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles key events targetted for the standard key report.
/// It updates the standard report with the given event.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param translationCode information on how the scan code is translated to a reported value
/////////////////////////////////////////////////////////////////////////////////
void kbapp_stdRptProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode)
{
    // Processing depends on whether the event is an up or down event
    if (upDownFlag == KEY_DOWN)
    {
        kbapp_stdRptProcEvtKeyDown(upDownFlag, keyCode, translationCode);
    }
    else
    {
        kbapp_stdRptProcEvtKeyUp(upDownFlag, keyCode, translationCode);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles a key down event for the standard key report. It adds the
/// given key to the report if it is not already present.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param translationCode information on how the scan code is translated to a reported value
/////////////////////////////////////////////////////////////////////////////////
void kbapp_stdRptProcEvtKeyDown(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode)
{
    uint8_t i;

    // Check if the key is already in the report
    for (i=0; i < kbAppState->keysInStdRpt; i++)
    {
        if (kbAppState->stdRpt.keyCodes[i] == translationCode)
        {
            // It is. Ignore the event
            return;
        }
    }

    // Check if the std report has room
    if (i < kbAppConfig.maxKeysInStdRpt)
    {
        // Add the new key to the report
        kbAppState->stdRpt.keyCodes[i] = translationCode;

        // Update the number of keys in the report
        kbAppState->keysInStdRpt++;

        // Flag that the standard key report has changed
        kbAppState->stdRptChanged = TRUE;
    }
    else
    {
        // No room in report. Call error handler
        kbapp_stdRptProcOverflow();
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles a key up event for the standard key report. It removes
/// the key from the report if it is already present. Otherwise it does nothing.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param translationCode information on how the scan code is translated to a reported value
/////////////////////////////////////////////////////////////////////////////////
void kbapp_stdRptProcEvtKeyUp(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode)
{
    uint8_t i;

    // Find the key in the current standard report
    for (i=0; i < kbAppState->keysInStdRpt; i++)
    {
        if (kbAppState->stdRpt.keyCodes[i] == translationCode)
        {
            // Found it. Remvove it by replacing it with the last key and
            // reducing the key count by one. We can do this because the
            // order of keys in the report is not important.
            kbAppState->keysInStdRpt--;
            // The following code is funky because compiler will not allow a simple expression
            kbAppState->stdRpt.keyCodes[i] = kbAppState->stdRpt.keyCodes[kbAppState->keysInStdRpt];
            // Clear the last key
            kbAppState->stdRpt.keyCodes[kbAppState->keysInStdRpt] = 0;

            // Flag that the standard key report has changed
            kbAppState->stdRptChanged = TRUE;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles overflow of the standard key report. This happens when
/// more than 6 (or the configured number of) standard keys are pressed at a time.
/// This function does a FW/HW reset in response.
/////////////////////////////////////////////////////////////////////////////////
void kbapp_stdRptProcOverflow(void)
{
    WICED_BT_TRACE("OverFlow\n");
    kbapp_stdErrRespWithFwHwReset();
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles modifier key events. It updates the modifier key bits
/// in the standard report structure.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param translationCode bitmap of the modifier key used for report generation
/////////////////////////////////////////////////////////////////////////////////
void kbapp_stdRptProcEvtModKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode)
{
    // Process the key event and update the modifier key bits in the standard report

    // Check if this is a down key or up key
    if (upDownFlag == KEY_DOWN)
    {
        // Key down. Update report only if the key state has changed
        if (!(kbAppState->stdRpt.modifierKeys & translationCode))
        {
            // Mark the appropriate modifier key as down
            kbAppState->stdRpt.modifierKeys |= translationCode;

            // Flag that the standard key report has changed
            kbAppState->stdRptChanged = TRUE;

            // Increment the number of mod keys that are down
            kbAppState->modKeysInStdRpt++;
        }
    }
    else
    {
        // Key up. Update report only if the key state has changed
        if (kbAppState->stdRpt.modifierKeys & translationCode)
        {
            // Mark the appropriate modifier key as down
            kbAppState->stdRpt.modifierKeys &= ~translationCode;

            // Flag that the standard key report has changed
            kbAppState->stdRptChanged = TRUE;

            // Decrement the number of mod keys that are down
            kbAppState->modKeysInStdRpt--;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits all modified key reports as long as we are not trying to
/// recover from an error. Note that it only transmits the standard report
/// in boot mode.
/////////////////////////////////////////////////////////////////////////////////
void kbapp_txModifiedKeyReports(void)
{
    // Only transmit reports if recovery is not in progress
    if (!kbAppState->recoveryInProgress)
    {
        // Transmit standard report
        if (kbAppState->stdRptChanged)
        {
            kbapp_stdRptSend();
        }

        // Transmit the rest of the reports only in report mode
        if (kbapp_protocol == PROTOCOL_REPORT)
        {
            // Transmit bit mapped report
            if (kbAppState->bitRptChanged)
            {
                kbapp_bitRptSend();
            }

            // Transmit sleep report
            if (kbAppState->slpRptChanged)
            {
                kbapp_slpRptSend();
            }

            // Transmit the func-lock report
            if (kbAppState->funcLockRptChanged)
            {
                kbapp_funcLockRptSend();
            }

            // Transmit scroll report
            if (kbAppState->scrollRptChanged)
            {
                kbapp_scrollRptSend();
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the scroll report over the interrupt channel
/////////////////////////////////////////////////////////////////////////////////
void kbapp_scrollRptSend(void)
{
    // Flag that the scroll report has not changed since it was sent the last time
    kbAppState->scrollRptChanged = FALSE;
    WICED_BT_TRACE("ScrollRpt\n");

    //set gatt attribute value here before sending the report
    if (kbAppState->scrollReport.motionAxis0>0)
    {   //USAGE (Volume Up)
        blekb_scroll_rpt = SCROLL_REPORT_VOLUME_UP;
    }
    else
    {   //USAGE (Volume Down)
        blekb_scroll_rpt = SCROLL_REPORT_VOLUME_DOWN;
    }

    wiced_ble_hidd_link_send_report(kbAppState->scrollReport.reportID,WICED_HID_REPORT_TYPE_INPUT,
            &blekb_scroll_rpt, 1); //kbAppConfig.scrollReportLen);

    blekb_scroll_rpt = 0;
    wiced_ble_hidd_link_send_report(kbAppState->scrollReport.reportID,WICED_HID_REPORT_TYPE_INPUT,
            &blekb_scroll_rpt, 1); //kbAppConfig.scrollReportLen);
}

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the func-lock report over the interrupt channel
/////////////////////////////////////////////////////////////////////////////////
void kbapp_funcLockRptSend(void)
{
    // Flag that the func-lock report has not changed since it was sent the last time
    kbAppState->funcLockRptChanged = FALSE;
    WICED_BT_TRACE("FuncLockRpt\n");

    //set gatt attribute value here before sending the report
    blekb_func_lock_rpt = kbAppState->funcLockRpt.status;

    // Send
    wiced_ble_hidd_link_send_report(kbAppState->funcLockRpt.reportID,WICED_HID_REPORT_TYPE_INPUT,
         &(kbAppState->funcLockRpt.status),sizeof(kbAppState->funcLockRpt.status));
}

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the sleep report over the interrupt channel
/////////////////////////////////////////////////////////////////////////////////
void kbapp_slpRptSend(void)
{
    // Flag that the sleep report has not changed since it was sent the last time
    kbAppState->slpRptChanged = FALSE;
    WICED_BT_TRACE("SleepRpt\n");
    //set gatt attribute value here before sending the report
    blekb_sleep_rpt = kbAppState->slpRpt.sleepVal;

    // Send the sleep report
    wiced_ble_hidd_link_send_report(kbAppState->slpRpt.reportID,WICED_HID_REPORT_TYPE_INPUT,
         &(kbAppState->slpRpt.sleepVal),sizeof(kbAppState->slpRpt.sleepVal));
}

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the bit mapped report over the interrupt channel
/////////////////////////////////////////////////////////////////////////////////
void kbapp_bitRptSend(void)
{
    // Flag that the bit mapped key report has not changed since it was sent the last time
    kbAppState->bitRptChanged = FALSE;
    WICED_BT_TRACE("BitRpt\n");

    //set gatt attribute value here before sending the report
    memcpy(blekb_bitmap_rpt, kbAppState->bitMappedReport.bitMappedKeys, kbAppState->bitReportSize);

    // Send the rpt.
    wiced_ble_hidd_link_send_report(kbAppState->bitMappedReport.reportID,WICED_HID_REPORT_TYPE_INPUT,
         kbAppState->bitMappedReport.bitMappedKeys,kbAppState->bitReportSize);
}


/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the battery report over the interrupt channel
/////////////////////////////////////////////////////////////////////////////////
void kbapp_batRptSend(void)
{
    //WICED_BT_TRACE("BASRpt\n");

    //set gatt attribute value here before sending the report
    battery_level = kbAppState->batRpt.level[0];

    if (WICED_SUCCESS == wiced_ble_hidd_link_send_report(kbAppState->batRpt.reportID,WICED_HID_REPORT_TYPE_INPUT,
                                                         kbAppState->batRpt.level,kbAppState->batRptSize))
    {
        wiced_hal_batmon_set_battery_report_sent_flag(WICED_TRUE);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the standard report over the interrupt channel and
/// marks internally that the report has been sent
/////////////////////////////////////////////////////////////////////////////////
void kbapp_stdRptSend(void)
{
    // Flag that the standard key report ha not changed since it was sent the last time
    kbAppState->stdRptChanged = FALSE;
    //WICED_BT_TRACE("\nStdRpt");

    //set gatt attribute value here before sending the report
    memcpy(blekb_key_std_rpt, &(kbAppState->stdRpt.modifierKeys), kbAppState->stdRptSize);

    // Send the report
    wiced_ble_hidd_link_send_report(kbAppState->stdRpt.reportID,WICED_HID_REPORT_TYPE_INPUT,
        &(kbAppState->stdRpt.modifierKeys),kbAppState->stdRptSize);

    // Snap the current BT clock for idle rate
    kbAppState->stdRptTxInstant = wiced_hidd_get_current_native_bt_clock();
}

// Keyscan interrupt
void kbapp_userKeyPressDetected(void* unused)
{
    //WICED_BT_TRACE("kbapp_userKeyPressDetected\n");
    kbAppState->keyInterrupt_On = wiced_hal_keyscan_is_any_key_pressed();

    // Poll the app.
    kbapp_pollReportUserActivity();
}


// Scroll/Quadrature interrupt
void kbapp_userScrollDetected(void* unused)
{
    //WICED_BT_TRACE("kbapp_userScrollDetected\n");
    //Poll the app.
    kbapp_pollReportUserActivity();
}

void kbapp_stateChangeNotification(uint32_t newState)
{
    int32_t flags;
    WICED_BT_TRACE("Transport state changed to %d\n", newState);

    kbAppState->allowSDS = WICED_FALSE;

    //stop allow Shut Down Sleep (SDS) timer
    if (wiced_is_timer_in_use(&blekb_allow_sleep_timer))
    {
        wiced_stop_timer(&blekb_allow_sleep_timer);
    }

    //stop conn param update timer
    if (wiced_is_timer_in_use(&blekb_conn_param_update_timer))
    {
        wiced_stop_timer(&blekb_conn_param_update_timer);
    }

    kbapp_LED_off(BLEKB_LED_BLUE);

    if(newState == BLEHIDLINK_CONNECTED)
    {
        //get host client configuration characteristic descriptor values
        flags = wiced_ble_hidd_host_info_get_flags(ble_hidd_link.gatts_peer_addr, ble_hidd_link.gatts_peer_addr_type);
        if(flags != -1)
        {
            WICED_BT_TRACE("host config flag:%08x\n",flags);
            kbapp_updateGattMapWithNotifications(flags);
        }
#ifndef  TESTING_USING_HCI
        // enable ghost detection
        wiced_hal_keyscan_enable_ghost_detection(TRUE);
#endif
        //enable application polling
        wiced_ble_hidd_link_enable_poll_callback(WICED_TRUE);

        if(firstTransportStateChangeNotification)
        {
            //Wake up from shutdown sleep (SDS) and already have a connection then allow SDS in 1 second
            //This will allow time to send a key press.
            wiced_start_timer(&blekb_allow_sleep_timer,1000); // 1 second. timeout in ms
        }
        else
        {
            //We connected after power on reset
            //Start 20 second timer to allow time to setup connection encryption before allowing shutdown sleep (SDS).
            wiced_start_timer(&blekb_allow_sleep_timer,20000); //20 seconds. timeout in ms

            //start 15 second timer to make sure connection param update is requested before SDS
            wiced_start_timer(&blekb_conn_param_update_timer,15000); //15 seconds. timeout in ms
        }
    }
    else if(newState == BLEHIDLINK_DISCONNECTED)
    {
        //allow Shut Down Sleep (SDS) only if we are not attempting reconnect
        if (!wiced_is_timer_in_use(&ble_hidd_link.reconnect_timer))
            wiced_start_timer(&blekb_allow_sleep_timer,2000); // 2 seconds. timeout in ms

#ifndef  TESTING_USING_HCI
        // disable Ghost detection
        wiced_hal_keyscan_enable_ghost_detection(FALSE);
#endif
        // disable application polling
        wiced_ble_hidd_link_enable_poll_callback(WICED_FALSE);
    }
    else if (newState == BLEHIDLINK_DISCOVERABLE)
    {
        kbapp_LED_on(BLEKB_LED_BLUE);
    }
    else if ((newState == BLEHIDLINK_ADVERTISING_IN_uBCS_DIRECTED) || (newState == BLEHIDLINK_ADVERTISING_IN_uBCS_UNDIRECTED))
    {
        kbAppState->allowSDS = WICED_TRUE;
    }

    if(firstTransportStateChangeNotification)
        firstTransportStateChangeNotification = 0;
}

void kbapp_batLevelChangeNotification(uint32_t newLevel)
{
    WICED_BT_TRACE("bat level changed to %d\n", newLevel);

    if (kbapp_protocol == PROTOCOL_REPORT)
    {
        kbAppState->batRpt.level[0] = newLevel;
        kbapp_batRptSend();
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function clears all dynamic reports defined by the standard keyboard application
/// except the func-lock report. These are
///     - Standard report
///     - Bit mapped report
///     - Sleep report
///     - Pin code report
///     - Scroll report
/// The reports are also flagged as unchanged since last transmission.
/// The func-lock report is not cleared since it is a "state of func-lock" report
/// rather than the "state of func-lock key" report. It is prepared and sent
/// whenever the func-lock state changes and doesn't have the current state of the
/// func-lock key that needs to be cleared.
/////////////////////////////////////////////////////////////////////////////////
void kbapp_clearAllReports(void)
{
    kbapp_stdRptClear();
    kbapp_bitRptClear();
    kbapp_slpRptClear();
    kbapp_scrollRptClear();

    // Flag that the reports have not been sent
    kbAppState->bitRptChanged = kbAppState->slpRptChanged = kbAppState->stdRptChanged =
        kbAppState->scrollRptChanged = kbAppState->funcLockRptChanged = FALSE;
}

////////////////////////////////////////////////////////////////////////////////
/// This function flushes all queued events and unprocessed fractional scroll
/// activity. It also clears all reports.
////////////////////////////////////////////////////////////////////////////////
void kbapp_flushUserInput(void)
{
    // Flush any partial scroll count
    kbAppState->scrollFractional = 0;

    // Flag that recovery is no longer in progress
    kbAppState->recoveryInProgress = 0;

    // Clear all dynamic reports
    kbapp_clearAllReports();

    // Flush the event fifo
    // wiced_hidd_event_queue_flush(&kbAppState->eventQueue);
}

/////////////////////////////////////////////////////////////////////////////////
/// This function initializes the led report.
/////////////////////////////////////////////////////////////////////////////////
void kbapp_ledRptInit(void)
{
    kbAppState->ledReport.reportID = kbAppConfig.ledReportID;
    kbAppState->ledReport.ledStates = kbAppConfig.defaultLedState;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function clears the standard key report including internal count of
/// standard and modifier keys
/////////////////////////////////////////////////////////////////////////////////
void kbapp_stdRptClear(void)
{
    // Indicate that there are no keys in the standard report
    kbAppState->modKeysInStdRpt = kbAppState->keysInStdRpt = 0;

    // Initialize the std report completely
    kbAppState->stdRpt.reportID = kbAppConfig.stdRptID;
    kbAppState->stdRpt.modifierKeys = 0;
    kbAppState->stdRpt.reserved = 0;
    memset((void*)&kbAppState->stdRpt.keyCodes, 0, sizeof(kbAppState->stdRpt.keyCodes));
}

/////////////////////////////////////////////////////////////////////////////////
/// This function initializes the rollover report
/////////////////////////////////////////////////////////////////////////////////
void kbapp_stdRptRolloverInit(void)
{
    kbAppState->rolloverRpt.reportID = kbAppConfig.stdRptID;
    kbAppState->rolloverRpt.modifierKeys = 0;
    kbAppState->rolloverRpt.reserved = 0;
    memset((void*)&kbAppState->rolloverRpt.keyCodes,
           KEYRPT_CODE_ROLLOVER,
           sizeof(kbAppState->rolloverRpt.keyCodes));
}

/////////////////////////////////////////////////////////////////////////////////
/// This function clears the sleep report
/////////////////////////////////////////////////////////////////////////////////
void kbapp_slpRptClear(void)
{
    // Initialize the sleep report completely
    kbAppState->slpRpt.reportID = kbAppConfig.sleepReportID;
    kbAppState->slpRpt.sleepVal = 0;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function clears the bitmapped key report
/////////////////////////////////////////////////////////////////////////////////
void kbapp_bitRptClear(void)
{
    // Indicate that there are no keys in the bit report
    kbAppState->keysInBitRpt = 0;

    // Initialize the bit mapped report completely
    kbAppState->bitMappedReport.reportID = kbAppConfig.bitReportID;
    memset((void*)&kbAppState->bitMappedReport.bitMappedKeys, 0, sizeof(kbAppState->bitMappedReport.bitMappedKeys));
}


/////////////////////////////////////////////////////////////////////////////////
/// This function initializes the func-lock report. The header and report ID are
/// set and the status field is set based on the current state of func-lock
/////////////////////////////////////////////////////////////////////////////////
void kbapp_funcLockRptInit(void)
{
    // Set the report ID to the configured value
    kbAppState->funcLockRpt.reportID = kbAppConfig.funcLockReportID;

    // Set the current state of func-lock as well as the event flag
    kbAppState->funcLockRpt.status = kbAppState->funcLockInfo.state | 2;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles toggles the func lock state and updates the func-lock
/// report but doesn't send it. Note that it assumes that the func lock report
/// is sent in a specific format
/////////////////////////////////////////////////////////////////////////////////
void kbapp_funcLockToggle(void)
{
    // Toggle func lock state
    kbapp_funcLock_state = kbAppState->funcLockInfo.state = (kbAppState->funcLockInfo.state == FUNC_LOCK_STATE_OFF ? FUNC_LOCK_STATE_ON:FUNC_LOCK_STATE_OFF);

    // Update the func lock report. Always set the func-lock event flag
    kbAppState->funcLockRpt.status = kbAppState->funcLockInfo.state | 2;

    // Mark the funct-lock report as changed
    kbAppState->funcLockRptChanged = TRUE;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles func-lock dependent key events. It uses the current
/// func-lock state to determine whether the key should be sent to the bit
/// key handler or the std key handler. Note that up keys
/// are sent to both handlers to ensure that
/// up keys are not lost after a boot<->report protocol switch. Also
/// note that func-lock is assumed to be down in boot mode. Also
/// note that a down key set the func-lock toglle on key up flag
/// unconditionally. This allows func-lock to be used as a temporary override of
/// its own state.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param funcLockDepKeyTableIndex index in the func-lock dependent key
///        description table
/////////////////////////////////////////////////////////////////////////////////
void kbapp_funcLockProcEvtDepKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t funcLockDepKeyTableIndex)
{
    // Check if this is a down key or up key
    if (upDownFlag == KEY_DOWN)
    {
        // Check if we are in boot mode or the func-lock state is down
        if (kbAppState->funcLockInfo.state == FUNC_LOCK_STATE_ON ||
            kbapp_protocol == PROTOCOL_BOOT)
        {
            // Pass this to the standard report handler
            kbapp_stdRptProcEvtKey(upDownFlag, keyCode, kbFuncLockDepKeyTransTab[funcLockDepKeyTableIndex].stdRptCode);
        }
        else
        {
            // Pass it to the bit report handler
            kbapp_bitRptProcEvtKey(upDownFlag, keyCode, kbFuncLockDepKeyTransTab[funcLockDepKeyTableIndex].bitRptCode);
        }

        // Flag that we had a func lock dependent key pressed. Note that this will only be used
        // if func-lock is down so we don't need to check for it.
        kbAppState->funcLockInfo.toggleStateOnKeyUp = TRUE;
    }
    else
    {
        // Key up. Send it to both the standard and bit mapped report handler
        kbapp_stdRptProcEvtKey(upDownFlag, keyCode, kbFuncLockDepKeyTransTab[funcLockDepKeyTableIndex].stdRptCode);
        kbapp_bitRptProcEvtKey(upDownFlag, keyCode, kbFuncLockDepKeyTransTab[funcLockDepKeyTableIndex].bitRptCode);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function initializes the scroll report. The header and report ID are
/// set and the rest of the report is set to 0
/////////////////////////////////////////////////////////////////////////////////
void kbapp_scrollRptClear(void)
{
    // Initialize the scroll report
    memset(&kbAppState->scrollReport, 0, sizeof(kbAppState->scrollReport));

    // Fill in the report ID information
    kbAppState->scrollReport.reportID = kbAppConfig.scrollReportID;

    // Flag that the scroll report has not changed since it was sent the last time
    kbAppState->scrollRptChanged = FALSE;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function implements the rxSetReport function defined by
/// the HID application to handle "Set Report" messages.
/// This function looks at the report ID and passes the message to the
/// appropriate handler.
/// \param hidTransport transport over which any responses should be sent
/// \param reportType type of incoming report, e.g. feature
/// \param reportId of the incoming report
/// \param payload pointer to data that came along with the set report request
///          after the report ID
/// \param payloadSize size of the payload excluding the report ID
/// \return TSC_SUCCESS or TSC_ERR* if the message has a problem
/////////////////////////////////////////////////////////////////////////////////
void kbapp_setReport(wiced_hidd_report_type_t reportType,
                     uint8_t reportId,
                     void *payload,
                     uint16_t payloadSize)
{
    // We only handle output report types
    if (reportType == WICED_HID_REPORT_TYPE_OUTPUT)
    {
        // Pass to handler based on report ID. Ensure that report ID is in the payload
        if (payloadSize >= 1)
        {
            // Demux on report ID
            if(reportId == kbAppConfig.ledReportID)
            {
                kbAppState->ledReport.ledStates = blekb_kb_output_rpt = *((uint8_t *)payload);
                WICED_BT_TRACE("KB LED report : %d\n", kbAppState->ledReport.ledStates);

#ifdef LED_USE_PWM
                if (kbAppState->ledReport.ledStates & 0x01)
                {
                    kbapp_LED_on(BLEKB_PWM_LED1);
                }
                else
                {
                    kbapp_LED_off(BLEKB_PWM_LED1);
                }

                if (kbAppState->ledReport.ledStates & 0x02)
                {
                    kbapp_LED_on(BLEKB_PWM_LED2);
                }
                else
                {
                    kbapp_LED_off(BLEKB_PWM_LED2);
                }
#else
                //CAPS LED
                if (kbAppState->ledReport.ledStates & 0x02)
                {
                    kbapp_LED_on(BLEKB_LED_CAPS);
                }
                else
                {
                    kbapp_LED_off(BLEKB_LED_CAPS);
                }
#endif
            }
        }
    }

#ifdef PTS_HIDS_CONFORMANCE_TC_CW_BV_03_C
    blekb_connection_ctrl_rpt = *((uint8_t *)payload);
    WICED_BT_TRACE("PTS_HIDS_CONFORMANCE_TC_CW_BV_03_C write val: %d \n", blekb_connection_ctrl_rpt);
#endif
}

void kbapp_ctrlPointWrite(wiced_hidd_report_type_t reportType,
                          uint8_t reportId,
                          void *payload,
                          uint16_t payloadSize)
{
    WICED_BT_TRACE("disconnecting\n");

    wiced_ble_hidd_link_disconnect();
}


void kbapp_clientConfWriteRptStd(wiced_hidd_report_type_t reportType,
                                 uint8_t reportId,
                                 void *payload,
                                 uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t  indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

    WICED_BT_TRACE("clientConfWriteRptStd\n");

    kbapp_updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_STD_RPT);
}

void kbapp_clientConfWriteRptBitMapped(wiced_hidd_report_type_t reportType,
                                       uint8_t reportId,
                                       void *payload,
                                       uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t  indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

    WICED_BT_TRACE("clientConfWriteRptBitMapped\n");

    kbapp_updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_BIT_MAPPED_RPT);
}

void kbapp_clientConfWriteRptSlp(wiced_hidd_report_type_t reportType,
                                 uint8_t reportId,
                                 void *payload,
                                 uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t  indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

    WICED_BT_TRACE("clientConfWriteRptSlp\n");
    kbapp_updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_SLP_RPT);
}

void kbapp_clientConfWriteRptFuncLock(wiced_hidd_report_type_t reportType,
                                      uint8_t reportId,
                                      void *payload,
                                      uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t  indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

    WICED_BT_TRACE("clientConfWriteRptFuncLock\n");

    kbapp_updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_FUNC_LOCK_RPT);
}

void kbapp_clientConfWriteScroll(wiced_hidd_report_type_t reportType,
                                      uint8_t reportId,
                                      void *payload,
                                      uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t  indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

    WICED_BT_TRACE("clientConfWriteScroll\n");

    kbapp_updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_SCROLL_RPT);
}


void kbapp_clientConfWriteBootMode(wiced_hidd_report_type_t reportType,
                                   uint8_t reportId,
                                   void *payload,
                                   uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t  indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

    WICED_BT_TRACE("clientConfWriteBootMode\n");

    kbapp_updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_BOOT_RPT);
}

void kbapp_clientConfWriteBatteryRpt(wiced_hidd_report_type_t reportType,
                                     uint8_t reportId,
                                     void *payload,
                                     uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t  indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

    WICED_BT_TRACE("clientConfWriteBatteryRpt\n");

    kbapp_updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_BATTERY_RPT);
}


void kbapp_setProtocol(wiced_hidd_report_type_t reportType,
                                   uint8_t reportId,
                                   void *payload,
                                   uint16_t payloadSize)
{
    kbapp_protocol = *((uint8_t *)payload);

    WICED_BT_TRACE("New Protocol = %d\n", kbapp_protocol);

    if(kbapp_protocol == PROTOCOL_REPORT)
    {
        // If the current protocol is report, register the report mode table
        wiced_blehidd_register_report_table(reportModeGattMap, sizeof(reportModeGattMap)/sizeof(reportModeGattMap[0]));
    }
    else
    {
        //otherwise register the boot mode table
        wiced_blehidd_register_report_table(bootModeGattMap, sizeof(bootModeGattMap)/sizeof(bootModeGattMap[0]));
    }
}

void kbapp_updateClientConfFlags(uint16_t enable, uint16_t featureBit)
{
    kbapp_updateGattMapWithNotifications(wiced_ble_hidd_host_info_update_flags(ble_hidd_link.gatts_peer_addr, ble_hidd_link.gatts_peer_addr_type, enable,featureBit));
}

void kbapp_updateGattMapWithNotifications(uint16_t flags)
{
    uint8_t i = 0;
    wiced_blehidd_report_gatt_characteristic_t* map = bootModeGattMap;

    blehostlist_flags = flags;

    //update characteristic_client_configuration for gatt read req
    for (i=0; i<MAX_NUM_CLIENT_CONFIG_NOTIF; i++)
    {
        characteristic_client_configuration[i] = (flags >> i) & 0x0001;
    }

    // Set the boot mode report first
    for(i = 0; i < sizeof(bootModeGattMap)/sizeof(bootModeGattMap[0]); i++)
    {
        if(map->reportType == WICED_HID_REPORT_TYPE_INPUT &&
            map->clientConfigBitmap == KBAPP_CLIENT_CONFIG_NOTIF_BOOT_RPT)
        {
            // If this is the boot mode input report we are looking for,
            // set/clear based on the new flags.
            map->sendNotification =
                ((flags & KBAPP_CLIENT_CONFIG_NOTIF_BOOT_RPT) == KBAPP_CLIENT_CONFIG_NOTIF_BOOT_RPT) ? TRUE : FALSE;

            break;
        }

        map++;
    }

    // not update the report mode map
    map = reportModeGattMap;

    for(i = 0; i < sizeof(reportModeGattMap)/sizeof(reportModeGattMap[0]); i++)
    {
        if(map->reportType == WICED_HID_REPORT_TYPE_INPUT)
        {
            map->sendNotification =
                ((flags & map->clientConfigBitmap) == map->clientConfigBitmap) ? TRUE : FALSE;
        }

        map++;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Sleep permit query to check if sleep (normal or SDS) is allowed and sleep time
///
/// \param type - sleep poll type
///
/// \return   sleep permission or sleep time, depending on input param
////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t kbapp_sleep_handler(wiced_sleep_poll_type_t type )
{
    uint32_t ret = WICED_SLEEP_NOT_ALLOWED;

#ifndef  TESTING_USING_HCI
    // Check if keys are pressed before deciding whether sleep is allowed
    kbAppState->keyInterrupt_On = wiced_hal_keyscan_is_any_key_pressed();
#endif

    switch(type)
    {
        case WICED_SLEEP_POLL_TIME_TO_SLEEP:
            ret = WICED_SLEEP_MAX_TIME_TO_SLEEP;

            //if we are in the middle of kescan recovery, no sleep
            if (kbAppState->recoveryInProgress)
            {
                ret = 0;
            }

            break;

        case WICED_SLEEP_POLL_SLEEP_PERMISSION:
            ret = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;

            if (!kbAppState->allowSDS)
            {
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
            }

            //if key is not released, no Shut Down Sleep (SDS)
            if (kbAppState->keyInterrupt_On)
            {
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
            }

#ifdef OTA_FIRMWARE_UPGRADE
            if ( wiced_ota_fw_upgrade_is_active() )
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
#endif

            break;

    }

    return ret;
}


////////////////////////////////////////////////////////////////////////////////////
/// restore contents from Always On Memory. This should be called when wake up from SDS
///////////////////////////////////////////////////////////////////////////////////
void kbapp_aon_restore(void)
{
    if (!wiced_hal_mia_is_reset_reason_por())
    {
        kbAppState->funcLockInfo.state = kbapp_funcLock_state;
        wiced_ble_hidd_link_aon_action_handler(BLEHIDLINK_RESTORE_FROM_AON);
    }
}

void kbapp_LED_init(void)
{
#ifdef LED_USE_PWM
    uint16_t init_value = 0x3FF - BLEKB_PWM_STEPS;
    uint16_t toggle_val = 0x3FF - 0;

    wiced_hal_gpio_configure_pin(BLEKB_PWM_LED1, GPIO_OUTPUT_ENABLE, 0);
    wiced_hal_gpio_configure_pin(BLEKB_PWM_LED2, GPIO_OUTPUT_ENABLE, 0);

    // base clock 24MHz,
    wiced_hal_aclk_enable(256000, ACLK1, ACLK_FREQ_24_MHZ);
    wiced_hal_pwm_start(BLEKB_PWM_LED1 - BLEKB_PWM_LED_BASE , PMU_CLK, toggle_val, init_value, 0);
    wiced_hal_pwm_start(BLEKB_PWM_LED2 - BLEKB_PWM_LED_BASE , PMU_CLK, toggle_val, init_value, 0);
#else
    if (wiced_hal_mia_is_reset_reason_por())
    {
        wiced_hal_gpio_configure_pin(BLEKB_LED_CAPS, GPIO_OUTPUT_ENABLE, 1);
        wiced_hal_gpio_configure_pin(BLEKB_LED_BLUE, GPIO_OUTPUT_ENABLE, 1);
        wiced_hal_gpio_configure_pin(BLEKB_LED_GREEN, GPIO_OUTPUT_ENABLE, 0);
        wiced_hal_gpio_configure_pin(BLEKB_LED_RED, GPIO_OUTPUT_ENABLE, 1);
    }

    //maintain LED state during SDS
    wiced_hal_gpio_slimboot_reenforce_cfg(BLEKB_LED_CAPS, GPIO_OUTPUT_ENABLE);
    wiced_hal_gpio_slimboot_reenforce_cfg(BLEKB_LED_BLUE, GPIO_OUTPUT_ENABLE);
    wiced_hal_gpio_slimboot_reenforce_cfg(BLEKB_LED_GREEN, GPIO_OUTPUT_ENABLE);
    wiced_hal_gpio_slimboot_reenforce_cfg(BLEKB_LED_RED, GPIO_OUTPUT_ENABLE);
#endif
}


void kbapp_LED_on(uint8_t gpio)
{
#ifdef LED_USE_PWM
    /*
     * turn on LED by setting toggle value value to (0x3FF - BLEKB_PWM_STEPS/2)
     * BLEKB_PWM_STEPS(1000) to makes 48 kHz
    */
    uint16_t init_value = 0x3FF - BLEKB_PWM_STEPS;
    uint16_t toggle_val = 0x3FF - BLEKB_PWM_STEPS/10; // On(0): 10%, off(1):90%

    wiced_hal_pwm_change_values(gpio - BLEKB_PWM_LED_BASE, toggle_val,init_value);
#else
    wiced_hal_gpio_set_pin_output(gpio, 0);
#endif
}


void kbapp_LED_off(uint8_t gpio)
{
#ifdef LED_USE_PWM
    /*
     * turn off LED by setting toggle value to max(0x3FF)
     * GPIO will be set 0.
    */
    uint16_t init_value = 0x3FF - BLEKB_PWM_STEPS;
    uint16_t toggle_val = 0x3FF - 0;

    wiced_hal_pwm_change_values(gpio - BLEKB_PWM_LED_BASE, toggle_val,init_value);
#else
    wiced_hal_gpio_set_pin_output(gpio, 1);
#endif
}

#ifdef OTA_FIRMWARE_UPGRADE
////////////////////////////////////////////////////////////////////////////////
/// Process OTA firmware upgrade status change event
////////////////////////////////////////////////////////////////////////////////
void blekbapp_ota_fw_upgrade_status(uint8_t status)
{
    WICED_BT_TRACE("OTAFU status:%d\n", status);

    switch (status)
    {
    case OTA_FW_UPGRADE_STATUS_STARTED:             // Client started OTA firmware upgrade process
        WICED_BT_TRACE("allow slave latency 0\n");
        wiced_blehidd_allow_slave_latency(FALSE);
        break;

    case OTA_FW_UPGRADE_STATUS_ABORTED:             // Aborted or failed verification */
        WICED_BT_TRACE("allow slave latency 1\n");
        wiced_blehidd_allow_slave_latency(TRUE);
        break;

    case OTA_FW_UPGRADE_STATUS_COMPLETED:           // firmware upgrade completed, will reboot
        // ToDo should probably clean disconnect so that host does not wait for link super before start scanning
        break;
    }
}
#endif /* OTA_FIRMWARE_UPGRADE */

