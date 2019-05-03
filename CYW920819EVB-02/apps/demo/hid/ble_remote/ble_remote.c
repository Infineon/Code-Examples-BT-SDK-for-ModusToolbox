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
* BLE Remote Control
*
* The BLE Remote Control application is a single chip SoC compliant with HID over GATT Profile (HOGP).
* Supported features include key, microphone (voice over HOGP), Infrared Transmit (IR TX), TouchPad.
*
* During initialization the app registers with LE stack, WICED HID Device Library and
* keyscan and external HW peripherals to receive various notifications including
* bonding complete, connection status change, peer GATT request/commands and
* interrupts for key pressed/released, ADC audio, and Touchpad.
* Press any key will start LE advertising. When device is successfully bonded, the app
* saves bonded host's information in the NVRAM.
* When user presses/releases any key, a key report will be sent to the host.
* On connection up or battery level changed, a battery report will be sent to the host.
* When battery level is bellowed shutdown voltage, device will critical shutdown.
* When user presses and holds microphone key, voice streaming starts until user releases
* microphone key.
* When user presses and holds power key, IR TX starts until power key is released.
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
* 1. Plug the CYW920819EVB_02 board or the 20819A1 Remote Control HW into your computer
* 2. Put on jumper to bypass Serial Flash (i.e. jumper on J5 in CYW920819EVB_02 board), then power up the board or Remote Control HW.
* 3. Remove the jumper so that download procedure bellowed can write to Serial Flash.
* 4. Build and download the application (to the EVAL board or Remote Control HW)
* 5. If download failed due to not able to detecting device, just repeat step 4 again.
* 6. Unplug the EVAL board or Remote Control HW from your computer (i.e. unplug the UART cable)
* 7. Power cycle the EVAL board or Remote Control HW.
* 8. Press any key to start LE advertising, then pair with a TV
*    If using the CYW920819EVB_02 board, use a fly wire to connect GPIO P0 and P11 to simulate key '0' press,
*     and remove the wire to simulate key release.
* 9. Once connected, it becomes the remote control of the TV.
* 10. If you have the 20819A1 Remote Control HW:
*     - Press and hold microphone key, voice streaming starts until the key is released.
*     - Touch touchpad, touchpad report will be sent to the TV.
*
* In case what you have is the WICED EVAL board, you can either use fly wire to connect to GPIOs to simulate key press and release.
* Or using the ClientControl tool in the tools to simulate key press and release.*
* 1. Plug the WICED EVAL board into your computer
* 2. Build and download the application (to the WICED board)
* 3. If failed to download due to device not detected, just repeat step 2 again.
* 4. Press any key to start LE advertising, then pair with a TV
*     Use a fly wire to connect GPIO P0 and P11 to simulate key '0' press,
*     and remove the wire to simulate key release.
* 5. Once connected, it becomes the remote control of the TV.
*
* To use ClientControl tool + WICED EVAL board to simulate key press and release.
* NOTE: Make sure you use "TESTING_USING_HCI=1" in application settings.
* In ModusToolbox, select right click on app and select 'Change Application Settings'
*
* 1~3. same download procedure as above
* 4. Run ClientControl.exe.
* 5. Choose 3M as Baudrate and select the serial port in ClientControl tool window.
* 6. Press Reset button on the board and open the port.
* 7. Press "Enter Pairing Mode"or "Connect" to start LE advertising, then pair with a PC or Tablet
* 8. Once connected, it becomes the remote control of the TV.
*  - Select Interrupt channel, Input report, enter the contents of the report
*    and click on the Send button, to send the report.  For example to send
*    key down event when key '1' is pushed, report should be
*    01 00 00 1e 00 00 00 00 00.  All keys up 01 00 00 00 00 00 00 00 00.
*    Please make sure you always send a key up report following key down report.
*/

#include "gki_target.h"
#include "wiced_bt_cfg.h"
#include "ble_remote_gatts.h"
#include "ble_remote.h"
#include "wiced_bt_gatt.h"
#include "wiced_hal_mia.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_keyscan.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_sdp.h"
#ifdef SUPPORT_TOUCHPAD
#include "touchpad/touchPad.h"
#endif
#include "appDefs.h"
#include "wiced_hal_batmon.h"
#include "wiced_memory.h"

#ifdef OTA_FIRMWARE_UPGRADE
#include "wiced_bt_ota_firmware_upgrade.h"
#endif

#ifdef SUPPORT_AUDIO
#include "wiced_hidd_micaudio.h"

#define AUDIO_FIFO_CNT    10
#define audioIsActive()   (wiced_hidd_mic_audio_is_active())

wiced_hidd_voice_report_t audioData[AUDIO_FIFO_CNT] = {0,};
uint16_t    dataCount[AUDIO_FIFO_CNT] = {0,};
wiced_hidd_microphone_config_t audioConfig;

extern wiced_hidd_microphone_enhanced_config_t blehid_audiocfg;
#else
#define audioIsActive()     0
#endif

#ifdef SUPPORT_TOUCHPAD
#define VIRUAL_KEY_INDEX_BASE           20       // virtual key index base
#define touchpadIsActive()(touchpad && touchpad->isActive())

TouchPadIf_t     * touchpad   = NULL;
uint8_t  vKeyIndexCode[ZONE_MAX_COUNT]  = {VKEY_INDEX_CENTER, VKEY_INDEX_RIGHT, VKEY_INDEX_LEFT, VKEY_INDEX_DOWN, VKEY_INDEX_UP};
uint8_t  bleremote_touchpad_rpt[TOUCHPAD_RPT_PAYLOAD_SIZE] = {0, };
#else
#define touchpadIsActive()    0
#endif

#ifdef SUPPORT_MOTION
#define motionIsEnabled() (motionsensor && motionsensor->isEnabled())
#define motionIsActive()  (motionsensor && motionsensor->isActive())

MotionSensorICM_c        * motionsensor  = NULL;
uint8_t bleremote_motion_rpt[MOTIONRPT_MAX_DATA] = {0, };
extern MotionSensorICM_mode_t motion_mode;
#else
#define motionIsEnabled() 0
#define motionIsActive()  0
#endif

#ifdef SUPPORT_IR
#define irtxIsActive()    (ir && ir->isActive())

uint8_t pending_IR = FALSE;
uint8_t pending_IR_code;
uint8_t pending_IR_repeat = FALSE;
uint8_t pending_IR_repeat_count;
tIrTxDriverVtbl  *ir = NULL;
wiced_timer_t allow_irtx_timer;
#endif

#ifdef SUPPORTING_FINDME
tAppAlertConfig appAlert_cfg =
{
    // mild alert Buz config
  {
    {WICED_PWMBUZ_FREQ_MANUAL, // WICED_PWMBUZ_4000,
#if 0 //4K
    65473,              // init_value
    65503,             // toggle_val
#else // 4.2K
    65476,              // init_value
    65505,             // toggle_val
#endif
    1000,          // buz_on_ms
    2000,          // buz_off_ms
    10},          //repeat_nun

    // high alert Buz config
    {WICED_PWMBUZ_FREQ_MANUAL, // WICED_PWMBUZ_4000,  // freq
#if 0 //4K
    65473,              // init_value
    65503,             // toggle_val
#else // 4.2K
    65476,              // init_value
    65505,             // toggle_val
#endif
    300,           //buz_on_ms
    300,           // buz_off_ms
    20}         //repeat_nun
  },
    // mild alert Led config
    {{1000,        // led_on_ms
    2000,          // led_off_ms
    10},          // repeat_nun

    // high alert Led config
    {300,          // led_on_ms
    300,           // led_off_ms
    20}},         // repeat_nun
};

tAppFindmeState *appFindmeState;
wiced_timer_t findme_led_timer;
wiced_timer_t findme_buz_timer;
#endif /* SUPPORTING_FINDME */


wiced_blehidd_report_gatt_characteristic_t bleRemoteReportModeGattMap[] =
{
    // STD keyboard Input report
    {STD_KB_REPORT_ID   ,   WICED_HID_REPORT_TYPE_INPUT ,   HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_STD_INPUT_VAL,          FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_STD_RPT},
    // Std output report
    {STD_KB_REPORT_ID   ,   WICED_HID_REPORT_TYPE_OUTPUT,   HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_STD_OUTPUT_VAL,         FALSE,bleremoteapp_setReport, KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    //Bitmapped report
    {BITMAPPED_REPORT_ID,   WICED_HID_REPORT_TYPE_INPUT ,   HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_BITMAP_VAL,             FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_BIT_MAPPED_RPT},
    // Battery Input report
    {BATTERY_REPORT_ID  ,   WICED_HID_REPORT_TYPE_INPUT ,   HANDLE_BLEREMOTE_BATTERY_SERVICE_CHAR_LEVEL_VAL,                FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_BATTERY_RPT},
    //motion report
    {MOTION_REPORT_ID   ,   WICED_HID_REPORT_TYPE_INPUT ,   HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_MOTION_VAL,             FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_MOTION_RPT},
    //user defined 0 key report
    {0x0A,                  WICED_HID_REPORT_TYPE_INPUT ,   HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0_VAL,     FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_USER_DEFINED_KEY_RPT},
    //voice report
    {WICED_HIDD_VOICE_REPORT_ID, WICED_HID_REPORT_TYPE_INPUT, HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_VAL,            FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_VOICE_RPT},
    //voice ctrl report
    {WICED_HIDD_VOICE_CTL_REPORT_ID,   WICED_HID_REPORT_TYPE_INPUT ,   HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT_VAL,   FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_VOICE_CTRL_RPT},
    {WICED_HIDD_VOICE_CTL_REPORT_ID,   WICED_HID_REPORT_TYPE_FEATURE,  HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_FEA_VAL,     FALSE,bleremoteapp_setReport, KBAPP_CLIENT_CONFIG_NOTIF_NONE},
#ifdef SUPPORT_TOUCHPAD
    //touchpad report
    {RPT_ID_IN_ABS_XY,      WICED_HID_REPORT_TYPE_INPUT ,   HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_TOUCHPAD_VAL,           FALSE,NULL, KBAPP_CLIENT_CONFIG_NOTIF_TOUCHPAD_RPT},
#endif

    //connection control feature
    {0xCC, WICED_HID_REPORT_TYPE_FEATURE,       HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL_VAL,                FALSE,bleremoteapp_setReport     ,KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, WICED_HID_REPORT_TYPE_OTHER  ,       HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_HID_CTRL_POINT_VAL,                 FALSE,bleremoteapp_ctrlPointWrite,KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, WICED_HID_CLIENT_CHAR_CONF, HANDLE_BLEREMOTE_BATTERY_SERVICE_CHAR_CFG_DESCR,                            FALSE,bleremoteapp_clientConfWriteBatteryRpt  ,KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, WICED_HID_CLIENT_CHAR_CONF, HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_STD_INPUT_CHAR_CFG_DESCR,           FALSE,bleremoteapp_clientConfWriteRptStd      ,KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, WICED_HID_CLIENT_CHAR_CONF, HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_BITMAP_CHAR_CFG_DESCR,              FALSE,bleremoteapp_clientConfWriteRptBitMapped,KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, WICED_HID_CLIENT_CHAR_CONF, HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_MOTION_CHAR_CFG_DESCR,              FALSE,bleremoteapp_clientConfWriteRptMotion,          KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, WICED_HID_CLIENT_CHAR_CONF, HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0_CHAR_CFG_DESCR,      FALSE,bleremoteapp_clientConfWriteRptUserDefinedKey,  KBAPP_CLIENT_CONFIG_NOTIF_NONE},
#ifdef SUPPORT_AUDIO
    {0xFF, WICED_HID_CLIENT_CHAR_CONF, HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_CHAR_CFG_DESCR,               FALSE,bleremoteapp_clientConfWriteRptVoice,           KBAPP_CLIENT_CONFIG_NOTIF_NONE},
    {0xFF, WICED_HID_CLIENT_CHAR_CONF, HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT_CHAR_CFG_DESCR,    FALSE,bleremoteapp_clientConfWriteRptVoiceCtrl,       KBAPP_CLIENT_CONFIG_NOTIF_NONE},
#endif
#ifdef SUPPORT_TOUCHPAD
    {0xFF, WICED_HID_CLIENT_CHAR_CONF, HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_TOUCHPAD_CHAR_CFG_DESCR,            FALSE,bleremoteapp_clientConfWriteRptTouchpad,        KBAPP_CLIENT_CONFIG_NOTIF_NONE},
#endif

};

extern wiced_bt_cfg_settings_t wiced_bt_hid_cfg_settings;
extern uint16_t blehostlist_flags;
extern wiced_bool_t blehidlink_connection_param_updated;


extern KbAppConfig kbAppConfig;
extern KbKeyConfig kbKeyConfig[];
extern uint8_t kbKeyConfig_size;
extern RemoteAppConfig remoteAppConfig;

tRemoteAppState ble_remote_application_state = {0, };
tRemoteAppState  *bleRemoteAppState  = &ble_remote_application_state;

uint16_t  characteristic_client_configuration[MAX_NUM_CLIENT_CONFIG_NOTIF] = {0,};
uint8_t bleremote_key_std_rpt[KEYRPT_LEN] = {0, };       //map to (&(bleRemoteAppState->stdRpt.modifierKeys))[bleRemoteAppState->stdRptSize]
uint8_t bleremote_bitmap_rpt[KEYRPT_NUM_BYTES_IN_BIT_MAPPED_REPORT] = {0, };  //map to bleRemoteAppState->bitMappedReport.bitMappedKeys[]
uint8_t bleremote_output_rpt = 0;
uint8_t bleremote_connection_ctrl_rpt = 0;
uint8_t battery_level = 100;
uint8_t firstTransportStateChangeNotification = 1;
uint8_t bleremote_user_defined_0_rpt[8] = {0, };
#ifdef ATT_MTU_SIZE_180
uint8_t bleremote_voice_rpt[WICED_HIDD_MIC_AUDIO_BUFFER_SIZE*2+1] = {0,};
#else
uint8_t bleremote_voice_rpt[20] = {0,};
#endif
uint8_t bleremote_voice_ctrl_input_rpt[sizeof(wiced_hidd_voice_control_report_t)-1] = {0,};
uint8_t bleremote_voice_ctrl_feature_rpt[sizeof(wiced_hidd_voice_control_report_t)-1] = {0,};

wiced_timer_t allow_sleep_timer;
wiced_timer_t mic_stop_command_pending_timer;
wiced_timer_t bleremote_conn_param_update_timer;

#ifdef OTA_FIRMWARE_UPGRADE
void          bleremoteapp_ota_fw_upgrade_status(uint8_t status);
#endif

extern uint8_t pmu_attemptSleepState;

extern void sfi_enter_deep_power_down(void);
extern void sfi_exit_deep_power_down(BOOL8 forceExitDeepPowerDown);

/////////////////////////////////////////////////////////////////////////////////////////////
/// set up LE Advertising data
/////////////////////////////////////////////////////////////////////////////////////////////
void bleremote_setUpAdvData(void)
{
    wiced_bt_ble_advert_elem_t bleremote_adv_elem[4];
    uint8_t bleremote_adv_flag = BTM_BLE_LIMITED_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint16_t bleremote_adv_appearance = APPEARANCE_GENERIC_HID_DEVICE;
    uint16_t bleremote_adv_service = UUID_SERVCLASS_LE_HID;

    // flag
    bleremote_adv_elem[0].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    bleremote_adv_elem[0].len          = sizeof(uint8_t);
    bleremote_adv_elem[0].p_data       = &bleremote_adv_flag;

    // Appearance
    bleremote_adv_elem[1].advert_type  = BTM_BLE_ADVERT_TYPE_APPEARANCE;
    bleremote_adv_elem[1].len          = sizeof(uint16_t);
    bleremote_adv_elem[1].p_data       = (uint8_t *)&bleremote_adv_appearance;

    //16 bits Service: UUID_SERVCLASS_LE_HID
    bleremote_adv_elem[2].advert_type  = BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE;
    bleremote_adv_elem[2].len          = sizeof(uint16_t);
    bleremote_adv_elem[2].p_data       = (uint8_t *)&bleremote_adv_service;

    //dev name
    bleremote_adv_elem[3].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    bleremote_adv_elem[3].len          = strlen(dev_local_name);
    bleremote_adv_elem[3].p_data       = (uint8_t *)dev_local_name;

    wiced_bt_ble_set_raw_advertisement_data(4, bleremote_adv_elem);
}

#ifdef SUPPORT_IR
////////////////////////////////////////////////////////////////////////////////
/// This function is the timeout handler for allow_irtx_timer
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_allowIrTx_timeout( uint32_t arg)
{
    if (pending_IR)
    {
        pending_IR = FALSE;
        //send IR
        ir->SendIR(pending_IR_code, pending_IR_repeat, pending_IR_repeat_count);
    }
}
#endif

////////////////////////////////////////////////////////////////////////////////
/// This function is the timeout handler for allow_sleep_timer
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_allowsleep_timeout( uint32_t arg )
{
    WICED_BT_TRACE("allow SDS\n");

    bleRemoteAppState->allowSDS = 1;
}

////////////////////////////////////////////////////////////////////////////////
/// This function is the timeout handler for conn_param_update_timer
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_connparamupdate_timeout( uint32_t arg )
{
    //request connection param update if it not requested before
    if ( !blehidlink_connection_param_updated
#ifdef OTA_FIRMWARE_UPGRADE
        // if we are not in the middle of OTAFWU
        && !wiced_ota_fw_upgrade_is_active()
#endif
        )
    {
        wiced_ble_hidd_link_conn_param_update();
    }
}

#ifdef SUPPORT_AUDIO

////////////////////////////////////////////////////////////////////////////////
/// This function is the timeout handler for mic_stop_command_pending_timer
////////////////////////////////////////////////////////////////////////////////
void mic_stop_command_pending_timeout( uint32_t arg )
{
    WICED_BT_TRACE("MIC Command Pending Timeout\n");
    stopMicCommandPending = FALSE;
#ifdef OTA_FIRMWARE_UPGRADE
    if (!wiced_ota_fw_upgrade_is_active())
#endif
    {
        wiced_blehidd_allow_slave_latency(TRUE);
    }
}
#endif

#if defined(SUPPORT_MOTION)
////////////////////////////////////////////////////////////////////////////////
/// Interrupt handler from motion sensor
/////////////////////////////////////////////////////////////////////////////////
static void bleremote_motionsensorActivityDetected(void* remApp, uint8_t unsued)
{
    // when not connected, use motion to trigger connect
    // when connected we rely on the regular poll to get data
    if (!wiced_ble_hidd_link_is_connected())
    {
        bleremoteapp_pollReportUserActivity();
    }
#ifdef POLL_MOTION_WHILE_CONNECTED_AND_ACTIVE
    else
    {
        // we are connected, use regular poll to retrieve data untill motion
        // is idle, then we reenable interrupt
        if (motionsensor)
        {
            motionsensor->disableInterrupt();
        }
    }
#endif
}
#endif

void bleremoteapp_pre_init(void)
{
    uint8_t index;

    WICED_BT_TRACE("bleremoteapp_pre_init\n");

#ifdef SUPPORT_IR
    //IR Tx feature,P38
    ir = (tIrTxDriverVtbl*)appIRtx_appIRtx(2, 6);
#endif
#ifdef SUPPORT_TOUCHPAD
    touchpad = TouchPad(gpioActivityDetected, NULL);

    //touchpad event
    bleRemoteAppState->touchpadEvent.eventInfo.eventType = HID_EVENT_TP_DATA;
#else
    // hold touchpad in reset state so touchpad won't be active
    wiced_hal_gpio_configure_pin(GPIO_RSTN_TP, GPIO_OUTPUT_ENABLE, GPIO_TOUCHPAD_OFF);
#endif
    // reenforce GPIO configuration so we don't lose it after entering uBCS
    wiced_hal_gpio_slimboot_reenforce_cfg(GPIO_RSTN_TP, GPIO_OUTPUT_ENABLE);

#ifdef SUPPORT_MOTION
    // We put auto detect here in case if the firmware is running on Development board without Motion hardware.
    // In that case, we don't want to enable MOTION.

    //Check if there is pull-up on I2C SDA.
    //Put a low on the GPIO
    wiced_hal_gpio_configure_pin(GPIO_SDA, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);

    //Re-configure the pin as an input and check if SDA will be pulled high
    wiced_hal_gpio_configure_pin(GPIO_SDA, GPIO_INPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);

    if(wiced_hal_gpio_get_pin_input_status(GPIO_SDA) == 1)
    {
        motionsensor = CMotionSensorICM_CMotionSensorICM(
            bleremote_motionsensorActivityDetected,
            NULL,
            GPIO_MOTION_INT,
            INTR_LVL_HIGH,   // interrupt active high
            GPIO_EN_INT_RISING_EDGE);
    }
    else
    {
        //Motion hardware not detected.
        // Set P5 back to input disabled to avoid current drain issues.
        wiced_hal_gpio_configure_pin(GPIO_SDA, GPIO_INPUT_DISABLE, GPIO_PIN_OUTPUT_LOW);
    }

#endif

#ifdef SUPPORT_AUDIO
    audioConfig.mic_codec    = NULL;
    audioConfig.audio_fifo   = audioData;
    audioConfig.data_count   = dataCount;
    audioConfig.fifo_count   = AUDIO_FIFO_CNT;
    audioConfig.enable       = TRUE;
    audioConfig.audio_gain   = remoteAppConfig.audio_gain;
    audioConfig.audio_boost  = remoteAppConfig.audio_boost;
    audioConfig.audio_delay  = remoteAppConfig.audio_delay;
    audioConfig.codec_sampling_freq =  WICED_HIDD_CODEC_SAMP_FREQ_16K;

    wiced_hidd_mic_audio_config(&audioConfig);
    wiced_hidd_mic_audio_config_enhanced((uint8_t *)&blehid_audiocfg);

    //voice event
    bleRemoteAppState->voiceEvent.eventInfo.eventType = HID_EVENT_VOICE_DATA_AVAILABLE;

    wiced_init_timer( &mic_stop_command_pending_timer, mic_stop_command_pending_timeout, 0, WICED_MILLI_SECONDS_TIMER );
#endif

    //null event
    bleRemoteAppState->eventNULL.eventInfo.eventType = HID_EVENT_ANY;


    for (index = 0; index < remoteAppConfig.numOfRemoteRpt; ++index)
    {
        bleRemoteAppState->remoteRptSize[index] = remoteAppConfig.maxBytesInRemoteRpt;
    }

#ifdef ATT_MTU_SIZE_180
    wiced_ble_hidd_link_set_preferred_conn_params(wiced_bt_hid_cfg_settings.ble_scan_cfg.conn_min_interval,        // 8*1.25=10ms
                                        wiced_bt_hid_cfg_settings.ble_scan_cfg.conn_max_interval,        // 8*1.25=10ms
                                        wiced_bt_hid_cfg_settings.ble_scan_cfg.conn_latency,             //  99. i.e. 1 second slave latency
                                        wiced_bt_hid_cfg_settings.ble_scan_cfg.conn_supervision_timeout);//500 * 10=500ms=5 seconds
#else
#ifdef SUPPORT_AUDIO
    if (audioConfig.codec_sampling_freq == WICED_HIDD_CODEC_SAMP_FREQ_8K)
    {
        wiced_ble_hidd_link_set_preferred_conn_params(12,     // 12*1.25=15ms
                                            12,     // 12*1.25=15ms
                                            66,     // ~1 second slave latency
                                            500);   //500 * 10=500ms=5 seconds
    }
    else
    {
        wiced_ble_hidd_link_set_preferred_conn_params(6,      // 6*1.25=7.5ms
                                            6,      // 6*1.25=7.5ms
                                            132,     // ~1 second slave latency
                                            500);   //500 * 10=500ms=5 seconds
    }
#else  // no audio
    wiced_ble_hidd_link_set_preferred_conn_params(wiced_bt_hid_cfg_settings.ble_scan_cfg.conn_min_interval,        // 8*1.25=10ms
                                        wiced_bt_hid_cfg_settings.ble_scan_cfg.conn_max_interval,        // 8*1.25=10ms
                                        wiced_bt_hid_cfg_settings.ble_scan_cfg.conn_latency,             //  99. i.e. 1 second slave latency
                                        wiced_bt_hid_cfg_settings.ble_scan_cfg.conn_supervision_timeout);//500 * 10=500ms=5 seconds
#endif
#endif
    bleremote_setUpAdvData();

    //timer to allow Shut Down Sleep (SDS)
    wiced_init_timer( &allow_sleep_timer, bleremoteapp_allowsleep_timeout, 0, WICED_MILLI_SECONDS_TIMER );

    //timer to request connection param update
    wiced_init_timer( &bleremote_conn_param_update_timer, bleremoteapp_connparamupdate_timeout, 0, WICED_MILLI_SECONDS_TIMER );

#ifdef SUPPORT_IR
    if (ir)
    {
        wiced_init_timer( &allow_irtx_timer, bleremoteapp_allowIrTx_timeout, 0, WICED_MILLI_SECONDS_TIMER );
        if (!wiced_hal_mia_is_reset_reason_por())
        {
            wiced_start_timer(&allow_irtx_timer,50); // 50ms. timeout in ms
        }
    }
#endif

}

/////////////////////////////////////////////////////////////////////////////////////////////
/// This function will be called from blehid_app_init() during start up.
/////////////////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_create(void)
{
    WICED_BT_TRACE("bleremoteapp_create\n");
    //battery monitoring configuraion
    wiced_hal_batmon_config(ADC_INPUT_VDDIO,      // ADC input pin
                            3000,               // Period in millisecs between battery measurements
                            8,                  // Number of measurements averaged for a report, max 16
                            3200,               // The full battery voltage in mili-volts
                            1800,               // The voltage at which the batteries are considered drained (in milli-volts)
                            1700,               // System should shutdown if it detects battery voltage at or below this value (in milli-volts)
                            100,                // battery report max level
                            BATTERY_REPORT_ID,  // battery report ID
                            1,                  // battery report length
                            1);                 // Flag indicating that a battery report should be sent when a connection is established

    //keyscan initialize
    wiced_hal_keyscan_configure(NUM_KEYSCAN_ROWS, NUM_KEYSCAN_COLS);
    wiced_hal_keyscan_init();

    wiced_hidd_event_queue_init(&bleRemoteAppState->appEventQueue, (uint8_t *)wiced_memory_permanent_allocate(kbAppConfig.maxEventNum * kbAppConfig.maxEventSize),
                    kbAppConfig.maxEventSize, kbAppConfig.maxEventNum);

    bleremoteapp_pre_init();

    bleremoteapp_init();

#ifdef SUPPORTING_FINDME
    bleremoteapp_findme_init();
#endif

    WICED_BT_TRACE("Free RAM bytes=%d bytes\n", wiced_memory_get_free_bytes());

}


////////////////////////////////////////////////////////////////////////////////
/// The remote application activation method.
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_init(void)
{
    WICED_BT_TRACE("bleremoteapp_init\n");

    // Determine the size of the standard report. Report ID will not be sent.
    bleRemoteAppState->stdRptSize = kbAppConfig.maxKeysInStdRpt +
        (sizeof(KeyboardStandardReport) - sizeof(bleRemoteAppState->stdRpt.keyCodes)) - 1;

    // Determine the size of the Battery. Report ID will not be sent.
    bleRemoteAppState->batRpt.reportID = BATTERY_REPORT_ID;
    bleRemoteAppState->batRptSize = sizeof(bleRemoteAppState->batRpt.level);

    // Determine the size of the bit mapped report.Report ID will not be sent.
    // and round up to the next largest integer
    bleRemoteAppState->bitReportSize = (kbAppConfig.numBitMappedKeys + 7)/8;

    // We are not in recovery
    bleRemoteAppState->recoveryInProgress = 0;

    // Initialize temporaries used for events
    bleRemoteAppState->kbKeyEvent.eventInfo.eventType = HID_EVENT_KEY_STATE_CHANGE;

    wiced_hal_keyscan_register_for_event_notification(bleremoteapp_userKeyPressDetected, NULL);

    bleremoteapp_stdRptRolloverInit();
    bleremoteapp_ledRptInit();
    bleremoteapp_clearAllReports();

#ifdef SUPPORT_MOTION
    if (motionsensor)
    {
        if (wiced_hal_mia_is_reset_reason_por())
        {
            motionsensor->initialize();
        }
        else if (motion_mode == SENSOR_ENABLED)
        {
            motion_mode = SENSOR_DISABLED;
            motionsensor->enable(TRUE);
        }
    }
#endif

#ifdef SUPPORT_AUDIO
    wiced_hidd_mic_audio_init(bleremoteapp_appActivityDetected, NULL);
#endif


#ifdef SUPPORT_TOUCHPAD
    if (touchpad)
    {
        if (wiced_hal_mia_is_reset_reason_por())
        {
            touchpad->reInitialize();
        }
        else
        {
            touchpad->setEnable(TRUE);
        }
    }
#endif

    wiced_hal_batmon_add_battery_observer(bleremoteapp_batLevelChangeNotification);

     //register App low battery shut down handler
    wiced_hal_batmon_register_low_battery_shutdown_cb(bleremoteapp_shutdown);

    wiced_ble_hidd_link_add_state_observer(bleremoteapp_transportStateChangeNotification);

    wiced_ble_hidd_link_register_poll_callback(bleremoteapp_pollReportUserActivity);

    wiced_ble_hidd_link_register_sleep_permit_handler(bleremoteapp_sleep_handler);

    wiced_blehidd_register_report_table(bleRemoteReportModeGattMap, sizeof(bleRemoteReportModeGattMap)/sizeof(bleRemoteReportModeGattMap[0]));

    wiced_ble_hidd_link_init();

    wiced_hal_mia_enable_mia_interrupt(TRUE);
    wiced_hal_mia_enable_lhl_interrupt(TRUE);//GPIO interrupt

}

////////////////////////////////////////////////////////////////////////////////
/// This function is called when battery voltage drops below the configured threshold.
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_shutdown(void)
{
    WICED_BT_TRACE("bleremoteapp_shutdown\n");

    bleremoteapp_flushUserInput();

    // Disable key detection
    wiced_hal_keyscan_turnOff();

#ifdef SUPPORT_AUDIO
    //stop audio
    wiced_hidd_mic_audio_stop();
#endif
#ifdef SUPPORT_MOTION
    //stop motion sensor
    if (motionsensor)
    {
        motionsensor->shutdown();
    }
#endif

#ifdef SUPPORT_TOUCHPAD
    if (touchpad)
    {
        touchpad->shutdown();
    }
#endif

    if(wiced_ble_hidd_link_is_connected())
    {
        wiced_ble_hidd_link_disconnect();
    }
    // Disable Interrupts
    wiced_hal_mia_enable_mia_interrupt(FALSE);
    wiced_hal_mia_enable_lhl_interrupt(FALSE);

}

////////////////////////////////////////////////////////////////////////////////
/// This function flushes all queued events and clears all reports.
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_flushUserInput(void)
{
    // Flag that recovery is no longer in progress
    bleRemoteAppState->recoveryInProgress = 0;

    // Clear all dynamic reports
    bleremoteapp_clearAllReports();

#ifdef SUPPORT_AUDIO
    //reset the counter that keeps track of voice events # in the event queue
    bleRemoteAppState->audioPacketInQueue = 0;
#endif

    // Flush the event queue
    //wiced_hidd_event_queue_flush(&bleRemoteAppState->appEventQueue);
}

/////////////////////////////////////////////////////////////////////////////////
/// This method handles connect button pressed events. It performs the following
/// actions in order:
///  - If we are configured to generate a VC unplug on connect button press
///    it generates a VC unplug to the BT transport
///  - If we are configured to become discoverable on connect button press
///    it tells the BT transport to become discoverable
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_connectButtonPressed(void)
{
#ifdef CONNECTED_ADVERTISING_SUPPORTED
    blehidlink_allowDiscoverable();
#else
    wiced_ble_hidd_link_virtual_cable_unplug();
#endif
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
void bleremoteapp_connectButtonHandler(ConnectButtonPosition connectButtonPosition)
{
    // The connect button was not pressed. Check if it is now pressed
    if (connectButtonPosition == CONNECT_BUTTON_DOWN)
    {
        WICED_BT_TRACE("Connect Btn Pressed\n");
        bleremoteapp_connectButtonPressed();
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
///    by calling the event processing functions, e.g. procEvtKey() etc
///  - This function also tracks the recovery period after an error. If
///    the recovery count is non-zero, it is decremented as long as there is room
///    for one report in the transport
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_generateAndTxReports(void)
{
    HidEvent *curEvent;

    {
        // Normal state

        // If we are recovering from an error, decrement the recovery count as long as the transport
        // has room. Avoid the case where no event processing is done during recovery because
        // transport is full, as the failure might be a non-responding transport.
        if (bleRemoteAppState->recoveryInProgress)
        {
            // If recovery is complete, transmit any modified reports that we have been hoarding
            if (!--bleRemoteAppState->recoveryInProgress)
            {
                bleremoteapp_txModifiedKeyReports();
            }
        }
        // Continue report generation as long as the transport has room and we have events to process
        while ((wiced_bt_buffer_poolutilization (HCI_ACL_POOL_ID) < 80) &&
               ((curEvent = (HidEvent *)wiced_hidd_event_queue_get_current_element(&bleRemoteAppState->appEventQueue)) != NULL))
        {
            // Further processing depends on the event type
            switch (curEvent->eventType)
            {
                case HID_EVENT_KEY_STATE_CHANGE:
                    bleremoteapp_procEvtKey();
                    break;
                case HID_EVENT_EVENT_FIFO_OVERFLOW:
                    WICED_BT_TRACE("HID_EVENT_EVENT_FIFO_OVERFLOW\n");
                    // Call event queue error handler
                    bleremoteapp_procErrEvtQueue();
                    break;
                default:
                    bleremoteapp_procEvtUserDefined();
                    break;
            }

            // The current event should be deleted by the event processing function.
            // Additional events may also be consumed but we don't care about that
        }

    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function processes keys from the event queue until the end of scan cycle event is seen.
/// During this process it accumulates changes to key reports. Once the end-of-scan-cycle
/// event is seen, it generates any modified reports. If any errors are detected
/// during the processing of events it calls one of the error handlers. Note that
/// if this function is called, there should be at least one event in the event queue
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_procEvtKey(void)
{
    uint8_t keyCode=KB_MAX_KEYS;
    uint8_t upDownFlag;
    uint8_t keyType, keyTranslationCode;
    HidEventKey *keyEvent;

    // Process events until we get an end-of cycle event
    // or an error (which doubles as an end-of-scan cycle event)
    // or we run out of events
    do
    {
        // Grab the next event. The first time we enter this loop we will have at least one
        // event. Subsequent iterations may run out of events
        if ((keyEvent = (HidEventKey *)wiced_hidd_event_queue_get_current_element(&bleRemoteAppState->appEventQueue)) != NULL)
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
                if (keyCode < kbKeyConfig_size) //(keyCode < KB_MAX_KEYS)
                {
                    // This is a normal key event. Translate it to event type and tranlation code
                    keyType = kbKeyConfig[keyCode].type;
                    keyTranslationCode = kbKeyConfig[keyCode].translationValue;

                    // Depending on the key type, call the appropriate function for handling
                    // Pass unknown key types to user function
                    switch(keyType)
                    {
                        case KEY_TYPE_STD:
                            bleremoteapp_stdRptProcEvtKey(upDownFlag, keyCode, keyTranslationCode);
                            break;
                        case KEY_TYPE_MODIFIER:
                            bleremoteapp_stdRptProcEvtModKey(upDownFlag, keyCode, keyTranslationCode);
                            break;
                        case KEY_TYPE_BIT_MAPPED:
                            bleremoteapp_bitRptProcEvtKey(upDownFlag, keyCode, keyTranslationCode);
                            break;
                        default:
                            bleremoteapp_procEvtUserDefinedKey(upDownFlag, keyCode, keyTranslationCode);
                            break;
                    }
                }
                // Check if we have an end of scan cycle event
                else if (keyCode == END_OF_SCAN_CYCLE)
                {
                    bleremoteapp_txModifiedKeyReports();
                }
                else
                {
                    // Call error handler for all other events
                    bleremoteapp_procErrKeyscan();
                    break;
                }
            }
            else
            {
                // We probably have event queue overflow. Call the event queue error handler
                bleremoteapp_procErrEvtQueue();
                break;
            }

            // Delete the current event since we have consumed it
            wiced_hidd_event_queue_remove_current_element(&bleRemoteAppState->appEventQueue);
        }
        else
        {
            // We ran out of events before we saw an end-of-scan-cycle event
            // Call error handler and manually exit the loop
            bleremoteapp_procErrEvtQueue();
            break;
        }
    } while (keyCode < KB_MAX_KEYS);
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
void bleremoteapp_bitRptProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t rowCol)
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
            if (!(bleRemoteAppState->bitMappedReport.bitMappedKeys[row] & keyMask))
            {
                // The following code is funky because compiler will not allow a simple expression
                bleRemoteAppState->bitMappedReport.bitMappedKeys[row] |= keyMask;

                // Increment the number of keys in the bit report
                bleRemoteAppState->keysInBitRpt++;

                // Flag that the bit report has changed
                bleRemoteAppState->bitRptChanged = TRUE;
            }
        }
        else
        {
            // Key up.  Update the report only if the state of the key changed
            if (bleRemoteAppState->bitMappedReport.bitMappedKeys[row] & keyMask)
            {
                // The following code is funky because compiler will not allow a simple expression

                bleRemoteAppState->bitMappedReport.bitMappedKeys[row] &= ~keyMask;

                // Decrement the number of keys in the bit report
                bleRemoteAppState->keysInBitRpt--;

                // Flag that the bit report has changed
                bleRemoteAppState->bitRptChanged = TRUE;
            }
        }
    }
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
void bleremoteapp_stdErrResp(void)
{
    // Clear all reports unconditionally
    bleremoteapp_clearAllReports();

    // Send a rollover report if
    //   - we are not already in the middle of a recovery OR
    //   - the rollover report generation period is non-zero and we have exceeded that period
    if (!bleRemoteAppState->recoveryInProgress)
    {
        // Send rollover report
        bleremoteapp_stdRptRolloverSend();
    }

    // Reset recovery timeout
    bleRemoteAppState->recoveryInProgress = kbAppConfig.recoveryPollCount;

    // Mark all reports as not sent. This ensures that all reports will be sent once
    // the recovery is complete
    bleRemoteAppState->bitRptChanged = bleRemoteAppState->stdRptChanged = TRUE;

    // Assume connect button is now up
//    bleremoteapp_connectButtonHandler(CONNECT_BUTTON_UP);
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
void bleremoteapp_stdErrRespWithFwHwReset(void)
{
    // Provide standard error response
    bleremoteapp_stdErrResp();

    // Flush the event fifo
    wiced_hidd_event_queue_flush(&bleRemoteAppState->appEventQueue);

    // Reset the keyscan HW
    wiced_hal_keyscan_reset();

    // Configure GPIOs for keyscan operation
    wiced_hal_keyscan_config_gpios();
}

////////////////////////////////////////////////////////////////////////////////
/// This function handles event queue errors. This includes event queue overflow
/// unexpected events, missing expected events, and events in unexpected order.
/// This function does a FW/HW reset via stdErrRespWithFwHwReset in an attempt
/// to address the problem. A user defined implementation should at least
/// remove the first element in the queue if this event is an overflow event
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_procErrEvtQueue(void)
{
    WICED_BT_TRACE("KSQerr\n");
    bleremoteapp_stdErrRespWithFwHwReset();
}


/////////////////////////////////////////////////////////////////////////////////
/// This function handles key events targetted for the standard key report.
/// It updates the standard report with the given event.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param translationCode information on how the scan code is translated to a reported value
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_stdRptProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode)
{
    // Processing depends on whether the event is an up or down event
    if (upDownFlag == KEY_DOWN)
    {
        bleremoteapp_stdRptProcEvtKeyDown(upDownFlag, keyCode, translationCode);
    }
    else
    {
        bleremoteapp_stdRptProcEvtKeyUp(upDownFlag, keyCode, translationCode);
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
void bleremoteapp_stdRptProcEvtKeyDown(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode)
{
    uint8_t i;

    // Check if the key is already in the report
    for (i=0; i < bleRemoteAppState->keysInStdRpt; i++)
    {
        if (bleRemoteAppState->stdRpt.keyCodes[i] == translationCode)
        {
            // It is. Ignore the event
            return;
        }
    }

    // Check if the std report has room
    if (i < kbAppConfig.maxKeysInStdRpt)
    {
        // Add the new key to the report
        bleRemoteAppState->stdRpt.keyCodes[i] = translationCode;

        // Update the number of keys in the report
        bleRemoteAppState->keysInStdRpt++;

        // Flag that the standard key report has changed
        bleRemoteAppState->stdRptChanged = TRUE;
    }
    else
    {
        // No room in report. Call error handler
        bleremoteapp_stdRptProcOverflow();
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
void bleremoteapp_stdRptProcEvtKeyUp(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode)
{
    uint8_t i;

    // Find the key in the current standard report
    for (i=0; i < bleRemoteAppState->keysInStdRpt; i++)
    {
        if (bleRemoteAppState->stdRpt.keyCodes[i] == translationCode)
        {
            // Found it. Remvove it by replacing it with the last key and
            // reducing the key count by one. We can do this because the
            // order of keys in the report is not important.
            bleRemoteAppState->keysInStdRpt--;
            // The following code is funky because compiler will not allow a simple expression
            bleRemoteAppState->stdRpt.keyCodes[i] = bleRemoteAppState->stdRpt.keyCodes[bleRemoteAppState->keysInStdRpt];
            // Clear the last key
            bleRemoteAppState->stdRpt.keyCodes[bleRemoteAppState->keysInStdRpt] = 0;

            // Flag that the standard key report has changed
            bleRemoteAppState->stdRptChanged = TRUE;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles overflow of the standard key report. This happens when
/// more than 6 (or the configured number of) standard keys are pressed at a time.
/// This function does a FW/HW reset in response.
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_stdRptProcOverflow(void)
{
    WICED_BT_TRACE("OverFlow\n");
    bleremoteapp_stdErrRespWithFwHwReset();
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles modifier key events. It updates the modifier key bits
/// in the standard report structure.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param translationCode bitmap of the modifier key used for report generation
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_stdRptProcEvtModKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode)
{
    // Process the key event and update the modifier key bits in the standard report

    // Check if this is a down key or up key
    if (upDownFlag == KEY_DOWN)
    {
        // Key down. Update report only if the key state has changed
        if (!(bleRemoteAppState->stdRpt.modifierKeys & translationCode))
        {
            // Mark the appropriate modifier key as down
            bleRemoteAppState->stdRpt.modifierKeys |= translationCode;

            // Flag that the standard key report has changed
            bleRemoteAppState->stdRptChanged = TRUE;

            // Increment the number of mod keys that are down
            bleRemoteAppState->modKeysInStdRpt++;
        }
    }
    else
    {
        // Key up. Update report only if the key state has changed
        if (bleRemoteAppState->stdRpt.modifierKeys & translationCode)
        {
            // Mark the appropriate modifier key as down
            bleRemoteAppState->stdRpt.modifierKeys &= ~translationCode;

            // Flag that the standard key report has changed
            bleRemoteAppState->stdRptChanged = TRUE;

            // Decrement the number of mod keys that are down
            bleRemoteAppState->modKeysInStdRpt--;
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the bit mapped report over the interrupt channel
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_bitRptSend(void)
{
    // Flag that the bit mapped key report has not changed since it was sent the last time
    bleRemoteAppState->bitRptChanged = FALSE;
    WICED_BT_TRACE("BitRpt\n");

    //set gatt attribute value here before sending the report
    memcpy(bleremote_bitmap_rpt, bleRemoteAppState->bitMappedReport.bitMappedKeys, bleRemoteAppState->bitReportSize);

    // Send the rpt.
    wiced_ble_hidd_link_send_report(bleRemoteAppState->bitMappedReport.reportID,WICED_HID_REPORT_TYPE_INPUT,
         bleRemoteAppState->bitMappedReport.bitMappedKeys,bleRemoteAppState->bitReportSize);
}


/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the battery report over the interrupt channel
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_batRptSend(void)
{
    //WICED_BT_TRACE("\nBASRpt");

    //set gatt attribute value here before sending the report
    battery_level = bleRemoteAppState->batRpt.level[0];

    if ( WICED_SUCCESS == wiced_ble_hidd_link_send_report(bleRemoteAppState->batRpt.reportID,WICED_HID_REPORT_TYPE_INPUT,
                                                          bleRemoteAppState->batRpt.level,bleRemoteAppState->batRptSize))
    {
        wiced_hal_batmon_set_battery_report_sent_flag(WICED_TRUE);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the standard report over the interrupt channel and
/// marks internally that the report has been sent
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_stdRptSend(void)
{
    // Flag that the standard key report ha not changed since it was sent the last time
    bleRemoteAppState->stdRptChanged = FALSE;
    //WICED_BT_TRACE("\nStdRpt");

    //set gatt attribute value here before sending the report
    memcpy(bleremote_key_std_rpt, &(bleRemoteAppState->stdRpt.modifierKeys), bleRemoteAppState->stdRptSize);

    // Send the report
    wiced_ble_hidd_link_send_report(bleRemoteAppState->stdRpt.reportID,WICED_HID_REPORT_TYPE_INPUT,
        &(bleRemoteAppState->stdRpt.modifierKeys),bleRemoteAppState->stdRptSize);

}

/////////////////////////////////////////////////////////////////////////////////
/// This function is the application handler for keyscan interrupt
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_userKeyPressDetected(void* unused)
{
    bleRemoteAppState->keyInterrupt_On = wiced_hal_keyscan_is_any_key_pressed();

    // Poll the app.
    bleremoteapp_pollReportUserActivity();
}


/////////////////////////////////////////////////////////////////////////////////
/// This function will send out battery report when battery level changed
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_batLevelChangeNotification(uint32_t newLevel)
{
    WICED_BT_TRACE("bat level changed to %d\n", newLevel);

    {
        bleRemoteAppState->batRpt.level[0] = newLevel;
        bleremoteapp_batRptSend();

        //we do not want to save battery level value to NVRAM (i.e. SFLASH). too many writes, damage SFLASH lifetime.
        //blehostlist_SetClientBatLevelAtTop(newLevel);
    }
}



/////////////////////////////////////////////////////////////////////////////////
/// This function sends a rollover report. And if audio is active, it will stop audio ADC and inform
/// the peer by sending BRCM_RC_MIC_STOP_REQ msg
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_stdRptRolloverSend(void)
{
    // Tx rollover report
    WICED_BT_TRACE("RollOverRpt\n");

    //set gatt attribute value here before sending the report
    memcpy(bleremote_key_std_rpt, &(bleRemoteAppState->rolloverRpt.modifierKeys), bleRemoteAppState->stdRptSize);

    wiced_ble_hidd_link_send_report(bleRemoteAppState->rolloverRpt.reportID,WICED_HID_REPORT_TYPE_INPUT,
            &(bleRemoteAppState->rolloverRpt.modifierKeys),bleRemoteAppState->stdRptSize);

#ifdef SUPPORT_AUDIO
    if(audioIsActive())
    {
        wiced_hidd_voice_control_report_t audioMsgReq;
        wiced_hidd_mic_audio_stop();
        //re-enable app polling
        wiced_ble_hidd_link_enable_poll_callback(WICED_TRUE);

        memset(&audioMsgReq, 0, sizeof(wiced_hidd_voice_control_report_t));
        audioMsgReq.reportId = WICED_HIDD_VOICE_CTL_REPORT_ID;
        audioMsgReq.format = WICED_HIDD_RC_MIC_STOP_REQ;
        audioMsgReq.rsvd = remoteAppConfig.audio_mode; //put "audio mode" info in the reserved byte

        //set gatt attribute value here before sending the report
        memcpy(bleremote_voice_ctrl_input_rpt, &audioMsgReq.format, sizeof(wiced_hidd_voice_control_report_t) - sizeof(audioMsgReq.reportId));
        wiced_ble_hidd_link_send_report(WICED_HIDD_VOICE_CTL_REPORT_ID,WICED_HID_REPORT_TYPE_INPUT,&audioMsgReq.format,
                    sizeof(wiced_hidd_voice_control_report_t) - sizeof(audioMsgReq.reportId));
    }
#endif
}
////////////////////////////////////////////////////////////////////////////////
/// This function handles error events reported by the keyscan HW. Typically
/// these would be ghost events.
/// This function does a FW/HW reset via stdErrRespWithFwHwReset in an attempt
/// to address the problem.
/// NOTE: if audio stop event was flushed out, put it back into event queue too.
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_procErrKeyscan(void)
{
#ifdef SUPPORT_TOUCHPAD
    AbsXYRptPtr tp;
#endif
    WICED_BT_TRACE("bleremoteapp_procErrKeyscan\n");

    //call base class handling
    bleremoteapp_stdErrRespWithFwHwReset();

    //NOTE: Undo the change done in base class handling.
    bleRemoteAppState->bitRptChanged = bleRemoteAppState->stdRptChanged = FALSE;

#ifdef SUPPORT_AUDIO
    //reset the counter that keeps track of voice events # in the event queue
    bleRemoteAppState->audioPacketInQueue = 0;

    //if we have WICED_HIDD_RC_MIC_STOP_REQ in the event queue that was flushed out, put it back into the event queue.
    //Otherwise, we missed the handling for user released the audio button!!!
    if (bleRemoteAppState->audioStopEventInQueue)
    {
        bleRemoteAppState->voiceCtrlEvent.eventInfo.eventType = HID_EVENT_RC_MIC_STOP_REQ;
        wiced_hidd_event_queue_add_event_with_overflow(&bleRemoteAppState->appEventQueue,
                     &bleRemoteAppState->voiceCtrlEvent.eventInfo,
                     sizeof(bleRemoteAppState->voiceCtrlEvent),
                     bleRemoteAppState->pollSeqn);
    }
    //if we have WICED_HIDD_MIC_STOP in the event queue that was flushed out, put it back into the event queue.
    //Otherwise, we missed the handling for host stopping audio!!!
    else if (bleRemoteAppState->micStopEventInQueue)
    {
        bleRemoteAppState->voiceCtrlEvent.eventInfo.eventType = HID_EVENT_MIC_STOP;
        wiced_hidd_event_queue_add_event_with_overflow(&bleRemoteAppState->appEventQueue,
                     &bleRemoteAppState->voiceCtrlEvent.eventInfo,
                     sizeof(bleRemoteAppState->voiceCtrlEvent),
                     bleRemoteAppState->pollSeqn);
    }
#endif

#ifdef SUPPORT_TOUCHPAD
    // clear touchpad data buffer. If the last report was finger down, we report finger up
    if (touchpad)
    {
        tp = touchpad->getAbsFingerUpRpt();
        if (tp)
        {
            //set gatt attribute value here before sending the report
            memset(bleremote_touchpad_rpt, 0, TOUCHPAD_RPT_PAYLOAD_SIZE);
            memcpy(bleremote_touchpad_rpt, TOUCHPAD_REPORT_DATA(tp), TOUCHPAD_REPORT_DATA_SIZE(tp));

            wiced_ble_hidd_link_send_report(tp->reportID, WICED_HID_REPORT_TYPE_INPUT, bleremote_touchpad_rpt, TOUCHPAD_REPORT_DATA_SIZE(tp));
        }
    }
#endif


}

#ifdef SUPPORT_AUDIO
/////////////////////////////////////////////////////////////////////////////////
/// This function handles the voice control message.
/// \param voice control report
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_handleVoiceCtrlMsg(wiced_hidd_voice_control_report_t* voiceCtrlRpt)
{
    switch( voiceCtrlRpt->format )
    {
        case WICED_HIDD_MIC_START:
            if (bleRemoteAppState->audiobutton_pressed)
            {
                if( !audioIsActive() )
                {
#ifdef SUPPORT_MOTION
                    if (motionsensor && motionsensor->isEnabled())
                    {
                        //stop motion sensor
                        motionsensor->enable(FALSE);
                    }
#endif
                    //queue to event queue, so that the Rpt will be sent after HANDSHAKE message
                    bleRemoteAppState->voiceCtrlEvent.eventInfo.eventType = HID_EVENT_MIC_START;
                    wiced_hidd_event_queue_add_event_with_overflow(&bleRemoteAppState->appEventQueue,
                             &bleRemoteAppState->voiceCtrlEvent.eventInfo,
                             sizeof(bleRemoteAppState->voiceCtrlEvent),
                             bleRemoteAppState->pollSeqn);
                }
            }
            break;

        case WICED_HIDD_MIC_STOP:
            bleRemoteAppState->audioStopEventInQueue = FALSE;
             //queue to event queue, so that the Rpt will be sent after HANDSHAKE message
             bleRemoteAppState->micStopEventInQueue = TRUE;
             bleRemoteAppState->voiceCtrlEvent.eventInfo.eventType = HID_EVENT_MIC_STOP;
             wiced_hidd_event_queue_add_event_with_overflow(&bleRemoteAppState->appEventQueue,
                             &bleRemoteAppState->voiceCtrlEvent.eventInfo,
                             sizeof(bleRemoteAppState->voiceCtrlEvent),
                             bleRemoteAppState->pollSeqn);
            break;

        case WICED_HIDD_RC_CODECSETTINGS_RD_REQ:
            {
                bleRemoteAppState->codecSettingMsg_type     = voiceCtrlRpt->rsvd;
                bleRemoteAppState->voiceCtrlEvent.eventInfo.eventType = HID_EVENT_AUDIO_CODEC_RD;
                wiced_hidd_event_queue_add_event_with_overflow(&bleRemoteAppState->appEventQueue,
                             &bleRemoteAppState->voiceCtrlEvent.eventInfo,
                             sizeof(bleRemoteAppState->voiceCtrlEvent),
                             bleRemoteAppState->pollSeqn);
            }
            break;

        case WICED_HIDD_RC_CODECSETTINGS_WT_REQ:
            {
                uint8_t j;
                bleRemoteAppState->codecSettingMsg_type     = voiceCtrlRpt->rsvd;
                bleRemoteAppState->codecSettingMsg_dataCnt  = voiceCtrlRpt->dataCnt;
                for (j=0; j<voiceCtrlRpt->dataCnt; j++)
                {
                    bleRemoteAppState->codecSettingMsg_dataBuffer[j] = voiceCtrlRpt->dataBuffer[j];
                }
                bleRemoteAppState->voiceCtrlEvent.eventInfo.eventType = HID_EVENT_AUDIO_CODEC_WT;
                wiced_hidd_event_queue_add_event_with_overflow(&bleRemoteAppState->appEventQueue,
                             &bleRemoteAppState->voiceCtrlEvent.eventInfo,
                             sizeof(bleRemoteAppState->voiceCtrlEvent),
                             bleRemoteAppState->pollSeqn);
            }
            break;

        case WICED_HIDD_RC_VOICEMODE_RD_REQ:
            //queue to event queue, so that the Rpt will be sent after HANDSHAKE message.
            bleRemoteAppState->voiceCtrlEvent.eventInfo.eventType = HID_EVENT_AUDIO_MODE;
            wiced_hidd_event_queue_add_event_with_overflow(&bleRemoteAppState->appEventQueue,
                             &bleRemoteAppState->voiceCtrlEvent.eventInfo,
                             sizeof(bleRemoteAppState->voiceCtrlEvent),
                             bleRemoteAppState->pollSeqn);
            break;

        case WICED_HIDD_SPK_START:
        case WICED_HIDD_SPK_STOP:
        case WICED_HIDD_PHONECALL_START:
        case WICED_HIDD_PHONECALL_STOP:
        default:
            break;
    }

}
#endif
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
void bleremoteapp_setReport(wiced_hidd_report_type_t reportType,
                     uint8_t reportId,
                     void *payload,
                     uint16_t payloadSize)
{
    WICED_BT_TRACE("bleremoteapp_setReport: %d\n", payloadSize);

#ifdef SUPPORT_AUDIO
    //we only handle FEATURE report type
    if ((reportType == WICED_HID_REPORT_TYPE_FEATURE) && (reportId == WICED_HIDD_VOICE_CTL_REPORT_ID))
    {
        wiced_hidd_voice_control_report_t* voiceCtrlRpt = (wiced_hidd_voice_control_report_t *)((uint8_t *)payload - 1);

        //save gatt attribute value here when receiving the report
        memset(bleremote_voice_ctrl_feature_rpt, 0, sizeof(wiced_hidd_voice_control_report_t)-1);
        memcpy(bleremote_voice_ctrl_feature_rpt, &(voiceCtrlRpt->format), MIN(payloadSize, (sizeof(wiced_hidd_voice_control_report_t)-1)));

        bleremoteapp_handleVoiceCtrlMsg(voiceCtrlRpt);
    }
    else
#endif
    if ((reportType == WICED_HID_REPORT_TYPE_FEATURE) && (reportId == 0xCC))
    {
#ifdef PTS_HIDS_CONFORMANCE_TC_CW_BV_03_C
        bleremote_connection_ctrl_rpt = *((uint8_t*)payload);
        WICED_BT_TRACE("PTS_HIDS_CONFORMANCE_TC_CW_BV_03_C write val: %d \n", bleremote_connection_ctrl_rpt);
#endif
    }
    else if (reportType == WICED_HID_REPORT_TYPE_OUTPUT)
    {
        // Pass to handler based on report ID. Ensure that report ID is in the payload
        if (payloadSize >= 1)
        {
            // Demux on report ID
            if(reportId == kbAppConfig.ledReportID)
            {
                WICED_BT_TRACE("KB LED report\n");
                bleRemoteAppState->ledReport.ledStates = bleremote_output_rpt = *((uint8_t*)payload);
            }
        }
    }

}

/////////////////////////////////////////////////////////////////////////////////
/// This function initializes the led report.
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_ledRptInit(void)
{
    bleRemoteAppState->ledReport.reportID = kbAppConfig.ledReportID;
    bleRemoteAppState->ledReport.ledStates = kbAppConfig.defaultLedState;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function clears the standard key report including internal count of
/// standard and modifier keys
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_stdRptClear(void)
{
    // Indicate that there are no keys in the standard report
    bleRemoteAppState->modKeysInStdRpt = bleRemoteAppState->keysInStdRpt = 0;

    // Initialize the std report completely
    bleRemoteAppState->stdRpt.reportID = kbAppConfig.stdRptID;
    bleRemoteAppState->stdRpt.modifierKeys = 0;
    bleRemoteAppState->stdRpt.reserved = 0;
    memset((void*)&bleRemoteAppState->stdRpt.keyCodes, 0, sizeof(bleRemoteAppState->stdRpt.keyCodes));
}

/////////////////////////////////////////////////////////////////////////////////
/// This function initializes the rollover report
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_stdRptRolloverInit(void)
{
    bleRemoteAppState->rolloverRpt.reportID = kbAppConfig.stdRptID;
    bleRemoteAppState->rolloverRpt.modifierKeys = 0;
    bleRemoteAppState->rolloverRpt.reserved = 0;
    memset((void*)&bleRemoteAppState->rolloverRpt.keyCodes,
           KEYRPT_CODE_ROLLOVER,
           sizeof(bleRemoteAppState->rolloverRpt.keyCodes));
}


/////////////////////////////////////////////////////////////////////////////////
/// This function clears the bitmapped key report
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_bitRptClear(void)
{
    // Indicate that there are no keys in the bit report
    bleRemoteAppState->keysInBitRpt = 0;

    // Initialize the bit mapped report completely
    bleRemoteAppState->bitMappedReport.reportID = kbAppConfig.bitReportID;
    memset((void*)&bleRemoteAppState->bitMappedReport.bitMappedKeys, 0,
           sizeof(bleRemoteAppState->bitMappedReport.bitMappedKeys));
}

/////////////////////////////////////////////////////////////////////////////////
/// This function clears the remote report
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_remoteRptClear(void)
{
    uint8_t rptIndex;

    // Initialize the remote report completely
    memset(bleRemoteAppState->remoteRpt, 0, sizeof(bleRemoteAppState->remoteRpt));

    for (rptIndex = 0; rptIndex < remoteAppConfig.numOfRemoteRpt; ++rptIndex)
    {
        // Indicate that there are no keys in the remote report
        bleRemoteAppState->bytesInRemoteRpt[rptIndex] = 0;
        bleRemoteAppState->remoteRptChanged[rptIndex] = FALSE;
        bleRemoteAppState->remoteRpt[rptIndex].reportID = remoteAppConfig.remoteUserDefinedReportConfig[rptIndex].rptID;
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function clears all remote reports and calls the KB clearAllReports().
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_clearAllReports(void)
{
    bleremoteapp_stdRptClear();
    bleremoteapp_bitRptClear();

    // Flag that the reports have not been sent
    bleRemoteAppState->bitRptChanged = bleRemoteAppState->stdRptChanged = FALSE;

    bleremoteapp_remoteRptClear();
}


/////////////////////////////////////////////////////////////////////////////////
/// This function transmits all modified reports as long as we are not trying to
/// recover from an error.
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_txModifiedKeyReports(void)
{
    uint8_t rptIndex;

    // Only transmit reports if recovery is not in progress
    if (!bleRemoteAppState->recoveryInProgress)
    {
        // Transmit standard report
        if (bleRemoteAppState->stdRptChanged)
        {
            bleremoteapp_stdRptSend();
        }

        // Transmit bit mapped report
        if (bleRemoteAppState->bitRptChanged)
        {
            bleremoteapp_bitRptSend();
        }

        // Transmit remote reports
        for (rptIndex = 0; rptIndex < remoteAppConfig.numOfRemoteRpt; ++rptIndex)
        {
            if (bleRemoteAppState->remoteRptChanged[rptIndex])
            {
                bleremoteapp_remoteRptSend(rptIndex);
            }
        }
    }
}



/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the remote report over the interrupt channel and
/// marks internally that the report has been sent
///
/// \param rptIndex index of the remote report
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_remoteRptSend(uint8_t rptIndex)
{
    // Flag that the remote report has not changed since it was sent the last time
    bleRemoteAppState->remoteRptChanged[rptIndex] = FALSE;

    //set gatt attribute value here before sending the report
    memset(bleremote_user_defined_0_rpt, 0, 8);
    memcpy(bleremote_user_defined_0_rpt, &(bleRemoteAppState->remoteRpt[rptIndex].keyCodes[0]), bleRemoteAppState->remoteRptSize[rptIndex]);

    wiced_ble_hidd_link_send_report(bleRemoteAppState->remoteRpt[rptIndex].reportID,WICED_HID_REPORT_TYPE_INPUT,
            &(bleRemoteAppState->remoteRpt[rptIndex].keyCodes[0]),bleRemoteAppState->remoteRptSize[rptIndex]);

}


/////////////////////////////////////////////////////////////////////////////////
/// This function checks if the key is already in the current report.

/// \param rptIndex index of the current report
/// \param keyCode  scan code of this key
/// \param translationCodeSize size of the translation code
/// \return TRUE if found, otherwise FALSE
/////////////////////////////////////////////////////////////////////////////////
uint8_t bleremoteapp_findKeyInRemoteRpt(uint8_t rptIndex, uint8_t keyCode, uint8_t translationCodeSize)
{
    uint8_t locInRpt;

    for (locInRpt = 0; locInRpt + translationCodeSize <= bleRemoteAppState->bytesInRemoteRpt[rptIndex]; locInRpt += translationCodeSize)
    {
        if (memcmp(&bleRemoteAppState->remoteRpt[rptIndex].keyCodes[locInRpt],
            remoteAppConfig.remoteKeyTranslationCode[keyCode].translationValue,
            translationCodeSize)
            == 0)
        {
            return TRUE;
        }
    }

    return FALSE;
}



/////////////////////////////////////////////////////////////////////////////////
/// This function adds the key to the current report if space is enough.

/// \param rptIndex index of the current report
/// \param keyCode  scan code of this key
/// \param translationCodeSize size of the translation code
/// \return TRUE if added, otherwise FALSE
/////////////////////////////////////////////////////////////////////////////////
uint8_t bleremoteapp_addKeytoRemoteRpt(uint8_t rptIndex, uint8_t keyCode, uint8_t translationCodeSize)
{
    if (bleRemoteAppState->bytesInRemoteRpt[rptIndex] + translationCodeSize <= remoteAppConfig.maxBytesInRemoteRpt)
    {
        // Add the new key to the report
        memcpy(&bleRemoteAppState->remoteRpt[rptIndex].keyCodes[bleRemoteAppState->bytesInRemoteRpt[rptIndex]],
            (uint8_t *)(&remoteAppConfig.remoteKeyTranslationCode[keyCode].translationValue[0]),
            translationCodeSize);

        // Update the number of keys in the report
        bleRemoteAppState->bytesInRemoteRpt[rptIndex] += translationCodeSize;

        // Flag that the standard key report has changed
        bleRemoteAppState->remoteRptChanged[rptIndex] = TRUE;

        return TRUE;
    }

    return FALSE;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function removes the key from the current report.

/// \param rptIndex index of the current report
/// \param keyCode  scan code of this key
/// \param translationCodeSize size of the translation code
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_removeKeyfromRemoteRpt(uint8_t rptIndex, uint8_t keyCode, uint8_t translationCodeSize)
{
    uint8_t locInRpt;

    for (locInRpt = 0; locInRpt + translationCodeSize <= bleRemoteAppState->bytesInRemoteRpt[rptIndex]; locInRpt += translationCodeSize)
    {
        // Find the key in the current report
        if (memcmp(&bleRemoteAppState->remoteRpt[rptIndex].keyCodes[locInRpt],
            remoteAppConfig.remoteKeyTranslationCode[keyCode].translationValue,
            translationCodeSize)
            == 0)
        {
            // Found it. Remvove it by replacing it with the last key and
            // reducing the key count by one. We can do this because the
            // order of keys in the report is not important.
            bleRemoteAppState->bytesInRemoteRpt[rptIndex] -= translationCodeSize;

            // Replace the current key with the last key
            memcpy(&bleRemoteAppState->remoteRpt[rptIndex].keyCodes[locInRpt],
                &bleRemoteAppState->remoteRpt[rptIndex].keyCodes[bleRemoteAppState->bytesInRemoteRpt[rptIndex]],
                translationCodeSize);

            // Clear the last key
            memset(&bleRemoteAppState->remoteRpt[rptIndex].keyCodes[bleRemoteAppState->bytesInRemoteRpt[rptIndex]],
                0,
                translationCodeSize);

            // Flag that the remote report has changed
            bleRemoteAppState->remoteRptChanged[rptIndex] = TRUE;

            return;
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function handles a key down event for the remote key report. It adds the
/// given key to the report if it is not already present.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param rptID report ID of this key
/// \param translationCodeSize size of the translation code
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_remoteRptProcEvtKeyDown(uint8_t upDownFlag, uint8_t keyCode, uint8_t rptID, uint8_t translationCodeSize)
{
    uint8_t rptIndex;

    // Check if the key is already in the report
    for (rptIndex = 0; rptIndex < remoteAppConfig.numOfRemoteRpt; ++rptIndex)
    {
        if (bleRemoteAppState->remoteRpt[rptIndex].reportID == rptID)
        {
            if (bleremoteapp_findKeyInRemoteRpt(rptIndex, keyCode, translationCodeSize))
            {
                return;
            }

            break;
        }
    }

    // Add key to the report if it has room
    for (rptIndex = 0; rptIndex < remoteAppConfig.numOfRemoteRpt; ++rptIndex)
    {
        if (bleRemoteAppState->remoteRpt[rptIndex].reportID == rptID)
        {
            if (!bleremoteapp_addKeytoRemoteRpt(rptIndex, keyCode, translationCodeSize))
            {
                // No room in report. Call error handler
                bleremoteapp_stdRptProcOverflow();
            }

            break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles a key up event for the remote key report. It removes
/// the key from the report if it is already present. Otherwise it does nothing.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param rptID report ID of this key
/// \param translationCodeSize size of the translation code
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_remoteRptProcEvtKeyUp(uint8_t upDownFlag, uint8_t keyCode, uint8_t rptID, uint8_t translationCodeSize)
{
    uint8_t rptIndex;

    // Remove key if the key is already in the report
    for (rptIndex = 0; rptIndex < remoteAppConfig.numOfRemoteRpt; ++rptIndex)
    {
        if (bleRemoteAppState->remoteRpt[rptIndex].reportID == rptID)
        {
            bleremoteapp_removeKeyfromRemoteRpt(rptIndex, keyCode, translationCodeSize);
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function handles key events targetted for the remote key report.
/// It updates the standard report with the given event.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param rptID report ID of this key
/// \param translationCodeSize size of the translation code
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_remoteRptProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t rptID, uint8_t translationCodeSize)
{
    // Processing depends on whether the event is an up or down event
    if (upDownFlag == KEY_DOWN)
    {
        bleremoteapp_remoteRptProcEvtKeyDown(upDownFlag, keyCode, rptID, translationCodeSize);
    }
    else
    {
        bleremoteapp_remoteRptProcEvtKeyUp(upDownFlag, keyCode, rptID, translationCodeSize);
    }
}


////////////////////////////////////////////////////////////////////////////////
/// Process a user defined key event.
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param translationCode user defined translation code associated with this key.
////////////////////////////////////////////////////////////////////////////////
//Todd: not used in blekb.c ???
void bleremoteapp_procEvtUserDefinedKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode)
{
    uint8_t keyType = kbKeyConfig[keyCode].type;

    if ((keyType >= KEY_TYPE_USER_DEF_0) && (keyType <= KEY_TYPE_USER_DEF_7))
    {
        uint8_t rptID = remoteAppConfig.remoteUserDefinedReportConfig[keyType - KEY_TYPE_USER_DEF_0].rptID;
        uint8_t translationCodeSize = remoteAppConfig.remoteUserDefinedReportConfig[keyType - KEY_TYPE_USER_DEF_0].translationCodeSize;
        bleremoteapp_remoteRptProcEvtKey(upDownFlag, keyCode, rptID, translationCodeSize);
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function should be called by the transport when it wants the application
/// to poll for user activity. This function performs the following actions:
///  - Polls for activity. If user activity is detected, events should be
///    queued up for processing
///  - If an unmasked user activity is detected, it passes the activity type to the
///    transports
///  - If the active transport is connected, requests generation of reports via
///    generateAndTransmitReports()
///  - Does connect button polling and informs the BT transport once the connect
///    button has been held for the configured amount of time.
/// Note: transport may be NULL if no transport context is required - like when
/// we are interested in event detection only
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_pollReportUserActivity(void)
{
    uint8_t activitiesDetectedInLastPoll;

    // Increment polling sequence number.
    bleRemoteAppState->pollSeqn++;

    // Check for activity. This should queue events if any user activity is detected
    activitiesDetectedInLastPoll = bleremoteapp_pollActivityUser();

    // Check if we have any user activity.
    if (activitiesDetectedInLastPoll != BLEHIDLINK_ACTIVITY_NONE &&
        !wiced_ble_hidd_link_is_connected())
    {
        // ask the transport to connect.
        wiced_ble_hidd_link_connect();
    }

    // Check if the active transport is connected
    if(wiced_ble_hidd_link_is_connected())
    {
        // Generate a report
        if(wiced_bt_hid_cfg_settings.security_requirement_mask)
        {
            if (wiced_blehidd_is_link_encrypted())
            {
                bleremoteapp_generateAndTxReports();
            }
        }
        else
        {
            bleremoteapp_generateAndTxReports();
        }

        if (!audioIsActive())
        {
#ifdef OTA_FIRMWARE_UPGRADE
            if (!wiced_ota_fw_upgrade_is_active())
#endif
            {
                // Poll the battery monitor
                wiced_hal_batmon_poll_monitor();
            }
        }
    }

}

////////////////////////////////////////////////////////////////////////////////
///  This function provides an implementation for the HID application abstract
///  function pollActivityUser(). It polls the following sources for user activity:
///        - Keys
///  Any detected activity is queued as events in the event fifo.
///  When pin code entry is in progress, this function will also call
///  handlePinCodeEntry to do pin code processing.
///
/// \return
///   Bit mapped value indicating
///       - HID_APP_ACTIVITY_NON_REPORTABLE - if any key (excluding connect button) is down. Always
///         set in pin code entry state
///       - HID_APP_ACTIVITY_REPORTABLE - if any event is queued. Always
///         set in pin code entry state
///       - HID_APP_ACTIVITY_NONE otherwise
///  As long as it is not ACTIVITY_NONE, the btlpm will be notified for low power management.
////////////////////////////////////////////////////////////////////////////////
uint8_t bleremoteapp_pollActivityUser(void)
{
    uint8_t status;

    // Poll the hardware for events
    wiced_hal_mia_pollHardware();

    // Poll and queue key activity
    bleremoteapp_pollActivityKey();

    // For all other cases, return value indicating whether any event is pending or
    status = (wiced_hidd_event_queue_get_num_elements(&bleRemoteAppState->appEventQueue) ? BLEHIDLINK_ACTIVITY_REPORTABLE : BLEHIDLINK_ACTIVITY_NONE);
    status |= ((bleRemoteAppState->modKeysInStdRpt||bleRemoteAppState->keysInStdRpt|| bleRemoteAppState->keysInBitRpt)? BLEHIDLINK_ACTIVITY_NON_REPORTABLE : BLEHIDLINK_ACTIVITY_NONE);


#ifdef SUPPORT_TOUCHPAD
    if (touchpad)
    {
        status |= pollTouchpadActivity();
    }
#endif

#ifdef SUPPORT_MOTION
    //Poll motion sensor activity
    if (motionsensor)
    {
        status |= bleremoteapp_pollActivitySensor();
    }
#endif

#ifdef SUPPORT_AUDIO
    //Poll voice activity
    bleremoteapp_pollActivityVoice();
#endif

    return status;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function polls for key activity and queues any key events in the
/// FW event queue. Events from the keyscan driver are processed until the driver
/// runs out of events. Connect button events are separated out and handled here
/// since we don't want them to go through the normal event queue. If necessary,
/// the end of scan cycle event after the connect button is suppressed. Also
/// note that connect button events are suppressed during recovery to eliminate
/// spurious connect button events.
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_pollActivityKey(void)
{
    uint8_t suppressEndScanCycleAfterConnectButton;
#ifdef SUPPORT_IR
    uint8_t IR_code = 0x3D;
#endif

    // Assume that end-of-cycle event suppression is on
    suppressEndScanCycleAfterConnectButton = TRUE;

    // Process all key events from the keyscan driver
    while (wiced_hal_keyscan_get_next_event(&bleRemoteAppState->kbKeyEvent.keyEvent))
    {
        //WICED_BT_TRACE("keyCode=0x%x, upDown=%d\n", bleRemoteAppState->kbKeyEvent.keyEvent.keyCode, bleRemoteAppState->kbKeyEvent.keyEvent.upDownFlag);
#ifdef CONNECTED_ADVERTISING_SUPPORTED
        // while in CONNECTED - advertising state, any key press shall terminate terminate advertising and go back to CONNECTED state
        if (ble_hidd_link.second_conn_state && wiced_ble_hidd_link_is_connected() &&
            (bleRemoteAppState->kbKeyEvent.keyEvent.keyCode != END_OF_SCAN_CYCLE) && (bleRemoteAppState->kbKeyEvent.keyEvent.upDownFlag == KEY_DOWN))
        {
            wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
        }
#endif
        // Check for connect button
        if (bleRemoteAppState->kbKeyEvent.keyEvent.keyCode == kbAppConfig.connectButtonScanIndex)
        {
            // Pass current connect button state to connect button handler
            bleremoteapp_connectButtonHandler(
                    ((bleRemoteAppState->kbKeyEvent.keyEvent.upDownFlag == KEY_DOWN)?
                     CONNECT_BUTTON_DOWN:CONNECT_BUTTON_UP));
        }
#ifdef SUPPORT_IR
        //IR button
        else if (ir && (bleRemoteAppState->kbKeyEvent.keyEvent.keyCode == remoteAppConfig.IR_ButtonScanIndex))
        {
            if (bleRemoteAppState->kbKeyEvent.keyEvent.upDownFlag == KEY_DOWN)
            {
                if( wiced_is_timer_in_use(&allow_irtx_timer))
                {
                  pending_IR = TRUE;
                  pending_IR_code = IR_code;
                  pending_IR_repeat = TRUE;
                  pending_IR_repeat_count = 2;
                }
                else
                {
                //send IR
                ir->SendIR(IR_code, TRUE, 2);
                }
            }
            else
            {
                //stop IR
                ir->stopRepeat();
                pending_IR_repeat = FALSE;
            }
        }
#endif
#ifdef SUPPORT_MOTION
        //motion START button
        else if (motionsensor && (bleRemoteAppState->kbKeyEvent.keyEvent.keyCode == remoteAppConfig.MotionStart_ButtonScanIndex))
        {
            if (!motionsensor->isEnabled() //motion is not enabled
                && !audioIsActive()) // audio is not active
            {
                //start motion sensor
                motionsensor->enable(TRUE);
                wiced_blehidd_allow_slave_latency(FALSE);
            }
        }
        //motion STOP button
        else if (motionsensor && (bleRemoteAppState->kbKeyEvent.keyEvent.keyCode == remoteAppConfig.MotionStop_ButtonScanIndex))
        {
            if (!audioIsActive()) //audio is not active
            {
                if (motionsensor->isEnabled())
                {
                    //stop motion sensor
                    motionsensor->enable(FALSE);
#ifdef OTA_FIRMWARE_UPGRADE
                    if (!wiced_ota_fw_upgrade_is_active())
#endif
                    {
                        wiced_blehidd_allow_slave_latency(TRUE);
                    }
                }
                //NOTE: if disconnected, any key press except connectbutton should trigger a reconnect attempt.
                if (!wiced_ble_hidd_link_is_connected())
                {
                    wiced_hidd_event_queue_add_event_with_overflow(&bleRemoteAppState->appEventQueue,
                                &bleRemoteAppState->eventNULL.eventInfo,
                                sizeof(bleRemoteAppState->eventNULL),
                                bleRemoteAppState->pollSeqn);
                }
            }
        }
#endif
#ifdef SUPPORT_AUDIO
        //audio button
        else if (bleRemoteAppState->kbKeyEvent.keyEvent.keyCode == remoteAppConfig.Voice_ButtonScanIndex)
        {
            if (remoteAppConfig.audio_mode == WICED_HIDD_AUDIO_BUTTON_SEND_MSG)
            {
                if (!bleRemoteAppState->recoveryInProgress && (bleRemoteAppState->kbKeyEvent.keyEvent.upDownFlag == KEY_DOWN))
                {
                    bleRemoteAppState->audiobutton_pressed = TRUE;
#ifdef SUPPORT_MOTION
                    //stop motion sensor while play audio
                    if (motionsensor && motionsensor->isEnabled())
                    {
                        motionsensor->enable(FALSE);
                    }
#endif
                    bleRemoteAppState->voiceCtrlEvent.eventInfo.eventType = HID_EVENT_RC_MIC_START_REQ;
                }
                else
                {
                    bleRemoteAppState->audiobutton_pressed = FALSE;
                    bleRemoteAppState->audioStopEventInQueue = TRUE;
                    bleRemoteAppState->voiceCtrlEvent.eventInfo.eventType = HID_EVENT_RC_MIC_STOP_REQ;
                }
                wiced_hidd_event_queue_add_event_with_overflow(&bleRemoteAppState->appEventQueue,
                            &bleRemoteAppState->voiceCtrlEvent.eventInfo,
                            sizeof(bleRemoteAppState->voiceCtrlEvent),
                            bleRemoteAppState->pollSeqn);
            }
            else if (remoteAppConfig.audio_mode == WICED_HIDD_AUDIO_BUTTON_SEND_PCM)
            {
                uint8_t cond_startAudio = (bleRemoteAppState->kbKeyEvent.keyEvent.upDownFlag == KEY_DOWN)  && !audioIsActive() ;
                uint8_t cond_stopAudio  = !(bleRemoteAppState->kbKeyEvent.keyEvent.upDownFlag == KEY_DOWN) && audioIsActive() ;

                if (cond_startAudio && !bleRemoteAppState->recoveryInProgress)
                {
#ifdef SUPPORT_MOTION
                    //stop motion sensor while play audio
                    if ( motionIsEnabled() )
                    {
                        motionsensor->enable(FALSE);
                    }
#endif
                    // audio is inactive, activate
                    wiced_hidd_mic_audio_set_active(WICED_TRUE);
                }
                else if (cond_stopAudio)
                {
                    // audio is active, stop audio
                    wiced_hidd_mic_audio_stop();

                    //re-enable app polling
                    wiced_ble_hidd_link_enable_poll_callback(WICED_TRUE);

                    WICED_BT_TRACE("overflow = %d\n", wiced_hidd_mic_audio_is_overflow());
                }
            }
        }
#endif
        else
        {
            // Check if this is an end-of-scan cycle event
            if (bleRemoteAppState->kbKeyEvent.keyEvent.keyCode == END_OF_SCAN_CYCLE)
            {
                // Yes. Queue it if it need not be suppressed
                if (!suppressEndScanCycleAfterConnectButton)
                {
                    wiced_hidd_event_queue_add_event_with_overflow(&bleRemoteAppState->appEventQueue, &bleRemoteAppState->kbKeyEvent.eventInfo, sizeof(bleRemoteAppState->kbKeyEvent), bleRemoteAppState->pollSeqn);
                }

                // Enable end-of-scan cycle suppression since this is the start of a new cycle
                suppressEndScanCycleAfterConnectButton = TRUE;
            }
            else
            {
#ifdef SUPPORT_TOUCHPAD
                HidEventKey * ke = &(bleRemoteAppState->kbKeyEvent);
                uint8_t sendKey = TRUE;

                if (touchpad && (TOUCHPAD_BUTTON_KEYINDEX == ke->keyEvent.keyCode))
                {
                    // this function will update key event to the real event we want to send
                    sendKey = handleTouchpadVirtualKey(ke);
                }

                if (sendKey)
                {
#endif
                    // No. Queue the key event
                    wiced_hidd_event_queue_add_event_with_overflow(&bleRemoteAppState->appEventQueue, &bleRemoteAppState->kbKeyEvent.eventInfo,
                            sizeof(bleRemoteAppState->kbKeyEvent), bleRemoteAppState->pollSeqn);
                    // Disable end-of-scan cycle suppression
                    suppressEndScanCycleAfterConnectButton = FALSE;
#ifdef SUPPORT_TOUCHPAD
                }
#endif
            }
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function polls for motion sensor activity and queues any events in the
/// FW event queue.
/////////////////////////////////////////////////////////////////////////////////
uint8_t bleremoteapp_pollActivitySensor(void)
{
#ifdef SUPPORT_MOTION
    HidEventUserDefine event;
    if (HID_EVENT_AVAILABLE == motionsensor->pollActivity(&event))
    {
        wiced_hidd_event_queue_add_event_with_overflow(&bleRemoteAppState->appEventQueue,
                     (HidEvent *)&event,
                     sizeof(HidEventUserDefine),
                     bleRemoteAppState->pollSeqn);
        return BLEHIDLINK_ACTIVITY_REPORTABLE;
    }
#endif
    return BLEHIDLINK_ACTIVITY_NONE;
}


#ifdef SUPPORT_MOTION
/////////////////////////////////////////////////////////////////////////////////
/// This function retrieves the motion event, generates reports as
/// necessary
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_procEvtMotion(void)
{
    HidEventUserDefine *pEvt;
    while (((pEvt = (HidEventUserDefine *)wiced_hidd_event_queue_get_current_element(&bleRemoteAppState->appEventQueue)) != NULL) &&
           (pEvt->eventInfo.eventType == HID_EVENT_MOTION_DATA_AVAILABLE))
    {
        memcpy(bleremote_motion_rpt, pEvt->userDataPtr, MOTIONRPT_MAX_DATA);

        wiced_ble_hidd_link_send_report(MOTION_REPORT_ID, WICED_HID_REPORT_TYPE_INPUT, bleremote_motion_rpt, MOTIONRPT_MAX_DATA);

        // We are done with this event. Delete it
        wiced_hidd_event_queue_remove_current_element(&bleRemoteAppState->appEventQueue);
    }
}
#endif

////////////////////////////////////////////////////////////////////////////////
/// Process a user defined event. By default the keyboard application
/// define key and scroll events. If an application needs additional types of
/// events it should define them and override this function to process them.
/// This function should remove the user defined event from the event queue
/// after processing it. This function can consume additional events after
/// the user defined event.
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_procEvtUserDefined(void)
{
    HidEvent *curEvent = (HidEvent *)wiced_hidd_event_queue_get_current_element(&bleRemoteAppState->appEventQueue);

    switch (curEvent->eventType)
    {
#ifdef SUPPORT_MOTION
        case HID_EVENT_MOTION_DATA_AVAILABLE:
            bleremoteapp_procEvtMotion();
            break;
#endif
#ifdef SUPPORT_AUDIO
        case HID_EVENT_RC_MIC_START_REQ:
            wiced_blehidd_allow_slave_latency(FALSE);
            bleremoteapp_procEvtVoiceCtrl(curEvent->eventType);
            break;

        case HID_EVENT_RC_MIC_STOP_REQ:
            bleremoteapp_procEvtVoiceCtrl(curEvent->eventType);

            //stop audio codec
            if( audioIsActive() )
            {
                stopMicCommandPending = TRUE;
                wiced_start_timer(&mic_stop_command_pending_timer,100); // 100 mSec timeout in ms,
                wiced_hidd_mic_audio_stop();

                //re-enable app polling
                wiced_ble_hidd_link_enable_poll_callback(WICED_TRUE);

                WICED_BT_TRACE("overflow = %d\n", wiced_hidd_mic_audio_is_overflow());
            }
            break;

        case HID_EVENT_VOICE_DATA_AVAILABLE: //audio data
            bleremoteApp_procEvtVoice();
            break;

        case HID_EVENT_MIC_START: //start audio
            // Delete it
            while (HID_EVENT_MIC_START == curEvent->eventType)
            {
                wiced_hidd_event_queue_remove_current_element(&bleRemoteAppState->appEventQueue);
                curEvent = (HidEvent *)wiced_hidd_event_queue_get_current_element(&bleRemoteAppState->appEventQueue);
            }

            //start audio codec
            if( !audioIsActive() )
            {
                //reset the counter that keeps track of voice events # in the event queue
                bleRemoteAppState->audioPacketInQueue = 0;

                // audio is inactive, activate
                wiced_hidd_mic_audio_set_active(WICED_TRUE);
            }
            break;

        case HID_EVENT_MIC_STOP: //stop audio
            // Delete it
            while (HID_EVENT_MIC_STOP == curEvent->eventType)
            {
                wiced_hidd_event_queue_remove_current_element(&bleRemoteAppState->appEventQueue);
                bleRemoteAppState->micStopEventInQueue = FALSE;
                curEvent = (HidEvent *)wiced_hidd_event_queue_get_current_element(&bleRemoteAppState->appEventQueue);
            }

            stopMicCommandPending = FALSE;
            if (wiced_is_timer_in_use(&mic_stop_command_pending_timer))
            {
                wiced_stop_timer(&mic_stop_command_pending_timer);
#ifdef OTA_FIRMWARE_UPGRADE
                if (!wiced_ota_fw_upgrade_is_active())
#endif
                {
                    wiced_blehidd_allow_slave_latency(TRUE);
                }
            }

            //stop audio codec
            if( audioIsActive() )
            {
                wiced_hidd_mic_audio_stop();

                //re-enable app polling
                wiced_ble_hidd_link_enable_poll_callback(WICED_TRUE);

                WICED_BT_TRACE("overflow = %d\n", wiced_hidd_mic_audio_is_overflow());
            }

            break;

        case HID_EVENT_AUDIO_MODE:
            // Delete it
            wiced_hidd_event_queue_remove_current_element(&bleRemoteAppState->appEventQueue);
            //send Rpt
            bleremoteapp_voiceModeSend();
            break;

        case HID_EVENT_AUDIO_CODEC_RD:
            // Delete it
            wiced_hidd_event_queue_remove_current_element(&bleRemoteAppState->appEventQueue);

            //send Rpt
            bleremoteapp_voiceReadCodecSetting();
            break;

        case HID_EVENT_AUDIO_CODEC_WT:
            // Delete it
            wiced_hidd_event_queue_remove_current_element(&bleRemoteAppState->appEventQueue);

            //send Rpt
            bleremoteapp_voiceWriteCodecSetting();
            break;
#endif
#ifdef SUPPORT_TOUCHPAD
        case HID_EVENT_TP_FINGER_STATUS_CHANGE:
            // do nothing for now, FALL THROUGH
        case HID_EVENT_TP_DATA:
            bleremoteapp_procEvtTouchpad();
            wiced_hidd_event_queue_remove_current_element(&bleRemoteAppState->appEventQueue);
            break;
#endif

        case HID_EVENT_ANY:
            //delete it.
            while (HID_EVENT_ANY == curEvent->eventType)
            {
                wiced_hidd_event_queue_remove_current_element(&bleRemoteAppState->appEventQueue);
                curEvent = (HidEvent *)wiced_hidd_event_queue_get_current_element(&bleRemoteAppState->appEventQueue);
            };
            break;

        default:
            //delete it.
            wiced_hidd_event_queue_remove_current_element(&bleRemoteAppState->appEventQueue);
            break;
    }
}

#ifdef SUPPORT_AUDIO
/////////////////////////////////////////////////////////////////////////////////
/// This function polls for voice activity and queues any events in the
/// FW event queue.
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_pollActivityVoice(void)
{
    //voice is active, poll and queue voice data
    if ( audioIsActive() )
    {
        if (wiced_hidd_mic_audio_poll_activity((void *)&bleRemoteAppState->voiceEvent))
        {
            //disable app polling. App polling interval is not reliable, to avoid audio packet loss, rely on audio interrupt instead.
            wiced_ble_hidd_link_enable_poll_callback(WICED_FALSE);
            wiced_hidd_event_queue_add_event_with_overflow(&bleRemoteAppState->appEventQueue,
                     &bleRemoteAppState->voiceEvent.eventInfo,
                     sizeof(bleRemoteAppState->voiceEvent),
                     bleRemoteAppState->pollSeqn);

            //Good audio quality means at any time the # of voice events in queue should be less than audio FIFO size.
            //Otherwise, audio FIFO overflows.
            if (++bleRemoteAppState->audioPacketInQueue >= wiced_hidd_mic_audio_FIFO_count())
            {
                wiced_hidd_mic_audio_overflow();
            }
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function retrieves the voice control event, generates reports as
/// necessary
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_procEvtVoiceCtrl(uint8_t eventType)
{
    HidEventUserDefine *voicectrl_event;
    wiced_hidd_voice_control_report_t audioMsgReq;

    //if detecting continuous voice control event, only process once.
    while (((voicectrl_event = (HidEventUserDefine *)wiced_hidd_event_queue_get_current_element(&bleRemoteAppState->appEventQueue))!=NULL) &&
           (voicectrl_event->eventInfo.eventType == eventType))
    {
        // Delete it
        wiced_hidd_event_queue_remove_current_element(&bleRemoteAppState->appEventQueue);
    }

    memset(&audioMsgReq, 0, sizeof(wiced_hidd_voice_control_report_t));
    audioMsgReq.reportId = WICED_HIDD_VOICE_CTL_REPORT_ID;
    audioMsgReq.format = (HID_EVENT_RC_MIC_STOP_REQ == eventType) ?  WICED_HIDD_RC_MIC_STOP_REQ :  WICED_HIDD_RC_MIC_START_REQ;
    audioMsgReq.rsvd = remoteAppConfig.audio_mode; //put "audio mode" info in the reserved byte

    //set gatt attribute value here before sending the report
    memcpy(bleremote_voice_ctrl_input_rpt, &audioMsgReq.format, sizeof(wiced_hidd_voice_control_report_t) - sizeof(audioMsgReq.reportId));
    wiced_ble_hidd_link_send_report(WICED_HIDD_VOICE_CTL_REPORT_ID,WICED_HID_REPORT_TYPE_INPUT,&audioMsgReq.format,
                        sizeof(wiced_hidd_voice_control_report_t) - sizeof(audioMsgReq.reportId));

}

/////////////////////////////////////////////////////////////////////////////////
/// This function retrieves the motion event, generates reports as
/// necessary
/////////////////////////////////////////////////////////////////////////////////
void bleremoteApp_procEvtVoice(void)
{
    uint16_t len;
    uint8_t i;
    uint8_t audio_outData[WICED_HIDD_MIC_AUDIO_BUFFER_SIZE*2+1];
    HidEventUserDefine *voice_event = (HidEventUserDefine *)wiced_hidd_event_queue_get_current_element(&bleRemoteAppState->appEventQueue);
    wiced_hidd_voice_report_t *audioPtr = (wiced_hidd_voice_report_t *)voice_event->userDataPtr;

    len = wiced_hidd_mic_audio_get_audio_out_data(audioPtr, audio_outData);

    if (len > 0)
    {
#ifdef ATT_MTU_SIZE_180
        //send audio out as one GATT MTU.
        memcpy(bleremote_voice_rpt, audio_outData, len);
        wiced_ble_hidd_link_send_report(audioPtr->reportId,WICED_HID_REPORT_TYPE_INPUT, bleremote_voice_rpt, len);
#else
#ifdef SBC_ENCODER
        //60 bytes data splits into 3 parts and sends out
        for(i=0; i<3; i++)
        {
            memcpy(bleremote_voice_rpt, &audio_outData[20*i], 20);
            wiced_ble_hidd_link_send_report(audioPtr->reportId,WICED_HID_REPORT_TYPE_INPUT, bleremote_voice_rpt, 20);
        }
#endif
#ifdef CELT_ENCODER
        //89 bytes data splits into 5 parts and sends out
        for(i=0; i<4; i++)
        {
            memcpy(bleremote_voice_rpt, &audio_outData[20*i], 20);
            wiced_ble_hidd_link_send_report(audioPtr->reportId,WICED_HID_REPORT_TYPE_INPUT, bleremote_voice_rpt, 20);
        }
        memcpy(bleremote_voice_rpt, &audio_outData[80], 9);
        wiced_ble_hidd_link_send_report(audioPtr->reportId,WICED_HID_REPORT_TYPE_INPUT, bleremote_voice_rpt, 9);
#endif
#endif
    }

    // We are done with this event. Delete it
    wiced_hidd_event_queue_remove_current_element(&bleRemoteAppState->appEventQueue);
    bleRemoteAppState->audioPacketInQueue--;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the voice control report(WICED_HIDD_RC_VOICEMODE_RD_ACK)
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_voiceModeSend(void)
{
    wiced_hidd_voice_control_report_t audiomodeRsp;
    memset(&audiomodeRsp, 0, sizeof(wiced_hidd_voice_control_report_t));
    audiomodeRsp.reportId = WICED_HIDD_VOICE_CTL_REPORT_ID;
    audiomodeRsp.format = WICED_HIDD_RC_VOICEMODE_RD_ACK;
    audiomodeRsp.rsvd = remoteAppConfig.audio_mode; //audio mode

    //set gatt attribute value here before sending the report
    memcpy(bleremote_voice_ctrl_input_rpt, &audiomodeRsp.format, sizeof(wiced_hidd_voice_control_report_t) - sizeof(audiomodeRsp.reportId));
    wiced_ble_hidd_link_send_report(WICED_HIDD_VOICE_CTL_REPORT_ID,WICED_HID_REPORT_TYPE_INPUT,&audiomodeRsp.format,
                sizeof(wiced_hidd_voice_control_report_t) - sizeof(audiomodeRsp.reportId));
}


/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the voice control report(WICED_HIDD_RC_CODECSETTINGS_RD_ACK)
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_voiceReadCodecSetting(void)
{
    wiced_hidd_voice_control_report_t audioCodecRsp;
    memset(&audioCodecRsp, 0, sizeof(wiced_hidd_voice_control_report_t));
    audioCodecRsp.reportId = WICED_HIDD_VOICE_CTL_REPORT_ID;
    audioCodecRsp.format =  WICED_HIDD_RC_CODECSETTINGS_RD_ACK;
    audioCodecRsp.rsvd = bleRemoteAppState->codecSettingMsg_type;  // This information is saved in rxData
    if (wiced_hidd_mic_audio_read_codec_setting(&audioCodecRsp))
    {
        //set gatt attribute value here before sending the report
        memcpy(bleremote_voice_ctrl_input_rpt, &audioCodecRsp.format, sizeof(wiced_hidd_voice_control_report_t) - sizeof(audioCodecRsp.reportId));

        wiced_ble_hidd_link_send_report(WICED_HIDD_VOICE_CTL_REPORT_ID,WICED_HID_REPORT_TYPE_INPUT,&audioCodecRsp.format,
                sizeof(wiced_hidd_voice_control_report_t) - sizeof(audioCodecRsp.reportId));
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the voice control report(WICED_HIDD_RC_CODECSETTINGS_WT_ACK)
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_voiceWriteCodecSetting(void)
{
    wiced_hidd_voice_control_report_t audioCodecRsp;
    memset(&audioCodecRsp, 0, sizeof(wiced_hidd_voice_control_report_t));
    audioCodecRsp.reportId = WICED_HIDD_VOICE_CTL_REPORT_ID;
    audioCodecRsp.format = WICED_HIDD_RC_CODECSETTINGS_WT_ACK;

    audioCodecRsp.rsvd    = bleRemoteAppState->codecSettingMsg_type;
    if (bleRemoteAppState->codecSettingMsg_dataCnt > 0)
    {
        // First Write
        wiced_hidd_mic_audio_write_codec_setting(audioCodecRsp.rsvd, bleRemoteAppState->codecSettingMsg_dataBuffer);

        // Now read the values from the codec registers
        if (wiced_hidd_mic_audio_read_codec_setting(&audioCodecRsp))
        {
            //set gatt attribute value here before sending the report
            memcpy(bleremote_voice_ctrl_input_rpt, &audioCodecRsp.format, sizeof(wiced_hidd_voice_control_report_t) - sizeof(audioCodecRsp.reportId));

            wiced_ble_hidd_link_send_report(WICED_HIDD_VOICE_CTL_REPORT_ID,WICED_HID_REPORT_TYPE_INPUT,&audioCodecRsp.format,
                sizeof(wiced_hidd_voice_control_report_t) - sizeof(audioCodecRsp.reportId));
        }
    }
}
#endif

////////////////////////////////////////////////////////////////////////////////
/// Generic interrupt handler, audio
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_appActivityDetected(void *remApp)
{
    bleremoteapp_pollReportUserActivity();
}


/////////////////////////////////////////////////////////////////////////////////
/// This function informs the application that the state of a link changed.
/// Note that it is expected that the application will do mainly link agnostic
/// activities in this method
///
/// \param newState new state of the link
/////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_transportStateChangeNotification(uint32_t newState)
{
    int16_t flags;
    WICED_BT_TRACE("Transport state changed to %d\n", newState);

    bleRemoteAppState->allowSDS = 0;

    //stop allow Shut Down Sleep (SDS) timer
    wiced_stop_timer(&allow_sleep_timer);

    if(newState == BLEHIDLINK_CONNECTED)
    {
        //get host client configuration characteristic descriptor values
        flags = wiced_ble_hidd_host_info_get_flags(ble_hidd_link.gatts_peer_addr, ble_hidd_link.gatts_peer_addr_type);
        if(flags != -1)
        {
            WICED_BT_TRACE("host config flag:%08x\n",flags);
            bleremoteapp_updateGattMapWithNotifications(flags);
        }
        else
        {
            WICED_BT_TRACE("host NOT found!\n");
        }

        // enable ghost detection
        wiced_hal_keyscan_enable_ghost_detection(TRUE);

#ifdef SUPPORT_TOUCHPAD
        if (touchpad)
        {
            touchpad->setEnable(TRUE);
        }
#endif

#if defined(SUPPORT_MOTION)
        if (motionsensor && motionsensor->isEnabled())
        {
            // if connected and motion is enabled, prepare to send motion data
            wiced_blehidd_allow_slave_latency(FALSE);
        }
#endif

        wiced_ble_hidd_link_enable_poll_callback(WICED_TRUE);

        if(firstTransportStateChangeNotification)
        {
            //Wake up from HID Off and already have a connection then allow HID Off in 1 second
            //This will allow time to send a key press.
            //To do need to check if key event is in the queue at lpm query
            wiced_start_timer(&allow_sleep_timer,1000); // 1 second. timeout in ms
        }
        else
        {
            //We connected after power on reset or HID off recovery.
            //Start 20 second timer to allow time to setup connection encryption
            //before allowing HID Off/Micro-BCS.
            wiced_start_timer(&allow_sleep_timer,20000); //20 seconds. timeout in ms

            //start 15 second timer to make sure connection param update is requested before SDS
            wiced_start_timer(&bleremote_conn_param_update_timer,15000); //15 seconds. timeout in ms
        }

    }
    else if (newState == BLEHIDLINK_DISCONNECTED)
    {
        //allow Shut Down Sleep (SDS) only if we are not attempting reconnect
        if (!wiced_is_timer_in_use(&ble_hidd_link.reconnect_timer))
            wiced_start_timer(&allow_sleep_timer, 2000); // 2 seconds. timeout in ms

        // disable Ghost detection
        wiced_hal_keyscan_enable_ghost_detection(FALSE);

#ifdef SUPPORT_AUDIO
        //stop audio
        wiced_hidd_mic_audio_stop();
#endif
#ifdef SUPPORT_MOTION
        if (motionsensor)
        {
            if (wiced_ble_hidd_host_info_is_bonded())
            {
                // if it's bonded we want motion sensor to wake us up to try reconnect
                motionsensor->enableInterrupt();
            }
            else
            {
                //stop motion sensor
                if (motionsensor->isEnabled())
                {
                    motionsensor->enable(FALSE);
#ifdef OTA_FIRMWARE_UPGRADE
                    if (!wiced_ota_fw_upgrade_is_active())
#endif
                    {
                        wiced_blehidd_allow_slave_latency(TRUE);
                    }
                }
            }
        }
#endif

        // Tell the transport to stop polling
        wiced_ble_hidd_link_enable_poll_callback(WICED_FALSE);
    }
#ifdef SUPPORT_TOUCHPAD
    else if (newState == BLEHIDLINK_DISCOVERABLE)
    {
        if (touchpad)
        {
            touchpad->clearEvent();
        }
    }
#endif
    else if ((newState == BLEHIDLINK_ADVERTISING_IN_uBCS_DIRECTED) || (newState == BLEHIDLINK_ADVERTISING_IN_uBCS_UNDIRECTED))
    {
        bleRemoteAppState->allowSDS = 1;
    }

    if(firstTransportStateChangeNotification)
        firstTransportStateChangeNotification = 0;

}

////////////////////////////////////////////////////////////////////////////////
/// Client characteritics conf write handler for HID control point
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_ctrlPointWrite(wiced_hidd_report_type_t reportType,
                          uint8_t reportId,
                          void *payload,
                          uint16_t payloadSize)
{
    WICED_BT_TRACE("disconnecting\n");

    wiced_ble_hidd_link_disconnect();
}

////////////////////////////////////////////////////////////////////////////////
/// Client characteritics conf write handler for standard key report
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_clientConfWriteRptStd(wiced_hidd_report_type_t reportType,
                                 uint8_t reportId,
                                 void *payload,
                                 uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t  indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

    WICED_BT_TRACE("clientConfWriteRptStd\n");

    bleremoteapp_updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_STD_RPT);
}

////////////////////////////////////////////////////////////////////////////////
/// Client characteritics conf write handler for bitmap report
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_clientConfWriteRptBitMapped(wiced_hidd_report_type_t reportType,
                                       uint8_t reportId,
                                       void *payload,
                                       uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t  indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

    WICED_BT_TRACE("clientConfWriteRptBitMapped\n");

    bleremoteapp_updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_BIT_MAPPED_RPT);
}

////////////////////////////////////////////////////////////////////////////////
/// Client characteritics conf write handler for battery report
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_clientConfWriteBatteryRpt(wiced_hidd_report_type_t reportType,
                                     uint8_t reportId,
                                     void *payload,
                                     uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t  indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

    WICED_BT_TRACE("clientConfWriteBatteryRpt\n");

    bleremoteapp_updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_BATTERY_RPT);
}


////////////////////////////////////////////////////////////////////////////////
/// Client characteritics conf write handler for motion report
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_clientConfWriteRptMotion(wiced_hidd_report_type_t reportType,
                                 uint8_t reportId,
                                 void *payload,
                                 uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

    bleremoteapp_updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_MOTION_RPT);
}

////////////////////////////////////////////////////////////////////////////////
/// Client characteritics conf write handler for user defined key report
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_clientConfWriteRptUserDefinedKey(wiced_hidd_report_type_t reportType,
                                 uint8_t reportId,
                                 void *payload,
                                 uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

    bleremoteapp_updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_USER_DEFINED_KEY_RPT);
}

#ifdef SUPPORT_AUDIO
////////////////////////////////////////////////////////////////////////////////
/// Client characteristics conf write handler for audio report
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_clientConfWriteRptVoice(wiced_hidd_report_type_t reportType,
                                 uint8_t reportId,
                                 void *payload,
                                 uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

    bleremoteapp_updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_VOICE_RPT);
}

////////////////////////////////////////////////////////////////////////////////
/// Client characteristics conf write handler for audio control report
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_clientConfWriteRptVoiceCtrl(wiced_hidd_report_type_t reportType,
                                 uint8_t reportId,
                                 void *payload,
                                 uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

    bleremoteapp_updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_VOICE_CTRL_RPT);
}
#endif

#ifdef SUPPORT_TOUCHPAD
////////////////////////////////////////////////////////////////////////////////
/// Client characteristics conf write handler for touchpad report
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_clientConfWriteRptTouchpad(wiced_hidd_report_type_t reportType,
                                 uint8_t reportId,
                                 void *payload,
                                 uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //WICED_BT_TRACE("\nbleremoteapp_clientConfWriteRptTouchpad: notification=%x, payload=%x\n", notification, *(uint16_t *)payload);
    bleremoteapp_updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_TOUCHPAD_RPT);
}
#endif

////////////////////////////////////////////////////////////////////////////////
///This function updates the client configuration characteristic values for the client in NVRAM
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_updateClientConfFlags(uint16_t enable, uint16_t featureBit)
{
    bleremoteapp_updateGattMapWithNotifications(wiced_ble_hidd_host_info_update_flags(ble_hidd_link.gatts_peer_addr, ble_hidd_link.gatts_peer_addr_type, enable,featureBit));
}

////////////////////////////////////////////////////////////////////////////////
/// handler for GATT map update
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_updateGattMapWithNotifications(uint16_t flags)
{
    uint8_t i = 0;
    wiced_blehidd_report_gatt_characteristic_t* map = bleRemoteReportModeGattMap;

    blehostlist_flags = flags;
    //update characteristic_client_configuration for gatt read req
    for (i=0; i<MAX_NUM_CLIENT_CONFIG_NOTIF; i++)
    {
        characteristic_client_configuration[i] = (flags >> i) & 0x0001;
    }

    for(i = 0; i < sizeof(bleRemoteReportModeGattMap)/sizeof(bleRemoteReportModeGattMap[0]); i++)
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
uint32_t bleremoteapp_sleep_handler(wiced_sleep_poll_type_t type )
{
    uint32_t ret = WICED_SLEEP_NOT_ALLOWED;

#ifndef  TESTING_USING_HCI
    // Check if keys are pressed before deciding whether sleep is allowed
    bleRemoteAppState->keyInterrupt_On = wiced_hal_keyscan_is_any_key_pressed();
#endif

    switch(type)
    {
        case WICED_SLEEP_POLL_TIME_TO_SLEEP:
            ret = WICED_SLEEP_MAX_TIME_TO_SLEEP;

            //if we are in the middle of keyscan recovery, no sleep
            if (bleRemoteAppState->recoveryInProgress)
            {
                ret = 0;
            }

#ifdef SUPPORT_IR
            //if IR Tx is active, no sleep
            if ( irtxIsActive() )
            {
                ret = 0;
            }
#endif
#ifdef SUPPORT_AUDIO
            //if audio is active, no sleep
            if ( audioIsActive() )
            {
                ret = 0;
            }
#endif
#if defined(SUPPORT_TOUCHPAD) && defined(HANDLE_STUCK_FINGER)
            // sometimes TP fails to interrupt us when finger is up, causing
            // touchpad->isActive() to be true and preventing sleep.
            // Added HANDLE_STUCK_FINGER logic in touchPad code, now we should
            // poll TP to determine sleep
            if ( touchpadIsActive() )
            {
                ret = 0;
            }
#endif
#ifdef SUPPORTING_FINDME
            if (!bleremoteapp_isAlertIdle())
            {
                ret = 0;
            }
#endif

#if defined(CYW20735B1)
            //app can do pre-SDS operation here when sleep time non-zero and pmu_attemptSleepState == PMU_SLEEP_SDS
            if ( ret && (pmu_attemptSleepState == 5))
            {
                sfi_exit_deep_power_down(FALSE);
                sfi_enter_deep_power_down();
            }
#endif
            break;

        case WICED_SLEEP_POLL_SLEEP_PERMISSION:
            ret = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;

            if (!bleRemoteAppState->allowSDS)
            {
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
            }

            //if key is not released, no Shut Down Sleep (SDS)
            if (bleRemoteAppState->keyInterrupt_On)
            {
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
            }

#ifdef SUPPORT_TOUCHPAD
            //if touch pad is actively sending tp data, no SDS
            if ( touchpadIsActive() )
            {
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
            }
#endif

#ifdef OTA_FIRMWARE_UPGRADE
            if ( wiced_ota_fw_upgrade_is_active() )
            {
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
            }
#endif

#if defined(SUPPORT_MOTION)
            if ( motionIsActive())
            {
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
            }
#endif
            break;

    }

    return ret;
}

////////////////////////////////////////////////////////////////////////////////////
/// restore contents from Always On Memory.
///////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_aon_restore(void)
{
    //cold boot
    if (wiced_hal_mia_is_reset_reason_por())
    {
#ifdef SUPPORT_MOTION
        motion_mode = SENSOR_DISABLED;
#endif
    }
    else //wake from SDS
    {
        wiced_ble_hidd_link_aon_action_handler(BLEHIDLINK_RESTORE_FROM_AON);
    }
}


#ifdef SUPPORTING_FINDME
////////////////////////////////////////////////////////////////////////////////
///  This function is the FIND ME profile initialization
/// - configure LED for find me alert
/// - register write handle cb for find me attribute handle.
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_findme_init(void)
{
    appFindmeState = (tAppFindmeState*)wiced_memory_permanent_allocate(sizeof(tAppFindmeState));
    memset(appFindmeState, 0x00, sizeof(tAppFindmeState));

    appFindmeState->alertType = FINDME_ALERT_TYPE; // ALERT_BUZ; // ALERT_BUZ_LED; // alert both Buz and Led
    if (appFindmeState->alertType & ALERT_BUZ)
    {
        appFindmeState->buz_id = FINDME_BUZ_PWM_ID;
//        wiced_blehidd_pwm_buz_init(GPIO_BUZ_PWM, 0);
    }
    //configure LED
    wiced_hal_gpio_configure_pin(GPIO_PORT_LED, GPIO_PULL_UP | GPIO_OUTPUT_ENABLE, 1);

    wiced_init_timer( &findme_led_timer, bleremoteapp_alertLed_timeout, 0, WICED_MILLI_SECONDS_TIMER );
    wiced_init_timer( &findme_buz_timer, bleremoteapp_alertBuz_timeout, 0, WICED_MILLI_SECONDS_TIMER );

    wiced_bt_gatt_legattdb_regWriteHandleCb((wiced_bt_gatt_LEGATTDB_WRITE_CB)bleremoteapp_findme_writeCb);
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to check if "find me alert" is active.
///   i.e.buz/led active or not.
////////////////////////////////////////////////////////////////////////////////
uint8_t bleremoteapp_isAlertIdle(void)
{
    return (appFindmeState->buz_alert_active | appFindmeState->led_alert_active) ? FALSE : TRUE;
}

////////////////////////////////////////////////////////////////////////////////
///  This function is the alert BUZ timer timeout handler
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertBuz_timeout(uint32_t unused)
{
    uint8_t patten_id = appFindmeState->buz_pattern_id;

//WICED_BT_TRACE("\nbuz alert");
    // reached to timeout value.expired.
    if (appFindmeState->buz_on) // buz is on state
    {
//WICED_BT_TRACE(" off\n");
        bleremoteapp_alertBuzOff(appFindmeState->buz_id);

        appFindmeState->buz_on = 0;
        appFindmeState->buz_repeat--;

        if (appFindmeState->buz_repeat == 0)
        {
            appFindmeState->buz_alert_active = 0; // buz alert is done.
            return;
        }

        if (appAlert_cfg.alertBuzCfg[patten_id].buz_off_ms > 0)
        {
            wiced_start_timer(&findme_buz_timer, appAlert_cfg.alertBuzCfg[patten_id].buz_off_ms); //timeout in ms
        }
    }
    else
    {
//WICED_BT_TRACE(" on\n");
        bleremoteapp_alertBuzOn(appFindmeState->buz_id);

        if (appAlert_cfg.alertBuzCfg[patten_id].buz_on_ms > 0)
        {
            wiced_start_timer(&findme_buz_timer, appAlert_cfg.alertBuzCfg[patten_id].buz_on_ms); //timeout in ms
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to set BUZ frequency
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertBuzFreq(uint8_t freq, uint16_t init_value, uint16_t toggle_val)
{
//    wiced_blehidd_pwm_buz_freq(freq, init_value, toggle_val);
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to turn on BUZ
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertBuzOn(uint8_t pwm_id)
{
//    wiced_blehidd_pwm_buz_on(pwm_id);
    appFindmeState->buz_on = 1;
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to turn off BUZ
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertBuzOff(uint8_t pwm_id)
{
//    wiced_blehidd_pwm_buz_off(pwm_id);
    appFindmeState->buz_on = 0;
}

////////////////////////////////////////////////////////////////////////////////
///  This function is the handling of alert BUZ based on alert level
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertBuzPlay(uint8_t pattern_id)
{
    if ((appAlert_cfg.alertBuzCfg[pattern_id].repeat_num == 0) ||
        (appAlert_cfg.alertBuzCfg[pattern_id].buz_on_ms == 0) ||
        (pattern_id >= APP_ALERT_PATTERN_MAX_ID)) // invalid parameter.
    {
        return;
    }

    bleremoteapp_alertBuzStop();

    appFindmeState->buz_pattern_id = pattern_id;
    appFindmeState->buz_repeat = appAlert_cfg.alertBuzCfg[pattern_id].repeat_num;

    bleremoteapp_alertBuzFreq(appAlert_cfg.alertBuzCfg[pattern_id].freq,
            appAlert_cfg.alertBuzCfg[pattern_id].init_value,
            appAlert_cfg.alertBuzCfg[pattern_id].toggle_val);
    bleremoteapp_alertBuzOn(appFindmeState->buz_id);
    //bleremoteapp_StartAlertBuzTimer(appAlert_cfg.alertBuzCfg[pattern_id].buz_on_ms);
    if (appAlert_cfg.alertBuzCfg[pattern_id].buz_on_ms > 0)
        wiced_start_timer(&findme_buz_timer, appAlert_cfg.alertBuzCfg[pattern_id].buz_on_ms); //timeout in ms
    appFindmeState->buz_alert_active = 1; // buz alert is activated.
}

////////////////////////////////////////////////////////////////////////////////
///  This function is the handling of NO ALERT (BUZ)
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertBuzStop(void)
{
    if (appFindmeState->buz_alert_active)
    {
        wiced_stop_timer(&findme_buz_timer);
        bleremoteapp_alertBuzOff(appFindmeState->buz_id);
        appFindmeState->buz_alert_active = 0; // buz alert is deactivated.
    }
}

////////////////////////////////////////////////////////////////////////////////
///  This function is the alert LED timer timeout handler
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertLed_timeout(uint32_t unused)
{
    uint8_t pattern_id = appFindmeState->led_pattern_id;

//WICED_BT_TRACE("\nled alert");
    if (appFindmeState->led_on) // led is on state
    {
        bleremoteapp_alertLedOff();
        appFindmeState->led_repeat--;

        if (appFindmeState->led_repeat == 0)
        {
            appFindmeState->led_alert_active = 0; // led alert has beed done.
            return;
        }

        wiced_start_timer(&findme_led_timer, appAlert_cfg.alertLedCfg[pattern_id].led_off_ms);
    }
    else
    {
        bleremoteapp_alertLedOn();
        wiced_start_timer(&findme_led_timer, appAlert_cfg.alertLedCfg[pattern_id].led_on_ms);
    }
}


////////////////////////////////////////////////////////////////////////////////
///  This function is to turn on LED
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertLedOn(void)
{
    //configure LED on
    wiced_hal_gpio_set_pin_output(GPIO_PORT_LED, LED_ON);
    appFindmeState->led_on = 1;
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to turn off LED
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertLedOff(void)
{
    //configure LED off
    wiced_hal_gpio_set_pin_output(GPIO_PORT_LED, LED_OFF);
    appFindmeState->led_on = 0;
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to start the LED play based on alert level
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertLedPlay(uint8_t pattern_id)
{
    if ((appAlert_cfg.alertLedCfg[pattern_id].repeat_num == 0) ||
        (appAlert_cfg.alertLedCfg[pattern_id].led_on_ms == 0) ||
        (pattern_id >= APP_ALERT_PATTERN_MAX_ID)) // invalid parameter.
        return;

    bleremoteapp_alertLedStop();

    appFindmeState->led_pattern_id = pattern_id;
    appFindmeState->led_repeat = appAlert_cfg.alertLedCfg[pattern_id].repeat_num;

    bleremoteapp_alertLedOn();

    //bleremoteapp_StartAlertLedTimer(appAlert_cfg.alertLedCfg[pattern_id].led_on_ms);
    wiced_start_timer(&findme_led_timer, appAlert_cfg.alertLedCfg[pattern_id].led_on_ms);

    appFindmeState->led_alert_active = 1; // led alert is activated.

}

////////////////////////////////////////////////////////////////////////////////
///  This function is the handling of NO ALERT (LED)
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_alertLedStop(void)
{
    if (appFindmeState->led_alert_active)
    {
        wiced_stop_timer(&findme_led_timer);

        //configure LED off
        wiced_hal_gpio_set_pin_output(GPIO_PORT_LED, LED_OFF);
        appFindmeState->led_alert_active = 0; // led alert is deactivated.
    }
}

////////////////////////////////////////////////////////////////////////////////
///  This function is the callback function for the find me attribute handle.
/// It controls the LED behavior depending on the value of the write command
////////////////////////////////////////////////////////////////////////////////
int bleremoteapp_findme_writeCb(void *p)
{
    wiced_bt_gatt_write_t *p_data = (wiced_bt_gatt_write_t *)p;

    if ((HANDLE_BLEREMOTE_IMMEDIATE_ALERT_SERVICE_CHAR_LEVEL_VAL == p_data->handle) && (p_data->val_len == 1)) // alert level len is 1byte
    {

        appFindmeState->activeAlterLevel = *(p_data->p_val);

        switch(appFindmeState->activeAlterLevel)
        {
            case NO_ALERT:
                /* Action to Stop Alert */
                /*
                bleremoteapp_alertBuzStop();
                bleremoteapp_alertLedStop();
                */
            break;

            case MILD_ALERT:
                /* Action for Mild Alert */
                /*
                if (appFindmeState->alertType & ALERT_BUZ)
                {
                    bleremoteapp_alertBuzPlay(APP_ALERT_PATTERN_MILD_ID);
                }
                if (appFindmeState->alertType & ALERT_LED)
                {
                    bleremoteapp_alertLedPlay(APP_ALERT_PATTERN_MILD_ID);
                }
                */
            break;

            case HIGH_ALERT:
                /* Action for HIGH Alert */
                /*
                if (appFindmeState->alertType & ALERT_BUZ)
                {
                    bleremoteapp_alertBuzPlay(APP_ALERT_PATTERN_HIGH_ID);
                }
                if (appFindmeState->alertType & ALERT_LED)
                {
                    bleremoteapp_alertLedPlay(APP_ALERT_PATTERN_HIGH_ID);
                }
                */
            break;
        }

    }

    return 0;
}
#endif  /* SUPPORTING_FINDME */

#ifdef SUPPORT_TOUCHPAD
////////////////////////////////////////////////////////////////////////////////
/// Callback to handle GPIO activity interrupt due to peripherals, e.g. touchpad
////////////////////////////////////////////////////////////////////////////////
void gpioActivityDetected(void *appData, uint8_t portPin)
{
    bleremoteapp_appActivityDetected(appData);
}

////////////////////////////////////////////////////////////////////////////////
/// Poll touchpad activity
////////////////////////////////////////////////////////////////////////////////
uint8_t pollTouchpadActivity(void)
{
    // check for touuchpad
    if (touchpad->getPinActive())
    {
        uint8_t dataAvailable;
        dataAvailable = touchpad->pollActivity(&bleRemoteAppState->touchpadEvent, FALSE);

        // ignore touchpad events?
        if (!wiced_ble_hidd_link_is_discoverable())
        {
            if (!wiced_ble_hidd_host_info_is_bonded())
            {
                if (!touchpad->proximityRpt())      // Don't use proximity finger event to initiate discovery
                {
                    wiced_ble_hidd_link_connect();
                }
            }
            else if ( dataAvailable )
            {
                if (dataAvailable != HID_EVENT_TP_INFO)
                {
                    if (!audioIsActive() && !motionIsEnabled())
                    {
                        //WICED_BT_TRACE("\npollTouchpadActivity: dataAvailable=%d, prt=0x%x\n", dataAvailable, bleRemoteAppState->touchpadEvent.userDataPtr);
                        wiced_hidd_event_queue_add_event_with_overflow(&bleRemoteAppState->appEventQueue,
                              &bleRemoteAppState->touchpadEvent.eventInfo,  sizeof(bleRemoteAppState->touchpadEvent),
                              bleRemoteAppState->pollSeqn);

                        return BLEHIDLINK_ACTIVITY_REPORTABLE;
                    }
                }
                return (wiced_ble_hidd_link_is_connected() && !touchpad->proximityRpt())?
                    BLEHIDLINK_ACTIVITY_NONE : BLEHIDLINK_ACTIVITY_REPORTABLE;
            }
        }
    }
    return BLEHIDLINK_ACTIVITY_NONE;
}

////////////////////////////////////////////////////////////////////////////////
/// Handle virtual key generation upon pressing key embedded under tocuhpad
/// \param ke: On input the key event corresponding to touchpad key
///            On output updated with the virtual key index based on zone touched.
/// \return True if a virtual key pressed, i.e. a zone touched that maps to a key.
////////////////////////////////////////////////////////////////////////////////
uint8_t handleTouchpadVirtualKey(HidEventKey * ke)
{
    static uint8_t zone= ZONE_UNDEFINED;
    uint8_t isDown = ke->keyEvent.upDownFlag == KEY_DOWN;
    if (isDown)
    {
        //WICED_BT_TRACE("\ngettin zone\n", zone);
        zone = touchpad->getZone();
    }
    // we translate the key if valid zone found
    if (zone == ZONE_UNDEFINED)
    {
        WICED_BT_TRACE("%s no zone\n", isDown?"DN":"UP");
        return FALSE;
    }
    ke->keyEvent.keyCode = zone + VIRUAL_KEY_INDEX_BASE;
    WICED_BT_TRACE("%s zone=%d\n", isDown?"DN":"UP", zone);
    return TRUE;
}

////////////////////////////////////////////////////////////////////////////////
/// Process TP event
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_procEvtTouchpad(void)
{
    HidEventUserDefine *tp_event = (HidEventUserDefine *)wiced_hidd_event_queue_get_current_element(&bleRemoteAppState->appEventQueue);
    AbsXYReport *tp = (AbsXYReport* )tp_event->userDataPtr;

    //set gatt attribute value here before sending the report
    memset(bleremote_touchpad_rpt, 0, TOUCHPAD_REPORT_DATA_SIZE(tp));
    memcpy(bleremote_touchpad_rpt, TOUCHPAD_REPORT_DATA(tp), TOUCHPAD_REPORT_DATA_SIZE(tp));

    wiced_ble_hidd_link_send_report(tp->reportID, WICED_HID_REPORT_TYPE_INPUT, bleremote_touchpad_rpt, TOUCHPAD_REPORT_DATA_SIZE(tp));
}
#endif /* SUPPORT_TOUCHPAD */

#ifdef OTA_FIRMWARE_UPGRADE
////////////////////////////////////////////////////////////////////////////////
/// Process OTA firmware upgrade status change event
////////////////////////////////////////////////////////////////////////////////
void bleremoteapp_ota_fw_upgrade_status(uint8_t status)
{
    WICED_BT_TRACE("OTAFU status:%d\n", status);

    switch (status)
    {
    case OTA_FW_UPGRADE_STATUS_STARTED:             // Client started OTA firmware upgrade process
        WICED_BT_TRACE("allow slave latency 0\n");
        wiced_blehidd_allow_slave_latency(FALSE);
        break;

    case OTA_FW_UPGRADE_STATUS_ABORTED:             // Aborted or failed verification */
#ifdef SUPPORT_MOTION
        if (!motionIsEnabled())
#endif
#ifdef SUPPORT_AUDIO
            if (!audioIsActive())
#endif
            {
                WICED_BT_TRACE("allow slave latency 1\n");
                wiced_blehidd_allow_slave_latency(TRUE);
            }
        break;

    case OTA_FW_UPGRADE_STATUS_COMPLETED:           // firmware upgrade completed, will reboot
        // ToDo should probably clean disconnect so that host does not wait for link super before start scanning
        break;
    }
}
#endif /* OTA_FIRMWARE_UPGRADE */
