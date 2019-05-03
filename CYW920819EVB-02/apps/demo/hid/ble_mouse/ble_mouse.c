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
* The BLE Mouse application is a single chip SoC compliant with HID over GATT Profile (HOGP).
*
* During initialization the app registers with LE stack, WICED HID Device Library and
* keyscan HW to receive various notifications including bonding complete, connection
* status change, peer GATT request/commands and interrupts for button pressed/released.
* Press any button will start LE advertising. When device is successfully bonded, the app
* saves bonded host's information in the NVRAM.
* When user presses/releases button, a HID report will be sent to the host.
* On connection up or battery level changed, a battery report will be sent to the host.
* When battery level is bellowed shutdown voltage, device will critical shutdown.
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
* 1. Plug the CYW920819EVB_02 board or 20819A1 mouse HW into your computer
* 2. Put on jumper to bypass Serial Flash (i.e. jumper on J5 in CYW920819EVB_02 board), then power up the board or mouse HW.
* 3. Remove the jumper so that download procedure bellowed can write to Serial Flash.
* 4. Build and download the application (to the EVAL board or the mouse HW)
* 5. If download failed due to not able to detecting device, just repeat step 4 again.
* 6. Unplug the EVAL board or the mouse HW from your computer (i.e. unplug the UART cable)
* 7. power cycle the EVAL board or the mouse HW.
* 8. Press any button to start LE advertising, then pair with a PC or Tablet
*    If using the CYW920819EVB board, use a fly wire to connect GPIO P0 and P8 to simulate LEFT button press,
*     and remove the wire to simulate button release.
* 9. Once connected, it becomes the mouse of the PC or Tablet.
*
*
* In case what you have is only the WICED EVAL board, you can only use fly wire to connect to GPIOs (GPIO P0 and P8) to simulate mouse button press and release.
* Or using the ClientControl tool in the tools to simulate button press and mouse movement.
* 1. Plug the WICED EVAL board into your computer
* 2. Build and download the application (to the WICED board)
* 3. If failed to download due to device not detected, just repeat step 2 again.
* 4. Press any button to start LE advertising, then pair with a PC or Tablet
*     Use a fly wire to connect GPIO P0 and P8 to simulate LEFT button press,
*     and remove the wire to simulate button release.
* 5. Once connected, it becomes the mouse of the PC. However, you can't see scroll or mouse cursor movement
*     since you don't have the real HW.

* You can also use the WICED board and ClientControl tool test the basic BLE functions.
* NOTE: Make sure you use "TESTING_USING_HCI=1" in application settings.
* In ModusToolbox, select right click on app and select 'Change Application Settings'
*
* 1~3. same download procedure as above
* 4. Run ClientControl.exe.
* 5. Choose 3M as Baudrate and select the serial port in ClientControl tool window.
* 6. Press Reset button on the board and open the port.
* 7. Press "Enter Pairing Mode"or "Connect" to start LE advertising, then pair with a PC or Tablet
* 8. Once connected, it becomes the mouse of the PC or Tablet.
*  - Select Interrupt channel, Input report, enter the contents of the report
*    and click on the Send button, to send the report.  For example:
*    To send button down event when LEFT button is pushed, report should be
*    02 01 00 00 00 00.  All buttons up 02 00 00 00 00 00.
*    To send scroll up event, report should be
*    02 00 00 00 00 ff.
*    To send scroll down event, report should be
*    02 00 00 00 00 01.
*    To send mouse cursor (Y+8) event, report can be
*    02 00 00 80 ff 00.
*    To send mouse cursor (Y-8) event, report can be
*    02 00 00 80 00 00.
*    To send mouse cursor (X+8) event, report can be
*    02 00 08 00 00 00.
*    To send mouse cursor (X-8) event, report can be
*    02 00 f8 0f 00 00 .
*    Please make sure you always send a button up report following button down report.
*/

#include "spar_utils.h"
#include "gki_target.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_mia.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_keyscan.h"
#include "wiced_hal_keyscan_button.h"
#include "wiced_hal_batmon.h"
#include "wiced_hal_adc.h"
#include "wiced_hal_quadrature.h"
#ifdef LED_USE_PWM
#include "wiced_hal_aclk.h"
#include "wiced_hal_pwm.h"
#endif

#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_sdp.h"
#ifdef OTA_FIRMWARE_UPGRADE
#include "wiced_bt_ota_firmware_upgrade.h"
#endif

#include "wiced_timer.h"
#include "wiced_memory.h"

#include "ble_mouse_gatts.h"
#include "ble_mouse.h"
#ifdef SUPPORT_MOTION
#include "motion/PAW3805_opticalsensor.h"
#endif

//////////////////////////////////////////////////////////////////////////////
//                      local interface declaration
//////////////////////////////////////////////////////////////////////////////
extern MouseAppConfig blemouseAppConfig;
//extern QuadratureConfig quadratureConfig;
extern uint16_t blehostlist_flags;
extern wiced_bool_t blehidlink_connection_param_updated;

tMouseAppState  ble_mouse_application_state = {0, };
tMouseAppState *mouseAppState = &ble_mouse_application_state;

uint16_t  characteristic_client_configuration[MAX_NUM_CLIENT_CONFIG_NOTIF] = {0,};
uint8_t   mouseapp_protocol = PROTOCOL_REPORT;
uint8_t   battery_level = 100;

uint8_t blemouse_input_rpt[MOUSE_REPORT_SIZE] = {0, };       //map to (&(kbAppState->kbapp_stdRpt.modifierKeys))[kbAppState->kbapp_stdRptSize]
uint8_t blemouse_connection_ctrl_rpt = 0;


extern wiced_bt_cfg_settings_t wiced_bt_hid_cfg_settings;

uint8_t firstTransportStateChangeNotification = 1;
wiced_timer_t blemouse_allow_sleep_timer;
wiced_timer_t blemouse_conn_param_update_timer;

#ifdef SUPPORT_MOTION
wiced_bool_t motionOn = WICED_FALSE;

#define MOTION_ON   (motionOn && !wiced_hidd_is_transport_detected())
#endif

wiced_blehidd_report_gatt_characteristic_t mreportModeGattMap[] =
{
    //Mouse REPORT Input report
    {MOUSE_REPORT_ID   ,WICED_HID_REPORT_TYPE_INPUT, HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_STD_INPUT_VAL,FALSE,NULL,MOUSEAPP_CLIENT_CONFIG_NOTIF_STD_RPT},
    // Battery Input report
    {BATTERY_REPORT_ID ,WICED_HID_REPORT_TYPE_INPUT, HANDLE_BLEMOUSE_BATTERY_SERVICE_CHAR_LEVEL_VAL          ,FALSE,NULL,MOUSEAPP_CLIENT_CONFIG_NOTIF_BATTERY_RPT},

    //Set Client characteristic configuration of mouse Battery REPORT Input report
    {0xFF,  WICED_HID_CLIENT_CHAR_CONF    , HANDLE_BLEMOUSE_BATTERY_SERVICE_CHAR_CFG_DESCR       ,FALSE,mouseapp_clientConfWriteBatteryRpt,MOUSEAPP_CLIENT_CONFIG_NOTIF_NONE},
    //Set Client characteristic configuration of mouse REPORT Input report
    {0xFF,  WICED_HID_CLIENT_CHAR_CONF    , HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_STD_INPUT_CHAR_CFG_DESCR,FALSE,mouseapp_clientConfWriteRptStd,MOUSEAPP_CLIENT_CONFIG_NOTIF_NONE},
    //Set Protocol Mode of HID
    {0xFF,WICED_HID_REPORT_TYPE_OTHER     , HANDLE_BLEMOUSE_LE_HID_SERVICE_PROTO_MODE_VAL         ,FALSE,mouseapp_setProtocol,MOUSEAPP_CLIENT_CONFIG_NOTIF_NONE},
    // Set HID control point
    {0xFF,WICED_HID_REPORT_TYPE_OTHER     , HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_HID_CTRL_POINT_VAL,FALSE,mouseapp_ctrlPointWrite,MOUSEAPP_CLIENT_CONFIG_NOTIF_NONE},


    //Boot mouse input client conf write
    {0xFF, WICED_HID_CLIENT_CHAR_CONF, HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_BT_MOUSE_INPUT_CHAR_CFG_DESCR,   FALSE, mouseapp_clientConfWriteBootMode,    MOUSEAPP_CLIENT_CONFIG_NOTIF_NONE},
};

wiced_blehidd_report_gatt_characteristic_t mbootModeGattMap[] =
{
     // Boot mouse Input report
    {MOUSE_REPORT_ID ,WICED_HID_REPORT_TYPE_INPUT ,HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_BT_MOUSE_INPUT_VAL,FALSE,NULL,MOUSEAPP_CLIENT_CONFIG_NOTIF_BOOT_RPT},
    // Boot mouse client conf write
    {0xFF     ,WICED_HID_CLIENT_CHAR_CONF,HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_BT_MOUSE_INPUT_CHAR_CFG_DESCR,FALSE,mouseapp_clientConfWriteBootMode,MOUSEAPP_CLIENT_CONFIG_NOTIF_NONE},
    //Set Protocol Mode of HID
    {0xFF     ,WICED_HID_REPORT_TYPE_OTHER   ,HANDLE_BLEMOUSE_LE_HID_SERVICE_PROTO_MODE_VAL,FALSE,mouseapp_setProtocol,MOUSEAPP_CLIENT_CONFIG_NOTIF_NONE},
};

/////////////////////////////////////////////////////////////////////////////////////////////
/// set up LE Advertising data
/////////////////////////////////////////////////////////////////////////////////////////////
void mouseapp_setUpAdvData(void)
{
    wiced_bt_ble_advert_elem_t mouseapp_adv_elem[4];
    uint8_t mouseapp_adv_flag = BTM_BLE_LIMITED_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint16_t mouseapp_adv_appearance = APPEARANCE_HID_MOUSE;
    uint16_t mouseapp_adv_service = UUID_SERVCLASS_LE_HID;

    // flag
    mouseapp_adv_elem[0].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    mouseapp_adv_elem[0].len          = sizeof(uint8_t);
    mouseapp_adv_elem[0].p_data       = &mouseapp_adv_flag;

    // Appearance
    mouseapp_adv_elem[1].advert_type  = BTM_BLE_ADVERT_TYPE_APPEARANCE;
    mouseapp_adv_elem[1].len          = sizeof(uint16_t);
    mouseapp_adv_elem[1].p_data       = (uint8_t *)&mouseapp_adv_appearance;

    //16 bits Service: UUID_SERVCLASS_LE_HID
    mouseapp_adv_elem[2].advert_type  = BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE;
    mouseapp_adv_elem[2].len          = sizeof(uint16_t);
    mouseapp_adv_elem[2].p_data       = (uint8_t *)&mouseapp_adv_service;

    //dev name
    mouseapp_adv_elem[3].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    mouseapp_adv_elem[3].len          = strlen(dev_local_name);
    mouseapp_adv_elem[3].p_data       = (uint8_t *)dev_local_name;

    wiced_bt_ble_set_raw_advertisement_data(4,  mouseapp_adv_elem);
}


////////////////////////////////////////////////////////////////////////////////
/// This function is the timeout handler for allow_sleep_timer
////////////////////////////////////////////////////////////////////////////////
void mouseapp_allowsleep_timeout( uint32_t arg )
{
    WICED_BT_TRACE("allow SDS\n");

#ifdef SUPPORT_MOTION
    if (MOTION_ON && !wiced_ble_hidd_link_is_connected())
    {
        //flush motion data so that MOTION interrupt will be inactive
        PAW3805_flushMotion();
    }
#endif
    mouseAppState->allowSDS = WICED_TRUE;
}

////////////////////////////////////////////////////////////////////////////////
/// This function is the timeout handler for conn_param_update_timer
////////////////////////////////////////////////////////////////////////////////
void mouseapp_connparamupdate_timeout( uint32_t arg )
{
#ifdef SUPPORT_MOTION
    if (MOTION_ON)
    {
        PAW3805_enable_deep_sleep_mode();
    }
#endif

    //request connection param update if it not requested before
    if (!blehidlink_connection_param_updated)
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
void blemouseapp_create(void)
{
    uint16_t button_report_bits[] = {0x0001,  //1st button 'LEFT'. report bitmap 0x0001
                                     0x0004,  //2nd button 'MID".report bitmap 0x0004
                                     0x0002,  //3rd button 'RIGHT'. report bitmap 0x0002
                                     0x8000}; //4th button 'PAIRING'. map it to the last button bit.

    WICED_BT_TRACE("mouseCreate\n");

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

    //button click: p0+P8:LEFT, p0+P9:MID, p0+P10:RIGHT, p0+P11:PAIRING
    wiced_hal_keyscan_button_configure(4, WICED_FALSE, WICED_FALSE, button_report_bits);
    //button driver init
    wiced_hal_keyscan_button_init();

#ifdef SUPPORT_SCROLL
    //configure P29 as QOC3, P6 as qdz0, P7 as qdz1 based on HW schematics in referenced design.
    wiced_hal_quadrature_configure(QUADRATURE_LED_CONFIG_SOURCE |            //QOC_LEDs_output_polarity for QOC0 bit[1:0]
                                   QUADRATURE_LED_CONFIG_SOURCE << 2 |       //QOC_LEDs_output_polarity for QOC2 bit[3:2]
                                   QUADRATURE_LED_CONFIG_SOURCE << 4 |       //QOC_LEDs_output_polarity for QOC2 bit[5:4]
                                   QUADRATURE_LED_CONFIG_SOURCE << 6,        //QOC_LEDs_output_polarity for QOC3 bit[7:6]
                                   GPIO_PULL_DOWN | GPIO_EN_INT_RISING_EDGE, //quadratureInputGpioConfig
                                   ENABLE_PORT_0_PINS_AS_QUAD_INPUT,         //port0PinsUsedAsQuadratureInput
                                   WICED_FALSE,                              //configureP26AsQOC0
                                   WICED_FALSE,                              //configureP27AsQOC1
                                   WICED_FALSE,                              //configureP28AsQOC2
                                   WICED_TRUE,                               //configureP29AsQOC3
                                   CH_Z_ENABLE | CH_XY_DISABLE | CH_Z_SAMPLE_ONCE_PER_LHL_PWM,   //channelEnableAndSamplingRate
                                   WICED_FALSE,                              //pollXAxis
                                   WICED_FALSE,                              //pollYAxis
                                   WICED_TRUE);                              //pollZAxis
    //quadrature driver init
    wiced_hal_quadrature_init();
#endif
#ifdef SUPPORT_MOTION
    //SPIFFY1 can not co-exist with UART at the same time in 20819A1
    if (!wiced_hidd_is_transport_detected())
    {
        motionOn = WICED_TRUE;
        PAW3805_init(mouseapp_userMotionXYDetected, NULL);
    }
    else
    {
        WICED_BT_TRACE("UART detected !!! disable MOTION!\n");
    }
#endif


    //initialize event queue
    wiced_hidd_event_queue_init(&mouseAppState->mouseappEventQueue, (uint8_t *)wiced_memory_permanent_allocate(blemouseAppConfig.maxEventNum * blemouseAppConfig.maxEventSize),
                    blemouseAppConfig.maxEventSize, blemouseAppConfig.maxEventNum);

    mouseapp_init();

    WICED_BT_TRACE("Free RAM bytes=%d bytes\n", wiced_memory_get_free_bytes());
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// This function will be called from blemouseapp_create() during start up.
/////////////////////////////////////////////////////////////////////////////////////////////
void mouseapp_init(void)
{
    wiced_ble_hidd_link_set_preferred_conn_params(wiced_bt_hid_cfg_settings.ble_scan_cfg.conn_min_interval,        // 8*1.25=10ms
                                        wiced_bt_hid_cfg_settings.ble_scan_cfg.conn_max_interval,        // 8*1.25=10ms
                                        wiced_bt_hid_cfg_settings.ble_scan_cfg.conn_latency,             //  49. i.e. 500ms slave latency
                                        wiced_bt_hid_cfg_settings.ble_scan_cfg.conn_supervision_timeout);//600 * 10=600ms=6 seconds

    mouseapp_setUpAdvData();

    //timer to allow Shut Down Sleep (SDS)
    wiced_init_timer( &blemouse_allow_sleep_timer, mouseapp_allowsleep_timeout, 0, WICED_MILLI_SECONDS_TIMER );

    //timer to request connection param update
    wiced_init_timer( &blemouse_conn_param_update_timer, mouseapp_connparamupdate_timeout, 0, WICED_MILLI_SECONDS_TIMER );

    // Clear motion/scroll normal and fractional counts
    mouseAppState->mouseapp_xMotion = mouseAppState->mouseapp_yMotion = mouseAppState->mouseapp_scroll =
        mouseAppState->mouseapp_xFractional = mouseAppState->mouseapp_yFractional = mouseAppState->mouseapp_scrollFractional = 0;

    // Clear tick counts used for clearing fractional motion
    mouseAppState->mouseapp_pollsSinceXYMotion = mouseAppState->mouseapp_pollsSinceScroll = 0;

    // Initally no reportable data is present
    mouseAppState->mouseapp_reportableDataInReportSet = FALSE;

    // Initialize the boot mode report
    memset(&mouseAppState->mouseapp_bootModeReport, 0, sizeof(mouseAppState->mouseapp_bootModeReport));
    mouseAppState->mouseapp_bootModeReport.reportID = MOUSE_REPORT_ID;

    // Initialize report mode report. Set ID from config to allow
    // apps to select their own
    memset(&mouseAppState->mouseapp_reportModeReport, 0, sizeof(mouseAppState->mouseapp_reportModeReport));
    mouseAppState->mouseapp_reportModeReport.reportID = blemouseAppConfig.motionReportID;

    // Initialize button event
    mouseAppState->mouseapp_buttonEvent.eventInfo.eventType = HID_EVENT_NEW_BUTTON_STATE;
    mouseAppState->mouseapp_buttonEvent.buttonState = 0;
    mouseAppState->mouseapp_scrollEvent.eventInfo.eventType = HID_EVENT_MOTION_AXIS_0;

    // Determine the size of the Battery. Report ID will not be sent.
    mouseAppState->mouseapp_batRpt.reportID = BATTERY_REPORT_ID;
    mouseAppState->mouseapp_batRptSize = sizeof(mouseAppState->mouseapp_batRpt.level);

    // Register for Interrupt ( Buttons)
    wiced_hal_keyscan_register_for_event_notification(mouseapp_userKeyPressDetected, NULL);

#ifdef SUPPORT_SCROLL
    // Register for quadrature Interrupt
    wiced_hal_quadrature_register_for_event_notification(mouseapp_userScrollDetected, NULL);
#endif

    //add battery observer
    wiced_hal_batmon_add_battery_observer(mouseapp_batLevelChangeNotification);

    //register App low battery shut down handler
    wiced_hal_batmon_register_low_battery_shutdown_cb(mouseapp_shutdown);

    wiced_ble_hidd_link_add_state_observer(mouseapp_stateChangeNotification);

    wiced_ble_hidd_link_register_poll_callback(mouseapp_pollReportUserActivity);

    wiced_ble_hidd_link_register_sleep_permit_handler(mouseapp_sleep_handler);

    if(mouseapp_protocol == PROTOCOL_REPORT)
    {
        // If the current protocol is report, register the report mode table
        wiced_blehidd_register_report_table(mreportModeGattMap, sizeof(mreportModeGattMap)/sizeof(mreportModeGattMap[0]));
    }
    else
    {
        //otherwise, register the boot mode table
        wiced_blehidd_register_report_table(mbootModeGattMap, sizeof(mbootModeGattMap)/sizeof(mbootModeGattMap[0]));
    }

#ifdef SUPPORT_MOTION
    if ( MOTION_ON && wiced_ble_hidd_host_info_is_bonded())
    {
        WICED_BT_TRACE("MOTION INRT enabled!\n");
        PAW3805_enable_Interrupt(WICED_TRUE);
    }
#endif
    wiced_ble_hidd_link_init();

    wiced_hal_mia_enable_mia_interrupt(TRUE);
    wiced_hal_mia_enable_lhl_interrupt(TRUE);
}

////////////////////////////////////////////////////////////////////////////////
/// This function is called when battery voltage drops below the configured threshold.
////////////////////////////////////////////////////////////////////////////////
void mouseapp_shutdown(void)
{
    WICED_BT_TRACE("mouseapp_shutdown\n");

    mouseapp_flushUserInput();

#ifdef SUPPORT_SCROLL
    // Disable the quadrature HW
    wiced_hal_quadrature_turnOff();
#endif

#ifdef SUPPORT_MOTION
    if (motionOn)
    {
        //flush motion data
        PAW3805_flushMotion();
        //power down
        PAW3805_powerdown();
    }
#endif

    // Disable button detection
    wiced_hal_keyscan_turnOff();

    if(wiced_ble_hidd_link_is_connected())
    {
        wiced_ble_hidd_link_disconnect();
    }

    // Disable Interrupts
    wiced_hal_mia_enable_mia_interrupt(FALSE);
    wiced_hal_mia_enable_lhl_interrupt(FALSE);

}

////////////////////////////////////////////////////////////////////////////////
/// This function will poll user activities and send reports
////////////////////////////////////////////////////////////////////////////////
void mouseapp_pollReportUserActivity(void)
{
    uint8_t activitiesDetectedInLastPoll;

    mouseAppState->mouseapp_pollSeqn++;

    if((mouseAppState->mouseapp_pollSeqn % 64) == 0)
    {
        WICED_BT_TRACE(".");
    }

    activitiesDetectedInLastPoll = mouseapp_pollActivityUser();

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
                mouseapp_generateAndTxReports();
            }
        }
        else
        {
            mouseapp_generateAndTxReports();
        }

#ifdef OTA_FIRMWARE_UPGRADE
        if (!wiced_ota_fw_upgrade_is_active())
#endif
        {
            // Poll the battery monitor
            wiced_hal_batmon_poll_monitor();
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
///  This function provides an implementation for HidApp::pollUserActivity()
///  It polls the following sources for user activity:
///        - Motion
///        - Scroll Wheel
///        - Buttons
///  Note that buttons are polled last to minimize latency.
/// Any detected activity is queued as events in the event fifo.
///
/// \return
///   Bit mapped value indicating
///       - ACTIVITY_REPORTABLE - if any event is queued
///       - ACTIVITY_NON_REPORTABLE - if any button (excluding connect button) is down
///       - ACTIVITY_NONE otherwise
/////////////////////////////////////////////////////////////////////////////////
uint8_t mouseapp_pollActivityUser(void)
{
    // Poll the hardware for events
    wiced_hal_mia_pollHardware();

#ifdef SUPPORT_MOTION
    if (MOTION_ON)
    {
        // Check for XY motion
        mouseapp_pollActivityXYSensor();
    }
#endif
#ifdef SUPPORT_SCROLL
    // Check for scroll
    mouseapp_pollActivityScroll();
#endif
    // Check for buttons state change
    mouseapp_pollActivityButton();

    // Return value indicating whether any button is down.
    return (wiced_hidd_event_queue_get_num_elements(&mouseAppState->mouseappEventQueue) ? BLEHIDLINK_ACTIVITY_REPORTABLE : BLEHIDLINK_ACTIVITY_NONE);
}

/////////////////////////////////////////////////////////////////////////////////
/// This function polls the scroll interface to get any newly detected
/// scroll count. It negates the data and performs any scaling if configured to do so.
/// If configured to do so, it discards any fractional value after the configured
/// number of polls. If any non-fractional scroll activity is accumulated,
/// it queues a scroll event.
/////////////////////////////////////////////////////////////////////////////////
void mouseapp_pollActivityScroll(void)
{
    int16_t scrollCurrent;

    // Check for scroll
    if ((scrollCurrent = wiced_hal_quadrature_get_scroll_count()))
    {//WICED_BT_TRACE("scroll count:%d\n", scrollCurrent);
        // Negate scroll value if enabled
        if (blemouseAppConfig.negateScroll)
        {
            scrollCurrent = -scrollCurrent;
        }

        // Check if scroll scaling is enabled
        if (blemouseAppConfig.scrollScale)
        {
            // Yes. Add the current scroll count to the fractional count
            mouseAppState->mouseapp_scrollFractional += scrollCurrent;

            // Scale and adjust accumulated scroll value. Fractional value will be
            // left in the factional part. Place the whole number in the scroll
            // event
            mouseAppState->mouseapp_scrollEvent.motion =
                mouseapp_scaleValue(&mouseAppState->mouseapp_scrollFractional, blemouseAppConfig.scrollScale);

            // Reset the scroll discard counter
            mouseAppState->mouseapp_pollsSinceScroll = 0;
        }
        else
        {
            // No scaling is required. Put the data in the scroll event
            mouseAppState->mouseapp_scrollEvent.motion = scrollCurrent;
        }

        // Queue scroll event with the proper seqn
        wiced_hidd_event_queue_add_event_with_overflow(&mouseAppState->mouseappEventQueue,
                                          &mouseAppState->mouseapp_scrollEvent.eventInfo, sizeof(mouseAppState->mouseapp_scrollEvent), mouseAppState->mouseapp_pollSeqn);
    }
    else
    {
        // If scroll scaling timeout is not infinite, bump up the
        // inactivity counter and check if we have crossed the threshold.
        if (blemouseAppConfig.pollsToKeepFracScrollData &&
            ++mouseAppState->mouseapp_pollsSinceScroll >= blemouseAppConfig.pollsToKeepFracScrollData)
        {
            // We have. Discard any fractional scroll data
            mouseAppState->mouseapp_scrollFractional = 0;

            // Reset the scroll discard counter
            mouseAppState->mouseapp_pollsSinceScroll = 0;
        }
    }
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
int16_t mouseapp_scaleValue(int16_t *val, uint8_t scaleFactor)
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
void mouseapp_connectButtonHandler(ConnectButtonPosition connectButtonPosition)
{
    // The connect button was not pressed. Check if it is now pressed
    if (connectButtonPosition == CONNECT_BUTTON_DOWN)
    {
        WICED_BT_TRACE("Connect Btn Pressed\n");
        mouseapp_connectButtonPressed();
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
void mouseapp_connectButtonPressed(void)
{
    wiced_ble_hidd_link_virtual_cable_unplug();
}

/////////////////////////////////////////////////////////////////////////////////
/// This function provides an implementation for HidApp::generateAndTxReports.
/// If the number of packets in the hardware
/// fifo is less than the report generation threshold and the event queue is not empty,
/// this function will
///     - Check if there is any reportable data left over from the previous report set
///       If not, it will generate a new report set by calling mouseAppGenerateReportSet_BR
///     - It will attempt to transmit the current report set by calling
///       mouseAppTransmitReportSet_BR
/////////////////////////////////////////////////////////////////////////////////
void mouseapp_generateAndTxReports(void)
{

    // Check if the active transport has room and there are unprocessed events in the queue,
    while ((wiced_bt_buffer_poolutilization (HCI_ACL_POOL_ID) < 80) && wiced_hidd_event_queue_get_num_elements(&mouseAppState->mouseappEventQueue))
    {
        // If there is no reportable data left in the current report set, generate a new report set
        if (!mouseAppState->mouseapp_reportableDataInReportSet)
        {
            // Process events necessary to generate a report set
            mouseapp_generateReportSet();
        }

        // Now transmit the report set
        mouseapp_txReportSet();
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the battery report over the interrupt channel
/////////////////////////////////////////////////////////////////////////////////
void mouseapp_batRptSend(void)
{
    //set gatt attribute value here before sending the report
    battery_level = mouseAppState->mouseapp_batRpt.level[0];

    if ( WICED_SUCCESS == wiced_ble_hidd_link_send_report(mouseAppState->mouseapp_batRpt.reportID,WICED_HID_REPORT_TYPE_INPUT,
                                        mouseAppState->mouseapp_batRpt.level,mouseAppState->mouseapp_batRptSize))
    {
        wiced_hal_batmon_set_battery_report_sent_flag(WICED_TRUE);
    }
}

////////////////////////////////////////////////////////////////////////////////
///  This is called whenever a Mouse activity is detected.
///  ~ Planar Motion Activity
///  ~ Scroll Activity
///  ~ Button Activity
/////////////////////////////////////////////////////////////////////////////////
void mouseapp_userKeyPressDetected(void *MApp)
{
    //WICED_BT_TRACE("kbapp_userKeyPressDetected\n");
    mouseAppState->keyInterrupt_On = wiced_hal_keyscan_is_any_key_pressed();

    // Poll the app.
    mouseapp_pollReportUserActivity();
}


// Scroll/Quadrature interrupt
void mouseapp_userScrollDetected(void* unused)
{
    //WICED_BT_TRACE("mouseapp_userScrollDetected\n");
    //Poll the app.
    mouseapp_pollReportUserActivity();
}

/////////////////////////////////////////////////////////////////////////////////
/// This function informs the application that the state of a link changed.
/// Note that it is expected that the application will do mainly link agnostic
/// activities in this method
///
/// \param newState new state of the link
/////////////////////////////////////////////////////////////////////////////////
void mouseapp_stateChangeNotification(uint32_t newState)
{
    int16_t flags;
    WICED_BT_TRACE("Transport state changed to %d\n", newState);

    mouseAppState->allowSDS = FALSE;

    //stop allow SDS (shut down sleep) timer
    wiced_stop_timer(&blemouse_allow_sleep_timer);

    if(newState == BLEHIDLINK_CONNECTED)
    {
        //get host client configuration characteristic descriptor values
        flags = wiced_ble_hidd_host_info_get_flags(ble_hidd_link.gatts_peer_addr, ble_hidd_link.gatts_peer_addr_type);
        if(flags != -1)
        {
            WICED_BT_TRACE("host config flag:%08x\n",flags);
            mouseapp_updateGattMapWithNotifications(flags);
        }

        //enable application polling
        wiced_ble_hidd_link_enable_poll_callback(WICED_TRUE);

#ifdef SUPPORT_MOTION
        if (MOTION_ON)
        {
            //enable motion interrupt
            PAW3805_enable_Interrupt(WICED_TRUE);
        }
#endif

        if(firstTransportStateChangeNotification)
        {
            //Wake up from shutdown sleep (SDS) and already have a connection then allow SDS in 1 second
            //This will allow time to send a mouse report
            wiced_start_timer(&blemouse_allow_sleep_timer,1000); // 1 second. timeout in ms
        }
        else
        {
            //We connected after power on reset
            //Start 20 second timer to allow time to setup connection encryption before allowing shutdown sleep (SDS).
            wiced_start_timer(&blemouse_allow_sleep_timer,20000); //20 seconds. timeout in ms

            //start 15 second timer to make sure connection param update is requested before SDS
            wiced_start_timer(&blemouse_conn_param_update_timer,15000); //15 seconds. timeout in ms
        }
    }
    else if(newState == BLEHIDLINK_DISCONNECTED)
    {
        //allow Shut Down Sleep (SDS) only if we are not attempting reconnect
        if (!wiced_is_timer_in_use(&ble_hidd_link.reconnect_timer))
        {
            wiced_start_timer(&blemouse_allow_sleep_timer,2000); // 2 seconds. timeout in ms
        }

        wiced_hal_mia_enable_mia_interrupt(TRUE);
        // Tell the transport to stop polling
        wiced_ble_hidd_link_enable_poll_callback(WICED_FALSE);
    }
    else if ((newState == BLEHIDLINK_ADVERTISING_IN_uBCS_DIRECTED) || (newState == BLEHIDLINK_ADVERTISING_IN_uBCS_UNDIRECTED))
    {
        mouseAppState->allowSDS = TRUE;
    }

    if(firstTransportStateChangeNotification)
    {
        firstTransportStateChangeNotification = 0;
    }
}

void mouseapp_batLevelChangeNotification(uint32_t newLevel)
{
    //WICED_BT_TRACE("bat level changed to %d\n", newLevel);

    if (mouseapp_protocol == PROTOCOL_REPORT)
    {
        mouseAppState->mouseapp_batRpt.level[0] = newLevel;
        mouseapp_batRptSend();

        //we do not want to save battery level value to NVRAM (i.e. SFLASH). too many writes, damage SFLASH lifetime.
        //blehostlist_SetClientBatLevelAtTop(newLevel);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function clears all dynamic reports defined by the mouse application
/////////////////////////////////////////////////////////////////////////////////
void mouseapp_clearAllReports(void)
{
//do nothing
}

////////////////////////////////////////////////////////////////////////////////
/// This function flushes all queued events and unprocessed fractional scroll
/// activity. It also clears all reports.
////////////////////////////////////////////////////////////////////////////////
void mouseapp_flushUserInput(void)
{
    // Clear motion/scroll normal and fractional counts
    mouseAppState->mouseapp_xMotion = mouseAppState->mouseapp_yMotion
            = mouseAppState->mouseapp_scroll = mouseAppState->mouseapp_xFractional
            = mouseAppState->mouseapp_yFractional = mouseAppState->mouseapp_scrollFractional = 0;

    // Clear tick counts used for clearing fractional motion
    mouseAppState->mouseapp_pollsSinceXYMotion = mouseAppState->mouseapp_pollsSinceScroll = 0;

    mouseAppState->mouseapp_reportableDataInReportSet = FALSE;

    mouseAppState->mouseapp_bootModeReport.buttonState =
        mouseAppState->mouseapp_reportModeReport.buttonState = mouseAppState->mouseapp_buttonEvent.buttonState=0;

    // Flush the event fifo since we have no idea what made it in and what didn't
    wiced_hidd_event_queue_flush(&mouseAppState->mouseappEventQueue);

    // Clear all dynamic reports
    mouseapp_clearAllReports();
}

void mouseapp_clientConfWriteRptStd(wiced_hidd_report_type_t reportType,
                                 uint8_t reportId,
                                 void *payload,
                                 uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t  indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

    WICED_BT_TRACE("mouseapp_clientConfWriteRptStd\n");

    mouseapp_updateClientConfFlags(notification, MOUSEAPP_CLIENT_CONFIG_NOTIF_STD_RPT);
}

/////////////////////////////////////////////////////////////////////////////////
/// This function polls the XY sensor to get any newly detected XY count.
/// It negates the data and performs any scaling if configured to do so.
/// If configured to do so, it discards any fractional value after the configured
/// number of polls. If any non-fractional XY activity is accumulated,
/// it queues an XY motion event.
/////////////////////////////////////////////////////////////////////////////////
void mouseapp_pollActivityXYSensor(void)
{
    int16_t xCurrent = 0, yCurrent = 0, tmp = 0;
#ifdef SUPPORT_MOTION
    // Check for XY motion
    PAW3805_getMotion(&xCurrent,&yCurrent);
#endif
    // Check if any motion was detected
    if (xCurrent || yCurrent)
    {
        // Swap XY if enabled
        if (blemouseAppConfig.swapXY)
        {
            // Use global temporary for swapping
            tmp = xCurrent;
            xCurrent = yCurrent;
            yCurrent = tmp;
        }

        // Negate X if enabled
        if (blemouseAppConfig.negateX)
        {
            xCurrent = -xCurrent;
        }

        // Negate Y if enabled
        if (blemouseAppConfig.negateY)
        {
            yCurrent = -yCurrent;
        }

        mouseAppState->mouseapp_xyMotionEvent[mouseAppState->motion_fifo_in++].eventInfo.eventType = HID_EVENT_MOTION_AXIS_X_Y;
        if (mouseAppState->motion_fifo_in == MOTION_FIFO_CNT)
        {
            mouseAppState->motion_fifo_in = 0;
        }

        // Check if XY scaling is enabled
        if (blemouseAppConfig.xScale | blemouseAppConfig.yScale)
        {
            // Yes. Add the new motion to the accumulated motion
            mouseAppState->mouseapp_xFractional += xCurrent;
            mouseAppState->mouseapp_yFractional += yCurrent;

            // Scale and adjust accumulated X motion value. Fractional value will be
            // left in the factional part. Place the whole number in the XY motion event.
            mouseAppState->mouseapp_xyMotionEvent[mouseAppState->motion_fifo_in].motionX = mouseapp_scaleValue(&mouseAppState->mouseapp_xFractional, blemouseAppConfig.xScale);

            // Scale and adjust accumulated Y motion value. Fractional value will be
            // left in the factional part. Place the whole number in the XY motion event.
            mouseAppState->mouseapp_xyMotionEvent[mouseAppState->motion_fifo_in].motionY = mouseapp_scaleValue(&mouseAppState->mouseapp_yFractional, blemouseAppConfig.yScale);

            // Reset the XY discard counter
            mouseAppState->mouseapp_pollsSinceXYMotion = 0;
        }
        else
        {
            // Scaling is not enabled. Place the detected XY motion directly in the xy event
            mouseAppState->mouseapp_xyMotionEvent[mouseAppState->motion_fifo_in].motionX = xCurrent;
            mouseAppState->mouseapp_xyMotionEvent[mouseAppState->motion_fifo_in].motionY = yCurrent;
        }

        // Queue the event with the proper seqn
        wiced_hidd_event_queue_add_event_with_overflow(&mouseAppState->mouseappEventQueue, &mouseAppState->mouseapp_xyMotionEvent[mouseAppState->motion_fifo_in].eventInfo, sizeof(HidEventMotionXY), mouseAppState->mouseapp_pollSeqn);
    }
    else
    {
        // If XY scaling timeout is not infinite, bump up the
        // inactivity counter and check if we have crossed the threshold.
        if (blemouseAppConfig.pollsToKeepFracXYData &&
            ++mouseAppState->mouseapp_pollsSinceXYMotion >= blemouseAppConfig.pollsToKeepFracXYData)
        {
            // We have. Discard any fractional XY data
            mouseAppState->mouseapp_xFractional = mouseAppState->mouseapp_yFractional = 0;

            // Reset the XY poll counter
            mouseAppState->mouseapp_pollsSinceXYMotion = 0;
        }
    }
}

#ifdef SUPPORT_MOTION
// Motion interrupt
void mouseapp_userMotionXYDetected(void* unused, uint8_t port)
{
    if (MOTION_ON)
    {
        mouseapp_pollReportUserActivity();
    }
    else
    {WICED_BT_TRACE("MOTION INRT disabled!\n");
        PAW3805_enable_Interrupt(WICED_FALSE);
    }
}
#endif

/////////////////////////////////////////////////////////////////////////////////
/// This function processes queued events to complete data collection for one
/// report set. A report set is defined as:
///     - If event combining is disabled, it consists of all events collected in
///       a single poll cycle.
///     - If report combining is enabled it consists of either
///         - all events collected in a single poll
///         - XY motion events spread across multiple polls
///         - scroll events spread across multiple polls
///     - Note that button changes across multiple polls are never combined and
///       neither are scroll/XY events from seperate polls mixed together.
/////////////////////////////////////////////////////////////////////////////////
void mouseapp_generateReportSet(void)
{
    HidEvent *event;
    uint8_t combiningEventType;

    // Initialize combining flag depending on whether combining is allowed or not
    combiningEventType = (blemouseAppConfig.eventCombining ? HID_EVENT_ANY : HID_EVENT_NONE);

    // Get pointer to the first event. We know it is valid
    event = (HidEvent *)wiced_hidd_event_queue_get_current_element(&mouseAppState->mouseappEventQueue);

    // Continue processing events as long as:
    // - we have events and
    // - we are combining and the new event type is acceptable for combining
    do
    {
        // Processing depends on the event type
        switch (event->eventType)
        {
            case HID_EVENT_MOTION_AXIS_0:
                // This is a scroll event. Add to scroll count and set the reportable data flag
                // if the scroll count is not zero
                if ((mouseAppState->mouseapp_scroll += ((HidEventMotionSingleAxis *)event)->motion))
                {
                    mouseAppState->mouseapp_reportableDataInReportSet = TRUE;
                }

                // We can only continue combining scroll events. If any other event types have
                // been processed, we must disable any further combining.
                if (combiningEventType == HID_EVENT_ANY || combiningEventType == HID_EVENT_MOTION_AXIS_0)
                {
                    combiningEventType = HID_EVENT_MOTION_AXIS_0;
                }
                else
                {
                    // Disable combining for all other cases
                    combiningEventType = HID_EVENT_NONE;
                }
                break;
            case HID_EVENT_MOTION_AXIS_X_Y:
#ifdef SUPPORT_MOTION
                //NOTE: We'd like to disable motion interrupt and rely on app polling to stream motion XY movement.
                //Otherwise, too many motion interrupt will cause unnecessary handling and make the motion XY movement unsmooth.
                //disable motion INTERRUPT
                PAW3805_enable_Interrupt(WICED_FALSE);
#endif
                // Update XY counts
                mouseAppState->mouseapp_xMotion += ((HidEventMotionXY *)event)->motionX;
                mouseAppState->mouseapp_yMotion += ((HidEventMotionXY *)event)->motionY;

                // Set the reportable data flag if we have either X or Y motion
                if (mouseAppState->mouseapp_xMotion || mouseAppState->mouseapp_yMotion)
                {
                    mouseAppState->mouseapp_reportableDataInReportSet = TRUE;
                }

                // We can only continue combining XY events. If any other event types have
                // been processed, we must disable further combining.
                if (combiningEventType == HID_EVENT_ANY || combiningEventType == HID_EVENT_MOTION_AXIS_X_Y)
                {
                    combiningEventType = HID_EVENT_MOTION_AXIS_X_Y;
                }
                else
                {
                    // Disable combining for all other cases
                    combiningEventType = HID_EVENT_NONE;
                }
                break;

            case HID_EVENT_NEW_BUTTON_STATE:
                // Update button state in the reports, both boot and report
                mouseAppState->mouseapp_bootModeReport.buttonState = mouseAppState->mouseapp_reportModeReport.buttonState =
                    ((HidEventButtonStateChange *)event)->buttonState;

                // Disable combining since we can't combine reports with different button states
                combiningEventType = HID_EVENT_NONE;

                // Button state change is always reportable
                mouseAppState->mouseapp_reportableDataInReportSet = TRUE;

                break;
            case HID_EVENT_EVENT_FIFO_OVERFLOW:
                WICED_BT_TRACE("event queue overflow. FLUSH!!!\n");
                // Flush the event fifo since we have no idea what made it in and what didn't
                wiced_hidd_event_queue_flush(&mouseAppState->mouseappEventQueue);

                // Grab the current button state from the button event and stuff it in the current reports
                // Note that the button event always has the current button state.
                mouseAppState->mouseapp_bootModeReport.buttonState =
                    mouseAppState->mouseapp_reportModeReport.buttonState = mouseAppState->mouseapp_buttonEvent.buttonState;

                // Force report generation at this point to get the latest button state out
                mouseAppState->mouseapp_reportableDataInReportSet = TRUE;
                break;

            default:
                break;
        }

        // We are done with the current event. Remove it from the queue
       wiced_hidd_event_queue_remove_current_element(&mouseAppState->mouseappEventQueue);

        // Get pointer to the next event.
        event = (HidEvent *)wiced_hidd_event_queue_get_current_element(&mouseAppState->mouseappEventQueue);
    }while (event && (event->eventType == combiningEventType || combiningEventType == HID_EVENT_ANY));
}

int16_t mouseapp_limitRange(int16_t input, int16_t limit)
{
    // Check if input exceeds the positive threshold
    if (input > limit)
    {
        // Return limit value
        return limit;
    }
    // Next check the lower bound
    else if (input < (int16_t)-limit)
    {
        // Return minimum value
        return -limit;
    }
    else
    {
        // Value is in range. Return as is
        return input;
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// Create and transmit boot mode report
/////////////////////////////////////////////////////////////////////////////////
void mouseapp_createAndTxBootModeReport(void)
{
    int16_t tmp;

    // The report ID is already initialized

    // Button state is already filled in

    // We scale X/Y values by 2 in boot mode to simplify hanlding of large X/Y offsets
    // Assuming 12.5 ms sniff, this allows us to send 20,000 counts/sec
    // Note that scaling may lose us one point in the end. We'll live with this limitation
    // for boot mode operation

    // Fill in X motion sacled by 2 and subtract from current count.
    tmp = mouseapp_limitRange(mouseAppState->mouseapp_xMotion, MAX_BOOT_X_Y);
    mouseAppState->mouseapp_bootModeReport.xMotion = (tmp >> 1);
    mouseAppState->mouseapp_xMotion -= tmp;

    // Fill in Y motion scaled by 2 and subtract from current count
    tmp = mouseapp_limitRange(mouseAppState->mouseapp_yMotion, MAX_BOOT_X_Y);
    mouseAppState->mouseapp_bootModeReport.yMotion = (tmp >> 1);
    mouseAppState->mouseapp_yMotion -= tmp;

    // No scroll(wheel) in boot mode
    mouseAppState->mouseapp_scroll -= mouseAppState->mouseapp_bootModeReport.scroll;

    // Transfer the report over the active transport. We know we have space for at
    // least one report
    WICED_BT_TRACE("Boot Rpt\n");

    //set gatt attribute value here before sending the report
    memcpy(blemouse_input_rpt, &mouseAppState->mouseapp_bootModeReport.buttonState, blemouseAppConfig.motionReportBootModeSize);

    wiced_ble_hidd_link_send_report(mouseAppState->mouseapp_bootModeReport.reportID,WICED_HID_REPORT_TYPE_INPUT,
                        &mouseAppState->mouseapp_bootModeReport.buttonState, blemouseAppConfig.motionReportBootModeSize);
}

/////////////////////////////////////////////////////////////////////////////////
/// Create and transmit report mode report. This includes 12 bit X/Y.
/////////////////////////////////////////////////////////////////////////////////
void mouseapp_createAndTxReportModeReport(void)
{
    int16_t tmp;

    // The report ID is already initialized

    // Button state is already filled in

    // Fill in X motion and subtract from current count.
    tmp = mouseapp_limitRange(mouseAppState->mouseapp_xMotion, MAX_REPORT_X_Y);
    mouseAppState->mouseapp_reportModeReport.xMotionLow = tmp;
    mouseAppState->mouseapp_reportModeReport.xMotionHigh = (tmp >> 8);
    mouseAppState->mouseapp_xMotion -= tmp;

    // Fill in Y motion and subtract from current count
    tmp = mouseapp_limitRange(mouseAppState->mouseapp_yMotion, MAX_REPORT_X_Y);
    mouseAppState->mouseapp_reportModeReport.yMotionLow = tmp;
    mouseAppState->mouseapp_reportModeReport.yMotionHigh = (tmp >> 4);;
    mouseAppState->mouseapp_yMotion -= tmp;

    // Fill in scroll and subtract from current count
    mouseAppState->mouseapp_reportModeReport.scroll = mouseapp_limitRange(mouseAppState->mouseapp_scroll, MAX_SCROLL);
    mouseAppState->mouseapp_scroll -= mouseAppState->mouseapp_reportModeReport.scroll;

    //set gatt attribute value here before sending the report
    memcpy(blemouse_input_rpt, &mouseAppState->mouseapp_reportModeReport.buttonState, MOUSE_REPORT_SIZE);

    // Transfer the report over the active transport. We know we have space for at
    // least one report
    wiced_ble_hidd_link_send_report(mouseAppState->mouseapp_reportModeReport.reportID,WICED_HID_REPORT_TYPE_INPUT,
                        &mouseAppState->mouseapp_reportModeReport.buttonState,blemouseAppConfig.motionReportReportModeSize);
}

/////////////////////////////////////////////////////////////////////////////////
/// Create and transmit report
/////////////////////////////////////////////////////////////////////////////////
void mouseapp_createAndTxReport(void)
{
    // Generate report based on report/boot mode
    if (mouseapp_protocol == PROTOCOL_REPORT)
    {
        mouseapp_createAndTxReportModeReport();
    }
    else
    {
        mouseapp_createAndTxBootModeReport();
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function queues a report set for transmission. A report is generated
/// and transmitted as long as we have reportable data in the current report set
/// (determined using the reportableDataInReportSet flag) and the active transport
/// is willing to accept more data.
/// After transmitting one report, the reportable data in report set flag
/// is recalculated by checking for any unreported motions or scroll data. Since
/// button data doesn't require multiple reports to send, it is not used in the
/// recalculation.
/////////////////////////////////////////////////////////////////////////////////
void mouseapp_txReportSet(void)
{
    // Now transmit the report set as long as the active transport is willing
    // to accept data and we have reportable data in the report set
    while (mouseAppState->mouseapp_reportableDataInReportSet)
    {
        // Create and transmit one report
        mouseapp_createAndTxReport();

        // Check if the report set still has reportable data. Note that
        // only motion data counts here since we have already reported button data.
        mouseAppState->mouseapp_reportableDataInReportSet = (mouseAppState->mouseapp_xMotion ||
                                                             mouseAppState->mouseapp_yMotion ||
                                                             mouseAppState->mouseapp_scroll);
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function polls buttons. First current button status is retrieved.
/// Then connect button is processed.
/// Then the button state (excluding connect buttons) is compared with the previous
/// state to determine if the state of reported buttons has changed. If the button
/// state has changed a button event is queued into the event fifo.
/////////////////////////////////////////////////////////////////////////////////
void mouseapp_pollActivityButton(void)
{
    uint16_t newButtonStatus;
    static ConnectButtonPosition prevConnectButtonPosition = CONNECT_BUTTON_UP;

    do
    {
        // Get the button status
        newButtonStatus =  wiced_hal_keyscan_button_get_current_state();

        if (newButtonStatus & blemouseAppConfig.connectButtonMask)
        {
            prevConnectButtonPosition = CONNECT_BUTTON_DOWN;
        }
        else
        {
            if (prevConnectButtonPosition == CONNECT_BUTTON_DOWN)
            {
               wiced_hal_keyscan_flush_HW_events();
               prevConnectButtonPosition = CONNECT_BUTTON_UP;
            }
        }

        // Extract connect button state and update hid application
        mouseapp_connectButtonHandler((newButtonStatus & blemouseAppConfig.connectButtonMask)?
                         CONNECT_BUTTON_DOWN: CONNECT_BUTTON_UP);

        // Eliminate connect button state
        newButtonStatus &= ~blemouseAppConfig.connectButtonMask;

        // Check if button status has changed
        if (newButtonStatus != mouseAppState->mouseapp_buttonEvent.buttonState)
        {
            // Queue button event with the current button state
            mouseAppState->mouseapp_buttonEvent.buttonState = newButtonStatus;
            wiced_hidd_event_queue_add_event_with_overflow(&mouseAppState->mouseappEventQueue, &mouseAppState->mouseapp_buttonEvent.eventInfo, sizeof(mouseAppState->mouseapp_buttonEvent), mouseAppState->mouseapp_pollSeqn);
        }
    }
    while (wiced_hal_keyscan_events_pending());

}


void mouseapp_ctrlPointWrite(wiced_hidd_report_type_t reportType,
                          uint8_t reportId,
                          void *payload,
                          uint16_t payloadSize)
{
    WICED_BT_TRACE("mouseapp_ctrlPointWrite\n");

    wiced_ble_hidd_link_disconnect();
}

//
// Write to Client Characteristic Configuration of Boot Mouse Input Report
//
void mouseapp_clientConfWriteBootMode(wiced_hidd_report_type_t reportType,
                                   uint8_t reportId,
                                   void *payload,
                                   uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t  indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

    WICED_BT_TRACE("mouseapp_clientConfWriteBootMode\n");

    mouseapp_updateClientConfFlags(notification, MOUSEAPP_CLIENT_CONFIG_NOTIF_BOOT_RPT);
}

void mouseapp_clientConfWriteBatteryRpt(wiced_hidd_report_type_t reportType,
                                     uint8_t reportId,
                                     void *payload,
                                     uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t  indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

    WICED_BT_TRACE("mouseapp_clientConfWriteBatteryRpt\n");

    mouseapp_updateClientConfFlags(notification, MOUSEAPP_CLIENT_CONFIG_NOTIF_BATTERY_RPT);
}

void mouseapp_setProtocol(wiced_hidd_report_type_t reportType,
                          uint8_t reportId,
                          void *payload,
                          uint16_t payloadSize)
{
    mouseapp_protocol = *((uint8_t*)payload);

    if(mouseapp_protocol == PROTOCOL_REPORT)
    {
        // If the current protocol is report, register the report mode table
        wiced_blehidd_register_report_table(mreportModeGattMap, sizeof(mreportModeGattMap)/sizeof(mreportModeGattMap[0]));
    }
    else
    {
        //otherwise, register the boot mode table
        wiced_blehidd_register_report_table(mbootModeGattMap, sizeof(mbootModeGattMap)/sizeof(mbootModeGattMap[0]));
    }
}

void mouseapp_updateClientConfFlags(uint16_t enable, uint16_t featureBit)
{
    mouseapp_updateGattMapWithNotifications(wiced_ble_hidd_host_info_update_flags(ble_hidd_link.gatts_peer_addr, ble_hidd_link.gatts_peer_addr_type, enable,featureBit));
}

void mouseapp_updateGattMapWithNotifications(uint16_t flags)
{
    int i = 0;

    wiced_blehidd_report_gatt_characteristic_t* map = mbootModeGattMap;

    blehostlist_flags = flags;

    // Set the boot mode report first
    for(i = 0; i < sizeof(mbootModeGattMap)/sizeof(mbootModeGattMap[0]); i++)
    {
        if(map->reportType == WICED_HID_REPORT_TYPE_INPUT &&
            map->clientConfigBitmap == MOUSEAPP_CLIENT_CONFIG_NOTIF_BOOT_RPT)
        {
            // If this is the boot mode input report we are looking for,
            // set/clear based on the new flags.
            map->sendNotification =
                ((flags & MOUSEAPP_CLIENT_CONFIG_NOTIF_BOOT_RPT) == MOUSEAPP_CLIENT_CONFIG_NOTIF_BOOT_RPT) ? TRUE : FALSE;

            break;
        }
        map++;
    }

    // not update the report mode map
    map = mreportModeGattMap;
    for(i = 0; i < sizeof(mreportModeGattMap)/sizeof(mreportModeGattMap[0]); i++)
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
uint32_t mouseapp_sleep_handler(wiced_sleep_poll_type_t type )
{
    uint32_t ret = WICED_SLEEP_NOT_ALLOWED;

#ifndef  TESTING_USING_HCI
    // Check if keys are pressed before deciding whether sleep is allowed
    mouseAppState->keyInterrupt_On = wiced_hal_keyscan_is_any_key_pressed();
#endif

    switch(type)
    {
        case WICED_SLEEP_POLL_TIME_TO_SLEEP:
            ret = WICED_SLEEP_MAX_TIME_TO_SLEEP;
            break;

        case WICED_SLEEP_POLL_SLEEP_PERMISSION:
            ret = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;

            if (!mouseAppState->allowSDS)
            {
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
            }

            //if key is not released, no Shut Down Sleep (SDS)
            if (mouseAppState->keyInterrupt_On)
            {
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
            }

#ifdef SUPPORT_MOTION
            if (MOTION_ON && PAW3805_isActive())
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
            break;

    }

    return ret;
}

////////////////////////////////////////////////////////////////////////////////////
/// restore contents from Always On Memory. This should be called when wake up from SDS
///////////////////////////////////////////////////////////////////////////////////
void mouseapp_aon_restore(void)
{
    if (!wiced_hal_mia_is_reset_reason_por())
    {
        wiced_ble_hidd_link_aon_action_handler(BLEHIDLINK_RESTORE_FROM_AON);
    }
}

#ifdef OTA_FIRMWARE_UPGRADE
////////////////////////////////////////////////////////////////////////////////
/// Process OTA firmware upgrade status change event
////////////////////////////////////////////////////////////////////////////////
void blemouseapp_ota_fw_upgrade_status(uint8_t status)
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
        if (!MOTION_ON)
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
