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
 * Runtime Bluetooth stack and remote control configuration parameters
 *
 */
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_uuid.h"
#include "appDefs.h"
#include "ble_remote.h"
#include "ble_remote_gatts.h"
#include "wiced_hidd_micaudio.h"

#ifdef OTA_FIRMWARE_UPGRADE
#include "wiced_bt_ota_firmware_upgrade.h"
#endif


const char  dev_local_name[]             = "CY BLE REMOTE";
const uint8_t dev_appearance_name[2]     = {BIT16_TO_8(APPEARANCE_GENERIC_HID_DEVICE)};
const uint8_t dev_pnp_id[]               ={0x01, 0x31, 0x01, 0xB4, 0x04, 0x01, 0x00}; //BT SIG, cypress semiconductor, 0x04B4, 0x0001
const char dev_char_mfr_name_value[]     = "Cypress Semiconductor";
const char dev_char_fw_version_value[3]        ={'1','2','3'};

const uint8_t   rpt_ref_battery[]             = {BATTERY_REPORT_ID ,0x01};
const uint8_t   rpt_ref_std_key_input[]       = {STD_KB_REPORT_ID,0x01};
const uint8_t   rpt_ref_std_key_output[]      = {STD_KB_REPORT_ID,0x02};
const uint8_t   rpt_ref_bitmap[]              = {BITMAPPED_REPORT_ID,0x01};
const uint8_t   rpt_ref_motion[]              = {MOTION_REPORT_ID,0x01};
const uint8_t   rpt_ref_user_defined_0[]      = {0x0A,0x01};
const uint8_t   rpt_ref_voice[]               = {WICED_HIDD_VOICE_REPORT_ID, 0x01};
const uint8_t   rpt_ref_voice_ctrl_input[]    = {0xF8, 0x01};
const uint8_t   rpt_ref_voice_ctrl_feature[]  = {0xF8, 0x03}; //feature rpt, OK to change to output report. But need chang in host BSA.
const uint8_t   rpt_ref_connection_ctrl[]     = {0xCC,0x03}; //feature rpt
#ifdef SUPPORT_TOUCHPAD
const uint8_t   rpt_ref_touchpad[]            = {RPT_ID_IN_ABS_XY, 0x01};
#endif

const uint8_t dev_hid_information[] = {0x00, 0x01, 0x00, 0x00};      // Verison 1.00, Not localized, Cannot remote wake, not normally connectable
const uint16_t dev_battery_service_uuid = UUID_CHARACTERISTIC_BATTERY_LEVEL;

extern uint16_t characteristic_client_configuration[];
extern uint8_t battery_level;

extern uint8_t bleremote_output_rpt;
extern uint8_t bleremote_connection_ctrl_rpt;

extern uint8_t bleremote_key_std_rpt[];
extern uint8_t bleremote_bitmap_rpt[];
#ifdef SUPPORT_MOTION
extern uint8_t bleremote_motion_rpt[];
#endif

extern uint8_t bleremote_user_defined_0_rpt[];
extern uint8_t bleremote_voice_rpt[];
extern uint8_t bleremote_voice_ctrl_input_rpt[];
extern uint8_t bleremote_voice_ctrl_feature_rpt[];
#ifdef SUPPORT_TOUCHPAD
extern uint8_t bleremote_touchpad_rpt[];
#endif



/*****************************************************************************
 * This is the GATT database for the BLE HID Remote application.  It defines
 * services, characteristics and descriptors supported by the sensor.  Each
 * attribute in the database has a handle, (characteristic has two, one for
 * characteristic itself, another for the value).  The handles are used by
 * the peer to access attributes, and can be used locally by application for
 * example to retrieve data written by the peer.  Definition of characteristics
 * and descriptors has GATT Properties (read, write, notify...) but also has
 * permissions which identify if application is allowed to read or write
 * into it.  Handles do not need to be sequential, but need to be in order.
 ****************************************************************************/
const uint8_t blehid_db_data[]=
{
    // Declare gatt service
    PRIMARY_SERVICE_UUID16
        ( HANDLE_BLEREMOTE_GATT_SERVICE, UUID_SERVCLASS_GATT_SERVER ),

   // Declare GAP service. Device Name and Appearance are mandatory
    // characteristics of GAP service
    PRIMARY_SERVICE_UUID16
        ( HANDLE_BLEREMOTE_GAP_SERVICE, UUID_SERVCLASS_GAP_SERVER ),

    // Declare mandatory GAP service characteristic: Dev Name
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEREMOTE_GAP_SERVICE_CHAR_DEV_NAME,
        HANDLE_BLEREMOTE_GAP_SERVICE_CHAR_DEV_NAME_VAL,
        GATT_UUID_GAP_DEVICE_NAME,
        LEGATTDB_CHAR_PROP_READ,
        LEGATTDB_PERM_READABLE
    ),

    // Declare mandatory GAP service characteristic: Appearance
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEREMOTE_GAP_SERVICE_CHAR_DEV_APPEARANCE,
        HANDLE_BLEREMOTE_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
        GATT_UUID_GAP_ICON,
        LEGATTDB_CHAR_PROP_READ,
        LEGATTDB_PERM_READABLE
    ),

    // Declare GAP service characteristic: Peripheral Prefered Connection Parameter
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEREMOTE_GAP_SERVICE_CHAR_PERI_PREFER_CONNPARAM,
        HANDLE_BLEREMOTE_GAP_SERVICE_CHAR_PERI_PREFER_CONNPARAM_VAL,
        GATT_UUID_GAP_PREF_CONN_PARAM,
        LEGATTDB_CHAR_PROP_READ,
        LEGATTDB_PERM_READABLE
    ),

    // Declare Device info service
    PRIMARY_SERVICE_UUID16
        ( HANDLE_BLEREMOTE_DEV_INFO_SERVICE, UUID_SERVCLASS_DEVICE_INFO ),

    // Handle 0x29: characteristic PnP ID, handle 0x2A characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEREMOTE_DEV_INFO_SERVICE_CHAR_PNP_ID,
        HANDLE_BLEREMOTE_DEV_INFO_SERVICE_CHAR_PNP_ID_VAL,
        GATT_UUID_PNP_ID,
        LEGATTDB_CHAR_PROP_READ,
        LEGATTDB_PERM_READABLE
    ),

    // Handle 0x2B: characteristic Manufacturer Name, handle 0x2C characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEREMOTE_DEV_INFO_SERVICE_CHAR_MFR_NAME,
        HANDLE_BLEREMOTE_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
        GATT_UUID_MANU_NAME,
        LEGATTDB_CHAR_PROP_READ,
        LEGATTDB_PERM_READABLE
    ),

    // Handle 0x2D: characteristic Firmware Revision String, handle 0x2E characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEREMOTE_DEV_INFO_SERVICE_CHAR_FW_VER,
        HANDLE_BLEREMOTE_DEV_INFO_SERVICE_CHAR_FW_VER_VAL,
        GATT_UUID_FW_VERSION_STR,
        LEGATTDB_CHAR_PROP_READ,
        LEGATTDB_PERM_READABLE
    ),

   // Declare Battery service
   PRIMARY_SERVICE_UUID16
        ( HANDLE_BLEREMOTE_BATTERY_SERVICE, UUID_SERVCLASS_BATTERY),

   // Handle 0x31: characteristic Battery Level, handle 0x32 characteristic value
   CHARACTERISTIC_UUID16
    (
        HANDLE_BLEREMOTE_BATTERY_SERVICE_CHAR_LEVEL,       // attribute handle
        HANDLE_BLEREMOTE_BATTERY_SERVICE_CHAR_LEVEL_VAL, // attribute value handle
        GATT_UUID_BATTERY_LEVEL,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_NOTIFY,
        LEGATTDB_PERM_READABLE
    ),

    // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_BLEREMOTE_BATTERY_SERVICE_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
    ),

    // Handle 0x34: report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEREMOTE_BATTERY_SERVICE_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),


    // Declare Scan Parameters service
    PRIMARY_SERVICE_UUID16
        ( HANDLE_BLEREMOTE_SCAN_PARAM_SERVICE, UUID_SERVCLASS_SCAN_PARAM),

    // Handle 0x41: characteristic Battery Level, handle 0x42 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE
    (
        HANDLE_BLEREMOTE_SCAN_PARAM_SERVICE_CHAR_SCAN_INT_WINDOW,
        HANDLE_BLEREMOTE_SCAN_PARAM_SERVICE_CHAR_SCAN_INT_WINDOW_VAL,
        GATT_UUID_SCAN_INT_WINDOW,
        LEGATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
        LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ
    ),

    // Declare HID over LE
    PRIMARY_SERVICE_UUID16
        ( HANDLE_BLEREMOTE_LE_HID_SERVICE, UUID_SERVCLASS_LE_HID),


    // Include BSA SERVICE
    INCLUDE_SERVICE_UUID16
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_INC_BAS_SERVICE,
        HANDLE_BLEREMOTE_BATTERY_SERVICE,
        HANDLE_BLEREMOTE_BATTERY_SERVICE_RPT_REF_DESCR,
        UUID_SERVCLASS_BATTERY
    ),

    // HID control point
    // Handle 0x51: characteristic HID Report, handle 0x52 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_HID_CTRL_POINT,
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_HID_CTRL_POINT_VAL,
        GATT_UUID_HID_CONTROL_POINT,
        LEGATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
        LEGATTDB_PERM_WRITE_CMD
    ),

    // Handle 0x53: characteristic HID information, handle 0x54 characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_INFO,
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_INFO_VAL,
        GATT_UUID_HID_INFORMATION,
        LEGATTDB_CHAR_PROP_READ,
        LEGATTDB_PERM_READABLE
    ),


    // Handle 0x55: characteristic HID Report MAP, handle 0x56 characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_MAP,
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_MAP_VAL,
        GATT_UUID_HID_REPORT_MAP,
        LEGATTDB_CHAR_PROP_READ,
        LEGATTDB_PERM_READABLE
    ),


   // include Battery Service
   // Handle 0x57: external report reference
   CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_EXT_RPT_REF_DESCR,
        GATT_UUID_EXT_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),


    // STD Input report
    // Handle 0x5D: characteristic HID Report, handle 0x5E characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_STD_INPUT,
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_STD_INPUT_VAL,
        GATT_UUID_HID_REPORT,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_NOTIFY,
        LEGATTDB_PERM_READABLE
    ),

   // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
   CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_STD_INPUT_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
    ),

   // Handle 0x60: report reference
   CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_STD_INPUT_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),

    // STD Output report
    // Handle 0x61: characteristic HID Report, handle 0x62 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_STD_OUTPUT,
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_STD_OUTPUT_VAL,
        GATT_UUID_HID_REPORT,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_WRITE_NO_RESPONSE|LEGATTDB_CHAR_PROP_WRITE,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
    ),

   // Handle 0x63: report reference
   CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_STD_OUTPUT_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),

    // Bit mapped report, Report ID=2
    // Handle 0x64: characteristic HID Report, handle 0x65 characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_BITMAP,
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_BITMAP_VAL,
        GATT_UUID_HID_REPORT,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_NOTIFY,
        LEGATTDB_PERM_READABLE
    ),

   // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
   CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_BITMAP_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
    ),

   // Handle 0x67: report reference
   CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_BITMAP_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),

#ifdef SUPPORT_MOTION
    // motion report
    // Handle 0x68: characteristic HID Report, handle 0x69 characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_MOTION,
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_MOTION_VAL,
        GATT_UUID_HID_REPORT,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_NOTIFY,
        LEGATTDB_PERM_READABLE
    ),

   // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
   CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_MOTION_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
    ),

   // Handle 0x6B: report reference
   CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_MOTION_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),
#endif
    // user defined 0 report
    // Handle 0x6C: characteristic HID Report, handle 0x6D characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0,
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0_VAL,
        GATT_UUID_HID_REPORT,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_NOTIFY,
        LEGATTDB_PERM_READABLE
    ),

    // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
    ),

    // Handle 0x6F: report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),

#ifdef SUPPORT_AUDIO
    //Voice report
    // Handle 0x70: characteristic HID Report, handle 0x71 characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE,
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_VAL,
        GATT_UUID_HID_REPORT,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_NOTIFY,
        LEGATTDB_PERM_READABLE
    ),

   // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
   CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
    ),

   // Handle 0x73: report reference
   CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),

     //Voice Ctl INPUT report (remote->host)
     // Handle 0x74: characteristic HID Report, handle 0x75 characteristic value
     CHARACTERISTIC_UUID16
     (
         HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT,
         HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT_VAL,
         GATT_UUID_HID_REPORT,
         LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_NOTIFY,
         LEGATTDB_PERM_READABLE
     ),

    // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
         HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT_CHAR_CFG_DESCR,
         GATT_UUID_CHAR_CLIENT_CONFIG,
         LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
     ),

     // Handle 0x77: report reference
     CHAR_DESCRIPTOR_UUID16
     (
         HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT_RPT_REF_DESCR,
         GATT_UUID_RPT_REF_DESCR,
         LEGATTDB_PERM_READABLE
     ),

     //Voice Ctl FEATURE report (host->remote)
     // Handle 0x78: characteristic HID Report, handle 0x79 characteristic value
     CHARACTERISTIC_UUID16_WRITABLE
     (
         HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_FEA,
         HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_FEA_VAL,
         GATT_UUID_HID_REPORT,
         LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_WRITE,
         LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
     ),

     // Handle 0x7A: report reference
     CHAR_DESCRIPTOR_UUID16
     (
         HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_FEA_RPT_REF_DESCR,
         GATT_UUID_RPT_REF_DESCR,
         LEGATTDB_PERM_READABLE
     ),
#endif

#ifdef SUPPORT_TOUCHPAD
    //touchpad report
    // Handle 0x7B: characteristic HID Report, handle 0x7C characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_TOUCHPAD,     //0x7B(CHARACTERISTIC handle)
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_TOUCHPAD_VAL, //0x7C(CHARACTERISTIC value handle)
        GATT_UUID_HID_REPORT,                                 //0x2A4D (Report)
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_NOTIFY,    //0x12 (Read|Notify)
        LEGATTDB_PERM_READABLE                                //permission
    ),

    // Declare client specific characteristic cfg desc.
    // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection
    // for bonded devices
    // Handle 0x7D: client specific characteristic cfg desc
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_TOUCHPAD_CHAR_CFG_DESCR,       //0x7D (client CHARACTERISTIC cfg desc handle)
        GATT_UUID_CHAR_CLIENT_CONFIG,                                          //0x2902 (Client CHARACTERISTIC configuration)
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ //permission
    ),

    // Handle 0x7E: report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_TOUCHPAD_RPT_REF_DESCR, //0x7E (Report reference handle)
        GATT_UUID_RPT_REF_DESCR,                                        //0x2908 (Report reference)
        LEGATTDB_PERM_READABLE                                          //permission
    ),
#endif

    // Connection control feature
    // Handle 0x7F: characteristic HID Report, handle 0x80 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL,
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL_VAL,
        GATT_UUID_HID_REPORT,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_WRITE,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_REQ
    ),

   // Handle 0x81: report reference
   CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),

#ifdef SUPPORTING_FINDME
    // Declare Imediate Alert Service
    PRIMARY_SERVICE_UUID16
        ( HANDLE_BLEREMOTE_IMMEDIATE_ALERT_SERVICE, UUID_SERVCLASS_IMMEDIATE_ALERT),

    // Handle 0x91: characteristic alert level, handle 0x92 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE
    (
        HANDLE_BLEREMOTE_IMMEDIATE_ALERT_SERVICE_CHAR_LEVEL,
        HANDLE_BLEREMOTE_IMMEDIATE_ALERT_SERVICE_CHAR_LEVEL_VAL,
        GATT_UUID_ALERT_LEVEL,
        LEGATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
        LEGATTDB_PERM_WRITE_CMD
    ),
#endif

#ifdef OTA_FIRMWARE_UPGRADE
#ifdef OTA_SECURE_FIRMWARE_UPGRADE
    // Handle 0xff00: Cypress vendor specific WICED Secure OTA Upgrade Service.
    PRIMARY_SERVICE_UUID128
        (HANDLE_OTA_FW_UPGRADE_SERVICE, UUID_OTA_SEC_FW_UPGRADE_SERVICE),
#else
    // Handle 0xff00: Cypress vendor specific WICED OTA Upgrade Service.
    PRIMARY_SERVICE_UUID128
        ( HANDLE_OTA_FW_UPGRADE_SERVICE, UUID_OTA_FW_UPGRADE_SERVICE ),
#endif
    // Handles 0xff03: characteristic WS Control Point, handle 0xff04 characteristic value.
    CHARACTERISTIC_UUID128_WRITABLE
    (
        HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT,
        HANDLE_OTA_FW_UPGRADE_CONTROL_POINT,
        UUID_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT,
        LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE,
        LEGATTDB_PERM_VARIABLE_LENGTH | LEGATTDB_PERM_WRITE_REQ  | LEGATTDB_PERM_AUTH_WRITABLE
    ),

    // Declare client characteristic configuration descriptor
    // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection
    // for bonded devices.  Setting value to 1 tells this application to send notification
    // when value of the characteristic changes.  Value 2 is to allow indications.
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_AUTH_WRITABLE
    ),

    // Handle 0xff07: characteristic WS Data, handle 0xff08 characteristic value. This
    // characteristic is used to send next portion of the FW Similar to the control point
    CHARACTERISTIC_UUID128_WRITABLE
    (
        HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_DATA,
        HANDLE_OTA_FW_UPGRADE_DATA,
        UUID_OTA_FW_UPGRADE_CHARACTERISTIC_DATA,
        LEGATTDB_CHAR_PROP_WRITE,
        LEGATTDB_PERM_VARIABLE_LENGTH | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_AUTH_WRITABLE
    ),
#endif
};
const uint16_t blehid_db_size = sizeof(blehid_db_data);

/*****************************************************************************
 * This is the report map for HID Service
  ****************************************************************************/
const uint8_t blehid_rpt_map[] =
{
                // STD_KB_REPORT_ID
                // Input Report, 8 bytes
                // 1st byte:Keyboard LeftControl/Keyboard Right GUI
                // 2nd byte:Constant, 3rd ~ 6th: keycode
                // Output Report, 1 byte: LED control
                0x05 , 0x01,                    // USAGE_PAGE (Generic Desktop)
                0x09 , 0x06,                    // USAGE (Keyboard)
                0xA1 , 0x01,                    // COLLECTION (Application)
                0x85 , STD_KB_REPORT_ID,        //    REPORT_ID (1)
                0x75 , 0x01,                    //    REPORT_SIZE (1)
                0x95 , 0x08,                    //    REPORT_COUNT (8)
                0x05 , 0x07,                    //    USAGE_PAGE (Keyboard)
                0x19 , 0xE0,                    //    USAGE_MINIMUM (Keyboard LeftControl)
                0x29 , 0xE7,                    //    USAGE_MAXIMUM (Keyboard Right GUI)
                0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
                0x25 , 0x01,                    //    LOGICAL_MAXIMUM (1)
                0x81 , 0x02,                    //    INPUT (Data,Var,Abs)
                0x95 , 0x01,                    //    REPORT_COUNT (1)
                0x75 , 0x08,                    //    REPORT_SIZE (8)
                0x81 , 0x03,                    //    INPUT (Cnst,Var,Abs)
                0x95 , 0x05,                    //    REPORT_COUNT (5)
                0x75 , 0x01,                    //    REPORT_SIZE (1)
                0x05 , 0x08,                    //    USAGE_PAGE (LEDs)
                0x19 , 0x01,                    //    USAGE_MINIMUM (Num Lock)
                0x29 , 0x05,                    //    USAGE_MAXIMUM (Kana)
                0x91 , 0x02,                    //    OUTPUT (Data,Var,Abs)
                0x95 , 0x01,                    //    REPORT_COUNT (1)
                0x75 , 0x03,                    //    REPORT_SIZE (3)
                0x91 , 0x03,                    //    OUTPUT (Cnst,Var,Abs)
                0x95 , 0x06,                    //    REPORT_COUNT (6)
                0x75 , 0x08,                    //    REPORT_SIZE (8)
                0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
                0x26 , 0xFF , 0x00,             //    LOGICAL_MAXIMUM (255)
                0x05 , 0x07,                    //    USAGE_PAGE (Keyboard)
                0x19 , 0x00,                    //    USAGE_MINIMUM (Reserved (no event indicated))
                0x29 , 0xFF,                    //    USAGE_MAXIMUM (Reserved (no event indicated))
                0x81 , 0x00,                    //    INPUT (Data,Ary,Abs)
                0xC0,                           // END_COLLECTION

                //Bit mapped report, BITMAPPED_REPORT_ID
                0x05 , 0x0C,                    // USAGE_PAGE (Consumer Devices)
                0x09 , 0x01,                    // USAGE (Consumer Control)
                0xA1 , 0x01,                    // COLLECTION (Application)
                0x85 , BITMAPPED_REPORT_ID,     //    REPORT_ID (2)
                0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
                0x25 , 0x01,                    //    LOGICAL_MAXIMUM (1)
                0x75 , 0x01,                    //    REPORT_SIZE (1)
                0x95 , 0x12,                    //    REPORT_COUNT (18)
                //byte 0
                0x0A , 0x94 , 0x01,             //    USAGE (AL Local Machine Browser)
                0x0A , 0x92 , 0x01,             //    USAGE (AL Calculator)
                0x0A , 0x83 , 0x01,             //    USAGE (Media)
                0x0A , 0x23 , 0x02,             //    USAGE (WWW Home)
                0x0A , 0x8A , 0x01,             //    USAGE (AL Email)
                0x0A , 0x82 , 0x01,             //    USAGE (Programmable Button Control)
                0x0A , 0x21 , 0x02,             //    USAGE (AC Search)
                0x0A , 0x24 , 0x02,             //    USAGE (AC Back)
                // byte 1
                0x0A , 0x25 , 0x02,             //    USAGE (AC Forward)
                0x0A , 0x26 , 0x02,             //    USAGE (AC Stop)
                0x0A , 0x27 , 0x02,             //    USAGE (AC Refresh)
                0x09 , 0xB6,                    //    USAGE (Scan Previous Track)
                0x09 , 0xB5,                    //    USAGE (Scan Next Track)
                0x09 , 0xB7,                    //    USAGE (Stop)
                0x09 , 0xB0,                    //    USAGE (Play)
                0x09 , 0xE9,                    //    USAGE (Volume Up)
                //byte 2
                0x09 , 0xEA,                    //    USAGE (Volume Down)
                0x09 , 0xE2,                    //    USAGE (Mute)
                0x81 , 0x02,                    //    INPUT (Data,Var,Abs)
                0x95 , 0x01,                    //    REPORT_COUNT (1)
                0x75 , 0x06,                    //    REPORT_SIZE (6)
                0x81 , 0x03,                    //    INPUT (Cnst,Var,Abs)
                0xC0,                           // END_COLLECTION
#ifdef SUPPORT_MOTION
#ifdef USE_MOTION_AS_AIR_MOUSE
                0x05, 0x01,             //    Usage Page (Generic Desktop),
                0x09, 0x02,             //    Usage (Mouse),
                0xA1, 0x01,             //    Collection: (Application),
                0x85, RPT_ID_MOUSE,             //        REPORT_ID (2)
                0x09, 0x01,             //        Usage (Pointer),
                0xA1, 0x00,             //        Collection: (Linked),
                0x05, 0x09,             //            Usage Page (Buttons),
                0x19, 0x01,             //            Usage Minimum (01),
                0x29, 0x03,             //            Usage Maximum (03),
                0x15, 0x00,             //            Log Min (0),
                0x25, 0x01,             //            Log Max (1),
                0x75, 0x01,             //            Report Size (1),
                0x95, 0x03,             //            Report Count (03),
                0x81, 0x02,             //            Input (Data, Variable, Absolute),
                0x75, 0x05,             //            Report Size (5),
                0x95, 0x01,             //            Report Count (1),
                0x81, 0x01,             //            Input (Constant),
                0x05, 0x01,             //            Usage Page (Generic Desktop),
                0x09, 0x30,             //            Usage (X),
                0x09, 0x31,             //            Usage (Y),
                0x15, 0x81,             //            Logical min (-127),
                0x25, 0x7F,             //            Logical Max (127),
                0x75, 0x08,             //            Report Size (8),
                0x95, 0x02,             //            Report Count (2) (X,Y)
                0x81, 0x06,             //            Input (Data, Variable, Relative),
                0x09, 0x38,             //            Usage (Wheel),
                0x15, 0x81,             //            Logical min (-127),
                0x25, 0x7F,             //            Logical Max (127),
                0x75, 0x08,             //            Report Size (8),
                0x95, 0x01,             //            Report Count (1) (Wheel)
                0x81, 0x06,             //            Input (Data, Variable, Relative),
                0xC0,                    //        END_COLLECTION (Logical)
                0xC0,                    //        END_COLLECTION
#else
                //motion report,  MOTION_REPORT_ID
                0x05 , 0x0C,                    // Usage Page (Consumer Devices)
                0x09 , 0x01,                    //    Usage (Consumer Control)
                0xA1 , 0x01,                    // COLLECTION (Application)
                0x85 , MOTION_REPORT_ID,        //    REPORT_ID (0x08)
                //0x95 , 0x12,                  //    REPORT_COUNT (18)
                0x95 , MOTIONRPT_MAX_DATA,      //    REPORT_COUNT (18)
                0x75 , 0x08,                    //    REPORT_SIZE (8)
                0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
                0x26 , 0xFF , 0x00,             //    LOGICAL_MAXIMUM (255)
                0x81 , 0x00,                    //    INPUT (Data,Ary,Abs)
                0xC0,                           // END_COLLECTION
#endif
#endif
                //User defined 0 report, 0x0A
                0x05 , 0x0C,                    // Usage Page (Consumer Devices)
                0x09 , 0x01,                    //    Usage (Consumer Control)
                0xA1 , 0x01,                    //    Collection (Application)
                0x85 , 0x0A,                    //      Report ID=0A
                0x95 , 0x08,                    //    REPORT_COUNT (8)
                0x75 , 0x08,                    //    REPORT_SIZE (8)
                0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
                0x26 , 0xFF , 0x00,             //    LOGICAL_MAXIMUM (255)
                0x05 , 0x07,                    //    USAGE_PAGE (Keyboard)
                0x19 , 0x00,                    //    USAGE_MINIMUM (Reserved (no event indicated))
                0x29 , 0xFF,                    //    USAGE_MAXIMUM (Reserved (no event indicated))
                0x81 , 0x00,                    //    INPUT (Data,Ary,Abs)
                0xC0,                           // END_COLLECTION

                //audio report, WICED_HIDD_VOICE_REPORT_ID
                0x05 , 0x0C,                    // Usage Page (Consumer Devices)
                0x09 , 0x01,                    //    Usage (Consumer Control)
                0xA1 , 0x01,                    //    Collection (Application)
                0x85 , WICED_HIDD_VOICE_REPORT_ID, //    Report ID=0xF7
#ifdef ATT_MTU_SIZE_180
                0x95 , 0xAC,                     //    REPORT_COUNT (172)
#else
                0x95 , 0x14,                     //    REPORT_COUNT (20)
#endif
                0x75 , 0x08,                    //    REPORT_SIZE (8)
                0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
                0x26 , 0xFF , 0x00,             //    LOGICAL_MAXIMUM (255)
                0x81 , 0x00,                    //    INPUT (Data,Ary,Abs)
                0xC0,                           // END_COLLECTION

                //voice ctrl report, WICED_HIDD_VOICE_CTL_REPORT_ID
                0x05 , 0x0C,                    // Usage Page (Consumer Devices)
                0x09 , 0x01,                    //    Usage (Consumer Control)
                0xA1 , 0x01,                    //    Collection (Application)
                0x85 , 0xF8,                    //    Report ID=0xF8
                0x95 , 0x0B,                     //    REPORT_COUNT (11)
                0x75 , 0x08,                    //    REPORT_SIZE (8)
                0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
                0x26 , 0xFF , 0x00,             //    LOGICAL_MAXIMUM (255)
                0x81 , 0x00,                    //    INPUT (Data,Ary,Abs)
                0x95 , 0x0B,                    //    REPORT_COUNT (11)
                0x75 , 0x01,                    //    REPORT_SIZE (8)
                0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
                0x26 , 0xFF , 0x00,             //    LOGICAL_MAXIMUM (255)
                0xB1 , 0x00,                    //    FEATURE (Data,Ary,Abs)
                0xC0,                           // END_COLLECTION

#ifdef SUPPORT_TOUCHPAD
                //touchpad report,  RPT_ID_IN_ABS_XY
                0x05 , 0x0C,                    // Usage Page (Consumer Devices)
                0x09 , 0x01,                    //    Usage (Consumer Control)
                0xA1 , 0x01,                    // COLLECTION (Application)
                0x85 , RPT_ID_IN_ABS_XY,         //    REPORT_ID (0x20)
                0x95 , 0x09,                     //    REPORT_COUNT (9)
                0x75 , 0x08,                    //    REPORT_SIZE (8)
                0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
                0x26 , 0xFF , 0x00,             //    LOGICAL_MAXIMUM (255)
                0x81 , 0x00,                    //    INPUT (Data,Ary,Abs)
                0xC0,                           // END_COLLECTION
#endif
                //Battery report
                0x05 , 0x0C,                    // Usage Page (Consumer Devices),
                0x09 , 0x01,                    // Usage (Consumer Control),
                0xA1 , 0x01,                    // COLLECTION (Application)
                0x85 , BATTERY_REPORT_ID,       //    REPORT_ID (3)
                0x05 , 0x01,                    //    Usage Page (Generic Desktop),
                0x09 , 0x06,                    //    Usage (Keyboard)
                0xA1 , 0x02,                    //    Collection: (Logical),
                0x05 , 0x06,                    //        USAGE PAGE (Generic Device Control),
                0x09 , 0x20,                    //        USAGE (Battery Strength),
                0x15 , 0x00,                    //        Log Min (0),
                0x26 , 0x64 , 0x00,             //        Log Max (255),
                0x75 , 0x08,                    //        Report Size (8),
                0x95 , 0x01,                    //        Report Count (1),
                0x81 , 0x02,                    //        Input (Data, Variable, Absolute),
                0xC0,                           //    END_COLLECTION (Logical)
                0xC0,                           // END_COLLECTION
            // END of Battery report
};

/*****************************************************************************
 * This is the attribute table containing LEGATTDB_PERM_READABLE attributes
  ****************************************************************************/
const attribute_t blehid_gattAttributes[] =
{
    {
        HANDLE_BLEREMOTE_GAP_SERVICE_CHAR_DEV_NAME_VAL,
        sizeof(dev_local_name),
        dev_local_name  //fixed
    },

    {
        HANDLE_BLEREMOTE_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
        sizeof(dev_appearance_name),
        dev_appearance_name //fixed
    },

    {
        HANDLE_BLEREMOTE_GAP_SERVICE_CHAR_PERI_PREFER_CONNPARAM_VAL,
        8,
        ble_hidd_link.prefered_conn_params //fixed
    },

    {
        HANDLE_BLEREMOTE_DEV_INFO_SERVICE_CHAR_PNP_ID_VAL,
        sizeof(dev_pnp_id),
        dev_pnp_id //fixed
    },

    {
        HANDLE_BLEREMOTE_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
        sizeof(dev_char_mfr_name_value),
        dev_char_mfr_name_value //fixed
    },

    {
        HANDLE_BLEREMOTE_DEV_INFO_SERVICE_CHAR_FW_VER_VAL,
        3,
        dev_char_fw_version_value //fixed
    },

    {
        HANDLE_BLEREMOTE_BATTERY_SERVICE_CHAR_LEVEL_VAL,
        1,
        &battery_level //get it from ADC
    },

    {
        HANDLE_BLEREMOTE_BATTERY_SERVICE_CHAR_CFG_DESCR,
        2,
        &characteristic_client_configuration[5]  //bit mask: KBAPP_CLIENT_CONFIG_NOTIF_BATTERY_RPT       (0x20)
    },

    {
        HANDLE_BLEREMOTE_BATTERY_SERVICE_RPT_REF_DESCR,
        2,
        rpt_ref_battery  //fixed
    },

    {
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_INFO_VAL,
        4,
        dev_hid_information   //fixed
    },

    {
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_MAP_VAL,
        sizeof(blehid_rpt_map),
        blehid_rpt_map, //fixed
    },

    {
        HANDLE_BLEREMOTE_LE_HID_SERVICE_EXT_RPT_REF_DESCR,
        2,
        &dev_battery_service_uuid //fixed
    },

    {
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_STD_INPUT_VAL,
        KEYRPT_MAX_KEYS_IN_STD_REPORT+2,
        bleremote_key_std_rpt //updated everytime a std key input report sent
    },

    {
       HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_STD_INPUT_CHAR_CFG_DESCR,
        2,
        &characteristic_client_configuration[1]  //bit mask: KBAPP_CLIENT_CONFIG_NOTIF_STD_RPT           (0x02)
    },

    {
       HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_STD_INPUT_RPT_REF_DESCR,
        2,
        rpt_ref_std_key_input //fixed
    },

    {
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_STD_OUTPUT_VAL,
        1,
        &bleremote_output_rpt //updated everytime a std key output report received
    },

    {
       HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_STD_OUTPUT_RPT_REF_DESCR,
        2,
        rpt_ref_std_key_output //fixed
    },

    {
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_BITMAP_VAL,
        KEYRPT_NUM_BYTES_IN_BIT_MAPPED_REPORT,
        bleremote_bitmap_rpt //updated everytime a bitmap report sent
    },

    {
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_BITMAP_CHAR_CFG_DESCR,
        2,
        &characteristic_client_configuration[2]  //bit mask: KBAPP_CLIENT_CONFIG_NOTIF_BIT_MAPPED_RPT    (0x04)
    },

    {
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_BITMAP_RPT_REF_DESCR,
        2,
        rpt_ref_bitmap //fixed
    },

#ifdef SUPPORT_MOTION
    {
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_MOTION_VAL,
        MOTIONRPT_MAX_DATA,
        bleremote_motion_rpt //updated everytime a motion report sent
    },

    {
       HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_MOTION_CHAR_CFG_DESCR,
        2,
        &characteristic_client_configuration[3]  //bit mask: KBAPP_CLIENT_CONFIG_NOTIF_MOTION_RPT                (0x08)
    },

    {
       HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_MOTION_RPT_REF_DESCR,
        2,
        rpt_ref_motion //fixed
    },
#endif
    {
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0_VAL,
        8,
        bleremote_user_defined_0_rpt    //updated everytime a user defined 0/key report sent
    },

    {
       HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0_CHAR_CFG_DESCR,
        2,
        &characteristic_client_configuration[4]  //bit mask: KBAPP_CLIENT_CONFIG_NOTIF_USER_DEFINED_KEY_RPT      (0x10)
    },

    {
       HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0_RPT_REF_DESCR,
        2,
        rpt_ref_user_defined_0  //fixed
    },

    {
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_VAL,
#ifdef ATT_MTU_SIZE_180
       WICED_HIDD_MIC_AUDIO_BUFFER_SIZE*2+1,
#else
        20,
#endif
        bleremote_voice_rpt //updated everytime a voice report sent
    },

    {
       HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_CHAR_CFG_DESCR,
        2,
        &characteristic_client_configuration[6]  //bit mask: KBAPP_CLIENT_CONFIG_NOTIF_VOICE_RPT                 (0x40)
    },

    {
       HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_RPT_REF_DESCR,
        2,
        rpt_ref_voice   //fixed
    },

    {
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT_VAL,
        11,
        bleremote_voice_ctrl_input_rpt  //updated everytime a voice ctrl input report sent
    },

    {
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT_CHAR_CFG_DESCR,
        2,
        &characteristic_client_configuration[7]  //bit mask: KBAPP_CLIENT_CONFIG_NOTIF_VOICE_CTRL_RPT            (0x80)
    },

    {
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT_RPT_REF_DESCR,
        2,
        rpt_ref_voice_ctrl_input    //fixed
    },

    {
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_FEA_VAL,
        11,
        bleremote_voice_ctrl_feature_rpt   //updated everytime a voice ctrl feature report received
    },

    {
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_FEA_RPT_REF_DESCR,
        2,
        rpt_ref_voice_ctrl_feature  //fixed
    },

#ifdef SUPPORT_TOUCHPAD
    {
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_TOUCHPAD_VAL,
        9,
        bleremote_touchpad_rpt  //updated everytime a touchpad report sent
    },

    {
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_TOUCHPAD_CHAR_CFG_DESCR,
        2,
        &characteristic_client_configuration[8]  //bit mask: KBAPP_CLIENT_CONFIG_NOTIF_TOUCHPAD_RPT            (0x100)
    },

    {
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_TOUCHPAD_RPT_REF_DESCR,
        2,
        rpt_ref_touchpad    //fixed
    },
#endif

    {
        HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL_VAL,
        1,
        &bleremote_connection_ctrl_rpt  //even though it is defined. But no usage. ignore now.
    },

    {
       HANDLE_BLEREMOTE_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL_RPT_REF_DESCR,
        2,
        rpt_ref_connection_ctrl     //fixed
    },
};
const uint16_t blehid_gattAttributes_size = sizeof(blehid_gattAttributes)/sizeof(attribute_t);


/*****************************************************************************
* Keyboard application configuration. Defines behavior of the keyboard
* application
*****************************************************************************/
KbAppConfig kbAppConfig =
{
    // Standard report ID
    1,

    // Maximum number of keys in standard report
    6,

    // Report ID of bit mapped report
    2,

    // Number of bit mapped keys
    18,

    // Sleep report ID
    4,

    // Pin report ID
    0xff,

    // LED (output) reportID
    1,

    // Default LED state is all off
    0,

    // Connect button scan index:
    18,

    // Recovery poll count
    3,

    // HW fifo threshold for idle rate report generation
    3,

    // Repeat the rollover report after every half a second, i.e. 1600 BT clocks
    1600,

    // Only repeat the rollover report if the HW fifo has fewer than this number of packets
    3,

    // Func lock report ID
    5,

    // Default state of the func-lock key
    0,

    // Scroll report ID
    6,

    // Scroll report length
    1,

    // No negation of scroll values
    FALSE,

    // Scroll scaling of 2 (2^1) for normal mechanical scroll
    1,

    // Keep fractional scroll data around for 50 polls
    50,

    // Enable scroll combining
    TRUE,

    // Size of each event
    6,

    // Maximum number of events
    44
};

/*****************************************************************************
/// Key translation table. It maps keyscan matrix position to key types and
/// specific usage within the type. For example, row 5, column 6 may be
/// mapped as a standard key with usage "ESCAPE". This means that the key
/// will be reported in the standard report with a USB usage of "ESCAPE"
/// See config documentation for details and the keyboard config for an example.
/// By default this table is initialized for the BCM keyboard
*****************************************************************************/
KbKeyConfig kbKeyConfig[] =
{
    // Column 0:  order is row0 ->row4
    {KEY_TYPE_STD,              0x1e},  //#0, 1
    {KEY_TYPE_STD,              0x1f},  //#1, 2
    {KEY_TYPE_STD,              0x20},  //#2, 3
    {KEY_TYPE_STD,              0x81},  //#3, VOL_DOWN
    {KEY_TYPE_STD,              0x28},  //#4, ENTER touchpad

    // Column 1: order is row0 ->row4
    {KEY_TYPE_STD,              0x21},  //#5,  4
    {KEY_TYPE_STD,              0x22},  //#6,  5
    {KEY_TYPE_STD,              0x23},  //#7, 6
    {KEY_TYPE_STD,              0x7f},  //#8, MUTE
    {KEY_TYPE_STD,              0xf1},  //#9, BACK

   // Column 2: order is row0 ->row4
    {KEY_TYPE_STD,              0x24},  //#10, 7
    {KEY_TYPE_STD,              0x25},  //#11, 8
    {KEY_TYPE_STD,              0x26},  //#12, 9
    {KEY_TYPE_STD,              0x80},  //#13, VOL_UP
    {KEY_TYPE_STD,              0x4a},  //#14, HOME

    // Column 3: order is row0 ->row4
    {KEY_TYPE_STD,              0x27},  //#15, 0
    {KEY_TYPE_USER_DEF_1,          0},  //#16, AUDIO
    {KEY_TYPE_STD,              0x66},  //#17, PWR
    {KEY_TYPE_USER_DEF_3,          0},  //#18, Heart (discoverable)
    {KEY_TYPE_USER_DEF_0,       0x94},  //#19, EXIT

    // Column 4: order is row0 ->row4
    // This column defines the virtual keys for touchpad
    {KEY_TYPE_STD,              0x28},  //#20, CENTER
    {KEY_TYPE_STD,              0x4f},  //#21, VIRTUAL RIGHT
    {KEY_TYPE_STD,              0x50},  //#22, VIRTUAL LEFT
    {KEY_TYPE_NONE,             0x51},  //#23, VIRTUAL DOWN
    {KEY_TYPE_STD,              0x52},  //#24, VIRTUAL UP

};
const uint8_t kbKeyConfig_size = sizeof(kbKeyConfig)/sizeof(KbKeyConfig);  //"Number of columns" = 5; "Number of rows" = 5

/*****************************************************************************
 * Remote application configuration
 ****************************************************************************/
RemoteAppConfig remoteAppConfig =
{
    /// the default lpm index for mode "HIGH", #not used in BLE remote
    0,

    /// the lpm index for motion, #not used in BLE remote
    0,

    /// the lpm index for voice, #not used in BLE remote
    0,

    /// report ID for motion data, #don't care in BLE remote. It is defined in bleremote.h, since it needs to be included in BLE HID Remote app GATT database
    8,

    /// KeyScan index of the IR button
    17,

    /// KeyScan index of the Motion START button. Temporaryly, X to start motion
    19,

    /// KeyScan index of the Motion STOP button. Temporaryly, Mute to stop motion
    8,

    /// KeyScan index of the Voice button
    16,

    /// delay sending audio time period in ms
    100,

    /// audio mode
    1, // WICED_HIDD_AUDIO_BUTTON_SEND_MSG,

    /// Audio ADC Gain in dB, When DRC is disabled 21 dB is recommended
    30,

    /// boost of the audio codec, #not used.
    TRUE,

    /// Maximum number of data bytes in remote report
    8,
    /// Number of different typpes of remote reports
    8,

    /// Maximum sample number read in one slot callback, #not used
    40,

    /// Remote user defined report configuration
    {
    ///rptID, size
        {0xA, 1},
        {0xB, 1},
        {0xC, 1},
        {0xD, 1},
        {0xE, 1},
        {0xF, 1},
        {0x10, 1},
        {0x11, 1},
    },

    /// Remote Key Translation Code
    // "Number of columns" = 4
    // "Number of rows" = 5
    {
        {{0x1e}}, {{0x0}},    //#0, 1
        {{0x1f}}, {{0x0}},    //#1, 2
        {{0x20}}, {{0x0}},    //#2, 3
        {{0x81}}, {{0x0}},    //#3, VOL_DOWN
        {{0x28}}, {{0x0}},    //#4, ENTER touchpad

        // Column 1
        {{0x21}}, {{0x0}},    //#5, 4
        {{0x22}}, {{0x0}},    //#6, 5
        {{0x23}}, {{0x0}},    //#7, 6
        {{0x7f}}, {{0x0}},    //#8, MUTE
        {{0xf1}}, {{0x0}},    //#9, BACK

        // Column 2
        {{0x24}}, {{0x0}},    //#10, 7
        {{0x25}}, {{0x0}},    //#11, 8
        {{0x26}}, {{0x0}},    //#12, 9
        {{0x80}}, {{0x0}},    //#13, VOL_UP
        {{0x4a}}, {{0x0}},    //#14, HOME

        // Column 3
        {{0x27}}, {{0x0}},    //#15, 0
        {{0xff}}, {{0x0}},    //#16, AUDIO
        {{0x66}}, {{0x0}},    //#17, PWR
        {{0xfe}}, {{0x0}},    //#18, Heart (discoverable)
        {{0x94}}, {{0x0}},    //#19, EXIT
    },

};

/*****************************************************************************
 * Application Audio related config
 ****************************************************************************/
wiced_hidd_microphone_enhanced_config_t blehid_audiocfg =
{
    //# audio enc type: 0=PCM, 1=mSBC, 2=OPUS CELT
#ifdef CELT_ENCODER
    2,
#endif
#ifdef SBC_ENCODER
    1,
#endif

    {
    //# DRC settings
    1,              // 1 Enable DRC, 0 Disable DRC
    0x02EE,     // Wait time in mSec, 0x2EE = 750 mSec.
    70,             // Knee 1, 68.5dB,       2660, in 1/2 dB steps.  10^((RSSI_target/2 + 30)/20).
    85,              // Knee 2, 75dB,         5623, in 1/2 dB steps.  10^((RSSI_target/2 + 30)/20).
    95,             // Knee 3, 81dB,        11220, in 1/2 dB steps.  10^((RSSI_target/2 + 30)/20).
    0x03E8,     // Attack time in mSec.  0x03E8 = 1000 mSec
    0x001F,     // Decay time in mSec.  0x001F = 31 mSec.
    0x6800,     // Saturation Level, 0x6800 = 26624.  This will be the max output level.
               // The DRC will behave like an AGC when the DRC curve exceeds this amount.
               // This value will be used when the pga gain is set lower than 18dB by the DRC loop.
    },
    //# DRC custom gain boost. Default value = 1000
    1496,
    //# End of DRC settings

#ifdef ENABLE_ADC_AUDIO_ENHANCEMENTS
    //#Anti Alias Audio Filter Coefficients.  Set index 0 to 0x00 0x00 for default filter settings
    0x6D00,  //#index 0
    0xB5FF,  //#index 1
    0x23FF,  //#index 2
    0xE4FF,  //#index 3
    0x1B01,  //#index 4
    0x4E00,  //#index 5
    0x4EFE,  //#index 6
    0x10FF,  //#index 7
    0x3E02,  //#index 8
    0xEC01,  //#index 9
    0x2BFD,  //#index 10
    0x70FC,  //#index 11
    0x5C03,  //#index 12
    0x6606,  //#index 13
    0x36FC,  //#index 14
    0x87F3,  //#index 15
    0x1204,  //#index 16
    0x5D28,  //#index 17
    0xD53B,  //#index 18
    //#End of Anti Alias Audio Filter Coefficients.


    /*# EQ Filter 1, 116 Coefficients(int16_t)
    #        1       2          3        4         5          6         7         8        9         10
    #    LSB  MSB LSB  MSB   LSB  MSB LSB  MSB  LSB  MSB  LSB  MSB  LSB  MSB  LSB  MSB  LSB  MSB  LSB  MSB
    #   --------- --------- --------- --------- --------- --------- --------- --------- --------- ---------
    #   To disable EQ filter, set the first two bytes below to 0.
    #   Customer should fill in the actually EQ coeff's based on specific test setup and HW */
    0,
#endif
};

/*****************************************************************************
 * wiced_bt core stack configuration
 ****************************************************************************/
const wiced_bt_cfg_settings_t wiced_bt_hid_cfg_settings =
{
    (uint8_t *)dev_local_name,                             /**< Local device name (NULL terminated) */
    {0x00, 0x05, 0xc0},                                             /**< Local device class */
    BTM_SEC_ENCRYPT,                                                  /**< Security requirements mask (BTM_SEC_NONE, or combinination of BTM_SEC_IN_AUTHENTICATE, BTM_SEC_OUT_AUTHENTICATE, BTM_SEC_ENCRYPT (see #wiced_bt_sec_level_e)) */
    1,                                                              /**< Maximum number simultaneous links to different devices */


    /* BR/EDR scan config */
    {
        BTM_SCAN_TYPE_STANDARD,                                     /**< Inquiry scan type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_INTERVAL,                 /**< Inquiry scan interval  (0 to use default) */
        WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_WINDOW,                   /**< Inquiry scan window (0 to use default) */

        BTM_SCAN_TYPE_STANDARD,                                     /**< Page scan type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        WICED_BT_CFG_DEFAULT_PAGE_SCAN_INTERVAL,                    /**< Page scan interval  (0 to use default) */
        WICED_BT_CFG_DEFAULT_PAGE_SCAN_WINDOW                       /**< Page scan window (0 to use default) */
    },

    /* BLE scan settings  */
    {
        BTM_BLE_SCAN_MODE_PASSIVE,                                  /**< BLE scan mode (BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE) */

        WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_INTERVAL,               /**< High duty scan interval */
        WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_WINDOW,                 /**< High duty scan window */
        5,                                                          /**< High duty scan duration in seconds (0 for infinite) */

        WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_INTERVAL,                /**< Low duty scan interval  */
        WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_WINDOW,                  /**< Low duty scan window */
        5,                                                          /**< Low duty scan duration in seconds (0 for infinite) */

        /* Connection scan intervals */
        WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_INTERVAL,          /**< High duty cycle connection scan interval */
        WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_WINDOW,            /**< High duty cycle connection scan window */
        30,                                                         /**< High duty cycle connection duration in seconds (0 for infinite) */

        WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_INTERVAL,           /**< Low duty cycle connection scan interval */
        WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_WINDOW,             /**< Low duty cycle connection scan window */
        30,                                                         /**< Low duty cycle connection duration in seconds (0 for infinite) */

#ifdef CELT_ENCODER
        /* Connection configuration */
        16,                                                          /**< Minimum connection interval. 16*1.25=20ms */
        16,                                                          /**< Maximum connection interval. 16*1.25=20ms */
        49,                                                         /**< Connection latency */
        500                                                         /**< Connection link supervision timeout. 500*10=5000ms */
#else
        /* Connection configuration */
        8,                                                          /**< Minimum connection interval. 8*1.25=10ms */
        8,                                                          /**< Maximum connection interval. 8*1.25=10ms */
        99,                                                         /**< Connection latency */
        500                                                         /**< Connection link supervsion timeout. 500*10=5000ms */
#endif
    },

    /* BLE advertisement settings */
    {
        BTM_BLE_ADVERT_CHNL_37 |                                    /**< Advertising channel map (mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39) */
        BTM_BLE_ADVERT_CHNL_38 |
        BTM_BLE_ADVERT_CHNL_39,

        32,                                                         /**< High duty undirected connectable minimum advertising interval. 32 *0.625 = 20ms */
        32,                                                         /**< High duty undirected connectable maximum advertising interval. 32 *0.625 = 20ms */
#ifdef ALLOW_SDS_IN_DISCOVERABLE
        0,                                                          /**< High duty undirected connectable advertising duration in seconds (0 for infinite) */
#else
        30,                                                         /**< High duty undirected connectable advertising duration in seconds (0 for infinite) */
#endif

        48,                                                         /**< Low duty undirected connectable minimum advertising interval. 48 *0.625 = 30ms */
        48,                                                         /**< Low duty undirected connectable maximum advertising interval */
#ifdef ALLOW_SDS_IN_DISCOVERABLE
        0,                                                          /**< Low duty undirected connectable advertising duration in seconds (0 for infinite) */
#else
        180,                                                        /**< Low duty undirected connectable advertising duration in seconds (0 for infinite) */
#endif
        32,                                                         /**< High duty directed connectable minimum advertising interval. 32 *0.625 = 20ms */
        32,                                                         /**< High duty directed connectable maximum advertising interval. 32 *0.625 = 20ms */

        2048,                                                       /**< Low duty directed connectable minimum advertising interval. 2048 * 0.625ms = 1.28 seconds */
        2048,                                                       /**< Low duty directed connectable maximum advertising interval. 2048 * 0.625ms = 1.28 seconds */
        0,                                                         /**< Low duty directed connectable advertising duration in seconds (0 for infinite) */

        WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,    /**< High duty non-connectable minimum advertising interval */
        WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,    /**< High duty non-connectable maximum advertising interval */
        30,                                                         /**< High duty non-connectable advertising duration in seconds (0 for infinite) */

        WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,     /**< Low duty non-connectable minimum advertising interval */
        WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,     /**< Low duty non-connectable maximum advertising interval */
        0                                                           /**< Low duty non-connectable advertising duration in seconds (0 for infinite) */
    },

    /* GATT configuration */
    {
        APPEARANCE_GENERIC_TAG,                                     /**< GATT appearance (see gatt_appearance_e) */
        1,                                                          /**< Client config: maximum number of servers that local client can connect to  */
        1,                                                          /**< Server config: maximum number of remote clients connections allowed by the local */
        246,                                                        /**< Maximum attribute length; gki_cfg must have a corresponding buffer pool that can hold this length */
        251                                                         /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5 )*/
    },

    /* RFCOMM configuration */
    {
        0,                                                          /**< Maximum number of simultaneous connected remote devices*/
        0                                                           /**< Maximum number of simultaneous RFCOMM ports */
    },

    /* Application managed l2cap protocol configuration */
    {
        0,                                                          /**< Maximum number of application-managed l2cap links (BR/EDR and LE) */

        /* BR EDR l2cap configuration */
        0,                                                          /**< Maximum number of application-managed BR/EDR PSMs */
        0,                                                          /**< Maximum number of application-managed BR/EDR channels  */

        /* LE L2cap connection-oriented channels configuration */
        0,                                                          /**< Maximum number of application-managed LE PSMs */
        0,                                                          /**< Maximum number of application-managed LE channels */
        /* LE L2cap fixed channel configuration */
        0                                                           /**< Maximum number of application managed fixed channels supported (in addition to mandatory channels 4, 5 and 6). > */
    },


    /* Audio/Video Distribution configuration */
    {
        0,                                                          /**< Maximum simultaneous audio/video links */
        0                                                           /**< Maximum number of stream end points */
    },

    /* Audio/Video Remote Control configuration */
    {
        0,                                                          /**< Mask of local roles supported (AVRC_CONN_INITIATOR|AVRC_CONN_ACCEPTOR) */
        0                                                           /**< Maximum simultaneous remote control links */
    },

		
	/* LE Address Resolution DB size  */
    5,                                                               /**< LE Address Resolution DB settings - effective only for pre 4.2 controller*/

    /* Maximum number of buffer pools */
    4,                                                              /**< Maximum number of buffer pools in p_btm_cfg_buf_pools and by wiced_create_pool */

#ifdef LE_LOCAL_PRIVACY_SUPPORT
    /* Interval of  random address refreshing */
    WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_CHANGE_TIMEOUT,             /**< Interval of  random address refreshing - secs */
#else
    WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE,               /**< Interval of  random address refreshing - secs */
#endif

    /* BLE white list size */
    0                                                               /**< Maximum number of white list devices allowed. Cannot be more than 128 */
};
/*****************************************************************************5
 * wiced_bt_stack buffer pool configuration
 *
 * Configure buffer pools used by the stack
 *
 * Pools must be ordered in increasing buf_size.
 * If a pool runs out of buffers, the next  pool will be used
 *****************************************************************************/
const wiced_bt_cfg_buf_pool_t wiced_bt_hid_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS] =
{
/*  { buf_size, buf_count } */
    { 64,       4   },      /* Small Buffer Pool */
    { 100,      30  },      /* Medium Buffer Pool (used for HCI & RFCOMM control messages, min recommended size is 360) */
    { 300,       8  },      /* Large Buffer Pool  (used for HCI ACL messages) */
    { 1024,      2  },      /* Extra Large Buffer Pool - Used for avdt media packets and miscellaneous (if not needed, set buf_count to 0) */
};



