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
 * Runtime Bluetooth stack and mouse configuration parameters
 *
 */
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_uuid.h"
#include "ble_mouse.h"
#include "ble_mouse_gatts.h"

#ifdef OTA_FIRMWARE_UPGRADE
#include "wiced_bt_ota_firmware_upgrade.h"
#endif


const char  dev_local_name[]             = "CY BLE MOUSE";
const uint8_t dev_appearance_name[2]     = {BIT16_TO_8(APPEARANCE_HID_MOUSE)};
const uint8_t dev_pnp_id[]               ={0x01, 0x31, 0x01, 0xB4, 0x04, 0x01, 0x00}; //BT SIG, cypress semiconductor, 0x04B4, 0x0001
const char dev_char_mfr_name_value[]     = "Cypress Semiconductor";

const uint8_t   rpt_ref_battery[]             = {BATTERY_REPORT_ID ,0x01};
const uint8_t   rpt_ref_input[]               = {MOUSE_REPORT_ID,0x01};

const uint8_t dev_hid_information[] = {0x00, 0x01, 0x00, 0x00};      // Verison 1.00, Not localized, Cannot remote wake, not normally connectable
const uint16_t dev_battery_service_uuid = UUID_CHARACTERISTIC_BATTERY_LEVEL;

extern uint16_t characteristic_client_configuration[];
extern uint8_t mouseapp_protocol;
extern uint8_t battery_level;

extern uint8_t blemouse_input_rpt[];



/*****************************************************************************
 * This is the GATT database for the BLE HID MOUSE application.  It defines
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
        ( HANDLE_BLEMOUSE_GATT_SERVICE, UUID_SERVCLASS_GATT_SERVER ),

   // Declare GAP service. Device Name and Appearance are mandatory
    // characteristics of GAP service
    PRIMARY_SERVICE_UUID16
        ( HANDLE_BLEMOUSE_GAP_SERVICE, UUID_SERVCLASS_GAP_SERVER ),

    // Declare mandatory GAP service characteristic: Dev Name
    CHARACTERISTIC_UUID16
        (
          HANDLE_BLEMOUSE_GAP_SERVICE_CHAR_DEV_NAME,
          HANDLE_BLEMOUSE_GAP_SERVICE_CHAR_DEV_NAME_VAL,
          GATT_UUID_GAP_DEVICE_NAME,
          LEGATTDB_CHAR_PROP_READ,
          LEGATTDB_PERM_READABLE
        ),

    // Declare mandatory GAP service characteristic: Appearance
    CHARACTERISTIC_UUID16
       (
         HANDLE_BLEMOUSE_GAP_SERVICE_CHAR_DEV_APPEARANCE,
         HANDLE_BLEMOUSE_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
         GATT_UUID_GAP_ICON,
         LEGATTDB_CHAR_PROP_READ,
         LEGATTDB_PERM_READABLE
       ),

    // Declare GAP service characteristic: Peripheral Prefered Connection Parameter
    CHARACTERISTIC_UUID16
       (
         HANDLE_BLEMOUSE_GAP_SERVICE_CHAR_PERI_PREFER_CONNPARAM,
         HANDLE_BLEMOUSE_GAP_SERVICE_CHAR_PERI_PREFER_CONNPARAM_VAL,
         GATT_UUID_GAP_PREF_CONN_PARAM,
         LEGATTDB_CHAR_PROP_READ,
         LEGATTDB_PERM_READABLE
       ),

    // Declare Device info service
    PRIMARY_SERVICE_UUID16
        ( HANDLE_BLEMOUSE_DEV_INFO_SERVICE, UUID_SERVCLASS_DEVICE_INFO ),

    // Handle 0x29: characteristic PnP ID, handle 0x2A characteristic value
    CHARACTERISTIC_UUID16
        (
          HANDLE_BLEMOUSE_DEV_INFO_SERVICE_CHAR_PNP_ID,
          HANDLE_BLEMOUSE_DEV_INFO_SERVICE_CHAR_PNP_ID_VAL,
          GATT_UUID_PNP_ID,
          LEGATTDB_CHAR_PROP_READ,
          LEGATTDB_PERM_READABLE
        ),

    // Handle 0x2B: characteristic Manufacturer Name, handle 0x2C characteristic value
    CHARACTERISTIC_UUID16
        (
          HANDLE_BLEMOUSE_DEV_INFO_SERVICE_CHAR_MFR_NAME,
          HANDLE_BLEMOUSE_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
          GATT_UUID_MANU_NAME,
          LEGATTDB_CHAR_PROP_READ,
          LEGATTDB_PERM_READABLE
        ),

   // Declare Battery service
   PRIMARY_SERVICE_UUID16
        ( HANDLE_BLEMOUSE_BATTERY_SERVICE, UUID_SERVCLASS_BATTERY),

   // Handle 0x31: characteristic Battery Level, handle 0x32 characteristic value
   CHARACTERISTIC_UUID16
        (
          HANDLE_BLEMOUSE_BATTERY_SERVICE_CHAR_LEVEL,       // attribute handle
          HANDLE_BLEMOUSE_BATTERY_SERVICE_CHAR_LEVEL_VAL, // attribute value handle
          GATT_UUID_BATTERY_LEVEL,
          LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_NOTIFY,
          LEGATTDB_PERM_READABLE
        ),

   // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
   CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_BLEMOUSE_BATTERY_SERVICE_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
    ),

   // Handle 0x34: report reference
   CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEMOUSE_BATTERY_SERVICE_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),


   // Declare Scan Parameters service
   PRIMARY_SERVICE_UUID16
        ( HANDLE_BLEMOUSE_SCAN_PARAM_SERVICE, UUID_SERVCLASS_SCAN_PARAM),

   // Handle 0x41: characteristic Battery Level, handle 0x42 characteristic value
   CHARACTERISTIC_UUID16_WRITABLE
    (
        HANDLE_BLEMOUSE_SCAN_PARAM_SERVICE_CHAR_SCAN_INT_WINDOW,
        HANDLE_BLEMOUSE_SCAN_PARAM_SERVICE_CHAR_SCAN_INT_WINDOW_VAL,
        GATT_UUID_SCAN_INT_WINDOW,
        LEGATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
        LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ
    ),

   // Declare HID over LE
   PRIMARY_SERVICE_UUID16
        ( HANDLE_BLEMOUSE_LE_HID_SERVICE, UUID_SERVCLASS_LE_HID),


    // Include BSA SERVICE
    INCLUDE_SERVICE_UUID16
    (
        HANDLE_BLEMOUSE_LE_HID_SERVICE_INC_BAS_SERVICE,
        HANDLE_BLEMOUSE_BATTERY_SERVICE,
        HANDLE_BLEMOUSE_BATTERY_SERVICE_RPT_REF_DESCR,
        UUID_SERVCLASS_BATTERY
    ),

    // Handle 0x51: characteristic Protocol Mode, handle 0x52 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE
    (
        HANDLE_BLEMOUSE_LE_HID_SERVICE_PROTO_MODE,
        HANDLE_BLEMOUSE_LE_HID_SERVICE_PROTO_MODE_VAL,
        GATT_UUID_HID_PROTO_MODE,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD
    ),

    // Handle 0x53: characteristic HID information, handle 0x54 characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_INFO,
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_INFO_VAL,
        GATT_UUID_HID_INFORMATION,
        LEGATTDB_CHAR_PROP_READ,
        LEGATTDB_PERM_READABLE
    ),


    // Handle 0x55: characteristic Boot Mouse Input report, handle 0x56 characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_BT_MOUSE_INPUT,
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_BT_MOUSE_INPUT_VAL,
        GATT_UUID_HID_BT_MOUSE_INPUT,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_NOTIFY,
        LEGATTDB_PERM_READABLE
    ),


   // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
   CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_BT_MOUSE_INPUT_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
    ),


    // Handle 0x58: characteristic HID Report MAP, handle 0x59 characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_MAP,
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_MAP_VAL,
        GATT_UUID_HID_REPORT_MAP,
        LEGATTDB_CHAR_PROP_READ,
        LEGATTDB_PERM_READABLE
    ),


   // include Battery Service
   // Handle 0x5A: external report reference
   CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEMOUSE_LE_HID_SERVICE_EXT_RPT_REF_DESCR,
        GATT_UUID_EXT_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),


    // Mouse Input report
    // Handle 0x5B: characteristic HID Report, handle 0x5C characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_STD_INPUT,
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_STD_INPUT_VAL,
        GATT_UUID_HID_REPORT,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_NOTIFY,
        LEGATTDB_PERM_READABLE
    ),

   // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
   CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_STD_INPUT_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
    ),

   // Handle 0x5E: report reference
   CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_STD_INPUT_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),

    // HID control point
    // Handle 0x62: characteristic HID Report, handle 0x63 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE
    (
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_HID_CTRL_POINT,
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_HID_CTRL_POINT_VAL,
        GATT_UUID_HID_CONTROL_POINT,
        LEGATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
        LEGATTDB_PERM_WRITE_CMD
    ),

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
            // MOUSE_REPORT_ID
            //mouse (3 buttom + X+Y+Wheel), 34 + 32=66 bytes
            0x05, 0x01,             //  Usage Page (Generic Desktop),
            0x09, 0x02,             //  Usage (Mouse),
            0xA1, 0x01,             //  Collection: (Application),
            0x85, MOUSE_REPORT_ID,   //     REPORT_ID (2)
            0x09, 0x01,             //      Usage (Pointer),
            0xA1, 0x00,             //      Collection: (Linked),
            // byte 0  bit 0(LEFT), 1(RIGHT), 2: button(MID). bit 3-7 not used.
            0x05, 0x09,             //          Usage Page (Buttons),
            0x19, 0x01,             //          Usage Minimum (01),
            0x29, 0x03,             //          Usage Maximum (03),
            0x15, 0x00,             //          Log Min (0),
            0x25, 0x01,             //          Log Max (1),
            0x75, 0x01,             //          Report Size (1),
            0x95, 0x03,             //          Report Count (03),
            0x81, 0x02,             //          Input (Data, Variable, Absolute),
            0x75, 0x05,             //          Report Size (5),
            0x95, 0x01,             //          Report Count (1),
            0x81, 0x01,             //          Input (Constant),
            // x: 12 bits, y:12 bits. wheel: 8bits, total:4bytes
            // byte 1 ~ 5
            0x05, 0x01,             //          Usage Page (Generic Desktop),
            0x09, 0x30,             //          Usage (X),
            0x09, 0x31,             //          Usage (Y),
            0x16, 0x01, 0xF8,       //          Logical min (-2047),
            0x26, 0xFF, 0x07,       //          Logical Max (2047),
            0x75, 0x0C,             //          Report Size (12),
            0x95, 0x02,             //          Report Count (2) (X,Y)
            0x81, 0x06,             //          Input (Data, Variable, Relative),
            0x09, 0x38,             //          Usage (Wheel),
            0x15, 0x81,             //          Logical min (-127),
            0x25, 0x7F,             //          Logical Max (127),
            0x75, 0x08,             //          Report Size (8),
            0x95, 0x01,             //          Report Count (1) (Wheel)
            0x81, 0x06,             //          Input (Data, Variable, Relative),
            0xC0,                   //      END_COLLECTION (Logical)
            0xC0,                   //      END_COLLECTION

            //Battery Report
            0x05, 0x0C,             //      Usage Page (Consumer Devices),
            0x09, 0x01,             //      Usage (Consumer Control),
            0xA1, 0x01,             //      COLLECTION (Application)
            0x85, BATTERY_REPORT_ID,//      REPORT_ID (3)
            0x05, 0x06,             //      USAGE PAGE (Generic Device Control),
            0x09, 0x20,             //      USAGE (Battery Strength),
            0x15, 0x00,             //      Log Min (0),
            0x26, 0x64, 0x00,       //      Log Max (100),
            0x75, 0x08,             //      Report Size (8),
            0x95, 0x01,             //      Report Count (1),
            0x81, 0x02,             //      Input (Data, Variable, Absolute),
            0xC0,                   //      END_COLLECTION

};

/*****************************************************************************
 * This is the attribute table containing LEGATTDB_PERM_READABLE attributes
  ****************************************************************************/
const attribute_t blehid_gattAttributes[] =
{
    {
        HANDLE_BLEMOUSE_GAP_SERVICE_CHAR_DEV_NAME_VAL,
        sizeof(dev_local_name),
        dev_local_name  //fixed
    },

    {
        HANDLE_BLEMOUSE_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
        sizeof(dev_appearance_name),
        dev_appearance_name //fixed
    },

    {
        HANDLE_BLEMOUSE_GAP_SERVICE_CHAR_PERI_PREFER_CONNPARAM_VAL,
        8,
        ble_hidd_link.prefered_conn_params //fixed
    },

    {
        HANDLE_BLEMOUSE_DEV_INFO_SERVICE_CHAR_PNP_ID_VAL,
        sizeof(dev_pnp_id),
        dev_pnp_id  //fixed
    },

    {
        HANDLE_BLEMOUSE_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
        sizeof(dev_char_mfr_name_value),
        dev_char_mfr_name_value //fixed
    },

    {
        HANDLE_BLEMOUSE_BATTERY_SERVICE_CHAR_LEVEL_VAL,
        1,
        &battery_level //get the battery reading from ADC
    },

    {
        HANDLE_BLEMOUSE_BATTERY_SERVICE_CHAR_CFG_DESCR,
        2,
        &characteristic_client_configuration[5]  //bit mask: KBAPP_CLIENT_CONFIG_NOTIF_BATTERY_RPT   (0x20)
    },

    {
        HANDLE_BLEMOUSE_BATTERY_SERVICE_RPT_REF_DESCR,
        2,
        rpt_ref_battery     //fixed
    },

    {
        HANDLE_BLEMOUSE_LE_HID_SERVICE_PROTO_MODE_VAL,
        1,
        &mouseapp_protocol   //Report Protocol mode
    },

    {
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_INFO_VAL,
        4,
        dev_hid_information   //fixed
    },

    {
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_BT_MOUSE_INPUT_VAL,
        5,
        blemouse_input_rpt   //updated before a bootmode input report sent
    },

    {
       HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_BT_MOUSE_INPUT_CHAR_CFG_DESCR,
        2,
        &characteristic_client_configuration[0]  //bit mask: KBAPP_CLIENT_CONFIG_NOTIF_BOOT_RPT          (0x01)
    },

    {
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_MAP_VAL,
        sizeof(blehid_rpt_map),
        blehid_rpt_map  //fixed
    },

    {
        HANDLE_BLEMOUSE_LE_HID_SERVICE_EXT_RPT_REF_DESCR,
        2,
        &dev_battery_service_uuid   //fixed
    },

    {
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_STD_INPUT_VAL,
        5,
        blemouse_input_rpt   //updated before a std key input report sent
    },

    {
       HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_STD_INPUT_CHAR_CFG_DESCR,
        2,
        &characteristic_client_configuration[1]  //bit mask: KBAPP_CLIENT_CONFIG_NOTIF_STD_RPT           (0x02)
    },

    {
       HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_STD_INPUT_RPT_REF_DESCR,
        2,
        rpt_ref_input   //fixed
    },
};
const uint16_t blehid_gattAttributes_size = sizeof(blehid_gattAttributes)/sizeof(attribute_t);

/*****************************************************************************
* Mouse application configuration. Defines behavior of the mouse application
*****************************************************************************/
MouseAppConfig blemouseAppConfig =
{
    /// Mouse motion report ID
    //uint8_t motionReportID
    MOUSE_REPORT_ID,

    /// Mouse motion report size in boot mode
    //uint8_t motionReportBootModeSize
    4,

    /// Mouse motion report size in report mode
    //uint8_t motionReportReportModeSize
    5,

    /// Connect button mask.
    //uint16_t connectButtonMask
    0x8000,

    /// Certain high resolution sensors keep track of more motion than can fit in a byte
    /// but only report it one byte at a time. Such sensors may have to be polled
    /// multiple times to get all the motion. This specifies how many times the
    /// a sensor can be polled. Note that this is a max. value. The sensor may
    /// be polled fewer times if reports no motion (either through a motion line
    /// or because the last motion value is 0).
    //uint8_t maxNumXYReadsPerPoll
    3,

    /// Flag indicating whether X/Y from the sensor should be swapped
    //uint8_t swapXY
    0,

    /// Flags indicating X data from the sensor should be negated.
    /// Note that X negation is done after swapping.
    //uint8_t negateX;
    0,
    /// Flags indicating Y data from the sensor should be negated.
    /// Note that Y negation is done after swapping.
    //uint8_t negateY;
    0,
    /// Flag indicating that scroll data should be negated
    //uint8_t negateScroll
    1,

    /// Scale values for X motion data. Should be set to zero if no scaling is desired
    /// Scaling always divides the input data, i.e. its a shift right.
    //uint8_t xScale;
    0,

    /// Scale values for Y motion data. Should be set to zero if no scaling is desired
    /// Scaling always divides the input data, i.e. its a shift right.
    //uint8_t yScale
    0,

    /// Scale values for scroll wheel data  . Should be set to zero if no scaling is desired
    /// Scaling always divides the input data, i.e. its a shift right.
    //uint8_t scrollScale
    1,

    /// Maximum number of ticks for which fractional motion data is kept, i.e. if no
    /// additional motion is detected, remaining fractional data is discarded. If set
    /// to 0, data is never discarded
    //uint8_t pollsToKeepFracXYData
    0,

    /// Maximum number of ticks for which fractional scroll wheel motion data is kept,
    /// i.e. if no additional motion is detected, remaining fractional data is discarded.
    /// If set to 0, data is never discarded. If scroll scaling is not used, should be set to
    /// 0 to improve execution efficiency.
    //uint8_t pollsToKeepFracScrollData
    50,

    /// Flag to enable report combining. If enabled, multiple motion events of the same type
    /// will be combined by the FW into a single report. Note that this only works for motion/scroll
    /// data and will not combine motion and scroll events together. Also note that events detected
    /// in the same poll period are always combined into a single mouse report
    /// regardless of the value of this flag.
    //uint8_t eventCombining
    1,

    /// Size of each element in the event queue. Note: This has to be at least as large as the
    /// largest event that the app will handle
    6,

    /// Maximum number of events that the event queue can hold.
    100
};

/*****************************************************************************
 * wiced_bt core stack configuration
 ****************************************************************************/
const wiced_bt_cfg_settings_t wiced_bt_hid_cfg_settings =
{
    (uint8_t *)dev_local_name,                             /**< Local device name (NULL terminated) */
    {0x40,0x05, 0x00},                                             /**< Local device class */
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

        /* Connection configuration */
        8,                                                          /**< Minimum connection interval */
        8,                                                          /**< Maximum connection interval */
        49,                                                        /**< Connection latency */
        600                                                         /**< Connection link supervsion timeout */
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
        512,                                                        /**< Maximum attribute length; gki_cfg must have a corresponding buffer pool that can hold this length */
        517                                                         /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5 )*/
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
    5,                                                              /**< LE Address Resolution DB settings - effective only for pre 4.2 controller*/
    /* Maximum number of buffer pools */
    4,                                                              /**< Maximum number of buffer pools in p_btm_cfg_buf_pools and by wiced_create_pool */

    /* Interval of  random address refreshing */
    WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_CHANGE_TIMEOUT,             /**< Interval of  random address refreshing - secs */

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
    { 300,      10  },      /* Large Buffer Pool  (used for HCI ACL messages) */
    { 700,      2   },      /* Extra Large Buffer Pool - Used for avdt media packets and miscellaneous (if not needed, set buf_count to 0) */
};



