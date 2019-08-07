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
 * Runtime Bluetooth stack configuration parameters
 *
 */

#include "wiced_app_cfg.h"
#include "wiced_bt_avrc_defs.h"
#include "wiced_bt_avrc.h"
//#include "wiced_bt_hid_defs.h"

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_sdp.h"
#include "wiced_app.h"
#include "wiced_bt_avrc_tg.h"
#include "wiced_bt_audio.h"
#include "wiced_bt_avdt.h"

/* If APP_AVRC_TRACK_INFO_SUPPORTED, APP_AVRC_PLAY_STATUS_SUPPORTED or APP_AVRC_SETTING_CHANGE_SUPPORTED are supported, set the AVRC profile
 version as 1.3, else set it to 1.0 */
#if (defined(APP_AVRC_TRACK_INFO_SUPPORTED) || defined(APP_AVRC_PLAY_STATUS_SUPPORTED) || defined(APP_AVRC_SETTING_CHANGE_SUPPORTED))
#define AVRC_PROFILE_VER  AVRC_REV_1_3
#else
#define AVRC_PROFILE_VER  AVRC_REV_1_0
#endif

/*
 * Definitions
 */
// SDP Record handle for AVDT Source
#define HANDLE_AVDT_SOURCE                      0x10001
// SDP Record handle for AVRC TARGET
#define HANDLE_AVRC_TARGET                      0x10002
// SDP Record handle for AVRC TARGET
#define HANDLE_AVRC_CONTROLLER                  0x10003
// SDP Record handle for PNP (Device Information)
#define HANDLE_PNP                              0x10006

#define WICED_DEVICE_NAME                       "Watch"

#define AV_SBC_MAX_BITPOOL          53

#if defined(CYW43012C0) || defined(CYW20819A1)
#define AUDIO_TX_BUFFER_SIZE        0x2000
#else
#define AUDIO_TX_BUFFER_SIZE        11000
#endif

#define AUDIO_CODEC_BUFFER_SIZE     0x2000

const uint8_t pincode[WICED_PIN_CODE_LEN] = { 0x30, 0x30, 0x30, 0x30 };

/*****************************************************************************
 * wiced_bt core stack configuration
 ****************************************************************************/
const wiced_bt_cfg_settings_t wiced_bt_cfg_settings =
{
    .device_name                         = ( uint8_t* )WICED_DEVICE_NAME,                                /**< Local device name ( NULL terminated ) */
    .device_class                        = {0x20, 0x07, 0x04},                                           /**< Local device class (Wearable/Wristwatch) */
    .security_requirement_mask           = ( BTM_SEC_IN_AUTHENTICATE | BTM_SEC_OUT_AUTHENTICATE | BTM_SEC_ENCRYPT ), /**< Security requirements mask ( BTM_SEC_NONE, or combination of BTM_SEC_IN_AUTHENTICATE, BTM_SEC_OUT_AUTHENTICATE, BTM_SEC_ENCRYPT ( see #wiced_bt_sec_level_e ) ) */

    .max_simultaneous_links              = 3,                                                            /**< Maximum number simultaneous links to different devices */

    .br_edr_scan_cfg =                                                                                   /* BR/EDR scan config */
    {
        .inquiry_scan_type               = BTM_SCAN_TYPE_STANDARD,                                     /**< Inquiry scan type ( BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED ) */
        .inquiry_scan_interval           = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_INTERVAL,                 /**< Inquiry scan interval  ( 0 to use default ) */
        .inquiry_scan_window             = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_WINDOW,                   /**< Inquiry scan window ( 0 to use default ) */

        .page_scan_type                  = BTM_SCAN_TYPE_STANDARD,                                     /**< Page scan type ( BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED ) */
        .page_scan_interval              = WICED_BT_CFG_DEFAULT_PAGE_SCAN_INTERVAL,                    /**< Page scan interval  ( 0 to use default ) */
        .page_scan_window                = WICED_BT_CFG_DEFAULT_PAGE_SCAN_WINDOW                       /**< Page scan window ( 0 to use default ) */
    },

    .ble_scan_cfg =                                                                                    /* BLE scan settings  */
    {
        .scan_mode                       = BTM_BLE_SCAN_MODE_ACTIVE,                                   /**< BLE scan mode ( BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE ) */

        /* Advertisement scan configuration */
        .high_duty_scan_interval         = 96,                                                         /**< High duty scan interval */
        .high_duty_scan_window           = 48,                                                         /**< High duty scan window */
        .high_duty_scan_duration         = 30,                                                         /**< High duty scan duration in seconds ( 0 for infinite ) */

        .low_duty_scan_interval          = 2048,                                                       /**< Low duty scan interval  */
        .low_duty_scan_window            = 48,                                                         /**< Low duty scan window */
        .low_duty_scan_duration          = 60,                                                         /**< Low duty scan duration in seconds ( 0 for infinite ) */

        /* Connection scan configuration */
        .high_duty_conn_scan_interval    = 96,                                                         /**< High duty cycle connection scan interval */
        .high_duty_conn_scan_window      = 48,                                                         /**< High duty cycle connection scan window */
        .high_duty_conn_duration         = 30,                                                         /**< High duty cycle connection duration in seconds ( 0 for infinite ) */

        .low_duty_conn_scan_interval     = 2048,                                                       /**< Low duty cycle connection scan interval */
        .low_duty_conn_scan_window       = 48,                                                         /**< Low duty cycle connection scan window */
        .low_duty_conn_duration          = 30,                                                         /**< Low duty cycle connection duration in seconds ( 0 for infinite ) */

        /* Connection configuration */
        .conn_min_interval               = 112,                                                        /**< Minimum connection interval */
        .conn_max_interval               = 128,                                                        /**< Maximum connection interval */
        .conn_latency                    = WICED_BT_CFG_DEFAULT_CONN_LATENCY,                          /**< Connection latency */
        .conn_supervision_timeout        = WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT,              /**< Connection link supervision timeout */
    },

    .ble_advert_cfg =                                                                                  /* BLE advertisement settings */
    {
        .channel_map                     = BTM_BLE_ADVERT_CHNL_37 |                                    /**< Advertising channel map ( mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39 ) */
                                           BTM_BLE_ADVERT_CHNL_38 |
                                           BTM_BLE_ADVERT_CHNL_39,

        .high_duty_min_interval          = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MIN_INTERVAL,            /**< High duty undirected connectable minimum advertising interval */
        .high_duty_max_interval          = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MAX_INTERVAL,            /**< High duty undirected connectable maximum advertising interval */
        .high_duty_duration              = 30,                                                         /**< High duty undirected connectable advertising duration in seconds ( 0 for infinite ) */

        .low_duty_min_interval           = WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MIN_INTERVAL,             /**< Low duty undirected connectable minimum advertising interval */
        .low_duty_max_interval           = WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MAX_INTERVAL,             /**< Low duty undirected connectable maximum advertising interval */
        .low_duty_duration               = 60,                                                         /**< Low duty undirected connectable advertising duration in seconds ( 0 for infinite ) */

        .high_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL,   /**< High duty directed connectable minimum advertising interval */
        .high_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL,   /**< High duty directed connectable maximum advertising interval */

        .low_duty_directed_min_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL,    /**< Low duty directed connectable minimum advertising interval */
        .low_duty_directed_max_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL,    /**< Low duty directed connectable maximum advertising interval */
        .low_duty_directed_duration      = 30,                                                         /**< Low duty directed connectable advertising duration in seconds ( 0 for infinite ) */

        .high_duty_nonconn_min_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,    /**< High duty non-connectable minimum advertising interval */
        .high_duty_nonconn_max_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,    /**< High duty non-connectable maximum advertising interval */
        .high_duty_nonconn_duration      = 30,                                                         /**< High duty non-connectable advertising duration in seconds ( 0 for infinite ) */

        .low_duty_nonconn_min_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,     /**< Low duty non-connectable minimum advertising interval */
        .low_duty_nonconn_max_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,     /**< Low duty non-connectable maximum advertising interval */
        .low_duty_nonconn_duration       = 0                                                           /**< Low duty non-connectable advertising duration in seconds ( 0 for infinite ) */
    },

    .gatt_cfg =                                                                                        /* GATT configuration */
    {
        .appearance                     = APPEARANCE_GENERIC_TAG,                                      /**< GATT appearance ( see gatt_appearance_e ) */
        .client_max_links               = WICED_MAX_LE_CLIENT_CONN,                                    /**< Client config: maximum number of servers that local client can connect to  */
        .server_max_links               = 1,                                                           /**< Server config: maximum number of remote clients connections allowed by the local */
        .max_attr_len                   = 360,                                                         /**< Maximum attribute length; gki_cfg must have a corresponding buffer pool that can hold this length */
#ifndef CYW20706A2
        .max_mtu_size                   = 365                                                          /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
#endif
    },

    .rfcomm_cfg =                                                                                      /* RFCOMM configuration */
    {
        .max_links                      = 0,                                                           /**< Maximum number of simultaneous RFCOMM ports */
        .max_links                      = 0                                                            /**< Maximum number of simultaneous RFCOMM connections */
    },


    .  l2cap_application =                                                                             /* Application managed l2cap protocol configuration */
    {
        .max_links                      = 0,                                                           /**< Maximum number of application-managed l2cap links (BR/EDR and LE) */

        /* BR EDR l2cap configuration */
        .max_psm                        = 0,                                                           /**< Maximum number of application-managed BR/EDR PSMs */
        .max_channels                   = 0,                                                           /**< Maximum number of application-managed BR/EDR channels  */

        /* LE L2cap connection-oriented channels configuration */
        .max_le_psm                     = 0,                                                           /**< Maximum number of application-managed LE PSMs */
        .max_le_channels                = 0,                                                           /**< Maximum number of application-managed LE channels */
#ifndef CYW20706A2
        .max_le_l2cap_fixed_channels    = 0,	                                                       /**< Maximum number of application managed fixed channels supported (in addition to mandatory channels 4, 5 and 6). > */
#endif
    },

    .avdt_cfg =                                                                                        /* Audio/Video Distribution configuration */
    {
        .max_links                      = 1,                                                           /**< Maximum simultaneous audio/video links */
#ifndef CYW20706A2
        .max_seps                       = 3,                                                           /**< Maximum number of stream end points */
#endif
    },

    . avrc_cfg =                                                                                       /* Audio/Video Remote Control configuration */
    {
        .roles                          = 1,                                                           /**< 1 if AVRC_CONN_ACCEPTOR is supported */
        .max_links                      = 1,                                                           /**< Maximum simultaneous remote control links */
    },

    .addr_resolution_db_size            = 5,                                                           /**< LE Address Resolution DB settings - effective only for pre 4.2 controller*/
#ifdef CYW20706A2
    .max_mtu_size                       = 517,                                                         /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
    .max_pwr_db_val                     = 12                                                           /**< Max. power level of the device */
#endif

#if ( defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20719B0) )
    .max_number_of_buffer_pools         = 7,                                                           /**< Maximum number of buffer pools in p_btm_cfg_buf_pools and by wiced_create_pool */
    .rpa_refresh_timeout                = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE,            /**< Interval of  random address refreshing - secs */
#endif

#if defined(CYW20735B0)
    .max_mtu_size                       = 517,                                                            /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
    .def_ble_pwr_level                  = 12                                                              /**< Max. power level of the device */
#endif

#if defined(CYW20719B2) || defined(CYW20721B2) || defined(CYW20735B1) || defined(CYW43012C0) || defined(CYW20819A1)
    .max_number_of_buffer_pools         = 7,                                                              /**< Maximum number of buffer pools in p_btm_cfg_buf_pools and by wiced_create_pool */

    /* Interval of  random address refreshing */
    .rpa_refresh_timeout                = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE,             /**< Interval of  random address refreshing - secs */
    /* BLE white list size */
    .ble_white_list_size                = 2,                                                               /**< Maximum number of white list devices allowed. Cannot be more than 128 */
#endif

#if defined(CYW20719B2) || defined(CYW20721B2) || defined(CYW20819A1)                                                            /**< Maximum number of white list devices allowed. Cannot be more than 128 */
    .default_ble_power_level            = 12                                                             /**< Default LE power level, Refer lm_TxPwrTable table for the power range */
#endif
};

#if (WICED_APP_AUDIO_SRC_INCLUDED == WICED_TRUE) && (WICED_APP_AUDIO_RC_TG_INCLUDED == WICED_TRUE) && (WICED_APP_AUDIO_RC_CT_INCLUDED == WICED_TRUE)
#define SLEN ((56 + 2) + (56 + 2) + (59 + 2) + (69 + 2))
#elif (WICED_APP_AUDIO_RC_TG_INCLUDED == WICED_TRUE) && (WICED_APP_AUDIO_RC_CT_INCLUDED == WICED_TRUE)
#define SLEN ((56 + 2) + (59 + 2) + (69 + 2))
#elif (WICED_APP_AUDIO_SRC_INCLUDED == WICED_TRUE) && (WICED_APP_AUDIO_RC_TG_INCLUDED == WICED_TRUE)
#define SLEN ((56 + 2) + (56 + 2) + (69 + 2))
#elif (WICED_APP_AUDIO_SRC_INCLUDED == WICED_TRUE) && (WICED_APP_AUDIO_RC_CT_INCLUDED == WICED_TRUE)
#define SLEN ((56 + 2) + (59 + 2) + (69 + 2))
#elif (WICED_APP_AUDIO_SRC_INCLUDED == WICED_TRUE)
#define SLEN ((56 + 2) + (69 + 2))  // AV
#elif (WICED_APP_AUDIO_RC_TG_INCLUDED == WICED_TRUE)
#define SLEN ((56 + 2) + (69 + 2))   // AVRC TG
#elif (WICED_APP_AUDIO_RC_CT_INCLUDED == WICED_TRUE)
#define SLEN ((59 + 2) + (69 + 2))   // AVRC CT
#else
#define SLEN (69 + 2)				// PNP
#endif

/*
 * This is the SDP database for the whole hci_control application
 */
const uint8_t wiced_app_cfg_sdp_record[] =
{
    // length is the sum of all records
    SDP_ATTR_SEQUENCE_2(SLEN),

#if (WICED_APP_AUDIO_SRC_INCLUDED == WICED_TRUE)
    // SDP Record for AVDT Source
    SDP_ATTR_SEQUENCE_1(56),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVDT_SOURCE),                         // 8 bytes
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_AUDIO_SOURCE),                     // 8 bytes
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),   // 3 + 2 bytes
            SDP_ATTR_SEQUENCE_1(6),                                         // 2 bytes
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),                       // 3 bytes
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVDTP),                         // 3 bytes
            SDP_ATTR_SEQUENCE_1(6),                                         // 2 bytes
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVDTP),                       // 3 bytes
                SDP_ATTR_VALUE_UINT2(0x103),                                // 3 bytes
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),  // 3 + 2 bytes
            SDP_ATTR_SEQUENCE_1(6),                                         // 2 bytes
                SDP_ATTR_UUID16(UUID_SERVCLASS_ADV_AUDIO_DISTRIBUTION),     // 3 bytes
                SDP_ATTR_VALUE_UINT2(0x103),                                // 3 bytes
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, 0x0001),
#endif

#if (WICED_APP_AUDIO_RC_TG_INCLUDED == WICED_TRUE)
    // SDP Record for AVRC Target
    SDP_ATTR_SEQUENCE_1(56),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVRC_TARGET),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),
            SDP_ATTR_SEQUENCE_1(3),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REM_CTRL_TARGET),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVCTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVCTP),
                SDP_ATTR_VALUE_UINT2(0x100),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_VALUE_UINT2(AVRC_PROFILE_VER),
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, AVRC_SUPF_TG_CAT1),
#endif

#if (WICED_APP_AUDIO_RC_CT_INCLUDED == WICED_TRUE)
    // SDP Record for AVRC Controller
    SDP_ATTR_SEQUENCE_1(59),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVRC_CONTROLLER),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REM_CTRL_CONTROL),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVCTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVCTP),
                SDP_ATTR_VALUE_UINT2(0x104),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_VALUE_UINT2(AVRC_REV_1_5),
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, AVRC_SUPF_CT_CAT2),
#endif

    // SDP record Device ID (total = 69 + 2 = 71)
    SDP_ATTR_SEQUENCE_1(69),                                                // 2 bytes, length of the record
        SDP_ATTR_RECORD_HANDLE(HANDLE_PNP),                                 // 8 bytes
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_PNP_INFORMATION),                  // 8
        SDP_ATTR_PROTOCOL_DESC_LIST(1),                                     // 18
        SDP_ATTR_UINT2(ATTR_ID_SPECIFICATION_ID, 0x103),                    // 6
        SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID, 0x0f),                            // 6
        SDP_ATTR_UINT2(ATTR_ID_PRODUCT_ID, 0x0201),                         // 6
        SDP_ATTR_UINT2(ATTR_ID_PRODUCT_VERSION, 0x0001),                    // 6
        SDP_ATTR_BOOLEAN(ATTR_ID_PRIMARY_RECORD, 0x01),                     // 5
        SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID_SOURCE, DI_VENDOR_ID_SOURCE_BTSIG),// 6
};

/*****************************************************************************
 * wiced_bt  buffer pool configuration
 *
 * Configure buffer pools used by the stack  according to application's requirement
 *
 * Pools must be ordered in increasing buf_size.
 * If a pool runs out of buffers, the next  pool will be used
 *****************************************************************************/
const wiced_bt_cfg_buf_pool_t wiced_app_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS] =
{
/*  { buf_size, buf_count } */
    { 64,       16   },      /* Small Buffer Pool */
    { 360,      4   },      /* Medium Buffer Pool (used for HCI & RFCOMM control messages, min recommended size is 360) */
#ifdef CYW20819A1
    { 360,      9   },      /* Large Buffer Pool  (used for HCI ACL messages) */
    { 1024,     3   },      /* Extra Large Buffer Pool - Used for avdt media packets and miscellaneous (if not needed, set buf_count to 0) */
#else
    { 1024,     9   },      /* Large Buffer Pool  (used for HCI ACL messages) */
    { 1024,     5   },      /* Extra Large Buffer Pool - Used for avdt media packets and miscellaneous (if not needed, set buf_count to 0) */
#endif
};

/**  Audio buffer configuration configuration */
const wiced_bt_audio_config_buffer_t wiced_bt_audio_buf_config = {
    .role                             =   WICED_AUDIO_SOURCE_ROLE,
    .audio_tx_buffer_size             =   AUDIO_TX_BUFFER_SIZE,
    .audio_codec_buffer_size          =   AUDIO_CODEC_BUFFER_SIZE
#if defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20819A1)
    ,.audio_tx_buffer_watermark_level =   50
#endif
};

/*
 * wiced_app_cfg_buf_pools_get_num
 */
const wiced_bt_cfg_settings_t *wiced_app_cfg_get_settings(void)
{
    return &wiced_bt_cfg_settings;
}

/*
 * wiced_app_cfg_buf_pools_get_num
 */
int wiced_app_cfg_buf_pools_get_num(void)
{
    return (int)sizeof(wiced_app_cfg_buf_pools)/sizeof(wiced_app_cfg_buf_pools[0]);
}

/*
 * wiced_app_cfg_buf_pools_get
 */
const wiced_bt_cfg_buf_pool_t *wiced_app_cfg_buf_pools_get(void)
{
    return wiced_app_cfg_buf_pools;
}

/*
 * wiced_app_cfg_sdp_record_get
 */
uint8_t *wiced_app_cfg_sdp_record_get(void)
{
    return (uint8_t *)wiced_app_cfg_sdp_record;
}

/*
 * wiced_app_cfg_sdp_record_get_size
 */
uint16_t wiced_app_cfg_sdp_record_get_size(void)
{
    return (uint16_t)sizeof(wiced_app_cfg_sdp_record);
}
