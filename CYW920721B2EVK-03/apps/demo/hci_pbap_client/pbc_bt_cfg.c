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
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_sdp.h"
#include "wiced_memory.h"
#include "pbc.h"


/*****************************************************************************
 * wiced_bt core stack configuration
 ****************************************************************************/

const wiced_bt_cfg_settings_t pbap_client_cfg_settings =
{
    ( uint8_t* )PBAP_CLIENT_NAME,                                   /**< Local device name ( NULL terminated ) */
    {0x24, 0x04, 0x38},                                             /**< Local device class */
    ( WICED_BT_PBC_SECURITY ),
    /**< Security requirements mask ( BTM_SEC_NONE, or combination of BTM_SEC_IN_AUTHENTICATE, BTM_SEC_OUT_AUTHENTICATE, BTM_SEC_ENCRYPT ( see #wiced_bt_sec_level_e ) ) */
    /**< Security requirements mask ( BTM_SEC_NONE, or combinination of BTM_SEC_IN_AUTHENTICATE, BTM_SEC_OUT_AUTHENTICATE, BTM_SEC_ENCRYPT ( see #wiced_bt_sec_level_e ) ) */

    3,                                                              /**< Maximum number simultaneous links to different devices */

    /* BR/EDR scan config */
    {
        BTM_SCAN_TYPE_STANDARD,                                     /**< Inquiry scan type ( BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED ) */
        WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_INTERVAL,                 /**< Inquiry scan interval  ( 0 to use default ) */
        WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_WINDOW,                   /**< Inquiry scan window ( 0 to use default ) */

        BTM_SCAN_TYPE_STANDARD,                                     /**< Page scan type ( BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED ) */
        WICED_BT_CFG_DEFAULT_PAGE_SCAN_INTERVAL,                    /**< Page scan interval  ( 0 to use default ) */
        WICED_BT_CFG_DEFAULT_PAGE_SCAN_WINDOW                       /**< Page scan window ( 0 to use default ) */
    },

    /* BLE scan settings  */
    {
        BTM_BLE_SCAN_MODE_ACTIVE,                                   /**< BLE scan mode ( BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE ) */

        /* Advertisement scan configuration */
        96,                 /**< High duty scan interval */
        48,                 /**< High duty scan window */
        30,                                                         /**< High duty scan duration in seconds ( 0 for infinite ) */

        2048,                /**< Low duty scan interval  */
        48,                  /**< Low duty scan window */
        30,                                                         /**< Low duty scan duration in seconds ( 0 for infinite ) */

        /* Connection scan configuration */
        96,            /**< High duty cycle connection scan interval */
        48,            /**< High duty cycle connection scan window */
        30,                                                         /**< High duty cycle connection duration in seconds ( 0 for infinite ) */

        2048,           /**< Low duty cycle connection scan interval */
        48,             /**< Low duty cycle connection scan window */
        30,                                                         /**< Low duty cycle connection duration in seconds ( 0 for infinite ) */

        /* Connection configuration */
        WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL,                     /**< Minimum connection interval */
        WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL,                     /**< Maximum connection interval */
        WICED_BT_CFG_DEFAULT_CONN_LATENCY,                          /**< Connection latency */
        WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT,              /**< Connection link supervision timeout */
    },

    /* BLE advertisement settings */
    {
        BTM_BLE_ADVERT_CHNL_37 |                                    /**< Advertising channel map ( mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39 ) */
        BTM_BLE_ADVERT_CHNL_38 |
        BTM_BLE_ADVERT_CHNL_39,

        WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MIN_INTERVAL,            /**< High duty undirected connectable minimum advertising interval */
        WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MAX_INTERVAL,            /**< High duty undirected connectable maximum advertising interval */
        30,                                                         /**< High duty undirected connectable advertising duration in seconds ( 0 for infinite ) */

        WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MIN_INTERVAL,             /**< Low duty undirected connectable minimum advertising interval */
        WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MAX_INTERVAL,             /**< Low duty undirected connectable maximum advertising interval */
        60,                                                         /**< Low duty undirected connectable advertising duration in seconds ( 0 for infinite ) */

        WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL,   /**< High duty directed connectable minimum advertising interval */
        WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL,   /**< High duty directed connectable maximum advertising interval */

        WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL,    /**< Low duty directed connectable minimum advertising interval */
        WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL,    /**< Low duty directed connectable maximum advertising interval */
        30,                                                         /**< Low duty directed connectable advertising duration in seconds ( 0 for infinite ) */

        WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,    /**< High duty non-connectable minimum advertising interval */
        WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,    /**< High duty non-connectable maximum advertising interval */
        30,                                                         /**< High duty non-connectable advertising duration in seconds ( 0 for infinite ) */

        WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,     /**< Low duty non-connectable minimum advertising interval */
        WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,     /**< Low duty non-connectable maximum advertising interval */
        0                                                           /**< Low duty non-connectable advertising duration in seconds ( 0 for infinite ) */
    },

    /* GATT configuration */
    {
        APPEARANCE_GENERIC_TAG,                                     /**< GATT appearance ( see gatt_appearance_e ) */
        3,                                                          /**< Client config: maximum number of servers that local client can connect to  */
        3,                                                          /**< Server config: maximum number of remote clients connections allowed by the local */
        512                                                         /**< Clinet config: maximum number of bytes that local client can reciver over LE link  */
    },

    /* RFCOMM configuration */
    {
        7,                                                          /**< Maximum number of simultaneous RFCOMM ports */
        7                                                           /**< Maximum number of simultaneous RFCOMM connections */
    },

    /* Application managed l2cap protocol configuration */
    {
        2,                                                          /**< Maximum number of application-managed l2cap links (BR/EDR and LE) */

        /* BR EDR l2cap configuration */
        7,                                                          /**< Maximum number of application-managed BR/EDR PSMs */
        7,                                                          /**< Maximum number of application-managed BR/EDR channels  */

        /* LE L2cap connection-oriented channels configuration */
        0,                                                          /**< Maximum number of application-managed LE PSMs */
        0,                                                          /**< Maximum number of application-managed LE channels */
    },

    /* Audio/Video Distribution configuration */
    {
        1,                                                          /**< Maximum simultaneous audio/video links */
    },

    /* Audio/Video Remote Control configuration */
    {
        1,                                                           /**< 1 if AVRC_CONN_ACCEPTOR is supported */
        1,                                                           /**< Maximum simultaneous remote control links */
    },
    5,                                                               /**< LE Address Resolution DB settings - effective only for pre 4.2 controller*/
#ifdef CYW20706A2
    517,                                                              /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5 )*/
    12
#else
    7,
    WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE,
#if ( defined(CYW43012C0) || defined(CYW20735B1) )
    2                                   /**< Maximum number of white list devices allowed. Cannot be more than 128 */
#endif
#endif
};


/*****************************************************************************
 * SDP database for the pbap_client application
 ****************************************************************************/

const uint8_t pbap_client_sdp_db[] = // Define SDP database
{
    SDP_ATTR_SEQUENCE_1(120),                                               // length is the sum of all records

    // SDP record for HF ( total length of record: 51 )
    SDP_ATTR_SEQUENCE_1( 49 ),                                              // 2 bytes, length of the record
        SDP_ATTR_RECORD_HANDLE( HDLR_HANDS_FREE_UNIT ),                     // 8 byte ( handle=0x10001 )
        SDP_ATTR_ID( ATTR_ID_SERVICE_CLASS_ID_LIST ),                       // 3 bytes
        SDP_ATTR_SEQUENCE_1( 6 ),                                           // 2 bytes
        SDP_ATTR_UUID16( UUID_SERVCLASS_HF_HANDSFREE ),                     // 3 bytes ServiceClass0 UUID_SERVCLASS_HF_HANDSFREE
        SDP_ATTR_UUID16( UUID_SERVCLASS_GENERIC_AUDIO ),                    // 3 bytes ServiceClass1 UUID_SERVCLASS_GENERIC_AUDIO
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST( HANDS_FREE_SCN ),               // 17 bytes ( SCN=2 )
        SDP_ATTR_PROFILE_DESC_LIST( UUID_SERVCLASS_HF_HANDSFREE, 0x0106 ),  // 13 bytes UUID_SERVCLASS_HF_HANDSFREE, version 0x0106

    // SDP Record for PBAP Client
    SDP_ATTR_SEQUENCE_1(67),                                                 // 2 bytes, length of the record
        SDP_ATTR_RECORD_HANDLE(HDLR_PBAP_CLIENT_UNIT),                       // 8 bytes
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),                          // 3 bytes
        SDP_ATTR_SEQUENCE_1(3),                                              // 2 bytes
        SDP_ATTR_UUID16(UUID_SERVCLASS_PBAP_PCE),                            // 3 bytes
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST(PBAP_CLIENT_SCN),                 // 17 bytes
        SDP_ATTR_BROWSE_LIST,                                                // 8 bytes
        SDP_ATTR_PROFILE_DESC_LIST(UUID_SERVCLASS_PHONE_ACCESS, 0x0102),     // 13 bytes
        SDP_ATTR_SERVICE_NAME(8),                                            // 13 bytes
            'P', 'B', 'A', 'P', ' ', 'P', 'C', 'E',
};


/*****************************************************************************
 * wiced_bt  buffer pool configuration
 *
 * Configure buffer pools used by the stack  according to application's requirement
 *
 * Pools must be ordered in increasing buf_size.
 * If a pool runs out of buffers, the next  pool will be used
 *****************************************************************************/

const wiced_bt_cfg_buf_pool_t pbap_client_cfg_buf_pools[] =
{
/*  { buf_size, buf_count } */
    { 64,      12  },      /* Small Buffer Pool */
    { 272,      6  },      /* Medium Buffer Pool (used for HCI & RFCOMM control messages, min recommended size is 360) */
    { 1056,     6  },      /* Large Buffer Pool  (used for HCI ACL messages) */
    { 1056,     1  },      /* Extra Large Buffer Pool - Used for avdt media packets and miscellaneous (if not needed, set buf_count to 0) */
};


/*
 * wiced_app_cfg_sdp_record_get_size
 */
uint16_t wiced_app_cfg_sdp_record_get_size(void)
{
    return (uint16_t)sizeof(pbap_client_sdp_db);
}
