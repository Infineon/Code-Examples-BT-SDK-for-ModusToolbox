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
/*
 * File name: wiced_bt_cfg.c
 *
 * Description: This file defines various runtime Bluetooth stack configuration parameters
 *
 */

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_avrc.h"
#include "wiced_bt_cfg.h"

/* Null-Terminated Local Device Name */
uint8_t BT_LOCAL_NAME[] = { 'l','o','w','_','p','o','w','e','r','_','2','0','8','1','9','\0' };
const uint16_t BT_LOCAL_NAME_SIZE = sizeof(BT_LOCAL_NAME);


/*******************************************************************
 * wiced_bt core stack configuration
 ******************************************************************/
const wiced_bt_cfg_settings_t wiced_bt_cfg_settings =
{
    .device_name =                          (uint8_t*)BT_LOCAL_NAME,                                    /**< Local device name (NULL terminated) */
    .device_class =                         {0x00, 0x00, 0x00},                                         /**< Local device class */
    .security_requirement_mask =            BTM_SEC_NONE,                                               /**< Security requirements mask (BTM_SEC_NONE, or combination of BTM_SEC_IN_AUTHENTICATE, BTM_SEC_OUT_AUTHENTICATE, BTM_SEC_ENCRYPT */
    .max_simultaneous_links =               1,                                                          /**< Maximum number of simultaneous links to different devices */

    /* BR/EDR Scan Configuration */
    .br_edr_scan_cfg = {
        .inquiry_scan_type =                BTM_SCAN_TYPE_STANDARD,                                     /**< Inquiry Scan Type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        .inquiry_scan_interval =            0x0000,                                                     /**< Inquiry Scan Interval (0 to use default) */
        .inquiry_scan_window =              0x0000,                                                     /**< Inquiry Scan Window (0 to use default) */

        .page_scan_type =                   BTM_SCAN_TYPE_STANDARD,                                     /**< Page Scan Type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        .page_scan_interval =               0x0000,                                                     /**< Page Scan Interval (0 to use default) */
        .page_scan_window =                 0x0000,                                                     /**< Page Scan Window (0 to use default) */
    },

    /* BLE Scan Settings */
    .ble_scan_cfg = {
        .scan_mode =                        BTM_BLE_SCAN_MODE_PASSIVE,                                  /**< BLE Scan Mode (BTM_BLE_SCAN_MODE_PASSIVE or BTM_BLE_SCAN_MODE_ACTIVE) */

        /* Advertisement Scan Configuration */
        .high_duty_scan_interval =          WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_INTERVAL,               /**< High Duty Scan Interval */
        .high_duty_scan_window =            WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_WINDOW,                 /**< High Duty Scan Window */
        .high_duty_scan_duration =          5,                                                          /**< High Duty Scan Duration in seconds (0 for infinite) */

        .low_duty_scan_interval =           WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_INTERVAL,                /**< Low Duty Scan Interval */
        .low_duty_scan_window =             WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_WINDOW,                  /**< Low Duty Scan Window */
        .low_duty_scan_duration =           5,                                                          /**< Low Duty Scan Duration in seconds (0 for infinite) */

        /* Connection Scan Configuration */
        .high_duty_conn_scan_interval =     WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_INTERVAL,          /**< High Duty Connection Cycle Connection Scan Interval */
        .high_duty_conn_scan_window =       WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_WINDOW,            /**< High Duty Connection Cycle Connection Scan Window */
        .high_duty_conn_duration =          30,                                                         /**< High Duty Connection Cycle Connection Duration in seconds (0 for infinite) */

        .low_duty_conn_scan_interval =      WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_INTERVAL,           /**< Low Duty Connection Cycle Connection Scan Interval */
        .low_duty_conn_scan_window =        WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_WINDOW,             /**< Low Duty Connection Cycle Connection Scan Window */
        .low_duty_conn_duration =           30,                                                         /**< Low Duty Connection Cycle Connection Duration in seconds (0 for infinite) */

        /* Connection Configuration */
        .conn_min_interval =                WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL,                     /**< Minimum Connection Interval */
        .conn_max_interval =                WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL,                     /**< Maximum Connection Interval */
        .conn_latency =                     WICED_BT_CFG_DEFAULT_CONN_LATENCY,                          /**< Connection Latency */
        .conn_supervision_timeout =         WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT,              /**< Connection Link Supervision Timeout */
    },

    /* BLE Advertisement Settings */
    .ble_advert_cfg = {
        .channel_map =                      BTM_BLE_ADVERT_CHNL_37 |                                    /**< Advertising Channel Map (mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39) */
                                            BTM_BLE_ADVERT_CHNL_38 |
                                            BTM_BLE_ADVERT_CHNL_39,

        .high_duty_min_interval =           WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MIN_INTERVAL,            /**< High Duty Undirected Connectable Minimum Advertising Interval */
        .high_duty_max_interval =           WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MAX_INTERVAL,            /**< High Duty Undirected Connectable Maximum Advertising Interval */
        .high_duty_duration =               30,                                                         /**< High Duty Undirected Connectable Advertising Duration in seconds (0 for infinite) */

        .low_duty_min_interval =            WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MIN_INTERVAL,             /**< Low Duty Undirected Connectable Minimum Advertising Interval */
        .low_duty_max_interval =            WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MAX_INTERVAL,             /**< Low Duty Undirected Connectable Maximum Advertising Interval */
        .low_duty_duration =                0,                                                         /**< Low Duty Undirected Connectable Advertising Duration in seconds (0 for infinite) */

        .high_duty_directed_min_interval =  WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL,   /**< High Duty Directed Minimum Advertising Interval */
        .high_duty_directed_max_interval =  WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL,   /**< High Duty Directed Maximum Advertising Interval */

        .low_duty_directed_min_interval =   WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL,    /**< Low Duty Directed Minimum Advertising Interval */
        .low_duty_directed_max_interval =   WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL,    /**< Low Duty Directed Maximum Advertising Interval */
        .low_duty_directed_duration =       30,                                                         /**< Low Duty Directed Advertising Duration in seconds (0 for infinite) */

        .high_duty_nonconn_min_interval =   WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,    /**< High Duty Non-Connectable Minimum Advertising Interval */
        .high_duty_nonconn_max_interval =   WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,    /**< High Duty Non-Connectable Maximum Advertising Interval */
        .high_duty_nonconn_duration =       30,                                                         /**< High Duty Non-Connectable Advertising Duration in seconds (0 for infinite) */

        .low_duty_nonconn_min_interval =    WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,     /**< Low Duty Non-Connectable Minimum Advertising Interval */
        .low_duty_nonconn_max_interval =    WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,     /**< Low Duty Non-Connectable Maximum Advertising Interval */
        .low_duty_nonconn_duration =        0,                                                          /**< Low Duty Non-Connectable Advertising Duration in seconds (0 for infinite) */
    },

    /* GATT Configuration */
    .gatt_cfg = {
        .appearance =                       0x0000,                                                     /**< GATT Appearance */
        .client_max_links =                 1,                                                          /**< Client Config: Maximum number of servers that local client can connect to */
        .server_max_links =                 1,                                                          /**< Server Config: Maximum number of remote client connections allowed by local server */
        .max_attr_len =                     23,                                                         /**< Maximum attribute length; wiced_bt_cfg must have a corresponding buffer pool that can hold this length */
        .max_mtu_size =                     28,                                                         /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
    },

    /* RFCOMM Configuration */
    .rfcomm_cfg = {
        .max_links =                        0,                                                          /**< Maximum number of simultaneous connected remote devices */
        .max_ports =                        0,                                                          /**< Maximum Number of simultaneous RFCOMM ports */
    },

    /* Application-Managed L2CAP Protocol Configuration */
    .l2cap_application = {
        .max_links =                        0,                                                          /**< Maximum Number of Application-Managed L2CAP Links (BR/EDR and BLE) */
        .max_psm =                          0,                                                          /**< Maximum Number of Application-Managed BR/EDR PSMs */
        .max_channels =                     0,                                                          /**< Maximum Number of Application-Managed BR/EDR Channels */
        .max_le_psm =                       0,                                                          /**< Maximum Number of Application-Managed LE PSMs */
        .max_le_channels =                  0,                                                          /**< Maximum Number of Application-Managed LE Channels */
        .max_le_l2cap_fixed_channels =      0,                                                          /**< Maximum Number of Application-Managed LE L2CAP Fixed Channnels supported (in addition to mandatory channels 4, 5, and 6 */
    },

    /* Audio/Video Distribution Configuration */
    .avdt_cfg = {
        .max_links =                        0,                                                          /**< Maximum Number of simultaneous Audio/Video links */
        .max_seps =                         0,                                                          /**< Maximum Number of stream end points */
    },

    /* AVRC Configuration */
    .avrc_cfg = {
        .roles =                            0,                                                          /**< Local Roles supported (AVRC_CONN_INITIATOR or AVRC_CONN_ACCEPTOR) */
        .max_links =                        0,                                                          /**< Maximum simultaneous Remote Control links */
    },

    /* LE Address Resolution Database Settings */
    .addr_resolution_db_size =              10,                                                         /**< LE Address Resolution Database Size - Effective only for pre-4.2 controller */
    .max_number_of_buffer_pools =           4,                                                          /**< Maximum number of buffer pools in p_btm_cfg_buf_pools and by wiced_create_pool */
    .rpa_refresh_timeout =                  WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE,         /**< Interval of random address refreshing - secs */
};

/*******************************************************************
 * wiced_bt_stack buffer pool configuration
 *
 * Configure buffer pools used by the stack
 *
 * Pools must be ordered in increasing buf_size.
 * If a pools runs out of buffers, the next pool will be used.
 ******************************************************************/
const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS] =
{
/*  { buf_size, buf_count, }, */
    { 64,       12,        }, /* Small Buffer Pool */
    { 360,      4,         }, /* Medium Buffer Pool (used for HCI & RFCOMM control messages, min recommended size is 360) */
    { 512,      4,         }, /* Large Buffer Pool  (used for HCI ACL messages) */
    { 1024,     2,         }, /* Extra Large Buffer Pool (used for AVDT media packets and miscellaneous; if not needed, set buf_count to 0) */
};
