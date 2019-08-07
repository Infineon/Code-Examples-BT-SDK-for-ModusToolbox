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
* Beacon sample
*
* This app demonstrates use of Google Eddystone and Apple iBeacons via the
* beacon library. It also demonstrates uses of multi-advertisement feature.
*
* During initialization the app configures advertisement packets for Eddystone and iBeacon
* and starts advertisements via multi-advertisement APIs.
* It also sets up a 1 sec timer to update Eddystone TLM advertisement data
*
* Features demonstrated
*  - configuring Apple iBeacon & Google Eddystone advertisements
*  - Apple iBeacon & Google Eddystone adv message will be advertised simultaneously.
*  - OTA Firmware Upgrade
*
* To demonstrate the app, work through the following steps.
*
* 1. Plug the WICED eval board into your computer
* 2. Build and download the application to the WICED board
* 3. Monitor advertisement packets -
*        - on Android, download  app such as 'Beacon Scanner' by Nicholas Briduox
*        - on iOS, download app such as 'Locate Beacon'. Add UUID for iBeacon (see below UUID_IBEACON)
*        - or use over the air sniffer
* 4. Run BTSpy app, capture protocol and snoop traces and view the view viewer such as Fronline
*
*/
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif
#include "wiced_bt_stack.h"
#include "wiced_timer.h"
#include "wiced_bt_beacon.h"
#include "string.h"
#include "sparcommon.h"
#include "GeneratedSource/cycfg_gatt_db.h"
#ifndef CYW43012C0
#include "wiced_bt_ota_firmware_upgrade.h"
#endif


/******************************************************************************
*                                Constants
******************************************************************************/
/* Multi advertisement instance ID */
#define BEACON_EDDYSTONE_UID 1
#define BEACON_EDDYSTONE_URL 2
#define BEACON_EDDYSTONE_EID 3
#define BEACON_EDDYSTONE_TLM 4
#define BEACON_IBEACON       5

/* User defined UUID for iBeacon */
#define UUID_IBEACON     0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f

#ifdef OTA_SECURE_FIRMWARE_UPGRADE
#include "bt_types.h"
#include "p_256_multprecision.h"
#include "p_256_ecc_pp.h"

// If secure version of the OTA firmware upgrade is used, the app should be linked with the ecdsa256_pub.c
// which exports the public key
extern Point    ecdsa256_public_key;
#endif
/******************************************************************************
 *                                Structures
 ******************************************************************************/

#if defined(CYW20735B1) || defined(CYW20819A1) || defined(CYW20719B2) || defined(CYW20721B2) || defined (WICEDX)
/* Adv parameter used for multi-adv*/
wiced_bt_ble_multi_adv_params_t adv_param =
#else
wiced_bt_beacon_multi_advert_data_t adv_param =
#endif
{
    .adv_int_min = BTM_BLE_ADVERT_INTERVAL_MIN,
    .adv_int_max = BTM_BLE_ADVERT_INTERVAL_MAX,
    .adv_type = MULTI_ADVERT_NONCONNECTABLE_EVENT,
    .channel_map = BTM_BLE_ADVERT_CHNL_37 | BTM_BLE_ADVERT_CHNL_38 | BTM_BLE_ADVERT_CHNL_39,
    .adv_filter_policy = BTM_BLE_ADVERT_FILTER_WHITELIST_CONNECTION_REQ_WHITELIST_SCAN_REQ,
#if defined(CYW20719B2) || defined(CYW20721B2)
    .adv_tx_power = MULTI_ADV_TX_POWER_MAX_INDEX,
#else
    .adv_tx_power = MULTI_ADV_TX_POWER_MAX,
#endif
    .peer_bd_addr = {0},
    .peer_addr_type = BLE_ADDR_PUBLIC,
    .own_bd_addr = {0},
    .own_addr_type = BLE_ADDR_PUBLIC
};

/******************************************************************************
 *                              Variables Definitions
 ******************************************************************************/
/* Beacon timer */
static wiced_timer_t beacon_timer;
uint16_t      beacon_conn_id = 0;

extern const wiced_bt_cfg_settings_t app_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t app_buf_pools[];
/******************************************************************************
 *                             Local Function Definitions
 ******************************************************************************/
static void                     beacon_init(void);
static wiced_result_t           beacon_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static void                     beacon_advertisement_stopped(void);
static wiced_bt_gatt_status_t   beacon_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
static void                     beacon_set_eddystone_ibecon_advertisement_data(void);

static wiced_bt_gatt_status_t   beacon_connection_status_event(wiced_bt_gatt_connection_status_t *p_status);
static wiced_bt_gatt_status_t   beacon_gatts_req_callback(wiced_bt_gatt_attribute_request_t *p_data);
static wiced_bt_gatt_status_t   beacon_gatts_req_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data);
static wiced_bt_gatt_status_t   beacon_gatts_req_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t *p_write_data);
static wiced_bt_gatt_status_t   beacon_indication_cfm_handler(uint16_t conn_id, uint16_t handle);

static void beacon_start_advertisement( void );
static void beacon_stop_advertisement(void);
static void beacon_set_timer(void);
static void beacon_data_update(uint32_t arg);
static void beacon_set_app_advertisement_data();

static void beacon_set_eddystone_uid_advertisement_data(void);
static void beacon_set_eddystone_url_advertisement_data(void);
static void beacon_set_eddystone_eid_advertisement_data(void);
static void beacon_set_eddystone_tlm_advertisement_data(void);
static void beacon_set_ibeacon_advertisement_data(void);

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
#ifdef CYW20735B0
void application_start(void)
#else
APPLICATION_START()
#endif
{
    wiced_bt_gatt_status_t gatt_status;
#ifdef WICED_BT_TRACE_ENABLE
    /*
     * Set the debug uart to enable the debug traces
     */

    /*
     * Sets the UART type as WICED_ROUTE_DEBUG_TO_PUART,
     * For disabling the traces set the UART type as WICED_ROUTE_DEBUG_NONE, and
     * For sending debug strings over the WICED debug interface set the UART type as WICED_ROUTE_DEBUG_TO_WICED_UART
     */

    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

    // To set to HCI to see traces on HCI uart -
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

#endif

    WICED_BT_TRACE("beacon_application_start\n");

    // Register call back and configuration with stack
    wiced_bt_stack_init(beacon_management_callback, &app_cfg_settings, app_buf_pools);
}

/*
 * This function is executed in the BTM_ENABLED_EVT management callback.
 */
void beacon_init(void)
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t         result;

#ifdef CYW20706A2
#if defined(USE_256K_SECTOR_SIZE)
    wiced_hal_sflash_use_erase_sector_size_256K(1);
    wiced_hal_sflash_use_4_byte_address(1);
#endif
#endif

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(beacon_gatts_callback);

    WICED_BT_TRACE("wiced_bt_gatt_register: %d\n", gatt_status);

    /*  Tell stack to use our GATT database */
    gatt_status =  wiced_bt_gatt_db_init(gatt_database, gatt_database_len);

    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\n", gatt_status);
    wiced_bt_dev_register_hci_trace(NULL);
    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

#ifndef CYW43012C0
    /* OTA Firmware upgrade Initialization */
#ifdef OTA_SECURE_FIRMWARE_UPGRADE
    if (!wiced_ota_fw_upgrade_init(&ecdsa256_public_key, NULL, NULL))
#else
    if (!wiced_ota_fw_upgrade_init(NULL, NULL, NULL))
#endif
    {
          WICED_BT_TRACE("OTA upgrade Init failure !!! \n");
    }
#endif
    /* Set the advertising params and make the device discoverable */
    beacon_set_app_advertisement_data();

    /* Fill the adv data and start advertisements */
    beacon_set_eddystone_ibecon_advertisement_data();
    beacon_start_advertisement();

    result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    WICED_BT_TRACE("wiced_bt_start_advertisements %d\n", result);
}

/*
 * This function set adv data
 */
static void beacon_set_eddystone_ibecon_advertisement_data()
{
    /* Set Google Eddystone adv data for each time of frame */
    beacon_set_eddystone_uid_advertisement_data();
    beacon_set_eddystone_url_advertisement_data();
    beacon_set_eddystone_eid_advertisement_data();
    beacon_set_eddystone_tlm_advertisement_data();

    /* Set Apple iBeacon adv data */
    beacon_set_ibeacon_advertisement_data();
}

/*
 * This function prepares Google Eddystone UID advertising data
 */
static void beacon_set_eddystone_uid_advertisement_data(void)
{
    uint8_t adv_data_uid[31];
    uint8_t adv_len_uid = 0;

    /* Set sample values for Eddystone UID*/
    uint8_t eddystone_ranging_data = 0xf0;
    uint8_t eddystone_namespace[EDDYSTONE_UID_NAMESPACE_LEN] = { 1,2,3,4,5,6,7,8,9,0 };
    uint8_t eddystone_instance[EDDYSTONE_UID_INSTANCE_ID_LEN] = { 0,1,2,3,4,5 };

    memset(adv_data_uid, 0, 31);

    /* Call Eddystone UID api to prepare adv data*/
    wiced_bt_eddystone_set_data_for_uid(eddystone_ranging_data, eddystone_namespace, eddystone_instance, adv_data_uid, &adv_len_uid);

    /* Sets adv data for multi adv instance*/
    wiced_set_multi_advertisement_data(adv_data_uid, adv_len_uid, BEACON_EDDYSTONE_UID);
}

/*
* This function prepares Google Eddystone URL advertising data
*/
static void beacon_set_eddystone_url_advertisement_data(void)
{
    uint8_t adv_data_url[31];
    uint8_t adv_len_url = 0;

    /* Set sample values for Eddystone URL*/
    uint8_t tx_power = 0x01;
    uint8_t urlscheme = EDDYSTONE_URL_SCHEME_0;
    uint8_t encoded_url[EDDYSTONE_URL_VALUE_MAX_LEN] = "cypress.com";

    memset(adv_data_url, 0, 31);

    /* Call Eddystone URL api to prepare adv data*/
    wiced_bt_eddystone_set_data_for_url(tx_power, urlscheme, encoded_url, adv_data_url, &adv_len_url);

    /* Sets adv data for multi adv instance*/
    wiced_set_multi_advertisement_data(adv_data_url, adv_len_url, BEACON_EDDYSTONE_URL);
}

/*
* This function prepares Google Eddystone EID advertising data
*/
static void beacon_set_eddystone_eid_advertisement_data(void)
{
    uint8_t adv_data_eid[31];
    uint8_t adv_len_eid = 0;

    /* Set sample values for Eddystone EID*/
    uint8_t eddystone_ranging_data = 0xf0;
    uint8_t eid[EDDYSTONE_EID_LEN] = { 0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8 };

    memset(adv_data_eid, 0, 31);

    /* Call Eddystone EID api to prepare adv data*/
    wiced_bt_eddystone_set_data_for_eid(eddystone_ranging_data, eid, adv_data_eid, &adv_len_eid);

    /* Sets adv data for multi adv instance*/
    wiced_set_multi_advertisement_data(adv_data_eid, adv_len_eid, BEACON_EDDYSTONE_EID);
}

/*
* This function prepares Google Eddystone TLM advertising data
*/
static uint32_t adv_cnt = 0;
static uint32_t sec_cnt = 0;
static void beacon_set_eddystone_tlm_advertisement_data(void)
{
    uint8_t adv_data_tlm[31];
    uint8_t adv_len_tlm = 0;

    /* Set sample values for Eddystone TLM */
    uint16_t vbatt = 10;
    uint16_t temp =  15;

    /* For each invocation of API, update Advertising PDU count and Time since power-on or reboot*/
    adv_cnt++;
    sec_cnt++;

    memset(adv_data_tlm, 0, 31);

    /* Call Eddystone TLM api to prepare adv data*/
    wiced_bt_eddystone_set_data_for_tlm_unencrypted(vbatt, temp, adv_cnt, sec_cnt, adv_data_tlm, &adv_len_tlm);

    /* Sets adv data for multi adv instance*/
    wiced_set_multi_advertisement_data(adv_data_tlm, adv_len_tlm, BEACON_EDDYSTONE_TLM);
}

/*
* This function prepares Apple iBeacon advertising data
*/
static void beacon_set_ibeacon_advertisement_data(void)
{
    uint8_t adv_data_ibeacon[31];
    uint8_t adv_len_ibeacon = 0;

    /* Set sample values for iBeacon */
    uint8_t ibeacon_uuid[LEN_UUID_128] = { UUID_IBEACON };
    uint16_t ibeacon_major_number = 0x01;
    uint16_t ibeacon_minor_number = 0x02;
    uint8_t tx_power_lcl = 0xb3;

    WICED_BT_TRACE("beacon_set_ibeacon_advertisement_data \n");

    /* Call iBeacon api to prepare adv data*/
    wiced_bt_ibeacon_set_adv_data(ibeacon_uuid, ibeacon_major_number, ibeacon_minor_number, tx_power_lcl,
            adv_data_ibeacon, &adv_len_ibeacon);

    /* Sets adv data for multi adv instance*/
    wiced_set_multi_advertisement_data(adv_data_ibeacon, adv_len_ibeacon, BEACON_IBEACON);
}


/*
* This function starts the advertisements.
*/
static void beacon_start_advertisement(void)
{
    /* Start Eddystone UID advertisements */
    adv_param.adv_int_min = 320; // 200 ms
    adv_param.adv_int_max = 320;
#if defined(CYW20735B1) || defined(CYW20819A1) || defined(CYW20719B2) || defined(CYW20721B2) || defined (WICEDX)
    wiced_set_multi_advertisement_params(BEACON_EDDYSTONE_UID, &adv_param);
#else
    wiced_set_multi_advertisement_params(adv_param.adv_int_min, adv_param.adv_int_max, adv_param.adv_type,
            adv_param.own_addr_type, adv_param.own_bd_addr, adv_param.peer_addr_type, adv_param.peer_bd_addr,
            adv_param.channel_map, adv_param.adv_filter_policy,
            BEACON_EDDYSTONE_UID, adv_param.adv_tx_power);
#endif

    wiced_start_multi_advertisements(MULTI_ADVERT_START, BEACON_EDDYSTONE_UID);

    /* Start Eddystone URL advertisements */
    adv_param.adv_int_min = 80; // 50 ms
    adv_param.adv_int_max = 80;
#if defined(CYW20735B1) || defined(CYW20819A1) || defined(CYW20719B2) || defined(CYW20721B2) || defined (WICEDX)
    wiced_set_multi_advertisement_params(BEACON_EDDYSTONE_URL, &adv_param);
#else
    wiced_set_multi_advertisement_params(adv_param.adv_int_min, adv_param.adv_int_max, adv_param.adv_type,
            adv_param.own_addr_type, adv_param.own_bd_addr, adv_param.peer_addr_type, adv_param.peer_bd_addr,
            adv_param.channel_map, adv_param.adv_filter_policy,
            BEACON_EDDYSTONE_URL, adv_param.adv_tx_power);
#endif

    wiced_start_multi_advertisements(MULTI_ADVERT_START, BEACON_EDDYSTONE_URL);

    /* Start Eddystone EID advertisements */
    adv_param.adv_int_min = 480; // 300 ms
    adv_param.adv_int_max = 480;
#if defined(CYW20735B1) || defined(CYW20819A1) || defined(CYW20719B2) || defined(CYW20721B2) || defined (WICEDX)
    wiced_set_multi_advertisement_params(BEACON_EDDYSTONE_EID, &adv_param);
#else
    wiced_set_multi_advertisement_params(adv_param.adv_int_min, adv_param.adv_int_max, adv_param.adv_type,
            adv_param.own_addr_type, adv_param.own_bd_addr, adv_param.peer_addr_type, adv_param.peer_bd_addr,
            adv_param.channel_map, adv_param.adv_filter_policy,
            BEACON_EDDYSTONE_EID, adv_param.adv_tx_power);
#endif

    wiced_start_multi_advertisements(MULTI_ADVERT_START, BEACON_EDDYSTONE_EID);

    /* Start iBeacon advertisements */
    adv_param.adv_int_min = 160; // 100 ms
    adv_param.adv_int_max = 160;
#if defined(CYW20735B1) || defined(CYW20819A1) || defined(CYW20719B2) || defined(CYW20721B2) || defined (WICEDX)
    wiced_set_multi_advertisement_params(BEACON_IBEACON, &adv_param);
#else
    wiced_set_multi_advertisement_params(adv_param.adv_int_min, adv_param.adv_int_max, adv_param.adv_type,
            adv_param.own_addr_type, adv_param.own_bd_addr, adv_param.peer_addr_type, adv_param.peer_bd_addr,
            adv_param.channel_map, adv_param.adv_filter_policy,
            BEACON_IBEACON, adv_param.adv_tx_power);
#endif
    wiced_start_multi_advertisements(MULTI_ADVERT_START, BEACON_IBEACON);

    /* start timer to change beacon ADV data */
    beacon_set_timer();

    WICED_BT_TRACE("beacon_start_advertisement \n");
}

/*
 * This function set a timer which will change Eddystone TLM advertising data
 * on every interval(1 sec)
 */
static void beacon_set_timer(void)
{
    wiced_result_t result = WICED_BT_SUCCESS;

    result = wiced_init_timer ( &beacon_timer,
                                beacon_data_update,
                                0, WICED_SECONDS_PERIODIC_TIMER);

    WICED_BT_TRACE("wiced_init_timer = %d\n", result);

    if ( result == WICED_BT_SUCCESS )
    {
        if ( wiced_start_timer( &beacon_timer, 1 ) )
        {
            WICED_BT_TRACE("started timer to update tlm data\n");
        }
    }
}

/* Function called on timer */
void beacon_data_update(uint32_t arg)
{
    /* Stops Eddystone TLM advertisements */
    wiced_start_multi_advertisements(MULTI_ADVERT_STOP, BEACON_EDDYSTONE_TLM);

    // Set Eddystone TLM adv data
    beacon_set_eddystone_tlm_advertisement_data();

    /* Sets adv data for multi adv instance*/
    adv_param.adv_int_min = 1280; // 800 ms
    adv_param.adv_int_max = 1280; // 800 ms
#if defined(CYW20735B1) || defined(CYW20819A1) || defined(CYW20719B2) || defined(CYW20721B2) || defined (WICEDX)
    wiced_set_multi_advertisement_params(BEACON_EDDYSTONE_TLM, &adv_param);
#else
    wiced_set_multi_advertisement_params(adv_param.adv_int_min, adv_param.adv_int_max, adv_param.adv_type,
            adv_param.own_addr_type, adv_param.own_bd_addr, adv_param.peer_addr_type, adv_param.peer_bd_addr,
            adv_param.channel_map, adv_param.adv_filter_policy,
            BEACON_EDDYSTONE_TLM, adv_param.adv_tx_power);
#endif
    /* Starts Eddystone TLM advertisements */
    wiced_start_multi_advertisements(MULTI_ADVERT_START, BEACON_EDDYSTONE_TLM);
}

/*
* This function stops the advertisements.
*/
static void beacon_stop_advertisement(void)
{
    /* Stop timer*/
    wiced_stop_timer( &beacon_timer);

    /* stop all advertisements */
    wiced_start_multi_advertisements(MULTI_ADVERT_STOP, BEACON_EDDYSTONE_UID);
    wiced_start_multi_advertisements(MULTI_ADVERT_STOP, BEACON_EDDYSTONE_URL);
    wiced_start_multi_advertisements(MULTI_ADVERT_STOP, BEACON_EDDYSTONE_EID);
    wiced_start_multi_advertisements(MULTI_ADVERT_STOP, BEACON_IBEACON);
    wiced_start_multi_advertisements(MULTI_ADVERT_STOP, BEACON_EDDYSTONE_TLM);
}

/*
 * Setup advertisement data with 16 byte UUID and device name
 */
void beacon_set_app_advertisement_data(void)
{
    wiced_bt_ble_advert_elem_t adv_elem[2];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((const char *)app_cfg_settings.device_name);
    adv_elem[num_elem].p_data       = (uint8_t*)app_cfg_settings.device_name;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

/*
 * This function is invoked when advertisements stop.  Continue advertising if there
 * are no active connections
 */
void beacon_advertisement_stopped(void)
{
    wiced_result_t result;

    // while we are not connected
    if (beacon_conn_id == 0)
    {
        result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);
        WICED_BT_TRACE("wiced_bt_start_advertisements: %d\n", result);
    }
    else
    {
        WICED_BT_TRACE("ADV stop\n");
    }
}

/*
 * Application management callback.  Stack passes various events to the function that may
 * be of interest to the application.
 */
wiced_result_t beacon_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                    result = WICED_BT_SUCCESS;
    uint8_t                          *p_keys;
    wiced_bt_ble_advert_mode_t       *p_mode;

    WICED_BT_TRACE("beacon_management_callback: %x\n", event);

    switch(event)
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
        beacon_init();
        break;

    case BTM_DISABLED_EVT:
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_BOND | BTM_LE_AUTH_REQ_MITM;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
        break;

    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        break;

     case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
         break;


     case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        p_mode = &p_event_data->ble_advert_state_changed;
        WICED_BT_TRACE("Advertisement State Change: %d\n", *p_mode);
        if (*p_mode == BTM_BLE_ADVERT_OFF)
        {
            beacon_advertisement_stopped();
        }
        break;

    default:
        break;
    }

    return result;
}

/*
 * Process Read request or command from peer device
 */
wiced_bt_gatt_status_t beacon_gatts_req_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data)
{
    int to_copy;

#ifndef CYW43012C0
    // if read request is for the OTA FW upgrade service, pass it to the library to process
    if (p_read_data->handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        return wiced_ota_fw_upgrade_read_handler(conn_id, p_read_data);
    }
#endif
    switch(p_read_data->handle)
    {
    case HDLC_GAP_DEVICE_NAME_VALUE:
        if (p_read_data->offset >= strlen((const char *)app_cfg_settings.device_name))
            return WICED_BT_GATT_INVALID_OFFSET;

        to_copy = strlen((const char *)app_cfg_settings.device_name) - p_read_data->offset;
        if (*p_read_data->p_val_len < to_copy)
            to_copy = *p_read_data->p_val_len;

        memcpy(p_read_data->p_val, app_cfg_settings.device_name + p_read_data->offset, to_copy);
        *p_read_data->p_val_len = to_copy;
        break;

    case HDLC_GAP_APPEARANCE_VALUE:
        if (p_read_data->offset >= 2)
            return WICED_BT_GATT_INVALID_OFFSET;

        to_copy = 2 - p_read_data->offset;
        if (*p_read_data->p_val_len < to_copy)
            to_copy = *p_read_data->p_val_len;

        memcpy(p_read_data->p_val, ((uint8_t*)&app_cfg_settings.gatt_cfg.appearance) + p_read_data->offset, to_copy);
        *p_read_data->p_val_len = to_copy;
        break;

    default:
        return WICED_BT_GATT_INVALID_HANDLE;
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process write request or write command from peer device
 */
wiced_bt_gatt_status_t beacon_gatts_req_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t *p_write_data)
{
#ifndef CYW43012C0
    // if write request is for the OTA FW upgrade service, pass it to the library to process
    if (p_write_data->handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        return wiced_ota_fw_upgrade_write_handler(conn_id, p_write_data);
    }
#endif

    return WICED_BT_GATT_INVALID_HANDLE;
}

/*
 * Process indication_confirm from peer device
 */
wiced_bt_gatt_status_t beacon_indication_cfm_handler(uint16_t conn_id, uint16_t handle)
{
#ifndef CYW43012C0
    // if indication confirmation is for the OTA FW upgrade service, pass it to the library to process
    if (handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        return wiced_ota_fw_upgrade_indication_cfm_handler(conn_id, handle);
    }
#endif

    return WICED_BT_GATT_INVALID_HANDLE;
}

/*
 * Connection up/down event
 */
wiced_bt_gatt_status_t beacon_connection_status_event(wiced_bt_gatt_connection_status_t *p_status)
{
    wiced_result_t result;

    if (p_status->connected)
    {
        beacon_conn_id = p_status->conn_id;
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
    }
    else
    {
        beacon_conn_id = 0;
        result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
        WICED_BT_TRACE("[%s] start adv status %d \n", __FUNCTION__, result);
    }
#ifndef CYW43012C0
    // Pass connection up/down event to the OTA FW upgrade library
    wiced_ota_fw_upgrade_connection_status_event(p_status);
#endif
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process GATT request from the peer
 */
wiced_bt_gatt_status_t beacon_gatts_req_callback(wiced_bt_gatt_attribute_request_t *p_data)
{
    wiced_result_t result = WICED_BT_GATT_INVALID_PDU;

    switch (p_data->request_type)
    {
    case GATTS_REQ_TYPE_READ:
        result = beacon_gatts_req_read_handler(p_data->conn_id, &p_data->data.read_req);
        break;

    case GATTS_REQ_TYPE_WRITE:
    case GATTS_REQ_TYPE_PREP_WRITE:
        result = beacon_gatts_req_write_handler(p_data->conn_id, &p_data->data.write_req);
        break;

    case GATTS_REQ_TYPE_WRITE_EXEC:
        result = WICED_BT_GATT_SUCCESS;
        break;

    case GATTS_REQ_TYPE_MTU:
        result = WICED_BT_GATT_SUCCESS;
        break;

    case GATTS_REQ_TYPE_CONF:
        result = beacon_indication_cfm_handler(p_data->conn_id, p_data->data.handle);
        break;

   default:
        WICED_BT_TRACE("%s: Unsupported type: %d\n", __func__, p_data->request_type);
        break;
    }
    return result;
}

/*
 * Callback for various GATT events.  As this application performs only as a GATT server, some of the events are ommitted.
 */
wiced_bt_gatt_status_t beacon_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch (event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = beacon_connection_status_event(&p_data->connection_status);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = beacon_gatts_req_callback(&p_data->attribute_request);
        break;

    default:
        break;
    }
    return result;
}

/* Note for OTA support - The handles for OTA services should be defined as below in cycfg_gatt_db.h. If the
   application is updated with Bluetooth Configurator, ensure the handles are set as below.
   Also the OTA service should be last service in the GATT database.

// Service FWUpgradeService
#define HDLS_FWUPGRADESERVICE                                            HANDLE_OTA_FW_UPGRADE_SERVICE
// Characteristic FWUpgradeContolPoint
#define HDLC_FWUPGRADESERVICE_FWUPGRADECONTOLPOINT                       HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT
#define HDLC_FWUPGRADESERVICE_FWUPGRADECONTOLPOINT_VALUE                 HANDLE_OTA_FW_UPGRADE_CONTROL_POINT
// Descriptor Client Characteristic Configuration
#define HDLD_FWUPGRADESERVICE_FWUPGRADECONTOLPOINT_CLIENT_CHAR_CONFIG    HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR
// Characteristic FWUpgradeData
#define HDLC_FWUPGRADESERVICE_FWUPGRADEDATA                              HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_DATA
#define HDLC_FWUPGRADESERVICE_FWUPGRADEDATA_VALUE                        HANDLE_OTA_FW_UPGRADE_DATA
*/