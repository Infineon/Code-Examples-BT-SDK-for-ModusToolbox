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
 * File name: tput.c
 *
 * Description: Main file which handles BLE management events, app timers,
 *              send GATT notifications and GATT read/write operations.
 *
 * Features demonstrated:
 *  - GATT throughput measurement by sending repeated GATT notifications.
 *  - LE Connection-Oriented-Channel (COC) throughput support is disabled
 *    by default in this app. Enable LE COC on Modus IDE by Right clicking
 *    on project -> "Change Application Settings" -> LE_COC_SUPPORT.
 *  - PHY 1Mbps/2Mbps switching.
 *
 *  Controls:
 *  - Toggle 1M/2M PHY through user button1.
 *  - Configure connection interval in CySmart desktop app.
 *
 *  LED Behavior:
 *  - LED 2: OFF by default; ON when GAP connected.
 *  - LED 1: OFF by default; ON during GATT notifications or LE CoC bytes transmission;
 *           OFF when pkt congestion or error during transmit happened.
 *
 *  User Button 1:
 *  - 1M PHY by default.
 *  - Alternately set to 1M/2M PHY on every button press if the peer supports both speeds.
 */

#include "wiced_bt_dev.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_gpio.h"
#include "wiced_transport.h"
#include "wiced_bt_l2c.h"
#include "wiced_bt_trace.h"
#ifdef LE_COC_SUPPORT
#include "le_coc/tput_le_coc.h"
#endif
#include "GeneratedSource/cycfg_pins.h"
#include "sparcommon.h"
#include "wiced_result.h"
#include "GeneratedSource/cycfg_gatt_db.h"
#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#include "tput_util.h"
#include "wiced_timer.h"

/*****************************************************************************
 *                              EXTERNS and FUNCTIONS
 *****************************************************************************/
#ifdef LE_COC_SUPPORT
extern void                     le_coc_init( void );
extern void                     tput_send_l2cap_coc_pkt( void );
extern uint8_t                  wiced_bt_get_number_of_buffer_pools( void );
#endif
static wiced_bt_gatt_status_t   tput_gatts_callback( wiced_bt_gatt_evt_t, wiced_bt_gatt_event_data_t* );
wiced_bt_dev_status_t           tput_management_callback( wiced_bt_management_evt_t, wiced_bt_management_evt_data_t* );
static void                     tput_app_sec_timeout( uint32_t );
static void                     tput_app_msec_timeout( uint32_t );
static void                     tput_btn_interrupt_handler( void*, uint8_t );
static void                     tput_set_advertisement_data( void );
static void                     tput_send_notification( void );
void                            tput_init( void );
void                            tput_advertisement_stopped( void );

/******************************************************
 *                    Structures
 ******************************************************/
#pragma pack(1)
typedef PACKED struct
{
    BD_ADDR     bdaddr; // BD address of the bonded host
    uint16_t    characteristic_client_configuration; // Current value of the client configuration descriptor
} host_info_t;
#pragma pack()

typedef struct
{
    BD_ADDR     remote_addr;  // remote peer device address
    uint16_t    conn_id;      // connection ID referenced by the stack
    uint16_t    peer_mtu;     // peer MTU
} tput_conn_state_t;

extern const wiced_bt_cfg_settings_t     tput_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t     tput_bt_cfg_buf_pools[];
extern const wiced_platform_led_config_t platform_led[];

/******************************************************************************
 *                           Constants and Enumerations
 ******************************************************************************/
#define DUMP_BUFFER_STATS                   0   // Disabled by default. Needed for buffer pool tuning
#define CONNECTION_INTERVAL                 21  // (21 * 1.25) = 26.25ms
#define GATT_NOTIFY_BYTES_LEN               244 // (251B Max DLE TX Length - 4B L2CAP_HEADER - 3B ATT_HEADER)
#define ADV_ELEM_LEN                        3   // GATT advertisement element length
#define MINIMUM_TX_BUFFER_LEN               1   // Minimum tx buffer length
#define BTN_PRESS_EVEN                      2   // Even time button press
#define BLE_CONN_TIMEOUT                    512 // BLE connection timeout value
#define CONN_INTERVAL_MAJOR(a)              (a / 100)
#define CONN_INTERVAL_MINOR(a)              (a % 100)
#define CONN_INTERVAL_MULTIPLIER            125
#define ONE_BYTE                            8
#define ONE_SEC_TIME                        1
#define ONE_MS_TIME                         1
#define GATT_CONNECT_LED                    WICED_GET_PIN_FOR_LED(2) //LED2(D2 on kit)
#define CONGESTION_LED                      WICED_GET_PIN_FOR_LED(1) //LED1(D1 on kit)

/**************************************************************************************
 *                                Variable Definitions
 *************************************************************************************/
uint16_t             tput_mtu               = 0; // local variable to keep track of peer's mtu
uint32_t             numBytesTx             = 0; // no of bytes transferred successfully as GATT notifications
uint32_t             failed_to_send         = 0; // no of bytes rejected for transfer
uint8_t              phy_1m_2m              = 1; // PHY config selected by the application(value: 1 - 1M; 2 - 2M)
uint8_t              phy_config             = 1; // PHY config returned from the stack event after app has set
wiced_timer_t        tput_app_sec_timer;         // application seconds timer
wiced_timer_t        tput_app_msec_timer;        // application milliseconds timer
uint8_t              phy_configure          = 1; // configure phy on every button1 press
tput_conn_state_t    tput_conn_state;
host_info_t          tput_hostinfo;
uint8_t              tput_char_notify_value[GATT_NOTIFY_BYTES_LEN]  = {0};

/*
 * Transport PUART and HCI configuration.
 */
const wiced_transport_cfg_t  transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg.uart_cfg = { .mode = WICED_TRANSPORT_UART_HCI_MODE, .baud_rate = 3000000 },
    .rx_buff_pool_cfg = { .buffer_size = 0, .buffer_count = 0 },
    .p_status_handler = NULL,
    .p_data_handler = NULL,
    .p_tx_complete_cback = NULL
};

/*************************************************************************************
 *                                Function Definitions
 *************************************************************************************/
/*******************************************************************************
* Function Name: void application_start(void)
********************************************************************************
* Summary: Initialize transport configuration and register BLE
*          management event callback.
*
* Parameters:
*   None
*
* Return:
*  None
*
********************************************************************************/
APPLICATION_START()
{
    wiced_transport_init(&transport_cfg);

    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);

    /* Initialize Bluetooth controller and host stack */
    wiced_bt_stack_init(tput_management_callback, &tput_bt_cfg_settings,
                                                 tput_bt_cfg_buf_pools);

    WICED_BT_TRACE("\r[%s]  ***** THROUGHPUT APPLICATION ***** \n", __func__);
}

/**********************************************************************************************
* Function Name: wiced_result_t tput_management_callback(wiced_bt_management_evt_t event,
*                                           wiced_bt_management_evt_data_t *p_event_data)
***********************************************************************************************
* Summary:
*   This is a Bluetooth management event handler function to receive events from
*   BLE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : BLE event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to BLE management event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
***********************************************************************************************/
wiced_result_t tput_management_callback(wiced_bt_management_evt_t event,
                                        wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_dev_ble_pairing_info_t *p_info;
    wiced_bt_ble_advert_mode_t  *p_mode;
    wiced_bt_device_address_t bda;
    uint8_t *p_keys;
    uint16_t conn_interval = 0;

#ifdef VERBOSE_THROUGHPUT_OUTPUT
    WICED_BT_TRACE("\r[%s] Received %s event from stack \n", __func__, getStackEventStr(event));
#endif

    switch (event)
    {
        case BTM_ENABLED_EVT:
            /* Init TPUT App */
            tput_init();
#ifdef LE_COC_SUPPORT
            le_coc_init();
#endif
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS,
                                           p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
#ifdef VERBOSE_THROUGHPUT_OUTPUT
            WICED_BT_TRACE("\rPassKey Notification. BDA %B, Key %d \n",
                            p_event_data->user_passkey_notification.bd_addr,
                            p_event_data->user_passkey_notification.passkey);
#endif
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS,
                                           p_event_data->user_passkey_notification.bd_addr);
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE; /* No IO capabilities on this platform */
            p_event_data->pairing_io_capabilities_ble_request.oob_data     = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req     = BTM_LE_AUTH_REQ_NO_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys    = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys    = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_info =  &p_event_data->pairing_complete.pairing_complete_info.ble;
            wiced_bt_get_identity_address (tput_conn_state.remote_addr, bda );
            WICED_BT_TRACE("\rPairing Complete: %B %d\n", bda, p_info->reason);
            break;

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
            WICED_BT_TRACE("\rScan %s \n", (BTM_BLE_SCAN_TYPE_NONE !=
                              p_event_data->ble_scan_state_changed) ?
                                                "started":"stopped");
            break;

        case BTM_SECURITY_REQUEST_EVT:
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            p_mode = &p_event_data->ble_advert_state_changed;
            WICED_BT_TRACE("\rAdvertisement State Change: %s\n", getAdvStatusStr(*p_mode));
            if (*p_mode == BTM_BLE_ADVERT_OFF)
            {
                tput_advertisement_stopped();
            }
            break;

        case BTM_BLE_PHY_UPDATE_EVT:
#ifdef VERBOSE_THROUGHPUT_OUTPUT
            WICED_BT_TRACE("\rReceived BTM_BLE_PHY_UPDATE_EVT: PHY status: "
                                             "%d, RX_PHY: %d, TX_PHY: %d\n",
                                  p_event_data->ble_phy_update_event.status,
                                  p_event_data->ble_phy_update_event.rx_phy,
                                  p_event_data->ble_phy_update_event.tx_phy);
#endif
            phy_config = p_event_data->ble_phy_update_event.tx_phy;
            if (phy_config == phy_1m_2m)
            {
                WICED_BT_TRACE("\rPHY switched to %dM successfully\n", phy_1m_2m);
            }
            else
            {
                WICED_BT_TRACE("\rSelected PHY - %dM\n", phy_config);
            }
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            /* Connection parameters updated */
            if(WICED_SUCCESS == p_event_data->ble_connection_param_update.status)
            {
                conn_interval = (p_event_data->ble_connection_param_update.conn_interval) * CONN_INTERVAL_MULTIPLIER;
                WICED_BT_TRACE("\rNew connection parameters:"
                               "\n\rConnection interval = %d.%dms\n",
                               CONN_INTERVAL_MAJOR(conn_interval), CONN_INTERVAL_MINOR(conn_interval));
            }
            else
            {
#ifdef VERBOSE_THROUGHPUT_OUTPUT
                WICED_BT_TRACE("\rConnection parameters update failed\n");
#endif
            }
            break;

        default:
#ifdef VERBOSE_THROUGHPUT_OUTPUT
            WICED_BT_TRACE("\r[%s] received event (%s) not processed \n", __func__,
                                                          getStackEventStr(event));
#endif
            break;
    }

    return (result);
}

/*********************************************************************
* Function Name: void tput_init(void)
**********************************************************************
* Summary:
*   This function initializes/registers button interrupt, app timers,
*   gatt database and starts undirected advertisement HIGH.
*
* Parameters:
*   None
*
* Return:
*   None
*
**********************************************************************/
void tput_init(void)
{
    wiced_bt_gatt_status_t gatt_status;

    /* configure button interrupt callback */
    wiced_platform_register_button_callback(WICED_PLATFORM_BUTTON_1,
                                            tput_btn_interrupt_handler,
                                            NULL,
                                            WICED_PLATFORM_BUTTON_RISING_EDGE);

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(tput_gatts_callback);
    if (WICED_SUCCESS != gatt_status)
    {
        WICED_BT_TRACE("\r GATT registration failed\n");
    }

    /*  Tell stack to use our GATT databse */
    gatt_status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len);
    if (WICED_SUCCESS != gatt_status)
    {
        WICED_BT_TRACE("\r GATT database init failed\n");
    }

    /* start app timers */
    if (wiced_init_timer(&tput_app_sec_timer, tput_app_sec_timeout, 0,
                         WICED_SECONDS_PERIODIC_TIMER) == WICED_SUCCESS)
    {
        if (wiced_start_timer(&tput_app_sec_timer, ONE_SEC_TIME) != WICED_BT_SUCCESS)
        {
#ifdef VERBOSE_THROUGHPUT_OUTPUT
            WICED_BT_TRACE("\rTPUT: seconds timer start failed\n");
#endif
        }
    }
    else
    {
#ifdef VERBOSE_THROUGHPUT_OUTPUT
        WICED_BT_TRACE("\rTPUT: seconds timer init failed\n");
#endif
    }
    if (wiced_init_timer(&tput_app_msec_timer, tput_app_msec_timeout, 0,
                         WICED_MILLI_SECONDS_PERIODIC_TIMER) == WICED_SUCCESS)
    {
        if (wiced_start_timer(&tput_app_msec_timer, ONE_MS_TIME) != WICED_BT_SUCCESS)
        {
#ifdef VERBOSE_THROUGHPUT_OUTPUT
            WICED_BT_TRACE("\rTPUT: millisecond timer start failed\n");
#endif
        }
    }
    else
    {
#ifdef VERBOSE_THROUGHPUT_OUTPUT
        WICED_BT_TRACE("\rTPUT: millisecond timer init failed\n");
#endif
    }

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* Set the advertising params and make the device discoverable */
    tput_set_advertisement_data();

    wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, BLE_ADDR_PUBLIC, NULL);
}

/*********************************************************************
* Function Name: void tput_set_advertisement_data(void)
**********************************************************************
* Summary:
*   Setup advertisement data with 16 byte UUID and device name.
*
* Parameters:
*   None
*
* Return:
*   None
*
**********************************************************************/
void tput_set_advertisement_data(void)
{
    wiced_bt_ble_advert_elem_t adv_elem[ADV_ELEM_LEN];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t tput_service_uuid[LEN_UUID_128] = {HDLS_TPUT};

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE;
    adv_elem[num_elem].len          = LEN_UUID_128;
    adv_elem[num_elem].p_data       = tput_service_uuid;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((const char *)tput_bt_cfg_settings.device_name);
    adv_elem[num_elem].p_data       = (uint8_t *)tput_bt_cfg_settings.device_name;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

/*********************************************************************
* Function Name: static void tput_send_notification(void)
**********************************************************************
* Summary:
*   Send GATT notification every millisecond.
*
* Parameters:
*   None
*
* Return:
*   None
*
**********************************************************************/
static void tput_send_notification(void)
{
    wiced_bt_gatt_status_t  status;
    uint32_t buf_left;

    buf_left = wiced_bt_ble_get_available_tx_buffers();
    if (buf_left > MINIMUM_TX_BUFFER_LEN)
    {
        status = wiced_bt_gatt_send_notification(tput_conn_state.conn_id,
                                                  HDLC_TPUT_NOTIFY_VALUE,
                                          sizeof(tput_char_notify_value),
                                                 tput_char_notify_value);
        if (status == WICED_BT_GATT_SUCCESS)
        {
            numBytesTx += sizeof(tput_char_notify_value);
            /* LED 1 ON */
            wiced_hal_gpio_set_pin_output(CONGESTION_LED, GPIO_PIN_OUTPUT_LOW);
        }
        else
        {
#ifdef VERBOSE_THROUGHPUT_OUTPUT
            WICED_BT_TRACE("\rTPUT: GATT tput_send_notification status=FAILED\n");
#endif
            /* LED 1 OFF since TX failed */
            wiced_hal_gpio_set_pin_output(CONGESTION_LED, GPIO_PIN_OUTPUT_HIGH);
        }
    }
    else
    {
        failed_to_send++;
        /* LED 1 OFF since TX failed */
        wiced_hal_gpio_set_pin_output(CONGESTION_LED, GPIO_PIN_OUTPUT_HIGH);
    }
}

/****************************************************************************
* Function Name: void tput_app_sec_timeout(uint32_t unused)
*****************************************************************************
* Summary:
*   Five second timer callback. Check and display the LE COC connection status
*   and GATT notifications status message on the console.
*
* Parameters:
*   uint32_t unused: Variable registered by wiced_init_timer to pass to
*                    timer callback. Not used for now.
*
* Return:
*   None
*
****************************************************************************/
void tput_app_sec_timeout(uint32_t unused)
{
    // Display GATT throughput result
    if ((tput_conn_state.conn_id) &&
        (tput_hostinfo.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION))
    {
        WICED_BT_TRACE("\rTPUT: ######### GATT TPUT: %d Bytes per second. PHY: %dM ######\n",
                        numBytesTx, phy_config);
        numBytesTx      = 0; // Reset the byte counter
        failed_to_send  = 0; // Reset rejected byte counter
    }

#ifdef LE_COC_SUPPORT
    // Display LE COC throughput result
    if (le_coc_cb.local_cid && le_coc_cb.local_cid < MAX_LOCAL_CID)
    {
        WICED_BT_TRACE("\rTPUT: ######### LE COC TPUT: %d Bytes per second. PHY: %dM ######\n",
                        numBytesTx_coc, phy_config);
        numBytesTx_coc      = 0; // Reset the byte counter
        failed_to_send_coc  = 0; // Reset rejected byte counter

#if DUMP_BUFFER_STATS //Dump dynamic buffer use statistics
        wiced_bt_buffer_statistics_t buff_stats[wiced_bt_get_number_of_buffer_pools()];
        uint8_t i;
        if (wiced_bt_get_buffer_usage(buff_stats, sizeof(buff_stats)) == WICED_BT_SUCCESS )
        {
            WICED_BT_TRACE("\r");
            for(i = 0; i < wiced_bt_get_number_of_buffer_pools(); i++)
            {
                WICED_BT_TRACE("\rpool: id:%d size:%d curr_cnt:%d max_cnt:%d total:%d \n",
                                  buff_stats[i].pool_id, buff_stats[i].pool_size,
                                  buff_stats[i].current_allocated_count,
                                  buff_stats[i].max_allocated_count,
                                  buff_stats[i].total_count);
            }
        }
#endif //Dump dynamic buffer statistics
    }
#endif
}

/****************************************************************************
* Function Name: void tput_app_msec_timeout( uint32_t unused )
*****************************************************************************
* Summary:
*   One ms timer callback. Check and send GATT notifications and
*   LE COC bytes every ms.
*
* Parameters:
*   uint32_t unused: Variable registered by wiced_init_timer to pass to
*                    timer callback. Not used for now.
*
* Return:
*   None
*
****************************************************************************/
void tput_app_msec_timeout(uint32_t unused)
{
    /* Send GATT Notification */
    if (tput_hostinfo.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION)
    {
        //sending GATT notification.
        tput_send_notification();
    }

#ifdef LE_COC_SUPPORT
    /* Send LE COC bytes*/
    if (le_coc_cb.local_cid && le_coc_cb.local_cid < MAX_LOCAL_CID)
    {
        // send LE COC pkts.
        tput_send_l2cap_coc_pkt();
    }
#endif //LE_COC_SUPPORT
}

/*******************************************************************************
* Function Name: void tput_btn_interrupt_handler(void* user_data, uint8_t value)
********************************************************************************
* Summary:
*   User button 1 interrupt callback function.
*   Switch between 1M and 2M PHY.
*
* Parameters:
*   void* unused_user_data  : Not used.
*   uint8_t unused          : Not used.
*
* Return:
*   None
*
*******************************************************************************/
void tput_btn_interrupt_handler(void* unused_user_data, uint8_t unused)
{
    wiced_bt_ble_phy_preferences_t phy_preferences;
    wiced_bt_dev_status_t result;

    phy_configure = !phy_configure;
    /*
     * Configure phy to 1M or 2M.
     * phy_preferences.tx_phys = 1 for 1M PHY
     * phy_preferences.tx_phys = 2 for 2M PHY
     */
    phy_preferences.rx_phys = phy_preferences.tx_phys = (1 << (phy_configure));

    memcpy(phy_preferences.remote_bd_addr, tput_conn_state.remote_addr, BD_ADDR_LEN);
    result = wiced_bt_ble_set_phy(&phy_preferences);
    if (result == WICED_BT_SUCCESS)
    {
#ifdef VERBOSE_THROUGHPUT_OUTPUT
        WICED_BT_TRACE("\rTPUT: Request to switch PHY to %dM\n", phy_preferences.tx_phys);
#endif
        phy_1m_2m = phy_preferences.tx_phys;
    }
    else
    {
#ifdef VERBOSE_THROUGHPUT_OUTPUT
        WICED_BT_TRACE("\rTPUT: PHY switch request failed, result: %d\n", result);
#endif
    }
    return;
}

/*************************************************************************************
* Function Name: wiced_bt_gatt_status_t
*                tput_gatts_connection_up(wiced_bt_gatt_connection_status_t *p_status)
**************************************************************************************
* Summary: This function is invoked when GATT connection is established
*
* Parameters:
*   wiced_bt_gatt_connection_status_t *p_status: GATT connection status handle
*
* Return:
*   wiced_bt_gatt_status_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*************************************************************************************/
wiced_bt_gatt_status_t tput_gatts_connection_up(wiced_bt_gatt_connection_status_t *p_status)
{
    wiced_result_t result;

    /* Update the connection handler.  Save address of the connected device. */
    tput_conn_state.conn_id = p_status->conn_id;
    memcpy(tput_conn_state.remote_addr, p_status->bd_addr, sizeof(BD_ADDR));

    /* Stop advertising */
    result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, BLE_ADDR_PUBLIC, NULL);
#ifdef VERBOSE_THROUGHPUT_OUTPUT
    WICED_BT_TRACE( "\rTPUT: Stop ADV: %d\n", result );
#endif

    /* send connection interval update if required */
    wiced_bt_l2cap_update_ble_conn_params(p_status->bd_addr, CONNECTION_INTERVAL,
                                       CONNECTION_INTERVAL, 0, BLE_CONN_TIMEOUT);

    return result;
}

/**************************************************************************************
* Function Name: wiced_bt_gatt_status_t
*                tput_gatts_connection_down(wiced_bt_gatt_connection_status_t *p_status)
***************************************************************************************
* Summary: This function is invoked on GATT disconnect
*
* Parameters:
*   wiced_bt_gatt_connection_status_t *p_status: GATT connection status handle
*
* Return:
*   wiced_bt_gatt_status_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
**************************************************************************************/
wiced_bt_gatt_status_t
tput_gatts_connection_down(wiced_bt_gatt_connection_status_t *p_status)
{
    wiced_result_t result;
    gatt_db_lookup_table_t *puAttribute;

    /* Resetting the device info */
    memset(tput_conn_state.remote_addr, 0, sizeof(BD_ADDR));
    tput_conn_state.conn_id = 0;

    /*
     * If disconnection was caused by the peer, start low advertisements,
     * so that peer can connect when it wakes up.
     */
    result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, BLE_ADDR_PUBLIC, NULL);
#ifdef VERBOSE_THROUGHPUT_OUTPUT
    WICED_BT_TRACE("\rTPUT: Start ADV LOW: %d\n", result);
#endif

    /* Disable GATT notification */
    tput_hostinfo.characteristic_client_configuration = 0;

    /* Update GATT DB */
    if ((puAttribute = tput_get_attribute(HDLD_TPUT_NOTIFY_DESCRIPTOR)) != NULL)
    {
        memcpy(puAttribute->p_data,
                (uint8_t *)&tput_hostinfo.characteristic_client_configuration,
                puAttribute->max_len);
    }

    WICED_SUPPRESS_WARNINGS(result);

    return result;
}

/*************************************************************************************
* Function Name: wiced_bt_gatt_status_t
*                tput_gatts_conn_status_cb(wiced_bt_gatt_connection_status_t *p_status)
**************************************************************************************
* Summary: This function is invoked on GATT connection status change
*
* Parameters:
*   wiced_bt_gatt_connection_status_t *p_status: GATT connection status handle
*
* Return:
*   wiced_bt_gatt_status_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*************************************************************************************/
wiced_bt_gatt_status_t tput_gatts_conn_status_cb(wiced_bt_gatt_connection_status_t *p_status)
{
    wiced_bt_gatt_status_t result;

    if (p_status->connected)
    {
        result = tput_gatts_connection_up(p_status);
        if (result == WICED_BT_GATT_SUCCESS)
        {
#ifdef VERBOSE_THROUGHPUT_OUTPUT
            WICED_BT_TRACE("\rGATT connected\n");
#endif
            /* GATT Connected - LED1 ON */
            wiced_hal_gpio_set_pin_output(GATT_CONNECT_LED, GPIO_PIN_OUTPUT_LOW);

            if (!(tput_hostinfo.characteristic_client_configuration &
                                    GATT_CLIENT_CONFIG_NOTIFICATION))
            {
                WICED_BT_TRACE("\r####### TPUT: GATT Notifications are disabled #######\n");
            }
#ifdef LE_COC_SUPPORT
            if (!(le_coc_cb.local_cid && le_coc_cb.local_cid < MAX_LOCAL_CID))
            {
                WICED_BT_TRACE("\r####### TPUT: LE COC not established #######\n");
            }
#endif
        }
        return result;
    }

    result = tput_gatts_connection_down(p_status);
    if (result == WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("\rTPUT: Disconnected from peer.\n");
        /* GATT Disconnected - LED1 OFF */
        wiced_hal_gpio_set_pin_output(GATT_CONNECT_LED, GPIO_PIN_OUTPUT_HIGH);
    }
    return result;
}

/*************************************************************************************************
* Function Name: wiced_bt_gatt_status_t
*                tput_gatts_req_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data)
**************************************************************************************************
* Summary: GATT attribute read function.
*          Process read command from peer device.
*
* Parameters:
*   uint16_t conn_id                    : GATT connection id
*   wiced_bt_gatt_read_t * p_read_data  : GATT read attribute handle
*
* Return:
*   wiced_bt_gatt_status_t: WICED_BT_GATT_SUCCESS or WICED_BT_GATT_INVALID_HANDLE
*
*************************************************************************************************/
wiced_bt_gatt_status_t
tput_gatts_req_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data)
{
    gatt_db_lookup_table_t *puAttribute;
    int          attr_len_to_copy;

    if ((puAttribute = tput_get_attribute(p_read_data->handle)) == NULL)
    {
#ifdef VERBOSE_THROUGHPUT_OUTPUT
        WICED_BT_TRACE("\rread_hndlr attr not found hdl:%x\n", p_read_data->handle );
#endif
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    attr_len_to_copy = puAttribute->cur_len;

    if (p_read_data->offset >= puAttribute->cur_len)
    {
        attr_len_to_copy = 0;
    }

    if (attr_len_to_copy != 0)
    {
        uint8_t *from;
        int to_copy = attr_len_to_copy - p_read_data->offset;

        if (to_copy > *p_read_data->p_val_len)
        {
            to_copy = *p_read_data->p_val_len;
        }

        from = ((uint8_t *)puAttribute->p_data) + p_read_data->offset;
        *p_read_data->p_val_len = to_copy;
        memcpy( p_read_data->p_val, from, to_copy);
    }

    return WICED_BT_GATT_SUCCESS;
}

/*************************************************************************************
* Function Name: wiced_bt_gatt_status_t
*                tput_gatts_req_write_handler(wiced_bt_gatt_connection_status_t *p_status)
**************************************************************************************
* Summary: GATT attribute write function.
*          Process write request or write command from peer device.
*
* Parameters:
*   uint16_t conn_id                : GATT connection id
*   wiced_bt_gatt_write_t * p_data  : GATT write attribute handle
*
* Return:
*   wiced_bt_gatt_status_t: WICED_BT_GATT_SUCCESS or WICED_BT_GATT_INVALID_HANDLE or
*                           WICED_BT_GATT_INVALID_ATTR_LEN
*
*************************************************************************************/
wiced_bt_gatt_status_t
tput_gatts_req_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t * p_data)
{
    wiced_bt_gatt_status_t result    = WICED_BT_GATT_SUCCESS;
    uint8_t                *p_attr   = p_data->p_val;
    gatt_db_lookup_table_t *puAttribute;

#ifdef VERBOSE_THROUGHPUT_OUTPUT
    WICED_BT_TRACE("\rwrite_handler: conn_id:%d hdl:0x%x prep:%d offset:%d len:%d\n ", conn_id,
                                                                                p_data->handle,
                                                                                p_data->is_prep,
                                                                                p_data->offset,
                                                                                p_data->val_len);
#endif

    switch (p_data->handle)
    {
    /* By writing into Characteristic Client Configuration descriptor
     * peer can enable or disable notification
     */
    case HDLD_TPUT_NOTIFY_DESCRIPTOR:
        if (p_data->val_len != sizeof(tput_hostinfo.characteristic_client_configuration))
        {
            return WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        tput_hostinfo.characteristic_client_configuration = p_attr[0] | (p_attr[1] << ONE_BYTE);

        /* Update GATT DB */
        if ((puAttribute = tput_get_attribute(p_data->handle)) != NULL)
        {
            memcpy(puAttribute->p_data,
                    (uint8_t *)&tput_hostinfo.characteristic_client_configuration,
                    puAttribute->max_len);
        }

        /* LED 1 ON(notify enabled); OFF(notify disabled) */
        wiced_hal_gpio_set_pin_output(CONGESTION_LED,
                                !(tput_hostinfo.characteristic_client_configuration &
                                GATT_CLIENT_CONFIG_NOTIFICATION));

        if (!(tput_hostinfo.characteristic_client_configuration &
                                GATT_CLIENT_CONFIG_NOTIFICATION))
        {
            WICED_BT_TRACE("\r####### TPUT: GATT Notifications are disabled #######\n");
        }
        else
        {
            WICED_BT_TRACE("\r####### TPUT: GATT Notifications enabled #######\n");
        }
        break;

    default:
        result = WICED_BT_GATT_INVALID_HANDLE;
        break;
    }

    return result;
}

/**************************************************************************************
 * Function Name: void tput_send_message(void)
 **************************************************************************************
 * Summary: Send GATT notifications on conditions satisfied.
 *
 * Parameters:
 *   None
 *
 * Return:
 *   None
 *
 *************************************************************************************/
void tput_send_message(void)
{
    /* If client has not registered for indication or notification, no action */
    if (!tput_hostinfo.characteristic_client_configuration)
    {
        WICED_BT_TRACE("\r####### TPUT: client not registered for notification/indication #######");
        return;
    }
    /* send GATT notifications only if notification bit is set in CCCD */
    if (tput_hostinfo.characteristic_client_configuration &
                           GATT_CLIENT_CONFIG_NOTIFICATION)
    {
        WICED_BT_TRACE("\r####### TPUT: client registered for notification #######");

        wiced_bt_gatt_send_notification(tput_conn_state.conn_id,
                                         HDLC_TPUT_NOTIFY_VALUE,
                                 sizeof(tput_char_notify_value),
                                        tput_char_notify_value);
    }
}

/*************************************************************************************
 * Function Name: wiced_bt_gatt_status_t
 *                tput_gatts_req_cb(wiced_bt_gatt_attribute_request_t *p_data)
 **************************************************************************************
 * Summary: Process GATT Read/Write/MTU request from the peer.
 *
 * Parameters:
 *   wiced_bt_gatt_attribute_request_t *p_data: GATT request information handle.
 *
 * Return:
 *   wiced_bt_gatt_status_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
 *
 *************************************************************************************/
wiced_bt_gatt_status_t tput_gatts_req_cb(wiced_bt_gatt_attribute_request_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch (p_data->request_type)
    {
    case GATTS_REQ_TYPE_READ:
        result = tput_gatts_req_read_handler(p_data->conn_id, &(p_data->data.read_req));
        break;
    case GATTS_REQ_TYPE_WRITE:
        result = tput_gatts_req_write_handler(p_data->conn_id, &(p_data->data.write_req));
        break;
    case GATTS_REQ_TYPE_MTU:
        tput_mtu = p_data->data.mtu;
        break;
   default:
        break;
    }
    return result;
}

/**************************************************************************************
 * Function Name: wiced_bt_gatt_status_t tput_gatts_callback(wiced_bt_gatt_evt_t event,
 *                                                  wiced_bt_gatt_event_data_t *p_data)
 **************************************************************************************
 * Summary: Callback for various GATT events.
 *
 * Parameters:
 *   wiced_bt_gatt_evt_t event          : GATT event code.
 *   wiced_bt_gatt_event_data_t *p_data : GATT event information handle.
 *
 * Return:
 *   wiced_bt_gatt_status_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
 *
 *************************************************************************************/
wiced_bt_gatt_status_t
tput_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = tput_gatts_conn_status_cb(&p_data->connection_status);
        break;
    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = tput_gatts_req_cb(&p_data->attribute_request);
        break;
    default:
        break;
    }
    return result;
}

/***********************************************************************************
 * Function Name: void tput_advertisement_stopped(void)
 ***********************************************************************************
 * Summary: This function is invoked when advertisements stop. If we are configured
 *          to stay connected, disconnection was caused by the peer, start low
 *          advertisements, so that peer can connect when it wakes up.
 *
 * Parameters:
 *   None
 *
 * Return:
 *   None
 *
 **********************************************************************************/
void tput_advertisement_stopped(void)
{
    if (!tput_conn_state.conn_id)
    {
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, BLE_ADDR_PUBLIC, NULL);
    }
}
