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
 * AG Plus Device Sample Application for 2070X devices.
 *
 * This file implements 2070x embedded application controlled over UART.
 * Current version of the application exposes Handsfree Audio Gateway and
 * GATT/GAP and SPP functionality.
 * MCU connected over UART can send commands to execute certain functionality
 * while configuration is local in the application including SDP and GATT
 * databases, as well as configuration of different activities like inquiry
 * advertisements or scanning.
 * Current version of the application does not support full HF AG functionality
 * but only enough to create service level connection to hands-free device
 * and establishment of the voice connection which can be used for example for
 * the voice recognition feature.
 *
 * To demonstrate the app, work through the following steps.
 * 1. Plug the WICED Bluetooth ( 2070x ) evaluation board into your computer
 * 2. Build and download the application ( to the 2070x board )
 * 3. Use ClientControl application to send various commands
 *
 * The sample app performs as a Bluetooth HF AG, SPP client/server and
 * GATT server/client.
 *
 * The sample Windows ClientControl application is provided to show sample
 * MCU implementation on Windows platform.
 *
 * Features demonstrated
 *  - WICED BT Handsfree (Audio Gateway) APIs
 *  - WICED BT GATT APIs
 *  - Handling of the UART WICED protocol
 *  - SDP and GATT descriptor/attribute configuration
 *  - Setting of the Local Bluetooth Device address from the host MCU
 *
 * On startup this demo:
 *  - Initializes the Bluetooth sub system
 *  - Receive NVRAM information from the host
 *
 * Application Instructions
 *  - Build application to produce a downloadable hcd file.  For example
 *    demo.hci_audio_gateway-CYW920706WCDEVAL DIRECT_LOAD=1 build
 *  - Connect a PC terminal to the serial port of the WICED Eval board.
 *  - Start ClientControl application.
 *  - Select the COM port assigned to the WICED Eval board.
 *  - Modify Local Bluetooth address if necessary.
 *  - Enter the full path of the HCD file to download, for example
 *    C:\Users\<username>\Documents\WICED-Studio-X.X\
 *      20706-A2_Bluetooth\build\
 *      hci_audio_gateway-CYW920706WCDEVAL-rom-ram-Wiced-release\
 *      hci_audio_gateway-CYW920706WCDEVAL-rom-ram-Wiced-release.hcd
 *  - Click on the Download button to push the HCD file, Local Address and
 *    peer host information if it was saved before.  Note: User input as well
 *    as peer host information is saved in the Windows registry under
 *    HKCU\Software\Broadcom\HciControl hive.  This is the place to clean
 *    up, for example if information about the paired should be empty.
 *
 * BLE
 *  - To find Gatt devices: Click on the "Start Ble Discovery" button
 *  - To start adverterisements: Click on the "Start Adverts" button
 *  - To connect Ble device: Choose device from the drop down combo box and
 *    click "Connect" button
 *  - To discover services: Click on the "Discover Services" button
 *  - To discover characteristics: Enter the handles in the edit box and click
 *    on "Discover Characteristics"
 *  - To discover descriptors: Enter the handles in the edit box and click on
 *    "Discover Characteristics"
 *  - Enter the Handle and Hex Value to write to the remote device using buttons
 *     "Write" : Write hex value to remote handle
 *     "Write no rsp" : Write hex value without response to remote handle
 *     "Value Notify" : Write notification value to the remote handle
 *     "Value Indicate" : Write indication value to the remote handle
 *
 * BR/EDR
 * - To find BR/EDR devices: Click on "Start BR/EDR Discovery"
 *
 * SPP
 * - To create an SPP Connection to remote SPP server choose the bluetooth address
 *   of the remote device from the BR/EDR combo box
 *
 * AG Connection
 * - To create audio gateway connection to remote handsfree controller, choose the bluetooth
 *   address of the remote device from the BR/EDR combo box
 * - Click "Connect" button under AG
 * - To establish SCO connection, click on Audio connect
 * - Check SCO audio quality
 * - NOTE : Default WBS is disabled, update hf_control_esco_params structure to enabled it.
 */


#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_dev.h"
#include "sparcommon.h"
#include "wiced_bt_cfg.h"
#include "wiced_memory.h"
#include "wiced_platform.h"
#include "hci_control.h"
#include "hci_control_api.h"
#include "wiced_transport.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "string.h"
#include "wiced_transport.h"
#include "wiced_hal_wdog.h"

/*****************************************************************************
**  Constants
*****************************************************************************/
#define WICED_HS_EIR_BUF_MAX_SIZE               264
#define HCI_CONTROL_FIRST_VALID_NVRAM_ID        0x10
#define HCI_CONTROL_INVALID_NVRAM_ID            0x00
// SDP Record handle for AVDT Source
#define HANDLE_AVDT_SOURCE                      0x10001
// SDP Record handle for AVRC TARGET
#define HANDLE_AVRC_TARGET                      0x10002
// SDP Record handle for SPP
#define HANDLE_SPP                              0x10003

#define BCM920706 20706
#define BCM920707 20707

#define READ_LITTLE_ENDIAN_TO_UINT16(into, m,dl)      \
        (into) = ((m)[0] | ((m)[1]<<8));\
        (m) +=2; (dl)-=2;

#define KEY_INFO_POOL_BUFFER_SIZE               145 //Size of the buffer used for holding the peer device key info
#define KEY_INFO_POOL_BUFFER_COUNT              10  //Correspond's to the number of peer devices


/*****************************************************************************
**  Structures
*****************************************************************************/
typedef struct
{
    void    *p_next;
    uint16_t nvram_id;
    uint8_t  chunk_len;
    uint8_t  data[1];
} hci_control_nvram_chunk_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/
/*
 * This is the SDP database for the whole hci_control application
 */
const uint8_t hci_control_sdp_db[] =
{
    SDP_ATTR_SEQUENCE_1( 127 ),                                             // length is the sum of all records

    // SDP record for HF ( total length of record: 51 )
    SDP_ATTR_SEQUENCE_1( 49 ),                                              // 2 bytes, length of the record
        SDP_ATTR_RECORD_HANDLE( 0x10001 ),                                  // 8 byte ( handle=0x10001 )
        SDP_ATTR_ID( ATTR_ID_SERVICE_CLASS_ID_LIST ),                       // 3 bytes
        SDP_ATTR_SEQUENCE_1( 6 ),                                           // 2 bytes
        SDP_ATTR_UUID16( 0x111F ),                                          // 3 bytes ServiceClass0 UUID_SERVCLASS_HF_HANDSFREE
        SDP_ATTR_UUID16( 0X1203 ),                                          // 3 bytes ServiceClass1 UUID_SERVCLASS_GENERIC_AUDIO
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST( 1 ),                            // 17 bytes ( SCN=1 )
        SDP_ATTR_PROFILE_DESC_LIST( UUID_SERVCLASS_AG_HANDSFREE, 0x0105 ),  // 13 bytes UUID_SERVCLASS_HF_HANDSFREE, version 0x0105

    // SDP record for Serial Port ( total length of record: 76 )
    SDP_ATTR_SEQUENCE_1(74),                                                // 2 bytes
        SDP_ATTR_RECORD_HANDLE(0x10002),                                    // 8 bytes
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_SERIAL_PORT),                      // 8
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST( 2 ),                            // 17 bytes
        SDP_ATTR_BROWSE_LIST,                                               // 8
        SDP_ATTR_PROFILE_DESC_LIST(UUID_SERVCLASS_SERIAL_PORT, 0x0102),     // 13 byte
        SDP_ATTR_SERVICE_NAME(15),                                      // 20
        'B', 'R', 'C', 'M', ' ', 'S', 'P', 'P', ' ', 'S', 'E', 'R', 'V', 'E', 'R'
};

hci_control_nvram_chunk_t *p_nvram_first = NULL;

/* HS control block */
#if BTA_DYNAMIC_MEMORY == FALSE
hci_control_cb_t  hci_control_cb;
#endif

wiced_transport_buffer_pool_t *rfcomm_trans_pool;   // Trans pool for sending the RFCOMM data to host
wiced_timer_t rfcomm_trans_data_timer;
uint16_t rfcomm_trans_data_len = 0;
uint8_t* p_rfcomm_trans_buf = NULL;
wiced_timer_t rfcomm_flow_control_timer;
wiced_bt_buffer_pool_t* p_key_info_pool;//Pool for storing the  key info

uint8_t pincode[4]  = {0x30,0x30,0x30,0x30};

static uint32_t hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length );
static void      hci_control_tx_complete( wiced_transport_buffer_pool_t* p_pool );

const wiced_transport_cfg_t  transport_cfg =
{
    WICED_TRANSPORT_UART,
    {{ WICED_TRANSPORT_UART_HCI_MODE, HCI_UART_DEFAULT_BAUD}},
    { TRANS_UART_BUFFER_SIZE, 1},
    NULL,
    hci_control_proc_rx_cmd,
    hci_control_tx_complete
};


/******************************************************
 *               Function Declarations
 ******************************************************/
static void     hci_control_handle_reset_cmd( void );
static void     hci_control_handle_trace_enable( uint8_t *p_data );
static void     hci_control_device_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len );
static void     hci_control_handle_set_local_bda( uint8_t *p_bda );
static void     hci_control_inquiry( uint8_t enable );
static void     hci_control_handle_set_visibility( uint8_t discoverability, uint8_t connectability );
static void     hci_control_send_device_started_evt( void );
static void     hci_control_send_pairing_completed_evt( uint8_t status , wiced_bt_device_address_t bdaddr );
static void     hci_control_send_encryption_changed_evt( uint8_t encrypted , wiced_bt_device_address_t bdaddr );
static void     hci_control_misc_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len );
static void     hci_control_misc_handle_get_version( void );
static wiced_result_t hci_control_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static void hci_control_handle_set_pairability ( uint8_t pairing_allowed );

extern void wiced_bt_trace_array( const char *string, const uint8_t* array, const uint16_t len );
extern void hci_control_spp_startup( void );
extern void hci_control_le_init( void );
extern void hci_control_ag_sco_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
extern void hci_control_le_scan_state_changed( wiced_bt_ble_scan_type_t state );
extern void hci_control_le_advert_state_changed( wiced_bt_ble_advert_mode_t mode );
extern void wdog_generate_hw_reset(void);
extern uint8_t avrc_is_abs_volume_capable( void );


/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 *  Application Start, ie, entry point to the application.
 */
APPLICATION_START( )
{
    wiced_transport_init( &transport_cfg );

#ifdef WICED_BT_TRACE_ENABLE
    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
#if ( defined(CYW20706A2) || defined(CYW20735B0) || defined(CYW20719B0) || defined(CYW43012C0) )
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif

    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy
    // Note: WICED HCI must be configured to use this - see wiced_trasnport_init(), must
    // be called with wiced_transport_cfg_t.wiced_tranport_data_handler_t callback present
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

    WICED_BT_TRACE( "Audio Gateway APP START\n" );

    /* Register the dynamic configurations */
    wiced_bt_stack_init( hci_control_management_callback, &hci_ag_cfg_settings, hci_ag_cfg_buf_pools);

    rfcomm_trans_pool = wiced_transport_create_buffer_pool( SPP_DEVICE_MTU, SPP_TRANS_MAX_BUFFERS );

    /* Configure Audio buffer */
    wiced_audio_buffer_initialize (hci_ag_audio_buf_config);
}

/*
 *  Prepare extended inquiry response data.  Current version publishes headset,
 *  handsfree and generic audio services.
 */
void hci_control_write_eir( void )
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t length;

    pBuf = (uint8_t*)wiced_bt_get_buffer( WICED_HS_EIR_BUF_MAX_SIZE );
    WICED_BT_TRACE( "hci_control_write_eir %x\n", pBuf );

    if ( !pBuf )
    {
        return;
    }
    p = pBuf;

    //p = ( uint8_t * )( pBuf + 1 );
    //p += 4;

    length = strlen( (char *)hci_ag_cfg_settings.device_name );

    *p++ = length + 1;
    *p++ = 0x09;            // EIR type full name
    memcpy( p, hci_ag_cfg_settings.device_name, length );
    p += length;
    *p++ = ( 3 * 2 ) + 1;     // length of services + 1
    *p++ =   0x02;            // EIR type full list of 16 bit service UUIDs
    *p++ =   UUID_SERVCLASS_HEADSET         & 0xff;
    *p++ = ( UUID_SERVCLASS_HEADSET >> 8 ) & 0xff;
    *p++ =   UUID_SERVCLASS_HF_HANDSFREE        & 0xff;
    *p++ = ( UUID_SERVCLASS_HF_HANDSFREE >> 8 ) & 0xff;
    *p++ =   UUID_SERVCLASS_GENERIC_AUDIO        & 0xff;
    *p++ = ( UUID_SERVCLASS_GENERIC_AUDIO >> 8 ) & 0xff;
    *p++ = 0;

    // print EIR data
    wiced_bt_trace_array( "EIR :", ( uint8_t* )( pBuf+1 ), MIN( p-( uint8_t* )pBuf,100 ) );
    wiced_bt_dev_write_eir( pBuf, (uint16_t)(p - pBuf) );

    return;
}

/*
 *  Pass protocol traces up through the UART
 */
void hci_control_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    //send the trace
    wiced_transport_send_hci_trace( NULL, type, length, p_data  );
}

/*
 *  Management callback receives various notifications from the stack
 */
wiced_result_t hci_control_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                     result = WICED_BT_SUCCESS;
    wiced_bt_dev_status_t              dev_status;
    wiced_bt_dev_ble_pairing_info_t   *p_pairing_info;
    wiced_bt_dev_encryption_status_t  *p_encryption_status;
    int                                bytes_written, bytes_read;
    int                                nvram_id;
    wiced_bt_dev_pairing_cplt_t        *p_pairing_cmpl;
    uint8_t                             pairing_result;

    WICED_BT_TRACE( "hci_control_management_callback 0x%02x\n", event );

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            hci_control_write_eir( );

            /* initialize everything */
            memset( &hci_control_cb, 0, sizeof( hci_control_cb ) );

            /* create SDP records */
            wiced_bt_sdp_db_init( ( uint8_t * )hci_control_sdp_db, sizeof( hci_control_sdp_db ) );

            /* Enable HCI traces by default */
            wiced_bt_dev_register_hci_trace( hci_control_hci_trace_cback );

            /* Perform the rfcomm init before hf and spp start up */
            wiced_bt_rfcomm_init( 200, 5 );

            hci_control_ag_startup( );

            hci_control_spp_startup( );

            hci_control_le_init( );

            p_key_info_pool = wiced_bt_create_pool( KEY_INFO_POOL_BUFFER_SIZE, KEY_INFO_POOL_BUFFER_COUNT );
            WICED_BT_TRACE( "wiced_bt_create_pool %x\n", p_key_info_pool );

            hci_control_send_device_started_evt( );
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_PIN_REQUEST_EVT:
            WICED_BT_TRACE("remote address= %B\n", p_event_data->pin_request.bd_addr);
            wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr,result/*WICED_BT_SUCCESS*/,4, &pincode[0]);
        break;


        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            /* User confirmation request for pairing (sample app always accepts) */
            wiced_bt_dev_confirm_req_reply (WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            /* Use the default security for BR/EDR*/
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_REQUEST_EVT bda %B\n",
                                            p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
            p_event_data->pairing_io_capabilities_br_edr_request.oob_data     = WICED_FALSE;
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap          = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data   = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req   = BTM_LE_AUTH_REQ_BOND | BTM_LE_AUTH_REQ_MITM;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            break;


        case BTM_PAIRING_COMPLETE_EVT:
            p_pairing_cmpl = &p_event_data->pairing_complete;

            if(p_pairing_cmpl->transport == BT_TRANSPORT_BR_EDR)
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
            }
            else
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.ble.reason;
            }

            hci_control_send_pairing_completed_evt( pairing_result, p_event_data->pairing_complete.bd_addr );
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_encryption_status = &p_event_data->encryption_status;

            WICED_BT_TRACE( "Encryption Status Event: bd ( %B ) res %d\n", p_encryption_status->bd_addr, p_encryption_status->result );

            hci_control_send_encryption_changed_evt ( p_encryption_status->result, p_encryption_status->bd_addr);
            break;

        case BTM_SECURITY_REQUEST_EVT:
            wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /* Check if we already have information saved for this bd_addr */
            if ( ( nvram_id = hci_control_find_nvram_id( p_event_data->paired_device_link_keys_update.bd_addr, BD_ADDR_LEN ) ) == 0)
            {
                /* This is the first time, allocate id for the new memory chunk */
                nvram_id = hci_control_alloc_nvram_id( );
                WICED_BT_TRACE( "Allocated NVRAM ID:%d\n", nvram_id );
            }
            bytes_written = hci_control_write_nvram( nvram_id, sizeof( wiced_bt_device_link_keys_t ), &p_event_data->paired_device_link_keys_update, FALSE );

            WICED_BT_TRACE("NVRAM write:id:%d bytes:%d dev: [%B]\n", nvram_id, bytes_written, p_event_data->paired_device_link_keys_update.bd_addr);
//            WICED_BT_TRACE_ARRAY(((uint8_t *)&p_event_data->paired_device_link_keys_update.key_data), sizeof(p_event_data->paired_device_link_keys_update.key_data), "key: ");
//            WICED_BT_TRACE("\n");
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* read existing key from the NVRAM  */

            WICED_BT_TRACE("\t\tfind device %B\n", p_event_data->paired_device_link_keys_request.bd_addr);

            if ( ( nvram_id = hci_control_find_nvram_id( p_event_data->paired_device_link_keys_request.bd_addr, BD_ADDR_LEN ) ) != 0)
            {
                 bytes_read = hci_control_read_nvram( nvram_id, &p_event_data->paired_device_link_keys_request, sizeof( wiced_bt_device_link_keys_t ) );

                 result = WICED_BT_SUCCESS;
                 WICED_BT_TRACE("Read:nvram_id:%d bytes:%d\n", nvram_id, bytes_read);
            }
            else
            {
                result = WICED_BT_ERROR;
                WICED_BT_TRACE("Key retrieval failure\n");
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* Request to store newly generated local identity keys to NVRAM */
            /* (sample app does not store keys to NVRAM) */

            break;


        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /*
             * Request to restore local identity keys from NVRAM
             * (requested during Bluetooth start up)
             * */
            /* (sample app does not store keys to NVRAM)
             * New local identity keys will be generated
             * */
            result = WICED_BT_NO_RESOURCES;
            break;

        case BTM_SCO_CONNECTED_EVT:
        {
            wiced_bt_sco_connected_t *p_sco_data = ( wiced_bt_sco_connected_t * )p_event_data;
            hci_control_ag_sco_management_callback( event, p_event_data );
            break;
        }

        case BTM_SCO_DISCONNECTED_EVT:
        {
            wiced_bt_sco_disconnected_t *p_sco_data = ( wiced_bt_sco_disconnected_t * )p_event_data;
            hci_control_ag_sco_management_callback( event, p_event_data );
            break;
        }

        case BTM_SCO_CONNECTION_REQUEST_EVT:
        {
            wiced_bt_sco_connection_request_t *p_sco_data = ( wiced_bt_sco_connection_request_t * )p_event_data;
            hci_control_ag_sco_management_callback( event, p_event_data );
            break;
        }

        case BTM_SCO_CONNECTION_CHANGE_EVT:
            hci_control_ag_sco_management_callback( event, p_event_data );
            break;

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
            hci_control_le_scan_state_changed( p_event_data->ble_scan_state_changed );
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            hci_control_le_advert_state_changed( p_event_data->ble_advert_state_changed );
            break;

        default:
            result = WICED_BT_USE_DEFAULT_SECURITY;
            break;
    }
    return result;
}

/*
 * Handle received command over UART. Please refer to the WICED HCI Control Protocol
 * Software User Manual (WICED-SWUM10x-R) for details on the
 * HCI UART control protocol.
*/
static uint32_t hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t* p_rx_buf = p_data;

    WICED_BT_TRACE( "hci_control_proc_rx_cmd:%d, %x \n", length ,p_rx_buf);

    if ( !p_rx_buf )
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }
    //Expected minimum 4 byte as the wiced header
    if( length < 4 )
    {
        WICED_BT_TRACE("invalid params\n");
        wiced_transport_free_buffer( p_rx_buf );
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }


    READ_LITTLE_ENDIAN_TO_UINT16(opcode, p_data, length);     // Get opcode
    READ_LITTLE_ENDIAN_TO_UINT16(payload_len, p_data, length); // Get len

    WICED_BT_TRACE("cmd_opcode 0x%02x\n", opcode);

    switch((opcode >> 8) & 0xff)
    {
    case HCI_CONTROL_GROUP_DEVICE:
        hci_control_device_handle_command( opcode, p_data, payload_len );
        break;

    case HCI_CONTROL_GROUP_LE:
    case HCI_CONTROL_GROUP_GATT:
        hci_control_le_handle_command( opcode, p_data, payload_len );
        break;

    case HCI_CONTROL_GROUP_SPP:
        hci_control_spp_handle_command( opcode, p_data, payload_len );
        break;

    case HCI_CONTROL_GROUP_AG:
        hci_control_ag_handle_command( opcode, p_data, payload_len );
        break;

    case HCI_CONTROL_GROUP_MISC:
        hci_control_misc_handle_command(opcode, p_data, payload_len);
        break;

    default:
        WICED_BT_TRACE( "unknown class code\n");
        break;
    }

    //Freeing the buffer in which data is received
    wiced_transport_free_buffer( p_rx_buf );
    return status;
}

void hci_control_device_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len )
{
    uint8_t bytes_written;
    switch( cmd_opcode )
    {
    case HCI_CONTROL_COMMAND_RESET:
        hci_control_handle_reset_cmd( );
        break;

    case HCI_CONTROL_COMMAND_TRACE_ENABLE:
        hci_control_handle_trace_enable( p_data );
        break;

    case HCI_CONTROL_COMMAND_SET_LOCAL_BDA:
        hci_control_handle_set_local_bda( p_data );
        break;

    case HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA:
        bytes_written = hci_control_write_nvram( p_data[0] | ( p_data[1] << 8 ), data_len - 2, &p_data[2], TRUE );
        WICED_BT_TRACE( "NVRAM write: %d dev: [%B]\n", bytes_written , &p_data[2]);
        break;

    case HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA:
        hci_control_delete_nvram( p_data[0] | ( p_data[1] << 8 ), WICED_TRUE);
        WICED_BT_TRACE( "NVRAM delete: %d\n", p_data[0] | ( p_data[1] << 8 ) );
        break;

    case HCI_CONTROL_COMMAND_INQUIRY:
        hci_control_inquiry( *p_data );
        break;

    case HCI_CONTROL_COMMAND_SET_VISIBILITY:
        hci_control_handle_set_visibility( p_data[0], p_data[1] );
        break;

    case  HCI_CONTROL_COMMAND_SET_PAIRING_MODE:
        hci_control_handle_set_pairability( p_data[0] );
        break;

    default:
        WICED_BT_TRACE( "??? Unknown command code\n" );
        break;
    }
}

/*
 * handle reset command from UART
 */
void hci_control_handle_reset_cmd( void )
{
    // trip watch dog now.
    wiced_hal_wdog_reset_system( );
}

/*
 * handle command from UART to configure traces
 */
void hci_control_handle_trace_enable( uint8_t *p_data )
{
    uint8_t hci_trace_enable = *p_data++;
    wiced_debug_uart_types_t route_debug = (wiced_debug_uart_types_t)*p_data;

    if ( hci_trace_enable )
    {
        /* Register callback for receiving hci traces */
        // Disable while streaming audio over the uart.
        wiced_bt_dev_register_hci_trace( hci_control_hci_trace_cback );
    }
    else
    {
        wiced_bt_dev_register_hci_trace( NULL);
    }
    wiced_set_debug_uart( route_debug );
}

/*
 * handle command to set local Bluetooth device address
 */
void hci_control_handle_set_local_bda( uint8_t *p_bda)
{
    BD_ADDR bd_addr;
    STREAM_TO_BDADDR(bd_addr,p_bda);
    wiced_bt_set_local_bdaddr( bd_addr, BLE_ADDR_PUBLIC );

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}
/*
 *  Handle Set Pairability command received over UART
 */
static void hci_control_handle_set_pairability ( uint8_t pairing_allowed )
{
    uint8_t                   status = HCI_CONTROL_STATUS_SUCCESS;
    hci_control_nvram_chunk_t *p1;

    if ( hci_control_cb.pairing_allowed != pairing_allowed )
    {
        if ( pairing_allowed )
        {
            // Check if key buffer pool has buffer available. If not, cannot enable pairing until nvram entries are deleted
            if ( ( p1 = ( hci_control_nvram_chunk_t * )wiced_bt_get_buffer_from_pool( p_key_info_pool ) ) == NULL)
            {
                pairing_allowed = 0; //The key buffer pool is full therefore we cannot allow pairing to be enabled
                status = HCI_CONTROL_STATUS_OUT_OF_MEMORY;
            }
            else
            {
                wiced_bt_free_buffer( p1 );
            }
        }

    hci_control_cb.pairing_allowed = pairing_allowed;
        wiced_bt_set_pairable_mode( hci_control_cb.pairing_allowed, 0 );
        WICED_BT_TRACE( " Set the pairing allowed to %d \n", hci_control_cb.pairing_allowed );
    }

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, status );
}

/*
 *  Handle Inquiry result callback from teh stack, format and send event over UART
 */
void hci_control_inquiry_result_cback( wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data )
{
    int       i;
    uint8_t   len;
    uint8_t   tx_buf[300];
    uint16_t  code;
    uint8_t   *p = tx_buf;

    if ( p_inquiry_result == NULL )
    {
        code = HCI_CONTROL_EVENT_INQUIRY_COMPLETE;
    }
    else
    {
        code = HCI_CONTROL_EVENT_INQUIRY_RESULT;
        WICED_BT_TRACE( "inquiry result %B\n", p_inquiry_result->remote_bd_addr );
        for ( i = 0; i < 6; i++ )
            *p++ = p_inquiry_result->remote_bd_addr[5 - i];
        for ( i = 0; i < 3; i++ )
            *p++ = p_inquiry_result->dev_class[2 - i];
        *p++ = p_inquiry_result->rssi;

        // currently callback does not pass the data of the adv data, need to go through the data
        // zero len in the LTV means that there is no more data
        while (p_eir_data && ( len = *p_eir_data ) != 0 )
        {
            // In the HCI event all parameters should fit into 255 bytes
            if ( p + len + 1 > tx_buf + 255 )
            {
                WICED_BT_TRACE( "Bad data\n" );
                break;
            }
            for ( i = 0; i < len + 1; i++ )
                *p++ = *p_eir_data++;
        }
    }
    wiced_transport_send_data( code, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  Handle Inquiry command received over UART
 */
void hci_control_inquiry( uint8_t enable )
{
    wiced_result_t           result;
    wiced_bt_dev_inq_parms_t params;

    if ( enable )
    {

        memset( &params, 0, sizeof( params ) );

        params.mode             = BTM_GENERAL_INQUIRY;
        params.duration         = 5;
        params.filter_cond_type = BTM_CLR_INQUIRY_FILTER;

        result = wiced_bt_start_inquiry( &params, &hci_control_inquiry_result_cback );
        WICED_BT_TRACE( "inquiry started:%d\n", result );
    }
    else
    {
        result = wiced_bt_cancel_inquiry( );
        WICED_BT_TRACE( "cancel inquiry:%d\n", result );
    }
    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 *  Handle Set Visibility command received over UART
 */
void hci_control_handle_set_visibility( uint8_t discoverability, uint8_t connectability )
{
    wiced_bt_dev_status_t dev_status;
    uint8_t               visibility;
    uint8_t               status = HCI_CONTROL_STATUS_SUCCESS;

    // we cannot be discoverable and not connectable
    if ( ( ( discoverability != 0 ) && ( connectability == 0 ) ) ||
           ( discoverability > 1 ) ||
           ( connectability > 1 ) )
    {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS );
    }
    else
    {
        wiced_bt_dev_set_discoverability( ( discoverability != 0 ) ? BTM_GENERAL_DISCOVERABLE : BTM_NON_DISCOVERABLE ,
                                            BTM_DEFAULT_DISC_WINDOW,
                                            BTM_DEFAULT_DISC_INTERVAL);

        wiced_bt_dev_set_connectability( ( connectability != 0 ) ? WICED_TRUE : WICED_FALSE ,
                                            BTM_DEFAULT_CONN_WINDOW,
                                            BTM_DEFAULT_CONN_INTERVAL);

        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
    }
}


/*
 *  Send Device Started event through UART
 */
void hci_control_send_device_started_evt( void )
{
    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0 );

    WICED_BT_TRACE( "maxLinks:%d maxChannels:%d maxpsm:%d rfcom max links%d, rfcom max ports:%d\n", hci_ag_cfg_settings.l2cap_application.max_links,
                    hci_ag_cfg_settings.l2cap_application.max_channels, hci_ag_cfg_settings.l2cap_application.max_psm, hci_ag_cfg_settings.rfcomm_cfg.max_links,hci_ag_cfg_settings.rfcomm_cfg.max_ports );
}

/*
* transfer command status event to UART
*/
void hci_control_send_command_status_evt( uint16_t code, uint8_t status )
{
    wiced_transport_send_data( code, &status, 1 );
}

/*
 *  Send Pairing Completed event through UART
 */
void hci_control_send_pairing_completed_evt( uint8_t status , wiced_bt_device_address_t bdaddr )
{
    uint8_t event_data[BD_ADDR_LEN + sizeof(uint8_t)];
    int     cmd_bytes = 0;

    event_data[cmd_bytes++] = status;

    {
        int i;
        for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
            event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];
    }

    WICED_BT_TRACE(" sending the pairing complete evt: %B as %B status %d\n", bdaddr, &event_data[1], status);

    wiced_transport_send_data(HCI_CONTROL_EVENT_PAIRING_COMPLETE, event_data, cmd_bytes );
}

/*
 *  Send Encryption Changed event through UART
 */
void hci_control_send_encryption_changed_evt( uint8_t encrypted ,  wiced_bt_device_address_t bdaddr )
{
    uint8_t event_data[BD_ADDR_LEN + sizeof(uint8_t)];
    int     cmd_bytes = 0;

    event_data[cmd_bytes++] = encrypted;

    {
        int i;
        for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
            event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];
    }

    wiced_transport_send_data( HCI_CONTROL_EVENT_ENCRYPTION_CHANGED, event_data, cmd_bytes );
}

/*
 *  send audio connect complete event to UART
 */
wiced_result_t hci_control_audio_send_connect_complete( wiced_bt_device_address_t bd_addr, uint8_t status, uint16_t handle )
{
    int i;
    const int cmd_size = BD_ADDR_LEN + sizeof(handle) + sizeof(uint8_t);
    uint8_t event_data[BD_ADDR_LEN + sizeof(uint8_t) + sizeof(handle)];

    WICED_BT_TRACE( "[%s] %B status %x handle %x\n", __FUNCTION__, bd_addr, status, handle );

    //Build event payload
    if ( status == WICED_SUCCESS )
    {
        for ( i = 0; i < BD_ADDR_LEN; i++ )                     // bd address
            event_data[i] = bd_addr[BD_ADDR_LEN - 1 - i];

        event_data[i++] = handle & 0xff;                        //handle
        event_data[i++]   = ( handle >> 8 ) & 0xff;

        event_data[i] = avrc_is_abs_volume_capable();

        return wiced_transport_send_data( HCI_CONTROL_AUDIO_EVENT_CONNECTED, event_data, cmd_size );
    }
    else
    {
        return wiced_transport_send_data( HCI_CONTROL_AUDIO_EVENT_CONNECTION_FAILED, NULL, 0 );
    }
}

/*
 *  send audio disconnect complete event to UART
 */
wiced_result_t hci_control_audio_send_disconnect_complete( uint16_t handle, uint8_t status, uint8_t reason )
{
    int i;
    uint8_t event_data[4];

    WICED_BT_TRACE( "[%s] %04x status %d reason %d\n", __FUNCTION__, handle, status, reason );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;
    event_data[2] = status;                                 // status
    event_data[3] = reason;                                 // reason(1 byte)

    return wiced_transport_send_data( HCI_CONTROL_AUDIO_EVENT_DISCONNECTED, event_data, 4 );
}

/*
 *  send audio connect complete event to UART
 */
wiced_result_t hci_control_audio_send_started_stopped( uint16_t handle, wiced_bool_t started )
{
    uint8_t event_data[2];

    WICED_BT_TRACE( "[%s] handle %04x\n", __FUNCTION__, handle );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    return wiced_transport_send_data(started ? HCI_CONTROL_AUDIO_EVENT_STARTED : HCI_CONTROL_AUDIO_EVENT_STOPPED, event_data, 2);
}

/*
 *  send AVRC event to UART
 */
wiced_result_t hci_control_send_avrc_event( int type, uint16_t handle )
{
    uint8_t event_data[2];

    WICED_BT_TRACE( "[%s] handle %04x\n", __FUNCTION__, handle );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    return wiced_transport_send_data( type, event_data, 2 );
}

wiced_result_t hci_control_send_avrc_volume( uint16_t handle, uint8_t volume )
{
    uint8_t event_data[3];

    //Build event payload
    event_data[0] = handle & 0xff;                          // handle
    event_data[1] = ( handle >> 8 ) & 0xff;
    event_data[2] = volume;                  // Volume level in percentage

    WICED_BT_TRACE( "[%s] handle %04x Volume(Pct): %d\n", __FUNCTION__, handle, event_data[2] );

    return wiced_transport_send_data( HCI_CONTROL_AVRC_TARGET_EVENT_VOLUME_LEVEL, event_data, 3 );
}

/*
 * Write NVRAM function is called to store information in the RAM.  This can be called when
 * stack requires persistent storage, for example to save link keys.  In this case
 * data is also formatted and send to the host for real NVRAM storage.  The same function is
 * called when host pushes NVRAM chunks during the startup.  Parameter from_host in this
 * case is set to FALSE indicating that data does not need to be forwarded.
 */
int hci_control_write_nvram( int nvram_id, int data_len, void *p_data, BOOLEAN from_host )
{
    uint8_t                    tx_buf[257];
    uint8_t                   *p = tx_buf;
    hci_control_nvram_chunk_t *p1;
    wiced_result_t            result;

    /* first check if this ID is being reused and release the memory chunk */
    hci_control_delete_nvram( nvram_id ,WICED_FALSE);

    if ( ( p1 = ( hci_control_nvram_chunk_t * )wiced_bt_get_buffer_from_pool( p_key_info_pool ) ) == NULL )
    {
        WICED_BT_TRACE( "Failed to alloc:%d\n", data_len );
        return ( 0 );
    }
    if ( wiced_bt_get_buffer_size( p1 ) < ( sizeof( hci_control_nvram_chunk_t ) + data_len - 1 ) )
    {
        WICED_BT_TRACE( "Insufficient buffer size, Buff Size %d, Len %d  \n",
                        wiced_bt_get_buffer_size( p1 ), ( sizeof( hci_control_nvram_chunk_t ) + data_len - 1 ) );
        wiced_bt_free_buffer( p1 );
        return ( 0 );
    }
    p1->p_next    = p_nvram_first;
    p1->nvram_id  = nvram_id;
    p1->chunk_len = data_len;
    memcpy( p1->data, p_data, data_len );

    p_nvram_first = p1;

    {
        wiced_bt_device_link_keys_t * p_keys = ( wiced_bt_device_link_keys_t *) p_data;
#ifdef CYW20706A2
        result = wiced_bt_dev_add_device_to_address_resolution_db( p_keys ,
                p_keys->key_data.ble_addr_type );
#else
        result = wiced_bt_dev_add_device_to_address_resolution_db( p_keys );
#endif
    }

    WICED_BT_TRACE("Updated Addr Resolution DB:%d\n", result );

    /* If NVRAM chunk arrived from host, no need to send it back, otherwise send over transport */
    if (!from_host)
    {
        *p++ = nvram_id & 0xff;
        *p++ = (nvram_id >> 8) & 0xff;
        memcpy(p, p_data, data_len);

        wiced_transport_send_data( HCI_CONTROL_EVENT_NVRAM_DATA, tx_buf, ( int )( data_len + 2 ) );
    }
    return (data_len);
}

/*
 * Find nvram_id of the NVRAM chunk with first bytes matching specified byte array
 */
int hci_control_find_nvram_id(uint8_t *p_data, int len)
{
    hci_control_nvram_chunk_t *p1;

    /* Go through the linked list of chunks */
    for (p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t *)p1->p_next)
    {
        WICED_BT_TRACE( "find %B %B len:%d\n", p1->data, p_data, len );
        if ( memcmp( p1->data, p_data, len ) == 0 )
        {
            return ( p1->nvram_id );
        }
    }
    return HCI_CONTROL_INVALID_NVRAM_ID;
}

/*
 * Delete NVRAM function is called when host deletes NVRAM chunk from the persistent storage.
 */
void hci_control_delete_nvram( int nvram_id ,wiced_bool_t from_host)
{
    hci_control_nvram_chunk_t *p1, *p2;

    if ( p_nvram_first == NULL )
	return;

    /* Special case when need to remove the first chunk */
    if ( ( p_nvram_first != NULL ) && ( p_nvram_first->nvram_id == nvram_id ) )
    {
        p1 = p_nvram_first;

        if ( from_host && ( wiced_bt_dev_delete_bonded_device (p1->data) == WICED_ERROR ) )
        {
            WICED_BT_TRACE("ERROR: while Unbonding device \n");
        }
        else
        {
            p_nvram_first = (hci_control_nvram_chunk_t *)p_nvram_first->p_next;
            wiced_bt_free_buffer( p1 );
        }
        return;
    }

    /* Go through the linked list of chunks */
    for ( p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t *)p1->p_next )
    {
        p2 = (hci_control_nvram_chunk_t *)p1->p_next;
        if ( ( p2 != NULL ) && ( p2->nvram_id == nvram_id ) )
        {
            if ( from_host && ( wiced_bt_dev_delete_bonded_device (p2->data) == WICED_ERROR ) )
            {
                WICED_BT_TRACE("ERROR: while Unbonding device \n");
            }
            else
            {
                p1->p_next = p2->p_next;
                wiced_bt_free_buffer( p2 );
            }
            return;
        }
    }
}

/*
 * Read NVRAM actually finds the memory chunk in the RAM
 */
int hci_control_read_nvram( int nvram_id, void *p_data, int data_len )
{
    hci_control_nvram_chunk_t *p1;
    int                        data_read = 0;

    /* Go through the linked list of chunks */
    for ( p1 = p_nvram_first; p1 != NULL; p1 = p1->p_next )
    {
        if ( p1->nvram_id == nvram_id )
        {
            data_read = ( data_len < p1->chunk_len ) ? data_len : p1->chunk_len;
            memcpy( p_data, p1->data, data_read );
            break;
        }
    }
    return ( data_read );
}

/*
 * Allocate nvram_id to save new NVRAM chunk
 */
int hci_control_alloc_nvram_id( )
{
    hci_control_nvram_chunk_t *p1 = p_nvram_first;
    int                        nvram_id;

    /* Go through the linked list of chunks */
    WICED_BT_TRACE ( "hci_control_alloc_nvram_id\n" );
    for ( nvram_id = HCI_CONTROL_FIRST_VALID_NVRAM_ID; p1 != NULL; nvram_id++ )
    {
        for ( p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t *)p1->p_next )
        {
            if ( p1->nvram_id == nvram_id )
            {
                /* this nvram_id is already used */
                break;
            }
        }
        if ( p1 == NULL )
        {
            break;
        }
    }
    WICED_BT_TRACE ( "hci_control_alloc_nvram_id:%d\n", nvram_id );
    return ( nvram_id );
}

void hci_control_tx_complete( wiced_transport_buffer_pool_t* p_pool )
{
    WICED_BT_TRACE ( "hci_control_tx_complete :%x \n", p_pool );
}

/*
 * This utility copies a character string to another
 */
char *utl_strcpy( char *p_dst, char *p_src )
{
    register char *pd = p_dst;
    register char *ps = p_src;

    while ( *ps )
        *pd++ = *ps++;

    *pd++ = 0;

    return ( p_dst );
}

/*
 * This utility function converts a uint16_t to a string.  The string is
 * NULL-terminated.  The length of the string is returned;
 * Returns Length of string.
 */
uint8_t utl_itoa( uint16_t i, char *p_s )
{
    uint16_t  j, k;
    char     *p = p_s;
    BOOLEAN   fill = FALSE;

    if ( i == 0 )
    {
        /* take care of zero case */
        *p++ = '0';
    }
    else
    {
        for ( j = 10000; j > 0; j /= 10 )
        {
            k = i / j;
            i %= j;
            if ( k > 0 || fill || ( j == 1 ) )
            {
              *p++ = k + '0';
              fill = TRUE;
            }
        }
    }
    *p = 0;
    return ( uint8_t ) ( p - p_s );
}

/*
 * This utility function compares two strings in uppercase. String p_s must be
 * uppercase.  String p_t is converted to uppercase if lowercase.  If p_s ends
 * first, the substring match is counted as a match.
 * Returns 0 if strings match, nonzero otherwise.
 */
int utl_strucmp ( char *p_s, char *p_t )
{
    char c;

    while ( *p_s && *p_t )
    {
        c = *p_t++;
        if ( c >= 'a' && c <= 'z' )
        {
            c -= 0x20;
        }
        if ( *p_s++ != c )
        {
            return -1;
        }
    }
    /* if p_t hit null first, no match */
    if ( *p_t == 0 && *p_s != 0 )
    {
        return 1;
    }
    /* else p_s hit null first, count as match */
    else
    {
        return 0;
    }
}

/*
 * This utility counts the characteers in a string
 * Returns  number of characters ( excluding the terminating '0' )
 */
int utl_strlen( char *p_str )
{
    register int  xx = 0;

    while ( *p_str++ != 0 )
        xx++;

    return ( xx );
}

/*
 * This utility function converts a character string to an integer.  Acceptable
 * values in string are 0-9.  If invalid string or string value too large, -1
 * is returned.  Leading spaces are skipped.
 * Returns          Integer value or -1 on error.
 */
INT16 utl_str2int( char *p_s )
{
    INT32   val = 0;

    for ( ; *p_s == ' ' && *p_s != 0; p_s++ );

    if ( *p_s == 0 ) return -1;

    for ( ;; )
    {
        if ( ( *p_s < '0' ) || ( *p_s > '9' ) ) return -1;

        val += ( INT32 ) ( *p_s++ - '0' );

        if ( val > 32767 ) return -1;

        if ( *p_s == 0 )
        {
            return ( INT16 ) val;
        }
        else
        {
            val *= 10;
        }
    }
}

/*
 * Copy bd addr b to a.
 */
void utl_bdcpy( BD_ADDR a, BD_ADDR b )
{
    int i;

    for ( i = BD_ADDR_LEN; i != 0; i-- )
    {
        *a++ = *b++;
    }
}

/* Handle misc command group */
void hci_control_misc_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len )
{
    switch( cmd_opcode )
    {
    case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
        hci_control_misc_handle_get_version();
        break;
    }
}


/* Handle get version command */
void hci_control_misc_handle_get_version( void )
{
    uint8_t   tx_buf[15];
    uint8_t   cmd = 0;
    uint32_t  chip = CHIP;

    tx_buf[cmd++] = WICED_SDK_MAJOR_VER;
    tx_buf[cmd++] = WICED_SDK_MINOR_VER;
    tx_buf[cmd++] = WICED_SDK_REV_NUMBER;
    tx_buf[cmd++] = WICED_SDK_BUILD_NUMBER & 0xFF;
    tx_buf[cmd++] = (WICED_SDK_BUILD_NUMBER>>8) & 0xFF;
    tx_buf[cmd++] = chip & 0xFF;
    tx_buf[cmd++] = (chip>>8) & 0xFF;
    tx_buf[cmd++] = (chip>>24) & 0xFF;
    tx_buf[cmd++] = 0; // not used

    /* Send MCU app the supported features */
    tx_buf[cmd++] = HCI_CONTROL_GROUP_AG;
    tx_buf[cmd++] = HCI_CONTROL_GROUP_GATT;
    tx_buf[cmd++] = HCI_CONTROL_GROUP_SPP;

    wiced_transport_send_data( HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd );
}
