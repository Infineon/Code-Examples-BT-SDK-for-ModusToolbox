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
 * Message Access Client Sample Application
 *
 * The Message Access Client application is designed to connect and access
 * service on the Message Access Server device. It can be used to access
 * SMS-MMS messages received on the Message Access Server device such as
 * a smartphone.
 *
 * Features demonstrated
 *  - MAS instance discovery
 *  - Connect/disconnect to MAP server
 *  - Get folder listing
 *  - Change folder
 *  - Get message listing
 *  - Get message
 *
 * To demonstrate the app, work through the following steps.
 * 1. Plug the WICED eval board into your computer
 * 2. Build and download the application (to the WICED board)
 * 3. Launch Client Control application
 * 4. From Client Control UI, Open port to connect to the eval board
 * 5. Start BR/EDR discovery
 * 6. Select the MAP server device to be connected
 * 7. In MAP Client tab, click Connect
 * 8. Accept pairing request and MAP access request on the phone
 * 9. On the Client Control UI, click a folder (such as inbox) in the
 *    folder list to display the list of messages in this folder
 * 10. Click a message ID in the message list to show message content
 *
 */

#include "sparcommon.h"
#include "string.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_trace.h"
#include "wiced_platform.h"
#include "wiced_memory.h"
#include "wiced_transport.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_wdog.h"
#include "hci_control_api.h"
#include "wiced_bt_mce_api.h"

/*****************************************************************************
**  Constants
*****************************************************************************/
#define WICED_MCE_EIR_BUF_MAX_SIZE              254
#define KEY_INFO_POOL_BUFFER_SIZE               145 //Size of the buffer used for holding the peer device key info
#define KEY_INFO_POOL_BUFFER_COUNT              10  //Corresponds to the number of peer devices

#define READ_LITTLE_ENDIAN_TO_UINT16(into, m,dl)      \
        (into) = ((m)[0] | ((m)[1]<<8));\
        (m) +=2; (dl)-=2;

#define MAP_CLIENT_NVRAM_VSID_LOCAL_KEYS        (WICED_NVRAM_VSID_START + 1)

#define WICED_BT_MCE_SECURITY   (BTM_SEC_IN_AUTHENTICATE | BTM_SEC_OUT_AUTHENTICATE | BTM_SEC_ENCRYPT)

#define MAP_CLIENT_MAX_FOLDER_NAME_LEN          128

#define HCI_MAX_DATA_LEN                        200

/*****************************************************************************
**  Structures
*****************************************************************************/
typedef struct
{
    char     folder[MAP_CLIENT_MAX_FOLDER_NAME_LEN];
    uint8_t  *p_push_msg;
    uint16_t push_msg_offset;
} map_client_cb_t;

typedef struct
{
    uint8_t type;
    uint8_t length;
    uint8_t value[1];
} tlv_t;

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];

/******************************************************
 *               Variables Definitions
 ******************************************************/
/*
 * This is the SDP database for the whole hci_control application
 */
const uint8_t map_client_sdp_db[] =
{
    SDP_ATTR_SEQUENCE_1(65),                                                // length is the sum of all records

    // SDP record for Message Notification service ( total length of record: 65 )
    SDP_ATTR_SEQUENCE_1(63),                                                // 2 bytes
        SDP_ATTR_RECORD_HANDLE(0x10001),                                    // 8 bytes
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_MESSAGE_NOTIFICATION),             // 8 bytes
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(17),   // 22 bytes
            SDP_ATTR_SEQUENCE_1(3),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
            SDP_ATTR_SEQUENCE_1(5),
                SDP_ATTR_UUID16(UUID_PROTOCOL_RFCOMM),
                SDP_ATTR_VALUE_UINT1(WICED_BT_MNS_RFCOMM_SCN),
            SDP_ATTR_SEQUENCE_1(3),
                SDP_ATTR_UUID16(UUID_PROTOCOL_OBEX),
        SDP_ATTR_SERVICE_NAME(7),                                           // 12 bytes
        'M', 'A', 'P', ' ', 'M', 'N', 'S',
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
        SDP_ATTR_PROFILE_DESC_LIST(UUID_SERVCLASS_MAP_PROFILE, 0x0102),     // 13 byte
        SDP_ATTR_ID(ATTR_ID_SUPPORTED_FEATURES_32), SDP_ATTR_SEQUENCE_1(7), // 12 byte
            SDP_ATTR_SEQUENCE_1(5),
                SDP_ATTR_VALUE_UINT4(WICED_BT_MA_DEFAULT_SUPPORTED_FEATURES),
        SDP_ATTR_ID(ATTR_ID_OBX_OVR_L2CAP_PSM), SDP_ATTR_SEQUENCE_1(5),     // 10 byte
            SDP_ATTR_SEQUENCE_1(3),
                SDP_ATTR_VALUE_UINT2(WICED_BT_MNS_L2CAP_PSM)
#else
        SDP_ATTR_PROFILE_DESC_LIST(UUID_SERVCLASS_MAP_PROFILE, 0x0100),     // 13 byte
#endif
};

/* MCE control block */
map_client_cb_t map_client_cb;

wiced_bt_buffer_pool_t* p_key_info_pool;  //Pool for storing the  key info

static uint32_t hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length );

const wiced_transport_cfg_t  transport_cfg =
{
    WICED_TRANSPORT_UART,
    {{ WICED_TRANSPORT_UART_HCI_MODE, HCI_UART_DEFAULT_BAUD }},
    { 0, 0 },
    NULL,
    hci_control_proc_rx_cmd,
    NULL
};

uint8_t pairing_allowed = 0;


/******************************************************
 *               Function Declarations
 ******************************************************/
static wiced_result_t map_client_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static void     map_client_app_init();
static void     map_client_write_eir();
static void     hci_control_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data );

static void     map_client_event_cback(wiced_bt_mce_evt_t event, wiced_bt_mce_t *p_data);
static void     map_client_handle_discover_event(wiced_bt_mce_discover_t * p_discover);
static void     map_client_handle_open_event(wiced_bt_mce_ma_open_close_t * p_open);
static void     map_client_handle_close_event(wiced_bt_mce_ma_open_close_t * p_close);
static void     map_client_handle_set_msg_status_event(wiced_bt_mce_set_msg_status_t * p_set_status);
static void     map_client_handle_update_inbox_event(wiced_bt_mce_update_inbox_t * p_update_inbox);
static void     map_client_handle_set_folder_event(wiced_bt_mce_set_folder_t * p_set_folder);
static void     map_client_handle_folder_list_event(wiced_bt_mce_list_data_t * p_list_data);
static void     map_client_handle_msg_list_event(wiced_bt_mce_list_data_t * p_list_data);
static void     map_client_handle_get_msg_event(wiced_bt_mce_get_msg_t * p_get_msg);
static void     map_client_handle_push_msg_event(wiced_bt_mce_push_msg_t * p_push_msg);
static void     map_client_handle_abort_event(wiced_bt_mce_abort_t * p_abort);
static void     map_client_handle_notif_reg_event(wiced_bt_mce_notif_reg_t * p_reg);
static void     map_client_handle_notif_event(wiced_bt_mce_notif_t * p_notif);

static void     map_client_handle_mce_command(uint16_t cmd_opcode, uint8_t *p_data, uint32_t data_len);
static void     map_client_handle_get_mas_instances(uint8_t *p_data, uint32_t data_len);
static void     map_client_handle_connect(uint8_t *p_data, uint32_t data_len);
static void     map_client_handle_disconnect(uint8_t *p_data, uint32_t data_len);
static void     map_client_handle_list_folders(uint8_t *p_data, uint32_t data_len);
static void     map_client_handle_set_folder(uint8_t *p_data, uint32_t data_len);
static void     map_client_handle_list_messages(uint8_t *p_data, uint32_t data_len);
static void     map_client_handle_get_message(uint8_t *p_data, uint32_t data_len);
static void     map_client_handle_push_message(uint8_t *p_data, uint32_t data_len);
static void     map_client_handle_update_inbox(uint8_t *p_data, uint32_t data_len);
static void     map_client_handle_set_message_status(uint8_t *p_data, uint32_t data_len);
static void     map_client_handle_abort(uint8_t *p_data, uint32_t data_len);
static void     map_client_handle_notif_reg(uint8_t *p_data, uint32_t data_len);

static tlv_t *  wiced_find_tlv(uint8_t *p_data, int data_len, uint8_t type);
static uint16_t wiced_add_tlv(uint8_t *p_buf, uint8_t type, uint8_t *p_value, uint8_t value_len);

extern void     hci_control_device_handle_command( uint16_t cmd_opcode, uint8_t *p_data, uint32_t data_len );
extern void     hci_control_misc_handle_command( uint16_t cmd_opcode, uint8_t *p_data, uint32_t data_len );
extern void     hci_control_send_device_started_evt( void );
extern void     hci_control_send_pairing_completed_evt( uint8_t status , wiced_bt_device_address_t bdaddr );
extern void     hci_control_send_encryption_changed_evt( uint8_t encrypted ,  wiced_bt_device_address_t bdaddr );
extern int      hci_control_find_nvram_id(uint8_t *p_data, int len);
extern int      hci_control_read_nvram( int nvram_id, void *p_data, int data_len );
extern int      hci_control_write_nvram( int nvram_id, int data_len, void *p_data, BOOLEAN from_host );

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 *  Application Start, ie, entry point to the application.
 */
APPLICATION_START( )
{
    wiced_transport_init(&transport_cfg);

#ifdef WICED_BT_TRACE_ENABLE
    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
    //wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#if ( defined(CYW20706A2) || defined(CYW20735B0) || defined(CYW20719B0) || defined(CYW43012C0) )
    wiced_hal_puart_select_uart_pads(WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif
#ifndef CYW20706A2
    wiced_hal_puart_set_baudrate(921600);
#endif
#endif

    /* Register the dynamic configurations */
    wiced_bt_stack_init(map_client_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);
}

/*
 *  Management callback receives various notifications from the stack
 */
wiced_result_t map_client_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                     result = WICED_BT_SUCCESS;
    wiced_bt_dev_status_t              dev_status;
    wiced_bt_dev_ble_pairing_info_t   *p_pairing_info;
    wiced_bt_dev_encryption_status_t  *p_encryption_status;
    int                                nvram_id;
    uint8_t                            pairing_result;

    WICED_BT_TRACE("map_client_management_callback 0x%02x\n", event);

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            map_client_app_init();

            hci_control_send_device_started_evt();
            break;

        case BTM_DISABLED_EVT:
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
            pairing_result = p_event_data->pairing_complete.pairing_complete_info.br_edr.status;
            hci_control_send_pairing_completed_evt(pairing_result, p_event_data->pairing_complete.bd_addr);
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_encryption_status = &p_event_data->encryption_status;
            hci_control_send_encryption_changed_evt(p_encryption_status->result, p_encryption_status->bd_addr);
            break;

        case BTM_SECURITY_REQUEST_EVT:
            if (pairing_allowed)
            {
                wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
            }
            else
            {
                // Pairing not allowed, return error
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            if ((nvram_id = hci_control_find_nvram_id(p_event_data->paired_device_link_keys_update.bd_addr, BD_ADDR_LEN)) == 0)
            {
                // This is the first time, allocate id for the new memory chunk
                nvram_id = hci_control_alloc_nvram_id();
                WICED_BT_TRACE( "Allocated NVRAM ID:%d\n", nvram_id );
            }
            hci_control_write_nvram(nvram_id, sizeof(wiced_bt_device_link_keys_t), &p_event_data->paired_device_link_keys_update, WICED_FALSE);
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            if ((nvram_id = hci_control_find_nvram_id(p_event_data->paired_device_link_keys_request.bd_addr,
                            BD_ADDR_LEN)) != 0)
            {
                hci_control_read_nvram(nvram_id, &p_event_data->paired_device_link_keys_request, sizeof(wiced_bt_device_link_keys_t));
            }
            else
            {
                result = WICED_BT_ERROR;
                WICED_BT_TRACE("Link key not found\n");
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* Request to store newly generated local identity keys to NVRAM */
            wiced_hal_write_nvram (MAP_CLIENT_NVRAM_VSID_LOCAL_KEYS, sizeof(wiced_bt_local_identity_keys_t), (uint8_t*)&p_event_data->local_identity_keys_update, &result);
            break;


        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /* Request to restore local identity keys from NVRAM */
            if (wiced_hal_read_nvram(MAP_CLIENT_NVRAM_VSID_LOCAL_KEYS, sizeof(wiced_bt_local_identity_keys_t), (uint8_t *)&p_event_data->local_identity_keys_request, &result)
                    != sizeof(wiced_bt_local_identity_keys_t))
            {
                result = WICED_BT_ERROR;
                WICED_BT_TRACE("Local key retrieval failure\n");
            }
            break;

        default:
            result = WICED_BT_USE_DEFAULT_SECURITY;
            break;
    }
    return result;
}

void map_client_app_init()
{
    map_client_write_eir();

    /* initialize everything */
    memset(&map_client_cb, 0, sizeof(map_client_cb));

    /* Initialize MAP library */
    wiced_bt_mce_enable(map_client_event_cback, 0);

    /* create SDP records */
    wiced_bt_sdp_db_init((uint8_t *)map_client_sdp_db, sizeof(map_client_sdp_db));

    /* Start MN service */
    wiced_bt_mce_mn_start(WICED_BT_MCE_SECURITY, "MAP MNS", WICED_BT_MA_DEFAULT_SUPPORTED_FEATURES);

    /* Creating a buffer pool for holding the peer devices's key info */
    p_key_info_pool = wiced_bt_create_pool( KEY_INFO_POOL_BUFFER_SIZE, KEY_INFO_POOL_BUFFER_COUNT);

    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace(hci_control_hci_trace_cback);
}

/*
 *  Prepare extended inquiry response data.  Current version publishes Message
 *  Notification service.
 */
void map_client_write_eir()
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t length;

    pBuf = (uint8_t*)wiced_bt_get_buffer( WICED_MCE_EIR_BUF_MAX_SIZE );
    WICED_BT_TRACE( "map_client_write_eir %x\n", pBuf );

    if ( !pBuf )
    {
        return;
    }
    p = pBuf;

    length = strlen( (char *)wiced_bt_cfg_settings.device_name );

    *p++ = length + 1;
    *p++ = BT_EIR_COMPLETE_LOCAL_NAME_TYPE;
    memcpy( p, wiced_bt_cfg_settings.device_name, length );
    p += length;
    *p++ = (1 * 2 ) + 1;      // length of services + 1
    *p++ = BT_EIR_COMPLETE_16BITS_UUID_TYPE;
    *p++ =   UUID_SERVCLASS_MESSAGE_NOTIFICATION        & 0xff;
    *p++ = ( UUID_SERVCLASS_MESSAGE_NOTIFICATION >> 8 ) & 0xff;
    *p++ = 0;

    WICED_BT_TRACE_ARRAY( ( uint8_t *)( pBuf+1 ), MIN( p-( uint8_t *)pBuf,100 ), "EIR :" );
    wiced_bt_dev_write_eir( pBuf, (uint16_t)(p - pBuf) );

    return;
}

/*
 *  Pass protocol traces up through the UART
 */
void hci_control_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    //Enable below to receive traces over HCI UART send the trace
    wiced_transport_send_hci_trace( NULL, type, length, p_data  );
}

void map_client_event_cback(wiced_bt_mce_evt_t event, wiced_bt_mce_t *p_data)
{
    switch (event)
    {
    case WICED_BT_MCE_ENABLE_EVT:
        break;

    case WICED_BT_MCE_START_EVT:
        break;

    case WICED_BT_MCE_STOP_EVT:
        break;

    case WICED_BT_MCE_DISCOVER_EVT:
        map_client_handle_discover_event(&p_data->discover);
        break;

    case WICED_BT_MCE_MA_OPEN_EVT:
        map_client_handle_open_event(&p_data->ma_open);
        break;

    case WICED_BT_MCE_MA_CLOSE_EVT:
        map_client_handle_close_event(&p_data->ma_close);
        break;

    case WICED_BT_MCE_SET_MSG_STATUS_EVT:
        map_client_handle_set_msg_status_event(&p_data->set_msg_sts);
        break;

    case WICED_BT_MCE_UPDATE_INBOX_EVT:
        map_client_handle_update_inbox_event(&p_data->upd_ibx);
        break;

    case WICED_BT_MCE_SET_FOLDER_EVT:
        map_client_handle_set_folder_event(&p_data->set_folder);
        break;

    case WICED_BT_MCE_FOLDER_LIST_EVT:
        map_client_handle_folder_list_event(&p_data->list_data);
        break;

    case WICED_BT_MCE_MSG_LIST_EVT:
        map_client_handle_msg_list_event(&p_data->list_data);
        break;

    case WICED_BT_MCE_GET_MSG_EVT:
        map_client_handle_get_msg_event(&p_data->get_msg);
        break;

    case WICED_BT_MCE_PUSH_MSG_EVT:
        map_client_handle_push_msg_event(&p_data->push_msg);
        break;

    case WICED_BT_MCE_ABORT_EVT:
        map_client_handle_abort_event(&p_data->abort);
        break;

    case WICED_BT_MCE_NOTIF_REG_EVT:
        map_client_handle_notif_reg_event(&p_data->notif_reg);
        break;

    case WICED_BT_MCE_NOTIF_EVT:
        map_client_handle_notif_event(&p_data->notif);
        break;

    default:
        WICED_BT_TRACE("MAP client event %d not supported\n", event);
        break;
    }

}

void map_client_handle_discover_event(wiced_bt_mce_discover_t * p_discover)
{
    uint8_t buf[HCI_MAX_DATA_LEN];
    uint8_t *p = buf;
    uint8_t status;
    uint8_t i;

    WICED_BT_TRACE("map_client_handle_discover_event status %d\n", p_discover->status);

    status = p_discover->status == WICED_BT_SDP_SUCCESS ? WICED_BT_MA_STATUS_OK : WICED_BT_MA_STATUS_FAIL;
    p += wiced_add_tlv(p, HCI_CONTROL_MCE_PARAM_STATUS, &status, 1);

    if (status == WICED_BT_MA_STATUS_OK)
    {
        p += wiced_add_tlv(p, HCI_CONTROL_MCE_PARAM_NUM_MAS_INST, &p_discover->num_mas_srv, 1);

        for (i = 0; i < p_discover->num_mas_srv; i++)
        {
            p += wiced_add_tlv(p, HCI_CONTROL_MCE_PARAM_MAS_INS_ID, &p_discover->rec[i].mas_inst_id, 1);
            p += wiced_add_tlv(p, HCI_CONTROL_MCE_PARAM_NAME, (uint8_t *)p_discover->rec[i].name, strlen(p_discover->rec[i].name));
            p += wiced_add_tlv(p, HCI_CONTROL_MCE_PARAM_SUPPORTED_TYPE, &p_discover->rec[i].supported_msg_type, 1);
        }
    }

    wiced_transport_send_data(HCI_CONTROL_MCE_EVENT_MAS_INSTANCES, buf, p - buf);
}

void map_client_handle_open_event(wiced_bt_mce_ma_open_close_t * p_open)
{
    uint8_t buf[HCI_MAX_DATA_LEN];
    uint8_t *p = buf;

    WICED_BT_TRACE("map_client_handle_open_event status %d\n", p_open->status);

    p += wiced_add_tlv(p, HCI_CONTROL_MCE_PARAM_STATUS, &p_open->status, 1);
    p += wiced_add_tlv(p, HCI_CONTROL_MCE_PARAM_BDA, p_open->bd_addr, BD_ADDR_LEN);
    p += wiced_add_tlv(p, HCI_CONTROL_MCE_PARAM_MAS_INS_ID, &p_open->mas_inst_id, 1);

    if (p_open->status == WICED_BT_MA_STATUS_OK)
    {
        p += wiced_add_tlv(p, HCI_CONTROL_MCE_PARAM_SESS_HANDLE, (uint8_t *)&p_open->session_id, 2);
    }

    wiced_transport_send_data(HCI_CONTROL_MCE_EVENT_CONNECTED, buf, p - buf);
}

void map_client_handle_close_event(wiced_bt_mce_ma_open_close_t * p_close)
{
    uint8_t buf[16];
    uint8_t *p = buf;

    WICED_BT_TRACE("map_client_handle_close_event status %d\n", p_close->status);

    p += wiced_add_tlv(p, HCI_CONTROL_MCE_PARAM_STATUS, &p_close->status, 1);
    p += wiced_add_tlv(p, HCI_CONTROL_MCE_PARAM_SESS_HANDLE, (uint8_t *)&p_close->session_id, 2);

    wiced_transport_send_data(HCI_CONTROL_MCE_EVENT_DISCONNECTED, buf, p - buf);
}

static void map_client_send_hci_status(uint16_t code, uint8_t status)
{
    uint8_t buf[16];
    uint8_t *p = buf;

    p += wiced_add_tlv(p, HCI_CONTROL_MCE_PARAM_STATUS, &status, 1);

    wiced_transport_send_data(code, buf, p - buf);
}

void map_client_handle_set_msg_status_event(wiced_bt_mce_set_msg_status_t * p_set_status)
{
    WICED_BT_TRACE("map_client_handle_set_msg_status_event status %d\n", p_set_status->status);

    map_client_send_hci_status(HCI_CONTROL_MCE_EVENT_MESSAGE_STATUS_SET, p_set_status->status);
}

void map_client_handle_update_inbox_event(wiced_bt_mce_update_inbox_t * p_update_inbox)
{
    WICED_BT_TRACE("map_client_handle_update_inbox_event status %d\n", p_update_inbox->status);

    map_client_send_hci_status(HCI_CONTROL_MCE_EVENT_INBOX_UPDATED, p_update_inbox->status);
}

void map_client_handle_set_folder_event(wiced_bt_mce_set_folder_t * p_set_folder)
{
    WICED_BT_TRACE("map_client_handle_set_folder_event status %d\n", p_set_folder->status);

    map_client_send_hci_status(HCI_CONTROL_MCE_EVENT_FOLDER_SET, p_set_folder->status);
}

static void map_client_send_hci_data(uint16_t code, uint8_t status, uint8_t *p_data, uint16_t data_len, wiced_bool_t is_final)
{
    uint8_t buf[HCI_MAX_DATA_LEN];
    uint8_t *p = buf;
    uint16_t available_len;
    uint8_t send_len;
    uint8_t param_type;

    p += wiced_add_tlv(p, HCI_CONTROL_MCE_PARAM_STATUS, &status, 1);

    if (status == WICED_BT_MA_STATUS_OK)
    {
        if (data_len == 0 && is_final)
        {
            p += wiced_add_tlv(p, HCI_CONTROL_MCE_PARAM_DATA_END, p_data, 0);

            wiced_transport_send_data(code, buf, p - buf);
        }
        else
        {
            uint8_t *p_temp = p;

            available_len = HCI_MAX_DATA_LEN - (p - buf) - 2;

            while (data_len)
            {
                p = p_temp;

                param_type = HCI_CONTROL_MCE_PARAM_DATA;
                if (data_len > available_len)
                {
                    send_len = available_len;
                }
                else
                {
                    send_len = data_len;
                    if (is_final)
                        param_type = HCI_CONTROL_MCE_PARAM_DATA_END;
                }
                p += wiced_add_tlv(p, param_type, p_data, send_len);

                wiced_transport_send_data(code, buf, p - buf);

                p_data += send_len;
                data_len -= send_len;
            }
        }
    }
    else
        wiced_transport_send_data(code, buf, p - buf);
}

void map_client_handle_folder_list_event(wiced_bt_mce_list_data_t * p_list_data)
{
    WICED_BT_TRACE("map_client_handle_folder_list_event status %d\n", p_list_data->status);

    map_client_send_hci_data(HCI_CONTROL_MCE_EVENT_FOLDER_LIST, p_list_data->status,
            p_list_data->p_data, p_list_data->len, p_list_data->is_final);
}

void map_client_handle_msg_list_event(wiced_bt_mce_list_data_t * p_list_data)
{
    WICED_BT_TRACE("map_client_handle_msg_list_event status %d\n", p_list_data->status);

    map_client_send_hci_data(HCI_CONTROL_MCE_EVENT_MESSAGE_LIST, p_list_data->status,
            p_list_data->p_data, p_list_data->len, p_list_data->is_final);
}

void map_client_handle_get_msg_event(wiced_bt_mce_get_msg_t * p_get_msg)
{
    WICED_BT_TRACE("map_client_handle_get_msg_event status %d\n", p_get_msg->status);

    map_client_send_hci_data(HCI_CONTROL_MCE_EVENT_MESSAGE, p_get_msg->status,
            p_get_msg->p_data, p_get_msg->len, p_get_msg->is_final);
}

void map_client_handle_push_msg_event(wiced_bt_mce_push_msg_t * p_push_msg)
{
    uint8_t buf[16];
    uint8_t *p = buf;

    WICED_BT_TRACE("map_client_handle_push_msg_event status %d\n", p_push_msg->status);

    p += wiced_add_tlv(p, HCI_CONTROL_MCE_PARAM_STATUS, &p_push_msg->status, 1);

    if (p_push_msg->status == WICED_BT_MA_STATUS_OK)
    {
        p += wiced_add_tlv(p, HCI_CONTROL_MCE_PARAM_MSG_HANDLE, p_push_msg->msg_handle, WICED_BT_MA_HANDLE_SIZE);
    }

    wiced_transport_send_data(HCI_CONTROL_MCE_EVENT_MESSAGE_PUSHED, buf, p - buf);
}

void map_client_handle_abort_event(wiced_bt_mce_abort_t * p_abort)
{
    WICED_BT_TRACE("map_client_handle_abort_event status %d\n", p_abort->status);

    map_client_send_hci_status(HCI_CONTROL_MCE_EVENT_ABORTED, p_abort->status);
}

void map_client_handle_notif_reg_event(wiced_bt_mce_notif_reg_t * p_reg)
{
    WICED_BT_TRACE("map_client_handle_notif_reg_event status %d\n", p_reg->status);

    map_client_send_hci_status(HCI_CONTROL_MCE_EVENT_NOTIF_REG, p_reg->status);
}

void map_client_handle_notif_event(wiced_bt_mce_notif_t * p_notif)
{
    WICED_BT_TRACE("map_client_handle_notif_event status %d, len %d\n data %s\n", p_notif->status, p_notif->len, p_notif->p_object);

    map_client_send_hci_data(HCI_CONTROL_MCE_EVENT_NOTIF, p_notif->status,
            p_notif->p_object, p_notif->len, p_notif->final);
}

/*
 * Handle received command over UART. Please refer to the WICED HCI Control Protocol
 * Software User Manual (WICED-SWUM10x-R) for details on the
 * HCI UART control protocol.
*/
uint32_t hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t *p_rx_buf = p_data;

    WICED_BT_TRACE("hci_control_proc_rx_cmd:%d, %x\n", length, p_rx_buf);

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

    case HCI_CONTROL_GROUP_MCE:
        map_client_handle_mce_command( opcode, p_data, payload_len );
        break;

    case HCI_CONTROL_GROUP_MISC:
        hci_control_misc_handle_command(opcode, p_data, payload_len);
        break;

    default:
        WICED_BT_TRACE("unknown class code\n");
        break;
    }

    //Freeing the buffer in which data is received
    wiced_transport_free_buffer( p_rx_buf );
    return status;
}

void map_client_handle_mce_command( uint16_t cmd_opcode, uint8_t *p_data, uint32_t data_len )
{
    switch (cmd_opcode)
    {
    case HCI_CONTROL_MCE_COMMAND_GET_MAS_INSTANCES:
        map_client_handle_get_mas_instances(p_data, data_len);
        break;

    case HCI_CONTROL_MCE_COMMAND_CONNECT:
        map_client_handle_connect(p_data, data_len);
        break;

    case HCI_CONTROL_MCE_COMMAND_DISCONNECT:
        map_client_handle_disconnect(p_data, data_len);
        break;

    case HCI_CONTROL_MCE_COMMAND_LIST_FOLDERS:
        map_client_handle_list_folders(p_data, data_len);
        break;

    case HCI_CONTROL_MCE_COMMAND_SET_FOLDER:
        map_client_handle_set_folder(p_data, data_len);
        break;

    case HCI_CONTROL_MCE_COMMAND_LIST_MESSAGES:
        map_client_handle_list_messages(p_data, data_len);
        break;

    case HCI_CONTROL_MCE_COMMAND_GET_MESSAGE:
        map_client_handle_get_message(p_data, data_len);
        break;

    case HCI_CONTROL_MCE_COMMAND_PUSH_MESSAGE:
        map_client_handle_push_message(p_data, data_len);
        break;

    case HCI_CONTROL_MCE_COMMAND_UPDATE_INBOX:
        map_client_handle_update_inbox(p_data, data_len);
        break;

    case HCI_CONTROL_MCE_COMMAND_SET_MESSAGE_STATUS:
        map_client_handle_set_message_status(p_data, data_len);
        break;

    case HCI_CONTROL_MCE_COMMAND_ABORT:
        map_client_handle_abort(p_data, data_len);
        break;

    case HCI_CONTROL_MCE_COMMAND_NOTIF_REG:
        map_client_handle_notif_reg(p_data, data_len);
        break;

    default:
        WICED_BT_TRACE("Unknown command code 0x%x\n", cmd_opcode);
        break;
    }
}

void map_client_handle_get_mas_instances(uint8_t *p_data, uint32_t data_len)
{
    wiced_bt_device_address_t bd_addr;
    tlv_t *p_param;
    uint8_t *p;

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_BDA);
    p = p_param->value;
    STREAM_TO_BDADDR(bd_addr, p);

    wiced_bt_mce_discover(bd_addr);
}

void map_client_handle_connect(uint8_t *p_data, uint32_t data_len)
{
    wiced_bt_device_address_t bd_addr;
    wiced_bt_ma_inst_id_t mas_instance_id;
    tlv_t *p_param;
    uint8_t *p;

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_BDA);
    p = p_param->value;
    STREAM_TO_BDADDR(bd_addr, p);

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_MAS_INS_ID);
    mas_instance_id = p_param->value[0];

    wiced_bt_mce_open(bd_addr, mas_instance_id, WICED_BT_MCE_SECURITY);
}

void map_client_handle_disconnect(uint8_t *p_data, uint32_t data_len)
{
    wiced_bt_ma_sess_handle_t sess_handle;
    tlv_t *p_param;
    uint8_t *p;

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_SESS_HANDLE);
    p = p_param->value;
    STREAM_TO_UINT16(sess_handle, p);

    wiced_bt_mce_close(sess_handle);
}

void map_client_handle_list_folders(uint8_t *p_data, uint32_t data_len)
{
    wiced_bt_ma_sess_handle_t sess_handle;
    tlv_t *p_param;
    uint8_t *p;

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_SESS_HANDLE);
    p = p_param->value;
    STREAM_TO_UINT16(sess_handle, p);

    wiced_bt_mce_get_folder_list(sess_handle, 1024, 0);
}

void map_client_handle_set_folder(uint8_t *p_data, uint32_t data_len)
{
    wiced_bt_ma_sess_handle_t sess_handle;
    wiced_bt_ma_dir_nav_t flag;
    tlv_t *p_param;
    uint8_t *p;

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_SESS_HANDLE);
    p = p_param->value;
    STREAM_TO_UINT16(sess_handle, p);

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_NAV_FLAG);
    flag = p_param->value[0];

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_FOLDER);
    memcpy(map_client_cb.folder, p_param->value, p_param->length);
    map_client_cb.folder[p_param->length] = 0;

    wiced_bt_mce_set_folder(sess_handle, flag, map_client_cb.folder);
}

void map_client_handle_list_messages(uint8_t *p_data, uint32_t data_len)
{
    wiced_bt_ma_sess_handle_t sess_handle;
    tlv_t *p_param;
    uint8_t *p;

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_SESS_HANDLE);
    p = p_param->value;
    STREAM_TO_UINT16(sess_handle, p);

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_FOLDER);
    if (p_param)
    {
        memcpy(map_client_cb.folder, p_param->value, p_param->length);
        map_client_cb.folder[p_param->length] = 0;
    }
    else
        map_client_cb.folder[0] = 0;

    wiced_bt_mce_get_msg_list(sess_handle, map_client_cb.folder, NULL);
}

void map_client_handle_get_message(uint8_t *p_data, uint32_t data_len)
{
    wiced_bt_ma_sess_handle_t sess_handle;
    wiced_bt_ma_get_msg_param_t msg_params;
    tlv_t *p_param;
    uint8_t *p;

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_SESS_HANDLE);
    p = p_param->value;
    STREAM_TO_UINT16(sess_handle, p);

    memset(&msg_params, 0, sizeof(wiced_bt_ma_get_msg_param_t));
    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_MSG_HANDLE);
    if (p_param->length >= WICED_BT_MA_HANDLE_SIZE)
    {
        WICED_BT_TRACE("Wrong handle size:%d\n", p_param->length);
        return;
    }
    memcpy(msg_params.handle, p_param->value, p_param->length);
    msg_params.charset = WICED_BT_MA_CHARSET_UTF_8;
    msg_params.fraction_request = WICED_BT_MA_FRAC_REQ_NO;

    wiced_bt_mce_get_msg(sess_handle, &msg_params);
}

void map_client_handle_push_message(uint8_t *p_data, uint32_t data_len)
{
    wiced_bt_ma_sess_handle_t sess_handle;
    wiced_bt_ma_push_msg_param_t push_params;
    tlv_t *p_param;
    uint8_t *p;

    memset(&push_params, 0, sizeof(wiced_bt_ma_push_msg_param_t));

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_SESS_HANDLE);
    p = p_param->value;
    STREAM_TO_UINT16(sess_handle, p);

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_FOLDER);
    if (p_param)
    {
        memcpy(map_client_cb.folder, p_param->value, p_param->length);
        map_client_cb.folder[p_param->length] = 0;
        push_params.p_folder = map_client_cb.folder;
    }

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_DATA);
    if (!p_param)
    {
        p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_DATA_END);
        if (p_param)
        {
            push_params.is_final = WICED_TRUE;
        }
        else
        {
            map_client_send_hci_status(HCI_CONTROL_MCE_EVENT_MESSAGE_PUSHED, WICED_BT_MA_STATUS_FAIL);
            return;
        }
    }
    push_params.p_msg = p_param->value;
    push_params.len = p_param->length;
    push_params.charset = WICED_BT_MA_CHARSET_UTF_8;

    wiced_bt_mce_push_msg(sess_handle, &push_params);
}

void map_client_handle_update_inbox(uint8_t *p_data, uint32_t data_len)
{
    wiced_bt_ma_sess_handle_t sess_handle;
    tlv_t *p_param;
    uint8_t *p;

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_SESS_HANDLE);
    p = p_param->value;
    STREAM_TO_UINT16(sess_handle, p);

    wiced_bt_mce_update_inbox(sess_handle);
}

void map_client_handle_set_message_status(uint8_t *p_data, uint32_t data_len)
{
    wiced_bt_ma_sess_handle_t sess_handle;
    wiced_bt_ma_msg_handle_t msg_handle;
    wiced_bt_ma_sts_indctr_t indicator;
    wiced_bt_ma_sts_value_t value;
    tlv_t *p_param;
    uint8_t *p;

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_SESS_HANDLE);
    p = p_param->value;
    STREAM_TO_UINT16(sess_handle, p);

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_MSG_HANDLE);
    if (p_param->length >= WICED_BT_MA_HANDLE_SIZE)
    {
        WICED_BT_TRACE("Wrong handle size:%d\n", p_param->length);
        return;
    }
    memset(msg_handle, 0, WICED_BT_MA_HANDLE_SIZE);
    memcpy(msg_handle, p_param->value, p_param->length);

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_MSG_STATUS_INDIC);
    indicator = p_param->value[0];

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_MSG_STATUS_VALUE);
    value = p_param->value[0];

    wiced_bt_mce_set_msg_status(sess_handle, msg_handle, indicator, value);
}

void map_client_handle_abort(uint8_t *p_data, uint32_t data_len)
{
    wiced_bt_device_address_t bd_addr;
    wiced_bt_ma_inst_id_t mas_instance_id;
    tlv_t *p_param;
    uint8_t *p;

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_BDA);
    p = p_param->value;
    STREAM_TO_BDADDR(bd_addr, p);

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_MAS_INS_ID);
    mas_instance_id = p_param->value[0];

    wiced_bt_mce_abort(bd_addr, mas_instance_id);
}

void map_client_handle_notif_reg(uint8_t *p_data, uint32_t data_len)
{
    wiced_bt_ma_sess_handle_t sess_handle;
    wiced_bt_ma_notif_status_t notif_status;
    tlv_t *p_param;
    uint8_t *p;

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_SESS_HANDLE);
    p = p_param->value;
    STREAM_TO_UINT16(sess_handle, p);

    p_param = wiced_find_tlv(p_data, data_len, HCI_CONTROL_MCE_PARAM_NOTIF_STATUS);
    notif_status = p_param->value[0];

    wiced_bt_mce_notif_reg(sess_handle, notif_status);
}

tlv_t *wiced_find_tlv(uint8_t *p_buf, int data_len, uint8_t type)
{
    tlv_t *p_tlv;
    uint16_t tlv_len;

    while (data_len > 0)
    {
        p_tlv = (tlv_t *)p_buf;
        if (p_tlv->type == type)
            return p_tlv;

        tlv_len = p_tlv->length + 2;
        p_buf += tlv_len;
        data_len -= tlv_len;
    }

    return NULL;
}

uint16_t wiced_add_tlv(uint8_t *p_buf, uint8_t type, uint8_t *p_value, uint8_t value_len)
{
    tlv_t *p_tlv = (tlv_t *)p_buf;

    p_tlv->type = type;
    p_tlv->length = value_len;
    memcpy(p_tlv->value, p_value, value_len);

    return value_len + 2;
}
