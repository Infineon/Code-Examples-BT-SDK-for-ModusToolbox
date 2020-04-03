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
 * LE COC Application (Server/Client)
 *
 * Features demonstrated
 *  - WICED BT LE L2CAP APIs for Connection Oriented Channels
 *
 * Server:
 *  - Start advertisements from client control
 *  - Waits for connection from peer application (on the chosen l2cap psm)
 *  - On connection, waits for data from peer
 *  - Upon receiving data from peer, displays received data in the client control
 *
 * Client:
 *  - Scan from the Client control for the Server
 *  - Connect to server and send data
 *
 */

#include "le_coc.h"

#include "sparcommon.h"

#include "wiced_bt_l2c.h"
#include "wiced_result.h"
#include "wiced_transport.h"
#include "wiced_hal_nvram.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"
#include "hci_control_api.h"
#include "wiced_bt_ble.h"
#include "wiced_platform.h"
/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

uint32_t le_coc_proc_rx_cmd(uint8_t *p_data, uint32_t length);
void le_coc_congestion_cback(void *context, UINT16 local_cid, BOOLEAN congested);
void le_coc_connect_cfm_cback(void *context, UINT16 local_cid, UINT16 result, UINT16 mtu_peer);
void le_coc_connect_ind_cback(void *context, BD_ADDR bd_addr, UINT16 local_cid, UINT16 psm, UINT8 id, UINT16 mtu_peer);
void le_coc_connect(wiced_bt_device_address_t bd_addr, wiced_bt_ble_address_type_t bd_addr_type);
void le_coc_data_cback(void *context, UINT16 local_cid, UINT8 *p_buff, UINT16 buf_len);
void le_coc_disconnect_cfm_cback(void *context, UINT16 local_cid, UINT16 result);
void le_coc_disconnect_ind_cback(void *context, UINT16 local_cid, BOOLEAN ack);
void le_coc_disconnect(void);
void le_coc_hci_trace_cback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);
void le_coc_init(void);
void le_coc_send_to_client_control(uint16_t code, uint8_t* p_data, uint16_t length);
void le_coc_set_advertisement_data(void);
void le_coc_transport_status(wiced_transport_type_t type);
void le_coc_tx_complete_cback(void *context, uint16_t local_cid, uint16_t bufcount);
wiced_bt_dev_status_t le_coc_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);

const char* getStackEventStr(wiced_bt_management_evt_t event);
const char* getOpcodeStr(uint16_t opcode);

const wiced_transport_cfg_t transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg =
    {
        .uart_cfg =
        {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
            .baud_rate =  HCI_UART_DEFAULT_BAUD
        },
    },
    .rx_buff_pool_cfg =
    {
        .buffer_size  = 1024,
        .buffer_count = 2
    },
    .p_status_handler    = le_coc_transport_status,
    .p_data_handler      = le_coc_proc_rx_cmd,
    .p_tx_complete_cback = NULL
};

/******************************************************
 *               Variable Definitions
 ******************************************************/
/* Declare other app info for all needed PSMs */
wiced_bt_l2cap_le_appl_information_t l2c_appl_info =
{
    le_coc_connect_ind_cback,
    le_coc_connect_cfm_cback,
    le_coc_disconnect_ind_cback,
    le_coc_disconnect_cfm_cback,
    le_coc_data_cback,
    le_coc_congestion_cback,
    le_coc_tx_complete_cback };

le_coc_cb_t le_coc_cb;
uint16_t psm;
uint16_t mtu;
wiced_transport_buffer_pool_t* rxBuffPoolPtr = NULL;

/******************************************************
 *               Function Definitions
 ******************************************************/
APPLICATION_START()
{
    wiced_transport_init(&transport_cfg);

    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
    WICED_BT_TRACE("[%s]  ***** LE COC ***** \r\n", __func__);

    /* Initialize Bluetooth controller and host stack */
    wiced_bt_stack_init(le_coc_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);
}

void le_coc_load_keys_for_address_resolution(void)
{
    wiced_bt_device_link_keys_t link_keys;
    wiced_result_t result;
    uint8_t *p;

    memset(&link_keys, 0, sizeof(wiced_bt_device_link_keys_t));

    p = (uint8_t*) &link_keys;
    wiced_hal_read_nvram( LE_COC_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), p, &result);

    if (result == WICED_BT_SUCCESS)
    {
        result = wiced_bt_dev_add_device_to_address_resolution_db(&link_keys);
    }
}

/* Bluetooth management event handler */
wiced_bt_dev_status_t le_coc_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_bt_dev_status_t status = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda;
    uint8_t *p_keys;

    WICED_BT_TRACE("[%s] Received %s event from stack \r\n", __func__, getStackEventStr(event));

    switch (event)
    {
        case BTM_ENABLED_EVT:

            /* Register callback for receiving hci traces */
            wiced_bt_dev_register_hci_trace(le_coc_hci_trace_cback);

            /* Allow peer to pair */
            wiced_bt_set_pairable_mode(WICED_TRUE, 0);

            /* Load keys for address resolution */
            le_coc_load_keys_for_address_resolution();

            /* Set advertisement data */
            le_coc_set_advertisement_data();

            break;

        case BTM_SECURITY_REQUEST_EVT:
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE; /* No IO capabilities on this platform */
            p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_BOND; /* Bonding required */
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            WICED_BT_TRACE("[%s] Adv %s \r\n", __func__, (BTM_BLE_ADVERT_OFF != p_event_data->ble_advert_state_changed) ? "started" : "stopped");
            le_coc_send_to_client_control(HCI_CONTROL_LE_COC_EVENT_ADV_STS, &p_event_data->ble_advert_state_changed, 1);
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /* save keys to NVRAM */
            p_keys = (uint8_t*) &p_event_data->paired_device_link_keys_update;
            wiced_hal_write_nvram( LE_COC_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), p_keys, &status);
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* read keys from NVRAM */
            p_keys = (uint8_t *) &p_event_data->paired_device_link_keys_request;
            wiced_hal_read_nvram( LE_COC_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), p_keys, &status);
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* save keys to NVRAM */
            p_keys = (uint8_t*) &p_event_data->local_identity_keys_update;
            wiced_hal_write_nvram( LE_COC_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &status);
            break;

        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /* read keys from NVRAM */
            p_keys = (uint8_t *) &p_event_data->local_identity_keys_request;
            wiced_hal_read_nvram( LE_COC_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &status);
            break;

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
            WICED_BT_TRACE("[%s] Scan %s \r\n", __func__, (BTM_BLE_SCAN_TYPE_NONE != p_event_data->ble_scan_state_changed) ? "started" : "stopped");
            break;

        default:
            WICED_BT_TRACE("[%s] received event (%s) not processed \r\n", __func__, getStackEventStr(event));
            break;
    }

    return (status);
}

/* L2CAP Data RX callback */
void le_coc_data_cback(void *context, UINT16 local_cid, UINT8 *p_data, UINT16 len)
{
    WICED_BT_TRACE("[%s] received %d bytes\r\n", __func__, len);

    /* send the received data to the client control */
    le_coc_send_to_client_control(HCI_CONTROL_LE_COC_EVENT_RX_DATA, p_data, len);

    return;
}

/* L2CAP connection management callback */
void le_coc_connect_ind_cback(void *context, BD_ADDR bda, UINT16 local_cid, UINT16 psm, UINT8 id, UINT16 mtu_peer)
{
    uint8_t *p_data = le_coc_cb.peer_bda;

    WICED_BT_TRACE("[%s] from %B CID %d PSM 0x%x MTU %d \r\n", __func__, bda, local_cid, psm, mtu_peer);

    /* Accept the connection */
    wiced_bt_l2cap_le_connect_rsp(bda, id, local_cid, L2CAP_CONN_OK, mtu, L2CAP_DEFAULT_BLE_CB_POOL_ID);

    /* Store peer info for reference*/
    le_coc_cb.local_cid = local_cid;
    BDADDR_TO_STREAM(p_data, bda);
    le_coc_cb.peer_mtu = mtu_peer;

    /* Stop advertising */
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);

    /* Indicate to client control */
    le_coc_send_to_client_control(HCI_CONTROL_LE_COC_EVENT_CONNECTED, le_coc_cb.peer_bda, BD_ADDR_LEN);
}

void le_coc_connect_cfm_cback(void *context, UINT16 local_cid, UINT16 result, UINT16 mtu_peer)
{
    WICED_BT_TRACE("[%s] result :%02x MTU %d \r\n", __func__, result, mtu_peer);

    if (result == 0)
    {
        /* Store peer info for reference*/
        le_coc_cb.local_cid = local_cid;
        le_coc_cb.peer_mtu = mtu_peer;
    }
    else
    {
        /* For now sending NULL bd address can be accept by CC to consider connection failure. Todo: proper error code handshake */
        memset(le_coc_cb.peer_bda, 0, BD_ADDR_LEN);
    }

    /* Indicate to client control */
    le_coc_send_to_client_control(HCI_CONTROL_LE_COC_EVENT_CONNECTED, le_coc_cb.peer_bda, BD_ADDR_LEN);
}

void le_coc_disconnect_ind_cback(void *context, UINT16 local_cid, BOOLEAN ack)
{
    wiced_bt_device_address_t bda;
    uint16_t reason;
    uint8_t* peer_address = le_coc_cb.peer_bda;

    WICED_BT_TRACE("[%s] CID %d \r\n", __func__, local_cid);

    /* Send disconnect response if needed */
    if (ack)
    {
        wiced_bt_l2cap_le_disconnect_rsp(local_cid);
    }

    if (le_coc_cb.local_cid == local_cid)
    {
        STREAM_TO_BDADDR(bda, peer_address);

        /* Get the disconnect reason - MUST call wiced_bt_l2cap_get_disconnect_reason w/in this context */
        reason = wiced_bt_l2cap_get_disconnect_reason(bda, BT_TRANSPORT_LE);
        WICED_BT_TRACE("Disconnect reason %x\n", reason);

        /* Indicate to client control */
        le_coc_send_to_client_control(HCI_CONTROL_LE_COC_EVENT_DISCONNECTED, le_coc_cb.peer_bda, BD_ADDR_LEN);

        le_coc_cb.local_cid = 0xFFFF;
        memset(le_coc_cb.peer_bda, 0, BD_ADDR_LEN);
    }
}

void le_coc_disconnect_cfm_cback(void *context, UINT16 local_cid, UINT16 result)
{
    WICED_BT_TRACE("[%s] CID %d \r\n", __func__, local_cid);

    if (le_coc_cb.local_cid == local_cid)
    {
        /* Indicate to client control */
        le_coc_send_to_client_control(HCI_CONTROL_LE_COC_EVENT_DISCONNECTED, le_coc_cb.peer_bda, BD_ADDR_LEN);

        le_coc_cb.local_cid = 0xFFFF;
        memset(le_coc_cb.peer_bda, 0, BD_ADDR_LEN);
    }
}

void le_coc_congestion_cback(void *context, UINT16 local_cid, BOOLEAN congested)
{
    WICED_BT_TRACE("[%s] CID %d \r\n", __func__, local_cid);

    if (!(congested))
    {
        //TODO: send data if pending
    }
}

void le_coc_tx_complete_cback(void *context, uint16_t local_cid, uint16_t bufcount)
{
    uint8_t status;

    WICED_BT_TRACE("[%s] CID %d bufcount %d\r\n", __func__, local_cid, bufcount);

    le_coc_send_to_client_control(HCI_CONTROL_LE_COC_EVENT_TX_COMPLETE, &status, 1);
}

void le_coc_disconnect(void)
{
    if (le_coc_cb.local_cid)
        wiced_bt_l2cap_le_disconnect_req(le_coc_cb.local_cid);
}

void le_coc_set_advertisement_data(void)
{
    wiced_bt_ble_advert_elem_t adv_data[3];
    uint8_t adv_flags = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t num_elem = 0;
    wiced_result_t result;

    adv_data[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_data[num_elem].len = 1;
    adv_data[num_elem].p_data = &adv_flags;
    num_elem++;

    adv_data[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_SHORT;
    adv_data[num_elem].len = strlen((const char *)wiced_bt_cfg_settings.device_name);
    adv_data[num_elem].p_data = wiced_bt_cfg_settings.device_name;
    num_elem++;

    if ((result = wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_data)) != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("[%s] Unble to set ADV data... \r\n", __func__);
    }
}

/* Initialize Extended Data Packet Server/Client */
void le_coc_init(void)
{
    /* Clear app control block */
    memset(&le_coc_cb, 0, sizeof(le_coc_cb_t));
    le_coc_cb.local_cid = 0xFFFF;

    /* Register LE l2cap callbacks */
    wiced_bt_l2cap_le_register(psm, &l2c_appl_info, NULL);

    rxBuffPoolPtr = wiced_transport_create_buffer_pool(mtu + 64, 5);
}

uint32_t le_coc_send_data(uint8_t* p_data, uint32_t data_len)
{
    uint8_t ret_val = 0;

    //TODO: Check if the received data_len is larger that the peer MTU
    ret_val = wiced_bt_l2cap_le_data_write(le_coc_cb.local_cid, p_data, data_len, 0);
    WICED_BT_TRACE("[%s] ret_val : %d data_len : %d\r\n", __func__, ret_val, data_len);

    return ret_val;
}

/* Initiate connection */
void le_coc_connect(wiced_bt_device_address_t bd_addr, wiced_bt_ble_address_type_t bd_addr_type)
{
    uint8_t req_security = 0;
    uint8_t req_encr_key_size = 0;
    uint8_t *p_data = le_coc_cb.peer_bda;

    /* Initiate the connection L2CAP connection */
    wiced_bt_l2cap_le_connect_req(psm, (uint8_t*) bd_addr, bd_addr_type, BLE_CONN_MODE_HIGH_DUTY, mtu,
    L2CAP_DEFAULT_BLE_CB_POOL_ID, req_security, req_encr_key_size);

    BDADDR_TO_STREAM(p_data, bd_addr);
}

/*
 *  Pass protocol traces up through the UART
 */
void le_coc_hci_trace_cback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    wiced_transport_send_hci_trace( NULL, type, length, p_data);
}

/*
 * Process advertisement packet received
 */
void le_coc_scan_result_cback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data)
{
    uint32_t i;
    uint8_t tx_buf[128 + 64], *p = tx_buf, len;

    if (p_scan_result)
    {

        *p++ = p_scan_result->ble_evt_type;
        *p++ = p_scan_result->ble_addr_type;

        for (i = 0; i < 6; i++)
            *p++ = p_scan_result->remote_bd_addr[5 - i];

        *p++ = p_scan_result->rssi;

        if (p_adv_data)
        {
            while ((len = *p_adv_data) != 0)
            {
                for (i = 0; i < len + 1; i++)
                    *p++ = *p_adv_data++;
            }
        }

        WICED_BT_TRACE("[%s] found : %B len : %d\r\n", __func__, p_scan_result->remote_bd_addr, (uint32_t) (p - tx_buf));
        le_coc_send_to_client_control( HCI_CONTROL_LE_EVENT_ADVERTISEMENT_REPORT, tx_buf, (uint32_t) (p - tx_buf));
    }
}

void le_coc_handle_le_coc_group(uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len)
{
    uint16_t i = 0;
    uint8_t peer_addr[BD_ADDR_LEN], *peer_addr_ptr;
    wiced_bt_ble_phy_preferences_t phy_preferences;
    wiced_bt_dev_status_t return_val;

    switch (cmd_opcode)
    {
        case HCI_CONTROL_LE_COC_COMMAND_CONNECT:

            for (i = 0; i < BD_ADDR_LEN; i++)
                peer_addr[i] = p_data[BD_ADDR_LEN - 1 - i];

            le_coc_connect(peer_addr, BLE_ADDR_PUBLIC);
            break;

        case HCI_CONTROL_LE_COC_COMMAND_DISCONNECT:
            le_coc_disconnect();
            break;

        case HCI_CONTROL_LE_COC_COMMAND_SEND_DATA:
            le_coc_send_data(p_data, data_len);
            break;

        case HCI_CONTROL_LE_COC_COMMAND_SET_MTU:
            if (2 == data_len)
            {
                mtu = *((uint16_t*) p_data);
            }

            /* now that we have MTU and PSM .. initialize LE COC */
            le_coc_init();
            break;

        case HCI_CONTROL_LE_COC_COMMAND_SET_PSM:
            if (2 == data_len)
            {
                psm = *((uint16_t*) p_data);
            }
            break;

        case HCI_CONTROL_LE_COC_COMMAND_ENABLE_LE2M:

            phy_preferences.rx_phys = phy_preferences.tx_phys = ((1 == *p_data) ? BTM_BLE_PREFER_2M_PHY : BTM_BLE_PREFER_1M_PHY);
#if defined(CYW20721B2) || defined(CYW20719B2)
            phy_preferences.phy_opts = BTM_BLE_PREFER_NO_LELR;
#else
            phy_preferences.phy_opts = BTM_BLE_PREFER_CODED_PHY_NONE;
#endif
            peer_addr_ptr = le_coc_cb.peer_bda;
            STREAM_TO_BDADDR(phy_preferences.remote_bd_addr, peer_addr_ptr);

            return_val = wiced_bt_ble_set_phy(&phy_preferences);

            WICED_BT_TRACE("[%s] phy_preferences.remote_bd_addr %B \r\n", __func__, phy_preferences.remote_bd_addr);
            WICED_BT_TRACE("[%s] return_val %d \r\n", __func__, return_val);
            break;

        default:
            break;
    }
}

void le_coc_handle_le_group(uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len)
{
    wiced_result_t status;
    wiced_bool_t enable = TRUE;
    wiced_bt_ble_scan_type_t scan_type;
    uint8_t hci_status;

    switch (cmd_opcode)
    {
        case HCI_CONTROL_LE_COMMAND_SCAN:
            enable = (wiced_bool_t) p_data[0];

            WICED_BT_TRACE("[%s] enable %d \r\n", __func__, enable);

            scan_type = (1 == enable) ? BTM_BLE_SCAN_TYPE_HIGH_DUTY : BTM_BLE_SCAN_TYPE_NONE;
            status = wiced_bt_ble_scan(scan_type, 0, le_coc_scan_result_cback);

            WICED_BT_TRACE("[%s] status %d \r\n", __func__, status);
            hci_status = ((status == WICED_BT_SUCCESS) || (status == WICED_BT_PENDING)) ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;

            le_coc_send_to_client_control( HCI_CONTROL_LE_EVENT_COMMAND_STATUS, &hci_status, 1);
            break;

        case HCI_CONTROL_LE_COMMAND_ADVERTISE:
            enable = (wiced_bool_t) p_data[0];
            wiced_bt_start_advertisements(((1 == enable) ? BTM_BLE_ADVERT_UNDIRECTED_HIGH : BTM_BLE_ADVERT_OFF), 0, NULL);
            break;

        default:
            break;
    }
}

void le_coc_handle_get_version(void)
{
    uint8_t   tx_buf[20];
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
    tx_buf[cmd++] = HCI_CONTROL_GROUP_LE_COC;

    wiced_transport_send_data(HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd);

}

/*
 * Handle received command over UART.
 *
 */
uint32_t le_coc_proc_rx_cmd(uint8_t *p_data, uint32_t length)
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t *p_data_copy = p_data;

    //Expected minimum 4 byte as the wiced header
    if ((length < 4) || (!p_data))
    {
        wiced_transport_free_buffer(p_data);
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    STREAM_TO_UINT16(opcode, p_data);  // Get OpCode
    STREAM_TO_UINT16(payload_len, p_data);  // Get Payload Length

    WICED_BT_TRACE("[%s] Received %s event \r\n", __func__, getOpcodeStr(opcode));

    if(opcode == HCI_CONTROL_MISC_COMMAND_GET_VERSION)
    {
        WICED_BT_TRACE("HCI_CONTROL_MISC_COMMAND_GET_VERSION\n");
        le_coc_handle_get_version();
    }

    switch ((opcode >> 8) & 0xff)
    {
        case HCI_CONTROL_GROUP_LE_COC:
            le_coc_handle_le_coc_group(opcode, p_data, payload_len);
            break;

        case HCI_CONTROL_GROUP_LE:
            le_coc_handle_le_group(opcode, p_data, payload_len);
            break;

        default:
            WICED_BT_TRACE("[%s] Opcode (0x%x) not processed \r\n", __func__, opcode);
            break;
    }

    wiced_transport_free_buffer(p_data_copy);

    return 0;
}

void le_coc_transport_status(wiced_transport_type_t type)
{
    le_coc_send_to_client_control( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0);
}

void le_coc_send_to_client_control(uint16_t code, uint8_t* p_data, uint16_t length)
{
    uint8_t *dataPtr = NULL;

    WICED_BT_TRACE("[%s] Sending 0x%x length %d \r\n", __func__, code, length);

    if (length < (268 - 16))
    {
        wiced_transport_send_data(code, p_data, length);
    }
    else
    {
        dataPtr = wiced_transport_allocate_buffer(rxBuffPoolPtr);
        if (dataPtr)
        {
            memcpy(dataPtr, p_data, length);
            wiced_transport_send_buffer(code, dataPtr, length);
        }
    }
}

#define STR(x) #x

const char* lecocCmdStr[] =
{
    STR(HCI_CONTROL_LE_COC_COMMAND_CONNECT),
    STR(HCI_CONTROL_LE_COC_COMMAND_DISCONNECT),
    STR(HCI_CONTROL_LE_COC_COMMAND_SEND_DATA),
    STR(HCI_CONTROL_LE_COC_COMMAND_SET_MTU),
    STR(HCI_CONTROL_LE_COC_COMMAND_SET_PSM),
    STR(HCI_CONTROL_LE_COC_COMMAND_ENABLE_LE2M), };

const char* leCmdStr[] =
{
    STR(HCI_CONTROL_LE_COMMAND_SCAN),
    STR(HCI_CONTROL_LE_COMMAND_ADVERTISE),
    STR(HCI_CONTROL_LE_COMMAND_CONNECT),
    STR(HCI_CONTROL_LE_COMMAND_CANCEL_CONNECT),
    STR(HCI_CONTROL_LE_COMMAND_DISCONNECT),
    STR(HCI_CONTROL_LE_RE_PAIR),
    STR(HCI_CONTROL_LE_COMMAND_GET_IDENTITY_ADDRESS),
    STR(HCI_CONTROL_LE_COMMAND_SET_CHANNEL_CLASSIFICATION),
    STR(HCI_CONTROL_LE_COMMAND_SET_CONN_PARAMS),
    STR(HCI_CONTROL_LE_COMMAND_SET_RAW_ADVERTISE_DATA), };

const char* deviceCmdStr[] =
{
    STR(HCI_CONTROL_COMMAND_RESET),
    STR(HCI_CONTROL_COMMAND_TRACE_ENABLE),
    STR(HCI_CONTROL_COMMAND_SET_LOCAL_BDA),
    STR(HCI_CONTROL_COMMAND_SET_BAUD_RATE),
    STR(HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA),
    STR(HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA),
    STR(HCI_CONTROL_COMMAND_INQUIRY),
    STR(HCI_CONTROL_COMMAND_SET_VISIBILITY),
    STR(HCI_CONTROL_COMMAND_SET_PAIRING_MODE),
    STR(HCI_CONTROL_COMMAND_UNBOND),
    STR(HCI_CONTROL_COMMAND_USER_CONFIRMATION),
    STR(HCI_CONTROL_COMMAND_ENABLE_COEX),
    STR(HCI_CONTROL_COMMAND_DISABLE_COEX),
    STR(HCI_CONTROL_COMMAND_SET_BATTERY_LEVEL),
    STR(HCI_CONTROL_COMMAND_READ_LOCAL_BDA),
    STR(HCI_CONTROL_COMMAND_BOND),
    STR(HCI_CONTROL_COMMAND_READ_BUFF_STATS),
    STR(HCI_CONTROL_COMMAND_SET_LOCAL_NAME), };

const char* eventStr[] =
{
    STR(BTM_ENABLED_EVT),
    STR(BTM_DISABLED_EVT),
    STR(BTM_POWER_MANAGEMENT_STATUS_EVT),
    STR(BTM_PIN_REQUEST_EVT),
    STR(BTM_USER_CONFIRMATION_REQUEST_EVT),
    STR(BTM_PASSKEY_NOTIFICATION_EVT),
    STR(BTM_PASSKEY_REQUEST_EVT),
    STR(BTM_KEYPRESS_NOTIFICATION_EVT),
    STR(BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT),
    STR(BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT),
    STR(BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT),
    STR(BTM_PAIRING_COMPLETE_EVT),
    STR(BTM_ENCRYPTION_STATUS_EVT),
    STR(BTM_SECURITY_REQUEST_EVT),
    STR(BTM_SECURITY_FAILED_EVT),
    STR(BTM_SECURITY_ABORTED_EVT),
    STR(BTM_READ_LOCAL_OOB_DATA_COMPLETE_EVT),
    STR(BTM_REMOTE_OOB_DATA_REQUEST_EVT),
    STR(BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT),
    STR(BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT),
    STR(BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT),
    STR(BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT),
    STR(BTM_BLE_SCAN_STATE_CHANGED_EVT),
    STR(BTM_BLE_ADVERT_STATE_CHANGED_EVT),
    STR(BTM_SMP_REMOTE_OOB_DATA_REQUEST_EVT),
    STR(BTM_SMP_SC_REMOTE_OOB_DATA_REQUEST_EVT),
    STR(BTM_SMP_SC_LOCAL_OOB_DATA_NOTIFICATION_EVT),
    STR(BTM_SCO_CONNECTED_EVT),
    STR(BTM_SCO_DISCONNECTED_EVT),
    STR(BTM_SCO_CONNECTION_REQUEST_EVT),
    STR(BTM_SCO_CONNECTION_CHANGE_EVT),
    STR(BTM_BLE_CONNECTION_PARAM_UPDATE), };

const char* getStackEventStr(wiced_bt_management_evt_t event)
{
    if (event >= sizeof(eventStr) / sizeof(uint8_t*))
    {
        return "** UNKNOWN **";
    }

    return eventStr[event];
}

const char* getOpcodeStr(uint16_t opcode)
{
    const char *str = NULL;

    switch ((opcode >> 8) & 0xff)
    {
        case HCI_CONTROL_GROUP_LE_COC:
            str = lecocCmdStr[((opcode) & 0xff) - 1];
            break;

        case HCI_CONTROL_GROUP_LE:
            str = leCmdStr[((opcode) & 0xff) - 1];
            break;

        case HCI_CONTROL_GROUP_DEVICE:
            str = deviceCmdStr[((opcode) & 0xff) - 1];
            break;

        default:
            str = "** UNKNOWN **";
            break;
    }

    return str;
}
