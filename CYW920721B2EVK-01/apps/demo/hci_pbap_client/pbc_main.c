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
 * PBAP Client Device Sample Application for 2070X devices.
 *
 * This file implements 2070x embedded application controlled over UART.
 * Current version of the application exposes PBAP Client
 *
 * MCU connected over UART can send commands to execute certain functionality
 * while configuration is local in the application including SDP
 * databases, as well as configuration of different activities like inquiry
 * advertisements or scanning.
 *
 * To demonstrate the app, work through the following steps.
 * 1. Plug the WICED Bluetooth ( 2070x ) evaluation board into your computer
 * 2. Build and download the application ( to the 2070x board )
 * 3. Use ClientControl application to send various commands
 *
 * The sample app performs as a Bluetooth PBAP Client.
 *
 * The sample Windows ClientControl application is provided to show sample
 * MCU implementation.
 *
 * Features demonstrated
 *  - WICED BT PBAP Client APIs
 *  - Handling of the UART WICED protocol
 *  - SDP configuration
 *  - Setting of the Local Bluetooth Device address from the host MCU
 *
 * On startup this demo:
 *  - Initializes the Bluetooth sub system
 *  - Receive NVRAM information from the host
 *
 * BR/EDR
 * - To find BR/EDR devices: Click on "Start BR/EDR Discovery"
 *
 * PBAP Client Connection
 * - To create pbap client connection to remote pbap server , choose the bluetooth
 *   address of the remote device from the BR/EDR combo box
 * - Click "Connect" button under PBAP Client
 *
 */

#include "sparcommon.h"
#include "data_types.h"
#include "wiced_bt_types.h"
#ifndef CYW20706A2
#include "wiced_bt_dev.h"
#endif
#include "wiced_transport.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_sdp.h"
#include "wiced_memory.h"
#include "wiced_platform.h"
#include "hci_control_api.h"
#include "wiced_bt_pbc_api.h"
#include "xml_vlist_api.h"
#include "pbc.h"
#include "string.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"

/*******************************************************************************
 Definitions
 *******************************************************************************/
#define WICED_BT_PBC_LIST_DIR_LEN_MAX     512
#define WICED_BT_PBC_VCARD_LISTING_SIZE            8192
#define WICED_BT_PBC_MAX_VLIST_ITEMS               (WICED_BT_PBC_VCARD_LISTING_SIZE / sizeof(tXML_VLIST_ENTRY))

#define PBC_CLNT_MAX_GKI_BLOCKS           20

#define WICED_BT_PBC_FREE                 0x00
#define WICED_BT_PBC_OPEN_PENDING         0x01
#define WICED_BT_PBC_OPENED               0x02
#define WICED_BT_PBC_CANCEL               0x04

#define APP_PBC_FEATURES    ( WICED_BT_PBC_SUP_FEA_DOWNLOADING | WICED_BT_PBC_SUP_FEA_BROWSING | \
		WICED_BT_PBC_SUP_FEA_DATABASE_ID | WICED_BT_PBC_SUP_FEA_FOLDER_VER_COUNTER | \
		WICED_BT_PBC_SUP_FEA_VCARD_SELECTING | WICED_BT_PBC_SUP_FEA_ENH_MISSED_CALLS | \
		WICED_BT_PBC_SUP_FEA_UCI_VCARD_FIELD | WICED_BT_PBC_SUP_FEA_UID_VCARD_FIELD | \
		WICED_BT_PBC_SUP_FEA_CONTACT_REF | WICED_BT_PBC_SUP_FEA_DEF_CONTACT_IMAGE_FORMAT )

#define APP_PBC_SETPATH_ROOT              "/"
#define APP_PBC_SETPATH_UP                ".."

#define TRANS_UART_BUFFER_SIZE            1024
#define TRANS_UART_BUFFER_COUNT           2

typedef struct
{
    tXML_VLIST_PARSER p_xml_parser;
    UINT16 num_list_items; /* Current index into list array */
    UINT16 max_peer_items;
    BOOLEAN new_parse;
    tXML_VLIST_ENTRY flist[WICED_BT_PBC_MAX_VLIST_ITEMS];
} wiced_bt_pbc_list_parser_t;

/* PBC control block */
typedef struct
{
    UINT8                       sec_mask;
    BOOLEAN                     is_xml;
    wiced_bt_device_address_t   bd_address;
    wiced_bt_pbc_list_parser_t  xml_parser;
    BOOLEAN                     disable_pending;
    UINT8                       service;
    UINT8                       state_bitfield; /* service state */
    BOOLEAN                     auth_already_failed_once;
    uint16_t                    opcode;
    uint8_t req_pending;
} wiced_bt_pbc_sv_cb_t;

/*******************************************************************************
 Externs
 *******************************************************************************/
extern void hci_control_hci_trace_cback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);

/*******************************************************************************
 Local functions
 *******************************************************************************/
void wiced_bt_pbc_enable_hdlr();
void wiced_bt_pbc_disable_hdlr();
void wiced_bt_pbc_auth_rsp_hdlr();
void wiced_bt_pbc_get_hdlr();
void wiced_bt_pbc_set_hdlr();
void wiced_bt_pbc_open_hdlr(wiced_bt_device_address_t bd_address);
void wiced_bt_pbc_close_hdlr();
void wiced_bt_pbc_abort_hdlr();
void wiced_bt_pbc_cancel_hdlr();

void wiced_bt_pbc_event_callback(wiced_bt_pbc_evt_t event, wiced_bt_pbc_t* p_buffer);
void wiced_bt_pbc_data_callback(const UINT8 *p_buf, UINT16 nbytes);

/*******************************************************************************
 Globals
 *******************************************************************************/
wiced_bt_pbc_sv_cb_t wiced_bt_pbc_sv_cb;
wiced_bt_pbc_app_cfg_t wiced_bt_pbc_app_cfg;

wiced_transport_buffer_pool_t* transport_pool; // Trans pool for sending the RFCOMM data to host
wiced_bt_buffer_pool_t* p_key_info_pool;  //Pool for storing the  key info

const wiced_transport_cfg_t transport_cfg =
    {   WICED_TRANSPORT_UART,
        {{ WICED_TRANSPORT_UART_HCI_MODE, HCI_UART_DEFAULT_BAUD }},
        { TRANS_UART_BUFFER_SIZE, 2 },
        NULL, hci_control_proc_rx_cmd,
        NULL
    };


uint8_t pairing_allowed = 0;

extern int utl_strncmp(const char *s1, const char *s2, int n);
/*******************************************************************************
 Functions
 *******************************************************************************/
/*******************************************************************************
**
** Function         wiced_bt_pbc_check_set_pending_req
**
** Description      check pending req flag.
**
** Returns          void
**
*******************************************************************************/
static uint8_t wiced_bt_pbc_check_set_pending_req(void)
{
    if (wiced_bt_pbc_sv_cb.req_pending)
    {
        WICED_BT_TRACE("WICED PBC ***Previous Operation Outstanding. Please Abort previous action and try again.***");
        return TRUE;
    }

    wiced_bt_pbc_sv_cb.req_pending = TRUE;

    return FALSE;
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_clear_pending_req
**
** Description     clear pending req flag
**
** Returns          void
**
*******************************************************************************/
static void wiced_bt_pbc_clear_pending_req(void)
{
    wiced_bt_pbc_sv_cb.req_pending = FALSE;
}

uint8_t wiced_bt_pbc_is_connected()
{
    return (wiced_bt_pbc_sv_cb.state_bitfield == WICED_BT_PBC_OPENED);
}

uint16_t wiced_bt_pbc_return_evt_code()
{
    uint16_t opcode = wiced_bt_pbc_sv_cb.opcode;

    if(opcode == HCI_CONTROL_PBC_COMMAND_GET_PHONEBOOK)
        return HCI_CONTROL_PBC_EVENT_PHONEBOOK;
    else if(opcode == HCI_CONTROL_PBC_COMMAND_GET_CALL_HISTORY)
        return HCI_CONTROL_PBC_EVENT_CALL_HISTORY;
    else if(opcode == HCI_CONTROL_PBC_COMMAND_GET_INCOMMING_CALLS)
        return HCI_CONTROL_PBC_EVENT_INCOMMING_CALLS;
    else if(opcode == HCI_CONTROL_PBC_COMMAND_GET_OUTGOING_CALLS)
        return HCI_CONTROL_PBC_EVENT_OUTGOING_CALLS;
    else if(opcode == HCI_CONTROL_PBC_COMMAND_GET_MISSED_CALLS)
        return HCI_CONTROL_PBC_EVENT_MISSED_CALLS;

    return HCI_CONTROL_PBC_EVENT_PHONEBOOK;
}

void hci_control_send_pbc_event(uint16_t evt, hci_control_pbc_event_t *p_data, uint16_t size)
{
    WICED_BT_TRACE("hci_control_send_pbc_event: Sending Event: %u  to UART\n",  evt);
    uint8_t  *p_trans_buffer;

    if ((p_trans_buffer = (uint8_t *)wiced_transport_allocate_buffer(transport_pool)) == NULL)
    {
        WICED_BT_TRACE("Error no transport buffer\n");
        return;
    }

    // copy received data to transport buffer
    memcpy(p_trans_buffer, p_data, size);

    WICED_BT_TRACE("hci_control_send_pbc_event: Sending Event: %u  Size: %u to UART\n", evt, size);

    if (wiced_transport_send_buffer(evt, p_trans_buffer, size) != WICED_SUCCESS)
    {
        WICED_BT_TRACE("Err: failed to transport buffer\n");
    }
}

void pbap_client_write_eir()
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t length;

    pBuf = (uint8_t*) wiced_bt_get_buffer( WICED_HS_EIR_BUF_MAX_SIZE);
    WICED_BT_TRACE( "hci_control_write_eir %x\n", pBuf );

    if (!pBuf)
    {
        return;
    }
    p = pBuf;

    length = strlen((char *) pbap_client_cfg_settings.device_name);

    *p++ = length + 1;
    *p++ = 0x09;            // EIR type full name
    memcpy(p, pbap_client_cfg_settings.device_name, length);
    p += length;
    *p++ = ( 3 * 2 ) + 1;     // length of services + 1
    *p++ = 0x02;            // EIR type full list of 16 bit service UUIDs
    *p++ = UUID_SERVCLASS_PBAP_PCE & 0xff;
    *p++ = ( UUID_SERVCLASS_PBAP_PCE >> 8) & 0xff;
    *p++ =   UUID_SERVCLASS_HF_HANDSFREE        & 0xff;
    *p++ = ( UUID_SERVCLASS_HF_HANDSFREE >> 8 ) & 0xff;
    *p++ =   UUID_SERVCLASS_GENERIC_AUDIO        & 0xff;
    *p++ = ( UUID_SERVCLASS_GENERIC_AUDIO >> 8 ) & 0xff;
    *p++ = 0;

    // print EIR data
    wiced_bt_trace_array("EIR :", (uint8_t*) (pBuf + 1), MIN(p - (uint8_t* )pBuf, 100));
    wiced_bt_dev_write_eir(pBuf, (uint16_t) (p - pBuf));

    return;
}

void pbap_client_post_bt_init(wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_ERROR;
    if (p_event_data->enabled.status == WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("Bluetooth stack initialized\n");

        /* Set-up EIR data */
        pbap_client_write_eir();
        /* Set-up SDP database */
        wiced_bt_sdp_db_init((uint8_t *) pbap_client_sdp_db, wiced_app_cfg_sdp_record_get_size());

#ifdef CYW20706A2
        // Initialize RFCOMM.  We will not be using application buffer pool and will rely on the
        // stack pools configured in the pbc_bt_cfg.c
        wiced_bt_rfcomm_init(1024, 1);
#endif
        wiced_bt_set_pairable_mode(WICED_TRUE, 0);

        wiced_bt_pbc_enable_hdlr();

    } else
    {
        WICED_BT_TRACE("Bluetooth stack initialization failure!!\n");
        return;
    }
}

/*
 * Write NVRAM function is called to store information in the NVRAM.
 */
int pbap_client_write_nvram(int nvram_id, int data_len, void *p_data)
{
    wiced_result_t result;
    int bytes_written = wiced_hal_write_nvram(nvram_id, data_len, (uint8_t*) p_data, &result);

    WICED_BT_TRACE("NVRAM ID:%d written :%d bytes result:%d\n", nvram_id, bytes_written, result);
    return (bytes_written);
}

/*
 * Read data from the NVRAM and return in the passed buffer
 */
int pbap_client_read_nvram(int nvram_id, void *p_data, int data_len)
{
    uint16_t read_bytes = 0;
    wiced_result_t result;

    if (data_len >= sizeof(wiced_bt_device_link_keys_t))
    {
        read_bytes = wiced_hal_read_nvram(nvram_id, sizeof(wiced_bt_device_link_keys_t), p_data, &result);
        WICED_BT_TRACE("NVRAM ID:%d read out of %d bytes:%d result:%d\n", nvram_id, sizeof(wiced_bt_device_link_keys_t), read_bytes, result );
    }
    return (read_bytes);
}

/*******************************************************************************
 **
 ** Function         wiced_bt_pbc_xml_folder_parse
 **
 ** Description      Handler function called when a msg is received from a BTA Callback
 **
 ** Returns          void
 **
 *******************************************************************************/
static UINT16 wiced_bt_pbc_xml_folder_parse(wiced_bt_pbc_list_t *p_list, hci_control_pbc_event_t * p_dst_data)
{
    BOOLEAN x_error = FALSE;
    wiced_bt_pbc_list_parser_t *parser_cb = &wiced_bt_pbc_sv_cb.xml_parser;
    tXML_VLIST_RES x_result = XML_VLIST_NO_RES;
    UINT8 *p_buf;
    UINT16 num_list_items;

    uint16_t handle = 0; // TODO for parsing list not implemented yet

    UINT16 size = sizeof(tXML_VLIST_ENTRY) * WICED_BT_PBC_MAX_VLIST_ITEMS;

    if (parser_cb->new_parse)
    {
        parser_cb->num_list_items = 0; /* reset */
        parser_cb->new_parse = FALSE;
        memset((void *) parser_cb->flist, 0, sizeof(parser_cb->flist)); /* this is not needed by parser */

        XML_VlistInit(&parser_cb->p_xml_parser, parser_cb->flist,
        WICED_BT_PBC_MAX_VLIST_ITEMS);

        WICED_BT_TRACE("wiced_bt_pbc_xml_folder_parse: XML_FolderInit");
    }

    do
    {
        size = WICED_BT_PBC_LIST_DIR_LEN_MAX;

        if (p_dst_data != NULL)
        {
            memset((void*) p_dst_data, 0, sizeof(hci_control_pbc_event_t));
            p_dst_data->get.type = WICED_BT_PBC_GET_PARAM_LIST;
            p_dst_data->get.status = p_list->status;
            p_dst_data->get.param.list.is_xml = wiced_bt_pbc_sv_cb.is_xml;

            /* enqueue memory block heads */
            num_list_items = parser_cb->num_list_items; /* Current index into list array */

            WICED_BT_TRACE("wiced_bt_pbc_xml_folder_parse: num_list_items %d size:%d"
                    , parser_cb->num_list_items, size );

            x_result = XML_VlistParse(&parser_cb->p_xml_parser, p_list->data, p_list->len, &p_dst_data->get.param.list.data[0], &size,
                    &parser_cb->num_list_items);

            WICED_BT_TRACE("wiced_bt_pbc_xml_folder_parse: x_result %d", x_result );
            p_dst_data->get.param.list.num_entry = parser_cb->num_list_items - num_list_items;
            p_dst_data->get.param.list.len = size;
            p_buf = p_dst_data->get.param.list.data;

            WICED_BT_TRACE("wiced_bt_pbc_xml_folder_parse AFTER: num_list_items %d size:%d", parser_cb->num_list_items, size );

            for (num_list_items = 0; num_list_items < p_dst_data->get.param.list.num_entry * 2; num_list_items++)
            {
                WICED_BT_TRACE("%s", p_buf);
                p_buf += 1 + strlen((char *) p_buf);
            }

            p_dst_data->get.param.list.final = p_list->final;
            if (x_result == XML_VLIST_DST_NO_RES)
                p_dst_data->get.param.list.final = FALSE;

            hci_control_send_pbc_event(HCI_CONTROL_PBC_GET_EVT, p_dst_data, sizeof(hci_control_pbc_event_t));

        } else
        {
            x_result = XML_VlistParse(&parser_cb->p_xml_parser, NULL, 0, NULL, NULL, &parser_cb->num_list_items);

            WICED_BT_TRACE("wiced_bt_pbc_xml_folder_parse: Out of Resources,discard partial data. ");
            break;
        }
    } while (x_result == XML_VLIST_DST_NO_RES);

    if (x_result == XML_VLIST_OK || x_result == XML_VLIST_NO_RES)
        x_error = TRUE;

    WICED_BT_TRACE("wiced_bt_pbc_xml_folder_parse: XML_FolderParse ( final:%d, status:%d, len:%d entries %d): %d",
            p_list->final, p_list->status, p_list->len, parser_cb->num_list_items, x_result );

    /* if last package of list data and no error occurs   */
    /* display parsed folder entry                        */
    if (p_list->final && !x_error)
    {
        return WICED_BT_SUCCESS;
    }

    return WICED_BT_PBC_ERROR_XML_PARS;
}

/*******************************************************************************
 **
 ** Function         wiced_bt_pbc_list_evt_handler
 **
 ** Description      Handler function called when a msg is received from a BTA Callback
 **
 ** Returns          void
 **
 *******************************************************************************/
static void wiced_bt_pbc_list_evt_handler(wiced_bt_pbc_list_t *p_list)
{
    hci_control_pbc_event_t* p_dst_data = NULL;
    wiced_bt_pbc_list_parser_t *parser_cb = &wiced_bt_pbc_sv_cb.xml_parser;
    UINT16 offset = 0;
    parser_cb->new_parse = TRUE;

    WICED_BT_TRACE("wiced_bt_pbc_list_evt_handler is_xml %d, final %d", wiced_bt_pbc_sv_cb.is_xml, p_list->final);

    if ((p_dst_data = (hci_control_pbc_event_t *) GKI_getbuf(sizeof(hci_control_pbc_event_t))) == NULL)
    {
        WICED_BT_TRACE("wiced_bt_pbc_list_evt_handler FAILED to allocate a buffer");
        return;
    }

    memset((void*) p_dst_data, 0, sizeof(hci_control_pbc_event_t));
    p_dst_data->get.type = WICED_BT_PBC_GET_PARAM_LIST;
    p_dst_data->get.status = p_list->status;

    if (p_dst_data->get.status != WICED_BT_SUCCESS)
    {
        hci_control_send_pbc_event(HCI_CONTROL_PBC_GET_EVT, p_dst_data, sizeof(hci_control_pbc_event_t));
        GKI_freebuf(p_dst_data);
        return;
    }

    p_dst_data->get.param.list.is_xml = wiced_bt_pbc_sv_cb.is_xml;
    if (p_dst_data->get.param.list.is_xml == FALSE)
    {
        /* for now we just pars folder */
        wiced_bt_pbc_xml_folder_parse(p_list, p_dst_data);
    } else
    {
        p_dst_data->get.param.list.num_entry = 1;
        while (p_list->len)
        {
            p_dst_data->get.param.list.final = p_list->final;

            p_dst_data->get.param.list.len = p_list->len;
            if (p_dst_data->get.param.list.len > (WICED_BT_PBC_LIST_DIR_LEN_MAX - 1))
                p_dst_data->get.param.list.len = (WICED_BT_PBC_LIST_DIR_LEN_MAX - 1);

            memcpy(p_dst_data->get.param.list.data, p_list->data + offset, p_dst_data->get.param.list.len);
            p_dst_data->get.param.list.data[p_dst_data->get.param.list.len] = 0;
            p_list->len -= p_dst_data->get.param.list.len;
            offset += p_dst_data->get.param.list.len;

            p_dst_data->get.param.list.final = p_list->final;
            if (p_list->len > 0)
            {
                p_dst_data->get.param.list.final = FALSE;
            }

            WICED_BT_TRACE("wiced_bt_pbc_list_evt_handler p_list->len %d, p_dst_data->param.len %d",
                    p_list->len, p_dst_data->get.param.list.len);
            hci_control_send_pbc_event(HCI_CONTROL_PBC_GET_EVT, p_dst_data, sizeof(hci_control_pbc_event_t));
        }
    }

    GKI_freebuf(p_dst_data);
}

/*******************************************************************************

 Function:       wiced_bt_pbc_init

 Description:    Initializes the PBC control block

 Arguments:      None

 *******************************************************************************/
void wiced_bt_pbc_init(void)
{
    WICED_BT_TRACE("wiced_bt_pbc_init");
    wiced_bt_pbc_sv_cb.disable_pending = FALSE;
    wiced_bt_pbc_sv_cb.service = 0;
    wiced_bt_pbc_sv_cb.state_bitfield = WICED_BT_PBC_FREE;
    wiced_bt_pbc_sv_cb.is_xml = TRUE;
}

/*******************************************************************************
 **
 ** Function         wiced_bt_pbc_client_disconnected
 **
 ** Description      Initializes the control block and disables PBC.
 **
 ** Returns          void
 **
 *******************************************************************************/
void wiced_bt_pbc_client_disconnected()
{
    WICED_BT_TRACE("wiced_bt_pbc_client_disconnected ");

    wiced_bt_pbc_op_disable();
}

/*******************************************************************************
 **
 ** Function         wiced_bt_pbc_evt_code
 **
 ** Description      Returns event as string representation of the event code
 **
 ** Returns          char* pointing to the string
 **
 *******************************************************************************/
char *wiced_bt_pbc_evt_code(wiced_bt_pbc_evt_t evt_code)
{
    switch (evt_code)
    {
    case WICED_BT_PBC_ENABLE_EVT:
        return "WICED_BT_PBC_ENABLE_EVT";
    case WICED_BT_PBC_DISABLE_EVT:
        return "WICED_BT_PBC_DISABLE_EVT";
    case WICED_BT_PBC_OPEN_EVT:
        return "WICED_BT_PBC_OPEN_EVT";
    case WICED_BT_PBC_CLOSE_EVT:
        return "WICED_BT_PBC_CLOSE_EVT";
    case WICED_BT_PBC_AUTH_EVT:
        return "WICED_BT_PBC_AUTH_EVT";
    case WICED_BT_PBC_LIST_EVT:
        return "WICED_BT_PBC_LIST_EVT";
    case WICED_BT_PBC_PROGRESS_EVT:
        return "WICED_BT_PBC_PROGRESS_EVT";
    case WICED_BT_PBC_GETFILE_EVT:
        return "WICED_BT_PBC_GETFILE_EVT";
    case WICED_BT_PBC_CHDIR_EVT:
        return "WICED_BT_PBC_CHDIR_EVT";
    case WICED_BT_PBC_PHONEBOOK_EVT:
        return "WICED_BT_PBC_PHONEBOOK_EVT";

    default:
        return "unknown PBC event code";
    }
    return NULL;
}

/*******************************************************************************
 **
 ** Function         wiced_bt_pbc_enable_hdlr
 **
 ** Description      Handler function called when a msg is received from a client
 **
 ** Returns          void
 **
 *******************************************************************************/
void wiced_bt_pbc_enable_hdlr()
{
    int app_id = 1;
    WICED_BT_TRACE("wiced_bt_pbc_enable_hdlr");

    memset(&wiced_bt_pbc_sv_cb, 0, sizeof(wiced_bt_pbc_sv_cb_t));
    memset(&wiced_bt_pbc_app_cfg, 0, sizeof(wiced_bt_pbc_app_cfg_t));

    // Basic configuration PBAP Client will use
    wiced_bt_pbc_app_cfg.pbc_security = WICED_BT_PBC_SECURITY;
    utl_strcpy(wiced_bt_pbc_app_cfg.pbc_password, WICED_BT_PBC_PASSWORD);
    utl_strcpy(wiced_bt_pbc_app_cfg.pbc_userid, WICED_BT_PBC_USERID);
    wiced_bt_pbc_app_cfg.pbc_filter_mask = WICED_BT_PBC_FILTER;
    wiced_bt_pbc_app_cfg.pbc_format = WICED_BT_PBC_FORMAT;
    wiced_bt_pbc_app_cfg.pbc_order = WICED_BT_PBC_ORDER;
    wiced_bt_pbc_app_cfg.pbc_attr = WICED_BT_PBC_ATTR;
    wiced_bt_pbc_app_cfg.pbc_selector = WICED_BT_PBC_SELECTOR;
    wiced_bt_pbc_app_cfg.pbc_selector_op = WICED_BT_PBC_SELECTOR_OP;
    wiced_bt_pbc_app_cfg.pbc_max_list_count = WICED_BT_PBC_MAX_LCOUNT;
    wiced_bt_pbc_app_cfg.pbc_offset = 0;
    wiced_bt_pbc_app_cfg.pbc_filter = 0; // No Filter

    wiced_bt_pbc_op_enable(wiced_bt_pbc_event_callback, wiced_bt_pbc_data_callback, app_id, APP_PBC_FEATURES);
}

/*******************************************************************************
 **
 ** Function         wiced_bt_pbc_disable_hdlr
 **
 ** Description      Handler function called when a msg is received from a client
 **
 ** Returns          void
 **
 *******************************************************************************/
void wiced_bt_pbc_disable_hdlr()
{
    WICED_BT_TRACE("wiced_bt_pbc_disable_hdlr ");

    wiced_bt_pbc_clear_pending_req();

    if (wiced_bt_pbc_is_connected())
    {
        WICED_BT_TRACE("wiced_bt_pbc_disable_hdlr BTA_PbcClose service = %d", wiced_bt_pbc_sv_cb.service);
        wiced_bt_pbc_sv_cb.disable_pending = TRUE;
        wiced_bt_pbc_op_close();
    }
    else
    {
        wiced_bt_pbc_op_disable();
    }
}

/*******************************************************************************
 **
 ** Function         wiced_bt_pbc_open_hdlr
 **
 ** Description      Handler function called when a msg is received from a client
 **
 ** Returns          void
 **
 *******************************************************************************/
void wiced_bt_pbc_open_hdlr(wiced_bt_device_address_t bd_address)
{
    WICED_BT_TRACE("wiced_bt_pbc_open_hdlr");
    if (wiced_bt_pbc_check_set_pending_req())
      return;

    wiced_bt_pbc_sv_cb.state_bitfield = WICED_BT_PBC_OPEN_PENDING; // set state to be open pending
    memcpy(wiced_bt_pbc_sv_cb.bd_address, bd_address, BD_ADDR_LEN);
    wiced_bt_pbc_op_open(bd_address, WICED_BT_PBC_SECURITY);
}

/*******************************************************************************
 **
 ** Function         wiced_bt_pbc_close_hdlr
 **
 ** Description      Handler function called when a msg is received from a client
 **
 ** Returns          void
 **
 *******************************************************************************/
void wiced_bt_pbc_close_hdlr()
{
    WICED_BT_TRACE("wiced_bt_pbc_close_hdlr");

    if (wiced_bt_pbc_sv_cb.state_bitfield == WICED_BT_PBC_FREE)
    {
        WICED_BT_TRACE("wiced_bt_pbc_close_hdlr No PBC connection");
        return;
    }

    if (wiced_bt_pbc_is_connected())
    {
        wiced_bt_pbc_op_close();
    }
}

/*******************************************************************************
 **
 ** Function         wiced_bt_pbc_cancel_hdlr
 **
 ** Description      Handler function called when a cancel msg is received from a client
 **
 ** Returns          void
 **
 *******************************************************************************/
void wiced_bt_pbc_cancel_hdlr()
{
    WICED_BT_TRACE("wiced_bt_pbc_cancel_hdlr");

    if (wiced_bt_pbc_is_connected())
    {
        // client already asked to cancel, do not do anything
        if (wiced_bt_pbc_sv_cb.state_bitfield == WICED_BT_PBC_CANCEL)
        {
            WICED_BT_TRACE("wiced_bt_pbc_cancel_hdlr, already trying to cancel");
        }

        // If connection is already open, close it
        else if (wiced_bt_pbc_sv_cb.state_bitfield == WICED_BT_PBC_OPENED)
        {
            WICED_BT_TRACE("BTA_PbcClose");
            wiced_bt_pbc_op_close();
        }
        // if connection is not yet open, wait for open callback
        else if (wiced_bt_pbc_sv_cb.state_bitfield == WICED_BT_PBC_OPEN_PENDING)
        {
            WICED_BT_TRACE("BTA_PbcClose, WICED_BT_PBC_OPEN_PENDING");
            wiced_bt_pbc_sv_cb.state_bitfield = WICED_BT_PBC_CANCEL;
        }
    }
}

/*******************************************************************************
 **
 ** Function         wiced_bt_pbc_auth_rsp_hdlr
 **
 ** Description      Handler function called when a msg is received from a client
 **
 ** Returns          void
 **
 *******************************************************************************/
void wiced_bt_pbc_auth_rsp_hdlr()
{
    WICED_BT_TRACE("wiced_bt_pbc_auth_rsp_hdlr");

    /* If the client is the one which registered/enabled PBC*/
    if (wiced_bt_pbc_is_connected())
    {
        wiced_bt_pbc_op_authrsp("0000", "guest"); //p_buffer->password, p_buffer->userid);
    }
}


/*******************************************************************************
 **
 ** Function         wiced_bt_pbc_get_phonebook
 **
 ** Description      gets phonebook from peer device
 **
 **
 ** Returns          void
 *******************************************************************************/
BOOLEAN wiced_bt_pbc_get_phonebook(uint16_t opcode, UINT16 max_list_count, UINT16 list_start_offset, BOOLEAN is_reset_miss_calls)
{
    UINT16 len = 0;
    BOOLEAN ret = FALSE;
    char *p = NULL;

    char *file_name  = NULL;

    if(opcode == HCI_CONTROL_PBC_COMMAND_GET_PHONEBOOK)
        file_name = "telecom/pb.vcf";
    else if(opcode == HCI_CONTROL_PBC_COMMAND_GET_CALL_HISTORY)
        file_name = "telecom/cch.vcf";
    else if(opcode == HCI_CONTROL_PBC_COMMAND_GET_INCOMMING_CALLS)
        file_name = "telecom/ich.vcf";
    else if(opcode == HCI_CONTROL_PBC_COMMAND_GET_OUTGOING_CALLS)
        file_name = "telecom/och.vcf";
    else if(opcode == HCI_CONTROL_PBC_COMMAND_GET_MISSED_CALLS)
        file_name = "telecom/mch.vcf";
    else
        return ret;

    if (wiced_bt_pbc_check_set_pending_req())
      return ret;

    wiced_bt_pbc_sv_cb.opcode  = opcode;
    len = strlen(file_name) + 1;

    p = (char*)file_name;

    do
    {
        /* double check the specified phonebook is correct */
        if (utl_strncmp(p, "SIM1/", 5) == 0)
            p += 5;
        if (utl_strncmp(p, "telecom/", 8) != 0)
            break;
        p += 8;

        if (strcmp(p, "pb.vcf"))
        {
#if (defined(WICED_PBAP_1_2_SUPPORTED) && WICED_PBAP_1_2_SUPPORTED == TRUE)
            if (strcmp(p, "fav.vcf") && strcmp(p, "spd.vcf"))
#endif
            {
                p++;
                if (strcmp(p, "ch.vcf") != 0)
                    break;
            }
        }

        ret = TRUE;

        WICED_BT_TRACE("file name = %s, len = %d", file_name, len);

        //        wiced_bt_pbc_cb.bytes_transferred = 0;
        wiced_bt_pbc_op_getphonebook(NULL, file_name, wiced_bt_pbc_app_cfg.pbc_filter_mask, wiced_bt_pbc_app_cfg.pbc_format, max_list_count,
                list_start_offset, is_reset_miss_calls, wiced_bt_pbc_app_cfg.pbc_selector, wiced_bt_pbc_app_cfg.pbc_selector_op);

    } while (0);

    return ret;
}

/*******************************************************************************
 **
 ** Function         wiced_bt_pbc_get_vcard
 **
 ** Description      gets vcard from peer device
 **
 **
 ** Returns          void
 *******************************************************************************/
BOOLEAN wiced_bt_pbc_get_vcard(char *file_name)
{

    wiced_bt_pbc_op_getcard(NULL, file_name, wiced_bt_pbc_app_cfg.pbc_filter_mask, wiced_bt_pbc_app_cfg.pbc_format);

    return TRUE;
}

/*******************************************************************************
 **
 ** Function         wiced_bt_pbc_list_vcards
 **
 ** Description      refreshes peer vcard listing
 **
 **
 ** Returns          void
 *******************************************************************************/
BOOLEAN wiced_bt_pbc_list_vcards(char *p_dir, char *p_value, UINT16 max_list_count, UINT16 list_start_offset, BOOLEAN is_reset_miss_calls)
{
    WICED_BT_TRACE("wiced_bt_pbc_list_vcards");

    wiced_bt_pbc_op_listcards(p_dir, wiced_bt_pbc_app_cfg.pbc_order, p_value, wiced_bt_pbc_app_cfg.pbc_attr, max_list_count, list_start_offset,
            is_reset_miss_calls, wiced_bt_pbc_app_cfg.pbc_selector, wiced_bt_pbc_app_cfg.pbc_selector_op);
    return TRUE;
}

/*******************************************************************************
 **
 ** Function         wiced_bt_pbc_abort_hdlr
 **
 ** Description      Handler function called when a msg is received from a client
 **
 ** Returns          void
 **
 *******************************************************************************/
void wiced_bt_pbc_abort_hdlr()
{
    WICED_BT_TRACE("wiced_bt_pbc_abort_hdlr");
    wiced_bt_pbc_clear_pending_req();
    /* If the client is the one which registered/enabled PBC*/
    wiced_bt_pbc_op_abort();
}

/*
 * BTA Callback message Handler functions
 */

void wiced_bt_pbc_data_callback(const UINT8 *p_buf, UINT16 nbytes)
{
    WICED_BT_TRACE("wiced_bt_pbc_data_callback: nbytes=%d ", nbytes);

    uint8_t* p_trans_buffer;
    hci_control_pbc_event_t* p_pbc_event;

    if ((p_trans_buffer = (uint8_t *) wiced_transport_allocate_buffer(transport_pool)) == NULL)
    {
        WICED_BT_TRACE("Error no transport buffer\n");
        return;
    }

    p_pbc_event = (hci_control_pbc_event_t*) p_trans_buffer;

    p_pbc_event->get.status = WICED_BT_SUCCESS;
    p_pbc_event->get.type = WICED_BT_PBC_GET_PARAM_PHONEBOOK_DATA;
    p_pbc_event->get.param.pb_data.len = nbytes;

    // copy received data to transport buffer
    memcpy(p_trans_buffer + 4, p_buf, nbytes);

    if (wiced_transport_send_buffer(wiced_bt_pbc_return_evt_code(), p_trans_buffer, nbytes + 4) != WICED_SUCCESS)
    {
        WICED_BT_TRACE("Err: failed to transport buffer\n");
    }
}

/*******************************************************************************
 **
 ** Function         wiced_bt_pbc_event_callback
 **
 ** Description      Handler function called to process events received from a BTA Callback
 **
 ** Returns          void
 **
 *******************************************************************************/
void wiced_bt_pbc_event_callback(wiced_bt_pbc_evt_t event, wiced_bt_pbc_t* p_buffer)
{
    hci_control_pbc_event_t p_val;
    int res = 0;
    memset(&p_val, 0, sizeof(hci_control_pbc_event_t));

    hci_control_pbc_event_t pbc_event;
    int realm_len;

    uint8_t byteval = 0;

    WICED_BT_TRACE("wiced_bt_pbc_event_callback: e=%d(%s) , Size of Struct = %d", event, wiced_bt_pbc_evt_code(event), sizeof(pbc_event));

    if (!p_buffer && (event != WICED_BT_PBC_CLOSE_EVT))
    {
        WICED_BT_TRACE("wiced_bt_pbc_event_callback: e=%d Buffer is NULL", event);
        return;
    }

    memset(&pbc_event, 0, sizeof(pbc_event));

    switch (event)
    {
    case WICED_BT_PBC_ENABLE_EVT:
        WICED_BT_TRACE("WICED_BT_PBC_ENABLE_EVT event");
        break;

    case WICED_BT_PBC_DISABLE_EVT: /* Phone Book Client Access Disabled*/
        WICED_BT_TRACE("WICED_BT_PBC_DISABLE_EVT event");
        wiced_bt_pbc_clear_pending_req();
        wiced_bt_pbc_init();
        break;

    case WICED_BT_PBC_OPEN_EVT: /* Connection Open*/
        WICED_BT_TRACE("WICED_BT_PBC_OPEN_EVT event");
        wiced_bt_pbc_clear_pending_req();
        pbc_event.open.service = p_buffer->open.service;
        pbc_event.open.status = WICED_BT_SUCCESS;
        wiced_bt_pbc_sv_cb.service = p_buffer->open.service;
        pbc_event.open.peer_features = p_buffer->open.peer_features;

        if (wiced_bt_pbc_sv_cb.state_bitfield == WICED_BT_PBC_OPEN_PENDING)
            pbc_event.open.initiator = TRUE;
        else
            pbc_event.open.initiator = FALSE;

        wiced_bt_pbc_sv_cb.state_bitfield = p_buffer->open.service ? WICED_BT_PBC_OPENED : WICED_BT_PBC_FREE;

        WICED_BT_TRACE("WICED_BT_PBC_OPEN_EVT event service = %d", wiced_bt_pbc_sv_cb.service);

        memcpy(pbc_event.open.bd_addr , wiced_bt_pbc_sv_cb.bd_address, BD_ADDR_LEN);

        hci_control_send_pbc_event(HCI_CONTROL_PBC_EVENT_CONNECTED, &pbc_event, sizeof(pbc_event.open));

        break;

    case WICED_BT_PBC_CLOSE_EVT:
        WICED_BT_TRACE("WICED_BT_PBC_CLOSE_EVT event");
        wiced_bt_pbc_clear_pending_req();
        if (wiced_bt_pbc_is_connected())
        {
            memset(&pbc_event, 0, sizeof(pbc_event));

            wiced_bt_pbc_sv_cb.service = 0;
            wiced_bt_pbc_sv_cb.state_bitfield = WICED_BT_PBC_FREE;

            if (wiced_bt_pbc_sv_cb.disable_pending)
            {
                wiced_bt_pbc_op_disable();
                wiced_bt_pbc_sv_cb.disable_pending = FALSE;
            }

            pbc_event.close.status = WICED_BT_PBC_CLOSE_CLOSED;

            if (p_buffer && p_buffer->status == WICED_BT_PBC_ABORTED)
            {
                WICED_BT_TRACE("WICED_BT_PBC_CLOSE_EVT p_buffer =%x, p_buffer->status = %d", p_buffer, p_buffer->status);
                pbc_event.close.status = WICED_BT_PBC_CLOSE_CONN_LOSS;
            }

            hci_control_send_pbc_event(HCI_CONTROL_PBC_EVENT_DISCONNECTED, &pbc_event, sizeof(pbc_event.close));
        }
        break;

    case WICED_BT_PBC_AUTH_EVT:
        WICED_BT_TRACE("WICED_BT_PBC_AUTH_EVT event");

        if (!wiced_bt_pbc_sv_cb.auth_already_failed_once)
        {
            wiced_bt_pbc_sv_cb.auth_already_failed_once = TRUE;
            wiced_bt_pbc_op_authrsp(wiced_bt_pbc_app_cfg.pbc_password, wiced_bt_pbc_app_cfg.pbc_userid);
        } else
        {
            WICED_BT_TRACE("Phone Book Access Client AUTH Failure (pin %s)...", wiced_bt_pbc_app_cfg.pbc_password);
            wiced_bt_pbc_sv_cb.auth_already_failed_once = FALSE;
            wiced_bt_pbc_op_close(); /* Close the connection because pin code didn't match */
        }
        break;

    case WICED_BT_PBC_LIST_EVT:
        WICED_BT_TRACE("WICED_BT_PBC_LIST_EVT event status = %d",p_buffer->status);
        if (wiced_bt_pbc_is_connected())
        {
            wiced_bt_pbc_list_evt_handler(&(p_buffer->list));
        }
        break;

    case WICED_BT_PBC_PROGRESS_EVT:
        WICED_BT_TRACE("WICED_BT_PBC_PROGRESS_EVT event status = %d",p_buffer->status);

        WICED_BT_TRACE("WICED_BT_PBC_PROGRESS_EVT  event: %d/%d", p_buffer->prog.file_size, p_buffer->prog.bytes);
        if (wiced_bt_pbc_is_connected())
        {
            pbc_event.get.status = WICED_BT_SUCCESS;
            pbc_event.get.type = WICED_BT_PBC_GET_PARAM_PROGRESS;
            pbc_event.get.param.prog.file_size = p_buffer->prog.file_size;
            pbc_event.get.param.prog.num_bytes = p_buffer->prog.bytes;

            hci_control_send_pbc_event(wiced_bt_pbc_return_evt_code(), &pbc_event, sizeof(pbc_event.get));
        }
        break;

    case WICED_BT_PBC_CHDIR_EVT:
        WICED_BT_TRACE("WICED_BT_PBC_CHDIR_EVT event");

        if (wiced_bt_pbc_is_connected())
        {
            //Not Implemented
            //  pbc_event.set.type = WICED_BT_PBC_SET_PARAM_CHDIR;
            //  pbc_event.set.status = p_buffer->status;
            //  hci_control_send_pbc_event(HCI_CONTROL_PBC_SET_EVT, &pbc_event, sizeof(pbc_event.set));
        }
        break;

    case WICED_BT_PBC_PHONEBOOK_EVT:
        WICED_BT_TRACE("WICED_BT_PBC_PHONEBOOK_EVT event: Status %d Phone book size %d", p_buffer->status, p_buffer->pb.phone_book_size );

        if (wiced_bt_pbc_is_connected())
        {
            pbc_event.get.status = WICED_BT_SUCCESS;
            pbc_event.get.type = WICED_BT_PBC_GET_PARAM_PHONEBOOK;
            pbc_event.get.param.pb.phone_book_size = p_buffer->pb.phone_book_size;
            pbc_event.get.param.pb.pbs_exist = p_buffer->pb.pbs_exist;
            pbc_event.get.param.pb.new_missed_calls = p_buffer->pb.new_missed_calls;
            pbc_event.get.param.pb.nmc_exist = p_buffer->pb.nmc_exist;

            hci_control_send_pbc_event(wiced_bt_pbc_return_evt_code(), &pbc_event, sizeof(pbc_event.get));
        }
        break;

    case WICED_BT_PBC_GETFILE_EVT:
        WICED_BT_TRACE("WICED_BT_PBC_GETFILE_EVT event status = %d",p_buffer->status);
        wiced_bt_pbc_clear_pending_req();
        if (wiced_bt_pbc_is_connected() >= 0)
        {
            byteval = p_buffer->status;
            switch (p_buffer->status)
            {
            case WICED_BT_PBC_OK:
                WICED_BT_TRACE("Transfer Complete");
                break;
            case WICED_BT_PBC_NO_PERMISSION:
                WICED_BT_TRACE("Transfer Failed No permission");
                break;
            case WICED_BT_PBC_NOT_FOUND:
                WICED_BT_TRACE("Transfer Failed, File Not Found");
                break;
            case WICED_BT_PBC_FULL:
                WICED_BT_TRACE("Transfer Failed, Full or Too Big");
                break;
            case WICED_BT_PBC_ABORTED:
                WICED_BT_TRACE("Transfer Aborted");
                break;
            default:
                byteval = WICED_BT_PBC_FAIL;
                WICED_BT_TRACE("Transfer Failed");
                break;
            }

            pbc_event.get.status = byteval;
            pbc_event.get.type = WICED_BT_PBC_GET_PARAM_FILE_TRANSFER_STATUS;
            hci_control_send_pbc_event(wiced_bt_pbc_return_evt_code(), &pbc_event, 2);

        }
        break;

    default:
        WICED_BT_TRACE("wiced_bt_pbc_event_callback: e=%d unsupported", event);
        break;
    }
}

wiced_result_t pbap_client_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    int nvram_id;
    int bytes_written, bytes_read;
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_dev_pairing_cplt_t *p_pairing_cmpl;
    uint8_t pairing_result;
    wiced_bt_dev_encryption_status_t *p_encryption_status;

    WICED_BT_TRACE( "Bluetooth management callback event: 0x%02x\n", event );

    switch (event)
    {
    case BTM_ENABLED_EVT:
        pbap_client_post_bt_init(p_event_data);

        //Creating a buffer pool for holding the peer devices's key info
        p_key_info_pool = wiced_bt_create_pool( KEY_INFO_POOL_BUFFER_SIZE,
        KEY_INFO_POOL_BUFFER_COUNT);
        WICED_BT_TRACE( "wiced_bt_create_pool %x\n", p_key_info_pool );

        wiced_bt_dev_register_hci_trace(hci_control_hci_trace_cback);

        hci_control_send_device_started_evt();

        break;

    case BTM_DISABLED_EVT:
        break;

    case BTM_SECURITY_REQUEST_EVT:
        if (pairing_allowed)
        {
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
        } else
        {
            // Pairing not allowed, return error
            result = WICED_BT_ERROR;
        }
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        p_pairing_cmpl = &p_event_data->pairing_complete;

        if (p_pairing_cmpl->transport == BT_TRANSPORT_BR_EDR)
        {
            pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
        } else
        {
            pairing_result = p_pairing_cmpl->pairing_complete_info.ble.reason;
        }
        hci_control_send_pairing_completed_evt(pairing_result, p_event_data->pairing_complete.bd_addr);
        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        break;

    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        /* Check if we already have information saved for this bd_addr */
        if ((nvram_id = hci_control_find_nvram_id(p_event_data->paired_device_link_keys_update.bd_addr,
        BD_ADDR_LEN)) == 0)
        {
            // This is the first time, allocate id for the new memory chunk
            nvram_id = hci_control_alloc_nvram_id();
            WICED_BT_TRACE( "Allocated NVRAM ID:%d\n", nvram_id );
        }
        bytes_written = hci_control_write_nvram(nvram_id, sizeof(wiced_bt_device_link_keys_t),
                &p_event_data->paired_device_link_keys_update, WICED_FALSE);

        WICED_BT_TRACE("NVRAM write:id:%d bytes:%d dev: [%B]\n", nvram_id, bytes_written, p_event_data->paired_device_link_keys_update.bd_addr);
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        /* read existing key from the NVRAM  */

        WICED_BT_TRACE("\t\tfind device %B\n", p_event_data->paired_device_link_keys_request.bd_addr);

        if ((nvram_id = hci_control_find_nvram_id(p_event_data->paired_device_link_keys_request.bd_addr,
        BD_ADDR_LEN)) != 0)
        {
            bytes_read = hci_control_read_nvram(nvram_id, &p_event_data->paired_device_link_keys_request,
                    sizeof(wiced_bt_device_link_keys_t));

            result = WICED_BT_SUCCESS;
            WICED_BT_TRACE("Read:nvram_id:%d bytes:%d\n", nvram_id, bytes_read);
        } else
        {
            result = WICED_BT_ERROR;
            WICED_BT_TRACE("Key retrieval failure\n");
        }
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        /* Use the default security for BLE */
        WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT bda %B\n",
                p_event_data->pairing_io_capabilities_ble_request.bd_addr);
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC_MITM_BOND;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size = 16;
        p_event_data->pairing_io_capabilities_ble_request.init_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
        /* Use the default security for BR/EDR*/
        WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT bda %B\n", p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
        p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_br_edr_request.auth_req = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
        p_event_data->pairing_io_capabilities_br_edr_request.oob_data =
        WICED_FALSE;
        break;

    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        p_encryption_status = &p_event_data->encryption_status;
        WICED_BT_TRACE( "Encryption Status:(%B) res:%d\n", p_encryption_status->bd_addr, p_encryption_status->result );
    default:
        break;
    }
    return result;
}

/*
 *  Application Start, ie, entry point to the application.
 */
APPLICATION_START()
{
#if defined WICED_BT_TRACE_ENABLE || defined HCI_TRACE_OVER_TRANSPORT
    wiced_transport_init( &transport_cfg );

    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    // Note: WICED HCI must be configured to use this - see wiced_trasnport_init(), must
    // be called with wiced_transport_cfg_t.wiced_tranport_data_handler_t callback present
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

    wiced_transport_init(&transport_cfg);

    WICED_BT_TRACE( "Starting PBAP Client Application...\n" );

    /* Initialize Bluetooth stack */
    wiced_bt_stack_init(pbap_client_management_callback, &pbap_client_cfg_settings, pbap_client_cfg_buf_pools);

    transport_pool = wiced_transport_create_buffer_pool( TRANS_UART_BUFFER_SIZE,
    TRANS_UART_BUFFER_COUNT);
}
