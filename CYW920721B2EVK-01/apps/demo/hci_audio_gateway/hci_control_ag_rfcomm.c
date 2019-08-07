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
 * RFCOMM for HF Device sample application.
 *
 */

#include "wiced_bt_cfg.h"
#include "hci_control.h"
#include "hci_control_api.h"
#include "string.h"
#include "wiced_bt_rfcomm.h"

static void hci_control_ag_rfcomm_closed( hci_control_ag_session_cb_t *p_scb );
static void hci_control_ag_rfcomm_opened( hci_control_ag_session_cb_t *p_scb );
static void hci_control_ag_rfcomm_acceptor_opened( hci_control_ag_session_cb_t *p_scb );
void hci_control_ag_rfcomm_do_open( hci_control_ag_session_cb_t *p_scb );
extern void hci_control_ag_process_open_callback( hci_control_ag_session_cb_t *p_scb, uint8_t status );

/*
 * Process RFCOMM data received from the peer
 */
int hci_control_ag_rfcomm_data_callback( uint16_t port_handle, void *p_data, uint16_t len )
{
    char                        *p = ( char * )p_data;
    hci_control_ag_session_cb_t *p_scb;
    uint8_t                      i;

	for ( i = 0, p_scb = &hci_control_cb.ag_scb[0]; i < HCI_CONTROL_AG_NUM_SCB; i++, p_scb++ )
    {
        if ( ( port_handle == p_scb->rfc_conn_handle ) )
            break;
    }

    if ( i == HCI_CONTROL_AG_NUM_SCB )
    {
        WICED_BT_TRACE( "hci_control_ag_rfcomm_data_callback: Unknown port handle: 0x%04x\n", port_handle );
        return 0;
    }

    //Ash DumpData( "RECV:", p_data, len );

    if ( p_scb->res_len == 0 )
    {
        memset( p_scb->res_buf, 0, HCI_CONTROL_AG_AT_MAX_LEN );
    }

    if ( ( p_scb->res_len + len ) > HCI_CONTROL_AG_AT_MAX_LEN )
    {
        WICED_BT_TRACE( "[%u]handle_rcvd_data: too much data res_len %u  len: %u\n", p_scb->app_handle, p_scb->res_len, len );
    }
    else if ( len > 0 )
    {
        memcpy( &p_scb->res_buf[p_scb->res_len], p, len );
        p_scb->res_len += len;

        hci_control_ag_parse_AT_command (p_scb);
    }

    return ( ( int )len );
}


/*
 * RFCOMM management callback
 */
static void hci_control_ag_rfcomm_control_callback( uint32_t port_status, uint16_t port_handle )
{
    hci_control_ag_session_cb_t *p_scb;
    uint8_t                      i;

	for ( i = 0, p_scb = &hci_control_cb.ag_scb[0]; i < HCI_CONTROL_AG_NUM_SCB; i++, p_scb++ )
    {
        if ( ( port_handle == p_scb->rfc_serv_handle ) || ( port_handle == p_scb->rfc_conn_handle ) )
            break;
    }

    if ( i == HCI_CONTROL_AG_NUM_SCB )
    {
        WICED_BT_TRACE( "hci_control_ag_rfcomm_control_callback: Unknown port: 0x%04x   SCB1: %u 0x%04x 0x%04x  SCB2: %u 0x%04x 0x%04x\n",
                         port_handle,
						 hci_control_cb.ag_scb[0].state, hci_control_cb.ag_scb[0].rfc_serv_handle, hci_control_cb.ag_scb[0].rfc_conn_handle,
						 hci_control_cb.ag_scb[1].state, hci_control_cb.ag_scb[1].rfc_serv_handle, hci_control_cb.ag_scb[1].rfc_conn_handle);
        return;
    }

    WICED_BT_TRACE( "[%u]hci_control_ag_rfcomm_control_callback : Status = %d, port: 0x%04x  SCB state: %u  Srv: 0x%04x  Conn: 0x%04x\n",
                    p_scb->app_handle, port_status, port_handle, p_scb->state, p_scb->rfc_serv_handle, p_scb->rfc_conn_handle );

    /* ignore close event for port handles other than connected handle */
    if ( ( port_status != WICED_BT_RFCOMM_SUCCESS ) && ( port_handle != p_scb->rfc_conn_handle ) )
    {
        WICED_BT_TRACE( "hci_control_ag_rfcomm_control_callback ignoring handle:%d", port_handle );
        return;
    }

    if ( ( port_status == WICED_BT_RFCOMM_SUCCESS ) && ( p_scb->state != HCI_CONTROL_AG_STATE_CLOSING) )
    {
        wiced_bt_rfcomm_set_data_callback( port_handle, hci_control_ag_rfcomm_data_callback );

        i = p_scb->state;
        p_scb->state = HCI_CONTROL_AG_STATE_OPEN;

        if ( i == HCI_CONTROL_AG_STATE_IDLE )
            hci_control_ag_rfcomm_acceptor_opened( p_scb );
        else
            hci_control_ag_rfcomm_opened( p_scb );

        hci_control_ag_sco_create( p_scb, WICED_FALSE );                  /* Listen for a SCO */
    }
    else
    {
        hci_control_ag_rfcomm_closed( p_scb );
    }
}

/*
 * Setup RFCOMM server, in listen mode.
 */
void hci_control_ag_rfcomm_start_server( hci_control_ag_session_cb_t *p_scb )
{
    wiced_bt_rfcomm_result_t rfcomm_result;

    p_scb->state = HCI_CONTROL_AG_STATE_IDLE;

    if ( !p_scb->rfc_serv_handle )
    {
        rfcomm_result = wiced_bt_rfcomm_create_connection( UUID_SERVCLASS_AG_HANDSFREE,
                                                           HFP_RFCOMM_SCN, WICED_TRUE, HFP_DEVICE_MTU, bd_addr_any,
                                                           &p_scb->rfc_serv_handle,
                                                           ( wiced_bt_port_mgmt_cback_t * )hci_control_ag_rfcomm_control_callback );

        WICED_BT_TRACE( "[%u]hci_control_ag_rfcomm_start_server: rfcomm_create Res: 0x%x  Port: 0x%04x\n",
                        p_scb->app_handle, rfcomm_result, p_scb->rfc_serv_handle );
    }
    else
    {
        WICED_BT_TRACE( "[%u]hci_control_ag_rfcomm_start_server: rfcomm_create Port Already set to: 0x%04x\n",
                        p_scb->app_handle, p_scb->rfc_serv_handle );
    }
}

/*
 * Open an RFCOMM connection to the peer device.
 */
void hci_control_ag_rfcomm_do_open( hci_control_ag_session_cb_t *p_scb )
{
    wiced_bt_rfcomm_result_t rfcomm_result;

    WICED_BT_TRACE( "[%u]hci_control_ag_rfcomm_do_open: rfc_serv_handle: 0x%04x\n",
                    p_scb->app_handle, p_scb->rfc_serv_handle );

    rfcomm_result = wiced_bt_rfcomm_create_connection( UUID_SERVCLASS_HF_HANDSFREE,
                                                      p_scb->hf_scn, WICED_FALSE,
                                                      HFP_DEVICE_MTU,
                                                      p_scb->hf_addr,
                                                      &p_scb->rfc_conn_handle,
                                                      ( wiced_bt_port_mgmt_cback_t * )hci_control_ag_rfcomm_control_callback );

    WICED_BT_TRACE( "[%u]hci_control_ag_rfcomm_do_open - rfcomm_create Res: 0x%x   Port: 0x%04x\n",
                     p_scb->app_handle, rfcomm_result, p_scb->rfc_conn_handle );

    if ( rfcomm_result != WICED_BT_RFCOMM_SUCCESS )
    {
        /* Pass back that the connection attempt failed */
        hci_control_ag_open_t    open;

        /* call app callback with open event */
        open.status = rfcomm_result;

        utl_bdcpy( open.bd_addr, p_scb->hf_addr );

        hci_control_send_ag_event( HCI_CONTROL_AG_EVENT_OPEN, p_scb->app_handle, (hci_control_ag_event_t *)&open );

        hci_control_ag_rfcomm_start_server( p_scb );
    }

}

/*
 * Close RFCOMM connection.
 */
void hci_control_ag_rfcomm_do_close( hci_control_ag_session_cb_t *p_scb )
{
    wiced_bt_rfcomm_result_t rfcomm_result;

    if ( p_scb->rfc_conn_handle )
    {
        p_scb->state = HCI_CONTROL_AG_STATE_CLOSING;

        // Disconnect RFCOMM keeping server listening
        rfcomm_result = wiced_bt_rfcomm_remove_connection( p_scb->rfc_conn_handle, FALSE );

        WICED_BT_TRACE( "[%u]wiced_bt_rfcomm_remove_connection (0x%04x) result 0x%x\n",
                        p_scb->app_handle, p_scb->rfc_conn_handle, rfcomm_result );
    }
    else
    {
        WICED_BT_TRACE( "[%u]wiced_bt_rfcomm_remove_connection - conn_handle zero\n", p_scb->app_handle );
        hci_control_ag_rfcomm_start_server (p_scb);
    }
}

/*
 * RFCOMM connection closed.
 */
void hci_control_ag_rfcomm_closed( hci_control_ag_session_cb_t *p_scb )
{
    /* call appropriate close cback */
    if ( p_scb->state == HCI_CONTROL_AG_STATE_OPENING )
        hci_control_ag_process_open_callback( p_scb, HCI_CONTROL_HF_STATUS_FAIL_RFCOMM );
    else
        hci_control_send_ag_event( HCI_CONTROL_AG_EVENT_CLOSE, p_scb->app_handle, NULL );

    /* Clear peer bd_addr */
    utl_bdcpy( p_scb->hf_addr, bd_addr_null );

    hci_control_ag_sco_close( p_scb );

    p_scb->sco_idx         = BTM_INVALID_SCO_INDEX;
    p_scb->rfc_conn_handle = 0;
    p_scb->state           = HCI_CONTROL_AG_STATE_IDLE;

    /* Reopen server if needed */
    hci_control_ag_rfcomm_start_server (p_scb);
}

/*
 * Handle RFCOMM channel opened.
 */
static void hci_control_ag_rfcomm_opened ( hci_control_ag_session_cb_t *p_scb )
{
    p_scb->state    = HCI_CONTROL_AG_STATE_OPEN;

    /* reinitialize stuff */
    p_scb->hf_features          = 0;
    p_scb->b_slc_is_up          = WICED_FALSE;
    p_scb->res_len              = 0;
    p_scb->sco_idx              = BTM_INVALID_SCO_INDEX;
    p_scb->b_sco_opened         = WICED_FALSE;

#if (BTM_WBS_INCLUDED == TRUE)
    p_scb->peer_supports_msbc   = WICED_FALSE;
    p_scb->msbc_selected        = WICED_FALSE;
#endif

    hci_control_ag_process_open_callback ( p_scb, HCI_CONTROL_HF_STATUS_SUCCESS );

    WICED_BT_TRACE( "[%u]RFCOMM Connected  isInit: %u  Serv: 0x%04x   Conn: 0x%04x\n",
                    p_scb->app_handle, p_scb->b_is_initiator, p_scb->rfc_serv_handle, p_scb->rfc_conn_handle );
}

/*
 * Handle RFCOMM channel opened when accepting connection.
 */
void hci_control_ag_rfcomm_acceptor_opened( hci_control_ag_session_cb_t *p_scb )
{
    uint16_t lcid;
    int      i;
    int      status;

    /* set role and connection handle */
    p_scb->b_is_initiator  = WICED_FALSE;
    p_scb->rfc_conn_handle = p_scb->rfc_serv_handle;

    /* get bd addr of peer */
    if ( WICED_BT_RFCOMM_SUCCESS != ( status = wiced_bt_rfcomm_check_connection( p_scb->rfc_conn_handle, p_scb->hf_addr, &lcid ) ) )
    {
        WICED_BT_TRACE( "[%u]hci_control_ag_rfcomm_acceptor_opened error PORT_CheckConnection returned status %d\n", p_scb->app_handle, status );
    }

    /* do service discovery to get features */
    hci_control_ag_sdp_start_discovery( p_scb );

    /* continue with common open processing */
    hci_control_ag_rfcomm_opened( p_scb );
}
