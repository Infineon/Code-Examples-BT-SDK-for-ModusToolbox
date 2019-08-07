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
 * This file implement handsfree audio gateway application controlled over UART
 *
 */
#include "hci_control.h"
#include "hci_control_api.h"
#include "string.h"
#include "wiced_transport.h"
#include "wiced_bt_rfcomm.h"

void hci_control_ag_process_open_callback( hci_control_ag_session_cb_t *p_scb, uint8_t status );

/*****************************************************************************
 **  Constants
 *****************************************************************************/

#if (BTM_WBS_INCLUDED == TRUE )
UINT32 ag_features = HCI_CONTROL_AG_FEAT_VREC | HCI_CONTROL_AG_FEAT_CODEC | HCI_CONTROL_AG_FEAT_ESCO;
#else
UINT32 ag_features = HCI_CONTROL_AG_FEAT_VREC | HCI_CONTROL_AG_FEAT_ESCO;
#endif

/* global constant for "any" bd addr */
BD_ADDR bd_addr_any  = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
BD_ADDR bd_addr_null = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

wiced_bt_voice_path_setup_t ag_sco_path = {
#ifdef CYW20706A2
    .path = WICED_BT_SCO_OVER_I2SPCM,
#else
    .path = WICED_BT_SCO_OVER_PCM,
#endif
};

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 * Start up  the handsfree service.
 */
void hci_control_ag_startup( void )
{
    hci_control_ag_session_cb_t *p_scb = &hci_control_cb.ag_scb[0];
    wiced_bt_dev_status_t result;
    int i;

    memset( &hci_control_cb, 0, sizeof( hci_control_cb ) );

    for ( i = 0; i < HCI_CONTROL_AG_NUM_SCB; i++, p_scb++ )
    {
        p_scb->app_handle = ( uint16_t ) ( i + 1 );

        wiced_init_timer( &p_scb->cn_timer, hci_control_cn_timeout, ( uint32_t ) p_scb, WICED_SECONDS_TIMER );

        p_scb->sco_idx = BTM_INVALID_SCO_INDEX;

        /* start RFCOMM server */
        hci_control_ag_rfcomm_start_server( p_scb );
    }

//    Default mode is set to I2S Master so no need to call this API
//    To change the mode please call below API and to update PCM configuration use wiced_hal_set_pcm_config API
//    result = wiced_bt_sco_setup_voice_path(&ag_sco_path);
//    WICED_BT_TRACE("[%s] SCO Setting up voice path = %d\n",__func__, result);
}

/*
 * Handle Handsfree commands received over UART.
 */
void hci_control_ag_handle_command( uint16_t opcode, uint8_t* p_data, uint32_t length )
{
    uint16_t handle;
    uint8_t  hs_cmd;
    uint8_t  *p = ( uint8_t * ) p_data;

    switch ( opcode )
    {
    case HCI_CONTROL_AG_COMMAND_CONNECT:
        hci_control_ag_connect( p );
        break;

    case HCI_CONTROL_AG_COMMAND_DISCONNECT:
        handle = p[0] | ( p[1] << 8 );
        hci_control_ag_disconnect( handle );
        break;

    case HCI_CONTROL_AG_COMMAND_OPEN_AUDIO:
        handle = p[0] | ( p[1] << 8 );
        hci_control_ag_audio_open( handle );
        break;

    case HCI_CONTROL_AG_COMMAND_CLOSE_AUDIO:
        handle = p[0] | ( p[1] << 8 );
        hci_control_ag_audio_close( handle );
        break;

    default:
        WICED_BT_TRACE ( "hci_control_ag_handle_command - unkn own opcode: %u %u\n", opcode);
        break;
    }
}

/*
 * Opens a connection to an HF device.  When connection is opened callback
 * function is called with a HCI_CONTROL_HF_EVENT_CONNECTED. Only the service
 * level data connection is opened. The audio connection is not.
 */
void hci_control_ag_connect( BD_ADDR bd_addr )
{
    hci_control_ag_session_cb_t *p_scb = &hci_control_cb.ag_scb[0];
    int                          i;

    for ( i = 0; i < HCI_CONTROL_AG_NUM_SCB; i++, p_scb++ )
    {
        if ( p_scb->state == HCI_CONTROL_AG_STATE_IDLE )
            break;
    }

    if ( i == HCI_CONTROL_AG_NUM_SCB )
    {
        WICED_BT_TRACE ( "hci_control_ag_connect - no free control block for connection, States: %u %u\n",
                        hci_control_cb.ag_scb[0].state, hci_control_cb.ag_scb[1].state );
        return;
    }

    p_scb->state = HCI_CONTROL_AG_STATE_OPENING;

    /* store parameters */
    STREAM_TO_BDADDR(p_scb->hf_addr,bd_addr);

    /* close RFCOMM server, if listening on this SCB */
    if ( p_scb->rfc_serv_handle )
    {
        wiced_bt_rfcomm_remove_connection( p_scb->rfc_serv_handle, WICED_TRUE );
        p_scb->rfc_serv_handle = 0;
    }

    /* set role */
    p_scb->b_is_initiator = WICED_TRUE;

    /* do service search */
    hci_control_ag_sdp_start_discovery( p_scb );
}

/*
 * Close the current connection to an audio gateway.  Any current audio
 * connection will also be closed
 */
void hci_control_ag_disconnect( uint16_t handle )
{
    hci_control_ag_session_cb_t *p_scb = &hci_control_cb.ag_scb[handle - 1];

    WICED_BT_TRACE( "[%u]hci_control_ag_disconnect   State: %u\n", p_scb->app_handle, p_scb->state );

    if ( p_scb->state == HCI_CONTROL_AG_STATE_OPENING )
    {
        p_scb->state = HCI_CONTROL_AG_STATE_CLOSING;
        hci_control_ag_rfcomm_do_close( p_scb );
    }
    else if ( p_scb->state == HCI_CONTROL_AG_STATE_OPEN )
    {
        p_scb->state = HCI_CONTROL_AG_STATE_CLOSING;

        /* if SCO is open close SCO and wait on RFCOMM close */
        if ( !p_scb->b_sco_opened )
            hci_control_ag_rfcomm_do_close( p_scb );

        /* always do SCO shutdown to handle all SCO corner cases */
        hci_control_ag_sco_close( p_scb );
    }
}

/*
 * Opens an audio connection to the currently connected audio gateway
 */
void hci_control_ag_audio_open( uint16_t handle )
{
    hci_control_ag_session_cb_t *p_scb = &hci_control_cb.ag_scb[handle - 1];

    WICED_BT_TRACE( "hci_control_ag_audio_open - state: %u  SCO inx: 0x%02x\n", p_scb->state, p_scb->sco_idx );

    /* If already open, just return success */
    if ( p_scb->b_sco_opened )
    {
        hci_control_send_ag_event( HCI_CONTROL_AG_EVENT_AUDIO_OPEN, handle, NULL );
        return;
    }

    if ( p_scb->state == HCI_CONTROL_AG_STATE_OPEN )
    {
        /* Assume we are bringing up a SCO for voice recognition, so send BVRA */
        hci_control_ag_send_BVRA_to_hf( p_scb, TRUE );
        hci_control_ag_sco_create( p_scb, TRUE );
    }
}

/*
 * Close the currently active audio connection to a audio gateway. The data
 * connection remains open
 */
void hci_control_ag_audio_close( uint16_t handle )
{
    hci_control_ag_session_cb_t *p_scb = &hci_control_cb.ag_scb[handle - 1];

    if ( p_scb->b_sco_opened )
    {
        /* Assume we had brought up the SCO for voice recognition, so send BVRA */
        hci_control_ag_send_BVRA_to_hf( p_scb, FALSE );
        hci_control_ag_sco_close( p_scb );
    }
}

/*
 * Find SCB associated with AG BD address.
 */
hci_control_ag_session_cb_t *hci_control_ag_find_scb_by_sco_index( uint16_t sco_idx )
{
    hci_control_ag_session_cb_t *p_scb = &hci_control_cb.ag_scb[0];
    uint16_t i;

    for ( i = 0; i < HCI_CONTROL_AG_NUM_SCB; i++, p_scb++ )
    {
        if ( p_scb->sco_idx == sco_idx )
            return ( p_scb );
    }

    /* no scb found */
    WICED_BT_TRACE( "No scb for SCO inx: %u\n", sco_idx );
    return NULL;
}

/*
 * Send open callback event to application.
 */
void hci_control_ag_process_open_callback( hci_control_ag_session_cb_t *p_scb, uint8_t status )
{
    hci_control_ag_open_t open;

    /* call app callback with open event */
    open.status = status;

    if ( status == HCI_CONTROL_HF_STATUS_SUCCESS )
        utl_bdcpy( open.bd_addr, p_scb->hf_addr );
    else
        utl_bdcpy( p_scb->hf_addr, bd_addr_null );

    hci_control_send_ag_event( HCI_CONTROL_AG_EVENT_OPEN, p_scb->app_handle, ( hci_control_ag_event_t * ) &open );
}

/*
 * Service level connection opened
 */
void hci_control_ag_service_level_up( hci_control_ag_session_cb_t *p_scb )
{
    hci_control_ag_connect_t evt;

    /* Only tell it once */
    if ( !p_scb->b_slc_is_up )
    {
        p_scb->b_slc_is_up = WICED_TRUE;

        evt.peer_features = p_scb->hf_features;

        /* call callback */
        hci_control_send_ag_event( HCI_CONTROL_AG_EVENT_CONNECTED, p_scb->app_handle, ( hci_control_ag_event_t * ) &evt );
    }
}

/*
 * HF event callback. Format the data to be sent over the UART
 *
 * Format of transmit buffer:
 *          1 byte   HFP event code
 *          2 bytes  handle
 *          n bytes  data depending on event code
 */
void hci_control_send_ag_event( uint16_t evt, uint16_t handle, hci_control_ag_event_t *p_data )
{
    uint8_t   tx_buf[300];
    uint8_t  *p = tx_buf;
    int       i;

    WICED_BT_TRACE("[%u]hci_control_send_ag_event: Sending Event: %u  to UART\n", handle, evt);

    *p++ = ( uint8_t ) ( handle );
    *p++ = ( uint8_t ) ( handle >> 8 );

    switch ( evt )
    {
    case HCI_CONTROL_AG_EVENT_OPEN:       /* HS connection opened or connection attempt failed  */
        for ( i = 0; i < BD_ADDR_LEN; i++ )
            *p++ = p_data->open.bd_addr[BD_ADDR_LEN - 1 - i];
        *p++ = p_data->open.status;
        break;

    case HCI_CONTROL_AG_EVENT_CONNECTED: /* HS Service Level Connection is UP */
        *p++ = ( uint8_t ) ( p_data->conn.peer_features );
        *p++ = ( uint8_t ) ( p_data->conn.peer_features >> 8 );
        break;

    default:                             /* Rest have no parameters */
        break;
    }

    wiced_transport_send_data( evt, tx_buf, ( int ) ( p - tx_buf ) );
}
