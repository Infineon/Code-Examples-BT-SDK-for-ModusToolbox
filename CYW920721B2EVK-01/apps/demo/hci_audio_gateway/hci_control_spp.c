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
 * This file implement serial port profile application controlled over UART
 *
 */
#include "hci_control.h"
#include "hci_control_api.h"
#include "wiced_transport.h"
#include "string.h"
#include "wiced_memory.h"
#include "wiced_bt_rfcomm.h"

/*****************************************************************************
**  Constants
*****************************************************************************/

/******************************************************
*                 Global Variables
******************************************************/
static wiced_bt_uuid_t  spp_uuid = {2, {UUID_SERVCLASS_SERIAL_PORT}};

/* global constant for "any" bd addr */
static BD_ADDR bd_addr_any_spp = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

extern wiced_transport_buffer_pool_t *rfcomm_trans_pool;
extern wiced_timer_t rfcomm_trans_data_timer;
extern uint16_t rfcomm_trans_data_len;
extern uint8_t* p_rfcomm_trans_buf;
extern wiced_timer_t rfcomm_flow_control_timer;

/******************************************************
 *               Function Definitions
 ******************************************************/
static void    hci_control_spp_rfcomm_closed( hci_control_spp_session_cb_t *p_scb );
static void    hci_control_spp_rfcomm_opened( hci_control_spp_session_cb_t *p_scb );
static void    hci_control_spp_rfcomm_acceptor_opened( hci_control_spp_session_cb_t *p_scb );
static void    hci_control_port_event_cback( wiced_bt_rfcomm_port_event_t event, uint16_t port_handle );
static void    hci_control_spp_sdp_start_discovery( hci_control_spp_session_cb_t *p_scb );
static BOOLEAN hci_control_spp_sdp_find_attr( hci_control_spp_session_cb_t *p_scb );
static void    hci_control_spp_sdp_cback( uint16_t status );
static void    hci_control_spp_sdp_free_db( hci_control_spp_session_cb_t *p_scb );
static void    hci_control_spp_send_event( hci_control_spp_session_cb_t *p_scb, uint16_t event );
static void    hci_control_spp_rfcomm_trans_data_timer( uint32_t param );
static void    hci_control_spp_rfcomm_flow_control_timer( uint32_t param );

uint32_t spp_total_bytes = 0;
uint32_t spp_send_data_ctr = 0;

wiced_bt_rfcomm_result_t hci_spp_write( uint16_t handle, uint8_t * p_data, uint16_t length, uint16_t *p_bytes_written )
{
    wiced_bt_rfcomm_result_t  result ;

    result = wiced_bt_rfcomm_write_data( handle, ( char * ) p_data, length, p_bytes_written );

    if ( result == WICED_BT_RFCOMM_SUCCESS )
    {
        spp_total_bytes += length;
    }

    // WICED_BT_TRACE( "hspp exit: %d %d \n", spp_total_bytes, result );
    return result;
}

#if 0
uint8_t gau8Rfcomm[248];
uint32_t send_counter = 0;
uint32_t last_sent = '0';
volatile uint32_t rfcomm_fc = 0;
void hci_control_spp_generate_and_send( uint16_t handle)
{
    wiced_bt_rfcomm_result_t result ;
    int to_write = sizeof(gau8Rfcomm)/sizeof(gau8Rfcomm[0]);

    while(1){
        uint16_t bytes_written;
        int written = 0;

        memcpy(gau8Rfcomm + written, &send_counter, sizeof(send_counter));
        written += sprintf(gau8Rfcomm + written, "%08x",  send_counter);

        memset( gau8Rfcomm + written, last_sent, to_write - written );

        result = hci_spp_write( handle, gau8Rfcomm, to_write , &bytes_written );
        if(result != WICED_BT_RFCOMM_SUCCESS){
            WICED_BT_TRACE(" breaking off @ %d res %d wr %d", send_counter, result, bytes_written);
            break;
        }

        last_sent++;
        if(last_sent == ('9'  + 1)){
            last_sent = 'a';
        }else if(last_sent == ('z' + 1)){
            last_sent = '0';
        }

        send_counter++;
    }
}
#endif

/*
 * Start up  the SPP service.
 */
void hci_control_spp_startup( void )
{
    hci_control_spp_session_cb_t *p_scb = &hci_control_cb.spp_scb;

    wiced_init_timer( &rfcomm_trans_data_timer, hci_control_spp_rfcomm_trans_data_timer, 0, WICED_MILLI_SECONDS_TIMER );
    wiced_init_timer( &rfcomm_flow_control_timer, hci_control_spp_rfcomm_flow_control_timer, 0, WICED_MILLI_SECONDS_TIMER );

    /* start RFCOMM server */
    hci_control_spp_rfcomm_start_server( p_scb );
}

/*
 * Hanlde various SPP commands received over UART
 */
void hci_control_spp_handle_command( uint16_t cmd_opcode, uint8_t* p, uint32_t data_len )
{
    uint16_t handle;
    switch ( cmd_opcode )
    {
    case HCI_CONTROL_SPP_COMMAND_CONNECT:
        hci_control_spp_connect( p );
        break;

    case HCI_CONTROL_SPP_COMMAND_DISCONNECT:
        handle = p[0] | ( p[1] << 8 );
        hci_control_spp_disconnect( handle );
        break;

    case HCI_CONTROL_SPP_COMMAND_DATA:
        handle = p[0] | ( p[1] << 8 );
        hci_control_spp_send_data( handle, &p[2], data_len - 2 );
        break;
    }
}

/*
 * Opens a connection to a SPP server gateway.  When connection is opened callback
 * function is called with a HCI_CONTROL_SPP_EVENT_CONNECTED.
 */
void hci_control_spp_connect( BD_ADDR bd_addr )
{
    hci_control_spp_session_cb_t *p_scb = &hci_control_cb.spp_scb;
    int                          i;

    if ( p_scb->state != HCI_CONTROL_SPP_STATE_IDLE )
    {
        WICED_BT_TRACE ( "hci_control_spp_connect - no free control block for connection, State: %u\n", p_scb->state);
        return;
    }

    p_scb->state = HCI_CONTROL_SPP_STATE_OPENING;

    /* store parameters */
    STREAM_TO_BDADDR(p_scb->server_addr,bd_addr);

    /* close RFCOMM server, if listening on this SCB */
    if ( p_scb->rfc_serv_handle )
    {
        wiced_bt_rfcomm_remove_connection( p_scb->rfc_serv_handle, WICED_FALSE );
        p_scb->rfc_serv_handle = 0;
    }

    /* set role */
    p_scb->b_is_initiator = WICED_TRUE;

    /* do service search */
    hci_control_spp_sdp_start_discovery( p_scb );
}

/*
 * Close the current connection to an audio gateway.  Any current audio
 * connection will also be closed
 */
void hci_control_spp_disconnect( uint16_t handle )
{
    hci_control_spp_session_cb_t *p_scb = &hci_control_cb.spp_scb;

    WICED_BT_TRACE( "hci_control_spp_disconnect   State: %u\n", p_scb->state );

    if ( p_scb->state == HCI_CONTROL_SPP_STATE_OPENING )
    {
        p_scb->state = HCI_CONTROL_SPP_STATE_CLOSING;
        hci_control_spp_rfcomm_do_close( p_scb );
    }
    else if ( p_scb->state == HCI_CONTROL_SPP_STATE_OPEN )
    {
        p_scb->state = HCI_CONTROL_SPP_STATE_CLOSING;

        hci_control_spp_rfcomm_do_close( p_scb );
    }
}

void hci_control_spp_send_data_to_host( void )
{
    WICED_BT_TRACE( "Send to Host:%x,  %d\n", p_rfcomm_trans_buf, rfcomm_trans_data_len );
    if ( p_rfcomm_trans_buf && rfcomm_trans_data_len )
    {
        wiced_transport_send_buffer( HCI_CONTROL_SPP_EVENT_RX_DATA, p_rfcomm_trans_buf, rfcomm_trans_data_len );
        p_rfcomm_trans_buf = NULL;
        rfcomm_trans_data_len = 0;
    }
}

/*
 * Process RFCOMM data received from the peer
 */
int hci_control_spp_rfcomm_data_callback( uint16_t port_handle, void *p_data, uint16_t len )
{

    uint8_t*    p_buf;
    uint16_t    copy_len = 0;
    uint8_t     alloc_buff = TRUE;
    uint16_t    buffer_size = wiced_transport_get_buffer_size( rfcomm_trans_pool );

    WICED_BT_TRACE("   spp_data_cb: %x, %d \n", p_data, len );
    if ( ( p_rfcomm_trans_buf ) && ( rfcomm_trans_data_len < buffer_size ) )
    {
        if ( len <= ( buffer_size - rfcomm_trans_data_len ) )
        {
            copy_len = len;
            alloc_buff = FALSE;
        }
        else
        {
            copy_len = buffer_size - rfcomm_trans_data_len;
        }
        memcpy( p_rfcomm_trans_buf + rfcomm_trans_data_len , p_data, copy_len );
        rfcomm_trans_data_len += copy_len;
        len -= copy_len;
    }
    if ( rfcomm_trans_data_len == buffer_size )
    {
        hci_control_spp_send_data_to_host();
    }
    if ( alloc_buff )
    {
        //Disable the flow if less than half buffers available
        //Timer is running when the flow is disbled
        if ( !wiced_is_timer_in_use( &rfcomm_flow_control_timer ) )
        {
            if ( wiced_transport_get_buffer_count( rfcomm_trans_pool ) < (SPP_TRANS_MAX_BUFFERS/2) )
            {
                wiced_bt_rfcomm_flow_control ( port_handle, WICED_FALSE );
                wiced_start_timer( &rfcomm_flow_control_timer, 500 );
                WICED_BT_TRACE( " RFCOMM flow control enabled \n");
            }
        }
        if ( len + 2 <= buffer_size )
        {
            p_rfcomm_trans_buf = (uint8_t*)wiced_transport_allocate_buffer( rfcomm_trans_pool );
            if ( p_rfcomm_trans_buf )
            {
                p_buf = p_rfcomm_trans_buf;
                *p_buf = (port_handle & 0xff);
                *( p_buf + 1 ) = ((port_handle >> 8) & 0xff);
                 memcpy( p_buf + 2, ((uint8_t*)p_data+copy_len), len );
                 rfcomm_trans_data_len = len + 2;
            }
            else
            {
                 WICED_BT_TRACE( "Buffer Not Available \n");
            }
        }
        else
        {
            WICED_BT_TRACE( "Buffer Alloc Size Not Available\n");
        }
    }

    if ( wiced_is_timer_in_use(&rfcomm_trans_data_timer) )
    {
        wiced_stop_timer( &rfcomm_trans_data_timer );
    }
    if ( WICED_SUCCESS != wiced_start_timer( &rfcomm_trans_data_timer, 500 ) )
    {
        hci_control_spp_send_data_to_host();
    }
    wiced_bt_free_buffer( p_data );

    return ( ( int )len );
}

/*
 * RFCOMM management callback
 */
static void hci_control_spp_rfcomm_control_callback( uint32_t port_status, uint16_t port_handle )
{
    hci_control_spp_session_cb_t *p_scb = &hci_control_cb.spp_scb;

    WICED_BT_TRACE( "hci_control_spp_rfcomm_control_callback : Status = %d, port: 0x%04x  SCB state: %u  Srv: 0x%04x  Conn: 0x%04x\n",
                    port_status, port_handle, p_scb->state, p_scb->rfc_serv_handle, p_scb->rfc_conn_handle );

    /* ignore close event for port handles other than connected handle */
    if ( ( port_status != WICED_BT_RFCOMM_SUCCESS ) && ( port_handle != p_scb->rfc_conn_handle ) )
    {
        WICED_BT_TRACE( "hci_control_spp_rfcomm_control_callback ignoring handle:%d", port_handle );
        return;
    }

    if ( ( port_status == WICED_BT_RFCOMM_SUCCESS ) && ( p_scb->state != HCI_CONTROL_SPP_STATE_CLOSING) )
    {
        wiced_bt_rfcomm_set_data_callback( port_handle, hci_control_spp_rfcomm_data_callback );
        wiced_bt_rfcomm_set_event_mask( port_handle, PORT_MASK_ALL );

        if ( p_scb->state == HCI_CONTROL_SPP_STATE_IDLE )
            hci_control_spp_rfcomm_acceptor_opened( p_scb );
        else
            hci_control_spp_rfcomm_opened( p_scb );
    }
    else
    {
        hci_control_spp_rfcomm_closed( p_scb );
    }
}


/*
 * Setup RFCOMM server for use by HS.
 */
void hci_control_spp_rfcomm_start_server( hci_control_spp_session_cb_t *p_scb )
{
    wiced_bt_rfcomm_result_t rfcomm_result;

    p_scb->state = HCI_CONTROL_SPP_STATE_IDLE;

    if ( !p_scb->rfc_serv_handle )
    {
        rfcomm_result = wiced_bt_rfcomm_create_connection( UUID_SERVCLASS_SERIAL_PORT,
                                                           SPP_RFCOMM_SCN, WICED_TRUE, SPP_DEVICE_MTU, bd_addr_any_spp,
                                                           &p_scb->rfc_serv_handle,
                                                           ( wiced_bt_port_mgmt_cback_t * )hci_control_spp_rfcomm_control_callback );

        WICED_BT_TRACE( "hci_control_spp_rfcomm_start_server: rfcomm_create Res: 0x%x  Port: 0x%04x\n", rfcomm_result, p_scb->rfc_serv_handle );
    }
}

/*
 * Open an RFCOMM connection to the peer device.
 */
void hci_control_spp_rfcomm_do_open( hci_control_spp_session_cb_t *p_scb )
{
    wiced_bt_rfcomm_result_t rfcomm_result;

    /* Close the server, if listening on this SCB */
    if ( p_scb->rfc_serv_handle )
    {
        wiced_bt_rfcomm_remove_connection( p_scb->rfc_serv_handle, WICED_FALSE );
        p_scb->rfc_serv_handle = 0;
    }

    rfcomm_result = wiced_bt_rfcomm_create_connection( UUID_SERVCLASS_SERIAL_PORT,
                                                      p_scb->server_scn, WICED_FALSE,
                                                      HFP_DEVICE_MTU,
                                                      p_scb->server_addr,
                                                      &p_scb->rfc_conn_handle,
                                                      ( wiced_bt_port_mgmt_cback_t * )hci_control_spp_rfcomm_control_callback );

    WICED_BT_TRACE( "hci_control_spp_rfcomm_do_open - rfcomm_create Res: 0x%x   Port: 0x%04x\n", rfcomm_result, p_scb->rfc_conn_handle );

    if ( rfcomm_result != WICED_BT_RFCOMM_SUCCESS )
    {
        /* TBD Pass back that the connection attempt failed */

        hci_control_spp_rfcomm_start_server( p_scb );
    }

}

/*
 * Close RFCOMM connection.
 */
void hci_control_spp_rfcomm_do_close( hci_control_spp_session_cb_t *p_scb )
{
    wiced_bt_rfcomm_result_t rfcomm_result;

    if ( p_scb->rfc_conn_handle )
    {
        // Close connection keeping server listening
        p_scb->state = HCI_CONTROL_SPP_STATE_CLOSING;
        rfcomm_result = wiced_bt_rfcomm_remove_connection( p_scb->rfc_conn_handle, WICED_FALSE );

        WICED_BT_TRACE( "wiced_bt_rfcomm_remove_connection (0x%04x) result 0x%x\n", p_scb->rfc_conn_handle, rfcomm_result );
    }
    else
    {
        WICED_BT_TRACE( "wiced_bt_rfcomm_remove_connection - conn_handle zero\n");
        hci_control_spp_rfcomm_start_server (p_scb);
    }
}

/*
 * RFCOMM connection closed.
 */
void hci_control_spp_rfcomm_closed( hci_control_spp_session_cb_t *p_scb )
{
    /* call appropriate close cback */
     if ( ( p_scb->state == HCI_CONTROL_SPP_STATE_OPENING ) || ( !p_scb->b_is_initiator && ( p_scb->state == HCI_CONTROL_SPP_STATE_IDLE ) ) )
        hci_control_spp_send_event( p_scb, HCI_CONTROL_SPP_EVENT_CONNECTION_FAILED );
    else
        hci_control_spp_send_event( p_scb, HCI_CONTROL_SPP_EVENT_DISCONNECTED );

    /* Clear peer bd_addr */
    memset( p_scb->server_addr, 0, BD_ADDR_LEN );

    p_scb->rfc_conn_handle = 0;

    /* Reopen server if needed */
    hci_control_spp_rfcomm_start_server (p_scb);
}

/*
 * Handle RFCOMM channel opened.
 */
void hci_control_spp_rfcomm_opened( hci_control_spp_session_cb_t *p_scb )
{
    p_scb->state         = HCI_CONTROL_SPP_STATE_OPEN;
    p_scb->pending_bytes = 0;

    hci_control_spp_send_event( p_scb, HCI_CONTROL_SPP_EVENT_CONNECTED );

    wiced_bt_rfcomm_set_event_callback( p_scb->rfc_conn_handle, hci_control_port_event_cback );

    WICED_BT_TRACE( "RFCOMM Connected  isInit: %u  Serv: 0x%04x   Conn: 0x%04x\n",
                    p_scb->b_is_initiator, p_scb->rfc_serv_handle, p_scb->rfc_conn_handle );
}

/*
 * Handle RFCOMM channel opened when accepting connection.
 */
void hci_control_spp_rfcomm_acceptor_opened( hci_control_spp_session_cb_t *p_scb )
{
    uint16_t lcid;
    int      status;

    /* set role and connection handle */
    p_scb->b_is_initiator  = WICED_FALSE;
    p_scb->rfc_conn_handle = p_scb->rfc_serv_handle;

    /* get bd addr of peer */
    if ( WICED_BT_RFCOMM_SUCCESS != ( status = wiced_bt_rfcomm_check_connection( p_scb->rfc_conn_handle, p_scb->server_addr, &lcid ) ) )
    {
        WICED_BT_TRACE( "hci_control_spp_rfcomm_acceptor_opened error PORT_CheckConnection returned status %d\n", status );
        hci_control_spp_rfcomm_closed( p_scb );
    }
    else
    {
        /* continue with common open processing */
        hci_control_spp_rfcomm_opened( p_scb );
    }
}


/*
 * SDP callback function.
 */
static void hci_control_spp_sdp_cback( uint16_t sdp_status )
{
    hci_control_spp_session_cb_t  *p_scb = &hci_control_cb.spp_scb;

    WICED_BT_TRACE( "hci_control_spp_sdp_cback status:0x%x\n", sdp_status );

    if ( ( sdp_status == WICED_BT_SDP_SUCCESS ) || ( sdp_status == WICED_BT_SDP_DB_FULL ) )
    {
        if ( hci_control_spp_sdp_find_attr( p_scb ) )
        {
            hci_control_spp_rfcomm_do_open( p_scb );
        }
        else
        {
            /* reopen server and notify app of the failure */
            p_scb->state = HCI_CONTROL_SPP_STATE_IDLE;
            hci_control_spp_rfcomm_start_server( p_scb );
            hci_control_spp_send_event( p_scb, HCI_CONTROL_SPP_EVENT_SERVICE_NOT_FOUND );
        }
    }
    else
    {
        p_scb->state = HCI_CONTROL_SPP_STATE_IDLE;
        hci_control_spp_rfcomm_start_server( p_scb );
        hci_control_spp_send_event( p_scb, HCI_CONTROL_SPP_EVENT_CONNECTION_FAILED );
    }
    hci_control_spp_sdp_free_db( p_scb );
}

/*
 * Process SDP discovery results to find requested attributes for requested service.
 * Returns TRUE if results found, FALSE otherwise.
 */
BOOLEAN hci_control_spp_sdp_find_attr( hci_control_spp_session_cb_t *p_scb )
{
    wiced_bt_sdp_discovery_record_t     *p_rec = ( wiced_bt_sdp_discovery_record_t * ) NULL;
    wiced_bt_sdp_protocol_elem_t        pe;
    BOOLEAN                             result = WICED_TRUE;

    WICED_BT_TRACE( "Looking for SPP service" );

    p_rec = wiced_bt_sdp_find_service_uuid_in_db( p_scb->p_sdp_discovery_db, &spp_uuid, p_rec );
    if ( p_rec == NULL )
    {
        WICED_BT_TRACE( "hci_control_spp_sdp_find_attr( ) - could not find SPP service" );
        return ( WICED_FALSE );
    }

    /*** Look up the server channel number in the protocol list element ***/
    if ( wiced_bt_sdp_find_protocol_list_elem_in_rec( p_rec, UUID_PROTOCOL_RFCOMM, &pe ) )
    {
        WICED_BT_TRACE( "hci_control_spp_sdp_find_attr - num of proto elements -RFCOMM =0x%x\n",  pe.num_params );
        if ( pe.num_params > 0 )
        {
            p_scb->server_scn = ( uint8_t )pe.params[0];
            WICED_BT_TRACE( "hci_control_spp_sdp_find_attr - found SCN in SDP record. SCN=0x%x\n", p_scb->server_scn );
        }
        else
            result = WICED_FALSE;
    }
    else
    {
        result = WICED_FALSE;
    }
    return result;
}


/*
 * Do service discovery.
 */
void hci_control_spp_sdp_start_discovery( hci_control_spp_session_cb_t *p_scb )
{
    uint16_t        attr_list[4];
    uint8_t         num_attr;

    /* We need to get Service Class (to compare UUID and Protocol Description to get SCN to connect */
    attr_list[0] = ATTR_ID_SERVICE_CLASS_ID_LIST;
    attr_list[1] = ATTR_ID_PROTOCOL_DESC_LIST;
    num_attr = 2;

    /* allocate buffer for sdp database */
    p_scb->p_sdp_discovery_db = ( wiced_bt_sdp_discovery_db_t * ) wiced_bt_get_buffer( WICED_BUFF_MAX_SIZE );

    /* set up service discovery database; attr happens to be attr_list len */
    wiced_bt_sdp_init_discovery_db( p_scb->p_sdp_discovery_db, WICED_BUFF_MAX_SIZE, 1, &spp_uuid, num_attr, attr_list );

    /* initiate service discovery */
    if ( !wiced_bt_sdp_service_search_attribute_request( p_scb->server_addr, p_scb->p_sdp_discovery_db, &hci_control_spp_sdp_cback ) )
    {
        /* Service discovery not initiated - free discover db, reopen server, tell app  */
        hci_control_spp_sdp_free_db( p_scb );

        hci_control_spp_rfcomm_start_server( p_scb );
        hci_control_spp_send_event( p_scb, HCI_CONTROL_SPP_EVENT_CONNECTION_FAILED );
    }
}

/*
 * Free discovery database.
 */
void hci_control_spp_sdp_free_db( hci_control_spp_session_cb_t *p_scb )
{
    if ( p_scb->p_sdp_discovery_db != NULL )
    {
        wiced_bt_free_buffer( p_scb->p_sdp_discovery_db );
        p_scb->p_sdp_discovery_db = NULL;
    }
}

/*
 * Send a data packet to the server
 */
uint8_t hci_spp_send_tx_complete( uint16_t handle, wiced_bt_rfcomm_result_t result )
{
    uint8_t  tx_buf[5];
    uint8_t *p = tx_buf;
    uint8_t  trans_rc;

    *p++ = (handle & 0xff);
    *p++ = ((handle >> 8) & 0xff);
    *p++ = result;

    trans_rc = wiced_transport_send_data( HCI_CONTROL_SPP_EVENT_TX_COMPLETE, tx_buf, ( int )( p - tx_buf ) );
    return trans_rc;
}

void hci_spp_send(hci_control_spp_session_cb_t *p_scb, uint16_t handle, uint8_t * p_data, uint32_t length, char * from)
{
    wiced_bt_rfcomm_result_t  result;
    uint16_t                  bytes_written = 0;
    wiced_bool_t is_an_error ;

    result = hci_spp_write( handle, p_data, length, &bytes_written );

    /* CMD_PENDING is not an error. It means that the buffer has been enqueued */
    if ( result == WICED_BT_RFCOMM_CMD_PENDING )
       result = WICED_BT_RFCOMM_SUCCESS;

    is_an_error = ! (( result == WICED_BT_RFCOMM_SUCCESS ) || ( result == WICED_BT_RFCOMM_NO_MEM ));

    if( ( bytes_written != length ) && ( !is_an_error ) )
    {
        hci_control_spp_session_cb_t *p_scb        = &hci_control_cb.spp_scb;
        uint16_t                     bytes_to_copy = length - bytes_written;

        p_scb->pending_bytes = bytes_to_copy;
        memmove( p_scb->pending_buffer, p_data + bytes_written, bytes_to_copy );
    }
    else
    {
        p_scb->pending_bytes = 0;
        hci_spp_send_tx_complete( handle, result );
    }
}

/*
 * Send a data packet to the server
 */
void hci_control_spp_send_data( uint16_t handle, uint8_t *p_data, uint32_t length )
{
    hci_control_spp_session_cb_t *p_scb = &hci_control_cb.spp_scb;

    // hci_control_spp_generate_and_send( handle);

    spp_send_data_ctr++;
    if ( length > HCI_CONTROL_SPP_MAX_TX_BUFFER )
    {
        hci_spp_send_tx_complete( handle, WICED_BT_RFCOMM_NO_MEM );
        WICED_BT_TRACE( "%d. SPP Send handle:%d %d too many bytes \n", spp_send_data_ctr, handle, length);
        return;
    }

    if ( p_scb->pending_bytes )
    {
        WICED_BT_TRACE("\t\t %d PENDING BYTES : %d\n", spp_send_data_ctr, p_scb->pending_bytes);
    }

    hci_spp_send(p_scb, handle, (uint8_t *)p_data, length, "send_data");
}


/*
 * Process RFCOMM events
 */
void hci_control_port_event_cback( wiced_bt_rfcomm_port_event_t event, uint16_t handle )
{
    hci_control_spp_session_cb_t *p_scb = &hci_control_cb.spp_scb;

    if ( ( ( event & PORT_EV_TXCHAR ) || ( event & PORT_EV_FCS ) ) && ( p_scb->pending_bytes != 0 ) )
    {
        hci_spp_send(p_scb, handle, p_scb->pending_buffer, p_scb->pending_bytes, "cback");
    }
}

/*
 * Send SPP event over UART
 */
void hci_control_spp_send_event( hci_control_spp_session_cb_t *p_scb, uint16_t event )
{
    uint8_t   tx_buf[10];
    uint8_t  *p = tx_buf;
    int       i;
    uint16_t  handle = p_scb->b_is_initiator ? p_scb->rfc_conn_handle : p_scb->rfc_serv_handle;

    WICED_BT_TRACE( "hci_control_spp_send_event: Sending Event:%x to UART\n", event );

    if ( event == HCI_CONTROL_SPP_EVENT_CONNECTED )
    {
        for ( i = 0; i < BD_ADDR_LEN; i++)
            *p++ = p_scb->server_addr[BD_ADDR_LEN - 1 - i];
    }

    *p++ = (handle & 0xff);
    *p++ = ((handle >> 8) & 0xff);

    wiced_transport_send_data( event, tx_buf, ( int )( p - tx_buf ) );
}


void hci_control_spp_rfcomm_trans_data_timer( uint32_t param )
{
    WICED_BT_TRACE("Timer Expired \n");
    hci_control_spp_send_data_to_host();
}

void hci_control_spp_rfcomm_flow_control_timer( uint32_t param )
{
    hci_control_spp_session_cb_t *p_scb = &hci_control_cb.spp_scb;
    uint16_t                     handle = p_scb->b_is_initiator ? p_scb->rfc_conn_handle : p_scb->rfc_serv_handle;

    WICED_BT_TRACE("Flow Timer Expired %x \n", handle );
    //Disable the flow control if 3/4 buffers available
    if ( wiced_transport_get_buffer_count( rfcomm_trans_pool ) > ( (SPP_TRANS_MAX_BUFFERS * 3)/4 ) )
    {
        wiced_bt_rfcomm_flow_control ( handle, WICED_TRUE );
        WICED_BT_TRACE( " RFCOMM flow control disabled \n");
    }
    else
    {
        wiced_start_timer( &rfcomm_flow_control_timer, 500 );
    }
}
