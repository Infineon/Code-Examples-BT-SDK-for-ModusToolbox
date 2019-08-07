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

#include "wiced_bt_types.h"
#include "wiced_platform.h"
#ifndef CYW20706A2
#include "wiced_bt_dev.h"
#endif
#include "wiced_transport.h"
#include "wiced_bt_trace.h"
#include "pbc.h"
#include "wiced_memory.h"
#include "wiced_hal_wdog.h"
#include "string.h"

#define BCM920706 20706
#define BCM920707 20707

#define WICED_BT_PBC_MAX_ENTRIES    100
#define WICED_BT_PBC_START_OFFSET   0

void hci_control_send_command_status_evt( uint16_t code, uint8_t status );
void hci_control_device_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len );
void hci_control_pbc_handle_command(uint16_t opcode, uint8_t* p_data, uint32_t length);
void hci_control_pbc_at_command (uint16_t handle, uint8_t command, int num, uint8_t* p_data);
void hci_control_misc_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len );
void hci_control_misc_handle_get_version( void );
void hci_control_send_device_started_evt( void );
void hci_control_send_pairing_completed_evt( uint8_t status , wiced_bt_device_address_t bdaddr );

extern wiced_bt_buffer_pool_t* p_key_info_pool;//Pool for storing the  key info

/*
 * handle reset command from UART
 */
void hci_control_handle_reset_cmd( void )
{
    // trip watch dog now.
    wiced_hal_wdog_reset_system( );
}


/*
 *  Pass protocol traces up through the UART
 */
void hci_control_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    //Enable below to receive traces over HCI UART send the trace
    wiced_transport_send_hci_trace( NULL, type, length, p_data  );
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
    STREAM_TO_BDADDR( bd_addr,p_bda);
    wiced_bt_set_local_bdaddr( bd_addr, BLE_ADDR_PUBLIC );

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
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
        while (p_eir_data && (( len = *p_eir_data ) != 0) )
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
 *  Handle Set Pairability command received over UART
 */
void hci_control_handle_set_pairability ( uint8_t allowed )
{
    uint8_t                   status = HCI_CONTROL_STATUS_SUCCESS;
    void                      *p1;

    if ( pairing_allowed != allowed )
    {
        if ( allowed )
        {
            // Check if key buffer pool has buffer available. If not, cannot enable pairing until nvram entries are deleted
            if ( ( p1 = ( void * )wiced_bt_get_buffer_from_pool( p_key_info_pool ) ) == NULL)
            {
                allowed = 0; //The key buffer pool is full therefore we cannot allow pairing to be enabled
                status = HCI_CONTROL_STATUS_OUT_OF_MEMORY;
            }
            else
            {
                wiced_bt_free_buffer( p1 );
            }
        }

        pairing_allowed = allowed;
        wiced_bt_set_pairable_mode( pairing_allowed, 0 );
        WICED_BT_TRACE( " Set the pairing allowed to %d \n", pairing_allowed );
    }

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, status );
}

/*
 *  Send Device Started event through UART
 */
void hci_control_send_device_started_evt( void )
{
    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0 );
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
 * Handle received command over UART.
 */
uint32_t hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t* p_rx_buf = p_data;

    WICED_BT_TRACE( "hci_control_proc_rx_cmd:%d\n", length );

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


    STREAM_TO_UINT16(opcode, p_data);     // Get opcode
    STREAM_TO_UINT16(payload_len, p_data); // Get len

    WICED_BT_TRACE("cmd_opcode 0x%02x\n", opcode);

    switch((opcode >> 8) & 0xff)
    {
    case HCI_CONTROL_GROUP_DEVICE:
        hci_control_device_handle_command( opcode, p_data, payload_len );
        break;

    case HCI_CONTROL_GROUP_PBC:
        hci_control_pbc_handle_command ( opcode, p_data, payload_len );
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
        WICED_BT_TRACE( "NVRAM write: %d\n", bytes_written );
        break;

    case HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA:
        hci_control_delete_nvram( p_data[0] | ( p_data[1] << 8 ) ,WICED_TRUE);
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

void wiced_bt_pbc_open_hdlr( wiced_bt_device_address_t bd_address );
/*
 * Handle PBAP Client commands received over UART.
 */
void hci_control_pbc_handle_command(uint16_t opcode, uint8_t* p_data, uint32_t length)
{
    uint16_t                  handle;
    uint8_t                   hs_cmd;
    int                       num;
    uint8_t                  *p = (uint8_t *)p_data;
    wiced_bt_device_address_t bd_addr;

    switch (opcode)
    {
    case HCI_CONTROL_PBC_COMMAND_CONNECT:
        STREAM_TO_BDADDR(bd_addr,p);
        wiced_bt_pbc_open_hdlr( bd_addr);
    break;

    case HCI_CONTROL_PBC_COMMAND_DISCONNECT:
        wiced_bt_pbc_close_hdlr();
    break;

    case HCI_CONTROL_PBC_COMMAND_GET_PHONEBOOK:
    case HCI_CONTROL_PBC_COMMAND_GET_CALL_HISTORY:
    case HCI_CONTROL_PBC_COMMAND_GET_INCOMMING_CALLS:
    case HCI_CONTROL_PBC_COMMAND_GET_OUTGOING_CALLS:
    case HCI_CONTROL_PBC_COMMAND_GET_MISSED_CALLS:

        wiced_bt_pbc_get_phonebook(opcode, WICED_BT_PBC_MAX_ENTRIES, WICED_BT_PBC_START_OFFSET, FALSE);

    break;

    case HCI_CONTROL_PBC_COMMAND_ABORT:
        wiced_bt_pbc_abort_hdlr();
        break;

    default:
        break;
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
    tx_buf[cmd++] = HCI_CONTROL_GROUP_PBC;

    wiced_transport_send_data( HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd );
}
