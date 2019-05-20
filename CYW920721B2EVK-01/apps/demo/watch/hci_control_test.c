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
 * This file implements the Test Commands controlled over UART. Please refer to the
 * WICED HCI Control Protocol Software User Manual (WICED-SWUM10x-R) for additional
 * details on the HCI UART control protocol
 */
#include "wiced_bt_cfg.h"
#include "hci_control.h"
#include "hci_control_api.h"
#include "hci_control_test.h"
#include "wiced_transport.h"

/******************************************************************************
 *                          Constants
 ******************************************************************************/
#define VENDOR_SPECIFIC_OPCODE                  0x21ff
#define CONNECTIONLESS_RX_TEST_OPCODE           0xFC52
#define RX_TEST_STATISTICS_SUBCODE              0x07

/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/
static void hci_control_send_encapsulated_hci_event( uint8_t * p_data, uint16_t length );
static void wiced_bt_send_test_command( uint16_t opcode, uint8_t* params, uint8_t params_length );

extern void btu_hcif_send_cmd (uint8_t controller_id, BT_HDR *p_buf);

/******************************************************************************
 *                          Variable Definitions
 ******************************************************************************/
hci_control_test_command_t test_command;

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
void hci_control_test_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len )
{
    switch( cmd_opcode )
    {
    case HCI_CONTROL_TEST_COMMAND_ENCAPSULATED_HCI_COMMAND:
        wiced_bt_send_test_command( ( p_data[1] | ( p_data[2] << 8 ) ), &p_data[4], p_data[3] );
        break;

    default:
        WICED_BT_TRACE( "unknown test command\n");
        break;
    }
}

/*
* transfer HCI event within a WICED HCI Event message
*/
void hci_control_send_encapsulated_hci_event( uint8_t * p_data, uint16_t length )
{
    wiced_transport_send_data( HCI_CONTROL_TEST_EVENT_ENCAPSULATED_HCI_EVENT, p_data, length );
}

void hci_control_handle_hci_test_event( uint8_t * p_data, uint16_t length )
{
    uint16_t  opcode;

    // Only send back the Command Complete HCI Event for the command opcode we sent to the controller
    opcode = p_data[3] | ( p_data[4] << 8 );
    if( opcode == test_command.opcode )
    {
        hci_control_send_encapsulated_hci_event( p_data, length );

        /* If we are enabling the Rx_Test, test execution will continue since we will need
         * to process the Rx Test Statistics event messages that we will periodically receive */
        if ( opcode != CONNECTIONLESS_RX_TEST_OPCODE )
        {
            test_command.test_executing = WICED_FALSE;
        }
    }

    /* Check for Vendor Specific opcode - Needed to send Connectionless Rx Test Statistics */
    opcode = p_data[0] | ( p_data[1] << 8 );
    if( ( opcode == VENDOR_SPECIFIC_OPCODE ) && ( p_data[2] == RX_TEST_STATISTICS_SUBCODE ) )
    {
        hci_control_send_encapsulated_hci_event( p_data, length );
    }
}

static void wiced_bt_send_test_command( uint16_t opcode, uint8_t* params, uint8_t params_length )
{
    BT_HDR          *p_command;
    uint8_t         *p;
    wiced_result_t  status;

    if ( ( opcode >> 10 ) == 0x3F ) // Vendor Specific Command
    {
        status = wiced_bt_dev_vendor_specific_command( opcode, params_length, params, NULL );
        WICED_BT_TRACE ("wiced_bt_dev_vendor_specific_command status %d \n", status);
    }
    else
    {
        if ( ( p_command = HCI_GET_CMD_BUF( params_length ) ) == NULL )
        {
            hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_FAILED );
            return;
        }

        p = ( uint8_t * )( p_command + 1 );

        p_command->event  = BT_EVT_TO_LM_HCI_CMD;
        p_command->len    = HCIC_PREAMBLE_SIZE + params_length;
        p_command->offset = 0;

        UINT16_TO_STREAM( p, opcode );
        UINT8_TO_STREAM( p, params_length );

        if ( params_length )
        {
            ARRAY_TO_STREAM( p, params, params_length );
        }

        btu_hcif_send_cmd (LOCAL_BR_EDR_CONTROLLER_ID,  p_command);
    }

    test_command.test_executing = WICED_TRUE;
    test_command.opcode         = opcode;
}
