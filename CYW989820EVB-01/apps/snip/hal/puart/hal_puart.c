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
 * WICED sample application for PUART usage
 *
 * This application demonstrates how to use WICED PUART driver interface
 * to send and receive bytes or a stream of bytes over the UART hardware.
 *
 * Features demonstrated
 * - PUART WICED APIs
 *
 * Application Instructions
 *   Connect a PC terminal to the serial port of the WICED Eval board,
 *   then build and download the application as described in the WICED
 *   Quick Start Guide
 *
 * Usage
 * Follow the prompts printed on the terminal
 */

#include "wiced_bt_dev.h"
#include "sparcommon.h"
#include "wiced_hal_puart.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_stack.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/

/******************************************************************************
 *                                Structures
 ******************************************************************************/

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
 static wiced_result_t  sample_puart_app_management_cback( wiced_bt_management_evt_t event,
                                                           wiced_bt_management_evt_data_t *p_event_data);
 static void            test_puart_driver( void );
 /******************************************************************************
 *                                Function Definitions
 ******************************************************************************/
/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
APPLICATION_START( )
{
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_NONE );

    wiced_bt_stack_init( sample_puart_app_management_cback, NULL, NULL );
}

wiced_result_t sample_puart_app_management_cback( wiced_bt_management_evt_t event,
                                                  wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_SUCCESS;

    WICED_BT_TRACE("sample_puart_app_management_cback %d", event);

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            test_puart_driver( );
            break;
        default:
            break;
    }
    return result;
}

void puar_rx_interrupt_callback(void* unused)
{
    // There can be at most 16 bytes in the HW FIFO.
    uint8_t  readbyte;

    wiced_hal_puart_read( &readbyte );

    /* send one byte via the TX line. */
    wiced_hal_puart_write( readbyte+1 );

    if( readbyte == 'S' )
    {
        /* send a string of characters via the TX line */
        wiced_hal_puart_print( "\nYou typed 'S'.\n" );
    }
    wiced_hal_puart_reset_puart_interrupt( );
}

/* Sample code to test puart driver. Initialises puart, selects puart pads,
 * turn off flow control, and enables Tx and Rx.
 * Echoes the input byte with increment by 1.
 */
void test_puart_driver( void )
{
    uint8_t read_5_bytes[5];

    wiced_hal_puart_init( );

    // Possible uart tx and rx combination.
    // Pin for Rx: p2, Pin for Tx: p0
    // Note that p2 and p0 might not be avaliable for use on your
    // specific hardware platform.
    // Please see the User Documentation to reference the valid pins.
#if defined(CYW20706A2)
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif

    /* Turn off flow control */
    wiced_hal_puart_flow_off( );  // call wiced_hal_puart_flow_on(); to turn on flow control

    // BEGIN - puart interrupt
    wiced_hal_puart_register_interrupt(puar_rx_interrupt_callback);

#if !defined(CYW20706A2)
    //set watermak level to 1 to receive interrupt up on receiving each byte
    wiced_hal_puart_set_watermark_level(1);
#endif

    /* Turn on Tx */
    wiced_hal_puart_enable_tx( ); // call wiced_hal_puart_disable_tx to disable transmit capability.
    wiced_hal_puart_print( "Hello World!\r\nType something! Keystrokes are echoed to the terminal ...\r\n");

    /* Enable to change puart baud rate. eg: 9600, 19200, 38200 */
    wiced_hal_puart_set_baudrate( 115200 );
}
