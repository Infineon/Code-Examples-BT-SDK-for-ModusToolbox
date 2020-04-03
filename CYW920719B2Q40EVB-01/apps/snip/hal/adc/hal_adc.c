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
 * WICED ADC sample application.
 *
 * This application demonstrates how to configure and use
 * ADC in WICED Eval. boards to measure DC voltage on
 * various DC input channels.
 *
 * Features demonstrated
 * - ADC WICED API's
 *
 * Application Instructions
 * - Connect a PC terminal to the serial port of the WICED Eval board,
 *   then build and download the application as described in the WICED
 *   Quick Start Guide.
 *
 * - Every 5 seconds voltage is measured on 2 channels
 *      1) Vdd Core
 *      2) selected GPIO pin (CHANNEL_TO_MEAS_DC_VOLT)
 *
 * When the selected GPIO pin is connected to ground, the output on the terminal emulator will show 0 to ~2mV.
 * When the selected GPIO pin is connected to 3.3V, the output on the terminal emulator will show approximately 3300mV +/- (3% of 3300mV).
 * When the selected GPIO pin is left unconnected, the output will capture environmental noise resulting in some lower voltage levels in the terminal emulator.
 *
 * Please refer to the datasheet for more electrical specifications of ADC like Full scale voltage, Bandgap reference.,etc.
 *
 * Usage
 * - Follow the prompts printed on the terminal
 */

#include "wiced_bt_dev.h"
#include "sparcommon.h"
#include "wiced_hal_adc.h"
#include "wiced_gki.h"
#include "wiced_timer.h"

#include "wiced_bt_cfg.h"

#include "wiced_bt_trace.h"
#include "wiced_platform.h"
#include "wiced_bt_stack.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define APP_TIMEOUT_IN_SECONDS        5                   /* Seconds timer (Timeout in seconds) */
#define CHANNEL_TO_MEASURE_VDD_CORE   ADC_INPUT_VDD_CORE  /* Input channel to measure DC voltage */
#define CHANNEL_TO_MEASURE_DC_VOLT    ADC_INPUT_P28       /* Input channel to measure DC voltage */

/******************************************************************************
 *                                Structures
 ******************************************************************************/
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
wiced_timer_t seconds_timer; /* wiced BT app. seconds timer instance */

/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/
wiced_result_t sample_adc_app_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
void application_start( void )
{

#ifdef WICED_BT_TRACE_ENABLE
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

     WICED_BT_TRACE( "\n\r--------------------------------------------------------- \r\n\n"
                     "              ADC Sample Application \n\r\n\r"
                     "---------------------------------------------------------\n\r"
                     "This application measures voltage on the selected DC \r\n"
                     "channel every 5 seconds(configurable) and displays both \r\n"
                     "the raw and converted voltage values via chosen UART. \n\r"
                     "---------------------------------------------------------\n\n\r" );

     wiced_bt_stack_init( sample_adc_app_management_cback,
        &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);
}

 /* The function invoked on timeout of app. seconds timer. */
 void seconds_app_timer_cb( uint32_t arg )
 {
     uint32_t voltage_val = 0;
     uint32_t raw_val = 0;

     /* measure the voltage(raw and actual values) on the channel being passed as an argument */
#if !defined(CYW20721B2) && !defined(CYW20819) && !defined(CYW20719B2)
     raw_val = wiced_hal_adc_read_raw_sample( CHANNEL_TO_MEASURE_VDD_CORE );
     WICED_BT_TRACE("Raw value of VDD_CORE \t\t: %d\t\t", raw_val);
     raw_val = wiced_hal_adc_read_raw_sample( CHANNEL_TO_MEASURE_DC_VOLT );
     WICED_BT_TRACE("Raw value of ADC Channel %d \t: %d\t\t", CHANNEL_TO_MEASURE_DC_VOLT, raw_val);
#endif
     voltage_val = wiced_hal_adc_read_voltage( CHANNEL_TO_MEASURE_VDD_CORE );
     WICED_BT_TRACE("Voltage value of VDD_CORE(in mV)\t: %d\r\n", voltage_val);
     voltage_val = wiced_hal_adc_read_voltage( CHANNEL_TO_MEASURE_DC_VOLT );
     WICED_BT_TRACE("Voltage value of ADC Channel %d(in mV)  : %d\r\n\n", CHANNEL_TO_MEASURE_DC_VOLT, voltage_val);
 }

 /* This function is invoked after the controller's initialization is complete. */
 wiced_result_t sample_adc_app_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
 {
     wiced_result_t result = WICED_SUCCESS;

     WICED_BT_TRACE("Received Event : %d\n\n\r", event);

     switch ( event )
     {

     /* Bluetooth  stack enabled */
     case BTM_ENABLED_EVT:

         /* Initialize the necessary peripherals (ADC) */
         wiced_hal_adc_init( );

         /* Configure seconds periodic timer and start timer with APP_TIMEOUT_IN_SECONDS*/
         if ( WICED_SUCCESS == wiced_init_timer( &seconds_timer, &seconds_app_timer_cb, 0, WICED_SECONDS_PERIODIC_TIMER ) )
         {
             if ( wiced_start_timer( &seconds_timer, APP_TIMEOUT_IN_SECONDS ) != WICED_SUCCESS )
             {
                 WICED_BT_TRACE( "Seconds Timer Error\n\r" );
             }
         }
         break;

     default:
         WICED_BT_TRACE("Unknown Event :( \r\n");
         break;
     }

     return result;
 }
