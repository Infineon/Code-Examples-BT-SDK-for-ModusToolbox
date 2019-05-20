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
 *  spi_slave_thermistor.c
 *
 *  @brief
 *  SPI Slave snippet
 *
 * This application demonstrates how to use SPI driver interface
 * to send and receive bytes or a stream of bytes over the SPI hardware as a slave.
 *
 * Features demonstrated:
 * - SPI WICED APIs
 * - ADC sampling the analog temperature values from the on-board thermistor
 *
 * Requirements and Usage:
 *
 * Program 1 kit with the dual_spi_master app and another kit with
 * spi_slave_sensor app. Connect the SPI lines and ground on the 2 kits.
 * Power on the kits. You can see the logs through PUART.
 *
 * Hardware Connections:
 *
 * This snip example configures the following SPI functionalities in
 * CYW920819EVB-02 Evaluation board as follows:
 *
 * CLK     WICED_P09    D13
 * MISO    WICED_P17    D12
 * MOSI    WICED_P14    D8
 * CS      WICED_P15    D10
 * GND
 *
 */

/******************************************************************************
 *                                Includes
 ******************************************************************************/
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_gpio.h"
#include "wiced_platform.h"
#include "wiced_hal_pspi.h"
#include "wiced_hal_puart.h"
#include "wiced_timer.h"
#include "wiced_rtos.h"
#include "thermistor_temp_db.h"
#include "wiced_hal_adc.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/

/* Using SPI instance to replicate SPI Slave as sensor (Look up table) */
#ifdef CYW20819A1
#define CLK                                 WICED_P09       /* D13 */
#define MISO                                WICED_P17       /* D12 */
#define MOSI                                WICED_P14       /* D8  */
#define CS                                  WICED_P15       /* D10 */

#define THERMISTOR_PIN                      ADC_INPUT_P8    /* P8 - A0 */
static char *pin_name                    =  "ADC_INPUT_P8";

#endif

/* 1.85V is used instead of 1.8V because of +/-5% inaccuracy */
#define VREF_THRESHOLD_1P85                 (1850)

/* Manufacturer ID denoting Cypress Semiconductor */
#define MANUFACTURER_ID                     (0x000A)

/* Unit ID denoting temperature is in Celsius scale */
#define UNIT_ID                             (0x000B)

#define SPI                                 (SPI1)
enum
{
    SEND_MANUFACTURER_ID    =   0x01,
    SEND_UNIT               =   0x02,
    SEND_TEMPERATURE        =   0x03
};

#define PACKET_HEADER                       (0xC819)


#define SLEEP_TIMEOUT                       (1)
#define NORM_FACTOR                         (100)
#define MAX_RETRIES                         (25)
#define RESET_COUNT                         (0)
/******************************************************************************
 *                                Structures
 ******************************************************************************/

/* pSPI data packet */
typedef struct
{
    uint16_t data ;
    uint16_t header;
}data_packet;

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
static uint32_t              spi_slave_gpio_cfg      = 0;

/******************************************************************************
 *                                Function Prototypes
 ******************************************************************************/
wiced_result_t      bt_cback(wiced_bt_management_evt_t event,
                             wiced_bt_management_evt_data_t *p_event_data );

static void         initialize_app(void);

static uint32_t     spi_slave_get_gpio_cfg(uint8_t cs,
                                           uint8_t clk,
                                           uint8_t mosi,
                                           uint8_t miso);

static int16_t      get_ambient_temperature(void);

/**************************************************************************************
 *                                Function Definitions
 *************************************************************************************/
/*******************************************************************************
* Function Name: void application_start(void)
********************************************************************************
* Summary: Initialize transport configuration and register BLE
*          management event callback.
*
* Parameters:
*   None
*
* Return:
*   None
*
********************************************************************************/

void application_start(void)
{
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

    if(WICED_SUCCESS != wiced_bt_stack_init( bt_cback, NULL, NULL ))
    {
        WICED_BT_TRACE("BT stack initialization failed \n\r");
    }


}

/**********************************************************************************************
* Function Name: wiced_result_t bt_cback(wiced_bt_management_evt_t event,
*                                        wiced_bt_management_evt_data_t *p_event_data)
***********************************************************************************************
* Summary:
*   This is a Bluetooth management event handler function to receive events from
*   BLE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : BLE event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to BLE management event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
***********************************************************************************************/

wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t result = WICED_SUCCESS;

    switch(event)
    {
    /* BlueTooth stack enabled */
    case BTM_ENABLED_EVT:
        WICED_BT_TRACE("\r\nSample SPI Slave Application\r\n");
        wiced_hal_adc_init();
        initialize_app();
        break;

    default:
        break;
    }
    return result;
}

/*******************************************************************************
* Function Name: void initialize_app( void )
********************************************************************************
* Summary:This functions initializes the SPI Slave
*
* Parameters:
*   None
*
* Return:
*   None
*
********************************************************************************/

void initialize_app( void )
{

    data_packet     send_data;
    data_packet     rec_data;
    uint32_t        rx_fifo_count       = 0;
    uint32_t        tx_fifo_count       = 0;
    uint8_t         retries             = RESET_COUNT;
    int16_t         thermistor_reading  = 0;

    spi_slave_gpio_cfg = spi_slave_get_gpio_cfg(CS, CLK, MOSI, MISO);

    WICED_BT_TRACE("\r\nSPI Slave GPIO Config Value: \t %x \r\n",spi_slave_gpio_cfg);

    wiced_hal_pspi_reset(SPI);

    /*Initialize SPI slave*/
    wiced_hal_pspi_init( SPI,
                         SPI_SLAVE,
                         INPUT_PIN_PULL_UP,
                         spi_slave_gpio_cfg,
                         0,
                         SPI_LSB_FIRST,
                         SPI_SS_ACTIVE_LOW,
                         SPI_MODE_0,
                         CS);

    /*Enable Tx and Rx buffers*/
    wiced_hal_pspi_slave_enable_rx(SPI);
    wiced_hal_pspi_slave_enable_tx(SPI);

    while(1)
    {
        /* Checking tx_fifo count to know if the last response was sent to
           master.*/
        tx_fifo_count = wiced_hal_pspi_slave_get_tx_fifo_count(SPI);
        if(tx_fifo_count == 0)// tell why check tx_fifo count
        {
            wiced_hal_pspi_slave_enable_rx(SPI);

            /*Check for number of bytes received*/
            rx_fifo_count = wiced_hal_pspi_slave_get_rx_fifo_count(SPI);

            if(sizeof(rec_data) <= rx_fifo_count)
            {
                if (SPIFFY_SUCCESS
                        != wiced_hal_pspi_slave_rx_data(SPI, sizeof(rec_data),
                                                        (uint8_t*) &rec_data))
                {
                    WICED_BT_TRACE("Receive failed\r\n");
                }
                wiced_hal_pspi_slave_disable_rx(SPI);

                if(rec_data.header == PACKET_HEADER)
                {
                    switch (rec_data.data)
                    {
                    case SEND_MANUFACTURER_ID:
                        WICED_BT_TRACE("Received Command:\t\t\t\t %x\r\n", rec_data.data);

                        /* Configuring send_data data packet to contain response
                           for command SEND_MANUFACTURER_ID*/
                        send_data.data = MANUFACTURER_ID;
                        send_data.header = PACKET_HEADER;
                        wiced_hal_pspi_slave_tx_data(SPI,
                                                     sizeof(send_data),
                                                     (uint8_t*) &send_data);
                        WICED_BT_TRACE("Sent Number:\t\t\t\t\t %x\r\n\n", send_data.data);
                        break;

                    case SEND_UNIT:
                        WICED_BT_TRACE("Received Command:\t\t\t\t %x\r\n", rec_data.data);

                        /* Configuring send_data data packet to contain response
                           for command SEND_UNIT*/
                        send_data.data = UNIT_ID;
                        send_data.header = PACKET_HEADER;
                        wiced_hal_pspi_slave_tx_data(SPI,
                                                     sizeof(send_data),
                                                     (uint8_t*) &send_data);
                        WICED_BT_TRACE("Sent Number:\t\t\t\t\t %x\r\n\n", send_data.data);
                        break;

                    case SEND_TEMPERATURE:
                        WICED_BT_TRACE("Received Command:\t\t\t\t %x\r\n", rec_data.data);

                        /* Configuring send_data data packet to contain response
                           for command SEND_TEMPERATURE*/
                        send_data.data = get_ambient_temperature();
                        send_data.header = PACKET_HEADER;
                        wiced_hal_pspi_slave_tx_data(SPI,
                                                     sizeof(send_data),
                                                     (uint8_t*) &send_data);
                        WICED_BT_TRACE("Sent Number:\t\t\t\t\t %x\r\n\n", send_data.data);
                        break;

                    default:
                        WICED_BT_TRACE("Invalid Command:\t\t\t\t %x\r\n", rec_data.data);

                    }
                }
                else
                {
                    retries++;
                    if(retries > MAX_RETRIES)
                    {

                        /* If the number of retries exceeds the maximum,
                           SPI interface is reset. This reset resolves clock
                           synchronization issues and ensures the data is
                           interpreted correctly.*/
                        retries = RESET_COUNT;
                        wiced_hal_pspi_reset(SPI);
                        wiced_hal_pspi_slave_enable_tx(SPI);
                    }
                }
            }
        }
        wiced_rtos_delay_milliseconds(SLEEP_TIMEOUT, ALLOW_THREAD_TO_SLEEP);
    }
}

/*******************************************************************************
* Function Name: uint32_t spi_slave_get_gpio_cfg(uint8_t cs, uint8_t clk, uint8_t mosi, uint8_t miso)
********************************************************************************
* Summary: Obtains the SPI slave configuration register value.
*
* Parameters:
*   uint8_t cs      Chip Select pin number
*   uint8_t clk     Clock pin number
*   uint8_t mosi    MOSI pin number
*   uint8_t miso    MISO pin number
*
* Return:
*   uint32_t        32 bit SPI configuration register value
*
********************************************************************************/
uint32_t spi_slave_get_gpio_cfg(uint8_t cs, uint8_t clk, uint8_t mosi, uint8_t miso)
{
    uint32_t spiGpioCfg = 0;

    spiGpioCfg |= (cs << 24);
    spiGpioCfg |= (clk << 16);
    spiGpioCfg |= (mosi << 8);
    spiGpioCfg |= (miso);

    return (spiGpioCfg);
}

/*******************************************************************************
* Function Name: static int16_t get_ambient_temperature(void)
********************************************************************************
* Summary: Obtains ambient temperature from thermistor.
*
* Parameters:
*   None
*
* Return:
*   int16_t         Temperature reading from thermistor.
*
********************************************************************************/
static int16_t get_ambient_temperature(void)
{
    volatile uint16_t   voltage_val_adc_in_mv   = 0;
    volatile uint16_t   vddio_mv                = 0;
    volatile int16_t    temperature             = 0;

    /*
     * Measure the voltage(in milli volts) on the channel being passed as an argument
     * To measure the voltage across the thermistor input channel - THERMISTOR_PIN
     * To measure the reference voltage (vref) - ADC_INPUT_VDDIO
     */

    /* Input channel to measure Reference voltage for Voltage divider calculation for Thermistor */
    vddio_mv = wiced_hal_adc_read_voltage(ADC_INPUT_VDDIO);
    WICED_BT_TRACE("Voltage in VDDIO channel\t\t\t%d mV\r\n", vddio_mv);

    if(vddio_mv < VREF_THRESHOLD_1P85)
    {
           wiced_hal_adc_set_input_range(ADC_RANGE_0_1P8V);
           /* Input channel to measure DC voltage(temperature)-> THERMISTOR_PIN */
           voltage_val_adc_in_mv = wiced_hal_adc_read_voltage(THERMISTOR_PIN);
    }
    else
    {
           wiced_hal_adc_set_input_range(ADC_RANGE_0_3P6V);
           /* Input channel to measure DC voltage(temperature)-> THERMISTOR_PIN */
           voltage_val_adc_in_mv = wiced_hal_adc_read_voltage(THERMISTOR_PIN);
     }

    WICED_BT_TRACE("Voltage in thermistor channel\t\t\t%d mV\r\n", voltage_val_adc_in_mv);

    /*
     * Temperature values might vary to +/-2 degree Celsius
     */
    temperature = get_temp_in_celsius(vddio_mv, voltage_val_adc_in_mv);
    WICED_BT_TRACE("Temperature (in degree Celsius) \t\t%d.%02d \r\n", (temperature / NORM_FACTOR),
                                                                         (temperature % NORM_FACTOR));

    return temperature;
}
