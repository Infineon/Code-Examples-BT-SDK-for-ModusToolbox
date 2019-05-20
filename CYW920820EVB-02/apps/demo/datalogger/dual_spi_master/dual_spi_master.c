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

/** File name: dual_spi_master.c
 *
 * WICED sample application for SPI master running two SPI instances
 *
 * This application demonstrates how to use SPI driver interface to send and
 * receive bytes or a stream of bytes over the SPI hardware as a master and then
 * store the received data to SFLASH.
 *
 * Features demonstrated:
 * - SPI WICED APIs
 * - SFLASH WICED APIs
 * - WICED RTOS APIs
 *
 * Requirements and Usage:
 * Program 1 kit with the dual_spi_master app and another kit with
 * spi_slave_sensor app. Connect the SPI lines and ground on the 2 kits.
 * Power on the kits. You can see the logs through PUART
 *
 * Hardware Connections:
 * This snip example configures the following SPI functionalities in
 * CYW920819EVB-02 Evaluation board as follows:
 *
 * SPI sensor
 * CLK     WICED_P15    J3.8
 * MISO    WICED_P14    J3.10
 * MOSI    WICED_P13    J12.6
 * CS      WICED_P12    J12.5
 * GND
 *
 * SFLASH
 * CLK     WICED_P09    J3.5
 * MISO    WICED_P17    J3.6
 * MOSI    WICED_P06    J3.7
 * CS      WICED_P11    J2.9
 * GND
 */

/******************************************************************************
 *                                Includes
 ******************************************************************************/
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_gpio.h"
#include "wiced_platform.h"
#include "wiced_hal_pspi.h"
#include "wiced_hal_puart.h"
#include "wiced_rtos.h"
#include "wiced_bt_stack.h"
#include "wiced_rtc.h"
#include "wiced_hal_sflash.h"
#include "wiced_hal_nvram.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
/* Threads defines */
/* Sensible stack size for most threads*/
#define THREAD_STACK_MIN_SIZE                 (1024)
/* Defining thread priority levels*/
#define PRIORITY_HIGH                         (3)
#define PRIORITY_MEDIUM                       (5)
#define PRIORITY_LOW                          (7)

/* Queue defines*/
/* Message size corresponds to size of the temperature records being stored*/
#define MESSAGE_SIZE                          (16)
/* Queue length is chosen as 17 to ensure user is writing one page of size
 * 256 bytes to SFLASH. The page size corresponds to 16 temperature records.
 * The queue length is kept to accomodate one more record to prevent blocking
 * SPI sensor thread from pushing next record to queue while SFLASH is written*/
#define WATERMARK                             (16)
#define QUEUE_LENGTH                          (WATERMARK + 1)

/*SPI 1 defines*/
#define CLK_1                                 WICED_P15
#define MISO_1                                WICED_P14
#define MOSI_1                                WICED_P13
#define CS_1                                  WICED_P12
/* 1 MHz frequency*/
#define DEFAULT_FREQUENCY                     (1000000u)

/* SPI register configuration macro*/
#define GPIO_CFG(CS_1,CLK_1,MOSI_1,MISO_1)    ((((UINT32)CS_1&0xff)<<24)|((UINT32)CLK_1&0xff)<<16)|(((UINT32)MOSI_1&0xff)<<8)|((UINT32)MISO_1)

/* Header for SPI data packet ensures the SPI sensor connections are intact*/
#define PACKET_HEADER                         (0xC819)
/* Manufacturer ID denoting Cypress Semiconductor*/
#define MANUFACTURER_ID                       (0x000A)
/* Unit ID denoting temperature is in Celsius scale*/
#define UNIT_ID                               (0x000B)
/* Master interrogates sensor every 1 s for temperature reading*/
#define SLEEP_TIMEOUT                         (1000)
/* Delay between transmitting and receiving SPI messages from sensor, to prevent
 * reading earlier responses.*/
#define TX_RX_TIMEOUT                         (50)
#define RECORD_START                          (1)

#define SFLASH_START_ADDRESS                  (0x00000000)
/* Size is used to identify if Master is writing into new sector, so that an
 * erase can be performed before writing.*/
#define SECTOR_SIZE                           (4096)

/*To reset SPI master handling sensor when wrong data is sent repeatedly*/
#define MAX_RETRIES                           (5)
/* Resetting retry count variable to 0 when valid data is received.*/
#define RESET_COUNT                           (0)
/* Temperature data is received as 16 bit integer, the decimal and fractional
 * parts of temperature can be obtained from the quotient and remainder when the
 * temperature data is divided by 100*/
#define NORM_FACTOR                           (100)

/******************************************************************************
 *                                Structures
 ******************************************************************************/

/* pSPI data packet*/
typedef struct
{
    uint16_t data ;
    uint16_t header;
}data_packet;

/*Temperature record structure stored in sflash*/
typedef struct
{
    uint16_t record_no;
    uint8_t dec_temp;
    uint8_t frac_temp;
    wiced_rtc_time_t timestamp;
}temperature_record;

/* Enumeration listing SPI sensor commands
 * GET_MANUFACTURER_ID: Command to get Manufacturer ID.
 * GET_UNIT: Command to get unit scale.
 * MEASURE_TEMPERATURE: Command to get temperature reading.*/
typedef enum
{
    GET_MANUFACTURER_ID = 1,
    GET_UNIT,
    MEASURE_TEMPERATURE
}sensor_cmd;

/* Enumeration listing SPI Master states
 * SENSOR_DETECT: Master checks for presence of SLAVE by verifying received
 *                packet header and obtains response to Manufacturer ID on
 *                verification.
 * GET_UNIT: Master checks for presence of SLAVE by verifying received packet
 *           header and obtains response to Unit ID on verification.
 * GET_TEMPERATURE: Master checks for presence of SLAVE by verifying received
 *                  packet header and obtains response to Temperature reading on
 *                  verification.*/
typedef enum
{
    SENSOR_DETECT,
    READ_UNIT,
    READ_TEMPERATURE
}master_state;

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/

static wiced_thread_t       *spi_1;
static wiced_thread_t       *spi_2;
/* Temperature readings received from SPI sensor are stored into temperature
 * record which includes meta data like time stamp, record number along with
 * temperature. These temperature records are stored in a message queue to
 * ensure one page size data is available to be written to SFLASH.*/
static wiced_queue_t        *msg_q;
/* Semaphore signals the SFLASH thread to start popping data from queue once one
 * page size of data is available*/
static wiced_semaphore_t    *sem;
/* Holds the value of next temperature record number*/
static uint16_t             new_record;
/* current address being written */
static uint32_t             curr_addr = SFLASH_START_ADDRESS;
/* address where the user has to start reading from after button press*/
static uint32_t             rd_addr   = SFLASH_START_ADDRESS;

/******************************************************************************
 *                                Function Prototypes
 ******************************************************************************/

wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
void           initialize_app( void );
static void    spi_sensor_thread( uint32_t arg);
static void    sflash_thread( uint32_t arg );
static void    button_cback( void *data, uint8_t port_pin );
void           spi_sensor_utility (data_packet *send_msg, data_packet *rec_msg);

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

    switch( event )
    {
    /* BlueTooth stack enabled*/
    case BTM_ENABLED_EVT:

        WICED_BT_TRACE("\r\nSample SPI Master Application\r\n");

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
* Summary:This functions initializes the SPI,SFLASH, threads, message queue,
*         semaphore and GPIO
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/

void initialize_app( void )
{
    wiced_result_t  result;
    wiced_result_t  status;
    uint16_t        readBytes;
    uint16_t        record = 0;


    wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON_1,
                                               button_cback,
                                               NULL );
    wiced_hal_gpio_configure_pin( WICED_GPIO_PIN_BUTTON_1,
                                ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE ),
                                  GPIO_PIN_OUTPUT_HIGH );

    wiced_hal_sflash_init();
    WICED_BT_TRACE("Size of sflash %d bytes\r\n", wiced_hal_sflash_get_size());
    readBytes =  wiced_hal_read_nvram( WICED_NVRAM_VSID_START,
                                       sizeof(record),
                                       (uint8_t *)&record, &status);

    /* Checks if NVRAM read failed. On first startup since nothing is written to
     * NVRAM, NVRAM read fails. This implies no records have been stored. The
     * read value is assigned to 0 to prevent junk values from being read.
     * NVRAM read and write is used so that the stored temperature records are
     * not overwritten on reset or power down. Note, RTC value will not maintain
     * continuity for new temperature records.*/
    if(0 == readBytes)
    {
        WICED_BT_TRACE("Read from NVRAM failed\n\r",record);
        record = 0;
    }
    curr_addr = sizeof(temperature_record) * record;
    new_record = ++record;

    /* Initialize RTC. RTC by default is set to the time 00:00:00 Hrs, January 1,
       2010.*/
    wiced_rtc_init();

    wiced_hal_pspi_init(SPI1,
                        SPI_MASTER,
                        INPUT_PIN_PULL_UP,
                        GPIO_CFG(CS_1,CLK_1,MOSI_1,MISO_1),
                        DEFAULT_FREQUENCY,
                        SPI_LSB_FIRST,
                        SPI_SS_ACTIVE_LOW,
                        SPI_MODE_0,
                        CS_1);

    spi_1 = wiced_rtos_create_thread();
    if ( WICED_SUCCESS == wiced_rtos_init_thread(spi_1,
                                                 PRIORITY_MEDIUM,
                                                 "SPI 1 instance",
                                                 spi_sensor_thread,
                                                 THREAD_STACK_MIN_SIZE,
                                                 NULL ) )
    {
        WICED_BT_TRACE( "SPI Sensor thread created\n\r" );
    }
    else
    {
        WICED_BT_TRACE( "Failed to create SPI Sensor thread \n\r" );
    }

    spi_2 = wiced_rtos_create_thread();
    if(WICED_SUCCESS == wiced_rtos_init_thread(spi_2,
                                               PRIORITY_MEDIUM,
                                               "SPI 2 instance",
                                               sflash_thread,
                                               THREAD_STACK_MIN_SIZE,
                                               NULL))
    {
        WICED_BT_TRACE( "SFLASH thread created\n\r" );
    }
    else
    {
        WICED_BT_TRACE( "Failed to create SFLASH thread \n\r" );
    }

    WICED_BT_TRACE( "Press SW3 to read temperature records\n\r" );
    msg_q = wiced_rtos_create_queue();
    if(NULL == msg_q)
    {
        WICED_BT_TRACE("Queue creation error\r\n");
    }
    if(WICED_SUCCESS != wiced_rtos_init_queue(msg_q,
                                              "queue",
                                              MESSAGE_SIZE,
                                              QUEUE_LENGTH))
    {
        WICED_BT_TRACE("Queue initialization failed \r\n");
    }

    sem = wiced_rtos_create_semaphore();
    if(WICED_SUCCESS != wiced_rtos_init_semaphore(sem))
    {
        WICED_BT_TRACE("Semaphore initialization failed \r\n");
    }
}

/*******************************************************************************
* Function Name: void spi_sensor_thread(uint32_t arg )
********************************************************************************
* Summary:Starts and maintains transfer of SPI sensor data.
*         Also, once the queue is half full it signals spi_master_driver_2
*         to pop queue data using semaphore.
*
* Parameters:
*   uint32_t arg: unused argument
*
* Return:
*   None
*
*******************************************************************************/

void spi_sensor_thread(uint32_t arg )
{

    data_packet send_data;
    data_packet rec_data;
    wiced_result_t result;
    uint8_t num_retries = RESET_COUNT;
    uint32_t count;
    temperature_record sampled_temp;
    master_state curr_state = SENSOR_DETECT;

    while(WICED_TRUE)
    {
        switch(curr_state)
        {
        case SENSOR_DETECT:

            /* Configuring send_data data packet to contain command
               GET_MANUFACTURER_ID*/
            send_data.data = GET_MANUFACTURER_ID;
            send_data.header = PACKET_HEADER;

            /* This function is responsible for transmitting and receiving SPI
               data. It uses the send_data data packet, configured before, to
               transmit and stores the received data packet to rec_data*/
            spi_sensor_utility(&send_data,&rec_data);
            if(PACKET_HEADER == rec_data.header)
            {
                if(MANUFACTURER_ID == rec_data.data)
                {
                    WICED_BT_TRACE("Manufacturer: Cypress Semiconductor\n\r");
                    curr_state = READ_UNIT;
                    num_retries = RESET_COUNT;
                }
                else
                {
                    num_retries++;
                    WICED_BT_TRACE("Unknown manufacturer \n\r");
                }
            }
            else
            {
                num_retries++;
            }
            break;

        case READ_UNIT:

            /* Configuring send_data data packet to contain command GET_UNIT*/
            send_data.data = GET_UNIT;
            send_data.header = PACKET_HEADER;
            spi_sensor_utility(&send_data,&rec_data);
            if(PACKET_HEADER == rec_data.header)
            {
                if(UNIT_ID == rec_data.data)
                {
                    WICED_BT_TRACE("Unit: Celsius \n\r");
                    curr_state = READ_TEMPERATURE;
                    num_retries = RESET_COUNT;
                }
                else
                {
                    num_retries++;
                    WICED_BT_TRACE("Unknown unit \n\r");
                }
            }
            else
            {
                num_retries++;
            }
            break;

        case READ_TEMPERATURE:

            /* Configuring send_data data packet to contain command
               MEASURE_TEMPERATURE*/
            send_data.data = MEASURE_TEMPERATURE;
            send_data.header = PACKET_HEADER;
            spi_sensor_utility(&send_data,&rec_data);
            if(PACKET_HEADER == rec_data.header)
            {
                num_retries = RESET_COUNT;

                /* The temperature data received is 16 bit integer. Say if
                   temperature is 23.45 Celsius, the received temperature data
                   is 2345. So, to obtain the decimal and fractional parts, the
                   quotient and remainder are found.*/
                sampled_temp.dec_temp = rec_data.data / NORM_FACTOR;
                sampled_temp.frac_temp = rec_data.data % NORM_FACTOR;
                wiced_rtc_get_time(&(sampled_temp.timestamp));
                sampled_temp.record_no = new_record++;
                result = wiced_rtos_push_to_queue(msg_q,
                        &sampled_temp,
                        WICED_NO_WAIT );
                if(WICED_SUCCESS != result)
                {
                    WICED_BT_TRACE("Push failed \n\r");
                }

                /* Checking the queue occupancy to alert the SFLASH thread
                   when 16 temperature records are available in the queue by
                   setting semaphore.*/
                result = wiced_rtos_get_queue_occupancy(msg_q, &count);
                if(WICED_SUCCESS != result)
                {
                    WICED_BT_TRACE("Queue occupancy is unknown\n\r");
                }
                if( count >= WATERMARK)
                {
                    wiced_rtos_set_semaphore(sem);
                }
            }
            else
            {
                num_retries++;
            }
            break;

        default:
            break;
        }
        if(num_retries > MAX_RETRIES)
        {
            /* If the number of retries exceeds the maximum, the current
               state is changed to SENSOR_DETECT and the SPI interface is reset.
               This reset resolves clock synchronization issues and ensures the
               data is interpreted correctly.*/
            curr_state = SENSOR_DETECT;
            num_retries = RESET_COUNT;
            wiced_hal_pspi_reset(SPI1);
        }
        wiced_rtos_delay_milliseconds(SLEEP_TIMEOUT, ALLOW_THREAD_TO_SLEEP);
    }
}

/*******************************************************************************
* Function Name: void sflash_thread(uint32_t arg )
********************************************************************************
* Summary: Pops data from queue and writes to SFLASH.
*
* Parameters:
*   uint32_t arg: unused argument
*
* Return:
*   None
*
*******************************************************************************/

void sflash_thread(uint32_t arg )
{
    uint8_t loop_count;
    uint32_t no_of_bytes_written;
    wiced_result_t result;
    uint32_t sect_id;
    wiced_result_t status;
    uint16_t writeBytes;
    uint16_t record;
    temperature_record sampled_temperature[WATERMARK];

    while(WICED_TRUE)
    {
        /* Checking if location where data will be written to is in a new sector.
           If so, then the new sector is erased before writing. This is done to
           prevent writing wrong values to SFLASH.*/
        sect_id = curr_addr % SECTOR_SIZE;
        if(0 == sect_id)
        {
            wiced_hal_sflash_erase(curr_addr, SECTOR_SIZE );
        }

        /* The thread is in sleep state here, until the semaphore is set by the
           SPI sensor thread.*/
        wiced_rtos_get_semaphore(sem,  WICED_WAIT_FOREVER);

        /* 16 temperature records are popped from the queue and stored in an
           array of same size. Then, these records are written to SFLASH at
           location pointed by curr_addr.*/
        for(loop_count = 0; loop_count < WATERMARK ; loop_count++)
        {
            result = wiced_rtos_pop_from_queue(msg_q,
                                               &(sampled_temperature[loop_count]),
                                               WICED_WAIT_FOREVER );
        }
        if(WICED_SUCCESS == result)
        {
            no_of_bytes_written = wiced_hal_sflash_write(curr_addr,
                                                         sizeof(sampled_temperature),
                                                         (uint8_t *)sampled_temperature);
            if(sizeof(sampled_temperature) != no_of_bytes_written)
            {
                WICED_BT_TRACE("SFLASH write failed \n\r");
            }
            else
            {
                curr_addr += (sizeof(sampled_temperature));
            }
        }
        else
        {
            WICED_BT_TRACE("Failed to pop data from queue\n\r");
        }

        /* Obtaining the record number of the last record that was written in
           SFLASH. This value is then stored in NVRAM using a write operation.*/
        record = sampled_temperature[--loop_count].record_no;
        writeBytes = wiced_hal_write_nvram( WICED_NVRAM_VSID_START,
                                            sizeof(record),
                                            (uint8_t *)&record,
                                            &status);
        if(0 == writeBytes)
        {
            WICED_BT_TRACE("Write to NVRAM failed \n\r");
        }
    }
}

/*******************************************************************************
* Function Name: static void button_cback( void )
********************************************************************************
* Summary:ISR reads data from SFLASH on button press.
*
* Parameters:
*   void *data:   unused argument
*   uint32_t arg: unused argument
*
* Return:
*   None
*
*******************************************************************************/

static void button_cback( void *data, uint8_t port_pin )
{
    uint32_t no_of_bytes_read;
    char record_temperature[96];
    temperature_record stored_temperature;

    /* rd_addr contains location of the last record that was read. It is compared
       with the location that was written last, pointed by curr_addr to ensure
       that only locations that were written are read. If both locations are same
       then appropriate message is displayed on serial terminal.*/
    if(rd_addr >= curr_addr)
    {
        WICED_BT_TRACE("No temperature records to be read \n\r");
    }

    while(rd_addr < curr_addr)
    {
        no_of_bytes_read = wiced_hal_sflash_read(rd_addr,
                                                 sizeof(temperature_record),
                                                 (uint8_t*)&stored_temperature);
        if(sizeof(stored_temperature) != no_of_bytes_read)
        {
            WICED_BT_TRACE("SFLASH read failed \n\r");
        }
        else
        {
            rd_addr += sizeof(temperature_record);
            wiced_rtc_ctime(&(stored_temperature.timestamp),record_temperature);
            WICED_BT_TRACE("Record #: %d | "
                           "Time stamp: %s | "
                           "Temperature:%d.%02d \n\r",
                           stored_temperature.record_no,
                           record_temperature,
                           stored_temperature.dec_temp,
                           stored_temperature.frac_temp);

        }
    }

}

/*******************************************************************************
* Function Name: void spi_sensor_utility(data_packet *send_msg,data_packet *rec_msg)
********************************************************************************
* Summary:Utility function that performs SPI transactions with SPI sensor.
*
* Parameters:
*   data_packet *send_msg: pointer to the data packet that is sent.
*   data_packet *rec_msg: pointer to the data packet that is received.
*
* Return:
*   None
*
*******************************************************************************/

void spi_sensor_utility(data_packet *send_msg,data_packet *rec_msg)
{
    /* Chip select is set to LOW to select the slave for SPI transactions*/
    wiced_hal_gpio_set_pin_output(CS_1, GPIO_PIN_OUTPUT_LOW);

    /* Sending command to slave*/
    wiced_hal_pspi_tx_data(SPI1,
                           sizeof(*send_msg),
                           (uint8_t*)send_msg);

    /*Allowing slave time to fill its tx buffers before receiving*/
    wiced_rtos_delay_milliseconds(TX_RX_TIMEOUT,ALLOW_THREAD_TO_SLEEP);

    /* Receving response from slave*/
    wiced_hal_pspi_rx_data(SPI1,
                           sizeof(*rec_msg),
                           (uint8_t*)rec_msg);
    wiced_hal_gpio_set_pin_output(CS_1, GPIO_PIN_OUTPUT_HIGH);
    return;
}
