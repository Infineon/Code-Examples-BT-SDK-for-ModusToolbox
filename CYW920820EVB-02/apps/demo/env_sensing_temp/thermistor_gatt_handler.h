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
 * thermistor_gatt_handler.h
 *
 *  @brief
 * This file consists of the utility functions that will help debugging and developing the
 * applications easier with much more meaningful information.
 *
 */

#ifndef __THERMISTOR_GATT_HANDLER_H__
#define __THERMISTOR_GATT_HANDLER_H__

/* *******************************************************************
 *                              INCLUDES
 * *******************************************************************/
#include "wiced_bt_gatt.h"
#include "wiced_platform.h"
#include "GeneratedSource/cycfg_pins.h"

/* *******************************************************************
 *                              CONSTANTS
 * *******************************************************************/
#define CONNECTION_LED                  WICED_GET_PIN_FOR_LED(WICED_PLATFORM_LED_2)

/* *******************************************************************
 *                              VARIABLES
 * *******************************************************************/
/* A Global variable to check the status of this device if it is connected to any peer devices*/
extern uint16_t thermistor_conn_id;

/* LED configuration for the particular platform */
extern const wiced_platform_led_config_t platform_led[];

/* *******************************************************************
 *                              FUNCTION DECLARATIONS
 * *******************************************************************/
wiced_bt_gatt_status_t thermistor_write_handler(wiced_bt_gatt_write_t *p_write_req,
                                                uint16_t conn_id);

wiced_bt_gatt_status_t thermistor_read_handler(wiced_bt_gatt_read_t *p_read_req,
                                               uint16_t conn_id);

wiced_bt_gatt_status_t thermistor_indication_cfm_handler(uint16_t conn_id, uint16_t handle);

wiced_bt_gatt_status_t thermistor_connect_callback(wiced_bt_gatt_connection_status_t *p_conn_status);

wiced_bt_gatt_status_t thermistor_server_callback(uint16_t conn_id,
                                                  wiced_bt_gatt_request_type_t type,
                                                  wiced_bt_gatt_request_data_t *p_data);

wiced_bt_gatt_status_t thermistor_event_handler(wiced_bt_gatt_evt_t event,
                                                wiced_bt_gatt_event_data_t *p_event_data);

wiced_bt_gatt_status_t thermistor_get_value(uint16_t attr_handle,
                                            uint16_t conn_id,
                                            uint8_t *p_val,
                                            uint16_t len,
                                            uint16_t *p_len);

wiced_bt_gatt_status_t thermistor_set_value(uint16_t attr_handle,
                                            uint16_t conn_id,
                                            uint8_t *p_val,
                                            uint16_t len);

wiced_bt_dev_status_t set_ble_2m_phy(wiced_bt_gatt_connection_status_t *p_conn_status);

#endif      /* __THERMISTOR_GATT_HANDLER_H__ */
