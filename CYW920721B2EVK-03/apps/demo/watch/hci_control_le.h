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
 * This file implement BTLE application controlled over UART.
 * The GATT database is defined in this file and is not changed by the MCU.
 *
 */


#ifndef _HCI_CONTROL_LE_H_
#define _HCI_CONTROL_LE_H_

#include "wiced_app.h"

void hci_control_le_init( void );
void hci_control_le_enable( void );

extern void le_slave_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status);
extern void le_slave_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status);
extern wiced_bt_gatt_status_t le_slave_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data);
extern wiced_bt_gatt_status_t le_slave_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_data);
extern wiced_bt_gatt_status_t le_slave_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data);
extern uint32_t wiced_bt_ble_get_available_tx_buffers( void );


#endif /* _HCI_CONTROL_LE_H_ */
