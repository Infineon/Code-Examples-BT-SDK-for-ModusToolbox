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

#ifndef __WICED_H
#define __WICED_H

#include <stdint.h>
#include "wiced.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_app.h"
#include "wiced_timer.h"

// Subunit initialization complete callback
typedef void (*init_complete_cback_t) (int);
typedef void (*gap_client_init_complete_cback_t) (int, uint8_t *p_name, uint8_t name_len);

// Time of the day notification
typedef void (*time_notification_cback_t)(uint8_t *time_data, int len);

// ANCS event notification
typedef void (*ancs_notification_cback_t)(void *p_ancs_event);

// AMS event notification
typedef void (*ams_notification_cback_t)(uint16_t opcode, uint8_t *p_data, uint16_t len);

// Application state
typedef struct
{
#define WATCH_INIT_STATE_NONE       0
#define WATCH_INIT_STATE_GATT       1
#define WATCH_INIT_STATE_GAP        2
#define WATCH_INIT_STATE_TIME       3
#define WATCH_INIT_STATE_FINDME     4
#define WATCH_INIT_STATE_ANCS       5
#define WATCH_INIT_STATE_AMS        6

    uint8_t      init_state;
    uint8_t      start_discovery;
    uint16_t     conn_id;
    BD_ADDR      remote_addr;   //address of currently connected client
    wiced_bt_ble_address_type_t addr_type;
    wiced_bt_transport_t transport;              /**< Transport type of the connection */
    wiced_bool_t hid_report_client_configuration;
    wiced_bool_t hid_empty_report_pending;
    wiced_timer_t timer;
    wiced_bool_t encrypted;
} watch_app_state_t;

extern watch_app_state_t watch_app_state;

void watch_util_send_discover(uint16_t conn_id, wiced_bt_gatt_discovery_type_t type, uint16_t uuid,
                              uint16_t s_handle, uint16_t e_handle);
void watch_util_send_read_by_handle(uint16_t conn_id, uint16_t handle);
wiced_bool_t watch_util_send_read_by_type(uint16_t conn_id, uint16_t s_handle, uint16_t e_handle, uint16_t uuid);

wiced_bt_gatt_status_t ancs_client_discovery_result(wiced_bt_gatt_discovery_result_t *p_data);
wiced_bt_gatt_status_t ams_client_discovery_result(wiced_bt_gatt_discovery_result_t *p_data);

wiced_bt_gatt_status_t ancs_client_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data);
wiced_bt_gatt_status_t ams_client_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data);

void ancs_client_read_rsp(wiced_bt_gatt_operation_complete_t *p_data);
void ams_client_read_rsp(wiced_bt_gatt_operation_complete_t *p_data);

void ancs_client_write_rsp(wiced_bt_gatt_operation_complete_t *p_data);
void ams_client_write_rsp(wiced_bt_gatt_operation_complete_t *p_data);

void ancs_client_notification_handler(wiced_bt_gatt_operation_complete_t *p_data);
void ams_client_notification_handler(wiced_bt_gatt_operation_complete_t *p_data);

void gatt_client_indication_handler(wiced_bt_gatt_operation_complete_t *p_data);
void time_client_indication_handler(wiced_bt_gatt_operation_complete_t *p_data);
void findme_client_indication_handler(wiced_bt_gatt_operation_complete_t *p_data);
void ancs_client_indication_handler(wiced_bt_gatt_operation_complete_t *p_data);
void ams_client_indication_handler(wiced_bt_gatt_operation_complete_t *p_data);

void ancs_perform_action( uint32_t uid, uint32_t action_id );
char *utl_strcpy( char *p_dst, char *p_src );

void le_slave_app_init(void);
void le_slave_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status);
void le_slave_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status);
void le_slave_encryption_status_changed(wiced_bt_dev_encryption_status_t *p_status);

wiced_bt_gatt_status_t le_slave_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data);
wiced_bt_gatt_status_t le_slave_gatt_discovery_result  (wiced_bt_gatt_discovery_result_t *p_data);
wiced_bt_gatt_status_t le_slave_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data);
wiced_bt_gatt_status_t le_slave_gatt_req_cb(wiced_bt_gatt_attribute_request_t *p_req);

void serial_gatt_service_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status);
void serial_gatt_service_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status);
wiced_bt_gatt_status_t serial_gatt_service_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t *p_write_req);
wiced_bt_gatt_status_t serial_gatt_service_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t *p_read_req);
wiced_bt_gatt_status_t serial_gatt_service_mtu_handler(uint16_t conn_id, uint16_t mtu);

#endif
