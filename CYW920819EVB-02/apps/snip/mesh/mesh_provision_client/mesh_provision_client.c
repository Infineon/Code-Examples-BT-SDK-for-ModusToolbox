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
 * This file shows how to create a device which implements mesh provisioner client.
 * The main purpose of the app is to process messages coming from the MCU and call Mesh Core
 * Library to perform functionality.
 */
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_bt_mesh_core.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_mesh_provision.h"
#include "wiced_bt_trace.h"
#include "wiced_transport.h"
#include "hci_control_api.h"
#include "mesh_vendor_server.h"
#include "wiced_bt_mesh_client.h"
#include "wiced_memory.h"

uint32_t mesh_onoff_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
uint32_t mesh_level_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
uint32_t mesh_light_lightness_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
uint32_t mesh_light_hsl_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
uint32_t mesh_light_ctl_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
uint32_t mesh_default_transition_time_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
uint32_t mesh_sensor_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
uint32_t mesh_scene_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
uint32_t mesh_vendor_service_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
wiced_bool_t mesh_gatt_client_local_device_set(wiced_bt_mesh_local_device_set_data_t *p_data);

/******************************************************
 *          Constants
 ******************************************************/
#define MESH_PID                0x301D
#define MESH_VID                0x0001
#define MESH_FWID               0x301D000101010001
#define MESH_CACHE_REPLAY_SIZE  200

/******************************************************
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
static void mesh_app_init(wiced_bool_t is_provisioned);
static uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
static void mesh_config_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static uint8_t mesh_provisioner_process_set_local_device(uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_set_dev_key(uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_connect(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_disconnect(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_start(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_oob_value(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_search_proxy(uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_proxy_connect(uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_proxy_disconnect(uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_scan_capabilities_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_scan_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_scan_start(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_scan_stop(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_extended_scan_start(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_node_reset(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_beacon_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_beacon_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_composition_data_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_default_ttl_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_default_ttl_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_gatt_proxy_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_gatt_proxy_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_relay_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_relay_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_friend_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_friend_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_key_refresh_phase_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_key_refresh_phase_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_node_identity_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_node_identity_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_model_publication_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_model_publication_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_model_subscription_change(wiced_bt_mesh_event_t *p_event, uint8_t opcode, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_model_subscription_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_netkey_change(wiced_bt_mesh_event_t *p_event, uint8_t operation, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_netkey_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_appkey_change(wiced_bt_mesh_event_t *p_event, uint8_t operation, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_appkey_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_model_app_change(wiced_bt_mesh_event_t *p_event, uint8_t operation, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_model_app_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_heartbeat_subscription_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_heartbeat_subscription_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_heartbeat_publication_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_heartbeat_publication_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_network_transmit_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_health_fault_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_health_fault_clear(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_health_fault_test(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_health_period_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_health_period_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_health_attention_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_health_attention_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_lpn_poll_timeout_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_network_transmit_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_proxy_filter_type_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static uint8_t mesh_provisioner_process_proxy_filter_change(wiced_bt_mesh_event_t *p_event, wiced_bool_t is_add, uint8_t *p_data, uint32_t length);
static void mesh_provisioner_hci_event_provision_end_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_provision_status_data_t *p_data);
void mesh_provisioner_hci_event_scan_report_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_provision_scan_report_data_t *p_data);
void mesh_provisioner_hci_event_scan_extended_report_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_provision_scan_extended_report_data_t *p_data);
void mesh_provisioner_hci_event_proxy_device_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_proxy_device_network_data_t *p_data);
void mesh_provisioner_hci_event_provision_link_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_provision_link_status_data_t *p_data);
void mesh_provisioner_hci_event_provision_link_report_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_provision_link_report_data_t *p_data);
static void mesh_provisioner_hci_event_device_capabilities_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_provision_device_capabilities_data_t *p_data);
static void mesh_provisioner_hci_event_device_get_oob_data_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_provision_device_oob_request_data_t *p_data);
static void mesh_provisioner_hci_event_node_reset_status_send(wiced_bt_mesh_hci_event_t *p_hci_event);
static void mesh_provisioner_hci_event_node_identity_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_node_identity_status_data_t *p_data);
static void mesh_provisioner_hci_event_composition_data_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_composition_data_status_data_t *p_data);
static void mesh_provisioner_hci_event_friend_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_friend_status_data_t *p_data);
static void mesh_provisioner_hci_event_key_refresh_phase_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_key_refresh_phase_status_data_t *p_data);
static void mesh_provisioner_hci_event_default_ttl_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_default_ttl_status_data_t *p_data);
static void mesh_provisioner_hci_event_relay_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_relay_status_data_t *p_data);
static void mesh_provisioner_hci_event_gatt_proxy_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_gatt_proxy_status_data_t *p_data);
static void mesh_provisioner_hci_event_beacon_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_beacon_status_data_t *p_data);
static void mesh_provisioner_hci_event_model_publication_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_model_publication_status_data_t *p_data);
static void mesh_provisioner_hci_event_model_subscription_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_model_subscription_status_data_t *p_data);
static void mesh_provisioner_hci_event_model_subscription_list_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_model_subscription_list_data_t *p_data);
static void mesh_provisioner_hci_event_netkey_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_netkey_status_data_t *p_data);
static void mesh_provisioner_hci_event_netkey_list_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_netkey_list_data_t *p_data);
static void mesh_provisioner_hci_event_appkey_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_appkey_status_data_t *p_data);
static void mesh_provisioner_hci_event_appkey_list_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_appkey_list_data_t *p_data);
static void mesh_provisioner_hci_event_model_app_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_model_app_bind_status_data_t *p_data);
static void mesh_provisioner_hci_event_model_app_list_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_model_app_list_data_t *p_data);
static void mesh_provisioner_hci_event_hearbeat_subscription_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_heartbeat_subscription_status_data_t *p_data);
static void mesh_provisioner_hci_event_hearbeat_publication_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_heartbeat_publication_status_data_t *p_data);
static void mesh_provisioner_hci_event_network_transmit_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_network_transmit_status_data_t *p_data);
static void mesh_provisioner_hci_event_health_current_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_health_fault_status_data_t *p_data);
static void mesh_provisioner_hci_event_health_fault_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_health_fault_status_data_t *p_data);
static void mesh_provisioner_hci_event_health_period_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_health_period_status_data_t *p_data);
static void mesh_provisioner_hci_event_health_attention_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_health_attention_status_data_t *p_data);
static void mesh_provisioner_hci_event_lpn_poll_timeout_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_lpn_poll_timeout_status_data_t *p_data);
static void mesh_provisioner_hci_event_proxy_filter_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_proxy_filter_status_data_t *p_data);
static void mesh_provisioner_hci_event_scan_capabilities_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_provision_scan_capabilities_status_data_t *p_data);
static void mesh_provisioner_hci_event_scan_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_provision_scan_status_data_t *p_data);

static void mesh_provisioner_hci_send_status(uint8_t status);

/******************************************************
 *          Variables Definitions
 ******************************************************/
#ifndef WICEDX
// Use hardcoded PTS default priv key. In real app it will be generated once and written into OTP memory
uint8_t pb_priv_key[WICED_BT_MESH_PROVISION_PRIV_KEY_LEN] = { 0x52, 0x9A, 0xA0, 0x67, 0x0D, 0x72, 0xCD, 0x64, 0x97, 0x50, 0x2E, 0xD4, 0x73, 0x50, 0x2B, 0x03, 0x7E, 0x88, 0x03, 0xB5, 0xC6, 0x08, 0x29, 0xA5, 0xA3, 0xCA, 0xA2, 0x19, 0x50, 0x55, 0x30, 0xBA };
#endif

char   *mesh_dev_name                                                      = "Provisioner Client";
uint8_t mesh_appearance[WICED_BT_MESH_PROPERTY_LEN_DEVICE_APPEARANCE]      = { BIT16_TO_8(APPEARANCE_GENERIC_TAG) };
uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME] = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER]     = { '1', '2', '3', '4', 0, 0, 0, 0 };
uint8_t mesh_system_id[8]                                                  = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

extern wiced_transport_buffer_pool_t* host_trans_pool;

wiced_bool_t mesh_vendor_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);
uint16_t     mesh_vendor_server_scene_store_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len);
uint16_t     mesh_vendor_server_scene_recall_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len, uint32_t transition_time, uint32_t delay);

wiced_bt_mesh_core_config_model_t   mesh_element1_models[] =
{
    WICED_BT_MESH_DEVICE,
    WICED_BT_MESH_MODEL_CONFIG_CLIENT,
    WICED_BT_MESH_MODEL_HEALTH_CLIENT,
    WICED_BT_MESH_MODEL_REMOTE_PROVISION_SERVER,
    WICED_BT_MESH_MODEL_REMOTE_PROVISION_CLIENT,
    WICED_BT_MESH_MODEL_DEFAULT_TRANSITION_TIME_CLIENT,
    WICED_BT_MESH_MODEL_SENSOR_CLIENT,
    WICED_BT_MESH_MODEL_SCENE_CLIENT,
    WICED_BT_MESH_MODEL_ONOFF_CLIENT,
    WICED_BT_MESH_MODEL_LEVEL_CLIENT,
    WICED_BT_MESH_MODEL_LIGHT_LIGHTNESS_CLIENT,
    WICED_BT_MESH_MODEL_LIGHT_CTL_CLIENT,
    WICED_BT_MESH_MODEL_LIGHT_HSL_CLIENT,
    { MESH_VENDOR_COMPANY_ID, MESH_VENDOR_MODEL_ID, mesh_vendor_server_message_handler, mesh_vendor_server_scene_store_handler, mesh_vendor_server_scene_recall_handler },
};
#define MESH_APP_NUM_MODELS  (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t))

#define MESH_PROVISIONER_CLIENT_ELEMENT_INDEX   0

wiced_bt_mesh_core_config_element_t mesh_elements[] =
{
    {
        .location = MESH_ELEM_LOC_MAIN,                                 // location description as defined in the GATT Bluetooth Namespace Descriptors section of the Bluetooth SIG Assigned Numbers
        .default_transition_time = MESH_DEFAULT_TRANSITION_TIME_IN_MS,  // Default transition time for models of the element in milliseconds
        .onpowerup_state = WICED_BT_MESH_ON_POWER_UP_STATE_RESTORE,     // Default element behavior on power up
        .default_level = 0,                                             // Default value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_min = 1,                                                 // Minimum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_max = 0xffff,                                            // Maximum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .move_rollover = 0,                                             // If true when level gets to range_max during move operation, it switches to min, otherwise move stops.
        .properties_num = 0,                                            // Number of properties in the array models
        .properties = NULL,                                             // Array of properties in the element.
        .sensors_num = 0,                                               // Number of sensors in the sensor array
        .sensors = NULL,                                                // Array of sensors of that element
        .models_num = MESH_APP_NUM_MODELS,                              // Number of models in the array models
        .models = mesh_element1_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
};

wiced_bt_mesh_core_config_t  mesh_config =
{
    .company_id         = MESH_COMPANY_ID_CYPRESS,                  // Company identifier assigned by the Bluetooth SIG
    .product_id         = MESH_PID,                                 // Vendor-assigned product identifier
    .vendor_id          = MESH_VID,                                 // Vendor-assigned product version identifier
    .firmware_id        = MESH_FWID,                                // Vendor-assigned firmware version identifier
    .replay_cache_size  = MESH_CACHE_REPLAY_SIZE,                   // Number of replay protection entries, i.e. maximum number of mesh devices that can send application messages to this device.
    .features                  = 0,                                 // no relay, no friend, no proxy
    .friend_cfg         =                                           // Empty Configuration of the Friend Feature
    {
        .receive_window        = 0,                                 // Receive Window value in milliseconds supported by the Friend node.
        .cache_buf_len         = 0                                  // Length of the buffer for the cache
    },
    .low_power          =                                           // Configuration of the Low Power Feature
    {
        .rssi_factor           = 0,                                 // contribution of the RSSI measured by the Friend node used in Friend Offer Delay calculations.
        .receive_window_factor = 0,                                 // contribution of the supported Receive Window used in Friend Offer Delay calculations.
        .min_cache_size_log    = 0,                                 // minimum number of messages that the Friend node can store in its Friend Cache.
        .receive_delay         = 0,                                 // Receive delay in 1 ms units to be requested by the Low Power node.
        .poll_timeout          = 0                                  // Poll timeout in 100ms units to be requested by the Low Power node.
    },
    .gatt_client_only          = WICED_FALSE,                       // Can connect to mesh over GATT or ADV
    .elements_num  = (uint8_t)(sizeof(mesh_elements) / sizeof(mesh_elements[0])),   // number of elements on this device
    .elements      = mesh_elements                                  // Array of elements for this device
};

/*
 * Mesh application library will call into application functions if provided by the application.
 */
wiced_bt_mesh_app_func_table_t wiced_bt_mesh_app_func_table =
{
    mesh_app_init,          // application initialization
    NULL,                   // Default SDK platform button processing
    NULL,                   // GATT connection status
    NULL,                   // attention processing
    NULL,                   // notify period set
    mesh_app_proc_rx_cmd,   // WICED HCI command
    NULL,                   // LPN sleep
    NULL                    // factory reset
};

extern void mesh_onoff_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
extern void mesh_scene_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
extern void mesh_level_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_level_status_data_t *p_data);
extern void mesh_light_lightness_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
extern void mesh_light_ctl_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
extern void mesh_light_hsl_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
extern void mesh_default_transition_time_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
extern void mesh_sensor_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

// This application is the only one that can connect to the mesh over proxy and
// need to process proxy status messages.  It needs to fill the pointer in the mesh_application.c.
extern void *p_proxy_status_message_handler;
extern void mesh_proxy_client_process_filter_status(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);


/******************************************************
 *               Function Definitions
 ******************************************************/
void mesh_app_init(wiced_bool_t is_provisioned)
{
#if 0
    extern uint8_t wiced_bt_mesh_model_trace_enabled;
    wiced_bt_mesh_model_trace_enabled = WICED_TRUE;
#endif
#if 0
#include "fid_app.h"

    // enable core trace
    extern void wiced_bt_mesh_core_set_trace_level(uint32_t fids_mask, uint8_t level);

    wiced_bt_mesh_core_set_trace_level(0xffffffff, 4);      //(ALL, TRACE_DEBUG)
    wiced_bt_mesh_core_set_trace_level((1 << FID_MESH_APP__CORE_AES_CCM_C), 0);
//    wiced_bt_mesh_core_set_trace_level((1 << FID_MESH_APP__PROVISIONING_C), 0);
//    wiced_bt_mesh_core_set_trace_level((1 << FID_MESH_APP__PB_TRANSPORT_C), 0);
#endif

    wiced_bt_mesh_remote_provisioning_server_init();

    // for the embedded client we are already provisioned because
    // host should know everything and setup local device configuration through WICED HCI commands
    wiced_bt_mesh_provision_client_init(mesh_config_client_message_handler, is_provisioned);
    wiced_bt_mesh_client_init(mesh_config_client_message_handler, is_provisioned);

    wiced_bt_mesh_config_client_init(mesh_config_client_message_handler, is_provisioned);
    wiced_bt_mesh_health_client_init(mesh_config_client_message_handler, is_provisioned);
    wiced_bt_mesh_proxy_client_init(mesh_config_client_message_handler, is_provisioned);

    wiced_bt_mesh_model_onoff_client_init(0, mesh_onoff_client_message_handler, is_provisioned);
    wiced_bt_mesh_model_level_client_init(0, mesh_level_client_message_handler, is_provisioned);
    wiced_bt_mesh_model_light_lightness_client_init(0, mesh_light_lightness_client_message_handler, is_provisioned);
    wiced_bt_mesh_model_light_ctl_client_init(0, mesh_light_ctl_client_message_handler, is_provisioned);
    wiced_bt_mesh_model_light_hsl_client_init(0, mesh_light_hsl_client_message_handler, is_provisioned);
    wiced_bt_mesh_model_default_transition_time_client_init(0, mesh_default_transition_time_client_message_handler, is_provisioned);
    wiced_bt_mesh_model_sensor_client_init(0, mesh_sensor_client_message_handler, is_provisioned);
    wiced_bt_mesh_model_scene_client_init(0, mesh_scene_client_message_handler, is_provisioned);

    p_proxy_status_message_handler = mesh_proxy_client_process_filter_status;

}

/*
 * Process event received from the Configuration Server.
 */
void mesh_config_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
    wiced_bt_mesh_hci_event_t *p_hci_event = wiced_bt_mesh_create_hci_event(p_event);
    if (p_hci_event == NULL)
    {
        WICED_BT_TRACE("config clt no mem event:%d\n", event);
        return;
    }
    WICED_BT_TRACE("config clt msg:%d\n", event);

    switch (event)
    {
    case WICED_BT_MESH_TX_COMPLETE:
        WICED_BT_TRACE("tx complete status:%d\n", p_event->tx_status);
        wiced_bt_mesh_send_hci_tx_complete(p_hci_event, p_event);
        break;

    case WICED_BT_MESH_CONFIG_NODE_RESET_STATUS:
        mesh_provisioner_hci_event_node_reset_status_send(p_hci_event);
        break;

    case WICED_BT_MESH_CONFIG_COMPOSITION_DATA_STATUS:
        mesh_provisioner_hci_event_composition_data_status_send(p_hci_event, (wiced_bt_mesh_config_composition_data_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_CONFIG_FRIEND_STATUS:
        mesh_provisioner_hci_event_friend_status_send(p_hci_event, (wiced_bt_mesh_config_friend_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_CONFIG_GATT_PROXY_STATUS:
        mesh_provisioner_hci_event_gatt_proxy_status_send(p_hci_event, (wiced_bt_mesh_config_gatt_proxy_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_CONFIG_RELAY_STATUS:
        mesh_provisioner_hci_event_relay_status_send(p_hci_event, (wiced_bt_mesh_config_relay_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_CONFIG_BEACON_STATUS:
        mesh_provisioner_hci_event_beacon_status_send(p_hci_event, (wiced_bt_mesh_config_beacon_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_CONFIG_DEFAULT_TTL_STATUS:
        mesh_provisioner_hci_event_default_ttl_status_send(p_hci_event, (wiced_bt_mesh_config_default_ttl_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_CONFIG_NODE_IDENTITY_STATUS:
        mesh_provisioner_hci_event_node_identity_status_send(p_hci_event, (wiced_bt_mesh_config_node_identity_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_CONFIG_MODEL_PUBLICATION_STATUS:
        mesh_provisioner_hci_event_model_publication_status_send(p_hci_event, (wiced_bt_mesh_config_model_publication_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_CONFIG_MODEL_SUBSCRIPTION_STATUS:
        mesh_provisioner_hci_event_model_subscription_status_send(p_hci_event, (wiced_bt_mesh_config_model_subscription_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_CONFIG_MODEL_SUBSCRIPTION_LIST:
        mesh_provisioner_hci_event_model_subscription_list_send(p_hci_event, (wiced_bt_mesh_config_model_subscription_list_data_t *)p_data);
        break;

    case WICED_BT_MESH_CONFIG_NETKEY_STATUS:
        mesh_provisioner_hci_event_netkey_status_send(p_hci_event, (wiced_bt_mesh_config_netkey_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_CONFIG_KEY_REFRESH_PHASE_STATUS:
        mesh_provisioner_hci_event_key_refresh_phase_status_send(p_hci_event, (wiced_bt_mesh_config_key_refresh_phase_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_CONFIG_NETKEY_LIST:
        mesh_provisioner_hci_event_netkey_list_send(p_hci_event, (wiced_bt_mesh_config_netkey_list_data_t *)p_data);
        break;

    case WICED_BT_MESH_CONFIG_APPKEY_STATUS:
        mesh_provisioner_hci_event_appkey_status_send(p_hci_event, (wiced_bt_mesh_config_appkey_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_CONFIG_APPKEY_LIST:
        mesh_provisioner_hci_event_appkey_list_send(p_hci_event, (wiced_bt_mesh_config_appkey_list_data_t *)p_data);
        break;

    case WICED_BT_MESH_CONFIG_MODEL_APP_BIND_STATUS:
        mesh_provisioner_hci_event_model_app_status_send(p_hci_event, (wiced_bt_mesh_config_model_app_bind_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_CONFIG_MODEL_APP_BIND_LIST:
        mesh_provisioner_hci_event_model_app_list_send(p_hci_event, (wiced_bt_mesh_config_model_app_list_data_t *)p_data);
        break;

    case WICED_BT_MESH_CONFIG_HEARBEAT_SUBSCRIPTION_STATUS:
        mesh_provisioner_hci_event_hearbeat_subscription_status_send(p_hci_event, (wiced_bt_mesh_config_heartbeat_subscription_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_CONFIG_HEARBEAT_PUBLICATION_STATUS:
        mesh_provisioner_hci_event_hearbeat_publication_status_send(p_hci_event, (wiced_bt_mesh_config_heartbeat_publication_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_CONFIG_NETWORK_TRANSMIT_STATUS:
        mesh_provisioner_hci_event_network_transmit_status_send(p_hci_event, (wiced_bt_mesh_config_network_transmit_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_CONFIG_LPN_POLL_TIMEOUT_STATUS:
        mesh_provisioner_hci_event_lpn_poll_timeout_status_send(p_hci_event, (wiced_bt_mesh_lpn_poll_timeout_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_HEALTH_CURRENT_STATUS:
        mesh_provisioner_hci_event_health_current_status_send(p_hci_event, (wiced_bt_mesh_health_fault_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_HEALTH_FAULT_STATUS:
        mesh_provisioner_hci_event_health_fault_status_send(p_hci_event, (wiced_bt_mesh_health_fault_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_HEALTH_PERIOD_STATUS:
        mesh_provisioner_hci_event_health_period_status_send(p_hci_event, (wiced_bt_mesh_health_period_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_HEALTH_ATTENTION_STATUS:
        mesh_provisioner_hci_event_health_attention_status_send(p_hci_event, (wiced_bt_mesh_health_attention_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_PROXY_FILTER_STATUS:
        mesh_provisioner_hci_event_proxy_filter_status_send(p_hci_event, (wiced_bt_mesh_proxy_filter_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_PROVISION_SCAN_CAPABILITIES_STATUS:
        mesh_provisioner_hci_event_scan_capabilities_status_send(p_hci_event, (wiced_bt_mesh_provision_scan_capabilities_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_PROVISION_SCAN_STATUS:
        mesh_provisioner_hci_event_scan_status_send(p_hci_event, (wiced_bt_mesh_provision_scan_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_PROVISION_SCAN_REPORT:
        mesh_provisioner_hci_event_scan_report_send(p_hci_event, (wiced_bt_mesh_provision_scan_report_data_t *)p_data);
        break;

    case WICED_BT_MESH_PROVISION_SCAN_EXTENDED_REPORT:
        mesh_provisioner_hci_event_scan_extended_report_send(p_hci_event, (wiced_bt_mesh_provision_scan_extended_report_data_t *)p_data);
        break;

    case WICED_BT_MESH_PROXY_DEVICE:
        mesh_provisioner_hci_event_proxy_device_send(p_hci_event, (wiced_bt_mesh_proxy_device_network_data_t *)p_data);
        break;

    case WICED_BT_MESH_PROVISION_LINK_STATUS:
        mesh_provisioner_hci_event_provision_link_status_send(p_hci_event, (wiced_bt_mesh_provision_link_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_PROVISION_LINK_REPORT:
        mesh_provisioner_hci_event_provision_link_report_send(p_hci_event, (wiced_bt_mesh_provision_link_report_data_t *)p_data);
        break;

    case WICED_BT_MESH_PROVISION_END:
        mesh_provisioner_hci_event_provision_end_send(p_hci_event, (wiced_bt_mesh_provision_status_data_t *)p_data);
        break;

    case WICED_BT_MESH_PROVISION_DEVICE_CAPABILITIES:
        mesh_provisioner_hci_event_device_capabilities_send(p_hci_event, (wiced_bt_mesh_provision_device_capabilities_data_t *)p_data);
        break;

    case WICED_BT_MESH_PROVISION_GET_OOB_DATA:
        mesh_provisioner_hci_event_device_get_oob_data_send(p_hci_event, (wiced_bt_mesh_provision_device_oob_request_data_t *)p_data);
        break;

    default:
        WICED_BT_TRACE("ignored\n");
        break;
    }
    wiced_bt_mesh_release_event(p_event);
}


/*
 * In 2 chip solutions MCU can send commands to change provisioner state.
 */
uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_event_t *p_event;
    uint8_t status = HCI_CONTROL_MESH_STATUS_SUCCESS;

    WICED_BT_TRACE("%s opcode:%x\n", __FUNCTION__, opcode);

    if (mesh_onoff_client_proc_rx_cmd(opcode, p_data, length) ||
        mesh_level_client_proc_rx_cmd(opcode, p_data, length) ||
        mesh_light_lightness_client_proc_rx_cmd(opcode, p_data, length) ||
        mesh_light_hsl_client_proc_rx_cmd(opcode, p_data, length) ||
        mesh_light_ctl_client_proc_rx_cmd(opcode, p_data, length) ||
        mesh_default_transition_time_proc_rx_cmd(opcode, p_data, length) ||
        mesh_sensor_client_proc_rx_cmd(opcode, p_data, length) ||
        mesh_scene_client_proc_rx_cmd(opcode, p_data, length) ||
        mesh_vendor_service_proc_rx_cmd(opcode, p_data, length))
        return WICED_TRUE;

    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_SET_LOCAL_DEVICE:
        status = mesh_provisioner_process_set_local_device(p_data, length);
        mesh_provisioner_hci_send_status(status);
        return WICED_TRUE;

    case HCI_CONTROL_MESH_COMMAND_SET_DEVICE_KEY:
        status = mesh_provisioner_process_set_dev_key(p_data, length);
        mesh_provisioner_hci_send_status(status);
        return WICED_TRUE;

    case HCI_CONTROL_MESH_COMMAND_SEARCH_PROXY:
        status = mesh_provisioner_process_search_proxy(p_data, length);
        mesh_provisioner_hci_send_status(status);
        return WICED_TRUE;

    case HCI_CONTROL_MESH_COMMAND_PROXY_CONNECT:
        status = mesh_provisioner_process_proxy_connect(p_data, length);
        mesh_provisioner_hci_send_status(status);
        return WICED_TRUE;

    case HCI_CONTROL_MESH_COMMAND_PROXY_DISCONNECT:
        status = mesh_provisioner_process_proxy_disconnect(p_data, length);
        mesh_provisioner_hci_send_status(status);
        return WICED_TRUE;

    case HCI_CONTROL_MESH_COMMAND_PROVISION_SCAN_CAPABILITIES_GET:
    case HCI_CONTROL_MESH_COMMAND_PROVISION_SCAN_GET:
    case HCI_CONTROL_MESH_COMMAND_PROVISION_SCAN_START:
    case HCI_CONTROL_MESH_COMMAND_PROVISION_SCAN_STOP:
    case HCI_CONTROL_MESH_COMMAND_PROVISION_SCAN_EXTENDED_START:
    case HCI_CONTROL_MESH_COMMAND_PROVISION_CONNECT:
    case HCI_CONTROL_MESH_COMMAND_PROVISION_DISCONNECT:
    case HCI_CONTROL_MESH_COMMAND_PROVISION_START:
    case HCI_CONTROL_MESH_COMMAND_PROVISION_OOB_VALUE:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_NODE_RESET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_BEACON_GET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_BEACON_SET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_COMPOSITION_DATA_GET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_DEFAULT_TTL_GET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_DEFAULT_TTL_SET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_GATT_PROXY_GET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_GATT_PROXY_SET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_RELAY_GET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_RELAY_SET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_FRIEND_GET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_FRIEND_SET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_KEY_REFRESH_PHASE_GET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_KEY_REFRESH_PHASE_SET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_NODE_IDENTITY_GET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_NODE_IDENTITY_SET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_MODEL_PUBLICATION_GET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_MODEL_PUBLICATION_SET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_MODEL_SUBSCRIPTION_ADD:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_MODEL_SUBSCRIPTION_DELETE:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_MODEL_SUBSCRIPTION_OVERWRITE:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_MODEL_SUBSCRIPTION_DELETE_ALL:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_MODEL_SUBSCRIPTION_GET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_NET_KEY_ADD:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_NET_KEY_DELETE:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_NET_KEY_UPDATE:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_NET_KEY_GET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_APP_KEY_ADD:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_APP_KEY_DELETE:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_APP_KEY_UPDATE:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_APP_KEY_GET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_MODEL_APP_BIND:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_MODEL_APP_UNBIND:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_MODEL_APP_GET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_HEARBEAT_SUBSCRIPTION_GET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_HEARBEAT_SUBSCRIPTION_SET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_HEARBEAT_PUBLICATION_GET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_HEARBEAT_PUBLICATION_SET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_NETWORK_TRANSMIT_GET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_NETWORK_TRANSMIT_SET:
    case HCI_CONTROL_MESH_COMMAND_CONFIG_LPN_POLL_TIMEOUT_GET:
        p_event = wiced_bt_mesh_create_event_from_wiced_hci(opcode, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT, &p_data, &length);
        if (p_event == NULL)
        {
            WICED_BT_TRACE("bad hdr\n");
            return WICED_FALSE;
        }
        break;

    case HCI_CONTROL_MESH_COMMAND_PROXY_FILTER_TYPE_SET:
    case HCI_CONTROL_MESH_COMMAND_PROXY_FILTER_ADDRESSES_ADD:
    case HCI_CONTROL_MESH_COMMAND_PROXY_FILTER_ADDRESSES_DELETE:
        p_event = wiced_bt_mesh_create_event_from_wiced_hci(opcode, 0xFFFF, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT, &p_data, &length);
        if (p_event == NULL)
        {
            WICED_BT_TRACE("bad hdr\n");
            return WICED_FALSE;
        }
        break;

    case HCI_CONTROL_MESH_COMMAND_HEALTH_FAULT_GET:
    case HCI_CONTROL_MESH_COMMAND_HEALTH_FAULT_CLEAR:
    case HCI_CONTROL_MESH_COMMAND_HEALTH_FAULT_TEST:
    case HCI_CONTROL_MESH_COMMAND_HEALTH_PERIOD_GET:
    case HCI_CONTROL_MESH_COMMAND_HEALTH_PERIOD_SET:
    case HCI_CONTROL_MESH_COMMAND_HEALTH_ATTENTION_GET:
    case HCI_CONTROL_MESH_COMMAND_HEALTH_ATTENTION_SET:
        p_event = wiced_bt_mesh_create_event_from_wiced_hci(opcode, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_HEALTH_CLNT, &p_data, &length);
        if (p_event == NULL)
        {
            WICED_BT_TRACE("bad hdr\n");
            return WICED_FALSE;
        }
        break;

    default:
        WICED_BT_TRACE("bad hdr\n");
        return WICED_FALSE;
    }

    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_PROVISION_SCAN_CAPABILITIES_GET:
        status = mesh_provisioner_process_scan_capabilities_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_PROVISION_SCAN_GET:
        status = mesh_provisioner_process_scan_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_PROVISION_SCAN_START:
        status = mesh_provisioner_process_scan_start(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_PROVISION_SCAN_STOP:
        status = mesh_provisioner_process_scan_stop(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_PROVISION_SCAN_EXTENDED_START:
        status = mesh_provisioner_process_extended_scan_start(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_PROVISION_CONNECT:
        status = mesh_provisioner_process_connect(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_PROVISION_DISCONNECT:
        status = mesh_provisioner_process_disconnect(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_PROVISION_START:
        status = mesh_provisioner_process_start(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_PROVISION_OOB_VALUE:
        status = mesh_provisioner_process_oob_value(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_NODE_RESET:
        status = mesh_provisioner_process_node_reset(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_BEACON_GET:
        status = mesh_provisioner_process_beacon_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_BEACON_SET:
        status = mesh_provisioner_process_beacon_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_COMPOSITION_DATA_GET:
        status = mesh_provisioner_process_composition_data_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_DEFAULT_TTL_GET:
        status = mesh_provisioner_process_default_ttl_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_DEFAULT_TTL_SET:
        status = mesh_provisioner_process_default_ttl_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_GATT_PROXY_GET:
        status = mesh_provisioner_process_gatt_proxy_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_GATT_PROXY_SET:
        status = mesh_provisioner_process_gatt_proxy_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_RELAY_GET:
        status = mesh_provisioner_process_relay_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_RELAY_SET:
        status = mesh_provisioner_process_relay_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_FRIEND_GET:
        status = mesh_provisioner_process_friend_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_FRIEND_SET:
        status = mesh_provisioner_process_friend_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_KEY_REFRESH_PHASE_GET:
        status = mesh_provisioner_process_key_refresh_phase_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_KEY_REFRESH_PHASE_SET:
        status = mesh_provisioner_process_key_refresh_phase_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_NODE_IDENTITY_GET:
        status = mesh_provisioner_process_node_identity_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_NODE_IDENTITY_SET:
        status = mesh_provisioner_process_node_identity_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_MODEL_PUBLICATION_GET:
        status = mesh_provisioner_process_model_publication_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_MODEL_PUBLICATION_SET:
        status = mesh_provisioner_process_model_publication_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_MODEL_SUBSCRIPTION_ADD:
        status = mesh_provisioner_process_model_subscription_change(p_event, OPERATION_ADD, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_MODEL_SUBSCRIPTION_DELETE:
        status = mesh_provisioner_process_model_subscription_change(p_event, OPERATION_DELETE, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_MODEL_SUBSCRIPTION_OVERWRITE:
        status = mesh_provisioner_process_model_subscription_change(p_event, OPERATION_OVERWRITE, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_MODEL_SUBSCRIPTION_DELETE_ALL:
        status = mesh_provisioner_process_model_subscription_change(p_event, OPERATION_DELETE_ALL, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_MODEL_SUBSCRIPTION_GET:
        status = mesh_provisioner_process_model_subscription_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_NET_KEY_ADD:
        status = mesh_provisioner_process_netkey_change(p_event, OPERATION_ADD, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_NET_KEY_DELETE:
        status = mesh_provisioner_process_netkey_change(p_event, OPERATION_DELETE, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_NET_KEY_UPDATE:
        status = mesh_provisioner_process_netkey_change(p_event, OPERATION_OVERWRITE, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_NET_KEY_GET:
        status = mesh_provisioner_process_netkey_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_APP_KEY_ADD:
        status = mesh_provisioner_process_appkey_change(p_event, OPERATION_ADD, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_APP_KEY_DELETE:
        status = mesh_provisioner_process_appkey_change(p_event, OPERATION_DELETE, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_APP_KEY_UPDATE:
        status = mesh_provisioner_process_appkey_change(p_event, OPERATION_OVERWRITE, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_APP_KEY_GET:
        status = mesh_provisioner_process_appkey_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_MODEL_APP_BIND:
        status = mesh_provisioner_process_model_app_change(p_event, OPERATION_ADD, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_MODEL_APP_UNBIND:
        status = mesh_provisioner_process_model_app_change(p_event, OPERATION_DELETE, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_MODEL_APP_GET:
        status = mesh_provisioner_process_model_app_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_HEARBEAT_SUBSCRIPTION_GET:
        status = mesh_provisioner_process_heartbeat_subscription_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_HEARBEAT_SUBSCRIPTION_SET:
        status = mesh_provisioner_process_heartbeat_subscription_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_HEARBEAT_PUBLICATION_GET:
        status = mesh_provisioner_process_heartbeat_publication_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_HEARBEAT_PUBLICATION_SET:
        status = mesh_provisioner_process_heartbeat_publication_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_NETWORK_TRANSMIT_GET:
        status = mesh_provisioner_process_network_transmit_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_NETWORK_TRANSMIT_SET:
        status = mesh_provisioner_process_network_transmit_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_CONFIG_LPN_POLL_TIMEOUT_GET:
        status = mesh_provisioner_process_lpn_poll_timeout_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_PROXY_FILTER_TYPE_SET:
        status = mesh_provisioner_process_proxy_filter_type_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_PROXY_FILTER_ADDRESSES_ADD:
    case HCI_CONTROL_MESH_COMMAND_PROXY_FILTER_ADDRESSES_DELETE:
        status = mesh_provisioner_process_proxy_filter_change(p_event, opcode == HCI_CONTROL_MESH_COMMAND_PROXY_FILTER_ADDRESSES_ADD, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_HEALTH_FAULT_GET:
        status = mesh_provisioner_process_health_fault_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_HEALTH_FAULT_CLEAR:
        status = mesh_provisioner_process_health_fault_clear(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_HEALTH_FAULT_TEST:
        status = mesh_provisioner_process_health_fault_test(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_HEALTH_PERIOD_GET:
        status = mesh_provisioner_process_health_period_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_HEALTH_PERIOD_SET:
        status = mesh_provisioner_process_health_period_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_HEALTH_ATTENTION_GET:
        status = mesh_provisioner_process_health_attention_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_HEALTH_ATTENTION_SET:
        status = mesh_provisioner_process_health_attention_set(p_event, p_data, length);
        break;

    default:
        wiced_bt_mesh_release_event(p_event);
        break;
    }
    mesh_provisioner_hci_send_status(status);
    return WICED_TRUE;
}

/*
 * Process command from MCU to setup local device
 */
uint8_t mesh_provisioner_process_set_local_device(uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_local_device_set_data_t set;

    STREAM_TO_UINT16(set.addr, p_data);
    STREAM_TO_ARRAY(set.dev_key, p_data, 16);
    STREAM_TO_ARRAY(set.network_key, p_data, 16);
    STREAM_TO_UINT16(set.net_key_idx, p_data);
    STREAM_TO_UINT32(set.iv_idx, p_data);
    STREAM_TO_UINT8(set.key_refresh, p_data);
    STREAM_TO_UINT8(set.iv_update, p_data);

    WICED_BT_TRACE("addr:%x net_key_idx:%x iv_idx:%x key_refresh:%d iv_upd:%d\n", set.addr, set.net_key_idx, set.iv_idx, set.key_refresh, set.iv_update);
    mesh_gatt_client_local_device_set(&set);
    mesh_app_init(WICED_TRUE);
    return HCI_CONTROL_MESH_STATUS_SUCCESS;
}

/*
* Process command from MCU to set device key.  MCU can set device key once and then perform multiple configuration commands.
*/
uint8_t mesh_provisioner_process_set_dev_key(uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_set_dev_key_data_t set;
    STREAM_TO_UINT16(set.dst, p_data);
    STREAM_TO_ARRAY(set.dev_key, p_data, 16);
    wiced_bt_mesh_provision_set_dev_key(&set);
    return HCI_CONTROL_MESH_STATUS_SUCCESS;
}

/*
 * Send Scan Info Get command to Remote Provisioning Server
 */
uint8_t mesh_provisioner_process_scan_capabilities_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    return wiced_bt_mesh_provision_scan_capabilities_get(p_event) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Send Scan Get command to Remote Provisioning Server
 */
uint8_t mesh_provisioner_process_scan_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    return wiced_bt_mesh_provision_scan_get(p_event) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Send Start Scan command to Remote Provisioning Server
 */
uint8_t mesh_provisioner_process_scan_start(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_provision_scan_start_data_t data;

    STREAM_TO_UINT8(data.scanned_items_limit, p_data);
    STREAM_TO_UINT8(data.timeout, p_data);
    return wiced_bt_mesh_provision_scan_start(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Send Scan Extended Start command to Remote Provisioning Server
 */
uint8_t mesh_provisioner_process_extended_scan_start(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_provision_scan_extended_start_t data;
    int i;

    memset(&data, 0, sizeof(data));
    STREAM_TO_UINT8(data.timeout, p_data);
    length -= 1;

    for (i = 0; (i < WICED_BT_MESH_AD_FILTER_TYPES_MAX) && (length > 0); i++)
    {
        data.ad_filter_types[i] = *p_data++;
        length--;
        if (data.ad_filter_types[i] == 0)
            break;
    }
    if ((length != 16) && (length != 0))
    {
        WICED_BT_TRACE("ext scan start len:%d\n", length);
        return WICED_FALSE;
    }
    if (length == 16)
    {
        data.uuid_present = WICED_TRUE;
        memcpy(data.uuid, p_data, 16);
    }
    return wiced_bt_mesh_provision_scan_extended_start(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Send Stop Scan command to Remote Provisioning Server
 */
uint8_t mesh_provisioner_process_scan_stop(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    return wiced_bt_mesh_provision_scan_stop(p_event) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to connect provisioning link
 */
uint8_t mesh_provisioner_process_connect(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_provision_connect_data_t connect;
    uint8_t use_pb_gatt;

    STREAM_TO_ARRAY(connect.uuid, p_data, 16);
    STREAM_TO_UINT8(connect.identify_duration, p_data);
    STREAM_TO_UINT8(use_pb_gatt, p_data);

    return wiced_bt_mesh_provision_connect(p_event, &connect, use_pb_gatt)  ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to disconnect provisioning link
 */
uint8_t mesh_provisioner_process_disconnect(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    return wiced_bt_mesh_provision_disconnect(p_event) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to start provisioning on the established provisioning link
 */
uint8_t mesh_provisioner_process_start(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_provision_start_data_t start;

    STREAM_TO_UINT16(start.addr, p_data);
    STREAM_TO_UINT8(start.algorithm, p_data);
    STREAM_TO_UINT8(start.public_key_type, p_data);
    STREAM_TO_UINT8(start.auth_method, p_data);
    STREAM_TO_UINT8(start.auth_action, p_data);
    STREAM_TO_UINT8(start.auth_size, p_data);

    return wiced_bt_mesh_provision_start(p_event, &start) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to with OOB value to be used in the link calculation
 */
uint8_t mesh_provisioner_process_oob_value(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_provision_oob_value_data_t oob;

    STREAM_TO_UINT8(oob.data_size, p_data);
    STREAM_TO_ARRAY(oob.data, p_data, length);

    return wiced_bt_mesh_provision_client_set_oob(p_event, &oob) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Start Search for devices that support GATT Proxy functionality
 */
uint8_t mesh_provisioner_process_search_proxy(uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_client_search_proxy(p_data[0]);
    return HCI_CONTROL_MESH_STATUS_SUCCESS;
}

/*
 * Process command from MCU to connect to a GATT Proxy
 */
uint8_t mesh_provisioner_process_proxy_connect(uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_proxy_connect_data_t connect;

    if (length == 3)
    {
        connect.connect_type = CONNECT_TYPE_NODE_ID;
        STREAM_TO_UINT16(connect.node_id, p_data);
        STREAM_TO_UINT8(connect.scan_duration, p_data);
    }
    else if (length == BD_ADDR_LEN + 2)
    {
        connect.connect_type = CONNECT_TYPE_BDADDR;
        STREAM_TO_BDADDR(connect.bd_addr, p_data);
        STREAM_TO_UINT8(connect.bd_addr_type, p_data);
        STREAM_TO_UINT8(connect.scan_duration, p_data);
    }
    else if (length == 1)
    {
        connect.connect_type = CONNECT_TYPE_NET_ID;
        STREAM_TO_UINT8(connect.scan_duration, p_data);
    }
    else
    {
        return HCI_CONTROL_MESH_STATUS_ERROR;
    }
    return wiced_bt_mesh_client_proxy_connect(&connect)  ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to disconnect GATT Proxy
 */
uint8_t mesh_provisioner_process_proxy_disconnect(uint8_t *p_data, uint32_t length)
{
    return wiced_bt_mesh_client_proxy_disconnect() ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}


/*
 * Process command from MCU to Reset Node
 */
uint8_t mesh_provisioner_process_node_reset(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    return wiced_bt_mesh_config_node_reset(p_event) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Get Mesh Secure Beacon Status
 */
uint8_t mesh_provisioner_process_beacon_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    return wiced_bt_mesh_config_beacon_get(p_event) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Set Mesh Secure Beacon Status
 */
uint8_t mesh_provisioner_process_beacon_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_beacon_set_data_t data;

    STREAM_TO_UINT8(data.state, p_data);

    return wiced_bt_mesh_config_beacon_set(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Get Mesh Node Composition Data
 */
uint8_t mesh_provisioner_process_composition_data_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_composition_data_get_data_t data;

    STREAM_TO_UINT8(data.page_number, p_data);

    return wiced_bt_mesh_config_composition_data_get(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Get Value of the Node's Default TTL
 */
uint8_t mesh_provisioner_process_default_ttl_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    return wiced_bt_mesh_config_default_ttl_get(p_event) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Set Value of the Node's Default TTL
 */
uint8_t mesh_provisioner_process_default_ttl_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_default_ttl_set_data_t data;

    STREAM_TO_UINT8(data.ttl, p_data);

    return wiced_bt_mesh_config_default_ttl_set(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Get GATT Proxy State
 */
uint8_t mesh_provisioner_process_gatt_proxy_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    return wiced_bt_mesh_config_gatt_proxy_get(p_event) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Set GATT Proxy Status
 */
uint8_t mesh_provisioner_process_gatt_proxy_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_gatt_proxy_set_data_t data;

    STREAM_TO_UINT8(data.state, p_data);

    return wiced_bt_mesh_config_gatt_proxy_set(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Get Relay Status and parameters
 */
uint8_t mesh_provisioner_process_relay_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    return wiced_bt_mesh_config_relay_get(p_event) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Set Relay Status and parameters
 */
uint8_t mesh_provisioner_process_relay_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_relay_set_data_t data;

    STREAM_TO_UINT8(data.state, p_data);
    STREAM_TO_UINT8(data.retransmit_count, p_data);
    STREAM_TO_UINT16(data.retransmit_interval, p_data);

    return wiced_bt_mesh_config_relay_set(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Get Friend State
 */
uint8_t mesh_provisioner_process_friend_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    return wiced_bt_mesh_config_friend_get(p_event) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Set Friend State
 */
uint8_t mesh_provisioner_process_friend_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_friend_set_data_t data;

    STREAM_TO_UINT8(data.state, p_data);

    return wiced_bt_mesh_config_friend_set(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Get Key Refresh Phase
 */
uint8_t mesh_provisioner_process_key_refresh_phase_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_key_refresh_phase_get_data_t data;

    STREAM_TO_UINT16(data.net_key_idx, p_data);

    return wiced_bt_mesh_config_key_refresh_phase_get(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Set Get Key Refresh Phase
 */
uint8_t mesh_provisioner_process_key_refresh_phase_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_key_refresh_phase_set_data_t data;

    STREAM_TO_UINT16(data.net_key_idx, p_data);
    STREAM_TO_UINT8(data.transition, p_data);

    return wiced_bt_mesh_config_key_refresh_phase_set(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Get Node Identity State
 */
uint8_t mesh_provisioner_process_node_identity_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_node_identity_get_data_t data;

    STREAM_TO_UINT16(data.net_key_idx, p_data);

    return wiced_bt_mesh_config_node_identity_get(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Set Node Identity State
 */
uint8_t mesh_provisioner_process_node_identity_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_node_identity_set_data_t data;

    STREAM_TO_UINT16(data.net_key_idx, p_data);
    STREAM_TO_UINT8(data.identity, p_data);

    return wiced_bt_mesh_config_node_identity_set(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Model Publication
 */
uint8_t mesh_provisioner_process_model_publication_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_model_publication_get_data_t data;

    STREAM_TO_UINT16(data.element_addr, p_data);
    STREAM_TO_UINT16(data.company_id, p_data);
    STREAM_TO_UINT16(data.model_id, p_data);

    return wiced_bt_mesh_config_model_publication_get(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Set Model Publication
 */
uint8_t mesh_provisioner_process_model_publication_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_model_publication_set_data_t data;

    STREAM_TO_UINT16(data.element_addr, p_data);
    STREAM_TO_UINT16(data.company_id, p_data);
    STREAM_TO_UINT16(data.model_id, p_data);
    STREAM_TO_ARRAY(data.publish_addr, p_data, 16);
    STREAM_TO_UINT16(data.app_key_idx, p_data);
    STREAM_TO_UINT8(data.credential_flag, p_data);
    STREAM_TO_UINT8(data.publish_ttl, p_data);
    STREAM_TO_UINT32(data.publish_period, p_data);
    STREAM_TO_UINT8(data.publish_retransmit_count, p_data);
    STREAM_TO_UINT16(data.publish_retransmit_interval, p_data);

    return wiced_bt_mesh_config_model_publication_set(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Add, Delete, Overwrite. or Delete All addresses from a Model Subscription
 */
uint8_t mesh_provisioner_process_model_subscription_change(wiced_bt_mesh_event_t *p_event, uint8_t operation, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_model_subscription_change_data_t data;

    data.operation = operation;
    STREAM_TO_UINT16(data.element_addr, p_data);
    STREAM_TO_UINT16(data.company_id, p_data);
    STREAM_TO_UINT16(data.model_id, p_data);
    if (operation != OPERATION_DELETE_ALL)
    {
        STREAM_TO_ARRAY(data.addr, p_data, 16);
    }
    return wiced_bt_mesh_config_model_subscription_change(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Get Model Subscription list
 */
uint8_t mesh_provisioner_process_model_subscription_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_model_subscription_get_data_t data;

    STREAM_TO_UINT16(data.element_addr, p_data);
    STREAM_TO_UINT16(data.company_id, p_data);
    STREAM_TO_UINT16(data.model_id, p_data);
    return wiced_bt_mesh_config_model_subscription_get(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Add, Delete, Update Network Key
 */
uint8_t mesh_provisioner_process_netkey_change(wiced_bt_mesh_event_t *p_event, uint8_t operation, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_netkey_change_data_t data;

    data.operation = operation;
    STREAM_TO_UINT16(data.net_key_idx, p_data);
    if (operation != OPERATION_DELETE)
    {
        STREAM_TO_ARRAY(data.net_key, p_data, 16);
    }
    return wiced_bt_mesh_config_netkey_change(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Get Network Keys list
 */
uint8_t mesh_provisioner_process_netkey_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    return wiced_bt_mesh_config_netkey_get(p_event) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Add, Delete, Update an Application Key
 */
uint8_t mesh_provisioner_process_appkey_change(wiced_bt_mesh_event_t *p_event, uint8_t operation, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_appkey_change_data_t data;

    data.operation = operation;
    STREAM_TO_UINT16(data.net_key_idx, p_data);
    STREAM_TO_UINT16(data.app_key_idx, p_data);
    if (operation != OPERATION_DELETE)
    {
        STREAM_TO_ARRAY(data.app_key, p_data, 16);
    }
    return wiced_bt_mesh_config_appkey_change(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Get Application Keys list
 */
uint8_t mesh_provisioner_process_appkey_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_appkey_get_data_t data;

    STREAM_TO_UINT16(data.net_key_idx, p_data);
    return wiced_bt_mesh_config_appkey_get(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Bond or Unbind a Model to an Application Key
 */
uint8_t mesh_provisioner_process_model_app_change(wiced_bt_mesh_event_t *p_event, uint8_t operation, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_model_app_bind_data_t data;

    data.operation = operation;
    STREAM_TO_UINT16(data.element_addr, p_data);
    STREAM_TO_UINT16(data.company_id, p_data);
    STREAM_TO_UINT16(data.model_id, p_data);
    STREAM_TO_UINT16(data.app_key_idx, p_data);
    return wiced_bt_mesh_config_model_app_bind(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to Get Application Keys bound to a Model
 */
uint8_t mesh_provisioner_process_model_app_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_model_app_get_data_t data;

    STREAM_TO_UINT16(data.element_addr, p_data);
    STREAM_TO_UINT16(data.company_id, p_data);
    STREAM_TO_UINT16(data.model_id, p_data);
    return wiced_bt_mesh_config_model_app_get(p_event, &data) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to get hearbeat subscription on a device
 */
uint8_t mesh_provisioner_process_heartbeat_subscription_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    return wiced_bt_mesh_config_heartbeat_subscription_get(p_event) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to set hearbeat subscription on a device
 */
uint8_t mesh_provisioner_process_heartbeat_subscription_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_heartbeat_subscription_set_data_t set;

    STREAM_TO_UINT16(set.subscription_src, p_data);
    STREAM_TO_UINT16(set.subscription_dst, p_data);
    STREAM_TO_UINT32(set.period, p_data);

    return wiced_bt_mesh_config_heartbeat_subscription_set(p_event, &set) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to get hearbeat publication on a device
 */
uint8_t mesh_provisioner_process_heartbeat_publication_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    return wiced_bt_mesh_config_heartbeat_publication_get(p_event) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}


/*
 * Process command from MCU to set hearbeat publication on a device
 */
uint8_t mesh_provisioner_process_heartbeat_publication_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_heartbeat_publication_set_data_t set;

    STREAM_TO_UINT16(set.publication_dst, p_data);
    STREAM_TO_UINT32(set.count, p_data);
    STREAM_TO_UINT32(set.period, p_data);
    STREAM_TO_UINT8(set.ttl, p_data);
    STREAM_TO_UINT8(set.feature_relay, p_data);
    STREAM_TO_UINT8(set.feature_proxy, p_data);
    STREAM_TO_UINT8(set.feature_friend, p_data);
    STREAM_TO_UINT8(set.feature_low_power, p_data);
    STREAM_TO_UINT16(set.net_key_idx, p_data);

    return wiced_bt_mesh_config_heartbeat_publication_set(p_event, &set) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to get network transmit parameters of a device
 */
uint8_t mesh_provisioner_process_network_transmit_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    return wiced_bt_mesh_config_network_transmit_params_get(p_event) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
* Process command from MCU to set network transmit parameters of a device
*/
uint8_t mesh_provisioner_process_network_transmit_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_config_network_transmit_set_data_t set;

    STREAM_TO_UINT8(set.count, p_data);
    STREAM_TO_UINT16(set.interval, p_data);

    return wiced_bt_mesh_config_network_transmit_params_set(p_event, &set) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to get the current Registered Fault state identified by Company ID of an element
 */
uint8_t mesh_provisioner_process_health_fault_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_health_fault_get_data_t get;

    STREAM_TO_UINT16(get.company_id, p_data);

    return wiced_bt_mesh_health_fault_get(p_event, &get) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to clear the current Registered Fault state identified by Company ID of an element
 */
uint8_t mesh_provisioner_process_health_fault_clear(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_health_fault_clear_data_t clear;

    STREAM_TO_UINT16(clear.company_id, p_data);

    return wiced_bt_mesh_health_fault_clear(p_event, &clear) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to nvoke a self-test procedure of an element
 */
uint8_t mesh_provisioner_process_health_fault_test(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_health_fault_test_data_t test;

    STREAM_TO_UINT8(test.id, p_data);
    STREAM_TO_UINT16(test.company_id, p_data);

    return wiced_bt_mesh_health_fault_test(p_event, &test) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to get the current Health Period state of an element
 */
uint8_t mesh_provisioner_process_health_period_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    return wiced_bt_mesh_health_period_get(p_event) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

/*
 * Process command from MCU to set the current Health Period state of an element
 */
uint8_t mesh_provisioner_process_health_period_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_health_period_set_data_t period;

    STREAM_TO_UINT8(period.divisor, p_data);

    return wiced_bt_mesh_health_period_set(p_event, &period) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

uint8_t mesh_provisioner_process_health_attention_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    return wiced_bt_mesh_health_attention_get(p_event) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

uint8_t mesh_provisioner_process_health_attention_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_health_attention_set_data_t attention;

    STREAM_TO_UINT8(attention.timer, p_data);

    return wiced_bt_mesh_health_attention_set(p_event, &attention) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

uint8_t mesh_provisioner_process_lpn_poll_timeout_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_lpn_poll_timeout_get_data_t get;

    STREAM_TO_UINT16(get.lpn_addr, p_data);

    return wiced_bt_mesh_lpn_poll_timeout_get(p_event, &get) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

uint8_t mesh_provisioner_process_proxy_filter_type_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_proxy_filter_set_type_data_t set;

    STREAM_TO_UINT8(set.type, p_data);

    return wiced_bt_mesh_proxy_set_filter_type(p_event, &set) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}

uint8_t mesh_provisioner_process_proxy_filter_change(wiced_bt_mesh_event_t *p_event, wiced_bool_t is_add, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_proxy_filter_change_addr_data_t *p_addr;
    uint16_t addr_num = length / 2;
    uint16_t i;
    uint8_t  res;

    if ((p_addr = (wiced_bt_mesh_proxy_filter_change_addr_data_t *)wiced_bt_get_buffer(sizeof(wiced_bt_mesh_proxy_filter_change_addr_data_t) + (addr_num - 1) * 2)) == NULL)
        return HCI_CONTROL_MESH_STATUS_ERROR;

    p_addr->addr_num = addr_num;
    for (i = 0; i < addr_num; i++)
        STREAM_TO_UINT16(p_addr->addr[i], p_data);

    res = wiced_bt_mesh_proxy_filter_change_addr(p_event, is_add, p_addr) ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
    wiced_bt_free_buffer(p_addr);
    return res;
}

void mesh_provisioner_hci_send_status(uint8_t status)
{
    uint8_t *p_buffer = wiced_transport_allocate_buffer(host_trans_pool);
    uint8_t *p = p_buffer;

    UINT8_TO_STREAM(p, status);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_COMMAND_STATUS, p_buffer, (uint16_t)(p - p_buffer));
}

/*
 * Send Remote Provisioner scan info event over transport
 */
void mesh_provisioner_hci_event_scan_capabilities_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_provision_scan_capabilities_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT8_TO_STREAM(p, p_data->max_scanned_items);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_PROVISION_SCAN_CAPABILITIES_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Remote Provisioner scan status event over transport
 */
void mesh_provisioner_hci_event_scan_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_provision_scan_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT8_TO_STREAM(p, p_data->status);
    UINT8_TO_STREAM(p, p_data->phase);
    UINT8_TO_STREAM(p, p_data->scanned_items_limit);
    UINT8_TO_STREAM(p, p_data->timeout);
    UINT8_TO_STREAM(p, p_data->scanning_type);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_PROVISION_SCAN_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send provisioner scan report event over transport
 */
void mesh_provisioner_hci_event_scan_report_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_provision_scan_report_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT8_TO_STREAM(p, p_data->rssi);
    ARRAY_TO_STREAM(p, p_data->uuid, 16);
    UINT16_TO_STREAM(p, p_data->oob);
    UINT32_TO_STREAM(p, p_data->uri_hash);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_PROVISION_SCAN_REPORT, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send provisioner scan extended report event over transport
 */
void mesh_provisioner_hci_event_scan_extended_report_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_provision_scan_extended_report_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;
    uint8_t *p_adv_data;

    *p++ = p_data->status;
    memcpy(p, p_data->uuid, 16);
    p += 16;

    p_adv_data = p_data->adv_data;
    while (*p_adv_data != 0)
    {
        memcpy(p, p_adv_data, *p_adv_data + 1);
        p += *p_adv_data + 1;
        p_adv_data += *p_adv_data + 1;
    }
    *p++ = 0;
    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_PROVISION_SCAN_EXTENDED_REPORT, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Proxy Device Network Data event over transport
 */
void mesh_provisioner_hci_event_proxy_device_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_proxy_device_network_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    BDADDR_TO_STREAM(p, p_data->bd_addr);
    UINT8_TO_STREAM(p, p_data->bd_addr_type);
    UINT8_TO_STREAM(p, p_data->rssi);
    UINT16_TO_STREAM(p, p_data->net_key_idx);
    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_PROXY_DEVICE_NETWORK_DATA, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));

}

/*
 * Send link status event over transport
 */
void mesh_provisioner_hci_event_provision_link_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_provision_link_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("provision connection not sent status from %x addr:%x connected:%d\n", p_hci_event->src, p_data->status);
    UINT8_TO_STREAM(p, p_data->status);

    // mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_PROVISION_LINK_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_provision_link_report_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_provision_link_report_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;
    uint8_t connected = ((p_data->rpr_state == WICED_BT_MESH_REMOTE_PROVISION_STATE_LINK_ACTIVE) || (p_data->rpr_state == WICED_BT_MESH_REMOTE_PROVISION_STATE_OUTBOUNT_PDU_TRANSFER));

    WICED_BT_TRACE("provision link report from %x status:%d state:%d reason:%d\n", p_hci_event->src, p_data->link_status, p_data->rpr_state, p_data->reason);
    UINT16_TO_STREAM(p, p_hci_event->src);
    UINT16_TO_STREAM(p, 0);                     // address
    UINT8_TO_STREAM(p, connected);
    UINT8_TO_STREAM(p, 0);                      // don't know if it is over GATT

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_PROVISION_LINK_REPORT, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send provisioner provisioning end event over transport
 */
void mesh_provisioner_hci_event_provision_end_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_provision_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("provision end addr:%x result:%d\n", p_data->addr, p_data->result);

    UINT16_TO_STREAM(p, p_data->provisioner_addr);
    UINT16_TO_STREAM(p, p_data->addr);
    UINT16_TO_STREAM(p, p_data->net_key_idx);
    UINT8_TO_STREAM(p, p_data->result);
    ARRAY_TO_STREAM(p, p_data->dev_key, WICED_BT_MESH_KEY_LEN);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_PROVISION_END, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send to the MCU device capabilities received during provisioning
 */
void mesh_provisioner_hci_event_device_capabilities_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_provision_device_capabilities_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("mesh prov caps from:%x num_elelements:%d key_type:%d\n", p_data->provisioner_addr, p_data->elements_num, p_data->pub_key_type);

    UINT16_TO_STREAM(p, p_data->provisioner_addr);
    UINT8_TO_STREAM(p, p_data->elements_num);
    UINT16_TO_STREAM(p, p_data->algorithms);
    UINT8_TO_STREAM(p, p_data->pub_key_type);
    UINT8_TO_STREAM(p, p_data->static_oob_type);
    UINT8_TO_STREAM(p, p_data->output_oob_size);
    UINT16_TO_STREAM(p, p_data->output_oob_action);
    UINT8_TO_STREAM(p, p_data->input_oob_size);
    UINT16_TO_STREAM(p, p_data->input_oob_action);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_PROVISION_DEVICE_CAPABITIES, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send to the MCU Out of Band data request
 */
void mesh_provisioner_hci_event_device_get_oob_data_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_provision_device_oob_request_data_t *p_oob_data)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("mesh prov oob req from:%x type:%d size:%d action:%d\n", p_oob_data->provisioner_addr, p_oob_data->type, p_oob_data->size, p_oob_data->action);

    UINT16_TO_STREAM(p, p_oob_data->provisioner_addr);
    UINT8_TO_STREAM(p, p_oob_data->type);
    UINT8_TO_STREAM(p, p_oob_data->size);
    UINT8_TO_STREAM(p, p_oob_data->action);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_PROVISION_OOB_DATA, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_proxy_filter_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_proxy_filter_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT8_TO_STREAM(p, p_data->type);
    UINT16_TO_STREAM(p, p_data->list_size);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_PROXY_FILTER_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_node_reset_status_send(wiced_bt_mesh_hci_event_t *p_hci_event)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("node reset status\n", p_hci_event->src);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_NODE_RESET_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_node_identity_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_node_identity_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("node identity status src:%x\n", p_hci_event->src);

    UINT8_TO_STREAM(p, p_data->status);
    UINT16_TO_STREAM(p, p_data->net_key_idx);
    UINT8_TO_STREAM(p, p_data->identity);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_NODE_IDENTITY_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}


void mesh_provisioner_hci_event_composition_data_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_composition_data_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("comp status src:%x page_num:%d len:%d\n", p_hci_event->src, p_data->page_number, p_data->data_len);

    UINT8_TO_STREAM(p, p_data->page_number);
    ARRAY_TO_STREAM(p, p_data->data, p_data->data_len);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_COMPOSITION_DATA_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
    wiced_bt_free_buffer(p_data);
}

void mesh_provisioner_hci_event_friend_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_friend_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("friend status src:%x\n", p_hci_event->src);

    UINT8_TO_STREAM(p, p_data->state);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_FRIEND_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_key_refresh_phase_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_key_refresh_phase_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("kr status src:%x\n", p_hci_event->src);

    UINT8_TO_STREAM(p, p_data->status);
    UINT16_TO_STREAM(p, p_data->net_key_idx);
    UINT8_TO_STREAM(p, p_data->phase);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_KEY_REFRESH_PHASE_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_default_ttl_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_default_ttl_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("default_ttl status src:%x\n", p_hci_event->src);

    UINT8_TO_STREAM(p, p_data->ttl);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_DEFAULT_TTL_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_relay_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_relay_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("relay status src:%x\n", p_hci_event->src);

    UINT8_TO_STREAM(p, p_data->state);
    UINT8_TO_STREAM(p, p_data->retransmit_count);
    UINT16_TO_STREAM(p, p_data->retransmit_interval);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_RELAY_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_gatt_proxy_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_gatt_proxy_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("gatt_proxy status src:%x\n", p_hci_event->src);

    UINT8_TO_STREAM(p, p_data->state);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_GATT_PROXY_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_beacon_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_beacon_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("beacon status src:%x\n", p_hci_event->src);

    UINT8_TO_STREAM(p, p_data->state);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_BEACON_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_model_publication_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_model_publication_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("model pub status src:%x\n", p_hci_event->src);

    UINT8_TO_STREAM(p, p_data->status);
    UINT16_TO_STREAM(p, p_data->element_addr);
    UINT16_TO_STREAM(p, p_data->company_id);
    UINT16_TO_STREAM(p, p_data->model_id);
    UINT16_TO_STREAM(p, p_data->publish_addr);
    UINT16_TO_STREAM(p, p_data->app_key_idx);
    UINT8_TO_STREAM(p, p_data->credential_flag);
    UINT8_TO_STREAM(p, p_data->publish_ttl);
    UINT32_TO_STREAM(p, p_data->publish_period);
    UINT8_TO_STREAM(p, p_data->publish_retransmit_count);
    UINT16_TO_STREAM(p, p_data->publish_retransmit_interval);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_MODEL_PUBLICATION_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_model_subscription_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_model_subscription_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("model sub status src:%x\n", p_hci_event->src);

    UINT8_TO_STREAM(p, p_data->status);
    UINT16_TO_STREAM(p, p_data->element_addr);
    UINT16_TO_STREAM(p, p_data->company_id);
    UINT16_TO_STREAM(p, p_data->model_id);
    UINT16_TO_STREAM(p, p_data->addr);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_MODEL_SUBSCRIPTION_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_model_subscription_list_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_model_subscription_list_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;
    int i;

    WICED_BT_TRACE("model sub list src:%x\n", p_hci_event->src);

    UINT8_TO_STREAM(p, p_data->status);
    UINT16_TO_STREAM(p, p_data->element_addr);
    UINT16_TO_STREAM(p, p_data->company_id);
    UINT16_TO_STREAM(p, p_data->model_id);
    for (i = 0; i < p_data->num_addr; i++)
    {
        UINT16_TO_STREAM(p, p_data->addr[i]);
    }
    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_MODEL_SUBSCRIPTION_LIST, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_netkey_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_netkey_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("netkey status src:%x\n", p_hci_event->src);

    UINT8_TO_STREAM(p, p_data->status);
    UINT16_TO_STREAM(p, p_data->net_key_idx);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_NETKEY_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_netkey_list_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_netkey_list_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;
    int i;

    WICED_BT_TRACE("netkey list src:%x\n", p_hci_event->src);

    for (i = 0; i < p_data->num_keys; i++)
    {
        UINT16_TO_STREAM(p, p_data->net_key_idx[i]);
    }
    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_NETKEY_LIST, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_appkey_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_appkey_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("appkey status src:%x\n", p_hci_event->src);

    UINT8_TO_STREAM(p, p_data->status);
    UINT16_TO_STREAM(p, p_data->net_key_idx);
    UINT16_TO_STREAM(p, p_data->app_key_idx);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_APPKEY_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_appkey_list_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_appkey_list_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;
    int i;

    WICED_BT_TRACE("appkey list src:%x\n", p_hci_event->src);

    UINT8_TO_STREAM(p, p_data->status);
    UINT16_TO_STREAM(p, p_data->net_key_idx);
    for (i = 0; i < p_data->num_keys; i++)
    {
        UINT16_TO_STREAM(p, p_data->app_key_idx[i]);
    }
    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_APPKEY_LIST, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_model_app_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_model_app_bind_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("model app status src:%x\n", p_hci_event->src);

    UINT8_TO_STREAM(p, p_data->status);
    UINT16_TO_STREAM(p, p_data->element_addr);
    UINT16_TO_STREAM(p, p_data->company_id);
    UINT16_TO_STREAM(p, p_data->model_id);
    UINT16_TO_STREAM(p, p_data->app_key_idx);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_MODEL_APP_BIND_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_model_app_list_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_model_app_list_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;
    int i;

    WICED_BT_TRACE("model_app list src:%x\n", p_hci_event->src);

    UINT8_TO_STREAM(p, p_data->status);
    UINT16_TO_STREAM(p, p_data->element_addr);
    UINT16_TO_STREAM(p, p_data->company_id);
    UINT16_TO_STREAM(p, p_data->model_id);
    for (i = 0; i < p_data->num_keys; i++)
    {
        UINT16_TO_STREAM(p, p_data->app_key_idx[i]);
    }
    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_MODEL_APP_LIST, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_hearbeat_subscription_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_heartbeat_subscription_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("subs status src:%x status:%d subs src/dst:%x/%x period:%d count:%d hops min/max:%d/%d\n",
        p_hci_event->src, p_data->status, p_data->subscription_src, p_data->subscription_dst, p_data->period, p_data->count, p_data->min_hops, p_data->max_hops);

    UINT8_TO_STREAM(p, p_data->status);
    UINT16_TO_STREAM(p, p_data->subscription_src);
    UINT16_TO_STREAM(p, p_data->subscription_dst);
    UINT32_TO_STREAM(p, p_data->period);
    UINT16_TO_STREAM(p, p_data->count);
    UINT8_TO_STREAM(p, p_data->min_hops);
    UINT8_TO_STREAM(p, p_data->max_hops);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_HEARTBEAT_SUBSCRIPTION_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_hearbeat_publication_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_heartbeat_publication_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("pubs status src:%x status:%d pubs dst:%x period:%d count:%d hops ttl:%d net_key_idx:%d\n",
        p_hci_event->src, p_data->status, p_data->publication_dst, p_data->period, p_data->count, p_data->ttl, p_data->net_key_idx);

    UINT8_TO_STREAM(p, p_data->status);
    UINT16_TO_STREAM(p, p_data->publication_dst);
    UINT32_TO_STREAM(p, p_data->period);
    UINT16_TO_STREAM(p, p_data->count);
    UINT8_TO_STREAM(p, p_data->ttl);
    UINT8_TO_STREAM(p, p_data->feature_relay);
    UINT8_TO_STREAM(p, p_data->feature_proxy);
    UINT8_TO_STREAM(p, p_data->feature_friend);
    UINT8_TO_STREAM(p, p_data->feature_low_power);
    UINT16_TO_STREAM(p, p_data->net_key_idx);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_HEARTBEAT_PUBLICATION_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_network_transmit_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_config_network_transmit_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("net xmit params src:%x count:%d interval:%d\n", p_hci_event->src, p_data->count, p_data->interval);

    UINT8_TO_STREAM(p, p_data->count);
    UINT32_TO_STREAM(p, p_data->interval);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_NETWORK_TRANSMIT_PARAMS_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_health_current_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_health_fault_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, p_data->app_key_idx);
    UINT8_TO_STREAM(p, p_data->test_id);
    UINT16_TO_STREAM(p, p_data->company_id);
    ARRAY_TO_STREAM(p, p_data->fault_array, p_data->count);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_HEALTH_CURRENT_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_health_fault_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_health_fault_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, p_data->app_key_idx);
    UINT8_TO_STREAM(p, p_data->test_id);
    UINT16_TO_STREAM(p, p_data->company_id);
    ARRAY_TO_STREAM(p, p_data->fault_array, p_data->count);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_HEALTH_FAULT_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_health_period_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_health_period_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, p_data->app_key_idx);
    UINT8_TO_STREAM(p, p_data->divisor);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_HEALTH_PERIOD_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_health_attention_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_health_attention_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT8_TO_STREAM(p, p_data->timer);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_HEALTH_ATTENTION_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_provisioner_hci_event_lpn_poll_timeout_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_lpn_poll_timeout_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, p_data->lpn_addr);
    UINT32_TO_STREAM(p, p_data->poll_timeout);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LPN_POLL_TIMEOUT_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

// ToDo. Currently provisioner client will only scan passive not returning the name of the unprovisioned device
// which is in the scan response.  To change that, we need to use wiced_bt_ble_scan instead of wiced_bt_ble_observe.
void wiced_bt_ble_set_scan_mode(uint8_t is_active)
{
}
