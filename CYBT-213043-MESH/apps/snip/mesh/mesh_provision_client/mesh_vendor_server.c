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
 *
 * This file shows how to create a device supporting a vendor specific model.
 */
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_timer.h"
#include "wiced_bt_mesh_client.h"
#include "wiced_bt_mesh_app.h"

#ifdef HCI_CONTROL
#include "wiced_transport.h"
#include "hci_control_api.h"
#endif
#include "mesh_vendor_server.h"

#define MESH_VENDOR_SERVER_ELEMENT_INDEX   0

/******************************************************
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
static void mesh_vendor_server_send_status(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);
static void mesh_vendor_server_process_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);
static void mesh_vendor_server_process_data(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

#ifdef HCI_CONTROL
static void mesh_vendor_hci_event_send_data(wiced_bt_mesh_hci_event_t *p_hci_event, uint16_t opcode, uint8_t *p_data, uint16_t data_len);
#endif

/******************************************************
 *          Variables Definitions
 ******************************************************/
/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 * This function is called when core receives a valid message for the define Vendor
 * Model (MESH_VENDOR_COMPANY_ID/MESH_VENDOR_MODEL_ID) combination.  The function shall return TRUE if it
 * was able to process the message, and FALSE if the message is unknown.  In the latter case the core
 * will call other registered models.
 */
wiced_bool_t mesh_vendor_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len)
{
    switch (p_event->opcode)
    {
    // this is a special case.  The provisioner configured this device to periodically publish
    // information.  Process is the same way as the GET command
    case WICED_BT_MESH_OPCODE_UNKNOWN:
    case MESH_VENDOR_OPCODE_GET:
        mesh_vendor_server_process_get(p_event, p_data, data_len);
        break;

    case MESH_VENDOR_OPCODE_SET:
        p_event->reply = WICED_FALSE;
        mesh_vendor_server_process_data(p_event, p_data, data_len);
        break;

    case MESH_VENDOR_OPCODE_SET_UNACKED:
        p_event->reply = WICED_FALSE;
        mesh_vendor_server_process_data(p_event, p_data, data_len);
        break;

    case MESH_VENDOR_OPCODE_STATUS:
        p_event->reply = WICED_FALSE;
        mesh_vendor_server_process_data(p_event, p_data, data_len);
        break;

    default:
        return WICED_FALSE;
    }
    return WICED_TRUE;
}

/*
 * Scene Store Handler.  If the model need to be a part of a scene, store the data in the provided buffer.
 */
uint16_t mesh_vendor_server_scene_store_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len)
{
    // return number of the bytes stored in the buffer.
    return 0;
}

/*
 * Scene Store Handler.  If the model need to be a part of a scene, restore the data from the provided buffer.
 */
uint16_t mesh_vendor_server_scene_recall_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len, uint32_t transition_time, uint32_t delay)
{
    // return number of the bytes read from the buffer.
    return 0;
}

void mesh_vendor_server_process_data(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len)
{
    wiced_bt_mesh_hci_event_t *p_hci_event;

    WICED_BT_TRACE("vs process data from:%04x reply:%04x len:%d\n", p_event->src, p_event->reply, data_len);

    // Because the same app publishes and subscribes the same model, it will receive messages that it
    //sent out.
    if (p_event->src == wiced_bt_mesh_core_get_local_addr())
        return;

#if defined HCI_CONTROL
    p_hci_event = wiced_bt_mesh_create_hci_event(p_event);
    if (p_hci_event != NULL)
    {
        mesh_vendor_hci_event_send_data(p_hci_event, p_event->opcode, p_data, data_len);
    }
#endif
    if (p_event->reply)
    {
        // return the data that we received in the command.  Real app can send anything it wants.
        mesh_vendor_server_send_status(wiced_bt_mesh_create_reply_event(p_event), p_data, data_len);
    }
    else
    {
        wiced_bt_mesh_release_event(p_event);
    }
}

void mesh_vendor_server_process_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len)
{
    mesh_vendor_server_send_status(wiced_bt_mesh_create_reply_event(p_event), (uint8_t *)"Hello", 6);
}

/*
 * In 2 chip solutions MCU can send commands to set vendor specific status.
 */
uint32_t mesh_vendor_service_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_event_t *p_event;
    uint16_t dst         = p_data[0] + (p_data[1] << 8);
    uint16_t app_key_idx = p_data[2] + (p_data[3] << 8);
    uint8_t  element_idx = p_data[4];

    if (opcode != HCI_CONTROL_MESH_COMMAND_VENDOR_DATA)
        return WICED_FALSE;

    p_event = wiced_bt_mesh_create_event(element_idx, MESH_VENDOR_COMPANY_ID, MESH_VENDOR_MODEL_ID, dst, app_key_idx);
    if (p_event == NULL)
    {
        WICED_BT_TRACE("app_proc_rx_cmd: no mem\n");
        return WICED_TRUE;
    }
    mesh_vendor_server_send_status(p_event, p_data + 6, length - 6);
    return WICED_TRUE;
}

/*
 * Send Vendor Data status message to the Client
 */
void mesh_vendor_server_send_status(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len)
{
    p_event->opcode = MESH_VENDOR_OPCODE_STATUS;
    wiced_bt_mesh_client_vendor_data(p_event, p_data, data_len);
}

#ifdef HCI_CONTROL
/*
 * Send Vendor Data received from the mesh over transport
 */
void mesh_vendor_hci_event_send_data(wiced_bt_mesh_hci_event_t *p_hci_event, uint16_t opcode, uint8_t *p_data, uint16_t data_len)
{
    uint8_t *p = p_hci_event->data;

    ARRAY_TO_STREAM(p, p_data, data_len);

    mesh_transport_send_data(opcode == MESH_VENDOR_OPCODE_STATUS ?
            HCI_CONTROL_MESH_EVENT_VENDOR_STATUS : HCI_CONTROL_MESH_EVENT_VENDOR_SET, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

#endif
