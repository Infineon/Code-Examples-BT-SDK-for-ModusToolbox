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
 * This file shows how to create a device which implements mesh scene client.
 */

#ifdef WICED_BT_MESH_MODEL_SCENE_CLIENT_INCLUDED

#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_mesh_app.h"

#ifdef HCI_CONTROL
#include "wiced_transport.h"
#include "hci_control_api.h"
#endif

/******************************************************
 *          Constants
 ******************************************************/

/******************************************************
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
void mesh_scene_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void mesh_scene_client_store(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_scene_client_recall(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_scene_client_delete(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_scene_client_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_scene_client_register_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_scene_hci_register_status_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_scene_register_status_data_t *p_data);
static void mesh_scene_hci_status_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_scene_status_data_t *p_data);

/******************************************************
 *          Variables Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 * Process event received from the scene Server.
 */
void mesh_scene_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
    wiced_bt_mesh_scene_status_data_t          *p_scene_status;
    wiced_bt_mesh_scene_register_status_data_t *p_register_status;
#if defined HCI_CONTROL
    wiced_bt_mesh_hci_event_t *p_hci_event;
#endif

    WICED_BT_TRACE("scene clt msg:%d\n", event);

    switch (event)
    {
    case WICED_BT_MESH_TX_COMPLETE:
        WICED_BT_TRACE("tx complete status:%d\n", p_event->tx_status);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            wiced_bt_mesh_send_hci_tx_complete(p_hci_event, p_event);
#endif
        break;

    case WICED_BT_MESH_SCENE_REGISTER_STATUS:
        p_register_status = (wiced_bt_mesh_scene_register_status_data_t *)p_data;
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_scene_hci_register_status_event_send(p_hci_event, p_register_status);
#endif
        break;

    case WICED_BT_MESH_SCENE_STATUS:
        p_scene_status = (wiced_bt_mesh_scene_status_data_t *)p_data;
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_scene_hci_status_event_send(p_hci_event, p_scene_status);
#endif
        break;

    default:
        WICED_BT_TRACE("unknown\n");
        break;
    }
    wiced_bt_mesh_release_event(p_event);
}

/*
 * In 2 chip solutions MCU can send commands to change scene state.
 */
uint32_t mesh_scene_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_event_t *p_event;

    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_SCENE_STORE:
    case HCI_CONTROL_MESH_COMMAND_SCENE_RECALL:
    case HCI_CONTROL_MESH_COMMAND_SCENE_GET:
    case HCI_CONTROL_MESH_COMMAND_SCENE_REGISTER_GET:
    case HCI_CONTROL_MESH_COMMAND_SCENE_DELETE:
        break;

    default:
        return WICED_FALSE;
    }
    p_event = wiced_bt_mesh_create_event_from_wiced_hci(opcode, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_CLNT, &p_data, &length);
    if (p_event == NULL)
    {
        WICED_BT_TRACE("bad hdr\n");
        return WICED_TRUE;
    }
    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_SCENE_STORE:
        mesh_scene_client_store(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_SCENE_RECALL:
        mesh_scene_client_recall(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_SCENE_GET:
        mesh_scene_client_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_SCENE_REGISTER_GET:
        mesh_scene_client_register_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_SCENE_DELETE:
        mesh_scene_client_delete(p_event, p_data, length);
        break;
    }
    return WICED_TRUE;
}

/*
 * Send Scene Store command
 */
void mesh_scene_client_store(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_scene_request_t request;

    WICED_BT_TRACE("scene store elem:%04x dst:%04x key:%04x\n", p_event->element_idx, p_event->dst, p_event->app_key_idx);

    request.type = WICED_BT_MESH_SCENE_REQUEST_TYPE_STORE;
    STREAM_TO_UINT16(request.scene_number, p_data);

    wiced_bt_mesh_model_scene_client_send_request(p_event, &request);
}

/*
 * Send Scene Recall command
 */
void mesh_scene_client_recall(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_scene_recall_t request;

    WICED_BT_TRACE("scene recall elem:%04x dst:%04x key:%04x\n", p_event->element_idx, p_event->dst, p_event->app_key_idx);

    STREAM_TO_UINT16(request.scene_number, p_data);
    STREAM_TO_UINT32(request.transition_time, p_data);
    STREAM_TO_UINT16(request.delay, p_data);

    wiced_bt_mesh_model_scene_client_send_recall(p_event, &request);
}

/*
 * Send Scene Delete command
 */
void mesh_scene_client_delete(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_scene_request_t request;

    WICED_BT_TRACE("scene delete elem:%04x dst:%04x key:%04x\n", p_event->element_idx, p_event->dst, p_event->app_key_idx);

    request.type = WICED_BT_MESH_SCENE_REQUEST_TYPE_DELETE;
    STREAM_TO_UINT16(request.scene_number, p_data);

    wiced_bt_mesh_model_scene_client_send_request(p_event, &request);
}

/*
 * Send Scene Get command
 */
void mesh_scene_client_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_scene_request_t request;

    WICED_BT_TRACE("scene get elem:%04x dst:%04x key:%04x\n", p_event->element_idx, p_event->dst, p_event->app_key_idx);

    request.type = WICED_BT_MESH_SCENE_REQUEST_TYPE_GET;

    wiced_bt_mesh_model_scene_client_send_request(p_event, &request);
}

/*
 * Send Scene Register Get command
 */
void mesh_scene_client_register_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    WICED_BT_TRACE("scene register get elem:%04x dst:%04x key:%04x\n", p_event->element_idx, p_event->dst, p_event->app_key_idx);

    wiced_bt_mesh_model_scene_client_send_register_get(p_event);
}

#ifdef HCI_CONTROL
/*
 * Send Scene Status event over transport
 */
void mesh_scene_hci_register_status_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_scene_register_status_data_t *p_data)
{
    int i;
    uint8_t *p = p_hci_event->data;

    UINT8_TO_STREAM(p, p_data->status_code);
    UINT16_TO_STREAM(p, p_data->current_scene);
    for (i = 0; i < p_data->scene_num; i++)
        UINT16_TO_STREAM(p, p_data->scene[i]);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_SCENE_REGISTER_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send scene Status event over transport
 */
void mesh_scene_hci_status_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_scene_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT8_TO_STREAM(p, p_data->status_code);
    UINT16_TO_STREAM(p, p_data->current_scene);
    UINT16_TO_STREAM(p, p_data->target_scene);
    UINT32_TO_STREAM(p, p_data->remaining_time);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_SCENE_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

#endif

#endif
