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
 * This file shows how to create a device which implements mesh scheduler client.
 */
#ifdef WICED_BT_MESH_MODEL_SCHEDULER_CLIENT_INCLUDED

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
 *          Function Prototypes
 ******************************************************/
uint32_t mesh_scheduler_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
void mesh_scheduler_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void mesh_scheduler_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_scheduler_action_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_scheduler_action_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_scheduler_hci_scheduler_status_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_scheduler_status_t *p_data);
static void mesh_scheduler_hci_scheduler_action_status_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_scheduler_action_data_t *p_data);

/******************************************************
 *               Function Definitions
 ******************************************************/
/*
 * Process event received from the scheduler Server.
 */
void mesh_scheduler_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
    wiced_bt_mesh_scheduler_status_t      *p_scheduler_status;
    wiced_bt_mesh_scheduler_action_data_t *p_scheduler_action_status;
#if defined HCI_CONTROL
    wiced_bt_mesh_hci_event_t *p_hci_event = wiced_bt_mesh_create_hci_event(p_event);
    if (p_hci_event == NULL)
        return;
#endif

    WICED_BT_TRACE("scheduler clt msg:%d\n", event);

    switch (event)
    {
    case WICED_BT_MESH_TX_COMPLETE:
        WICED_BT_TRACE("tx complete status:%d\n", p_event->tx_status);
#if defined HCI_CONTROL
        wiced_bt_mesh_send_hci_tx_complete(p_hci_event, p_event);
#endif
        break;

    case WICED_BT_MESH_SCHEDULER_STATUS:
        p_scheduler_status = (wiced_bt_mesh_scheduler_status_t *)p_data;
#if defined HCI_CONTROL
        mesh_scheduler_hci_scheduler_status_event_send(p_hci_event, p_scheduler_status);
#endif
        break;

    case WICED_BT_MESH_SCHEDULER_ACTION_STATUS:
        p_scheduler_action_status = (wiced_bt_mesh_scheduler_action_data_t *)p_data;
#if defined HCI_CONTROL
        mesh_scheduler_hci_scheduler_action_status_event_send(p_hci_event, p_scheduler_action_status);
#endif
        break;

    default:
        WICED_BT_TRACE("unknown\n");
        break;
    }
    wiced_bt_mesh_release_event(p_event);
}

/*
 * In 2 chip solutions MCU can send commands to change scheduler state.
 */
uint32_t mesh_scheduler_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_event_t *p_event;

    WICED_BT_TRACE("[%s] cmd_opcode 0x%02x\n", __FUNCTION__, opcode);

    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_SCHEDULER_GET:
    case HCI_CONTROL_MESH_COMMAND_SCHEDULER_ACTION_GET:
    case HCI_CONTROL_MESH_COMMAND_SCHEDULER_ACTION_SET:
        break;

    default:
        WICED_BT_TRACE("unknown\n");
        return WICED_FALSE;
    }
    p_event = wiced_bt_mesh_create_event_from_wiced_hci(opcode, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_CLNT, &p_data, &length);
    if (p_event == NULL)
    {
        WICED_BT_TRACE("bad hdr\n");
        return WICED_TRUE;
    }
    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_SCHEDULER_GET:
        mesh_scheduler_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_SCHEDULER_ACTION_GET:
        mesh_scheduler_action_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_SCHEDULER_ACTION_SET:
        mesh_scheduler_action_set(p_event, p_data, length);
        break;
    }
    return WICED_TRUE;
}

/*
 * Send Scheduler Get command
 */
void mesh_scheduler_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_scheduler_client_send_get(p_event);
}

/*
 * Send Scheduler Action Get command
 */
void mesh_scheduler_action_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_scheduler_action_get_t request;

    STREAM_TO_UINT8(request.action_number, p_data);

    wiced_bt_mesh_model_scheduler_client_send_action_get(p_event, &request);
}

/*
 * Send Scheduler Action Get command
 */
void mesh_scheduler_action_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_scheduler_action_data_t request;

    STREAM_TO_UINT8(request.action_number, p_data);
    STREAM_TO_UINT8(request.year, p_data);
    STREAM_TO_UINT16(request.month, p_data);
    STREAM_TO_UINT8(request.day, p_data);
    STREAM_TO_UINT8(request.hour, p_data);
    STREAM_TO_UINT8(request.minute, p_data);
    STREAM_TO_UINT8(request.second, p_data);
    STREAM_TO_UINT8(request.day_of_week, p_data);
    STREAM_TO_UINT8(request.action, p_data);
    STREAM_TO_UINT32(request.transition_time, p_data);
    STREAM_TO_UINT16(request.scene_number, p_data);

    wiced_bt_mesh_model_scheduler_client_send_action_set(p_event, &request);
}

#ifdef HCI_CONTROL
/*
 * Send Scheduler Status event over transport
 */
void mesh_scheduler_hci_scheduler_status_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_scheduler_status_t *p_data)
{
    int i;
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, p_data->actions);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_SCHEDULER_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send scheduler Status event over transport
 */
void mesh_scheduler_hci_scheduler_action_status_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_scheduler_action_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT8_TO_STREAM(p, p_data->action_number);
    UINT8_TO_STREAM(p, p_data->year);
    UINT16_TO_STREAM(p, p_data->month);
    UINT8_TO_STREAM(p, p_data->day);
    UINT8_TO_STREAM(p, p_data->hour);
    UINT8_TO_STREAM(p, p_data->minute);
    UINT8_TO_STREAM(p, p_data->second);
    UINT8_TO_STREAM(p, p_data->day_of_week);
    UINT8_TO_STREAM(p, p_data->action);
    UINT32_TO_STREAM(p, p_data->transition_time);
    UINT16_TO_STREAM(p, p_data->scene_number);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_SCHEDULER_ACTION_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

#endif


#endif
