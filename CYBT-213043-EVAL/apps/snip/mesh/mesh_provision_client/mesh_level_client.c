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
 * This file shows how to create a device which implements mesh level client application.
 */

#ifdef WICED_BT_MESH_MODEL_LEVEL_CLIENT_INCLUDED

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
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
void mesh_level_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_level_status_data_t *p_data);
static void mesh_level_client_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_level_client_set_level(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_level_client_set_delta(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_level_client_set_move(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_level_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_level_status_data_t *p_data);

/******************************************************
 *          Variables Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
/*
* Process event received from the Level Server.
*/
void mesh_level_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_level_status_data_t *p_data)
{
#if defined HCI_CONTROL
    wiced_bt_mesh_hci_event_t *p_hci_event;
#endif
    WICED_BT_TRACE("level clt msg:%d\n", event);

    switch (event)
    {
    case WICED_BT_MESH_TX_COMPLETE:
        WICED_BT_TRACE("tx complete status:%d\n", p_event->tx_status);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            wiced_bt_mesh_send_hci_tx_complete(p_hci_event, p_event);
#endif
        break;

    case WICED_BT_MESH_LEVEL_STATUS:
        WICED_BT_TRACE("level present:%d target:%d remaining time:%d\n",
            p_data->present_level, p_data->target_level, p_data->remaining_time);

#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_level_hci_event_send(p_hci_event, p_data);
#endif
        break;

    default:
        break;
    }
    wiced_bt_mesh_release_event(p_event);
}

/*
* In 2 chip solutions MCU can send commands to get or change level state of another device.
*/
uint32_t mesh_level_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
#ifdef HCI_CONTROL
    wiced_bt_mesh_event_t *p_event;

    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_LEVEL_GET:
    case HCI_CONTROL_MESH_COMMAND_LEVEL_SET:
    case HCI_CONTROL_MESH_COMMAND_LEVEL_DELTA_SET:
    case HCI_CONTROL_MESH_COMMAND_LEVEL_MOVE_SET:
        break;

    default:
        return WICED_FALSE;
    }
    p_event = wiced_bt_mesh_create_event_from_wiced_hci(opcode, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_CLNT, &p_data, &length);
    if (p_event == NULL)
    {
        WICED_BT_TRACE("level bad hdr\n");
        return WICED_TRUE;
    }
    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_LEVEL_GET:
        mesh_level_client_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LEVEL_SET:
        mesh_level_client_set_level(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LEVEL_DELTA_SET:
        mesh_level_client_set_delta(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LEVEL_MOVE_SET:
        mesh_level_client_set_move(p_event, p_data, length);
        break;
    }
#endif
    return WICED_TRUE;
}

/*
 * Send level get command
 */
void mesh_level_client_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_level_client_send_get(p_event);
}

/*
 * Send level set command. This command will likely be used in the controlling application
 * where application remembers the target Level state.
 */
void mesh_level_client_set_level(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_level_set_level_t set_data;

    STREAM_TO_UINT16(set_data.level, p_data);
    STREAM_TO_UINT32(set_data.transition_time, p_data);
    STREAM_TO_UINT16(set_data.delay, p_data);

    wiced_bt_mesh_model_level_client_send_set(p_event, &set_data);
}

/*
 * Send level delta command. This command will likely be used in the controlling application
 * where application does not remember the target Level state. The instant command is just to
 * adjust level.
 */
void mesh_level_client_set_delta(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_level_set_delta_t set_data;

    STREAM_TO_UINT32(set_data.delta, p_data);
    STREAM_TO_UINT8(set_data.continuation, p_data);
    STREAM_TO_UINT32(set_data.transition_time, p_data);
    STREAM_TO_UINT16(set_data.delay, p_data);

    wiced_bt_mesh_model_level_client_send_delta_set(p_event, &set_data);
}

/*
 * Send move set command
 */
void mesh_level_client_set_move(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_level_set_move_t set_data;

    STREAM_TO_UINT16(set_data.delta, p_data);
    STREAM_TO_UINT32(set_data.transition_time, p_data);
    STREAM_TO_UINT16(set_data.delay, p_data);

    wiced_bt_mesh_model_level_client_send_move_set(p_event, &set_data);
}

#ifdef HCI_CONTROL
/*
* Send Level Status event over transport
*/
void mesh_level_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_level_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, p_data->present_level);
    UINT16_TO_STREAM(p, p_data->target_level);
    UINT32_TO_STREAM(p, p_data->remaining_time);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LEVEL_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}
#endif

#endif
