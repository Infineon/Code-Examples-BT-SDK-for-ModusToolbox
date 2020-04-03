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
 * This file shows how to create a device which implements mesh on/off client model.
 */

#ifdef WICED_BT_MESH_MODEL_LIGHT_LIGHTNESS_CLIENT_INCLUDED

#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_mesh_app.h"

#ifdef HCI_CONTROL
#include "wiced_transport.h"
#include "hci_control_api.h"
#endif

#define MESH_LIGHT_LIGHTNESS_CLIENT_ELEMENT_INDEX   0

/******************************************************
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
void mesh_light_lightness_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void mesh_light_lightness_client_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lightness_client_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lightness_client_linear_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lightness_client_linear_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lightness_client_last_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lightness_client_default_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lightness_client_default_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lightness_client_range_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lightness_client_range_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lightness_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lightness_status_data_t *p_data);
static void mesh_light_lightness_linear_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lightness_status_data_t *p_data);
static void mesh_light_lightness_last_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lightness_last_data_t *p_data);
static void mesh_light_lightness_default_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lightness_default_data_t *p_data);
static void mesh_light_lightness_range_hci_event_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lightness_range_status_data_t *p_data);

/******************************************************
 *          Variables Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
#if 0
void mesh_app_init(wiced_bool_t is_provisioned)
{
    wiced_bt_mesh_model_light_lightness_client_init(MESH_LIGHT_LIGHTNESS_CLIENT_ELEMENT_INDEX, mesh_light_lightness_client_message_handler, is_provisioned);
}
#endif

/*
 * Process event received from the Light Lightness Server.
 */
void mesh_light_lightness_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
    wiced_bt_mesh_light_lightness_status_data_t *p_status_data;
    wiced_bt_mesh_light_lightness_last_data_t *p_last_status_data;
    wiced_bt_mesh_light_lightness_default_data_t *p_default_status_data;
    wiced_bt_mesh_light_lightness_range_status_data_t *p_range_status_data;
#if defined HCI_CONTROL
    wiced_bt_mesh_hci_event_t *p_hci_event;
#endif

    WICED_BT_TRACE("lightness clt msg:%d\n", event);

    switch (event)
    {
    case WICED_BT_MESH_TX_COMPLETE:
        WICED_BT_TRACE("tx complete status:%d\n", p_event->tx_status);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            wiced_bt_mesh_send_hci_tx_complete(p_hci_event, p_event);
#endif
        break;

    case WICED_BT_MESH_LIGHT_LIGHTNESS_STATUS:
        p_status_data = (wiced_bt_mesh_light_lightness_status_data_t *)p_data;
        WICED_BT_TRACE("light:%d target:%d remaining time:%d\n",
                p_status_data->present, p_status_data->target, p_status_data->remaining_time);

#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_light_lightness_hci_event_send(p_hci_event, p_status_data);
#endif
        break;

    case WICED_BT_MESH_LIGHT_LIGHTNESS_LINEAR_STATUS:
        p_status_data = (wiced_bt_mesh_light_lightness_status_data_t *)p_data;
        WICED_BT_TRACE("linear light:%d target:%d remaining time:%d\n",
                p_status_data->present, p_status_data->target, p_status_data->remaining_time);

#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_light_lightness_linear_hci_event_send(p_hci_event, p_status_data);
#endif
        break;

    case WICED_BT_MESH_LIGHT_LIGHTNESS_LAST_STATUS:
        p_last_status_data = (wiced_bt_mesh_light_lightness_last_data_t *)p_data;
        WICED_BT_TRACE("last light:%d\n", p_last_status_data->last_level);

#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_light_lightness_last_hci_event_send(p_hci_event, p_last_status_data);
#endif
        break;

    case WICED_BT_MESH_LIGHT_LIGHTNESS_DEFAULT_STATUS:
        p_default_status_data = (wiced_bt_mesh_light_lightness_default_data_t *)p_data;
        WICED_BT_TRACE("default light:%d\n", p_default_status_data->default_level);

#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_light_lightness_default_hci_event_send(p_hci_event, p_default_status_data);
#endif
        break;

    case WICED_BT_MESH_LIGHT_LIGHTNESS_RANGE_STATUS:
        p_range_status_data = (wiced_bt_mesh_light_lightness_range_status_data_t *)p_data;
        WICED_BT_TRACE("status:%d min light:%d max:%d\n", p_range_status_data->status, p_range_status_data->min_level, p_range_status_data->max_level);

#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_light_lightness_range_hci_event_status_send(p_hci_event, p_range_status_data);
#endif
        break;

    }
    wiced_bt_mesh_release_event(p_event);
}

/*
 * In 2 chip solutions MCU can send commands to change onoff state.
 */
uint32_t mesh_light_lightness_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
#ifdef HCI_CONTROL
    wiced_bt_mesh_event_t *p_event;

    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LIGHTNESS_GET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LIGHTNESS_SET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LIGHTNESS_LINEAR_GET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LIGHTNESS_LINEAR_SET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LIGHTNESS_LAST_GET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LIGHTNESS_DEFAULT_GET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LIGHTNESS_DEFAULT_SET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LIGHTNESS_RANGE_GET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LIGHTNESS_RANGE_SET:
        break;

    default:
        return WICED_FALSE;
    }
    p_event = wiced_bt_mesh_create_event_from_wiced_hci(opcode, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LIGHTNESS_CLNT, &p_data, &length);
    if (p_event == NULL)
    {
        WICED_BT_TRACE("lightness bad hdr\n");
        return WICED_TRUE;
    }
    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LIGHTNESS_GET:
        mesh_light_lightness_client_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LIGHTNESS_SET:
        mesh_light_lightness_client_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LIGHTNESS_LINEAR_GET:
        mesh_light_lightness_client_linear_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LIGHTNESS_LINEAR_SET:
        mesh_light_lightness_client_linear_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LIGHTNESS_LAST_GET:
        mesh_light_lightness_client_last_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LIGHTNESS_DEFAULT_GET:
        mesh_light_lightness_client_default_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LIGHTNESS_DEFAULT_SET:
        mesh_light_lightness_client_default_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LIGHTNESS_RANGE_GET:
        mesh_light_lightness_client_range_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LIGHTNESS_RANGE_SET:
        mesh_light_lightness_client_range_set(p_event, p_data, length);
        break;
    }
#endif
    return WICED_TRUE;
}

/*
 * Send Light Lightness Get command
 */
void mesh_light_lightness_client_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_light_lightness_client_send_get(p_event);
}

/*
 * Send Light Lightness Set command
 */
void mesh_light_lightness_client_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_light_lightness_actual_set_t set_data;

    STREAM_TO_UINT16(set_data.lightness_actual, p_data);
    STREAM_TO_UINT32(set_data.transition_time, p_data);
    STREAM_TO_UINT16(set_data.delay, p_data);

    wiced_bt_mesh_model_light_lightness_client_send_set(p_event, &set_data);
}

/*
 * Send Light Lightness Linear Get command
 */
void mesh_light_lightness_client_linear_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_light_lightness_client_send_linear_get(p_event);
}

/*
 * Send Light Lightness Linear Set command
 */
void mesh_light_lightness_client_linear_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_light_lightness_linear_set_t set_data;

    STREAM_TO_UINT16(set_data.lightness_linear, p_data);
    STREAM_TO_UINT32(set_data.transition_time, p_data);
    STREAM_TO_UINT16(set_data.delay, p_data);

    wiced_bt_mesh_model_light_lightness_client_send_linear_set(p_event, &set_data);
}

/*
 * Send Light Lightness Last Get command
 */
void mesh_light_lightness_client_last_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_light_lightness_client_send_last_get(p_event);
}

/*
 * Send Light Lightness Default Get command
 */
void mesh_light_lightness_client_default_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_light_lightness_client_send_default_get(p_event);
}

/*
 * Send Light Lightness Default Set command
 */
void mesh_light_lightness_client_default_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_light_lightness_default_data_t set_data;

    STREAM_TO_UINT16(set_data.default_level, p_data);

    wiced_bt_mesh_model_light_lightness_client_send_default_set(p_event, &set_data);
}

/*
 * Send Light Lightness Range Get command
 */
void mesh_light_lightness_client_range_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_light_lightness_client_send_range_get(p_event);
}

/*
 * Send Light Lightness Range Set command
 */
void mesh_light_lightness_client_range_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_light_lightness_range_set_data_t set_data;

    STREAM_TO_UINT16(set_data.min_level, p_data);
    STREAM_TO_UINT16(set_data.max_level, p_data);

    wiced_bt_mesh_model_light_lightness_client_send_range_set(p_event, &set_data);
}

#ifdef HCI_CONTROL
/*
 * Send Light Lightness Status event over transport
 */
void mesh_light_lightness_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lightness_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, p_data->present);
    UINT16_TO_STREAM(p, p_data->target);
    UINT32_TO_STREAM(p, p_data->remaining_time);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LIGHT_LIGHTNESS_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Light Lightness Linear Status event over transport
 */
void mesh_light_lightness_linear_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lightness_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, p_data->present);
    UINT16_TO_STREAM(p, p_data->target);
    UINT32_TO_STREAM(p, p_data->remaining_time);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LIGHT_LIGHTNESS_LINEAR_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Light Lightness Last Status event over transport
 */
void mesh_light_lightness_last_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lightness_last_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, p_data->last_level);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LIGHT_LIGHTNESS_LAST_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Light Lightness Default Status event over transport
 */
void mesh_light_lightness_default_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lightness_default_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, p_data->default_level);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LIGHT_LIGHTNESS_DEFAULT_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Light Lightness Range Status event over transport
 */
void mesh_light_lightness_range_hci_event_status_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lightness_range_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT8_TO_STREAM(p, p_data->status);
    UINT16_TO_STREAM(p, p_data->min_level);
    UINT16_TO_STREAM(p, p_data->max_level);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LIGHT_LIGHTNESS_RANGE_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

#endif

#endif
