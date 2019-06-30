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
 * This file shows how to create a device which implements mesh Mesh Light CTL Client.
 */

#ifdef WICED_BT_MESH_MODEL_LIGHT_CTL_CLIENT_INCLUDED

#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_mesh_app.h"

#ifdef HCI_CONTROL
#include "wiced_transport.h"
#include "hci_control_api.h"
#endif

#define MESH_LIGHT_CTL_CLIENT_ELEMENT_INDEX   0

/******************************************************
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
void mesh_light_ctl_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void mesh_light_ctl_client_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_ctl_client_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_ctl_client_temperature_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_ctl_client_temperature_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_ctl_client_temperature_range_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_ctl_client_temperature_range_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_ctl_client_default_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_ctl_client_default_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_ctl_hci_event_send_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_ctl_status_data_t *p_data);
static void mesh_light_ctl_hci_event_send_temperature_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_ctl_status_data_t *p_data);
static void mesh_light_ctl_hci_event_send_default_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_ctl_default_data_t *p_data);
static void mesh_light_ctl_hci_event_send_temperature_range_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_ctl_temperature_range_status_data_t *p_data);

/******************************************************
 *          Variables Definitions
 ******************************************************/

/*
 * Process event received from the Light CTL Server.
 */
void mesh_light_ctl_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
#if defined HCI_CONTROL
    wiced_bt_mesh_hci_event_t *p_hci_event;
#endif
    wiced_bt_mesh_light_ctl_status_data_t *p_status;
    wiced_bt_mesh_light_ctl_temperature_range_status_data_t *p_temperature_range_status;
    wiced_bt_mesh_light_ctl_default_data_t *p_default_status;

    WICED_BT_TRACE("pwr lvl clt msg:%d\n", event);

    switch (event)
    {
    case WICED_BT_MESH_TX_COMPLETE:
        WICED_BT_TRACE("tx complete status:%d\n", p_event->tx_status);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            wiced_bt_mesh_send_hci_tx_complete(p_hci_event, p_event);
#endif
        break;

    case WICED_BT_MESH_LIGHT_CTL_STATUS:
        p_status = (wiced_bt_mesh_light_ctl_status_data_t *)p_data;
        WICED_BT_TRACE("light status light/temp present:%d/%d target:%d/%d remain time:%d\n",
                p_status->present.lightness, p_status->present.temperature,
                p_status->target.lightness, p_status->target.temperature, p_status->remaining_time);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_light_ctl_hci_event_send_status(p_hci_event, p_status);
#endif
        break;

    case WICED_BT_MESH_LIGHT_CTL_DEFAULT_STATUS:
        p_default_status = (wiced_bt_mesh_light_ctl_default_data_t *)p_data;
        WICED_BT_TRACE("default lightness:%d temp:%d uv:%d\n",
                p_default_status->default_status.lightness, p_default_status->default_status.temperature,
                p_default_status->default_status.delta_uv);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_light_ctl_hci_event_send_default_status(p_hci_event, p_default_status);
#endif
        break;

    case WICED_BT_MESH_LIGHT_CTL_TEMPERATURE_STATUS:
        p_status = (wiced_bt_mesh_light_ctl_status_data_t *)p_data;
        WICED_BT_TRACE("temp status: temp/UV present %d/%d target:%d/%d time:%d\n",
                p_status->present.temperature, p_status->present.delta_uv,
                p_status->target.temperature, p_status->target.delta_uv,
                p_status->remaining_time);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_light_ctl_hci_event_send_temperature_status(p_hci_event, p_status);
#endif
        break;

    case WICED_BT_MESH_LIGHT_CTL_TEMPERATURE_RANGE_STATUS:
        p_temperature_range_status = (wiced_bt_mesh_light_ctl_temperature_range_status_data_t *)p_data;
        WICED_BT_TRACE("temp range status: status %d min/max:%d/%d\n", p_temperature_range_status->status,
                p_temperature_range_status->min_level, p_temperature_range_status->max_level);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_light_ctl_hci_event_send_temperature_range_status(p_hci_event, p_temperature_range_status);
#endif
        break;

    default:
        break;
    }
    wiced_bt_mesh_release_event(p_event);
}


/*
 * In 2 chip solutions MCU can send commands to change onoff state.
 */
uint32_t mesh_light_ctl_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
#ifdef HCI_CONTROL
    wiced_bt_mesh_event_t *p_event;

    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_LIGHT_CTL_GET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_CTL_SET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_CTL_TEMPERATURE_GET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_CTL_TEMPERATURE_SET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_CTL_TEMPERATURE_RANGE_GET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_CTL_TEMPERATURE_RANGE_SET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_CTL_DEFAULT_GET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_CTL_DEFAULT_SET:
        break;

    default:
        return WICED_FALSE;
    }
    p_event = wiced_bt_mesh_create_event_from_wiced_hci(opcode, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_CTL_CLNT, &p_data, &length);
    if (p_event == NULL)
    {
        WICED_BT_TRACE("ctl bad hdr\n");
        return WICED_TRUE;
    }
    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_LIGHT_CTL_GET:
        mesh_light_ctl_client_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_CTL_SET:
        mesh_light_ctl_client_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_CTL_TEMPERATURE_GET:
        mesh_light_ctl_client_temperature_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_CTL_TEMPERATURE_SET:
        mesh_light_ctl_client_temperature_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_CTL_TEMPERATURE_RANGE_GET:
        mesh_light_ctl_client_temperature_range_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_CTL_TEMPERATURE_RANGE_SET:
        mesh_light_ctl_client_temperature_range_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_CTL_DEFAULT_GET:
        mesh_light_ctl_client_default_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_CTL_DEFAULT_SET:
        mesh_light_ctl_client_default_set(p_event, p_data, length);
        break;
}
#endif
    return WICED_TRUE;
}

/*
 * Send Light CTL get command
 */
void mesh_light_ctl_client_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_light_ctl_client_send_get(p_event);
}

/*
 * Send Light CTL set command
 */
void mesh_light_ctl_client_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_light_ctl_set_t set_data;

    STREAM_TO_UINT16(set_data.target.lightness, p_data);
    STREAM_TO_UINT16(set_data.target.temperature, p_data);
    STREAM_TO_UINT16(set_data.target.delta_uv, p_data);
    STREAM_TO_UINT32(set_data.transition_time, p_data);
    STREAM_TO_UINT16(set_data.delay, p_data);

    wiced_bt_mesh_model_light_ctl_client_send_set(p_event, &set_data);
}

/*
 * Send Light CTL Temperature get command
 */
void mesh_light_ctl_client_temperature_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_light_ctl_client_send_temperature_get(p_event);
}

/*
 * Send Light CTL Temperature set command
 */
void mesh_light_ctl_client_temperature_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_light_ctl_temperature_set_t set_data;

    STREAM_TO_UINT16(set_data.target_temperature, p_data);
    STREAM_TO_UINT16(set_data.target_delta_uv, p_data);
    STREAM_TO_UINT32(set_data.transition_time, p_data);
    STREAM_TO_UINT16(set_data.delay, p_data);

    wiced_bt_mesh_model_light_ctl_client_send_temperature_set(p_event, &set_data);
}

/*
 * Send Light CTL Get Default command
 */
void mesh_light_ctl_client_default_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_light_ctl_client_send_default_get(p_event);
}

/*
 * Send power Light CTL Set Default command
 */
void mesh_light_ctl_client_default_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_light_ctl_default_data_t set_data;

    STREAM_TO_UINT16(set_data.default_status.lightness, p_data);
    STREAM_TO_UINT16(set_data.default_status.temperature, p_data);
    STREAM_TO_UINT16(set_data.default_status.delta_uv, p_data);

    wiced_bt_mesh_model_light_ctl_client_send_default_set(p_event, &set_data);
}

/*
 * Send Light CTL Temperature Range Get command
 */
void mesh_light_ctl_client_temperature_range_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_light_ctl_client_send_temperature_range_get(p_event);
}

/*
 * Send Light CTL Temperature Range Set command
 */
void mesh_light_ctl_client_temperature_range_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_light_ctl_temperature_range_data_t set_data;

    STREAM_TO_UINT16(set_data.min_level, p_data);
    STREAM_TO_UINT16(set_data.max_level, p_data);

    wiced_bt_mesh_model_light_ctl_client_send_temperature_range_set(p_event, &set_data);
}

#ifdef HCI_CONTROL
/*
 * Send Power Level Status event over transport
 */
void mesh_light_ctl_hci_event_send_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_ctl_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, p_data->present.lightness);
    UINT16_TO_STREAM(p, p_data->present.temperature);
    UINT16_TO_STREAM(p, p_data->target.lightness);
    UINT16_TO_STREAM(p, p_data->target.temperature);
    UINT32_TO_STREAM(p, p_data->remaining_time);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LIGHT_CTL_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Light CTL Temperature Status event over transport
 */
void mesh_light_ctl_hci_event_send_temperature_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_ctl_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, p_data->present.temperature);
    UINT16_TO_STREAM(p, p_data->present.delta_uv);
    UINT16_TO_STREAM(p, p_data->target.temperature);
    UINT16_TO_STREAM(p, p_data->target.delta_uv);
    UINT32_TO_STREAM(p, p_data->remaining_time);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LIGHT_CTL_TEMPERATURE_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
* Send Light CTL Temperature Range Status event over transport
*/
void mesh_light_ctl_hci_event_send_temperature_range_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_ctl_temperature_range_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT8_TO_STREAM(p, p_data->status);
    UINT16_TO_STREAM(p, p_data->min_level);
    UINT16_TO_STREAM(p, p_data->max_level);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LIGHT_CTL_TEMPERATURE_RANGE_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Default Light CTL Default Status event over transport
 */
void mesh_light_ctl_hci_event_send_default_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_ctl_default_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, p_data->default_status.lightness);
    UINT16_TO_STREAM(p, p_data->default_status.temperature);
    UINT16_TO_STREAM(p, p_data->default_status.delta_uv);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LIGHT_CTL_DEFAULT_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

#endif

#endif
