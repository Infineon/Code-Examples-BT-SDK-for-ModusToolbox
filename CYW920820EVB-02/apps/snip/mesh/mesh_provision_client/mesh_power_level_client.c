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

#ifdef WICED_BT_MESH_MODEL_POWER_LEVEL_CLIENT_INCLUDED

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
uint32_t mesh_power_level_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
void mesh_power_level_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void mesh_power_level_client_level_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_power_level_client_level_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_power_level_client_last_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_power_level_client_default_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_power_level_client_default_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_power_level_client_range_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_power_level_client_range_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);

#ifdef HCI_CONTROL
static void mesh_power_level_hci_event_send_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_power_level_status_data_t *p_data);
static void mesh_power_level_hci_event_send_last_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_power_level_last_data_t *p_data);
static void mesh_power_level_hci_event_send_default_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_power_default_data_t *p_data);
static void mesh_power_level_hci_event_send_range_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_power_range_status_data_t *p_data);
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/
/*
 * Process event received from the onoff Server.
 */
void mesh_power_level_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
#if defined HCI_CONTROL
    wiced_bt_mesh_hci_event_t *p_hci_event;
#endif
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

    case WICED_BT_MESH_POWER_LEVEL_STATUS:
        WICED_BT_TRACE("present:%d target:%d remain time:%d\n",
                ((wiced_bt_mesh_power_level_status_data_t *)p_data)->present_power,
                ((wiced_bt_mesh_power_level_status_data_t *)p_data)->target_power,
                ((wiced_bt_mesh_power_level_status_data_t *)p_data)->remaining_time);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_power_level_hci_event_send_status(p_hci_event, (wiced_bt_mesh_power_level_status_data_t *)p_data);
#endif
        break;

    case WICED_BT_MESH_POWER_LEVEL_LAST_STATUS:
        WICED_BT_TRACE("last:%d\n", ((wiced_bt_mesh_power_level_last_data_t *)p_data)->power);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_power_level_hci_event_send_last_status(p_hci_event, (wiced_bt_mesh_power_level_last_data_t *)p_data);
#endif
        break;

    case WICED_BT_MESH_POWER_LEVEL_DEFAULT_STATUS:
        WICED_BT_TRACE("default:%d\n", ((wiced_bt_mesh_power_default_data_t *)p_data)->power);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_power_level_hci_event_send_default_status(p_hci_event, (wiced_bt_mesh_power_default_data_t *)p_data);
#endif
        break;

    case WICED_BT_MESH_POWER_LEVEL_RANGE_STATUS:
        WICED_BT_TRACE("range min:%d max:%d\n",
                ((wiced_bt_mesh_power_range_status_data_t *)p_data)->power_min,
                ((wiced_bt_mesh_power_range_status_data_t *)p_data)->power_max);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_power_level_hci_event_send_range_status(p_hci_event, (wiced_bt_mesh_power_range_status_data_t *)p_data);
#endif
        break;

    default:
        WICED_BT_TRACE("unknown\n");
        break;
    }
    wiced_bt_mesh_release_event(p_event);
}


/*
 * In 2 chip solutions MCU can send commands to change level state.
 */
uint32_t mesh_power_level_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_event_t *p_event;

    WICED_BT_TRACE("[%s] cmd_opcode 0x%02x\n", __FUNCTION__, opcode);

    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_POWER_LEVEL_GET:
    case HCI_CONTROL_MESH_COMMAND_POWER_LEVEL_SET:
    case HCI_CONTROL_MESH_COMMAND_POWER_LEVEL_LAST_GET:
    case HCI_CONTROL_MESH_COMMAND_POWER_LEVEL_DEFAULT_GET:
    case HCI_CONTROL_MESH_COMMAND_POWER_LEVEL_DEFAULT_SET:
    case HCI_CONTROL_MESH_COMMAND_POWER_LEVEL_RANGE_GET:
    case HCI_CONTROL_MESH_COMMAND_POWER_LEVEL_RANGE_SET:
        break;

    default:
        WICED_BT_TRACE("unknown\n");
        return WICED_FALSE;
    }
    p_event = wiced_bt_mesh_create_event_from_wiced_hci(opcode, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_LEVEL_CLNT, &p_data, &length);
    if (p_event == NULL)
    {
        WICED_BT_TRACE("bad hdr\n");
        return WICED_TRUE;
    }
    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_POWER_LEVEL_GET:
        mesh_power_level_client_level_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_POWER_LEVEL_SET:
        mesh_power_level_client_level_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_POWER_LEVEL_LAST_GET:
        mesh_power_level_client_last_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_POWER_LEVEL_DEFAULT_GET:
        mesh_power_level_client_default_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_POWER_LEVEL_DEFAULT_SET:
        mesh_power_level_client_default_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_POWER_LEVEL_RANGE_GET:
        mesh_power_level_client_range_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_POWER_LEVEL_RANGE_SET:
        mesh_power_level_client_range_set(p_event, p_data, length);
        break;
    }
    return WICED_TRUE;
}

/*
 * Send power onoff get command
 */
void mesh_power_level_client_level_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_power_level_client_send_get(p_event);
}

/*
 * Send power level set command
 */
void mesh_power_level_client_level_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_power_level_set_level_t set_data;

    STREAM_TO_UINT16(set_data.level, p_data);
    STREAM_TO_UINT32(set_data.transition_time, p_data);
    STREAM_TO_UINT16(set_data.delay, p_data);

    wiced_bt_mesh_model_power_level_client_send_set(p_event, &set_data);
}

/*
 * Send power Get Last Power Level command
 */
void mesh_power_level_client_last_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_power_level_client_send_last_get(p_event);
}

/*
 * Send power Get Default Power Level command
 */
void mesh_power_level_client_default_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_power_level_client_send_default_get(p_event);
}

/*
 * Send power Set Default Power Level command
 */
void mesh_power_level_client_default_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_power_default_data_t set_data;

    STREAM_TO_UINT16(set_data.power, p_data);

    wiced_bt_mesh_model_power_level_client_send_default_set(p_event, &set_data);
}

/*
 * Send power Get Default Power Level command
 */
void mesh_power_level_client_range_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_power_level_client_send_range_get(p_event);
}

/*
 * Send power Set Range Power Level command
 */
void mesh_power_level_client_range_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_power_level_range_set_data_t set_data;

    STREAM_TO_UINT16(set_data.power_min, p_data);
    STREAM_TO_UINT16(set_data.power_max, p_data);

    wiced_bt_mesh_model_power_level_client_send_range_set(p_event, &set_data);
}

#ifdef HCI_CONTROL
/*
 * Send Power Level Status event over transport
 */
void mesh_power_level_hci_event_send_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_power_level_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, p_data->present_power);
    UINT16_TO_STREAM(p, p_data->target_power);
    UINT32_TO_STREAM(p, p_data->remaining_time);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_POWER_LEVEL_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Last Power Level Status event over transport
 */
void mesh_power_level_hci_event_send_last_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_power_level_last_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, p_data->power);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_POWER_LEVEL_LAST_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Default Power Level Status event over transport
 */
void mesh_power_level_hci_event_send_default_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_power_default_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, p_data->power);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_POWER_LEVEL_DEFAULT_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Range Power Level Status event over transport
 */
void mesh_power_level_hci_event_send_range_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_power_range_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT8_TO_STREAM(p, p_data->status);
    UINT16_TO_STREAM(p, p_data->power_min);
    UINT16_TO_STREAM(p, p_data->power_max);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_POWER_LEVEL_RANGE_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}
#endif

#endif
