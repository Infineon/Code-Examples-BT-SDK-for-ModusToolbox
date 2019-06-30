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
 * This file shows how to create a device which implements mesh Mesh Light LC Client.
 */

#ifdef WICED_BT_MESH_MODEL_LIGHT_LC_CLIENT_INCLUDED

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
uint32_t mesh_light_lc_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
void mesh_light_lc_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void mesh_light_lc_client_mode_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_client_mode_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_client_occupancy_mode_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_client_occupancy_mode_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_client_onoff_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_client_onoff_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_client_property_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_client_property_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_light_lc_hci_event_send_mode_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lc_mode_set_data_t *p_data);
static void mesh_light_lc_hci_event_send_occupancy_mode_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lc_occupancy_mode_set_data_t *p_data);
static void mesh_light_lc_hci_event_send_light_onoff_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lc_light_onoff_status_data_t *p_data);
static void mesh_light_lc_hci_event_send_property_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lc_property_status_data_t *p_data);


/******************************************************
 *               Function Definitions
 ******************************************************/
/*
 * Process event received from the Light LC Server.
 */
void mesh_light_lc_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
    wiced_bt_mesh_light_lc_mode_set_data_t *p_mode;
    wiced_bt_mesh_light_lc_occupancy_mode_set_data_t *p_occupancy_mode;
    wiced_bt_mesh_light_lc_light_onoff_status_data_t *p_onoff_status;
    wiced_bt_mesh_light_lc_property_status_data_t *p_property_status;
#if defined HCI_CONTROL
    wiced_bt_mesh_hci_event_t *p_hci_event;
#endif

    WICED_BT_TRACE("light lc clnt msg:%d\n", event);

    switch (event)
    {
    case WICED_BT_MESH_TX_COMPLETE:
        WICED_BT_TRACE("tx complete status:%d\n", p_event->tx_status);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            wiced_bt_mesh_send_hci_tx_complete(p_hci_event, p_event);
#endif
        break;

    case WICED_BT_MESH_LIGHT_LC_MODE_STATUS:
        p_mode = (wiced_bt_mesh_light_lc_mode_set_data_t *)p_data;
        WICED_BT_TRACE("light lc mode:%d\n", p_mode->mode);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_light_lc_hci_event_send_mode_status(p_hci_event, p_mode);
#endif
        break;

    case WICED_BT_MESH_LIGHT_LC_OCCUPANCY_MODE_STATUS:
        p_occupancy_mode = (wiced_bt_mesh_light_lc_occupancy_mode_set_data_t *)p_data;
        WICED_BT_TRACE("light lc occupany mode:%d\n", p_occupancy_mode->mode);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_light_lc_hci_event_send_occupancy_mode_status(p_hci_event, p_occupancy_mode);
#endif
        break;

    case WICED_BT_MESH_LIGHT_LC_LIGHT_ONOFF_STATUS:
        p_onoff_status = (wiced_bt_mesh_light_lc_light_onoff_status_data_t *)p_data;
        WICED_BT_TRACE("light onoff status present:%d target:%d remain time:%d\n",
            p_onoff_status->present_onoff, p_onoff_status->target_onoff, p_onoff_status->remaining_time);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_light_lc_hci_event_send_light_onoff_status(p_hci_event, p_onoff_status);
#endif
        break;

    case WICED_BT_MESH_LIGHT_LC_PROPERTY_STATUS:
        p_property_status = (wiced_bt_mesh_light_lc_property_status_data_t *)p_data;
        WICED_BT_TRACE("light property status ID:%d len:%d\n", p_property_status->id, p_property_status->len);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_light_lc_hci_event_send_property_status(p_hci_event, p_property_status);
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
uint32_t mesh_light_lc_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
#ifdef HCI_CONTROL
    wiced_bt_mesh_event_t *p_event;

    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_MODE_GET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_MODE_SET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_OCCUPANCY_MODE_GET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_OCCUPANCY_MODE_SET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_ONOFF_GET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_ONOFF_SET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_PROPERTY_GET:
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_PROPERTY_SET:
        break;

    default:
        return WICED_FALSE;
    }
    p_event = wiced_bt_mesh_create_event_from_wiced_hci(opcode, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LC_CLNT, &p_data, &length);
    if (p_event == NULL)
    {
        WICED_BT_TRACE("bad hdr\n");
        return WICED_TRUE;
    }
    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_MODE_GET:
        mesh_light_lc_client_mode_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_MODE_SET:
        mesh_light_lc_client_mode_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_OCCUPANCY_MODE_GET:
        mesh_light_lc_client_occupancy_mode_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_OCCUPANCY_MODE_SET:
        mesh_light_lc_client_occupancy_mode_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_ONOFF_GET:
        mesh_light_lc_client_onoff_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_ONOFF_SET:
        mesh_light_lc_client_onoff_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_PROPERTY_GET:
        mesh_light_lc_client_property_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LIGHT_LC_PROPERTY_SET:
        mesh_light_lc_client_property_set(p_event, p_data, length);
        break;
    }
#endif
    return WICED_TRUE;
}

/*
 * Send Light LC Mode Get command
 */
void mesh_light_lc_client_mode_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_light_lc_client_send_mode_get(p_event);
}

/*
 * Send Light LC Mode set command
 */
void mesh_light_lc_client_mode_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_light_lc_mode_set_data_t set_data;

    STREAM_TO_UINT8(set_data.mode, p_data);

    wiced_bt_mesh_model_light_lc_client_send_mode_set(p_event, &set_data);
}

/*
 * Send Light Occupancy Mode get command
 */
void mesh_light_lc_client_occupancy_mode_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_light_lc_client_send_occupancy_mode_get(p_event);
}

/*
 * Send Light LC Occupancy Mode set command
 */
void mesh_light_lc_client_occupancy_mode_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_light_lc_occupancy_mode_set_data_t set_data;

    STREAM_TO_UINT8(set_data.mode, p_data);

    wiced_bt_mesh_model_light_lc_client_send_occupancy_mode_set(p_event, &set_data);
}

/*
 * Send Light LC OnOff get command
 */
void mesh_light_lc_client_onoff_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_light_lc_client_send_light_onoff_get(p_event);
}

/*
 * Send Light LC OnOff set command
 */
void mesh_light_lc_client_onoff_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_light_lc_light_onoff_set_data_t set_data;

    STREAM_TO_UINT8(set_data.light_onoff, p_data);
    STREAM_TO_UINT32(set_data.transition_time, p_data);
    STREAM_TO_UINT16(set_data.delay, p_data);

    wiced_bt_mesh_model_light_lc_client_send_light_onoff_set(p_event, &set_data);
}

/*
 * Send Light LC Property Get command
 */
void mesh_light_lc_client_property_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_light_lc_property_get_data_t get_data;

    STREAM_TO_UINT16(get_data.id, p_data);

    wiced_bt_mesh_model_light_lc_client_send_property_get(p_event, &get_data);
}

/*
 * Send Light LC Property set command
 */
void mesh_light_lc_client_property_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_light_lc_property_set_data_t set_data;

    STREAM_TO_UINT16(set_data.id, p_data);

    set_data.len = length - 2;
    memcpy(set_data.value, p_data, set_data.len);

    wiced_bt_mesh_model_light_lc_client_send_property_set(p_event, &set_data);
}


#ifdef HCI_CONTROL
/*
 * Send Light LC Mode Status event over transport
 */
void mesh_light_lc_hci_event_send_mode_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lc_mode_set_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT8_TO_STREAM(p, p_data->mode);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LIGHT_LC_MODE_CLIENT_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Light LC Occupancy Mode Status event over transport
 */
void mesh_light_lc_hci_event_send_occupancy_mode_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lc_occupancy_mode_set_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT8_TO_STREAM(p, p_data->mode);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LIGHT_LC_OCCUPANCY_MODE_CLIENT_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Light LC Occupancy Light OnOff Status event over transport
 */
void mesh_light_lc_hci_event_send_light_onoff_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lc_light_onoff_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT8_TO_STREAM(p, p_data->present_onoff);
    UINT8_TO_STREAM(p, p_data->target_onoff);
    UINT32_TO_STREAM(p, p_data->remaining_time);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LIGHT_LC_ONOFF_CLIENT_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Light LC Occupancy Property Status event over transport
 */
void mesh_light_lc_hci_event_send_property_status(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_light_lc_property_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, p_data->id);
    memcpy(p, p_data->value, p_data->len);
    p += p_data->len;

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LIGHT_LC_PROPERTY_CLIENT_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

#endif

#endif
