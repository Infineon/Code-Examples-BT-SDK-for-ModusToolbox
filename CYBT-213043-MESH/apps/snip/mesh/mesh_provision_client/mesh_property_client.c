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
 * This file shows how to create a device which implements mesh user property client.
 */

#ifdef WICED_BT_MESH_MODEL_PROPERTY_CLIENT_INCLUDED

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
uint32_t mesh_property_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
void mesh_property_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void mesh_properties_client_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_property_client_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_property_client_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_properties_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_properties_status_data_t *p_data);
static void mesh_property_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_property_status_data_t *p_data);

/******************************************************
 *               Function Definitions
 ******************************************************/
/*
 * Process event received from the property Server.
 */
void mesh_property_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
#if defined HCI_CONTROL
    wiced_bt_mesh_hci_event_t *p_hci_event;
#endif
    wiced_bt_mesh_property_status_data_t   *p_property_status;
    wiced_bt_mesh_properties_status_data_t *p_properties_status;
    WICED_BT_TRACE("property clt msg:%d\n", event);

    switch (event)
    {
    case WICED_BT_MESH_TX_COMPLETE:
        WICED_BT_TRACE("tx complete status:%d\n", p_event->tx_status);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            wiced_bt_mesh_send_hci_tx_complete(p_hci_event, p_event);
#endif
        break;

    case WICED_BT_MESH_CLIENT_PROPERTIES_STATUS:
    case WICED_BT_MESH_ADMIN_PROPERTIES_STATUS:
    case WICED_BT_MESH_MANUF_PROPERTIES_STATUS:
    case WICED_BT_MESH_USER_PROPERTIES_STATUS:
        p_properties_status = (wiced_bt_mesh_properties_status_data_t *)p_data;
        WICED_BT_TRACE("properties type:%d num:%d\n", p_properties_status->type, p_properties_status->properties_num);

#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_properties_hci_event_send(p_hci_event, p_properties_status);
#endif
        break;

    case WICED_BT_MESH_ADMIN_PROPERTY_STATUS:
    case WICED_BT_MESH_MANUF_PROPERTY_STATUS:
    case WICED_BT_MESH_USER_PROPERTY_STATUS:
        p_property_status = (wiced_bt_mesh_property_status_data_t *)p_data;
        WICED_BT_TRACE("property type:%d id:%04x len:%d\n", p_property_status->type, p_property_status->id, p_property_status->len);

#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_property_hci_event_send(p_hci_event, p_property_status);
#endif
        break;

    default:
        WICED_BT_TRACE("unknown\n");
        break;
    }
    wiced_bt_mesh_release_event(p_event);
}

/*
 * In 2 chip solutions MCU can send commands to change property state.
 */
uint32_t mesh_property_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_event_t *p_event;

    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_PROPERTIES_GET:
    case HCI_CONTROL_MESH_COMMAND_PROPERTY_GET:
    case HCI_CONTROL_MESH_COMMAND_PROPERTY_SET:
        break;

    default:
        return WICED_FALSE;
    }
    WICED_BT_TRACE("prop client cmd_opcode 0x%02x\n", opcode);

    p_event = wiced_bt_mesh_create_event_from_wiced_hci(opcode, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_PROPERTY_CLNT, &p_data, &length);
    if (p_event == NULL)
    {
        WICED_BT_TRACE("bad hdr\n");
        return WICED_TRUE;
    }
    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_PROPERTIES_GET:
        mesh_properties_client_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_PROPERTY_GET:
        mesh_property_client_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_PROPERTY_SET:
        mesh_property_client_set(p_event, p_data, length);
        break;
    }
    return WICED_TRUE;
}

/*
 * Send properties get command
 */
void mesh_properties_client_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_properties_get_data_t get_data;

    STREAM_TO_UINT8(get_data.type, p_data);

    // starting ID is valid for Client Properties
    if ((get_data.type == WICED_BT_MESH_PROPERTY_TYPE_CLIENT) && (length >= 3))
    {
        STREAM_TO_UINT16(get_data.starting_id, p_data);
    }
    else
    {
        get_data.starting_id = 0;
    }
    wiced_bt_mesh_model_property_client_send_properties_get(p_event, &get_data);
}

/*
 * Send property get command
 */
void mesh_property_client_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_property_get_data_t get_data;

    STREAM_TO_UINT8(get_data.type, p_data);
    STREAM_TO_UINT16(get_data.id, p_data);

    wiced_bt_mesh_model_property_client_send_property_get(p_event, &get_data);
}

/*
 * Send property set command
 */
void mesh_property_client_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_property_set_data_t set_data;

    memset(&set_data, 0, sizeof(set_data));

    STREAM_TO_UINT8(set_data.type, p_data);
    STREAM_TO_UINT16(set_data.id, p_data);
    STREAM_TO_UINT8(set_data.access, p_data);

    set_data.len = length - 4;
    if (set_data.len > sizeof(set_data.value))
        set_data.len = sizeof(set_data.value);

    STREAM_TO_ARRAY(set_data.value, p_data, set_data.len);

    wiced_bt_mesh_model_property_client_send_property_set(p_event, &set_data);
}

#ifdef HCI_CONTROL
/*
 * Send Properties Status event over transport
 */
void mesh_properties_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_properties_status_data_t *p_data)
{
    int i;
    uint8_t *p = p_hci_event->data;

    UINT8_TO_STREAM(p, p_data->type);
    for (i = 0; i < p_data->properties_num; i++)
        UINT16_TO_STREAM(p, p_data->id[i]);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_PROPERTIES_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send property Status event over transport
 */
void mesh_property_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_property_status_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT8_TO_STREAM(p, p_data->type);
    UINT8_TO_STREAM(p, p_data->access);
    UINT16_TO_STREAM(p, p_data->id);
    ARRAY_TO_STREAM(p, p_data->value, p_data->len);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_PROPERTY_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

#endif

#endif
