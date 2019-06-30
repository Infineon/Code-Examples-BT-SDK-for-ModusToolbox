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
 * This file shows how to create a device which can query or set location of the location setup server device.
 */

#ifdef WICED_BT_MESH_MODEL_LOCATION_CLIENT_INCLUDED

#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_mesh_event.h"
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
uint32_t mesh_location_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
void mesh_location_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void mesh_location_client_global_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_location_client_local_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_location_client_global_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_location_client_local_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_location_global_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_location_global_data_t *p_data);
static void mesh_location_local_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_location_local_data_t *p_data);

/******************************************************
 *               Function Definitions
 ******************************************************/
/*
 * Process event received from the Location Server.
 */
void mesh_location_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
    wiced_bt_mesh_location_global_data_t *p_global;
    wiced_bt_mesh_location_local_data_t *p_local;
#if defined HCI_CONTROL
    wiced_bt_mesh_hci_event_t *p_hci_event;
#endif

    WICED_BT_TRACE("location clt msg:%d\n", event);

    switch (event)
    {
    case WICED_BT_MESH_TX_COMPLETE:
        WICED_BT_TRACE("tx complete status:%d\n", p_event->tx_status);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            wiced_bt_mesh_send_hci_tx_complete(p_hci_event, p_event);
#endif
        break;

    case WICED_BT_MESH_LOCATION_GLOBAL_STATUS:
        p_global = (wiced_bt_mesh_location_global_data_t *)p_data;
        WICED_BT_TRACE("Global lat/long/alt:%d/%d/%d\n",
                p_global->global_latitude, p_global->global_longitude, p_global->global_altitude);

#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_location_global_hci_event_send(p_hci_event, p_global);
#endif
        break;

    case WICED_BT_MESH_LOCATION_LOCAL_STATUS:
        p_local = (wiced_bt_mesh_location_local_data_t *)p_data;
        WICED_BT_TRACE("Local north/east/alt %d/%d/%d mobile:%d time:%d precision:%d\n",
                p_local->local_north, p_local->local_east, p_local->local_altitude,
                p_local->is_mobile, p_local->update_time, p_local->precision);

#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_location_local_hci_event_send(p_hci_event, p_local);
#endif
        break;

    default:
        WICED_BT_TRACE("unknown\n");
        break;
    }
    wiced_bt_mesh_release_event(p_event);
}

/*
 * In 2 chip solutions MCU can send commands from the MCU.
 */
uint32_t mesh_location_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_event_t *p_event;

    WICED_BT_TRACE("[%s] cmd_opcode 0x%02x\n", __FUNCTION__, opcode);

    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_LOCATION_GLOBAL_GET:
    case HCI_CONTROL_MESH_COMMAND_LOCATION_LOCAL_GET:
    case HCI_CONTROL_MESH_COMMAND_LOCATION_GLOBAL_SET:
    case HCI_CONTROL_MESH_COMMAND_LOCATION_LOCAL_SET:
        break;

    default:
        WICED_BT_TRACE("unknown\n");
        return WICED_FALSE;
    }
    p_event = wiced_bt_mesh_create_event_from_wiced_hci(opcode, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LOCATION_CLNT, &p_data, &length);
    if (p_event == NULL)
    {
        WICED_BT_TRACE("bad hdr\n");
        return WICED_TRUE;
    }
    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_LOCATION_GLOBAL_GET:
        mesh_location_client_global_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LOCATION_LOCAL_GET:
        mesh_location_client_local_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LOCATION_GLOBAL_SET:
        mesh_location_client_global_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_LOCATION_LOCAL_SET:
        mesh_location_client_local_set(p_event, p_data, length);
        break;
    }
    return WICED_TRUE;
}

/*
 * Send Location Get message
 */
void mesh_location_client_global_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_location_client_send_global_get(p_event);
}

/*
 * Send Location Get message
 */
void mesh_location_client_local_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_location_client_send_local_get(p_event);
}

/*
 * Send Location Set message
 */
void mesh_location_client_global_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_location_global_data_t data;

    WICED_BT_TRACE("loc clnt global set\n");

    STREAM_TO_UINT32(data.global_latitude, p_data);
    STREAM_TO_UINT32(data.global_longitude, p_data);
    STREAM_TO_UINT16(data.global_altitude, p_data);

    wiced_bt_mesh_model_location_client_send_global_set(p_event, &data);
}

/*
 * Send Location Set message
 */
void mesh_location_client_local_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_location_local_data_t data;

    WICED_BT_TRACE("loc clnt local set\n");

    STREAM_TO_UINT16(data.local_north, p_data);
    STREAM_TO_UINT16(data.local_east, p_data);
    STREAM_TO_UINT16(data.local_altitude, p_data);
    STREAM_TO_UINT8(data.floor_number, p_data);
    STREAM_TO_UINT8(data.is_mobile, p_data);
    STREAM_TO_UINT8(data.update_time, p_data);
    STREAM_TO_UINT8(data.precision, p_data);

    wiced_bt_mesh_model_location_client_send_local_set(p_event, &data);
}

#ifdef HCI_CONTROL
/*
 * Send Location Status event over transport
 */
void mesh_location_global_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_location_global_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT32_TO_STREAM(p, p_data->global_latitude);
    UINT32_TO_STREAM(p, p_data->global_longitude);
    UINT16_TO_STREAM(p, p_data->global_altitude);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LOCATION_GLOBAL_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Location Status event over transport
 */
void mesh_location_local_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_location_local_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT16_TO_STREAM(p, p_data->local_north);
    UINT16_TO_STREAM(p, p_data->local_east);
    UINT16_TO_STREAM(p, p_data->local_altitude);
    UINT8_TO_STREAM(p, p_data->floor_number);
    UINT8_TO_STREAM(p, p_data->is_mobile);
    UINT8_TO_STREAM(p, p_data->update_time);
    UINT8_TO_STREAM(p, p_data->precision);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_LOCATION_LOCAL_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}
#endif

#endif
