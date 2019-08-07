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
 * This file shows how to create a device which implements mesh user time client.
 */

#ifdef WICED_BT_MESH_MODEL_TIME_CLIENT_INCLUDED

#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "rtc.h"
#include "wiced_bt_mesh_app.h"

#ifdef HCI_CONTROL
#include "wiced_transport.h"
#include "hci_control_api.h"
#endif

/******************************************************
 *          Function Prototypes
 ******************************************************/
uint32_t mesh_time_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
void mesh_time_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void mesh_time_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_time_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_time_zone_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_time_zone_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_time_tai_utc_delta_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t data_len);
static void mesh_time_tai_utc_delta_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_time_role_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_time_role_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_time_status_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_time_state_msg_t *p_time_status);
static void mesh_time_zone_status_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_time_zone_status_t *p_time_status);
static void mesh_time_tai_utc_delta_status_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_time_tai_utc_delta_status_t *p_time_delta_status);
static void mesh_time_role_status_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_time_role_msg_t *p_role_status);


/******************************************************
 *               Function Definitions
 ******************************************************/
/*
 * Process event received from the time Server.
 */
void mesh_time_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
#if defined HCI_CONTROL
    wiced_bt_mesh_hci_event_t *p_hci_event;
#endif
    WICED_BT_TRACE("time clt msg:%d\n", event);

    switch (event)
    {
    case WICED_BT_MESH_TIME_STATUS:
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_time_status_hci_event_send(p_hci_event, (wiced_bt_mesh_time_state_msg_t *)p_data);
#endif
        break;

    case WICED_BT_MESH_TIME_ZONE_STATUS:
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_time_zone_status_hci_event_send(p_hci_event, (wiced_bt_mesh_time_zone_status_t *)p_data);
#endif
        break;

    case WICED_BT_MESH_TAI_UTC_DELTA_STATUS:
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_time_tai_utc_delta_status_hci_event_send(p_hci_event, (wiced_bt_mesh_time_tai_utc_delta_status_t *)p_data);
#endif
        break;

    case WICED_BT_MESH_TIME_ROLE_STATUS:
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_time_role_status_hci_event_send(p_hci_event, (wiced_bt_mesh_time_role_msg_t *)p_data);
#endif
        break;

    default:
        WICED_BT_TRACE("unknown\n");
        break;
    }
    wiced_bt_mesh_release_event(p_event);
}

/*
 * In 2 chip solutions MCU can send commands to change time state.
 */
uint32_t mesh_time_client_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_event_t *p_event;

    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_TIME_GET:
    case HCI_CONTROL_MESH_COMMAND_TIME_SET:
    case HCI_CONTROL_MESH_COMMAND_TIME_ZONE_GET:
    case HCI_CONTROL_MESH_COMMAND_TIME_ZONE_SET:
    case HCI_CONTROL_MESH_COMMAND_TIME_TAI_UTC_DELTA_GET:
    case HCI_CONTROL_MESH_COMMAND_TIME_TAI_UTC_DELTA_SET:
    case HCI_CONTROL_MESH_COMMAND_TIME_ROLE_GET:
    case HCI_CONTROL_MESH_COMMAND_TIME_ROLE_SET:
        break;

    default:
        WICED_BT_TRACE("unknown\n");
        return WICED_FALSE;
    }
    p_event = wiced_bt_mesh_create_event_from_wiced_hci(opcode, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_CLNT, &p_data, &length);

    WICED_BT_TRACE("time client hci opcode %x event:%x\n", opcode, p_event);
    if (p_event == NULL)
    {
        WICED_BT_TRACE("bad hdr\n");
        return WICED_TRUE;
    }
    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_TIME_GET:
        mesh_time_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_TIME_SET:
        mesh_time_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_TIME_ZONE_GET:
        mesh_time_zone_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_TIME_ZONE_SET:
        mesh_time_zone_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_TIME_TAI_UTC_DELTA_GET:
        mesh_time_tai_utc_delta_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_TIME_TAI_UTC_DELTA_SET:
        mesh_time_tai_utc_delta_set(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_TIME_ROLE_GET:
        mesh_time_role_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_TIME_ROLE_SET:
        mesh_time_role_set(p_event, p_data, length);
        break;
    }
    return WICED_TRUE;
}

/*
 * Send time get command
 */
void mesh_time_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    WICED_BT_TRACE("mesh_time_get\n");
    wiced_bt_mesh_model_time_client_time_get_send(p_event);
}

/*
 * Send time set command
 */
void mesh_time_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_time_state_msg_t set_data;

    WICED_BT_TRACE("mesh_time_set\n");

    STREAM_TO_UINT40(set_data.tai_seconds, p_data);
    STREAM_TO_UINT8(set_data.subsecond, p_data);
    STREAM_TO_UINT8(set_data.uncertainty, p_data);
    STREAM_TO_UINT8(set_data.time_authority, p_data);
    STREAM_TO_UINT16(set_data.tai_utc_delta_current, p_data);
    STREAM_TO_UINT8(set_data.time_zone_offset_current, p_data);

    wiced_bt_mesh_model_time_client_time_set_send(p_event, &set_data);
}

/*
 * Send time zone get command
 */
void mesh_time_zone_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    WICED_BT_TRACE("mesh_time_zone_get\n");

    wiced_bt_mesh_model_time_client_time_zone_get_send(p_event);
}

/*
 * Send time zone set command
 */
void mesh_time_zone_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_time_zone_set_t set_data;

    WICED_BT_TRACE("mesh_time_zone_set\n");

    STREAM_TO_UINT8(set_data.time_zone_offset_new, p_data);
    STREAM_TO_UINT40(set_data.tai_of_zone_change, p_data);

    wiced_bt_mesh_model_time_client_time_zone_set_send(p_event, &set_data);
}

/*
 * Send time tai_utc delta get command
 */
void mesh_time_tai_utc_delta_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t data_len)
{
    WICED_BT_TRACE("mesh_time_tai_utc_delta_get\n");

    wiced_bt_mesh_model_time_client_tai_utc_delta_get_send(p_event);
}

/*
 * Send time tai_utc delta set command
 */
void mesh_time_tai_utc_delta_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_time_tai_utc_delta_set_t set_data;

    WICED_BT_TRACE("mesh_time_tai_utc_delta_set\n");

    STREAM_TO_UINT16(set_data.tai_utc_delta_new, p_data);
    STREAM_TO_UINT40(set_data.tai_of_delta_change, p_data);

    wiced_bt_mesh_model_time_client_tai_utc_delta_set_send(p_event, &set_data);
}

/*
 * Send time role get command
 */
void mesh_time_role_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    WICED_BT_TRACE("mesh_time_role_get\n");

    wiced_bt_mesh_model_time_client_time_role_get_send(p_event);
}

/*
 * Send time role set command
 */
void mesh_time_role_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_time_role_msg_t set_data;

    WICED_BT_TRACE("mesh_time_role_set\n");

    STREAM_TO_UINT8(set_data.role, p_data);

    wiced_bt_mesh_model_time_client_time_role_set_send(p_event, &set_data);
}

#ifdef HCI_CONTROL
/*
 * Send Time Status event over transport
 */
void mesh_time_status_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_time_state_msg_t *p_time_status)
{
    uint8_t *p = p_hci_event->data;

    UINT40_TO_STREAM(p, p_time_status->tai_seconds);
    UINT8_TO_STREAM(p, p_time_status->subsecond);
    UINT8_TO_STREAM(p, p_time_status->uncertainty);
    UINT8_TO_STREAM(p, p_time_status->time_authority);
    UINT16_TO_STREAM(p, p_time_status->tai_utc_delta_current);
    UINT8_TO_STREAM(p, p_time_status->time_zone_offset_current);

    WICED_BT_TRACE("TAI_seconds:%02x%08x\n", (uint32_t)(p_time_status->tai_seconds >> 32), (uint32_t)(p_time_status->tai_seconds & 0xFFFFFFFF));
    WICED_BT_TRACE("subsecond: %x\n", p_time_status->subsecond);
    WICED_BT_TRACE("uncertainty: %x\n", p_time_status->uncertainty);
    WICED_BT_TRACE("auth: %x\n", p_time_status->time_authority);
    WICED_BT_TRACE("tai_utc_delta_current: %x\n", p_time_status->tai_utc_delta_current);
    WICED_BT_TRACE("time_zone_offset_current: %x\n", p_time_status->time_zone_offset_current);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_TIME_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Time Zone Status event over transport
 */
void mesh_time_zone_status_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_time_zone_status_t *p_time_status)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("%s\n", __FUNCTION__);

    UINT8_TO_STREAM(p, p_time_status->time_zone_offset_current);
    UINT8_TO_STREAM(p, p_time_status->time_zone_offset_new);
    UINT40_TO_STREAM(p, p_time_status->tai_of_zone_change);

    WICED_BT_TRACE(" TAI_of_zone_change:\n");
    WICED_BT_TRACE("TAI_of_zone_change: %x\n", p_time_status->tai_of_zone_change);
    WICED_BT_TRACE("time_zone_offset_current: %x\n", p_time_status->time_zone_offset_current);
    WICED_BT_TRACE("time_zone_offset_new: %x\n",p_time_status->time_zone_offset_new);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_TIME_ZONE_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}


/*
 * Send Time TAI UTC delta status event over transport
 */
void mesh_time_tai_utc_delta_status_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_time_tai_utc_delta_status_t *p_time_delta_status)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("%s\n", __FUNCTION__);

    UINT16_TO_STREAM(p, p_time_delta_status->tai_utc_delta_current);
    UINT16_TO_STREAM(p, p_time_delta_status->tai_utc_delta_new);
    UINT40_TO_STREAM(p, p_time_delta_status->tai_of_delta_change);

    WICED_BT_TRACE(" tai_of_delta_change:\n");
    WICED_BT_TRACE("tai_of_delta_change: %x\n", p_time_delta_status->tai_of_delta_change);
    WICED_BT_TRACE("tai_utc_delta_current: %x\n", p_time_delta_status->tai_utc_delta_current);
    WICED_BT_TRACE("tai_utc_delta_new: %x\n",p_time_delta_status->tai_utc_delta_new);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_TIME_TAI_UTC_DELTA_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

/*
 * Send Time role status event over transport
 */
void mesh_time_role_status_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_time_role_msg_t *p_role_status)
{
    uint8_t *p = p_hci_event->data;

    WICED_BT_TRACE("%s role:%x\n", __FUNCTION__, p_role_status->role);

    UINT8_TO_STREAM(p, p_role_status->role);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_TIME_ROLE_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}
#endif


#endif
