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
 * This file implements audio application controlled over UART.
 *
 */
#include "wiced_bt_trace.h"
#include "hci_control_rc_controller.h"
#include "string.h"
#include "wiced_memory.h"

/******************************************************************************
 *                          Constants
 ******************************************************************************/
#define AVCT_MIN_CONTROL_MTU            48      /* Per the AVRC spec, minimum MTU for the control channel */
#define AVCT_CONTROL_MTU                256     /* MTU for the control channel */
#define AVCT_MIN_BROWSE_MTU             335     /* Per the AVRC spec, minimum MTU for the browsing channel */

#define SDP_DB_LEN                      400

#define MAX_POSSIBLE_APP_ATTR_SETTINGS  4
#define MAX_POSSIBLE_APP_ATTR_VALUES    4

/******************************************************************************
 *                         Variable Definitions
 ******************************************************************************/
typedef struct
{
    wiced_bool_t available;
    uint8_t      current_index;
    uint8_t      num_possible_values;
    uint8_t      possible_values[MAX_POSSIBLE_APP_ATTR_VALUES]; /* Values are all 1 based */
} tAVRC_APP_SETTING_ATTR;

typedef struct
{
    wiced_bt_device_address_t remote_addr;
    wiced_bt_avrc_ct_connection_state_t connection_state;

    uint8_t num_app_settings;
    uint8_t num_app_settings_init;
    tAVRC_APP_SETTING_ATTR app_setting[MAX_POSSIBLE_APP_ATTR_SETTINGS + 1];
    uint8_t handle;
} tRC_APP_CB;

static tRC_APP_CB rc_app_cb;

uint8_t avrc_supported_events[] =
{
    WICED_FALSE,
    WICED_TRUE,            /* AVRC_EVT_PLAY_STATUS_CHANGE             0x01    Playback Status Changed */
    WICED_TRUE,            /* AVRC_EVT_TRACK_CHANGE                   0x02    Track Changed */
    WICED_TRUE,            /* AVRC_EVT_TRACK_REACHED_END              0x03    Track End Reached */
    WICED_TRUE,            /* AVRC_EVT_TRACK_REACHED_START            0x04    Track Reached Start */
    WICED_TRUE,            /* AVRC_EVT_PLAY_POS_CHANGED               0x05    Playback position changed */
    WICED_FALSE,           /* AVRC_EVT_BATTERY_STATUS_CHANGE          0x06    Battery status changed */
    WICED_FALSE,           /* AVRC_EVT_SYSTEM_STATUS_CHANGE           0x07    System status changed */
    WICED_TRUE,            /* AVRC_EVT_APP_SETTING_CHANGE             0x08    Player application settings changed */
    WICED_FALSE,           /* AVRC_EVT_NOW_PLAYING_CHANGE             0x09    Now Playing Content Changed (AVRCP 1.4) */
    WICED_FALSE,           /* AVRC_EVT_AVAL_PLAYERS_CHANGE            0x0a    Available Players Changed Notification (AVRCP 1.4) */
    WICED_FALSE,           /* AVRC_EVT_ADDR_PLAYER_CHANGE             0x0b    Addressed Player Changed Notification (AVRCP 1.4) */
    WICED_FALSE,           /* AVRC_EVT_UIDS_CHANGE                    0x0c    UIDs Changed Notification (AVRCP 1.4) */
    WICED_FALSE            /* AVRC_EVT_VOLUME_CHANGE                  0x0d    Notify Volume Change (AVRCP 1.4) */
};

/* Map avrc status values to the most appropriate WICED result */
wiced_result_t avrc_status_to_wiced[] =
{
    WICED_ERROR,            /* #define AVRC_STS_BAD_CMD        0x00    Invalid command, sent if TG received a PDU that it did not understand. */
    WICED_BADARG,           /* #define AVRC_STS_BAD_PARAM      0x01    Invalid parameter, sent if the TG received a PDU with a parameter ID that it did not understand. Sent if there is only one parameter ID in the PDU. */
    WICED_BADOPTION,        /* #define AVRC_STS_NOT_FOUND      0x02    Specified parameter not found., sent if the parameter ID is understood, but content is wrong or corrupted. */
    WICED_ERROR,            /* #define AVRC_STS_INTERNAL_ERR   0x03    Internal Error, sent if there are error conditions not covered by a more specific error code. */
    WICED_SUCCESS,          /* #define AVRC_STS_NO_ERROR       0x04    Operation completed without error.  This is the status that should be returned if the operation was successful. */
    WICED_BADARG,           /* #define AVRC_STS_UID_CHANGED    0x05    UID Changed - The UIDs on the device have changed */
    WICED_ERROR,            /* #define AVRC_STS_GEN_ERROR      0x06    Unknown Error - this is changed to "reserved" */
    WICED_ERROR,            /* #define AVRC_STS_BAD_DIR        0x07    Invalid Direction - The Direction parameter is invalid - Change Path*/
    WICED_ERROR,            /* #define AVRC_STS_NOT_DIR        0x08    Not a Directory - The UID provided does not refer to a folder item  Change Path*/
    WICED_NOT_FOUND,        /* #define AVRC_STS_NOT_EXIST      0x09    Does Not Exist - The UID provided does not refer to any item    Change Path, PlayItem, AddToNowPlaying, GetItemAttributes*/
    WICED_BADVALUE,         /* #define AVRC_STS_BAD_SCOPE      0x0a    Invalid Scope - The scope parameter is invalid  GetFolderItems, PlayItem, AddToNowPlayer, GetItemAttributes, */
    WICED_BADVALUE,         /* #define AVRC_STS_BAD_RANGE      0x0b    Range Out of Bounds - The start of range provided is not valid  GetFolderItems*/
    WICED_BADVALUE,         /* #define AVRC_STS_UID_IS_DIR     0x0c    UID is a Directory - The UID provided refers to a directory, which cannot be handled by this media player   PlayItem, AddToNowPlaying */
    WICED_ERROR,            /* #define AVRC_STS_IN_USE         0x0d    Media in Use - The media is not able to be used for this operation at this time PlayItem, AddToNowPlaying */
    WICED_ERROR,            /* #define AVRC_STS_NOW_LIST_FULL  0x0e    Now Playing List Full - No more items can be added to the Now Playing List  AddToNowPlaying*/
    WICED_UNSUPPORTED,      /* #define AVRC_STS_SEARCH_NOT_SUP 0x0f    Search Not Supported - The Browsed Media Player does not support search Search */
    WICED_ERROR,            /* #define AVRC_STS_SEARCH_BUSY    0x10    Search in Progress - A search operation is already in progress  Search*/
    WICED_ERROR,            /* #define AVRC_STS_BAD_PLAYER_ID  0x11    Invalid Player Id - The specified Player Id does not refer to a valid player    SetAddressedPlayer, SetBrowsedPlayer*/
    WICED_UNSUPPORTED,      /* #define AVRC_STS_PLAYER_N_BR    0x12    Player Not Browsable - The Player Id supplied refers to a Media Player which does not support browsing. SetBrowsedPlayer */
    WICED_ERROR,            /* #define AVRC_STS_PLAYER_N_ADDR  0x13    Player Not Addressed.  The Player Id supplied refers to a player which is not currently addressed, and the command is not able to be performed if the player is not set as addressed.   Search, SetBrowsedPlayer*/
    WICED_ERROR,            /* #define AVRC_STS_BAD_SEARCH_RES 0x14    No valid Search Results - The Search result list does not contain valid entries, e.g. after being invalidated due to change of browsed player   GetFolderItems */
    WICED_ERROR,            /* #define AVRC_STS_NO_AVAL_PLAYER 0x15    No available players ALL */
    WICED_ERROR             /* #define AVRC_STS_ADDR_PLAYER_CHG 0x16   Addressed Player Changed - Register Notification */
};

#define avrc_status_to_wiced_result(a) ((a<=AVRC_STS_ADDR_PLAYER_CHG) ? avrc_status_to_wiced[a] : WICED_ERROR)

wiced_result_t app_avrc_hci_control_volume( uint8_t* p_data, uint32_t len );

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
/**
 *
 * Function         avrc_app_hci_pass_through
 *
 *                  AVRC Passthrough command requests from the MCU
 *                  are ... well ... passed through to the AVRCP api.
 *
 * @param[in]       op_id    : Pass through command id (see #AVRC_ID_XX)
 * @param[in]       state    : State of the pass through command (see #AVRC_STATE_XX)
 *
 * @return          wiced_result_t
 */
wiced_result_t avrc_app_hci_pass_through (uint16_t handle, uint8_t op_id, uint8_t state  )
{
    wiced_result_t result = WICED_NOT_CONNECTED;

    /* Make sure there is a connection before trying to send a command */
    if ( rc_app_cb.connection_state == REMOTE_CONTROL_CONNECTED )
    {
#ifdef WICED_APP_AUDIO_RC_CT_INCLUDED
        result = wiced_bt_avrc_ct_send_pass_through_cmd( handle, op_id, state, 0, NULL );
#endif
    }

    return result;
}

/**
 *
 * Function         avrcp_app_hci_set_player_settings
 *
 *                  Send MCU requests for application settings to the currently connected peer.
 *
 * @param[in]       attr_id    : Attribute ID (should be in the range of 1 to 4)
 * @param[in]       attr_value : Value to set for the attribute
 *
 * @return          wiced_result_t
 */
wiced_result_t avrcp_app_hci_set_player_settings (uint16_t handle, uint8_t attr_id )
{
    wiced_bt_avrc_player_app_param_t app_param;
    wiced_result_t result = WICED_NOT_CONNECTED;

    WICED_BT_TRACE( "%s: Enter: attr_id: %d \n", __FUNCTION__,  attr_id);

    /* Make sure there is a connection before trying to send a command */
    if ( rc_app_cb.connection_state == REMOTE_CONTROL_CONNECTED )
    {
        /* Put the request into the appropriate structure for the api. We only send a single
         * attribute at a time from the  MCU */
        app_param.num_attr      = 1;
        app_param.attr_id[0]    = attr_id;

        rc_app_cb.app_setting[attr_id].current_index =
                        (rc_app_cb.app_setting[attr_id].current_index + 1) %
                         rc_app_cb.app_setting[attr_id].num_possible_values;

        app_param.attr_value[0] =
                  rc_app_cb.app_setting[attr_id].possible_values[rc_app_cb.app_setting[attr_id].current_index];

        result = wiced_bt_avrc_ct_set_player_value_cmd( handle, &app_param );
    }

    return result;
}

/**
 *
 * Function         avrc_app_hci_connect
 *
 *                  Send MCU request for connection to a peer.
 *
 * @param[in]       p_data     : Data array containing peer address passed from MCU.
 *                               This array is sent in reverse order so it needs to be reordered
 *                               before it is used.
 * @param[in]       len        : Length of above array
 *
 * @return          wiced_result_t
 */
extern void hci_control_switch_avrcp_role(uint8_t new_role);

wiced_result_t avrc_app_hci_connect ( uint8_t* p_data, uint32_t len )
{
    wiced_bt_device_address_t bd_addr;

    if (avrcp_profile_role != AVRCP_CONTROLLER_ROLE)
    {
        hci_control_switch_avrcp_role(AVRCP_CONTROLLER_ROLE);
    }

    /* Reorder the bytes of the peer address */
    STREAM_TO_BDADDR(bd_addr,p_data);

    WICED_BT_TRACE( "%s: enter: <%B>\n", __FUNCTION__,  bd_addr);

    /* Request a connection to that address */
    return wiced_bt_avrc_ct_connect( bd_addr );
}

/**
 *
 * Function         avrc_app_hci_disconnect_connection
 *
 *                  Send MCU request for disconnect from a peer.
 *
 * @param[in]       p_data     : Data array containing peer address passed from MCU.
 *                               This array is sent in reverse order so it needs to be reordered
 *                               before it is used.
 * @param[in]       len        : Length of above array
 *
 * @return          wiced_result_t
 */
wiced_result_t avrc_app_hci_disconnect_connection ( uint16_t handle )
{
    wiced_result_t result = WICED_NOT_CONNECTED;

    /* Make sure there is a connection before trying to send a command */
    if ( rc_app_cb.connection_state == REMOTE_CONTROL_CONNECTED )
    {
        result = wiced_bt_avrc_ct_disconnect( handle );
    }

    return result;
}

wiced_result_t avrcp_app_hci_get_attrs_cmd(uint8_t* p_data, uint32_t len)
{
    wiced_result_t result = WICED_ERROR;
    uint16_t handle = p_data[0] | (p_data[1] << 8);

    result = wiced_bt_avrc_ct_get_element_attr_cmd(handle, 0, p_data[2], &p_data[3]);

    return result;
}

/**
 *
 * Function         avrc_connection_state_cback
 *
 *                  Callback invoked by the AVRCP api when a connection status change has occurred.
 *
 * @param[in]       remote_addr      : Peer address
 * @param[in]       status           : result of the connection status update attempted
 * @param[in]       connection_state : Connection state set by update
 * @param[in]       peer_features    : If new connection, this is a map of the peer's capabilities
 *
 * @return          Nothing
 */
void avrc_connection_state_cback( uint8_t handle,  wiced_bt_device_address_t remote_addr,wiced_result_t status,
        wiced_bt_avrc_ct_connection_state_t connection_state,
                                  uint32_t peer_features)
{
    WICED_BT_TRACE( "[%s]: %s (%d): <%B> features: 0x%x\n", __FUNCTION__,
                    (connection_state == REMOTE_CONTROL_DISCONNECTED) ? "REMOTE_CONTROL_DISCONNECTED" :
                    (connection_state == REMOTE_CONTROL_CONNECTED)    ? "REMOTE_CONTROL_CONNECTED" :
                    (connection_state == REMOTE_CONTROL_INITIALIZED)  ? "REMOTE_CONTROL_INITIALIZED" :
                                                                        "INVALID STATE",
                    connection_state,
                    remote_addr, peer_features);

    /* Service the connection state change. */
    switch( connection_state )
    {
    case REMOTE_CONTROL_DISCONNECTED:

        rc_app_cb.connection_state = REMOTE_CONTROL_DISCONNECTED;

        /* Inform the MCU of the disconnect */
        hci_control_avrc_send_disconnect_complete( handle );
        break;

    case REMOTE_CONTROL_CONNECTED:

        /* Save the peer address for other functions. */
        memcpy( rc_app_cb.remote_addr, remote_addr, BD_ADDR_LEN );
        rc_app_cb.connection_state = REMOTE_CONTROL_CONNECTED;

        /* Inform the MCU of the connection */
        hci_control_avrc_send_connect_complete( remote_addr, WICED_SUCCESS, handle );
        rc_app_cb.handle = handle;

        break;

    case REMOTE_CONTROL_INITIALIZED:
#ifdef WICED_APP_AUDIO_RC_CT_INCLUDED
        /* Find out what app controls the player has to offer. */
        wiced_bt_avrc_ct_list_player_attrs_cmd( handle );
#endif
        break;
    }
}

/* Track information to retrieve when a track change occurs */
uint8_t track_element_attributes[] =
{
    AVRC_MEDIA_ATTR_ID_TITLE,
    AVRC_MEDIA_ATTR_ID_ARTIST,
    AVRC_MEDIA_ATTR_ID_ALBUM,
    AVRC_MEDIA_ATTR_ID_TRACK_NUM,
    AVRC_MEDIA_ATTR_ID_NUM_TRACKS,
    AVRC_MEDIA_ATTR_ID_PLAYING_TIME
};
/**
 *
 * Function         avrc_handle_track_change_event
 *
 *                  On a registered track change event we submit a request for the updated track info.
 *
 * @param[in]       remote_addr : Address of the peer device
 *
 * @return          Nothing
 */
void avrc_handle_track_change_event(uint8_t handle)
{
#ifdef WICED_APP_AUDIO_RC_CT_INCLUDED
    wiced_result_t status = wiced_bt_avrc_ct_get_element_attr_cmd(
            handle, 0,
            (uint8_t)sizeof(track_element_attributes),
            track_element_attributes);

    WICED_BT_TRACE( "[%s]:handle: <%d> status: 0x%x\n", __FUNCTION__, handle, status);
#endif
}

/* Map the AVRCP events to the corresponding event sent to the MCU over the HCI */
static int avrc_event_id_to_hci_event[] =
{
    0, /* INVALID */
    HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_STATUS,
    HCI_CONTROL_AVRC_CONTROLLER_EVENT_TRACK_CHANGE,
    HCI_CONTROL_AVRC_CONTROLLER_EVENT_TRACK_END,
    HCI_CONTROL_AVRC_CONTROLLER_EVENT_TRACK_START,
    HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_POSITION,
    0,
    0,
    HCI_CONTROL_AVRC_CONTROLLER_EVENT_SETTING_CHANGE,
    0
};

#define MAX_REG_EVENT_SIZE (sizeof(uint16_t) + sizeof(uint8_t) + ((MAX_POSSIBLE_APP_ATTR_SETTINGS * 2) * sizeof(uint8_t)))

/**
 *
 * Function         avrc_handle_registered_notification_rsp
 *
 *                  Callback invoked by the avrc_response_cback to inform of a registered event being sent from the peer.
 *
 * @param[in]       remote_addr : Address of the peer device
 * @param[in]       avrc_rsp    : AVRC response messages
 *
 * @return          Nothing
 */
void avrc_handle_registered_notification_rsp(uint8_t handle,
                                             wiced_bt_avrc_response_t *avrc_rsp)
{
    wiced_result_t status = WICED_ERROR;
    uint8_t        event_data[MAX_REG_EVENT_SIZE] = {0};
    uint16_t       event_data_size = sizeof(uint16_t);

    wiced_bt_avrc_reg_notif_rsp_t *reg_notif = (wiced_bt_avrc_reg_notif_rsp_t *)avrc_rsp;

    WICED_BT_TRACE( "[%s]: handle: <%d> Notification Event: 0x%x\n", __FUNCTION__,
            handle, reg_notif->event_id);

    /* Handle?? */
    event_data[0] = handle;

    switch(reg_notif->event_id)
    {
    case AVRC_EVT_PLAY_STATUS_CHANGE:             /**< Playback Status Changed */
        event_data[event_data_size++] = reg_notif->param.play_status;
        status = WICED_SUCCESS;
        break;

    case AVRC_EVT_TRACK_CHANGE:                   /**< Track Changed */
        avrc_handle_track_change_event(handle);
        status = WICED_SUCCESS;
        break;

    case AVRC_EVT_TRACK_REACHED_END:              /**< Track End Reached */
    case AVRC_EVT_TRACK_REACHED_START:            /**< Track Reached Start */
        status = WICED_SUCCESS;
        break;

    case AVRC_EVT_PLAY_POS_CHANGED:               /**< Playback position changed */
        *((uint32_t *)&event_data[event_data_size]) = reg_notif->param.play_pos;
        event_data_size += sizeof(uint32_t);
        status = WICED_SUCCESS;
        break;

    case AVRC_EVT_APP_SETTING_CHANGE:             /**< Player application settings changed */
    {
        int i;

        event_data[event_data_size++] = reg_notif->param.player_setting.num_attr;

        for (i = 0; i < reg_notif->param.player_setting.num_attr; i++)
        {
            event_data[event_data_size++] = reg_notif->param.player_setting.attr_id[i];
            event_data[event_data_size++] = reg_notif->param.player_setting.attr_value[i];
        }

        status = WICED_SUCCESS;
    }
    break;

    default:
        WICED_BT_TRACE( "[%s]: unhandled event: Event ID: 0x%x\n", __FUNCTION__, reg_notif->event_id);
        break;
    }

    if (status == WICED_SUCCESS)
    {
        /* Send registered event information up to the MCU */
        if (avrc_event_id_to_hci_event[reg_notif->event_id])
            hci_control_send_avrc_event( avrc_event_id_to_hci_event[reg_notif->event_id], event_data, event_data_size );
    }

}

/**
 *
 * Function         avrc_handle_element_attribute_rsp
 *
 *                  Callback invoked by avrc_response_cback to inform of a response to the
 *                  get element attributes request.
 *
 * @param[in]       remote_addr : Address of the peer device
 * @param[in]       avrc_rsp    : AVRC response messages
 *
 * @return          Nothing
 */
void avrc_handle_element_attribute_rsp(uint8_t handle,
                                       wiced_bt_avrc_response_t *avrc_rsp)
{
    int i;
    int rsp_size;
    uint8_t *rsp;

    wiced_bt_avrc_get_elem_attrs_rsp_t *elem_attrs_rsp = &avrc_rsp->get_elem_attrs;

    /* If successful, make room for the response. */
    if (avrc_rsp->rsp.status == AVRC_STS_NO_ERROR)
    {
        /* Determine the number of bytes necessary to transport each element separately to MCU */
        for ( i = 0; i < elem_attrs_rsp->num_attr; i++ )
        {
            rsp_size = sizeof(uint16_t) + /* handle*/
                       sizeof(uint8_t)  + /* status */
                       sizeof(uint8_t)  + /* element type ID */
                       sizeof(uint16_t) + /* element string length */
                       elem_attrs_rsp->p_attrs[i].name.str_len;

            /* Make sure that there is enough room in the allocated buffer for the result */
            if ( rsp_size <= WICED_BUFF_MAX_SIZE )
            {
                WICED_BT_TRACE( "[%s]: rsp_size: %d attr: %d, strlen: %d\n", __FUNCTION__,
                                rsp_size,
                                elem_attrs_rsp->p_attrs[i].attr_id,
                                elem_attrs_rsp->p_attrs[i].name.str_len);
                rsp = (uint8_t *)wiced_bt_get_buffer( rsp_size );
                if (rsp != NULL)
                {
                    /* Playing Time attribute is an ASCII string containing milli-sec */
                    /* We need to check case where a duration of 0 is received */
                    if ((elem_attrs_rsp->p_attrs[i].attr_id == AVRC_MEDIA_ATTR_ID_PLAYING_TIME) &&
                        (elem_attrs_rsp->p_attrs[i].name.str_len >= 3))
                    {
                        /* Convert from milli-sec to sec (by ignoring the last 3 digits) */
                        elem_attrs_rsp->p_attrs[i].name.str_len -= 3;
                        rsp_size -= 3;
                    }
                    rsp[0] = handle;
                    rsp[1] = 0;
                    rsp[2] = avrc_status_to_wiced_result( avrc_rsp->rsp.status );
                    rsp[3] = ( uint8_t ) elem_attrs_rsp->p_attrs[i].attr_id;
                    rsp[4] = elem_attrs_rsp->p_attrs[i].name.str_len & 0xff;
                    rsp[5] = ( elem_attrs_rsp->p_attrs[i].name.str_len >> 8) & 0xff;
                    memcpy( &rsp[6], elem_attrs_rsp->p_attrs[i].name.p_str, elem_attrs_rsp->p_attrs[i].name.str_len );

                    hci_control_send_avrc_event( HCI_CONTROL_AVRC_CONTROLLER_EVENT_CURRENT_TRACK_INFO, rsp, (uint16_t)rsp_size );
                    wiced_bt_free_buffer(rsp);
                }
            }
        }
    }
}

/**
 *
 * Function         avrc_send_app_setting_info
 *
 *                  Send app seeting infomation up to the MCU after a conneciton
 *                  to allow the app there to know what controls are available and
 *                  what their available settings are.
 *
 * @param[in]       remote_addr : Address of the peer device
 *
 * @return          Nothing
 */
void avrc_send_app_setting_info(uint8_t handle)
{
    uint8_t         i, j, k;
    uint8_t         rsp[6] = {0, 0, 0, 0, 0, 0};
    wiced_result_t  result;

    rsp[0] = handle;
    /* Place available settings for the available controls in the
     * Array to be sent to the MCU */
    for (i=1, j = 2; i<=MAX_POSSIBLE_APP_ATTR_SETTINGS; i++, j++)
    {
        for (k=0; k<rc_app_cb.app_setting[i].num_possible_values; k++)
        {
            rsp[j] |= 1 << rc_app_cb.app_setting[i].possible_values[k];
        }
    }

    /* Send the information across the HCI */
    hci_control_send_avrc_event( HCI_CONTROL_AVRC_CONTROLLER_EVENT_SETTING_AVAILABLE, rsp, ( uint16_t )j );

    /* For each available app attribute determine the current settings.
     * The resulting event will update the MCU application */
    for (i=1; i<=MAX_POSSIBLE_APP_ATTR_SETTINGS; i++)
    {
        if (rc_app_cb.app_setting[i].available)
        {
            result = wiced_bt_avrc_ct_get_player_value_cmd( handle, 1, &i );
            if (result != WICED_SUCCESS)
            {
                WICED_BT_TRACE( "[%s]: ERROR!!! sending request for current setting of attr %d. result: %d\n",
                                __FUNCTION__, i, result);
            }
        }
    }
}

/**
 *
 * Function         avrc_handle_list_player_app_attribute_rsp
 *
 *                  Callback invoked by avrc_response_cback to inform of a response to the
 *                  get player application attributes request.
 *
 * @param[in]       remote_addr : Address of the peer device
 * @param[in]       avrc_rsp    : AVRC response messages
 *
 * @return          Nothing
 */
void avrc_handle_list_player_app_attribute_rsp(uint8_t handle,
                                               wiced_bt_avrc_response_t *avrc_rsp)
{
    wiced_bt_avrc_list_app_attr_rsp_t *list_app_attr = &avrc_rsp->list_app_attr;
    uint8_t         i;
    wiced_result_t  result;

    WICED_BT_TRACE( "[%s]: handle: <%d> PDU: 0x%x\n", __FUNCTION__,  handle, avrc_rsp->pdu);

    rc_app_cb.num_app_settings = rc_app_cb.num_app_settings_init = 0;
    if (list_app_attr->num_attr <= MAX_POSSIBLE_APP_ATTR_SETTINGS)
    {
        for (i = 0; i < list_app_attr->num_attr; i++)
        {
            WICED_BT_TRACE( "[%s]: attribute[%d]: %d\n", __FUNCTION__, i, list_app_attr->attrs[i] );

            if (list_app_attr->attrs[i] <= MAX_POSSIBLE_APP_ATTR_SETTINGS)
            {
                /* Cache the available settings and count them. this is necessary to determine
                 * when all possible value requests are completed */
                rc_app_cb.app_setting[list_app_attr->attrs[i]].available = WICED_TRUE;
                rc_app_cb.num_app_settings++;
            }
        }

        /* For each app attribute determine the possible values */
        for ( i = 0; i < list_app_attr->num_attr; i++ )
        {
            WICED_BT_TRACE( "[%s]: sending request for poss settings.\n", __FUNCTION__ );
            result = wiced_bt_avrc_ct_list_player_values_cmd(
                        handle, list_app_attr->attrs[i] );
            if ( result != WICED_SUCCESS )
            {
                break;
            }
        }
    }
}

/**
 *
 * Function         avrc_handle_list_player_app_values_rsp
 *
 *                  Callback invoked by avrc_response_cback to inform of a response to the
 *                  get player possible application attribute values request.
 *
 * @param[in]       remote_addr : Address of the peer device
 * @param[in]       avrc_rsp    : AVRC response messages
 *
 * @return          Nothing
 */
void avrc_handle_list_player_app_values_rsp(uint8_t handle,
                                           wiced_bt_avrc_response_t *avrc_rsp)
{
    wiced_bt_avrc_list_app_values_rsp_t *list_app_values = &avrc_rsp->list_app_values;
    uint8_t         attr_id = list_app_values->opcode;
    int             i;

    WICED_BT_TRACE( "[%s]: handle: <%d> PDU: 0x%x\n", __FUNCTION__,  handle, avrc_rsp->pdu );

    rc_app_cb.num_app_settings_init++;

    /* Cache the possible values for the player app attribute */
    rc_app_cb.app_setting[attr_id].num_possible_values = list_app_values->num_val;
    for ( i = 0; i < list_app_values->num_val; i++ )
    {
        rc_app_cb.app_setting[attr_id].possible_values[i] = list_app_values->vals[i];
    }

    /* Check if all app possible setting requests have completed */
    if (rc_app_cb.num_app_settings_init == rc_app_cb.num_app_settings)
    {
        /* All setting possible values have been acquired. Inform the MCU */
        avrc_send_app_setting_info(handle);
    }
}

/**
 *
 * Function         avrc_handle_list_player_app_values_rsp
 *
 *                  Callback invoked by avrc_response_cback to inform of a response to the
 *                  get current player application attribute value request.
 *
 * @param[in]       remote_addr : Address of the peer device
 * @param[in]       avrc_rsp    : AVRC response messages
 *
 * @return          Nothing
 */
void avrc_handle_get_player_app_value_rsp(uint8_t handle,
                                           wiced_bt_avrc_response_t *avrc_rsp)
{
    wiced_bt_avrc_get_cur_app_value_rsp_t *get_cur_app_val = &avrc_rsp->get_cur_app_val;

    WICED_BT_TRACE( "[%s]: handle: <%d> PDU: 0x%x\n", __FUNCTION__,  handle, avrc_rsp->pdu);

    /* We only asked for one attribute setting. */
    if (get_cur_app_val->num_val == 1)
    {
        uint8_t attr_id  = get_cur_app_val->p_vals->attr_id;
        uint8_t attr_val = get_cur_app_val->p_vals->attr_val;
        int i;

        /* Make sure the value is in the set of the list of possibilities */
        for ( i = 0; i <= rc_app_cb.app_setting[attr_id].num_possible_values; i++ )
        {
            if ( attr_val == rc_app_cb.app_setting[attr_id].possible_values[i] )
            {
                uint8_t rsp[5] = {0, 0, 1, 0, 0};
                rsp[0] = handle;
                rc_app_cb.app_setting[attr_id].current_index = i;

                rsp[3] = attr_id;
                rsp[4] = attr_val;

                WICED_BT_TRACE( "[%s]: attribute: %d value: %d\n", __FUNCTION__, attr_id, attr_val);

                /* Send notification to the MCU about the current value(s) setting. */
                hci_control_send_avrc_event( HCI_CONTROL_AVRC_CONTROLLER_EVENT_SETTING_CHANGE, rsp, 5 );
                break;
            }
        }
    }
}

/**
 *
 * Function         avrc_response_cback
 *
 *                  Callback invoked by the AVRCP api to inform of a response to an
 *                  AVRCP request submitted or possibly an event that was registered
 *                  on our behalf by the API.
 *
 * @param[in]       remote_addr : Address of the peer device
 * @param[in]       avrc_rsp    : AVRC response messages
 *
 * @return          Nothing
 */
void avrc_response_cback(  uint8_t handle,
                          wiced_bt_avrc_response_t *avrc_rsp)
{
    WICED_BT_TRACE( "[%s]: handle: <%d> PDU: 0x%x\n", __FUNCTION__, handle, avrc_rsp->pdu);

    /* Check the status. If successful handle the response */

    switch ( avrc_rsp->pdu )
    {
    case AVRC_PDU_REGISTER_NOTIFICATION:
        avrc_handle_registered_notification_rsp( handle, avrc_rsp );
        break;

    case AVRC_PDU_GET_ELEMENT_ATTR:
        avrc_handle_element_attribute_rsp( handle, avrc_rsp );
        break;

    case AVRC_PDU_LIST_PLAYER_APP_ATTR:
        avrc_handle_list_player_app_attribute_rsp( handle, avrc_rsp );
        break;

    case AVRC_PDU_LIST_PLAYER_APP_VALUES:
        avrc_handle_list_player_app_values_rsp( handle, avrc_rsp );
        break;

    case AVRC_PDU_GET_CUR_PLAYER_APP_VALUE:
        avrc_handle_get_player_app_value_rsp( handle, avrc_rsp );
        break;

    default:
        WICED_BT_TRACE( "[%s]: unhandled response: PDU: 0x%x\n", __FUNCTION__, avrc_rsp->pdu);
        break;
    }
}

/**
 *
 * Function         avrc_command_cback
 *
 *                  Callback invoked by the AVRCP api to inform of a command sent
 *                  from the peer.
 *
 *                  NOTE: As an AVRCP 1.3 controller this does not need to
 *                        be handled.
 *
 * @param[in]       remote_addr : Address of the peer device
 * @param[in]       avrc_cmd    : AVRC command messages
 *
 * @return          Nothing
 */
void avrc_command_cback( uint8_t handle,
                         wiced_bt_avrc_command_t *avrc_cmd)
{
    WICED_BT_TRACE( "%s: Unsupported command callback PDU: 0x%x\n", __FUNCTION__,  avrc_cmd->pdu);
}

/**
 *
 * Function         avrc_passthrough_cback
 *
 *                  Callback invoked on completion of a passthrough command. If the
 *                  command had been a "PRESS" state command then the "RELEASE" is automatically
 *                  sent by this callback except in the case of Fast Forward and Rewind which
 *                  require MCU intervention.
 *
 * @param[in]       remote_addr   : Address of the peer device
 * @param[in]       avrc_pass_rsp : AVRC passthrough command response
 *
 * @return          Nothing
 */
void avrc_passthrough_cback(uint8_t handle,
                             wiced_bt_avrc_msg_pass_t *avrc_pass_rsp )
{
    WICED_BT_TRACE( "[%s]: handle: <%d> op_id: 0x%x\n", __FUNCTION__,  handle, avrc_pass_rsp->op_id );

    if ( avrc_pass_rsp->hdr.ctype == AVRC_RSP_ACCEPT )
    {
        /* Assume that if the state of the keypress is "press" they will want to "release" */
        if ( avrc_pass_rsp->state == AVRC_STATE_PRESS )
        {
            /* Exceptions for FFWD and REW */
            if ((avrc_pass_rsp->op_id != AVRC_ID_FAST_FOR) &&
                (avrc_pass_rsp->op_id != AVRC_ID_REWIND))
            {
                avrc_app_hci_pass_through(handle, avrc_pass_rsp->op_id, AVRC_STATE_RELEASE );
            }
        }
    }
    else
    {
        WICED_BT_TRACE( "%s: op_id: 0x%x failed: 0x%x\n", __FUNCTION__,
                        avrc_pass_rsp->op_id, avrc_pass_rsp->hdr.ctype );
    }

    /* It is currently assumed that the MCU does not care about the results of this request.
     * Therefore no events are sent up at this point */
}

/**
 *
 * Function         hci_control_rc_controller_init
 *
 *                 Initialize the AVRCP api as a controller and set the
 *                 necessary callbacks for it to invoke.
 *
 * @return          Nothing
 */
void hci_control_rc_controller_init( void )
{
    uint32_t local_features = REMOTE_CONTROL_FEATURE_CONTROLLER;

    /* Clear the control block */
    memset(&rc_app_cb, 0, sizeof(tRC_APP_CB));

    /* Call library interface for avrc initialization */
    wiced_bt_avrc_ct_init( local_features,
            avrc_supported_events,
            avrc_connection_state_cback,
            avrc_command_cback,
            avrc_response_cback,
            avrc_passthrough_cback );
}

/**
 *
 * Function         hci_control_avrc_handle_command
 *
 *                  Handler for the commands sent from the MCU over the HCI to invoke AVRCP
 *                  connection and peer requests
 *
 * @param[in]       cmd_opcode   : HCI Command invoked by the MCU app
 * @param[in]       p_data       : command data bytes
 * @param[in]       payload_len  : command data byte length
 *
 * @return          uint8_t (HCI return status)
 */
uint8_t hci_control_avrc_handle_ctrlr_command( uint16_t cmd_opcode, uint8_t *p_data, uint16_t payload_len )
{
    wiced_result_t status = WICED_ERROR;
    uint16_t handle = (p_data[0]) | (p_data[1] << 8);

    switch ( cmd_opcode )
    {
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_CONNECT:                      /* Initiate a connection to the peer. */
        status = avrc_app_hci_connect( p_data, payload_len ) == WICED_PENDING ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;

    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_DISCONNECT:                   /* Disconnect a connection to the peer. */
        status = avrc_app_hci_disconnect_connection( handle ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;

    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PLAY:                         /* Passthrough Play Command */
        status = avrc_app_hci_pass_through (handle, AVRC_ID_PLAY, AVRC_STATE_PRESS ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;

    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_STOP:                         /* Passthrough Stop Command */
        status = avrc_app_hci_pass_through(handle, AVRC_ID_STOP, AVRC_STATE_PRESS ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;

    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PAUSE:                        /* Passthrough Pause Command */
        status = avrc_app_hci_pass_through (handle, AVRC_ID_PAUSE, AVRC_STATE_PRESS ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;

    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_BEGIN_FAST_FORWARD:          /* Passthrough FFWD Command */
        status = avrc_app_hci_pass_through (handle, AVRC_ID_FAST_FOR, AVRC_STATE_PRESS ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;

    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_END_FAST_FORWARD:             /* Passthrough FFWD Command */
        status = avrc_app_hci_pass_through (handle, AVRC_ID_FAST_FOR, AVRC_STATE_RELEASE ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;

    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_BEGIN_REWIND:                 /* Passthrough Rewind Command */
        status = avrc_app_hci_pass_through (handle, AVRC_ID_REWIND, AVRC_STATE_PRESS ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;

    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_END_REWIND:                   /* Passthrough Rewind Command */
        status = avrc_app_hci_pass_through (handle, AVRC_ID_REWIND, AVRC_STATE_RELEASE ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;

    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_NEXT_TRACK:                   /* Passthrough Next Command */
        status = avrc_app_hci_pass_through (handle, AVRC_ID_FORWARD, AVRC_STATE_PRESS ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;

    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PREVIOUS_TRACK:               /* Passthrough Prev Command */
        status = avrc_app_hci_pass_through (handle, AVRC_ID_BACKWARD, AVRC_STATE_PRESS ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;

    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_UP:                    /* Passthrough Vol Up Command */
        status = avrc_app_hci_pass_through (handle, AVRC_ID_VOL_UP, AVRC_STATE_PRESS ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;

    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_DOWN:                  /* Passthrough Vol Down Command */
        status = avrc_app_hci_pass_through (handle, AVRC_ID_VOL_DOWN, AVRC_STATE_PRESS ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;

    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_GET_TRACK_INFO:               /* Get Track Metadata */
        status = avrcp_app_hci_get_attrs_cmd(p_data, payload_len);
        break;

    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_SET_EQUALIZER_STATUS:         /* Turn Equalizer On/Off */
        status = avrcp_app_hci_set_player_settings (handle,  AVRC_PLAYER_SETTING_EQUALIZER );
        break;

    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_SET_REPEAT_MODE:              /* Set Repeat Mode */
        status = avrcp_app_hci_set_player_settings (handle,  AVRC_PLAYER_SETTING_REPEAT );
        break;

    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_SET_SHUFFLE_MODE:             /* Set Shuffle Mode */
        status = avrcp_app_hci_set_player_settings (handle,  AVRC_PLAYER_SETTING_SHUFFLE );
        break;

    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_SET_SCAN_STATUS:              /* Set Scan Mode to Off, All tracks or Group scan  */
        status = avrcp_app_hci_set_player_settings (handle,  AVRC_PLAYER_SETTING_SCAN );
        break;

    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_LEVEL:
        status = app_avrc_hci_control_volume( p_data, payload_len );
        break;

    default:
        WICED_BT_TRACE( "[%s] Unsupported_opcode: 0x%x\n", __FUNCTION__, cmd_opcode );
        status = HCI_CONTROL_STATUS_UNKNOWN_COMMAND;
        break;
    }
    hci_control_send_command_status_evt( HCI_CONTROL_AVRC_CONTROLLER_EVENT_COMMAND_STATUS, status );

    return (status == WICED_SUCCESS) ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;

}
