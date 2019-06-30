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
 * This file implements AVRC application controlled over UART.
 *
 */
#include "hci_control.h"
#include "wiced_bt_avrc_tg.h"
#include "hci_control_rc_target.h"
#include "string.h"
#include "wiced_transport.h"

#ifdef APP_AVRC_SETTING_CHANGE_SUPPORTED
static void app_avrc_settings_cmd(void);
#endif
#ifdef CATEGORY_2_PASSTROUGH
extern wiced_result_t wiced_bt_avrc_tg_button_press ( uint8_t label, uint8_t state, uint8_t op_id );
#endif // CATEGORY_2_PASSTROUGH
typedef struct
{
    wiced_bool_t connected;
} hci_control_rc_target_cb_t;

static hci_control_rc_target_cb_t hci_control_rc_target_cb;

/*
 * AVRC init
 */
void hci_control_rc_target_init(void)
{
    WICED_BT_TRACE( "[%s]:\n\r", __FUNCTION__);
    memset(&hci_control_rc_target_cb, 0, sizeof(hci_control_rc_target_cb));
#ifdef WICED_APP_AUDIO_RC_TG_INCLUDED
    wiced_bt_avrc_tg_init(app_avrc_event_cback);
#endif

#ifdef APP_AVRC_SETTING_CHANGE_SUPPORTED
    app_avrc_settings_cmd();
#endif
}
/*
 * AVRC connection processing.  Pass connected device address and handle to the MCU.
 */
void app_avrc_device_connected(wiced_bt_device_address_t bd_addr, uint16_t handle)
{
    int i;
    uint8_t event_data[BD_ADDR_LEN + sizeof(handle) ];

    hci_control_rc_target_cb.connected = WICED_TRUE;

    WICED_BT_TRACE( "[%s] %B status %x handle %x\n", __FUNCTION__, bd_addr, handle );

    /* Build event payload   */
    for ( i = 0; i < BD_ADDR_LEN; i++ )                     /* bd address */
        event_data[i] = bd_addr[BD_ADDR_LEN - 1 - i];

    event_data[i++] = handle & 0xff;                        /* handle */
    event_data[i++]   = ( handle >> 8 ) & 0xff;

    wiced_transport_send_data( HCI_CONTROL_AVRC_TARGET_EVENT_CONNECTED, event_data, i );
}

/*
 * AVRC connection processing.  Pass disconnected handle to the MCU.
 */
void app_avrc_device_disconnected(uint16_t handle)
{
    uint8_t event_data[ sizeof(handle) ];

    WICED_BT_TRACE( "[%s] %B status %x handle %x\n", __FUNCTION__, handle );

    hci_control_rc_target_cb.connected = WICED_FALSE;

    event_data[0] = handle & 0xff;                        /* handle */
    event_data[1]   = ( handle >> 8 ) & 0xff;

    wiced_transport_send_data( HCI_CONTROL_AVRC_TARGET_EVENT_DISCONNECTED, event_data,
            sizeof(handle) );
}

/*
 * AVRC connection processing. Check if AVRC Target is connected.
 */
wiced_bool_t hci_control_rc_target_is_connected(void)
{
    return hci_control_rc_target_cb.connected;
}

#ifdef APP_AVRC_TRACK_INFO_SUPPORTED

/*
 * Called by MCU app when it updates track information
 */
void app_avrc_track_info_cmd(uint8_t *p_data, uint16_t payload_len)
{
    uint16_t  commandBytes = 0;
    uint8_t   attr_id;
    uint8_t   str_len;
    uint16_t  copy_len;
    wiced_bt_avrc_tg_track_attr_t track_attr;

    /* While there is at least enough payload left for an attribute id and length ... */
    while (payload_len != commandBytes)
    {
        attr_id = p_data[commandBytes++];
        str_len = p_data[commandBytes++];

        if (attr_id > APP_AVRC_MAX_ATTR)
        {
            WICED_BT_TRACE( "[%s] : attr_id %d > APP_AVRC_MAX_ATTR %d \n\r", __FUNCTION__, attr_id, APP_AVRC_MAX_ATTR);
            return;
        }

        copy_len = 0;
        if ((str_len > 0) && (str_len < payload_len))
        {
            copy_len = (str_len < APP_AVRC_MAX_ATTR_LEN) ? str_len : APP_AVRC_MAX_ATTR_LEN;

            memcpy(track_attr.p_str, &p_data[commandBytes], copy_len);
            track_attr.p_str[copy_len] = '\0';

            commandBytes += str_len;
        }
        else if (str_len != 0)
        {
            WICED_BT_TRACE( "[%s] : str_len %d >= payload_len %d \n\r", __FUNCTION__, str_len, payload_len);
            return;
        }

        track_attr.attr_id = attr_id;
        track_attr.str_len = copy_len;

        WICED_BT_TRACE( "[%s] : attr_id %d, len %d\n\r", __FUNCTION__, attr_id, copy_len);

        wiced_bt_rc_set_track_info(&track_attr);
    }

    /* Inform peer of track information change. */
    wiced_bt_rc_track_changed();

}


#endif

#ifdef APP_AVRC_SETTING_CHANGE_SUPPORTED

#ifdef HCI_CONTROL_PLAYER_REPEAT_ENABLED
    uint8_t repeat_poss_settings[] =
    {
        AVRC_PLAYER_VAL_OFF,                    /* Current Value (should be overridden by MCU */
#ifdef HCI_CONTROL_PLAYER_REPEAT_SINGLE_ENABLED
        AVRC_PLAYER_VAL_SINGLE_REPEAT,
#endif
#ifdef HCI_CONTROL_PLAYER_REPEAT_ALL_ENABLED
        AVRC_PLAYER_VAL_ALL_REPEAT,
#endif
#ifdef HCI_CONTROL_PLAYER_REPEAT_GROUP_ENABLED
        AVRC_PLAYER_VAL_GROUP_REPEAT,
#endif
    };
#endif

#ifdef HCI_CONTROL_PLAYER_SHUFFLE_ENABLED
    uint8_t shuffle_poss_settings[] =
    {
        AVRC_PLAYER_VAL_OFF,        /* Current Value (should be overridden by MCU */
#ifdef HCI_CONTROL_PLAYER_REPEAT_ALL_ENABLED
        AVRC_PLAYER_VAL_ALL_SHUFFLE,
#endif
#ifdef HCI_CONTROL_PLAYER_REPEAT_GROUP_ENABLED
        AVRC_PLAYER_VAL_GROUP_SHUFFLE,
#endif
    };
#endif
/*
 * internal helper function
 */
void app_avrc_settings_cmd(void)
{
    wiced_bt_avrc_tg_player_attr_t player_attr;

#ifdef HCI_CONTROL_PLAYER_REPEAT_ENABLED
    player_attr.attr_id     = AVRC_PLAYER_SETTING_REPEAT;
    player_attr.curr_value  = AVRC_PLAYER_VAL_OFF;
    player_attr.num_val     = sizeof(repeat_poss_settings);
    memcpy(player_attr.vals, repeat_poss_settings, sizeof(repeat_poss_settings));
    wiced_bt_rc_set_player_settings(&player_attr);
#endif

#ifdef HCI_CONTROL_PLAYER_SHUFFLE_ENABLED
    player_attr.attr_id     = AVRC_PLAYER_SETTING_SHUFFLE;
    player_attr.curr_value  = AVRC_PLAYER_VAL_OFF;
    player_attr.num_val     = sizeof(shuffle_poss_settings);
    memcpy(player_attr.vals, shuffle_poss_settings, sizeof(shuffle_poss_settings));
    wiced_bt_rc_set_player_settings(&player_attr);
#endif

}
/*
 * Called when peer changes repeat settings value, update the MCU app
 */
void app_avrc_set_repeat_settings_event(uint8_t val)
{
    WICED_BT_TRACE( "[%s]: val %d\n\r", __FUNCTION__, val);
    wiced_transport_send_data( HCI_CONTROL_AVRC_TARGET_EVENT_REPEAT_SETTINGS, &val, 1 );
}

/*
 * Called when peer changes Shuffle settings value, update the MCU app
 */
void app_avrc_set_shuffle_settings_event(uint8_t val)
{
    WICED_BT_TRACE( "[%s]: val %d\n\r", __FUNCTION__, val);
    wiced_transport_send_data( HCI_CONTROL_AVRC_TARGET_EVENT_SHUFFLE_SETTINGS, &val, 1 );
}

/*
 * Called when MCU app changes player setting value
 */
void app_avrc_settings_changed(uint8_t attr_id, uint8_t value)
{
    WICED_BT_TRACE( "[%s]: attr_id %d, value %d \n\r", __FUNCTION__, attr_id, value);
    wiced_bt_rc_player_setting_changed(attr_id, value);
}

#endif

#ifdef APP_AVRC_PLAY_STATUS_SUPPORTED
/*
 * Called when MCU app provides player staus
 */
void app_avrc_player_status_cmd(uint8_t *p_data, uint16_t payload_len)
{
    uint16_t  commandBytes = 0;
    wiced_bt_avrc_tg_play_status_t play_status;
    uint32_t *pl_data;

    play_status.play_state = p_data[commandBytes++];

    pl_data = (uint32_t *)&p_data[commandBytes];
    play_status.song_len = pl_data[0];
    play_status.song_pos = pl_data[1];

    WICED_BT_TRACE( "[%s]: state %d, len %d, pos %d\n\r", __FUNCTION__, play_status.play_state, play_status.song_len, play_status.song_pos);

    wiced_bt_rc_set_player_status(&play_status);
}
#endif

/*
 *  Called by MCU app to connect AVRC profile
 */
wiced_result_t avrc_app_hci_initiate_connection ( uint8_t* p_data, uint32_t len )
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_bt_device_address_t bd_addr;
    int i;

    for ( i = 0; i < BD_ADDR_LEN; i++ )
    {
        bd_addr[i] = p_data[BD_ADDR_LEN - i - 1];
    }

    if (hci_control_rc_target_is_connected() == WICED_TRUE)
    {
        WICED_BT_TRACE("[%s]: Already connected ", __FUNCTION__);
        result = WICED_ALREADY_CONNECTED;
    }
    else
    {
       WICED_BT_TRACE("[%s]: connect to %B", __FUNCTION__, bd_addr);
       wiced_bt_avrc_tg_initiate_open( bd_addr );
    }

    return result;
}

/*
 *  Called by MCU app to disconnect AVRC profile
 */
wiced_result_t avrc_app_hci_disconnect_connection (void)
{
    wiced_result_t result = WICED_SUCCESS;

    wiced_bt_avrc_tg_initiate_close();
    return result;
}

/*
 *  Called when MCU changes volume, send Absolute volume request to peer
 */
wiced_result_t app_avrc_hci_control_volume( uint8_t* p_data, uint32_t len )
{
    uint16_t handle = p_data[0] + (p_data[1] << 8);
    uint8_t  volume = ( (p_data[2] * MAX_AVRCP_VOLUME_LEVEL) + 50 ) / 100; /* Convert from percentage to AVRCP scale */

    return wiced_bt_avrc_tg_absolute_volume_changed(handle, volume);
}

/*
 *  Called to handle avrc commands from MCU app
 */
uint8_t hci_control_avrc_handle_command( uint16_t cmd_opcode, uint8_t *p_data, uint16_t payload_len )
{
    uint8_t status = HCI_CONTROL_STATUS_SUCCESS;

    WICED_BT_TRACE( "[%s] Enter... cmd_opcode: 0x%x\n\r", __FUNCTION__, cmd_opcode );

    switch ( cmd_opcode )
    {
        case HCI_CONTROL_AVRC_TARGET_COMMAND_CONNECT:                /* Initiate a connection to the peer. */
            status = avrc_app_hci_initiate_connection( p_data, payload_len ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
            break;

        case HCI_CONTROL_AVRC_TARGET_COMMAND_DISCONNECT:             /* Disconnect a connection to the peer. */
            status = avrc_app_hci_disconnect_connection() == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
            break;

#ifdef APP_AVRC_TRACK_INFO_SUPPORTED
        case HCI_CONTROL_AVRC_TARGET_COMMAND_TRACK_INFO:
            app_avrc_track_info_cmd(p_data, payload_len);
            break;
#endif

#ifdef APP_AVRC_SETTING_CHANGE_SUPPORTED
        case HCI_CONTROL_AVRC_TARGET_COMMAND_REPEAT_MODE_CHANGE:
            app_avrc_settings_changed(AVRC_PLAYER_SETTING_REPEAT, p_data[0]);
            break;

        case HCI_CONTROL_AVRC_TARGET_COMMAND_SHUFFLE_MODE_CHANGE:
            app_avrc_settings_changed(AVRC_PLAYER_SETTING_SHUFFLE, p_data[0]);
            break;

#if APP_AVRC_EQUALIZER_SETTING_SUPPORTED
        case HCI_CONTROL_AVRC_TARGET_COMMAND_EQUALIZER_STATUS_CHANGE:
            app_avrc_settings_changed(AVRC_PLAYER_SETTING_EQUALIZER, p_data[0]);
            break;

        case HCI_CONTROL_AVRC_TARGET_COMMAND_SCAN_STATUS_CHANGE:
            app_avrc_settings_changed(AVRC_PLAYER_SETTING_SCAN, p_data[0]);
            break;
#endif


#endif

#ifdef APP_AVRC_PLAY_STATUS_SUPPORTED
        case HCI_CONTROL_AVRC_TARGET_COMMAND_PLAYER_STATUS:
            app_avrc_player_status_cmd(p_data, payload_len);
            break;
#endif

#ifdef CATEGORY_2_PASSTROUGH
        case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_UP:              /* Send Volume Up */
            status = wiced_bt_avrc_tg_volume_button_press ( AVRC_ID_VOL_UP ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
            break;

        case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_DOWN:            /* SendVolume Down */
            status = wiced_bt_avrc_tg_volume_button_press (  AVRC_ID_VOL_DOWN ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
            break;
#endif
        /* Handle AVRCP Abs vol command here (AVRCP Controller block) as watch app is used as both controller and target */
        case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_LEVEL:
            status = app_avrc_hci_control_volume( p_data, payload_len );
            break;

#ifdef CATEGORY_2_PASSTROUGH
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_MUTE:
        status = wiced_bt_avrc_tg_button_press ( 0, AVRC_STATE_PRESS, AVRC_ID_MUTE) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;
#endif // CATEGORY_2_PASSTROUGH

        default:
            WICED_BT_TRACE( "[%s] Unsupported_opcode: 0x%x\n\r", __FUNCTION__, cmd_opcode );
            status = HCI_CONTROL_STATUS_UNKNOWN_COMMAND;
            break;
    }

    hci_control_send_command_status_evt( HCI_CONTROL_AVRC_TARGET_EVENT_COMMAND_STATUS, status );

    return status;
}

/*
 * hci_control_rc_target_passthrough_cmd_event
 * Send received Passthrough Command to MCU
 */
static void hci_control_rc_target_passthrough_cmd_event(wiced_bt_avrc_tg_passthrough_cmd_t *p_passthrough)
{
    uint8_t event_data[sizeof(uint16_t)];
    uint16_t wiced_evt_opcode;
    uint8_t *p = event_data;

    /* Every AVRC Wiced Event start with AVRC Handle */
    UINT16_TO_STREAM(p,  p_passthrough->handle);

    switch(p_passthrough->command)
    {
    case APP_AVRC_EVENT_PASSTHROUGH_CMD_PLAY:
        wiced_evt_opcode = HCI_CONTROL_AVRC_TARGET_EVENT_PLAY;
        break;
    case APP_AVRC_EVENT_PASSTHROUGH_CMD_PAUSE:
        wiced_evt_opcode = HCI_CONTROL_AVRC_TARGET_EVENT_PAUSE;
        break;
    case APP_AVRC_EVENT_PASSTHROUGH_CMD_STOP:
        wiced_evt_opcode = HCI_CONTROL_AVRC_TARGET_EVENT_STOP;
        break;
    case APP_AVRC_EVENT_PASSTHROUGH_CMD_NEXT_TRACK:
        wiced_evt_opcode = HCI_CONTROL_AVRC_TARGET_EVENT_NEXT_TRACK;
        break;
    case APP_AVRC_EVENT_PASSTHROUGH_CMD_PREVIOUS_TRACK:
        wiced_evt_opcode = HCI_CONTROL_AVRC_TARGET_EVENT_PREVIOUS_TRACK;
        break;
    default:
        WICED_BT_TRACE( "[%s] Unknown cmd:%d\n\r", __FUNCTION__, p_passthrough->command );
        return;
    }

    /* Send the event over HCI */
    wiced_transport_send_data( wiced_evt_opcode, event_data, p - event_data );
}

/*
 * AVRC event handler
 */
void app_avrc_event_cback(uint8_t event_id,  wiced_bt_rc_event_t *p_event)
{
    uint8_t event_data[sizeof(uint16_t) + sizeof(uint8_t)];
    uint8_t *p;

    WICED_BT_TRACE( "[%s]: id %d\n\r", __FUNCTION__, event_id);

    switch(event_id)
    {
    case APP_AVRC_EVENT_DEVICE_CONNECTED:
        {
            wiced_bt_avrc_tg_register_absolute_volume_change();

            /* peer device connected, send info to MCU app */
            app_avrc_device_connected(p_event->bd_addr, p_event->handle);
        }
        break;

    case APP_AVRC_EVENT_DEVICE_DISCONNECTED:
        /* peer device disconnected, send info to MCU app */
        app_avrc_device_disconnected(p_event->handle);
        break;

#ifdef APP_AVRC_SETTING_CHANGE_SUPPORTED
    case APP_AVRC_EVENT_REPEAT_SETTINGS_CHANGED:
        {
            /* peer changed repeat settings, send info to MCU app */
            app_avrc_set_repeat_settings_event(p_event->setting_val);
        }
        break;
    case APP_AVRC_EVENT_SHUFFLE_SETTINGS_CHANGED:
        {
            /* peer changed shuffle settings, send info to MCU app */
            app_avrc_set_shuffle_settings_event(p_event->setting_val);
        }
        break;
#endif

    case APP_AVRC_EVENT_PASSTHROUGH_CMD:
        hci_control_rc_target_passthrough_cmd_event(&p_event->passthrough_command);
        break;

    case APP_AVRC_EVENT_ABS_VOL_CHANGED:
        p = event_data;

        /* Every AVRC Wiced Event start with AVRC Handle */
        UINT16_TO_STREAM(p,  p_event->absolute_volume.handle);
        UINT8_TO_STREAM(p,  p_event->absolute_volume.volume);
        /* Send the event over HCI */
        wiced_transport_send_data( HCI_CONTROL_AVRC_TARGET_EVENT_VOLUME_LEVEL,
                event_data, p - event_data );
        break;

    }
}
