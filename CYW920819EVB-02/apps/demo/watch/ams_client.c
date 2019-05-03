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
 * BLE Client for Apple Media Service (AMS).  See
 * https://developer.apple.com/library/ios/documentation/CoreBluetooth/Reference/AppleMediaService_Reference/Introduction/Introduction.html
 *
 * During initialization the app performs GATT discovery and registers
 * to receive various notifications from the player on the iOS device.
 * Received notifications are translated to BT AVRC events and passed
 * to the MCU over the UART/SPI transport.  MCU can send AVRC commands
 * which are translated into AMS commands and sent to iOS device.
 *
 * Features demonstrated
 *  - performing GATT service discovery
 *  - working with AMS service on iOS device
 *
 */
#include "wiced.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_avrc_defs.h"
#include "hci_control_api.h"
#include "ams_client.h"

#include "le_slave.h"
#include "wiced_memory.h"
#include "wiced_transport.h"
#include "string.h"

/******************************************************
 *                      Constants
 ******************************************************/
// 89D3502B-0F36-433A-8EF4-C502AD55F8DC
//const char AMS_SERVICE[]             = {0xDC, 0xF8, 0x55, 0xAD, 0x02, 0xC5, 0xF4, 0x8E, 0x3A, 0x43, 0x36, 0x0F, 0x2B, 0x50, 0xD3, 0x89};

// 89D3502B-0F36-433A-8EF4-C502AD55F8DC
const char AMS_SERVICE[]             = {0xDC, 0xF8, 0x55, 0xAD, 0x02, 0xC5, 0xF4, 0x8E, 0x3A, 0x43, 0x36, 0x0F, 0x2B, 0x50, 0xD3, 0x89};

// Remote Command: UUID 9B3C81D8-57B1-4A8A-B8DF-0E56F7CA51C2 (writeable)
const char AMS_REMOTE_CONTROL[]      = {0xC2, 0x51, 0xCA, 0xF7, 0x56, 0x0E, 0xDF, 0xB8, 0x8A, 0x4A, 0xB1, 0x57, 0xD8, 0x81, 0x3C, 0x9B};

// Entity Update: UUID 2F7CABCE-808D-411F-9A0C-BB92BA96C102 (writeable with response, notifiable)
const char AMS_ENTITY_UPDATE[]       = {0x02, 0xC1, 0x96, 0xBA, 0x92, 0xBB, 0x0C, 0x9A, 0x1F, 0x41, 0x8D, 0x80, 0xCE, 0xAB, 0x7C, 0x2F};

// Entity Attribute: UUID C6B2F38C-23AB-46D8-A6AB-A3A870BBD5D7 (readable, writeable)
const char AMS_ENTITY_ATTRIBUTE[]    = {0xD7, 0xD5, 0xBB, 0x70, 0xA8, 0xA3, 0xAB, 0xA6, 0xD8, 0x46, 0xAB, 0x23, 0x8C, 0xF3, 0xB2, 0xC6};

// Following flags can be change to 1 or 0 to enable or
// disable additional features
#define AMS_ADDITIONAL_TRACE            0   // Set to one to print additional traces to the debug output
#define AMS_SUPPORT_PLAYLIST_INFO       1   // Set to 1 to register to receive number of tracks/track number
#define AMS_SUPPORT_PLAYER_NAME         1   // Set to 1 to register to receive player name
#define AMS_SUPPORT_TRACK_POSITION      1   // Set to 1 to register to receive track length/track position


//#ifdef WICED_BT_TRACE_ENABLE
#if AMS_ADDITIONAL_TRACE

#define AMS_ENTITY_ID_MAX               3
static char *EntityId[] =
{
    "Player",
    "Queue",
    "Track",
    "Unknown"
};

#define AMS_PLAYER_ATTRIBUTE_ID_MAX     3
static char *PlayerAttributeId[] =
{
    "PlayerAttributeIDName",
    "PlayerAttributeIDPlaybackInfo",
    "PlayerAttributeIDVolume",
    "Unknown"
};

#define AMS_QUEUE_ATTRIBUTE_ID_MAX      4
static char *QueueAttributeId[] =
{
    "QueueAttributeIDIndex",
    "QueueAttributeIDCount",
    "QueueAttributeIDShuffleMode",
    "QueueAttributeIDRepeatMode",
    "Unknown"
};

#define AMS_TRACK_ATTRIBUTE_ID_MAX      4
static char *TrackAttributeId[] =
{
    "TrackAttributeIDArtist",
    "TrackAttributeIDAlbum",
    "TrackAttributeIDTitle",
    "TrackAttributeIDDuration",
    "Unknown"
};
#endif

#define AMS_COMMAND_ID_GET_NOTIFICATION_ATTRIBUTES     0
#define AMS_COMMAND_ID_GET_APP_ATTRIBUTES              1
#define AMS_COMMAND_ID_PERFORM_NOTIFICATION_ACTION     2


// service discovery states
enum
{
    AMS_CLIENT_STATE_IDLE                                           = 0x00,
    AMS_CLIENT_STATE_DISCOVER_ENTITY_UPDATE_CCCD                    = 0x01,
    AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_CCCD                       = 0x02,
    AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_PLAYER                     = 0x03,
    AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_QUEUE                      = 0x04,
    AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_TRACK                      = 0x05,
};

/******************************************************
 *                     Structures
 ******************************************************/
typedef struct t_AMS_CLIENT
{
    uint8_t   state;
    uint16_t  conn_id;
    BD_ADDR   remote_addr;
    uint16_t  ams_e_handle;
    uint16_t  remote_control_char_hdl;
    uint16_t  remote_control_val_hdl;
    uint16_t  entity_update_char_hdl;
    uint16_t  entity_update_val_hdl;
    uint16_t  entity_update_cccd_hdl;
    uint16_t  entity_attribute_char_hdl;
    uint16_t  entity_attribute_val_hdl;

    uint8_t   playback_status;
} AMS_CLIENT;

/******************************************************
 *               Variables Definitions
 ******************************************************/
AMS_CLIENT  ams_client;

// Following table translates from AMS play status to AVRC status
uint8_t ams_client_to_hci_playback_status[] = {AVRC_PLAYSTATE_PAUSED, AVRC_PLAYSTATE_PLAYING, AVRC_PLAYSTATE_REV_SEEK, AVRC_PLAYSTATE_FWD_SEEK};

// Following table translates from AMS shuffle mode to AVRC shuffle mode
uint8_t ams_client_to_hci_shuffle_mode[]    = {AVRC_PLAYER_VAL_OFF, AVRC_PLAYER_VAL_ON, AVRC_PLAYER_VAL_ALL_SHUFFLE};

// Following table translates from AMS repeat mode to AVRC repeat mode
uint8_t ams_client_to_hci_repeat_mode[]     = {AVRC_PLAYER_VAL_OFF, AVRC_PLAYER_VAL_ON, AVRC_PLAYER_VAL_ALL_REPEAT};

// following is the list of notification attributes that we are going
// to request for entity player.  Compile out attribute of no interest.
uint8_t  ams_client_player_notification_attribute[] =
{
#if AMS_SUPPORT_PLAYER_NAME
    AMS_PLAYER_ATTRIBUTE_ID_NAME,
#endif
    AMS_PLAYER_ATTRIBUTE_ID_PLAYBACK_INFO,
    AMS_PLAYER_ATTRIBUTE_ID_VOLUME
};

// following is the list of notification attributes that we are going
// to request for entity track.  Compile out attribute of no interest.
uint8_t  ams_client_track_notification_attribute[] =
{
    AMS_TRACK_ATTRIBUTE_ID_ARTIST,
    AMS_TRACK_ATTRIBUTE_ID_ALBUM,
    AMS_TRACK_ATTRIBUTE_ID_TITLE,
#if AMS_SUPPORT_TRACK_POSITION
    AMS_TRACK_ATTRIBUTE_ID_DURATION
#endif
};

// following is the list of notification attributes that we are going
// to request for entity queue.  Compile out attribute of no interest.
uint8_t  ams_client_queue_notification_attribute[] =
{
#if AMS_SUPPORT_PLAYLIST_INFO
    AMS_QUEUE_ATTRIBUTE_ID_INDEX,
    AMS_QUEUE_ATTRIBUTE_ID_COUNT,
#endif
    AMS_QUEUE_ATTRIBUTE_ID_SHUFFLE_MODE,
    AMS_QUEUE_ATTRIBUTE_ID_REPEAT_MODE
};


/******************************************************
 *               Function Prototypes
 ******************************************************/
static init_complete_cback_t     ams_client_initialize_complete_callback = NULL;
static ams_notification_cback_t  ams_client_message_received_callback = NULL;
static wiced_bt_gatt_status_t    ams_client_entity_update_write(uint8_t entity_id, uint8_t *p_attributes, int num_attributes);
static void                      ams_client_process_playback_info(uint8_t *p_info, int len);
static void                      ams_client_process_volume_info(uint8_t *p_info, int len);
static void                      ams_client_process_shuffle_mode(uint8_t *p_info, int len);
static void                      ams_client_process_repeat_mode(uint8_t *p_info, int len);
static wiced_result_t            ams_client_send_track_info(uint8_t info_id, uint8_t *attribute, uint8_t attribute_len);
static wiced_result_t            ams_client_send_play_status_change(uint8_t playback_status);
static wiced_result_t            ams_client_send_volume_level_change(uint8_t volume_level);
static wiced_result_t            ams_client_send_setting_change(uint8_t setting_id, uint8_t mode);

#if AMS_SUPPORT_PLAYLIST_INFO
static void                      ams_client_process_queue_index(uint8_t *p_info, int len);
#endif

#if AMS_SUPPORT_TRACK_POSITION
static wiced_result_t            ams_client_send_play_position_change(uint32_t elapsed_time);
static int                       ams_client_process_get_track_duration_len(uint8_t *p_info, int len);
#endif

#if AMS_SUPPORT_PLAYER_NAME
static wiced_result_t            ams_client_send_player_name_change(uint8_t *p_name, uint8_t len);
#endif

extern wiced_bt_gatt_status_t watch_util_set_client_config_descriptor(uint16_t conn_id, uint16_t handle, uint16_t value);
extern void hci_control_send_command_status_evt( uint16_t code, uint8_t status );

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 * Connection up event from the main application
 */
void ams_client_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    ams_client.conn_id = p_conn_status->conn_id;

    // save address of the connected device and print it out.
    memcpy(ams_client.remote_addr, p_conn_status->bd_addr, sizeof(ams_client.remote_addr));

    WICED_BT_TRACE("ams_client.connection_up: %08x%04x %d\n",
                (ams_client.remote_addr[5] << 24) + (ams_client.remote_addr[4] << 16) +
                (ams_client.remote_addr[3] << 8) + ams_client.remote_addr[2],
                (ams_client.remote_addr[1] << 8) + ams_client.remote_addr[0],
                ams_client.conn_id);
}

/*
 * Connection down event from the main application
 */
void ams_client_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    WICED_BT_TRACE("ams_client_connection_down:%08x%04x handle:%d\n",
                (ams_client.remote_addr[5] << 24) + (ams_client.remote_addr[4] << 16) +
                (ams_client.remote_addr[3] << 8) + ams_client.remote_addr[2],
                (ams_client.remote_addr[1] << 8) + ams_client.remote_addr[0],
                ams_client.conn_id);

	memset (&ams_client, 0, sizeof (ams_client));
}

/*
 * Command from the main app to start search for characteristics
 */
int ams_client_initialize(uint16_t conn_id, uint16_t s_handle, uint16_t e_handle, init_complete_cback_t initialize_complete_callback, ams_notification_cback_t message_received_callback)
{
    WICED_BT_TRACE("[%s] e_handle:%04x\n", __FUNCTION__, e_handle);

    if ((s_handle == 0) || (e_handle == 0))
        return WICED_FALSE;

    ams_client_initialize_complete_callback = initialize_complete_callback;
    ams_client_message_received_callback    = message_received_callback;

    memset (&ams_client, 0, sizeof (ams_client));

    ams_client.conn_id      = conn_id;
    ams_client.ams_e_handle = e_handle;
    ams_client.state        = AMS_CLIENT_STATE_IDLE;

    watch_util_send_discover(conn_id, GATT_DISCOVER_CHARACTERISTICS, 0, s_handle, e_handle);

    WICED_SUPPRESS_WARNINGS(ams_client_message_received_callback);

    return TRUE;
}

/*
 * Process discovery results from the stack.  We are looking for 3 characteristics
 * remote control, entity update, and entity attribute.  The second has client
 * configuration descriptor (CCCD).
 */
wiced_bt_gatt_status_t ams_client_discovery_result(wiced_bt_gatt_discovery_result_t *p_data)
{
    WICED_BT_TRACE("[%s]\n", __FUNCTION__);

    if (p_data->discovery_type == GATT_DISCOVER_CHARACTERISTICS)
    {
        // Result for characteristic discovery.  Save appropriate handle based on the UUID.
        wiced_bt_gatt_char_declaration_t *p_char = &p_data->discovery_data.characteristic_declaration;
        if (p_char->char_uuid.len == 16)
        {
            if (memcmp(p_char->char_uuid.uu.uuid128, AMS_REMOTE_CONTROL, 16) == 0)
            {
                ams_client.remote_control_char_hdl = p_char->handle;
                ams_client.remote_control_val_hdl  = p_char->val_handle;
                WICED_BT_TRACE("remote control hdl:%04x-%04x", ams_client.remote_control_char_hdl, ams_client.remote_control_val_hdl);
            }
            else if (memcmp(p_char->char_uuid.uu.uuid128, AMS_ENTITY_UPDATE, 16) == 0)
            {
                ams_client.entity_update_char_hdl = p_char->handle;
                ams_client.entity_update_val_hdl  = p_char->val_handle;
                WICED_BT_TRACE("entity update hdl:%04x-%04x", ams_client.entity_update_char_hdl, ams_client.entity_update_val_hdl);
            }
            else if (memcmp(p_char->char_uuid.uu.uuid128, AMS_ENTITY_ATTRIBUTE, 16) == 0)
            {
                ams_client.entity_attribute_char_hdl = p_char->handle;
                ams_client.entity_attribute_val_hdl  = p_char->val_handle;
                WICED_BT_TRACE("entity attribute hdl:%04x-%04x", ams_client.entity_attribute_char_hdl, ams_client.entity_attribute_val_hdl);
            }
        }
    }
    else if ((p_data->discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS) &&
             (p_data->discovery_data.char_descr_info.type.len == 2) &&
             (p_data->discovery_data.char_descr_info.type.uu.uuid16 == UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION))
    {
        if (ams_client.state == AMS_CLIENT_STATE_DISCOVER_ENTITY_UPDATE_CCCD)
        {
            ams_client.entity_update_cccd_hdl = p_data->discovery_data.char_descr_info.handle;
            WICED_BT_TRACE("entity_update_cccd_hdl hdl:%04x", ams_client.entity_update_cccd_hdl);
        }
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process discovery complete event from the stack
 */
wiced_bt_gatt_status_t ams_client_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data)
{
    uint16_t end_handle;

    WICED_BT_TRACE("[%s] state:%d\n", __FUNCTION__, ams_client.state);

    if (p_data->disc_type == GATT_DISCOVER_CHARACTERISTICS)
    {
        // done with AMS characteristics, start reading descriptor handles
        // make sure that all characteristics are present
        if ((ams_client.remote_control_char_hdl   == 0)  ||
            (ams_client.remote_control_val_hdl    == 0)  ||
            (ams_client.entity_update_char_hdl    == 0)  ||
            (ams_client.entity_update_val_hdl     == 0)  ||
            (ams_client.entity_attribute_char_hdl == 0)  ||
            (ams_client.entity_attribute_val_hdl  == 0))
        {
            // something is very wrong
            WICED_BT_TRACE("[%s] failed\n", __FUNCTION__);
            ams_client.state = AMS_CLIENT_STATE_IDLE;
            memset (&ams_client, 0, sizeof (ams_client));
            (*ams_client_initialize_complete_callback)(1);
            return WICED_BT_GATT_SUCCESS;
        }

        // search for descriptor from the characteristic value handle until the end of the
        // service or until the start of the next characteristic
        end_handle = ams_client.ams_e_handle;
        if (ams_client.remote_control_char_hdl > ams_client.entity_update_char_hdl)
            end_handle = ams_client.remote_control_char_hdl - 1;
        if ((ams_client.entity_attribute_char_hdl > ams_client.entity_update_char_hdl) && (ams_client.entity_attribute_char_hdl < end_handle))
            end_handle = ams_client.entity_attribute_char_hdl - 1;

        ams_client.state = AMS_CLIENT_STATE_DISCOVER_ENTITY_UPDATE_CCCD;
        watch_util_send_discover(p_data->conn_id, GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                 ams_client.entity_update_val_hdl + 1, end_handle);
    }
    else if (p_data->disc_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS)
    {
        if (ams_client.state == AMS_CLIENT_STATE_DISCOVER_ENTITY_UPDATE_CCCD)
        {
            // done with descriptor discovery, register for notifications for data source by writing 1 into CCCD.
            ams_client.state = AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_CCCD;
            watch_util_set_client_config_descriptor(p_data->conn_id, ams_client.entity_update_cccd_hdl, GATT_CLIENT_CONFIG_NOTIFICATION);
        }
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process read response from the stack.
 * Application passes it here if handle belongs to our service.
 */
void ams_client_read_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
}

/*
 * Process write response from the stack.
 * Application passes it here if handle belongs to our service.
 */
void ams_client_write_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
    WICED_BT_TRACE("[%s] state:%02x\n", __FUNCTION__, ams_client.state);

    // if we were writing to client configuration descriptor, start registration
    // for specific attributes
    if (ams_client.state == AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_CCCD)
    {
        ams_client.state = AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_PLAYER;
        if (sizeof(ams_client_player_notification_attribute) != 0)
        {
            ams_client_entity_update_write(AMS_ENTITY_ID_PLAYER, ams_client_player_notification_attribute, sizeof(ams_client_player_notification_attribute));
            return;
        }
    }
    if (ams_client.state == AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_PLAYER)
    {
        ams_client.state = AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_QUEUE;
        if (sizeof(ams_client_queue_notification_attribute) != 0)
        {
            ams_client_entity_update_write(AMS_ENTITY_ID_QUEUE, ams_client_queue_notification_attribute, sizeof(ams_client_queue_notification_attribute));
            return;
        }
    }
    if (ams_client.state == AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_QUEUE)
    {
        ams_client.state = AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_TRACK;
        if (sizeof(ams_client_track_notification_attribute) != 0)
        {
            ams_client_entity_update_write(AMS_ENTITY_ID_TRACK, ams_client_track_notification_attribute, sizeof(ams_client_track_notification_attribute));
            return;
        }
    }
    if (ams_client.state == AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_TRACK)
    {
        ams_client.state = AMS_CLIENT_STATE_IDLE;
        (*ams_client_initialize_complete_callback)(0);
    }
}

/*
 * Process GATT Notifications from the client.  Application passes it here only
 * if the handle belongs to this service.
 */
void ams_client_notification_handler(wiced_bt_gatt_operation_complete_t *p_data)
{
    uint16_t    handle = p_data->response_data.att_value.handle;
    uint8_t     *data  = p_data->response_data.att_value.p_data;
    uint16_t    len    = p_data->response_data.att_value.len;
    uint8_t     entity_id, attrib_id;

    // this service should only receive notifications for entity update characteristic
    // first 3 bytes are hardcoded at Entity ID, Attribute ID and EntityUpdateFlags
    if ((handle != ams_client.entity_update_val_hdl) || (len < 3))
    {
        WICED_BT_TRACE("AMS Notification bad handle:%02x, %d\n", (uint16_t )handle, len);
        return;
    }

    entity_id = data[0];
    attrib_id = data[1];

#if AMS_ADDITIONAL_TRACE
    {
        uint8_t *p_attribute_name = PlayerAttributeId[AMS_ENTITY_ID_PLAYER];

        if (entity_id == AMS_ENTITY_ID_PLAYER)
            p_attribute_name = (data[1] < AMS_PLAYER_ATTRIBUTE_ID_MAX) ? PlayerAttributeId[data[1]] : PlayerAttributeId[AMS_ENTITY_ID_PLAYER];
        else if (entity_id == AMS_ENTITY_ID_QUEUE)
            p_attribute_name = (data[1] < AMS_QUEUE_ATTRIBUTE_ID_MAX) ? QueueAttributeId[data[1]] : QueueAttributeId[AMS_ENTITY_ID_QUEUE];
        else if (entity_id == AMS_ENTITY_ID_TRACK)
            p_attribute_name = (data[1] < AMS_TRACK_ATTRIBUTE_ID_MAX) ? TrackAttributeId[data[1]] : TrackAttributeId[AMS_ENTITY_ID_TRACK];

        // buffer that we received data in should have some safe area at the end.  should be ok for debugging.
        p_data->response_data.att_value.p_data[len] = 0;

        WICED_BT_TRACE ("AMS Entity ID:%s Attribute:%s Flags:%04x Value:%s",
                (entity_id < AMS_ENTITY_ID_MAX) ? EntityId[entity_id] : EntityId[AMS_ENTITY_ID_MAX],
                p_attribute_name, data[2], &data[3]);
    }
#endif

    switch(entity_id)
    {
    case AMS_ENTITY_ID_PLAYER:
        switch(attrib_id)
        {
#if AMS_SUPPORT_PLAYER_NAME
        case AMS_PLAYER_ATTRIBUTE_ID_NAME:
            ams_client_send_player_name_change(&data[3], len - 3);
            break;
#endif
        case AMS_PLAYER_ATTRIBUTE_ID_PLAYBACK_INFO:
            ams_client_process_playback_info(&data[3], len - 3);
            break;
        case AMS_PLAYER_ATTRIBUTE_ID_VOLUME:
            ams_client_process_volume_info(&data[3], len - 3);
            break;
        default:
            break;
        }
        break;
    case AMS_ENTITY_ID_QUEUE:
        switch(attrib_id)
        {
#if AMS_SUPPORT_PLAYLIST_INFO
        case AMS_QUEUE_ATTRIBUTE_ID_INDEX:
            ams_client_process_queue_index(&data[3], len - 3);
            break;
        case AMS_QUEUE_ATTRIBUTE_ID_COUNT:
            ams_client_send_track_info(AVRC_MEDIA_ATTR_ID_NUM_TRACKS, &data[3], len - 3);
            break;
#endif
        case AMS_QUEUE_ATTRIBUTE_ID_SHUFFLE_MODE:
            ams_client_process_shuffle_mode(&data[3], len - 3);
            break;
        case AMS_QUEUE_ATTRIBUTE_ID_REPEAT_MODE:
            ams_client_process_repeat_mode(&data[3], len - 3);
            break;
        default:
            break;
        }
        break;
    case AMS_ENTITY_ID_TRACK:
        switch(attrib_id)
        {
        case AMS_TRACK_ATTRIBUTE_ID_ARTIST:
            ams_client_send_track_info(AVRC_MEDIA_ATTR_ID_ARTIST, &data[3], len - 3);
            break;
        case AMS_TRACK_ATTRIBUTE_ID_ALBUM:
            ams_client_send_track_info(AVRC_MEDIA_ATTR_ID_ALBUM, &data[3], len - 3);
            break;
        case AMS_TRACK_ATTRIBUTE_ID_TITLE:
            ams_client_send_track_info(AVRC_MEDIA_ATTR_ID_TITLE, &data[3], len - 3);
            break;
#if AMS_SUPPORT_TRACK_POSITION
        case AMS_TRACK_ATTRIBUTE_ID_DURATION:
            ams_client_send_track_info(AVRC_MEDIA_ATTR_ID_PLAYING_TIME, &data[3],
                    ams_client_process_get_track_duration_len(&data[3], len - 3));
            break;
#endif
        default:
            break;
        }
    }
}

/*
 * Process GATT Indications from the client.  Application passes it here only
 * if the handle belongs to this service.
 */
void ams_client_indication_handler(wiced_bt_gatt_operation_complete_t *p_data)
{
}

/*
 * Send command to iOS device to indicate which attributes are interested in for specific entity.
 */
wiced_bt_gatt_status_t ams_client_entity_update_write(uint8_t entity_id, uint8_t *p_attributes, int num_attributes)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    uint8_t                buf[sizeof(wiced_bt_gatt_value_t) + 10];
    wiced_bt_gatt_value_t *p_write = (wiced_bt_gatt_value_t*)buf;

    // Allocating a buffer to send the write request
    memset(buf, 0, sizeof(buf));

    p_write->handle   = ams_client.entity_update_val_hdl;
    p_write->len      = num_attributes + 1;
    p_write->auth_req = GATT_AUTH_REQ_NONE;
    p_write->value[0] = entity_id;
    memcpy (&p_write->value[1], p_attributes, num_attributes);

    status = wiced_bt_gatt_send_write(ams_client.conn_id, GATT_WRITE, p_write);

    WICED_BT_TRACE("wiced_bt_gatt_send_write conn_id:%d %d\n", ams_client.conn_id, status);
    return status;
}

/*
 * Send remote control command to iOS server.
 */
void ams_send_remote_command(uint16_t opcode)
{
    wiced_bool_t           bfound = WICED_TRUE;
    wiced_bt_gatt_value_t  write;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    // Allocating a buffer to send the write request
    memset(&write, 0, sizeof(wiced_bt_gatt_value_t));

    write.handle   = ams_client.remote_control_val_hdl;
    write.len      = 1;
    write.auth_req = GATT_AUTH_REQ_NONE;

    switch (opcode)
    {
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PLAY:
        write.value[0] = AMS_REMOTE_COMMAND_ID_PLAY;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PAUSE:
        write.value[0] = AMS_REMOTE_COMMAND_ID_PAUSE;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_NEXT_TRACK:
        write.value[0] = AMS_REMOTE_COMMAND_ID_NEXT_TRACK;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PREVIOUS_TRACK:
        write.value[0] = AMS_REMOTE_COMMAND_ID_PREVIOUS_TRACK;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_UP:
        write.value[0] = AMS_REMOTE_COMMAND_ID_VOLUME_UP;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_DOWN:
        write.value[0] = AMS_REMOTE_COMMAND_ID_VOLUME_DOWN;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_SET_REPEAT_MODE:
        write.value[0] = AMS_REMOTE_COMMAND_ID_ADVANCED_REPEAT_MODE;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_SET_SHUFFLE_MODE:
        write.value[0] = AMS_REMOTE_COMMAND_ID_ADVANCED_SHUFFLE_MODE;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_BEGIN_FAST_FORWARD:
        write.value[0] = AMS_REMOTE_COMMAND_ID_SKIP_FORWARD;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_BEGIN_REWIND:
        write.value[0] = AMS_REMOTE_COMMAND_ID_SKIP_BACKWARD;
        break;
   default:
        bfound = WICED_FALSE;
        break;
    }

    if (bfound)
    {
        status = wiced_bt_gatt_send_write(ams_client.conn_id, GATT_WRITE, &write);
        WICED_BT_TRACE("wiced_bt_gatt_send_write conn_id:%d %d\n", ams_client.conn_id, status);
    }
}

/*
 * Process playback information from the iOS device
 */
void ams_client_process_playback_info(uint8_t *p_info, int len)
{
    uint8_t playback_status = p_info[0] - '0';
    uint32_t elapsed_time = 0;

    WICED_BT_TRACE("playback info len:%d status:%d\n", len, playback_status);

    // Playback info concatenation of three comma-separated values
    if ((len < 2) || (playback_status > AMS_PLAYBACK_STATUS_MAX) || (p_info[1] != ','))
    {
        WICED_BT_TRACE("failed\n");
        return;
    }
    if (playback_status != ams_client.playback_status)
    {
        ams_client.playback_status = playback_status;
        ams_client_send_play_status_change(ams_client_to_hci_playback_status[playback_status]);
    }
#if AMS_SUPPORT_TRACK_POSITION
    p_info += 2;
    len -= 2;

    // second value is PlaybackRate: a string that represents the floating point value of the playback rate.  Skip it.
    while ((len != 0) && (*p_info != ','))
    {
        p_info++;
        len--;
    }
    if (len == 0)
        return;
    p_info++;
    while ((len != 0) && (*p_info != '.'))
    {
        elapsed_time = (elapsed_time * 10) + (*p_info - '0');
        p_info++;
        len--;
    }
    ams_client_send_play_position_change(elapsed_time);
#endif
}

/*
 * Process volume change notification from the iOS device
 */
void ams_client_process_volume_info(uint8_t *p_info, int len)
{
    uint8_t volume_level = 0;

    WICED_BT_TRACE("volume info len:%d\n", len);

    // A string that represents the floating point value of the volume, ranging from 0 (silent) to 1 (full volume).
    if ((len < 2) || (p_info[0] != '0') || (p_info[1] != '.'))
    {
        WICED_BT_TRACE("failed\n");
        return;
    }
    volume_level = (p_info[2] - '0') * 10;
    if (len > 3)
        volume_level += (p_info[3] - '0');

    ams_client_send_volume_level_change(volume_level);
}


#if AMS_SUPPORT_PLAYLIST_INFO
/*
 * ams_client_u32toa
 * Convert an uint32 to a string
 */
void ams_client_u32toa(char * p_buffer, int buffer_len, uint32_t value)
{
    int divisor = 1000000000;
    int digit;
    char * p = p_buffer;

    memset(p_buffer, 0, buffer_len);

    /* Search for the first significant (not null) dozen */
    while( divisor && ((value / divisor) == 0))
    {
        divisor /= 10;
    }

    if (divisor == 0)
    {
        *p = '0';
        return;
    }

    do
    {
        digit = value / divisor;
        *p++ = (char)(digit + '0');
        value -= digit * divisor;
        divisor /= 10;
    } while (divisor > 0);
}

/*
 * Process queue index change notification from the iOS device.
 * A string containing the integer value of the queue index, zero-based.
 * AVRC track number is 1 based
 * Convert the received string (number) to an uint32, add one and convert it
 * back to a string
 */
void ams_client_process_queue_index(uint8_t *p_info, int len)
{
    uint32_t track_number = 0;
    int     i;
    char    queue_idx_str[10]; /* uint32 requires 9 digits and '\0' */

    for (i = 0; i < len; i++)
    {
        track_number = (track_number * 10) + (p_info[i] - '0');
    }

    // AVRC track number is 1 based
    track_number += 1;

    ams_client_u32toa(queue_idx_str, sizeof(queue_idx_str), track_number);

    ams_client_send_track_info(AVRC_MEDIA_ATTR_ID_TRACK_NUM,
            (uint8_t*)queue_idx_str, strlen(queue_idx_str));
}
#endif

/*
 * Process shuffle mode change notification from the iOS device
 */
void ams_client_process_shuffle_mode(uint8_t *p_info, int len)
{
    uint8_t  shuffle_mode = p_info[0] - '0';

    if ((len < 1) || (shuffle_mode > AMS_SHUFFLE_MODE_MAX))
    {
        WICED_BT_TRACE("failed\n");
        return;
    }
    ams_client_send_setting_change(AVRC_PLAYER_SETTING_SHUFFLE, ams_client_to_hci_shuffle_mode[shuffle_mode]);
}

/*
 * Process shuffle mode change notification from the iOS device
 */
void ams_client_process_repeat_mode(uint8_t *p_info, int len)
{
    uint8_t  repeat_mode = p_info[0] - '0';

    if ((len < 1) || (repeat_mode > AMS_REPEAT_MODE_MAX))
    {
        WICED_BT_TRACE("failed\n");
        return;
    }
    ams_client_send_setting_change(AVRC_PLAYER_SETTING_REPEAT, ams_client_to_hci_repeat_mode[repeat_mode]);
}

/*
 * Process track duration notification from the iOS device
 * A string containing the floating point value of the total duration of the track in seconds.
 * WICED HCI AVRC is uint32 in seconds
 */
#if AMS_SUPPORT_TRACK_POSITION
int ams_client_process_get_track_duration_len(uint8_t *p_info, int len)
{
    int duration_len = 0;

    /* AMS sends the track duration using float (sec.ms) */
    /* Let's ignore the milli-sec part (starting from the '.') */
    while ((len != 0) && (*p_info++ != '.'))
    {
        duration_len++;
    }
    return duration_len;

}
#endif

/*
 * send track information change to the MCU
 */
wiced_result_t ams_client_send_track_info(uint8_t info_id, uint8_t *attribute, uint8_t attribute_len)
{
    wiced_result_t  rc = WICED_BT_ERROR;
    uint8_t         *p_event_data;
    uint16_t        msg_size = sizeof(uint16_t) + /* handle*/
                               sizeof(uint8_t)  + /* status */
                               sizeof(uint8_t)  + /* element type ID */
                               sizeof(uint16_t) + /* element string length */
                               attribute_len;
    WICED_BT_TRACE("[%s] id:%d\n", __FUNCTION__, info_id);
    p_event_data = (uint8_t *)wiced_bt_get_buffer(msg_size);
    if (p_event_data != NULL)
    {
        p_event_data[0] = 0;        // handle
        p_event_data[1] = 0;
        p_event_data[2] = 0;        // status
        p_event_data[3] = info_id;
        p_event_data[4] = attribute_len & 0xff;
        p_event_data[5] = (attribute_len >> 8) & 0xff;
        memcpy( &p_event_data[6], attribute, attribute_len );

        rc = wiced_transport_send_data( HCI_CONTROL_AVRC_CONTROLLER_EVENT_CURRENT_TRACK_INFO, p_event_data, msg_size );
        wiced_bt_free_buffer(p_event_data);
    }
    return rc;
}

/*
 *  send playback status to the MCU
 */
wiced_result_t ams_client_send_play_status_change(uint8_t playback_status)
{
    uint8_t event_data[3];

    WICED_BT_TRACE("[%s]:%d\n", __FUNCTION__, playback_status);

    event_data[0] = 0;                    // handle
    event_data[1] = 0;
    event_data[2] = playback_status;      // play status

    return wiced_transport_send_data(HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_STATUS, event_data, 3);
}

/*
 *  send playback position to the MCU
 */
#if AMS_SUPPORT_TRACK_POSITION
wiced_result_t ams_client_send_play_position_change(uint32_t elapsed_time)
{
    uint8_t event_data[6];

    WICED_BT_TRACE("[%s]:%d\n", __FUNCTION__, elapsed_time);

    event_data[0] = 0;                    // handle
    event_data[1] = 0;
    event_data[2] = elapsed_time & 0xff;  // play position
    event_data[3] = (elapsed_time >> 8)  & 0xff;
    event_data[4] = (elapsed_time >> 16) & 0xff;
    event_data[5] = (elapsed_time >> 24) & 0xff;

    return wiced_transport_send_data(HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_POSITION, event_data, 6);
}
#endif

/*
 *  send volume level change to the MCU
 */
wiced_result_t ams_client_send_volume_level_change(uint8_t volume_level)
{
    uint8_t event_data[3];

    WICED_BT_TRACE("[%s]:%d\n", __FUNCTION__, volume_level);

    event_data[0] = 0;                 // handle
    event_data[1] = 0;
    event_data[2] = volume_level;      // volume level status

    return wiced_transport_send_data(HCI_CONTROL_AVRC_TARGET_EVENT_VOLUME_LEVEL, event_data, 3);
}

/*
 *  send player name to the MCU
 */
#if AMS_SUPPORT_PLAYER_NAME
wiced_result_t ams_client_send_player_name_change(uint8_t *p_name, uint8_t len)
{
    uint8_t event_data[60];

    event_data[0] = 0;                 // handle
    event_data[1] = 0;

    if (len > sizeof(event_data) - 2)
        len = sizeof(event_data) - 2;

    memcpy(&event_data[2], p_name, len);

    return wiced_transport_send_data(HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAYER_CHANGE, event_data, len + 2);
}
#endif

/*
 *  send volume level change to the MCU
 */
wiced_result_t ams_client_send_setting_change(uint8_t setting_id, uint8_t mode)
{
    uint8_t event_data[5];

    WICED_BT_TRACE("[%s]:%d %d\n", __FUNCTION__, setting_id, mode);

    event_data[0] = 0;                  // handle
    event_data[1] = 0;
    event_data[2] = 1;                  // number of settings
    event_data[3] = setting_id;
    event_data[4] = mode;

    return wiced_transport_send_data(HCI_CONTROL_AVRC_CONTROLLER_EVENT_SETTING_CHANGE, event_data, 5);
}

/*
 * Return true if connection to AMS client is up
 */
wiced_bool_t hci_control_is_ams_connection_up()
{
    return (ams_client.conn_id != 0);
}

/*
 * Process HCI commands from the MCU
 */
void hci_control_ams_handle_command(uint16_t opcode, uint8_t *p_data, uint16_t payload_len)
{
    uint8_t status = HCI_CONTROL_STATUS_SUCCESS;

    WICED_BT_TRACE("cmd_opcode 0x%02x\n", opcode);

    switch (opcode)
    {
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PLAY:
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PAUSE:
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_NEXT_TRACK:
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PREVIOUS_TRACK:
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_UP:
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_DOWN:
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_SET_REPEAT_MODE:
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_SET_SHUFFLE_MODE:
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_BEGIN_FAST_FORWARD:
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_BEGIN_REWIND:
        ams_send_remote_command(opcode);
        break;

    default:
        WICED_BT_TRACE("cmd_opcode 0x%02x ignored\n", opcode);
        status = HCI_CONTROL_STATUS_UNKNOWN_COMMAND;
        break;
    }

    hci_control_send_command_status_evt( HCI_CONTROL_AVRC_CONTROLLER_EVENT_COMMAND_STATUS, status );
}
