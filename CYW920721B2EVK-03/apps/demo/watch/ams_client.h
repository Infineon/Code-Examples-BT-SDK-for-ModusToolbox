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
* BLE Client for Apple Media Service (AMS)
*
* Defines external definitions and function prototype for the AMS client.
*/
#ifndef AMS_CLIENT__H
#define AMS_CLIENT__H

#include "wiced_app.h"

//Defines moved to source file to avoid multiple define errors
// 89D3502B-0F36-433A-8EF4-C502AD55F8DC
extern const char AMS_SERVICE[];//             = {0xDC, 0xF8, 0x55, 0xAD, 0x02, 0xC5, 0xF4, 0x8E, 0x3A, 0x43, 0x36, 0x0F, 0x2B, 0x50, 0xD3, 0x89};

// Remote Command: UUID 9B3C81D8-57B1-4A8A-B8DF-0E56F7CA51C2 (writeable)
extern const char AMS_REMOTE_CONTROL[];//      = {0xC2, 0x51, 0xCA, 0xF7, 0x56, 0x0E, 0xDF, 0xB8, 0x8A, 0x4A, 0xB1, 0x57, 0xD8, 0x81, 0x3C, 0x9B};

// Entity Update: UUID 2F7CABCE-808D-411F-9A0C-BB92BA96C102 (writeable with response, notifiable)
extern const char AMS_ENTITY_UPDATE[];//       = {0x02, 0xC1, 0x96, 0xBA, 0x92, 0xBB, 0x0C, 0x9A, 0x1F, 0x41, 0x8D, 0x80, 0xCE, 0xAB, 0x7C, 0x2F};

// Entity Attribute: UUID C6B2F38C-23AB-46D8-A6AB-A3A870BBD5D7 (readable, writeable)
extern const char AMS_ENTITY_ATTRIBUTE[];//    = {0xD7, 0xD5, 0xBB, 0x70, 0xA8, 0xA3, 0xAB, 0xA6, 0xD8, 0x46, 0xAB, 0x23, 0x8C, 0xF3, 0xB2, 0xC6};

//  RemoteCommandID values
#define AMS_REMOTE_COMMAND_ID_PLAY                      0
#define AMS_REMOTE_COMMAND_ID_PAUSE                     1
#define AMS_REMOTE_COMMAND_ID_TOGGLE_PLAY_PAUSE         2
#define AMS_REMOTE_COMMAND_ID_NEXT_TRACK                3
#define AMS_REMOTE_COMMAND_ID_PREVIOUS_TRACK            4
#define AMS_REMOTE_COMMAND_ID_VOLUME_UP                 5
#define AMS_REMOTE_COMMAND_ID_VOLUME_DOWN               6
#define AMS_REMOTE_COMMAND_ID_ADVANCED_REPEAT_MODE      7
#define AMS_REMOTE_COMMAND_ID_ADVANCED_SHUFFLE_MODE     8
#define AMS_REMOTE_COMMAND_ID_SKIP_FORWARD              9
#define AMS_REMOTE_COMMAND_ID_SKIP_BACKWARD             10

// EntityID values
#define AMS_ENTITY_ID_PLAYER                            0
#define AMS_ENTITY_ID_QUEUE                             1
#define AMS_ENTITY_ID_TRACK                             2

// EntityUpdateFlags
#define AMS_ENTITY_UPDATE_FLAG_TRUNCATED               (1 << 0)

// PlayerAttributeID values

#define AMS_PLAYBACK_STATUS_PAUSED                      0
#define AMS_PLAYBACK_STATUS_PLAYING                     1
#define AMS_PLAYBACK_STATUS_REWINDING                   2
#define AMS_PLAYBACK_STATUS_FAST_FORWARDING             3
#define AMS_PLAYBACK_STATUS_MAX                         AMS_PLAYBACK_STATUS_FAST_FORWARDING

// A string containing the localized name of the app.
#define AMS_PLAYER_ATTRIBUTE_ID_NAME                    0

// A concatenation of three comma-separated values:
//    PlaybackState: a string that represents the integer value of the playback state:
//        PlaybackStatePaused = 0
//        PlaybackStatePlaying = 1
//        PlaybackStateRewinding = 2
//        PlaybackStateFastForwarding = 3
//    PlaybackRate: a string that represents the floating point value of the playback rate.
//    ElapsedTime: a string that represents the floating point value of the elapsed time of the current track, in seconds, at the moment the value was sent to the MR.
#define AMS_PLAYER_ATTRIBUTE_ID_PLAYBACK_INFO           1

// A string that represents the floating point value of the volume, ranging from 0 (silent) to 1 (full volume).
#define AMS_PLAYER_ATTRIBUTE_ID_VOLUME                  2

// QueueAttributeID values

// A string containing the integer value of the queue index, zero-based.
#define AMS_QUEUE_ATTRIBUTE_ID_INDEX                    0

// A string containing the integer value of the total number of items in the queue.
#define AMS_QUEUE_ATTRIBUTE_ID_COUNT                    1

// A string containing the integer value of the shuffle mode. Table A-6 lists the shuffle mode constants.
#define AMS_QUEUE_ATTRIBUTE_ID_SHUFFLE_MODE             2

// A string containing the integer value value of the repeat mode. Table A-7 lists the repeat mode constants.
#define AMS_QUEUE_ATTRIBUTE_ID_REPEAT_MODE              3

// Shuffle Mode constants
#define AMS_SHUFFLE_MODE_OFF                            0
#define AMS_SHUFFLE_MODE_ONE                            1
#define AMS_SHUFFLE_MODE_ALL                            2
#define AMS_SHUFFLE_MODE_MAX                            AMS_SHUFFLE_MODE_ALL

// Repeat Mode constants
#define AMS_REPEAT_MODE_OFF                             0
#define AMS_REPEAT_MODE_ONE                             1
#define AMS_REPEAT_MODE_ALL                             2
#define AMS_REPEAT_MODE_MAX                             AMS_REPEAT_MODE_ALL

// TrackAttributeID values

// A string containing the name of the artist.
#define AMS_TRACK_ATTRIBUTE_ID_ARTIST                   0

// A string containing the name of the album.
#define AMS_TRACK_ATTRIBUTE_ID_ALBUM                    1

// A string containing the title of the track.
#define AMS_TRACK_ATTRIBUTE_ID_TITLE                    2

// A string containing the floating point value of the total duration of the track in seconds.
#define AMS_TRACK_ATTRIBUTE_ID_DURATION                 3

// AMS event description
typedef struct
{
    uint8_t     entity_id;
    uint8_t     attribute_id;
    uint8_t     flags;
    uint8_t     value[255];
} ams_event_t;

extern void     ams_client_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status);
extern void     ams_client_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status);
extern void     ams_client_bonded(void);
extern void     ams_client_encryption_changed(void);

#endif
