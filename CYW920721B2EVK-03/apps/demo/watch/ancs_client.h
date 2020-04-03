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
* BLE Client for Apple Notification Center Service (ANCS)
*
* Defines external definitions and function prototype for the ANCS client.
*/
#ifndef ANCS_CLIENT__H
#define ANCS_CLIENT__H

#include "wiced_app.h"

// 7905F431-B5CE-4E99-A40F-4B1E122D00D0
extern const char ANCS_SERVICE[];//             = {0xD0, 0x00, 0x2D, 0x12, 0x1E, 0x4B, 0x0F, 0xA4, 0x99, 0x4E, 0xCE, 0xB5, 0x31, 0xF4, 0x05, 0x79};

// Notification Source: UUID 9FBF120D-6301-42D9-8C58-25E699A21DBD (notifiable)
extern const char ANCS_NOTIFICATION_SOURCE[];// = {0xBD, 0x1D, 0xA2, 0x99, 0xE6, 0x25, 0x58, 0x8C, 0xD9, 0x42, 0x01, 0x63, 0x0D, 0x12, 0xBF, 0x9F};

// Control Point: UUID 69D1D8F3-45E1-49A8-9821-9BBDFDAAD9D9 (writeable with response)
extern const char ANCS_CONTROL_POINT[];//       = {0xD9, 0xD9, 0xAA, 0xFD, 0xBD, 0x9B, 0x21, 0x98, 0xA8, 0x49, 0xE1, 0x45, 0xF3, 0xD8, 0xD1, 0x69};

// Data Source: UUID 22EAC6E9-24D6-4BB5-BE44-B36ACE7C7BFB (notifiable)
extern const char ANCS_DATA_SOURCE[];//         = {0xFB, 0x7B, 0x7C, 0xCE, 0x6A, 0xB3, 0x44, 0xBE, 0xB5, 0x4B, 0xD6, 0x24, 0xE9, 0xC6, 0xEA, 0x22};

// max notifications to queue
#define ANCS_MAX_QUEUED_NOTIFICATIONS                   20

#define ANCS_EVENT_ID_NOTIFICATION_ADDED                0
#define ANCS_EVENT_ID_NOTIFICATION_MODIFIED             1
#define ANCS_EVENT_ID_NOTIFICATION_REMOVED              2
#define ANCS_EVENT_ID_MAX                               3

// Definitions for attributes we are not interested in are commented out
//#define ANCS_NOTIFICATION_ATTR_ID_APP_ID                0
#define ANCS_NOTIFICATION_ATTR_ID_TITLE                 1
//#define ANCS_NOTIFICATION_ATTR_ID_SUBTITLE              2
#define ANCS_NOTIFICATION_ATTR_ID_MESSAGE               3
#define ANCS_NOTIFICATION_ATTR_ID_MESSAGE_SIZE          4
//#define ANCS_NOTIFICATION_ATTR_ID_DATE                  5
#define ANCS_NOTIFICATION_ATTR_ID_POSITIVE_ACTION_LABEL 6
#define ANCS_NOTIFICATION_ATTR_ID_NEGATIVE_ACTION_LABEL 7
#define ANCS_NOTIFICATION_ATTR_ID_MAX                   8

#define ANCS_EVENT_FLAG_SILENT                          (1 << 0)
#define ANCS_EVENT_FLAG_IMPORTANT                       (1 << 1)
#define ANCS_EVENT_FLAG_PREEXISTING                     (1 << 2)
#define ANCS_EVENT_FLAG_POSITIVE_ACTION                 (1 << 3)
#define ANCS_EVENT_FLAG_NEGATIVE_ACTION                 (1 << 4)

// following is the list of notification attributes that we are going
// to request.  Compile out attribute of no interest
extern uint8_t  ancs_client_notification_attribute[];
// Maximum length we are going to request.  The values are valid for
// title subtitle and message.  The number of elements should match number
// of elements in the ancs_client_notification_attribute above
extern uint8_t  ancs_client_notification_attribute_length[];

// ANCS event description
typedef struct
{
    void        *p_next;
    uint32_t    notification_uid;
    uint8_t     command;
    uint8_t     flags;
    uint8_t     category;
    uint8_t     title[20];
    uint8_t     message[255];
    uint8_t     positive_action_label[10];
    uint8_t     negative_action_label[10];
} ancs_event_t;

// ANCS queued event description. Notification is queued while
// we are busy retrieving data from the current notification.
typedef struct
{
    void        *p_next;
    uint32_t    notification_uid;
    uint8_t     command;
    uint8_t     flags;
    uint8_t     category;
} ancs_queued_event_t;

extern void     ancs_client_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status);
extern void     ancs_client_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status);
extern void     ancs_client_bonded(void);
extern void     ancs_client_encryption_changed(void);

extern uint8_t *ancs_client_event_pool;

#endif
