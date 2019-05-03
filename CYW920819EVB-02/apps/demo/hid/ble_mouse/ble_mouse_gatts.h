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

#ifndef _BLEMOUSE_GATTS_H
#define _BLEMOUSE_GATTS_H

/******************************************************************************
 *                         Type Definitions
 ******************************************************************************/
typedef enum
{
    HANDLE_BLEMOUSE_GATT_SERVICE = 0x1, // service handle
        HANDLE_BLEMOUSE_GATT_SERVICE_CHANGED,
        HANDLE_BLEMOUSE_GATT_SERVICE_CHANGED_VAL,

    HANDLE_BLEMOUSE_GAP_SERVICE = 0x14, // service handle
        HANDLE_BLEMOUSE_GAP_SERVICE_CHAR_DEV_NAME, // characteristic handl
        HANDLE_BLEMOUSE_GAP_SERVICE_CHAR_DEV_NAME_VAL, // char value handle

        HANDLE_BLEMOUSE_GAP_SERVICE_CHAR_DEV_APPEARANCE, // characteristic handl
        HANDLE_BLEMOUSE_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,// char value handle

        HANDLE_BLEMOUSE_GAP_SERVICE_CHAR_PERI_PREFER_CONNPARAM, // characteristic handl
        HANDLE_BLEMOUSE_GAP_SERVICE_CHAR_PERI_PREFER_CONNPARAM_VAL,// char value handle

    HANDLE_BLEMOUSE_DEV_INFO_SERVICE = 0x28,
        HANDLE_BLEMOUSE_DEV_INFO_SERVICE_CHAR_PNP_ID, // characteristic handle
        HANDLE_BLEMOUSE_DEV_INFO_SERVICE_CHAR_PNP_ID_VAL,// char value handle

        HANDLE_BLEMOUSE_DEV_INFO_SERVICE_CHAR_MFR_NAME, // characteristic handle
        HANDLE_BLEMOUSE_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,// char value handle

    HANDLE_BLEMOUSE_BATTERY_SERVICE = 0x30, // service handle
        HANDLE_BLEMOUSE_BATTERY_SERVICE_CHAR_LEVEL, // characteristic handl
        HANDLE_BLEMOUSE_BATTERY_SERVICE_CHAR_LEVEL_VAL, // char value handle
        HANDLE_BLEMOUSE_BATTERY_SERVICE_CHAR_CFG_DESCR, // charconfig desc handl
        HANDLE_BLEMOUSE_BATTERY_SERVICE_RPT_REF_DESCR, // char desc handl

    HANDLE_BLEMOUSE_SCAN_PARAM_SERVICE = 0x40, // service handle
        HANDLE_BLEMOUSE_SCAN_PARAM_SERVICE_CHAR_SCAN_INT_WINDOW, // characteristic handl
        HANDLE_BLEMOUSE_SCAN_PARAM_SERVICE_CHAR_SCAN_INT_WINDOW_VAL, // char value handle

    HANDLE_BLEMOUSE_LE_HID_SERVICE = 0x4F, // service handle
        HANDLE_BLEMOUSE_LE_HID_SERVICE_INC_BAS_SERVICE,    //include service

        HANDLE_BLEMOUSE_LE_HID_SERVICE_PROTO_MODE,         // characteristic handl
        HANDLE_BLEMOUSE_LE_HID_SERVICE_PROTO_MODE_VAL,     // char value handle

        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_INFO,         // characteristic handl
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_INFO_VAL,     // char value handle

        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_BT_MOUSE_INPUT,         // characteristic handl
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_BT_MOUSE_INPUT_VAL,     // char value handle
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_BT_MOUSE_INPUT_CHAR_CFG_DESCR, // charconfig desc handl

        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_MAP,         // characteristic handl
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_MAP_VAL,     // char value handle

        HANDLE_BLEMOUSE_LE_HID_SERVICE_EXT_RPT_REF_DESCR, // char desc handl

        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_STD_INPUT,         // characteristic handl
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_STD_INPUT_VAL,     // char value handle
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_STD_INPUT_CHAR_CFG_DESCR, //charconfig desc handl
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_STD_INPUT_RPT_REF_DESCR, // char desc handl

        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_HID_CTRL_POINT,         // characteristic handl
        HANDLE_BLEMOUSE_LE_HID_SERVICE_HID_RPT_HID_CTRL_POINT_VAL,     // char value handle
}blemouse_db_tags;

typedef struct{
    uint16_t  handle;
    uint16_t  attr_len;
    const void *        p_attr;
}attribute_t;


extern const char  dev_local_name[];


#endif //_BLEMOUSE_GATTS_H
