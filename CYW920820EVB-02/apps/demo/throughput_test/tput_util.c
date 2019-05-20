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

/*
 * File Name: tput_util.c
 *
 * Description:
 *  This file defines the utility functions that are being used in throughput application.
 *  Define any utility functions in this file and declare the same in tput_util.h.
 */

#include "sparcommon.h"
#include "tput_util.h"

/***************************************
 *     Constants and Enumerations
 **************************************/
#define   STR(x)  #x

/*
 * BLE Advertisement status code mapping to string
 */
const char* advStr[] =
{
    STR(BTM_BLE_ADVERT_OFF),
    STR(BTM_BLE_ADVERT_DIRECTED_HIGH),
    STR(BTM_BLE_ADVERT_DIRECTED_LOW),
    STR(BTM_BLE_ADVERT_UNDIRECTED_HIGH),
    STR(BTM_BLE_ADVERT_UNDIRECTED_LOW),
    STR(BTM_BLE_ADVERT_NONCONN_HIGH),
    STR(BTM_BLE_ADVERT_NONCONN_LOW),
    STR(BTM_BLE_ADVERT_DISCOVERABLE_HIGH),
    STR(BTM_BLE_ADVERT_DISCOVERABLE_LOW)
};

/*
 * BLE Even Management code mapping to string
 */
const char* eventStr[] =
{
    STR(BTM_ENABLED_EVT),
    STR(BTM_DISABLED_EVT),
    STR(BTM_POWER_MANAGEMENT_STATUS_EVT),
    STR(BTM_PIN_REQUEST_EVT),
    STR(BTM_USER_CONFIRMATION_REQUEST_EVT),
    STR(BTM_PASSKEY_NOTIFICATION_EVT),
    STR(BTM_PASSKEY_REQUEST_EVT),
    STR(BTM_KEYPRESS_NOTIFICATION_EVT),
    STR(BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT),
    STR(BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT),
    STR(BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT),
    STR(BTM_PAIRING_COMPLETE_EVT),
    STR(BTM_ENCRYPTION_STATUS_EVT),
    STR(BTM_SECURITY_REQUEST_EVT),
    STR(BTM_SECURITY_FAILED_EVT),
    STR(BTM_SECURITY_ABORTED_EVT),
    STR(BTM_READ_LOCAL_OOB_DATA_COMPLETE_EVT),
    STR(BTM_REMOTE_OOB_DATA_REQUEST_EVT),
    STR(BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT),
    STR(BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT),
    STR(BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT),
    STR(BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT),
    STR(BTM_BLE_SCAN_STATE_CHANGED_EVT),
    STR(BTM_BLE_ADVERT_STATE_CHANGED_EVT),
    STR(BTM_SMP_REMOTE_OOB_DATA_REQUEST_EVT),
    STR(BTM_SMP_SC_REMOTE_OOB_DATA_REQUEST_EVT),
    STR(BTM_SMP_SC_LOCAL_OOB_DATA_NOTIFICATION_EVT),
    STR(BTM_SCO_CONNECTED_EVT),
    STR(BTM_SCO_DISCONNECTED_EVT),
    STR(BTM_SCO_CONNECTION_REQUEST_EVT),
    STR(BTM_SCO_CONNECTION_CHANGE_EVT),
    STR(BTM_BLE_CONNECTION_PARAM_UPDATE),
    STR(BTM_BLE_PHY_UPDATE_EVT),
};

/*************************************************************************************
* Function Name: const char* getStackEventStr(wiced_bt_management_evt_t event)
**************************************************************************************
* Summary: Parse BLE management event code to string
*
* Parameters:
*   wiced_bt_management_evt_t event: BLE management event code
*
* Return:
*   const uint8_t*: Pointer to BLE management string mapped event code
*
*************************************************************************************/
const char* getStackEventStr(wiced_bt_management_evt_t event)
{
    if (event >= sizeof(eventStr) / sizeof(char*))
    {
        return "** UNKNOWN **";
    }

    return eventStr[event];
}

/*************************************************************************************
* Function Name: const char* getAdvStatusStr(wiced_bt_ble_advert_mode_t event)
**************************************************************************************
* Summary: Parse Advertisement status code to string
*
* Parameters:
*   wiced_bt_ble_advert_mode_t event: BLE advertisement status code
*
* Return:
*   const uint8_t*: Pointer to BLE advertisement status string mapped event code
*
*************************************************************************************/
const char* getAdvStatusStr(wiced_bt_ble_advert_mode_t event)
{
    if (event >= sizeof(advStr) / sizeof(char*))
    {
        return "** UNKNOWN **";
    }

    return advStr[event];
}

/*************************************************************************************
* Function Name: gatt_db_lookup_table_t* tput_get_attribute( uint16_t handle )
**************************************************************************************
* Summary: Find attribute description by handle
*
* Parameters:
*   uint16_t handle: Attribute handle
*
* Return:
*   gatt_db_lookup_table_t*: Pointer to BLE GATT attribute handle
*
*************************************************************************************/
gatt_db_lookup_table_t* tput_get_attribute(uint16_t handle)
{
    int i;
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == handle)
        {
            return (&app_gatt_db_ext_attr_tbl[i]);
        }
    }
#ifdef VERBOSE_THROUGHPUT_OUTPUT
    WICED_BT_TRACE("\rTPUT[%s]: Attr not found:%x\n", __func__, handle);
#endif
    return NULL;
}

