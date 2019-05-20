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

#pragma once

#include "wiced_bt_dev.h"
#include "wiced_gki.h"
#include "wiced_bt_cfg.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif


#if !defined(CYW20819A1) && !defined(CYW20721B2) && !defined(CYW20719B2)
#include "wiced_bt_app_hal_common.h"
#endif

/******************************************************************************
 *                                Constants
 ******************************************************************************/

enum
{
    LE_COC_STATE_INIT,
    LE_COC_STATE_ERROR_NO_PRIVATE_BUF,
    LE_COC_STATE_IDLE,
    LE_COC_STATE_IDLE_CONNECTED,
    LE_COC_STATE_SENDING,
    LE_COC_STATE_WAIT_FOR_BUFS
};

/******************************************************
 *                    Structures
 ******************************************************/
/* Application control block */
typedef struct
{
    uint16_t local_cid;
    wiced_bt_device_address_t peer_bda;
    uint8_t congested;
    uint16_t peer_mtu;
} le_coc_cb_t;

/*****************************************************************************
 * Globals
 *****************************************************************************/
/* Stack and buffer pool configuration tables */
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];

#define LE_COC_LARGE_POOL_BUFFER_COUNT            5
#define LE_COC_VS_ID                      WICED_NVRAM_VSID_START
#define LE_COC_LOCAL_KEYS_VS_ID          ( WICED_NVRAM_VSID_START + 1 )
#define LE_COC_PAIRED_KEYS_VS_ID          ( WICED_NVRAM_VSID_START + 2 )

/*****************************************************************************
 * Function Prototypes
 *****************************************************************************/
