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
 * This file implements the entry point of the Wiced Application
 */
#include "sparcommon.h"
#include "wiced.h"
#include "wiced_bt_dev.h"
#include "wiced_app.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_trace.h"
#include "hci_control.h"
#if (WICED_APP_LE_INCLUDED == WICED_TRUE)
#include "hci_control_le.h"
#endif
#if (WICED_APP_LE_SLAVE_CLIENT_INCLUDED == WICED_TRUE)
#include "le_slave.h"
#endif
#if (WICED_APP_AUDIO_SRC_INCLUDED == WICED_TRUE)
#include "hci_control_audio.h"
#endif

#ifdef CYW43012C0
#include "wiced_memory.h"

static wiced_bt_buffer_pool_t* watch_app_pool_big = NULL;
static wiced_bt_buffer_pool_t* watch_app_pool_small = NULL;
#endif
/*
 *  Application Start, ie, entry point to the application.
 */
APPLICATION_START( )
{
    // Initialize HCI
    hci_control_init();

    WICED_BT_TRACE( "APP START\n" );

#if (WICED_APP_LE_INCLUDED == WICED_TRUE)
    hci_control_le_init();
#endif

#if (WICED_APP_LE_SLAVE_CLIENT_INCLUDED == WICED_TRUE)
    le_slave_app_init();
#endif
#ifndef DEBUG
    //Enable the bt coex functionality
//    wiced_bt_coex_enable();
#endif
}
