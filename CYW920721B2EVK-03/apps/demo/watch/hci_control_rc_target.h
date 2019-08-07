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
 * AVRC support for HCI AV Source application
 */

#ifndef __HCI_CONTROL_RC_TARGET_H_
#define __HCI_CONTROL_RC_TARGET_H_

#include "wiced_bt_avrc_tg.h"


/*
 * AVRC init
 */
void hci_control_rc_target_init( void );

/*
 * AVRC connection processing. Check if AVRC Target is connected.
 */
wiced_bool_t hci_control_rc_target_is_connected(void);
/*
 * AVRC connection processing.  Pass connected device address and handle to the MCU.
 */
void app_avrc_device_connected(wiced_bt_device_address_t bd_addr, uint16_t handle);

#ifdef APP_AVRC_SETTING_CHANGE_SUPPORTED
/*
 * Called when peer changes repeat settings value, update the MCU app
 */
void app_avrc_set_repeat_settings_event(uint8_t val);

/*
 * Called when peer changes Shuffle settings value, update the MCU app
 */
void app_avrc_set_shuffle_settings_event(uint8_t val);
#endif

#ifdef APP_AVRC_PLAY_STATUS_SUPPORTED
/*
 * Called when MCU app provides player staus
 */
void app_avrc_player_status_cmd(uint8_t *p_data, uint16_t payload_len);
#endif

/*
 *  Called when MCU changes volume, send Absolute volume request to peer
 */
wiced_result_t app_avrc_hci_control_volume( uint8_t* p_data, uint32_t len );

/*
 *  Called to handle avrc commands from MCU app
 */
uint8_t hci_control_avrc_handle_command( uint16_t opcode, uint8_t *data, uint16_t payload_len );

/*
 * AVRC event handler
 */
void app_avrc_event_cback(uint8_t event_id,  wiced_bt_rc_event_t *p_event);

#endif /* __HCI_CONTROL_RC_TARGET_H_ */
