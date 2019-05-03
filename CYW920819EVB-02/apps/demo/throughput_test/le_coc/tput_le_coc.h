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
#ifdef LE_COC_SUPPORT

#ifndef __TPUT_LE_COC_H__
#define __TPUT_LE_COC_H__

#include "wiced_platform.h"
#include "wiced_bt_l2c.h"
#include "wiced_bt_trace.h"
#include "../GeneratedSource/cycfg_pins.h"

/*****************************************************************************
 *                          Variable Definitions
 ****************************************************************************/
#define STR(x)                          #x
#define COC_BYTES                       465     //LE COC Bytes length
#define MAX_LOCAL_CID                   0xFFFF  //Maximum local channel id value
#define MINIMUM_LE_COC_TX_BUFFER_LEN    1       //Minimum tx buffer length
#if !defined (CONGESTION_LED)
#define CONGESTION_LED                  WICED_GET_PIN_FOR_LED(1)
#endif

extern uint32_t                          numBytesTx_coc;
extern uint32_t                          failed_to_send_coc;
extern const wiced_platform_led_config_t platform_led[];
char                                     coc_data[COC_BYTES]         = {0};                  //LE COC bytes buffer
wiced_bool_t                             coc_data_congested          = FALSE;                //Flag to keep track of packet congestion status
uint16_t                                 mtu                         = 512;                  //Local MTU value
uint16_t                                 psm                         = 19;                   //Local PSM value

/*****************************************************************************
 *                              FUNCTIONS
 ****************************************************************************/
extern uint32_t                 wiced_bt_ble_get_available_tx_buffers(void);
extern const char*              getOpcodeStr(uint16_t opcode);
void                            le_coc_congestion_cback(void *context, UINT16 local_cid, BOOLEAN congested);
void                            le_coc_connect_cfm_cback(void *context, UINT16 local_cid, UINT16 result, UINT16 mtu_peer);
void                            le_coc_connect_ind_cback(void *context, BD_ADDR bd_addr, UINT16 local_cid, UINT16 psm, UINT8 id, UINT16 mtu_peer);
void                            le_coc_connect(wiced_bt_device_address_t bd_addr, wiced_bt_ble_address_type_t bd_addr_type);
void                            le_coc_disconnect_cfm_cback(void *context, UINT16 local_cid, UINT16 result);
void                            le_coc_disconnect_ind_cback(void *context, UINT16 local_cid, BOOLEAN ack);
void                            le_coc_init(void);
void                            le_coc_data_cback(void *context, UINT16 local_cid, UINT8 *p_buff, UINT16 buf_len);
void                            tput_send_l2cap_coc_pkt(void);

/******************************************************
 *                    Structures
 ******************************************************/
/* Declare other app info for all needed PSMs */
wiced_bt_l2cap_le_appl_information_t l2c_appl_info =
{
    le_coc_connect_ind_cback,
    le_coc_connect_cfm_cback,
    le_coc_disconnect_ind_cback,
    le_coc_disconnect_cfm_cback,
    le_coc_data_cback,
    le_coc_congestion_cback,
    NULL
};

typedef struct
{
    uint16_t local_cid;
    wiced_bt_device_address_t peer_bda;
    uint8_t congested;
    uint16_t peer_mtu;
} le_coc_cb_t;
le_coc_cb_t                     le_coc_cb;

#endif //__TPUT_LE_COC_H__

#endif // LE_COC_SUPPORT
