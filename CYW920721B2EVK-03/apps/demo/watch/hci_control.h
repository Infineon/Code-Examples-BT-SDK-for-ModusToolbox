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
 * This file provides the private interface definitions for hci_control app
 *
 */
#ifndef HCI_CONTROL_H
#define HCI_CONTROL_H

#include "wiced_app.h"

/*****************************************************************************
**  Constants that define the capabilities and configuration
*****************************************************************************/
#define BSG_RFCOMM_SCN                  1
#define BSG_DEVICE_MTU                  800
#define WICED_BUFF_MAX_SIZE             264
#define BSG_TRANS_MAX_BUFFERS           10

#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_SPI)
#define TRANS_SPI_BUFFER_SIZE           1024
#endif
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_UART)
#define TRANS_UART_BUFFER_SIZE          1024
#endif

#include "wiced_bt_dev.h"
#include "wiced_bt_rfcomm.h"
#include "wiced_bt_avrc.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_dev.h"
#include "hci_control_api.h"
#include "wiced_bt_cfg.h"

////// TEMP for compiling
#ifndef BTM_SCO_PKT_TYPES_MASK_HV1
#define BTM_INVALID_SCO_INDEX           0xFFFF
#define BTM_SCO_LINK_ALL_PKT_MASK       0x003F
#define BTM_SCO_LINK_ONLY_MASK          0x0007
#define BTM_SCO_PKT_TYPES_MASK_HV3      0x0004
#define BTM_SCO_PKT_TYPES_MASK_EV3      0x0008
#define BTM_SCO_PKT_TYPES_MASK_EV4      0x0010
#define BTM_SCO_PKT_TYPES_MASK_EV5      0x0020
#define BTM_SCO_PKT_TYPES_MASK_NO_2_EV3 0x0040
#define BTM_SCO_PKT_TYPES_MASK_NO_3_EV3 0x0080
#define BTM_SCO_PKT_TYPES_MASK_NO_2_EV5 0x0100
#define BTM_SCO_PKT_TYPES_MASK_NO_3_EV5 0x0200
#define BTM_ESCO_RETRANS_POWER          1
#define BTM_ESCO_RETRANS_QUALITY        2
#endif
#define BTA_HS_DEBUG WICED_FALSE


#define HCI_CONTROL_HF_NUM_SCB          2           /* Number of SCBs - max simultaneous connections to AGs */
#define BTA_HS_MTU                      255         /* RFCOMM MTU SIZE */

#define BTA_HS_SCO_PKT_TYPES    ( BTM_SCO_PKT_TYPES_MASK_HV3 | \
                                 BTM_SCO_PKT_TYPES_MASK_EV3 | \
                                 BTM_SCO_PKT_TYPES_MASK_EV4 | \
                                 BTM_SCO_PKT_TYPES_MASK_EV5 | \
                                 BTM_SCO_PKT_TYPES_MASK_NO_3_EV3 | \
                                 BTM_SCO_PKT_TYPES_MASK_NO_3_EV5 )

#if ( HCI_CONTROL_HF_VERSION >= HFP_VERSION_1_7 && HCI_CONTROL_HF_IND_SUPPORTED == WICED_TRUE )
/* Max number of peer HF indicator */
#define BTA_HS_MAX_NUM_PEER_HF_IND     20
#endif

/* feature mask that matches spec */
#if ( BTM_WBS_INCLUDED == WICED_TRUE )
#define HCI_CONTROL_HF_FEAT_SPEC        ( HCI_CONTROL_HF_FEAT_ECNR | HCI_CONTROL_HF_FEAT_3WAY | \
                                  HCI_CONTROL_HF_FEAT_CLIP | HCI_CONTROL_HF_FEAT_VREC | \
                                  HCI_CONTROL_HF_FEAT_RVOL | HCI_CONTROL_HF_FEAT_ECS  | \
                                  HCI_CONTROL_HF_FEAT_ECC  | HCI_CONTROL_HF_FEAT_CODEC| \
                                  HCI_CONTROL_HF_FEAT_HF_IND | HCI_CONTROL_HF_FEAT_ESCO )

#else
#define HCI_CONTROL_HF_FEAT_SPEC        ( HCI_CONTROL_HF_FEAT_ECNR | HCI_CONTROL_HF_FEAT_3WAY | \
                                  HCI_CONTROL_HF_FEAT_CLIP | HCI_CONTROL_HF_FEAT_VREC | \
                                  HCI_CONTROL_HF_FEAT_RVOL | HCI_CONTROL_HF_FEAT_ECS  | \
                                  HCI_CONTROL_HF_FEAT_ECC  | HCI_CONTROL_HF_FEAT_HF_IND | \
                                  HCI_CONTROL_HF_FEAT_ESCO )
#endif


#define HCI_CONTROL_HF_AT_TIMEOUT_VALUE     5          /* 5 seconds */

/* AT command argument capabilities */
#define HCI_CONTROL_HF_AT_NONE              0x01        /* no argument */
#define HCI_CONTROL_HF_AT_SET               0x02        /* set value */
#define HCI_CONTROL_HF_AT_READ              0x04        /* read value */
#define HCI_CONTROL_HF_AT_TEST              0x08        /* test value range */
#define HCI_CONTROL_HF_AT_FREE              0x10        /* freeform argument */

/* AT argument format */
#define HCI_CONTROL_HF_AT_FMT_NONE          0           /* no arguments */
#define HCI_CONTROL_HF_AT_FMT_STR           1           /* string */
#define HCI_CONTROL_HF_AT_FMT_INT           2           /* integer */


#define AVRCP_TARGET_ROLE               1
#define AVRCP_CONTROLLER_ROLE           2

/* NVRAM Ids */
#define HCI_CONTROL_FIRST_VALID_NVRAM_ID        0x10
#define HCI_CONTROL_INVALID_NVRAM_ID            0x00

#if (defined(SLEEP_SUPPORTED) && (SLEEP_SUPPORTED == WICED_TRUE) && !defined(CYW20706A2))
typedef enum
{
    HCI_CONTROL_STATE_NOT_IDLE,
    HCI_CONTROL_STATE_IDLE,
}hci_control_mode_t;
#endif
/*****************************************************************************
**  Data types
*****************************************************************************/

/* The main application control block */
typedef struct
{
    uint8_t pairing_allowed;
#if (defined(SLEEP_SUPPORTED) && (SLEEP_SUPPORTED == WICED_TRUE) && !defined(CYW20706A2))
    uint8_t application_state;        //application state, see hci_control_mode_t
#endif
} hci_control_cb_t;

typedef struct
{
    wiced_bool_t test_executing;
    uint16_t     opcode;
} hci_control_test_command_t;

/*****************************************************************************
**  Global data
*****************************************************************************/

/* control block declaration */
#if BTA_DYNAMIC_MEMORY == WICED_FALSE
extern hci_control_cb_t hci_control_cb;
#else
extern hci_control_cb_t *hci_control_cb_ptr;
#define hci_control_cb( *hci_control_cb_ptr )
#endif

extern wiced_bool_t avrcp_profile_role;

/*****************************************************************************
**  Function prototypes
*****************************************************************************/
/* main functions */
extern void     hci_control_init( void );
extern void     hci_control_send_command_status_evt( uint16_t code, uint8_t status );

/* LE control interface */
extern void     hci_control_le_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len );
extern void     hci_control_le_handle_scan_cmd( wiced_bool_t enable, wiced_bool_t filter_duplicates );
extern void     hci_control_le_handle_advertise_cmd( wiced_bool_t enable );
extern void     hci_control_le_handle_connect_cmd( uint8_t addr_type, BD_ADDR addr );
extern void     hci_control_le_handle_cancel_connect_cmd( uint8_t addr_type, BD_ADDR addr );
extern void     hci_control_le_handle_disconnect_cmd( uint16_t con_handle );
extern void     hci_control_le_handle_service_discovery( uint16_t con_handle, uint16_t s_handle, uint16_t e_handle );
extern void     hci_control_le_handle_characteristic_discovery( uint16_t con_handle, uint16_t s_handle, uint16_t e_handle );
extern void     hci_control_le_handle_descriptor_discovery( uint16_t con_handle, uint16_t s_handle, uint16_t e_handle );
extern void     hci_control_le_handle_read_req( uint16_t con_handle, uint16_t handle );
extern void     hci_control_le_handle_read_rsp( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len );
extern void     hci_control_le_handle_write_req( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len );
extern wiced_bool_t  hci_control_le_handle_write_cmd( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len );
extern void     hci_control_le_handle_indicate( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len );
extern wiced_bool_t  hci_control_le_handle_notify( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len );

extern int      hci_control_write_nvram( int nvram_id, int data_len, void *p_data, wiced_bool_t from_host );
extern int      hci_control_read_nvram( int nvram_id, void *p_data, int data_len );
extern int      hci_control_find_nvram_id( uint8_t *p_data, int len);
extern void     hci_control_delete_nvram( int nvram_id, wiced_bool_t from_host );
extern int      hci_control_alloc_nvram_id( void );

extern void     hci_control_switch_avrcp_role(uint8_t new_role);

/* a2dp function prototypes */
extern wiced_result_t a2dp_app_hci_control_connect( uint8_t* p_data, uint32_t len );
extern wiced_result_t a2dp_app_hci_control_disconnect( uint8_t* p_data, uint32_t len );
extern wiced_result_t a2dp_app_hci_control_start( uint8_t* p_data, uint32_t len );
extern wiced_result_t a2dp_app_hci_control_stop( uint8_t* p_data, uint32_t len );

/* Events provided by hci_control_audio.c */
extern wiced_result_t hci_control_audio_send_connect_complete( wiced_bt_device_address_t bd_addr, uint8_t status, uint16_t handle );
extern wiced_result_t hci_control_audio_send_disconnect_complete( uint16_t handle, uint8_t status, uint8_t reason );
extern wiced_result_t hci_control_audio_send_started_stopped(uint16_t handle, wiced_bool_t started);

/* avrcp function prototypes */
extern wiced_result_t hci_control_avrc_send_connect_complete( wiced_bt_device_address_t bd_addr, uint8_t status, uint16_t handle );
extern wiced_result_t hci_control_avrc_send_disconnect_complete( uint16_t handle );
extern wiced_result_t hci_control_avrc_send_play_status_change( uint16_t handle, uint8_t play_status );
extern wiced_result_t hci_control_send_avrc_event( int type, uint8_t *p_data, uint16_t data_size );
extern wiced_result_t app_avrc_hci_control_volume(uint8_t* p_data, uint32_t len);

/* Events provided by hci_control_rc_target.c */
extern wiced_result_t hci_control_send_avrc_target_event( int type, uint16_t handle );

/* Events provided by ancs_client */
extern void hci_control_ancs_handle_command(uint16_t opcode, uint8_t *p_data, uint16_t payload_len);

/* Events provided by ams_client */
extern wiced_bool_t hci_control_is_ams_connection_up ( void );
extern void         hci_control_ams_handle_command(uint16_t opcode, uint8_t *p_data, uint16_t payload_len);

extern void wiced_bt_trace_array( const char *string, const uint8_t* array, const uint16_t len );
extern void hci_control_le_enable( void );
extern void le_slave_encryption_status_changed(wiced_bt_dev_encryption_status_t *p_status);
extern void hci_control_le_scan_state_changed( wiced_bt_ble_scan_type_t state );
extern void hci_control_le_advert_state_changed( wiced_bt_ble_advert_mode_t mode );
extern uint8_t hci_control_audio_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t len );
extern uint8_t hci_control_avrc_handle_command( uint16_t cmd_opcode, uint8_t *p_data, uint16_t payload_len );
extern wiced_bool_t hci_control_rc_target_is_connected(void);
extern uint8_t hci_control_avrc_handle_ctrlr_command( uint16_t cmd_opcode, uint8_t *p_data, uint16_t payload_len );
extern void hci_control_misc_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len );
extern uint8_t wiced_bt_get_number_of_buffer_pools ( void );

extern hci_control_test_command_t test_command;

extern BD_ADDR  bd_addr_any;
extern BD_ADDR  bd_addr_null;

#endif /* BTA_HS_INT_H */
