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

/*****************************************************************************
**  Constants that define the capabilities and configuration
*****************************************************************************/
#undef BTM_WBS_INCLUDED
#define BTM_WBS_INCLUDED                TRUE
#define HCI_CONTROL_AG_VERSION          HFP_VERSION_1_6

#define HFP_RFCOMM_SCN                  1
#define SPP_RFCOMM_SCN                  2
#define HFP_DEVICE_MTU                  255
#define SPP_DEVICE_MTU                  800
#define WICED_BUFF_MAX_SIZE             360
#define SPP_TRANS_MAX_BUFFERS           10
#define TRANS_UART_BUFFER_SIZE          1024

#include "wiced_bt_dev.h"
#include "wiced_bt_rfcomm.h"
#include "wiced_bt_trace.h"

#include "wiced_bt_sdp.h"
#include "wiced_timer.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_sco.h"
#include "hci_control_api.h"
#include "hci_control_ag.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_audio.h"

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


#define HCI_CONTROL_AG_NUM_SCB          2           /* Max simultaneous connections to HFs */

#define BTA_AG_SCO_PKT_TYPES    ( BTM_SCO_PKT_TYPES_MASK_HV3 | \
                                 BTM_SCO_PKT_TYPES_MASK_EV3 | \
                                 BTM_SCO_PKT_TYPES_MASK_EV4 | \
                                 BTM_SCO_PKT_TYPES_MASK_EV5 | \
                                 BTM_SCO_PKT_TYPES_MASK_NO_3_EV3 | \
                                 BTM_SCO_PKT_TYPES_MASK_NO_3_EV5 )


/*****************************************************************************
**  Data types
*****************************************************************************/

/* type for each service control block */
/* Handsfree device control block */
typedef struct
{
#define     HCI_CONTROL_AG_STATE_IDLE       0
#define     HCI_CONTROL_AG_STATE_OPENING    1
#define     HCI_CONTROL_AG_STATE_OPEN       2
#define     HCI_CONTROL_AG_STATE_CLOSING    3

    uint8_t             state;                  /* state machine state */
    uint16_t            app_handle;             /* Handle used to identify with the app */

    uint8_t             b_is_initiator;         /* initiator of the connection ( true ) or acceptor ( false ) */
    BOOLEAN             b_slc_is_up;            /* set to TRUE when service level connection up */

    uint16_t            rfc_serv_handle;        /* RFCOMM server handle */
    uint16_t            rfc_conn_handle;        /* RFCOMM handle of connected service */
    uint8_t             hf_scn;                 /* HF's scn */
    BD_ADDR             hf_addr;                /* HF's bd address */

    char                res_buf[HCI_CONTROL_AG_AT_MAX_LEN + 1];     /* temp parsing buffer */
    int                 res_len;                                    /* length of data in temp buffer */

    wiced_bt_sdp_discovery_db_t *p_sdp_discovery_db;                /* pointer to discovery database */

    uint32_t            hf_features;            /* HF device features */
    uint16_t            hf_version;             /* HF device profile version */

#if (BTM_WBS_INCLUDED == TRUE )
    wiced_timer_t       cn_timer;       /* codec negotiation timer */

    BOOLEAN             peer_supports_msbc;     /* TRUE if peer supports mSBC */
    BOOLEAN             msbc_selected;          /* TRUE if we have selected mSBC */
#endif

    uint16_t            sco_idx;                /* SCO handle */
    BOOLEAN             b_sco_opened;           /* set to TRUE when SCO connection is open */

    BOOLEAN             retry_with_sco_only;    /* ind to try with SCO only if eSCO fails */

    BOOLEAN             clip_enabled;   /* set to TRUE if HF enables CLIP reporting */
    BOOLEAN             cmer_enabled;   /* set to TRUE if HF enables CMER reporting */
    BOOLEAN             cmee_enabled;   /* set to TRUE if HF enables CME ERROR reporting */

} hci_control_ag_session_cb_t;

/* SPP control block */
typedef struct
{
#define     HCI_CONTROL_SPP_STATE_IDLE       0
#define     HCI_CONTROL_SPP_STATE_OPENING    1
#define     HCI_CONTROL_SPP_STATE_OPEN       2
#define     HCI_CONTROL_SPP_STATE_CLOSING    3

    uint8_t             state;                          /* state machine state */

    uint8_t             b_is_initiator;                 /* initiator of the connection ( true ) or acceptor ( false ) */

    uint16_t            rfc_serv_handle;                /* RFCOMM server handle */
    uint16_t            rfc_conn_handle;                /* RFCOMM handle of connected service */
    uint8_t             server_scn;                     /* server's scn */
    BD_ADDR             server_addr;                    /* server's bd address */

    uint16_t            pending_bytes;                                   /* number of bytes waiting for transmission */
    uint8_t             pending_buffer[HCI_CONTROL_SPP_MAX_TX_BUFFER];   /* pending buffer */

    wiced_bt_sdp_discovery_db_t *p_sdp_discovery_db;    /* pointer to discovery database */

} hci_control_spp_session_cb_t;

/* The main application control block */
typedef struct
{
    hci_control_ag_session_cb_t  ag_scb[HCI_CONTROL_AG_NUM_SCB];       /* service control blocks */
    hci_control_spp_session_cb_t spp_scb;
    uint8_t pairing_allowed;
} hci_control_cb_t;



/*****************************************************************************
**  Global data
*****************************************************************************/

/* control block declaration */
#if BTA_DYNAMIC_MEMORY == FALSE
extern hci_control_cb_t hci_control_cb;
#else
extern hci_control_cb_t *hci_control_cb_ptr;
#define hci_control_cb( *hci_control_cb_ptr )
#endif

/*****************************************************************************
**  Function prototypes
*****************************************************************************/

/* main functions */
extern void     hci_control_ag_handle_command(uint16_t opcode, uint8_t* p_data, uint32_t length);
extern void     hci_control_send_command_status_evt( uint16_t code, uint8_t status );

extern hci_control_ag_session_cb_t *hci_control_ag_find_scb_by_sco_index( uint16_t sco_idx );
extern void     hci_control_ag_service_level_up(hci_control_ag_session_cb_t *p_scb);


/* SDP functions */
extern void     hci_control_ag_sdp_init(void);
extern void     hci_control_ag_sdp_start_discovery( hci_control_ag_session_cb_t *p_scb );

/* RFCOMM functions */
extern void     hci_control_ag_rfcomm_start_server( hci_control_ag_session_cb_t *p_scb );
extern void     hci_control_ag_rfcomm_do_close( hci_control_ag_session_cb_t *p_scb );


/* SCO functions */
extern void     hci_control_ag_sco_create( hci_control_ag_session_cb_t *p_scb, BOOLEAN is_orig );
extern void     hci_control_ag_sco_close( hci_control_ag_session_cb_t *p_scb );
extern void     hci_control_cn_timeout ( uint32_t scb );

/* AT functions */
extern void hci_control_ag_send_BCS_to_hf (hci_control_ag_session_cb_t *p_scb);
extern void hci_control_ag_send_BVRA_to_hf (hci_control_ag_session_cb_t *p_scb, BOOLEAN b_is_active);
extern void hci_control_ag_parse_AT_command (hci_control_ag_session_cb_t *p_scb);


/* UART interface */
extern void     hci_control_send_ag_event( uint16_t evt, uint16_t handle, hci_control_ag_event_t *p_data );

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
extern BOOLEAN  hci_control_le_handle_write_cmd( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len );
extern void     hci_control_le_handle_indicate( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len );
extern BOOLEAN  hci_control_le_handle_notify( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len );

/* SPP RFCOMM functions */
extern void     hci_control_spp_handle_command( uint16_t cmd_opcode, uint8_t* p, uint32_t data_len );
extern void     hci_control_spp_rfcomm_start_server( hci_control_spp_session_cb_t *p_scb );
extern void     hci_control_spp_rfcomm_do_close( hci_control_spp_session_cb_t *p_scb );
extern void     hci_control_spp_connect( BD_ADDR bd_addr );
extern void     hci_control_spp_disconnect( uint16_t handle );
extern void     hci_control_spp_send_data( uint16_t handle, uint8_t *p_data, uint32_t length );

/* String Utility functions */
extern char    *utl_strcpy( char *p_dst, char *p_src );
extern int      utl_strlen( char *p_str );
extern int16_t  utl_str2int( char *p_s );
extern int      utl_strucmp( char *p_s, char *p_t );
extern uint8_t  utl_itoa( uint16_t i, char *p_s );
extern void     utl_bdcpy( BD_ADDR a, BD_ADDR b );
extern int      hci_control_write_nvram( int nvram_id, int data_len, void *p_data, BOOLEAN from_host );
extern int      hci_control_read_nvram( int nvram_id, void *p_data, int data_len );
extern int      hci_control_find_nvram_id( uint8_t *p_data, int len);
extern void     hci_control_delete_nvram( int nvram_id , wiced_bool_t from_host);
extern int      hci_control_alloc_nvram_id( );

/* Configuration */
extern BD_ADDR             bd_addr_any;
extern BD_ADDR             bd_addr_null;

extern const wiced_bt_cfg_settings_t hci_ag_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t hci_ag_cfg_buf_pools[];
extern const wiced_bt_audio_config_buffer_t hci_ag_audio_buf_config;
#endif /* BTA_HS_INT_H */
