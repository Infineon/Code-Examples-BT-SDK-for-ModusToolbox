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

#include "bt_types.h"
#include "wiced_bt_cfg.h"
#include "hci_control_api.h"
#include "wiced_bt_pbc_int.h"

#define WICED_BT_PBC_LIST_DIR_LEN_MAX           512
#define WICED_BT_PBC_MAX_REALM_LEN              30

/* No client connected identifier */
#define WICED_BT_NULL_CLIENT                    -1

#define WICED_BT_PBC_CLOSE_CLOSED               1
#define WICED_BT_PBC_CLOSE_CONN_LOSS            2


// SDP Record for PBAP Client
#define HDLR_PBAP_CLIENT_UNIT                   0x10001
#define PBAP_CLIENT_SCN                         0x01
#define PBAP_CLIENT_NAME                        "pbap client"

#define HDLR_HANDS_FREE_UNIT                    0x10002
#define HANDS_FREE_SCN                          0x02

#define TRANS_UART_BUFFER_SIZE                  1024

#define PBAP_CLIENT_NVRAM_ID                    0x46

#define WICED_HS_EIR_BUF_MAX_SIZE               264
#define KEY_INFO_POOL_BUFFER_SIZE               145 //Size of the buffer used for holding the peer device key info
#define KEY_INFO_POOL_BUFFER_COUNT              10  //Corresponds to the number of peer devices

/*
 * PBC (Phone Book Client) Errors
 */
#define WICED_BT_PBC_ERROR_XML_PARS      1350   /* XML folder parsing Failure  */
#define WICED_BT_PBC_ERROR_FIRST         1351   /* Offset for error conversion */
#define WICED_BT_PBC_ERROR_FAIL          (WICED_BT_PBC_FAIL          + WICED_BT_PBC_ERROR_FIRST) /* Generic Failure */
#define WICED_BT_PBC_ERROR_NO_PERMISSION (WICED_BT_PBC_NO_PERMISSION + WICED_BT_PBC_ERROR_FIRST) /* Permision denied */
#define WICED_BT_PBC_ERROR_NOT_FOUND     (WICED_BT_PBC_NOT_FOUND     + WICED_BT_PBC_ERROR_FIRST) /* File not found */
#define WICED_BT_PBC_ERROR_FULL          (WICED_BT_PBC_FULL          + WICED_BT_PBC_ERROR_FIRST) /* File too big or FS full */
#define WICED_BT_PBC_ERROR_BUSY          (WICED_BT_PBC_BUSY          + WICED_BT_PBC_ERROR_FIRST) /* Another operating ongoing */
#define WICED_BT_PBC_ERROR_ABORTED       (WICED_BT_PBC_ABORTED       + WICED_BT_PBC_ERROR_FIRST) /* Operation was aborted */



#ifndef WICED_BT_PBC_FORMAT
#define WICED_BT_PBC_FORMAT     (WICED_BT_PBC_FORMAT_CARD_21)
#endif

#ifndef WICED_BT_PBC_FILTER
#define WICED_BT_PBC_FILTER     (WICED_BT_PBC_FILTER_ALL) /* 0 */
#endif

#ifndef WICED_BT_PBC_ORDER
#define WICED_BT_PBC_ORDER      (WICED_BT_PBC_ORDER_INDEXED)
#endif

#ifndef WICED_BT_PBC_ATTR
#define WICED_BT_PBC_ATTR       (WICED_BT_PBC_ATTR_NAME)
#endif

#ifndef WICED_BT_PBC_SELECTOR
#define WICED_BT_PBC_SELECTOR   (WICED_BT_PBC_FILTER_ALL) /* 0 */
#endif

#ifndef WICED_BT_PBC_SELECTOR_OP
#define WICED_BT_PBC_SELECTOR_OP    (0) /* 0 OR, 1 AND */
#endif

#ifndef WICED_BT_PBC_MAX_LCOUNT
#define WICED_BT_PBC_MAX_LCOUNT     (50)
#endif

#ifndef WICED_BT_PBC_PASSWORD
#define WICED_BT_PBC_PASSWORD       "1234"
#endif

#ifndef WICED_BT_PBC_USERID
#define WICED_BT_PBC_USERID         "guest"
#endif

#ifndef WICED_BT_PBC_SECURITY
#define WICED_BT_PBC_SECURITY       (BTM_SEC_IN_AUTHENTICATE | BTM_SEC_OUT_AUTHENTICATE | BTM_SEC_ENCRYPT)
#endif

#define WICED_BT_PBC_MAX_AUTH_KEY_LENGTH    16

typedef struct
{
     char                           pbc_password[WICED_BT_PBC_MAX_AUTH_KEY_LENGTH + 1];
     char                           pbc_userid[WICED_BT_PBC_MAX_AUTH_KEY_LENGTH + 1];
     UINT8                          pbc_security;
     UINT32                         pbc_filter;
     UINT8                          pbc_format;
     UINT8                          pbc_order;
     UINT8                          pbc_attr;
     UINT16                         pbc_max_list_count;
     UINT16                         pbc_offset;
     wiced_bt_pbc_filter_mask_t     pbc_filter_mask;
     wiced_bt_pbc_sup_fea_mask_t    pbc_local_features;
     wiced_bt_pbc_filter_mask_t     pbc_selector;
     UINT8                          pbc_selector_op;

} wiced_bt_pbc_app_cfg_t;


typedef enum
{
    HCI_CONTROL_PBC_OPEN_EVT,       /* Connection to peer is open. */
    HCI_CONTROL_PBC_CLOSE_EVT,      /* Connection to peer closed. */
    HCI_CONTROL_PBC_DISABLE_EVT,    /* Connection Disable */
    HCI_CONTROL_PBC_ABORT_EVT,      /* Connection Abort */
    HCI_CONTROL_PBC_AUTH_EVT,       /* Request for Authentication key and user id */
    HCI_CONTROL_PBC_GET_EVT,        /* Event contains a response to Get request */
    HCI_CONTROL_PBC_SET_EVT         /* Event contains a response to Get request */
} tHCI_CONTROL_PBC_EVT;


extern const wiced_bt_cfg_settings_t pbap_client_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t pbap_client_cfg_buf_pools[];

extern uint32_t  hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length );
extern const uint8_t pbap_client_sdp_db[];

extern uint8_t pairing_allowed;


// /* data associated with PBC_OPEN_EVT */
// typedef struct
// {
    // BD_ADDR             bd_addr;
    // uint8_t             status;
// } hci_control_pbc_open_t;

// /* data associated with AT command response event */
// typedef struct
// {
    // uint16_t            num;
    // //char                str[WICED_BT_PBC_MAX_AT_CMD_LEN];
// } hci_control_pbc_value_t;

// /* data associated with PBC_CONNECTED_EVT */
// typedef struct
// {
    // uint16_t           peer_features;
// } hci_control_pbc_connect_t;

// /* union of data associated with HS callback */
// typedef union
// {
    // hci_control_pbc_open_t    open;
    // hci_control_pbc_connect_t conn;
    // hci_control_pbc_value_t   val;
// } hci_control_pbc_event_t;

#define WICED_BT_PBC_FEA_DOWNLOADING                    0x00000001      /* Downloading */
#define WICED_BT_PBC_FEA_BROWSING                       0x00000002      /* Browsing */
#define WICED_BT_PBC_FEA_DATABASE_ID                    0x00000004      /* Database identifier */
#define WICED_BT_PBC_FEA_FOLDER_VER_COUNTER             0x00000008      /* Folder version counter */
#define WICED_BT_PBC_FEA_VCARD_SELECTING                0x00000010      /* Vcard selecting */
#define WICED_BT_PBC_FEA_ENH_MISSED_CALLS               0x00000020      /* Enhanced missed calls */
#define WICED_BT_PBC_FEA_UCI_VCARD_FIELD                0x00000040      /* UCI Vcard field */
#define WICED_BT_PBC_FEA_UID_VCARD_FIELD                0x00000080      /* UID Vcard field */
#define WICED_BT_PBC_FEA_CONTACT_REF                    0x00000100      /* Contact Referencing */
#define WICED_BT_PBC_FEA_DEF_CONTACT_IMAGE_FORMAT       0x00000200      /* Default contact image format */

#define HCI_CONTROL_PBC_STATUS_BAD_PARAM                200 /* Server Bad parameter */
#define HCI_CONTROL_PBC_STATUS_BAD_REQ_SIZE             201 /* Server Bad Request Size */
#define HCI_CONTROL_PBC_STATUS_BAD_MSG_ID               202 /* Server Bad message identifier */
#define HCI_CONTROL_PBC_STATUS_ALREADY_ACTIVE           203 /* Server this profile is already active */
#define HCI_CONTROL_PBC_STATUS_NYI                      204 /* Server API not Yet Implemented */
#define HCI_CONTROL_PBC_STATUS_BAD_CLIENT               205 /* Server Bad client number */
#define HCI_CONTROL_PBC_STATUS_MEM_FULL                 206 /* Server No more memory */
#define HCI_CONTROL_PBC_STATUS_HW_ERROR                 207 /* Server HardWare error */
#define HCI_CONTROL_PBC_STATUS_BLUETOOTH_DISABLE        208 /* Bluetooth is disabled */
#define HCI_CONTROL_PBC_STATUS_INTERNAL                 209 /* Internal error */
#define HCI_CONTROL_PBC_STATUS_NOT_COMPILED             210 /* Server Functionality not compiled */
#define HCI_CONTROL_PBC_STATUS_WLAN_RESET               211 /* Common USB is not available. WLAN RESET */

typedef UINT32 hci_control_pbc_fea_mask_t;

//enum
//{
//    WICED_BT_PBC_FORMAT_CARD_21, /* vcard format 2.1 */
//    WICED_BT_PBC_FORMAT_CARD_30, /* vcard format 3.0 */
//    WICED_BT_PBC_FORMAT_MAX
//};
//typedef uint8 hci_control_pbc_format_t;

typedef struct
{
    uint16_t                    phone_book_size;
    wiced_bool_t                pbs_exist;          /* phone_book_size is present in the response */
    uint8_t                     new_missed_calls;
    wiced_bool_t                nmc_exist;          /* new_missed_calls is present in the response */
} hci_control_pbc_pb_param_t;

typedef struct
{
    uint8_t                     data[WICED_BT_PBC_LIST_DIR_LEN_MAX];
    uint16_t                    len;
    uint16_t                    num_entry; /* number of entries listed */
    wiced_bool_t                final;     /* if true, entry is last of the series */
    wiced_bool_t                is_xml;
    hci_control_pbc_pb_param_t  param;
} hci_control_pbc_list_t;

enum
{
    WICED_BT_PBC_GET_PARAM_STATUS = 0,          /* status only*/
    WICED_BT_PBC_GET_PARAM_LIST,                /* list message */
    WICED_BT_PBC_GET_PARAM_PROGRESS,            /* progress message*/
    WICED_BT_PBC_GET_PARAM_PHONEBOOK,           /* phonebook param*/
    WICED_BT_PBC_GET_PARAM_PHONEBOOK_DATA,           /* phonebook param*/
    WICED_BT_PBC_GET_PARAM_FILE_TRANSFER_STATUS /* file transfer status*/
};

typedef uint8_t hci_control_pbc_get_param_type_t;

typedef struct
{
    uint32_t file_size;   /* total size of file (bsa_fs_len_unknown if unknown) */
    uint16_t num_bytes;       /* number of bytes read or written since last progress event */
} hci_control_pbc_progress_t;

typedef struct
{
    uint16_t                    len;
    uint8_t*                    p_data;
} hci_control_pbc_pb_data_t;


typedef struct
{
    hci_control_pbc_get_param_type_t type;
    uint8_t                          status;

    union {
        /* get operations */
        hci_control_pbc_list_t       list;
        hci_control_pbc_progress_t   prog;
        hci_control_pbc_pb_param_t   pb;
        hci_control_pbc_pb_data_t    pb_data;
    } param;

} hci_control_pbc_get_t;


/* wiced_bt_pbc_open_evt callback event data */
typedef struct
{
    uint8_t                     status;
    BD_ADDR                     bd_addr;
    wiced_bt_service_id_t       service;
    wiced_bool_t                initiator;          /* connection initiator, local true, peer false */
    hci_control_pbc_fea_mask_t  peer_features;      /* peer supported features */
} hci_control_pbc_open_t;

/* wiced_bt_pbc_open_evt callback event data */
typedef struct
{
    uint8_t                     status;
} hci_control_pbc_close_t;

/* data for all pbc events */
typedef union
{
    hci_control_pbc_open_t       open;
    hci_control_pbc_get_t        get;
    hci_control_pbc_close_t      close;
} hci_control_pbc_event_t;

/* External Definitions */
extern void     wiced_bt_trace_array( const char *string, const uint8_t* array, const uint16_t len );
extern uint16_t wiced_app_cfg_sdp_record_get_size(void);
extern void *GKI_getbuf (uint16_t);
extern void GKI_freebuf (void *memPtr);
extern char *utl_strcpy( char *p_dst, char *p_src );
extern int hci_control_find_nvram_id(uint8_t *p_data, int len);
extern int hci_control_alloc_nvram_id( );
extern int hci_control_write_nvram( int nvram_id, int data_len, void *p_data, BOOLEAN from_host );
extern int hci_control_read_nvram( int nvram_id, void *p_data, int data_len );
extern void hci_control_send_device_started_evt( void );
extern void hci_control_send_pairing_completed_evt( uint8_t status , wiced_bt_device_address_t bdaddr );
extern void hci_control_delete_nvram( int nvram_id ,wiced_bool_t from_host);
extern void wiced_bt_pbc_close_hdlr();
extern BOOLEAN wiced_bt_pbc_get_phonebook(uint16_t opcode, UINT16 max_list_count, UINT16 list_start_offset, BOOLEAN is_reset_miss_calls);
extern void wiced_bt_pbc_abort_hdlr();
