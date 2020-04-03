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
 * This file contains functions for processing AT commands and responses
 *
 */

#include "hci_control.h"
#include "hci_control_api.h"
#include "string.h"

/*****************************************************************************
**  Constants
*****************************************************************************/

#define BTA_AG_ERR_OP_NOT_ALLOWED   3       /* Operation not allowed */
#define BTA_AG_ERR_OP_NOT_SUPPORTED 4       /* Operation not supported */

#define BTA_AG_CMD_MAX_VAL      32767       /* Maximum value is signed 16-bit value */
#define BTA_AG_CIND_INFO        "(\"call\",(0,1)),(\"callsetup\",(0-3)),(\"service\",(0,1))"
#define BTA_AG_CIND_VALUES       "0,0,1"


/* enumeration of HFP AT commands matches HFP command interpreter table */
enum
{
    BTA_AG_HF_CMD_A,
    BTA_AG_HF_CMD_D,
    BTA_AG_HF_CMD_VGS,
    BTA_AG_HF_CMD_VGM,
    BTA_AG_HF_CMD_CCWA,
    BTA_AG_HF_CMD_CHLD,
    BTA_AG_HF_CMD_CHUP,
    BTA_AG_HF_CMD_CIND,
    BTA_AG_HF_CMD_CLIP,
    BTA_AG_HF_CMD_CMER,
    BTA_AG_HF_CMD_VTS,
    BTA_AG_HF_CMD_BINP,
    BTA_AG_HF_CMD_BLDN,
    BTA_AG_HF_CMD_BVRA,
    BTA_AG_HF_CMD_BRSF,
    BTA_AG_HF_CMD_NREC,
    BTA_AG_HF_CMD_CNUM,
    BTA_AG_HF_CMD_BTRH,
    BTA_AG_HF_CMD_CLCC,
    BTA_AG_HF_CMD_COPS,
    BTA_AG_HF_CMD_CMEE,
    BTA_AG_HF_CMD_BIA,
    BTA_AG_HF_CMD_CBC,
    BTA_AG_HF_CMD_BCC,
    BTA_AG_HF_CMD_BCS,
    BTA_AG_HF_CMD_BIND,
    BTA_AG_HF_CMD_BIEV,
    BTA_AG_HF_CMD_BAC
};

/* AT command interpreter table for HFP */
static const struct
{
    char        *p_cmd;                     /* AT command string */
    UINT8       arg_type;                   /* allowable argument type syntax */
#define BTA_AG_AT_NONE          0x01        /* no argument */
#define BTA_AG_AT_SET           0x02        /* set value */
#define BTA_AG_AT_READ          0x04        /* read value */
#define BTA_AG_AT_TEST          0x08        /* test value range */
#define BTA_AG_AT_FREE          0x10        /* freeform argument */

    UINT8       fmt;                        /* whether arg is int or string */
#define BTA_AG_AT_STR           0           /* string */
#define BTA_AG_AT_INT           1           /* integer */

    UINT8       min;                        /* minimum value for int arg */
    INT16       max;                        /* maximum value for int arg */

} bta_ag_hfp_cmd[] =
{
    {"A",       BTA_AG_AT_NONE,                     BTA_AG_AT_STR,   0,   0},
    {"D",       (BTA_AG_AT_NONE | BTA_AG_AT_FREE),  BTA_AG_AT_STR,   0,   0},
    {"+VGS",    BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0,  15},
    {"+VGM",    BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0,  15},
    {"+CCWA",   BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0,   1},
    {"+CHLD",   (BTA_AG_AT_SET | BTA_AG_AT_TEST),   BTA_AG_AT_STR,   0,   4},
    {"+CHUP",   BTA_AG_AT_NONE,                     BTA_AG_AT_STR,   0,   0},
    {"+CIND",   (BTA_AG_AT_READ | BTA_AG_AT_TEST),  BTA_AG_AT_STR,   0,   0},
    {"+CLIP",   BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0,   1},
    {"+CMER",   BTA_AG_AT_SET,                      BTA_AG_AT_STR,   0,   0},
    {"+VTS",    BTA_AG_AT_SET,                      BTA_AG_AT_STR,   0,   0},
    {"+BINP",   BTA_AG_AT_SET,                      BTA_AG_AT_INT,   1,   1},
    {"+BLDN",   BTA_AG_AT_NONE,                     BTA_AG_AT_STR,   0,   0},
    {"+BVRA",   BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0,   1},
    {"+BRSF",   BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0,   BTA_AG_CMD_MAX_VAL},
    {"+NREC",   BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0,   0},
    {"+CNUM",   BTA_AG_AT_NONE,                     BTA_AG_AT_STR,   0,   0},
    {"+BTRH",   (BTA_AG_AT_READ | BTA_AG_AT_SET),   BTA_AG_AT_INT,   0,   2},
    {"+CLCC",   BTA_AG_AT_NONE,                     BTA_AG_AT_STR,   0,   0},
    {"+COPS",   (BTA_AG_AT_READ | BTA_AG_AT_SET),   BTA_AG_AT_STR,   0,   0},
    {"+CMEE",   BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0,   1},
    {"+BIA",    BTA_AG_AT_SET,                      BTA_AG_AT_STR,   0,   20},
    {"+CBC",    BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0,   100},
    {"+BCC",    BTA_AG_AT_NONE,                     BTA_AG_AT_STR,   0,   0},
    {"+BCS",    BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0,   BTA_AG_CMD_MAX_VAL},
    {"+BIND",   BTA_AG_AT_SET | BTA_AG_AT_READ | BTA_AG_AT_TEST , BTA_AG_AT_STR,   0,   0},
    {"+BIEV",   BTA_AG_AT_SET,                      BTA_AG_AT_STR,   0,   0},
    {"+BAC",    BTA_AG_AT_SET,                      BTA_AG_AT_STR,   0,   0},
    {"",        BTA_AG_AT_NONE,                     BTA_AG_AT_STR,   0,   0}
};

/* AT result code table element */
typedef struct
{
    const char  *p_res;         /* AT result string */
    UINT8       fmt;            /* whether argument is int or string */
} tBTA_AG_RESULT;

/* AT result code argument types */
enum
{
    BTA_AG_RES_FMT_NONE,       /* no argument */
    BTA_AG_RES_FMT_INT,        /* integer argument */
    BTA_AG_RES_FMT_STR         /* string argument */
};

/* enumeration of AT result codes, matches constant table */
enum
{
    BTA_AG_RES_OK,
    BTA_AG_RES_ERROR,
    BTA_AG_RES_CHLD,
    BTA_AG_RES_CIND,
    BTA_AG_RES_CIEV,
    BTA_AG_RES_BVRA,
    BTA_AG_RES_BRSF,
    BTA_AG_RES_BCS,
    BTA_AG_RES_COPS
};

/* AT result code constant table  (Indexed by result code) */
const tBTA_AG_RESULT bta_ag_result_tbl[] =
{
    {"OK",      BTA_AG_RES_FMT_NONE},
    {"ERROR",   BTA_AG_RES_FMT_NONE},
    {"+CHLD: ", BTA_AG_RES_FMT_STR},
    {"+CIND: ", BTA_AG_RES_FMT_STR},
    {"+CIEV: ", BTA_AG_RES_FMT_STR},
    {"+BVRA: ", BTA_AG_RES_FMT_INT},
    {"+BRSF: ", BTA_AG_RES_FMT_INT},
    {"+BCS: ",  BTA_AG_RES_FMT_INT},
    {"+COPS: ", BTA_AG_RES_FMT_STR},
    {"",        BTA_AG_RES_FMT_STR}
};


/*******************************************************************************
**
** Function         _send_result_to_hf
**
** Description      Send an AT result code.
**
**
** Returns          void
**
*******************************************************************************/
static void _send_result_to_hf (hci_control_ag_session_cb_t *p_scb, UINT8 code, char *p_arg, INT16 int_arg)
{
    char    buf[HCI_CONTROL_AG_AT_MAX_LEN + 16];
    char    *p = buf;
    UINT16  len;

    /* init with \r\n */
    *p++ = '\r';
    *p++ = '\n';

    /* copy result code string */
    BCM_STRNCPY_S(p, sizeof(buf), bta_ag_result_tbl[code].p_res, HCI_CONTROL_AG_AT_MAX_LEN);
    p += strlen(bta_ag_result_tbl[code].p_res);

    /* copy argument if any */
    if (bta_ag_result_tbl[code].fmt == BTA_AG_RES_FMT_INT)
    {
        p += utl_itoa((UINT16) int_arg, p);
    }
    else if (bta_ag_result_tbl[code].fmt == BTA_AG_RES_FMT_STR)
    {
        BCM_STRNCPY_S(p, sizeof(buf), p_arg, HCI_CONTROL_AG_AT_MAX_LEN);
        p += strlen(p_arg);
    }

    /* finish with \r\n */
    *p++ = '\r';
    *p++ = '\n';

    /* send to RFCOMM */
    WICED_BT_TRACE( "[%u]Sent AT response[0x%02x]: %s", p_scb->app_handle, code, &buf[2] );

    wiced_bt_rfcomm_write_data( p_scb->rfc_conn_handle, buf, ( uint16_t )( p - buf ), &len );
}

/*******************************************************************************
**
** Function         _send_OK_to_hf
**
** Description      Send an OK result code.
**
**
** Returns          void
**
*******************************************************************************/
static void _send_OK_to_hf (hci_control_ag_session_cb_t *p_scb)
{
    _send_result_to_hf (p_scb, BTA_AG_RES_OK, NULL, 0);
}

/*******************************************************************************
**
** Function         _send_error_to_hf
**
** Description      Send an ERROR result code.
**                  errcode - used to send verbose errocode
**
**
** Returns          void
**
*******************************************************************************/
static void _send_error_to_hf (hci_control_ag_session_cb_t *p_scb, INT16 errcode)
{
    /* If HFP and extended audio gateway error codes are enabled */
    _send_result_to_hf (p_scb, BTA_AG_RES_ERROR, NULL, 0);
}


#if (BTM_WBS_INCLUDED == TRUE )
/*******************************************************************************
**
** Function         _parse_bac_command
**
** Description      Parse AT+BAC parameter string.
**
** Returns          Returns bitmap of supported codecs.
**
*******************************************************************************/
static void _parse_bac_command (hci_control_ag_session_cb_t *p_scb, char *p_s)
{
    UINT16              codec;
    BOOLEAN             cont = FALSE;       /* Continue processing */
    char                *p;

    p_scb->peer_supports_msbc = FALSE;

    while (p_s)
    {
        /* skip to comma delimiter */
        for (p = p_s; *p != ',' && *p != 0; p++)
            ;

        /* get integer value */
        if (*p != 0)
        {
            *p = 0;
            cont = TRUE;
        }
        else
            cont = FALSE;

        codec = utl_str2int(p_s);
        switch (codec)
        {
            case HCI_CONTROL_CODEC_CVSD:
                break;
            case HCI_CONTROL_CODEC_MSBC:
                p_scb->peer_supports_msbc = TRUE;
                break;
            default:
                /* Ignore any proprietary codecs */
                WICED_BT_TRACE ("Unknown Codec UUID(%d) received", codec);
                break;
        }

        if (cont)
            p_s = p + 1;
        else
            break;
    }
}
#endif


/*******************************************************************************
**
** Function         _handle_command_from_hf
**
** Description      AT command processing.
**
** Returns          void
**
*******************************************************************************/
static void _handle_command_from_hf (hci_control_ag_session_cb_t *p_scb, UINT16 cmd, UINT8 arg_type, char *p_arg, INT16 int_arg)
{
    UINT32      i;
#if (BTM_WBS_INCLUDED == TRUE )
    UINT16      codec_type;
#endif

    WICED_BT_TRACE ("HFP AT cmd:%d arg_type:%d arg:%d arg:%s", cmd, arg_type, int_arg, p_arg);

    switch (cmd)
    {
        case BTA_AG_HF_CMD_A:
        case BTA_AG_HF_CMD_VGS:
        case BTA_AG_HF_CMD_VGM:
        case BTA_AG_HF_CMD_CHUP:
        case BTA_AG_HF_CMD_CBC:
        case BTA_AG_HF_CMD_BIA:
        case BTA_AG_HF_CMD_CNUM:
        case BTA_AG_HF_CMD_CCWA:
            _send_OK_to_hf(p_scb);                          /* send OK */
            break;

        case BTA_AG_HF_CMD_BLDN:
        case BTA_AG_HF_CMD_D:
        case BTA_AG_HF_CMD_BINP:
        case BTA_AG_HF_CMD_NREC:
        case BTA_AG_HF_CMD_CLCC:
        case BTA_AG_HF_CMD_BTRH:
        case BTA_AG_HF_CMD_VTS:
        case BTA_AG_HF_CMD_CMEE:
            _send_error_to_hf (p_scb, BTA_AG_ERR_OP_NOT_SUPPORTED);
            break;

        case BTA_AG_HF_CMD_CHLD:
            if (arg_type == BTA_AG_AT_TEST)
            {
                /* send CHLD string followed by "OK" */
                _send_result_to_hf(p_scb, BTA_AG_RES_CHLD, "(0,1,2,3,4)", 0);

                /* if service level conn. not already open and our features or
                ** peer features does not have HF indicator, service level conn. now open
                ** Otherwise, SLC will be open later.
                */
                if ( !((ag_features & HCI_CONTROL_AG_FEAT_HF_IND) && (p_scb->hf_features & HCI_CONTROL_HF_FEAT_HF_IND)) )
                {
                    hci_control_ag_service_level_up (p_scb);
                }
            }
            _send_OK_to_hf(p_scb);
            break;

        case BTA_AG_HF_CMD_CIND:
            if (arg_type == BTA_AG_AT_TEST)
            {
                _send_result_to_hf (p_scb, BTA_AG_RES_CIND, BTA_AG_CIND_INFO, 0);
            }
            else if (arg_type == BTA_AG_AT_READ)
            {
                _send_result_to_hf (p_scb, BTA_AG_RES_CIND, BTA_AG_CIND_VALUES, 0);
            }
            _send_OK_to_hf(p_scb);
            break;

        case BTA_AG_HF_CMD_CLIP:
            /* store setting, send OK */
            p_scb->clip_enabled = (BOOLEAN) int_arg;
            _send_OK_to_hf(p_scb);
            break;

        case BTA_AG_HF_CMD_BVRA:
            _send_OK_to_hf (p_scb);
            if (arg_type == BTA_AG_AT_SET)
            {
                if ((int_arg == 1) && (!p_scb->b_sco_opened))
                    hci_control_ag_sco_create (p_scb, TRUE);
                else if ((int_arg == 0) && (p_scb->b_sco_opened))
                    hci_control_ag_sco_close (p_scb);
            }
            break;

        case BTA_AG_HF_CMD_BRSF:
            /* store peer features */
            p_scb->hf_features = (UINT16) int_arg;
            WICED_BT_TRACE ("BRSF HF : 0x%x , phone : 0x%x", p_scb->hf_features, ag_features);

            /* send BRSF, send OK */
            _send_result_to_hf(p_scb, BTA_AG_RES_BRSF, NULL, (INT16)ag_features);
            _send_OK_to_hf(p_scb);
            break;

        case BTA_AG_HF_CMD_COPS:
            _send_result_to_hf(p_scb, BTA_AG_RES_COPS, "0", 0);
            break;

        case BTA_AG_HF_CMD_CMER:
            _send_OK_to_hf(p_scb);                          /* send OK */
            hci_control_ag_service_level_up (p_scb);
            break;

#if (BTM_WBS_INCLUDED == TRUE )
        case BTA_AG_HF_CMD_BAC:
            _send_OK_to_hf(p_scb);

            /* parse codecs and set a flag if peer supports mSBC */
            _parse_bac_command (p_scb, p_arg);
            break;

        case BTA_AG_HF_CMD_BCS:
            _send_OK_to_hf(p_scb);

            /* stop cn timer */
            wiced_stop_timer(&p_scb->cn_timer);

            if (int_arg == HCI_CONTROL_CODEC_MSBC)
                p_scb->msbc_selected = WICED_TRUE;
            else
                p_scb->msbc_selected = WICED_FALSE;

            hci_control_ag_sco_create (p_scb, TRUE);
            break;

        case BTA_AG_HF_CMD_BCC:
            _send_OK_to_hf(p_scb);
            hci_control_ag_sco_create (p_scb, TRUE);
            break;
#endif

        default:
            break;
    }
}

/*******************************************************************************
**
** Function         hci_control_ag_send_BVRA_to_hf
**
** Description      Send +BVRA AT unsolicited response to peer.
**
** Returns          void
**
*******************************************************************************/
void hci_control_ag_send_BVRA_to_hf (hci_control_ag_session_cb_t *p_scb, BOOLEAN b_is_active)
{
    _send_result_to_hf (p_scb, BTA_AG_RES_BVRA, NULL, b_is_active ? 1 : 0);
}

#if (BTM_WBS_INCLUDED == TRUE )
/*******************************************************************************
**
** Function         hci_control_ag_send_BCS_to_hf
**
** Description      Send +BCS AT unsolicited response to peer.
**
** Returns          void
**
*******************************************************************************/
void hci_control_ag_send_BCS_to_hf (hci_control_ag_session_cb_t *p_scb)
{
    UINT16 codec_uuid;

    /* Try to use mSBC if the peer supports it */
    if (p_scb->peer_supports_msbc)
        codec_uuid = HCI_CONTROL_CODEC_MSBC;
    else
        codec_uuid = HCI_CONTROL_CODEC_CVSD;

    /* send +BCS */
    WICED_BT_TRACE ("send +BCS codec is %d", codec_uuid);

    _send_result_to_hf (p_scb, BTA_AG_RES_BCS, NULL, codec_uuid);
}
#endif

/******************************************************************************
**
** Function         hci_control_ag_parse_AT_command
**
** Description      Parse AT commands.  This function will take the input
**                  character string and parse it for AT commands according to
**                  the AT command table passed in the control block.
**
** Returns          void
**
******************************************************************************/
void hci_control_ag_parse_AT_command (hci_control_ag_session_cb_t *p_scb)
{
    int         i;
    UINT16      idx;
    UINT8       arg_type;
    char        *p_arg;
    INT16       int_arg = 0;

    /* Remove CR/LF from the buffer */
    for (i = 0; i < p_scb->res_len; i++)
        if ( (p_scb->res_buf[i] == '\r') || (p_scb->res_buf[i] == '\n') )
          p_scb->res_buf[i] = 0;

    for (i = 0; i < p_scb->res_len; i++)
    {
        if ( ((p_scb->res_buf[i]   != 'A') && (p_scb->res_buf[i]   != 'a'))
          || ((p_scb->res_buf[i+1] != 'T') && (p_scb->res_buf[i+1] != 't')) )
            continue;

        /* Got an "AT" */
        i += 2;

        /* loop through at command table looking for match */
        for (idx = 0; bta_ag_hfp_cmd[idx].p_cmd[0] != 0; idx++)
        {
            if (!utl_strucmp(bta_ag_hfp_cmd[idx].p_cmd, &p_scb->res_buf[i]))
                break;
        }

        if (bta_ag_hfp_cmd[idx].p_cmd[0] == 0)
        {
            _send_error_to_hf (p_scb, BTA_AG_ERR_OP_NOT_SUPPORTED);
            break;
        }

        /* Got a match; verify argument type */
        /* start of argument is p + strlen matching command */
        p_arg = p_scb->res_buf + i + strlen(bta_ag_hfp_cmd[idx].p_cmd);

        if (p_arg[0] == 0)                                  /* if no argument */
        {
            arg_type = BTA_AG_AT_NONE;
        }
        else if (p_arg[0] == '?' && p_arg[1] == 0)          /* else if arg is '?' and it is last character */
        {
            /* we have a read */
            arg_type = BTA_AG_AT_READ;
        }
        else if (p_arg[0] == '=' && p_arg[1] != 0)          /* else if arg is '=' */
        {
            if (p_arg[1] == '?' && p_arg[2] == 0)
            {
                /* we have a test */
                arg_type = BTA_AG_AT_TEST;
            }
            else
            {
                /* we have a set */
                arg_type = BTA_AG_AT_SET;

                /* skip past '=' */
                p_arg++;
            }
        }
        else                                                /* else it is freeform argument */
        {
            arg_type = BTA_AG_AT_FREE;
        }

        /* if arguments match command capabilities */
        if ((arg_type & bta_ag_hfp_cmd[idx].arg_type) != 0)
        {
            /* if it's a set integer check max, min range */
            if (arg_type == BTA_AG_AT_SET && bta_ag_hfp_cmd[idx].fmt == BTA_AG_AT_INT)
            {
                int_arg = utl_str2int(p_arg);

                if (int_arg < (INT16) bta_ag_hfp_cmd[idx].min || int_arg > (INT16) bta_ag_hfp_cmd[idx].max)
                {
                    WICED_BT_TRACE ("arg out of range: value: %u", int_arg);

                    /* arg out of range; error */
                    _send_error_to_hf (p_scb, BTA_AG_ERR_OP_NOT_SUPPORTED);
                }
                else
                {
                    _handle_command_from_hf (p_scb, idx, arg_type, p_arg, int_arg);
                }
            }
            else
            {
                _handle_command_from_hf (p_scb, idx, arg_type, p_arg, int_arg);
            }
        }
        else                /* else error */
        {
            _send_error_to_hf (p_scb, BTA_AG_ERR_OP_NOT_SUPPORTED);
        }

        break;              /* Only process one command at a time */
    }

    p_scb->res_len = 0;
}
