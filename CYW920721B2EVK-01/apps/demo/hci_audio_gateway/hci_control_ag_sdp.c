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
 * This file contains SDP functionality required HF Device sample application.
 * SDP database definition is contained in this file and is not changed from
 * the controlling MCU.
 *
 */
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_cfg.h"
#include "hci_control_api.h"
#include "hci_control.h"
#include "wiced_memory.h"

extern void hci_control_ag_process_open_callback( hci_control_ag_session_cb_t *p_scb, uint8_t status );
extern void hci_control_ag_rfcomm_do_open( hci_control_ag_session_cb_t *p_scb );

/******************************************************
*                 Global Variables
******************************************************/
static wiced_bt_uuid_t  hf_uuid = {2, {UUID_SERVCLASS_HF_HANDSFREE}};

/******************************************************
*               Function Declarations
******************************************************/

/* declare sdp callback functions */
static void hci_control_ag_sdp_cback_1( uint16_t status );
static void hci_control_ag_sdp_cback_2( uint16_t status );

static void hci_control_ag_sdp_free_db( hci_control_ag_session_cb_t *p_scb );

/* SDP callback function table */
wiced_bt_sdp_discovery_complete_cback_t *hci_control_ag_sdp_callback_tbl[] =
{
    hci_control_ag_sdp_cback_1,
    hci_control_ag_sdp_cback_2,
};

static BOOLEAN hci_control_ag_sdp_find_attr( hci_control_ag_session_cb_t *p_scb );

/*
 * SDP callback function.
 */
static void hci_control_ag_sdp_cback( uint16_t sdp_status, uint8_t idx )
{
    uint16_t                      event;
    hci_control_ag_session_cb_t  *p_scb = &hci_control_cb.ag_scb[idx - 1];

    WICED_BT_TRACE( "hci_control_ag_sdp_cback status:0x%x", sdp_status );

    /* set event according to int/acp */
    if ( !p_scb->b_is_initiator )
    {
        if ( ( sdp_status == WICED_BT_SDP_SUCCESS ) || ( sdp_status == WICED_BT_SDP_DB_FULL ) )
        {
            hci_control_ag_sdp_find_attr ( p_scb );
        }
    }
    else
    {
        if ( ( sdp_status == WICED_BT_SDP_SUCCESS ) || ( sdp_status == WICED_BT_SDP_DB_FULL ) )
        {
            if ( hci_control_ag_sdp_find_attr( p_scb ) )
            {
                hci_control_ag_rfcomm_do_open( p_scb );
            }
            else
            {
                /* reopen server and notify app of the failure */
                hci_control_ag_rfcomm_start_server( p_scb );
                hci_control_ag_process_open_callback( p_scb, HCI_CONTROL_HF_STATUS_FAIL_SDP );
            }
        }
        else
        {
            /* reopen server and notify app of the failure */
            hci_control_ag_rfcomm_start_server(p_scb);
            hci_control_ag_process_open_callback(p_scb, HCI_CONTROL_HF_STATUS_FAIL_SDP);
        }
    }
    hci_control_ag_sdp_free_db( p_scb );
}

/*
 * SDP callback functions. Since there is no way to distinguish scb from the
 * callback we need separate callbacks for each scb.
 */
void hci_control_ag_sdp_cback_1( uint16_t status )
{
    hci_control_ag_sdp_cback( status, 1 );
}

void hci_control_ag_sdp_cback_2( uint16_t status )
{
    hci_control_ag_sdp_cback( status, 2 );
}

/*
 * Process SDP discovery results to find requested attributes for requested service.
 * Returns TRUE if results found, FALSE otherwise.
 */
BOOLEAN hci_control_ag_sdp_find_attr( hci_control_ag_session_cb_t *p_scb )
{
    wiced_bt_sdp_discovery_record_t     *p_rec = ( wiced_bt_sdp_discovery_record_t * ) NULL;
    wiced_bt_sdp_protocol_elem_t        pe;
    wiced_bt_sdp_discovery_attribute_t  *p_attr;
    BOOLEAN                             result = WICED_TRUE;

    WICED_BT_TRACE( "Looking for HFP service" );

    p_rec = wiced_bt_sdp_find_service_uuid_in_db( p_scb->p_sdp_discovery_db, &hf_uuid, p_rec );
    if ( p_rec == NULL )
    {
        WICED_BT_TRACE( "hci_control_ag_sdp_find_attr( ) - could not find AG service" );
        return ( WICED_FALSE );
    }

    /*** Look up the server channel number in the protocol list element ***/
    if ( wiced_bt_sdp_find_protocol_list_elem_in_rec( p_rec, UUID_PROTOCOL_RFCOMM, &pe ) )
    {
        WICED_BT_TRACE( "hci_control_ag_sdp_find_attr - num of proto elements -RFCOMM =0x%x\n",  pe.num_params );
        if ( pe.num_params > 0 )
        {
            p_scb->hf_scn = ( uint8_t )pe.params[0];
            WICED_BT_TRACE( "hci_control_ag_sdp_find_attr - found SCN in SDP record. SCN=0x%x\n", p_scb->hf_scn );
        }
        else
            result = WICED_FALSE;
    }
    else
    {
        result = WICED_FALSE;
    }

    /* get HFP supported features ( attribute ID 0x0311 ) */
    if ( ( p_attr = wiced_bt_sdp_find_attribute_in_rec( p_rec, 0x0311 ) ) != NULL )
    {
        /* Found attribute. Get value. but do not overwrite peer_feature if we already received +BRSF */
        if ( p_scb->hf_features == 0 )
            p_scb->hf_features = p_attr->attr_value.v.u16;
    }

    if ( wiced_bt_sdp_find_profile_version_in_rec( p_rec, UUID_SERVCLASS_HF_HANDSFREE, &p_scb->hf_version ) )
    {
        WICED_BT_TRACE( "HF device profile version: 0x%x\n", p_scb->hf_version );
    }

    return result;
}


/*
 * Do service discovery.
 */
void hci_control_ag_sdp_start_discovery( hci_control_ag_session_cb_t *p_scb )
{
    uint16_t        attr_list[4];
    uint8_t         num_attr;

    /* initiator - get proto list and features */
    if ( p_scb->b_is_initiator )
    {
        attr_list[0] = ATTR_ID_SERVICE_CLASS_ID_LIST;
        attr_list[1] = ATTR_ID_PROTOCOL_DESC_LIST;
        attr_list[2] = ATTR_ID_BT_PROFILE_DESC_LIST;
        attr_list[3] = ATTR_ID_SUPPORTED_FEATURES;
        num_attr = 4;
    }
    /* HFP acceptor; get features */
    else
    {
        attr_list[0] = ATTR_ID_SERVICE_CLASS_ID_LIST;
        attr_list[1] = ATTR_ID_BT_PROFILE_DESC_LIST;
        attr_list[2] = ATTR_ID_SUPPORTED_FEATURES;
        num_attr = 3;
    }

    /* allocate buffer for sdp database */
    p_scb->p_sdp_discovery_db = ( wiced_bt_sdp_discovery_db_t * ) wiced_bt_get_buffer( WICED_BUFF_MAX_SIZE );

    /* set up service discovery database; attr happens to be attr_list len */
    wiced_bt_sdp_init_discovery_db( p_scb->p_sdp_discovery_db, WICED_BUFF_MAX_SIZE, 1, &hf_uuid, num_attr, attr_list );

    /* initiate service discovery */
    if ( !wiced_bt_sdp_service_search_attribute_request( p_scb->hf_addr, p_scb->p_sdp_discovery_db,
                                      hci_control_ag_sdp_callback_tbl[p_scb->app_handle - 1] ) )
    {
        /* Service discovery not initiated - free discover db, reopen server, tell app  */
        hci_control_ag_sdp_free_db( p_scb );

        if ( p_scb->b_is_initiator )
        {
            hci_control_ag_rfcomm_start_server( p_scb );
            hci_control_ag_process_open_callback( p_scb, HCI_CONTROL_HF_STATUS_FAIL_SDP );
        }
    }
}

/*
 * Free discovery database.
 */
void hci_control_ag_sdp_free_db( hci_control_ag_session_cb_t *p_scb )
{
    if ( p_scb->p_sdp_discovery_db != NULL )
    {
        wiced_bt_free_buffer( p_scb->p_sdp_discovery_db );
        p_scb->p_sdp_discovery_db = NULL;
    }
}
