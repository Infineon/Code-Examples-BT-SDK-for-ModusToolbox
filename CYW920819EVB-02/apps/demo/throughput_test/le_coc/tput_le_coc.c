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
/*
 * File name: tput_le_coc.c
 *
 * Description:
 *  This file has changes for LE CoC throughput.
 *  Enable/Disable the LE COC support using the global macro
 *  LE_COC_SUPPORT is defined in makefile.mk. By default, they are disabled.
 *
 *  PSM=19 and MTU=512 are defined in tput_le_coc.h.
 */

#include "sparcommon.h"
#include "tput_le_coc.h"

/*****************************************************************************
 *                         Variable Definitions
 ****************************************************************************/
uint32_t     numBytesTx_coc     = 0;    //no of bytes transferred successfully as LE CoC packets
uint32_t     failed_to_send_coc = 0;    //no of bytes rejected for transfer

/*****************************************************************************
 *                         Function Definitions
 ****************************************************************************/
/**************************************************************************************************
* Function Name: void le_coc_data_cback(void *context, UINT16 local_cid, UINT8 *p_data, UINT16 len)
***************************************************************************************************
* Summary: L2CAP Data RX callback from stack
*
* Parameters:
*   void *context   : Caller context provided with wiced_bt_l2cap_le_register().
*   UINT16 local_cid: Local CID assigned to the connection.
*   UINT8 *p_data   : Pointer to buffer.
*   UINT16 len      : Buffer length.
*
* Return:
*   None
*
*********************************************************************************************/
void le_coc_data_cback(void *context, UINT16 local_cid, UINT8 *p_data, UINT16 len)
{
#ifdef VERBOSE_THROUGHPUT_OUTPUT
    WICED_BT_TRACE("\r[%s] received %d bytes\n", __func__, len);
#endif
    return;
}

/********************************************************************************************
* Function Name: void le_coc_connect_ind_cback(void *context, BD_ADDR bda, UINT16 local_cid,
*                                              UINT16 psm, UINT8 id, UINT16 mtu_peer)
*********************************************************************************************
* Summary:
*   L2CAP connection indication callback from stack.
*   Save the peer info and stop ADV.
*
* Parameters:
*   void *context   : Caller context provided with wiced_bt_l2cap_le_register().
*   BD_ADDR bda     : BD Address of remote.
*   UINT16 local_cid: Local CID assigned to the connection
*   UINT16 psm      : PSM that the remote wants to connect to.
*   UINT8 id        : Identifier that the remote sent.
*   UINT16 mtu_peer : MTU of the peer.
*
* Return:
*   None
*
*********************************************************************************************/
void le_coc_connect_ind_cback(void *context, BD_ADDR bda, UINT16 local_cid,
                                     UINT16 psm, UINT8 id, UINT16 mtu_peer)
{
    uint8_t *p_data = le_coc_cb.peer_bda;

#ifdef VERBOSE_THROUGHPUT_OUTPUT
    WICED_BT_TRACE("\r[%s] from %B CID %d PSM 0x%x MTU %d \n", __func__, bda,
                                                   local_cid, psm, mtu_peer);
#endif

    /* Accept the connection */
    wiced_bt_l2cap_le_connect_rsp(bda, id, local_cid, L2CAP_CONN_OK, mtu,
                                           L2CAP_DEFAULT_BLE_CB_POOL_ID);

    /* Store peer info for reference*/
    le_coc_cb.local_cid = local_cid;
    BDADDR_TO_STREAM(p_data, bda);
    le_coc_cb.peer_mtu = mtu_peer;

    /* Stop advertising */
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
    WICED_BT_TRACE("\rTPUT: ADV Stop.\n");

    if (le_coc_cb.local_cid && le_coc_cb.local_cid < MAX_LOCAL_CID)
    {
        WICED_BT_TRACE("\rTPUT: LE COC connection established.\n");
    }
}

/*********************************************************************************
* Function Name: void le_coc_connect_cfm_cback(void *context, UINT16 local_cid,
*                                              UINT16 result, UINT16 mtu_peer)
**********************************************************************************
* Summary: LE COC connect confirmation callback from stack.
*
* Parameters:
*   void *context   : Caller context provided with wiced_bt_l2cap_le_register().
*   UINT16 local_cid: Local CID assigned to the connection
*   UINT16 result   : Result - 0 = connected, non-zero means failure reason.
*   UINT16 mtu_peer : MTU of the peer.
*
* Return:
*   None
*
*********************************************************************************/
void le_coc_connect_cfm_cback(void *context, UINT16 local_cid,
                               UINT16 result, UINT16 mtu_peer)
{
#ifdef VERBOSE_THROUGHPUT_OUTPUT
    WICED_BT_TRACE("\r[%s] MTU %d \n", __func__, mtu_peer);
#endif
    if (result == 0)
    {
        /* Store peer info for reference*/
        le_coc_cb.local_cid = local_cid;
        le_coc_cb.peer_mtu = mtu_peer;
    }
}

/*********************************************************************************************
* Function Name: void le_coc_disconnect_ind_cback(void *context, UINT16 local_cid, BOOLEAN ack)
**********************************************************************************************
* Summary: LE COC disconnect indication callback from stack.
*          Send ACK if needed by peer and reset the peer info.
*
* Parameters:
*   void *context   : Caller context provided with wiced_bt_l2cap_le_register().
*   UINT16 local_cid: Local CID assigned to the connection
*   BOOLEAN ack     : Boolean whether upper layer should ack this.
*
* Return:
*   None
*
*********************************************************************************************/
void le_coc_disconnect_ind_cback(void *context, UINT16 local_cid, BOOLEAN ack)
{
#ifdef VERBOSE_THROUGHPUT_OUTPUT
    WICED_BT_TRACE("\r[%s] CID %d \n", __func__, local_cid);
#endif

    /* Send disconnect response if needed */
    if (ack)
    {
        wiced_bt_l2cap_le_disconnect_rsp(local_cid);
    }

    if (le_coc_cb.local_cid == local_cid)
    {
        le_coc_cb.local_cid = MAX_LOCAL_CID;
        memset(le_coc_cb.peer_bda, 0, BD_ADDR_LEN);
        //Reset congestion status to the default
        coc_data_congested = FALSE;
        WICED_BT_TRACE("\r###### TPUT: LE CoC disconnected ######\n");
    }
}

/************************************************************************************************
* Function Name: void le_coc_disconnect_cfm_cback(void *context, UINT16 local_cid, UINT16 result)
*************************************************************************************************
* Summary: LE COC disconnect confirmation callback from stack.
*          Reset the peer info.
*
* Parameters:
*   void *context   : Caller context provided with wiced_bt_l2cap_le_register().
*   UINT16 local_cid: Local CID assigned to the connection
*   UINT16 result   : Result.
*
* Return:
*   None
*
************************************************************************************************/
void le_coc_disconnect_cfm_cback(void *context, UINT16 local_cid, UINT16 result)
{
#ifdef VERBOSE_THROUGHPUT_OUTPUT
    WICED_BT_TRACE("\r[%s] CID %d \n", __func__, local_cid);
#endif
    if (le_coc_cb.local_cid == local_cid)
    {
        le_coc_cb.local_cid = MAX_LOCAL_CID;
        memset(le_coc_cb.peer_bda, 0, BD_ADDR_LEN);
    }
}

/**********************************************************************************************
* Function Name: void tput_send_l2cap_coc_pkt(void)
***********************************************************************************************
* Summary: Send LE CoC packets only when TX buffers are available and
*          when there is no congestion. Update the number of bytes
*          successfully transmitted.
*
* Parameters:
*   None
*
* Return:
*   None
*
**********************************************************************************************/
void tput_send_l2cap_coc_pkt(void)
{
    uint8_t status = L2CAP_DATAWRITE_SUCCESS;
    uint16_t data_len = 0;
    uint32_t buf_left = 0;

    data_len = sizeof(coc_data)/sizeof(coc_data[0]);

    if (data_len > le_coc_cb.peer_mtu)
    {
#ifdef VERBOSE_THROUGHPUT_OUTPUT
        WICED_BT_TRACE("\rTPUT: Write data len(%d) > Peer's MTU(%d).\n",
                                          data_len, le_coc_cb.peer_mtu);
#endif
        return;
    }

    buf_left = wiced_bt_ble_get_available_tx_buffers();
    if (buf_left > MINIMUM_LE_COC_TX_BUFFER_LEN)
    {
        if (!coc_data_congested)
        {
            status = wiced_bt_l2cap_le_data_write(le_coc_cb.local_cid,
                                (uint8_t *)&coc_data[0], data_len, 0);
            if (status == L2CAP_DATAWRITE_SUCCESS)
            {
                numBytesTx_coc += data_len;
                /* LED 1 ON */
                wiced_hal_gpio_set_pin_output(CONGESTION_LED, GPIO_PIN_OUTPUT_LOW);
            }
            else
            {
#ifdef VERBOSE_THROUGHPUT_OUTPUT
                WICED_BT_TRACE("\rTPUT: L2CAP write: %s\n.", (status==L2CAP_DATAWRITE_FAILED)?
                                                                                     "Failed":
                                                                                  "Congested");
#endif
                if (status == L2CAP_DATAWRITE_CONGESTED) {
                    coc_data_congested = TRUE;
                }
                /* LED 1 OFF */
                wiced_hal_gpio_set_pin_output(CONGESTION_LED, GPIO_PIN_OUTPUT_HIGH);
            }
        } //(!coc_data_congested)
        else
        {
#ifdef VERBOSE_THROUGHPUT_OUTPUT
            WICED_BT_TRACE("\rTPUT: LE CONGESTION");
#endif
           /* LED 1 OFF */
           wiced_hal_gpio_set_pin_output(CONGESTION_LED, GPIO_PIN_OUTPUT_HIGH);
        }
    }
    else
    {
        failed_to_send_coc++;
#ifdef VERBOSE_THROUGHPUT_OUTPUT
        WICED_BT_TRACE("TPUT:TX BUFFER FULL\r");
#endif
        /* LED 1 OFF since TX failed */
        wiced_hal_gpio_set_pin_output(CONGESTION_LED, GPIO_PIN_OUTPUT_HIGH);
    }
}

/************************************************************************************************
* Function Name: void le_coc_congestion_cback(void *context, UINT16 local_cid, BOOLEAN congested)
*************************************************************************************************
* Summary: LE COC congestion callback from stack.
*          This gets invoked on every congestion status(congested or free) change.
*          Check for congestion free and update the local variable.
*
* Parameters:
*   void *context       : Caller context provided with wiced_bt_l2cap_register().
*   UINT16 local_cid    : Local CID.
*   BOOLEAN congested   : TRUE if congested, FALSE if uncongested.
*
* Return:
*   None
*
************************************************************************************************/
void le_coc_congestion_cback(void *context, UINT16 local_cid, BOOLEAN congested)
{
#ifdef VERBOSE_THROUGHPUT_OUTPUT
    WICED_BT_TRACE("\r[%s] CID: %d, Congested:%d \n", __func__, local_cid, congested);
#endif
    //update the congestion status for local reference.
    coc_data_congested = congested;
}

/**************************************************************
* Function Name: void le_coc_init(void)
***************************************************************
* Summary: LE COC initialization on BTM_ENABLED_EVENT.
*          Register LE COC callbacks.
*
* Parameters:
*   None
*
* Return:
*   None
*
***************************************************************/
void le_coc_init(void)
{
    /* Clear app control block */
    memset(&le_coc_cb, 0, sizeof(le_coc_cb_t));
    le_coc_cb.local_cid = MAX_LOCAL_CID;

    /* Register LE l2cap callbacks */
    wiced_bt_l2cap_le_register(psm, &l2c_appl_info, NULL);
}

#endif // LE_COC_SUPPORT
