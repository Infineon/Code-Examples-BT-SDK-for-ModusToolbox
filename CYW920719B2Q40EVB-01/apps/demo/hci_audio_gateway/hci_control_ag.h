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
 * This is the public interface file for the handsfree (HF) subsystem of
 * HCI_CONTROL application.
 */
#ifndef HCI_CONTROL_HF_H
#define HCI_CONTROL_HF_H

/******************************************************
 *                     Constants
 ******************************************************/
#define HFP_VERSION_1_5                 0x0105
#define HFP_VERSION_1_6                 0x0106
#define HFP_VERSION_1_7                 0x0107

/* HF feature masks */
#define HCI_CONTROL_HF_FEAT_ECNR        0x00000001   /* Echo cancellation and/or noise reduction */
#define HCI_CONTROL_HF_FEAT_3WAY        0x00000002   /* Call waiting and three-way calling */
#define HCI_CONTROL_HF_FEAT_CLIP        0x00000004   /* Caller ID presentation capability  */
#define HCI_CONTROL_HF_FEAT_VREC        0x00000008   /* Voice recoginition activation capability  */
#define HCI_CONTROL_HF_FEAT_RVOL        0x00000010   /* Remote volume control capability  */
#define HCI_CONTROL_HF_FEAT_ECS         0x00000020   /* Enhanced Call Status  */
#define HCI_CONTROL_HF_FEAT_ECC         0x00000040   /* Enhanced Call Control  */
#define HCI_CONTROL_HF_FEAT_CODEC       0x00000080   /* Codec negotiation */
#define HCI_CONTROL_HF_FEAT_HF_IND      0x00000100   /* HF Indicators */
#define HCI_CONTROL_HF_FEAT_ESCO        0x00000200   /* eSCO S4 ( and T2 ) setting supported */


/* AG feature masks */
#define HCI_CONTROL_AG_FEAT_3WAY        0x00000001   /* Three-way calling */
#define HCI_CONTROL_AG_FEAT_ECNR        0x00000002   /* Echo cancellation and/or noise reduction */
#define HCI_CONTROL_AG_FEAT_VREC        0x00000004   /* Voice recognition */
#define HCI_CONTROL_AG_FEAT_INBAND      0x00000008   /* In-band ring tone */
#define HCI_CONTROL_AG_FEAT_VTAG        0x00000010   /* Attach a phone number to a voice tag */
#define HCI_CONTROL_AG_FEAT_REJECT      0x00000020   /* Ability to reject incoming call */
#define HCI_CONTROL_AG_FEAT_ECS         0x00000040   /* Enhanced Call Status */
#define HCI_CONTROL_AG_FEAT_ECC         0x00000080   /* Enhanced Call Control */
#define HCI_CONTROL_AG_FEAT_EXTERR      0x00000100   /* Extended error codes */
#define HCI_CONTROL_AG_FEAT_CODEC       0x00000200   /* Codec Negotiation */
#define HCI_CONTROL_AG_FEAT_HF_IND      0x00000400   /* HF Indicators */
#define HCI_CONTROL_AG_FEAT_ESCO        0x00000800   /* eSCO S4 ( and T2 ) setting supported */

/* SCO Codec Types */
#define HCI_CONTROL_CODEC_CVSD          0x0001
#define HCI_CONTROL_CODEC_MSBC          0x0002


/* HS settings */
typedef struct
{
    uint8_t        spk_vol;
    uint8_t        mic_vol;
    BOOLEAN        ecnr_enabled;
} hci_control_ag_settings_t;

/* special handle values used with APIs */
#define HCI_CONTROL_HF_HANDLE_NONE      0
#define HCI_CONTROL_HF_HANDLE_ALL       0xFFFF




/* data associated with AG_OPEN_EVT */
typedef struct
{
    BD_ADDR             bd_addr;
    uint8_t             status;
} hci_control_ag_open_t;

/* data associated with AG_CONNECTED_EVT */
typedef struct
{
     uint32_t   peer_features;
} hci_control_ag_connect_t;

/* union of data associated with AG callback */
typedef union
{
    hci_control_ag_open_t    open;
    hci_control_ag_connect_t conn;
} hci_control_ag_event_t;

/* ASCII charcter string of arguments to the AT command or response */
#define HCI_CONTROL_AG_AT_MAX_LEN       200


#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************************************
**  External Function Declarations
*****************************************************************************/

extern UINT32 ag_features;

/*
 * Start the handsfree service. This function must be called once to initialize
 * the handsfree device.  Internally function starts SDP and RFCOMM processing.
 * This function must be called before other function in the HF API called.
 */
extern void hci_control_ag_startup( );

/*
 * Opens a connection to a audio gateway.  When connection is open callback
 * function is called with a HCI_CONTROL_HF_EVENT_OPEN event indicating
 * success or failure of the operation. Only the service level data connection
 * is opened. The audio connection is not opened.
 */
extern void hci_control_ag_connect ( BD_ADDR bd_addr );

/*
 * Close the current connection to a audio gateway.  Any current audio
 * connection will also be closed.
 */
extern void hci_control_ag_disconnect( uint16_t handle );

/*
 * Opens an audio connection to the currently connected audio gateway specified
 * by the handle.
 */
extern  void hci_control_ag_audio_open( uint16_t handle );

/*
 * Close the currently active audio connection to a audio gateway specified by
 * the handle. The data connection remains opened.
 */
extern void hci_control_ag_audio_close( uint16_t handle );


#ifdef __cplusplus
}
#endif

#endif /* HF_CONTROL_H */
