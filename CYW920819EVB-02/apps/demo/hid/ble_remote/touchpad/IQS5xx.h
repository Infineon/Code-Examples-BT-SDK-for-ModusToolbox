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

#ifdef SUPPORT_TOUCHPAD

#ifndef __IQS5XX_H__
#define __IQS5XX_H__

#define SHUTDOWN_TP_BY_RSTN    1

#include "i2cDevice.h"
#include "AzoteqIQS572.h"
#include "wiced.h"

// i2c slave device address
#define IQS5xx_ADDR 0x74
//#define TP_I2C_ADDRESS (IQS5xx_ADDR << 1)
#define TP_I2C_ADDRESS IQS5xx_ADDR
//#define TPBL_I2C_ADDRESS (TP_I2C_ADDRESS ^ 0x80)
#define MAX_INIT_DATA_SIZE 154 // 234  // add more entries for cal. Adjust this to match data in cgs file
#define WAKEUP_ADDR  0xA3
#define SUPPORTED_FINGER_COUNT 1
#define BOOTLOADER_VER_SIZE    2
#define BATTERY_MONITOR_INTERRUPT   7
#define TP_RSTN_ReleaseReset() GPIO_OUT(GPIO_RSTN_TP, GPIO_TOUCHPAD_ON)
#define TP_RSTN_HoldReset() GPIO_OUT(GPIO_RSTN_TP, GPIO_TOUCHPAD_OFF)

#define PROXIMITY_TAG_ID    0x80
#define xyDataStatus()      rpt.xyData.info
#define fingers(n)           rpt.xyData.fingers[n]

#pragma pack(1)
typedef PACKED struct
{
    uint8_t  tagId;
    uint8_t Xpos_H;
    uint8_t Xpos_L;
    uint8_t Ypos_H;
    uint8_t Ypos_L;
    uint8_t Strength_H;
    uint8_t Strength_L;
} XY_Data_finger_t;

typedef PACKED struct
{
    uint8_t                info;
    // sizeof(XY_Data_finger_t) * SUPPORTED_FINGER_COUNT = 7 * 1
    XY_Data_finger_t    fingers[SUPPORTED_FINGER_COUNT];
} Cmd_XY_Data_t;


typedef PACKED struct
{
    uint16_t  productN;
    uint16_t  projectN;
    uint8_t    major;
    uint8_t    minor;
    uint16_t  hwID;
    uint16_t  hwRev;
} cmd_Version_Info_t;

///////////////////////////////////
// Absolute XY report
typedef PACKED struct
{
    uint8_t            reportID;
    Cmd_XY_Data_t   xyData; // sizeof(Cmd_XY_Data_t) = 8
    uint8_t            count;
}AbsXYReport, * AbsXYRptPtr;
#pragma pack()

#define TOUCHPAD_RPT_PAYLOAD_SIZE       (sizeof(AbsXYReport) - 1) // minus reprot ID size
#define TOUCHPAD_REPORT_DATA(p)         (uint8_t*)&((p)->xyData)
#define TOUCHPAD_REPORT_DATA_SIZE(p)    (sizeof(AbsXYReport) - sizeof((p)->reportID))  // repportID is nto sent in LE
typedef struct
{
    uint8_t (*readFirmwareVersion)(uint8_t * buff);
    uint8_t (*init)();
    uint8_t (*shutdown)();
    void (*hwReset)();
    uint8_t (*wakeUp)();
    uint8_t (*waitForRDY)(uint8_t logic, uint32_t waitMs);
    uint8_t (*getInfo)();
    uint8_t (*proximityRpt)();
    uint8_t (*fingerCount)();
    uint8_t (*readXYData)();
    void (*clearData)();
    void (*clearFingerData)(XY_Data_finger_t * finger);
    AbsXYReport * (*getRpt)();
} TPDrv_t;

//class IQS5xx : public i2cDevice, i2cBitbang
typedef struct
{
    AbsXYReport rpt;
    uint8_t rdyGPIO;
    uint8_t bbMode;
} IQS5xx_t;

TPDrv_t * IQS5xx(uint8_t rdy_gpio);
#endif // __IQS5XX_H__

#endif // SUPPORT_TOUCHPAD
