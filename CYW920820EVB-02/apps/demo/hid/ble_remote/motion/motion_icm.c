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
 * Motion Sensor Interface
 *
 */

#ifdef SUPPORT_MOTION

#include "brcm_fw_types.h"
#include "spar_utils.h"
#include "motion_icm.h"
#include "sensor_icm20608.h"
#include "wiced_hal_i2c.h"
#include "appDefs.h"

#include "wiced_result.h"
#include "wiced_timer.h"
#include "wiced_bt_trace.h"

// My own defines
#define SIZE_GYRO_ACC   12
#ifdef USE_MOTION_AS_AIR_MOUSE
 #define deltaX icm20608DeltaStatus.Delta.X
 #define deltaY icm20608DeltaStatus.Delta.Y
#endif
#define MOTION_IDLE_TIMEOUT_mS      5000 // 5 sec


///////////////////////////////////////////////////////////////////////////////////////////////
// Private variables
///////////////////////////////////////////////////////////////////////////////////////////////
// When touchpad is added this variable should also govern the behavior of TP.
// It is explicitly saved in AON memory
PLACE_DATA_IN_RETENTION_RAM MotionSensorICM_mode_t motion_mode;


MotionSensorICM_c * motionSensorPtr;
Intr_State          motionIntr;

t_struct_AIR_MOTION_ProcessDeltaSamples icm20608Samples;
#ifdef USE_MOTION_AS_AIR_MOUSE
t_struct_AIR_MOTION_ProcessDeltaStatus icm20608DeltaStatus;
MotionSensorICM_Mouse8bitXY_t motionData[MOTION_ICM_FIFO_CNT];
uint8_t newMouseClickState;        // Mouse click state
#else
// raw motion data
IcmRawMotionData_t motionData[MOTION_ICM_FIFO_CNT];
#endif
uint8_t fifoIndex;
uint8_t active      = FALSE;
uint8_t idleTimerOn = FALSE;
wiced_timer_t idle_timer;


void CMotionSensorICM_setMode(MotionSensorICM_mode_t m);
extern void i2c_setMux();
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
// private functions
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void CMotionSensorICM_initializeSensor(void)
{
#ifdef USE_MOTION_AS_AIR_MOUSE
    t_struct_AIR_MOTION_Init icm20608InitParameters;

    // Re-initialize hardware and get ready for option
    ICM20608_Init();

    icm20608InitParameters.DeltaGain.X = 15;
    icm20608InitParameters.DeltaGain.Y = 15;
    icm20608InitParameters.GyroOffsets.X = 0;
    icm20608InitParameters.GyroOffsets.Y = 0;
    icm20608InitParameters.GyroOffsets.Z = 0;
    icm20608InitParameters.GyroStaticMaxNoise = 16;
    icm20608InitParameters.StaticSamples = 400;
    icm20608InitParameters.SwipeMinDist = 250;
    icm20608InitParameters.SwipeMaxNoise = 200;
    icm20608InitParameters.StartupSamples = 16;
    icm20608InitParameters.ClickStillSamples = 30;
    icm20608InitParameters.ClickStillTolerance = AirMotionNormal;
    icm20608InitParameters.IsRollCompEnabled = TRUE;
    icm20608InitParameters.Acc1gLsb = 4096;
    icm20608InitParameters.GyroSensitivity = 262;
    // call Invensense init routine
    AIR_MOTION_Init( &icm20608InitParameters );
#else
    // Re-initialize hardware and get ready for option
    ICM20608_Init();
#endif
}

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
void CMotionSensorICM_initialize()
{
    // initialize variables
    fifoIndex = 0;
#ifdef USE_MOTION_AS_AIR_MOUSE
    newMouseClickState = 0;
#endif

    active      = FALSE;
    idleTimerOn = FALSE;

}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// Setting sensor mode to enabled, disabled, or shutdown
///////////////////////////////////////////////////////////////////////////////
void CMotionSensorICM_setMode(MotionSensorICM_mode_t m)
{
    if (m == motion_mode)
    {
        return;
    }
    motion_mode = m;

    wiced_stop_timer(&idle_timer);
    if (m == SENSOR_ENABLED)
    {
        WICED_BT_TRACE("mMode Active\n");
        CMotionSensorICM_initialize();
        CMotionSensorICM_initializeSensor();
        ICM20608_StartMotionDetection();
        motionSensorPtr->enableInterrupt();
    }
    else
    {
        WICED_BT_TRACE("mMode Inactive\n");
        motionSensorPtr->disableInterrupt();
        ICM20608_GYRO_Stop();
        ICM20608_ACC_Stop();
        idleTimerOn = FALSE;
        active  = FALSE;
    }
}

#ifndef USE_MOTION_AS_AIR_MOUSE
// a fake process of raw data. returns true if there is delta in data
uint8_t processRawData(t_struct_AIR_MOTION_ProcessDeltaSamples * pSample, IcmRawMotionData_t *pData)
{
    uint8_t delta;
    static t_struct_AIR_MOTION_ProcessDeltaSamples prevData = {0};
    delta = (uint8_t)memcmp(&prevData, pSample, sizeof(t_struct_AIR_MOTION_ProcessDeltaSamples));
    if (delta)
    {
        memcpy(&prevData, pSample, sizeof(t_struct_AIR_MOTION_ProcessDeltaSamples));
        pData->GyroSamples.X = pSample->GyroSamples.X;
        pData->GyroSamples.Y = pSample->GyroSamples.Y;
        pData->GyroSamples.Z = pSample->GyroSamples.Z;
        pData->AccSamples.X  = pSample->AccSamples.X;
        pData->AccSamples.Y  = pSample->AccSamples.Y;
        pData->AccSamples.Z  = pSample->AccSamples.Z;
    }
    return delta;
}
#endif
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void CMotionSensorICM_pollSensorData()
{
    //Poll ICM-20608 motion sensor activity
    ICM20608_GYRO_GetSample((int16_t *)&(icm20608Samples.GyroSamples));
    ICM20608_ACC_GetSample((int16_t *) &(icm20608Samples.AccSamples));
//WICED_BT_TRACE("g x=%d y=%d ", icm20608Samples.GyroSamples.X, icm20608Samples.GyroSamples.Y);
//WICED_BT_TRACE("a x=%d y=%d\n", icm20608Samples.AccSamples.X, icm20608Samples.AccSamples.Y);
#ifdef USE_MOTION_AS_AIR_MOUSE
    icm20608Samples.GyroSamples.X ^= 0xFFFF;
    icm20608Samples.GyroSamples.Y ^= 0xFFFF;
    // Invert accelerometer z upon request from Invensense
    icm20608Samples.AccSamples.Z ^= 0xFFFF;

    // Call the Invensense process delta function
    icm20608DeltaStatus = AIR_MOTION_ProcessDelta( icm20608Samples );
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
// Intr_C base class functions implementation
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
void CMotionSensorICM_enableInterrupt()
{
    // just call base class
    intrVtblPtr->enableInterrupt(&motionIntr);
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
void CMotionSensorICM_disableInterrupt()
{
    // just call base class
    intrVtblPtr->disableInterrupt(&motionIntr);
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
void CMotionSensorICM_setInterruptEnable(uint8_t en)
{
    // just call base class
    intrVtblPtr->setInterruptEnable(&motionIntr, en);
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
void CMotionSensorICM_clearInterrupt()
{
    // just call base class
    intrVtblPtr->clearInterrupt(&motionIntr);
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
uint8_t CMotionSensorICM_isInterruptPending()
{
    // just call base class
    return intrVtblPtr->isInterruptPending(&motionIntr);
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
uint8_t CMotionSensorICM_isInterruptEnabled()
{
    // just call base class
    return intrVtblPtr->isInterruptEnabled(&motionIntr);
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
uint8_t CMotionSensorICM_isInterruptPinActive()
{
    // just call base class
    return intrVtblPtr->isInterruptPinActive(&motionIntr);
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
void CMotionSensorICM_registerForInterrupt(void (*userfn)(void*, uint8_t), void * objPtr)
{
    // just call base class
    intrVtblPtr->registerForInterrupt(&motionIntr, userfn, objPtr);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// My new virtual functions
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
#ifdef USE_MOTION_AS_AIR_MOUSE
void CMotionSensorICM_setClickState(uint8_t s)
{
    WICED_BT_TRACE("m:click%d\n",s);
    newMouseClickState = s;
}
#endif

void idleTimerCb( uint32_t arg)
{
    if (active)
    {
        WICED_BT_TRACE("motion active, restart timer\n");
        // was still active, wait one more round.
        wiced_start_timer(&idle_timer, MOTION_IDLE_TIMEOUT_mS);
    }
    else
    {
        WICED_BT_TRACE("motion idle\n");
        idleTimerOn = FALSE;
#ifdef POLL_MOTION_WHILE_CONNECTED_AND_ACTIVE
        // when active we disable interrupt and use poll,
        // now are are inactive, enable interupt to wake us up.
        CMotionSensorICM_enableInterrupt();
#endif
    }
    active  = FALSE;
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
#define MAX_SKIP_CNT  100
uint8_t CMotionSensorICM_pollActivity(HidEventUserDefine * eventPtr)
{
#ifdef USE_MOTION_AS_AIR_MOUSE
    static uint8_t oldMouseClickState = 0;
#endif

    // clear any pending interrupt
    if (motionSensorPtr->isInterruptPinActive())
    {
        motionSensorPtr->clearInterrupt();
    }
    else
    {
        return NO_EVENTS;
    }

    if (!motionSensorPtr->isEnabled())
    {
        return NO_EVENTS;
    }

    // poll for new sensor data
    CMotionSensorICM_pollSensorData();

//WICED_BT_TRACE("\ndx=%d dy=%d\n", deltaX, deltaY);
    // we generate report only if there is delta X/Y change or mouse click change
#ifdef USE_MOTION_AS_AIR_MOUSE
    if (deltaX || deltaY || (oldMouseClickState != newMouseClickState))
#else
    if (processRawData(&icm20608Samples, &motionData[fifoIndex]))
#endif
    {
#ifdef USE_MOTION_AS_AIR_MOUSE
        // construct mouse report
        MotionSensorICM_Mouse8bitXY_t * ptr = &motionData[fifoIndex];
        //ptr->reportID = RPT_ID_MOUSE;
        ptr->buttonState = oldMouseClickState = newMouseClickState;
        ptr->xMotion = deltaX;
        ptr->yMotion = deltaY;
        ptr->scroll = 0;
#endif

        // construct event
        eventPtr->eventInfo.eventType = HID_EVENT_MOTION_DATA_AVAILABLE;
        eventPtr->userDataPtr = &motionData[fifoIndex];

        // advance index
        if (++fifoIndex >= MOTION_ICM_FIFO_CNT)
        {
            fifoIndex = 0;
        }

        if (idleTimerOn)
        {
            active = TRUE;
        }
        if (!idleTimerOn)
        {
WICED_BT_TRACE("start motion idle timer\n");
            wiced_start_timer(&idle_timer, MOTION_IDLE_TIMEOUT_mS);
            idleTimerOn = TRUE;
        }
        return HID_EVENT_AVAILABLE;
    }

    return NO_EVENTS;
}

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
void CMotionSensorICM_enable(uint8_t active)
{
    CMotionSensorICM_setMode(active ? SENSOR_ENABLED : SENSOR_DISABLED);
}

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
uint8_t CMotionSensorICM_isActive()
{
    return (active || idleTimerOn);
}


//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
uint8_t CMotionSensorICM_isEnabled()
{
    return motion_mode == SENSOR_ENABLED;
}

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
void CMotionSensorICM_shutdown()
{
    CMotionSensorICM_setMode(SENSOR_SHUTDOWN);
}

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
MotionSensorICM_c motionSensor = {
    CMotionSensorICM_enableInterrupt,
    CMotionSensorICM_disableInterrupt,
    CMotionSensorICM_setInterruptEnable,
    CMotionSensorICM_clearInterrupt,
    CMotionSensorICM_isInterruptPending,
    CMotionSensorICM_isInterruptEnabled,
    CMotionSensorICM_isInterruptPinActive,
    CMotionSensorICM_registerForInterrupt,
    CMotionSensorICM_pollActivity,
    CMotionSensorICM_enable,
    CMotionSensorICM_isEnabled,
    CMotionSensorICM_isActive,
    CMotionSensorICM_initialize,
    CMotionSensorICM_shutdown,
#ifdef USE_MOTION_AS_AIR_MOUSE
    CMotionSensorICM_setClickState,
#endif
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// Constructor
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
MotionSensorICM_c * CMotionSensorICM_CMotionSensorICM(void (*userfn)(void*, uint8_t), void * objPtr, uint8_t gpio, uint8_t activeLogic, uint16_t config)
{
    WICED_BT_TRACE("CMotion Cstr\n");

    i2c_setMux(); //IO mux must set
    wiced_hal_i2c_init(); //enable I2C clock
    wiced_hal_i2c_set_speed(I2CM_SPEED_400KHZ);

    // Call base Intr class contructor
    Intr_CIntr_Incomplete(&motionIntr, gpio, activeLogic, config);
    motionSensor.registerForInterrupt(userfn, objPtr);

    wiced_init_timer(&idle_timer, (wiced_timer_callback_fp)idleTimerCb, 0, WICED_MILLI_SECONDS_TIMER );
    motionSensorPtr = &motionSensor;
    return motionSensorPtr;
}


#endif // SUPPORT_MOTION
