/*
	Copyright © 2014-2015 InvenSense Inc. All rights reserved.
	This software, related documentation and any modifications thereto (collectively “Software”)
	is subject to InvenSense and its licensors' intellectual property rights under U.S. and international copyright and other intellectual property rights laws.
	InvenSense and its licensors retain all intellectual property and proprietary rights in
	and to the Software and any use, reproduction, disclosure or distribution of the
	Software without an express license agreement from InvenSense is strictly prohibited.
*/


/**
 *  @ifnot INVN_LIB_DISABLE_MAINPAGE
 *  @mainpage AIR MOTION LIBRARY
 *  @endif
 *  @author  Invensense
 *  @image html logo.png
 *  @image rtf  logo.png
 *  @version V_03.07.00
 *  @if INVN_LIB_DISABLE_MAINPAGE
 *  @ingroup Invensense_pointing
 *  @endif
 *
 *  @section AML_INTRO Introduction
 *    The Air Motion Library is a highly programmable API for managing mouse cursor motion from 2, or 3, axes gyroscope and optional 3 axes accelerometer.
 *    Embedded Invensense proprietary algorithms are responsible for converting motion sensors data into delta X and delta Y pointer movements.
 *    The library is intended to be used in free space pointing devices to operate in-air point and click navigation, just like a classic 2D mouse will do on a desk.
 *    In addition to its pointing feature, Air Motion Library is also capable of swipe motion recognition: 'Up', 'Down', 'Left', 'Right', 'Clockwise' and 'CounterClockwise' swipes are currently supported.
 *
 *  @section AML_REF AIR MOTION Library sensors referential
 *      @li When the device lies flat on its back, accelerometer must see +1G on Z axis
 *      @li When the device lies on its right side, accelerometer must see +1G on Y axis
 *      @li When the device is held vertically pointing down, accelerometer must see +1G on X axis
 *      @li A clockwise rotation around the gyroscope X axis generates positive values
 *      @li A clockwise rotation around the gyroscope Y axis generates positive values i.e. negative delta Y on screen (cursor moves to the top)
 *      @li A clockwise rotation around the gyroscope Z axis generates positive values i.e. positive delta X on screen (cursor moves to the right)
 *
 *  @image html axis.png
 *  @image rtf  axis.png
 *
 *  @section AML_API API
 *    The AIR MOTION Library uses the following functions :
 *      @li AIR_MOTION_Init() : This function must be called for library initialization, each time processing (re)starts.
 *      @li AIR_MOTION_ProcessDelta() : This function must be called each time new sensors values are available. The library is designed to run at 100Hz.
 *
 *  @section AML_CALIBRATION Calibration
 *    Gyroscope offsets values can fluctuate over time depending on various parameters.
 *    The library supports a continuous calibration which computes gyroscope offsets values in real time.
 *
 *    Each time the AIR_MOTION_ProcessDelta() is called, gyroscope instant "staticness" is checked, based on 'GyroStaticMaxNoise' parameter.
 *    If the device is considered static for 'StaticSamples' consecutive samples, new gyroscope offset values are computed. The remote can then be considered as calibrated.
 *    You can check 'IsStatic' and 'NewGyroOffset' parameters to track calibration status over time.
 *
 *    'GyroSensitivity' parameter is used to calibrate motion algorithms.
 *    To determine its value, use the following formula:
 *    @li GyroSensitivity = ( 2 ^ ( number_of_bits_gyro - 1) * 16 ) / max_scale_gyro
 *
 *  Typical gyroscopes scale are 400 deg/s or 2000 deg/s.
 *  Example: 16-bits gyroscope, +/-2000 dps
 *           GyroSensitivity = 2^15 * 16 / 2000 ~= 262
 *
 *  @section AML_ROLLCOMPENSATION RollCompensation
 *    The Air Motion Library embeds a roll-compensation algorithm along with its pointing feature.
 *    Roll-compensation allows user's movement to be reliably reproduced on the screen, independently from the device roll orientation.
 *    To turn on or off this feature, set 'IsRollCompEnabled' parameter accordingly (feature only available for 3A3G configurations).
 *
 *    The field 'Acc1gLsb' is only used when roll-compensation feature is enabled.
 *    To determine its value, use the following formula:
 *    @li Acc1gLsb = 2 ^ ( number_of_bits_acc - 1) / max_scale_acc
 *
 *  Typical accelerometers scale are +/-2G, +/-4G or +/-8G.
 *  Example: 12-bits accelerometer, +/-8G
 *           Acc1gLsb = 2^11 / 8 ~= 256
 *
 *  @section AML_EASYCLICK EasyClick
 *    This feature allows user to perform more accurate and stable mouse clicks for a better experience.
 *    On click press, the library will freeze the pointer (dX and dY null) for maximum 'ClickStillSamples' samples to avoid undesired movement.
 *    During this period, if the device movement quantity is too important (see 'ClickStillTolerance'), the pointer will be released sooner.
 *
 */


/**
*********************************************************************************
* @file AIR_MOTION_Lib.h
* Header file for Air Motion Library
*********************************************************************************
*/


#ifndef __AIR_MOTION_LIB_H
#define __AIR_MOTION_LIB_H


/* Includes ------------------------------------------------------------------*/

// Standard integer
#include "stdint.h"


// Standard bool
#include "inven_stdbool.h"
//#include "data_types.h"


/* Public define -------------------------------------------------------------*/

// Version
#define AIRMOTIONLIB_VERSION_MAJOR  3
#define AIRMOTIONLIB_VERSION_MINOR  7
#define AIRMOTIONLIB_VERSION_PATCH  0
#define AIRMOTIONLIB_VERSION_SUFFIX ""


/* Public typedef ------------------------------------------------------------*/

/**
 * Holds status bits reporting the last detected gesture if any
 */
typedef struct
{
    uint8_t Left                :1;                         //!< Asserted if a left swipe has been detected
    uint8_t Right               :1;                         //!< Asserted if a right swipe has been detected
    uint8_t Up                  :1;                         //!< Asserted if a up swipe has been detected
    uint8_t Down                :1;                         //!< Asserted if a down swipe has been detected
    uint8_t Clockwise           :1;                         //!< Asserted if a clockwise swipe has been detected (3A3G only)
    uint8_t CounterClockwise    :1;                         //!< Asserted if a counter clockwise swipe has been detected (3A3G only)
} t_struct_AIR_MOTION_Swipes;


/**
 * Holds sensor 3 axes values
 * May hold the sensor data itself or may be used to specify sensor axes attribute (e.g. offsets)
 */
typedef struct
{
    int16_t X;                                              //!< Data pertaining to the X axis
    int16_t Y;                                              //!< Data pertaining to the Y axis
    int16_t Z;                                              //!< Data pertaining to the Z axis
} t_struct_AIR_MOTION_SensorAxis;


/**
* Holds computed delta values
* May hold the delta data itself or may be used to specify delta gain values
*/
typedef struct
{
    int8_t  X;                                              //!< Delta data relative to the X axis
    int8_t  Y;                                              //!< Delta data relative to the Y axis
} t_struct_AIR_MOTION_Delta;

/**
* Holds a feature sensitivity level
*/
typedef enum
{
    AirMotionLow,
    AirMotionNormal,
    AirMotionHigh
} t_enum_AIR_MOTION_Level;


/**
* Holds input parameters to pass to the AIR_MOTION_Init() function
*/
typedef struct
{
    t_struct_AIR_MOTION_Delta       DeltaGain;              //!< Delta gain values for X and Y axes
    t_struct_AIR_MOTION_SensorAxis  GyroOffsets;            //!< Initial gyroscope offset values
    uint8_t                         GyroStaticMaxNoise;     //!< Gyroscope maximum value per axis for device to be considered currently static
    uint16_t                        StaticSamples;          //!< Number of consecutive "static" samples for device to be considered fully static
    uint16_t                        SwipeMinDist;           //!< Minimum distance, as deltas sum, for a swipe to be detected ('0' means no processing)
    uint16_t                        SwipeMaxNoise;          //!< Maximum noise level, as deltas sum, for a swipe to be rejected
    uint8_t                         StartupSamples;         //!< Number of samples to discard before starting computation
    uint8_t                         ClickStillSamples;      //!< Maximum number of null pointing samples after a click press
    t_enum_AIR_MOTION_Level         ClickStillTolerance;    //!< Stillness tolerance level, as maximum movement quantity, for pointing samples to be forced null after a click press
    inven_bool                      IsRollCompEnabled;      //!< Activate roll-compensation feature
    uint16_t                        Acc1gLsb;               //!< Norm value read from accelerometer when device is static (Earth's gravitational acceleration)
    uint16_t                        GyroSensitivity;        //!< Gyroscope sensitivity as '16*LSB / °/s'
} t_struct_AIR_MOTION_Init;


/**
* Holds input parameters to pass to the AIR_MOTION_ProcessDelta() function
*/
typedef struct
{
    t_struct_AIR_MOTION_SensorAxis  GyroSamples;            //!< Gyroscope samples
    t_struct_AIR_MOTION_SensorAxis  AccSamples;             //!< Accelerometer samples
    uint8_t                         ClickSample;            //!< Click buttons state sample
} t_struct_AIR_MOTION_ProcessDeltaSamples;


/**
* Holds status bits of the last sample processing
*/
typedef struct
{
    uint8_t IsDeltaComputed     :1;                         //!< True if delta have been computed
    uint8_t IsStatic            :1;                         //!< True if device is considered currently static
    uint8_t NewGyroOffset       :1;                         //!< True if new gyroscope offsets have been computed
} t_struct_AIR_MOTION_Status;


/**
* Holds output parameters of the AIR_MOTION_ProcessDelta() function
*/
typedef struct
{
    t_struct_AIR_MOTION_Status      Status;                 //!< Status bits of the last sample processing
    t_struct_AIR_MOTION_SensorAxis  GyroOffsets;            //!< Gyroscope offset values
    t_struct_AIR_MOTION_Delta       Delta;                  //!< Computed delta values
    t_struct_AIR_MOTION_Swipes      SwipesDetected;         //!< Bitfield of detected swipes
} t_struct_AIR_MOTION_ProcessDeltaStatus;


/* Public variable -----------------------------------------------------------*/

/* Public function prototype -------------------------------------------------*/


/**
* This function initializes the library with user parameters.
* Below is a configuration example, with gyroscope data read on 16 bits with scale fixed to +/- 2000 deg/s.
* Accelerometer data are read on 12 bits with scale fixed to +/- 8G.
* @code
*   t_struct_AIR_MOTION_Init lInitParameters;
*
*   // Fill structure fields
*   // If gyroscope is 2 axes, only Y and Z axes values are taken into account in motion processing algorithm
*   lInitParameters.DeltaGain.X             = 15;
*   lInitParameters.DeltaGain.Y             = 15;
*   lInitParameters.GyroOffsets.X           = 0;
*   lInitParameters.GyroOffsets.Y           = 0;
*   lInitParameters.GyroOffsets.Z           = 0;
*   lInitParameters.GyroStaticMaxNoise      = 16;
*   lInitParameters.StaticSamples           = 400;
*   lInitParameters.SwipeMinDist            = 250;
*   lInitParameters.SwipeMaxNoise           = 200;
*   lInitParameters.StartupSamples          = 16;
*   lInitParameters.ClickStillSamples       = 30;
*   lInitParameters.ClickStillTolerance     = AirMotionNormal;
*   lInitParameters.IsRollCompEnabled       = true;
*   lInitParameters.Acc1gLsb                = 256;
*   lInitParameters.GyroSensitivity         = 262;
*
*   // Call the init function
*   AIR_MOTION_Init(&lInitParameters);
* @endcode
* @param [in]   pInitParameters :   Address of the init parameters structure to use
* @return None
*/
void AIR_MOTION_Init(const t_struct_AIR_MOTION_Init *pInitParameters);


/**
* This function re-initializes swipe recognition.
* Allows to restart gesture processing without resetting pointing.
* @return None
*/
void AIR_MOTION_ResetSwipeRecognition();


/**
* This function processes mouse delta and swipe recognition.
* This function must be called at 100Hz.
* When a swipe is detected, the library stops the swipe detection.
* @code
*   t_struct_AIR_MOTION_ProcessDeltaSamples  lSensorSamples;
*   t_struct_AIR_MOTION_ProcessDeltaStatus   lProcessDeltaStatus;
*
*   // Get gyroscope samples
*   GYRO_GetSample(&lSensorSamples.GyroSamples);
*
*   // Get accelerometer samples
*   ACC_GetSample(&lSensorSamples.AccSamples);
*
*   // Get click state sample ('0' iff click button(s) released)
*   CLICK_GetSample(&lSensorSamples.ClickSample);
*
*   // Call the process delta function
*   lProcessDeltaStatus = AIR_MOTION_ProcessDelta(lSensorSamples);
*
*   // For example, send delta on radio
*   RADIO_SendDelta(lProcessDeltaStatus.Delta);
* @endcode
* @param [in]   lSensorSamples :    Sensors samples structure
* @return See t_struct_AIR_MOTION_ProcessDeltaStatus for documentation
*/
t_struct_AIR_MOTION_ProcessDeltaStatus AIR_MOTION_ProcessDelta(t_struct_AIR_MOTION_ProcessDeltaSamples lSensorSamples);


#endif // __AIR_MOTION_LIB_H
