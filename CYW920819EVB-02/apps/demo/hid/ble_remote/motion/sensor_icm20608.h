/* Copyright Â© 2014-2015 InvenSense Inc. All rights reserved.
This software, related documentation and any modifications thereto (collectively "Software")
is subject to InvenSense and its licensors' intellectual property rights under U.S. and international copyright and other intellectual property rights laws.
InvenSense and its licensors retain all intellectual property and proprietary rights in
and to the Software and any use, reproduction, disclosure or distribution of the
Software without an express license agreement from InvenSense is strictly prohibited. */

#ifdef SUPPORT_MOTION

#ifndef SENSOR_ICM20608_H
#define SENSOR_ICM20608_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include <stdint.h>
//#include <stdbool.h>

/* Public define ------------------------------------------------------------*/

/* Public typedef -----------------------------------------------------------*/

/* Public variable -----------------------------------------------------------*/

/* Public function prototype -------------------------------------------------*/

// Initialise GPIO for regulator of GYRO
void ICM20608_Init(void);

// Put ICM20608 GYRO only in Power down.
void ICM20608_GYRO_Stop(void);

// Put ICM20608 ACC in Power down
void ICM20608_ACC_Stop(void);

// Put ICM20608 ACC and GYRO in Normal mode.
void ICM20608_GYROACC_Start(void);

// Start accelerometer in motion detection mode
void ICM20608_StartMotionDetection(void);

// Disable motion detection mode
void ICM20608_StopMotionDetection(void);

// Get X, Y and Z GYRO informations
void ICM20608_GYRO_GetSample(int16_t Sample[3]);

// Get X, Y and Z ACC informations
void ICM20608_ACC_GetSample(int16_t Sample[3]);

// Change referential
void ICM20608_GYRO_ChangeRef(int16_t Sample[3]);

// Change referential
void ICM20608_ACC_ChangeRef(int16_t Sample[3]);

// Manage interrupt
void ICM20608_ProcessIt(void);

#ifdef __cplusplus
};
#endif

#endif

#endif // SUPPORT_MOTION
