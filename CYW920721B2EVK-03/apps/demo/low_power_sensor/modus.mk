#
# Copyright 2019, Cypress Semiconductor Corporation or a subsidiary of
# Cypress Semiconductor Corporation. All Rights Reserved.
#
# This software, including source code, documentation and related
# materials ("Software"), is owned by Cypress Semiconductor Corporation
# or one of its subsidiaries ("Cypress") and is protected by and subject to
# worldwide patent protection (United States and foreign),
# United States copyright laws and international treaty provisions.
# Therefore, you may use this Software only as provided in the license
# agreement accompanying the software package from which you
# obtained this Software ("EULA").
# If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
# non-transferable license to copy, modify, and compile the Software
# source code solely for use in connection with Cypress's
# integrated circuit products. Any reproduction, modification, translation,
# compilation, or representation of this Software except as specified
# above is prohibited without the express written permission of Cypress.
#
# Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
# reserves the right to make changes to the Software without notice. Cypress
# does not assume any liability arising out of the application or use of the
# Software or any product or circuit described in the Software. Cypress does
# not authorize its products for use in any products where a malfunction or
# failure of the Cypress product may reasonably be expected to result in
# significant property damage, injury or death ("High Risk Product"). By
# including Cypress's product in a High Risk Product, the manufacturer
# of such system or application assumes all risk of such use and in doing
# so agrees to indemnify Cypress against all liability.
#
TOOLCHAIN = GCC

PLATFORMS_VERSION = 1.0

CONFIG = Debug

CY_EXAMPLE_NAME = BLE_LowPowerSensor

CY_EXAMPLE_DESCRIPTION = This application demonstrates how to build a low power BLE sensor along with various sleep modes

CY_SHOW_NEW_PROJECT := true

CY_VALID_PLATFORMS = CYW920721B2EVK-03

PLATFORM = CYW920721B2EVK-03

CY_APP_DEFINES =   \
  -DWICED_BT_TRACE_ENABLE \
  -DSLEEP_MODE_NO_TRANSPORT=0 \
  -DSLEEP_MODE_TRANSPORT=1 \
  -DSLEEP_TYPE_SDS=1 \
  -DSLEEP_TYPE_PDS=2 \

APP_FEATURES=\
        SLEEP_MODE,app,enum,SLEEP_MODE_NO_TRANSPORT,SLEEP_MODE_NO_TRANSPORT,SLEEP_MODE_TRANSPORT \
        SLEEP_TYPE,app,enum,SLEEP_TYPE_SHUTDOWN,SLEEP_TYPE_SHUTDOWN,SLEEP_TYPE_NOT_SHUTDOWN
SLEEP_MODE ?= SLEEP_MODE_NO_TRANSPORT
SLEEP_TYPE ?= SLEEP_TYPE_SHUTDOWN

ifeq ($(SLEEP_MODE), SLEEP_MODE_NO_TRANSPORT)
CY_APP_DEFINES += -DWICED_SLEEP_MODE=0
else
ifeq ($(SLEEP_MODE), SLEEP_MODE_TRANSPORT)
CY_APP_DEFINES += -DWICED_SLEEP_MODE=1
endif
endif

ifeq ($(SLEEP_TYPE), SLEEP_TYPE_SHUTDOWN)
CY_APP_DEFINES += -DWICED_SLEEP_TYPE=1
else
ifeq ($(SLEEP_TYPE), SLEEP_TYPE_NOT_SHUTDOWN)
CY_APP_DEFINES += -DWICED_SLEEP_TYPE=2
endif
endif

# NOTE: This variable cannot be renamed or moved to a different file. It is updated by the ModusToolbox
# middleware editor.
CY_MAINAPP_SWCOMP_EXT =

ifneq ($(filter CYW920721B2EVK-01 CYW920721B2EVK-03 CYW920719B2Q40EVB-01,$(PLATFORM)),)
endif # PLATFORM


CY_APP_SOURCE =   \
  ./low_power_sensor.c \
  ./low_power_sensor.h \
  ./low_power_sensor_pin_config.c \
  ./wiced_bt_cfg.c \
  ./readme.txt

CY_APP_RESOURCES =

# declare local directory for include path
CY_APP_PATH := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
# handle cygwin drive/path mangling for GNU tools - replace /cygdrive/c/ with c:/
ifneq ($(findstring cygdrive,$(CY_APP_PATH)),)
CY_APP_PATH := $(subst /, ,$(patsubst /cygdrive/%,%,$(CY_APP_PATH)))
CY_APP_PATH := $(subst $(empty) $(empty),/,$(patsubst %,%:,$(firstword $(CY_APP_PATH))) $(wordlist 2,$(words $(CY_APP_PATH)),$(CY_APP_PATH)))
endif

ifndef CYSDK
$(error The SDK must be defined via the CYSDK environment variable)
endif
include $(CYSDK)/libraries/platforms-$(PLATFORMS_VERSION)/common/find_platform.mk
