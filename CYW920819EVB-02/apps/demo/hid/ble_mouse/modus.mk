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

CY_EXAMPLE_NAME = BLE_HID_Mouse

CY_EXAMPLE_DESCRIPTION = This application demonstrates a HID BLE mouse

CY_SHOW_NEW_PROJECT := true

# Note: TESTING_USING_HCI only enabled for testing via HCI UART using ClientControl tool.
# It is only for the case that the WICED BOARD you get doesn't have any GPIO pins
# exposed, you can't use any fly wire to connect to GPIOs to simulate button press
# and button release to test out the application. ClientControl tool can help you
# simulate button press and release by sending report via HCI to the WICED BOARD
#
# Use OTA_FW_UPGRADE=1 to enable Over-the-air firmware upgrade functionality
# Use OTA_SEC_FW_UPGRADE=1 in the make target to use secure OTA procedure.
# See instructions in ota_firmware_upgrade.c file how to use secure upgrade.
# Use ASSYMETRIC_SLAVE_LATENCY=1 if master won't accept slave connection parameter update
# request.
# Use ENABLE_SCROLL=1 to enable scroll wheel
# Use ENABLE_MOTION=1 to enable motion

TESTING_USING_HCI_DEFAULT=0
OTA_FW_UPGRADE_DEFAULT=1
OTA_SEC_FW_UPGRADE_DEFAULT=0
ENABLE_SCROLL_DEFAULT=1
ENABLE_MOTION_DEFAULT=1
ASSYMETRIC_SLAVE_LATENCY_DEFAULT=0

TESTING_USING_HCI ?= $(TESTING_USING_HCI_DEFAULT)
OTA_FW_UPGRADE ?= $(OTA_FW_UPGRADE_DEFAULT)
OTA_SEC_FW_UPGRADE ?= $(OTA_SEC_FW_UPGRADE_DEFAULT)
ENABLE_SCROLL ?= $(ENABLE_SCROLL_DEFAULT)
ENABLE_MOTION ?= $(ENABLE_MOTION_DEFAULT)
ASSYMETRIC_SLAVE_LATENCY ?= $(ASSYMETRIC_SLAVE_LATENCY_DEFAULT)

# declare the feature for GUI settings processing
APP_FEATURES= \
    TESTING_USING_HCI,app,enum,$(TESTING_USING_HCI_DEFAULT),0,1 \
    OTA_FW_UPGRADE,app,enum,$(OTA_FW_UPGRADE_DEFAULT),0,1 \
    OTA_SEC_FW_UPGRADE,app,enum,$(OTA_SEC_FW_UPGRADE_DEFAULT),0,1 \
    ENABLE_SCROLL,app,enum,$(ENABLE_SCROLL_DEFAULT),0,1 \
    ENABLE_MOTION,app,enum,$(ENABLE_MOTION_DEFAULT),0,1 \
    ASSYMETRIC_SLAVE_LATENCY,app,enum,$(ASSYMETRIC_SLAVE_LATENCY_DEFAULT),0,1

CY_VALID_PLATFORMS = CYW920819EVB-02

PLATFORM = CYW920819EVB-02

CY_APP_DEFINES = \
  -DWICED_BT_TRACE_ENABLE \
  -DLE_HIDD_ONLY

ifeq ($(TESTING_USING_HCI),1)
CY_APP_DEFINES += -DTESTING_USING_HCI
endif
ifeq ($(ENABLE_SCROLL),1)
CY_APP_DEFINES += -DSUPPORT_SCROLL
endif
ifeq ($(ENABLE_MOTION),1)
CY_APP_DEFINES += -DSUPPORT_MOTION
endif
ifeq ($(ASSYMETRIC_SLAVE_LATENCY),1)
CY_APP_DEFINES += -DASSYM_SLAVE_LATENCY
endif

# NOTE: This variable cannot be renamed or moved to a different file. It is updated by the ModusToolbox
# middleware editor.
CY_MAINAPP_SWCOMP_USED = \
  $(CY_WICED_LIB_COMP_BASE)/BT-SDK/common/libraries/hidd_lib

#enable OTA Firmware Update
ifeq ($(OTA_FW_UPGRADE),1)
CY_APP_DEFINES += -DOTA_FIRMWARE_UPGRADE
CY_APP_DEFINES += -DOTA_SKIP_CONN_PARAM_UPDATE
CY_APP_DEFINES += -DSFLASH_SIZE_2M_BITS

CY_MAINAPP_SWCOMP_USED += \
  $(CY_WICED_LIB_COMP_BASE)/BT-SDK/common/libraries/fw_upgrade_lib

endif # OTA_FW_UPGRADE

# NOTE: This variable cannot be renamed or moved to a different file. It is updated by the ModusToolbox
# middleware editor.
CY_MAINAPP_SWCOMP_EXT =

CY_APP_SOURCE = \
  ./appinit_ble_mouse.c \
  ./ble_mouse.c \
  ./ble_mouse.h \
  ./ble_mouse_gatts.c \
  ./ble_mouse_gatts.h \
  ./wiced_bt_cfg.c \
  ./readme.txt \
  ./secure/ecdsa256_pub.c \
  ./motion/interrupt.c \
  ./motion/interrupt.h \
  ./motion/PAW3805_opticalsensor.c \
  ./motion/PAW3805_opticalsensor.h

ifeq ($(OTA_SEC_FW_UPGRADE),1)
ifneq ($(OTA_FW_UPGRADE),1)
$(error setting OTA_SEC_FW_UPGRADE=1 requires OTA_FW_UPGRADE also set to 1)
else
CY_APP_DEFINES += -DOTA_SECURE_FIRMWARE_UPGRADE
endif # ifneq OTA_FW_UPGRADE
endif # OTA_SEC_FW_UPGRADE

CY_APP_RESOURCES =

CY_APP_PATCH_LIBS += wiced_hidd_lib.a

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
