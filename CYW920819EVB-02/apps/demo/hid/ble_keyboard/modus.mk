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

CY_EXAMPLE_NAME = BLE_HID_Keyboard

CY_EXAMPLE_DESCRIPTION = This application demonstrates a HID BLE keyboard controller

CY_SHOW_NEW_PROJECT := true

# Note: TESTING_USING_HCI only enabled for testing via HCI UART using ClientControl tool.
# It is only for the case that the WICED BOARD you get doesn't have any GPIO pins
# exposed, you can't use any fly wire to connect to GPIOs to simulate key press
# and key release to test out the application. ClientControl tool can help you
# simulate key press and release by sending key report via HCI to the WICED BOARD
#
# Use OTA_FW_UPGRADE=1 to enable Over-the-air firmware upgrade functionality
# Use OTA_SEC_FW_UPGRADE=1 in the make target to use secure OTA procedure.
# See instructions in ota_firmware_upgrade.c file how to use secure upgrade.
# Use ASSYMETRIC_SLAVE_LATENCY=1 if master won't accept slave connection parameter update
# request.
# Use AUTO_RECONNECT=1 to automatically reconnect when connection drops
# Use LE_LOCAL_PRIVACY=1 to advertise with Resolvable Private Address (RPA)
# Use SKIP_PARAM_UPDATE=1 to not request connection parameter update immediately when
# received LE conn param update complete event with non-preferred values
#
# defaults if not overridden on the command line:
TESTING_USING_HCI_DEFAULT=0
OTA_FW_UPGRADE_DEFAULT=1
OTA_SEC_FW_UPGRADE_DEFAULT=0
ASSYMETRIC_SLAVE_LATENCY_DEFAULT=0
LE_LOCAL_PRIVACY_DEFAULT=0
SKIP_PARAM_UPDATE_DEFAULT=1
AUTO_RECONNECT_DEFAULT=0

TESTING_USING_HCI ?= $(TESTING_USING_HCI_DEFAULT)
OTA_FW_UPGRADE ?= $(OTA_FW_UPGRADE_DEFAULT)
OTA_SEC_FW_UPGRADE ?= $(OTA_SEC_FW_UPGRADE_DEFAULT)
ASSYMETRIC_SLAVE_LATENCY ?= $(ASSYMETRIC_SLAVE_LATENCY_DEFAULT)
LE_LOCAL_PRIVACY ?= $(LE_LOCAL_PRIVACY_DEFAULT)
SKIP_PARAM_UPDATE ?= $(SKIP_PARAM_UPDATE_DEFAULT)
AUTO_RECONNECT ?= $(AUTO_RECONNECT_DEFAULT)

# declare the feature for GUI settings processing
APP_FEATURES= \
    TESTING_USING_HCI,app,enum,$(TESTING_USING_HCI_DEFAULT),0,1 \
    OTA_FW_UPGRADE,app,enum,$(OTA_FW_UPGRADE_DEFAULT),0,1 \
    OTA_SEC_FW_UPGRADE,app,enum,$(OTA_SEC_FW_UPGRADE_DEFAULT),0,1 \
    ASSYMETRIC_SLAVE_LATENCY,app,enum,$(ASSYMETRIC_SLAVE_LATENCY_DEFAULT),0,1 \
    LE_LOCAL_PRIVACY,app,enum,$(LE_LOCAL_PRIVACY_DEFAULT),0,1 \
    SKIP_PARAM_UPDATE,app,enum,$(SKIP_PARAM_UPDATE_DEFAULT),0,1 \
    AUTO_RECONNECT,app,enum,$(AUTO_RECONNECT_DEFAULT),0,1

CY_VALID_PLATFORMS = CYW920819EVB-02

PLATFORM = CYW920819EVB-02

CY_APP_DEFINES = \
  -DWICED_BT_TRACE_ENABLE \
  -DLE_HIDD_ONLY

ifeq ($(TESTING_USING_HCI),1)
CY_APP_DEFINES += -DTESTING_USING_HCI
endif
ifeq ($(ASSYMETRIC_SLAVE_LATENCY),1)
CY_APP_DEFINES += -DASSYM_SLAVE_LATENCY
endif
ifeq ($(LE_LOCAL_PRIVACY),1)
CY_APP_DEFINES += -DLE_LOCAL_PRIVACY_SUPPORT
endif
ifeq ($(SKIP_PARAM_UPDATE),1)
CY_APP_DEFINES += -DSKIP_CONNECT_PARAM_UPDATE_EVEN_IF_NO_PREFERED
endif
ifeq ($(AUTO_RECONNECT),1)
CY_APP_DEFINES += -DAUTO_RECONNECT
endif

# NOTE: This variable cannot be renamed or moved to a different file. It is updated by the ModusToolbox
# middleware editor.
CY_MAINAPP_SWCOMP_USED = \
  $(CY_WICED_LIB_COMP_BASE)/BT-SDK/common/libraries/hidd_lib

#enable OTA Firmware Update
ifeq ($(OTA_FW_UPGRADE),1)
CY_APP_DEFINES += -DOTA_FIRMWARE_UPGRADE
CY_APP_DEFINES += -DDISABLED_SLAVE_LATENCY_ONLY
CY_APP_DEFINES += -DOTA_SKIP_CONN_PARAM_UPDATE

CY_MAINAPP_SWCOMP_USED += \
  $(CY_WICED_LIB_COMP_BASE)/BT-SDK/common/libraries/fw_upgrade_lib

endif # OTA_FW_UPGRADE

# NOTE: This variable cannot be renamed or moved to a different file. It is updated by the ModusToolbox
# middleware editor.
CY_MAINAPP_SWCOMP_EXT =

CY_APP_SOURCE = \
  ./appinit_ble_keyboard.c \
  ./ble_keyboard.c \
  ./ble_keyboard.h \
  ./ble_keyboard_gatts.c \
  ./ble_keyboard_gatts.h \
  ./wiced_bt_cfg.c \
  ./readme.txt \
  ./wiced_platform.h \
  ./design.modus \
  ./GeneratedSource/cycfg_routing.h \
  ./GeneratedSource/cycfg_pins.c \
  ./GeneratedSource/cycfg_pins.h \
  ./GeneratedSource/cycfg_notices.h \
  ./secure/ecdsa256_pub.c

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
