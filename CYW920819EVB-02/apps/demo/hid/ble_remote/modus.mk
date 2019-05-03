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

CY_EXAMPLE_NAME = BLE_HID_Remote

CY_EXAMPLE_DESCRIPTION = This application demonstrates a HID BLE remote

CY_SHOW_NEW_PROJECT := true

# Note: TESTING_USING_HCI only enabled for testing via HCI UART using ClientControl tool.
# It is only for the case that the WICED BOARD you get doesn't have any GPIO pins
# exposed, you can't use any fly wire to connect to GPIOs to simulate key press
# and key release to test out the application. ClientControl tool can help you
# simulate key press and release by sending report via HCI to the WICED BOARD
#
# Use OTA_FW_UPGRADE=1 to enable Over-the-air firmware upgrade functionality
# Use OTA_SEC_FW_UPGRADE=1 in the make target to use secure OTA procedure.
# See instructions in ota_firmware_upgrade.c file how to use secure upgrade.
# Use ASSYMETRIC_SLAVE_LATENCY=1 if master won't accept slave connection parameter update
# request.
# Use ENABLE_TOUCHPAD=1 to enable touchpad support
# Use ENABLE_AUDIO=1 to enable audio microphone
# Use ENABLE_DIGITAL_MIC=1 to enable digital microphone
# Use ENABLE_IR=1 to enable IR support
# Use ENABLE_MOTION=1 to enable motion
# Use OPUS_CELT_ENCODER=1 to use OPUS CELT encoder (else default is mSBC)
# Use AUTO_RECONNECT=1 to automatically reconnect when connection drops
# Use SKIP_PARAM_UPDATE=1 to not request connection parameter update immediately when
# received LE conn param update complete event with non-preferred values
# Use ENABLE_EASY_PAIR=1 to enable Easy Pair feature
# Use START_ADV_ON_POWERUP=1 to start advertising on power up if not connected
# Use ENABLE_CONNECTED_ADV=1 to enable advertising while connected
# Use DISCONNECTED_ENDLESS_ADV=1 to enable endless advertising while disconnected and bonded
# Use DISCOVERABLE_SDS=1 to allow SDS while in discoverable mode
# Use ENABLE_FINDME=1 to enable Find Me profile support
# Use POLL_MOTION_WHILE_CONNECTED=1 to use regular transport poll after receiving initial interrup
# to get motion data until motion idle, rather than receiving further motion data from interrupts.
# Use ENABLE_MOTION_AS_AIR_MOUSE=1 to convert motion data to mouse data, otherwise send raw data

# Initialize defaults:
TESTING_USING_HCI_DEFAULT=0
OTA_FW_UPGRADE_DEFAULT=1
OTA_SEC_FW_UPGRADE_DEFAULT=0
ENABLE_TOUCHPAD_DEFAULT=0
ENABLE_AUDIO_DEFAULT=1
ENABLE_DIGITAL_MIC_DEFAULT=0
ENABLE_IR_DEFAULT=0
ENABLE_MOTION_DEFAULT=0
OPUS_CELT_ENCODER_DEFAULT=0
AUTO_RECONNECT_DEFAULT=0
SKIP_PARAM_UPDATE_DEFAULT=1
ENABLE_EASY_PAIR_DEFAULT=0
START_ADV_ON_POWERUP_DEFAULT=0
ENABLE_CONNECTED_ADV_DEFAULT=0
DISCONNECTED_ENDLESS_ADV_DEFAULT=0
DISCOVERABLE_SDS_DEFAULT=0
ENABLE_FINDME_DEFAULT=0
POLL_MOTION_WHILE_CONNECTED_DEFAULT=1
ENABLE_MOTION_AS_AIR_MOUSE_DEFAULT=0
ASSYMETRIC_SLAVE_LATENCY_DEFAULT=0

TESTING_USING_HCI ?= $(TESTING_USING_HCI_DEFAULT)
OTA_FW_UPGRADE ?= $(OTA_FW_UPGRADE_DEFAULT)
OTA_SEC_FW_UPGRADE ?= $(OTA_SEC_FW_UPGRADE_DEFAULT)
ENABLE_TOUCHPAD ?= $(ENABLE_TOUCHPAD_DEFAULT)
ENABLE_AUDIO ?= $(ENABLE_AUDIO_DEFAULT)
ENABLE_DIGITAL_MIC ?= $(ENABLE_DIGITAL_MIC_DEFAULT)
ENABLE_IR ?= $(ENABLE_IR_DEFAULT)
ENABLE_MOTION ?= $(ENABLE_MOTION_DEFAULT)
OPUS_CELT_ENCODER ?= $(OPUS_CELT_ENCODER_DEFAULT)
AUTO_RECONNECT ?= $(AUTO_RECONNECT_DEFAULT)
SKIP_PARAM_UPDATE ?= $(SKIP_PARAM_UPDATE_DEFAULT)
ENABLE_EASY_PAIR ?= $(ENABLE_EASY_PAIR_DEFAULT)
START_ADV_ON_POWERUP ?= $(START_ADV_ON_POWERUP_DEFAULT)
ENABLE_CONNECTED_ADV ?= $(ENABLE_CONNECTED_ADV_DEFAULT)
DISCONNECTED_ENDLESS_ADV ?= $(DISCONNECTED_ENDLESS_ADV_DEFAULT)
DISCOVERABLE_SDS ?= $(DISCOVERABLE_SDS_DEFAULT)
ENABLE_FINDME ?= $(ENABLE_FINDME_DEFAULT)
POLL_MOTION_WHILE_CONNECTED ?= $(POLL_MOTION_WHILE_CONNECTED_DEFAULT)
ENABLE_MOTION_AS_AIR_MOUSE ?= $(ENABLE_MOTION_AS_AIR_MOUSE_DEFAULT)
ASSYMETRIC_SLAVE_LATENCY ?= $(ASSYMETRIC_SLAVE_LATENCY_DEFAULT)

# declare the feature for GUI settings processing
APP_FEATURES= \
    TESTING_USING_HCI,app,enum,$(TESTING_USING_HCI_DEFAULT),0,1 \
    OTA_FW_UPGRADE,app,enum,$(OTA_FW_UPGRADE_DEFAULT),0,1 \
    OTA_SEC_FW_UPGRADE,app,enum,$(OTA_SEC_FW_UPGRADE_DEFAULT),0,1 \
    ENABLE_TOUCHPAD,app,enum,$(ENABLE_TOUCHPAD_DEFAULT),0,1 \
    ENABLE_AUDIO,app,enum,$(ENABLE_AUDIO_DEFAULT),0,1 \
    ENABLE_DIGITAL_MIC,app,enum,$(ENABLE_DIGITAL_MIC_DEFAULT),0,1 \
    ENABLE_IR,app,enum,$(ENABLE_IR_DEFAULT),0,1 \
    ENABLE_MOTION,app,enum,$(ENABLE_MOTION_DEFAULT),0,1 \
    OPUS_CELT_ENCODER,app,enum,$(OPUS_CELT_ENCODER_DEFAULT),0,1 \
    AUTO_RECONNECT,app,enum,$(AUTO_RECONNECT_DEFAULT),0,1 \
    SKIP_PARAM_UPDATE,app,enum,$(SKIP_PARAM_UPDATE_DEFAULT),0,1 \
    ENABLE_EASY_PAIR,app,enum,$(ENABLE_EASY_PAIR_DEFAULT),0,1 \
    START_ADV_ON_POWERUP,app,enum,$(START_ADV_ON_POWERUP_DEFAULT),0,1 \
    ENABLE_CONNECTED_ADV,app,enum,$(ENABLE_CONNECTED_ADV_DEFAULT),0,1 \
    DISCONNECTED_ENDLESS_ADV,app,enum,$(DISCONNECTED_ENDLESS_ADV_DEFAULT),0,1 \
    DISCOVERABLE_SDS,app,enum,$(DISCOVERABLE_SDS_DEFAULT),0,1 \
    ENABLE_FINDME,app,enum,$(ENABLE_FINDME_DEFAULT),0,1 \
    POLL_MOTION_WHILE_CONNECTED,app,enum,$(POLL_MOTION_WHILE_CONNECTED_DEFAULT),0,1 \
    ENABLE_MOTION_AS_AIR_MOUSE,app,enum,$(ENABLE_MOTION_AS_AIR_MOUSE_DEFAULT),0,1 \
    ASSYMETRIC_SLAVE_LATENCY,app,enum,$(ASSYMETRIC_SLAVE_LATENCY_DEFAULT),0,1

CY_VALID_PLATFORMS = CYW920819EVB-02

PLATFORM = CYW920819EVB-02

CY_APP_DEFINES = \
  -DWICED_BT_TRACE_ENABLE \
  -DLE_HIDD_ONLY

CY_APP_PATCH_LIBS += wiced_hidd_lib.a

ifeq ($(TESTING_USING_HCI),1)
CY_APP_DEFINES += -DTESTING_USING_HCI
endif
ifeq ($(ENABLE_SCROLL),1)
CY_APP_DEFINES += -DSUPPORT_SCROLL
endif
ifeq ($(ASSYMETRIC_SLAVE_LATENCY),1)
CY_APP_DEFINES += -DASSYM_SLAVE_LATENCY
endif
ifeq ($(OPUS_CELT_ENCODER), 1)
 #use OPUS CELT encoder
 CY_APP_DEFINES += -DCELT_ENCODER
 ifeq (A_20819A1,$(BLD))
  CY_APP_PATCH_LIBS += celt_lib.a
 endif
else
 #use mSBC encoder
 CY_APP_DEFINES += -DSBC_ENCODER
 CY_APP_DEFINES += -DSFLASH_SIZE_2M_BITS
endif
ifeq ($(SKIP_PARAM_UPDATE),1)
CY_APP_DEFINES += -DSKIP_CONNECT_PARAM_UPDATE_EVEN_IF_NO_PREFERED
endif
ifeq ($(AUTO_RECONNECT),1)
CY_APP_DEFINES += -DAUTO_RECONNECT
endif
ifeq ($(ENABLE_AUDIO),1)
 CY_APP_DEFINES += -DSUPPORT_AUDIO
 # send audio data as 1 big gatt packet
 CY_APP_DEFINES += -DATT_MTU_SIZE_180
 #enabled audio enhancement
 CY_APP_DEFINES += -DENABLE_ADC_AUDIO_ENHANCEMENTS
 ifeq ($(PLATFORM),CYW920819EVB-02)
  CY_APP_PATCH_LIBS += adc_audio_lib.a
 endif
endif # ENABLE_AUDIO
ifeq ($(ENABLE_EASY_PAIR),1)
CY_APP_DEFINES += -DEASY_PAIR
endif
ifeq ($(START_ADV_ON_POWERUP),1)
CY_APP_DEFINES += -DSTART_ADV_WHEN_POWERUP_NO_CONNECTED
endif
ifeq ($(ENABLE_CONNECTED_ADV),1)
CY_APP_DEFINES += -DCONNECTED_ADVERTISING_SUPPORTED
endif
ifeq ($(DISCONNECTED_ENDLESS_ADV),1)
CY_APP_DEFINES += -DENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED
endif
ifeq ($(DISCOVERABLE_SDS),1)
CY_APP_DEFINES += -DALLOW_SDS_IN_DISCOVERABLE
endif
ifeq ($(ENABLE_FINDME),1)
CY_APP_DEFINES += -DSUPPORTING_FINDME
endif
ifeq ($(ENABLE_DIGITAL_MIC),1)
CY_APP_DEFINES += -DSUPPORT_DIGITAL_MIC
endif

# NOTE: This variable cannot be renamed or moved to a different file. It is updated by the ModusToolbox
# middleware editor.
CY_MAINAPP_SWCOMP_USED = \
  $(CY_WICED_LIB_COMP_BASE)/BT-SDK/common/libraries/hidd_lib

#enable OTA Firmware Update
ifeq ($(OTA_FW_UPGRADE),1)
CY_APP_DEFINES += -DOTA_FIRMWARE_UPGRADE
CY_APP_DEFINES += -DOTA_SKIP_CONN_PARAM_UPDATE

CY_MAINAPP_SWCOMP_USED += \
  $(CY_WICED_LIB_COMP_BASE)/BT-SDK/common/libraries/fw_upgrade_lib

endif # OTA_FW_UPGRADE

# NOTE: This variable cannot be renamed or moved to a different file. It is updated by the ModusToolbox
# middleware editor.
CY_MAINAPP_SWCOMP_EXT =

CY_APP_SOURCE = \
  ./appDefs.h \
  ./appinit_ble_remote.c \
  ./ble_remote.c \
  ./ble_remote.h \
  ./ble_remote_gatts.c \
  ./ble_remote_gatts.h \
  ./i2cDevice.c \
  ./i2cDevice.h \
  ./wiced_bt_cfg.c \
  ./readme.txt \
  ./ir/bleapp_appirtx.c \
  ./ir/bleapp_appirtx.h \
  ./interrupt.c \
  ./interrupt.h \
  ./motion/motion_icm.c \
  ./motion/motion_icm.h \
  ./motion/sensor_icm20608.c \
  ./motion/sensor_icm20608.h \
  ./motion/obj_secret/AIR_MOTION_Lib.h \
  ./motion/obj_secret/AIR_MOTION_Lib.o \
  ./motion/obj_secret/inven_stdbool.h \
  ./motion/obj_secret/RollCompFullDynamic.o \
  ./secure/ecdsa256_pub.c \
  ./touchpad/AzoteqIQS572.h \
  ./touchpad/IQS5xx.c \
  ./touchpad/IQS5xx.h \
  ./touchpad/touchPad.c \
  ./touchpad/touchPad.h

ifeq ($(OTA_SEC_FW_UPGRADE),1)
ifneq ($(OTA_FW_UPGRADE),1)
$(error setting OTA_SEC_FW_UPGRADE=1 requires OTA_FW_UPGRADE also set to 1)
else
CY_APP_DEFINES += -DOTA_SECURE_FIRMWARE_UPGRADE
endif # ifneq OTA_FW_UPGRADE
endif # OTA_SEC_FW_UPGRADE

ifeq ($(ENABLE_IR),1)
CY_APP_DEFINES += -DSUPPORT_IR
endif

ifeq ($(ENABLE_MOTION),1)
CY_APP_DEFINES += -DSUPPORT_MOTION
ifeq ($(POLL_MOTION_WHILE_CONNECTED),1)
CY_APP_DEFINES += -DPOLL_MOTION_WHILE_CONNECTED_AND_ACTIVE
endif
ifeq ($(ENABLE_MOTION_AS_AIR_MOUSE),1)
CY_APP_DEFINES += -DUSE_MOTION_AS_AIR_MOUSE
CY_APP_PATCH_LIBS += air_motion.a
endif
endif # ENABLE_MOTION

ifeq ($(ENABLE_TOUCHPAD),1)
CY_APP_DEFINES += -DSUPPORT_TOUCHPAD
endif

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
