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

CY_EXAMPLE_NAME = Watch

CY_EXAMPLE_DESCRIPTION = This reference application demonstrates streaming audio source and remote control.

CY_SHOW_NEW_PROJECT := true

CY_VALID_PLATFORMS = CYW920819EVB-02

PLATFORM = CYW920819EVB-02

ifeq ($(PLATFORM),$(filter $(PLATFORM), CYW920706WCDEVAL CYW920719Q40EVB-01 ))
SLEEP_SUPPORT_DEFAULT = 1
COEX_SUPPORT_DEFAULT = 1
else
SLEEP_SUPPORT_DEFAULT = 0
COEX_SUPPORT_DEFAULT = 0
endif

APP_FEATURES += SLEEP_SUPPORT,app,enum,$(SLEEP_SUPPORT_DEFAULT),0,1
APP_FEATURES += COEX_SUPPORT,app,enum,$(COEX_SUPPORT_DEFAULT),0,1
SLEEP_SUPPORT ?= $(SLEEP_SUPPORT_DEFAULT)
COEX_SUPPORT ?= $(COEX_SUPPORT_DEFAULT)

CY_APP_DEFINES =   \
  -DWICED_BT_TRACE_ENABLE \
  -DCATEGORY_2_PASSTROUGH \
  -DWICED_HCI_TRANSPORT_UART=1 \
  -DWICED_HCI_TRANSPORT_SPI=2

#CY_APP_DEFINES += -DPTS_TEST_ONLY

ifeq ($(TRANSPORT),SPI)
#$(info Transport=SPI)

ifeq ($(PLATFORM),$(filter $(PLATFORM), CYW920706WCDEVAL CYW920719Q40EVB-01 ))
CY_APP_DEFINES += -DWICED_HCI_TRANSPORT=2
else
$(error invalid platform for SPI transport, supported only for CYW920706WCDEVAL and CYW920719Q40EVB-01)
endif

else
#$(info Transport=UART)
CY_APP_DEFINES += -DWICED_HCI_TRANSPORT=1
endif

ifeq ($(PLATFORM),CYW920706WCDEVAL)
CY_APP_PATCH_LIBS = wiced_audio_source.a
ifeq ($(TRANSPORT),SPI)
CY_APP_PATCH_LIBS += wiced_hal_lib.a
endif
endif # PLATFORM

ifeq ($(PLATFORM),CYW920719Q40EVB-01)
CY_APP_PATCH_LIBS = wiced_audio_source_lib.a
ifeq ($(TRANSPORT),SPI)
CY_APP_PATCH_LIBS += wiced_transport_spi_lib.a
endif
endif # PLATFORM


ifeq ($(PLATFORM),CYW943012EVB-04-BT)
CY_APP_DEFINES += -DWICED_HCI_BAUDRATE=115200
endif # PLATFORM

CY_APP_DEFINES +=  \
  -DSLEEP_SUPPORTED=$(SLEEP_SUPPORT) \
  -DCOEX_SUPPORTED=$(COEX_SUPPORT)

# NOTE: This variable cannot be renamed or moved to a different file. It is updated by the ModusToolbox
# middleware editor.
CY_MAINAPP_SWCOMP_EXT =

ifneq ($(filter 20819 CYBT-213043-MESH CYW920819EVB-02,$(PLATFORM)),)
CY_APP_PATCH_LIBS += wiced_ble_pre_init_lib.a
endif

CY_MAINAPP_SWCOMP_USED =  \
  $(CY_WICED_LIB_COMP_BASE)/BT-SDK/common/libraries/avrc_controller \
  $(CY_WICED_LIB_COMP_BASE)/BT-SDK/common/libraries/avrc_target

CY_APP_SOURCE = \
  ./wiced_app_cfg.c \
  ./wiced_app_cfg.h \
  ./wiced_app.c \
  ./wiced_app.h \
  ./watch.c \
  ./hci_control.h \
  ./hci_control_le.c \
  ./hci_control_le.h \
  ./hci_control_audio.c \
  ./hci_control_audio.h \
  ./hci_control_rc_target.c \
  ./hci_control_rc_target.h \
  ./hci_control_test.c \
  ./hci_control_test.h \
  ./hci_control_misc.c \
  ./hci_control_misc.h \
  ./hci_control_rc_controller.c \
  ./hci_control_rc_controller.h \
  ./le_slave.c \
  ./le_slave.h \
  ./ams_client.c \
  ./ams_client.h \
  ./ancs_client.c \
  ./ancs_client.h \
  ./readme.txt \
  ./GeneratedSource/cycfg_bt.h \
  ./GeneratedSource/cycfg_gatt_db.c \
  ./GeneratedSource/cycfg_gatt_db.h

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
