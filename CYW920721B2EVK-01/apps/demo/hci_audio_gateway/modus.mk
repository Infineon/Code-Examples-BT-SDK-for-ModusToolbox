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

CY_EXAMPLE_NAME = BT_AudioGateway

CY_EXAMPLE_DESCRIPTION = This application demonstrates Bluetooth Audio Gateway profile

CY_SHOW_NEW_PROJECT := true

CY_VALID_PLATFORMS = CYW920721B2EVK-01

PLATFORM = CYW920721B2EVK-01

CY_APP_DEFINES = \
  -DWICED_BT_TRACE_ENABLE

ifeq ($(PLATFORM),CYW920706WCDEVAL)
CY_APP_PATCH_LIBS = wiced_voice_path.a
endif # PLATFORM

ifeq ($(PLATFORM),CYW920721B2EVK-01)
CY_APP_PATCH_LIBS = wiced_sco_lib.a
endif # PLATFORM

# NOTE: This variable cannot be renamed or moved to a different file. It is updated by the ModusToolbox
# middleware editor.
CY_MAINAPP_SWCOMP_EXT =

CY_APP_SOURCE = \
  ./hci_control.c \
  ./hci_control.h \
  ./hci_control_spp.c \
  ./hci_control_ag.c \
  ./hci_control_ag.h \
  ./hci_control_le.c \
  ./hci_control_ag_sdp.c \
  ./hci_control_ag_rfcomm.c \
  ./hci_control_ag_sco.c \
  ./hci_control_ag_command.c \
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
