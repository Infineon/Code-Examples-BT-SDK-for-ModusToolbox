/*******************************************************************************
* File Name: cycfg_routing.h
*
* Description:
* Establishes all necessary connections between hardware elements.
* This file was automatically generated and should not be modified.
*
********************************************************************************
* Copyright 2017-2019 Cypress Semiconductor Corporation
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
********************************************************************************/

#if !defined(CYCFG_ROUTING_H)
#define CYCFG_ROUTING_H

#if defined(__cplusplus)
extern "C" {
#endif

#include "cycfg_notices.h"
void init_cycfg_routing(void);
#define init_cycfg_connectivity() init_cycfg_routing()
#define keyscan_0_ksi_0_TRIGGER_IN WICED_GPIO
#define keyscan_0_ksi_1_TRIGGER_IN WICED_GPIO
#define keyscan_0_ksi_2_TRIGGER_IN WICED_GPIO
#define keyscan_0_ksi_3_TRIGGER_IN WICED_GPIO
#define keyscan_0_ksi_4_TRIGGER_IN WICED_GPIO
#define keyscan_0_ksi_5_TRIGGER_IN WICED_GPIO
#define keyscan_0_ksi_6_TRIGGER_IN WICED_GPIO

#define keyscan_0_kso_0_TRIGGER_IN WICED_KSO0
#define keyscan_0_kso_1_TRIGGER_IN WICED_KSO1
#define keyscan_0_kso_2_TRIGGER_IN WICED_KSO2
#define keyscan_0_kso_3_TRIGGER_IN WICED_KSO3
#define keyscan_0_kso_4_TRIGGER_IN WICED_KSO4
#define keyscan_0_kso_5_TRIGGER_IN WICED_KSO5
#define keyscan_0_kso_6_TRIGGER_IN WICED_KSO6
#define keyscan_0_kso_7_TRIGGER_IN WICED_KSO7
#define uart_1_txd_0_TRIGGER_IN WICED_UART_2_TXD

#if defined(__cplusplus)
}
#endif


#endif /* CYCFG_ROUTING_H */
