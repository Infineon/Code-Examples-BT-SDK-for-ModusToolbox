/*******************************************************************************
* File Name: cycfg_pins.c
*
* Description:
* Pin configuration
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

#include "cycfg_pins.h"

#define PAIR_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_15].gpio_pin, \
    .config = GPIO_INPUT_ENABLE | GPIO_PULL_UP, \
    .default_state = GPIO_PIN_OUTPUT_HIGH, \
    .button_pressed_value = GPIO_PIN_OUTPUT_LOW, \
}
#define LED1_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_16].gpio_pin, \
    .config = GPIO_OUTPUT_ENABLE | GPIO_PULL_UP | GPIO_INTERRUPT_ENABLE | GPIO_EN_INT_FALLING_EDGE, \
    .default_state = GPIO_PIN_OUTPUT_HIGH, \
 }
#define LED2_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_17].gpio_pin, \
    .config = GPIO_OUTPUT_ENABLE | GPIO_PULL_UP, \
    .default_state = GPIO_PIN_OUTPUT_HIGH, \
 }
#define LED3_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_18].gpio_pin, \
    .config = GPIO_OUTPUT_ENABLE | GPIO_PULL_UP, \
    .default_state = GPIO_PIN_OUTPUT_HIGH, \
 }
#define LED4_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_19].gpio_pin, \
    .config = GPIO_OUTPUT_ENABLE | GPIO_PULL_UP, \
    .default_state = GPIO_PIN_OUTPUT_HIGH, \
 }
#define ioss_0_pin_37_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_21].gpio_pin, \
    .config = GPIO_OUTPUT_ENABLE | GPIO_PULL_DOWN, \
    .default_state = GPIO_PIN_OUTPUT_LOW, \
 }

const wiced_platform_gpio_t platform_gpio_pins[] =
{
	[PLATFORM_GPIO_0] = {WICED_P00, keyscan_0_ksi_0_TRIGGER_IN},
	[PLATFORM_GPIO_1] = {WICED_P01, keyscan_0_ksi_1_TRIGGER_IN},
	[PLATFORM_GPIO_2] = {WICED_P02, keyscan_0_ksi_2_TRIGGER_IN},
	[PLATFORM_GPIO_3] = {WICED_P03, keyscan_0_ksi_3_TRIGGER_IN},
	[PLATFORM_GPIO_4] = {WICED_P04, keyscan_0_ksi_4_TRIGGER_IN},
	[PLATFORM_GPIO_5] = {WICED_P05, keyscan_0_ksi_5_TRIGGER_IN},
	[PLATFORM_GPIO_6] = {WICED_P06, keyscan_0_ksi_6_TRIGGER_IN},
	[PLATFORM_GPIO_7] = {WICED_P08, keyscan_0_kso_0_TRIGGER_IN},
	[PLATFORM_GPIO_8] = {WICED_P09, keyscan_0_kso_1_TRIGGER_IN},
	[PLATFORM_GPIO_9] = {WICED_P10, keyscan_0_kso_2_TRIGGER_IN},
	[PLATFORM_GPIO_10] = {WICED_P11, keyscan_0_kso_3_TRIGGER_IN},
	[PLATFORM_GPIO_11] = {WICED_P12, keyscan_0_kso_4_TRIGGER_IN},
	[PLATFORM_GPIO_12] = {WICED_P13, keyscan_0_kso_5_TRIGGER_IN},
	[PLATFORM_GPIO_13] = {WICED_P14, keyscan_0_kso_6_TRIGGER_IN},
	[PLATFORM_GPIO_14] = {WICED_P15, keyscan_0_kso_7_TRIGGER_IN},
	[PLATFORM_GPIO_15] = {WICED_P17, WICED_GPIO},
	[PLATFORM_GPIO_16] = {WICED_P26, WICED_GPIO},
	[PLATFORM_GPIO_17] = {WICED_P27, WICED_GPIO},
	[PLATFORM_GPIO_18] = {WICED_P28, WICED_GPIO},
	[PLATFORM_GPIO_19] = {WICED_P29, WICED_GPIO},
	[PLATFORM_GPIO_20] = {WICED_P32, uart_1_txd_0_TRIGGER_IN},
	[PLATFORM_GPIO_21] = {WICED_P37, WICED_GPIO},
};
const wiced_platform_led_config_t platform_led[] =
{
	[WICED_PLATFORM_LED_1] = LED1_config,
	[WICED_PLATFORM_LED_2] = LED2_config,
	[WICED_PLATFORM_LED_3] = LED3_config,
	[WICED_PLATFORM_LED_1] = LED4_config,
};
const size_t led_count = (sizeof(platform_led) / sizeof(wiced_platform_led_config_t));
const wiced_platform_button_config_t platform_button[] =
{
	[WICED_PLATFORM_BUTTON_1] = PAIR_config,
};
const size_t button_count = (sizeof(platform_button) / sizeof(wiced_platform_button_config_t));
const wiced_platform_gpio_config_t platform_gpio[] =
{
	[WICED_PLATFORM_GPIO_1] = ioss_0_pin_37_config,
};
const size_t gpio_count = (sizeof(platform_gpio) / sizeof(wiced_platform_gpio_config_t));

