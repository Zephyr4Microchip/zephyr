/*
 * Copyright (c) 2019 Piotr Mienkowski
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TEST_GPIO_API_H_
#define TEST_GPIO_API_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/ztest.h>

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

const struct gpio_dt_spec output_gpio = GPIO_DT_SPEC_GET_OR(ZEPHYR_USER_NODE, output_gpios, {0});
const struct gpio_dt_spec input_gpio = GPIO_DT_SPEC_GET_OR(ZEPHYR_USER_NODE, input_gpios, {0});

#define TEST_NODE    DT_GPIO_CTLR(ZEPHYR_USER_NODE, input_gpios)
#define TEST_PIN     DT_GPIO_PIN(ZEPHYR_USER_NODE, input_gpios)
#define TEST_OUT_PIN DT_GPIO_PIN(ZEPHYR_USER_NODE, output_gpios)

#define TEST_GPIO_MAX_RISE_FALL_TIME_US 200

void test_gpio_int_edge_rising(void);
void test_gpio_int_edge_falling(void);
void test_gpio_int_edge_both(void);
void test_gpio_int_level_high_interrupt_count_1(void);
void test_gpio_int_level_high_interrupt_count_5(void);
void test_gpio_int_level_active(void);

#endif /* TEST_GPIO_API_H_ */
