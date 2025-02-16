/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file pinctrl_mchp_v1.c
 * @brief Pin control driver for Microchip devices.
 *
 * This file provides the implementation of pin control functions
 * for Microchip-based systems.
 */

#include <zephyr/drivers/pinctrl.h>
#include "pinctrl_mchp_v1.h"

/**
 * @brief Configure a specific pin based on the provided pin configuration.
 *
 * This helper function configures a pin by determining its port function and then
 * calling the appropriate hardware abstraction layer (HAL) functions to set
 * the pinmux and other pin configurations.
 *
 * @param pin The pin configuration to be applied. This is of type pinctrl_soc_pin_t.
 */
static void pinctrl_configure_pin(pinctrl_soc_pin_t pin)
{
	uint8_t port_func;

	port_func = SAM_PINMUX_FUNC_GET(pin);

	/* call pinmux hal function if alternate function is configured */
	if (port_func == SAM_PINMUX_FUNC_periph) {
		hal_mchp_pinctrl_pinmux(&pin, mchp_port_addrs);
	}
	/* set all other pin configurations */
	hal_mchp_pinctrl_set_flags(&pin, mchp_port_addrs);
}

/**
 * @brief Configure multiple pins.
 *
 * This function configures a set of pins based on the provided pin
 * configuration array.
 *
 * @param pins Pointer to an array of pinctrl_soc_pin_t structures that
 *             define the pin configurations.
 * @param pin_cnt Number of pins to configure.
 * @param reg Unused parameter.
 *
 * @return 0 on success.
 */
int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		pinctrl_configure_pin(*pins++);
	}

	return 0;
}
