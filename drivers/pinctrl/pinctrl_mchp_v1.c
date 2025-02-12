/*
 * Copyright (c) [2025] [Microchip Technology Inc.]
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include "pinctrl_mchp_v1.h"

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

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		pinctrl_configure_pin(*pins++);
	}

	return 0;
}
