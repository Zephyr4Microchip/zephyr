/*
 * Copyright (c) [2025] [Microchip Technology Inc.]
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip MCU family I/O Pin Controller (PORT)
 */

#ifndef ZEPHYR_DRIVERS_PINCTRL_PINCTRL_MCHP_V1_H_
#define ZEPHYR_DRIVERS_PINCTRL_PINCTRL_MCHP_V1_H_
#include <soc.h>

/** Utility macro that expands to the PORT port address if it exists */
#define MCHP_PORT_ADDR_OR_NONE(nodelabel)                                                          \
	IF_ENABLED(DT_NODE_EXISTS(DT_NODELABEL(nodelabel)),			\
		   (DT_REG_ADDR(DT_NODELABEL(nodelabel)),))

#if defined(CONFIG_SOC_SERIES_MCHP_SAME54)

static const uint32_t mchp_port_addrs[] = {
	MCHP_PORT_ADDR_OR_NONE(porta),
	MCHP_PORT_ADDR_OR_NONE(portb),
	MCHP_PORT_ADDR_OR_NONE(portc),
	MCHP_PORT_ADDR_OR_NONE(portd),
};

#include "port_u2210/hal_mchp_pinctrl.h"

#endif

#endif /* ZEPHYR_DRIVERS_PINCTRL_PINCTRL_MCHP_V1_H_ */
