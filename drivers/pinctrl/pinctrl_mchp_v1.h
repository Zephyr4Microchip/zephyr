/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file pinctrl_mchp_v1.h
 * @brief Microchip MCU family I/O Pin Controller (PORT)
 */

#ifndef MICROCHIP_PINCTRL_MCHP_V1_H_
#define MICROCHIP_PINCTRL_MCHP_V1_H_

#include <soc.h>

/**
 * @brief Utility macro that expands to the PORT address if it exists.
 *
 * This macro checks if a node label exists in the device tree and, if it does,
 * it expands to the register address of that node label. If the node label does
 * not exist, it expands to nothing.
 *
 * @param nodelabel The node label to check in the device tree.
 */
#define MCHP_PORT_ADDR_OR_NONE(nodelabel)                                                          \
	IF_ENABLED(DT_NODE_EXISTS(DT_NODELABEL(nodelabel)),            \
		(DT_REG_ADDR(DT_NODELABEL(nodelabel))))

#if defined(CONFIG_SOC_FAMILY_MCHP_SAM_D5X_E5X)

/**
 * @brief Array of port addresses for the MCHP SAMD5x_E5x series.
 *
 * This array contains the register addresses of the ports (PORTA, PORTB, PORTC, and PORTD)
 * for the MCHP SAMD5x_E5x series microcontrollers. The addresses are obtained using the
 * MCHP_PORT_ADDR_OR_NONE macro, which ensures that only existing ports are included.
 */
static const uint32_t mchp_port_addrs[] = {
	MCHP_PORT_ADDR_OR_NONE(porta),
	MCHP_PORT_ADDR_OR_NONE(portb),
	MCHP_PORT_ADDR_OR_NONE(portc),
	MCHP_PORT_ADDR_OR_NONE(portd),
};

#include "port/hal_mchp_pinctrl_port_u2210.h"

#endif

#endif /* MICROCHIP_PINCTRL_MCHP_V1_H_ */
