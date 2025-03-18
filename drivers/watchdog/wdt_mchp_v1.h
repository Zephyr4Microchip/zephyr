/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file wdt_mchp_v1.h
 * @brief WDT driver configuration for Microchip devices.
 *
 * This file provides macro definitions, structures, and
 * function-like macros for configuring and
 * initializing the WDT peripheral on Microchip devices.
 */

#ifndef MICROCHIP_WDT_MCHP_V1_H_
#define MICROCHIP_WDT_MCHP_V1_H_

#define DT_DRV_COMPAT microchip_wdt_u2251

#include "wdt_u2251/hal_mchp_wdt_u2251.h"
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
/**
 * @brief Define the HAL configuration for UART.
 *
 * This macro sets up the HAL configuration for the UART peripheral
 *
 * @param n Instance number.
 */
#define WDT_MCHP_HAL_DEFN(n) .hal.regs = (wdt_registers_t *)DT_INST_REG_ADDR(n),

#define WDT_MCHP_CLOCK_DEFN(n)                                                                     \
	.wdt_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                                 \
	.wdt_clock.mclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, mclk)),         \
			       .id = DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, id)}

#define MCHP_WDT_ENABLE_CLOCK(dev)                                                                 \
	clock_control_on(((const wdt_mchp_dev_cfg_t *)(dev->config))->wdt_clock.clock_dev,         \
			 &(((wdt_mchp_dev_cfg_t *)(dev->config))->wdt_clock.mclk_sys));

/**
 * @struct mchp_wdt_clock
 * @brief Structure to hold device clock configuration.
 */
typedef struct mchp_wdt_clock {

	/* Clock driver */
	const struct device *clock_dev;

	/* Main clock subsystem. */
	clock_control_mchp_subsys_t mclk_sys;

	/* Generic clock subsystem. */
	clock_control_mchp_subsys_t gclk_sys;

} mchp_wdt_clock_t;

#endif /* MICROCHIP_WDT_MCHP_V1_H_ */
