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

/**
 * @brief Define the HAL configuration for UART.
 *
 * This macro sets up the HAL configuration for the UART peripheral
 *
 * @param n Instance number.
 */
#define WDT_MCHP_HAL_DEFN(n) .hal.regs = (wdt_registers_t *)DT_INST_REG_ADDR(n),

#endif /* MICROCHIP_WDT_MCHP_V1_H_ */
