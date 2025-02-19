/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_MICROCHIP_SAM_GPIO_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_MICROCHIP_SAM_GPIO_H_

/**
 * @brief Enable GPIO pin debounce.
 *
 * The debounce flag is a Zephyr specific extension of the standard GPIO flags
 * specified by the Linux GPIO binding. Only applicable for Microchip SoCs.
 */
#define MCHP_GPIO_DEBOUNCE (1U << 8)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_MICROCHIP_SAM_GPIO_H_ */
