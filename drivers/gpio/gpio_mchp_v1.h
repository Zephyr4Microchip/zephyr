/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file gpio_mchp_v1.h
 * @brief GPIO driver configuration for Microchip devices.
 */

#ifndef MICROCHIP_GPIO_MCHP_V1_H_
#define MICROCHIP_GPIO_MCHP_V1_H_

#if defined(CONFIG_SOC_FAMILY_MCHP_SAM_D5X_E5X)

/* Define compatible string */
#define DT_DRV_COMPAT microchip_port_u2210_gpio

/* This typedef renames port_group_registers_t to hal_gpio_port_reg */
typedef port_group_registers_t hal_gpio_port_reg;

/* Include HAL file, specific to the peripheral IP */
#include "port/hal_mchp_gpio_port_u2210.h"

#endif /* CONFIG_SOC_FAMILY_MCHP_SAM_D5X_E5X */

#endif /* MICROCHIP_GPIO_MCHP_V1_H_ */
