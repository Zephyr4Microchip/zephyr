/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file mchp_flash.h
 * @brief Microchip Flash Controller IP Header Inclusion
 *
 * This header file serves as a central inclusion point for IP-specific
 * flash controller headers for Microchip devices. Depending on the build
 * configuration, it conditionally includes the appropriate header files
 * for the supported flash controller IP blocks.
 *
 * @note This file is to manage inclusion of the correct IP-specific headers.
 *
 */
#ifndef INCLUDE_ZEPHYR_DRIVERS_FLASH_MCHP_FLASH_H_
#define INCLUDE_ZEPHYR_DRIVERS_FLASH_MCHP_FLASH_H_

#ifdef CONFIG_FLASH_MCHP_NVMCTRL_U2409
#include "mchp_nvmctrl_u2409.h"
#endif /* CONFIG_FLASH_MCHP_NVMCTRL_U2409 */

#endif /* INCLUDE_ZEPHYR_DRIVERS_FLASH_MCHP_FLASH_H_ */
