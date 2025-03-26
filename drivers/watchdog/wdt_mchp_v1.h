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

#include <stdint.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include <zephyr/kernel.h>

/**
 * @brief Type definition for the WDT lock.
 *
 * This macro defines the type of the lock used to protect access to the WDT APIs.
 */
#define MCHP_WDT_LOCK_TYPE struct k_mutex

/**
 * @brief Timeout duration for acquiring the WDT lock.
 *
 * This macro defines the timeout duration for acquiring the WDT lock.
 * The timeout is specified in milliseconds.
 */
#define MCHP_WDT_LOCK_TIMEOUT K_MSEC(10)

/**
 * @brief Initialize the WDT lock.
 *
 * This macro initializes the WDT lock.
 *
 * @param p_lock Pointer to the lock to be initialized.
 */
#define MCHP_WDT_DATA_LOCK_INIT(p_lock) k_mutex_init(p_lock)

/**
 * @brief Acquire the WDT lock.
 *
 * This macro acquires the WDT lock. If the lock is not available, the
 * function will wait for the specified timeout duration.
 *
 * @param p_lock Pointer to the lock to be acquired.
 * @return 0 if the lock was successfully acquired, or a negative error code.
 */
#define MCHP_WDT_DATA_LOCK(p_lock) k_mutex_lock(p_lock, MCHP_WDT_LOCK_TIMEOUT)

/**
 * @brief Release the WDT lock.
 *
 * This macro releases the WDT lock.
 *
 * @param p_lock Pointer to the lock to be released.
 * @return 0 if the lock was successfully released, or a negative error code.
 */
#define MCHP_WDT_DATA_UNLOCK(p_lock) k_mutex_unlock(p_lock)

#if defined(CONFIG_SOC_FAMILY_MCHP_SAM_D5X_E5X)

/**
 * @brief Device Tree Driver Compatibility
 *
 * This macro defines the compatibility string used in the device tree to
 * identify the Microchip U2251 Watchdog Timer (WDT) driver. It is used by
 * the device driver infrastructure to match the driver with the corresponding
 * hardware described in the device tree.
 *
 * The compatibility string "microchip,wdt-u2251" should be used in the device
 * tree source file to specify the Microchip U2251 WDT node
 */
#define DT_DRV_COMPAT microchip_wdt_u2251

/**
 * @def MAX_INSTALLABLE_TIMEOUT_COUNT
 * @brief Macro to get the maximum installable timeout count from the device
 * tree.
 */
#define MAX_INSTALLABLE_TIMEOUT_COUNT (DT_PROP(DT_NODELABEL(wdog), max_installable_timeout_count))

/**
 * @def MAX_TIMEOUT_WINDOW
 * @brief Macro to get the maximum timeout window from the device tree.
 */
#define MAX_TIMEOUT_WINDOW (DT_PROP(DT_NODELABEL(wdog), max_timeout_window))

/**
 * @def MAX_TIMEOUT_WINDOW_MODE
 * @brief Macro to get the maximum timeout window mode from the device tree.
 */
#define MAX_TIMEOUT_WINDOW_MODE (DT_PROP(DT_NODELABEL(wdog), max_timeout_window_mode))

/**
 * @def MIN_WINDOW_LIMIT
 * @brief Macro to get the min window limit from the device tree.
 */
#define MIN_WINDOW_LIMIT (DT_PROP(DT_NODELABEL(wdog), min_window_limit))

/**
 * @def WDT_FLAG_ONLY_ONE_TIMEOUT_VALUE_SUPPORTED
 * @brief Macro to get the value of "only_one_timeout_val_supported_flag" from
 * the device tree.
 */
#define WDT_FLAG_ONLY_ONE_TIMEOUT_VALUE_SUPPORTED                                                  \
	(DT_PROP(DT_NODELABEL(wdog), only_one_timeout_val_supported_flag))

/**
 * @brief Structure for HAL WDT.
 *
 * This structure holds the register pointer to the WDT and is used to
 * configure and control the WDT.
 */
typedef struct hal_mchp_wdt {
	wdt_registers_t *regs; /**< Pointer to WDT registers */
} hal_mchp_wdt_t;

/**
 * @brief Define the HAL configuration for UART.
 *
 * This macro sets up the HAL configuration for the UART peripheral
 *
 * @param n Instance number.
 */
#define WDT_MCHP_HAL_DEFN(n) .hal_wdt.regs = (wdt_registers_t *)DT_INST_REG_ADDR(n),

/**
 * @brief Macro to define the Watchdog Timer (WDT) clock configuration.
 *
 * This macro defines the clock configuration for the Watchdog Timer (WDT) for a given
 * instance. It initializes the clock device and the main clock system (MCLK) for the WDT.
 *
 * @param n The instance number of the WDT.
 */
#define WDT_MCHP_CLOCK_DEFN(n)                                                                     \
	.wdt_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                                 \
	.wdt_clock.mclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, mclk)),         \
			       .id = DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, id)}

/**
 * @brief Macro to enable the clock for the Watchdog Timer (WDT).
 *
 * This macro enables the clock for the Watchdog Timer (WDT) for a given device.
 * It uses the clock control API to turn on the clock device and the main clock system
 * (MCLK) for the WDT.
 *
 * @param wdt_dev Pointer to the device structure for the driver instance.
 *
 */
#define MCHP_WDT_ENABLE_CLOCK(wdt_dev)                                                             \
	clock_control_on(((const wdt_mchp_dev_cfg_t *)(wdt_dev->config))->wdt_clock.clock_dev,     \
			 &(((wdt_mchp_dev_cfg_t *)(wdt_dev->config))->wdt_clock.mclk_sys));

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

/* Include HAL file, specific to the peripheral IP */
#include "wdt_u2251/hal_mchp_wdt_u2251.h"
#endif /* CONFIG_SOC_FAMILY_MCHP_SAM_D5X_E5X */
#endif /* MICROCHIP_WDT_MCHP_V1_H_ */
