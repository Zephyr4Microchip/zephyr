/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file entropy_mchp_trng_u2242.c
 * @brief Microchip U2242 True Random Number Generator (TRNG) Entropy driver for Zephyr
 *
 * This file implements the driver for the Microchip U2242 TRNG hardware,
 * providing entropy services to the Zephyr OS entropy subsystem.
 *
 */
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <string.h>
#include <errno.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(entropy_mchp_trng_u2242, CONFIG_INTC_LOG_LEVEL);
/**
 * @def DT_DRV_COMPAT
 * @brief Device Tree Driver Compatibility
 *
 * This macro defines the compatibility string used in the device tree to
 * identify the Microchip True Random Number Generator (TRNG) driver. It is used by
 * the device driver infrastructure to match the driver with the corresponding
 * hardware described in the device tree.
 *
 * The compatibility string "microchip,trng-u2242" should be used in the device
 * tree source file to specify the Microchip U2242 TRNG node.
 */
#define DT_DRV_COMPAT microchip_trng_u2242

/** @brief Timeout value for TRNG readiness polling. */
#define TRNG_TIMEOUT 1000000

#define ENTROPY_BLOCKING     ENTROPY_BUSYWAIT
#define ENTROPY_NON_BLOCKING 0

/** @brief Macro to access the TRNG register block from the device config. */
#define ENTROPY_REGS ((const entropy_mchp_config_t *)(dev)->config)->regs

/**
 * @struct entropy_mchp_clock
 * @brief Clock configuration structure for entropy (TRNG) peripheral.
 *
 * This structure defines the clock configuration for the entropy (TRNG) peripheral,
 * including the clock device and subsystem.
 */

typedef struct entropy_mchp_clock {
	const struct device *clock_dev;
	clock_control_mchp_subsys_t mclk_sys;
} entropy_mchp_clock_t;

/**
 * @struct entropy_mchp_config_t
 * @brief Entropy configuration structure.
 *
 * This structure defines the configuration for the entropy (TRNG) peripheral.
 */
typedef struct entropy_mchp_config {
	trng_registers_t *regs; /** Pointer to the TRNG registers */

	entropy_mchp_clock_t
		entropy_clock; /** Clock control configuration for the TRNG subsystem */
	uint8_t num_irq;       /** Number of interrupts associated with the TRNG */
} entropy_mchp_config_t;

/**
 * @brief Check if TRNG data is ready.
 *
 * This function checks if the TRNG has generated new random data.
 *
 * @param dev Pointer to the device structure for the TRNG peripheral.
 *
 * @retval 1 if data is ready
 * @retval 0 if not ready
 */
static inline uint8_t entropy_ready(const struct device *dev)
{
	return (ENTROPY_REGS->TRNG_INTFLAG & TRNG_INTFLAG_DATARDY_Msk);
}

/**
 * @brief Wait for TRNG to be ready.
 *
 * This function waits for the True Random Number Generator (TRNG) to be ready
 * by checking the readiness status. It will timeout if the TRNG is not ready
 * within a specified period.
 *
 * @param dev Pointer to the device structure for the TRNG peripheral.
 *
 * @retval 0 on success
 * @retval -ETIMEDOUT on timeout
 */
static int entropy_wait_ready(const struct device *dev)
{
	/* Wait for the TRNG to be ready */
	int timeout = TRNG_TIMEOUT; /* Timeout value */
	int ret = 0;

	while (entropy_ready(dev) == 0) {
		if (timeout == 0) {
			ret = -ETIMEDOUT;
			break;
		}
		timeout--;
	}
	return ret;
}

/**
 * @brief Read entropy data from the TRNG.
 *
 * This function reads random data from the TRNG hardware into the provided buffer.
 * It supports both blocking and non-blocking modes.
 *
 * @param dev Pointer to the device structure for the TRNG peripheral.
 * @param buffer Pointer to the buffer to store random data.
 * @param length Number of bytes to read.
 * @param flags Flags to control blocking/non-blocking behavior.
 *
 * @retval Number of bytes read on success
 * @retval -EAGAIN if data is not ready (non-blocking)
 * @retval -ETIMEDOUT if TRNG is not ready in time (blocking)
 * @retval -EINVAL on invalid arguments
 */
static int entropy_read(const struct device *dev, uint8_t *buffer, uint16_t length, uint32_t flags)
{
	int ret = -EINVAL;
	uint16_t cnt = length;

	while (length > 0) {
		size_t to_copy = 0;
		uint32_t value = 0;

		if ((flags & ENTROPY_BUSYWAIT) != 0U) {
			int wait = entropy_wait_ready(dev);

			if (wait < 0) {
				ret = wait;
				break;
			}
		} else {
			if (entropy_ready(dev) == 0) {
				ret = -EAGAIN;
				break;
			}
			ret = (int)(cnt - length);
		}

		value = ENTROPY_REGS->TRNG_DATA;
		to_copy = MIN(length, sizeof(value));
		memcpy(buffer, &value, to_copy);
		buffer += to_copy;
		length -= to_copy;
	}

	if (length == 0) {
		ret = (int)cnt;
	}

	return ret;
}

/**
 * @brief Get entropy (random data) from the TRNG (blocking).
 *
 * This function retrieves random data from the TRNG in a blocking manner.
 *
 * @param dev Pointer to the device structure for the TRNG peripheral.
 * @param buffer Pointer to the buffer to store random data.
 * @param length Number of bytes to read.
 *
 * @retval 0 on success
 * @retval -EINVAL on invalid arguments or failure
 */
static int entropy_mchp_get_entropy(const struct device *dev, uint8_t *buffer, uint16_t length)
{
	int ret = -EINVAL;

	if (length > 0) {
		/* Always blocking in this API */
		int bk_ret = entropy_read(dev, buffer, length, ENTROPY_BLOCKING);

		if (bk_ret == length) {
			ret = 0;
		}
	} else {
		LOG_DBG("Invalid length:error %d", ret);
	}

	return ret;
}

/**
 * @brief Get entropy (random data) from the TRNG (ISR context).
 *
 * This function retrieves random data from the TRNG, supporting both blocking
 * and non-blocking modes, suitable for use in interrupt service routines.
 *
 * @param dev Pointer to the device structure for the TRNG peripheral.
 * @param buffer Pointer to the buffer to store random data.
 * @param length Number of bytes to read.
 * @param flags Flags to control blocking/non-blocking behavior.
 *
 * @retval Number of bytes read on success
 * @retval -EAGAIN if data is not ready (non-blocking)
 * @retval -EINVAL on invalid arguments
 */
static int entropy_mchp_get_entropy_isr(const struct device *dev, uint8_t *buffer, uint16_t length,
					uint32_t flags)
{
	int ret = -EINVAL;

	if (length > 0) {
		if ((flags & ENTROPY_BUSYWAIT) == 0U) {
			/* Non-blocking: only read if ready */
			ret = entropy_read(dev, buffer, length, ENTROPY_NON_BLOCKING);

		} else {
			/* Blocking: wait until ready */
			ret = entropy_read(dev, buffer, length, ENTROPY_BLOCKING);
		}
	}
	return ret;
}

/**
 * @brief Initialize the entropy device.
 *
 * This function initializes the entropy device.
 *
 * @param dev Pointer to the device structure.
 *
 * @return 0 on success.
 */
static int entropy_mchp_init(const struct device *dev)
{
	const entropy_mchp_config_t *entropy_cfg = dev->config;
	int ret_val = 0;

	do {
		ret_val = clock_control_on(
			entropy_cfg->entropy_clock.clock_dev,
			(clock_control_subsys_t)&entropy_cfg->entropy_clock.mclk_sys);

		if ((ret_val < 0) && (ret_val != -EALREADY)) {
			LOG_ERR("Clock control on failed for mclk %d", ret_val);
			break;
		}

		ENTROPY_REGS->TRNG_CTRLA = TRNG_CTRLA_ENABLE(1); /* Enable the TRNG  */

	} while (0);

	ret_val = (ret_val == -EALREADY) ? 0 : ret_val;

	return ret_val;
}

/** Entropy driver API structure. */
static const struct entropy_driver_api entropy_mchp_driver_api = {
	.get_entropy = entropy_mchp_get_entropy, .get_entropy_isr = entropy_mchp_get_entropy_isr};

/**
 * @brief Initialize entropy device instances.
 *
 * This macro initializes instances of the entropy device for each compatible
 * device tree node.
 *
 * @param n Device instance number.
 */
#define ENTROPY_MCHP_CONFIG_DEFN(n)                                                                \
	static const entropy_mchp_config_t entropy_mchp_config_##n = {                             \
		.regs = (trng_registers_t *)DT_INST_REG_ADDR(n),                                   \
		.entropy_clock = {                                                                 \
			.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                           \
			.mclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, mclk)),   \
				     .id = DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, id)}}}
/**
 * @brief Initialize entropy device instances.
 *
 * This macro initializes instances of the entropy device for each compatible
 * device tree node.
 *
 * @param n Device instance number.
 */
#define ENTROPY_DEVICE_INIT(n)                                                                     \
	ENTROPY_MCHP_CONFIG_DEFN(n);                                                               \
	DEVICE_DT_INST_DEFINE(n, entropy_mchp_init,     /* init function */                        \
			      NULL,                     /* PM device (or NULL) */                  \
			      NULL,                     /* data (or pointer to data struct) */     \
			      &entropy_mchp_config_##n, /* config (or pointer to config struct) */ \
			      POST_KERNEL,              /* initialization level */                 \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, /* initialization priority */    \
			      &entropy_mchp_driver_api            /* API struct */                 \
	)

DT_INST_FOREACH_STATUS_OKAY(ENTROPY_DEVICE_INIT)
