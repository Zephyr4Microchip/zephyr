/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
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

/**
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
#define DT_DRV_COMPAT             microchip_trng_u2242
#define ENTROPY_MCHP_TRNG_TIMEOUT 1000000
/* ENTROPY Register */
#define ENTROPY_REGS              ((const entropy_mchp_config_t *)(dev)->config)->regs

/**
 * @brief Macro to enable the entropy (TRNG) clocks.
 *
 * This macro enables the clocks for the TRNG peripheral by turning on
 * the clock control for the specified device.
 *
 * @param dev Pointer to the device structure for the TRNG peripheral.
 */
#define ENTROPY_MCHP_ENABLE_CLOCK(dev)                                                             \
	clock_control_on(((const entropy_mchp_config_t *)(dev->config))->entropy_clock.clock_dev,  \
			 &(((entropy_mchp_config_t *)(dev->config))->entropy_clock.mclk_sys));

/**
 * @struct mchp_entropy_clock
 * @brief Clock configuration structure for entropy (TRNG) peripheral.
 *
 * This structure defines the clock configuration for the entropy (TRNG) peripheral,
 * including the clock device and subsystem.
 */

typedef struct mchp_entropy_clock {
	const struct device *clock_dev;
	clock_control_mchp_subsys_t mclk_sys;
} mchp_entropy_clock_t;

/**
 * @struct entropy_mchp_config_t
 * @brief Entropy configuration structure.
 *
 * This structure defines the configuration for the entropy (TRNG) peripheral.
 */
typedef struct entropy_mchp_config {
	trng_registers_t *regs; /** Pointer to the TRNG registers */
	mchp_entropy_clock_t
		entropy_clock; /** Clock control configuration for the TRNG subsystem */
	uint8_t num_irq;       /** Number of interrupts associated with the TRNG */
			       // void (*config_func)(const struct device *dev);
} entropy_mchp_config_t;

/**
 * @brief Check if TRNG data is ready.
 *
 * This function checks if the TRNG has generated new random data.
 *
 * @return 1 if data is ready, 0 if not ready, ENTROPY_MCHP_FAIL on error.
 */
static inline int mchp_entropy_ready(const struct device *dev)
{
	if (!ENTROPY_REGS) {
		return -ENODEV;
	}
	return (ENTROPY_REGS->TRNG_INTFLAG & TRNG_INTFLAG_DATARDY_Msk);
}

/**
 * @brief Wait for TRNG to be ready.
 *
 * This function waits for the True Random Number Generator (TRNG) to be ready
 * by checking the readiness status. It will timeout if the TRNG is not ready
 * within a specified period.
 *
 * @return 0 on success, -ETIMEDOUT on timeout.
 */
static int entropy_mchp_wait_ready(const struct device *dev)
{
	/* Wait for the TRNG to be ready */
	int timeout = ENTROPY_MCHP_TRNG_TIMEOUT; /* Timeout value */
	int ret = 0;

	while (!mchp_entropy_ready(dev)) {
		if (timeout == 0) {
			ret = -ETIMEDOUT;
			break;
		}
		timeout--;
	}
	return ret;
}

/**
 * @brief Retrieve entropy in a blocking manner.
 *
 * This function retrieves entropy from the device in a blocking manner, meaning
 * it will wait until the requested amount of entropy is available.
 *
 * @param dev Pointer to the device structure.
 * @param buffer Pointer to the buffer where entropy will be stored.
 * @param length Length of the entropy to be retrieved.
 *
 * @return 0 on success.
 */
static int entropy_mchp_get_entropy_blocking(const struct device *dev, uint8_t *buffer,
					     uint16_t length)
{
	int ret = 0;
	while (length > 0) {
		size_t to_copy = 0;
		uint32_t value = 0;

		/* Wait for the TRNG to be ready */
		int wait = entropy_mchp_wait_ready(dev);
		if (wait < 0) {
			ret = wait;
			break;
		}

		value = ENTROPY_REGS->TRNG_DATA; /* Read the random data from the TRNG
							register */

		to_copy = MIN(length, sizeof(value));
		memcpy(buffer, &value, to_copy);
		buffer += to_copy;
		length -= to_copy;
	}

	return ret;
}

/**
 * @brief Retrieve entropy in a non-blocking manner.
 *
 * This function retrieves entropy from the device in a non-blocking manner,
 * meaning it will return immediately if the requested amount of entropy is not
 * available.
 *
 * @param dev Pointer to the device structure.
 * @param buffer Pointer to the buffer where entropy will be stored.
 * @param length Length of the entropy to be retrieved.
 * @param flags Flags indicating the behavior of the function.
 *
 * @return 0 on success.
 */
static int entropy_mchp_get_entropy_non_blocking(const struct device *dev, uint8_t *buffer,
						 uint16_t length, uint32_t flags)
{
	int ret = 0;
	while (length > 0) {
		size_t to_copy = 0;
		uint32_t value = 0;

		if (!mchp_entropy_ready(dev)) {
			ret = 0;
			break;
		}
		value = ENTROPY_REGS->TRNG_DATA; /* Read the random data from the TRNG
							register */

		to_copy = MIN(length, sizeof(value));
		memcpy(buffer, &value, to_copy);
		buffer += to_copy;
		length -= to_copy;
	}

	return ret;
}

/**
 * @brief Retrieve entropy from the device.
 *
 * This function retrieves entropy from the device using a blocking method.
 *
 * @param dev Pointer to the device structure.
 * @param buffer Pointer to the buffer where entropy will be stored.
 * @param length Length of the entropy to be retrieved.
 *
 * @return 0 on success.
 */
static int entropy_mchp_get_entropy(const struct device *dev, uint8_t *buffer, uint16_t length)
{
	return entropy_mchp_get_entropy_blocking(dev, buffer, length);
}

/**
 * @brief Retrieve entropy from the device in an ISR context.
 *
 * This function retrieves entropy from the device in an ISR (Interrupt Service
 * Routine) context. It can operate in either blocking or non-blocking mode
 * based on the flags provided.
 *
 * @param dev Pointer to the device structure.
 * @param buffer Pointer to the buffer where entropy will be stored.
 * @param length Length of the entropy to be retrieved.
 * @param flags Flags indicating the behavior of the function.
 *
 * @return Number of bytes retrieved on success, or an error code.
 */
static int entropy_mchp_get_entropy_isr(const struct device *dev, uint8_t *buffer, uint16_t length,
					uint32_t flags)
{
	uint16_t cnt = length;
	int ret = cnt;

	if ((flags & ENTROPY_BUSYWAIT) == 0U) {
		int nb_ret = entropy_mchp_get_entropy_non_blocking(dev, buffer, length, flags);
		if (nb_ret != 0) {
			ret = nb_ret;
		}
	} else {
		int b_ret = entropy_mchp_get_entropy_blocking(dev, buffer, length);
		if (b_ret != 0) {
			ret = b_ret;
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
	ENTROPY_MCHP_ENABLE_CLOCK(dev); /* Enable the MCLK */

	ENTROPY_REGS->TRNG_CTRLA = TRNG_CTRLA_ENABLE(1); /* Enable the TRNG  */

	return 0;
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
