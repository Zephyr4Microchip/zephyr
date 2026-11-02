/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/entropy.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <string.h>
#include "cam_trng.h"

/*******************************************
 * @brief Devicetree definitions
 *******************************************/
#define DT_DRV_COMPAT microchip_trng_g2_entropy

/*******************************************
 * Const and Macro Defines
 *******************************************/
LOG_MODULE_REGISTER(entropy_mchp_trng_g2, CONFIG_ENTROPY_LOG_LEVEL);

/*******************************************
 * Data Structures
 *******************************************/
struct entropy_g2_trng_data {
	struct k_sem sem;
};

static struct entropy_g2_trng_data entropy_g2_trng_data_inst;

/*
 * The library's errors are positive TRNG_ERROR codes, which a caller of the
 * entropy API takes for success: sys_csrand_get() then returned 0 with the
 * buffer unfilled. Report them as -EIO. A failed health test leaves the
 * error bits set until DRV_CRYPTO_TRNG_IsrHelper() clears them, so clear
 * them and try once more, then once more clear them for the next caller.
 */
static int trng_read(uint8_t *buffer, uint16_t length)
{
	TRNG_ERROR error = DRV_CRYPTO_TRNG_ReadData(buffer, length);

	if (error != TRNG_NO_ERROR) {
		DRV_CRYPTO_TRNG_IsrHelper();
		error = DRV_CRYPTO_TRNG_ReadData(buffer, length);
	}

	if (error != TRNG_NO_ERROR) {
		DRV_CRYPTO_TRNG_IsrHelper();
		LOG_ERR("TRNG read failed with error code: %d", error);
		return -EIO;
	}

	return 0;
}

/* API implementation: driver initialization */
static int entropy_g2_trng_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	struct entropy_g2_trng_data *data = &entropy_g2_trng_data_inst;
	TRNG_ERROR error;

	/* Initialize semaphore with count of 1 (binary semaphore) */
	k_sem_init(&data->sem, 1, 1);

	error = DRV_CRYPTO_TRNG_Setup();
	if (error != TRNG_NO_ERROR) {
		LOG_ERR("TRNG setup failed with error code: %d", error);
		return -EIO;
	}
	LOG_INF("TRNG initialized");

	return 0;
}

/* API implementation: get_entropy */
static int entropy_g2_trng_get_entropy(const struct device *dev, uint8_t *buffer, uint16_t length)
{
	ARG_UNUSED(dev);
	struct entropy_g2_trng_data *data = &entropy_g2_trng_data_inst;
	int ret;

	if (length == 0) {
		return -EINVAL;
	}

	if (buffer == NULL) {
		return -EINVAL;
	}

	/* Acquire semaphore to protect hardware access */
	k_sem_take(&data->sem, K_FOREVER);

	ret = trng_read(buffer, length);

	/* Release semaphore */
	k_sem_give(&data->sem);

	return ret;
}

/* API implementation: get_entropy_isr */
static int entropy_g2_trng_get_entropy_isr(const struct device *dev, uint8_t *buffer,
					   uint16_t length, uint32_t flags)
{
	ARG_UNUSED(flags);
	struct entropy_g2_trng_data *data = &entropy_g2_trng_data_inst;
	int ret;

	if (length == 0) {
		return -EINVAL;
	}

	if (buffer == NULL) {
		return -EINVAL;
	}

	/* Non-blocking semaphore attempt for ISR context */
	if (k_sem_take(&data->sem, K_NO_WAIT) != 0) {
		LOG_WRN("TRNG busy, cannot service ISR request");
		return -EAGAIN;
	}

	ret = trng_read(buffer, length);

	k_sem_give(&data->sem);

	return (ret == 0) ? length : ret;
}

/* Entropy driver APIs structure */
static DEVICE_API(entropy, entropy_g2_trng_api) = {
	.get_entropy = entropy_g2_trng_get_entropy,
	.get_entropy_isr = entropy_g2_trng_get_entropy_isr
};

/* Entropy driver registration */
DEVICE_DT_INST_DEFINE(0, entropy_g2_trng_init, NULL, &entropy_g2_trng_data_inst, NULL,
		      PRE_KERNEL_1, CONFIG_ENTROPY_INIT_PRIORITY, &entropy_g2_trng_api);
