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

/* API implementation: driver initialization */
static int entropy_g2_trng_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	struct entropy_g2_trng_data *data = &entropy_g2_trng_data_inst;

	/* Initialize semaphore with count of 1 (binary semaphore) */
	k_sem_init(&data->sem, 1, 1);

	DRV_CRYPTO_TRNG_Setup();
	LOG_INF("TRNG initialized");

	return 0;
}

/* API implementation: get_entropy */
static int entropy_g2_trng_get_entropy(const struct device *dev, uint8_t *buffer, uint16_t length)
{
	ARG_UNUSED(dev);
	struct entropy_g2_trng_data *data = &entropy_g2_trng_data_inst;
	TRNG_ERROR error;

	if (length == 0) {
		return -EINVAL;
	}

	if (buffer == NULL) {
		return -EINVAL;
	}

	/* Acquire semaphore to protect hardware access */
	k_sem_take(&data->sem, K_FOREVER);

	error = DRV_CRYPTO_TRNG_ReadData(buffer, length);

	/* Release semaphore */
	k_sem_give(&data->sem);

	if (error == TRNG_NO_ERROR) {
		LOG_DBG("TRNG read %u bytes successfully", length);
		return 0;
	}
	LOG_ERR("TRNG read failed with error code: %d", error);
	return error;
}

/* API implementation: get_entropy_isr */
static int entropy_g2_trng_get_entropy_isr(const struct device *dev, uint8_t *buffer,
					   uint16_t length, uint32_t flags)
{
	ARG_UNUSED(flags);
	struct entropy_g2_trng_data *data = &entropy_g2_trng_data_inst;
	TRNG_ERROR error;

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

	error = DRV_CRYPTO_TRNG_ReadData(buffer, length);

	k_sem_give(&data->sem);

	if (error == TRNG_NO_ERROR) {
		LOG_DBG("TRNG ISR read %u bytes successfully", length);
		return length;
	} else {
		LOG_ERR("TRNG ISR read failed with error code: %d", error);
		return error;
	}
}

/* Entropy driver APIs structure */
static DEVICE_API(entropy, entropy_g2_trng_api) = {
	.get_entropy = entropy_g2_trng_get_entropy,
	.get_entropy_isr = entropy_g2_trng_get_entropy_isr
};

/* Entropy driver registration */
DEVICE_DT_INST_DEFINE(0, entropy_g2_trng_init, NULL, &entropy_g2_trng_data_inst, NULL,
		      PRE_KERNEL_1, CONFIG_ENTROPY_INIT_PRIORITY, &entropy_g2_trng_api);
