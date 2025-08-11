/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 *
 * @file hwinfo_mchp_g1.c
 * @brief Microchip  hardware information and reset cause driver implementation.
 *
 * This driver provides access to the device ID and reset cause information for Microchip devices
 * using Zephyr's hardware info and reset APIs.
 */

#include <soc.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

/*******************************************
 * @brief Devicetree definitions
 *******************************************/
#define DT_DRV_COMPAT microchip_hwinfo_g1

/*******************************************
 * Const and Macro Defines
 *******************************************/
LOG_MODULE_REGISTER(hwinfo_mchp_g1, LOG_LEVEL_ERR);

#define HWINFO_ID_WORDS 4

#define RSTC_BASE_ADDR DT_REG_ADDR(DT_NODELABEL(rstc))

/*******************************************
 * @brief Data type definitions
 ******************************************/
/**
 * @brief Structure to hold a hardware unique identifier.
 *
 * This structure contains a 128-bit data array of four 32-bit unsigned integers,
 * representing the unique hardware ID.
 */
typedef struct hwinfo_id {
	uint32_t id[HWINFO_ID_WORDS];
} hwinfo_id_t;

/******************************************************************************
 * @brief API functions
 *****************************************************************************/
/**
 * @brief Get the unique device ID.
 *
 * This function reads the device's unique ID from hardware registers,
 * and copies it to the provided buffer.
 *
 * @param[out] buffer Pointer to the buffer to store the device ID.
 * @param[in]  length Length of the buffer.
 *
 * @return Number of bytes copied to the buffer.
 */
ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	hwinfo_id_t dev_id;

	dev_id.id[0] = sys_cpu_to_be32(*(const uint32_t *)DT_INST_REG_ADDR_BY_IDX(0, 0));
	dev_id.id[1] = sys_cpu_to_be32(*(const uint32_t *)DT_INST_REG_ADDR_BY_IDX(0, 1));
	dev_id.id[2] = sys_cpu_to_be32(*(const uint32_t *)DT_INST_REG_ADDR_BY_IDX(0, 2));
	dev_id.id[3] = sys_cpu_to_be32(*(const uint32_t *)DT_INST_REG_ADDR_BY_IDX(0, 3));

	if (length > sizeof(dev_id.id)) {
		length = sizeof(dev_id.id);
	}

	uint8_t *src = (uint8_t *)dev_id.id;

	for (size_t i = 0; i < length; i++) {
		buffer[i] = src[i];
	}

	return length;
}

/**
 * @brief Get the supported reset causes.
 *
 * This function returns a bitmask of all reset causes supported by the hardware.
 *
 * @param[out] supported Pointer to a variable to store the supported reset causes.
 *
 * @return 0 on success
 */
int z_impl_hwinfo_get_supported_reset_cause(uint32_t *supported)
{
	*supported = RESET_POR | RESET_BROWNOUT | RESET_PIN | RESET_WATCHDOG | RESET_SOFTWARE |
		     RESET_USER | RESET_LOW_POWER_WAKE;

	return 0;
}

/**
 * @brief Get the cause of the last reset.
 *
 * This function queries the hardware for the cause(s) of the last reset and
 * returns a bitmask of the detected causes.
 *
 * @param[out] cause Pointer to a variable to store the reset cause bitmask.
 *
 * @return 0 on success, -EINVAL if cause is NULL and -ENOSYS if the RSTC base address is not
 * defined.
 */
int z_impl_hwinfo_get_reset_cause(uint32_t *cause)
{
	int ret = 0;
	uint32_t result = 0;

	do {
		if (cause == NULL) {
			LOG_ERR("Invalid argument: NULL pointer passed");
			ret = -EINVAL;
			break;
		}

		if (RSTC_BASE_ADDR == 0) {
			LOG_ERR("RSTC base address is not defined");
			ret = -ENOSYS;
			break;
		}

		volatile uint8_t *rcause_reg = (uint8_t *)(RSTC_BASE_ADDR);
		uint8_t rcause = *rcause_reg;

		if ((rcause & BIT(0)) != 0) {
			result |= RESET_POR;
		}
		if ((rcause & BIT(1)) != 0) {
			result |= RESET_BROWNOUT;
		}
		if ((rcause & BIT(2)) != 0) {
			result |= RESET_BROWNOUT;
		}
		if ((rcause & BIT(4)) != 0) {
			result |= RESET_PIN | RESET_USER;
		}
		if ((rcause & BIT(5)) != 0) {
			result |= RESET_WATCHDOG;
		}
		if ((rcause & BIT(6)) != 0) {
			result |= RESET_SOFTWARE;
		}
		if ((rcause & BIT(7)) != 0) {
			result |= RESET_LOW_POWER_WAKE;
		}

		*cause = result;
	} while (0);

	return ret;
}
