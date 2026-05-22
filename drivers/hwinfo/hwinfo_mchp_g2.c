/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 *
 * @file hwinfo_mchp_g2.c
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
#define DT_DRV_COMPAT microchip_hwinfo_g2

/*******************************************
 * Const and Macro Defines
 *******************************************/
LOG_MODULE_REGISTER(hwinfo_mchp_g2, LOG_LEVEL_ERR);

#define RCON_BASE_ADDR DT_REG_ADDR_BY_IDX(DT_NODELABEL(clock), 1)

#define DT_DRV_COMPAT microchip_hwinfo_g2

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
	uint32_t dev_id;

	dev_id = sys_cpu_to_be32(*(const uint32_t *)DT_INST_REG_ADDR_BY_IDX(0, 0));

	if (length > sizeof(dev_id)) {
		length = sizeof(dev_id);
	}

	uint8_t *p_src = (uint8_t *)&dev_id;

	for (size_t i = 0; i < length; i++) {
		buffer[i] = p_src[i];
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
		     RESET_USER | RESET_LOW_POWER_WAKE | RESET_PARITY;

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

		if (RCON_BASE_ADDR == 0) {
			LOG_ERR("RCON base address is not defined");
			ret = -ENOSYS;
			break;
		}

		volatile uint32_t *rcause_reg = (uint32_t *)(RCON_BASE_ADDR + RCON_RCON_REG_OFST);
		uint32_t rcause = *rcause_reg;

		if ((rcause & RCON_RCON_POR_Msk) != 0) {
			result |= RESET_POR;
		}
		if ((rcause & RCON_RCON_BOR_Msk) != 0) {
			result |= RESET_BROWNOUT;
		}
		if ((rcause & RCON_RCON_WDTO_Msk) != 0) {
			result |= RESET_WATCHDOG;
		}
		if ((rcause & RCON_RCON_DMTO_Msk) != 0) {
			result |= RESET_USER;
		}
		if ((rcause & RCON_RCON_SWR_Msk) != 0) {
			result |= RESET_SOFTWARE;
		}
		if ((rcause & RCON_RCON_EXTR_Msk) != 0) {
			result |= RESET_PIN | RESET_USER;
		}
		if ((rcause & RCON_RCON_CMR_Msk) != 0) {
			result |= RESET_PARITY;
		}
		if ((rcause & RCON_RCON_DPSLP_Msk) != 0) {
			result |= RESET_LOW_POWER_WAKE;
		}

		*cause = result;
	} while (0);

	return ret;
}

int z_impl_hwinfo_clear_reset_cause(void)
{
	volatile uint32_t *rcause_reg = (uint32_t *)(RCON_BASE_ADDR + RCON_RCON_REG_OFST);
	uint32_t rcause = *rcause_reg;
	volatile uint32_t *rcause_clr_reg = (uint32_t *)(RCON_BASE_ADDR + RCON_RCONCLR_REG_OFST);

	*rcause_clr_reg |= rcause;

	return 0;
}
