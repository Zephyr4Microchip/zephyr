/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file reset_mchp_rstc_u2239.c
 * @brief Microchip RSTC U2239 Reset Controller Driver for Zephyr
 *
 * This file implements the driver for the Microchip RSTC U2239 reset controller,
 * providing APIs to assert, deassert, toggle, and query the status of reset lines.
 *
 */
#define DT_DRV_COMPAT microchip_rstc_u2239

#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/reset.h>

/** @brief Maximum number of reset lines supported by the controller. */
#define MCHP_RST_LINE_MAX 8

/**
 * @brief Macro to access the reset controller registers from the device config.
 *
 * @param dev Pointer to the device structure.
 * @return Pointer to the reset controller registers.
 */
#define RESET_REGS(dev) (((const reset_mchp_config_t *)((dev)->config))->regs)

/**
 * @brief Configuration structure for the Microchip RSTC U2239 reset controller.
 */
typedef struct reset_mchp_config {
	rstc_registers_t *regs; /* Pointer to the reset controller registers */
} reset_mchp_config_t;

/**
 * @brief Get the status of a reset line.
 *
 * This function checks if the specified reset line is currently asserted.
 *
 * @param[in]  dev    Pointer to the device structure for the driver instance.
 * @param[in]  id     Reset line ID (0-7).
 * @param[out] status Pointer to a variable to store the status (1 = asserted, 0 = not asserted).
 *
 * @retval 0        On success.
 * @retval -EINVAL  If the reset line ID is invalid.
 */
static int reset_mchp_status(const struct device *dev, uint32_t id, uint8_t *status)
{
	int ret = 0;
	uint8_t rcause = RESET_REGS(dev)->RSTC_RCAUSE;

	if (id >= MCHP_RST_LINE_MAX) {
		ret = -EINVAL;
	} else {
		*status = (rcause & BIT(id)) ? 1 : 0;
	}
	return ret;
}

/**
 * @brief Assert (activate) a reset line.
 *
 * This function asserts the specified reset line.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 * @param[in] id  Reset line ID.
 *
 * @retval -ENOTSUP Operation not supported by hardware.
 */
static int reset_mchp_line_assert(const struct device *dev, uint32_t id)
{
	/* Not supported by hardware */
	return -ENOTSUP;
}

/**
 * @brief Deassert (deactivate) a reset line.
 *
 * This function deasserts the specified reset line.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 * @param[in] id  Reset line ID.
 *
 * @retval -ENOTSUP Operation not supported by hardware.
 */
static int reset_mchp_line_deassert(const struct device *dev, uint32_t id)
{
	/* Not supported by hardware */
	return -ENOTSUP;
}

/**
 * @brief Toggle a reset line (assert then deassert).
 *
 * This function asserts and then deasserts the specified reset line, with a short delay in between.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 * @param[in] id  Reset line ID.
 *
 * @retval -ENOTSUP Operation not supported by hardware.
 */
static int reset_mchp_line_toggle(const struct device *dev, uint32_t id)
{
	/* Not supported by hardware */
	return -ENOTSUP;
}

/**
 * @brief Reset driver API structure for Microchip RSTC U2239.
 *
 * This structure provides pointers to the driver functions implementing the reset API.
 */
static const struct reset_driver_api reset_mchp_driver_api = {
	.status = reset_mchp_status,
	.line_assert = reset_mchp_line_assert,
	.line_deassert = reset_mchp_line_deassert,
	.line_toggle = reset_mchp_line_toggle,
};

/**
 * @brief Configuration instance for the Microchip RSTC U2239 reset controller.
 */
static const reset_mchp_config_t reset_mchp_config = {
	.regs = (rstc_registers_t *)DT_INST_REG_ADDR(0), /* Get the base address from DT */
};

/**
 * @brief Device instance definition for the Microchip RSTC U2239 reset controller.
 *
 * This macro defines and registers the device instance with the Zephyr device model.
 */
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) <= 1,
	     "Only one Microchip RSTC U2239 instance is supported.");

DEVICE_DT_INST_DEFINE(0, NULL,                            /* no init function */
		      NULL,                               /* PM device (or NULL) */
		      NULL,                               /* data (or pointer to data struct) */
		      &reset_mchp_config,                 /* config (or pointer to config struct) */
		      PRE_KERNEL_1,                       /* initialization level */
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, /* initialization priority */
		      &reset_mchp_driver_api              /* API struct */
)
