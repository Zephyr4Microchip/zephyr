/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file dac_mchp_v1.c
 * @brief Generic DAC Driver for Microchip MCUs.
 *
 * This file contains the implementation of a generic DAC driver for Microchip
 * microcontroller units (MCUs). It provides functions to configure and manage
 * DAC peripheral.
 */

#include <zephyr/logging/log.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include "dac_mchp_v1.h"

LOG_MODULE_REGISTER(dac_mchp_v1, CONFIG_DAC_LOG_LEVEL);

/**
 * @brief Device constant configuration parameters
 *
 * This structure holds the constant configuration parameters for the DAC
 * device.re
 */
typedef struct dac_mchp_dev_config {

	/* HAL layer for dac */
	hal_mchp_dac_t hal;

	/* Pin control device configuration */
	const struct pinctrl_dev_config *pcfg;

	/* Clock configuration */
	mchp_dac_clock_t dac_clock;

} dac_mchp_dev_config_t;

/**
 * @brief DAC run time data structure.
 *
 * This structure holds the runtime data for the DAC driver.
 */
typedef struct dac_mchp_dev_data {
	/* Status for the DAC Channel Configuration */
	bool is_configured[DAC_MAX_CHANNELS];

} dac_mchp_dev_data_t;

/**
 * @brief Set up a DAC channel with the given configuration.
 *
 * This function configures a DAC channel by setting resolution,
 * internal reference selection, buffer mode, and then enables the DAC controller.
 * It ensures that the channel configuration is valid before applying it.
 *
 *
 * @param dev Pointer to the device structure for the DAC instance.
 * @param channel_cfg Pointer to the DAC channel configuration structure.
 *
 * @retval 0 If the channel setup was successful.
 * @retval -EINVAL If the channel ID is invalid or not supported.
 * @retval -ENOTSUP If resolution, internal reference, or buffer setting is unsupported.
 */
static int dac_mchp_channel_setup(const struct device *dev,
				  const struct dac_channel_cfg *channel_cfg)
{
	const dac_mchp_dev_config_t *config = dev->config;
	const hal_mchp_dac_t *hal = &config->hal;
	dac_mchp_dev_data_t *const data = dev->data;
	int ret = 0;
	int8_t i = 0;

	/* Disable the Controller */
	hal_mchp_dac_disable_controller(hal);

	/* Check if resolution is valid */
	ret = hal_mchp_dac_set_resolution(hal, channel_cfg->resolution);
	if (ret != 0) {
		ret = -ENOTSUP;
	} else {
		/* Check if internal is valid */
		ret = hal_mchp_dac_set_internal(hal, channel_cfg->internal);
		if (ret != 0) {
			ret = -ENOTSUP;
		} else {
			/* Check if buffered is valid */
			ret = hal_mchp_dac_set_buffered(hal, channel_cfg->buffered);
			if (ret != 0) {
				ret = -ENOTSUP;
			}
		}
	}

	/* Validate channel ID here and only pass valid channels to the HAL
	 * to avoid validation within the HAL.
	 */
	if ((channel_cfg->channel_id >= DAC_MAX_CHANNELS) && (channel_cfg->channel_id != 0xFF)) {
		ret = -EINVAL;
	}

	/* Configure the DAC */
	if (ret == 0) {
		ret = hal_mchp_dac_configure(hal, channel_cfg->channel_id);
		/* Enable the DAC */
		hal_mchp_dac_enable_controller(hal);
		/* Wait till the DAC is ready for conversion after enabling the DAC */
		hal_mchp_dac_wait_ready(hal, channel_cfg->channel_id);
		if (channel_cfg->channel_id == 0xff) {
			for (i = 0; i < DAC_MAX_CHANNELS; i++) {
				data->is_configured[i] = true;
			}
		} else {
			data->is_configured[channel_cfg->channel_id] = true;
		}
	}

	return ret;
}

/**
 * @brief Write a value to the specified DAC channel.
 *
 * This function writes a digital value to the DAC data register
 * of the given channel. It first checks whether the channel
 * has been configured before writing. If the channel is not
 * configured, it returns an error.
 *
 * @param dev Pointer to the device structure for the DAC instance.
 * @param channel DAC channel number to which the value will be written.
 * @param value Digital value to write to the DAC.
 *
 * @retval 0 If the write was successful.
 * @retval -EINVAL If the specified channel is not configured.
 */
static int dac_mchp_write_value(const struct device *dev, uint8_t channel, uint32_t value)
{
	const dac_mchp_dev_config_t *config = dev->config;
	const hal_mchp_dac_t *hal = &config->hal;
	dac_mchp_dev_data_t *const data = dev->data;
	int ret = 0;

	if (channel == 0xFF) {
		/* Check all channels */
		for (int i = 0; i < DAC_MAX_CHANNELS; i++) {
			if (data->is_configured[i] == false) {
				ret = -EINVAL;
				break;
			}
		}
	} else if (channel >= DAC_MAX_CHANNELS || (data->is_configured[channel] == false)) {
		/* Invalid or unconfigured single channel */
		ret = -EINVAL;
	}

	if (ret == 0) {
		/* Write data into DAC */
		hal_mchp_dac_write_data(hal, channel, value);
	}
	return ret;
}

/**
 * @brief Initialize the Microchip DAC peripheral.
 *
 * This function performs the initial setup of the DAC peripheral,
 * including enabling the clock, applying pin configurations, resetting
 * the hardware, disabling the controller, setting operation modes,
 * and selecting the voltage reference.
 *
 * @param dev Pointer to the device structure for the DAC instance.
 *
 * @retval 0 Always returns 0 to indicate successful initialization.
 */
static int dac_mchp_init(const struct device *dev)
{
	const dac_mchp_dev_config_t *config = dev->config;
	const hal_mchp_dac_t *hal = &config->hal;

	/* Enable the DAC Clock */
	DAC_MCHP_ENABLE_CLOCK(dev);

	/* Apply pinctrl*/
	pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);

	/* Reset the DAC peripheral */
	hal_mchp_dac_reset(hal);

	/* Disable the DAC controller */
	hal_mchp_dac_disable_controller(hal);

	/* Set the DAC Operational Mode */
	hal_mchp_dac_set_diff_output(hal);

	/* Select the reference Volatage*/
	hal_mchp_dac_ref_selection(hal);

	return 0;
}

/**
 * @brief DAC driver API structure for Microchip DAC implementation.
 *
 * This structure binds the DAC driver API functions to the Microchip-specific
 * implementation. It is used by the Zephyr device driver model to call the
 * appropriate driver-specific functions for DAC operations.
 */
static const struct dac_driver_api dac_mchp_driver_api = {.channel_setup = dac_mchp_channel_setup,
							  .write_value = dac_mchp_write_value};

/**
 * @brief Macro to define dac device configuration
 *
 * @param n Instance number
 */
#define DAC_MCHP_CONFIG_DEFN(n)                                                                    \
	static const dac_mchp_dev_config_t dac_mchp_config_##n = {                                 \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		DAC_MCHP_CHANNEL_CONFIG(n) DAC_MCHP_HAL_DEFN(n) DAC_MCHP_CLOCK_DEFN(n)}

/**
 * @brief Macro to initialize dac device
 *
 * @param n Instance number
 */
#define DAC_MCHP_DEVICE_INIT(n)                                                                    \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	DAC_MCHP_CONFIG_DEFN(n);                                                                   \
	static dac_mchp_dev_data_t dac_mchp_data_##n = {};                                         \
	DEVICE_DT_INST_DEFINE(n, dac_mchp_init, NULL, &dac_mchp_data_##n, &dac_mchp_config_##n,    \
			      POST_KERNEL, CONFIG_DAC_INIT_PRIORITY, &dac_mchp_driver_api);

/**
 * @brief Initialize DAC devices for all compatible instances
 */
DT_INST_FOREACH_STATUS_OKAY(DAC_MCHP_DEVICE_INIT)
