/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file dac_mchp_g1.c
 * @brief DAC Driver for Microchip DAC G1 Peripherals.
 *
 * This file contains the implementation of a DAC driver
 * for microchip dac g1 peripherals.
 *
 * Supported SoC Families:
 * - SOC_FAMILY_MCHP_SAM_D5X_E5X
 */

#include <soc.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>

LOG_MODULE_REGISTER(dac_mchp_g1, CONFIG_DAC_LOG_LEVEL);

/* Define compatible string for device tree.*/
#define DT_DRV_COMPAT microchip_dac_g1

/* Maximum number of DAC channels supported by the device.*/
#define DAC_MAX_CHANNELS DT_PROP(DT_NODELABEL(dac), max_channels)

/* Conversion Speed of 100kSPS */
#define DAC_CC100K 100

/* Conversion Speed of 500kSPS */
#define DAC_CC1M 500

/* Conversion Speed of 1MSPS */
#define DAC_CC12M 1000

/* Refresh Period */
#define DAC_REFRESH_PERIOD 30

/* DAC Resolution */
#define DAC_RESOLUTION 12

/* (DAC DATA) Mask DATA[15:12] Bit */
#define DAC_DATA_MSB_MASK (0x0FFFU)

/* (DAC DATA) Mask DATA[3:0] Bit */
#define DAC_DATA_LSB_MASK (0xFFF0U)

/* DAC data right adjust */
#define DAC_DATA_RIGHT_ADJ 0

/* DAC data left adjust */
#define DAC_DATA_LEFT_ADJ 1

#define DAC_CHANNELS_ALL 0xFF

/**
 * @brief DAC channel configuration.
 *
 * Defines settings for a DAC channel, including output rate,
 * filtering, dithering, and sampling parameters.
 */
typedef struct {
	/* Channel identifier */
	uint8_t channel;

	/* Conversion speed in KSPS */
	int rate;

	/* External Filter */
	bool ext_filter;

	/* Data adjustment */
	uint8_t data_adj;

	/* Dither */
	bool dither;

	/* Oversampling ratio*/
	int sampling_ratio;

	/* Refresh Period */
	int refresh;

} dac_mchp_channel_t;

/**
 * @brief Clock configuration structure for the DAC.
 *
 * This structure contains the clock configuration parameters for the DAC
 * peripheral.
 */
typedef struct dac_mchp_clock {

	/* Clock driver */
	const struct device *clock_dev;

	/* Main clock subsystem. */
	clock_control_subsys_t mclk_sys;

	/* Generic clock subsystem. */
	clock_control_subsys_t gclk_sys;

} dac_mchp_clock_t;

/**
 * @brief Device constant configuration parameters
 *
 * This structure holds the constant configuration parameters for the DAC
 * device.
 */
typedef struct dac_mchp_dev_config {

	/** Pointer to DAC registers */
	dac_registers_t *regs;

	/* Reference voltage selection */
	uint8_t refsel;

	/* Pin control device configuration */
	const struct pinctrl_dev_config *pcfg;

	/* Clock configuration */
	dac_mchp_clock_t dac_clock;

	/* Channel configurations */
	dac_mchp_channel_t channels[DAC_MAX_CHANNELS];

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

/* Waits for the DAC synchronization process to complete. */
static inline void dac_wait_sync(dac_registers_t *dac_reg, uint32_t sync_flag)
{
	/* Wait for synchronization */
	while ((dac_reg->DAC_SYNCBUSY & sync_flag) != 0U) {
		/* Do nothing */
	}
}

/* Waits until the DAC channel is ready for conversion. */
static void dac_wait_ready(dac_registers_t *dac_reg, uint8_t channel_id)
{
	uint8_t mask = DAC_STATUS_READY0_Msk;

	if ((dac_reg->DAC_CTRLA & DAC_CTRLA_ENABLE_Msk) == DAC_CTRLA_ENABLE_Msk) {
		if (channel_id == DAC_CHANNELS_ALL) {
			/* Wait for synchronization channel 0*/
			while ((dac_reg->DAC_STATUS & DAC_STATUS_READY0_Msk) !=
			       DAC_STATUS_READY0_Msk) {
				/* Do nothing */
			}
			/* Wait for synchronization channel 1*/
			while ((dac_reg->DAC_STATUS & DAC_STATUS_READY1_Msk) !=
			       DAC_STATUS_READY1_Msk) {
				/* Do nothing */
			}
		} else {
			if (channel_id == 1) {
				mask = DAC_STATUS_READY1_Msk;
			}
			/* Wait for synchronization */
			while ((dac_reg->DAC_STATUS & mask) != mask) {
				/* Do nothing */
			}
		}
	}
}

/* Enables the DAC controller. */
static inline void dac_enable_controller(dac_registers_t *dac_reg)
{
	dac_reg->DAC_CTRLA |= DAC_CTRLA_ENABLE_Msk;
	dac_wait_sync(dac_reg, DAC_SYNCBUSY_ENABLE_Msk);
}

/* Disables the DAC controller. */
static inline void dac_disable_controller(dac_registers_t *dac_reg)
{
	dac_reg->DAC_CTRLA &= ~DAC_CTRLA_ENABLE_Msk;
	dac_wait_sync(dac_reg, DAC_SYNCBUSY_ENABLE_Msk);
}

/* Resets the DAC controller. */
static inline void dac_reset(dac_registers_t *dac_reg)
{
	dac_reg->DAC_CTRLA = DAC_CTRLA_SWRST_Msk;
	dac_wait_sync(dac_reg, DAC_SYNCBUSY_SWRST_Msk);
}

/* Configures the DAC for differential output mode. */
static inline void dac_set_diff_output(dac_registers_t *dac_reg)
{
#if defined(CONFIG_DAC_MCHP_DIFFERENTIAL)
	dac_reg->DAC_CTRLB = DAC_CTRLB_DIFF_Msk;
#endif /* CONFIG_DAC_MCHP_DIFFERENTIAL */
}

/* Sets the reference voltage selection for the DAC. */
static inline void dac_ref_selection(dac_registers_t *dac_reg, uint8_t refsel)
{
	dac_reg->DAC_CTRLB =
		(dac_reg->DAC_CTRLB & ~DAC_CTRLB_REFSEL_Msk) | DAC_CTRLB_REFSEL(refsel);
}

/* Enables a specific DAC channel. */
static inline void dac_channel_enable(dac_registers_t *dac_reg, uint8_t channel_id)
{
	dac_reg->DAC_DACCTRL[channel_id] |= DAC_DACCTRL_ENABLE_Msk;
}

/* Sets the conversion speed for a specific DAC channel. */
static void dac_conversion_speed(dac_registers_t *dac_reg, int rate, uint8_t channel_id)
{
	uint32_t cctrl_val;

	switch (rate) {
	case DAC_CC100K:
		cctrl_val = DAC_DACCTRL_CCTRL_CC100K;
		break;
	case DAC_CC1M:
		cctrl_val = DAC_DACCTRL_CCTRL_CC1M;
		break;
	case DAC_CC12M:
		cctrl_val = DAC_DACCTRL_CCTRL_CC12M;
		break;
	default:
		LOG_WRN("Invalid DAC conversion rate (%d), defaulting to DAC_CC100K", rate);
		cctrl_val = DAC_DACCTRL_CCTRL_CC100K;
		break;
	}

	dac_reg->DAC_DACCTRL[channel_id] =
		(dac_reg->DAC_DACCTRL[channel_id] & ~DAC_DACCTRL_CCTRL_Msk) | cctrl_val;
}

/* Configures the external filter for a specific DAC channel. */
static inline void dac_external_filter(dac_registers_t *dac_reg, bool ext_filter,
				       uint8_t channel_id)
{
	dac_reg->DAC_DACCTRL[channel_id] =
		(dac_reg->DAC_DACCTRL[channel_id] & ~DAC_DACCTRL_FEXT_Msk) |
		DAC_DACCTRL_FEXT(ext_filter);
}

/* Configures the data alignment adjustment for a specific DAC channel. */
static inline void dac_data_adj(dac_registers_t *dac_reg, uint8_t data_adj, uint8_t channel_id)
{
	dac_reg->DAC_DACCTRL[channel_id] =
		(dac_reg->DAC_DACCTRL[channel_id] & ~DAC_DACCTRL_LEFTADJ_Msk) |
		DAC_DACCTRL_LEFTADJ(data_adj);
}

/* Configures the dither setting for a specific DAC channel. */
static inline void dac_dither(dac_registers_t *dac_reg, uint8_t dither, uint8_t channel_id)
{
	dac_reg->DAC_DACCTRL[channel_id] =
		(dac_reg->DAC_DACCTRL[channel_id] & ~DAC_DACCTRL_DITHER_Msk) |
		DAC_DACCTRL_DITHER(dither);
}

/* Configures the refresh rate for a specific DAC channel. */
static inline void dac_refresh(dac_registers_t *dac_reg, uint8_t refresh, uint8_t channel_id)
{
	if (refresh != 0) {
		refresh = refresh / DAC_REFRESH_PERIOD;
	}
	dac_reg->DAC_DACCTRL[channel_id] =
		(dac_reg->DAC_DACCTRL[channel_id] & ~DAC_DACCTRL_REFRESH_Msk) |
		DAC_DACCTRL_REFRESH(refresh);
}

/* Sets the sampling ratio for a specific DAC channel. */
static void dac_sampling_ratio(dac_registers_t *dac_reg, uint8_t sampling_ratio, uint8_t channel_id)
{
	uint8_t osr;

	switch (sampling_ratio) {
	case 2:
		osr = 0x1;
	case 4:
		osr = 0x2;
	case 8:
		osr = 0x3;
	case 16:
		osr = 0x4;
	case 32:
		osr = 0x5;
	default:
		osr = 0x0;
	}

	dac_reg->DAC_DACCTRL[channel_id] =
		(dac_reg->DAC_DACCTRL[channel_id] & ~DAC_DACCTRL_OSR_Msk) | DAC_DACCTRL_OSR(osr);
}

/* Writes a digital value to the specified DAC channel. */
static void dac_write_data(const struct device *dev, uint8_t channel_id, uint32_t value)
{
	const dac_mchp_dev_config_t *dev_cfg = dev->config;

	if (channel_id == DAC_CHANNELS_ALL) {
		if ((dev_cfg->channels[0].data_adj) == DAC_DATA_LEFT_ADJ) {
			dev_cfg->regs->DAC_DATA[0] = DAC_DATA_LSB_MASK & DAC_DATA_DATA(value);
		} else {
			dev_cfg->regs->DAC_DATA[0] = DAC_DATA_MSB_MASK & DAC_DATA_DATA(value);
		}
		/* Wait for synchronization */
		dac_wait_sync(dev_cfg->regs, DAC_SYNCBUSY_DATA0_Msk);
		if ((dev_cfg->channels[1].data_adj) == DAC_DATA_LEFT_ADJ) {
			dev_cfg->regs->DAC_DATA[1] = DAC_DATA_LSB_MASK & DAC_DATA_DATA(value);
		} else {
			dev_cfg->regs->DAC_DATA[1] = DAC_DATA_MSB_MASK & DAC_DATA_DATA(value);
		}
		/* Wait for synchronization */
		dac_wait_sync(dev_cfg->regs, DAC_SYNCBUSY_DATA1_Msk);
	} else {
		if ((dev_cfg->channels[channel_id].data_adj) == DAC_DATA_LEFT_ADJ) {
			dev_cfg->regs->DAC_DATA[channel_id] =
				DAC_DATA_LSB_MASK & DAC_DATA_DATA(value);
		} else {
			dev_cfg->regs->DAC_DATA[channel_id] =
				DAC_DATA_MSB_MASK & DAC_DATA_DATA(value);
		}
		/* Wait for synchronization */
		if (channel_id == 0) {
			dac_wait_sync(dev_cfg->regs, DAC_SYNCBUSY_DATA0_Msk);
		} else {
			dac_wait_sync(dev_cfg->regs, DAC_SYNCBUSY_DATA1_Msk);
		}
	}
}

/* Configures a specific DAC channel with predefined settings. */
static int dac_configure(const struct device *dev, uint8_t channel_id)
{

	const dac_mchp_dev_config_t *dev_cfg = dev->config;
	uint8_t i = 0, start = 0, end = 0;

#if defined(CONFIG_DAC_MCHP_DIFFERENTIAL)
	/* If differential is selected, we can use only channel 0 */
	if (channel_cfg->channel_id != 0) {
		return -EINVAL;
	}
#endif /* CONFIG_DAC_MCHP_DIFFERENTIAL */
	/*
	 * Determine the range of channels to configure.
	 * If channel_id is 0xFF, configure all DAC channels by iterating
	 * from channel 0 to DAC_MAX_CHANNELS - 1.
	 * Otherwise, configure only the specified channel.
	 */
	if (channel_id == DAC_CHANNELS_ALL) {
		start = 0;
		end = DAC_MAX_CHANNELS;
	} else {
		start = channel_id;
		end = channel_id + 1;
	}

	for (i = start; i < end; i++) {
		/* Enable the DAC for channels */
		dac_channel_enable(dev_cfg->regs, i);

		/* Set the DATA Adjustment */
		dac_data_adj(dev_cfg->regs, dev_cfg->channels[i].data_adj, i);

		/* Set the Dither */
		dac_dither(dev_cfg->regs, dev_cfg->channels[i].dither, i);

		/* Set the refresh period */
		if (dev_cfg->channels[i].sampling_ratio != 1) {
			dac_refresh(dev_cfg->regs, 0, i);
		} else {
			dac_refresh(dev_cfg->regs, dev_cfg->channels[i].refresh, i);
		}
		/* Set the conversion speed */
		dac_conversion_speed(dev_cfg->regs, dev_cfg->channels[i].rate, i);

		/* Set the External filter */
		dac_external_filter(dev_cfg->regs, dev_cfg->channels[i].ext_filter, i);

		/* Set the Oversampling Ratio */
		dac_sampling_ratio(dev_cfg->regs, dev_cfg->channels[i].sampling_ratio, i);
	}

	return 0;
}

/* Sets the resolution of the DAC. */
static inline int dac_set_resolution(dac_registers_t *dac_reg, uint8_t resolution)
{
	int err = 0;

	if (resolution != DAC_RESOLUTION) {
		err = -ENOTSUP;
	}
	return err;
}

/* Configures the internal settings of the DAC. */
static inline int dac_set_internal(dac_registers_t *dac_reg, uint8_t internal)
{
	int err = 0;

	/* Not supported */
	if (internal == 1) {
		err = -ENOTSUP;
	}
	return err;
}

/* Enables or disables buffered mode for the DAC. */
static inline int dac_set_buffered(dac_registers_t *dac_reg, uint8_t buffered)
{
	int err = 0;

	/* Not supported */
	if (buffered == 1) {
		err = -ENOTSUP;
	}
	return err;
}

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
	const dac_mchp_dev_config_t *dev_cfg = dev->config;
	dac_mchp_dev_data_t *const data = dev->data;
	int ret = 0;
	int8_t i = 0;

	do {
		/* Disable the Controller */
		dac_disable_controller(dev_cfg->regs);

		/* Check if resolution is valid */
		ret = dac_set_resolution(dev_cfg->regs, channel_cfg->resolution);
		if (ret != 0) {
			break;
		}
		/* Check if internal is valid */
		ret = dac_set_internal(dev_cfg->regs, channel_cfg->internal);
		if (ret != 0) {
			break;
		}

		/* Check if buffered is valid */
		ret = dac_set_buffered(dev_cfg->regs, channel_cfg->buffered);
		if (ret != 0) {
			break;
		}

		/* Validate channel ID here and only pass valid channels
		 * to avoid validation within the helper function.
		 */
		if ((channel_cfg->channel_id >= DAC_MAX_CHANNELS) &&
		    (channel_cfg->channel_id != DAC_CHANNELS_ALL)) {
			ret = -EINVAL;
			break;
		}

		/* Configure the DAC */
		ret = dac_configure(dev, channel_cfg->channel_id);
		if (ret != 0) {
			break;
		}
		/* Enable the DAC */
		dac_enable_controller(dev_cfg->regs);
		/* Wait till the DAC is ready for conversion after enabling the DAC */
		dac_wait_ready(dev_cfg->regs, channel_cfg->channel_id);
		if (channel_cfg->channel_id == DAC_CHANNELS_ALL) {
			for (i = 0; i < DAC_MAX_CHANNELS; i++) {
				data->is_configured[i] = true;
			}
		} else {
			data->is_configured[channel_cfg->channel_id] = true;
		}

	} while (0);

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
	dac_mchp_dev_data_t *const data = dev->data;
	int ret = 0;

	if (channel == DAC_CHANNELS_ALL) {
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
		dac_write_data(dev, channel, value);
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
	const dac_mchp_dev_config_t *dev_cfg = dev->config;
	int ret = 0;

	do {
		/* Enable the DAC Clock */
		ret = clock_control_on(dev_cfg->dac_clock.clock_dev, (dev_cfg->dac_clock.gclk_sys));
		if ((ret != 0) && (ret != -EALREADY)) {
			LOG_ERR("Failed to enable the GCLK for DAC: %d", ret);
			break;
		}

		ret = clock_control_on(dev_cfg->dac_clock.clock_dev, (dev_cfg->dac_clock.mclk_sys));
		if ((ret != 0) && (ret != -EALREADY)) {
			LOG_ERR("Failed to enable the MCLK for DAC: %d", ret);
			break;
		}

		ret = (ret == -EALREADY) ? 0 : ret;

		/* Apply pinctrl*/
		pinctrl_apply_state(dev_cfg->pcfg, PINCTRL_STATE_DEFAULT);

		/* Reset the DAC peripheral */
		dac_reset(dev_cfg->regs);

		/* Disable the DAC controller */
		dac_disable_controller(dev_cfg->regs);

		/* Set the DAC Operational Mode */
		dac_set_diff_output(dev_cfg->regs);

		/* Select the reference Voltage */
		dac_ref_selection(dev_cfg->regs, dev_cfg->refsel);
	} while (0);

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
 * @brief Macro to define individual DAC channel configuration from Device Tree.
 *
 * This macro is applied to each child node of a DAC instance. It initializes
 * channel-specific parameters such as channel ID, rate, filters, and dithering.
 *
 * @param child Device Tree node representing a DAC channel.
 *
 * - `channel`         : DAC channel number.
 * - `rate`            : Conversion rate setting.
 * - `ext_filter`      : Enable external filter (boolean).
 * - `data_adj`        : Data alignment/adjustment method (enum).
 * - `dither`          : Enable/disable dithering.
 * - `sampling_ratio`  : Sampling ratio for oversampling.
 * - `refresh`         : Refresh rate for DAC output.
 */
#define DAC_MCHP_CHANNEL_DEFN(child)                                                               \
	[DT_PROP(child, channel)] = {.channel = DT_PROP(child, channel),                           \
				     .rate = DT_PROP(child, rate),                                 \
				     .ext_filter = DT_PROP(child, ext_filter),                     \
				     .data_adj = DT_ENUM_IDX(child, data_adj),                     \
				     .dither = DT_PROP(child, dither),                             \
				     .sampling_ratio = DT_PROP(child, sampling_ratio),             \
				     .refresh = DT_PROP(child, refresh)}

/**
 * @brief Macro to define dac device configuration
 *
 * @param n Instance number
 */
/* clang-format off */
#define DAC_MCHP_CONFIG_DEFN(n)                                                                    \
	static const dac_mchp_dev_config_t dac_mchp_config_##n = {                                 \
		.regs = (dac_registers_t *)DT_INST_REG_ADDR(n),                                    \
		.refsel = DT_ENUM_IDX(DT_DRV_INST(n), refsel),                                     \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.channels = {DT_FOREACH_CHILD_SEP(DT_DRV_INST(n), DAC_MCHP_CHANNEL_DEFN, (,))},   \
		.dac_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                         \
		.dac_clock.mclk_sys = (void *)DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, subsystem),     \
		.dac_clock.gclk_sys = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, subsystem)),   \
	}
/* clang-format on */

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
