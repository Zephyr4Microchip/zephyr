/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file dac_mchp_v1.h
 * @brief Generic DAC Driver for Microchip MCUs.
 *
 * This file provides macro definitions, structures, and
 * function-like macros for configuring and
 * initializing the DAC peripheral on Microchip devices.
 */

#ifndef MICROCHIP_DAC_MCHP_V1_H_
#define MICROCHIP_DAC_MCHP_V1_H_

#if defined(CONFIG_SOC_FAMILY_MCHP_SAM_D5X_E5X)

/* Peripheral IP specific features */

/**
 * @brief Define compatible string for device tree.
 */
#define DT_DRV_COMPAT microchip_dac_u2502

/**
 * @brief Maximum number of DAC channels supported by the device.
 *
 * This macro extracts the `max_channels` property from the DAC node in the
 * Device Tree (referenced via the `dac` node label). It defines the upper limit
 * for the number of DAC channels that the driver should handle.
 *
 * @note The `max_channels` property must be defined in the `dac` node of the Device Tree.
 */
#define DAC_MAX_CHANNELS DT_PROP(DT_NODELABEL(dac), max_channels)

/**
 * @brief IP specific configuration parameters
 *
 * This structure holds the IP specific configuration parameters for the DAC
 * device.
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
 * @brief HAL structure for Microchip DAC.
 *
 * This structure provides a hardware abstraction layer (HAL) for the
 * Microchip Digital-to-Analog Converter (DAC). It contains a pointer
 * to the DAC registers.
 */
typedef struct hal_mchp_dac {
	/** Pointer to DAC registers */
	dac_registers_t *regs;

	/* Reference voltage selection */
	uint8_t refsel;

	/* Channel configurations */
	dac_mchp_channel_t channels[DAC_MAX_CHANNELS];

} hal_mchp_dac_t;

/**
 * @brief Macro to define HAL-level DAC peripheral configuration for an instance.
 *
 * This sets the base register address and the reference voltage selection
 * using values from the Device Tree.
 *
 * @param n Device instance number.
 *
 * - `regs`    : Pointer to the DAC hardware registers base.
 * - `refsel`  : Reference voltage selection index (enum) from Device Tree.
 */
#define DAC_MCHP_HAL_DEFN(n)                                                                       \
	.hal.regs = (dac_registers_t *)DT_INST_REG_ADDR(n),                                        \
	.hal.refsel = DT_ENUM_IDX(DT_DRV_INST(n), refsel),

/**
 * @brief Macro to define all DAC channel configurations for a given instance.
 *
 * Iterates over all children nodes of a DAC instance in the Device Tree and
 * applies `DAC_MCHP_CHANNEL_DEFN` to define each channel's configuration.
 *
 * @param n Device instance number.
 *
 * - Expands into `.hal.channels = { ... }` initializer.
 */
/* clang-format off */
#define DAC_MCHP_CHANNEL_CONFIG(n)                                                                 \
	.hal.channels = {DT_FOREACH_CHILD_SEP(DT_DRV_INST(n), DAC_MCHP_CHANNEL_DEFN, (,))},
/* clang-format on */

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
 * @brief Clock configuration structure for the DAC.
 *
 * This structure contains the clock configuration parameters for the DAC
 * peripheral.
 */
typedef struct mchp_dac_clock {

	/* Clock driver */
	const struct device *clock_dev;

	/* Main clock subsystem. */
	clock_control_mchp_subsys_t mclk_sys;

	/* Generic clock subsystem. */
	clock_control_mchp_subsys_t gclk_sys;

} mchp_dac_clock_t;

/**
 * @brief Macro to define the DAC clock configuration for a given device instance.
 *
 * This macro initializes the clock configuration structure used by the DAC driver.
 * It sets up references to the clock devices and their respective IDs as defined in
 * the device tree.
 *
 * @param n Device instance number used to extract clock configuration from device tree.
 *
 * - `dac_clock.clock_dev`  : Points to the clock controller node (`clock` label).
 * - `dac_clock.mclk_sys`   : Main clock controller and ID obtained using name "mclk".
 * - `dac_clock.gclk_sys`   : Generic clock controller and ID obtained using name "gclk".
 *
 * @note Requires the device tree to define clocks using named clock cells `mclk` and `gclk`.
 */
#define DAC_MCHP_CLOCK_DEFN(n)                                                                     \
	.dac_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                                 \
	.dac_clock.mclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, mclk)),         \
			       .id = DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, id)},                    \
	.dac_clock.gclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, gclk)),         \
			       .id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, id)}

/**
 * @brief Macros for handling DAC peripheral clock configuration.
 *
 * These macros retrieve the clock frequency and enable the necessary
 * clocks for the DAC peripheral.
 */
#define DAC_MCHP_ENABLE_CLOCK(dev)                                                                 \
	clock_control_on(((const dac_mchp_dev_config_t *)(dev->config))->dac_clock.clock_dev,      \
			 &(((dac_mchp_dev_config_t *)(dev->config))->dac_clock.gclk_sys));         \
	clock_control_on(((const dac_mchp_dev_config_t *)(dev->config))->dac_clock.clock_dev,      \
			 &(((dac_mchp_dev_config_t *)(dev->config))->dac_clock.mclk_sys))

#include "dac/hal_mchp_dac_u2502.h"

#endif

#endif /* MICROCHIP_DAC_MCHP_V1_H_ */
