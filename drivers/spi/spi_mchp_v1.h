/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file spi_mchp_v1.h
 * @brief SPI driver configuration for Microchip devices.
 *
 * This file provides macro definitions, structures, and
 * function-like macros for configuring and
 * initializing the SPI peripheral on Microchip devices.
 */

#ifndef MICROCHIP_SPI_MCHP_V1_H_
#define MICROCHIP_SPI_MCHP_V1_H_

#if defined(CONFIG_SOC_FAMILY_MCHP_SAM_D5X_E5X)

/* Peripheral IP specific features */

/**
 * @brief Define compatible string for device tree.
 */
#define DT_DRV_COMPAT microchip_sercom_u2201_spi

/**
 * @brief Retrieve the SPI pads configuration from the device tree.
 *
 * @param n Instance number from the device tree.
 * @return Configured SERCOM SPI pads.
 */
#define SPI_MCHP_SERCOM_PADS(n)                                                                    \
	SERCOM_SPIM_CTRLA_DIPO(DT_INST_PROP(n, dipo)) |                                            \
		SERCOM_SPIM_CTRLA_DOPO(DT_INST_PROP(n, dopo))

/**
 * @brief SPI HAL configuration structure.
 *
 * This structure contains register mappings, clock configurations, and
 * optional DMA settings for the SPI peripheral.
 */
typedef struct hal_mchp_spi {
	/* Pointer to SPI master registers */
	sercom_registers_t *regs;

	/* SPI pad configuration */
	uint32_t pads;

} hal_mchp_spi_t;

/**
 * @brief Define peripheral IP-specific features
 * based on the selected configuration.
 *
 * If SPI is configured for asynchronous operation with DMA,
 * additional DMA parameters are included.
 * Otherwise, only standard register addresses
 *  and clock configurations are defined.
 */
#define SPI_MCHP_HAL_DEFN(n)                                                                       \
	.hal.regs = (sercom_registers_t *)DT_INST_REG_ADDR(n), .hal.pads = SPI_MCHP_SERCOM_PADS(n),

/**
 * @brief Define interrupt configuration macros based on device
 * capabilities.
 *
 * If the device has multiple interrupt indices, all of them are
 * configured. Otherwise, only a single interrupt is set up.
 */
/**
 * @brief Configure SPI interrupts for instance n.
 *
 * This function sets up the interrupt handlers for the SPI instance
 * specified by n.
 *
 * @param dev Pointer to the device structure for the driver instance.
 */
#if CONFIG_SPI_MCHP_INTERRUPT_DRIVEN || CONFIG_SPI_ASYNC
#if DT_INST_IRQ_HAS_IDX(0, 3)
#define SPI_MCHP_IRQ_HANDLER(n)                                                                    \
	static void spi_mchp_irq_config_##n(const struct device *dev)                              \
	{                                                                                          \
		MCHP_SPI_IRQ_CONNECT(n, 0);                                                        \
		MCHP_SPI_IRQ_CONNECT(n, 1);                                                        \
		MCHP_SPI_IRQ_CONNECT(n, 2);                                                        \
		MCHP_SPI_IRQ_CONNECT(n, 3);                                                        \
	}
#else
#define SPI_MCHP_IRQ_HANDLER(n)                                                                    \
	static void spi_mchp_irq_config_##n(const struct device *dev)                              \
	{                                                                                          \
		MCHP_SPI_IRQ_CONNECT(n, 0);                                                        \
	}
#endif
#else
#define SPI_MCHP_IRQ_HANDLER(n)
#endif

/* Do the peripheral clock related configuration */

/**
 * @brief Clock configuration structure for the SPI.
 *
 * This structure contains the clock configuration parameters for the SPI
 * peripheral.
 */
typedef struct mchp_spi_clock {
	/* Clock driver */
	const struct device *clock_dev;
	/* Main clock subsystem. */
	clock_control_subsys_t mclk_sys;
	/* Generic clock subsystem. */
	clock_control_subsys_t gclk_sys;
} mchp_spi_clock_t;

#define SPI_MCHP_CLOCK_DEFN(n)                                                                     \
	.spi_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                                 \
	.spi_clock.mclk_sys = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, subsystem)),           \
	.spi_clock.gclk_sys = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, subsystem))

/**
 * @brief Macros for handling SPI peripheral clock configuration.
 *
 * These macros retrieve the clock frequency and enable the necessary
 * clocks for the SPI peripheral.
 */
#define SPI_MCHP_GET_CLOCK_FREQ(dev, rate)                                                         \
	clock_control_get_rate(                                                                    \
		((const spi_mchp_dev_config_t *)(dev->config))->spi_clock.clock_dev,               \
		(((spi_mchp_dev_config_t *)(dev->config))->spi_clock.gclk_sys), &rate);

#define SPI_MCHP_ENABLE_CLOCK(dev)                                                                 \
	clock_control_on(((const spi_mchp_dev_config_t *)(dev->config))->spi_clock.clock_dev,      \
			 (((spi_mchp_dev_config_t *)(dev->config))->spi_clock.gclk_sys));         \
	clock_control_on(((const spi_mchp_dev_config_t *)(dev->config))->spi_clock.clock_dev,      \
			 (((spi_mchp_dev_config_t *)(dev->config))->spi_clock.mclk_sys))

/**
 * @brief Include HAL-specific implementation for the SPI peripheral.
 */
#include "sercom/hal_mchp_spi_sercom_u2201.h"

#endif /* CONFIG_SOC_FAMILY_MCHP_SAM_D5X_E5X */

#endif /* MICROCHIP_SPI_MCHP_V1_H_ */
