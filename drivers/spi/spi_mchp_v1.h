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

#ifndef _SPI_MCHP_V1_H_
#define _SPI_MCHP_V1_H_

#if defined(CONFIG_SOC_SERIES_MCHP_SAME54)

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
 * @brief Define peripheral IP-specific features
 * based on the selected configuration.
 *
 * If SPI is configured for asynchronous operation with DMA,
 * additional DMA parameters are included.
 * Otherwise, only standard register addresses
 *  and clock configurations are defined.
 */
#if CONFIG_SPI_ASYNC && CONFIG_SPI_MCHP_DMA_DRIVEN
#define SPI_MCHP_HAL_DEFN(n)                                                                       \
	.hal.mregs = (sercom_spim_registers_t *)DT_INST_REG_ADDR(n),                               \
	.hal.sregs = (sercom_spis_registers_t *)DT_INST_REG_ADDR(n),                               \
	.hal.pads = SPI_MCHP_SERCOM_PADS(n),                                                       \
	.hal.mclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, mclk)),               \
			 .id = DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, id)},                          \
	.hal.gclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, gclk)),               \
			 .id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, id)},                          \
	.hal.tx_dma_request = MICROCHIP_SAME54_DT_INST_DMA_TRIGSRC(n, tx),                         \
	.hal.tx_dma_channel = MICROCHIP_SAME54_DT_INST_DMA_CHANNEL(n, tx),                         \
	.hal.rx_dma_request = MICROCHIP_SAME54_DT_INST_DMA_TRIGSRC(n, rx),                         \
	.hal.rx_dma_channel = MICROCHIP_SAME54_DT_INST_DMA_CHANNEL(n, rx)
#else
#define SPI_MCHP_HAL_DEFN(n)                                                                       \
	.hal.mregs = (sercom_spim_registers_t *)DT_INST_REG_ADDR(n),                               \
	.hal.sregs = (sercom_spis_registers_t *)DT_INST_REG_ADDR(n),                               \
	.hal.pads = SPI_MCHP_SERCOM_PADS(n),                                                       \
	.hal.mclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, mclk)),               \
			 .id = DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, id)},                          \
	.hal.gclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, gclk)),               \
			 .id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, id)}
#endif

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

/**
 * @brief Macros for handling SPI peripheral clock configuration.
 *
 * These macros retrieve the clock frequency and enable the necessary
 * clocks for the SPI peripheral.
 */
#define SPI_MCHP_GET_CLOCK_FREQ(dev, rate)                                                         \
	clock_control_get_rate(((const struct spi_mchp_dev_config *)(dev->config))->clock_dev,     \
			       &(((struct spi_mchp_dev_config *)(dev->config))->hal.gclk_sys),     \
			       &rate);

#define SPI_MCHP_ENABLE_CLOCK(dev)                                                                 \
	clock_control_on(((const struct spi_mchp_dev_config *)(dev->config))->clock_dev,           \
			 &(((struct spi_mchp_dev_config *)(dev->config))->hal.gclk_sys));          \
	clock_control_on(((const struct spi_mchp_dev_config *)(dev->config))->clock_dev,           \
			 &(((struct spi_mchp_dev_config *)(dev->config))->hal.mclk_sys))

/**
 * @brief SPI HAL configuration structure.
 *
 * This structure contains register mappings, clock configurations, and
 * optional DMA settings for the SPI peripheral.
 */
struct hal_mchp_spi {
	/* Pointer to SPI master registers */
	sercom_spim_registers_t *mregs;

	/* Pointer to SPI slave registers */
	sercom_spis_registers_t *sregs;

	/* SPI pad configuration */
	uint32_t pads;

	/* MCLK subsystem configuration */
	clock_control_mchp_subsys_t mclk_sys;

	/* GCLK subsystem configuration */
	clock_control_mchp_subsys_t gclk_sys;

#if SPI_MCHP_DMA_DRIVEN
	/* TX DMA request line */
	uint8_t tx_dma_request;

	/* TX DMA channel */
	uint8_t tx_dma_channel;

	/* RX DMA request line */
	uint8_t rx_dma_request;

	/* RX DMA channel */
	uint8_t rx_dma_channel;
#endif
};

/**
 * @brief Include HAL-specific implementation for the SPI peripheral.
 */
#include "sercom/hal_mchp_spi_sercom_u2201.h"

#elif defined(CONFIG_SOC_SERIES_SAM71)
/**
 * @brief Add SAMV71 related SPI configuration here.
 */
#else
#define SPI_MCHP_HAL_DEFN(n)
#endif

#endif /* MICROCHIP_HAL_SPI_SERCOM_U2201_H_ */
