/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SPI_MCHP_V1_H_
#define _SPI_MCHP_V1_H_

#if defined(CONFIG_SOC_SERIES_MCHP_SAME54)

/**
 * Define comaptible string
 */

#define DT_DRV_COMPAT microchip_sercom_u2201_spi

/**
 * Get the pads from device tree
 */

#define SPI_MCHP_SERCOM_PADS(n)                                                                    \
	SERCOM_SPIM_CTRLA_DIPO(DT_INST_PROP(n, dipo)) |                                            \
		SERCOM_SPIM_CTRLA_DOPO(DT_INST_PROP(n, dopo))

/**
 * Peripheral IP specific features
 */

#if CONFIG_SPI_ASYNC && CONFIG_SPI_MCHP_DMA_DRIVEN
#define SPI_MCHP_HAL_DEFN(n)                                                                       \
	.hal.regs = (sercom_spim_registers_t *)DT_INST_REG_ADDR(n),                                \
	.hal.regs = (sercom_spim_registers_t *)DT_INST_REG_ADDR(n),                                \
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
	.hal.mregs = (sercom_spim_registers_t *)DT_INST_REG_ADDR(n),                                \
	.hal.sregs = (sercom_spis_registers_t *)DT_INST_REG_ADDR(n),                                \
	.hal.pads = SPI_MCHP_SERCOM_PADS(n),                                                       \
	.hal.mclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, mclk)),               \
			 .id = DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, id)},                          \
	.hal.gclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, gclk)),               \
			 .id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, id)}
#endif

/**
 * Do the peripheral interrupt related configuration
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
 * Do the peripheral clock related configuration
 */
#define SPI_MCHP_GET_CLOCK_FREQ(dev, rate)                                                         \
	clock_control_get_rate(((const struct spi_mchp_dev_config_t *)(dev->config))->clock_dev,   \
			       &(((struct spi_mchp_dev_config_t *)(dev->config))->hal.gclk_sys),   \
			       &rate);

#define SPI_MCHP_ENABLE_CLOCK(dev)                                                                 \
	clock_control_on(((const struct spi_mchp_dev_config_t *)(dev->config))->clock_dev,         \
			 &(((struct spi_mchp_dev_config_t *)(dev->config))->hal.gclk_sys));        \
	clock_control_on(((const struct spi_mchp_dev_config_t *)(dev->config))->clock_dev,         \
			 &(((struct spi_mchp_dev_config_t *)(dev->config))->hal.mclk_sys))

/**
 * Peripheral configuration structure
 */
struct hal_mchp_spi_t {
	sercom_spim_registers_t *mregs;
	sercom_spis_registers_t *sregs;
	uint32_t pads;

	struct clock_control_mchp_subsys_t mclk_sys;
	struct clock_control_mchp_subsys_t gclk_sys;

#if SPI_MCHP_DMA_DRIVEN
	uint8_t tx_dma_request;
	uint8_t tx_dma_channel;
	uint8_t rx_dma_request;
	uint8_t rx_dma_channel;
#endif
};

/**
 * Include HAL file, Specific to the peripheral
 */
#include "sercom_u2201/hal_mchp_spi.h"

#elif defined(CONFIG_SOC_SERIES_SAM71)
/**
 * @brief Added Samv71 related Confguraaion here.
 */
#else
#define SPI_MCHP_HAL_DEFN(n)
#endif
