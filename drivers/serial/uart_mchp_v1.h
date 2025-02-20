/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file uart_mchp_v1.h
 * @brief UART driver configuration for Microchip SAME54 series.
 *
 * This file contains the configuration macros and structures for the UART
 * driver specific to the Microchip SAME54 series.
 */

#ifndef MICROCHIP_UART_MCHP_V1_H_
#define MICROCHIP_UART_MCHP_V1_H_

#if defined(CONFIG_SOC_SERIES_MCHP_SAME54)

/* Include required header files */

/* Define compatible string */
#define DT_DRV_COMPAT microchip_sercom_u2201_uart

/* Do the peripheral pin related configuration */
#define UART_MCHP_SERCOM_RXPO(n) (DT_INST_PROP(n, rxpo))
#define UART_MCHP_SERCOM_TXPO(n) (DT_INST_PROP(n, txpo))

/* Peripheral IP specific features */
#if CONFIG_UART_MCHP_ASYNC
/**
 * @brief Define the HAL configuration for asynchronous UART.
 *
 * This macro sets up the HAL configuration for the UART peripheral,
 * including register addresses, clock settings, and DMA channels.
 *
 * @param n Instance number.
 */
#define UART_MCHP_HAL_DEFN(n)                                                                      \
	.hal.regs = (sercom_registers_t *)DT_INST_REG_ADDR(n), .hal.is_clock_internal = true,      \
	.hal.rxpo = UART_MCHP_SERCOM_RXPO(n), .hal.txpo = UART_MCHP_SERCOM_TXPO(n),                \
	.hal.mclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, mclk)),               \
			 .id = DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, id)},                          \
	.hal.gclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, gclk)),               \
			 .id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, id)},                          \
	.hal.tx_dma_request = MICROCHIP_SAME54_DT_INST_DMA_TRIGSRC(n, tx),                         \
	.hal.tx_dma_channel = MICROCHIP_SAME54_DT_INST_DMA_CHANNEL(n, tx),                         \
	.hal.rx_dma_request = MICROCHIP_SAME54_DT_INST_DMA_TRIGSRC(n, rx),                         \
	.hal.rx_dma_channel = MICROCHIP_SAME54_DT_INST_DMA_CHANNEL(n, rx)
#else
/**
 * @brief Define the HAL configuration for synchronous UART.
 *
 * This macro sets up the HAL configuration for the UART peripheral,
 * including register addresses and clock settings.
 *
 * @param n Instance number.
 */
#define UART_MCHP_HAL_DEFN(n)                                                                      \
	.hal.regs = (sercom_registers_t *)DT_INST_REG_ADDR(n), .hal.is_clock_internal = true,      \
	.hal.rxpo = UART_MCHP_SERCOM_RXPO(n), .hal.txpo = UART_MCHP_SERCOM_TXPO(n),                \
	.hal.mclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, mclk)),               \
			 .id = DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, id)},                          \
	.hal.gclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, gclk)),               \
			 .id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, id)}
#endif

/* Do the peripheral interrupt related configuration */
#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_MCHP_ASYNC
#if DT_INST_IRQ_HAS_IDX(0, 3)
/**
 * @brief Configure UART IRQ handler for multiple interrupts.
 *
 * This macro sets up the IRQ handler for the UART peripheral when
 * multiple interrupts are available.
 *
 * @param n Instance number.
 */
#define UART_MCHP_IRQ_HANDLER(n)                                                                   \
	static void uart_mchp_irq_config_##n(const struct device *dev)                             \
	{                                                                                          \
		MCHP_UART_IRQ_CONNECT(n, 0);                                                       \
		MCHP_UART_IRQ_CONNECT(n, 1);                                                       \
		MCHP_UART_IRQ_CONNECT(n, 2);                                                       \
		MCHP_UART_IRQ_CONNECT(n, 3);                                                       \
	}
#else
/**
 * @brief Configure UART IRQ handler for a single interrupt.
 *
 * This macro sets up the IRQ handler for the UART peripheral when
 * only a single interrupt is available.
 *
 * @param n Instance number.
 */
#define UART_MCHP_IRQ_HANDLER(n)                                                                   \
	static void uart_mchp_irq_config_##n(const struct device *dev)                             \
	{                                                                                          \
		MCHP_UART_IRQ_CONNECT(n, 0);                                                       \
	}
#endif
#else
#define UART_MCHP_IRQ_HANDLER(n)
#endif

#if CONFIG_UART_MCHP_ASYNC
/**
 * @brief Configure UART DMA channels.
 *
 * This macro sets the DMA device configuration for UART based on the provided instance number.
 * It uses the DEVICE_DT_GET macro to get the DMA controller device from the device tree.
 *
 * @param n Instance number of the UART device.
 */
#define UART_MCHP_DMA_CHANNELS(n)                                                                  \
	.dma_dev = DEVICE_DT_GET(MICROCHIP_SAME54_DT_INST_DMA_CTLR(n, tx)),
#else
/**
 * @brief Macro to configure UART DMA channels.
 *
 * This macro is defined as empty when CONFIG_UART_MCHP_ASYNC is not defined.
 *
 * @param n Instance number of the UART device.
 */
#define UART_MCHP_DMA_CHANNELS(n)
#endif

/* Do the peripheral clock related configuration */
/**
 * @brief Get the clock frequency for the UART peripheral.
 *
 * This macro retrieves the clock frequency for the UART peripheral.
 *
 * @param dev Device structure.
 * @param rate Variable to store the clock rate.
 */
#define UART_MCHP_GET_CLOCK_FREQ(dev, rate)                                                        \
	clock_control_get_rate(((const uart_mchp_dev_cfg_t *)(dev->config))->clock_dev,            \
			       &(((uart_mchp_dev_data_t *)(dev->data))->hal.gclk_sys), &rate)

/**
 * @brief Enable the clock for the UART peripheral.
 *
 * This macro enables the clock for the UART peripheral.
 *
 * @param dev Device structure.
 */
#define UART_MCHP_ENABLE_CLOCK(dev)                                                                \
	clock_control_on(((const uart_mchp_dev_cfg_t *)(dev->config))->clock_dev,                  \
			 &(((uart_mchp_dev_data_t *)(dev->data))->hal.gclk_sys));                  \
	clock_control_on(((const uart_mchp_dev_cfg_t *)(dev->config))->clock_dev,                  \
			 &(((uart_mchp_dev_data_t *)(dev->data))->hal.mclk_sys))

/* Peripheral configuration structure */
/**
 * @brief HAL configuration structure for the UART peripheral.
 *
 * This structure contains the configuration parameters for the UART
 * peripheral, including register addresses, clock settings, and DMA channels.
 */
typedef struct hal_mchp_uart {
	/* Pointer to the SERCOM registers. */
	sercom_registers_t *regs;
	/* Flag indicating if the clock is internal. */
	bool is_clock_internal;
	/* RX pinout configuration. */
	uint32_t rxpo;
	/* TX pinout configuration. */
	uint32_t txpo;
	/* Collision detection flag. */
	bool collision_detect;

	/* Main clock subsystem. */
	clock_control_mchp_subsys_t mclk_sys;
	/* Generic clock subsystem. */
	clock_control_mchp_subsys_t gclk_sys;

#if CONFIG_UART_MCHP_ASYNC
	/* TX DMA request line. */
	uint8_t tx_dma_request;
	/* TX DMA channel. */
	uint8_t tx_dma_channel;
	/* RX DMA request line. */
	uint8_t rx_dma_request;
	/* RX DMA channel. */
	uint8_t rx_dma_channel;
#endif
} hal_mchp_uart_t;

/* Include HAL file, specific to the peripheral IP */
#include "sercom/hal_mchp_uart_sercom_u2201.h"

#endif /* CONFIG_SOC_SERIES_MCHP_SAME54 */

#endif /* MICROCHIP_UART_MCHP_V1_H_ */
