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

#if defined(CONFIG_SOC_FAMILY_MCHP_SAM_D5X_E5X)

/* Define compatible string */
#define DT_DRV_COMPAT microchip_sercom_u2201_uart

/* Do the peripheral pin related configuration */
#define UART_MCHP_SERCOM_RXPO(n) (DT_INST_PROP(n, rxpo))
#define UART_MCHP_SERCOM_TXPO(n) (DT_INST_PROP(n, txpo))

/* Peripheral IP specific features */

/**
 * @brief HAL configuration structure for the UART peripheral.
 *
 * This structure contains the configuration parameters for the UART
 * peripheral, including register addresses.
 */
typedef struct hal_mchp_uart {
	/* Pointer to the SERCOM registers. */
	sercom_registers_t *regs;
	/* Flag indicating if the clock is external. */
	bool clock_external;
	/* RX pinout configuration. */
	uint32_t rxpo;
	/* TX pinout configuration. */
	uint32_t txpo;
} hal_mchp_uart_t;

/**
 * @brief Define the HAL configuration for UART.
 *
 * This macro sets up the HAL configuration for the UART peripheral
 *
 * @param n Instance number.
 */
#define UART_MCHP_HAL_DEFN(n)                                                                      \
	.hal.regs = (sercom_registers_t *)DT_INST_REG_ADDR(n),                                     \
	.hal.rxpo = UART_MCHP_SERCOM_RXPO(n), .hal.txpo = UART_MCHP_SERCOM_TXPO(n),                \
	.hal.clock_external = DT_INST_PROP(n, clock_external),

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

/* Do the peripheral clock related configuration */

/**
 * @brief Clock configuration structure for the UART.
 *
 * This structure contains the clock configuration parameters for the UART
 * peripheral.
 */
typedef struct mchp_uart_clock {
	/* Clock driver */
	const struct device *clock_dev;
	/* Main clock subsystem. */
	clock_control_subsys_t mclk_sys;
	/* Generic clock subsystem. */
	clock_control_subsys_t gclk_sys;
} mchp_uart_clock_t;

#define UART_MCHP_CLOCK_DEFN(n)                                                                    \
	.uart_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                                \
	.uart_clock.mclk_sys = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, subsystem)),          \
	.uart_clock.gclk_sys = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, subsystem)),

/**
 * @brief Get the clock frequency for the UART peripheral.
 *
 * This macro retrieves the clock frequency for the UART peripheral.
 *
 * @param dev Device structure.
 * @param rate Variable to store the clock rate.
 */
#define UART_MCHP_GET_CLOCK_FREQ(dev, rate)                                                        \
	clock_control_get_rate(((const uart_mchp_dev_cfg_t *)(dev->config))->uart_clock.clock_dev, \
			       (((uart_mchp_dev_cfg_t *)(dev->config))->uart_clock.gclk_sys),      \
			       &rate)

/**
 * @brief Enable the clock for the UART peripheral.
 *
 * This macro enables the clock for the UART peripheral.
 *
 * @param dev Device structure.
 */
#define UART_MCHP_ENABLE_CLOCK(dev)                                                                \
	clock_control_on(((const uart_mchp_dev_cfg_t *)(dev->config))->uart_clock.clock_dev,       \
			 ((uart_mchp_dev_cfg_t *)(dev->config))->uart_clock.gclk_sys);             \
	clock_control_on(((const uart_mchp_dev_cfg_t *)(dev->config))->uart_clock.clock_dev,       \
			 ((uart_mchp_dev_cfg_t *)(dev->config))->uart_clock.mclk_sys)

/* Include HAL file, specific to the peripheral IP */
#include "sercom/hal_mchp_uart_sercom_u2201.h"

#endif /* CONFIG_SOC_FAMILY_MCHP_SAM_D5X_E5X */

#endif /* MICROCHIP_UART_MCHP_V1_H_ */
