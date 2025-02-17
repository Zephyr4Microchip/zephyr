/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _I2C_MCHP_V1_H_
#define _I2C_MCHP_V1_H_

#if defined(CONFIG_SOC_SERIES_MCHP_SAME54)
/* Include rquired header files */

/* Define compatible sting */
#define DT_DRV_COMPAT microchip_sercom_u2201_i2c

/* DMA peripheral specific features */
#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
#define I2C_MCHP_DMA_CHANNELS(n)                                                                   \
	.hal.dma_dev = DEVICE_DT_GET(MICROCHIP_SAME54_DT_INST_DMA_CTLR(n, tx)),                    \
	.hal.write_dma_request = MICROCHIP_SAME54_DT_INST_DMA_TRIGSRC(n, tx),                      \
	.hal.read_dma_request = MICROCHIP_SAME54_DT_INST_DMA_TRIGSRC(n, rx),                       \
	.hal.tx_dma_channel = MICROCHIP_SAME54_DT_INST_DMA_CHANNEL(n, tx),                         \
	.hal.rx_dma_channel = MICROCHIP_SAME54_DT_INST_DMA_CHANNEL(n, rx)
#else
#define I2C_MCHP_DMA_CHANNELS(n)
#endif

/* Peripheral IP specific features */
#define I2C_MCHP_HAL_DEFN(n)                                                                       \
	.hal.regs = (sercom_registers_t *)DT_INST_REG_ADDR(n),                                     \
	.hal.mclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, mclk)),               \
			 .id = DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, id)},                          \
	.hal.gclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, gclk)),               \
			 .id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, id)}

/* Do the peripheral interrupt related configuration */
#if DT_INST_IRQ_HAS_IDX(0, 3)
#define I2C_MCHP_IRQ_HANDLER(n)                                                                    \
	static void i2c_mchp_irq_config_##n(const struct device *dev)                              \
	{                                                                                          \
		I2C_MCHP_IRQ_CONNECT(n, 0);                                                        \
		I2C_MCHP_IRQ_CONNECT(n, 1);                                                        \
		I2C_MCHP_IRQ_CONNECT(n, 2);                                                        \
		I2C_MCHP_IRQ_CONNECT(n, 3);                                                        \
	}
#else
#define I2C_MCHP_IRQ_HANDLER(n)                                                                    \
	static void i2c_mchp_irq_config_##n(const struct device *dev)                              \
	{                                                                                          \
		I2C_MCHP_IRQ_CONNECT(n, 0);                                                        \
	}
#endif

/* Enum representing the interrupt flags for the MCHP I2C peripheral */
enum i2c_mchp_intFlag {
	/* Indicates that the master is currently on the bus. */
	master_on_bus = 1,

	/* Indicates that the target is currently on the bus. */
	target_on_bus,

	/* Indicates an error condition in the I2C operation. */
	error = 128,

	/* Indicates that a STOP condition has been detected. */
	stop = 1,

	/* Indicates that an address match has occurred. */
	addr_match,

	/* Indicates that data is ready for transmission or reception. */
	data_ready = 4
};

/* Do the peripheral clock related configuration */
#define I2C_MCHP_GET_CLOCK_FREQ(dev, rate)                                                         \
	clock_control_get_rate(((const i2c_mchp_dev_config_t *)(dev->config))->clock_dev,          \
			       &(((i2c_mchp_dev_config_t *)(dev->config))->hal.gclk_sys), &rate);

#define I2C_MCHP_ENABLE_CLOCK(dev)                                                                 \
	clock_control_on(((const i2c_mchp_dev_config_t *)(dev->config))->clock_dev,                \
			 &(((i2c_mchp_dev_config_t *)(dev->config))->hal.gclk_sys));               \
	clock_control_on(((const i2c_mchp_dev_config_t *)(dev->config))->clock_dev,                \
			 &(((i2c_mchp_dev_config_t *)(dev->config))->hal.mclk_sys))

/* Hardware abstraction layer (HAL) structure for MCHP I2C peripheral */
struct hal_mchp_i2c {
	/* Pointer to the SERCOM peripheral registers. */
	sercom_registers_t *regs;

	/* Clock configuration for the main clock (MCLK) subsystem. */
	clock_control_mchp_subsys_t mclk_sys;

	/* Clock configuration for the generic clock (GCLK) subsystem. */
	clock_control_mchp_subsys_t gclk_sys;

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
	/* Pointer to the DMA controller device. */
	const struct device *dma_dev;

	/* DMA request line for I2C write operations. */
	uint8_t write_dma_request;

	/* DMA request line for I2C read operations. */
	uint8_t read_dma_request;

	/* DMA channel used for I2C operations. */
	uint8_t tx_dma_channel;
	uint8_t rx_dma_channel;
#endif
};

/* Include HAL file, specific to the peripheral IP */
#include "sercom_u2201/hal_mchp_i2c.h"

#endif

#endif /* _I2C_MCHP_V1_H_ */
