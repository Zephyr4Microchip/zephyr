/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MICROCHIP_I2C_MCHP_V1_H_
#define MICROCHIP_I2C_MCHP_V1_H_

#if defined(CONFIG_SOC_FAMILY_MCHP_SAM_D5X_E5X)
/* Include rquired header files */

/* Define compatible sting */
#define DT_DRV_COMPAT microchip_sercom_u2201_i2c

#define I2C_MCHP_HAL_DEFN(n) .hal.regs = (sercom_registers_t *)DT_INST_REG_ADDR(n),

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

typedef struct mchp_i2c_clock {
	/* Clock driver */
	const struct device *clock_dev;
	/* Main clock subsystem. */
	clock_control_subsys_t mclk_sys;
	/* Generic clock subsystem. */
	clock_control_subsys_t gclk_sys;
} mchp_i2c_clock_t;

#define I2C_MCHP_CLOCK_DEFN(n)                                                                     \
	.i2c_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                                 \
	.i2c_clock.mclk_sys = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, subsystem)),           \
	.i2c_clock.gclk_sys = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, subsystem)),

/* Do the peripheral clock related configuration */
#define I2C_MCHP_GET_CLOCK_FREQ(dev, rate)                                                         \
	clock_control_get_rate(                                                                    \
		((const i2c_mchp_dev_config_t *)(dev->config))->i2c_clock.clock_dev,               \
		(((i2c_mchp_dev_config_t *)(dev->config))->i2c_clock.gclk_sys), &rate)

#define I2C_MCHP_ENABLE_CLOCK(dev)                                                                 \
	clock_control_on(((const i2c_mchp_dev_config_t *)(dev->config))->i2c_clock.clock_dev,      \
			 (((i2c_mchp_dev_config_t *)(dev->config))->i2c_clock.gclk_sys));          \
	clock_control_on(((const i2c_mchp_dev_config_t *)(dev->config))->i2c_clock.clock_dev,      \
			 (((i2c_mchp_dev_config_t *)(dev->config))->i2c_clock.mclk_sys))

/* Hardware abstraction layer (HAL) structure for MCHP I2C peripheral */
typedef struct hal_mchp_i2c {
	/* Pointer to the SERCOM peripheral registers. */
	sercom_registers_t *regs;

} hal_mchp_i2c_t;

/* Include HAL file, specific to the peripheral IP */
#include "sercom/hal_mchp_i2c_sercom_u2201.h"

#endif

#endif /* MICROCHIP_I2C_MCHP_V1_H_ */
