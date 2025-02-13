/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file clock_control_mchp_v1.h
 * @brief Clock control mapping driver for Microchip devices.
 *
 * This file provides definitions and structures for clock control functions
 * for Microchip-based systems.
 */

#ifndef DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_MCHP_V1_H_
#define DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_MCHP_V1_H_

#include <zephyr/drivers/clock_control.h>

/**
 * @brief Enum representing the states of the clock control.
 */
enum clock_control_mchp_state {
	/* Clock is in a valid and operational state */
	CLOCK_CONTROL_MCHP_STATE_OK = 0,
	/* Clock is turned off */
	CLOCK_CONTROL_MCHP_STATE_OFF,
	/* Clock is turned on */
	CLOCK_CONTROL_MCHP_STATE_ON,
	/* Clock is in the process of starting */
	CLOCK_CONTROL_MCHP_STATE_STARTING,
	/* Clock is not a valid source */
	CLOCK_CONTROL_MCHP_STATE_NOT_SOURCE,
	/* Clock has no rate specified */
	CLOCK_CONTROL_MCHP_STATE_NO_RATE,
	/* Clock already has the same rate as previous */
	CLOCK_CONTROL_MCHP_STATE_SAME_RATE,
	/* Clock does not support the requested operation */
	CLOCK_CONTROL_MCHP_STATE_NO_SUPPORT,
	/* Clock is having a user-defined frequency */
	CLOCK_CONTROL_MCHP_STATE_USER_FREQ,
};

/**
 * @brief Structure representing the source configuration for a clock control.
 */
struct clock_control_mchp_source {
	/* Division factor for the clock source */
	uint32_t div;
	/* Multiplication factor for the clock source */
	uint32_t mul;
	/* Fractional multiplication value */
	uint32_t mul_frac;
	/* Fractional resolution */
	uint8_t mul_frac_res;
	/* Frequency of the clock source */
	uint32_t frequency;
	/* Target clock identifier */
	uint32_t target_clk;
	/* Address of the target clock */
	uint32_t target_clk_addr;
	/* Identifier for the user-defined frequency */
	uint8_t user_freq_id;
};

/**
 * @brief Structure mapping the clock identifier to a mask.
 */
struct clock_control_mchp_msk_map {
	/* Clock identifier */
	uint32_t clk;
	/* Mask associated with the clock */
	uint32_t msk;
};

/**
 * @brief Structure mapping the clock identifier to a sync setting.
 */
struct clock_control_mchp_sync_map {
	/* Clock identifier */
	uint32_t clk;
	/* Synchronization setting */
	uint32_t sync;
};

/**
 * @brief Structure mapping the clock identifier to an ID.
 */
struct clock_control_mchp_id_map {
	/* Clock identifier */
	uint32_t clk;
	/* ID associated with the clock */
	uint32_t id;
};

/**
 * @brief Structure for managing interrupt flags and masks for the clock control.
 */
struct clock_control_mchp_id_int_map {
	/* Clock identifier */
	uint32_t clk;
	/* ID associated with the clock */
	uint32_t id;
	/* Mask to set interrupt flags */
	uint32_t set_msk;
	/* Mask to clear interrupt flags */
	uint32_t clr_msk;
	/* Flag mask for interrupt conditions */
	uint32_t flag_msk;
};

/**
 * @brief Structure mapping the clock identifier to registers and frequency.
 */
struct clock_control_mchp_src_map {
	/* Clock identifier */
	uint32_t clk;
	/* Registers associated with the clock */
	uint32_t regs;
	/* Frequency of the clock */
	uint32_t frequency;
};

/**
 * @brief Macro to connect and enable interrupts for clock control.
 * @param node The device tree node representing the interrupt source
 * @param idx The index of the interrupt
 */
#define CLOCK_CONTROL_MCHP_IRQ_CONNECT_ENABLE(node, idx)                                           \
	IRQ_CONNECT(DT_IRQ_BY_IDX(node, idx, irq), DT_IRQ_BY_IDX(node, idx, priority),             \
		    clock_control_mchp_isr, DEVICE_DT_GET(node), 0);                               \
	irq_enable(DT_IRQ_BY_IDX(node, idx, irq))

#if defined(CONFIG_SOC_SERIES_MCHP_SAME54)

/* Include required header files */
#include <dt-bindings/clock/sam/same54/mchp_same54_clock.h>

/* Peripheral IP HAL specific features */
/**
 * @brief Macro to set user frequencies from device tree properties
 */
#define CLOCK_CONTROL_MCHP_USER_FREQUENCY_DEFN                                                     \
	{                                                                                          \
		.dfll = DT_PROP_OR(DT_NODELABEL(oscctrl), dfll_frequency, 48000000),               \
		.xosc = DT_PROP_OR(DT_NODELABEL(oscctrl), xosc_frequency, 0),                      \
		.xosc32k = DT_PROP_OR(DT_NODELABEL(osc32kctrl), xosc32k_frequency, 0),             \
		.gclk_in = DT_PROP_OR(DT_NODELABEL(gclk), in_frequency, (0, 0, 0, 0, 0, 0, 0, 0)), \
	}

/**
 * @brief User-defined frequency constants for various clock sources
 */
enum clock_control_mchp_user_defined_frequency {
	/* No user-defined frequency */
	CLOCK_CONTROL_MCHP_USER_DEFINED_FREQUENCY_NONE = 0,
	/* DFLL frequency */
	CLOCK_CONTROL_MCHP_USER_DEFINED_FREQUENCY_DFLL,
	/* External Oscillator frequency */
	CLOCK_CONTROL_MCHP_USER_DEFINED_FREQUENCY_XOSC,
	/* 32KHz External Oscillator frequency */
	CLOCK_CONTROL_MCHP_USER_DEFINED_FREQUENCY_XOSC32K,
	/* GCLK input 0 frequency */
	CLOCK_CONTROL_MCHP_USER_DEFINED_FREQUENCY_GCLK_IN0,
	/* GCLK input 1 frequency */
	CLOCK_CONTROL_MCHP_USER_DEFINED_FREQUENCY_GCLK_IN1,
	/* GCLK input 2 frequency */
	CLOCK_CONTROL_MCHP_USER_DEFINED_FREQUENCY_GCLK_IN2,
	/* GCLK input 3 frequency */
	CLOCK_CONTROL_MCHP_USER_DEFINED_FREQUENCY_GCLK_IN3,
	/* GCLK input 4 frequency */
	CLOCK_CONTROL_MCHP_USER_DEFINED_FREQUENCY_GCLK_IN4,
	/* GCLK input 5 frequency */
	CLOCK_CONTROL_MCHP_USER_DEFINED_FREQUENCY_GCLK_IN5,
	/* GCLK input 6 frequency */
	CLOCK_CONTROL_MCHP_USER_DEFINED_FREQUENCY_GCLK_IN6,
	/* GCLK input 7 frequency */
	CLOCK_CONTROL_MCHP_USER_DEFINED_FREQUENCY_GCLK_IN7
};

/* Do the peripheral interrupt related configuration */
#define CLOCK_CONTROL_MCHP_IRQ_CONNECT_ENABLE_DEFN                                                 \
	CLOCK_CONTROL_MCHP_IRQ_CONNECT_ENABLE(DT_NODELABEL(mclk), 0);                              \
	CLOCK_CONTROL_MCHP_IRQ_CONNECT_ENABLE(DT_NODELABEL(oscctrl), 0);                           \
	CLOCK_CONTROL_MCHP_IRQ_CONNECT_ENABLE(DT_NODELABEL(oscctrl), 1);                           \
	CLOCK_CONTROL_MCHP_IRQ_CONNECT_ENABLE(DT_NODELABEL(oscctrl), 2);                           \
	CLOCK_CONTROL_MCHP_IRQ_CONNECT_ENABLE(DT_NODELABEL(oscctrl), 3);                           \
	CLOCK_CONTROL_MCHP_IRQ_CONNECT_ENABLE(DT_NODELABEL(oscctrl), 4);                           \
	CLOCK_CONTROL_MCHP_IRQ_CONNECT_ENABLE(DT_NODELABEL(osc32kctrl), 0)

/* Do the peripheral clock related configuration */
/**
 * @brief Structure representing user-defined frequency settings for various clock sources.
 */
struct clock_control_mchp_user_frequency {
	/* DFLL frequency setting */
	uint32_t dfll;
	/* External Oscillator frequency setting */
	uint32_t xosc;
	/* 32KHz External Oscillator frequency setting */
	uint32_t xosc32k;
	/* Array of GCLK input frequencies (up to 8 inputs) */
	uint32_t gclk_in[8];
};

/* Include HAL file, specific to the peripheral IP */
/* HAL for MCLK (Main Clock) */
#include <mclk_u2408/hal_mchp_mclk.h>
/* HAL for OSCCTRL (Oscillator Control) */
#include <oscctrl_u2401/hal_mchp_oscctrl.h>
/* HAL for OSC32KCTRL (32KHz Control) */
#include <osc32kctrl_u2400/hal_mchp_osc32kctrl.h>
/* HAL for GCLK (Generic Clock) */
#include <gclk_u2122/hal_mchp_gclk.h>
/* HAL for SAME54 Clock */
#include <clock_same54/hal_mchp_clock.h>

#endif /* CONFIG_SOC_SERIES_MCHP_SAME54 */

#endif /* DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_MCHP_V1_H_ */
