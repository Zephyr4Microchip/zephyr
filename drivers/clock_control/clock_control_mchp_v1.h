/**
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_MCHP_V1_H_
#define DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_MCHP_V1_H_

#include <zephyr/drivers/clock_control.h>

enum clock_control_mchp_state_t {
	CLOCK_CONTROL_MCHP_STATE_OK = 0,
	CLOCK_CONTROL_MCHP_STATE_OFF,
	CLOCK_CONTROL_MCHP_STATE_ON,
	CLOCK_CONTROL_MCHP_STATE_STARTING,
	CLOCK_CONTROL_MCHP_STATE_NOT_SOURCE,
	CLOCK_CONTROL_MCHP_STATE_NO_RATE,
	CLOCK_CONTROL_MCHP_STATE_SAME_RATE,
	CLOCK_CONTROL_MCHP_STATE_NO_SUPPORT,
	CLOCK_CONTROL_MCHP_STATE_USER_FREQ,
};

struct clock_control_mchp_src_t {
	uint32_t div;
	uint32_t mul;
	uint32_t mul_frac;
	uint8_t mul_frac_res;
	uint32_t frequency;
	uint32_t target_clk;
	uint32_t target_clk_addr;
	uint8_t user_freq_id;
};

struct clock_control_mchp_msk_map_t {
	uint32_t clk;
	uint32_t msk;
};

struct clock_control_mchp_sync_map_t {
	uint32_t clk;
	uint32_t sync;
};

struct clock_control_mchp_id_map_t {
	uint32_t clk;
	uint32_t id;
};

struct clock_control_mchp_id_int_map_t {
	uint32_t clk;
	uint32_t id;
	uint32_t set_msk;
	uint32_t clr_msk;
	uint32_t flag_msk;
};

struct clock_control_mchp_src_map_t {
	uint32_t clk;
	uint32_t regs;
	uint32_t frequency;
};

#define CLOCK_CONTROL_MCHP_IRQ_CONNECT_ENABLE(node, idx)                                           \
	IRQ_CONNECT(DT_IRQ_BY_IDX(node, idx, irq), DT_IRQ_BY_IDX(node, idx, priority),             \
		    clock_control_mchp_isr, DEVICE_DT_GET(node), 0);                               \
	irq_enable(DT_IRQ_BY_IDX(node, idx, irq))

#if defined(CONFIG_SOC_SERIES_MCHP_SAME54)
/* Include rquired header files */
#include <dt-bindings/clock/sam/same54/mchp_same54_clock.h>

/* Peripheral IP HAL specific features */
#define CLOCK_CONTROL_MCHP_USER_FREQUENCY_NONE     0
#define CLOCK_CONTROL_MCHP_USER_FREQUENCY_DFLL     1
#define CLOCK_CONTROL_MCHP_USER_FREQUENCY_XOSC     2
#define CLOCK_CONTROL_MCHP_USER_FREQUENCY_XOSC32K  3
#define CLOCK_CONTROL_MCHP_USER_FREQUENCY_GCLK_IN0 4
#define CLOCK_CONTROL_MCHP_USER_FREQUENCY_GCLK_IN1 5
#define CLOCK_CONTROL_MCHP_USER_FREQUENCY_GCLK_IN2 6
#define CLOCK_CONTROL_MCHP_USER_FREQUENCY_GCLK_IN3 7
#define CLOCK_CONTROL_MCHP_USER_FREQUENCY_GCLK_IN4 8
#define CLOCK_CONTROL_MCHP_USER_FREQUENCY_GCLK_IN5 9
#define CLOCK_CONTROL_MCHP_USER_FREQUENCY_GCLK_IN6 10
#define CLOCK_CONTROL_MCHP_USER_FREQUENCY_GCLK_IN7 11

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
struct clock_control_mchp_user_frequency_t {
	uint32_t dfll;
	uint32_t xosc;
	uint32_t xosc32k;
	uint32_t gclk_in[8];
};

/* Include HAL file, specific to the peripheral IP */
#include <mclk_u2408/hal_mchp_mclk.h>
#include <oscctrl_u2401/hal_mchp_oscctrl.h>
#include <osc32kctrl_u2400/hal_mchp_osc32kctrl.h>
#include <gclk_u2122/hal_mchp_gclk.h>
#include <clock_same54/hal_mchp_clock.h>

#endif /* CONFIG_SOC_SERIES_MCHP_SAME54 */

#endif /* DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_MCHP_V1_H_ */
