/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file mchp_clock_pic32cm_jh.h
 * @brief Clock control header file for Microchip pic32cm_jh family.
 *
 * This file provides clock driver interface definitions and structures
 * for pic32cm_jh family
 */

#ifndef INCLUDE_ZEPHYR_DRIVERS_CLOCK_CONTROL_MCHP_CLOCK_PIC32CM_JH_H_
#define INCLUDE_ZEPHYR_DRIVERS_CLOCK_CONTROL_MCHP_CLOCK_PIC32CM_JH_H_

#include <zephyr/dt-bindings/clock/mchp_pic32cm_jh_clock.h>

typedef struct {
	/** @brief configure oscillator to ON, when a peripheral is requesting it as a source */
	bool on_demand_en;

	/** @brief configure oscillator to ON in standby sleep mode, unless on_demand_en is set */
	bool run_in_standby_en;
} clock_mchp_subsys_xosc_config_t;

/** @brief Control the oscillator frequency range by adjusting the division ratio
 * @anchor clock_mchp_osc48m_divider_freq_t
 */
typedef enum {
	CLOCK_MCHP_DIVIDER_48_MHZ,
	CLOCK_MCHP_DIVIDER_24_MHZ,
	CLOCK_MCHP_DIVIDER_16_MHZ,
	CLOCK_MCHP_DIVIDER_12_MHZ,
	CLOCK_MCHP_DIVIDER_9_6_MHZ,
	CLOCK_MCHP_DIVIDER_8_MHZ,
	CLOCK_MCHP_DIVIDER_6_86_MHZ,
	CLOCK_MCHP_DIVIDER_6_MHZ,
	CLOCK_MCHP_DIVIDER_5_33_MHZ,
	CLOCK_MCHP_DIVIDER_4_8_MHZ,
	CLOCK_MCHP_DIVIDER_4_36_MHZ,
	CLOCK_MCHP_DIVIDER_4_MHZ,
	CLOCK_MCHP_DIVIDER_3_69_MHZ,
	CLOCK_MCHP_DIVIDER_3_43_MHZ,
	CLOCK_MCHP_DIVIDER_3_2_MHZ,
	CLOCK_MCHP_DIVIDER_3_MHZ
} clock_mchp_osc48m_divider_freq_t;

typedef struct {
	/** @brief configure oscillator to ON, when a peripheral is requesting it as a source */
	bool on_demand_en;

	/** @brief configure oscillator to ON in standby sleep mode, unless on_demand_en is set */
	bool run_in_standby_en;

	/** @brief Control the oscillator frequency range by adjusting the division ratio */
	clock_mchp_osc48m_divider_freq_t post_divider_freq;
} clock_mchp_subsys_osc48m_config_t;

/** @brief FDPLL source clocks
 * @anchor clock_mchp_fdpll_src_clock_t
 */
typedef enum {
	CLOCK_MCHP_FDPLL_SRC_GCLK0,
	CLOCK_MCHP_FDPLL_SRC_GCLK1,
	CLOCK_MCHP_FDPLL_SRC_GCLK2,
	CLOCK_MCHP_FDPLL_SRC_GCLK3,
	CLOCK_MCHP_FDPLL_SRC_GCLK4,
	CLOCK_MCHP_FDPLL_SRC_GCLK5,
	CLOCK_MCHP_FDPLL_SRC_GCLK6,
	CLOCK_MCHP_FDPLL_SRC_GCLK7,
	CLOCK_MCHP_FDPLL_SRC_GCLK8,
	CLOCK_MCHP_FDPLL_SRC_XOSC32K,
	CLOCK_MCHP_FDPLL_SRC_XOSC,

	CLOCK_MCHP_FDPLL_SRC_MAX = CLOCK_MCHP_FDPLL_SRC_XOSC
} clock_mchp_fdpll_src_clock_t;

typedef struct {
	/** @brief configure oscillator to ON, when a peripheral is requesting it as a source */
	bool on_demand_en;

	/** @brief configure oscillator to ON in standby sleep mode, unless on_demand_en is set */
	bool run_in_standby_en;

	/** @brief Set the fractional part of the frequency multiplier. (0 - 31) */
	uint32_t divider_ratio_frac;

	/** @brief Set the integer part of the frequency multiplier. (0 - 4095) */
	uint32_t divider_ratio_int;

	/** @brief Set the XOSC clock division factor (0 - 2047) */
	uint32_t xosc_clock_divider;

	/** @brief Reference source clock selection @see @ref clock_mchp_fdpll_src_clock_t */
	clock_mchp_fdpll_src_clock_t src;
} clock_mchp_subsys_fdpll_config_t;

/** @brief RTC source clocks
 * @anchor clock_mchp_rtc_src_clock_t
 */
typedef enum {
	CLOCK_MCHP_RTC_SRC_ULP1K = OSC32KCTRL_RTCCTRL_RTCSEL_ULP1K,
	CLOCK_MCHP_RTC_SRC_ULP32K = OSC32KCTRL_RTCCTRL_RTCSEL_ULP32K,
	CLOCK_MCHP_RTC_SRC_OSC1K = OSC32KCTRL_RTCCTRL_RTCSEL_OSC1K,
	CLOCK_MCHP_RTC_SRC_OSC32K = OSC32KCTRL_RTCCTRL_RTCSEL_OSC32K,
	CLOCK_MCHP_RTC_SRC_XOSC1K = OSC32KCTRL_RTCCTRL_RTCSEL_XOSC1K,
	CLOCK_MCHP_RTC_SRC_XOSC32K = OSC32KCTRL_RTCCTRL_RTCSEL_XOSC32K
} clock_mchp_rtc_src_clock_t;

typedef struct {
	/** @brief RTC source clock selection @see @ref clock_mchp_rtc_src_clock_t */
	clock_mchp_rtc_src_clock_t src;
} clock_mchp_subsys_rtc_config_t;

typedef struct {
	/** @brief configure oscillator to ON, when a peripheral is requesting it as a source */
	bool on_demand_en;

	/** @brief configure oscillator to ON in standby sleep mode, unless on_demand_en is set */
	bool run_in_standby_en;
} clock_mchp_subsys_xosc32k_config_t;

typedef struct {
	/** @brief configure oscillator to ON, when a peripheral is requesting it as a source */
	bool on_demand_en;

	/** @brief configure oscillator to ON in standby sleep mode, unless on_demand_en is set */
	bool run_in_standby_en;
} clock_mchp_subsys_osc32k_config_t;

/** @brief Gclk Generator source clocks
 * @anchor clock_mchp_gclk_src_clock_t
 */
typedef enum {
	CLOCK_MCHP_GCLK_SRC_XOSC,
	CLOCK_MCHP_GCLK_SRC_GCLKPIN,
	CLOCK_MCHP_GCLK_SRC_GCLKGEN1,
	CLOCK_MCHP_GCLK_SRC_OSCULP32K,
	CLOCK_MCHP_GCLK_SRC_OSC32K,
	CLOCK_MCHP_GCLK_SRC_XOSC32K,
	CLOCK_MCHP_GCLK_SRC_OSC48M,
	CLOCK_MCHP_GCLK_SRC_FDPLL,

	CLOCK_MCHP_GCLK_SRC_MAX = CLOCK_MCHP_GCLK_SRC_FDPLL
} clock_mchp_gclk_src_clock_t;

typedef struct {
	/** @brief Represent a division value for the corresponding Generator. The actual division
	 * factor is dependent on the state of div_select (gclk1 0 - 65535, others 0 - 255)
	 */
	uint16_t div_factor;

	/** @brief configure oscillator to ON in standby sleep mode, unless on_demand_en is set */
	bool run_in_standby_en;

	/** @brief Generator source clock selection @see @ref clock_mchp_gclk_src_clock_t */
	clock_mchp_gclk_src_clock_t src;
} clock_mchp_subsys_gclkgen_config_t;

/** @brief GCLK generator numbers
 * @anchor clock_mchp_gclkgen_t
 */
typedef enum {
	CLOCK_MCHP_GCLKGEN_GEN0,
	CLOCK_MCHP_GCLKGEN_GEN1,
	CLOCK_MCHP_GCLKGEN_GEN2,
	CLOCK_MCHP_GCLKGEN_GEN3,
	CLOCK_MCHP_GCLKGEN_GEN4,
	CLOCK_MCHP_GCLKGEN_GEN5,
	CLOCK_MCHP_GCLKGEN_GEN6,
	CLOCK_MCHP_GCLKGEN_GEN7,
	CLOCK_MCHP_GCLKGEN_GEN8
} clock_mchp_gclkgen_t;

typedef struct {
	/** @brief gclk generator source of a peripheral clock @see @ref clock_mchp_gclkgen_t*/
	clock_mchp_gclkgen_t src;
} clock_mchp_subsys_gclkperiph_config_t;

/** @brief division ratio of mclk prescaler for CPU
 * @anchor clock_mchp_mclk_cpu_div_t
 */
typedef enum {
	CLOCK_MCHP_MCLK_CPU_DIV_1 = 1,
	CLOCK_MCHP_MCLK_CPU_DIV_2 = 2,
	CLOCK_MCHP_MCLK_CPU_DIV_4 = 4,
	CLOCK_MCHP_MCLK_CPU_DIV_8 = 8,
	CLOCK_MCHP_MCLK_CPU_DIV_16 = 16,
	CLOCK_MCHP_MCLK_CPU_DIV_32 = 32,
	CLOCK_MCHP_MCLK_CPU_DIV_64 = 64,
	CLOCK_MCHP_MCLK_CPU_DIV_128 = 128
} clock_mchp_mclk_cpu_div_t;

/** @brief MCLK configuration structure
 *
 * Used for CLOCK_MCHP_SUBSYS_TYPE_MCLKCPU
 */
typedef struct {
	/** @brief division ratio of mclk prescaler for CPU @see @ref clock_mchp_mclk_cpu_div_t	 */
	clock_mchp_mclk_cpu_div_t division_factor;
} clock_mchp_subsys_mclkcpu_config_t;

/** @brief clock rate datatype
 *
 * Used for setting a clock rate
 */
typedef uint32_t *clock_mchp_rate_t;

#endif /* INCLUDE_ZEPHYR_DRIVERS_CLOCK_CONTROL_MCHP_CLOCK_PIC32CM_JH_H_ */
