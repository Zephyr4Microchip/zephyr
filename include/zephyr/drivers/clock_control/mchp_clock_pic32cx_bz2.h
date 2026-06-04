/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file mchp_clock_pic32cx_bz2.h
 * @brief Clock control header file for Microchip pic32cx_bz2 family.
 *
 * This file provides clock driver interface definitions and structures
 * for pic32cx_bz2 family
 */

#ifndef MICROCHIP_MCHP_CLOCK_PIC32CX_BZ2_H_
#define MICROCHIP_MCHP_CLOCK_PIC32CX_BZ2_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/mchp_pic32cx_bz2_clock.h>

/** @brief Fast RC Clock Divider */
enum clock_mchp_sysclk_frc_div {
	CLOCK_MCHP_SYSCLK_FRC_DIV_1,
	CLOCK_MCHP_SYSCLK_FRC_DIV_2,
	CLOCK_MCHP_SYSCLK_FRC_DIV_4,
	CLOCK_MCHP_SYSCLK_FRC_DIV_8,
	CLOCK_MCHP_SYSCLK_FRC_DIV_16,
	CLOCK_MCHP_SYSCLK_FRC_DIV_32,
	CLOCK_MCHP_SYSCLK_FRC_DIV_64,
	CLOCK_MCHP_SYSCLK_FRC_DIV_256
};

/** @brief New Oscillator Selection for SYSCLK */
enum clock_mchp_sysclk_nosc {
	CLOCK_MCHP_SYSCLK_NOSC_FRCDIV,
	CLOCK_MCHP_SYSCLK_NOSC_SPLL1,
	CLOCK_MCHP_SYSCLK_NOSC_POSC,
	CLOCK_MCHP_SYSCLK_NOSC_SOSC,
	CLOCK_MCHP_SYSCLK_NOSC_LPRC
};

struct clock_mchp_subsys_sysclk_config {
	/** @brief Fast RC Clock Divider  [reg: CRU_OSCCON, bits: FRCDIV] */
	enum clock_mchp_sysclk_frc_div frc_div;

	/** @brief cNew Oscillator Selection for SYSCLK  [reg: CRU_OSCCON), bits: NOSC] */
	enum clock_mchp_sysclk_nosc new_osc;
};

/** @brief SPLL1 Post Divide Value configuration */
struct clock_mchp_subsys_spll1_config {
	/** @brief SPLL1 Post Divide Value (0 for 1, 1 for 1.5, 2-255)
	 *  [reg: CRU_SPLLCON, bits: SPLLPOSTDIV1]
	 */
	uint8_t post_div;
};

/** @brief Clock source for ADC Charge Pump */
enum clock_mchp_spll2_src {
	CLOCK_MCHP_SPLL2_SRC_SPLL3,
	CLOCK_MCHP_SPLL2_SRC_FRC,
	CLOCK_MCHP_SPLL2_SRC_POSC
};

struct clock_mchp_subsys_spll2_config {
	/** @brief Clock source for ADC Charge Pump [reg: SPLLCON, bits: SPLL_BYP] */
	enum clock_mchp_spll2_src src;

	/** @brief ADC-CP Post Divide Value (0 - 15) [reg: SPLLCON, bits: SPLLPOSTDIV2] */
	uint8_t post_div;
};

/** @brief Peripheral clock configuration */
struct clock_mchp_subsys_pbclk_config {
	/** @brief Peripheral Clock Divisor Control value (1 - 128) [reg: PBxDIV, bits: PBDIV] */
	uint8_t div;
};

/** @brief Reference Clock source oscillator Select */
enum clock_mchp_refclk_src {
	CLOCK_MCHP_REFCLK_SRC_FRC,
	CLOCK_MCHP_REFCLK_SRC_SPLL1,
	CLOCK_MCHP_REFCLK_SRC_POSC,
	CLOCK_MCHP_REFCLK_SRC_SOSC,
	CLOCK_MCHP_REFCLK_SRC_LPRC,
	CLOCK_MCHP_REFCLK_SRC_SPLL3,
	CLOCK_MCHP_REFCLK_SRC_PBCLK1,
	CLOCK_MCHP_REFCLK_SRC_SYSCLK,
	CLOCK_MCHP_REFCLK_SRC_REFIPIN
};

struct clock_mchp_subsys_refclk_config {
	/** @brief Reference Clock Divider (0 = no divider, 2*1 - 2*32767)
	 * [reg: REFOxCON, bits: RODIV]
	 */
	uint16_t div;

	/** @brief Stop peripheral in Idle Mode [reg: REFOxCON, bits: SIDL] */
	uint8_t stop_in_idle_en;

	/** @brief Continue to run oscillator in the Sleep mode [reg: REFOxCON, bits: RSLP] */
	uint8_t run_in_sleep_en;

	/** @brief Output the reference clock to REFOx pin [reg: REFOxCON, bits: OE] */
	uint8_t pin_out_en;

	/** @brief Reference Clock source oscillator Select [reg: REFOxCON, bits: ROSEL] */
	enum clock_mchp_refclk_src src_sel;

	/** @brief Provide fractional additive to refclk_div value (0- 511) /512
	 * [reg: REFOxTRIM, bits: ROTRIM]
	 */
	uint16_t trim_val;
};

/** @brief Peripheral clock source selection */
enum clock_mchp_gclkperiph_src {
	CLOCK_MCHP_GCLKPERIPH_SRC_NOCLK,
	CLOCK_MCHP_GCLKPERIPH_SRC_REFCLK_1,
	CLOCK_MCHP_GCLKPERIPH_SRC_REFCLK_2,
	CLOCK_MCHP_GCLKPERIPH_SRC_REFCLK_3,
	CLOCK_MCHP_GCLKPERIPH_SRC_REFCLK_4,
	CLOCK_MCHP_GCLKPERIPH_SRC_REFCLK_5,
	CLOCK_MCHP_GCLKPERIPH_SRC_REFCLK_6,
	CLOCK_MCHP_GCLKPERIPH_SRC_LPCLK
};

struct clock_mchp_subsys_gclkperiph_config {
	/** @brief Peripheral clock source delection [reg: CFGPCLKGENx, bits: xxxSEL] */
	enum clock_mchp_gclkperiph_src src_sel;
};

/** @brief WDT RUN Mode Clock Select */
enum clock_mchp_wdtclk_run_mode_clock {
	CLOCK_MCHP_WDTCLK_RUN_MOD_PBCLK,
	CLOCK_MCHP_WDTCLK_RUN_MOD_LPRC
};

struct clock_mchp_subsys_wdtclk_config {
	/** @brief WDT RUN Mode Clock Select [reg: CFGCON2, bits: WDTRMCS] */
	enum clock_mchp_wdtclk_run_mode_clock run_mode_clock_sel;
};

/** @brief VDDBUKPCORE 32 KHz Clock Source Selection */
enum clock_mchp_vbkpclk_src {
	CLOCK_MCHP_VBKPCLK_SRC_FRC,
	CLOCK_MCHP_VBKPCLK_SRC_POSC,
	CLOCK_MCHP_VBKPCLK_SRC_SOSC,
	CLOCK_MCHP_VBKPCLK_SRC_LPRC
};

struct clock_mchp_subsys_vbkpclk_config {
	/** @brief VDDBUKPCORE 32 KHz Clock Source Selection [reg: CFGCON4, bits: VBKP_32KCSEL] */
	enum clock_mchp_vbkpclk_src src_sel;
};

/** @brief Deep Sleep Watchdog Timer Reference Clock Select */
enum clock_mchp_dswdtclk_src {
	CLOCK_MCHP_DSWDTCLK_SRC_SOSC,
	CLOCK_MCHP_DSWDTCLK_SRC_LPRC
};

struct clock_mchp_subsys_dswdtclk_config {
	/** @brief Deep Sleep WDT Reference Clock Select [reg: CFGCON4, bits: DSWDTOSC] */
	enum clock_mchp_dswdtclk_src src_sel;
};

/** @brief LPCLK Modifier in Counter/Delay Mode */
enum clock_mchp_lpclk_div_modifier {
	CLOCK_MCHP_LPCLK_DIV_BY_1,
	CLOCK_MCHP_LPCLK_DIV_BY_1_024
};

struct clock_mchp_subsys_lpclk_config {
	/** @brief LPCLK Modifier in Counter/Delay Mode [reg: CFG_CFGCON4, bits: LPCLK_MOD] */
	enum clock_mchp_lpclk_div_modifier div_modifier;
};

/** @brief RTCC Counter Mode Clock Select */
enum clock_mchp_rtcclk_counter_mode {
	CLOCK_MCHP_RTCCLK_MODE_PROCESSED_32K,
	CLOCK_MCHP_RTCCLK_MODE_RAW_32K
};

/** @brief VDDBUKPCORE LPCLK Clock Divider Selection */
enum clock_mchp_rtcclk_vbkp_div {
	CLOCK_MCHP_RTCCLK_VBKP_DIV_BY_32,
	CLOCK_MCHP_RTCCLK_VBKP_DIV_BY_31_25
};

/** @brief VDDBUKPCORE LPCLK Clock Selection */
enum clock_mchp_rtcclk_vbkp_1k_32k_sel {
	CLOCK_MCHP_RTCCLK_VBKP_32KHZ,
	CLOCK_MCHP_RTCCLK_VBKP_DIV_BY_DIVSEL
};

struct clock_mchp_subsys_rtcclk_config {
	/** @brief RTC LPCLK counter mode clock select [reg: CFG_CFGCON4, bits: RTCNTM_CSEL] */
	enum clock_mchp_rtcclk_counter_mode counter_mode_sel;

	/** @brief RTC LPCLK div select [reg: CFG_CFGCON4, bits: VBKP_DIVSEL] */
	enum clock_mchp_rtcclk_vbkp_div vbkp_div_sel;

	/** @brief RTC LPCLK select 1K or 32K [reg: CFG_CFGCON4, bits: VBKP_1KCSEL] */
	enum clock_mchp_rtcclk_vbkp_1k_32k_sel vbkp_1k_32k_sel;
};

#endif /* MICROCHIP_MCHP_CLOCK_PIC32CX_BZ2_H_ */
