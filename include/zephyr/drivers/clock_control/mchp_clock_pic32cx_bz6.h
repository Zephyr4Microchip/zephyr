/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file mchp_clock_pic32cx_bz6.h
 * @brief Clock control header file for Microchip pic32cx_bz6 family.
 *
 * This file provides clock driver interface definitions and structures
 * for pic32cx_bz6 family
 */

#ifndef MICROCHIP_MCHP_CLOCK_PIC32CX_BZ6_H_
#define MICROCHIP_MCHP_CLOCK_PIC32CX_BZ6_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/mchp_pic32cx_bz6_clock.h>

/******************************************************************************
 * @brief Configuration datatypes for clock type: SUBSYS_TYPE_SYSCLK
 *****************************************************************************/
/** @brief Fast RC Clock Divider
 * @anchor clock_mchp_sysclk_frc_div_t
 */
typedef enum {
	CLOCK_MCHP_SYSCLK_FRC_DIV_1,
	CLOCK_MCHP_SYSCLK_FRC_DIV_2,
	CLOCK_MCHP_SYSCLK_FRC_DIV_4,
	CLOCK_MCHP_SYSCLK_FRC_DIV_8,
	CLOCK_MCHP_SYSCLK_FRC_DIV_16,
	CLOCK_MCHP_SYSCLK_FRC_DIV_32,
	CLOCK_MCHP_SYSCLK_FRC_DIV_64,
	CLOCK_MCHP_SYSCLK_FRC_DIV_256
} clock_mchp_sysclk_frc_div_t;

/** @brief New Oscillator Selection for SYSCLK
 * @anchor clock_mchp_sysclk_nosc_t
 */
typedef enum {
	CLOCK_MCHP_SYSCLK_NOSC_FRCDIV,
	CLOCK_MCHP_SYSCLK_NOSC_SPLL1,
	CLOCK_MCHP_SYSCLK_NOSC_POSC,
	CLOCK_MCHP_SYSCLK_NOSC_SOSC,
	CLOCK_MCHP_SYSCLK_NOSC_LPRC
} clock_mchp_sysclk_nosc_t;

typedef struct {
	/** @brief Fast RC Clock Divider  [reg: CRU_OSCCON, bits: FRCDIV] */
	clock_mchp_sysclk_frc_div_t frc_div;

	/** @brief cNew Oscillator Selection for SYSCLK  [reg: CRU_OSCCON), bits: NOSC] */
	clock_mchp_sysclk_nosc_t new_osc;
} clock_mchp_subsys_sysclk_config_t;

/******************************************************************************
 * @brief Configuration datatypes for clock type: SUBSYS_TYPE_SPLL1
 *****************************************************************************/
typedef struct {
	/** @brief SPLL1 Post Divide Value (0 and 1 for 1, 2-255)
	 *  [reg: CRU_SPLLCON, bits: SPLLPOSTDIV1]
	 */
	uint8_t post_div;
} clock_mchp_subsys_spll1_config_t;

/******************************************************************************
 * @brief Configuration datatypes for clock type: SUBSYS_TYPE_SPLL2
 *****************************************************************************/
/** @brief clock source for ADC Charge Pump
 * @anchor clock_mchp_spll2_src_t
 */
typedef enum {
	CLOCK_MCHP_SPLL2_SRC_SPLL3,
	CLOCK_MCHP_SPLL2_SRC_FRC,
	CLOCK_MCHP_SPLL2_SRC_POSC
} clock_mchp_spll2_src_t;

typedef struct {
	/** @brief clock source for ADC Charge Pump [reg: SPLLCON, bits: SPLL_BYP] */
	clock_mchp_spll2_src_t src;

	/** @brief ADC-CP Post Divide Value (0 - 15) [reg: SPLLCON, bits: SPLLPOSTDIV2] */
	uint8_t post_div;
} clock_mchp_subsys_spll2_config_t;

/******************************************************************************
 * @brief Configuration datatypes for clock type: SUBSYS_TYPE_PBCLK
 *****************************************************************************/
typedef struct {
	/** @brief Peripheral Clock Divisor Control value (1 - 128) [reg: PBxDIV, bits: PBDIV] */
	uint8_t div;
} clock_mchp_subsys_pbclk_config_t;

/******************************************************************************
 * @brief Configuration datatypes for clock type: SUBSYS_TYPE_REFCLK
 *****************************************************************************/
/** @brief Reference Clock source oscillator Select
 * @anchor clock_mchp_refclk_src_t
 */
typedef enum {
	CLOCK_MCHP_REFCLK_SRC_FRC,
	CLOCK_MCHP_REFCLK_SRC_SPLL1,
	CLOCK_MCHP_REFCLK_SRC_POSC,
	CLOCK_MCHP_REFCLK_SRC_SOSC,
	CLOCK_MCHP_REFCLK_SRC_LPRC,
	CLOCK_MCHP_REFCLK_SRC_EPLL2,
	CLOCK_MCHP_REFCLK_SRC_USBPLL,
	CLOCK_MCHP_REFCLK_SRC_SPLL3,
	CLOCK_MCHP_REFCLK_SRC_EPLL1,
	CLOCK_MCHP_REFCLK_SRC_PBCLK1,
	CLOCK_MCHP_REFCLK_SRC_SYSCLK,
	CLOCK_MCHP_REFCLK_SRC_REFIPIN
} clock_mchp_refclk_src_t;

typedef struct {
	/** @brief Reference Clock Divider (0 = no divider, 2*1 - 2*32767)
	 * [reg: REFOxCON, bits: RODIV]
	 */
	uint16_t div;

	/** @brief Stop peripheral in Idle Mode [reg: REFOxCON, bits: SIDL] */
	uint8_t stop_in_idle_en;

	/** @brief  Continue to run oscillator in the Sleep mode [reg: REFOxCON, bits: RSLP] */
	uint8_t run_in_sleep_en;

	/** @brief Output the reference clock to REFOx pin [reg: REFOxCON, bits: OE] */
	uint8_t pin_out_en;

	/** @brief Reference Clock source oscillator Select [reg: REFOxCON, bits: ROSEL] */
	clock_mchp_refclk_src_t src_sel;

	/** @brief Provide fractional additive to refclk_div value (0- 511) /512
	 * [reg: REFOxTRIM, bits: ROTRIM]
	 */
	uint16_t trim_val;
} clock_mchp_subsys_refclk_config_t;

/******************************************************************************
 * @brief Configuration datatypes for clock type: SUBSYS_TYPE_GCLKPERIPH
 *****************************************************************************/
/** @brief Peripheral clock source delection
 * @anchor clock_mchp_gclkperiph_src_t
 */
typedef enum {
	CLOCK_MCHP_GCLKPERIPH_SRC_NOCLK,
	CLOCK_MCHP_GCLKPERIPH_SRC_REFCLK_1,
	CLOCK_MCHP_GCLKPERIPH_SRC_REFCLK_2,
	CLOCK_MCHP_GCLKPERIPH_SRC_REFCLK_3,
	CLOCK_MCHP_GCLKPERIPH_SRC_REFCLK_4,
	CLOCK_MCHP_GCLKPERIPH_SRC_REFCLK_5,
	CLOCK_MCHP_GCLKPERIPH_SRC_REFCLK_6,
	CLOCK_MCHP_GCLKPERIPH_SRC_LPCLK
} clock_mchp_gclkperiph_src_t;

typedef struct {
	/** @brief Peripheral clock source delection [reg: CFGPCLKGENx, bits: xxxSEL] */
	clock_mchp_gclkperiph_src_t src_sel;
} clock_mchp_subsys_gclkperiph_config_t;

/******************************************************************************
 * @brief Configuration datatypes for clock type: SUBSYS_TYPE_WDTCLK
 *****************************************************************************/
/** @brief WDT RUN Mode Clock Select
 * @anchor clock_mchp_wdtclk_run_mode_clock_t
 */
typedef enum {
	CLOCK_MCHP_WDTCLK_RUN_MOD_PBCLK,
	CLOCK_MCHP_WDTCLK_RUN_MOD_LPRC
} clock_mchp_wdtclk_run_mode_clock_t;

typedef struct {
	/** @brief WDT RUN Mode Clock Select [reg: CFGCON2, bits: WDTRMCS] */
	clock_mchp_wdtclk_run_mode_clock_t run_mode_clock_sel;
} clock_mchp_subsys_wdtclk_config_t;

/******************************************************************************
 * @brief Configuration datatypes for clock type: SUBSYS_TYPE_VBKPCLK
 *****************************************************************************/
/** @brief VDDBUKPCORE 32 KHz Clock Source Selection
 * @anchor clock_mchp_vbkpclk_src_t
 */
typedef enum {
	CLOCK_MCHP_VBKPCLK_SRC_FRC,
	CLOCK_MCHP_VBKPCLK_SRC_POSC,
	CLOCK_MCHP_VBKPCLK_SRC_SOSC,
	CLOCK_MCHP_VBKPCLK_SRC_LPRC
} clock_mchp_vbkpclk_src_t;

typedef struct {
	/** @brief VDDBUKPCORE 32 KHz Clock Source Selection [reg: CFGCON4, bits: VBKP_32KCSEL] */
	clock_mchp_vbkpclk_src_t src_sel;
} clock_mchp_subsys_vbkpclk_config_t;

/******************************************************************************
 * @brief Configuration datatypes for clock type: SUBSYS_TYPE_DSWDTCLK
 *****************************************************************************/
/** @brief Deep Sleep Watchdog Timer Reference Clock Select
 * @anchor clock_mchp_dswdtclk_src_t
 */
typedef enum {
	CLOCK_MCHP_DSWDTCLK_SRC_SOSC,
	CLOCK_MCHP_DSWDTCLK_SRC_LPRC
} clock_mchp_dswdtclk_src_t;

typedef struct {
	/** @brief Deep Sleep WDT Reference Clock Select [reg: CFGCON4, bits: DSWDTOSC] */
	clock_mchp_dswdtclk_src_t src_sel;
} clock_mchp_subsys_dswdtclk_config_t;

/******************************************************************************
 * @brief Configuration datatypes for clock type: SUBSYS_TYPE_LPCLK
 *****************************************************************************/
/** @brief LPCLK Modifier in Counter/Delay Mode
 * @anchor clock_mchp_lpclk_div_modifier_t
 */
typedef enum {
	CLOCK_MCHP_LPCLK_DIV_BY_1,
	CLOCK_MCHP_LPCLK_DIV_BY_1_024
} clock_mchp_lpclk_div_modifier_t;

typedef struct {
	/** @brief LPCLK Modifier in Counter/Delay Mode [reg: CFG_CFGCON4, bits: LPCLK_MOD] */
	clock_mchp_lpclk_div_modifier_t div_modifier;
} clock_mchp_subsys_lpclk_config_t;

/******************************************************************************
 * @brief Configuration datatypes for clock type: SUBSYS_TYPE_RTCCLK
 *****************************************************************************/
/** @brief RTCC Counter Mode Clock Select
 * @anchor clock_mchp_rtcclk_counter_mode_t
 */
typedef enum {
	CLOCK_MCHP_RTCCLK_MODE_PROCESSED_32K,
	CLOCK_MCHP_RTCCLK_MODE_RAW_32K
} clock_mchp_rtcclk_counter_mode_t;

/** @brief VDDBUKPCORE LPCLK Clock Divider Selection
 * @anchor clock_mchp_rtcclk_vbkp_div_t
 */
typedef enum {
	CLOCK_MCHP_RTCCLK_VBKP_DIV_BY_32,
	CLOCK_MCHP_RTCCLK_VBKP_DIV_BY_31_25
} clock_mchp_rtcclk_vbkp_div_t;

/** @brief VDDBUKPCORE LPCLK Clock Selection
 * @anchor clock_mchp_rtcclk_vbkp_1k_32k_sel_t
 */
typedef enum {
	CLOCK_MCHP_RTCCLK_VBKP_32KHZ,
	CLOCK_MCHP_RTCCLK_VBKP_DIV_BY_DIVSEL
} clock_mchp_rtcclk_vbkp_1k_32k_sel_t;

typedef struct {
	/** @brief RTC LPCLK counter mode clock select [reg: CFG_CFGCON4, bits: RTCNTM_CSEL] */
	clock_mchp_rtcclk_counter_mode_t counter_mode_sel;

	/** @brief RTC LPCLK div select [reg: CFG_CFGCON4, bits: VBKP_DIVSEL] */
	clock_mchp_rtcclk_vbkp_div_t vbkp_div_sel;

	/** @brief RTC LPCLK select 1K or 32K [reg: CFG_CFGCON4, bits: VBKP_1KCSEL] */
	clock_mchp_rtcclk_vbkp_1k_32k_sel_t vbkp_1k_32k_sel;
} clock_mchp_subsys_rtcclk_config_t;

#endif /* MICROCHIP_MCHP_CLOCK_PIC32CX_BZ6_H_ */
