/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file clock_control_mchp_v1.h
 * @brief Clock control header file for Microchip devices.
 *
 * This file provides definitions and structures for clock control functions
 * for Microchip-based systems.
 */

#ifndef MICROCHIP_CLOCK_CONTROL_MCHP_V1_H_
#define MICROCHIP_CLOCK_CONTROL_MCHP_V1_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>

/** @brief MCLK AHB clock count */
#define CLOCK_CONTROL_MCHP_MCLK_AHB_COUNT     20
/** @brief MCLK APBA clock count */
#define CLOCK_CONTROL_MCHP_MCLK_APBA_COUNT    16
/** @brief MCLK APBB clock count */
#define CLOCK_CONTROL_MCHP_MCLK_APBB_COUNT    12
/** @brief MCLK APBC clock count */
#define CLOCK_CONTROL_MCHP_MCLK_APBC_COUNT    12
/** @brief MCLK APBD clock count */
#define CLOCK_CONTROL_MCHP_MCLK_APBD_COUNT    12
/** @brief OSCCTRL XOSCCTRL count */
#define CLOCK_CONTROL_MCHP_OSCCTRL_XOSC_COUNT 2
/** @brief OSCCTRL DPLLCTRLA count */
#define CLOCK_CONTROL_MCHP_OSCCTRL_DPLL_COUNT 2
/** @brief GCLK GENCTRL count */
#define CLOCK_CONTROL_MCHP_GCLK_GEN_COUNT     12
/** @brief GCLK PCHCTRL count */
#define CLOCK_CONTROL_MCHP_GCLK_PCH_COUNT     48

/** @brief Clocks handled by the Clock controllers.
 *
 * Structure which can be used as a sys argument in the clock_control API.
 * This structure defines the clock controller subsystem device and the
 * associated clock ID. It can be passed as part of the `sys` argument in
 * the `clock_control` API functions.
 */
typedef struct {
	/** @brief Device associated with clock controller subsystem device. */
	const struct device *dev;
	/** @brief ID of the clock to control for this subsystem. */
	uint32_t id;
} clock_control_mchp_subsys_t;

#if defined(CONFIG_SOC_SERIES_MCHP_SAME54)
/* Include required header files for the SAME54 series clock definitions */
#include <dt-bindings/clock/sam/same54/mchp_same54_clock.h>

/** @brief Clocks handled by the MCLK peripheral.
 *
 * This can be used as id in clock_control_mchp_subsys_t clock_control API.
 */
typedef enum {
	/** @brief MCLK CPU Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_CPU = CLOCK_MCHP_SAME54_MCLK_CPU,
	/** @brief MCLK High-Speed Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_HS = CLOCK_MCHP_SAME54_MCLK_HS,

	/** @brief MCLK AHB HPB0 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_AHB_HPB0 = CLOCK_MCHP_SAME54_MCLK_AHB_HPB0,
	/** @brief MCLK AHB HPB1 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_AHB_HPB1 = CLOCK_MCHP_SAME54_MCLK_AHB_HPB1,
	/** @brief MCLK AHB HPB2 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_AHB_HPB2 = CLOCK_MCHP_SAME54_MCLK_AHB_HPB2,
	/** @brief MCLK AHB HPB3 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_AHB_HPB3 = CLOCK_MCHP_SAME54_MCLK_AHB_HPB3,
	/** @brief MCLK AHB DSU Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_AHB_DSU = CLOCK_MCHP_SAME54_MCLK_AHB_DSU,
	/** @brief MCLK AHB NVMCTRL Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_AHB_NVMCTRL = CLOCK_MCHP_SAME54_MCLK_AHB_NVMCTRL,
	/** @brief MCLK AHB CMCC Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_AHB_CMCC = CLOCK_MCHP_SAME54_MCLK_AHB_CMCC,
	/** @brief MCLK AHB DMAC Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_AHB_DMAC = CLOCK_MCHP_SAME54_MCLK_AHB_DMAC,
	/** @brief MCLK AHB USB Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_AHB_USB = CLOCK_MCHP_SAME54_MCLK_AHB_USB,
	/** @brief MCLK AHB PAC Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_AHB_PAC = CLOCK_MCHP_SAME54_MCLK_AHB_PAC,
	/** @brief MCLK AHB QSPI Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_AHB_QSPI = CLOCK_MCHP_SAME54_MCLK_AHB_QSPI,
	/** @brief MCLK AHB GMAC Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_AHB_GMAC = CLOCK_MCHP_SAME54_MCLK_AHB_GMAC,
	/** @brief MCLK AHB SDHC0 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_AHB_SDHC0 = CLOCK_MCHP_SAME54_MCLK_AHB_SDHC0,
	/** @brief MCLK AHB SDHC1 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_AHB_SDHC1 = CLOCK_MCHP_SAME54_MCLK_AHB_SDHC1,
	/** @brief MCLK AHB CAN0 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_AHB_CAN0 = CLOCK_MCHP_SAME54_MCLK_AHB_CAN0,
	/** @brief MCLK AHB CAN1 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_AHB_CAN1 = CLOCK_MCHP_SAME54_MCLK_AHB_CAN1,
	/** @brief MCLK AHB PUKCC Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_AHB_PUKCC = CLOCK_MCHP_SAME54_MCLK_AHB_PUKCC,
	/** @brief MCLK AHB QSPI 2X Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_AHB_QSPI_2X = CLOCK_MCHP_SAME54_MCLK_AHB_QSPI_2X,
	/** @brief MCLK AHB NVMCTRL SMEEPROM Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_AHB_NVMCTRL_SMEEPROM =
		CLOCK_MCHP_SAME54_MCLK_AHB_NVMCTRL_SMEEPROM,
	/** @brief MCLK AHB NVMCTRL CACHE Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_AHB_NVMCTRL_CACHE = CLOCK_MCHP_SAME54_MCLK_AHB_NVMCTRL_CACHE,

	/** @brief MCLK APBA PAC Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBA_PAC = CLOCK_MCHP_SAME54_MCLK_APBA_PAC,
	/** @brief MCLK APBA PM Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBA_PM = CLOCK_MCHP_SAME54_MCLK_APBA_PM,
	/** @brief MCLK APBA MCLK Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBA_MCLK = CLOCK_MCHP_SAME54_MCLK_APBA_MCLK,
	/** @brief MCLK APBA RSTC Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBA_RSTC = CLOCK_MCHP_SAME54_MCLK_APBA_RSTC,
	/** @brief MCLK APBA OSCCTRL Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBA_OSCCTRL = CLOCK_MCHP_SAME54_MCLK_APBA_OSCCTRL,
	/** @brief MCLK APBA OSC32KCTRL Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBA_OSC32KCTRL = CLOCK_MCHP_SAME54_MCLK_APBA_OSC32KCTRL,
	/** @brief MCLK APBA SUPC Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBA_SUPC = CLOCK_MCHP_SAME54_MCLK_APBA_SUPC,
	/** @brief MCLK APBA GCLK Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBA_GCLK = CLOCK_MCHP_SAME54_MCLK_APBA_GCLK,
	/** @brief MCLK APBA WDT Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBA_WDT = CLOCK_MCHP_SAME54_MCLK_APBA_WDT,
	/** @brief MCLK APBA RTC Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBA_RTC = CLOCK_MCHP_SAME54_MCLK_APBA_RTC,
	/** @brief MCLK APBA EIC Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBA_EIC = CLOCK_MCHP_SAME54_MCLK_APBA_EIC,
	/** @brief MCLK APBA FREQM Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBA_FREQM = CLOCK_MCHP_SAME54_MCLK_APBA_FREQM,
	/** @brief MCLK APBA SERCOM0 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBA_SERCOM0 = CLOCK_MCHP_SAME54_MCLK_APBA_SERCOM0,
	/** @brief MCLK APBA SERCOM1 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBA_SERCOM1 = CLOCK_MCHP_SAME54_MCLK_APBA_SERCOM1,
	/** @brief MCLK APBA TC0 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBA_TC0 = CLOCK_MCHP_SAME54_MCLK_APBA_TC0,
	/** @brief MCLK APBA TC1 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBA_TC1 = CLOCK_MCHP_SAME54_MCLK_APBA_TC1,

	/** @brief MCLK APBB USB Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBB_USB = CLOCK_MCHP_SAME54_MCLK_APBB_USB,
	/** @brief MCLK APBB DSU Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBB_DSU = CLOCK_MCHP_SAME54_MCLK_APBB_DSU,
	/** @brief MCLK APBB NVMCTRL Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBB_NVMCTRL = CLOCK_MCHP_SAME54_MCLK_APBB_NVMCTRL,
	/** @brief MCLK APBB PORT Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBB_PORT = CLOCK_MCHP_SAME54_MCLK_APBB_PORT,
	/** @brief MCLK APBB EVSYS Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBB_EVSYS = CLOCK_MCHP_SAME54_MCLK_APBB_EVSYS,
	/** @brief MCLK APBB SERCOM2 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBB_SERCOM2 = CLOCK_MCHP_SAME54_MCLK_APBB_SERCOM2,
	/** @brief MCLK APBB SERCOM3 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBB_SERCOM3 = CLOCK_MCHP_SAME54_MCLK_APBB_SERCOM3,
	/** @brief MCLK APBB TCC0 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBB_TCC0 = CLOCK_MCHP_SAME54_MCLK_APBB_TCC0,
	/** @brief MCLK APBB TCC1 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBB_TCC1 = CLOCK_MCHP_SAME54_MCLK_APBB_TCC1,
	/** @brief MCLK APBB TC2 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBB_TC2 = CLOCK_MCHP_SAME54_MCLK_APBB_TC2,
	/** @brief MCLK APBB TC3 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBB_TC3 = CLOCK_MCHP_SAME54_MCLK_APBB_TC3,
	/** @brief MCLK APBB RAMECC Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBB_RAMECC = CLOCK_MCHP_SAME54_MCLK_APBB_RAMECC,

	/** @brief MCLK APBC GMAC Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBC_GMAC = CLOCK_MCHP_SAME54_MCLK_APBC_GMAC,
	/** @brief MCLK APBC TCC2 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBC_TCC2 = CLOCK_MCHP_SAME54_MCLK_APBC_TCC2,
	/** @brief MCLK APBC TCC3 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBC_TCC3 = CLOCK_MCHP_SAME54_MCLK_APBC_TCC3,
	/** @brief MCLK APBC TC4 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBC_TC4 = CLOCK_MCHP_SAME54_MCLK_APBC_TC4,
	/** @brief MCLK APBC TC5 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBC_TC5 = CLOCK_MCHP_SAME54_MCLK_APBC_TC5,
	/** @brief MCLK APBC PDEC Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBC_PDEC = CLOCK_MCHP_SAME54_MCLK_APBC_PDEC,
	/** @brief MCLK APBC AC Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBC_AC = CLOCK_MCHP_SAME54_MCLK_APBC_AC,
	/** @brief MCLK APBC AES Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBC_AES = CLOCK_MCHP_SAME54_MCLK_APBC_AES,
	/** @brief MCLK APBC TRNG Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBC_TRNG = CLOCK_MCHP_SAME54_MCLK_APBC_TRNG,
	/** @brief MCLK APBC ICM Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBC_ICM = CLOCK_MCHP_SAME54_MCLK_APBC_ICM,
	/** @brief MCLK APBC QSPI Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBC_QSPI = CLOCK_MCHP_SAME54_MCLK_APBC_QSPI,
	/** @brief MCLK APBC CCL Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBC_CCL = CLOCK_MCHP_SAME54_MCLK_APBC_CCL,

	/** @brief MCLK APBD SERCOM4 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBD_SERCOM4 = CLOCK_MCHP_SAME54_MCLK_APBD_SERCOM4,
	/** @brief MCLK APBD SERCOM5 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBD_SERCOM5 = CLOCK_MCHP_SAME54_MCLK_APBD_SERCOM5,
	/** @brief MCLK APBD SERCOM6 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBD_SERCOM6 = CLOCK_MCHP_SAME54_MCLK_APBD_SERCOM6,
	/** @brief MCLK APBD SERCOM7 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBD_SERCOM7 = CLOCK_MCHP_SAME54_MCLK_APBD_SERCOM7,
	/** @brief MCLK APBD TCC4 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBD_TCC4 = CLOCK_MCHP_SAME54_MCLK_APBD_TCC4,
	/** @brief MCLK APBD TC6 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBD_TC6 = CLOCK_MCHP_SAME54_MCLK_APBD_TC6,
	/** @brief MCLK APBD TC7 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBD_TC7 = CLOCK_MCHP_SAME54_MCLK_APBD_TC7,
	/** @brief MCLK APBD ADC0 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBD_ADC0 = CLOCK_MCHP_SAME54_MCLK_APBD_ADC0,
	/** @brief MCLK APBD ADC1 Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBD_ADC1 = CLOCK_MCHP_SAME54_MCLK_APBD_ADC1,
	/** @brief MCLK APBD DAC Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBD_DAC = CLOCK_MCHP_SAME54_MCLK_APBD_DAC,
	/** @brief MCLK APBD I2S Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBD_I2S = CLOCK_MCHP_SAME54_MCLK_APBD_I2S,
	/** @brief MCLK APBD PCC Clock */
	CLOCK_CONTROL_MCHP_V1_MCLK_APBD_PCC = CLOCK_MCHP_SAME54_MCLK_APBD_PCC,
} clock_control_mchp_mclk_sys_t;

/** @brief Clocks handled by the OSCCTRL peripheral.
 *
 * This can be used as id in clock_control_mchp_subsys_t clock_control API.
 */
typedef enum {
	/** @brief OSCCTRL External Oscillator 0 */
	CLOCK_CONTROL_MCHP_V1_OSCCTRL_XOSC0 = CLOCK_MCHP_SAME54_OSCCTRL_XOSC0,
	/** @brief OSCCTRL External Oscillator 1 */
	CLOCK_CONTROL_MCHP_V1_OSCCTRL_XOSC1 = CLOCK_MCHP_SAME54_OSCCTRL_XOSC1,
	/** @brief OSCCTRL Digital Frequency Locked Loop */
	CLOCK_CONTROL_MCHP_V1_OSCCTRL_DFLL = CLOCK_MCHP_SAME54_OSCCTRL_DFLL,
	/** @brief OSCCTRL Frequency Digital Phase-Locked Loop 0 */
	CLOCK_CONTROL_MCHP_V1_OSCCTRL_FDPLL0 = CLOCK_MCHP_SAME54_OSCCTRL_FDPLL0,
	/** @brief OSCCTRL Frequency Digital Phase-Locked Loop 1 */
	CLOCK_CONTROL_MCHP_V1_OSCCTRL_FDPLL1 = CLOCK_MCHP_SAME54_OSCCTRL_FDPLL1,
} clock_control_mchp_oscctrl_sys_t;

/** @brief Clocks handled by the OSC32KCTRL peripheral.
 *
 * This can be used as id in clock_control_mchp_subsys_t clock_control API.
 */
typedef enum {
	/** @brief OSC32KCTRL External Oscillator 32K */
	CLOCK_CONTROL_MCHP_V1_OSC32KCTRL_XOSC32K = CLOCK_MCHP_SAME54_OSC32KCTRL_XOSC32K,
	/** @brief OSC32KCTRL External Oscillator 32K Output 32K */
	CLOCK_CONTROL_MCHP_V1_OSC32KCTRL_XOSC32K_OUT32K =
		CLOCK_MCHP_SAME54_OSC32KCTRL_XOSC32K_OUT32K,
	/** @brief OSC32KCTRL External Oscillator 32K Output 1K */
	CLOCK_CONTROL_MCHP_V1_OSC32KCTRL_XOSC32K_OUT1K = CLOCK_MCHP_SAME54_OSC32KCTRL_XOSC32K_OUT1K,
	/** @brief OSC32KCTRL Oscillator ULP 32K */
	CLOCK_CONTROL_MCHP_V1_OSC32KCTRL_OSCULP32K = CLOCK_MCHP_SAME54_OSC32KCTRL_OSCULP32K,
	/** @brief OSC32KCTRL Oscillator ULP 32K Output 32K */
	CLOCK_CONTROL_MCHP_V1_OSC32KCTRL_OSCULP32K_OUT32K =
		CLOCK_MCHP_SAME54_OSC32KCTRL_OSCULP32K_OUT32K,
	/** @brief OSC32KCTRL Oscillator ULP 32K Output 1K */
	CLOCK_CONTROL_MCHP_V1_OSC32KCTRL_OSCULP32K_OUT1K =
		CLOCK_MCHP_SAME54_OSC32KCTRL_OSCULP32K_OUT1K,
	/** @brief OSC32KCTRL RTC Clock */
	CLOCK_CONTROL_MCHP_V1_OSC32KCTRL_RTC = CLOCK_MCHP_SAME54_OSC32KCTRL_RTC,
} clock_control_mchp_osc32kctrl_sys_t;

/** @brief Clocks handled by the GCLK peripheral.
 *
 * This can be used as id in clock_control_mchp_subsys_t clock_control API.
 */
typedef enum {
	/** @brief GCLK Generator 0 */
	CLOCK_CONTROL_MCHP_V1_GCLK_GEN0 = CLOCK_MCHP_SAME54_GCLK_GEN0,
	/** @brief GCLK Generator 1 */
	CLOCK_CONTROL_MCHP_V1_GCLK_GEN1 = CLOCK_MCHP_SAME54_GCLK_GEN1,
	/** @brief GCLK Generator 2 */
	CLOCK_CONTROL_MCHP_V1_GCLK_GEN2 = CLOCK_MCHP_SAME54_GCLK_GEN2,
	/** @brief GCLK Generator 3 */
	CLOCK_CONTROL_MCHP_V1_GCLK_GEN3 = CLOCK_MCHP_SAME54_GCLK_GEN3,
	/** @brief GCLK Generator 4 */
	CLOCK_CONTROL_MCHP_V1_GCLK_GEN4 = CLOCK_MCHP_SAME54_GCLK_GEN4,
	/** @brief GCLK Generator 5 */
	CLOCK_CONTROL_MCHP_V1_GCLK_GEN5 = CLOCK_MCHP_SAME54_GCLK_GEN5,
	/** @brief GCLK Generator 6 */
	CLOCK_CONTROL_MCHP_V1_GCLK_GEN6 = CLOCK_MCHP_SAME54_GCLK_GEN6,
	/** @brief GCLK Generator 7 */
	CLOCK_CONTROL_MCHP_V1_GCLK_GEN7 = CLOCK_MCHP_SAME54_GCLK_GEN7,
	/** @brief GCLK Generator 8 */
	CLOCK_CONTROL_MCHP_V1_GCLK_GEN8 = CLOCK_MCHP_SAME54_GCLK_GEN8,
	/** @brief GCLK Generator 9 */
	CLOCK_CONTROL_MCHP_V1_GCLK_GEN9 = CLOCK_MCHP_SAME54_GCLK_GEN9,
	/** @brief GCLK Generator 10 */
	CLOCK_CONTROL_MCHP_V1_GCLK_GEN10 = CLOCK_MCHP_SAME54_GCLK_GEN10,
	/** @brief GCLK Generator 11 */
	CLOCK_CONTROL_MCHP_V1_GCLK_GEN11 = CLOCK_MCHP_SAME54_GCLK_GEN11,

	/** @brief GCLK for OSCCTRL DFLL48 */
	CLOCK_CONTROL_MCHP_V1_GCLK_OSCCTRL_DFLL48 = CLOCK_MCHP_SAME54_GCLK_OSCCTRL_DFLL48,
	/** @brief GCLK for OSCCTRL FDPLL0 */
	CLOCK_CONTROL_MCHP_V1_GCLK_OSCCTRL_FDPLL0 = CLOCK_MCHP_SAME54_GCLK_OSCCTRL_FDPLL0,
	/** @brief GCLK for OSCCTRL FDPLL1 */
	CLOCK_CONTROL_MCHP_V1_GCLK_OSCCTRL_FDPLL1 = CLOCK_MCHP_SAME54_GCLK_OSCCTRL_FDPLL1,
	/** @brief GCLK for OSCCTRL SLOW */
	CLOCK_CONTROL_MCHP_V1_GCLK_OSCCTRL_SLOW = CLOCK_MCHP_SAME54_GCLK_OSCCTRL_SLOW,
	/** @brief GCLK for EIC */
	CLOCK_CONTROL_MCHP_V1_GCLK_EIC = CLOCK_MCHP_SAME54_GCLK_EIC,
	/** @brief GCLK for FREQM MSR */
	CLOCK_CONTROL_MCHP_V1_GCLK_FREQM_MSR = CLOCK_MCHP_SAME54_GCLK_FREQM_MSR,
	/** @brief GCLK for FREQM REF */
	CLOCK_CONTROL_MCHP_V1_GCLK_FREQM_REF = CLOCK_MCHP_SAME54_GCLK_FREQM_REF,
	/** @brief GCLK for SERCOM0 CORE */
	CLOCK_CONTROL_MCHP_V1_GCLK_SERCOM0_CORE = CLOCK_MCHP_SAME54_GCLK_SERCOM0_CORE,
	/** @brief GCLK for SERCOM1 CORE */
	CLOCK_CONTROL_MCHP_V1_GCLK_SERCOM1_CORE = CLOCK_MCHP_SAME54_GCLK_SERCOM1_CORE,
	/** @brief GCLK for TC0 and TC1 */
	CLOCK_CONTROL_MCHP_V1_GCLK_TC0_TC1 = CLOCK_MCHP_SAME54_GCLK_TC0_TC1,
	/** @brief GCLK for USB */
	CLOCK_CONTROL_MCHP_V1_GCLK_USB = CLOCK_MCHP_SAME54_GCLK_USB,
	/** @brief GCLK for EVSYS0 */
	CLOCK_CONTROL_MCHP_V1_GCLK_EVSYS0 = CLOCK_MCHP_SAME54_GCLK_EVSYS0,
	/** @brief GCLK for EVSYS1 */
	CLOCK_CONTROL_MCHP_V1_GCLK_EVSYS1 = CLOCK_MCHP_SAME54_GCLK_EVSYS1,
	/** @brief GCLK for EVSYS2 */
	CLOCK_CONTROL_MCHP_V1_GCLK_EVSYS2 = CLOCK_MCHP_SAME54_GCLK_EVSYS2,
	/** @brief GCLK for EVSYS3 */
	CLOCK_CONTROL_MCHP_V1_GCLK_EVSYS3 = CLOCK_MCHP_SAME54_GCLK_EVSYS3,
	/** @brief GCLK for EVSYS4 */
	CLOCK_CONTROL_MCHP_V1_GCLK_EVSYS4 = CLOCK_MCHP_SAME54_GCLK_EVSYS4,
	/** @brief GCLK for EVSYS5 */
	CLOCK_CONTROL_MCHP_V1_GCLK_EVSYS5 = CLOCK_MCHP_SAME54_GCLK_EVSYS5,
	/** @brief GCLK for EVSYS6 */
	CLOCK_CONTROL_MCHP_V1_GCLK_EVSYS6 = CLOCK_MCHP_SAME54_GCLK_EVSYS6,
	/** @brief GCLK for EVSYS7 */
	CLOCK_CONTROL_MCHP_V1_GCLK_EVSYS7 = CLOCK_MCHP_SAME54_GCLK_EVSYS7,
	/** @brief GCLK for EVSYS8 */
	CLOCK_CONTROL_MCHP_V1_GCLK_EVSYS8 = CLOCK_MCHP_SAME54_GCLK_EVSYS8,
	/** @brief GCLK for EVSYS9 */
	CLOCK_CONTROL_MCHP_V1_GCLK_EVSYS9 = CLOCK_MCHP_SAME54_GCLK_EVSYS9,
	/** @brief GCLK for EVSYS10 */
	CLOCK_CONTROL_MCHP_V1_GCLK_EVSYS10 = CLOCK_MCHP_SAME54_GCLK_EVSYS10,
	/** @brief GCLK for EVSYS11 */
	CLOCK_CONTROL_MCHP_V1_GCLK_EVSYS11 = CLOCK_MCHP_SAME54_GCLK_EVSYS11,
	/** @brief GCLK for SERCOM2 CORE */
	CLOCK_CONTROL_MCHP_V1_GCLK_SERCOM2_CORE = CLOCK_MCHP_SAME54_GCLK_SERCOM2_CORE,
	/** @brief GCLK for SERCOM3 CORE */
	CLOCK_CONTROL_MCHP_V1_GCLK_SERCOM3_CORE = CLOCK_MCHP_SAME54_GCLK_SERCOM3_CORE,
	/** @brief GCLK for TCC0 and TCC1 */
	CLOCK_CONTROL_MCHP_V1_GCLK_TCC0_TCC1 = CLOCK_MCHP_SAME54_GCLK_TCC0_TCC1,
	/** @brief GCLK for TC2 and TC3 */
	CLOCK_CONTROL_MCHP_V1_GCLK_TC2_TC3 = CLOCK_MCHP_SAME54_GCLK_TC2_TC3,
	/** @brief GCLK for CAN0 */
	CLOCK_CONTROL_MCHP_V1_GCLK_CAN0 = CLOCK_MCHP_SAME54_GCLK_CAN0,
	/** @brief GCLK for CAN1 */
	CLOCK_CONTROL_MCHP_V1_GCLK_CAN1 = CLOCK_MCHP_SAME54_GCLK_CAN1,
	/** @brief GCLK for TCC2 and TCC3 */
	CLOCK_CONTROL_MCHP_V1_GCLK_TCC2_TCC3 = CLOCK_MCHP_SAME54_GCLK_TCC2_TCC3,
	/** @brief GCLK for TC4 and TC5 */
	CLOCK_CONTROL_MCHP_V1_GCLK_TC4_TC5 = CLOCK_MCHP_SAME54_GCLK_TC4_TC5,
	/** @brief GCLK for PDEC */
	CLOCK_CONTROL_MCHP_V1_GCLK_PDEC = CLOCK_MCHP_SAME54_GCLK_PDEC,
	/** @brief GCLK for AC */
	CLOCK_CONTROL_MCHP_V1_GCLK_AC = CLOCK_MCHP_SAME54_GCLK_AC,
	/** @brief GCLK for CCL */
	CLOCK_CONTROL_MCHP_V1_GCLK_CCL = CLOCK_MCHP_SAME54_GCLK_CCL,
	/** @brief GCLK for SERCOM4 CORE */
	CLOCK_CONTROL_MCHP_V1_GCLK_SERCOM4_CORE = CLOCK_MCHP_SAME54_GCLK_SERCOM4_CORE,
	/** @brief GCLK for SERCOM5 CORE */
	CLOCK_CONTROL_MCHP_V1_GCLK_SERCOM5_CORE = CLOCK_MCHP_SAME54_GCLK_SERCOM5_CORE,
	/** @brief GCLK for SERCOM6 CORE */
	CLOCK_CONTROL_MCHP_V1_GCLK_SERCOM6_CORE = CLOCK_MCHP_SAME54_GCLK_SERCOM6_CORE,
	/** @brief GCLK for SERCOM7 CORE */
	CLOCK_CONTROL_MCHP_V1_GCLK_SERCOM7_CORE = CLOCK_MCHP_SAME54_GCLK_SERCOM7_CORE,
	/** @brief GCLK for TCC4 */
	CLOCK_CONTROL_MCHP_V1_GCLK_TCC4 = CLOCK_MCHP_SAME54_GCLK_TCC4,
	/** @brief GCLK for TC6 and TC7 */
	CLOCK_CONTROL_MCHP_V1_GCLK_TC6_TC7 = CLOCK_MCHP_SAME54_GCLK_TC6_TC7,
	/** @brief GCLK for ADC0 */
	CLOCK_CONTROL_MCHP_V1_GCLK_ADC0 = CLOCK_MCHP_SAME54_GCLK_ADC0,
	/** @brief GCLK for ADC1 */
	CLOCK_CONTROL_MCHP_V1_GCLK_ADC1 = CLOCK_MCHP_SAME54_GCLK_ADC1,
	/** @brief GCLK for DAC */
	CLOCK_CONTROL_MCHP_V1_GCLK_DAC = CLOCK_MCHP_SAME54_GCLK_DAC,
	/** @brief GCLK for I2S0 */
	CLOCK_CONTROL_MCHP_V1_GCLK_I2S0 = CLOCK_MCHP_SAME54_GCLK_I2S0,
	/** @brief GCLK for I2S1 */
	CLOCK_CONTROL_MCHP_V1_GCLK_I2S1 = CLOCK_MCHP_SAME54_GCLK_I2S1,
	/** @brief GCLK for SDHC0 */
	CLOCK_CONTROL_MCHP_V1_GCLK_SDHC0 = CLOCK_MCHP_SAME54_GCLK_SDHC0,
	/** @brief GCLK for SDHC1 */
	CLOCK_CONTROL_MCHP_V1_GCLK_SDHC1 = CLOCK_MCHP_SAME54_GCLK_SDHC1,
	/** @brief GCLK for CM4 Trace */
	CLOCK_CONTROL_MCHP_V1_GCLK_CM4_TRACE = CLOCK_MCHP_SAME54_GCLK_CM4_TRACE,
} clock_control_mchp_gclk_sys_t;

/** @brief MCLK (Main Clock) configuration structure.
 *
 * This structure defines the configuration for the Main Clock (MCLK), including the CPU clock
 * division and the enabling/disabling of peripheral clocks across the AHB, APBA, APBB, APBC, and
 * APBD buses.
 */
typedef struct {
	/** @brief MCLK CPU clock divider */
	uint8_t cpu_div;
	/** @brief MCLK AHB clock enables */
	bool ahb[CLOCK_CONTROL_MCHP_MCLK_AHB_COUNT];
	/** @brief MCLK APBA clock enables */
	bool apba[CLOCK_CONTROL_MCHP_MCLK_APBA_COUNT];
	/** @brief MCLK APBB clock enables */
	bool apbb[CLOCK_CONTROL_MCHP_MCLK_APBB_COUNT];
	/** @brief MCLK APBC clock enables */
	bool apbc[CLOCK_CONTROL_MCHP_MCLK_APBC_COUNT];
	/** @brief MCLK APBD clock enables */
	bool apbd[CLOCK_CONTROL_MCHP_MCLK_APBD_COUNT];
} clock_control_mchp_mclk_configuration_t;

/** @brief OSCCTRL (Oscillator Control) configuration structure.
 *
 * This structure holds the configuration for various oscillators and clock sources managed by the
 * OSCCTRL peripheral, including the XOSC, DFLL, and DPLL configurations.
 */
typedef struct {
	/** @brief OSCCTRL XOSCCTRL register values */
	uint32_t xoscctrl[CLOCK_CONTROL_MCHP_OSCCTRL_XOSC_COUNT];
	/** @brief OSCCTRL DFLLCTRLA register value */
	uint8_t dfllctrla;
	/** @brief OSCCTRL DFLLCTRLB register value */
	uint8_t dfllctrlb;
	/** @brief OSCCTRL DFLLVAL register value */
	uint32_t dfllval;
	/** @brief OSCCTRL DFLLMUL register value */
	uint32_t dfllmul;
	/** @brief OSCCTRL DPLLCTRLA register values */
	uint8_t dpllctrla[CLOCK_CONTROL_MCHP_OSCCTRL_DPLL_COUNT];
	/** @brief OSCCTRL DPLLRATIO register values */
	uint32_t dpllratio[CLOCK_CONTROL_MCHP_OSCCTRL_DPLL_COUNT];
	/** @brief OSCCTRL DPLLCTRLB register values */
	uint8_t dpllctrlb[CLOCK_CONTROL_MCHP_OSCCTRL_DPLL_COUNT];
} clock_control_mchp_oscctrl_configuration_t;

/** @brief OSC32KCTRL (32kHz Oscillator Control) configuration structure.
 *
 * This structure holds the configuration for various 32kHz oscillators and clock sources managed by
 * the OSC32KCTRL peripheral, including the RTC, XOSC32K, and ULP32K settings.
 */
typedef struct {
	/** @brief OSC32KCTRL RTCCTRL register value */
	uint8_t rtcctrl;
	/** @brief OSC32KCTRL XOSC32K register value */
	uint16_t xosc32k;
	/** @brief OSC32KCTRL CFDCTRL register value */
	uint8_t cfdctrl;
	/** @brief OSC32KCTRL OSCULP32K register value */
	uint32_t osculp32k;
} clock_control_mchp_osc32kctrl_configuration_t;

/** @brief GCLK (Generic Clock Controller) configuration structure.
 *
 * This structure holds the configuration for the Generic Clock Controller (GCLK), including the
 * GENCTRL register for controlling clock generation and the PCHCTRL registers for controlling
 * peripheral clock channels.
 */
typedef struct {
	/** @brief GCLK GENCTRL register values */
	uint32_t genctrl[CLOCK_CONTROL_MCHP_GCLK_GEN_COUNT];
	/** @brief GCLK PCHCTRL register values */
	uint32_t pchctrl[CLOCK_CONTROL_MCHP_GCLK_PCH_COUNT];
} clock_control_mchp_gclk_configuration_t;

/** @brief Overall clock control configuration structure.
 *
 * This structure holds all the configurations for different clock control subsystems, including
 * MCLK, OSCCTRL, OSC32KCTRL, and GCLK. It provides a unified configuration interface for managing
 * clock settings across the system.
 */
typedef struct {
	/** @brief Pointer to MCLK configuration */
	clock_control_mchp_mclk_configuration_t *mclk_configuration;
	/** @brief Pointer to OSCCTRL configuration */
	clock_control_mchp_oscctrl_configuration_t *oscctrl_configuration;
	/** @brief Pointer to OSC32KCTRL configuration */
	clock_control_mchp_osc32kctrl_configuration_t *osc32kctrl_configuration;
	/** @brief Pointer to GCLK configuration */
	clock_control_mchp_gclk_configuration_t *gclk_configuration;
} clock_control_mchp_configuration_t;

/** @brief MCLK rate configuration structure.
 * This structure holds the MCLK clock division rate configuration.
 */
typedef struct {
	/** @brief MCLK CPU clock division factor. */
	uint8_t cpu_div;
} clock_control_mchp_mclk_rate_t;

/** @brief OSCCTRL rate configuration structure.
 * This structure holds the rate configuration for the DFLL and DPLL oscillators, including
 * multiplier factors, loop control, and division values for DPLL.
 */
typedef struct {
	/** @brief OSCCTRL DFLL multiplier factor. */
	uint16_t dfll_mul_factor;
	/** @brief Indicates if OSCCTRL DFLL is in closed-loop mode. */
	bool is_dfll_closed_loop;
	/** @brief OSCCTRL DPLL XOSC division factors for DPLLs. */
	uint16_t dpll_xosc_div[CLOCK_CONTROL_MCHP_OSCCTRL_DPLL_COUNT];
	/** @brief OSCCTRL DPLL LDR (Loop Division Ratio) values for DPLLs. */
	uint16_t dpll_ldr[CLOCK_CONTROL_MCHP_OSCCTRL_DPLL_COUNT];
	/** @brief OSCCTRL DPLL LDR fractional values for DPLLs. */
	uint8_t dpll_ldrfrac[CLOCK_CONTROL_MCHP_OSCCTRL_DPLL_COUNT];
} clock_control_mchp_oscctrl_rate_t;

/** @brief OSC32KCTRL rate configuration structure.
 * This structure holds the rate configuration for the 32KHz oscillator.
 */
typedef struct {
	/** @brief OSC32KCTRL Oscillator 32K prescaler factor. */
	uint8_t xosc32k_cfd_presc;
} clock_control_mchp_osc32kctrl_rate_t;

/** @brief GCLK rate configuration structure.
 * This structure holds the rate configuration for the GCLK (Generic Clock) generator and its
 * dividers.
 */
typedef struct {
	/** @brief Division selection for each of the GCLK generators. */
	bool div_sel[CLOCK_CONTROL_MCHP_GCLK_GEN_COUNT];
	/** @brief GCLK generator division factor for each of the generators. */
	uint16_t gen_div[CLOCK_CONTROL_MCHP_GCLK_GEN_COUNT];
} clock_control_mchp_gclk_rate_t;

/** @brief Clock rate configuration structure.
 * This structure holds the rate configuration for all clock sources, including MCLK, OSCCTRL,
 * OSC32KCTRL, and GCLK.
 */
typedef struct {
	/** @brief Pointer to MCLK rate configuration. */
	clock_control_mchp_mclk_rate_t *mclk_rate;
	/** @brief Pointer to OSCCTRL rate configuration. */
	clock_control_mchp_oscctrl_rate_t *oscctrl_rate;
	/** @brief Pointer to OSC32KCTRL rate configuration. */
	clock_control_mchp_osc32kctrl_rate_t *osc32kctrl_rate;
	/** @brief Pointer to GCLK rate configuration. */
	clock_control_mchp_gclk_rate_t *gclk_rate;
} clock_control_mchp_rate_t;

#endif /* CONFIG_SOC_SERIES_MCHP_SAME54 */

#endif /* MICROCHIP_CLOCK_CONTROL_MCHP_V1_H_ */
