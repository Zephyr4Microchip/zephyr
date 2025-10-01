/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_MCHP_PIC32CXBZ_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_MCHP_PIC32CXBZ_PINCTRL_H_

#include <zephyr/dt-bindings/dt-util.h>

/** Position of the function field. */
#define MCHP_FUN_POS  24U
/** Mask for the function field. */
#define MCHP_FUN_MSK  0xFFU
/** Mask for the pull configuration field. */
#define MCHP_PULL_POS 18U
/** Mask for the pull configuration field. */
#define MCHP_PULL_MSK 0x3U
/** Position of the port field. */
#define MCHP_DIR_POS  16U
/** Mask for the pin field. */
#define MCHP_DIR_MSK  0x3U
/** Position of the pin field. */
#define MCHP_PORT_POS 8U
/** Mask for the pin field. */
#define MCHP_PORT_MSK 0xFFU
/** Position of the pin field. */
#define MCHP_PIN_POS  0U
/** Mask for the pin field. */
#define MCHP_PIN_MSK  0xFFU

/**
 * @name Microchip PIC32CXBZ pinctrl pin functions.
 * @{
 */
/** External Interrupt Controller */
#define MCHP_FUN_EXTINT0      0U
/** External Interrupt Controller */
#define MCHP_FUN_EXTINT1      1U
/** External Interrupt Controller */
#define MCHP_FUN_EXTINT2      2U
/** External Interrupt Controller */
#define MCHP_FUN_EXTINT3      3U
/** Non-Maskable Interrupt */
#define MCHP_FUN_NMI          4U
/** Serial Communication */
#define MCHP_FUN_SERCOM0_PAD0 5U
/** Serial Communication */
#define MCHP_FUN_SERCOM0_PAD1 6U
/** Serial Communication */
#define MCHP_FUN_SERCOM0_PAD2 7U
/** Serial Communication */
#define MCHP_FUN_SERCOM0_PAD3 8U
/** Serial Communication */
#define MCHP_FUN_SERCOM1_PAD0 9U
/** Serial Communication */
#define MCHP_FUN_SERCOM1_PAD1 10U
/** Serial Communication */
#define MCHP_FUN_SERCOM1_PAD2 11U
/** Serial Communication */
#define MCHP_FUN_SERCOM1_PAD3 12U
/** Serial Communication */
#define MCHP_FUN_SERCOM2_PAD0 13U
/** Serial Communication */
#define MCHP_FUN_SERCOM2_PAD1 14U
/** Serial Communication */
#define MCHP_FUN_SERCOM2_PAD2 15U
/** Serial Communication */
#define MCHP_FUN_SERCOM2_PAD3 16U
/** Serial Communication */
#define MCHP_FUN_SERCOM3_PAD0 17U
/** Serial Communication */
#define MCHP_FUN_SERCOM3_PAD1 18U
/** Serial Communication */
#define MCHP_FUN_SERCOM3_PAD2 19U
/** Serial Communication */
#define MCHP_FUN_SERCOM3_PAD3 20U
/** Quad-SPI Serial Clock */
#define MCHP_FUN_QSPI_SCK     21U
/** Quad-SPI Chip Select */
#define MCHP_FUN_QSPI_CS      22U
/** Quad-SPI Data Output (Data Input Output 0)*/
#define MCHP_FUN_QSPI_DATA0   23U
/** Quad-SPI Data Input (Data Input Output 1) */
#define MCHP_FUN_QSPI_DATA1   24U
/** Quad-SPI Data Input Output 2*/
#define MCHP_FUN_QSPI_DATA2   25U
/** Quad-SPI Data Input Output 3*/
#define MCHP_FUN_QSPI_DATA3   26U
/** Reference Clock Generator */
#define MCHP_FUN_REFI         27U
/** Configure Custom Logic Input */
#define MCHP_FUN_CCLIN0       28U
/** Configure Custom Logic Input */
#define MCHP_FUN_CCLIN1       29U
/** Configure Custom Logic Input */
#define MCHP_FUN_CCLIN2       30U
/** Configure Custom Logic Input */
#define MCHP_FUN_CCLIN3       31U
/** Configure Custom Logic Input */
#define MCHP_FUN_CCLIN4       32U
/** Configure Custom Logic Input */
#define MCHP_FUN_CCLIN5       33U
/** Time/Counter Output */
#define MCHP_FUN_TC0_WO0      34U
/** Time/Counter Output */
#define MCHP_FUN_TC0_WO1      35U
/** Time/Counter Output */
#define MCHP_FUN_TC1_WO0      36U
/** Time/Counter Output */
#define MCHP_FUN_TC1_WO1      37U
/** Time/Counter Output */
#define MCHP_FUN_TC2_WO0      38U
/** Time/Counter Output */
#define MCHP_FUN_TC2_WO1      39U
/** Time/Counter Output */
#define MCHP_FUN_TC3_WO0      40U
/** Time/Counter Output */
#define MCHP_FUN_TC3_WO1      41U
/** Timer/Counter for Control Applications Output */
#define MCHP_FUN_TCC0_WO0     42U
/** Timer/Counter for Control Applications Output */
#define MCHP_FUN_TCC0_WO1     43U
/** Timer/Counter for Control Applications Output */
#define MCHP_FUN_TCC0_WO2     44U
/** Timer/Counter for Control Applications Output */
#define MCHP_FUN_TCC0_WO3     45U
/** Timer/Counter for Control Applications Output */
#define MCHP_FUN_TCC0_WO4     46U
/** Timer/Counter for Control Applications Output */
#define MCHP_FUN_TCC0_WO5     47U
/** Timer/Counter for Control Applications Output */
#define MCHP_FUN_TCC1_WO0     48U
/** Timer/Counter for Control Applications Output */
#define MCHP_FUN_TCC1_WO1     49U
/** Timer/Counter for Control Applications Output */
#define MCHP_FUN_TCC1_WO2     50U
/** Timer/Counter for Control Applications Output */
#define MCHP_FUN_TCC1_WO3     51U
/** Timer/Counter for Control Applications Output */
#define MCHP_FUN_TCC1_WO4     52U
/** Timer/Counter for Control Applications Output */
#define MCHP_FUN_TCC1_WO5     53U
/** Timer/Counter for Control Applications Output */
#define MCHP_FUN_TCC2_WO0     54U
/** Timer/Counter for Control Applications Output */
#define MCHP_FUN_TCC2_WO1     55U
/** Reference Output */
#define MCHP_FUN_REFO1        56U
/** Reference Output */
#define MCHP_FUN_REFO2        57U
/** Reference Output */
#define MCHP_FUN_REFO3        58U
/** Reference Output */
#define MCHP_FUN_REFO4        59U
/** Configure Custom Logic Output */
#define MCHP_FUN_CCL_OUT0     60U
/** Configure Custom Logic Output */
#define MCHP_FUN_CCL_OUT1     61U

/**
 * @name Microchip PIC32CXBZ port.
 * @{
 */
/** Port None */
#define MCHP_PORT_NONE 0U
/** Port A */
#define MCHP_PORT_A    1U
/** Port B */
#define MCHP_PORT_B    2U

/** Pull-up disabled. */
#define MCHP_PULL_NONE 0U
/** Pull-down enabled. */
#define MCHP_PULL_DOWN 1U
/** Pull-up enabled. */
#define MCHP_PULL_UP   2U

/** Direction disabled. */
#define MCHP_DIR_NONE 0U
/** Input enabled. */
#define MCHP_DIR_IN   1U
/** Output enabled. */
#define MCHP_DIR_OUT  2U

/**
 * @brief Utility macro to build Microchip PIC32CXBZ psels property entry.
 *
 * @param fun Pin function configuration.
 * @param port Port.
 * @param pin Pin.
 */
#define MCHP_PSEL(fun, port, pin)                                                                  \
	((((MCHP_FUN_##fun) & MCHP_FUN_MSK) << MCHP_FUN_POS) |                                     \
	 (((MCHP_PORT_##port) & MCHP_PORT_MSK) << MCHP_PORT_POS) |                                 \
	 (((pin) & MCHP_PIN_MSK) << MCHP_PIN_POS))

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_MCHP_PIC32CXBZ_PINCTRL_H_ */
