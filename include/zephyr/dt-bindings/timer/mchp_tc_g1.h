/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file mchp_tc_g1.h
 * @brief Timer/counter DT-bindings for Microchip devices.
 *
 * This header file defines the macros for timer/counter, which can
 * be used in device tree nodes for managing timer/counter on
 * Microchip-based systems.
 */
#ifndef INCLUDE_ZEPHYR_DT_BINDINGS_TIMER_MCHP_TC_G1_H_

#define INCLUDE_ZEPHYR_DT_BINDINGS_TIMER_MCHP_TC_G1_H_

/** @brief IDs of the timer/counter peripherals
 *
 * This has to be given to the respective tc peripheral
 * based on to their id property.
 */

#define MCHP_G1_TC0_ID 0
#define MCHP_G1_TC1_ID 1
#define MCHP_G1_TC2_ID 2
#define MCHP_G1_TC3_ID 3
#define MCHP_G1_TC4_ID 4
#define MCHP_G1_TC5_ID 5
#define MCHP_G1_TC6_ID 6
#define MCHP_G1_TC7_ID 7

#endif /* INCLUDE_ZEPHYR_DT_BINDINGS_TIMER_MCHP_TC_G1_H_ */
