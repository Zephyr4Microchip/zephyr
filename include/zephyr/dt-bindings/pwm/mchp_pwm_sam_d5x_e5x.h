/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file mchp_pwm_sam_d5x_e5x.h
 * @brief Timer/counter DT-bindings for Microchip devices.
 *
 * This header file defines the macros for timer/counter, which can
 * be used in device tree nodes for managing timer/counter on
 * Microchip-based systems.
 */
#ifndef MICROCHIP_MCHP_PWM_SAM_D5X_E5X_H_

#define MICROCHIP_MCHP_PWM_SAM_D5X_E5X_H_

/** @brief IDs of the timer/counter peripherals
 *
 * This has to be given to the respective tc/tcc peripheral
 * based on to their id property.
 */

#define PWM_COUNTER_MCHP_SAM_D5X_E5X_TC0_ID 0
#define PWM_COUNTER_MCHP_SAM_D5X_E5X_TC1_ID 1
#define PWM_COUNTER_MCHP_SAM_D5X_E5X_TC2_ID 2
#define PWM_COUNTER_MCHP_SAM_D5X_E5X_TC3_ID 3

#define PWM_COUNTER_MCHP_SAM_D5X_E5X_TC4_ID 4
#define PWM_COUNTER_MCHP_SAM_D5X_E5X_TC5_ID 5

#define PWM_COUNTER_MCHP_SAM_D5X_E5X_TC6_ID 6
#define PWM_COUNTER_MCHP_SAM_D5X_E5X_TC7_ID 7

#define PWM_COUNTER_MCHP_SAM_D5X_E5X_TCC0_ID 8
#define PWM_COUNTER_MCHP_SAM_D5X_E5X_TCC1_ID 9
#define PWM_COUNTER_MCHP_SAM_D5X_E5X_TCC2_ID 10

#define PWM_COUNTER_MCHP_SAM_D5X_E5X_TCC3_ID 11
#define PWM_COUNTER_MCHP_SAM_D5X_E5X_TCC4_ID 12

#endif /* MICROCHIP_MCHP_PWM_SAM_D5X_E5X_H_ */
