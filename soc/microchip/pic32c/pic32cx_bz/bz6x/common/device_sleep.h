/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file device_sleep.h
 * @brief This file contains the Device Sleep functions.
 */

#ifndef DEVICE_SLEEP_H
#define DEVICE_SLEEP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * @brief Macros
 *****************************************************************************/

/******************************************************************************
 * @brief Data Types
 *****************************************************************************/

/******************************************************************************
 * @brief Function Prototypes
 *****************************************************************************/

/**@brief The API is used to enter system sleep mode
 *
 * @param[in] None
 * @param[out] None
 *
 * @retval None
 */
void device_enter_sleep_mode(void);

/**@brief The API is used to exit system sleep mode
 *
 * @param[in] None
 * @param[out] None
 *
 * @retval None
 */
void device_exit_sleep_mode(void);

#ifdef __cplusplus
}
#endif

#endif /* DEVICE_SLEEP_H */
