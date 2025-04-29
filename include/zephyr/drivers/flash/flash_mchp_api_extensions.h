/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file flash_mchp_api_extensions.h
 * @brief Flash API extensions for Microchip devices.
 *
 * This file provides additional operations for the Flash API,
 * for extending the Flash API capabilities Microchip devices.
 */

#ifndef FLASH_MCHP_API_EXTENSIONS_H_
#define FLASH_MCHP_API_EXTENSIONS_H_

#include <stdint.h>

/**
 * @brief Extended flash operation codes for MCHP flash controller.
 *
 * This enumeration defines the set of extended operations that can be performed
 * on the flash memory, such as erasing or writing the user row, and locking or
 * unlocking specific flash regions.
 */
typedef enum {
	FLASH_EX_OP_USER_ROW_ERASE, /**< Erase the user row in flash memory. */
	FLASH_EX_OP_USER_ROW_WRITE, /**< Write data to the user row in flash memory. */
	FLASH_EX_OP_REGION_LOCK,    /**< Lock a specific region of flash memory. */
	FLASH_EX_OP_REGION_UNLOCK   /**< Unlock a specific region of flash memory. */
} flash_mchp_ex_ops_t;

/**
 * @brief Structure for user row data operations in MCHP flash.
 *
 * This structure is used to specify the parameters for operations
 * involving the user row region of flash memory, such as writing data
 * to or erasing a portion of the user row.
 */
typedef struct flash_mchp_ex_op_userrow_data {
	const void *data; /**< Pointer to the data buffer to be written or read. */
	size_t data_len;  /**< Length of the data buffer in bytes. */
	off_t offset;     /**< Offset within the user row region where the operation starts. */
} flash_mchp_ex_op_userrow_data_t;

#endif /* FLASH_MCHP_API_EXTENSIONS_H_ */
