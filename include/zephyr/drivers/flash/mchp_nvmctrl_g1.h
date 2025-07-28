/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file mchp_nvmctrl_g1.h
 * @brief Extended Flash Operations for Microchip NVMCTRL G1
 *
 * This header provides definitions and data structures for additional
 * flash memory operations specific to the Microchip NVMCTRL G1
 * flash controller IP. It extends the standard flash driver capabilities
 * by enabling advanced operations such as user row access and region
 * locking/unlocking.
 *
 * @note This file is IP-specific and should only be included when targeting
 *       devices with the NVMCTRL G1 flash controller.
 *
 */
#ifndef INCLUDE_ZEPHYR_DRIVERS_FLASH_MCHP_NVMCTRL_G1_H_
#define INCLUDE_ZEPHYR_DRIVERS_FLASH_MCHP_NVMCTRL_G1_H_

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

#endif /*INCLUDE_ZEPHYR_DRIVERS_FLASH_MCHP_NVMCTRL_G1_H_*/
