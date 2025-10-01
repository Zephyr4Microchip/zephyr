/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file flash_mchp_nvmctrl_g1.c
 * @brief Flash driver for Microchip NVMCTRL G2 IP
 *
 * This file provides macro definitions, structures, and
 * function-like macros for configuring and initializing
 * the Flash peripheral on Microchip devices.
 */

#include <soc.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>

LOG_MODULE_REGISTER(flash_mchp_nvmctrl_g2);

/*******************************************
 * Const and Macro Defines
 *******************************************/

/* Device tree compatible string for Microchip nvm g2. */
#define DT_DRV_COMPAT microchip_nvmctrl_g2_flash

/* Device tree node identifier for SoC non-volatile flash instance 0. */
#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)

/* Size of the SoC non-volatile flash, in bytes. */
#define SOC_NV_FLASH_SIZE DT_REG_SIZE(SOC_NV_FLASH_NODE)

/* Base address of the SoC non-volatile flash. */
#define SOC_NV_FLASH_BASE_ADDRESS DT_REG_ADDR(SOC_NV_FLASH_NODE)

/* Write block size of the SoC non-volatile flash, in bytes. */
#define SOC_NV_FLASH_WRITE_BLOCK_SIZE DT_PROP(SOC_NV_FLASH_NODE, write_block_size)

/* Erase block size of the SoC non-volatile flash, in bytes. */
#define SOC_NV_FLASH_ERASE_BLOCK_SIZE DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)

/* Number of flash page layouts supported by the MCHP flash driver. */
#define FLASH_MCHP_LAYOUT_SIZE 0x1

#if defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ6)

/* Size of a quad word in bytes for MCHP flash. */
#define FLASH_MCHP_QUAD_WORD_SIZE 0x20

/* Size of a word in bytes for MCHP flash. */
#define FLASH_MCHP_WORD_SIZE 0x8

/* Number of NVM data registers in the Microchip flash device. */
#define FLASH_MCHP_NVM_DATA_REGS_COUNT 0x8

#else

/* Size of a quad word in bytes for MCHP flash. */
#define FLASH_MCHP_QUAD_WORD_SIZE      0x10

/* Size of a word in bytes for MCHP flash. */
#define FLASH_MCHP_WORD_SIZE           0x4

/* Number of NVM data registers in the Microchip flash device. */
#define FLASH_MCHP_NVM_DATA_REGS_COUNT 0x4

#endif

/* Size of a page in bytes for MCHP flash. */
#define FLASH_MCHP_PAGE_SIZE 0x400

/* Device config */
#define DEV_CFG(dev) ((const flash_mchp_dev_config_t *)(dev)->config)

/* NVMCTRL Register */
#define FLASH_REGS ((const flash_mchp_dev_config_t *)(dev)->config)->regs

/**< Enable this for getting the debug prints from the HAL APIs */
/*#define FLASH_DBG_MODE*/
#ifdef FLASH_DBG_MODE
#define DBG_FLASH(format, ...) printf(format, ##__VA_ARGS__)
#else
#define DBG_FLASH(format, ...)
#endif /*FLASH_DBG_MODE*/

/* Default value of flash memory after an erase operation. */
#define FLASH_ERASE_DEFAULT_VALUE 0xFF

/**
 * @def FLASH_MCHP_SUCCESS
 * @brief Macro indicating successful operation.
 */
#define FLASH_MCHP_SUCCESS 0

/**
 * @def FLASH_MCHP_FAIL
 * @brief Macro indicating failed operation.
 */
#define FLASH_MCHP_FAIL -1

/**
 * @brief Type definition for the flash lock.
 *
 * This macro defines the type of the lock used to protect access to the flash APIs.
 */
#define FLASH_MCHP_LOCK_TYPE struct k_mutex

/**
 * @brief Timeout duration for acquiring the flash lock.
 *
 * This macro defines the timeout duration for acquiring the flash lock.
 * The timeout is specified in milliseconds.
 */
#define FLASH_MCHP_LOCK_TIMEOUT K_MSEC(10)

/**
 * @brief Initialize the flash lock.
 *
 * This macro initializes the flash lock.
 *
 * @param p_lock Pointer to the lock to be initialized.
 */
#define FLASH_MCHP_DATA_MUTEX_INIT(p_lock) k_mutex_init(p_lock)

/**
 * @brief Acquire the flash lock.
 *
 * This macro acquires the flash lock. If the lock is not available, the
 * function will wait for the specified timeout duration.
 *
 * @param p_lock Pointer to the lock to be acquired.
 * @return 0 if the lock was successfully acquired, or a negative error code.
 */
#define FLASH_MCHP_DATA_LOCK(p_lock) k_mutex_lock(p_lock, FLASH_MCHP_LOCK_TIMEOUT)

/**
 * @brief Release the flash lock.
 *
 * This macro releases the flash lock.
 *
 * @param p_lock Pointer to the lock to be released.
 * @return 0 if the lock was successfully released, or a negative error code.
 */
#define FLASH_MCHP_DATA_UNLOCK(p_lock) k_mutex_unlock(p_lock)

/**
 * @brief Type definition for the flash command semaphore lock.
 *
 * This macro defines the type of the semaphore used to synchronize nvmctrl operations.
 */
#define FLASH_MCHP_CMD_LOCK_TYPE struct k_sem

/**
 * @brief Timeout duration for acquiring the flash semaphore lock.
 *
 * This macro defines the timeout duration for acquiring the flash semaphore lock.
 * The timeout is specified in milliseconds.
 */
#define FLASH_MCHP_CMD_SEM_TIMEOUT K_MSEC(10)

/**
 * @brief Initial count for the flash command semaphore.
 *
 * The semaphore is initialized as unavailable (count = 0), ensuring that
 * the first attempt to take the semaphore will block until it is released
 * by the ISR after a flash operation completes.
 */
#define FLASH_MCHP_CMD_SEM_INIT_COUNT 0

/**
 * @brief Maximum count (limit) for the flash command semaphore.
 *
 * This value sets the maximum number of times the semaphore can be given.
 * A limit of 1 is typical for binary semaphore usage, ensuring only one
 * outstanding operation can be tracked at a time.
 */
#define FLASH_MCHP_CMD_SEM_LIMIT 1

/**
 * @brief Initialize a semaphore for flash operations.
 *
 * This macro initializes a semaphore with a specified initial count and limit.
 *
 * @param p_sem Pointer to the semaphore to be initialized.
 * @param init_count Initial count of the semaphore.
 * @param limit Maximum count of the semaphore.
 */
#define FLASH_MCHP_CMD_SEM_INIT(p_sem, init_count, limit) k_sem_init(p_sem, init_count, limit)

/**
 * @brief Acquire the flash semaphore.
 *
 * This macro attempts to take the flash semaphore, blocking for a specified timeout period.
 *
 * @param p_sem Pointer to the semaphore to be acquired.
 * @param FLASH_MCHP_CMD_SEM_TIMEOUT Timeout value for the semaphore operation.
 */
#define FLASH_MCHP_CMD_SEM_TAKE(p_sem) k_sem_take(p_sem, FLASH_MCHP_CMD_SEM_TIMEOUT)

/**
 * @brief Release the semaphore for flash operations.
 *
 * This macro releases a semaphore that is used to synchronize flash operations.
 *
 * @param p_sem Pointer to the semaphore to be released.
 */
#define FLASH_MCHP_CMD_SEM_GIVE(p_sem) k_sem_give(p_sem)

/*******************************************
 * Enum and typedefs
 *******************************************/
/**
 * @struct flash_mchp_dev_data
 * @brief Structure to hold flash device data.
 */
typedef struct flash_mchp_dev_data {

	/**< Pointer to the Flash device instance. */
	const struct device *dev;

	/**< Semaphore lock for flash APIs operations */
	FLASH_MCHP_LOCK_TYPE flash_data_lock;

	/**< Semaphore lock for flash interrupt operations */
	FLASH_MCHP_CMD_LOCK_TYPE flash_cmd_lock;

	/**< Array of pointers to NVM data registers */
	volatile uint32_t *nvmdata_regs[FLASH_MCHP_NVM_DATA_REGS_COUNT];

} flash_mchp_dev_data_t;

/**
 * @struct flash_mchp_dev_config
 * @brief Structure to hold flash device configuration.
 */
typedef struct flash_mchp_dev_config {

	/**< HAL for flash */
	nvm_registers_t *regs;

	/**< Flash base address */
	uint32_t base_addr;

	/**< Function to configure IRQ */
	void (*irq_config_func)(const struct device *dev);

	/**< Flash memory parameters */
	struct flash_parameters flash_param;

#ifdef CONFIG_FLASH_PAGE_LAYOUT
	/**< Flash pages layouts */
	struct flash_pages_layout flash_layout;
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

} flash_mchp_dev_config_t;

/**
 * @enum flash_op_mode_t
 * @brief Enumeration of internal flash operation types.
 *
 * Defines the various operation modes for internal flash memory control,
 * such as program, erase, and no operation.
 */
typedef enum {
	FLASH_MCHP_PROGRAM_ERASE_OPERATION = 0x7,     /**< Program and erase operation. */
	FLASH_MCHP_PAGE_ERASE_OPERATION = 0x4,        /**< Page erase operation. */
	FLASH_MCHP_ROW_PROGRAM_OPERATION = 0x3,       /**< Row program operation. */
	FLASH_MCHP_QUAD_WORD_PROGRAM_OPERATION = 0x2, /**< Quad word program operation. */
	FLASH_MCHP_WORD_PROGRAM_OPERATION = 0x1,      /**< Word program operation. */
	FLASH_MCHP_NO_OPERATION = 0x0,                /**< No operation. */
} flash_op_mode_t;

/*******************************************
 * Internal Flash Programming Unlock Keys
 ******************************************/
typedef enum {
	NVM_UNLOCK_KEY0 = 0x00000000,
	NVM_UNLOCK_KEY1 = 0xAA996655,
	NVM_UNLOCK_KEY2 = 0x556699AA
} NVM_UNLOCK_KEYS;

/*******************************************
 * Helper functions
 *******************************************/
/**
 * @brief Check if a given value is aligned to a specified alignment.
 *
 * This function determines whether the provided value is aligned to the
 * specified alignment boundary. Alignment is typically a power of two,
 * and this function checks if the value is a multiple of the alignment.
 *
 * @param value The value to be checked for alignment.
 * @param alignment The alignment boundary to check against. This should
 *                  be a power of two.
 *
 * @return FLASH_MCHP_SUCCESS if the value is aligned to the specified alignment,
 *         FLASH_MCHP_FAIL otherwise.
 */
static inline uint8_t flash_aligned(size_t value, size_t alignment)
{
	return (((value & (alignment - 1)) == 0) ? FLASH_MCHP_SUCCESS : FLASH_MCHP_FAIL);
}

/**
 * @brief Performs the unlock sequence for flash operations.
 *
 * Writes the required unlock sequence to the NVMKEY register. This sequence is
 * necessary to prevent accidental writes or erasures of Flash memory by ensuring
 * that only intentional operations are executed.
 */
static inline void flash_operation_unlock_sequence(const struct device *dev)
{
	/* Write the unlock key sequence. */
	FLASH_REGS->NVM_NVMKEY = (uint32_t)NVM_UNLOCK_KEY0;
	FLASH_REGS->NVM_NVMKEY = (uint32_t)NVM_UNLOCK_KEY1;
	FLASH_REGS->NVM_NVMKEY = (uint32_t)NVM_UNLOCK_KEY2;
}

/**
 * @brief Sets the NVM operation type in the NVM controller.
 *
 * Configures the type of operation to be performed by setting the corresponding
 * NVM operation bit fields in the NVM controller register, based on the specified
 * operation value of type flash_op_mode_t.
 *
 * @param dev Pointer to the device structure representing the flash controller.
 * @param operation The flash operation type to set (see flash_op_mode_t).
 *
 * @note This function directly accesses hardware registers.
 */
static inline void flash_set_internal_op_type(const struct device *dev, flash_op_mode_t operation)
{
	FLASH_REGS->NVM_NVMCONCLR = NVM_NVMCON_NVMOP_Msk;
	FLASH_REGS->NVM_NVMCONSET =
		(NVM_NVMCON_NVMOP_Msk & (((uint32_t)operation) << NVM_NVMCON_NVMOP_Pos));
}

/**
 * @brief Starts a flash memory operation at the specified address.
 *
 * Sets the target Flash address to be operated on (destination).
 * Specifies the type of operation to be performed on the flash and
 * follows the required unlock sequence before starting the operation.
 *
 * @param dev Pointer to the device structure representing the flash controller.
 * @param operation The type of flash operation to perform (e.g., erase, write).
 */
static void flash_start_operation_at_address(const struct device *dev, uint32_t address,
					     flash_op_mode_t operation)
{
	/* Set the target Flash address to be operated on (destination). */
	FLASH_REGS->NVM_NVMADDR = address;

	/* NVMOP can be written only when WREN is zero. So, clear WREN. */
	FLASH_REGS->NVM_NVMCONCLR = NVM_NVMCON_WREN_Msk;

	flash_set_internal_op_type(dev, operation);

	/* Set WREN to enable writes to the WR bit and to prevent NVMOP modification. */
	FLASH_REGS->NVM_NVMCONSET = NVM_NVMCON_WREN_Msk;

	flash_operation_unlock_sequence(dev);

	/* Start the operation. */
	FLASH_REGS->NVM_NVMCONSET = NVM_NVMCON_WR_Msk;
}

/**
 * @brief Initializes the flash controller for the specified device.
 *
 * Performs the basic initializations required for the flash peripheral.
 * This function should be called before performing any flash operations
 * to ensure the controller is properly configured.
 *
 * @param dev Pointer to the device structure representing the flash controller.
 */
static void flash_controller_init(const struct device *dev)
{
	flash_start_operation_at_address(dev, FLASH_REGS->NVM_NVMADDR, FLASH_MCHP_NO_OPERATION);

	flash_mchp_dev_data_t *mchp_flash_data = dev->data;

	/* Initialize the NVM data register pointers */
	mchp_flash_data->nvmdata_regs[0] = &FLASH_REGS->NVM_NVMDATA0;
	mchp_flash_data->nvmdata_regs[1] = &FLASH_REGS->NVM_NVMDATA1;
	mchp_flash_data->nvmdata_regs[2] = &FLASH_REGS->NVM_NVMDATA2;
	mchp_flash_data->nvmdata_regs[3] = &FLASH_REGS->NVM_NVMDATA3;
#if defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ6)
	mchp_flash_data->nvmdata_regs[4] = &FLASH_REGS->NVM_NVMDATA4;
	mchp_flash_data->nvmdata_regs[5] = &FLASH_REGS->NVM_NVMDATA5;
	mchp_flash_data->nvmdata_regs[6] = &FLASH_REGS->NVM_NVMDATA6;
	mchp_flash_data->nvmdata_regs[7] = &FLASH_REGS->NVM_NVMDATA7;
#endif
}

/**
 * @brief Waits until the current flash operation is complete.
 *
 * Blocks execution until the flash controller completes a program or erase operation.
 * An interrupt is generated when the WR bit is cleared by the flash controller,
 * indicating the operation has finished.
 *
 * @param dev Pointer to the device structure representing the flash controller.
 */
static int flash_wait_until_operation_complete(const struct device *dev)
{
	flash_mchp_dev_data_t *mchp_flash_data = dev->data;

	return FLASH_MCHP_CMD_SEM_TAKE(&mchp_flash_data->flash_cmd_lock);
}

/**
 * @brief Retrieves the current status of the flash operation.
 *
 * Checks the NVM controller status register for error conditions such as
 * low voltage detect error or write error. Logs an appropriate debug message
 * if an error is detected and returns a status code indicating success or failure.
 *
 * @param dev Pointer to the device structure representing the flash controller.
 * @return int Returns @c NVMCTRL_MCHP_SUCCESS if no error is detected,
 *             otherwise returns @c NVMCTRL_MCHP_FAIL.
 */
static int flash_get_error_status(const struct device *dev)
{
	int ret = FLASH_MCHP_SUCCESS;

	/* Combine all error masks */
	const uint16_t error_mask = NVM_NVMCON_LVDERR_Msk | NVM_NVMCON_WRERR_Msk;

	if ((FLASH_REGS->NVM_NVMCON & error_mask) != 0) {
		LOG_ERR("LVDERR or WRERR detected 0x%x", FLASH_REGS->NVM_NVMCON);
		ret = FLASH_MCHP_FAIL;
	}

	return ret;
}

/**
 * @brief Erases a flash memory page at the specified address.
 *
 * Initiates a page erase operation at the given flash memory address.
 * The function starts the erase operation and checks the status of the operation.
 *
 * @param dev Pointer to the device structure representing the flash controller.
 * @param address The address of the flash page to erase.
 * @return int Returns @c NVMCTRL_MCHP_SUCCESS if the operation was initiated successfully.
 *
 */
static int flash_erase_page(const struct device *dev, uint32_t address)
{
	int ret = FLASH_MCHP_SUCCESS;

	flash_start_operation_at_address(dev, address, FLASH_MCHP_PAGE_ERASE_OPERATION);

	do {
		ret = flash_wait_until_operation_complete(dev);
		if (ret != FLASH_MCHP_SUCCESS) {
			LOG_ERR("Flash operation timeout: WR bit not set, failed to acquire "
				"semaphore");
			break;
		}

		ret = flash_get_error_status(dev);
		if (ret != FLASH_MCHP_SUCCESS) {
			ret = FLASH_MCHP_FAIL;
		}
	} while (0);

	return ret;
}

/**
 * @brief Writes a single word to flash memory at the specified address.
 *
 * This function writes one word from the provided data pointer to the flash memory
 * register, then starts a word program operation at the given address.
 *
 * @param data    Pointer to the data to be written to flash.
 * @param address Flash memory address where the data will be written.
 *
 * @return Always returns true (non-zero) to indicate the operation was started.
 */
static int flash_word_write(const struct device *dev, const void *data, uint32_t address)
{
	int ret = FLASH_MCHP_SUCCESS;
	uint8_t num_words = FLASH_MCHP_WORD_SIZE / sizeof(uint32_t);
	const uint32_t *src = (const uint32_t *)data;
	flash_mchp_dev_data_t *mchp_flash_data = dev->data;

	for (uint8_t i = 0; i < num_words; i++) {
		*mchp_flash_data->nvmdata_regs[i] = src[i];
	}

	flash_start_operation_at_address(dev, address, FLASH_MCHP_WORD_PROGRAM_OPERATION);

	do {
		ret = flash_wait_until_operation_complete(dev);
		if (ret != FLASH_MCHP_SUCCESS) {
			LOG_ERR("Flash operation timeout: WR bit not set, failed to acquire "
				"semaphore");
			break;
		}

		ret = flash_get_error_status(dev);
		if (ret != FLASH_MCHP_SUCCESS) {
			ret = FLASH_MCHP_FAIL;
		}
	} while (0);

	return ret;
}

/**
 * @brief Writes a quadword (4 words) to flash memory at the specified address.
 *
 * This function writes four consecutive words from the provided data pointer
 * to the flash memory registers, then starts a quadword program operation at
 * the given address.
 *
 * @param dev     Pointer to the device structure representing the flash controller.
 * @param data    Pointer to the data to be written to flash.
 * @param address Flash memory address where the data will be written.
 *
 * @return int    Returns @c NVMCTRL_MCHP_SUCCESS if the operation was initiated successfully,
 *                or an error code if the operation failed.
 */
static inline int flash_quadword_write(const struct device *dev, const void *data, uint32_t address)
{
	int ret = FLASH_MCHP_SUCCESS;
	uint8_t num_words = FLASH_MCHP_QUAD_WORD_SIZE / sizeof(uint32_t);
	const uint32_t *src = (const uint32_t *)data;
	flash_mchp_dev_data_t *mchp_flash_data = dev->data;

	for (uint8_t i = 0; i < num_words; i++) {
		*mchp_flash_data->nvmdata_regs[i] = src[i];
	}

	flash_start_operation_at_address(dev, address, FLASH_MCHP_QUAD_WORD_PROGRAM_OPERATION);

	do {
		ret = flash_wait_until_operation_complete(dev);
		if (ret != FLASH_MCHP_SUCCESS) {
			LOG_ERR("Flash operation timeout: WR bit not set, failed to acquire "
				"semaphore");
			break;
		}

		ret = flash_get_error_status(dev);
		if (ret != FLASH_MCHP_SUCCESS) {
			ret = FLASH_MCHP_FAIL;
		}
	} while (0);

	return ret;
}

/**
 * @brief Writes a row of data to flash memory at the specified address.
 *
 * Sets the source address for the data to be written, initiates a row program
 * operation at the given flash address, and checks the operation status.
 *
 * @param dev     Pointer to the device structure representing the flash controller.
 * @param data    Pointer to the data to be written to flash.
 * @param address Flash memory address where the row will be written.
 * @return int    Returns @c NVMCTRL_MCHP_SUCCESS if the operation was initiated successfully,
 *                or an error code if the operation failed.
 */
static int flash_row_write(const struct device *dev, const void *data, uint32_t address)
{
	int ret = FLASH_MCHP_SUCCESS;

	FLASH_REGS->NVM_NVMSRCADDR = (uint32_t)data;

	flash_start_operation_at_address(dev, address, FLASH_MCHP_ROW_PROGRAM_OPERATION);

	do {
		ret = flash_wait_until_operation_complete(dev);
		if (ret != FLASH_MCHP_SUCCESS) {
			LOG_ERR("Flash operation timeout: WR bit not set, failed to acquire "
				"semaphore");
			break;
		}

		ret = flash_get_error_status(dev);
		if (ret != FLASH_MCHP_SUCCESS) {
			ret = FLASH_MCHP_FAIL;
		}
	} while (0);

	return ret;
}

/**
 * @brief Validate the range of a flash memory operation.
 *
 * This function checks whether the specified offset and length are within
 * the valid range of the flash memory. It ensures that the operation does
 * not start before the base address of the flash and does not extend beyond
 * the total flash size.
 *
 * @param offset The starting offset of the flash memory operation.
 * @param len The length of the flash memory operation.
 *
 * @return 0 if the range is valid, -EINVAL if the range is invalid.
 */
static int flash_valid_range(off_t offset, size_t len)
{
	int ret = FLASH_MCHP_SUCCESS;

	do {
		if (offset < SOC_NV_FLASH_BASE_ADDRESS) {
			LOG_WRN("0x%lx: before start of flash", (long)offset);
			ret = -EINVAL;
			break;
		}

		if ((offset + len) > (SOC_NV_FLASH_BASE_ADDRESS + SOC_NV_FLASH_SIZE)) {
			LOG_WRN("0x%lx: ends past the end of flash", (long)offset);
			ret = -EINVAL;
			break;
		}
	} while (0);

	return ret;
}

#ifdef CONFIG_FLASH_HAS_PARTIAL_PAGE_WRITE
/**
 * @brief Handle unaligned start of a flash write operation.
 *
 * This function performs a read-modify-write for the initial unaligned bytes
 * at the start of a flash write, using word-sized (4-byte) operations. It updates
 * the offset, buffer pointer, and remaining length to reflect the bytes written,
 * so that the caller can proceed with aligned writes.
 *
 * @param dev     Pointer to the device structure representing the flash controller.
 * @param offset  Pointer to the current offset in flash memory; updated by this function.
 * @param buffer  Pointer to the current data buffer pointer; updated by this function.
 * @param len     Pointer to the remaining length to write; updated by this function.
 *
 * @return FLASH_MCHP_SUCCESS (0) on success,
 *         or a negative error code on failure.
 */
static int flash_handle_unaligned_start(const struct device *dev, off_t *offset,
					const uint8_t **buffer, size_t *len)
{
	int ret = FLASH_MCHP_SUCCESS;

	if (flash_aligned(*offset, FLASH_MCHP_WORD_SIZE) != FLASH_MCHP_SUCCESS) {

		uint32_t aligned_addr = *offset & ~(FLASH_MCHP_WORD_SIZE - 1);
		uint8_t word_buf[FLASH_MCHP_WORD_SIZE];
		size_t start_offset = (*offset - aligned_addr);
		const uint8_t *src = (const uint8_t *)aligned_addr;
		size_t bytes_to_update = ((*len) < (FLASH_MCHP_WORD_SIZE - start_offset))
						 ? (*len)
						 : (FLASH_MCHP_WORD_SIZE - start_offset);

		/*  Read existing data. */
		for (size_t i = 0; i < FLASH_MCHP_WORD_SIZE; i++) {
			word_buf[i] = src[i];
		}

		/* Overwrite the relevant bytes. */
		for (size_t i = 0; i < bytes_to_update; i++) {
			word_buf[start_offset + i] = (*buffer)[i];
		}

		ret = flash_word_write(dev, word_buf, aligned_addr);
		if (ret != FLASH_MCHP_SUCCESS) {
			LOG_ERR("double word write failed at 0x%lx", (long)aligned_addr);
			return ret;
		}

		(*offset) += bytes_to_update;
		(*buffer) += bytes_to_update;
		(*len) -= bytes_to_update;
	}

	return ret;
}

/**
 * @brief Handles unaligned end of a flash write operation.
 *
 * This function performs a read-modify-write for the final unaligned bytes
 * at the end of a flash write. It does not update the offset, buffer, or length,
 * as it is intended to be called after all aligned writes are complete.
 *
 * @param dev     Pointer to the device structure representing the flash controller.
 * @param offset  Offset in flash memory where the unaligned write should begin.
 * @param buffer  Pointer to the data buffer containing the bytes to write.
 * @param len     Number of bytes to write at the end (less than a double word).
 *
 * @return FLASH_MCHP_SUCCESS (0) on success, or a negative error code on failure.
 */
static int flash_handle_unaligned_end(const struct device *dev, off_t offset, const uint8_t *buffer,
				      size_t len)
{
	int ret = FLASH_MCHP_SUCCESS;

	if (len > 0) {
		uint32_t aligned_addr = offset;
		uint8_t word_buf[FLASH_MCHP_WORD_SIZE];
		const uint8_t *src = (const uint8_t *)aligned_addr;

		/* Read existing data */
		for (size_t i = 0; i < FLASH_MCHP_WORD_SIZE; i++) {
			word_buf[i] = src[i];
		}

		/* Overwrite the relevant bytes. */
		for (size_t i = 0; i < len; i++) {
			word_buf[i] = buffer[i];
		}

		ret = flash_word_write(dev, word_buf, aligned_addr);
		if (ret != FLASH_MCHP_SUCCESS) {
			LOG_ERR("double word write failed at 0x%lx", (long)aligned_addr);
		}
	}
	return ret;
}
#endif /*CONFIG_FLASH_HAS_PARTIAL_PAGE_WRITE*/

/**
 * @brief Writes data to flash memory, handling both aligned and unaligned writes.
 *
 * This function writes a buffer of data to flash memory at the specified offset.
 * It attempts to use the most efficient write size possible (page, quad-word, word)
 * for aligned regions. If CONFIG_FLASH_HAS_PARTIAL_PAGE_WRITE is enabled, it also handles
 * unaligned start and end regions using read-modify-write operations.
 * The function locks the flash data structure during the operation to ensure thread safety,
 * and verifies the written data for correctness.
 *
 * @param dev    Pointer to the device structure representing the flash controller.
 * @param offset Offset in flash memory where the write should begin (relative to base address).
 * @param data   Pointer to the buffer containing the data to be written.
 * @param len    Number of bytes to write from the buffer.
 *
 * @return FLASH_MCHP_SUCCESS (0) on success,
 *         -EINVAL if alignment requirements are not met (when partial page write is not enabled),
 *         or a negative error code if a write or verification operation fails.
 */
static int flash_mchp_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
	int ret = FLASH_MCHP_SUCCESS;
	flash_mchp_dev_data_t *mchp_flash_data = dev->data;
	const uint8_t *buffer = (const uint8_t *)data;
	size_t verify_len = len;

	offset += DEV_CFG(dev)->base_addr;
	off_t verify_offset = offset;

	FLASH_MCHP_DATA_LOCK(&mchp_flash_data->flash_data_lock);

	do {
#ifdef CONFIG_FLASH_HAS_PARTIAL_PAGE_WRITE
		/* Handle unaligned start */
		ret = flash_handle_unaligned_start(dev, &offset, &buffer, &len);
		if (ret != FLASH_MCHP_SUCCESS) {
			break;
		}
#else
		ret = flash_aligned(offset, SOC_NV_FLASH_WRITE_BLOCK_SIZE);
		if (ret != FLASH_MCHP_SUCCESS) {
			LOG_WRN("0x%lx: not on a write block boundary", (long)offset);
			ret = -EINVAL;
			break;
		}

		ret = flash_aligned(len, SOC_NV_FLASH_WRITE_BLOCK_SIZE);
		if (ret != FLASH_MCHP_SUCCESS) {
			LOG_WRN("%zu: not a integer number of write blocks", len);
			ret = -EINVAL;
			break;
		}
#endif

		while (len >= FLASH_MCHP_WORD_SIZE) {
			/* 1024-byte page write */
			if ((len >= FLASH_MCHP_PAGE_SIZE) &&
			    (flash_aligned(offset, FLASH_MCHP_PAGE_SIZE) == FLASH_MCHP_SUCCESS)) {
				ret = flash_row_write(dev, buffer, offset);
				if (ret != FLASH_MCHP_SUCCESS) {
					LOG_ERR("page write failed at 0x%lx", (long)offset);
					break;
				}
				offset += FLASH_MCHP_PAGE_SIZE;
				buffer += FLASH_MCHP_PAGE_SIZE;
				len -= FLASH_MCHP_PAGE_SIZE;
				continue;
			}

			/* 16-byte quad-word write */
			if ((len >= FLASH_MCHP_QUAD_WORD_SIZE) &&
			    (flash_aligned(offset, FLASH_MCHP_QUAD_WORD_SIZE) ==
			     FLASH_MCHP_SUCCESS)) {
				ret = flash_quadword_write(dev, buffer, offset);
				if (ret != FLASH_MCHP_SUCCESS) {
					LOG_ERR("quad word write failed at 0x%lx", (long)offset);
					break;
				}
				offset += FLASH_MCHP_QUAD_WORD_SIZE;
				buffer += FLASH_MCHP_QUAD_WORD_SIZE;
				len -= FLASH_MCHP_QUAD_WORD_SIZE;
				continue;
			}

			/* 4-byte word write */
			if ((len >= FLASH_MCHP_WORD_SIZE) &&
			    (flash_aligned(offset, FLASH_MCHP_WORD_SIZE) == FLASH_MCHP_SUCCESS)) {
				ret = flash_word_write(dev, buffer, offset);
				if (ret != FLASH_MCHP_SUCCESS) {
					LOG_ERR("double word write failed at 0x%lx", (long)offset);
					break;
				}
				offset += FLASH_MCHP_WORD_SIZE;
				buffer += FLASH_MCHP_WORD_SIZE;
				len -= FLASH_MCHP_WORD_SIZE;
				continue;
			}

			break;
		}

#if defined(CONFIG_FLASH_HAS_PARTIAL_PAGE_WRITE)
		/* Handle unaligned end */
		if (ret == FLASH_MCHP_SUCCESS) {
			ret = flash_handle_unaligned_end(dev, offset, buffer, len);
			if (ret != FLASH_MCHP_SUCCESS) {
				break;
			}
		}
#endif
		/* Only verify if all previous steps succeeded */
		if (ret == FLASH_MCHP_SUCCESS) {
			for (size_t i = 0; i < verify_len; i++) {
				if (((const uint8_t *)data)[i] !=
				    ((const uint8_t *)verify_offset)[i]) {
					LOG_ERR("verify error at offset 0x%lx",
						(long)verify_offset + i);
					ret = -EINVAL;
					break;
				}
			}
		}

	} while (0);

	FLASH_MCHP_DATA_UNLOCK(&mchp_flash_data->flash_data_lock);
	return ret;
}

/**
 * @brief Erases a specified region of flash memory.
 *
 * This function erases blocks of flash memory starting from the given offset and covering the
 * specified size. The operation ensures that the offset and size are aligned with the page
 * boundaries and sizes.
 *
 * @param dev Pointer to the device structure representing the flash controller.
 * @param offset The starting offset in flash memory where the erase operation will begin.
 * @param size The total size of the memory to be erased, in bytes.
 * @return Returns NVMCTRL_MCHP_SUCCESS upon successful completion, or a negative error code if the
 * operation fails.
 *
 * @note The function locks the flash data to prevent other threads from accessing the flash API
 * during the operation. It is important to ensure that the offset and size are aligned with the
 * page boundaries and sizes.
 */

static int flash_mchp_erase(const struct device *dev, off_t offset, size_t size)
{
	int ret = FLASH_MCHP_SUCCESS;
	flash_mchp_dev_data_t *mchp_flash_data = dev->data;
	uint32_t page_size = SOC_NV_FLASH_ERASE_BLOCK_SIZE;

	offset += DEV_CFG(dev)->base_addr;

	FLASH_MCHP_DATA_LOCK(&mchp_flash_data->flash_data_lock);

	do {
		ret = flash_valid_range(offset, size);
		if (ret != FLASH_MCHP_SUCCESS) {
			break;
		}

		ret = flash_aligned(offset, page_size);
		if (ret != FLASH_MCHP_SUCCESS) {
			LOG_WRN("0x%lx: not on a erase block boundary", (long)offset);
			ret = -EINVAL;
			break;
		}

		ret = flash_aligned(size, page_size);
		if (ret != FLASH_MCHP_SUCCESS) {
			LOG_WRN("%zu: not a integer number of erase blocks", size);
			ret = -EINVAL;
			break;
		}

		while (size > 0U) {
			/* Erase the page */
			ret = flash_erase_page(dev, offset);
			if (ret != FLASH_MCHP_SUCCESS) {
				LOG_ERR("erase operation failed at 0x%lx", (long)offset);
				ret = -EIO;
				break;
			}

			/* Update size and offset for the next pages */
			size -= page_size;
			offset += page_size;
		}

	} while (0);

	FLASH_MCHP_DATA_UNLOCK(&mchp_flash_data->flash_data_lock);

	return ret;
}

/**
 * @brief Read data from the flash memory.
 *
 * This function reads a specified number of bytes from the flash memory
 * at a given offset and copies it into the provided buffer.
 *
 * @param dev Pointer to the device structure representing the flash controller.
 * @param offset Offset in the flash memory from which to start reading.
 * @param data Pointer to the buffer where the read data will be stored.
 * @param len Number of bytes to read from the flash memory.
 *
 * @return Returns NVMCTRL_MCHP_SUCCESS upon successful completion, or a negative error code if the
 * operation fails.
 */
static int flash_mchp_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	int ret = FLASH_MCHP_SUCCESS;
	uint32_t flash_base_addr = DEV_CFG(dev)->base_addr;

	offset += flash_base_addr;

	do {
		ret = flash_valid_range(offset, len);
		if (ret != FLASH_MCHP_SUCCESS) {
			break;
		}

		uint8_t *dst = (uint8_t *)data;
		const uint8_t *src = (const uint8_t *)offset;

		for (size_t i = 0; i < len; i++) {
			dst[i] = src[i];
		}

	} while (0);

	return ret;
}

/**
 * @brief Retrieve the flash parameters for a given device.
 *
 * This function provides access to the flash parameters associated with
 * a specific flash device. The parameters include details such as the
 * minimal write alignment and size, device capabilities, and the value
 * used to fill erased areas of the flash memory.
 *
 * @param dev       Pointer to the device structure representing the flash controller.
 *
 * @return Pointer to a `flash_parameters` structure containing the flash
 *         device's parameters. The returned structure includes:
 *         - `write_block_size`: The minimal write alignment and size.
 *         - `caps.no_explicit_erase`: Indicates whether the device requires
 *           explicit erase operations or not.
 *         - `erase_value`: The value used to fill erased areas of the flash memory.
 */
static const struct flash_parameters *flash_mchp_get_parameters(const struct device *dev)
{
	return (&DEV_CFG(dev)->flash_param);
}

#ifdef CONFIG_FLASH_PAGE_LAYOUT
/**
 * @brief Retrieve the flash page layout for a Microchip NVM controller.
 *
 * This function provides the layout of flash pages for the specified device.
 * It retrieves the page layout and the size of the layout from the device's
 * configuration and assigns them to the provided pointers.
 *
 * @param dev Pointer to the device structure representing the flash device.
 * @param layout Pointer to store the address of the flash pages layout array.
 * @param layout_size Pointer to store the size of the flash pages layout array.
 *
 */
static void flash_mchp_page_layout(const struct device *dev,
				   const struct flash_pages_layout **layout, size_t *layout_size)
{
	*layout = &(DEV_CFG(dev)->flash_layout);
	*layout_size = FLASH_MCHP_LAYOUT_SIZE;
}
#endif /*CONFIG_FLASH_PAGE_LAYOUT*/

/**
 * @brief Flash controller interrupt service routine.
 *
 * Handles interrupts generated by the flash controller. This function releases
 * the command semaphore to signal the completion of a flash operation.
 *
 * @param dev Pointer to the NVM controller device structure.
 */
static void flash_mchp_isr(const struct device *dev)
{
	flash_mchp_dev_data_t *mchp_flash_data = dev->data;

	FLASH_MCHP_CMD_SEM_GIVE(&(mchp_flash_data->flash_cmd_lock));
}

/**
 * @brief Initializes the Microchip flash memory controller.
 *
 * This function sets up the necessary resources and configurations for the
 * Microchip flash memory controller to operate. It initializes mutexes and
 * semaphores, enables the clock for the controller, configures interrupts,
 * and performs any necessary hardware initialization.
 *
 * @param dev Pointer to the device structure for the flash memory controller.
 *                    This structure contains both configuration and runtime data
 *                    necessary for initialization.
 *
 * @return Returns 0 upon successful initialization.
 *
 */
static int flash_mchp_init(const struct device *dev)
{
	const flash_mchp_dev_config_t *const mchp_flash_cfg = DEV_CFG(dev);
	flash_mchp_dev_data_t *mchp_flash_data = dev->data;

	FLASH_MCHP_DATA_MUTEX_INIT(&(mchp_flash_data->flash_data_lock));

	FLASH_MCHP_CMD_SEM_INIT(&(mchp_flash_data->flash_cmd_lock), FLASH_MCHP_CMD_SEM_INIT_COUNT,
				FLASH_MCHP_CMD_SEM_LIMIT);

	mchp_flash_cfg->irq_config_func(dev);

	flash_controller_init(dev);

	return 0;
}

/**
 * @brief FLASH driver API structure.
 */
static const struct flash_driver_api flash_mchp_driver_api = {
	.write = flash_mchp_write,
	.read = flash_mchp_read,
	.erase = flash_mchp_erase,
	.get_parameters = flash_mchp_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_mchp_page_layout,
#endif /*CONFIG_FLASH_PAGE_LAYOUT*/
};

/**
 * @brief Declare the FLASH IRQ handler.
 *
 * @param n Instance number.
 */
#define FLASH_MCHP_IRQ_HANDLER_DECL(n)                                                             \
	static void flash_mchp_irq_config_##n(const struct device *dev)

/**
 * @brief Define and connect the FLASH IRQ handler for a given instance.
 *
 * This macro defines the IRQ configuration function for the specified FLASH instance,
 * connects the FLASH interrupt to its handler, and enables the IRQ.
 *
 * @param n Instance number.
 */
#define FLASH_MCHP_IRQ_HANDLER(n)                                                                  \
	static void flash_mchp_irq_config_##n(const struct device *dev)                            \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, 0, irq), DT_INST_IRQ_BY_IDX(n, 0, priority),     \
			    flash_mchp_isr, DEVICE_DT_INST_GET(n), 0);                             \
		irq_enable(DT_INST_IRQ_BY_IDX(n, 0, irq));                                         \
	}

/**
 * @brief Configures the flash memory page layout for the FLASH.
 *
 */
#ifdef CONFIG_FLASH_PAGE_LAYOUT
#define FLASH_LAYOUT                                                                               \
	.flash_layout = {.pages_count = SOC_NV_FLASH_SIZE / SOC_NV_FLASH_ERASE_BLOCK_SIZE,         \
			 .pages_size = SOC_NV_FLASH_ERASE_BLOCK_SIZE},
#else
#define FLASH_LAYOUT
#endif /*CONFIG_FLASH_PAGE_LAYOUT*/

/*
 * @brief Define the FLASH configuration.
 *
 * @param n Instance number.
 */
#define FLASH_MCHP_CONFIG_DEFN(n)                                                                  \
	static const flash_mchp_dev_config_t flash_mchp_config_##n = {                             \
		.regs = (nvm_registers_t *)DT_INST_REG_ADDR(n),                                    \
		.base_addr = SOC_NV_FLASH_BASE_ADDRESS,                                            \
		.irq_config_func = flash_mchp_irq_config_##n,                                      \
		.flash_param = {.write_block_size = SOC_NV_FLASH_WRITE_BLOCK_SIZE,                 \
				.caps = {.no_explicit_erase = false},                              \
				.erase_value = FLASH_ERASE_DEFAULT_VALUE},                         \
		FLASH_LAYOUT}

/**
 * @brief Macro to define the flash data structure for a specific instance.
 *
 * This macro defines the flash data structure for a specific instance of the Microchip flash
 * device.
 *
 * @param n Instance number.
 */
#define FLASH_MCHP_DATA_DEFN(n) static flash_mchp_dev_data_t flash_mchp_data_##n

/**
 * @brief Macro to define the device structure for a specific instance of the flash device.
 *
 * This macro defines the device structure for a specific instance of the Microchip flash device.
 * It uses the DEVICE_DT_INST_DEFINE macro to create the device instance with the specified
 * initialization function, data structure, configuration structure, and driver API.
 *
 * @param n Instance number.
 */
#define FLASH_MCHP_DEVICE_DT_DEFN(n)                                                               \
	DEVICE_DT_INST_DEFINE(n, flash_mchp_init, NULL, &flash_mchp_data_##n,                      \
			      &flash_mchp_config_##n, POST_KERNEL,                                 \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &flash_mchp_driver_api)

/**
 * @brief Initialize the FLASH device.
 *
 * @param n Instance number.
 */
#define FLASH_MCHP_DEVICE_INIT(n)                                                                  \
	FLASH_MCHP_IRQ_HANDLER_DECL(n);                                                            \
	FLASH_MCHP_CONFIG_DEFN(n);                                                                 \
	FLASH_MCHP_DATA_DEFN(n);                                                                   \
	FLASH_MCHP_DEVICE_DT_DEFN(n);                                                              \
	FLASH_MCHP_IRQ_HANDLER(n);

/**
 * @brief Initialize all FLASH instances.
 */
DT_INST_FOREACH_STATUS_OKAY(FLASH_MCHP_DEVICE_INIT)
