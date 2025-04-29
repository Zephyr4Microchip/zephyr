/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file flash_mchp_nvmctrl_u2409.c
 * @brief Flash driver for Microchip NVMCTRL_U2409 peripheral.
 *
 * Implements Zephyr Flash API support with basic flash memory
 * operations.
 *
 * Supported SoC Families:
 * - SOC_FAMILY_MCHP_SAM_D5X_E5X
 */

#include <soc.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include <zephyr/drivers/flash/flash_mchp_api_extensions.h>

LOG_MODULE_REGISTER(flash_mchp_nvmctrl_u2409);

/*******************************************
 * Const and Macro Defines
 *******************************************/
/* Device tree compatible string for Microchip nvmctrl u2409. */
#define DT_DRV_COMPAT microchip_nvmctrl_u2409

/* Number of lock regions in the SoC non-volatile flash. */
#define SOC_NV_FLASH_LOCK_REGIONS DT_INST_PROP(0, lock_regions)

/* Size of each lock region in the SoC non-volatile flash. */
#define SOC_NV_FLASH_LOCK_REGION_SIZE ((SOC_NV_FLASH_SIZE) / (SOC_NV_FLASH_LOCK_REGIONS))

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

/* Device tree node identifier for the user row region of SoC non-volatile flash. */
#define SOC_NV_USERROW_NODE DT_INST(1, soc_nv_flash)

/* Size of the userpage region in the SoC non-volatile flash, in bytes. */
#define SOC_NV_USERROW_SIZE DT_REG_SIZE(SOC_NV_USERROW_NODE)

/* Base address of the userpage region in the SoC non-volatile flash. */
#define SOC_NV_USERROW_BASE_ADDR DT_REG_ADDR(SOC_NV_USERROW_NODE)

/* Write block size of the userpage region, in bytes. */
#define SOC_NV_USERROW_WRITE_BLOCK_SIZE DT_PROP(SOC_NV_USERROW_NODE, write_block_size)

/* Erase block size of the userpage region, in bytes. */
#define SOC_NV_USERROW_ERASE_BLOCK_SIZE DT_PROP(SOC_NV_USERROW_NODE, erase_block_size)

/* Number of flash page layouts supported by the MCHP flash driver. */
#define FLASH_MCHP_LAYOUT_SIZE 0x1

/* Size of a double word in bytes for MCHP flash. */
#define FLASH_MCHP_DOUBLE_WORD_SIZE 0x8

/* Size of a quad word in bytes for MCHP flash. */
#define FLASH_MCHP_QUAD_WORD_SIZE 0x10

/* Size of a page in bytes for MCHP flash. */
#define FLASH_MCHP_PAGE_SIZE 0x200

/* Device config */
#define DEV_CFG(dev) ((const flash_mchp_dev_config_t *const)(dev)->config)

/* NVMCTRL Register */
#define NVM_REGS ((const flash_mchp_dev_config_t *)(dev)->config)->regs

/**< Enable this for getting the debug prints. */
/*#define FLASH_DBG_MODE*/
#ifdef FLASH_DBG_MODE
#define DBG_FLASH(format, ...) printf(format, ##__VA_ARGS__)
#else
#define DBG_FLASH(format, ...)
#endif /*FLASH_DBG_MODE*/

/** @brief Default value of flash memory after an erase operation. */
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

/**< Encodes the write mode value for the NVMCTRL_CTRLA register. */
#define FLASH_SET_WMODE(mode) ((mode) << NVMCTRL_CTRLA_WMODE_Pos)

/**
 * @brief Calculate the address in flash memory.
 *
 * This macro computes the address in flash memory by adding the specified
 * offset to the base address of the flash memory.
 *
 * @param a Offset to be added to the base address of the flash memory.
 */
#define FLASH_MEMORY(a) ((uint32_t *)((uint8_t *)((a) + SOC_NV_FLASH_BASE_ADDRESS)))

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

/*******************************************
 * Enum and typedefs
 *******************************************/
/**
 * @struct flash_mchp_clock
 * @brief Structure to hold device clock configuration.
 */
typedef struct flash_mchp_clock {
	const struct device *clock_dev;       /**< Clock driver */
	clock_control_mchp_subsys_t mclk_sys; /**< Main clock subsystem. */
} flash_mchp_clock_t;

/**
 * @struct flash_mchp_dev_data
 * @brief Structure to hold flash device data.
 */
typedef struct flash_mchp_dev_data {

	/**< Pointer to the Flash device instance. */
	const struct device *dev;

	/**< Semaphore lock for flash APIs operations */
	FLASH_MCHP_LOCK_TYPE flash_data_lock;

	/**< Stores the current interrupt flag status */
	volatile uint16_t interrupt_flag_status;

} flash_mchp_dev_data_t;

/**
 * @struct flash_mchp_dev_config
 * @brief Structure to hold flash device configuration.
 */
typedef struct flash_mchp_dev_config {

	/**< HAL for flash */
	nvmctrl_registers_t *regs;

	/**< Flash base address */
	uint32_t base_addr;

	/**< Minimum erase block size */
	uint32_t erase_block_size;

	/**< Flash clock control */
	flash_mchp_clock_t flash_clock;

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
 * @enum flash_mchp_write_mode_t
 * @brief Enumeration for Flash write modes.
 *
 * This enumeration defines the different write modes available for the
 * Flash. Each mode specifies how data is written to the non-volatile memory.
 */
typedef enum {
	NVMCTRL_WMODE_MAN, /**< Manual Write Mode */
	NVMCTRL_WMODE_ADW, /**< Automatic Double Word Write Mode */
	NVMCTRL_WMODE_AQW, /**< Automatic Quad Word Write Mode */
	NVMCTRL_WMODE_AP   /**< Automatic Page Write Mode */
} flash_mchp_write_mode_t;

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
 * @return true if the value is aligned to the specified alignment, false otherwise.
 */
static inline bool flash_aligned(size_t value, size_t alignment)
{
	return (value & (alignment - 1)) == 0;
}

/**
 * @brief Initializes the NVMCTRL module with automatic wait state generation.
 *
 * This function configures the NVMCTRL_CTRLA register to enable automatic wait
 * state generation by enabling the automatic wait state mask (AUTOWS).
 *
 * @param dev Pointer to the device structure representing the flash controller.
 */
static inline void flash_enable_auto_wait_state(const struct device *dev)
{
	/* Automatic wait state generation */
	NVM_REGS->NVMCTRL_CTRLA = NVMCTRL_CTRLA_AUTOWS_Msk;
}

/**
 * @brief Enable NVMCTRL interrupt.
 *
 * @param dev Pointer to the device structure representing the flash controller.
 *
 */
static inline void flash_interrupt_enable(const struct device *dev)
{
	uint16_t interrupt_enable_flag = NVMCTRL_INTENSET_ADDRE_Msk | NVMCTRL_INTENSET_PROGE_Msk |
					 NVMCTRL_INTENSET_LOCKE_Msk | NVMCTRL_INTENSET_NVME_Msk;

	NVM_REGS->NVMCTRL_INTENSET = interrupt_enable_flag;
}

/**
 * @brief Initializes the flash controller for the specified device.
 *
 * This function enables the automatic wait state and interrupt for the flash
 * controller associated with the given device. It should be called before
 * performing any flash operations to ensure the controller is properly configured.
 *
 * @param dev Pointer to the device structure representing the flash controller.
 */
static void flash_controller_init(const struct device *dev)
{
	flash_enable_auto_wait_state(dev);
	flash_interrupt_enable(dev);
}

/**
 * @brief Set the write mode for the NVMCTRL peripheral.
 *
 * This function configures the write mode of the NVMCTRL (Non-Volatile Memory
 * Controller) by updating the NVMCTRL_CTRLA register with the specified mode.
 * The function ensures that only the write mode bits are modified, preserving
 * the other bits in the register.
 *
 * @param dev Pointer to the device structure representing the flash controller.
 * @param mode Write mode to set for the NVMCTRL.
 */
static inline void flash_set_write_mode(const struct device *dev, flash_mchp_write_mode_t mode)
{
	uint16_t mode_value = FLASH_SET_WMODE(mode);

	NVM_REGS->NVMCTRL_CTRLA =
		(uint16_t)((NVM_REGS->NVMCTRL_CTRLA & (~NVMCTRL_CTRLA_WMODE_Msk)) | mode_value);
}

/**
 * @brief Retrieve and clear the interrupt flag status of the NVMCTRL
 * peripheral.
 *
 * This function reads the current interrupt flag status from the
 * NVMCTRL_INTFLAG register and then clears the interrupt flags by writing the
 * same value back to the register. This operation is typically used to
 * acknowledge and clear interrupt flags.
 *
 * @param dev Pointer to the device structure representing the flash controller.
 *
 */
static inline void flash_clear_interrupt_flag(const struct device *dev)
{
	flash_mchp_dev_data_t *mchp_flash_data = dev->data;

	mchp_flash_data->interrupt_flag_status = NVM_REGS->NVMCTRL_INTFLAG;
	/* Clear NVMCTRL INTFLAG register */
	NVM_REGS->NVMCTRL_INTFLAG = mchp_flash_data->interrupt_flag_status;
}

/**
 * @brief Retrieve and report the error status of the NVM controller.
 *
 * This function examines the interrupt flag status of the NVMCTRL (Non-Volatile
 * Memory Controller) to determine if any errors have occurred. It checks for
 * address, programming, lock, and NVM errors The function returns a success or
 * failure code based on the presence of errors.
 *
 * @return Returns `FLASH_MCHP_SUCCESS` if no errors are detected, or
 *         `FLASH_MCHP_FAIL` if any error flags are set.
 */
static int flash_get_interrupt_status_error(const struct device *dev)
{
	int ret = FLASH_MCHP_SUCCESS;

	flash_mchp_dev_data_t *mchp_flash_data = dev->data;
	uint16_t status = mchp_flash_data->interrupt_flag_status;

	/* Combine all error masks */
	const uint16_t error_mask = NVMCTRL_INTFLAG_ADDRE_Msk | NVMCTRL_INTFLAG_PROGE_Msk |
				    NVMCTRL_INTFLAG_LOCKE_Msk | NVMCTRL_INTFLAG_NVME_Msk;

	if ((status & error_mask) != 0) {
		LOG_ERR("flash operation failed with status 0x%x", status);
		ret = FLASH_MCHP_FAIL;
	}

	return ret;
}

/**
 * @brief Block until the NVMCTRL indicates it is ready.
 *
 * This function continuously checks the NVMCTRL status register until the
 * "ready" bit is set, indicating that the NVMCTRL is no longer busy with
 * programming or erasing operations and is ready for a new command.
 *
 * @param dev Pointer to the device structure representing the flash controller.
 *
 * @note This function blocks execution until the NVMCTRL is ready.
 */
static inline void flash_status_ready_wait(const struct device *dev)
{
	while (((NVM_REGS->NVMCTRL_STATUS & NVMCTRL_STATUS_READY_Msk) == 0)) {
		DBG_FLASH("NVM controller is busy programming or erasing.");
	}
}

/**
 * @brief Executes a flash memory controller command.
 *
 * Combines the specified command with the required command execution key and writes
 * the result to the NVM controller's control register to initiate the desired flash operation.
 *
 * @param dev Pointer to the device structure representing the flash controller.
 * @param command The flash controller command to execute (e.g., erase, unlock, write).
 */
static inline void flash_process_command(const struct device *dev, uint32_t command)
{
	NVM_REGS->NVMCTRL_CTRLB = command | NVMCTRL_CTRLB_CMDEX_KEY;
}

/**
 * @brief Issue a command to clear the flash page buffer.
 *
 * This function sends the Page Buffer Clear (PBC) command to the flash controller,
 * preparing the page buffer for a new write operation.
 *
 * @param dev Pointer to the device structure representing the flash controller.
 */
static inline void flash_pagebuffer_clear(const struct device *dev)
{
	flash_process_command(dev, NVMCTRL_CTRLB_CMD_PBC);
}

/**
 * @brief Write a quad word (128 bits) to flash memory.
 *
 * This function writes a quad word (typically 128 bits) to the specified flash memory address.
 * The data is written in 32-bit chunks, as required by the flash controller's page buffer.
 * The function waits for the flash to be ready before writing and checks the status after the
 * operation.
 *
 * @param dev     Pointer to the device structure representing the flash controller.
 * @param data    Pointer to the source data to be written (must be at least 128 bits).
 * @param address Destination address in flash memory where the data will be written.
 *
 * @retval FLASH_MCHP_SUCCESS if the write operation is successful.
 * @retval FLASH_MCHP_FAIL if the write operation fails.
 */
static int flash_quadword_write(const struct device *dev, const void *data, uint32_t address)
{
	int ret = FLASH_MCHP_SUCCESS;
	uint8_t num_words = FLASH_MCHP_QUAD_WORD_SIZE / 4U;
	const uint32_t *src = (const uint32_t *)data;
	uint32_t *dst = FLASH_MEMORY(address);

	flash_pagebuffer_clear(dev);

	flash_set_write_mode(dev, NVMCTRL_WMODE_AQW);

	/* writing 32-bit data into the given address. Writes to the page buffer must be 32 bits */
	for (uint8_t i = 0U; i < num_words; i++) {
		*dst = *src;
		dst++;
		src++;
	}

	flash_status_ready_wait(dev);

	ret = flash_get_interrupt_status_error(dev);
	if (ret != 0) {
		ret = FLASH_MCHP_FAIL;
	}

	return ret;
}

#ifdef CONFIG_FLASH_HAS_PARTIAL_PAGE_WRITE
/**
 * @brief Write a double word (64 bits) to flash memory.
 *
 * This function writes a double word (typically 64 bits) to the specified flash memory address.
 * The data is written in 32-bit chunks, as required by the flash controller's page buffer.
 * The function waits for the flash to be ready before writing and checks the status after the
 * operation.
 *
 * @param dev     Pointer to the device structure representing the flash controller.
 * @param data    Pointer to the source data to be written (must be at least 64 bits).
 * @param address Destination address in flash memory where the data will be written.
 *
 * @retval FLASH_MCHP_SUCCESS if the write operation is successful.
 * @retval FLASH_MCHP_FAIL if the write operation fails.
 */
static int flash_doubleword_write(const struct device *dev, const void *data, uint32_t address)
{
	int ret = FLASH_MCHP_SUCCESS;
	uint8_t num_words = FLASH_MCHP_DOUBLE_WORD_SIZE / 4U;
	const uint32_t *src = (const uint32_t *)data;
	uint32_t *dst = FLASH_MEMORY(address);

	flash_pagebuffer_clear(dev);

	flash_set_write_mode(dev, NVMCTRL_WMODE_ADW);

	/* writing 32-bit data into the given address. Writes to the page buffer must be 32 bits */
	for (uint8_t i = 0U; i < num_words; i++) {
		*dst = *src;
		dst++;
		src++;
	}

	flash_status_ready_wait(dev);

	ret = flash_get_interrupt_status_error(dev);
	if (ret != 0) {
		ret = FLASH_MCHP_FAIL;
	}

	return ret;
}

/**
 * @brief Writes a partial buffer to flash memory, handling unaligned addresses and sizes.
 *
 * This function writes a buffer of arbitrary length to flash memory, starting at the specified
 * offset. It handles cases where the start or end of the write is not aligned to the flash
 * double-word (8 bytes) or quad-word (16 bytes) boundaries by reading, modifying, and writing
 * the necessary portions. The function uses double-word and quad-word writes as appropriate
 * to maximize efficiency and ensure data integrity.
 *
 * The function performs the following steps:
 * - Handles unaligned start by updating only the necessary bytes in the first double-word.
 * - Writes aligned double-words until the address is quad-word aligned.
 * - Writes as many quad-words as possible for efficiency.
 * - Writes any remaining double-words.
 * - Handles unaligned end by updating only the necessary bytes in the last double-word.
 *
 * @param dev    Pointer to the device structure representing the flash controller.
 * @param buffer Pointer to the data buffer to be written.
 * @param len    Number of bytes to write from the buffer.
 * @param offset Offset in flash memory where the write should begin.
 *
 * @return FLASH_MCHP_SUCCESS (0) on success, or FLASH_MCHP_FAIL on failure.
 */
static int flash_partial_write(const struct device *dev, const uint8_t *buffer, size_t len,
			       const uint32_t offset)
{
	int ret = FLASH_MCHP_SUCCESS;
	uint32_t addr = offset;
	uint32_t aligned_addr = addr & (~(FLASH_MCHP_DOUBLE_WORD_SIZE - 1));

	do {
		/* Handle unaligned start (if addr is not 8-byte aligned) */
		if ((flash_aligned(addr, FLASH_MCHP_DOUBLE_WORD_SIZE) == 0) && (len > 0)) {
			uint8_t doubleword_buf[FLASH_MCHP_DOUBLE_WORD_SIZE];
			size_t start_offset = addr - aligned_addr;
			size_t bytes_to_update =
				(len < (FLASH_MCHP_DOUBLE_WORD_SIZE - start_offset))
					? len
					: (FLASH_MCHP_DOUBLE_WORD_SIZE - start_offset);

			/* Read existing data into doubleword_buf */
			const uint8_t *src = (const uint8_t *)aligned_addr;

			for (size_t i = 0; i < FLASH_MCHP_DOUBLE_WORD_SIZE; i++) {
				doubleword_buf[i] = src[i];
			}

			/* Overwrite the relevant bytes. */
			for (size_t i = 0; i < bytes_to_update; i++) {
				doubleword_buf[start_offset + i] = buffer[i];
			}

			ret = flash_doubleword_write(dev, doubleword_buf, aligned_addr);
			if (ret != 0) {
				LOG_ERR("double word write failed at 0x%lx", (long)aligned_addr);
				break;
			}

			aligned_addr += FLASH_MCHP_DOUBLE_WORD_SIZE;
			buffer += bytes_to_update;
			len -= bytes_to_update;
		}

		/* Do 8-byte writes until we reach a 16-byte aligned address. */
		while ((len >= FLASH_MCHP_DOUBLE_WORD_SIZE) &&
		       (flash_aligned(aligned_addr, FLASH_MCHP_QUAD_WORD_SIZE) == 0)) {
			ret = flash_doubleword_write(dev, buffer, aligned_addr);
			if (ret != 0) {
				LOG_ERR("double word write failed at 0x%lx", (long)aligned_addr);
				break;
			}

			aligned_addr += FLASH_MCHP_DOUBLE_WORD_SIZE;
			buffer += FLASH_MCHP_DOUBLE_WORD_SIZE;
			len -= FLASH_MCHP_DOUBLE_WORD_SIZE;
		}

		if (ret != 0) {
			break;
		}

		/* Write as many 16-byte aligned chunks as possible. */
		while ((len >= FLASH_MCHP_QUAD_WORD_SIZE) &&
		       (flash_aligned(aligned_addr, FLASH_MCHP_QUAD_WORD_SIZE) != 0)) {
			ret = flash_quadword_write(dev, buffer, aligned_addr);
			if (ret != 0) {
				LOG_ERR("quad word write failed at 0x%lx", (long)aligned_addr);
				break;
			}

			aligned_addr += FLASH_MCHP_QUAD_WORD_SIZE;
			buffer += FLASH_MCHP_QUAD_WORD_SIZE;
			len -= FLASH_MCHP_QUAD_WORD_SIZE;
		}

		if (ret != 0) {
			break;
		}

		/* Write any remaining 8-byte chunks. */
		while (len >= FLASH_MCHP_DOUBLE_WORD_SIZE) {
			ret = flash_doubleword_write(dev, buffer, aligned_addr);
			if (ret != 0) {
				LOG_ERR("double word write failed at 0x%lx", (long)aligned_addr);
				break;
			}

			aligned_addr += FLASH_MCHP_DOUBLE_WORD_SIZE;
			buffer += FLASH_MCHP_DOUBLE_WORD_SIZE;
			len -= FLASH_MCHP_DOUBLE_WORD_SIZE;
		}

		if (ret != 0) {
			break;
		}

		/* Handle unaligned end (if any bytes remain) */
		if (len > 0) {
			uint8_t doubleword_buf[FLASH_MCHP_DOUBLE_WORD_SIZE];

			/* Read existing data into doubleword_buf */
			const uint8_t *src = (const uint8_t *)aligned_addr;

			for (size_t i = 0; i < FLASH_MCHP_DOUBLE_WORD_SIZE; i++) {
				doubleword_buf[i] = src[i];
			}
			/*  Overwrite the relevant bytes */
			for (size_t i = 0; i < len; i++) {
				doubleword_buf[i] = buffer[i];
			}

			ret = flash_doubleword_write(dev, doubleword_buf, aligned_addr);
			if (ret != 0) {
				LOG_ERR("double word write failed at 0x%lx", (long)aligned_addr);
				break;
			}
		}
	} while (0);

	return ret;
}
#endif /*CONFIG_FLASH_HAS_PARTIAL_PAGE_WRITE*/

/**
 * @brief Erases a memory block in the Microchip NVMCTRL.
 *
 * This function issues a command to erase a block of memory at the specified
 * address in the Non-Volatile Memory Controller (NVMCTRL). It prepares the
 * controller to accept a new command, sets the address, and executes the
 * erase block command. The function then checks the status to ensure the
 * operation was successful.
 *
 * @param dev Pointer to the device structure representing the flash controller.
 * @param address The memory address of the block to be erased.
 *
 * @return Returns `FLASH_MCHP_SUCCESS` if the block erase operation is
 *         successful, or `FLASH_MCHP_FAIL` if an error occurs during the
 *         operation.
 *
 */
static int flash_erase_block(const struct device *dev, uint32_t address)
{
	int ret = FLASH_MCHP_SUCCESS;

	/* Set address and command */
	NVM_REGS->NVMCTRL_ADDR = address;
	flash_process_command(dev, NVMCTRL_CTRLB_CMD_EB);

	flash_status_ready_wait(dev);

	ret = flash_get_interrupt_status_error(dev);
	if (ret != 0) {
		ret = FLASH_MCHP_FAIL;
	}
	return ret;
}

/**
 * @brief Writes a page of data to flash memory at the specified address.
 *
 * This function writes a block of 32-bit data to the flash memory page starting
 * at the given address. The data is written in 32-bit words, and the write
 * operation is performed in page mode. The function waits until the flash is
 * ready, sets the appropriate write mode, and then writes the data to the page
 * buffer. After writing, it checks the status of the operation.
 *
 * @param dev     Pointer to the device structure representing the flash controller.
 * @param data    Pointer to the source data buffer to be written (must be 32-bit aligned).
 * @param address Destination address in flash memory where the data will be written.
 *
 * @return FLASH_MCHP_SUCCESS (0) on success, or FLASH_MCHP_FAIL on failure.
 */
static int flash_page_write(const struct device *dev, const void *data, uint32_t address)
{
	int ret = FLASH_MCHP_SUCCESS;
	uint8_t num_words = FLASH_MCHP_PAGE_SIZE / 4U;
	const uint32_t *src = (const uint32_t *)data;
	uint32_t *dst = FLASH_MEMORY(address);

	flash_pagebuffer_clear(dev);

	flash_set_write_mode(dev, NVMCTRL_WMODE_AP);

	/* Writes to the page buffer must be 32 bits */
	for (uint8_t i = 0U; i < num_words; i++) {
		*dst = *src;
		dst++;
		src++;
	}

	flash_status_ready_wait(dev);

	ret = flash_get_interrupt_status_error(dev);
	if (ret != 0) {
		ret = FLASH_MCHP_FAIL;
	}

	return ret;
}

/**
 * @brief Validate the range of a flash memory operation.
 *
 * This function checks whether the specified offset and length are within
 * the valid range of the flash memory. It ensures that the offset is not
 * negative and that the operation does not exceed the total flash size.
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
		if (offset < 0) {
			LOG_WRN("0x%lx: before start of flash", (long)offset);
			ret = -EINVAL;
			break;
		}
		if ((offset + len) > SOC_NV_FLASH_SIZE) {
			LOG_WRN("0x%lx: ends past the end of flash", (long)offset);
			ret = -EINVAL;
			break;
		}
	} while (0);

	return ret;
}

/**
 * @brief Writes data to flash memory, handling page and partial-page writes.
 *
 * This function writes a buffer of data to flash memory at the specified offset.
 * It manages alignment requirements, page boundaries, and optionally supports
 * partial page writes if enabled by configuration. The function ensures that
 * writes are performed safely by locking the flash data structure during the
 * operation and verifies the written data for correctness.
 *
 * The write process includes:
 * - Validating the write range and alignment (if partial page writes are not supported).
 * - Handling the first partial page if the offset is not page-aligned (when supported).
 * - Writing full pages as long as enough data remains.
 * - Handling the last partial page if there are remaining bytes (when supported).
 * - Verifying the written data by comparing it to the source buffer.
 *
 * @param dev    Pointer to the device structure representing the flash controller.
 * @param offset Offset in flash memory where the write should begin (relative to base address).
 * @param data   Pointer to the data buffer to be written.
 * @param len    Number of bytes to write from the data buffer.
 *
 * @return FLASH_MCHP_SUCCESS (0) on success,
 *         -EINVAL if alignment requirements are not met,
 *         -EIO if verification fails,
 *         or FLASH_MCHP_FAIL on other failures.
 */
static int flash_mchp_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
	int ret = FLASH_MCHP_SUCCESS;
	flash_mchp_dev_data_t *mchp_flash_data = dev->data;
	uint32_t write_page_size = DEV_CFG(dev)->flash_param.write_block_size;
	uint32_t remaining_length = len;
	const uint8_t *current_data_ptr = (const uint8_t *)data;

	offset += DEV_CFG(dev)->base_addr;
	uint32_t page_start_address = offset & ~(write_page_size - 1);

	FLASH_MCHP_DATA_LOCK(&mchp_flash_data->flash_data_lock);

	do {
		ret = flash_valid_range(offset, len);
		if (ret != 0) {
			break;
		}
#ifdef CONFIG_FLASH_HAS_PARTIAL_PAGE_WRITE
		uint32_t write_start_offset = offset - page_start_address;
		/* Handle the first partial page if the offset is not zero */
		if (write_start_offset != 0) {
			uint32_t bytes_to_write =
				MIN((write_page_size - write_start_offset), remaining_length);

			ret = flash_partial_write(dev, current_data_ptr, bytes_to_write, offset);
			if (ret != 0) {
				LOG_ERR("partial page write failed at 0x%lx", (long)offset);
				break;
			}

			remaining_length -= bytes_to_write;
			current_data_ptr += bytes_to_write;
			page_start_address += write_page_size;
		}
#else
		ret = flash_aligned(offset, SOC_NV_FLASH_WRITE_BLOCK_SIZE);
		if (ret == 0) {
			LOG_WRN("0x%lx: not on a write block boundary", (long)offset);
			ret = -EINVAL;
			break;
		}

		ret = flash_aligned(len, SOC_NV_FLASH_WRITE_BLOCK_SIZE);
		if (ret == 0) {
			LOG_WRN("%zu: not a integer number of write blocks", len);
			ret = -EINVAL;
			break;
		}
#endif /*CONFIG_FLASH_HAS_PARTIAL_PAGE_WRITE*/

		/* Handle full pages */
		while (remaining_length >= write_page_size) {

			ret = flash_page_write(dev, current_data_ptr, page_start_address);
			if (ret != 0) {
				LOG_ERR("page write failed at 0x%lx", (long)page_start_address);
				break;
			}

			remaining_length -= write_page_size;
			current_data_ptr += write_page_size;
			page_start_address += write_page_size;
		}

#ifdef CONFIG_FLASH_HAS_PARTIAL_PAGE_WRITE
		/* Handle the last partial page if there are remaining bytes */
		if (remaining_length > 0) {

			ret = flash_partial_write(dev, current_data_ptr, remaining_length,
						  page_start_address);
			if (ret != 0) {
				LOG_ERR("partial page write failed at 0x%lx", (long)offset);
				break;
			}
		}
#endif /*CONFIG_FLASH_HAS_PARTIAL_PAGE_WRITE*/

		for (size_t i = 0; i < len; i++) {
			if (((const uint8_t *)data)[i] != ((const uint8_t *)offset)[i]) {
				LOG_ERR("verify error at offset 0x%lx", (long)offset + i);
				ret = -EINVAL;
				break;
			}
		}

	} while (0);

	FLASH_MCHP_DATA_UNLOCK(&mchp_flash_data->flash_data_lock);

	return ret;
}

/**
 * @brief Erases the flash memory block containing the specified address.
 *
 * This function erases the block of flash memory in which the given offset resides.
 * The offset must be aligned to the erase block size, and the size must be a multiple
 * of the erase block size. The function locks the flash data structure during the
 * operation to ensure thread safety, and it checks for valid range and alignment
 * before proceeding. The function waits for the flash to be ready before performing
 * the erase operation.
 *
 * @param dev    Pointer to the device structure representing the flash controller.
 * @param offset Offset in flash memory where the erase should begin (relative to base address).
 * @param size   Number of bytes to erase (must be a multiple of the erase block size).
 *
 * @return FLASH_MCHP_SUCCESS (0) on success,
 *         -EINVAL if alignment requirements are not met,
 *         -EIO if an erase operation fails,
 *         or other error codes as appropriate.
 */
static int flash_mchp_erase(const struct device *dev, off_t offset, size_t size)
{
	int ret = FLASH_MCHP_SUCCESS;
	flash_mchp_dev_data_t *mchp_flash_data = dev->data;
	uint32_t page_size = DEV_CFG(dev)->erase_block_size;

	offset += DEV_CFG(dev)->base_addr;

	do {
		ret = flash_valid_range(offset, size);
		if (ret != 0) {
			break;
		}

		ret = flash_aligned(offset, page_size);
		if (ret == 0) {
			LOG_WRN("0x%lx: not on a erase block boundary", (long)offset);
			ret = -EINVAL;
			break;
		}

		ret = flash_aligned(size, page_size);
		if (ret == 0) {
			LOG_WRN("%zu: not a integer number of erase blocks", size);
			ret = -EINVAL;
			break;
		}

		FLASH_MCHP_DATA_LOCK(&mchp_flash_data->flash_data_lock);

		while (size > 0U) {

			/* Erase the block */
			if (flash_erase_block(dev, offset) != 0) {
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
 * @return Returns NVMCTRL_MCHP_SUCCESS upon successful completion, or a negative error code
 * if the operation fails.
 */
static int flash_mchp_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	int ret = FLASH_MCHP_SUCCESS;
	uint32_t flash_base_addr = DEV_CFG(dev)->base_addr;

	do {
		ret = flash_valid_range(offset, len);
		if (ret != 0) {
			break;
		}

		uint8_t *dst = (uint8_t *)data;
		const uint8_t *src = (const uint8_t *)flash_base_addr + offset;

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
 * @param dev Pointer to the device structure representing the flash controller.
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
 * @param dev Pointer to the device structure representing the flash controller.
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

#ifdef CONFIG_FLASH_EX_OP_ENABLED
/**
 * @brief Determines if a given address is outside the user row range in NVMCTRL.
 *
 * This function checks whether the specified address falls outside the
 * defined user row range in the NVMCTRL memory. If the address is outside
 * the range, it returns a failure code; otherwise, it returns a success code.
 *
 * @param address The address to be checked.
 * @return Returns FLASH_MCHP_SUCCESS if the address is within the user row range,
 *         otherwise returns FLASH_MCHP_FAIL if the address is outside the range.
 */
static int flash_check_offset_user_range(uint32_t address)
{
	int ret = FLASH_MCHP_SUCCESS;

	/* Check if the address is outside the user row range */
	if ((address < SOC_NV_USERROW_BASE_ADDR) ||
	    (address > (SOC_NV_USERROW_BASE_ADDR + SOC_NV_USERROW_SIZE))) {
		ret = FLASH_MCHP_FAIL;
	}

	return ret;
}

/**
 * @brief Erases a user row in the Microchip NVMCTRL.
 *
 * This function issues a command to erase a user row at the specified address
 * in the Non-Volatile Memory Controller (NVMCTRL). It prepares the controller
 * to accept a new command, sets the address, and executes the erase row
 * command. The function then checks the status to ensure the operation was
 * successful.
 *
 * @param[in] dev Pointer to the device structure representing the NVMCTRL hardware instance.
 * @param address The memory address of the user row to be erased.
 *
 * @return Returns `FLASH_MCHP_SUCCESS` if the user row erase operation is
 *         successful, or `FLASH_MCHP_FAIL` if an error occurs during the
 *         operation.
 */
static int flash_user_row_erase(const struct device *dev, uint32_t address)
{
	int ret = FLASH_MCHP_SUCCESS;

	/* Set address and command */
	NVM_REGS->NVMCTRL_ADDR = address;
	flash_process_command(dev, NVMCTRL_CTRLB_CMD_EP);

	flash_status_ready_wait(dev);

	ret = flash_get_interrupt_status_error(dev);
	if (ret != 0) {
		ret = FLASH_MCHP_FAIL;
	}
	return ret;
}

/**
 * @brief Writes data to the user row area of flash memory.
 *
 * This function writes a buffer of data to the user row section of flash memory,
 * starting at the specified offset. The address and data length must be aligned
 * to the user row write block size. The function checks for valid address range
 * and alignment before performing the write operation. Data is written in
 * quad-word (typically 16-byte) blocks.
 *
 * @param dev Pointer to the device structure representing the flash controller.
 * @param in  Pointer to a flash_mchp_ex_op_userrow_data_t structure containing
 *            the offset, data pointer, and data length to be written.
 * @param out Unused output pointer (reserved for future use).
 *
 * @return FLASH_MCHP_SUCCESS (0) on success,
 *         -EINVAL if alignment requirements are not met,
 *         or other error codes as appropriate.
 */
static int flash_ex_op_user_row_write(const struct device *dev, const uintptr_t in, void *out)
{
	ARG_UNUSED(out);
	int ret = FLASH_MCHP_SUCCESS;

	const flash_mchp_ex_op_userrow_data_t *userrow_data =
		(const flash_mchp_ex_op_userrow_data_t *)in;

	const uint8_t *buffer = (const uint8_t *)userrow_data->data;
	uint32_t address = userrow_data->offset + SOC_NV_USERROW_BASE_ADDR;
	size_t len = userrow_data->data_len;

	do {
		ret = flash_aligned(address, SOC_NV_USERROW_WRITE_BLOCK_SIZE);
		if (ret == 0) {
			LOG_WRN("0x%lx: not on a write block boundary", (long)address);
			ret = -EINVAL;
			break;
		}

		ret = flash_aligned(len, SOC_NV_USERROW_WRITE_BLOCK_SIZE);
		if (ret == 0) {
			LOG_WRN("%zu: not a integer number of write blocks", len);
			ret = -EINVAL;
			break;
		}

		ret = flash_check_offset_user_range(address);
		if (ret != 0) {
			break;
		}

		uint32_t num_quad_words = (len / SOC_NV_USERROW_WRITE_BLOCK_SIZE);

		for (uint32_t write_count = 0U; write_count < num_quad_words; write_count++) {
			ret = flash_quadword_write(dev, buffer, address);
			if (ret != 0) {
				break;
			}

			buffer += SOC_NV_USERROW_WRITE_BLOCK_SIZE;
			address += SOC_NV_USERROW_WRITE_BLOCK_SIZE;
		}
	} while (0);

	return ret;
}

/**
 * @brief Erases the user row area of flash memory.
 *
 * This function erases the entire user row section of flash memory, starting at
 * the base address defined by SOC_NV_USERROW_BASE_ADDR. It waits for the flash
 * to be ready before performing the erase operation. The input and output
 * parameters are currently unused.
 *
 * @param dev Pointer to the device structure representing the flash controller.
 * @param in  Unused input parameter (reserved for future use).
 * @param out Unused output parameter (reserved for future use).
 *
 * @return FLASH_MCHP_SUCCESS (0) on success,
 *         -EIO if the erase operation fails.
 */
static int flash_ex_op_user_row_erase(const struct device *dev, const uintptr_t in, void *out)
{
	ARG_UNUSED(in);
	ARG_UNUSED(out);

	int ret = FLASH_MCHP_SUCCESS;

	/* Erase the user page */
	if (flash_user_row_erase(dev, SOC_NV_USERROW_BASE_ADDR) != 0) {
		LOG_ERR("User page erase failed");
		ret = -EIO;
	}

	return ret;
}

/**
 * @brief Lock all regions of the SoC non-volatile flash.
 *
 * This function iterates over all lock regions of the SoC non-volatile flash and issues
 * a lock command for each region. It waits for the flash to be ready before issuing each command.
 *
 * @param dev Pointer to the flash device structure.
 * @param in  Unused input parameter (reserved for future use or interface compatibility).
 * @param out Unused output parameter (reserved for future use or interface compatibility).
 *
 * @retval FLASH_MCHP_SUCCESS if all regions are successfully locked.
 * @retval FLASH_MCHP_FAIL if locking any region fails.
 */
static int flash_ex_op_region_lock(const struct device *dev, const uintptr_t in, void *out)
{
	ARG_UNUSED(in);
	ARG_UNUSED(out);
	int ret = FLASH_MCHP_SUCCESS;

	for (off_t offset = 0; offset < SOC_NV_FLASH_SIZE;
	     offset += SOC_NV_FLASH_LOCK_REGION_SIZE) {

		/* Set address and command */
		NVM_REGS->NVMCTRL_ADDR = offset + SOC_NV_FLASH_BASE_ADDRESS;
		flash_process_command(dev, NVMCTRL_CTRLB_CMD_LR);

		ret = flash_get_interrupt_status_error(dev);
		if (ret != 0) {
			ret = FLASH_MCHP_FAIL;
			break;
		}
	}

	return ret;
}

/**
 * @brief Unlock all regions of the SoC non-volatile flash.
 *
 * This function iterates over all lock regions of the SoC non-volatile flash and issues
 * an unlock command for each region. It waits for the flash to be ready before issuing each
 * command.
 *
 * @param dev Pointer to the flash device structure.
 * @param in  Unused input parameter (reserved for future use or interface compatibility).
 * @param out Unused output parameter (reserved for future use or interface compatibility).
 *
 * @retval FLASH_MCHP_SUCCESS if all regions are successfully unlocked.
 * @retval FLASH_MCHP_FAIL if unlocking any region fails.
 */
static int flash_ex_op_region_unlock(const struct device *dev, const uintptr_t in, void *out)
{
	ARG_UNUSED(in);
	ARG_UNUSED(out);
	int ret = FLASH_MCHP_SUCCESS;

	for (off_t offset = 0; offset < SOC_NV_FLASH_SIZE;
	     offset += SOC_NV_FLASH_LOCK_REGION_SIZE) {

		/* Set address and command */
		NVM_REGS->NVMCTRL_ADDR = offset + SOC_NV_FLASH_BASE_ADDRESS;
		flash_process_command(dev, NVMCTRL_CTRLB_CMD_UR);

		ret = flash_get_interrupt_status_error(dev);
		if (ret != 0) {
			ret = FLASH_MCHP_FAIL;
			break;
		}
	}

	return ret;
}

/**
 * @brief Executes an extended flash operation based on the provided operation code.
 *
 * This function acts as a dispatcher for various extended flash operations, such as
 * erasing or writing the user row, and locking or unlocking flash regions. The specific
 * operation to perform is determined by the @p code parameter.
 *
 * @param dev  Pointer to the device structure representing the flash controller.
 * @param code Operation code specifying which extended operation to perform.
 * @param in   Pointer to input data required by the operation (usage depends on operation).
 * @param out  Pointer to output data buffer (usage depends on operation).
 *
 * @return FLASH_MCHP_SUCCESS (0) on success,
 *         -EINVAL if the operation code is invalid,
 *         or other error codes as returned by the respective functions.
 */
static int flash_mchp_ex_op(const struct device *dev, uint16_t code, const uintptr_t in, void *out)
{
	int ret = FLASH_MCHP_SUCCESS;

	switch (code) {
	case FLASH_EX_OP_USER_ROW_ERASE:
		ret = flash_ex_op_user_row_erase(dev, in, out);
		break;
	case FLASH_EX_OP_USER_ROW_WRITE:
		ret = flash_ex_op_user_row_write(dev, in, out);
		break;
	case FLASH_EX_OP_REGION_LOCK:
		ret = flash_ex_op_region_lock(dev, in, out);
		break;
	case FLASH_EX_OP_REGION_UNLOCK:
		ret = flash_ex_op_region_unlock(dev, in, out);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}
#endif /*CONFIG_FLASH_EX_OP_ENABLED*/

/**
 * @brief Interrupt Service Routine for the Microchip NVMCTRL peripheral.
 *
 * This function handles interrupts from the Microchip NVMCTRL peripheral.
 * It clears the interrupt flag to acknowledge the interrupt and releases
 * a semaphore to allow other operations to proceed.
 *
 * @param dev Pointer to the device structure for the flash controller.
 */
static void flash_mchp_isr(const struct device *dev)
{
	flash_clear_interrupt_flag(dev);
}

/**
 * @brief Initializes the Microchip NVMCTRL peripheral.
 *
 * This function sets up the necessary resources and configurations for the
 * Microchip flash memory controller to operate. It initializes mutexes and
 * semaphores, enables the clock for the controller, configures interrupts,
 * and performs any necessary hardware initialization.
 *
 * @param dev Pointer to the device structure for the flash controller.
 *
 * @return Returns 0 upon successful initialization.
 *
 */
static int flash_mchp_init(const struct device *dev)
{
	const flash_mchp_dev_config_t *const mchp_flash_cfg = DEV_CFG(dev);
	flash_mchp_dev_data_t *mchp_flash_data = dev->data;

	FLASH_MCHP_DATA_MUTEX_INIT(&(mchp_flash_data->flash_data_lock));

	clock_control_on(mchp_flash_cfg->flash_clock.clock_dev,
			 (clock_control_subsys_t)&mchp_flash_cfg->flash_clock.mclk_sys);

	mchp_flash_cfg->irq_config_func(dev);
	flash_controller_init(dev);

	return 0;
}

/**
 * @brief NVMCTRL driver API structure.
 */
static const struct flash_driver_api flash_mchp_driver_api = {
	.write = flash_mchp_write,
	.read = flash_mchp_read,
	.erase = flash_mchp_erase,
	.get_parameters = flash_mchp_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_mchp_page_layout,
#endif /*CONFIG_FLASH_PAGE_LAYOUT*/
#ifdef CONFIG_FLASH_EX_OP_ENABLED
	.ex_op = flash_mchp_ex_op,
#endif /*CONFIG_FLASH_EX_OP_ENABLED*/
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
		.regs = (nvmctrl_registers_t *)DT_INST_REG_ADDR(n),                                \
		.base_addr = SOC_NV_FLASH_BASE_ADDRESS,                                            \
		.erase_block_size = SOC_NV_FLASH_ERASE_BLOCK_SIZE,                                 \
		.flash_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                       \
		.flash_clock.mclk_sys = {.dev = DEVICE_DT_GET(                                     \
						 DT_INST_CLOCKS_CTLR_BY_NAME(n, mclk)),            \
					 .id = DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, id)},          \
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
