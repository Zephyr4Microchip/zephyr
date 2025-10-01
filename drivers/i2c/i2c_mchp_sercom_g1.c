/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file i2c_mchp_sercom_g1.c
 * @brief I2C driver for Microchip SERCOM_G1 peripheral.
 *
 * This driver provides an implementation of the Zephyr I2C API for the
 * Microchip SERCOM_G1 peripheral.
 *
 */

#include <soc.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/dma.h>
#include <mchp_dt_helper.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>

/*******************************************
 * @brief Devicetree definitions
 ********************************************/
#define DT_DRV_COMPAT microchip_sercom_g1_i2c

/*******************************************
 * Const and Macro Defines
 *******************************************/
/**
 * @brief Register I2C MCHP G1 driver with logging subsystem.
 */
LOG_MODULE_REGISTER(i2c_mchp_sercom_g1, CONFIG_I2C_LOG_LEVEL);

/**
 * @brief Including header here to avoid compilation error
 *
 * As i2c-priv.h header file usage Log messages, LOG_LEVEL has to be defined before.
 */
#include "i2c-priv.h"

/* Macro to check message direction in read mode */
#define I2C_MCHP_MESSAGE_DIR_READ_MASK 1

/** I2C Fast Speed: 400 kHz */
#define I2C_MCHP_SPEED_FAST 400000

/** I2C Fast plus Speed: 1 MHz */
#define I2C_MCHP_SPEED_FAST_PLUS 1000000

/* I2C high Speed: 3.4 MHz */
#define I2C_MCHP_SPEED_HIGH_SPEED 3400000

/* Combined max of BAUD_LOW and BAUD.*/
#define I2C_BAUD_LOW_HIGH_MAX 382U

/* i2c start condion setup time 100ns */
#define I2C_MCHP_START_CONDITION_SETUP_TIME (100.0f / 1000000000.0f)

/* Invalid I2C address indicator. */
#define I2C_INVALID_ADDR 0x00

/* I2C message direction: read operation. */
#define I2C_MESSAGE_DIR_READ 1U

#ifdef CONFIG_I2C_MCHP_TRANSFER_TIMEOUT
/* I2C transfer timeout in milliseconds, configurable via Kconfig. */
#define I2C_TRANSFER_TIMEOUT_MSEC K_MSEC(CONFIG_I2C_MCHP_TRANSFER_TIMEOUT)
#else
/* I2C transfer timeout set to infinite if not configured. */
#define I2C_TRANSFER_TIMEOUT_MSEC K_FOREVER
#endif

/* I2C_REGS Register */
#define I2C_REGS ((const i2c_mchp_dev_config_t *)(dev)->config)->regs

/**
 * @brief Type definition for the i2c lock.
 *
 * This macro defines the type of the lock used to protect access to the i2c APIs.
 */
#define I2C_MCHP_LOCK_TYPE struct k_mutex

/**
 * @brief Timeout duration for acquiring the i2c lock.
 *
 * This macro defines the timeout duration for acquiring the i2c lock.
 * The timeout is specified in milliseconds.
 */
#define I2C_MCHP_LOCK_TIMEOUT K_MSEC(10)

/**
 * @brief Initialize the i2c lock.
 *
 * This macro initializes the i2c lock.
 *
 * @param p_lock Pointer to the lock to be initialized.
 */
#define I2C_MCHP_DATA_MUTEX_INIT(p_lock) k_mutex_init(p_lock)

/**
 * @brief Acquire the i2c lock.
 *
 * This macro acquires the i2c lock. If the lock is not available, the
 * function will wait for the specified timeout duration.
 *
 * @param p_lock Pointer to the lock to be acquired.
 * @return 0 if the lock was successfully acquired, or a negative error code.
 */
#define I2C_MCHP_DATA_LOCK(p_lock) k_mutex_lock(p_lock, I2C_MCHP_LOCK_TIMEOUT)

/**
 * @brief Release the i2c lock.
 *
 * This macro releases the i2c lock.
 *
 * @param p_lock Pointer to the lock to be released.
 * @return 0 if the lock was successfully released, or a negative error code.
 */
#define I2C_MCHP_DATA_UNLOCK(p_lock) k_mutex_unlock(p_lock)

/**
 * @brief Type definition for the i2c command semaphore lock.
 *
 * This macro defines the type of the semaphore used to synchronize nvmctrl operations.
 */
#define I2C_MCHP_CMD_LOCK_TYPE struct k_sem

/**
 * @brief Timeout duration for acquiring the i2c semaphore lock.
 *
 * This macro defines the timeout duration for acquiring the i2c semaphore lock.
 * The timeout is specified in milliseconds.
 */
#define I2C_MCHP_CMD_SEM_TIMEOUT K_MSEC(10)

/** @brief Initial count for i2c command semaphore. */
#define I2C_MCHP_CMD_SEM_INIT_COUNT 0

/** @brief Maximum limit for i2c command semaphore. */
#define I2C_MCHP_CMD_SEM_LIMIT 1

/**
 * @brief Initialize a semaphore for i2c operations.
 *
 * This macro initializes a semaphore with a specified initial count and limit.
 *
 * @param p_sem Pointer to the semaphore to be initialized.
 * @param init_count Initial count of the semaphore.
 * @param limit Maximum count of the semaphore.
 */
#define I2C_MCHP_CMD_SEM_INIT(p_sem)                                                               \
	k_sem_init(p_sem, I2C_MCHP_CMD_SEM_INIT_COUNT, I2C_MCHP_CMD_SEM_LIMIT)

/**
 * @brief Acquire the i2c semaphore.
 *
 * This macro attempts to take the i2c semaphore, blocking for a specified timeout period.
 *
 * @param p_sem Pointer to the semaphore to be acquired.
 * @param FLASH_MCHP_CMD_SEM_TIMEOUT Timeout value for the semaphore operation.
 */
#define I2C_MCHP_CMD_SEM_TAKE(p_sem) k_sem_take(p_sem, I2C_MCHP_CMD_SEM_TIMEOUT)

/**
 * @brief Release the semaphore for i2c operations.
 *
 * This macro releases a semaphore that is used to synchronize i2c operations.
 *
 * @param p_sem Pointer to the semaphore to be released.
 */
#define I2C_MCHP_CMD_SEM_GIVE(p_sem) k_sem_give(p_sem)

/**
 * @brief Get the current frequency of the I2C peripheral clock.
 *
 * This macro retrieves the clock frequency for the I2C peripheral by calling
 * the clock control API with the appropriate clock device and clock ID from
 * the device configuration structure.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param rate Variable to store the retrieved clock frequency (in Hz).
 */
#define I2C_MCHP_GET_CLOCK_FREQ(dev, rate)                                                         \
	clock_control_get_rate(                                                                    \
		((const i2c_mchp_dev_config_t *)(dev->config))->i2c_clock.clock_dev,               \
		(((i2c_mchp_dev_config_t *)(dev->config))->i2c_clock.gclk_sys), &rate)

/**
 * @brief Enable the I2C peripheral clocks.
 *
 * This macro enables both the generic clock and main clock for the I2C peripheral
 * by calling the clock control API with the appropriate clock device and clock IDs
 * from the device configuration structure.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 */
#if defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ2)
#define I2C_MCHP_ENABLE_CLOCK(dev)                                                                 \
	clock_control_on(((const i2c_mchp_dev_config_t *)(dev->config))->i2c_clock.clock_dev,      \
			 (((i2c_mchp_dev_config_t *)(dev->config))->i2c_clock.gclk_sys));          \
	if (((const i2c_mchp_dev_config_t *)(dev->config))->regs == SERCOM1_REGS) {                \
		CFG_REGS->CFG_PMD3 &= ~CFG_PMD3_SER2MD_Msk;                                        \
	} else if (((const i2c_mchp_dev_config_t *)(dev->config))->regs == SERCOM2_REGS) {         \
		CFG_REGS->CFG_PMD3 &= ~CFG_PMD3_SER3MD_Msk;                                        \
	}
#elif defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ6)
#define I2C_MCHP_ENABLE_CLOCK(dev)                                                                 \
	clock_control_on(((const i2c_mchp_dev_config_t *)(dev->config))->i2c_clock.clock_dev,      \
			 (((i2c_mchp_dev_config_t *)(dev->config))->i2c_clock.gclk_sys));          \
	if (((const i2c_mchp_dev_config_t *)(dev->config))->regs == SERCOM1_REGS) {                \
		CFG_REGS->CFG_PMD3 &= ~CFG_PMD3_SER1MD_Msk;                                        \
	}
#else
#define I2C_MCHP_ENABLE_CLOCK(dev)                                                                 \
	clock_control_on(((const i2c_mchp_dev_config_t *)(dev->config))->i2c_clock.clock_dev,      \
			 (((i2c_mchp_dev_config_t *)(dev->config))->i2c_clock.gclk_sys));          \
	clock_control_on(((const i2c_mchp_dev_config_t *)(dev->config))->i2c_clock.clock_dev,      \
			 (((i2c_mchp_dev_config_t *)(dev->config))->i2c_clock.mclk_sys))
#endif

/*******************************************
 * Enum and typedefs
 *******************************************/

/* Mask representing all controller interrupt flags. */
#define I2C_MCHP_CONTROLLER_INTERRUPT_ALL_MASK                                                     \
	(I2C_MCHP_CONTROLLER_INTERRUPT_CONTROLLER_ON_BUS |                                         \
	 I2C_MCHP_CONTROLLER_INTERRUPT_TARGET_ON_BUS | I2C_MCHP_CONTROLLER_INTERRUPT_ERROR)

/**
 * @enum i2c_mchp_controller_interrupt_t
 * @brief Enum to set & clear interrupt bits in controller mode.
 *
 * This enumeration defines the interrupt flags for the MCHP I2C controller mode.
 */
typedef enum {
	/* Indicates that the controller is currently on the bus. */
	I2C_MCHP_CONTROLLER_INTERRUPT_CONTROLLER_ON_BUS = 1,

	/* Indicates that the target is currently on the bus. */
	I2C_MCHP_CONTROLLER_INTERRUPT_TARGET_ON_BUS = 2,

	/* Indicates an error condition in the I2C operation. */
	I2C_MCHP_CONTROLLER_INTERRUPT_ERROR = 4,

	/* Indicates all controller interrupt flags. */
	I2C_MCHP_CONTROLLER_INTERRUPT_ALL = I2C_MCHP_CONTROLLER_INTERRUPT_ALL_MASK
} i2c_mchp_controller_interrupt_t;

/* Mask representing all target interrupt flags. */
#define I2C_MCHP_TARGET_INTERRUPT_ALL_MASK                                                         \
	(I2C_MCHP_TARGET_INTERRUPT_STOP | I2C_MCHP_TARGET_INTERRUPT_ADDR_MATCH |                   \
	 I2C_MCHP_TARGET_INTERRUPT_DATA_READY | I2C_MCHP_TARGET_INTERRUPT_ERROR)

/**
 * @enum i2c_mchp_target_interrupt_t
 * @brief Enum to set & clear interrupt bits in target mode.
 *
 * This enumeration defines the interrupt flags for the MCHP I2C target mode.
 */
typedef enum {
	/* Indicates a STOP condition was detected. */
	I2C_MCHP_TARGET_INTERRUPT_STOP = 1,

	/* Indicates an address match event occurred. */
	I2C_MCHP_TARGET_INTERRUPT_ADDR_MATCH = 2,

	/* Indicates data is ready for processing. */
	I2C_MCHP_TARGET_INTERRUPT_DATA_READY = 4,

	/* Indicates an error condition in the I2C operation. */
	I2C_MCHP_TARGET_INTERRUPT_ERROR = 8,

	/* Indicates all target interrupt flags. */
	I2C_MCHP_TARGET_INTERRUPT_ALL = I2C_MCHP_TARGET_INTERRUPT_ALL_MASK

} i2c_mchp_target_interrupt_t;

/* Mask representing all controller interrupt flag bits. */
#define I2C_MCHP_CONTROLLER_INTFLAG_ALL_MASK                                                       \
	(I2C_MCHP_CONTROLLER_INTFLAG_CONTROLLER_ON_BUS |                                           \
	 I2C_MCHP_CONTROLLER_INTFLAG_TARGET_ON_BUS | I2C_MCHP_CONTROLLER_INTFLAG_ERROR)

/**
 * @enum i2c_mchp_controller_intflag_t
 * @brief Enum to define & return interrupt flag register bits in controller mode.
 *
 * This enumeration defines the interrupt flag bits for the MCHP I2C controller mode.
 */
typedef enum {
	/* Indicates that no interrupt has occurred. */
	I2C_MCHP_CONTROLLER_INTFLAG_NONE = 0,

	/* Indicates that the controller is currently on the bus. */
	I2C_MCHP_CONTROLLER_INTFLAG_CONTROLLER_ON_BUS = 1,

	/* Indicates that the target is currently on the bus. */
	I2C_MCHP_CONTROLLER_INTFLAG_TARGET_ON_BUS = 2,

	/* Indicates an error condition in the I2C operation. */
	I2C_MCHP_CONTROLLER_INTFLAG_ERROR = 4,

	/* Indicates all the interrupt flags. */
	I2C_MCHP_CONTROLLER_INTFLAG_ALL = I2C_MCHP_CONTROLLER_INTFLAG_ALL_MASK

} i2c_mchp_controller_intflag_t;

/** @brief Mask representing all target interrupt flag bits. */
#define I2C_MCHP_TARGET_INTFLAG_ALL_MASK                                                           \
	(I2C_MCHP_TARGET_INTFLAG_STOP | I2C_MCHP_TARGET_INTFLAG_ADDR_MATCH |                       \
	 I2C_MCHP_TARGET_INTFLAG_DATA_READY | I2C_MCHP_TARGET_INTFLAG_ERROR)

/**
 * @enum i2c_mchp_target_intflag_t
 * @brief Enum to define & return interrupt flag register bits in target mode.
 *
 * This enumeration defines the interrupt flag bits for the MCHP I2C target mode.
 */
typedef enum {

	/* Indicates that no interrupt has occurred. */
	I2C_MCHP_TARGET_INTFLAG_NONE = 0,

	/* Indicates that a STOP condition has been detected. */
	I2C_MCHP_TARGET_INTFLAG_STOP = 1,

	/* Indicates that an address match has occurred. */
	I2C_MCHP_TARGET_INTFLAG_ADDR_MATCH = 2,

	/* Indicates that data is ready for transmission or reception. */
	I2C_MCHP_TARGET_INTFLAG_DATA_READY = 4,

	/*  Indicates an error condition in the I2C operation. */
	I2C_MCHP_TARGET_INTFLAG_ERROR = 8,

	/*  Indicates all the interrupt flags. */
	I2C_MCHP_TARGET_INTFLAG_ALL = I2C_MCHP_TARGET_INTFLAG_ALL_MASK

} i2c_mchp_target_intflag_t;

/** @brief Mask representing all controller status flag bits. */
#define I2C_MCHP_CONTROLLER_STATUS_FLAG_ALL_MASK                                                   \
	(I2C_MCHP_CONTROLLER_STATUS_FLAG_BUS_ERROR |                                               \
	 I2C_MCHP_CONTROLLER_STATUS_FLAG_ARBITRATION_LOST |                                        \
	 I2C_MCHP_CONTROLLER_STATUS_FLAG_BUS_STATE_BUSY |                                          \
	 I2C_MCHP_CONTROLLER_STATUS_FLAG_MEXTTOUT | I2C_MCHP_CONTROLLER_STATUS_FLAG_SEXTTOUT |     \
	 I2C_MCHP_CONTROLLER_STATUS_FLAG_LOWTOUT | I2C_MCHP_CONTROLLER_STATUS_FLAG_LENERR)

/**
 * @enum i2c_mchp_controller_status_flag_t
 * @brief Enum representing the status register bits in controller mode.
 *
 * This enumeration defines the status flag bits for the MCHP I2C controller mode.
 */
typedef enum {

	/* Indicates that there is no status. */
	I2C_MCHP_CONTROLLER_STATUS_FLAG_NONE = 0,

	/* Indicates that errors on bus have occurred. */
	I2C_MCHP_CONTROLLER_STATUS_FLAG_BUS_ERROR = 1,

	/* Indicates that an arbitration lost condition has been detected. */
	I2C_MCHP_CONTROLLER_STATUS_FLAG_ARBITRATION_LOST = 2,

	/* Indicates that a bus busy condition was detected. */
	I2C_MCHP_CONTROLLER_STATUS_FLAG_BUS_STATE_BUSY = 4,

	/* Indicates that a Master SCL Low Extend Time-Out was detected. */
	I2C_MCHP_CONTROLLER_STATUS_FLAG_MEXTTOUT = 8,

	/* Indicates that a Slave SCL Low Extend Time-Out was detected. */
	I2C_MCHP_CONTROLLER_STATUS_FLAG_SEXTTOUT = 16,

	/* Indicates that a SCL Low Time-Out was detected. */
	I2C_MCHP_CONTROLLER_STATUS_FLAG_LOWTOUT = 32,

	/* Indicates that a Transaction Length Error was detected. */
	I2C_MCHP_CONTROLLER_STATUS_FLAG_LENERR = 64,

	/* Indicates all controller status flag bits. */
	I2C_MCHP_CONTROLLER_STATUS_FLAG_ALL = I2C_MCHP_CONTROLLER_STATUS_FLAG_ALL_MASK

} i2c_mchp_controller_status_flag_t;

/** @brief Mask representing all target status flag bits. */
#define I2C_MCHP_TARGET_STATUS_FLAG_ALL_MASK                                                       \
	(I2C_MCHP_TARGET_STATUS_FLAG_BUS_ERROR | I2C_MCHP_TARGET_STATUS_FLAG_COLL |                \
	 I2C_MCHP_TARGET_STATUS_FLAG_DATA_DIR_READ | I2C_MCHP_TARGET_STATUS_FLAG_LOWTOUT |         \
	 I2C_MCHP_TARGET_STATUS_FLAG_SEXTTOUT)

/**
 * @enum i2c_mchp_target_status_flag_t
 * @brief Enum representing the status register bits in target mode.
 *
 * This enumeration defines the status flag bits for the MCHP I2C target mode.
 */
typedef enum {

	/* Indicates that there is no status. */
	I2C_MCHP_TARGET_STATUS_FLAG_NONE = 0,

	/* Indicates that errors on bus have occurred. */
	I2C_MCHP_TARGET_STATUS_FLAG_BUS_ERROR = 1,

	/* Indicates that a Transmit Collision has been detected. */
	I2C_MCHP_TARGET_STATUS_FLAG_COLL = 2,

	/* Indicates that data direction is in read mode. */
	I2C_MCHP_TARGET_STATUS_FLAG_DATA_DIR_READ = 4,

	/* Indicates that a SCL Low Time-Out was detected. */
	I2C_MCHP_TARGET_STATUS_FLAG_LOWTOUT = 16,

	/* Indicates that a Slave SCL Low Extend Time-Out was detected. */
	I2C_MCHP_TARGET_STATUS_FLAG_SEXTTOUT = 32,

	/* Indicates all target status flag bits. */
	I2C_MCHP_TARGET_STATUS_FLAG_ALL = I2C_MCHP_TARGET_STATUS_FLAG_ALL_MASK

} i2c_mchp_target_status_flag_t;

/**
 * @enum i2c_mchp_target_ack_status_t
 * @brief Enumeration for I2C target acknowledge status.
 *
 * This enum represents the possible acknowledge status received by the I2C slave
 * after a byte transfer.
 */
typedef enum {
	I2C_MCHP_TARGET_ACK_STATUS_RECEIVED_ACK = 0,
	I2C_MCHP_TARGET_ACK_STATUS_RECEIVED_NACK
} i2c_mchp_target_ack_status_t;

/**
 * @enum i2c_mchp_target_cmd_t
 * @brief I2C target command options for SERCOM I2C peripheral.
 *
 * Defines commands for ACK/NACK and transaction control in I2C target mode.
 */
typedef enum {
	I2C_MCHP_TARGET_COMMAND_SEND_ACK = 0,
	I2C_MCHP_TARGET_COMMAND_SEND_NACK,
	I2C_MCHP_TARGET_COMMAND_RECEIVE_ACK_NAK,
	I2C_MCHP_TARGET_COMMAND_WAIT_FOR_START
} i2c_mchp_target_cmd_t;

/**
 * @struct i2c_mchp_clock
 * @brief Structure to hold device clock configuration.
 */
#if defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ2) ||                                            \
	defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ6)
typedef struct i2c_mchp_clock {

	/* Clock driver */
	const struct device *clock_dev;

	/* Generic clock subsystem. */
	clock_control_subsys_t gclk_sys;
} i2c_mchp_clock_t;
#else
typedef struct i2c_mchp_clock {

	/* Clock driver */
	const struct device *clock_dev;

	/* Main clock subsystem. */
	clock_control_subsys_t mclk_sys;

	/* Generic clock subsystem. */
	clock_control_subsys_t gclk_sys;

} i2c_mchp_clock_t;
#endif
/**
 * @struct i2c_mchp_dma
 * @brief DMA configuration parameters for I2C operations.
 *
 * This structure holds the configuration required to enable DMA-based
 * asynchronous transfers for the I2C peripheral, including device pointers,
 * request lines, and channel numbers for both TX and RX directions.
 */
typedef struct i2c_mchp_dma {

	/* DMA device used for asynchronous operations. */
	const struct device *dma_dev;

	/* DMA request line for TX (transmit) operations. */
	uint8_t tx_dma_request;

	/* DMA channel number for TX (transmit) operations. */
	uint8_t tx_dma_channel;

	/* DMA request line for RX (receive) operations. */
	uint8_t rx_dma_request;

	/* DMA channel number for RX (receive) operations. */
	uint8_t rx_dma_channel;

} i2c_mchp_dma_t;

/**
 * @struct i2c_mchp_dev_config
 * @brief Configuration structure for the MCHP I2C driver.
 *
 * This structure contains all necessary configuration parameters for
 * initializing and operating the MCHP I2C driver, including hardware
 * abstraction, clock and pin settings, bitrate, IRQ configuration, and
 * optional DMA support.
 */
typedef struct i2c_mchp_dev_config {

	/* Hardware Abstraction Layer for the I2C peripheral. */
	sercom_registers_t *regs;

	/* Clock configuration for the I2C peripheral. */
	i2c_mchp_clock_t i2c_clock;

	/* Pin configuration for SDA and SCL lines. */
	const struct pinctrl_dev_config *pcfg;

	/* Default bitrate for I2C communication (e.g., 100 kHz, 400 kHz). */
	uint32_t bitrate;

	/* Function pointer to configure IRQ during initialization. */
	void (*irq_config_func)(const struct device *dev);

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
	/* DMA configuration for I2C transfers (enabled if DMA is used). */
	i2c_mchp_dma_t i2c_dma;
#endif

	/* Enable peripheral operation in standby sleep mode. */
	uint8_t run_in_standby;

} i2c_mchp_dev_config_t;

/**
 * @struct i2c_mchp_msg
 * @brief Structure representing an I2C message for the MCHP I2C peripheral.
 *
 * This structure encapsulates the data buffer, size, and status information
 * for an I2C message transaction using the MCHP I2C peripheral.
 */
typedef struct i2c_mchp_msg {

	/* Pointer to the data buffer for the I2C message. */
	uint8_t *buffer;

	/* Size of the I2C message in bytes. */
	uint32_t size;

	/* Status of the I2C message, indicating success or error conditions. */
	i2c_mchp_controller_status_flag_t status;

} i2c_mchp_msg_t;

/**
 * @struct i2c_mchp_dev_data
 * @brief Structure representing runtime data for the MCHP I2C driver.
 *
 * This structure contains all runtime data required for managing the
 * operation of the MCHP I2C driver, including synchronization primitives,
 * message tracking, configuration, and optional callback and target mode
 * support.
 */
typedef struct i2c_mchp_dev_data {

	/**< Pointer to the I2C device instance. */
	const struct device *dev;

	/* Mutex for protecting access to the I2C driver data. */
	I2C_MCHP_LOCK_TYPE i2c_bus_mutex;

	/* Semaphore for signaling completion of I2C transfers. */
	I2C_MCHP_CMD_LOCK_TYPE i2c_sync_sem;

	/* Structure representing the current I2C message. */
	i2c_mchp_msg_t current_msg;

	/* Pointer to an array of I2C messages being transferred. */
	struct i2c_msg *msgs_array;

	/* Number of messages in the current I2C transfer sequence. */
	uint8_t num_msgs;

	/* Flag indicating whether the device is in target mode. */
	bool target_mode;

	/* Current I2C device configuration settings. */
	uint32_t dev_config;

	/* Address of the I2C target device. */
	uint32_t target_addr;

	/* Variable to track message buffer by indexing. */
	uint8_t msg_index;

#ifdef CONFIG_I2C_CALLBACK
	/* Callback function for asynchronous I2C operations. */
	i2c_callback_t i2c_async_callback;

	/* User data passed to the callback function. */
	void *user_data;
#endif

#ifdef CONFIG_I2C_TARGET
	/* Target configuration for I2C target mode. */
	struct i2c_target_config target_config;

	/* Target callbacks for I2C target mode. */
	struct i2c_target_callbacks target_callbacks;

	/* Data buffer for RX/TX operations in target mode. */
	uint8_t rx_tx_data;
#endif

	/* First byte read after an address match. */
	bool firstReadAfterAddrMatch;

} i2c_mchp_dev_data_t;

/*******************************************
 * Helper functions
 *******************************************/
#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
static int i2c_dma_write_config(const struct device *dev);
static int i2c_dma_read_config(const struct device *dev);
#endif

/**
 * @brief Get the source address for I2C DMA transfers.
 *
 * Returns a pointer to the I2C data register, which serves as the source address
 * for DMA transfers in I2C controller mode. This address is typically used to
 * configure the DMA controller for data transmission.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @return Pointer to the I2C data register to be used as the DMA source address.
 */
static inline void *i2c_get_dma_source_addr(const struct device *dev)
{
	return ((void *)&(I2C_REGS->I2CM.SERCOM_DATA));
}

/**
 * @brief Get the destination address for I2C DMA transfers.
 *
 * Returns a pointer to the I2C data register, which serves as the destination address
 * for DMA transfers in I2C controller mode. This address is typically used to
 * configure the DMA controller for data reception.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @return Pointer to the I2C data register to be used as the DMA destination address.
 */
static inline void *i2c_get_dma_dest_addr(const struct device *dev)
{
	return ((void *)&(I2C_REGS->I2CM.SERCOM_DATA));
}

/**
 * @brief Perform a software reset on the I2C peripheral.
 *
 * This function triggers a software reset of the I2C peripheral by setting the
 * appropriate bit in the control register. It then waits for the reset operation
 * to complete by polling the synchronization busy flag.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 */
static inline void i2c_swrst(const struct device *dev)
{
	sercom_registers_t *i2c_regs = ((const i2c_mchp_dev_config_t *)(dev)->config)->regs;

	i2c_regs->I2CM.SERCOM_CTRLA |= SERCOM_I2CM_CTRLA_SWRST(1);

	/* Wait for synchronization */
	while ((i2c_regs->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_SWRST_Msk) ==
	       SERCOM_I2CM_SYNCBUSY_SWRST_Msk) {

		/* Do nothing */
	};
}

/**
 * @brief Read a byte from the I2C data register in controller or target mode.
 *
 * This API reads a single byte from the I2C data register, automatically selecting
 * the appropriate register based on whether the peripheral is operating in controller
 * (master) or target (slave) mode.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @return The byte read from the I2C data register.
 */
static inline uint8_t i2c_byte_read(const struct device *dev)
{
	sercom_registers_t *i2c_regs = ((const i2c_mchp_dev_config_t *)(dev)->config)->regs;

	if ((i2c_regs->I2CM.SERCOM_CTRLA & SERCOM_I2CM_CTRLA_MODE_I2C_MASTER) ==
	    SERCOM_I2CM_CTRLA_MODE_I2C_MASTER) {
		return (uint8_t)i2c_regs->I2CM.SERCOM_DATA;
	} else {
		return (uint8_t)i2c_regs->I2CS.SERCOM_DATA;
	}
}

/**
 * @brief Write a byte to the I2C data register in controller or target mode.
 *
 * This API writes a single byte to the I2C data register, automatically selecting
 * the appropriate register based on whether the peripheral is operating in controller
 * (master) or target (slave) mode. In controller mode, the function waits for
 * synchronization to complete after writing the data.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param data The byte to write to the I2C data register.
 */
static void i2c_byte_write(const struct device *dev, uint8_t data)
{
	sercom_registers_t *i2c_regs = ((const i2c_mchp_dev_config_t *)(dev)->config)->regs;

	if ((i2c_regs->I2CM.SERCOM_CTRLA & SERCOM_I2CM_CTRLA_MODE_I2C_MASTER) ==
	    SERCOM_I2CM_CTRLA_MODE_I2C_MASTER) {
		i2c_regs->I2CM.SERCOM_DATA = data;

		/* Wait for synchronization */
		while ((i2c_regs->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_SYSOP_Msk) ==
		       SERCOM_I2CM_SYNCBUSY_SYSOP_Msk) {

			/* Do nothing */
		};
	} else {
		i2c_regs->I2CS.SERCOM_DATA = data;
	}
}

/**
 * @brief Enable or disable the I2C peripheral in controller (master) mode.
 *
 * This API enables or disables the I2C peripheral by setting or clearing the
 * enable bit in the control register. It waits for the synchronization process
 * to complete after the operation.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param enable Set to true to enable the controller, false to disable.
 */
static void i2c_controller_enable(const struct device *dev, bool enable)
{
	sercom_registers_t *i2c_regs = ((const i2c_mchp_dev_config_t *)(dev)->config)->regs;

	if (enable == true) {
		i2c_regs->I2CM.SERCOM_CTRLA |= SERCOM_I2CM_CTRLA_ENABLE(1);
	} else {
		i2c_regs->I2CM.SERCOM_CTRLA &= ~SERCOM_I2CM_CTRLA_ENABLE(1);
	}

	/* Wait for synchronization */
	while ((i2c_regs->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_ENABLE_Msk) ==
	       SERCOM_I2CM_SYNCBUSY_ENABLE_Msk) {

		/* Do nothing */
	};
}

/**
 * @brief Enable or disable RUNSTDBY for I2C controller (master) mode.
 *
 * This function sets or clears the RUNSTDBY bit in the SERCOM CTRLA register
 * for the I2C controller, allowing the peripheral to run in standby mode.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 */
static void i2c_controller_runstandby_enable(const struct device *dev)
{
	const i2c_mchp_dev_config_t *const i2c_cfg = dev->config;
	sercom_registers_t *i2c_regs = i2c_cfg->regs;
	uint32_t reg32_val = i2c_regs->I2CM.SERCOM_CTRLA;

	reg32_val &= ~SERCOM_I2CM_CTRLA_RUNSTDBY_Msk;
	reg32_val |= SERCOM_I2CM_CTRLA_RUNSTDBY(i2c_cfg->run_in_standby);
	i2c_regs->I2CM.SERCOM_CTRLA = reg32_val;
}

/**
 * @brief Enable automatic acknowledgment for the I2C controller.
 *
 * This API configures the I2C controller to automatically send an ACK (acknowledge)
 * after receiving a byte, by clearing the ACKACT bit in the control register.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 */
static inline void i2c_set_controller_auto_ack(const struct device *dev)
{
	sercom_registers_t *i2c_regs = ((const i2c_mchp_dev_config_t *)(dev)->config)->regs;

	i2c_regs->I2CM.SERCOM_CTRLB =
		((i2c_regs->I2CM.SERCOM_CTRLB & ~SERCOM_I2CM_CTRLB_ACKACT_Msk) |
		 SERCOM_I2CM_CTRLB_ACKACT(0));
}

/**
 * @brief Configure the I2C peripheral to operate in controller (master) mode.
 *
 * This API sets the necessary control register bits to enable controller (master) mode
 * for the I2C peripheral. It enables smart mode features, sets the controller mode,
 * enables SCL low time-out detection, and configures the inactive bus time-out.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 */
static inline void i2c_set_controller_mode(const struct device *dev)
{
	sercom_registers_t *i2c_regs = ((const i2c_mchp_dev_config_t *)(dev)->config)->regs;

	/* Enable i2c device smart mode features */
	i2c_regs->I2CM.SERCOM_CTRLB = ((i2c_regs->I2CM.SERCOM_CTRLB & ~SERCOM_I2CM_CTRLB_SMEN_Msk) |
				       SERCOM_I2CM_CTRLB_SMEN(1));

	i2c_regs->I2CM.SERCOM_CTRLA =
		(i2c_regs->I2CM.SERCOM_CTRLA &
		 ~(SERCOM_I2CM_CTRLA_MODE_Msk | SERCOM_I2CM_CTRLA_INACTOUT_Msk |
		   SERCOM_I2CM_CTRLA_LOWTOUTEN_Msk)) |
		(SERCOM_I2CM_CTRLA_MODE(0x5) | SERCOM_I2CM_CTRLA_LOWTOUTEN(1) |
		 SERCOM_I2CM_CTRLA_INACTOUT(0x3));
}

/**
 * @brief Send a Stop condition to terminate I2C communication in controller mode.
 *
 * This API issues a Stop condition on the I2C bus to terminate the current transfer
 * when operating in controller (master) mode. It sets the appropriate bits in the
 * control register and waits for the synchronization process to complete.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 */
static inline void i2c_controller_transfer_stop(const struct device *dev)
{
	sercom_registers_t *i2c_regs = ((const i2c_mchp_dev_config_t *)(dev)->config)->regs;

	i2c_regs->I2CM.SERCOM_CTRLB =
		(i2c_regs->I2CM.SERCOM_CTRLB &
		 ~(SERCOM_I2CM_CTRLB_ACKACT_Msk | SERCOM_I2CM_CTRLB_CMD_Msk)) |
		(SERCOM_I2CM_CTRLB_ACKACT(1) | SERCOM_I2CM_CTRLB_CMD(0x3));

	/* Wait for synchronization */
	while ((i2c_regs->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_SYSOP_Msk) ==
	       SERCOM_I2CM_SYNCBUSY_SYSOP_Msk) {

		/* Do nothing */
	};
}

/**
 * @brief Set the I2C controller bus state to idle.
 *
 * This API sets the I2C bus state to idle in controller (master) mode by writing
 * to the status register. It then waits for the synchronization process to complete.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 */
static inline void i2c_set_controller_bus_state_idle(const struct device *dev)
{
	sercom_registers_t *i2c_regs = ((const i2c_mchp_dev_config_t *)(dev)->config)->regs;

	i2c_regs->I2CM.SERCOM_STATUS = SERCOM_I2CM_STATUS_BUSSTATE(0x1);

	/* Wait for synchronization */
	while ((i2c_regs->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_SYSOP_Msk) ==
	       SERCOM_I2CM_SYNCBUSY_SYSOP_Msk) {

		/* Do nothing */
	};
}

/**
 * @brief Get the current I2C status flags in controller (master) mode.
 *
 * This API reads the I2C status register in controller mode and returns a bitmask
 * of status flags indicating various conditions such as bus error, arbitration lost,
 * bus busy, timeouts, and transaction length error.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @return Bitmask of current status flags (see @ref i2c_mchp_controller_status_flag_t).
 */
static i2c_mchp_controller_status_flag_t i2c_controller_status_get(const struct device *dev)
{
	uint16_t status_reg_val;
	i2c_mchp_controller_status_flag_t status_flags = I2C_MCHP_CONTROLLER_STATUS_FLAG_NONE;

	status_reg_val = I2C_REGS->I2CM.SERCOM_STATUS;

	if ((status_reg_val & SERCOM_I2CM_STATUS_BUSERR_Msk) == SERCOM_I2CM_STATUS_BUSERR_Msk) {
		status_flags |= I2C_MCHP_CONTROLLER_STATUS_FLAG_BUS_ERROR;
	}
	if ((status_reg_val & SERCOM_I2CM_STATUS_ARBLOST_Msk) == SERCOM_I2CM_STATUS_ARBLOST_Msk) {
		status_flags |= I2C_MCHP_CONTROLLER_STATUS_FLAG_ARBITRATION_LOST;
	}
	if ((status_reg_val & SERCOM_I2CM_STATUS_BUSSTATE_BUSY) ==
	    SERCOM_I2CM_STATUS_BUSSTATE_BUSY) {
		status_flags |= I2C_MCHP_CONTROLLER_STATUS_FLAG_BUS_STATE_BUSY;
	}
	if ((status_reg_val & SERCOM_I2CM_STATUS_MEXTTOUT_Msk) == SERCOM_I2CM_STATUS_MEXTTOUT_Msk) {
		status_flags |= I2C_MCHP_CONTROLLER_STATUS_FLAG_MEXTTOUT;
	}
	if ((status_reg_val & SERCOM_I2CM_STATUS_SEXTTOUT_Msk) == SERCOM_I2CM_STATUS_SEXTTOUT_Msk) {
		status_flags |= I2C_MCHP_CONTROLLER_STATUS_FLAG_SEXTTOUT;
	}
	if ((status_reg_val & SERCOM_I2CM_STATUS_LOWTOUT_Msk) == SERCOM_I2CM_STATUS_LOWTOUT_Msk) {
		status_flags |= I2C_MCHP_CONTROLLER_STATUS_FLAG_LOWTOUT;
	}
	if ((status_reg_val & SERCOM_I2CM_STATUS_ARBLOST_Msk) == SERCOM_I2CM_STATUS_ARBLOST_Msk) {
		status_flags |= I2C_MCHP_CONTROLLER_STATUS_FLAG_LENERR;
	}

	return status_flags;
}

/**
 * @brief Clear specific I2C status flags in controller (master) mode.
 *
 * This API clears the specified status flags in the I2C status register by writing
 * the corresponding bits. The function constructs a bitmask based on the provided
 * status flags and writes it to the status register to clear the indicated conditions.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param status_flags Bitmask of status flags to clear (see @ref
 * i2c_mchp_controller_status_flag_t).
 */
static void i2c_controller_status_clear(const struct device *dev,
					i2c_mchp_controller_status_flag_t status_flags)
{
	uint16_t status_clear = 0;

	if ((status_flags & I2C_MCHP_CONTROLLER_STATUS_FLAG_BUS_ERROR) ==
	    I2C_MCHP_CONTROLLER_STATUS_FLAG_BUS_ERROR) {
		status_clear |= SERCOM_I2CM_STATUS_BUSERR(1);
	}
	if ((status_flags & I2C_MCHP_CONTROLLER_STATUS_FLAG_ARBITRATION_LOST) ==
	    I2C_MCHP_CONTROLLER_STATUS_FLAG_ARBITRATION_LOST) {
		status_clear |= SERCOM_I2CM_STATUS_ARBLOST(1);
	}
	if ((status_flags & I2C_MCHP_CONTROLLER_STATUS_FLAG_BUS_STATE_BUSY) ==
	    I2C_MCHP_CONTROLLER_STATUS_FLAG_BUS_STATE_BUSY) {
		status_clear |= SERCOM_I2CM_STATUS_BUSSTATE(0x1);
	}
	if ((status_flags & I2C_MCHP_CONTROLLER_STATUS_FLAG_MEXTTOUT) ==
	    I2C_MCHP_CONTROLLER_STATUS_FLAG_MEXTTOUT) {
		status_clear |= SERCOM_I2CM_STATUS_MEXTTOUT(1);
	}
	if ((status_flags & I2C_MCHP_CONTROLLER_STATUS_FLAG_SEXTTOUT) ==
	    I2C_MCHP_CONTROLLER_STATUS_FLAG_SEXTTOUT) {
		status_clear |= SERCOM_I2CM_STATUS_SEXTTOUT(1);
	}
	if ((status_flags & I2C_MCHP_CONTROLLER_STATUS_FLAG_LOWTOUT) ==
	    I2C_MCHP_CONTROLLER_STATUS_FLAG_LOWTOUT) {
		status_clear |= SERCOM_I2CM_STATUS_LOWTOUT(1);
	}
	if ((status_flags & I2C_MCHP_CONTROLLER_STATUS_FLAG_LENERR) ==
	    I2C_MCHP_CONTROLLER_STATUS_FLAG_LENERR) {
		status_clear |= SERCOM_I2CM_STATUS_LENERR(1);
	}

	I2C_REGS->I2CM.SERCOM_STATUS = status_clear;
}

/**
 * @brief Enable I2C interrupts in controller (master) mode.
 *
 * This API enables specific I2C interrupts in controller mode by setting the
 * corresponding bits in the interrupt enable set register. The function constructs
 * a bitmask based on the provided interrupt flags and writes it to the register.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param controller_int Bitmask of controller interrupt flags to enable (see @ref
 * i2c_mchp_controller_interrupt_t).
 */
static void i2c_controller_int_enable(const struct device *dev,
				      i2c_mchp_controller_interrupt_t controller_int)
{
	uint8_t int_set = 0;

	if ((controller_int & I2C_MCHP_CONTROLLER_INTERRUPT_CONTROLLER_ON_BUS) ==
	    I2C_MCHP_CONTROLLER_INTERRUPT_CONTROLLER_ON_BUS) {
		int_set |= SERCOM_I2CM_INTENSET_MB(1);
	}
	if ((controller_int & I2C_MCHP_CONTROLLER_INTERRUPT_TARGET_ON_BUS) ==
	    I2C_MCHP_CONTROLLER_INTERRUPT_TARGET_ON_BUS) {
		int_set |= SERCOM_I2CM_INTENSET_SB(1);
	}
	if ((controller_int & I2C_MCHP_CONTROLLER_INTERRUPT_ERROR) ==
	    I2C_MCHP_CONTROLLER_INTERRUPT_ERROR) {
		int_set |= SERCOM_I2CM_INTENSET_ERROR(1);
	}

	I2C_REGS->I2CM.SERCOM_INTENSET = int_set;
}

/**
 * @brief Disable I2C interrupts in controller (master) mode.
 *
 * This API disables specific I2C interrupts in controller mode by setting the
 * corresponding bits in the interrupt enable clear register. The function constructs
 * a bitmask based on the provided interrupt flags and writes it to the register.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param controller_int Bitmask of controller interrupt flags to disable (see @ref
 * i2c_mchp_controller_interrupt_t).
 */
static void i2c_controller_int_disable(const struct device *dev,
				       i2c_mchp_controller_interrupt_t controller_int)
{
	uint8_t int_clear = 0;

	if ((controller_int & I2C_MCHP_CONTROLLER_INTERRUPT_CONTROLLER_ON_BUS) ==
	    I2C_MCHP_CONTROLLER_INTERRUPT_CONTROLLER_ON_BUS) {
		int_clear |= SERCOM_I2CM_INTENCLR_MB(1);
	}
	if ((controller_int & I2C_MCHP_CONTROLLER_INTERRUPT_TARGET_ON_BUS) ==
	    I2C_MCHP_CONTROLLER_INTERRUPT_TARGET_ON_BUS) {
		int_clear |= SERCOM_I2CM_INTENCLR_SB(1);
	}
	if ((controller_int & I2C_MCHP_CONTROLLER_INTERRUPT_ERROR) ==
	    I2C_MCHP_CONTROLLER_INTERRUPT_ERROR) {
		int_clear |= SERCOM_I2CM_INTENCLR_ERROR(1);
	}

	I2C_REGS->I2CM.SERCOM_INTENCLR = int_clear;
}

/**
 * @brief Get the I2C controller interrupt flag status.
 *
 * This API reads the I2C controller interrupt flag register and returns a bitmask
 * of currently set interrupt flags, such as master on bus, target on bus, and error.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @return Bitmask of currently set interrupt flags (see @ref i2c_mchp_controller_intflag_t).
 */
static i2c_mchp_controller_intflag_t i2c_controller_int_flag_get(const struct device *dev)
{
	uint8_t flag_reg_val;
	i2c_mchp_controller_intflag_t interrupt_flags = I2C_MCHP_CONTROLLER_INTFLAG_NONE;

	flag_reg_val = (uint8_t)I2C_REGS->I2CM.SERCOM_INTFLAG;

	if ((flag_reg_val & SERCOM_I2CM_INTFLAG_MB_Msk) == SERCOM_I2CM_INTFLAG_MB_Msk) {
		interrupt_flags |= I2C_MCHP_CONTROLLER_INTFLAG_CONTROLLER_ON_BUS;
	}
	if ((flag_reg_val & SERCOM_I2CM_INTFLAG_SB_Msk) == SERCOM_I2CM_INTFLAG_SB_Msk) {
		interrupt_flags |= I2C_MCHP_CONTROLLER_INTFLAG_TARGET_ON_BUS;
	}
	if ((flag_reg_val & SERCOM_I2CM_INTFLAG_ERROR_Msk) == SERCOM_I2CM_INTFLAG_ERROR_Msk) {
		interrupt_flags |= I2C_MCHP_CONTROLLER_INTFLAG_ERROR;
	}

	return interrupt_flags;
}

/**
 * @brief Clear specific I2C controller interrupt flags.
 *
 * This API clears the specified interrupt flags in the I2C controller by writing
 * the corresponding bits to the interrupt flag register.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param controller_intflag Bitmask of interrupt flags to clear (see @ref
 * i2c_mchp_controller_intflag_t).
 */
static void i2c_controller_int_flag_clear(const struct device *dev,
					  i2c_mchp_controller_intflag_t controller_intflag)
{
	uint8_t flag_clear = 0;

	if ((controller_intflag & I2C_MCHP_CONTROLLER_INTFLAG_CONTROLLER_ON_BUS) ==
	    I2C_MCHP_CONTROLLER_INTFLAG_CONTROLLER_ON_BUS) {
		flag_clear |= SERCOM_I2CM_INTFLAG_MB(1);
	}
	if ((controller_intflag & I2C_MCHP_CONTROLLER_INTFLAG_TARGET_ON_BUS) ==
	    I2C_MCHP_CONTROLLER_INTFLAG_TARGET_ON_BUS) {
		flag_clear |= SERCOM_I2CM_INTFLAG_SB(1);
	}
	if ((controller_intflag & I2C_MCHP_CONTROLLER_INTFLAG_ERROR) ==
	    I2C_MCHP_CONTROLLER_INTFLAG_ERROR) {
		flag_clear |= SERCOM_I2CM_INTFLAG_ERROR(1);
	}

	I2C_REGS->I2CM.SERCOM_INTFLAG = flag_clear;
}

/**
 * @brief Write the target address to the I2C controller address register.
 *
 * This API writes the specified address to the I2C controller's address register.
 * If the address indicates a read operation (based on the direction mask), the function
 * also enables automatic acknowledgment. After writing the address, it waits for
 * the synchronization process to complete.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param addr The 8-bit I2C address (including the R/W bit) to write to the address register.
 */
static void i2c_controller_addr_write(const struct device *dev, uint32_t addr)
{
	sercom_registers_t *i2c_regs = ((const i2c_mchp_dev_config_t *)(dev)->config)->regs;

	if ((addr & (uint32_t)I2C_MCHP_MESSAGE_DIR_READ_MASK) == I2C_MCHP_MESSAGE_DIR_READ_MASK) {
		i2c_set_controller_auto_ack(dev);
	}

	i2c_regs->I2CM.SERCOM_ADDR = (i2c_regs->I2CM.SERCOM_ADDR & ~SERCOM_I2CM_ADDR_ADDR_Msk) |
				     SERCOM_I2CM_ADDR_ADDR(addr);

	/* Wait for synchronization */
	while ((i2c_regs->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_SYSOP_Msk) ==
	       SERCOM_I2CM_SYNCBUSY_SYSOP_Msk) {

		/* Do nothing */
	};
}

/**
 * @brief Calculate the I2C baud rate register value.
 *
 * This API calculates the appropriate baud rate register value for the I2C peripheral
 * based on the desired bitrate and the system clock frequency. It supports standard,
 * fast, fast-plus, and high-speed I2C modes, and ensures the calculated value is within
 * valid hardware limits. The result is written to the provided output pointer.
 *
 * @param bitrate Desired I2C bus speed in Hz (e.g., 100000 for 100 kHz).
 * @param sys_clock_rate System clock frequency in Hz.
 * @param baud_val Pointer to store the calculated baud rate register value.
 * @return true if the calculation was successful, false if the system clock is too low.
 *
 */
static bool i2c_baudrate_calc(uint32_t bitrate, uint32_t sys_clock_rate, uint32_t *baud_val)
{
	uint32_t baud_value = 0U;
	float fsrc_clk_freq = (float)sys_clock_rate;
	float fi2c_clk_speed = (float)bitrate;
	float fbaud_value = 0.0f;
	bool is_calc_success = true;

	/* Reference clock frequency must be at least two times the baud rate */
	if (sys_clock_rate < (2U * bitrate)) {
		is_calc_success = false;
	} else {

		if (bitrate > I2C_SPEED_FAST_PLUS) {

			/* HS mode baud calculation */
			fbaud_value = (fsrc_clk_freq / fi2c_clk_speed) - 2.0f;
			baud_value = (uint32_t)fbaud_value;
		} else {

			/* Standard, FM and FM+ baud calculation */
			fbaud_value =
				(fsrc_clk_freq / fi2c_clk_speed) -
				((fsrc_clk_freq * I2C_MCHP_START_CONDITION_SETUP_TIME) + 10.0f);
			baud_value = (uint32_t)fbaud_value;
		}

		if (bitrate <= I2C_SPEED_FAST) {

			/* For I2C clock speed up to 400 kHz, the value of BAUD<7:0>
			 * determines both SCL_L and SCL_H with SCL_L = SCL_H
			 */
			if (baud_value > (0xFFU * 2U)) {

				/* Set baud rate to the maximum possible value */
				baud_value = 0xFFU;
			} else if (baud_value <= 1U) {

				/* Baud value cannot be 0. Set baud rate to minimum possible
				 * value
				 */
				baud_value = 1U;
			} else {
				baud_value /= 2U;
			}
		} else {

			/* To maintain the ratio of SCL_L:SCL_H to 2:1, the max value of
			 * BAUD_LOW<15:8>:BAUD<7:0> can be 0xFF:0x7F. Hence BAUD_LOW + BAUD
			 * can not exceed 255+127 = 382
			 */
			if (baud_value >= I2C_BAUD_LOW_HIGH_MAX) {

				/* Set baud rate to the minimum possible value while
				 * maintaining SCL_L:SCL_H to 2:1
				 */
				baud_value = (0xFFUL << 8U) | (0x7FU);
			} else if (baud_value <= 3U) {

				/* Baud value cannot be 0. Set baud rate to maximum possible
				 * value while maintaining SCL_L:SCL_H to 2:1
				 */
				baud_value = (2UL << 8U) | 1U;
			} else {

				/* For Fm+ mode, I2C SCL_L:SCL_H to 2:1 */
				baud_value = ((((baud_value * 2U) / 3U) << 8U) | (baud_value / 3U));
			}
		}
		*baud_val = baud_value;
	}

	return is_calc_success;
}

/**
 * @brief Set the I2C baud rate and speed mode for the controller.
 *
 * This API calculates and sets the appropriate baud rate register value and speed mode
 * for the I2C controller based on the desired bitrate and system clock frequency.
 * It supports standard, fast, fast-plus, and high-speed I2C modes, and configures
 * the SDA hold time as required by the selected mode.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param bitrate Desired I2C bus speed in Hz (e.g., 100000 for 100 kHz, 400000 for 400 kHz, 3400000
 * for 3.4 MHz).
 * @param sys_clock_rate System clock frequency in Hz.
 * @return true if the baud rate was successfully set, false otherwise.
 *
 */
static bool i2c_set_baudrate(const struct device *dev, uint32_t bitrate, uint32_t sys_clock_rate)
{
	uint32_t i2c_speed_mode = 0;
	uint32_t baud_value;
	uint32_t hsbaud_value;
	uint32_t sda_hold_time = 0;
	bool is_success = false;

	if (bitrate == I2C_SPEED_HIGH) {

		/* HS mode requires baud values for both FS and HS frequency. First
		 * calculate baud for FS
		 */
		if (i2c_baudrate_calc(I2C_SPEED_FAST, sys_clock_rate, &baud_value) == true) {
			if (i2c_baudrate_calc(bitrate, sys_clock_rate, &hsbaud_value) == true) {
				is_success = true;
				baud_value |= (hsbaud_value << 16U);
				i2c_speed_mode = 2U;
				sda_hold_time = 2U;
			}
		}
	} else {
		if (i2c_baudrate_calc(bitrate, sys_clock_rate, &baud_value) == true) {
			if (bitrate == I2C_SPEED_FAST_PLUS) {
				i2c_speed_mode = 1U;
				sda_hold_time = 1U;
			}
			is_success = true;
		}
	}

	if (is_success == true) {
		sercom_registers_t *i2c_regs = ((const i2c_mchp_dev_config_t *)(dev)->config)->regs;

		/* Baud rate - controller Baud Rate*/
		i2c_regs->I2CM.SERCOM_BAUD = baud_value;

		i2c_regs->I2CM.SERCOM_CTRLA =
			((i2c_regs->I2CM.SERCOM_CTRLA &
			  (~SERCOM_I2CM_CTRLA_SPEED_Msk | ~SERCOM_I2CM_CTRLA_SDAHOLD_Msk)) |
			 (SERCOM_I2CM_CTRLA_SPEED(i2c_speed_mode) |
			  SERCOM_I2CM_CTRLA_SDAHOLD(sda_hold_time)));
	}

	return is_success;
}

/**
 * @brief Terminates the current I2C operation in case of an error.
 *
 * This function checks for any errors in the I2C operation, stops ongoing DMA transfers
 * (if enabled), clears necessary flags, disables interrupts, stops the I2C transfer, and
 * releases the semaphore to signal the completion of the operation.
 *
 * @param dev Pointer to the device structure for the I2C driver.
 * @return true if the termination is successful, false if an error is detected.
 */
static bool i2c_is_terminate_on_error(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;
	bool retval = true;

	/* Retrieve and store the current I2C status in the device data structure. */
	data->current_msg.status = i2c_controller_status_get(dev);

	/* Check for any I2C errors. If an error is found, terminate early. */
	if (data->current_msg.status == I2C_MCHP_CONTROLLER_STATUS_FLAG_NONE) {
		retval = false;
	} else {

		/*
		 * Clear all the status flags that require an explicit clear operation.
		 * Some flags are cleared automatically by writing to specific registers (e.g., ADDR
		 * writes).
		 */
		i2c_controller_status_clear(dev, data->current_msg.status);

		/* Disable all I2C interrupts to prevent further processing. */
		i2c_controller_int_disable(dev, I2C_MCHP_CONTROLLER_INTERRUPT_ALL);

		/* Stop the I2C transfer explicitly. */
		i2c_controller_transfer_stop(dev);

#ifdef CONFIG_I2C_CALLBACK
		/* Callback to the application for async */
		data->i2c_async_callback(dev, (int)data->current_msg.status, data->user_data);
#else
		/* Release the semaphore to signal the completion of the operation. */
		k_sem_give(&data->i2c_sync_sem);
#endif
	}

	/* Return true to indicate successful termination of the operation. */
	return retval;
}

/**
 * @brief Restart the I2C transaction with the target device.
 *
 * This function handles preparing and restarting an I2C transaction. It configures
 * the address register, handles read or write-specific setups (including optional DMA),
 * and enables the I2C interrupts or DMA operations as needed.
 *
 * @param dev Pointer to the device structure for the I2C driver.
 */
static void i2c_restart(const struct device *dev)
{
	/* Retrieve device data and configuration. */
	i2c_mchp_dev_data_t *data = dev->data;

	/* Prepare the address register (left-shift address by 1 for R/W bit). */
	uint32_t addr_reg = data->target_addr << 1U;

	/* Check if the operation is a read. */
	if ((data->msgs_array[data->msg_index].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
		addr_reg |= 1U; /* Set the R/W bit to indicate a read operation. */

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
		/* Configure DMA for read operation. If it fails, exit the function. */
		if (i2c_dma_read_config(dev) != 0) {
			return;
		}
#endif

	} else { /* Write operation */
#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN

		/* Configure DMA for write operation. If it fails, exit the function. */
		if (i2c_dma_write_config(dev) != 0) {
			return;
		}
#endif
	}

	/*
	 * Writing the target address to the I2C hardware starts the transaction.
	 * This will issue a START or a repeated START as required.
	 */
	i2c_controller_addr_write(dev, addr_reg);

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
	const i2c_mchp_dev_config_t *const cfg = dev->config;

	/* Handle read or write transaction setup. */
	if ((data->msgs_array[data->msg_index].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {

		/* Start the DMA read operation and handle any errors. */
		int retval = dma_start(cfg->i2c_dma.dma_dev, cfg->i2c_dma.rx_dma_channel);

		if (retval != 0) {
			LOG_ERR("Read DMA start on %s failed: %d", dev->name, retval);
			data->i2c_async_callback(dev, retval, data->user_data);
			return;
		}

	} else { /* Write operation */

		/* Start the DMA write operation and handle any errors. */
		int retval = dma_start(cfg->i2c_dma.dma_dev, cfg->i2c_dma.tx_dma_channel);

		if (retval != 0) {
			LOG_ERR("Write DMA start on %s failed: %d", dev->name, retval);
			data->i2c_async_callback(dev, retval, data->user_data);
			return;
		}
	}

#else
	/* Enable I2C interrupts for non-DMA write operation. */
	i2c_controller_int_enable(dev, I2C_MCHP_CONTROLLER_INTERRUPT_ALL);
#endif
}

#ifdef CONFIG_I2C_TARGET
/**
 * @brief Get the I2C target interrupt flag status.
 *
 * This API reads the I2C target interrupt flag register and returns a bitmask
 * of currently set interrupt flags, such as STOP condition, address match, data ready, and error.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @return Bitmask of currently set interrupt flags (see @ref i2c_mchp_target_intflag_t).
 */
static i2c_mchp_target_intflag_t i2c_target_int_flag_get(const struct device *dev)
{
	uint8_t flag_reg_val;
	i2c_mchp_target_intflag_t interrupt_flags = I2C_MCHP_TARGET_INTFLAG_NONE;

	flag_reg_val = (uint8_t)I2C_REGS->I2CS.SERCOM_INTFLAG;

	if ((flag_reg_val & SERCOM_I2CS_INTFLAG_PREC_Msk) == SERCOM_I2CS_INTFLAG_PREC_Msk) {
		interrupt_flags |= I2C_MCHP_TARGET_INTFLAG_STOP;
	}
	if ((flag_reg_val & SERCOM_I2CS_INTFLAG_AMATCH_Msk) == SERCOM_I2CS_INTFLAG_AMATCH_Msk) {
		interrupt_flags |= I2C_MCHP_TARGET_INTFLAG_ADDR_MATCH;
	}
	if ((flag_reg_val & SERCOM_I2CS_INTFLAG_DRDY_Msk) == SERCOM_I2CS_INTFLAG_DRDY_Msk) {
		interrupt_flags |= I2C_MCHP_TARGET_INTFLAG_DATA_READY;
	}
	if ((flag_reg_val & SERCOM_I2CS_INTFLAG_ERROR_Msk) == SERCOM_I2CS_INTFLAG_ERROR_Msk) {
		interrupt_flags |= I2C_MCHP_TARGET_INTFLAG_ERROR;
	}

	return interrupt_flags;
}

/**
 * @brief Get the current I2C status flags in target (slave) mode.
 *
 * This API reads the I2C status register in target mode and returns a bitmask
 * of status flags indicating various conditions such as bus error, collision,
 * data direction, and timeouts.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @return Bitmask of current status flags (see @ref i2c_mchp_target_status_flag_t).
 */
static i2c_mchp_target_status_flag_t i2c_target_status_get(const struct device *dev)
{
	uint16_t status_reg_val;
	i2c_mchp_target_status_flag_t status_flags = I2C_MCHP_TARGET_STATUS_FLAG_NONE;

	status_reg_val = I2C_REGS->I2CS.SERCOM_STATUS;

	if ((status_reg_val & SERCOM_I2CS_STATUS_BUSERR_Msk) == SERCOM_I2CS_STATUS_BUSERR_Msk) {
		status_flags |= I2C_MCHP_TARGET_STATUS_FLAG_BUS_ERROR;
	}
	if ((status_reg_val & SERCOM_I2CS_STATUS_COLL_Msk) == SERCOM_I2CS_STATUS_COLL_Msk) {
		status_flags |= I2C_MCHP_TARGET_STATUS_FLAG_COLL;
	}
	if ((status_reg_val & SERCOM_I2CS_STATUS_DIR_Msk) == SERCOM_I2CS_STATUS_DIR_Msk) {
		status_flags |= I2C_MCHP_TARGET_STATUS_FLAG_DATA_DIR_READ;
	}
	if ((status_reg_val & SERCOM_I2CS_STATUS_LOWTOUT_Msk) == SERCOM_I2CS_STATUS_LOWTOUT_Msk) {
		status_flags |= I2C_MCHP_TARGET_STATUS_FLAG_LOWTOUT;
	}
	if ((status_reg_val & SERCOM_I2CS_STATUS_SEXTTOUT_Msk) == SERCOM_I2CS_STATUS_SEXTTOUT_Msk) {
		status_flags |= I2C_MCHP_TARGET_STATUS_FLAG_SEXTTOUT;
	}

	return status_flags;
}

/**
 * @brief Clear specific I2C status flags in target (slave) mode.
 *
 * This API clears the specified status flags in the I2C status register by writing
 * the corresponding bits. The function constructs a bitmask based on the provided
 * status flags and writes it to the status register to clear the indicated conditions.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param status_flags Bitmask of status flags to clear (see @ref i2c_mchp_target_status_flag_t).
 */
static void i2c_target_status_clear(const struct device *dev,
				    i2c_mchp_target_status_flag_t status_flags)
{
	uint16_t status_clear = 0;

	if ((status_flags & I2C_MCHP_TARGET_STATUS_FLAG_BUS_ERROR) ==
	    I2C_MCHP_TARGET_STATUS_FLAG_BUS_ERROR) {
		status_clear |= SERCOM_I2CS_STATUS_BUSERR(1);
	}
	if ((status_flags & I2C_MCHP_TARGET_STATUS_FLAG_COLL) == I2C_MCHP_TARGET_STATUS_FLAG_COLL) {
		status_clear |= SERCOM_I2CS_STATUS_COLL(1);
	}
	if ((status_flags & I2C_MCHP_TARGET_STATUS_FLAG_LOWTOUT) ==
	    I2C_MCHP_TARGET_STATUS_FLAG_LOWTOUT) {
		status_clear |= SERCOM_I2CS_STATUS_LOWTOUT(1);
	}
	if ((status_flags & I2C_MCHP_TARGET_STATUS_FLAG_SEXTTOUT) ==
	    I2C_MCHP_TARGET_STATUS_FLAG_SEXTTOUT) {
		status_clear |= SERCOM_I2CS_STATUS_SEXTTOUT(1);
	}

	I2C_REGS->I2CS.SERCOM_STATUS = status_clear;
}

/**
 * @brief Clear specific I2C target interrupt flags.
 *
 * This API clears the specified interrupt flags in the I2C target by writing
 * the corresponding bits to the interrupt flag register.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param target_intflag Bitmask of interrupt flags to clear (see @ref i2c_mchp_target_intflag_t).
 */
static void i2c_target_int_flag_clear(const struct device *dev,
				      i2c_mchp_target_intflag_t target_intflag)
{
	uint8_t flag_clear = 0;

	if ((target_intflag & I2C_MCHP_TARGET_INTFLAG_STOP) == I2C_MCHP_TARGET_INTFLAG_STOP) {
		flag_clear |= SERCOM_I2CS_INTFLAG_PREC(1);
	}
	if ((target_intflag & I2C_MCHP_TARGET_INTFLAG_ADDR_MATCH) ==
	    I2C_MCHP_TARGET_INTFLAG_ADDR_MATCH) {
		flag_clear |= SERCOM_I2CS_INTFLAG_AMATCH(1);
	}
	if ((target_intflag & I2C_MCHP_TARGET_INTFLAG_DATA_READY) ==
	    I2C_MCHP_TARGET_INTFLAG_DATA_READY) {
		flag_clear |= SERCOM_I2CS_INTFLAG_DRDY(1);
	}
	if ((target_intflag & I2C_MCHP_TARGET_INTFLAG_ERROR) == I2C_MCHP_TARGET_INTFLAG_ERROR) {
		flag_clear |= SERCOM_I2CS_INTFLAG_ERROR(1);
	}

	I2C_REGS->I2CS.SERCOM_INTFLAG = flag_clear;
}

/**
 * @brief Get the ACK/NACK status of the last byte transferred in I2C target mode.
 *
 * This function checks the SERCOM I2C target STATUS register to determine whether
 * the last byte transferred was acknowledged (ACK) or not acknowledged (NACK) by the I2C master.
 *
 * @return i2c_mchp_target_ack_status
 */
static inline i2c_mchp_target_ack_status_t
i2c_target_get_lastbyte_ack_status(const struct device *dev)
{
	return ((I2C_REGS->I2CS.SERCOM_STATUS & SERCOM_I2CS_STATUS_RXNACK_Msk) != 0U)
		       ? I2C_MCHP_TARGET_ACK_STATUS_RECEIVED_NACK
		       : I2C_MCHP_TARGET_ACK_STATUS_RECEIVED_ACK;
}

/**
 * @brief Set the I2C target (slave) command for the SERCOM peripheral.
 *
 * This function issues a specific command to the I2C target (slave) hardware
 * by updating the SERCOM CTRLB register.
 *
 * @param dev Pointer to the I2C device structure.
 * @param cmd The command to issue to the I2C target.
 */
void i2c_target_set_command(const struct device *dev, i2c_mchp_target_cmd_t cmd)
{
	sercom_registers_t *i2c_regs = ((const i2c_mchp_dev_config_t *)(dev)->config)->regs;

	/* Clear CMD bits first */
	i2c_regs->I2CS.SERCOM_CTRLB &= ~SERCOM_I2CS_CTRLB_CMD_Msk;

	switch (cmd) {
	case I2C_MCHP_TARGET_COMMAND_SEND_ACK:
		i2c_regs->I2CS.SERCOM_CTRLB =
			(i2c_regs->I2CS.SERCOM_CTRLB & ~SERCOM_I2CS_CTRLB_ACKACT_Msk) |
			SERCOM_I2CS_CTRLB_CMD(0x03UL);
		break;
	case I2C_MCHP_TARGET_COMMAND_SEND_NACK:
		i2c_regs->I2CS.SERCOM_CTRLB =
			(i2c_regs->I2CS.SERCOM_CTRLB | SERCOM_I2CS_CTRLB_ACKACT_Msk) |
			SERCOM_I2CS_CTRLB_CMD(0x03UL);
		break;
	case I2C_MCHP_TARGET_COMMAND_RECEIVE_ACK_NAK:
		i2c_regs->I2CS.SERCOM_CTRLB |= SERCOM_I2CS_CTRLB_CMD(0x03UL);
		break;
	case I2C_MCHP_TARGET_COMMAND_WAIT_FOR_START:
		i2c_regs->I2CS.SERCOM_CTRLB =
			(i2c_regs->I2CS.SERCOM_CTRLB | SERCOM_I2CS_CTRLB_ACKACT_Msk) |
			SERCOM_I2CS_CTRLB_CMD(0x02UL);
		break;
	default:
		break;
	}
}

/**
 * @brief Handle target mode interrupts for the I2C peripheral.
 *
 * This function processes I2C target-specific interrupt events such as
 * address match, data ready, and stop condition. It calls the appropriate
 * callbacks based on the interrupt type.
 *
 * @param dev Pointer to the I2C device structure.
 */
static void i2c_target_handler(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;

	/* Get the target configuration and callbacks */
	const struct i2c_target_callbacks *target_cb = &data->target_callbacks;
	int retval = 0;

	/* Retrieve the interrupt status for the I2C peripheral */
	i2c_mchp_target_intflag_t int_status = i2c_target_int_flag_get(dev);

	/*Get the current status of target device */
	i2c_mchp_target_status_flag_t target_status = i2c_target_status_get(dev);

	do {

		/* Handle error conditions */
		if ((int_status & I2C_MCHP_TARGET_INTFLAG_ERROR) == I2C_MCHP_TARGET_INTFLAG_ERROR) {
			i2c_target_int_flag_clear(dev, I2C_MCHP_TARGET_INTFLAG_ERROR);
			LOG_ERR("Interrupt Error generated");
			target_cb->stop(&data->target_config);
			break;
		}

		/* Handle address match */
		if ((int_status & I2C_MCHP_TARGET_INTFLAG_ADDR_MATCH) ==
		    I2C_MCHP_TARGET_INTFLAG_ADDR_MATCH) {

			i2c_target_set_command(dev, I2C_MCHP_TARGET_COMMAND_SEND_ACK);

			data->firstReadAfterAddrMatch = true;

			if ((target_status & I2C_MCHP_TARGET_STATUS_FLAG_DATA_DIR_READ) ==
			    I2C_MCHP_TARGET_STATUS_FLAG_DATA_DIR_READ) {

				/* Load the first byte for host read */
				target_cb->read_requested(&data->target_config, &data->rx_tx_data);
			} else {

				/* Host is writing */
				target_cb->write_requested(&data->target_config);
			}
		}

		/* Handle data ready (Read/Write Operations) */
		if ((int_status & I2C_MCHP_TARGET_INTFLAG_DATA_READY) ==
		    I2C_MCHP_TARGET_INTFLAG_DATA_READY) {

			if (((target_status & I2C_MCHP_TARGET_STATUS_FLAG_DATA_DIR_READ) ==
			     I2C_MCHP_TARGET_STATUS_FLAG_DATA_DIR_READ)) {

				if ((data->firstReadAfterAddrMatch == true) ||
				    (i2c_target_get_lastbyte_ack_status(dev) ==
				     I2C_MCHP_TARGET_ACK_STATUS_RECEIVED_ACK)) {

					/* Host is reading */
					i2c_byte_write(dev, data->rx_tx_data);

					/* first byte read after address match done*/
					data->firstReadAfterAddrMatch = false;

					i2c_target_set_command(
						dev, I2C_MCHP_TARGET_COMMAND_RECEIVE_ACK_NAK);

					/* Load the next byte for host read*/
					target_cb->read_processed(&data->target_config,
								  &data->rx_tx_data);

				} else {
					i2c_target_set_command(
						dev, I2C_MCHP_TARGET_COMMAND_WAIT_FOR_START);
				}

			} else {

				/* Host is writing */
				i2c_target_set_command(dev, I2C_MCHP_TARGET_COMMAND_SEND_ACK);
				data->rx_tx_data = i2c_byte_read(dev);
				retval = target_cb->write_received(&data->target_config,
								   data->rx_tx_data);
				if (retval != 0) {
					i2c_target_set_command(dev,
							       I2C_MCHP_TARGET_COMMAND_SEND_NACK);
				}
			}
		}

	} while (0);

	/* Handle stop condition interrupt */
	if ((int_status & I2C_MCHP_TARGET_INTFLAG_STOP) == I2C_MCHP_TARGET_INTFLAG_STOP) {
		i2c_target_int_flag_clear(dev, I2C_MCHP_TARGET_INTFLAG_STOP);

		/* Notify that a stop condition was received */
		target_cb->stop(&data->target_config);
	}

	i2c_target_status_clear(dev, target_status);
}
#endif /* CONFIG_I2C_TARGET */

static void i2c_mchp_isr(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;
	bool continue_next = false;

#ifdef CONFIG_I2C_TARGET
	/* Check if the device is operating in target mode */
	if (data->target_mode == true) {

		/* Delegate target-specific operations to a dedicated handler */
		i2c_target_handler(dev);
		return;
	}
#endif

	/* Get current interrupt status to identify the cause of the interrupt */
	i2c_mchp_controller_intflag_t int_status = i2c_controller_int_flag_get(dev);

	/* Terminate if there are any critical errors on the bus */
	if (i2c_is_terminate_on_error(dev) != false) {
		return;
	}

	/* Handle ERROR interrupt flag for controller mode transmit and receive */
	if (int_status == I2C_MCHP_CONTROLLER_INTFLAG_ERROR) {
		i2c_controller_transfer_stop(dev);

		/*Disable the all i2c interrupts*/
		i2c_controller_int_disable(dev, I2C_MCHP_CONTROLLER_INTERRUPT_ALL);

#ifdef CONFIG_I2C_CALLBACK
		/* Callback to the application for async */
		data->i2c_async_callback(dev, (int)data->current_msg.status, data->user_data);
#else
		k_sem_give(&data->i2c_sync_sem);
#endif
		return;
	}

	/* Check if there are remaining messages in the same direction */
	if ((data->current_msg.size == 1) && (data->num_msgs > 1)) {

		/* Check if there are current or next message direction are same */
		if ((data->msgs_array[data->msg_index].flags & I2C_MSG_RW_MASK) ==
		    (data->msgs_array[data->msg_index + 1].flags & I2C_MSG_RW_MASK)) {

			/* Check if there are next message buffer are not restart */
			if ((data->msgs_array[data->msg_index + 1].flags & I2C_MSG_RESTART) == 0) {
				continue_next = true;
			}
		}
	}

	/* Handle interrupts for controller mode transmit */
	if (int_status == I2C_MCHP_CONTROLLER_INTFLAG_CONTROLLER_ON_BUS) {

		/* If no data remains, complete the transfer */
		if (data->current_msg.size == 0) {
			i2c_controller_transfer_stop(dev);

			i2c_controller_int_disable(dev,
						   I2C_MCHP_CONTROLLER_INTFLAG_CONTROLLER_ON_BUS);

			if (data->num_msgs > 1) {

				data->msg_index++;
				data->num_msgs--;

				data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
				data->current_msg.size = data->msgs_array[data->msg_index].len;
				data->current_msg.status = I2C_MCHP_CONTROLLER_STATUS_FLAG_NONE;

				i2c_restart(dev);
			} else {
#ifdef CONFIG_I2C_CALLBACK

				/* Callback to the application for async */
				data->i2c_async_callback(dev, (int)data->current_msg.status,
							 data->user_data);
#else
				k_sem_give(&data->i2c_sync_sem);
#endif
			}

			return;
		}

		/* Transmit the next byte from the buffer */
		i2c_byte_write(dev, *data->current_msg.buffer);
		data->current_msg.buffer++;
		data->current_msg.size--;
	}

	/* Handle interrupts for controller mode receive */
	else if (int_status == I2C_MCHP_CONTROLLER_INTFLAG_TARGET_ON_BUS) {

		/* Prepare for an auto NACK if this is the last byte */
		if (!continue_next && (data->current_msg.size == 1)) {
			i2c_controller_transfer_stop(dev);
		}

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
		/*If transaction in DMA mode then no need to read data just send NACK if num
		 * messages are 1 or if nume messages greater then 1 then restart the communication
		 */
#else
		/* Receive the next byte into the buffer */
		*data->current_msg.buffer = i2c_byte_read(dev);
		data->current_msg.buffer++;
		data->current_msg.size--;
#endif

		/* Complete the transfer if no data remains */
		if ((continue_next == false) && (data->current_msg.size == 0)) {
			i2c_controller_int_disable(dev, I2C_MCHP_CONTROLLER_INTFLAG_TARGET_ON_BUS);

			if (data->num_msgs > 1) {

				data->msg_index++;
				data->num_msgs--;

				data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
				data->current_msg.size = data->msgs_array[data->msg_index].len;
				data->current_msg.status = I2C_MCHP_CONTROLLER_STATUS_FLAG_NONE;

				i2c_restart(dev);
			} else {
#ifdef CONFIG_I2C_CALLBACK

				/* Callback to the application for async */
				data->i2c_async_callback(dev, (int)data->current_msg.status,
							 data->user_data);
#else
				k_sem_give(&data->i2c_sync_sem);
#endif
			}

			return;
		}
	}

	/* Handle the continuation of messages in the same direction */
	if (continue_next == true) {
		data->msg_index++;
		data->num_msgs--;

		data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
		data->current_msg.size = data->msgs_array[data->msg_index].len;
		data->current_msg.status = I2C_MCHP_CONTROLLER_STATUS_FLAG_NONE;
	}
}

#ifdef CONFIG_I2C_MCHP_TARGET
/**
 * @brief Enable or disable the I2C peripheral in target (slave) mode.
 *
 * This API enables or disables the I2C peripheral by setting or clearing the
 * enable bit in the control register for target mode. It waits for the
 * synchronization process to complete after the operation.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param enable Set to true to enable the target, false to disable.
 */
static void i2c_target_enable(const struct device *dev, bool enable)
{
	sercom_registers_t *i2c_regs = ((const i2c_mchp_dev_config_t *)(dev)->config)->regs;

	if (enable == true) {
		i2c_regs->I2CS.SERCOM_CTRLA |= SERCOM_I2CS_CTRLA_ENABLE(1);
	} else {
		i2c_regs->I2CS.SERCOM_CTRLA &= SERCOM_I2CS_CTRLA_ENABLE(0);
	}

	/* Wait for synchronization */
	while ((i2c_regs->I2CS.SERCOM_SYNCBUSY & SERCOM_I2CS_SYNCBUSY_ENABLE_Msk) ==
	       SERCOM_I2CS_SYNCBUSY_ENABLE_Msk) {

		/* Do nothing */
	};
}

/**
 * @brief Enable or disable RUNSTDBY for I2C target (slave) mode.
 *
 * This function sets or clears the RUNSTDBY bit in the SERCOM CTRLA register
 * for the I2C target, allowing the peripheral to run in standby mode.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 */
static void i2c_target_runstandby_enable(const struct device *dev)
{
	const i2c_mchp_dev_config_t *const i2c_cfg = dev->config;
	sercom_registers_t *i2c_regs = i2c_cfg->regs;
	uint32_t reg32_val = i2c_regs->I2CS.SERCOM_CTRLA;

	reg32_val &= ~SERCOM_I2CS_CTRLA_RUNSTDBY_Msk;
	reg32_val |= SERCOM_I2CS_CTRLA_RUNSTDBY(i2c_cfg->run_in_standby);
	i2c_regs->I2CS.SERCOM_CTRLA = reg32_val;
}

/**
 * @brief Configure the I2C peripheral to operate in target (slave) mode.
 *
 * This API sets the necessary control register bits to enable target (slave) mode
 * for the I2C peripheral. It enables smart mode features, sets the target mode,
 * configures SDA hold time, and sets the speed.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 */
static void i2c_set_target_mode(const struct device *dev)
{
	sercom_registers_t *i2c_regs = ((const i2c_mchp_dev_config_t *)(dev)->config)->regs;

	/* Enable i2c device smart mode features */
	i2c_regs->I2CS.SERCOM_CTRLB = ((i2c_regs->I2CS.SERCOM_CTRLB & ~SERCOM_I2CS_CTRLB_SMEN_Msk) |
				       SERCOM_I2CS_CTRLB_SMEN(1));

	i2c_regs->I2CS.SERCOM_CTRLA =
		(i2c_regs->I2CS.SERCOM_CTRLA & ~SERCOM_I2CS_CTRLA_MODE_Msk) |
		(SERCOM_I2CS_CTRLA_MODE(0x4) | SERCOM_I2CS_CTRLA_SDAHOLD(0x1) |
		 SERCOM_I2CS_CTRLA_SPEED(0x1));
}

/**
 * @brief Reset the I2C target address register.
 *
 * This API resets the I2C target address register by clearing the address field.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 */
static void i2c_reset_target_addr(const struct device *dev)
{
	sercom_registers_t *i2c_regs = ((const i2c_mchp_dev_config_t *)(dev)->config)->regs;

	i2c_regs->I2CS.SERCOM_ADDR = (i2c_regs->I2CS.SERCOM_ADDR & ~SERCOM_I2CS_ADDR_ADDR_Msk) |
				     SERCOM_I2CS_ADDR_ADDR(0);
}

/**
 * @brief Enable I2C target (slave) interrupts.
 *
 * This API enables specific I2C target interrupts by setting the corresponding bits
 * in the interrupt enable set register. The function constructs a bitmask based on
 * the provided interrupt flags and writes it to the register.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param target_int Bitmask of target interrupt flags to enable (see @ref
 * i2c_mchp_target_interrupt_t).
 */
static void i2c_target_int_enable(const struct device *dev, i2c_mchp_target_interrupt_t target_int)
{
	uint8_t int_set = 0;

	if ((target_int & I2C_MCHP_TARGET_INTERRUPT_STOP) == I2C_MCHP_TARGET_INTERRUPT_STOP) {
		int_set |= SERCOM_I2CS_INTENSET_PREC(1);
	}
	if ((target_int & I2C_MCHP_TARGET_INTERRUPT_ADDR_MATCH) ==
	    I2C_MCHP_TARGET_INTERRUPT_ADDR_MATCH) {
		int_set |= SERCOM_I2CS_INTENSET_AMATCH(1);
	}
	if ((target_int & I2C_MCHP_TARGET_INTERRUPT_DATA_READY) ==
	    I2C_MCHP_TARGET_INTERRUPT_DATA_READY) {
		int_set |= SERCOM_I2CS_INTENSET_DRDY(1);
	}
	if ((target_int & I2C_MCHP_TARGET_INTERRUPT_ERROR) == I2C_MCHP_TARGET_INTERRUPT_ERROR) {
		int_set |= SERCOM_I2CS_INTENSET_ERROR(1);
	}

	I2C_REGS->I2CS.SERCOM_INTENSET = int_set;
}

/**
 * @brief Disable I2C target (slave) interrupts.
 *
 * This API disables specific I2C target interrupts by setting the corresponding bits
 * in the interrupt enable clear register. The function constructs a bitmask based on
 * the provided interrupt flags and writes it to the register.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param target_int Bitmask of target interrupt flags to disable (see @ref
 * i2c_mchp_target_interrupt_t).
 */
static void i2c_target_int_disable(const struct device *dev, i2c_mchp_target_interrupt_t target_int)
{
	uint8_t int_clear = 0;

	if ((target_int & I2C_MCHP_TARGET_INTERRUPT_STOP) == I2C_MCHP_TARGET_INTERRUPT_STOP) {
		int_clear |= SERCOM_I2CS_INTENCLR_PREC(1);
	}
	if ((target_int & I2C_MCHP_TARGET_INTERRUPT_ADDR_MATCH) ==
	    I2C_MCHP_TARGET_INTERRUPT_ADDR_MATCH) {
		int_clear |= SERCOM_I2CS_INTENCLR_AMATCH(1);
	}
	if ((target_int & I2C_MCHP_TARGET_INTERRUPT_DATA_READY) ==
	    I2C_MCHP_TARGET_INTERRUPT_DATA_READY) {
		int_clear |= SERCOM_I2CS_INTENCLR_DRDY(1);
	}
	if ((target_int & I2C_MCHP_TARGET_INTERRUPT_ERROR) == I2C_MCHP_TARGET_INTERRUPT_ERROR) {
		int_clear |= SERCOM_I2CS_INTENCLR_ERROR(1);
	}

	I2C_REGS->I2CS.SERCOM_INTENCLR = int_clear;
}

/**
 * @brief Set the I2C peripheral's own unique address in target (slave) mode.
 *
 * This API configures the I2C peripheral with its own address for target (slave) mode
 * by writing the specified address to the address register.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param addr The 7-bit or 10-bit I2C address to set for the target.
 */
static void i2c_set_target_addr(const struct device *dev, uint32_t addr)
{
	sercom_registers_t *i2c_regs = ((const i2c_mchp_dev_config_t *)(dev)->config)->regs;

	i2c_regs->I2CS.SERCOM_ADDR = (i2c_regs->I2CS.SERCOM_ADDR & ~SERCOM_I2CS_ADDR_ADDR_Msk) |
				     SERCOM_I2CS_ADDR_ADDR(addr);
}

/**
 * @brief Register the device in I2C target mode.
 *
 * This function configures the I2C peripheral to operate in target mode
 * with the specified target configuration, including address and callback functions.
 *
 * @param dev Pointer to the I2C device structure.
 * @param target_cfg Pointer to the target configuration structure.
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_mchp_target_register(const struct device *dev, struct i2c_target_config *target_cfg)
{
	i2c_mchp_dev_data_t *data = dev->data;
	int retval = 0;

	/* Check if the device is already operating in target mode */
	if (data->target_mode == true) {
		LOG_ERR("Device already registered in target mode.");
		return -EBUSY;
	}

	/* Validate the target configuration and its callbacks */
	if ((target_cfg == NULL) || (target_cfg->callbacks == NULL)) {
		return -EINVAL;
	}

	/*Check if the target address is invalid */
	if (target_cfg->address == I2C_INVALID_ADDR) {
		LOG_ERR("device can't be register in target mode with 0x00 address\n");
		return -EINVAL;
	}

	/* Acquire the mutex to ensure thread-safe access */
	k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);

	/*Store the target configuration in device data*/
	data->target_config.address = target_cfg->address;
	data->target_callbacks.write_requested = target_cfg->callbacks->write_requested;
	data->target_callbacks.write_received = target_cfg->callbacks->write_received;
	data->target_callbacks.read_requested = target_cfg->callbacks->read_requested;
	data->target_callbacks.read_processed = target_cfg->callbacks->read_processed;
	data->target_callbacks.stop = target_cfg->callbacks->stop;

	/* Disable the I2C peripheral for configuration changes */
	i2c_target_enable(dev, false);

	/* Disable all I2C target interrupts */
	i2c_target_int_disable(dev, I2C_MCHP_TARGET_INTERRUPT_ALL);

	/* Configure the I2C peripheral in target mode */
	i2c_set_target_mode(dev);

	/* Set the target device unique address */
	i2c_set_target_addr(dev, data->target_config.address);

	/* Enable all I2C target Interrupts */
	i2c_target_int_enable(dev, I2C_MCHP_TARGET_INTERRUPT_ALL);

	/* Mark the device as being in target mode*/
	data->target_mode = true;

	/* Enable runstandby for I2C target */
	i2c_target_runstandby_enable(dev);

	/*Re-enable the I2C peripheral*/
	i2c_target_enable(dev, true);

	/* Release the mutex */
	k_mutex_unlock(&data->i2c_bus_mutex);

	return retval;
}

/**
 * @brief Unregister the device in I2C target mode.
 *
 * This function unregister the I2C peripheral from operate in target mode
 * with the specified target configuration, including address and callback functions.
 *
 * @param dev Pointer to the I2C device structure.
 * @param target_cfg Pointer to the target configuration structure.
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_mchp_target_unregister(const struct device *dev,
				      struct i2c_target_config *target_cfg)
{
	i2c_mchp_dev_data_t *data = dev->data;
	int retval = 0;

	/*Check if the target configuration is NULL*/
	if (target_cfg == NULL) {
		return -EINVAL;
	}
	/*Check if the device is not configured as a target*/
	if (data->target_mode != true) {
		LOG_ERR("device are not configured as target device\n");
		return -EBUSY;
	}
	/*Check if the provided target configuration does not match the current configuration*/
	if (data->target_config.address != target_cfg->address) {
		return -EINVAL;
	}

	k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);

	/*Disable the I2C peripheral*/
	i2c_target_enable(dev, false);

	/* Disable all I2C target interrupts*/
	i2c_target_int_disable(dev, I2C_MCHP_TARGET_INTERRUPT_ALL);

	/*Reset the I2C target device address*/
	i2c_reset_target_addr(dev);

	/*Mark the device as not being in target mode & clear the target configuration*/
	data->target_mode = false;
	data->target_config.address = 0x00;
	data->target_config.callbacks = NULL;

	/*Re-enable the I2C peripheral*/
	i2c_target_enable(dev, true);

	k_mutex_unlock(&data->i2c_bus_mutex);

	return retval;
}
#endif

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
/**
 * @brief Callback function for DMA write completion.
 *
 * This function is called when a DMA write transfer is completed. It checks for errors,
 * finalizes the I2C operation if needed, and re-enables the I2C interrupt to handle the
 * final stages of the transaction.
 *
 * @param dma_dev Pointer to the DMA device.
 * @param arg Pointer to the I2C device structure (passed as argument during DMA configuration).
 * @param id DMA transaction ID (not used here).
 * @param error_code DMA operation error code (0 for success, negative for errors).
 */
static void i2c_dma_write_done(const struct device *dma_dev, void *arg, uint32_t id, int error_code)
{
	const struct device *dev = arg;
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	bool continue_next = false;
	int retval = 0;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);

	/* Lock interrupts to ensure atomic handling of the DMA completion. */
	unsigned int key = irq_lock();

	/* Check if the I2C operation should be terminated due to an error. */
	if (i2c_is_terminate_on_error(dev) != false) {

		/* If termination is required, invoke the callback to notify the upper layer. */
		data->i2c_async_callback(dev, (int)data->current_msg.status, data->user_data);
		irq_unlock(key);
		return;
	}

	/* Check for DMA-specific errors during the transfer. */
	if (error_code < 0) {
		LOG_ERR("DMA write error on %s: %d", dev->name, error_code);
		data->i2c_async_callback(dev, error_code, data->user_data);
		irq_unlock(key);
		return;
	}

	irq_unlock(key);

	/* Check if there are remaining messages in the same direction */
	if (data->num_msgs > 1) {

		/* Check if there are current or next message direction are same */
		if ((data->msgs_array[data->msg_index].flags & I2C_MSG_RW_MASK) ==
		    (data->msgs_array[data->msg_index + 1].flags & I2C_MSG_RW_MASK)) {

			/* Check if there are next message buffer are not restart */
			if ((data->msgs_array[data->msg_index + 1].flags & I2C_MSG_RESTART) == 0) {
				continue_next = true;
			}
		}
	}

	if (continue_next == true) {
		data->msg_index++;
		data->num_msgs--;

		data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
		data->current_msg.size = data->msgs_array[data->msg_index].len;
		data->current_msg.status = I2C_MCHP_CONTROLLER_STATUS_FLAG_NONE;

		/* Configure DMA for write operation. If it fails, exit the function. */
		if (i2c_dma_write_config(dev) != 0) {
			return;
		}

		retval = dma_start(cfg->i2c_dma.dma_dev, cfg->i2c_dma.tx_dma_channel);

		if (retval != 0) {
			data->i2c_async_callback(dev, retval, data->user_data);
			return;
		}

	} else {
		data->current_msg.size = 0;

		/* Re-enable I2C interrupts to handle the final stages of the transmission. */
		i2c_controller_int_enable(dev, I2C_MCHP_CONTROLLER_INTERRUPT_CONTROLLER_ON_BUS);
	}
}

/**
 * @brief Callback function for DMA read completion.
 *
 * This function is triggered when a DMA read transfer is completed. It checks for errors,
 * handles termination conditions, and allows the ISR to manage the final byte of data and
 * the terminating NACK.
 *
 * @param dma_dev Pointer to the DMA device.
 * @param arg Pointer to the I2C device structure (passed as an argument during DMA configuration).
 * @param id DMA transaction ID (not used here).
 * @param error_code DMA operation error code (0 for success, negative for errors).
 */
static void i2c_dma_read_done(const struct device *dma_dev, void *arg, uint32_t id, int error_code)
{
	const struct device *dev = arg;
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;

	bool continue_next = false;
	int retval = 0;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);

	/* Lock interrupts to ensure atomic handling of the DMA completion. */
	unsigned int key = irq_lock();

	/* Check if the I2C operation should be terminated due to an error. */
	if (i2c_is_terminate_on_error(dev) != false) {

		/* If termination is required, invoke the callback to notify the upper layer. */
		data->i2c_async_callback(dev, (int)data->current_msg.status, data->user_data);
		irq_unlock(key);
		return;
	}

	/* Check for DMA-specific errors during the transfer. */
	if (error_code < 0) {
		LOG_ERR("DMA read error on %s: %d", dev->name, error_code);
		data->i2c_async_callback(dev, error_code, data->user_data);
		irq_unlock(key);
		return;
	}

	irq_unlock(key);

	/* Check if there are remaining messages in the same direction */
	if (data->num_msgs > 1) {

		/* Check if there are current or next message direction are same */
		if ((data->msgs_array[data->msg_index].flags & I2C_MSG_RW_MASK) ==
		    (data->msgs_array[data->msg_index + 1].flags & I2C_MSG_RW_MASK)) {

			/* Check if there are next message buffer are not restart */
			if ((data->msgs_array[data->msg_index + 1].flags & I2C_MSG_RESTART) == 0) {
				continue_next = true;
			}
		}
	}

	if (continue_next == true) {
		data->msg_index++;
		data->num_msgs--;

		data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
		data->current_msg.size = data->msgs_array[data->msg_index].len;
		data->current_msg.status = I2C_MCHP_CONTROLLER_STATUS_FLAG_NONE;

		/* Configure DMA for write operation. If it fails, exit the function. */
		if (i2c_dma_read_config(dev) != 0) {
			return;
		}

		retval = dma_start(cfg->i2c_dma.dma_dev, cfg->i2c_dma.rx_dma_channel);

		if (retval != 0) {
			data->i2c_async_callback(dev, retval, data->user_data);
			return;
		}

	} else {

		/*
		 * DMA has read all but the last byte. Let the ISR handle the final byte
		 * and the terminating NACK to properly finish the I2C read operation.
		 */
		data->current_msg.size = 0;

		/* Re-enable I2C interrupts to handle the final stages of the read operation. */
		i2c_controller_int_enable(dev, I2C_MCHP_CONTROLLER_INTERRUPT_TARGET_ON_BUS);
	}
}

/**
 * @brief Configure DMA for I2C write operations.
 *
 * This function sets up the DMA configuration for transferring data from memory
 * to the I2C peripheral during a write operation. It validates the input,
 * initializes DMA configurations, and ensures proper error handling.
 *
 * @param dev Pointer to the I2C device structure.
 * @return true if DMA was successfully configured, false otherwise.
 */
static int i2c_dma_write_config(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;

	/* Initialize DMA configuration structures. */
	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};
	int retval = 0;

	/* Configure DMA transfer direction and data size. */
	dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.dma_callback = i2c_dma_write_done;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->i2c_dma.tx_dma_request;

	/* Set up DMA block configuration for the transfer. */
	dma_blk.block_size = data->current_msg.size;
	dma_blk.source_address = (uint32_t)data->current_msg.buffer;
	dma_blk.dest_address = (uint32_t)(i2c_get_dma_dest_addr(dev));
	dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	/* Configure the DMA with the prepared configurations. */
	retval = dma_config(cfg->i2c_dma.dma_dev, cfg->i2c_dma.tx_dma_channel, &dma_cfg);
	if (retval != 0) {

		/* Log an error message if DMA configuration fails. */
		LOG_ERR("Write DMA configure on %s failed: %d", dev->name, retval);
		data->i2c_async_callback(dev, retval, data->user_data);

		return retval;
	}

	return retval;
}

/**
 * @brief Configure DMA for I2C read operations.
 *
 * This function sets up the DMA configuration for transferring data from the I2C
 * peripheral to memory during a read operation. It validates the conditions for DMA usage,
 * initializes the DMA configuration, and handles errors gracefully.
 *
 * @param dev Pointer to the I2C device structure.
 * @return true if DMA was successfully configured, false otherwise.
 */
static int i2c_dma_read_config(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;

	/* Initialize DMA configuration structures. */
	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};
	int retval = 0;

	/* Configure DMA transfer direction and data size. */
	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.dma_callback = i2c_dma_read_done;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->i2c_dma.rx_dma_request;

	/* Configure the DMA block for the transfer. */
	dma_blk.block_size = data->current_msg.size;
	dma_blk.dest_address = (uint32_t)data->current_msg.buffer;
	dma_blk.source_address = (uint32_t)(i2c_get_dma_source_addr(dev));
	dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	/* Configure the DMA with the prepared configurations. */
	retval = dma_config(cfg->i2c_dma.dma_dev, cfg->i2c_dma.rx_dma_channel, &dma_cfg);
	if (retval != 0) {
		LOG_ERR("Read DMA configure on %s failed: %d", dev->name, retval);
		data->i2c_async_callback(dev, retval, data->user_data);

		return retval;
	}

	return retval;
}
#endif

#ifdef CONFIG_I2C_CALLBACK
/**
 * @brief Perform an I2C transfer with callback notification.
 *
 * This function initiates an I2C transfer, either read or write, using DMA or interrupt
 * mode, and registers a callback to notify upon completion. It handles message validation,
 * DMA configuration, interrupt setup, and status initialization.
 *
 * @param dev Pointer to the I2C device structure.
 * @param msgs Pointer to the array of I2C message structures.
 * @param num_msgs Number of messages in the array.
 * @param addr 7-bit or 10-bit I2C target address.
 * @param cb Callback function to invoke on transfer completion.
 * @param user_data User data to pass to the callback function.
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_mchp_transfer_cb(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				uint16_t addr, i2c_callback_t i2c_async_callback, void *user_data)
{
	i2c_mchp_dev_data_t *data = dev->data;
	uint32_t addr_reg;
	int retval = 0;

	/* Validate that there are messages to process. */
	if (num_msgs == 0) {
		return 0;
	}

	/* Check if the device is currently operating in target mode. */
	if (data->target_mode == true) {
		LOG_ERR("Device currently running in target mode\n");
		return -EBUSY;
	}

	/* Check for empty messages (invalid read/write). */
	for (uint8_t num_buffer = 0; num_buffer < num_msgs; num_buffer++) {
		if ((msgs[num_buffer].len == 0) || (msgs[num_buffer].buf == NULL)) {
			return -EINVAL;
		}
	}
	/* Lock the mutex to ensure exclusive access to the I2C device. */
	k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);

	/* Initialize data fields for the transfer. */
	data->num_msgs = num_msgs;
	data->msgs_array = msgs;
	data->i2c_async_callback = i2c_async_callback;
	data->user_data = user_data;
	data->target_addr = addr;
	data->msg_index = 0;

	/* Disable I2C interrupts, clear interrupt flags, and reset status registers. */
	i2c_controller_int_disable(dev, I2C_MCHP_CONTROLLER_INTERRUPT_ALL);
	i2c_controller_int_flag_clear(dev, I2C_MCHP_CONTROLLER_INTFLAG_ALL);
	i2c_controller_status_clear(dev, I2C_MCHP_CONTROLLER_STATUS_FLAG_ALL);

	/* Initialize message data for the transfer. */
	data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
	data->current_msg.size = data->msgs_array[data->msg_index].len;
	data->current_msg.status = I2C_MCHP_CONTROLLER_STATUS_FLAG_NONE;

	/* Prepare the address register with the 7-bit address and read/write bit. */
	addr_reg = addr << 1U;
	if ((data->msgs_array->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
		addr_reg |= I2C_MESSAGE_DIR_READ; /* Set the read bit for a read transaction. */

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
		/* Configure DMA for the read operation if enabled. */
		if (i2c_dma_read_config(dev) != 0) {
			k_mutex_unlock(&data->i2c_bus_mutex);
			return -EINVAL;
		}
#endif
	} else {
#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN

		/* Configure DMA for the write operation if enabled. */
		if (i2c_dma_write_config(dev) != 0) {
			k_mutex_unlock(&data->i2c_bus_mutex);
			return -EINVAL;
		}
#endif
	}

	/* Write the target address to the I2C address register to start the transaction. */
	i2c_controller_addr_write(dev, addr_reg);

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
	const i2c_mchp_dev_config_t *const cfg = dev->config;

	/* Determine whether to use DMA or interrupt mode for the transfer. */
	if ((data->msgs_array->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {

		/* Start DMA for the read operation. */
		retval = dma_start(cfg->i2c_dma.dma_dev, cfg->i2c_dma.rx_dma_channel);
		if (retval != 0) {
			LOG_ERR("Read DMA start on %s failed: %d", dev->name, retval);
			k_mutex_unlock(&data->i2c_bus_mutex);
			return retval;
		}

	} else {

		/* Start DMA for the write operation. */
		retval = dma_start(cfg->i2c_dma.dma_dev, cfg->i2c_dma.tx_dma_channel);
		if (retval != 0) {
			LOG_ERR("Write DMA start on %s failed: %d", dev->name, retval);
			k_mutex_unlock(&data->i2c_bus_mutex);
			return retval;
		}
	}
#else

	/* Enable I2C interrupts for interrupt-driven write. */
	i2c_controller_int_enable(dev, I2C_MCHP_CONTROLLER_INTERRUPT_ALL);
#endif

	/* Unlock the mutex after completing the setup. */
	k_mutex_unlock(&data->i2c_bus_mutex);

	/* Return the status of the operation. */
	return retval;
}
#endif

#if !defined(CONFIG_I2C_MCHP_INTERRUPT_DRIVEN)
/**
 * @brief Get the NACK status of the I2C controller or target during data transfer.
 *
 * This API checks whether a NACK (Not Acknowledge) condition has occurred during
 * transmit or receive operations in either controller (master) or target (slave) mode.
 * It reads the appropriate status register based on the current I2C mode and returns
 * true if a NACK was detected, or false otherwise.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @return true if a NACK condition is detected, false otherwise.
 */
static bool i2c_is_nack(const struct device *dev)
{
	bool retval;
	sercom_registers_t *i2c_regs = ((const i2c_mchp_dev_config_t *)(dev)->config)->regs;

	if ((i2c_regs->I2CM.SERCOM_CTRLA & SERCOM_I2CM_CTRLA_MODE_I2C_MASTER) ==
	    SERCOM_I2CM_CTRLA_MODE_I2C_MASTER) {
		if ((i2c_regs->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_RXNACK_Msk) ==
		    SERCOM_I2CM_STATUS_RXNACK_Msk) {
			retval = true;
		} else {
			retval = false;
		}
	} else {
		if ((i2c_regs->I2CS.SERCOM_STATUS & SERCOM_I2CS_STATUS_RXNACK_Msk) ==
		    SERCOM_I2CS_STATUS_RXNACK_Msk) {
			retval = true;
		} else {
			retval = false;
		}
	}

	return retval;
}

/**
 * @brief Perform a polled I2C read operation.
 *
 * This function reads bytes from the I2C bus using a polling mechanism.
 * It checks for a NACK to stop the transfer early if necessary and handles
 * byte-by-byte data reception.
 *
 * @param dev Pointer to the I2C device structure.
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_poll_in(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;

	/* Check for a NACK condition. If NACK is received, stop the transfer. */
	if (i2c_is_nack(dev) == true) {
		i2c_controller_transfer_stop(dev);
		return -EIO;
	}

	/* Loop through the message buffer and read each byte from the I2C bus. */
	for (uint32_t i = 0; i < data->current_msg.size; i++) {
		while ((i2c_controller_int_flag_get(dev) &
			I2C_MCHP_CONTROLLER_INTFLAG_TARGET_ON_BUS) !=
		       I2C_MCHP_CONTROLLER_INTFLAG_TARGET_ON_BUS) {
			/* Do nothing */
		}

		/* Stop the I2C transfer when reading the last byte. */
		if (i == data->current_msg.size - 1) {
			i2c_controller_transfer_stop(dev);
		}

		/* Read a byte from the I2C bus and store it in the buffer. */
		data->current_msg.buffer[i] = i2c_byte_read(dev);
	}

	return 0;
}

/**
 * @brief Perform a polled I2C write operation.
 *
 * This function writes bytes to the I2C bus using a polling mechanism.
 * It checks for a NACK after each byte is written and terminates the transfer
 * early if a NACK is detected.
 *
 * @param dev Pointer to the I2C device structure.
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_poll_out(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;

	/* Check for a NACK condition before starting the transfer. */
	if (i2c_is_nack(dev) == true) {
		i2c_controller_transfer_stop(dev);
		return -EIO;
	}

	/* Loop through the message buffer and write each byte to the I2C bus. */
	for (uint32_t i = 0; i < data->current_msg.size; i++) {

		/* Check if there are remaining messages in the same direction */
		while ((i2c_controller_int_flag_get(dev) &
			I2C_MCHP_CONTROLLER_INTFLAG_CONTROLLER_ON_BUS) !=
		       I2C_MCHP_CONTROLLER_INTFLAG_CONTROLLER_ON_BUS) {

			/* Do nothing */
		}

		/* Write a byte to the I2C bus. */
		i2c_byte_write(dev, data->current_msg.buffer[i]);

		/* Check for a NACK condition after writing each byte. */
		if (i2c_is_nack(dev) == true) {
			i2c_controller_transfer_stop(dev);
			return -EIO;
		}
	}

	/* Stop the I2C transfer after all bytes have been written. */
	i2c_controller_transfer_stop(dev);

	return 0;
}
#endif

/**
 * @brief Perform an I2C transfer.
 *
 * Handles reading or writing to the I2C bus with optional interrupt-driven or polled modes.
 * Supports multi-message transfers and checks for potential errors during communication.
 *
 * @param dev Pointer to the I2C device structure.
 * @param msgs Pointer to an array of I2C messages to be processed.
 * @param num_msgs Number of messages in the array.
 * @param addr Target device address on the I2C bus.
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_mchp_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			     uint16_t addr)
{
	i2c_mchp_dev_data_t *data = dev->data;
	uint32_t addr_reg;
	int retval = 0;

	/* Validate input parameters. */
	if (num_msgs == 0) {
		return 0; /* No messages to process. */
	}

	/* Check if the device is currently in target mode. */
	if (data->target_mode == true) {
		LOG_ERR("Device currently configured in target mode\n");
		return -EBUSY;
	}

	/* Check for empty messages (invalid read/write). */
	for (uint8_t num_buffer = 0; num_buffer < num_msgs; num_buffer++) {
		if ((msgs[num_buffer].len == 0) || (msgs[num_buffer].buf == NULL)) {
			return -EINVAL;
		}
	}

	k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);

	/* Disable I2C interrupts, clear interrupt flags, and reset status registers. */
	i2c_controller_int_disable(dev, I2C_MCHP_CONTROLLER_INTERRUPT_ALL);
	i2c_controller_int_flag_clear(dev, I2C_MCHP_CONTROLLER_INTFLAG_ALL);
	i2c_controller_status_clear(dev, I2C_MCHP_CONTROLLER_STATUS_FLAG_ALL);

	/* Set up transfer data. */
	data->num_msgs = num_msgs;
	data->msgs_array = msgs;
	data->msg_index = 0;
	data->target_addr = addr;

	while (data->num_msgs > 0) {

		/* Initialize message buffer and size. */
		data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
		data->current_msg.size = data->msgs_array[data->msg_index].len;
		data->current_msg.status = I2C_MCHP_CONTROLLER_STATUS_FLAG_NONE;

		/* Set up the I2C address register with the target address. */
		addr_reg = addr << 1U;
		if ((data->msgs_array[data->msg_index].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
			addr_reg |= I2C_MESSAGE_DIR_READ; /* Read operation. */
		}

		/* Writing the address starts the I2C transaction. */
		i2c_controller_addr_write(dev, addr_reg);

#ifdef CONFIG_I2C_MCHP_INTERRUPT_DRIVEN
		i2c_controller_int_enable(dev, I2C_MCHP_CONTROLLER_INTERRUPT_ALL);
#else
		/* Process the transfer based on the message direction (read/write). */
		if ((data->msgs_array[data->msg_index].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {

			retval = i2c_poll_in(dev);
			if (retval != 0) {

				/* Unlock the mutex before returning. */
				k_mutex_unlock(&data->i2c_bus_mutex);
				return retval;
			}
		} else {

			retval = i2c_poll_out(dev);
			if (retval != 0) {

				/* Unlock the mutex before returning. */
				k_mutex_unlock(&data->i2c_bus_mutex);
				return retval;
			}
		}
#endif

#ifdef CONFIG_I2C_MCHP_INTERRUPT_DRIVEN

		/* Wait for the interrupt handler to complete the transfer. */
		retval = k_sem_take(&data->i2c_sync_sem, I2C_TRANSFER_TIMEOUT_MSEC);
		if (retval != 0) {
			LOG_ERR("Transfer timeout on %s", dev->name);
			i2c_controller_transfer_stop(dev);

			/* Unlock the mutex before returning. */
			k_mutex_unlock(&data->i2c_bus_mutex);
			return retval;
		}

		/* Check the message status for errors. */
		if (data->current_msg.status != I2C_MCHP_CONTROLLER_STATUS_FLAG_NONE) {
			if ((data->current_msg.status &
			     I2C_MCHP_CONTROLLER_STATUS_FLAG_ARBITRATION_LOST) ==
			    I2C_MCHP_CONTROLLER_STATUS_FLAG_ARBITRATION_LOST) {
				LOG_DBG("Arbitration lost on %s", dev->name);

				/* Unlock the mutex before returning. */
				k_mutex_unlock(&data->i2c_bus_mutex);
				return -EAGAIN;
			}

			LOG_ERR("Transaction error on %s: %08X", dev->name,
				data->current_msg.status);

			/* Unlock the mutex before returning. */
			k_mutex_unlock(&data->i2c_bus_mutex);
			return -EIO;
		}
#endif

		/* Move to the next message in the array. */
		data->num_msgs--;
		data->msg_index++;
	}

	/* Unlock the mutex before returning. */
	k_mutex_unlock(&data->i2c_bus_mutex);

	return retval;
}

/**
 * @brief Recover the I2C bus from a hung or stuck state.
 *
 * This function disables the I2C peripheral, applies default pin configuration,
 * and forces the bus to an idle state to recover from any error conditions.
 *
 * @param dev Pointer to the I2C device structure.
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_mchp_recover_bus(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	int retval;

	/* Lock the mutex to ensure exclusive access to the I2C bus. */
	k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);

	/* Disable the I2C peripheral to prepare for bus recovery. */
	i2c_controller_enable(dev, false);

	/* Disable I2C interrupts to avoid interference during recovery. */
	i2c_controller_int_disable(dev, I2C_MCHP_CONTROLLER_INTERRUPT_ALL);

	/* Apply the default pin configuration state. */
	retval = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		LOG_ERR("Failed to apply default pin state: %d", retval);

		/* Unlock the mutex before returning. */
		k_mutex_unlock(&data->i2c_bus_mutex);
		return retval;
	}

	/* Re-enable the I2C peripheral. */
	i2c_controller_enable(dev, true);

	/* Force the I2C bus to idle state to recover from a bus hang. */
	i2c_set_controller_bus_state_idle(dev);

	/* Unlock the mutex before returning. */
	k_mutex_unlock(&data->i2c_bus_mutex);

	return retval;
}

/**
 * @brief Retrieve the current I2C device configuration.
 *
 * This function returns the current configuration of the I2C device,
 * such as speed, addressing mode, etc.
 *
 * @param dev Pointer to the I2C device structure.
 * @param dev_config Pointer to store the retrieved configuration.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_mchp_get_config(const struct device *dev, uint32_t *dev_config)
{
	i2c_mchp_dev_data_t *data = dev->data;

	/*Check if the device configuration is valid*/
	if (data->dev_config == 0) {
		return -EINVAL;
	}

	/* Retrieve the current device configuration */
	*dev_config = data->dev_config;

	LOG_DBG("Retrieved I2C device configuration: 0x%08X", *dev_config);

	return 0;
}

/**
 * @brief Set and apply the I2C bitrate configuration.
 *
 * This function configures the I2C bitrate based on the provided configuration.
 * It ensures proper synchronization, checks for valid speed modes, and updates the
 * device configuration after successful application.
 *
 * @param dev Pointer to the I2C device structure.
 * @param config The desired I2C configuration, including speed settings.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_set_apply_bitrate(const struct device *dev, uint32_t config)
{
	uint32_t sys_clock_rate = 0;
	uint32_t bitrate;
	int retval = 0;

	/* Determine the bitrate based on the I2C speed configuration */
	switch (I2C_SPEED_GET(config)) {
	case I2C_SPEED_STANDARD:
		bitrate = KHZ(100);
		break;
	case I2C_SPEED_FAST:
		bitrate = KHZ(400);
		break;
	case I2C_SPEED_FAST_PLUS:
		bitrate = MHZ(1);
		break;
#if !defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ2) &&                                           \
	!defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ6)
	case I2C_SPEED_HIGH:
		bitrate = MHZ(3.4);
		break;
#endif
	default:
		LOG_ERR("Unsupported speed code: %d", I2C_SPEED_GET(config));
		retval = -ENOTSUP;
	}

	if (retval == 0) {

		/* Retrieve the clock frequency for baud rate calculation */
		I2C_MCHP_GET_CLOCK_FREQ(dev, sys_clock_rate);

		if (sys_clock_rate == 0) {
			LOG_ERR("Failed to retrieve system clock rate.");
			retval = -EIO;
		}

		if (retval == 0) {

			/*Set the I2C baud rate */
			if (i2c_set_baudrate(dev, bitrate, sys_clock_rate) != true) {
				LOG_ERR("Failed to set I2C baud rate to %u Hz.", bitrate);
				retval = -EIO;
			}
		}
	}

	return retval;
}

/**
 * @brief Configure the I2C interface with the specified settings.
 *
 * This function configures the I2C device based on the provided settings,
 * including the mode (controller/target) and speed. It ensures the configuration
 * is applied safely by disabling the interface before making changes and re-enabling it
 * after.
 *
 * @param dev Pointer to the I2C device structure.
 * @param config Configuration flags specifying mode and speed.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_mchp_configure(const struct device *dev, uint32_t config)
{
	i2c_mchp_dev_data_t *data = dev->data;
	int retval = 0;

	/* Check if the device is currently operating in target mode */
	if (data->target_mode == true) {
		LOG_ERR("Cannot reconfigure while device is in target mode.");
		return -EBUSY;
	}

	/* Lock the mutex to ensure exclusive access to the I2C bus. */
	k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);

	/*Disable the I2C interface to allow configuration changes*/
	i2c_controller_enable(dev, false);

	/*Check if the configuration specifies the I2C controller mode*/
	if ((config & I2C_MODE_CONTROLLER) == I2C_MODE_CONTROLLER) {

		/* Set the I2C to controller mode*/
		i2c_set_controller_mode(dev);
	}

	/* Check and configure I2C speed if specified */
	if (I2C_SPEED_GET(config) != 0) {

		/*Set and apply the bitrate for the I2C interface*/
		retval = i2c_set_apply_bitrate(dev, config);
	}

	if (retval == 0) {

		/*Update the device configuration with the new speed*/
		data->dev_config = I2C_SPEED_GET(config);

		/* Re-enable the I2C interface after configuration */
		i2c_controller_enable(dev, true);

		/* Force the I2C bus state to idle to recover from any potential errors */
		i2c_set_controller_bus_state_idle(dev);
	}

	/* Unlock the mutex before returning. */
	k_mutex_unlock(&data->i2c_bus_mutex);

	return retval;
}

/**
 * @brief Initialize the I2C peripheral.
 *
 * This function performs the initialization of the I2C hardware, including
 * clock configuration, reset, pin control setup, IRQ configuration, and setting
 * the initial I2C mode and speed. It also initializes synchronization primitives
 * (mutex and semaphore) for the driver.
 *
 * @param dev Pointer to the I2C device structure.
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_mchp_init(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;

	int retval;

	/* Enable the GCLK clock for the specified I2C instance */
	I2C_MCHP_ENABLE_CLOCK(dev);

	/* Reset all I2C registers*/
	i2c_swrst(dev);

	/* Initialize mutex and semaphore for I2C data structure */
	k_mutex_init(&data->i2c_bus_mutex);
	k_sem_init(&data->i2c_sync_sem, 0, 1);
	data->target_mode = false;

	/* Apply pin control configuration for SDA and SCL lines */
	retval = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		return retval;
	}

	/* Configure the I2C peripheral with the specified bitrate and mode*/
	retval = i2c_configure(dev, (I2C_MODE_CONTROLLER | i2c_map_dt_bitrate(cfg->bitrate)));
	if (retval != 0) {
		LOG_ERR("Failed to apply pinctrl state. Error: %d", retval);
		return retval;
	}

	/* Disable the I2C peripheral before further configuration */
	i2c_controller_enable(dev, false);

	/*Configure the IRQ for the I2C peripheral*/
	cfg->irq_config_func(dev);

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
	/* Check if the DMA device is ready */
	if (device_is_ready(cfg->i2c_dma.dma_dev) == false) {
		return -ENODEV;
	}
#endif

	/* Enable runstandby for I2C controller */
	i2c_controller_runstandby_enable(dev);

	/*Enable the I2C peripheral*/
	i2c_controller_enable(dev, true);

	/*Force the I2C bus to idle state*/
	i2c_set_controller_bus_state_idle(dev);

	return 0;
}

/******************************************************************************
 * @brief Zephyr driver instance creation
 *****************************************************************************/
static DEVICE_API(i2c, i2c_mchp_api) = {
	.configure = i2c_mchp_configure,
	.get_config = i2c_mchp_get_config,
	.transfer = i2c_mchp_transfer,
#ifdef CONFIG_I2C_MCHP_TARGET
	.target_register = i2c_mchp_target_register,
	.target_unregister = i2c_mchp_target_unregister,
#endif
#ifdef CONFIG_I2C_CALLBACK
	.transfer_cb = i2c_mchp_transfer_cb,
#endif
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
	.recover_bus = i2c_mchp_recover_bus,
};

#define I2C_MCHP_REG_DEFN(n) .regs = (sercom_registers_t *)DT_INST_REG_ADDR(n),

/* Do the peripheral interrupt related configuration */
#if DT_INST_IRQ_HAS_IDX(0, 3)
#define I2C_MCHP_IRQ_HANDLER(n)                                                                    \
	static void i2c_mchp_irq_config_##n(const struct device *dev)                              \
	{                                                                                          \
		I2C_MCHP_IRQ_CONNECT(n, 0);                                                        \
		I2C_MCHP_IRQ_CONNECT(n, 1);                                                        \
		I2C_MCHP_IRQ_CONNECT(n, 2);                                                        \
		I2C_MCHP_IRQ_CONNECT(n, 3);                                                        \
	}
#else
#define I2C_MCHP_IRQ_HANDLER(n)                                                                    \
	static void i2c_mchp_irq_config_##n(const struct device *dev)                              \
	{                                                                                          \
		I2C_MCHP_IRQ_CONNECT(n, 0);                                                        \
	}
#endif

#if defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ2) ||                                            \
	defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ6)
#define I2C_MCHP_CLOCK_DEFN(n)                                                                     \
	.i2c_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                                 \
	.i2c_clock.gclk_sys = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, subsystem)),
#else
#define I2C_MCHP_CLOCK_DEFN(n)                                                                     \
	.i2c_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                                 \
	.i2c_clock.mclk_sys = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, subsystem)),           \
	.i2c_clock.gclk_sys = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, subsystem)),
#endif

#if CONFIG_I2C_MCHP_DMA_DRIVEN
#define I2C_MCHP_DMA_CHANNELS(n)                                                                   \
	.i2c_dma.dma_dev = DEVICE_DT_GET(MCHP_DT_INST_DMA_CTLR(n, tx)),                            \
	.i2c_dma.tx_dma_request = MCHP_DT_INST_DMA_TRIGSRC(n, tx),                                 \
	.i2c_dma.tx_dma_channel = MCHP_DT_INST_DMA_CHANNEL(n, tx),                                 \
	.i2c_dma.rx_dma_request = MCHP_DT_INST_DMA_TRIGSRC(n, rx),                                 \
	.i2c_dma.rx_dma_channel = MCHP_DT_INST_DMA_CHANNEL(n, rx),
#else
#define I2C_MCHP_DMA_CHANNELS(n)
#endif

#define I2C_MCHP_IRQ_CONNECT(n, m)                                                                 \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, m, irq), DT_INST_IRQ_BY_IDX(n, m, priority),     \
			    i2c_mchp_isr, DEVICE_DT_INST_GET(n), 0);                               \
		irq_enable(DT_INST_IRQ_BY_IDX(n, m, irq));                                         \
	} while (false)

#define I2C_MCHP_CONFIG_DEFN(n)                                                                    \
	static const i2c_mchp_dev_config_t i2c_mchp_dev_config_##n = {                             \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.bitrate = DT_INST_PROP(n, clock_frequency),                                       \
		.irq_config_func = &i2c_mchp_irq_config_##n,                                       \
		.run_in_standby = DT_INST_PROP(n, run_in_standby_en),                              \
		I2C_MCHP_REG_DEFN(n) I2C_MCHP_CLOCK_DEFN(n) I2C_MCHP_DMA_CHANNELS(n)}

#define I2C_MCHP_DEVICE_INIT(n)                                                                    \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void i2c_mchp_irq_config_##n(const struct device *dev);                             \
	I2C_MCHP_CONFIG_DEFN(n);                                                                   \
	static i2c_mchp_dev_data_t i2c_mchp_dev_data_##n;                                          \
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_mchp_init, NULL, &i2c_mchp_dev_data_##n,                  \
				  &i2c_mchp_dev_config_##n, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY, \
				  &i2c_mchp_api);                                                  \
	I2C_MCHP_IRQ_HANDLER(n)

DT_INST_FOREACH_STATUS_OKAY(I2C_MCHP_DEVICE_INIT)
