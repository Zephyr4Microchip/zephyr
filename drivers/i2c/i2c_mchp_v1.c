/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pinctrl.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(i2c_mchp_v1, CONFIG_I2C_LOG_LEVEL);

#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include "i2c-priv.h"
#include "i2c_mchp_v1.h"

#define INVALID_ADDR     0x00
#define MESSAGE_DIR_READ 1U

#if CONFIG_I2C_MCHP_TRANSFER_TIMEOUT
#define I2C_TRANSFER_TIMEOUT_MSEC K_MSEC(CONFIG_I2C_MCHP_TRANSFER_TIMEOUT)
#else
#define I2C_TRANSFER_TIMEOUT_MSEC K_FOREVER
#endif

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
static int i2c_mchp_dma_write_config(const struct device *dev);
static int i2c_mchp_dma_read_config(const struct device *dev);
#endif

/* This structure contains the DMA configuration parameters for the I2C */

typedef struct mchp_i2c_dma {
	/* DMA driver for asynchronous operations */
	const struct device *dma_dev;
	/* TX DMA request line. */
	uint8_t tx_dma_request;
	/* TX DMA channel. */
	uint8_t tx_dma_channel;
	/* RX DMA request line. */
	uint8_t rx_dma_request;
	/* RX DMA channel. */
	uint8_t rx_dma_channel;
} mchp_i2c_dma_t;

/* Configuration structure for the MCHP I2C driver */
typedef struct i2c_mchp_dev_config {
	/* Hardware abstraction layer for the I2C peripheral. */
	hal_mchp_i2c_t hal;

	/* Clock configuration */
	mchp_i2c_clock_t i2c_clock;

	/* Pin configuration for SDA and SCL lines. */
	const struct pinctrl_dev_config *pcfg;

	/* Default bitrate for I2C communication (e.g., 100 kHz, 400 kHz). */
	uint32_t bitrate;

	/* Function to configure the IRQ during initialization. */
	void (*irq_config_func)(const struct device *dev);
#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
	mchp_i2c_dma_t i2c_dma;
#endif

} i2c_mchp_dev_config_t;

/* Structure representing an I2C message for the MCHP I2C peripheral */
struct i2c_mchp_msg {
	/* Pointer to the data buffer for the I2C message. */
	uint8_t *buffer;

	/* Size of the I2C message in bytes. */
	uint32_t size;

	/* Status of the I2C message, indicating success or error conditions. */
	i2c_mchp_controller_status_flag_t status;
};

/* Structure representing runtime data for the MCHP I2C driver */
typedef struct i2c_mchp_dev_data {

	/* Mutex for protecting access to the I2C driver data. */
	struct k_mutex i2c_bus_mutex;

	/* Semaphore for signaling completion of I2C transfers. */
	struct k_sem i2c_sync_sem;

	/* Structure representing the current I2C message. */
	struct i2c_mchp_msg current_msg;

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

	/* Variable to track message buffer by indexing*/
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
#endif
} i2c_mchp_dev_data_t;

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
static bool i2c_mchp_is_terminate_on_error(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const hal_mchp_i2c_t *i2c_hal = &cfg->hal;
	bool retval = true;
	/* Retrieve and store the current I2C status in the device data structure. */
	data->current_msg.status = hal_mchp_i2c_controller_status_get(i2c_hal);
	/* Check for any I2C errors using the HAL function. If an error is found, terminate early.
	 */
	if (data->current_msg.status == I2C_MCHP_CONTROLLER_STATUS_FLAG_NONE) {
		retval = false;
	} else {

		/*
		 * Clear all the status flags that require an explicit clear operation.
		 * Some flags are cleared automatically by writing to specific registers (e.g., ADDR
		 * writes).
		 */
		hal_mchp_i2c_controller_status_clear(i2c_hal, data->current_msg.status);

		/* Disable all I2C interrupts to prevent further processing. */
		hal_mchp_i2c_controller_int_disable(i2c_hal, I2C_MCHP_CONTROLLER_INTERRUPT_ALL);

		/* Stop the I2C transfer explicitly. */
		hal_mchp_i2c_controller_transfer_stop(i2c_hal);

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
static void i2c_mchp_restart(const struct device *dev)
{
	/* Retrieve device data and configuration. */
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const hal_mchp_i2c_t *i2c_hal = &cfg->hal;
	/* Prepare the address register (left-shift address by 1 for R/W bit). */
	uint32_t addr_reg = data->target_addr << 1U;

	/* Check if the operation is a read. */
	if ((data->msgs_array[data->msg_index].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
		addr_reg |= 1U; /* Set the R/W bit to indicate a read operation. */

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
		/* Configure DMA for read operation. If it fails, exit the function. */
		if (i2c_mchp_dma_read_config(dev) != 0) {
			return;
		}
#endif

	} else { /* Write operation */
#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
		/* Configure DMA for write operation. If it fails, exit the function. */
		if (i2c_mchp_dma_write_config(dev) != 0) {
			return;
		}
#endif
	}

	/*
	 * Writing the target address to the I2C hardware starts the transaction.
	 * This will issue a START or a repeated START as required.
	 */
	hal_mchp_i2c_controller_addr_write(i2c_hal, addr_reg);

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
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
	hal_mchp_i2c_controller_int_enable(i2c_hal, I2C_MCHP_CONTROLLER_INTERRUPT_ALL);
#endif

	return;
}

#ifdef CONFIG_I2C_TARGET
/**
 * @brief Handle target mode interrupts for the I2C peripheral.
 *
 * This function processes I2C target-specific interrupt events such as
 * address match, data ready, and stop condition. It calls the appropriate
 * callbacks based on the interrupt type.
 *
 * @param dev Pointer to the I2C device structure.
 */
static void i2c_mchp_target_handler(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const hal_mchp_i2c_t *i2c_hal = &cfg->hal;
	/* Get the target configuration and callbacks */
	struct i2c_target_config *target_config = &data->target_config;
	const struct i2c_target_callbacks *target_cb = data->target_config.callbacks;
	static uint8_t rx_tx_data = 0;

	/* Retrieve the interrupt status for the I2C peripheral */
	i2c_mchp_target_intflag_t int_status = hal_mchp_i2c_target_int_flag_get(i2c_hal);

	/* Clear all interrupt flags */
	hal_mchp_i2c_target_int_flag_clear(i2c_hal, int_status);

	/*Get the current status of target device */
	i2c_mchp_target_status_flag_t target_status = hal_mchp_i2c_target_status_get(i2c_hal);

	/* Handle error conditions */
	if ((int_status & I2C_MCHP_TARGET_INTFLAG_ERROR) == I2C_MCHP_TARGET_INTFLAG_ERROR) {
		LOG_ERR("Interrupt Error generated");
		target_cb->stop(target_config);
		return;
	}

	/* Handle address match */
	if ((int_status & I2C_MCHP_TARGET_INTFLAG_ADDR_MATCH) == I2C_MCHP_TARGET_INTFLAG_ADDR_MATCH) {
		if ((target_status & I2C_MCHP_TARGET_STATUS_FLAG_DATA_DIR_READ) == I2C_MCHP_TARGET_STATUS_FLAG_DATA_DIR_READ) {
			rx_tx_data = 0;
			/* Master is reading */
			target_cb->read_requested(target_config, &rx_tx_data);
			// hal_mchp_i2c_byte_write(i2c_hal, rx_tx_data);
		} else {
			/* Master is writing */
			target_cb->write_requested(target_config);
		}
	}

	/* Handle data ready (Read/Write Operations) */
	if ((int_status & I2C_MCHP_TARGET_INTFLAG_DATA_READY) == I2C_MCHP_TARGET_INTFLAG_DATA_READY) {
		if ((target_status & I2C_MCHP_TARGET_STATUS_FLAG_DATA_DIR_READ) == I2C_MCHP_TARGET_STATUS_FLAG_DATA_DIR_READ){
			/* Master is reading */
			hal_mchp_i2c_byte_write(i2c_hal, rx_tx_data);

			target_cb->read_processed(target_config, &rx_tx_data);
		} else {
			/* Master is writing */
			rx_tx_data = hal_mchp_i2c_byte_read(i2c_hal);
			target_cb->write_received(target_config, rx_tx_data);
		}
	}

	/* Handle stop condition interrupt */
	if ((int_status & I2C_MCHP_TARGET_INTFLAG_STOP) == I2C_MCHP_TARGET_INTFLAG_STOP) {
		/* Notify that a stop condition was received */
		target_cb->stop(target_config);
	}
}

#endif

/**
 * @brief ISR (Interrupt Service Routine) for I2C operations.
 *
 * This function handles I2C interrupt events for both controller and target modes.
 * It manages message transmission, reception, and error conditions.
 *
 * @param dev Pointer to the I2C device structure.
 */
static void i2c_mchp_isr(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const hal_mchp_i2c_t *i2c_hal = &cfg->hal;
	bool continue_next = false;

#ifdef CONFIG_I2C_TARGET
	/* Check if the device is operating in target mode */
	if (data->target_mode == true) {
		/* Delegate target-specific operations to a dedicated handler */
		i2c_mchp_target_handler(dev);
		return;
	}
#endif

	/* Get current interrupt status to identify the cause of the interrupt */
	i2c_mchp_controller_intflag_t int_status = hal_mchp_i2c_controller_int_flag_get(i2c_hal);

	hal_mchp_i2c_target_int_flag_clear(i2c_hal, int_status);

	/* Terminate if there are any critical errors on the bus */
	if (i2c_mchp_is_terminate_on_error(dev) != false) {
		return;
	}

	/* Handle ERROR interrupt flag for controller mode transmit and recieve */
	if (int_status == I2C_MCHP_CONTROLLER_INTFLAG_ERROR) {
		hal_mchp_i2c_controller_transfer_stop(i2c_hal);
		/*Disable the all i2c interrupts*/
		hal_mchp_i2c_controller_int_disable(i2c_hal, I2C_MCHP_CONTROLLER_INTERRUPT_ALL);

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
			hal_mchp_i2c_controller_transfer_stop(i2c_hal);

			hal_mchp_i2c_controller_int_disable(
				i2c_hal, I2C_MCHP_CONTROLLER_INTFLAG_CONTROLLER_ON_BUS);

			if (data->num_msgs > 1) {

				data->msg_index++;
				data->num_msgs--;

				data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
				data->current_msg.size = data->msgs_array[data->msg_index].len;
				data->current_msg.status = I2C_MCHP_CONTROLLER_STATUS_FLAG_NONE;

				i2c_mchp_restart(dev);
				return;
			} else {
#ifdef CONFIG_I2C_CALLBACK
				/* Callback to the application for async */
				data->i2c_async_callback(dev, (int)data->current_msg.status,
							 data->user_data);
#else
				k_sem_give(&data->i2c_sync_sem);
#endif
				return;
			}
		}
		/* Transmit the next byte from the buffer */
		hal_mchp_i2c_byte_write(i2c_hal, *data->current_msg.buffer);
		data->current_msg.buffer++;
		data->current_msg.size--;
	}
	/* Handle interrupts for controller mode recieve */
	else if (int_status == I2C_MCHP_CONTROLLER_INTFLAG_TARGET_ON_BUS) {
		/* Prepare for an auto NACK if this is the last byte */
		if (!continue_next && (data->current_msg.size == 1)) {
			hal_mchp_i2c_controller_transfer_stop(i2c_hal);
		}
#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
		/*If transaction in DMA mode then no need to read data just send NACK if num
		 * messages are 1 or if nume messages greater then 1 then restart the communication
		 */
#else
		/* Receive the next byte into the buffer */
		*data->current_msg.buffer = hal_mchp_i2c_byte_read(i2c_hal);
		data->current_msg.buffer++;
		data->current_msg.size--;
#endif
		/* Complete the transfer if no data remains */
		if ((continue_next == false) && (data->current_msg.size == 0)) {
			hal_mchp_i2c_controller_int_disable(
				i2c_hal, I2C_MCHP_CONTROLLER_INTFLAG_TARGET_ON_BUS);

			if (data->num_msgs > 1) {

				data->msg_index++;
				data->num_msgs--;

				data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
				data->current_msg.size = data->msgs_array[data->msg_index].len;
				data->current_msg.status = I2C_MCHP_CONTROLLER_STATUS_FLAG_NONE;

				i2c_mchp_restart(dev);
				return;
			} else {
#ifdef CONFIG_I2C_CALLBACK
				/* Callback to the application for async */
				data->i2c_async_callback(dev, (int)data->current_msg.status,
							 data->user_data);
#else
				k_sem_give(&data->i2c_sync_sem);
#endif
				return;
			}
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
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const hal_mchp_i2c_t *i2c_hal = &cfg->hal;
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

	/*Store the target configuration in device data*/
	data->target_config.address = target_cfg->address;
	data->target_config.callbacks = target_cfg->callbacks;

	/*Check if the target address is invalid */
	if (data->target_config.address == INVALID_ADDR) {
		LOG_ERR("device can't be register in target mode with 0x00 address\n");
		return -EINVAL;
	}

	/* Acquire the mutex to ensure thread-safe access */
	k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);

	/* Disable the I2C peripheral for configuration changes */
	hal_mchp_i2c_target_enable(i2c_hal, false);

	/* Disable all I2C target interrupts */
	hal_mchp_i2c_target_int_disable(i2c_hal, I2C_MCHP_TARGET_INTERRUPT_ALL);

	/* Configure the I2C peripheral in target mode */
	hal_mchp_i2c_set_target_mode(i2c_hal);

	/* Set the target device unique address */
	hal_mchp_i2c_set_target_addr(i2c_hal, data->target_config.address);

	/* Enable all I2C target Interrupts */
	hal_mchp_i2c_target_int_enable(i2c_hal, I2C_MCHP_TARGET_INTERRUPT_ALL);

	/* Mark the device as being in target mode*/
	data->target_mode = true;

	/*Re-enable the I2C peripheral*/
	hal_mchp_i2c_target_enable(i2c_hal, true);

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
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const hal_mchp_i2c_t *i2c_hal = &cfg->hal;
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
	hal_mchp_i2c_target_enable(i2c_hal, false);

	/* Disable all I2C target interrupts*/
	hal_mchp_i2c_target_int_disable(i2c_hal, I2C_MCHP_TARGET_INTERRUPT_ALL);

	/*Reset the I2C target device address*/
	hal_mchp_i2c_reset_target_addr(i2c_hal);

	/*Mark the device as not being in target mode & clear the target configuration*/
	data->target_mode = false;
	data->target_config.address = 0x00;
	data->target_config.callbacks = NULL;

	/*Re-enable the I2C peripheral*/
	hal_mchp_i2c_target_enable(i2c_hal, true);

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
static void i2c_mchp_dma_write_done(const struct device *dma_dev, void *arg, uint32_t id,
				    int error_code)
{
	const struct device *dev = arg;
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const hal_mchp_i2c_t *i2c_hal = &cfg->hal;
	bool continue_next = false;
	int retval = 0;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);

	/* Lock interrupts to ensure atomic handling of the DMA completion. */
	unsigned int key = irq_lock();
	/* Check if the I2C operation should be terminated due to an error. */
	if (i2c_mchp_is_terminate_on_error(dev) != false) {
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
		if (i2c_mchp_dma_write_config(dev) != 0) {
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
		hal_mchp_i2c_controller_int_enable(i2c_hal,
						   I2C_MCHP_CONTROLLER_INTERRUPT_CONTROLLER_ON_BUS);
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
static int i2c_mchp_dma_write_config(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const hal_mchp_i2c_t *i2c_hal = &cfg->hal;
	/* Initialize DMA configuration structures. */
	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};
	int retval = 0;

	/* Configure DMA transfer direction and data size. */
	dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.dma_callback = i2c_mchp_dma_write_done;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->i2c_dma.tx_dma_request;

	/* Set up DMA block configuration for the transfer. */
	dma_blk.block_size = data->current_msg.size;
	dma_blk.source_address = (uint32_t)data->current_msg.buffer;
	dma_blk.dest_address = (uint32_t)(hal_mchp_i2c_get_dma_dest_addr(i2c_hal));
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
static void i2c_mchp_dma_read_done(const struct device *dma_dev, void *arg, uint32_t id,
				   int error_code)
{
	const struct device *dev = arg;
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const hal_mchp_i2c_t *i2c_hal = &cfg->hal;
	bool continue_next = false;
	int retval = 0;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);
	/* Lock interrupts to ensure atomic handling of the DMA completion. */
	unsigned int key = irq_lock();
	/* Check if the I2C operation should be terminated due to an error. */
	if (i2c_mchp_is_terminate_on_error(dev) != false) {
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
		if (i2c_mchp_dma_read_config(dev) != 0) {
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
		hal_mchp_i2c_controller_int_enable(i2c_hal,
						   I2C_MCHP_CONTROLLER_INTERRUPT_TARGET_ON_BUS);
	}
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
static int i2c_mchp_dma_read_config(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const hal_mchp_i2c_t *i2c_hal = &cfg->hal;
	/* Initialize DMA configuration structures. */
	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};
	int retval = 0;

	/* Configure DMA transfer direction and data size. */
	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.dma_callback = i2c_mchp_dma_read_done;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->i2c_dma.rx_dma_request;

	/* Configure the DMA block for the transfer. */
	dma_blk.block_size = data->current_msg.size;
	dma_blk.dest_address = (uint32_t)data->current_msg.buffer;
	dma_blk.source_address = (uint32_t)(hal_mchp_i2c_get_dma_source_addr(i2c_hal));
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
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const hal_mchp_i2c_t *i2c_hal = &cfg->hal;
	data->msg_index = 0;
	uint32_t addr_reg;
	int retval = 0;

	/* Set the target address for the transfer. */
	data->target_addr = addr;

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
	if ((data->msgs_array[data->msg_index].len == 0) ||
	    (data->msgs_array[data->msg_index].buf == NULL)) {
		return -EINVAL;
	}
	/* Lock the mutex to ensure exclusive access to the I2C device. */
	k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);

	/* Initialize data fields for the transfer. */
	data->num_msgs = num_msgs;
	data->msgs_array = msgs;
	data->i2c_async_callback = i2c_async_callback;
	data->user_data = user_data;

	/* Disable I2C interrupts, clear interrupt flags, and reset status registers. */
	hal_mchp_i2c_controller_int_disable(i2c_hal, I2C_MCHP_CONTROLLER_INTERRUPT_ALL);
	hal_mchp_i2c_controller_int_flag_clear(i2c_hal, I2C_MCHP_CONTROLLER_INTFLAG_ALL);
	hal_mchp_i2c_controller_status_clear(i2c_hal, I2C_MCHP_CONTROLLER_STATUS_FLAG_ALL);

	/* Initialize message data for the transfer. */
	data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
	data->current_msg.size = data->msgs_array[data->msg_index].len;
	data->current_msg.status = I2C_MCHP_CONTROLLER_STATUS_FLAG_NONE;

	/* Prepare the address register with the 7-bit address and read/write bit. */
	addr_reg = addr << 1U;
	if ((data->msgs_array->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
		addr_reg |= MESSAGE_DIR_READ; /* Set the read bit for a read transaction. */

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
		/* Configure DMA for the read operation if enabled. */
		if (i2c_mchp_dma_read_config(dev) != 0) {
			k_mutex_unlock(&data->i2c_bus_mutex);
			return -EINVAL;
		}
#endif
	} else {
#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
		/* Configure DMA for the write operation if enabled. */
		if (i2c_mchp_dma_write_config(dev) != 0) {
			k_mutex_unlock(&data->i2c_bus_mutex);
			return -EINVAL;
		}
#endif
	}

	/* Write the target address to the I2C address register to start the transaction. */
	hal_mchp_i2c_controller_addr_write(i2c_hal, addr_reg);

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
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
	hal_mchp_i2c_controller_int_enable(i2c_hal, I2C_MCHP_CONTROLLER_INTERRUPT_ALL);
#endif

	/* Unlock the mutex after completing the setup. */
	k_mutex_unlock(&data->i2c_bus_mutex);

	/* Return the status of the operation. */
	return retval;
}

#endif

#if !defined(CONFIG_I2C_MCHP_INTERRUPT_DRIVEN)

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
static int i2c_mchp_poll_in(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const hal_mchp_i2c_t *i2c_hal = &cfg->hal;

	/* Check for a NACK condition. If NACK is received, stop the transfer. */
	if (hal_mchp_i2c_is_nack(i2c_hal) == true) {
		hal_mchp_i2c_controller_transfer_stop(i2c_hal);
		return -EIO;
	}

	/* Loop through the message buffer and read each byte from the I2C bus. */
	for (uint32_t i = 0; i < data->current_msg.size; i++) {
		while ((hal_mchp_i2c_controller_int_flag_get(i2c_hal) &
			I2C_MCHP_CONTROLLER_INTFLAG_TARGET_ON_BUS) !=
		       I2C_MCHP_CONTROLLER_INTFLAG_TARGET_ON_BUS) {
			/* code */
		}
		/* Stop the I2C transfer when reading the last byte. */
		if (i == data->current_msg.size - 1) {
			hal_mchp_i2c_controller_transfer_stop(i2c_hal);
		}

		/* Read a byte from the I2C bus and store it in the buffer. */
		data->current_msg.buffer[i] = hal_mchp_i2c_byte_read(i2c_hal);
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
static int i2c_mchp_poll_out(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const hal_mchp_i2c_t *i2c_hal = &cfg->hal;

	/* Check for a NACK condition before starting the transfer. */
	if (hal_mchp_i2c_is_nack(i2c_hal) == true) {
		hal_mchp_i2c_controller_transfer_stop(i2c_hal);
		return -EIO;
	}

	/* Loop through the message buffer and write each byte to the I2C bus. */
	for (uint32_t i = 0; i < data->current_msg.size; i++) {
		/* Check if there are remaining messages in the same direction */
		while ((hal_mchp_i2c_controller_int_flag_get(i2c_hal) &
			I2C_MCHP_CONTROLLER_INTFLAG_CONTROLLER_ON_BUS) !=
		       I2C_MCHP_CONTROLLER_INTFLAG_CONTROLLER_ON_BUS) {
			/* code */
		}
		/* Write a byte to the I2C bus. */
		hal_mchp_i2c_byte_write(i2c_hal, data->current_msg.buffer[i]);

		/* Check for a NACK condition after writing each byte. */
		if (hal_mchp_i2c_is_nack(i2c_hal) == true) {
			hal_mchp_i2c_controller_transfer_stop(i2c_hal);
			return -EIO;
		}
	}

	/* Stop the I2C transfer after all bytes have been written. */
	hal_mchp_i2c_controller_transfer_stop(i2c_hal);
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
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const hal_mchp_i2c_t *i2c_hal = &cfg->hal;
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

	k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);

	/* Disable I2C interrupts, clear interrupt flags, and reset status registers. */
	hal_mchp_i2c_controller_int_disable(i2c_hal, I2C_MCHP_CONTROLLER_INTERRUPT_ALL);
	hal_mchp_i2c_controller_int_flag_clear(i2c_hal, I2C_MCHP_CONTROLLER_INTFLAG_ALL);
	hal_mchp_i2c_controller_status_clear(i2c_hal, I2C_MCHP_CONTROLLER_STATUS_FLAG_ALL);

	/* Set up transfer data. */
	data->num_msgs = num_msgs;
	data->msgs_array = msgs;
	data->msg_index = 0;
	data->target_addr = addr;

	while (data->num_msgs > 0) {
		/* Check for empty messages (invalid read/write) and buffer not NULL. */
		if ((data->msgs_array[data->msg_index].len == 0) ||
		    (data->msgs_array[data->msg_index].buf == NULL)) {
			/* Unlock the mutex before returning. */
			k_mutex_unlock(&data->i2c_bus_mutex);
			return -EINVAL;
		}

		/* Initialize message buffer and size. */
		data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
		data->current_msg.size = data->msgs_array[data->msg_index].len;
		data->current_msg.status = I2C_MCHP_CONTROLLER_STATUS_FLAG_NONE;

		/* Set up the I2C address register with the target address. */
		addr_reg = addr << 1U;
		if ((data->msgs_array[data->msg_index].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
			addr_reg |= MESSAGE_DIR_READ; /* Read operation. */
		}

		/* Writing the address starts the I2C transaction. */
		hal_mchp_i2c_controller_addr_write(i2c_hal, addr_reg);

#ifdef CONFIG_I2C_MCHP_INTERRUPT_DRIVEN
		hal_mchp_i2c_controller_int_enable(i2c_hal, I2C_MCHP_CONTROLLER_INTERRUPT_ALL);
#else
		/* Process the transfer based on the message direction (read/write). */
		if ((data->msgs_array[data->msg_index].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {

			retval = i2c_mchp_poll_in(dev);
			if (retval != 0) {
				/* Unlock the mutex before returning. */
				k_mutex_unlock(&data->i2c_bus_mutex);
				return retval;
			}
		} else {

			retval = i2c_mchp_poll_out(dev);
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
			hal_mchp_i2c_controller_transfer_stop(i2c_hal);
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
	const hal_mchp_i2c_t *i2c_hal = &cfg->hal;
	int retval;

	/* Lock the mutex to ensure exclusive access to the I2C bus. */
	k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);

	/* Disable the I2C peripheral to prepare for bus recovery. */
	hal_mchp_i2c_controller_enable(i2c_hal, false);

	/* Disable I2C interrupts to avoid interference during recovery. */
	hal_mchp_i2c_controller_int_disable(i2c_hal, I2C_MCHP_CONTROLLER_INTERRUPT_ALL);

	/* Apply the default pin configuration state. */
	retval = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		LOG_ERR("Failed to apply default pin state: %d", retval);
		/* Unlock the mutex before returning. */
		k_mutex_unlock(&data->i2c_bus_mutex);
		return retval;
	}

	/* Re-enable the I2C peripheral. */
	hal_mchp_i2c_controller_enable(i2c_hal, true);

	/* Force the I2C bus to idle state to recover from a bus hang. */
	hal_mchp_i2c_set_controller_bus_state_idle(i2c_hal);

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
static int i2c_mchp_set_apply_bitrate(const struct device *dev, uint32_t config)
{
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const hal_mchp_i2c_t *i2c_hal = &cfg->hal;
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
	case I2C_SPEED_HIGH:
		bitrate = MHZ(3.4);
		break;
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
			if (hal_mchp_i2c_set_baudrate(i2c_hal, bitrate, sys_clock_rate) != true) {
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
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const hal_mchp_i2c_t *i2c_hal = &cfg->hal;
	int retval = 0;

	/* Check if the device is currently operating in target mode */
	if (data->target_mode == true) {
		LOG_ERR("Cannot reconfigure while device is in target mode.");
		return -EBUSY;
	}

	/* Lock the mutex to ensure exclusive access to the I2C bus. */
	k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);

	/*Disable the I2C interface to allow configuration changes*/
	hal_mchp_i2c_controller_enable(i2c_hal, false);

	/*Check if the configuration specifies the I2C controller mode*/
	if ((config & I2C_MODE_CONTROLLER) == I2C_MODE_CONTROLLER) {
		/* Set the I2C to controller mode*/
		hal_mchp_i2c_set_controller_mode(i2c_hal);
	}

	/* Check and configure I2C speed if specified */
	if (I2C_SPEED_GET(config) != 0) {
		/*Set and apply the bitrate for the I2C interface*/
		retval = i2c_mchp_set_apply_bitrate(dev, config);
	}

	if (retval == 0) {
		/*Update the device configuration with the new speed*/
		data->dev_config = I2C_SPEED_GET(config);
		/* Re-enable the I2C interface after configuration */
		hal_mchp_i2c_controller_enable(i2c_hal, true);

		/* Force the I2C bus state to idle to recover from any potential errors */
		hal_mchp_i2c_set_controller_bus_state_idle(i2c_hal);
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
	const hal_mchp_i2c_t *i2c_hal = &cfg->hal;
	int retval;

	/* Enable the GCLK clock for the specified I2C instance */
	I2C_MCHP_ENABLE_CLOCK(dev);

	/* Reset all I2C registers*/
	hal_mchp_i2c_swrst(i2c_hal);

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
	retval = i2c_mchp_configure(dev, (I2C_MODE_CONTROLLER | i2c_map_dt_bitrate(cfg->bitrate)));
	if (retval != 0) {
		LOG_ERR("Failed to apply pinctrl state. Error: %d", retval);
		return retval;
	}

	/* Disable the I2C peripheral before further configuration */
	hal_mchp_i2c_controller_enable(i2c_hal, false);

	/*Configure the IRQ for the I2C peripheral*/
	cfg->irq_config_func(dev);

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
	/* Check if the DMA device is ready */
	if (device_is_ready(cfg->i2c_dma.dma_dev) == false) {
		return -ENODEV;
	}
#endif
	/*Enable the I2C peripheral*/
	hal_mchp_i2c_controller_enable(i2c_hal, true);

	/*Force the I2C bus to idle state*/
	hal_mchp_i2c_set_controller_bus_state_idle(i2c_hal);

	return 0;
}

static const struct i2c_driver_api i2c_mchp_driver_api = {
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
		I2C_MCHP_HAL_DEFN(n) I2C_MCHP_CLOCK_DEFN(n) I2C_MCHP_DMA_CHANNELS(n)}

#define I2C_MCHP_DEVICE_INIT(n)                                                                    \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void i2c_mchp_irq_config_##n(const struct device *dev);                             \
	I2C_MCHP_CONFIG_DEFN(n);                                                                   \
	static i2c_mchp_dev_data_t i2c_mchp_dev_data_##n;                                          \
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_mchp_init, NULL, &i2c_mchp_dev_data_##n,                  \
				  &i2c_mchp_dev_config_##n, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY, \
				  &i2c_mchp_driver_api);                                           \
	I2C_MCHP_IRQ_HANDLER(n)

DT_INST_FOREACH_STATUS_OKAY(I2C_MCHP_DEVICE_INIT)
