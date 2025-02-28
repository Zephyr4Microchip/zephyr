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

#define INVALID_ADDR 0x00

#if CONFIG_I2C_MCHP_TRANSFER_TIMEOUT
#define I2C_TRANSFER_TIMEOUT_MSEC K_MSEC(CONFIG_I2C_MCHP_TRANSFER_TIMEOUT)
#else
#define I2C_TRANSFER_TIMEOUT_MSEC K_FOREVER
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
	uint32_t status;
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

	uint8_t msg_index;

#ifdef CONFIG_I2C_CALLBACK
	/* Callback function for asynchronous I2C operations. */
	i2c_callback_t i2c_async_callback;

	/* User data passed to the callback function. */
	void *user_data;
#endif

#ifdef CONFIG_I2C_TARGET
	/* Target configuration for I2C target mode. */
	struct i2c_target_config *target_config;
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
static int i2c_mchp_terminate_on_error(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const hal_mchp_i2c_t *i2c = &cfg->hal;

	/* Check for any I2C errors using the HAL function. If an error is found, terminate early.
	 */
	if (hal_mchp_i2c_error_check(i2c) != 0) {
		return 0;
	}

	/* Retrieve and store the current I2C status in the device data structure. */
	data->current_msg.status = hal_mchp_i2c_status_get(i2c);

	/*
	 * Clear all the status flags that require an explicit clear operation.
	 * Some flags are cleared automatically by writing to specific registers (e.g., ADDR
	 * writes).
	 */
	hal_mchp_i2c_status_clear(i2c);

	/* Disable all I2C interrupts to prevent further processing. */
	hal_mchp_i2c_interrupt_disable(i2c);

	/* Stop the I2C transfer explicitly. */
	hal_mchp_i2c_transfer_stop(i2c);

#ifdef CONFIG_I2C_CALLBACK
	/* Callback to the application for async */
	data->i2c_async_callback(dev, data->current_msg.status, data->user_data);
#else
	/* Release the semaphore to signal the completion of the operation. */
	k_sem_give(&data->i2c_sync_sem);
#endif

	/* Return true to indicate successful termination of the operation. */
	return -EIO;
}

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
static int i2c_mchp_dma_write_config(const struct device *dev);
static int i2c_mchp_dma_read_config(const struct device *dev);
#endif

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
	const hal_mchp_i2c_t *i2c = &cfg->hal;
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
	hal_mchp_i2c_addr_write(i2c, addr_reg);

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
	hal_mchp_i2c_interrupt_enable(i2c);
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
	const hal_mchp_i2c_t *i2c = &cfg->hal;
	/* Get the target configuration and callbacks */
	struct i2c_target_config *target_config = data->target_config;
	const struct i2c_target_callbacks *target_cb = data->target_config->callbacks;
	uint8_t rx_tx_data = 0;

	/* Retrieve the interrupt status for the I2C peripheral */
	i2c_mchp_intFlag_t status = hal_mchp_i2c_int_flag_get(i2c);

	/* Check if the interrupt was generated for target mode */
	if ((status & hal_mchp_i2c_target_int_get(i2c)) != 0) {

		/* Handle error conditions */
		if ((status & I2C_MCHP_INTFLAG_ERROR) != 0) {
			LOG_ERR("Interrupt Error generated");
		}

		/* Handle address match interrupt */
		if ((status & I2C_MCHP_INTFLAG_ADDR_MATCH) != 0) {
			/* Address match detected. Perform any additional handling if needed. */
		}

		/* Handle data ready interrupt */
		if ((status & I2C_MCHP_INTFLAG_DATA_READY) != 0) {
			/* Check if the I2C is in write (data received) mode */
			if ((hal_mchp_i2c_status_get(i2c) & I2C_MCHP_INTFLAG_DATA_READY) == 0U) {
				/* Notify that a write request was received */
				target_cb->write_requested(target_config);

				/* Read the received byte */
				rx_tx_data = hal_mchp_i2c_byte_read(i2c);

				/* Notify that the data byte was received */
				target_cb->write_received(target_config, rx_tx_data);
			} else {
				/* Check if the I2C is in read (data transmitted) mode */
				if (hal_mchp_i2c_nack_get(i2c) == 0) {
					/* Notify that a read operation has completed */
					target_cb->read_processed(target_config, &rx_tx_data);

					/* Write the processed data byte */
					hal_mchp_i2c_byte_write(i2c, rx_tx_data);
				} else {
					/* Notify that a read request was received */
					target_cb->read_requested(target_config, &rx_tx_data);

					/* Write the requested data byte */
					hal_mchp_i2c_byte_write(i2c, rx_tx_data);
				}
			}
		}

		/* Handle stop condition interrupt */
		if ((status & I2C_MCHP_INTFLAG_STOP) != 0) {
			/* Notify that a stop condition was received */
			target_cb->stop(target_config);
		}
	}

	/* Clear all interrupt flags */
	hal_mchp_i2c_int_flag_Clear(i2c);
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
	const hal_mchp_i2c_t *i2c = &cfg->hal;
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
	i2c_mchp_intFlag_t status = hal_mchp_i2c_int_flag_get(i2c);

	hal_mchp_i2c_int_flag_Clear(i2c);

	/* Terminate if there are any critical errors on the bus */
	if (i2c_mchp_terminate_on_error(dev) != 0) {
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
	if (status == I2C_MCHP_INTFLAG_CONTROLLER_ON_BUS) {
		/* If no data remains, complete the transfer */
		if (data->current_msg.size == 0) {
			hal_mchp_i2c_transfer_stop(i2c);

			hal_mchp_i2c_interrupt_disable(i2c);

			if (data->num_msgs > 1) {

				data->msg_index++;
				data->num_msgs--;

				data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
				data->current_msg.size = data->msgs_array[data->msg_index].len;
				data->current_msg.status = 0;

				i2c_mchp_restart(dev);
				return;
			} else {
#ifdef CONFIG_I2C_CALLBACK
				/* Callback to the application for async */
				data->i2c_async_callback(dev, data->current_msg.status,
							 data->user_data);
#else
				k_sem_give(&data->i2c_sync_sem);
#endif
				return;
			}
		}
		/* Transmit the next byte from the buffer */
		hal_mchp_i2c_byte_write(i2c, *data->current_msg.buffer);
		data->current_msg.buffer++;
		data->current_msg.size--;
	}
	/* Handle interrupts for controller mode recieve */
	else if (status & I2C_MCHP_INTFLAG_TARGET_ON_BUS) {
		/* Prepare for an auto NACK if this is the last byte */
		if (!continue_next && (data->current_msg.size == 1)) {
			hal_mchp_i2c_transfer_stop(i2c);
		}

		/* Receive the next byte into the buffer */
		*data->current_msg.buffer = hal_mchp_i2c_byte_read(i2c);
		data->current_msg.buffer++;
		data->current_msg.size--;

		/* Complete the transfer if no data remains */
		if ((continue_next == false) && (data->current_msg.size == 0)) {
			hal_mchp_i2c_interrupt_disable(i2c);

			if (data->num_msgs > 1) {

				data->msg_index++;
				data->num_msgs--;

				data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
				data->current_msg.size = data->msgs_array[data->msg_index].len;
				data->current_msg.status = 0;

				i2c_mchp_restart(dev);
				return;
			} else {
#ifdef CONFIG_I2C_CALLBACK
				/* Callback to the application for async */
				data->i2c_async_callback(dev, data->current_msg.status,
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
		data->current_msg.status = 0;
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
	const hal_mchp_i2c_t *i2c = &cfg->hal;
	int retval = 0;

	/* Check if the device is already operating in target mode */
	if (data->target_mode == true) {
		LOG_ERR("Device already registered in target mode.");
		return -EBUSY;
	}

	/* Validate the target configuration and its callbacks */
	if (!target_cfg || !target_cfg->callbacks) {
		return -EINVAL;
	}

	/*Store the target configuration in device data*/
	data->target_config = target_cfg;

	/* Acquire the mutex to ensure thread-safe access */
	k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);

	/* Disable the I2C peripheral for configuration changes */
	hal_mchp_i2c_disable(i2c);

	/* Disable all I2C target interrupts */
	hal_mchp_i2c_target_int_disable(i2c);

	/*Check if the target address is invalid */
	if (data->target_config->address == INVALID_ADDR) {
		LOG_ERR("device can't be register in target mode with 0x00 address\n");
		k_mutex_unlock(&data->i2c_bus_mutex);

		return -EINVAL;
	}

	/* Configure the I2C peripheral in target mode */
	hal_mchp_i2c_target_mode(i2c);

	/* Set the target device unique address */
	hal_mchp_i2c_set_target_addr(i2c, data->target_config->address);

	/* Enable all I2C target Interrupts */
	hal_mchp_i2c_target_int_enable(i2c);

	/* Mark the device as being in target mode*/
	data->target_mode = true;

	/*Re-enable the I2C peripheral*/
	hal_mchp_i2c_enable(i2c);

	/*Force the I2C bus to idle state*/
	hal_mchp_i2c_bus_state_idle(i2c);

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
	const hal_mchp_i2c_t *i2c = &cfg->hal;
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
	if (data->target_config != target_cfg) {
		return -EINVAL;
	}

	k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);

	/*Disable the I2C peripheral*/
	hal_mchp_i2c_disable(i2c);

	/* Disable all I2C target interrupts*/
	hal_mchp_i2c_target_int_disable(i2c);

	/*Reset the I2C target device address*/
	hal_mchp_i2c_reset_addr(i2c);

	/*Mark the device as not being in target mode & clear the target configuration*/
	data->target_mode = false;
	data->target_config = NULL;

	/*Re-enable the I2C peripheral*/
	hal_mchp_i2c_enable(i2c);

	/* Force the I2C bus to idle state */
	hal_mchp_i2c_bus_state_idle(i2c);

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
	const hal_mchp_i2c_t *i2c = &cfg->hal;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);

	/* Lock interrupts to ensure atomic handling of the DMA completion. */
	unsigned int key = irq_lock();

	/* Check if the I2C operation should be terminated due to an error. */
	if (i2c_mchp_terminate_on_error(dev) != 0) {
		/* If termination is required, invoke the callback to notify the upper layer. */
		data->i2c_async_callback(dev, data->current_msg.status, data->user_data);
		irq_unlock(key);
		return;
	}

	/* Check for DMA-specific errors during the transfer. */
	if (error_code < 0) {
		LOG_ERR("DMA write error on %s: %d", dev->name, error_code);
		hal_mchp_i2c_int_flag_Clear(i2c);
		data->i2c_async_callback(dev, error_code, data->user_data);
		irq_unlock(key);
		return;
	}

	irq_unlock(key);

	/*
	 * DMA has completed writing the entire message to the I2C peripheral.
	 * Wait for the final I2C interrupt to confirm the transmission is complete.
	 */
	data->current_msg.size = 0;

	/* Re-enable I2C interrupts to handle the final stages of the transmission. */
	hal_mchp_i2c_interrupt_enable(i2c);
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
	const hal_mchp_i2c_t *i2c = &cfg->hal;
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
	dma_blk.dest_address = (uint32_t)(hal_mchp_i2c_get_dma_dest_addr(i2c));
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
	const hal_mchp_i2c_t *i2c = &cfg->hal;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);
	/* Lock interrupts to ensure atomic handling of the DMA completion. */
	unsigned int key = irq_lock();

	/* Check if the I2C operation should be terminated due to an error. */
	if (i2c_mchp_terminate_on_error(dev) != 0) {
		/* If termination is required, invoke the callback to notify the upper layer. */
		data->i2c_async_callback(dev, data->current_msg.status, data->user_data);
		irq_unlock(key);
		return;
	}

	/* Check for DMA-specific errors during the transfer. */
	if (error_code < 0) {
		LOG_ERR("DMA read error on %s: %d", dev->name, error_code);
		hal_mchp_i2c_int_flag_Clear(i2c);
		data->i2c_async_callback(dev, error_code, data->user_data);
		irq_unlock(key);
		return;
	}

	irq_unlock(key);

	/*
	 * DMA has read all but the last byte. Let the ISR handle the final byte
	 * and the terminating NACK to properly finish the I2C read operation.
	 */
	data->current_msg.buffer += data->current_msg.size - 1;
	data->current_msg.size = 1;

	/* Re-enable I2C interrupts to handle the final stages of the read operation. */
	hal_mchp_i2c_interrupt_enable(i2c);
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
	const hal_mchp_i2c_t *i2c = &cfg->hal;
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
	dma_blk.block_size = data->current_msg.size - 1;
	dma_blk.dest_address = (uint32_t)data->current_msg.buffer;
	dma_blk.source_address = (uint32_t)(hal_mchp_i2c_get_dma_source_addr(i2c));
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
 * This function initiates an I2C transfer, either read or write, using DMA or interrupt mode,
 * and registers a callback to notify upon completion. It handles message validation, DMA
 * configuration, interrupt setup, and status initialization.
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
	const hal_mchp_i2c_t *i2c = &cfg->hal;
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

	/* Lock the mutex to ensure exclusive access to the I2C device. */
	k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);

	/* Initialize data fields for the transfer. */
	data->num_msgs = num_msgs;
	data->msgs_array = msgs;
	data->i2c_async_callback = i2c_async_callback;
	data->user_data = user_data;

	/* Check for empty messages (invalid read/write). */
	if (data->msgs_array->len == 0) {
		k_mutex_unlock(&data->i2c_bus_mutex);
		return -EINVAL;
	}

	/* Disable all I2C interrupts and clear interrupt flags. */
	hal_mchp_i2c_interrupt_disable(i2c);
	hal_mchp_i2c_int_flag_Clear(i2c);

	/* Clear the I2C status register. */
	hal_mchp_i2c_status_clear(i2c);

	/* Initialize message data for the transfer. */
	data->current_msg.buffer = data->msgs_array->buf;
	data->current_msg.size = data->msgs_array->len;
	data->current_msg.status = 0;
	data->msg_index = 0;

	/* Prepare the address register with the 7-bit address and read/write bit. */
	addr_reg = addr << 1U;
	if ((data->msgs_array->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
		addr_reg |= 1U; /* Set the read bit for a read transaction. */

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
	hal_mchp_i2c_addr_write(i2c, addr_reg);

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
	hal_mchp_i2c_interrupt_enable(i2c);
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
	const hal_mchp_i2c_t *i2c = &cfg->hal;

	/* Check for a NACK condition. If NACK is received, stop the transfer. */
	if (hal_mchp_i2c_nack_get(i2c) == 0) {
		hal_mchp_i2c_transfer_stop(i2c);
		return -EIO;
	}

	/* Loop through the message buffer and read each byte from the I2C bus. */
	for (uint32_t i = 0; i < data->current_msg.size; i++) {
		while (hal_mchp_i2c_int_flag_get(i2c) != I2C_MCHP_INTFLAG_TARGET_ON_BUS) {
			/* code */
		}
		/* Stop the I2C transfer when reading the last byte. */
		if (i == data->current_msg.size - 1) {
			hal_mchp_i2c_transfer_stop(i2c);
		}

		/* Read a byte from the I2C bus and store it in the buffer. */
		data->current_msg.buffer[i] = hal_mchp_i2c_byte_read(i2c);
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
	const hal_mchp_i2c_t *i2c = &cfg->hal;

	/* Check for a NACK condition before starting the transfer. */
	if (hal_mchp_i2c_nack_get(i2c) == 0) {
		hal_mchp_i2c_transfer_stop(i2c);
		return -EIO;
	}

	/* Loop through the message buffer and write each byte to the I2C bus. */
	for (uint32_t i = 0; i < data->current_msg.size; i++) {
		/* Check if there are remaining messages in the same direction */
		while (hal_mchp_i2c_int_flag_get(i2c) != I2C_MCHP_INTFLAG_CONTROLLER_ON_BUS) {
			/* code */
		}
		/* Write a byte to the I2C bus. */
		hal_mchp_i2c_byte_write(i2c, *data->current_msg.buffer);

		/* Check for a NACK condition after writing each byte. */
		if (hal_mchp_i2c_nack_get(i2c) == 0) {
			hal_mchp_i2c_transfer_stop(i2c);
			return -EIO;
		}
	}

	/* Stop the I2C transfer after all bytes have been written. */
	hal_mchp_i2c_transfer_stop(i2c);

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
	const hal_mchp_i2c_t *i2c = &cfg->hal;
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
	hal_mchp_i2c_interrupt_disable(i2c);
	hal_mchp_i2c_int_flag_Clear(i2c);
	hal_mchp_i2c_status_clear(i2c);

	/* Set up transfer data. */
	data->num_msgs = num_msgs;
	data->msgs_array = msgs;
	data->msg_index = 0;
	data->target_addr = addr;

	while (data->num_msgs > 0) {
		/* Check for empty messages (invalid read/write). */
		if (data->msgs_array[data->msg_index].len == 0) {
			/* Unlock the mutex before returning. */
			k_mutex_unlock(&data->i2c_bus_mutex);
			return -EINVAL;
		}

		/* Initialize message buffer and size. */
		data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
		data->current_msg.size = data->msgs_array[data->msg_index].len;
		data->current_msg.status = 0;

		/* Set up the I2C address register with the target address. */
		addr_reg = addr << 1U;
		if ((data->msgs_array[data->msg_index].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
			addr_reg |= 1U; /* Read operation. */
		}

		/* Writing the address starts the I2C transaction. */
		hal_mchp_i2c_addr_write(i2c, addr_reg);

#ifdef CONFIG_I2C_MCHP_INTERRUPT_DRIVEN
		hal_mchp_i2c_interrupt_enable(i2c);
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
			hal_mchp_i2c_transfer_stop(i2c);
			/* Unlock the mutex before returning. */
			k_mutex_unlock(&data->i2c_bus_mutex);
			return retval;
		}

		/* Check the message status for errors. */
		if (data->current_msg.status != 0) {
			if (data->current_msg.status & hal_mchp_i2c_status_get(i2c)) {
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
	const hal_mchp_i2c_t *i2c = &cfg->hal;
	int retval;

	/* Lock the mutex to ensure exclusive access to the I2C bus. */
	k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);

	/* Disable the I2C peripheral to prepare for bus recovery. */
	hal_mchp_i2c_disable(i2c);

	/* Disable I2C interrupts to avoid interference during recovery. */
	hal_mchp_i2c_interrupt_disable(i2c);

	/* Apply the default pin configuration state. */
	retval = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		LOG_ERR("Failed to apply default pin state: %d", retval);
		/* Unlock the mutex before returning. */
		k_mutex_unlock(&data->i2c_bus_mutex);
		return retval;
	}

	/* Re-enable the I2C peripheral. */
	hal_mchp_i2c_enable(i2c);

	/* Force the I2C bus to idle state to recover from a bus hang. */
	hal_mchp_i2c_bus_state_idle(i2c);

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

	unsigned int key = irq_lock();
	/* Retrieve the current device configuration */
	*dev_config = data->dev_config;

	irq_unlock(key);
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
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const hal_mchp_i2c_t *i2c = &cfg->hal;
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
		return -ENOTSUP;
	}

	/* Disable interrupts to avoid race conditions during configuration */
	unsigned int key = irq_lock();

	/* Retrieve the clock frequency for baud rate calculation */
	I2C_MCHP_GET_CLOCK_FREQ(dev, sys_clock_rate);

	if (sys_clock_rate == 0) {
		LOG_ERR("Failed to retrieve system clock rate.");
		irq_unlock(key);
		return -EIO;
	}
	/*Set the I2C baud rate */
	if (hal_mchp_i2c_set_baudrate(i2c, bitrate, sys_clock_rate) != 0) {
		LOG_ERR("Failed to set I2C baud rate to %u Hz.", bitrate);
		irq_unlock(key);
		return -EIO;
	}
	/*Update the device configuration with the new speed*/
	data->dev_config = config;

	/* Unlock interrupts and the mutex */
	irq_unlock(key);

	return retval;
}

/**
 * @brief Configure the I2C interface with the specified settings.
 *
 * This function configures the I2C device based on the provided settings,
 * including the mode (controller/target) and speed. It ensures the configuration
 * is applied safely by disabling the interface before making changes and re-enabling it after.
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
	const hal_mchp_i2c_t *i2c = &cfg->hal;
	int retval = 0;

	/* Check if the device is currently operating in target mode */
	if (data->target_mode == true) {
		LOG_ERR("Cannot reconfigure while device is in target mode.");
		return -EBUSY;
	}
	/*Disable the I2C interface to allow configuration changes*/
	hal_mchp_i2c_disable(i2c);

	/*Check if the configuration specifies the I2C controller mode*/
	if (config & I2C_MODE_CONTROLLER) {
		/* Set the I2C to controller mode*/
		if (hal_mchp_i2c_controller_mode(i2c) != 0) {
			LOG_ERR("Failed to set I2C to controller mode.");
			return -EIO;
		}
	}

	/* Check and configure I2C speed if specified */
	if (config & I2C_SPEED_MASK) {
		/*Set and apply the bitrate for the I2C interface*/
		retval = i2c_mchp_set_apply_bitrate(dev, config);

		if (retval != 0) {
			return retval;
		}
	}
	/* Re-enable the I2C interface after configuration */
	hal_mchp_i2c_enable(i2c);

	/* Force the I2C bus state to idle to recover from any potential errors */
	hal_mchp_i2c_bus_state_idle(i2c);

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
	const hal_mchp_i2c_t *i2c = &cfg->hal;
	int retval;

	/* Enable the GCLK clock for the specified I2C instance */
	I2C_MCHP_ENABLE_CLOCK(dev);

	/* Reset all I2C registers*/
	hal_mchp_i2c_swrst(i2c);

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
	hal_mchp_i2c_disable(i2c);

	/*Configure the IRQ for the I2C peripheral*/
	cfg->irq_config_func(dev);

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
	/* Check if the DMA device is ready */
	if (device_is_ready(cfg->i2c_dma.dma_dev) == false) {
		return -ENODEV;
	}
#endif
	/*Enable the I2C peripheral*/
	hal_mchp_i2c_enable(i2c);

	/*Force the I2C bus to idle state*/
	hal_mchp_i2c_bus_state_idle(i2c);

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
