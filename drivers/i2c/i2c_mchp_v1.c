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

#include <zephyr/drivers/clock_control/clock_control_mchp_v1.h>
#include "i2c-priv.h"
#include "i2c_mchp_v1.h"

#if CONFIG_I2C_MCHP_TRANSFER_TIMEOUT
#define I2C_TRANSFER_TIMEOUT_MSEC K_MSEC(CONFIG_I2C_MCHP_TRANSFER_TIMEOUT)
#else
#define I2C_TRANSFER_TIMEOUT_MSEC K_FOREVER
#endif

/* Configuration structure for the MCHP I2C driver */
typedef struct i2c_mchp_dev_config {
	/* Hardware abstraction layer for the I2C peripheral. */
	struct hal_mchp_i2c hal;

	/* Pin configuration for SDA and SCL lines. */
	const struct pinctrl_dev_config *pcfg;

	/* Default bitrate for I2C communication (e.g., 100 kHz, 400 kHz). */
	uint32_t bitrate;

	/* Clock device supplying the clock signal for the I2C peripheral. */
	const struct device *clock_dev;

	/* Function to configure the IRQ during initialization. */
	void (*irq_config_func)(const struct device *dev);
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
	/* Semaphore for synchronizing I2C operations. */
	struct k_sem lock;

	/* Mutex for protecting access to the I2C driver data. */
	struct k_mutex mutex;

	/* Semaphore for signaling completion of I2C transfers. */
	struct k_sem sem;

	/* Structure representing the current I2C message. */
	struct i2c_mchp_msg msg;

	/* Pointer to an array of I2C messages being transferred. */
	struct i2c_msg *msgs;

	/* Number of messages in the current I2C transfer sequence. */
	uint8_t num_msgs;

	/* Flag indicating whether the device is in target (slave) mode. */
	bool target_mode;

	/* Current I2C device configuration settings. */
	uint32_t dev_config;

	/* Address of the I2C target device. */
	uint32_t target_addr;

#ifdef CONFIG_I2C_CALLBACK
	/* Callback function for asynchronous I2C operations. */
	i2c_callback_t cb;

	/* User data passed to the callback function. */
	void *user_data;
#endif

#ifdef CONFIG_I2C_TARGET
	/* Target configuration for I2C target (slave) mode. */
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
static bool i2c_mchp_terminate_on_error(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const struct hal_mchp_i2c *i2c = &cfg->hal;

	/* Check for any I2C errors using the HAL function. If an error is found, terminate early.
	 */
	if (hal_mchp_i2c_error_check(i2c)) {
		return false;
	}

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
	/* If DMA is enabled and a valid DMA channel are not configured, stop the DMA transfer. */
	if (cfg->hal.tx_dma_channel != 0xFF) {
		dma_stop(cfg->hal.dma_dev, cfg->hal.tx_dma_channel);
	}

	if (cfg->hal.rx_dma_channel != 0xFF) {
		dma_stop(cfg->hal.dma_dev, cfg->hal.rx_dma_channel);
	}
#endif

	/* Retrieve and store the current I2C status in the device data structure. */
	data->msg.status = hal_mchp_i2c_status_get(i2c);

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

	/* Release the semaphore to signal the completion of the operation. */
	k_sem_give(&data->sem);

	/* Return true to indicate successful termination of the operation. */
	return true;
}

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
static bool i2c_mchp_dma_write_config(const struct device *dev);
static bool i2c_mchp_dma_read_config(const struct device *dev);
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
	const struct hal_mchp_i2c *i2c = &cfg->hal;

	/* Prepare the address register (left-shift address by 1 for R/W bit). */
	uint32_t addr_reg = data->target_addr << 1U;

	/* Check if the operation is a read. */
	if ((data->msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
		addr_reg |= 1U; /* Set the R/W bit to indicate a read operation. */

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
		/* Configure DMA for read operation. If it fails, exit the function. */
		if (!i2c_mchp_dma_read_config(dev)) {
			return;
		}
#endif

	} else { /* Write operation */
#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
		/* Configure DMA for write operation. If it fails, exit the function. */
		if (!i2c_mchp_dma_write_config(dev)) {
			return;
		}
#endif
	}

	/* Lock interrupts to ensure atomic operation during transaction setup. */
	unsigned int key = irq_lock();

	/*
	 * Writing the target address to the I2C hardware starts the transaction.
	 * This will issue a START or a repeated START as required.
	 */
	hal_mchp_i2c_addr_write(i2c, addr_reg);

	/* Handle read or write transaction setup. */
	if ((data->msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
		/* Start the DMA read operation and handle any errors. */
		int retval = dma_start(cfg->hal.dma_dev, cfg->hal.rx_dma_channel);
		if (retval != 0) {
			LOG_ERR("Read DMA start on %s failed: %d", dev->name, retval);
			/* Ensure interrupts are re-enabled before returning. */
			irq_unlock(key);
			return;
		}
#else
		/* Enable I2C interrupts for non-DMA read operation. */
		hal_mchp_i2c_interrupt_enable(i2c);
#endif

	} else { /* Write operation */
#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
		/* Start the DMA write operation and handle any errors. */
		int retval = dma_start(cfg->hal.dma_dev, cfg->hal.tx_dma_channel);
		if (retval != 0) {
			LOG_ERR("Write DMA start on %s failed: %d", dev->name, retval);
			/* Ensure interrupts are re-enabled before returning. */
			irq_unlock(key);
			return;
		}
#else
		/* Enable I2C interrupts for non-DMA write operation. */
		hal_mchp_i2c_interrupt_enable(i2c);
#endif
	}

	/* Unlock interrupts after the transaction setup is complete. */
	irq_unlock(key);
}

#ifdef CONFIG_I2C_TARGET
/**
 * @brief Handle target (slave) mode interrupts for the I2C peripheral.
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
	const struct hal_mchp_i2c *i2c = &cfg->hal;
	/* Get the target configuration and callbacks */
	struct i2c_target_config *target_config = data->target_config;
	const struct i2c_target_callbacks *target_cb = data->target_config->callbacks;

	/* Retrieve the interrupt status for the I2C peripheral */
	uint32_t status = hal_mchp_i2c_int_flag_get(i2c);
	uint8_t data = 0;

	/* Check if the interrupt was generated for target mode */
	if ((status & hal_mchp_i2c_target_int_get(i2c)) != 0) {

		/* Handle error conditions */
		if ((status & error) != 0) {
			LOG_ERR("Interrupt Error generated");
		}

		/* Handle address match interrupt */
		if ((status & addr_match) != 0) {
			/* Address match detected. Perform any additional handling if needed. */
		}

		/* Handle data ready interrupt */
		if ((status & data_ready) != 0) {
			/* Check if the I2C is in write (data received) mode */
			if ((hal_mchp_i2c_status_get(i2c) & data_ready) == 0U) {
				/* Notify that a write request was received */
				target_cb->write_requested(target_config);

				/* Wait for synchronization */
				hal_mchp_i2c_wait_sync(i2c);

				/* Read the received byte */
				data = hal_mchp_i2c_byte_read(i2c);

				/* Notify that the data byte was received */
				target_cb->write_received(target_config, data);
			} else {
				/* Check if the I2C is in read (data transmitted) mode */
				if (hal_mchp_i2c_status_get(i2c) & hal_mchp_i2c_nack_get(i2c)) {
					/* Notify that a read operation has completed */
					target_cb->read_processed(target_config, &data);

					/* Write the processed data byte */
					hal_mchp_i2c_byte_write(i2c, (uint8_t)data);
				} else {
					/* Notify that a read request was received */
					target_cb->read_requested(target_config, &data);

					/* Write the requested data byte */
					hal_mchp_i2c_byte_write(i2c, (uint8_t)data);
				}
			}
		}

		/* Handle stop condition interrupt */
		if ((status & stop) != 0) {
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
 * This function handles I2C interrupt events for both master and target (slave) modes.
 * It manages message transmission, reception, and error conditions.
 *
 * @param dev Pointer to the I2C device structure.
 */
static void i2c_mchp_isr(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const struct hal_mchp_i2c *i2c = &cfg->hal;

#ifdef CONFIG_I2C_CALLBACK
	/* Retrieve the registered callback function, if available */
	i2c_callback_t cb = data->cb;
#endif

#ifdef CONFIG_I2C_TARGET
	/* Check if the device is operating in target (slave) mode */
	if (data->target_mode) {
		/* Delegate target-specific operations to a dedicated handler */
		i2c_mchp_target_handler(dev);
		return;
	}
#endif

	/* Get current interrupt status to identify the cause of the interrupt */
	uint32_t status = hal_mchp_i2c_int_flag_get(i2c);

	/* Terminate if there are any critical errors on the bus */
	if (i2c_mchp_terminate_on_error(dev)) {
		return;
	}

	/*
	 * Check whether the num_msgs > 1 then RESTART the communication.
	 */
	const bool continue_next = (data->msg.size == 1) && (data->num_msgs > 1);

	/* Handle interrupts for master mode operations */
	if (status & master_on_bus) {
		/* If no data remains, complete the transfer */
		if (!data->msg.size) {
			hal_mchp_i2c_interrupt_disable(i2c);

			if (data->num_msgs <= 1) {
				hal_mchp_i2c_transfer_stop(i2c);
#ifdef CONFIG_I2C_CALLBACK
				cb(dev, data->msg.status, data->user_data);
#endif
			}
#if !defined(CONFIG_I2C_CALLBACK)
			k_sem_give(&data->sem);
#endif
			return;
		}

		/* Transmit the next byte from the buffer */
		hal_mchp_i2c_byte_write(i2c, *data->msg.buffer);
		data->msg.buffer++;
		data->msg.size--;
	}
	/* Handle interrupts for target (slave) mode operations */
	else if (status & target_on_bus) {
		/* Prepare for an auto NACK if this is the last byte */
		if (!continue_next && (data->msg.size == 1)) {
			hal_mchp_i2c_transfer_stop(i2c);
		}

		/* Receive the next byte into the buffer */
		*data->msg.buffer = hal_mchp_i2c_byte_read(i2c);
		data->msg.buffer++;
		data->msg.size--;

		/* Complete the transfer if no data remains */
		if (!continue_next && !data->msg.size) {
			hal_mchp_i2c_interrupt_disable(i2c);
#ifdef CONFIG_I2C_CALLBACK
			cb(dev, data->msg.status, data->user_data);
#else
			k_sem_give(&data->sem);
#endif
			return;
		}
	}

	/* Clear all interrupt flags */
	hal_mchp_i2c_int_flag_Clear(i2c);

	/* Handle the continuation of messages in the same direction */
	if (continue_next) {
		uint8_t dir = data->msgs->flags;

		/* Move to the next message in the queue */
		data->msgs++;
		data->num_msgs--;

		data->msg.buffer = data->msgs->buf;
		data->msg.size = data->msgs->len;
		data->msg.status = 0;

		/* Check if the direction of the current and next messages is the same */
		if (dir == data->msgs->flags) {
#ifdef CONFIG_I2C_CALLBACK
			cb(dev, data->msg.status, data->user_data);
#endif
			/* Continue transmission or reception */
		} else {
			/* If the direction changes, stop the current transfer and restart */
			if (status & master_on_bus) {
				hal_mchp_i2c_transfer_stop(i2c);
#ifdef CONFIG_I2C_CALLBACK
				cb(dev, data->msg.status, data->user_data);
#endif
				hal_mchp_i2c_interrupt_disable(i2c);
			} else if (status & target_on_bus) {
				hal_mchp_i2c_transfer_stop(i2c);
#ifdef CONFIG_I2C_CALLBACK
				cb(dev, data->msg.status, data->user_data);
#endif
				hal_mchp_i2c_interrupt_disable(i2c);
			}

			/* Restart the transfer for the next message */
			i2c_mchp_restart(dev);
		}
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
	const struct hal_mchp_i2c *i2c = &cfg->hal;
	/*Store the target configuration in device data*/
	data->target_config = target_cfg;
	int retval = 0;
	/* Validate the target configuration and its callbacks */
	if (!target_cfg || !target_cfg->callbacks) {
		return -EINVAL;
	}
	/* Check if the device is already operating in target mode */
	if (data->target_mode) {
		LOG_ERR("Device already registered in target mode.");
		return -EBUSY;
	}

	/* Acquire the mutex to ensure thread-safe access */
	k_mutex_lock(&data->mutex, K_FOREVER);

	/* Disable the I2C peripheral for configuration changes */
	hal_mchp_i2c_disable(i2c);

	/* Disable all I2C target interrupts */
	hal_mchp_i2c_target_int_disable(i2c);

	/*Check if the target address is 0x00, which is invalid*/
	if (data->target_config->address == 0x00) {
		LOG_ERR("device can't be register in target mode with 0x00 address\n");
		k_mutex_unlock(&data->mutex);

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
	k_mutex_unlock(&data->mutex);

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
	const struct hal_mchp_i2c *i2c = &cfg->hal;
	int retval = 0;

	/*Check if the target configuration is NULL*/
	if (!target_cfg) {
		return -EINVAL;
	}
	/*Check if the device is not configured as a target*/
	if (!data->target_mode) {
		LOG_ERR("device are not configured as target device\n");
		return -EBUSY;
	}
	/*Check if the provided target configuration does not match the current configuration*/
	if (data->target_config != target_cfg) {
		return -EINVAL;
	}

	k_mutex_lock(&data->mutex, K_FOREVER);

	/*Disable the I2C peripheral*/
	hal_mchp_i2c_disable(i2c);

	/* Disable all I2C target interrupts*/
	hal_mchp_i2c_target_int_disable(i2c);

	/*Reset the I2C target device address*/
	hal_mchp_i2c_reset_addr(i2c)

		/*Mark the device as not being in target mode & clear the target configuration*/
		data->target_mode = false;
	data->target_config = NULL;

	/*Re-enable the I2C peripheral*/
	hal_mchp_i2c_enable(i2c);

	/* Force the I2C bus to idle state */
	hal_mchp_i2c_bus_state_idle(i2c);

	k_mutex_unlock(&data->mutex);

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
	const struct hal_mchp_i2c *i2c = &cfg->hal;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);

	/* Lock interrupts to ensure atomic handling of the DMA completion. */
	unsigned int key = irq_lock();

	/* Check if the I2C operation should be terminated due to an error. */
	if (i2c_mchp_terminate_on_error(dev)) {
		/* If termination is required, invoke the callback to notify the upper layer. */
		data->cb(dev, data->msg.status, data->user_data);
		irq_unlock(key);
		return;
	}

	/* Check for DMA-specific errors during the transfer. */
	if (error_code < 0) {
		LOG_ERR("DMA write error on %s: %d", dev->name, error_code);
		hal_mchp_i2c_int_flag_Clear(i2c);
		irq_unlock(key);

		/* Signal the semaphore to indicate the end of the operation. */
		k_sem_give(&data->sem);
		return;
	}

	irq_unlock(key);

	/*
	 * DMA has completed writing the entire message to the I2C peripheral.
	 * Wait for the final I2C interrupt to confirm the transmission is complete.
	 */
	data->msg.size = 0;
	printk("dma write callback hitted\n");
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
static bool i2c_mchp_dma_write_config(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const struct hal_mchp_i2c *i2c = &cfg->hal;
	int retval;

	/* Return false if the DMA channel is invalid (not configured). */
	if (cfg->hal.tx_dma_channel == 0xFF) {
		return false;
	}

	/* Avoid using DMA for single-byte transfers or empty writes. */
	if (data->msg.size <= 1) {
		/*
		 * Single-byte transfers are more efficiently handled without DMA.
		 * DMA setup overhead for small transfers is not justified.
		 */
		return false;
	}

	/* Initialize DMA configuration structures. */
	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};

	/* Configure DMA transfer direction and data size. */
	dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.dma_callback = i2c_mchp_dma_write_done;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->hal.write_dma_request;

	/* Set up DMA block configuration for the transfer. */
	dma_blk.block_size = data->msg.size;
	dma_blk.source_address = (uint32_t)data->msg.buffer;
	dma_blk.dest_address = (uint32_t)I2C_MCHP_DMA_DEST_ADDR;
	dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	/* Configure the DMA with the prepared configurations. */
	retval = dma_config(cfg->hal.dma_dev, cfg->hal.tx_dma_channel, &dma_cfg);
	if (retval != 0) {
		/* Log an error message if DMA configuration fails. */
		LOG_ERR("Write DMA configure on %s failed: %d", dev->name, retval);
		return false;
	}

	return true;
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
	const struct hal_mchp_i2c *i2c = &cfg->hal;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);

	/* Lock interrupts to ensure atomic handling of the DMA completion. */
	unsigned int key = irq_lock();

	/* Check if the I2C operation should be terminated due to an error. */
	if (i2c_mchp_terminate_on_error(dev)) {
		/* If termination is required, invoke the callback to notify the upper layer. */
		data->cb(dev, data->msg.status, data->user_data);
		irq_unlock(key);
		return;
	}

	/* Check for DMA-specific errors during the transfer. */
	if (error_code < 0) {
		LOG_ERR("DMA read error on %s: %d", dev->name, error_code);
		hal_mchp_i2c_int_flag_Clear(i2c);
		irq_unlock(key);

		/* Signal the semaphore to indicate the end of the operation. */
		k_sem_give(&data->sem);
		return;
	}

	irq_unlock(key);

	/*
	 * DMA has read all but the last byte. Let the ISR handle the final byte
	 * and the terminating NACK to properly finish the I2C read operation.
	 */
	data->msg.buffer += data->msg.size - 1;
	data->msg.size = 1;
	printk("dma read callback hitted\n");
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
static bool i2c_mchp_dma_read_config(const struct device *dev)
{
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const struct hal_mchp_i2c *i2c = &cfg->hal;
	int retval;

	/* Check if the DMA channel is invalid (not configured). */
	if (cfg->hal.rx_dma_channel == 0xFF) {
		return false;
	}

	/* Avoid using DMA for small reads (2 or fewer bytes). */
	if (data->msg.size <= 2) {
		/*
		 * For reads of 2 bytes or less, the final byte is always
		 * handled by the I2C ISR, making DMA unnecessary.
		 */
		return false;
	}

	/* Initialize DMA configuration structures. */
	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};

	/* Configure DMA transfer direction and data size. */
	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.dma_callback = i2c_mchp_dma_read_done;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->hal.read_dma_request;

	/* Configure the DMA block for the transfer. */
	dma_blk.block_size = data->msg.size - 1;
	dma_blk.dest_address = (uint32_t)data->msg.buffer;
	dma_blk.source_address = (uint32_t)I2C_MCHP_DMA_SRC_ADDR;
	dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	/* Configure the DMA with the prepared configurations. */
	retval = dma_config(cfg->hal.dma_dev, cfg->hal.rx_dma_channel, &dma_cfg);
	if (retval != 0) {
		LOG_ERR("Read DMA configure on %s failed: %d", dev->name, retval);
		return false;
	}

	return true;
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
				uint16_t addr, i2c_callback_t cb, void *user_data)
{
	i2c_mchp_dev_data_t *data = dev->data;
	const i2c_mchp_dev_config_t *const cfg = dev->config;
	const struct hal_mchp_i2c *i2c = &cfg->hal;
	uint32_t addr_reg;
	int retval = 0;

	/* Set the target address for the transfer. */
	data->target_addr = addr;

	/* Validate that there are messages to process. */
	if (!num_msgs) {
		return 0;
	}

	/* Check if the device is currently operating in target mode. */
	if (data->target_mode) {
		LOG_ERR("Device currently running in target mode\n");
		return -EBUSY;
	}

	/* Lock the mutex to ensure exclusive access to the I2C device. */
	k_mutex_lock(&data->mutex, K_FOREVER);

	/* Initialize data fields for the transfer. */
	data->num_msgs = num_msgs;
	data->msgs = msgs;
	data->cb = cb;
	data->user_data = user_data;

	/* Validate the first message's length. */
	if (!data->msgs->len) {
		if ((data->msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
			k_mutex_unlock(&data->mutex);
			return -EINVAL;
		}
	}

	/* Disable all I2C interrupts and clear interrupt flags. */
	hal_mchp_i2c_interrupt_disable(i2c);
	hal_mchp_i2c_int_flag_Clear(i2c);

	/* Clear the I2C status register. */
	hal_mchp_i2c_status_clear(i2c);

	/* Initialize message data for the transfer. */
	data->msg.buffer = data->msgs->buf;
	data->msg.size = data->msgs->len;
	data->msg.status = 0;

	/* Prepare the address register with the 7-bit address and read/write bit. */
	addr_reg = addr << 1U;
	if ((data->msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
		addr_reg |= 1U; /* Set the read bit for a read transaction. */

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
		/* Configure DMA for the read operation if enabled. */
		if (!i2c_mchp_dma_read_config(dev)) {
			k_mutex_unlock(&data->mutex);
			return -EINVAL;
		}
#endif
	} else {
#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
		/* Configure DMA for the write operation if enabled. */
		if (!i2c_mchp_dma_write_config(dev)) {
			k_mutex_unlock(&data->mutex);
			return -EINVAL;
		}
#endif
	}

	/* Lock interrupts to safely initiate the transfer. */
	unsigned int key = irq_lock();

	/* Write the target address to the I2C address register to start the transaction. */
	hal_mchp_i2c_addr_write(i2c, addr_reg);

	/* Determine whether to use DMA or interrupt mode for the transfer. */
	if ((data->msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
		/* Start DMA for the read operation. */
		retval = dma_start(cfg->hal.dma_dev, cfg->hal.rx_dma_channel);
		if (retval != 0) {
			LOG_ERR("Read DMA start on %s failed: %d", dev->name, retval);
			irq_unlock(key);
			k_mutex_unlock(&data->mutex);
			return retval;
		}
#else
		/* Enable I2C interrupts for interrupt-driven read. */
		hal_mchp_i2c_interrupt_enable(i2c);
#endif
	} else {
#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
		/* Start DMA for the write operation. */
		retval = dma_start(cfg->hal.dma_dev, cfg->hal.tx_dma_channel);
		if (retval != 0) {
			LOG_ERR("Write DMA start on %s failed: %d", dev->name, retval);
			irq_unlock(key);
			k_mutex_unlock(&data->mutex);
			return retval;
		}
#else
		/* Enable I2C interrupts for interrupt-driven write. */
		hal_mchp_i2c_interrupt_enable(i2c);
#endif
	}

	/* Unlock interrupts after setting up the transfer. */
	irq_unlock(key);

	/* Unlock the mutex after completing the setup. */
	k_mutex_unlock(&data->mutex);

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
	const struct hal_mchp_i2c *i2c = &cfg->hal;

	/* Check for a NACK condition. If NACK is received, stop the transfer. */
	if (hal_mchp_i2c_nack_get(i2c)) {
		hal_mchp_i2c_transfer_stop(i2c);
		return -EIO;
	}

	/* Loop through the message buffer and read each byte from the I2C bus. */
	for (uint8_t i = 0; i < data->msg.size; i++) {
		/* Stop the I2C transfer when reading the last byte. */
		if (i == data->msg.size - 1) {
			hal_mchp_i2c_transfer_stop(i2c);
		}

		/* Read a byte from the I2C bus and store it in the buffer. */
		data->msg.buffer[i] = hal_mchp_i2c_byte_read(i2c);

		/* Introduce a small delay to allow the I2C hardware to settle. */
		k_msleep(20);
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
	const struct hal_mchp_i2c *i2c = &cfg->hal;

	/* Check for a NACK condition before starting the transfer. */
	if (hal_mchp_i2c_nack_get(i2c)) {
		hal_mchp_i2c_transfer_stop(i2c);
		return -EIO;
	}

	/* Loop through the message buffer and write each byte to the I2C bus. */
	for (uint8_t i = 0; i < data->msg.size; i++) {
		/* Write a byte to the I2C bus. */
		hal_mchp_i2c_byte_write(i2c, *data->msg.buffer);

		/* Check for a NACK condition after writing each byte. */
		if (hal_mchp_i2c_nack_get(i2c)) {
			hal_mchp_i2c_transfer_stop(i2c);
			return -EIO;
		}

		/* Move to the next byte in the buffer. */
		data->msg.buffer++;
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
	const struct hal_mchp_i2c *i2c = &cfg->hal;
	uint32_t addr_reg;
	int retval = 0;

	/* Validate input parameters. */
	if (!num_msgs) {
		return 0; /* No messages to process. */
	}

	/* Check if the device is currently in target mode. */
	if (data->target_mode) {
		LOG_ERR("Device currently configured in target mode\n");
		return -EBUSY;
	}

	k_mutex_lock(&data->mutex, K_FOREVER);

	/* Disable I2C interrupts, clear interrupt flags, and reset status registers. */
	hal_mchp_i2c_interrupt_disable(i2c);
	hal_mchp_i2c_int_flag_Clear(i2c);
	hal_mchp_i2c_status_clear(i2c);

	/* Set up transfer data. */
	data->num_msgs = num_msgs;
	data->msgs = msgs;

	while (data->num_msgs > 0) {
		/* Check for empty messages (invalid read). */
		if (!data->msgs->len) {
			if ((data->msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
				retval = -EINVAL;
				goto cleanup;
			}
		}

		/* Initialize message buffer and size. */
		data->msg.buffer = data->msgs->buf;
		data->msg.size = data->msgs->len;
		data->msg.status = 0;

		/* Set up the I2C address register with the target address. */
		addr_reg = addr << 1U;
		if ((data->msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
			addr_reg |= 1U; /* Read operation. */
		}

		unsigned int key = irq_lock();

		/* Writing the address starts the I2C transaction. */
		hal_mchp_i2c_addr_write(i2c, addr_reg);

		/* Process the transfer based on the message direction (read/write). */
		if ((data->msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
#ifdef CONFIG_I2C_MCHP_INTERRUPT_DRIVEN
			hal_mchp_i2c_interrupt_enable(i2c);
#else
			retval = i2c_mchp_poll_in(dev);
			if (retval != 0) {
				irq_unlock(key);
				goto cleanup;
			}
#endif
		} else {
#ifdef CONFIG_I2C_MCHP_INTERRUPT_DRIVEN
			hal_mchp_i2c_interrupt_enable(i2c);
#else
			retval = i2c_mchp_poll_out(dev);
			if (retval != 0) {
				irq_unlock(key);
				goto cleanup;
			}
#endif
		}

		irq_unlock(key);

#ifdef CONFIG_I2C_MCHP_INTERRUPT_DRIVEN
		/* Wait for the interrupt handler to complete the transfer. */
		retval = k_sem_take(&data->sem, I2C_TRANSFER_TIMEOUT_MSEC);
		if (retval != 0) {
			LOG_ERR("Transfer timeout on %s", dev->name);
			hal_mchp_i2c_transfer_stop(i2c);
			goto cleanup;
		}

		/* Check the message status for errors. */
		if (data->msg.status) {
			if (data->msg.status & hal_mchp_i2c_status_get(i2c)) {
				LOG_DBG("Arbitration lost on %s", dev->name);
				retval = -EAGAIN;
				goto cleanup;
			}

			LOG_ERR("Transaction error on %s: %08X", dev->name, data->msg.status);
			retval = -EIO;
			goto cleanup;
		}
#endif

		/* Move to the next message in the array. */
		data->num_msgs--;
		data->msgs++;
	}

cleanup:
	k_mutex_unlock(&data->mutex);
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
	const struct hal_mchp_i2c *i2c = &cfg->hal;
	int retval;

	/* Disable the I2C peripheral to prepare for bus recovery. */
	hal_mchp_i2c_disable(i2c);

	/* Disable I2C interrupts to avoid interference during recovery. */
	hal_mchp_i2c_interrupt_disable(i2c);

	/* Lock the mutex to ensure exclusive access to the I2C bus. */
	k_mutex_lock(&data->mutex, K_FOREVER);

	/* Apply the default pin configuration state. */
	retval = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		LOG_ERR("Failed to apply default pin state: %d", retval);
		goto cleanup;
	}

	/* Re-enable the I2C peripheral. */
	hal_mchp_i2c_enable(i2c);

	/* Force the I2C bus to idle state to recover from a bus hang. */
	hal_mchp_i2c_bus_state_idle(i2c);

cleanup:
	/* Unlock the mutex before returning. */
	k_mutex_unlock(&data->mutex);

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
	if (!data->dev_config) {
		return -EINVAL;
	}

	/* Lock the mutex to ensure thread-safe access */
	k_mutex_lock(&data->mutex, K_FOREVER);

	/* Retrieve the current device configuration */
	*dev_config = data->dev_config;

	/* Unlock the mutex after accessing the configuration */
	k_mutex_unlock(&data->mutex);

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
	const struct hal_mchp_i2c *i2c = &cfg->hal;
	uint32_t sys_clock_rate = 0;
	uint32_t bitrate;
	int retval = 0;

#ifdef CONFIG_I2C_TARGET
	/* Check if the device is currently operating in target mode */
	if (data->target_mode) {
		LOG_ERR("Cannot reconfigure while device is in target mode.");
		return -EBUSY;
	}
#endif
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

	/* Lock the mutex for thread-safe access */
	k_mutex_lock(&data->mutex, K_FOREVER);

	/* Disable interrupts to avoid race conditions during configuration */
	unsigned int key = irq_lock();

	/* Retrieve the clock frequency for baud rate calculation */
	I2C_MCHP_GET_CLOCK_FREQ(dev, sys_clock_rate);

	if (sys_clock_rate == 0) {
		LOG_ERR("Failed to retrieve system clock rate.");
		irq_unlock(key);
		k_mutex_unlock(&data->mutex);
		return -EIO;
	}
	/*Set the I2C baud rate */
	if (hal_mchp_i2c_set_baudRate(i2c, bitrate, sys_clock_rate) == false) {
		LOG_ERR("Failed to set I2C baud rate to %u Hz.", bitrate);
		irq_unlock(key);
		k_mutex_unlock(&data->mutex);

		return -EIO;
	}
	/*Update the device configuration with the new speed*/
	data->dev_config = I2C_SPEED_GET(config);

	/* Unlock interrupts and the mutex */
	irq_unlock(key);
	k_mutex_unlock(&data->mutex);

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
	const struct hal_mchp_i2c *i2c = &cfg->hal;

	ARG_UNUSED(data);
	int retval = 0;
	/*Disable the I2C interface to allow configuration changes*/
	hal_mchp_i2c_disable(i2c);

	/*Check if the configuration specifies the I2C controller mode*/
	if (config & I2C_MODE_CONTROLLER) {
		/* Set the I2C to master mode*/
		if (!(hal_mchp_i2c_master_mode(i2c))) {
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
	const struct hal_mchp_i2c *i2c = &cfg->hal;
	int retval;

	/* Enable the GCLK clock for the specified SERCOM instance */
	I2C_MCHP_ENABLE_CLOCK(dev);

	/* Reset all I2C registers*/
	hal_mchp_i2c_swrst(i2c);

	/* Initialize mutex and semaphore for I2C data structure */
	k_mutex_init(&data->mutex);
	k_sem_init(&data->sem, 0, 1);

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
	if (!device_is_ready(cfg->hal.dma_dev)) {
		return -ENODEV;
	}
#endif
	/*Enable the I2C peripheral*/
	hal_mchp_i2c_enable(i2c);

	/*Force the I2C bus to idle state*/
	hal_mchp_i2c_bus_state_idle(i2c);

	return 0;
}

/**/
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
	.recover_bus = i2c_mchp_recover_bus};

#define I2C_MCHP_IRQ_CONNECT(n, m)                                                                 \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, m, irq), DT_INST_IRQ_BY_IDX(n, m, priority),     \
			    i2c_mchp_isr, DEVICE_DT_INST_GET(n), 0);                               \
		irq_enable(DT_INST_IRQ_BY_IDX(n, m, irq));                                         \
	} while (false)

#define I2C_MCHP_CONFIG_DEFN(n)                                                                    \
	static const i2c_mchp_dev_config_t i2c_mchp_dev_config_##n = {                             \
		I2C_MCHP_HAL_DEFN(n),                                                              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                                   \
		.bitrate = DT_INST_PROP(n, clock_frequency),                                       \
		.irq_config_func = &i2c_mchp_irq_config_##n,                                       \
		I2C_MCHP_DMA_CHANNELS(n)}

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
