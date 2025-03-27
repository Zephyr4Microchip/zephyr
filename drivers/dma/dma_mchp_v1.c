/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file dma_mchp_v1.c
 * @brief Generic DMA Driver for Microchip MCUs.
 *
 * This file contains the implementation of a generic DMA driver for Microchip
 * microcontroller units (MCUs). It provides functions to configure and manage
 * DMA channels, handle DMA transfers, and manage DMA interrupts.
 */

#include <zephyr/device.h>
#include <soc.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include <zephyr/kernel.h>
#include "dma_mchp_v1.h"

LOG_MODULE_REGISTER(dma_mchp, CONFIG_DMA_MCHP_LOG_LEVEL);

/**
 * @brief DMA Channel Configuration Structure
 *
 * This structure holds the configuration settings for a Microchip DMA channel.
 * It includes callback function details, user-defined data, and the channel state.
 */
typedef struct dma_mchp_channel_config {

	/* Callback function for DMA transfer events. */
	dma_callback_t cb;

	/* User-defined data passed to the callback. */
	void *user_data;

	/* Channel state */
	bool is_configured;

} dma_mchp_channel_config_t;

/**
 * @brief Configuration structure for the Microchip DMA controller.
 *
 * This structure holds the hardware abstraction layer (HAL) data,
 * the IRQ configuration function, and clock settings.
 */
typedef struct dma_mchp_dev_config {

	/* DMA HAL structure */
	hal_mchp_dma_t hal_dma;

	/* Pointer to the clock device used for controlling the DMA's clock. */
	const struct device *clock_dev;

	/* Function pointer for configuring DMA interrupts. */
	void (*irq_config)(void);

} dma_mchp_dev_config_t;

/**
 * @brief Structure holding runtime data for the Microchip DMA controller.
 *
 * This structure contains descriptor tables and channel-specific data used
 * during DMA operations.
 */
typedef struct dma_mchp_dev_data {

	/* dma context */
	struct dma_context dma_ctx;

	/* Declare the Data  */
	void *dma_hal_data;

	/* DMA Channels configuration */
	dma_mchp_channel_config_t *dma_channel_config;

	/* Pool of descriptor */
	struct k_fifo dma_desc_pool;

	/* Num of descriptors in the pool */
	uint32_t dma_desc_pool_cnt;

	/* Specifies the required DMA alignment, typically based on hardware constraints. */
	uint16_t dma_alignment;

} dma_mchp_dev_data_t;

/**
 * @brief Retrieves a descriptor from the DMA descriptor pool.
 *
 * This function fetches a descriptor from the DMA descriptor pool FIFO if
 * the descriptor pool count is non-zero. The function will block indefinitely
 * until a descriptor is available.
 *
 * @param dma_desc_pool Pointer to the DMA descriptor FIFO pool.
 * @param dma_desc_pool_cnt Pointer to the DMA descriptor pool counter, decremented on retrieval.
 *
 * @return Pointer to the retrieved descriptor, or NULL if the pool count is zero.
 */
void *dma_mchp_desc_get(struct k_fifo *dma_desc_pool, uint32_t *dma_desc_pool_cnt)
{
	void *desc = NULL;

	/* Ensure pool count is valid before accessing FIFO */
	if (dma_desc_pool_cnt != NULL && *dma_desc_pool_cnt != 0) {
		/* Decrement descriptor pool count */
		(*dma_desc_pool_cnt)--;
		/* Fetch a descriptor */
		desc = k_fifo_get(dma_desc_pool, K_FOREVER);
	}

	return desc;
}

/**
 * @brief Adds a descriptor to the DMA descriptor pool.
 *
 * This function inserts a descriptor into the DMA descriptor FIFO pool
 * and increments the descriptor pool count.
 *
 * @param dma_desc_pool Pointer to the DMA descriptor FIFO pool.
 * @param dma_desc_pool_cnt Pointer to the DMA descriptor pool counter, incremented when a
 * descriptor is added.
 * @param new_desc_node Pointer to the descriptor node being added to the pool.
 */
void dma_mchp_desc_put(struct k_fifo *dma_desc_pool, uint32_t *dma_desc_pool_cnt,
		       void *new_desc_node)
{
	/* Add descriptor to the FIFO */
	k_fifo_put(dma_desc_pool, new_desc_node);
	/* Increment descriptor pool count */
	(*dma_desc_pool_cnt)++;
}

/**
 * @brief Creates a pool of DMA descriptors with the specified alignment.
 *
 * This function initializes a FIFO queue for DMA descriptors, allocates memory-aligned
 * descriptors, and adds them to the descriptor pool.
 *
 * @param dma_desc_pool Pointer to the FIFO queue that will store the descriptors.
 * @param dma_desc_pool_cnt Pointer to the variable that keeps track of the number of descriptors in
 * the pool.
 * @param dma_alignment Alignment requirement for the DMA descriptors in bytes.
 * @param desc_count Number of descriptors to allocate.
 *
 * @return 0 on success.
 * @return -ENOMEM if memory allocation fails.
 */
int8_t dma_mchp_desc_pool_create(struct k_fifo *dma_desc_pool, uint32_t *dma_desc_pool_cnt,
				 uint16_t dma_alignment, int desc_count)
{
	int8_t ret = 0;
	/* Get the required size for a DMA descriptor */
	uint32_t desc_size = hal_mchp_dma_get_desc_size();

	/* Initialize the FIFO queue for DMA descriptors */
	k_fifo_init(dma_desc_pool);

	/* Allocate and enqueue the requested number of descriptors */
	for (int i = 0; i < desc_count; i++) {
		/* Allocate memory for a new descriptor */
		void *new_desc_node = (void *)k_aligned_alloc(dma_alignment, desc_size);

		/* Check if memory allocation failed */
		if (new_desc_node == NULL) {
			LOG_ERR("DMA Error : DMA Descriptor pool creation failed!");
			ret = -ENOMEM;
			break;
		}

		/* Validate descriptor size to ensure it meets FIFO requirements */
		if (desc_size < sizeof(void *)) {
			LOG_ERR("DMA Error: Descriptor size too small for FIFO!");
			/* Free the allocated memory before returning */
			k_free(new_desc_node);
			ret = -ENOMEM;
			break;
		}

		/* Add the newly allocated descriptor to the pool */
		dma_mchp_desc_put(dma_desc_pool, dma_desc_pool_cnt, new_desc_node);
	}

	return ret;
}

/**
 * @brief Enqueue used DMA descriptors into the descriptor pool.
 *
 * This function retrieves used DMA descriptors from the specified channel
 * and enqueues them into the descriptor pool for reuse.
 *
 * @param hal_dma Pointer to the DMA HAL instance.
 * @param dev_data Pointer to the DMA device data structure.
 * @param channel DMA channel number from which descriptors are retrieved.
 */
void dma_mchp_desc_return_pool(const hal_mchp_dma_t *hal_dma, dma_mchp_dev_data_t *dev_data,
			       int channel)
{
	void *desc = NULL;

	/* Retrieve used descriptors and enqueue them back into the pool */
	while ((desc = hal_mchp_dma_desc_get_used(hal_dma, channel)) != NULL) {
		if (dev_data->dma_desc_pool_cnt == CONFIG_DMA_MCHP_V1_MAX_DESC) {
			break;
		}
		/* Add the descriptor to the pool for reuse */
		dma_mchp_desc_put(&dev_data->dma_desc_pool, &dev_data->dma_desc_pool_cnt, desc);
	}
}

/**
 * @brief Log error message based on DMA error code.
 *
 * This function logs an appropriate error message based on the provided
 * DMA error code.
 *
 * @param err_code The DMA error code.
 */
void dma_mchp_log_dma_error(int err_code)
{
	switch (err_code) {
	case HAL_MCHP_DMA_ERR_INVALID_TRIGGER_SRC:
		LOG_ERR("Invalid parameter for DMA trigger source");
		break;
	case HAL_MCHP_DMA_ERR_INVALID_DIRECTION:
		LOG_ERR("Invalid parameter for DMA channel direction");
		break;
	case HAL_MCHP_DMA_ERR_INVALID_PRIORITY:
		LOG_ERR("Invalid parameter for DMA priority level");
		break;
	case HAL_MCHP_DMA_ERR_SRC_DST_BURST_LENGTH_MISMATCH:
		LOG_ERR("Source and destination burst lengths do not match");
		break;
	case HAL_MCHP_DMA_ERR_BURST_LENGTH_OVERFLOW:
		LOG_ERR("Burst length exceeds maximum allowed value");
		break;
	case HAL_MCHP_DMA_ERR_INVALID_SRC_DATA_SIZE:
		LOG_ERR("Invalid parameter for DMA source data size");
		break;
	case HAL_MCHP_DMA_ERR_INVALID_SRC_ADDR:
		LOG_ERR("Invalid parameter for DMA source address");
		break;
	case HAL_MCHP_DMA_ERR_INVALID_DST_ADDR:
		LOG_ERR("Invalid parameter for DMA destination address");
		break;
	case HAL_MCHP_DMA_ERR_SRC_DST_DATA_SIZE_MISMATCH:
		LOG_ERR("Source and destination data sizes do not match");
		break;
	case HAL_MCHP_DMA_ERR_DESCRIPTOR_OVERFLOW:
		LOG_ERR("Required Descriptors are not available in the pool");
		break;
	case HAL_MCHP_DMA_RELOAD_CONFIG_ERR:
		LOG_ERR("Multiple descriptors are configured, dma_reload configuration is not "
			"valid");
		break;
	case HAL_MCHP_DMA_ERR_INVALID_CONFIG:
		LOG_ERR("Invalid configuration");
		break;
	default:
		/* LOG_ERR("Unknown DMA error code: %d", err_code); */
		break;
	}
}

/**
 * @brief DMA interrupt service routine (ISR).
 *
 * This function handles DMA interrupts and delegates processing
 * to the appropriate ISR handler for the given DMA instance.
 *
 * @param dev Pointer to the DMA device structure.
 */
static void dma_mchp_isr(const struct device *dev)
{
	/* Retrieve device configuration */
	const dma_mchp_dev_config_t *dev_cfg = dev->config;
	/* Get HAL instance for DMA */
	const hal_mchp_dma_t *hal_dma = &dev_cfg->hal_dma;
	/* Retrieve device runtime data */
	dma_mchp_dev_data_t *const dev_data = ((dma_mchp_dev_data_t *const)(dev)->data);
	dma_mchp_channel_config_t *channel_config = NULL;
	uint32_t channel;

	/* Handle interrupt and get status */
	int status = hal_mchp_dma_interrupt_handle_status(hal_dma, &channel);

	/* Clear the descriptors */
	dma_mchp_desc_return_pool(hal_dma, dev_data, channel);

	channel_config = &dev_data->dma_channel_config[channel];

	if (channel_config->cb != NULL) {
		if (status == DMA_MCHP_INT_SUCCESS) {
			channel_config->cb(dev, channel_config->user_data, channel,
					   DMA_STATUS_COMPLETE);
		} else {
			channel_config->cb(dev, channel_config->user_data, channel, -1);
		}
	}
}

/**
 * @brief Configures a DMA channel with the given settings.
 *
 * This function initializes and configures a specified DMA channel, including setting up
 * the trigger source, priority, burst length, and linked descriptors for multi-block transfers.
 *
 * @param dev Pointer to the DMA device structure.
 * @param channel DMA channel number to configure.
 * @param config Pointer to the DMA configuration structure.
 *
 * @return 0 on successful configuration.
 * @return -EINVAL if an invalid parameter is provided.
 * @return -EBUSY if the requested channel is already active.
 * @return -ENOMEM if memory allocation for descriptors fails.
 */
static int dma_mchp_config(const struct device *dev, uint32_t channel, struct dma_config *config)
{
	/* Get device config */
	const dma_mchp_dev_config_t *const dev_cfg = dev->config;
	/* Get HAL Instance */
	const hal_mchp_dma_t *hal_dma = &dev_cfg->hal_dma;
	/* Get dev data */
	dma_mchp_dev_data_t *const dev_data = dev->data;
	uint32_t i = 0, key = 0;
	hal_dma_mchp_block_config_t block_config;
	hal_dma_mchp_ch_state_t ch_state = DMA_MCHP_CH_IDLE;
	struct dma_block_config *block = NULL;
	dma_mchp_channel_config_t *channel_config = NULL;
	int ret = 0;
	bool irq_locked = false;
	void *desc = NULL, *base_desc = NULL;

	do {
		/* Validate channel number */
		if (channel >= dev_data->dma_ctx.dma_channels) {
			LOG_ERR("Unsupported channel");
			ret = -EINVAL;
			break;
		}

		/* Check if the channel is already in use */
		ch_state = hal_mchp_dma_ch_get_state(hal_dma, channel);
		if (ch_state == DMA_MCHP_CH_ACTIVE) {
			LOG_ERR("DMA channel %d is already in use", channel);
			ret = -EBUSY;
			break;
		}

		/* Lock interrupts to ensure atomic operation */
		key = irq_lock();
		irq_locked = true;

		/* Configures the trigger source and transfer direction for the DMA channel.
		 * Returns an error code if the specified direction or trigger source is invalid.
		 */
		ret = hal_mchp_dma_ch_set_trig_src_n_dir(
			hal_dma, channel, config->dma_slot,
			(hal_dma_mchp_ch_direction_t)config->channel_direction);
		if (ret < 0) {
			break;
		}

		/* Configures the channel priority. Returns an error code if the specified priority
		 * is invalid.
		 */
		ret = hal_mchp_dma_ch_set_priority(hal_dma, channel, config->channel_priority);
		if (ret < 0) {
			break;
		}

		/* Sets the burst length for the DMA channel. Returns an error code if the specified
		 * burst length is invalid.
		 */
		ret = hal_mchp_dma_ch_set_burst_length(
			hal_dma, channel, config->source_burst_length, config->dest_burst_length);
		if (ret < 0) {
			break;
		}

		bool disable_err_interrupt = config->error_callback_dis ? true : false;

		/* Enable DMA interrupts for the channel */
		hal_mchp_dma_ch_interrupt_enable(hal_dma, channel, disable_err_interrupt);

		/* Ensure source and destination data sizes match */
		if (config->source_data_size != config->dest_data_size) {
			ret = HAL_MCHP_DMA_ERR_SRC_DST_DATA_SIZE_MISMATCH;
			break;
		}

		LOG_DBG("Available Descriptors in the pool : %d", dev_data->dma_desc_pool_cnt);

		/* Get the channel's First descriptor base address*/
		desc = hal_mchp_dma_desc_get_base_addr(hal_dma, channel);
		base_desc = desc;

		LOG_DBG("Channel %d: First descriptor base address: %p", channel, desc);

		/* Get the head block */
		block = config->head_block;
		HAL_DMA_MCHP_COPY_BLOCK_CONFIG(block_config, block);

		/* Configure the first block */
		LOG_DBG("BLOCK 1: Configure descriptor at address: %p", desc);
		ret = hal_mchp_dma_desc_block_config(&block_config,           /* Head block */
						     desc,                    /* current desc */
						     NULL,                    /* prev desc */
						     config->source_data_size /* Source data size */
		);
		if (ret < 0) {
			LOG_ERR("DMA Error: Block : 1 Configuration Failed!");
			break;
		}

		/* Check multi block */
		if (config->block_count > 1) {
			/* Check whether we have enough descriptors in the pool */
			if (dev_data->dma_desc_pool_cnt < config->block_count) {
				LOG_ERR("Requested number of Descriptors are not available in the "
					"Descriptor pool. Please configure more descriptors and "
					"try again\r\n");
				ret = HAL_MCHP_DMA_ERR_DESCRIPTOR_OVERFLOW;
				break;
			}
		}

		/* Configure multiple blocks here */
		for (i = 1; i < config->block_count; i++) {
			void *prev_desc = desc;

			/* Get the Descriptor from the pool */
			desc = dma_mchp_desc_get(&dev_data->dma_desc_pool,
						 &dev_data->dma_desc_pool_cnt);
			if (desc == NULL) {
				LOG_ERR("DMA Error: Failed to get descriptor for block %d", i);
				ret = -ENOMEM;
				break;
			}

			/* Move to the next block */
			block = block->next_block;
			HAL_DMA_MCHP_COPY_BLOCK_CONFIG(block_config, block);

			/* Configure the block */
			LOG_DBG("BLOCK %d: Configure descriptor at address: %p", i + 1, desc);
			ret = hal_mchp_dma_desc_block_config(
				&block_config,           /* block */
				desc,                    /* current desc */
				prev_desc,               /* prev desc */
				config->source_data_size /* Source data size */
			);

			if (i == (config->block_count - 1)) {
				/* Last descriptor, check if we should setup a circular chain */
				if (config->cyclic == true) {
					hal_mchp_dma_enable_cyclic_desc_chain(desc, base_desc);
				}
			}

			if (ret < 0) {
				LOG_ERR("DMA Error: Block : %d Configuration Failed!", (i + 1));
				break;
			}
		}

		/* If any error occurred inside for loop, break from do while loop */
		if (ret < 0) {
			break;
		}

		LOG_DBG("Available Descriptors in the pool : %d", dev_data->dma_desc_pool_cnt);

		channel_config = &dev_data->dma_channel_config[channel];
		/* Register the callback function for the channel*/
		channel_config->cb = config->dma_callback;
		channel_config->user_data = config->user_data;

		/* Update channel state to configured */
		channel_config->is_configured = true;

		ret = 0;
	} while (0);

	/* Unlock the IRQ */
	if (irq_locked == true) {
		irq_unlock(key);
	}

	/* Log the error message */
	if (ret != 0) {
		dma_mchp_log_dma_error(ret);
		ret = -EINVAL;
	}

	return ret;
}

/**
 * @brief Starts a DMA transfer on a specified channel.
 *
 * This function checks if the channel is valid, idle, and properly configured before
 * enabling the DMA transfer. It ensures atomic operations using interrupt locking.
 *
 * @param dev Pointer to the device structure for the DMA driver.
 * @param channel DMA channel number to start the transfer.
 *
 * @return 0 on success, indicating the DMA transfer started successfully.
 * @return -EINVAL if the channel number is invalid or descriptors are not configured.
 * @return -EBUSY if the DMA channel is not idle.
 */
static int dma_mchp_start(const struct device *dev, uint32_t channel)
{
	/* Retrieve device configuration */
	const dma_mchp_dev_config_t *dev_cfg = dev->config;
	/* Get HAL instance for DMA */
	const hal_mchp_dma_t *hal_dma = &dev_cfg->hal_dma;
	/* Get dev data */
	dma_mchp_dev_data_t *const dev_data = dev->data;
	dma_mchp_channel_config_t *channel_config = NULL;
	hal_dma_mchp_ch_state_t ch_state;
	int ret = 0;
	uint32_t key = 0;

	do {
		/* Validate channel number */
		if (channel >= dev_data->dma_ctx.dma_channels) {
			LOG_ERR("Unsupported channel");
			ret = -EINVAL;
			break;
		}

		/* Check if the channel is already in use */
		ch_state = hal_mchp_dma_ch_get_state(hal_dma, channel);
		if (ch_state == DMA_MCHP_CH_ACTIVE) {
			LOG_ERR("DMA channel:%d is currently busy", channel);
			ret = -EBUSY;
			break;
		}

		/* Check if the channel is configured */
		channel_config = &dev_data->dma_channel_config[channel];
		if (channel_config->is_configured != true) {
			LOG_ERR("DMA descriptors not configured for channel : %d", channel);
			ret = -EINVAL;
			break;
		}

		/* Lock interrupts to ensure atomic operation */
		key = irq_lock();

		/* Enable the DMA channel and start the transfer */
		hal_mchp_dma_ch_enable(hal_dma, channel, true);

		/* Unlock interrupts */
		irq_unlock(key);

		ret = 0;

	} while (0);

	return ret;
}

/**
 * @brief Stops a DMA transfer on the specified channel.
 *
 * This function disables the DMA channel, returns any allocated descriptors
 * to the pool, and marks the channel as unconfigured. It ensures atomic
 * operation using interrupt locking.
 *
 * @param dev Pointer to the device structure for the DMA driver.
 * @param channel DMA channel number to stop the transfer.
 *
 * @return 0 on success, indicating the DMA channel was successfully stopped.
 * @return -EINVAL if the provided channel number is invalid.
 */
static int dma_mchp_stop(const struct device *dev, uint32_t channel)
{
	/* Retrieve device configuration */
	const dma_mchp_dev_config_t *dev_cfg = dev->config;
	/* Get HAL instance for DMA */
	const hal_mchp_dma_t *hal_dma = &dev_cfg->hal_dma;
	/* Get dev data */
	dma_mchp_dev_data_t *const dev_data = dev->data;
	uint32_t key = 0;
	int ret = 0;

	do {
		/* Validate the DMA channel */
		if (channel >= dev_data->dma_ctx.dma_channels) {
			LOG_ERR("Unsupported channel");
			ret = -EINVAL;
			break;
		}

		/* Lock interrupts to ensure atomic operation */
		key = irq_lock();

		/* Disable the channel */
		hal_mchp_dma_ch_enable(hal_dma, channel, false);

		/* Put descriptors back to the pool, if configured and not returned to the pool */
		dma_mchp_desc_return_pool(hal_dma, dev_data, channel);

		/* Unlock interrupts */
		irq_unlock(key);

		ret = 0;

	} while (0);

	printk("ret : %d\n", ret);

	return ret;
}

/**
 * @brief Reloads a DMA transfer for the specified channel.
 *
 * This function updates the source and destination addresses for a DMA transfer
 * and reloads the descriptor with the new transfer size.
 *
 * @param dev Pointer to the DMA device structure.
 * @param channel DMA channel number to be reloaded.
 * @param src Source address for the DMA transfer.
 * @param dst Destination address for the DMA transfer.
 * @param size Transfer size in bytes.
 *
 * @return 0 on success, -EINVAL if the channel is invalid.
 */
static int dma_mchp_reload(const struct device *dev, uint32_t channel, uint32_t src, uint32_t dst,
			   size_t size)
{
	/* Get device config */
	const dma_mchp_dev_config_t *dev_cfg = dev->config;
	/* Get HAL Instance */
	const hal_mchp_dma_t *hal_dma = &dev_cfg->hal_dma;
	/* Get dev data */
	dma_mchp_dev_data_t *const dev_data = dev->data;
	dma_mchp_channel_config_t *channel_config = NULL;
	int ret = 0;
	uint32_t key = 0;
	bool irq_locked = false;
	hal_dma_mchp_ch_state_t ch_state = DMA_MCHP_CH_IDLE;

	do {
		/* Validate the DMA channel */
		if (channel >= dev_data->dma_ctx.dma_channels) {
			LOG_ERR("Unsupported channel");
			ret = -EINVAL;
			break;
		}

		/* Check if the channel is configured */
		channel_config = &dev_data->dma_channel_config[channel];
		if (channel_config->is_configured != true) {
			LOG_ERR("DMA descriptors not configured for channel : %d", channel);
			ret = -EINVAL;
			break;
		}

		/* Check if the channel is already in use */
		ch_state = hal_mchp_dma_ch_get_state(hal_dma, channel);
		if (ch_state == DMA_MCHP_CH_ACTIVE) {
			LOG_ERR("DMA channel:%d is currently busy", channel);
			ret = -EBUSY;
			break;
		}

		/* Lock interrupts to ensure atomic update */
		key = irq_lock();
		irq_locked = true;

		/* Reloads the DMA descriptor with the specified source, destination, and size.
		 * Returns an error code if the provided parameters are invalid.
		 */
		ret = hal_mchp_dma_desc_reload_block(hal_dma, channel, src, dst, size);
		if (ret < 0) {
			dma_mchp_log_dma_error(ret);
			ret = -EINVAL;
			break;
		}

		LOG_DBG("Reloaded channel %d for %08X to %08X (%u)", channel, src, dst, size);
		ret = 0;
	} while (0);

	if (irq_locked == true) {
		/* Unlock the IRQ */
		irq_unlock(key);
	}

	return ret;
}

/**
 * @brief Suspends an active DMA transfer on the specified channel.
 *
 * This function temporarily halts the DMA transfer on the given channel
 * without resetting its configuration, allowing it to be resumed later.
 *
 * @param dev Pointer to the DMA device structure.
 * @param channel DMA channel number to suspend.
 *
 * @return 0 on success, -EINVAL if the channel number is invalid.
 */
static int dma_mchp_suspend(const struct device *dev, uint32_t channel)
{
	/* Get device config */
	const dma_mchp_dev_config_t *dev_cfg = dev->config;
	/* Get HAL Instance */
	const hal_mchp_dma_t *hal_dma = &dev_cfg->hal_dma;
	/* Get dev data */
	dma_mchp_dev_data_t *const dev_data = dev->data;
	unsigned int key = 0;
	hal_dma_mchp_ch_state_t ch_state = DMA_MCHP_CH_IDLE;
	int ret = 0;

	do {
		/* Validate the DMA channel */
		if (channel >= dev_data->dma_ctx.dma_channels) {
			LOG_ERR("Unsupported channel");
			ret = -EINVAL;
		}

		/* Check if the channel is currently active */
		ch_state = hal_mchp_dma_ch_get_state(hal_dma, channel);
		if (ch_state != DMA_MCHP_CH_ACTIVE) {
			LOG_INF("nothing to suspend as dma channel %u is not busy", channel);
			ret = 0;
		}

		/* Lock interrupts to prevent race conditions */
		key = irq_lock();
		/* Suspend the specified DMA channel */
		hal_mchp_dma_ch_suspend(hal_dma, channel);
		LOG_DBG("channel %u is suspended", channel);
		/* Unlock interrupts */
		irq_unlock(key);

		ret = 0;

	} while (0);

	return ret;
}

/**
 * @brief Resumes a previously suspended DMA transfer on the specified channel.
 *
 * This function resumes a DMA transfer that was previously suspended using
 * dma_mchp_suspend(). The transfer continues from where it was paused.
 *
 * @param dev Pointer to the DMA device structure.
 * @param channel DMA channel number to resume.
 *
 * @return 0 on success, -EINVAL if the channel number is invalid.
 */
static int dma_mchp_resume(const struct device *dev, uint32_t channel)
{
	/* Get device config */
	const dma_mchp_dev_config_t *dev_cfg = dev->config;
	/* Get HAL Instance */
	const hal_mchp_dma_t *hal_dma = &dev_cfg->hal_dma;
	/* Get dev data */
	dma_mchp_dev_data_t *const dev_data = dev->data;
	uint32_t key = 0;
	hal_dma_mchp_ch_state_t ch_state = DMA_MCHP_CH_IDLE;
	int ret = 0;

	do {
		/* Validate the DMA channel */
		if (channel >= dev_data->dma_ctx.dma_channels) {
			LOG_ERR("Unsupported channel");
			ret = -EINVAL;
		}

		/* Check if the channel is already suspended */
		ch_state = hal_mchp_dma_ch_get_state(hal_dma, channel);
		if (ch_state != DMA_MCHP_CH_SUSPENDED) {
			LOG_INF("DMA channel %d is not in suspended state so cannot resume channel",
				channel);
			ret = 0;
		}

		/* Lock interrupts to prevent race conditions */
		key = irq_lock();
		/* Resume the specified DMA channel */
		hal_mchp_dma_ch_resume(hal_dma, channel);
		LOG_DBG("channel %u is resumed", channel);
		/* Unlock interrupts */
		irq_unlock(key);

		ret = 0;

	} while (0);

	return ret;
}

/**
 * @brief Retrieves the status of a DMA channel.
 *
 * This function checks the status of the specified DMA channel and fills
 * the provided dma_status structure with relevant information.
 *
 * @param dev Pointer to the DMA device structure.
 * @param channel DMA channel number to retrieve the status for.
 * @param stat Pointer to a dma_status structure to store the status.
 *
 * @return 0 on success, or an error code if the status retrieval fails.
 */
static int dma_mchp_get_status(const struct device *dev, uint32_t channel, struct dma_status *stat)
{
	/* Get device config */
	const dma_mchp_dev_config_t *dev_cfg = dev->config;
	/* Get HAL Instance */
	const hal_mchp_dma_t *hal_dma = &dev_cfg->hal_dma;
	/* Get dev data */
	dma_mchp_dev_data_t *const dev_data = dev->data;
	hal_dma_mchp_status_t dma_mchp_status;
	int ret = 0;

	do {
		/* Validate the DMA channel */
		if (channel >= dev_data->dma_ctx.dma_channels) {
			LOG_ERR("Unsupported channel");
			ret = -EINVAL;
		}

		/* Retrieve DMA channel status */
		ret = hal_mchp_dma_ch_get_status(hal_dma, channel, &dma_mchp_status);
		if (ret < 0) {
			break;
		}

		/* Copy the status to the stat variable */
		HAL_DMA_MCHP_COPY_STATUS(stat, dma_mchp_status);

		ret = 0;
	} while (0);

	/* Log the error message */
	if (ret != 0) {
		dma_mchp_log_dma_error(ret);
		ret = -EINVAL;
	}
	return ret;
}

/**
 * @brief Initializes the Microchip DMA controller.
 *
 * This function enables the DMA clock, resets the DMA controller, initializes
 * descriptors, sets default priority levels, enables the DMA module, and configures
 * the DMA interrupt.
 *
 * @param dev Pointer to the DMA device structure.
 *
 * @return 0 on successful initialization.
 * @return -ENOMEM if the descriptor pool creation fails due to memory allocation failure.
 * @return -EINVAL if an invalid parameter is encountered.
 */
static int dma_mchp_init(const struct device *dev)
{
	/* Get device configuration */
	const dma_mchp_dev_config_t *dev_cfg = dev->config;
	/* Get HAL Instance */
	const hal_mchp_dma_t *hal_dma = &dev_cfg->hal_dma;
	/* Get dev data */
	dma_mchp_dev_data_t *const dev_data = dev->data;
	int ret = 0;

	/* Enable DMA clock */
	DMA_MCHP_ENABLE_CLOCK(dev);

	/* Reset the DMA controller */
	hal_mchp_dma_controller_reset(hal_dma);

	/* Initialize DMA descriptors */
	hal_mchp_dma_desc_init(hal_dma);

	/* Set default priority levels */
	hal_mchp_dma_set_default_priority(hal_dma);

	/* Enable the DMA controller */
	hal_mchp_dma_enable(hal_dma, true);

	/* Configure DMA interrupt */
	dev_cfg->irq_config();

	/* Create the pool of descriptors */
	ret = dma_mchp_desc_pool_create(&dev_data->dma_desc_pool, &dev_data->dma_desc_pool_cnt,
					dev_data->dma_alignment, CONFIG_DMA_MCHP_V1_MAX_DESC);
	if (ret < 0) {
		LOG_ERR("DMA Pool creation failed!");
	}
	LOG_DBG("DMA Pool creation success!");

	return ret;
}

/**
 * @brief Declare the DMA IRQ connection handler for a specific instance.
 *
 * This macro defines a static function prototype for connecting
 * the DMA interrupt handler of a given DMA instance.
 *
 * @param n Instance index of the DMA controller.
 */
#define DMA_MCHP_IRQ_HANDLER_DECL(n) static void mchp_dma_irq_connect_##n(void)

/**
 * @brief DMA runtime data structure.
 *
 * This macro declares and initializes DMA-related data structures
 * required for a specific DMA instance.
 *
 * @param n Instance index of the DMA controller.
 *
 */
#define DMA_MCHP_DATA_DEFN(n)                                                                      \
	static __aligned(DT_INST_PROP(                                                             \
		n, dma_alignment)) uint8_t dma_hal_data_##n[HAL_DMA_MCHP_DATA_MEMORY_SIZE];        \
	ATOMIC_DEFINE(dma_mchp_atomic##n, DT_INST_PROP(n, dma_channels));                          \
	static dma_mchp_channel_config_t dma_channel_config_##n[DT_INST_PROP(n, dma_channels)];    \
	static dma_mchp_dev_data_t dma_mchp_dev_data_##n = {                                       \
		.dma_ctx =                                                                         \
			{                                                                          \
				.magic = DMA_MAGIC,                                                \
				.atomic = dma_mchp_atomic##n,                                      \
				.dma_channels = DT_INST_PROP(n, dma_channels),                     \
			},                                                                         \
		.dma_hal_data = &dma_hal_data_##n,                                                 \
		.dma_desc_pool_cnt = 0,                                                            \
		.dma_channel_config = dma_channel_config_##n,                                      \
		.dma_alignment = DT_INST_PROP(n, dma_alignment),                                   \
	};

/**
 * @brief DMA device configuration structure.
 *
 * This structure holds the static configuration of the DMA device,
 * including the hardware abstraction layer (HAL) and interrupt definitions.
 */
#define DMA_MCHP_CONFIG_DEFN(n)                                                                    \
	static const dma_mchp_dev_config_t dma_mchp_dev_config_##n = {                             \
		DMA_MCHP_HAL_DEFN(n), .hal_dma.data = &dma_hal_data_##n,                           \
		.irq_config = mchp_dma_irq_connect_##n,                                            \
		.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock))};

/**
 * @brief DMA driver API structure.
 *
 * This structure defines the supported API functions for the DMA driver.
 */
static const struct dma_driver_api dma_mchp_api = {
	.config = dma_mchp_config,
	.start = dma_mchp_start,
	.stop = dma_mchp_stop,
	.reload = dma_mchp_reload,
	.get_status = dma_mchp_get_status,
	.suspend = dma_mchp_suspend,
	.resume = dma_mchp_resume,
};

/**
 * @brief Enable IRQ lines for the DMA controller.
 *
 * This macro connects the interrupt line to the DMA ISR and enables it
 * if the instance has an associated IRQ.
 *
 * @param idx  The index of the IRQ in the instance.
 * @param inst The instance number from the device tree.
 */
#define DMA_MCHP_IRQ_CONNECT(idx, n)                                                               \
	IF_ENABLED(DT_INST_IRQ_HAS_IDX(n, idx), ( \
		/** Connect the IRQ to the DMA ISR */ \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, idx, irq), \
			DT_INST_IRQ_BY_IDX(n, idx, priority), \
			dma_mchp_isr, \
			DEVICE_DT_INST_GET(n), 0); \
		/** Enable the IRQ */ \
		irq_enable(DT_INST_IRQ_BY_IDX(n, idx, irq)); \
	))

/**
 * @brief Define the DMA IRQ handler function for a given instance.
 *
 * This macro generates a function to connect all IRQ lines of a DMA instance
 * by calling DMA_MCHP_IRQ_CONNECT for each available IRQ in the instance.
 *
 * @param n The DMA instance number.
 */
#define DMA_MCHP_IRQ_HANDLER(n)                                                                    \
	static void mchp_dma_irq_connect_##n(void)                                                 \
	{                                                                                          \
		/** Connect all IRQs for this instance */                                          \
		LISTIFY(\
			DT_NUM_IRQS(DT_DRV_INST(n)), \
			DMA_MCHP_IRQ_CONNECT, \
			(), \
			n\
		)                                                                          \
	}

/**
 * @brief Define and initialize the DMA device.
 *
 * This macro registers the DMA driver with the Zephyr device model.
 * It initializes the DMA controller during the pre-kernel phase.
 */
#define DMA_MCHP_DEVICE_INIT(n)                                                                    \
	DMA_MCHP_IRQ_HANDLER_DECL(n);                                                              \
	DMA_MCHP_DATA_DEFN(n);                                                                     \
	DMA_MCHP_CONFIG_DEFN(n);                                                                   \
	DEVICE_DT_INST_DEFINE(n, &dma_mchp_init, NULL, &dma_mchp_dev_data_##n,                     \
			      &dma_mchp_dev_config_##n, PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY,    \
			      &dma_mchp_api);                                                      \
	DMA_MCHP_IRQ_HANDLER(n)

DT_INST_FOREACH_STATUS_OKAY(DMA_MCHP_DEVICE_INIT);
