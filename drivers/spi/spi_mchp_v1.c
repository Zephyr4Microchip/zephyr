/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file spi_mchp_v1.c
 * @brief SPI driver for Microchip Technology Inc. devices
 *
 * This file contains the implementation of the SPI driver for Microchip
 * Technology Inc. devices. It includes initialization, configuration, and
 * data transfer functions.
 */

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_mchp_v1);

#include "spi_context.h"
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include <zephyr/irq.h>
#include <soc.h>
#include "spi_mchp_v1.h"

/**
 * @brief DMA configuration structure for the SPI
 *
 * This structure containes the DMA configuration parameters for the SPI
 * peripheral.
 */
typedef struct mchp_spi_dma {
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
} mchp_spi_dma_t;

/**
 * @brief Device constant configuration parameters
 *
 * This structure holds the constant configuration parameters for the SPI
 * device.
 */
typedef struct spi_mchp_dev_config {
	/* HAL layer for SPI */
	hal_mchp_spi_t hal;

	/* Pin control device configuration */
	const struct pinctrl_dev_config *pcfg;

#if CONFIG_SPI_MCHP_DMA_DRIVEN
	/* DMA device */
	mchp_spi_dma_t spi_dma;
#endif

#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_MCHP_INTERRUPT_DRIVEN)
	/*IRQ configuration function */
	void (*irq_config_func)(const struct device *dev);
#endif
	/* Clock configuration */
	mchp_spi_clock_t spi_clock;

} spi_mchp_dev_config_t;

/**
 * @brief SPI run time data structure.
 *
 * This structure holds the runtime data for the SPI driver.
 */
typedef struct spi_mchp_dev_data {
	/* SPI context structure. */
	struct spi_context ctx;
#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_MCHP_INTERRUPT_DRIVEN)
	/* Variable to store the dummysize for tx and rx. */
	uint8_t dummysize;
#endif
#if CONFIG_SPI_MCHP_DMA_DRIVEN
	/* Pointer to the device structure. */
	const struct device *dev;
	/* Length of the DMA segment. */
	uint32_t dma_segment_len;
#endif
} spi_mchp_dev_data_t;

/**
 * @brief Configure the SPI peripheral.
 *
 * This function configures the SPI peripheral based on the provided
 * configuration. It disables the SPI, checks if it is already configured,
 * sets character size, enables the receiver, sets baud rate, configures
 * operation mode, sets data out and pin out configuration, clock polarity,
 * clock phase, data order, and duplex mode. Finally, it enables the SPI
 * peripheral and saves the configuration into the context.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param config Pointer to the SPI configuration structure.
 *
 * @return 0 on success, negative error code on failure.
 */
static int spi_mchp_configure(const struct device *dev, const struct spi_config *config)
{
	const spi_mchp_dev_config_t *cfg = dev->config;
	const hal_mchp_spi_t *hal = &cfg->hal;
	spi_mchp_dev_data_t *const data = dev->data;
	int err;
	uint32_t clock_rate;

	/* Disable the SPI for Configuration */
	hal_mchp_spi_disable(hal);

	/* Check if SPI is already configured */
	if (spi_context_configured(&data->ctx, config) == true) {
		hal_mchp_spi_enable(hal);
		return 0;
	}

	/* Select the Character Size */
	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		/* Char size 9 bit mode is not supported */

		err = hal_mchp_spi_9bit_ch_size(hal);
		if (err != 0) {
			return -ENOTSUP;
		}
	} else {
		err = hal_mchp_spi_8bit_ch_size(hal);
		if (err != 0) {
			return -ENOTSUP;
		}
	}

	/* Enable the Receiver in SPI Peripheral */
	hal_mchp_spi_rx_enable(hal);

	/* Get the Clock Frequency*/
	SPI_MCHP_GET_CLOCK_FREQ(dev, clock_rate);

	/* Select the Baudrate */
	hal_mchp_spi_set_baudrate(hal, config, clock_rate);

	/* Set the SPI in Operation Mode */
	if (SPI_OP_MODE_GET(config->operation) != SPI_OP_MODE_MASTER) {
		err = hal_mchp_spi_slave_mode(hal);
		if (err != 0) {
			return -ENOTSUP;
		}
	} else {
		err = hal_mchp_spi_master_mode(hal);
		if (err != 0) {
			return -ENOTSUP;
		}

		/*Set the CS line*/
		if (data->ctx.num_cs_gpios != 0) {
			err = spi_context_cs_configure_all(&data->ctx);
			if (err < 0) {
				return err;
			}
		} else if (cfg->pcfg->states->pin_cnt == 4) {
			hal_mchp_spi_slave_select_enable(hal);
		} else {
			/* Handled by user */
		}
	}

	/*Set the Data out and Pin out Configuration*/
	if ((config->operation & SPI_MODE_LOOP) != 0U) {

		/* Set MISO and MOSI on the same pad */
		hal_mchp_spi_mode_loopback(hal);
	} else {
		/* Set the Pads */
		hal_mchp_spi_config_pinout(hal);
	}

	/* Set the Clock Polarity */
	if (((config->operation & SPI_MODE_CPOL)) != 0U) {
		hal_mchp_spi_cpol_idle_high(hal);
	} else {
		hal_mchp_spi_cpol_idle_low(hal);
	}

	/* Set the Clock Phase */
	if (((config->operation & SPI_MODE_CPHA)) != 0U) {
		hal_mchp_spi_cpha_trail_edge(hal);
	} else {
		hal_mchp_spi_cpha_lead_edge(hal);
	}

	/* Set the Data Order */
	if ((config->operation & SPI_TRANSFER_LSB) != 0U) {
		hal_mchp_spi_lsb_first(hal);
	} else {
		hal_mchp_spi_msb_first(hal);
	}

	/* Set SPI Duplex Mode */
	if ((config->operation & SPI_HALF_DUPLEX) != 0U) {
		err = hal_mchp_spi_half_duplex_mode(hal);
		if (err != 0) {
			return -ENOTSUP;
		}
	} else {
		err = hal_mchp_spi_full_duplex_mode(hal);
		if (err != 0) {
			return -ENOTSUP;
		}
	}

	/* Enable the SPI Peripheral */
	hal_mchp_spi_enable(hal);

#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_MCHP_INTERRUPT_DRIVEN)
	cfg->irq_config_func(dev);
#endif
#if CONFIG_SPI_MCHP_DMA_DRIVEN
	if (device_is_ready(cfg->spi_dma.dma_dev) != true) {
		return -ENODEV;
	}
	data->dev = dev;
#endif

	/* Save the config into the context */
	data->ctx.config = config;
	return 0;
}

#ifndef CONFIG_SPI_MCHP_INTERRUPT_DRIVEN
/**
 * @brief Check if an SPI transfer is in progress.
 *
 * This function checks the SPI context to determine if there is an ongoing
 * transmit or receive operation.
 *
 * @param data Pointer to the SPI device data structure.
 *
 * @return true if a transfer is in progress, false otherwise.
 */
static bool spi_mchp_transfer_in_progress(spi_mchp_dev_data_t *data)
{
	return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

/**
 * @brief Finish any ongoing writes and drop any remaining read data.
 *
 * This function ensures that any ongoing SPI transmissions are completed
 * and clears any remaining data in the SPI buffer.
 *
 * @param hal Pointer to the hal_mchp_spi_t structure.
 */
static int spi_mchp_finish(const hal_mchp_spi_t *hal)
{
	while (hal_mchp_spi_is_tx_comp(hal) != true) {
		/* Wait until transmit complete */
	};

	/* Clear the data */
	hal_mchp_spi_clr_data(hal);
	return 0;
}

/**
 * @brief Polls the SPI interface for incoming data and handles
 * transmission.
 *
 * This function polls the SPI interface for incoming data, transmits data
 * if available, and updates the SPI context for both transmission and
 * reception.
 *
 * @param hal Pointer to the hardware abstraction layer (HAL) structure for
 * the SPI interface.
 * @param data Pointer to the SPI device data structure.
 */
static int spi_mchp_poll_in(const hal_mchp_spi_t *hal, spi_mchp_dev_data_t *data)
{
	uint8_t tx_data;
	uint8_t rx_data;

	/* Check if there is data to transmit */
	if (spi_context_tx_buf_on(&data->ctx) == true) {
		tx_data = *(uint8_t *)(data->ctx.tx_buf);
	} else {
		tx_data = 0U;
	}

	while (hal_mchp_spi_is_data_empty(hal) != true) {
		/* wait until the SPI data is empty */
	};

	/* Write data to the SPI data */
	hal_mchp_spi_write_data(hal, tx_data);

	/* Update the SPI context for transmission */
	spi_context_update_tx(&data->ctx, 1, 1);

	while (hal_mchp_spi_is_rx_comp(hal) != true) {
		/* Wait for the reception to complete */
	};

	/* Read the received data from the SPI data register */
	rx_data = hal_mchp_spi_read_data(hal);

	/* Check if there is a buffer to store received data */
	if (spi_context_rx_buf_on(&data->ctx) == true) {
		*data->ctx.rx_buf = rx_data;
	}

	/* Update the SPI context for reception */
	spi_context_update_rx(&data->ctx, 1, 1);
	return 0;
}

/**
 * @brief Transmits data quickly over the SPI interface.
 *
 * This function transmits data from the provided buffer over the SPI
 * interface in a fast manner by directly writing each byte to the SPI data
 * register.
 *
 * @param hal Pointer to the hardware abstraction layer (HAL) structure for
 * the SPI interface.
 * @param tx_buf Pointer to the SPI buffer structure containing the data to
 * be transmitted.
 */
static int spi_mchp_fast_tx(const hal_mchp_spi_t *hal, const struct spi_buf *tx_buf)
{
	const uint8_t *tx_data_ptr = tx_buf->buf;
	uint8_t tx_data;
	size_t len = tx_buf->len;
	uint8_t dummy_data = 0U;
	int err;

	/* Transmit each byte in the buffer */
	while (len != 0) {
		if (tx_buf->buf != NULL) {
			tx_data = *tx_data_ptr++;
		} else {
			tx_data = dummy_data;
		}
		while (hal_mchp_spi_is_data_empty(hal) != true) {
			/* Wait until the tramist is complete */
		}
		hal_mchp_spi_write_data(hal, tx_data);
		len--;
	}

	/* Finish the SPI transmission */
	err = spi_mchp_finish(hal);
	return err;
}

/**
 * @brief Receives data quickly over the SPI interface.
 *
 * This function receives data to the provided buffer over the SPI
 * interface in a fast manner by directly reading each byte to the SPI data
 * register.
 *
 * @param hal Pointer to the hardware abstraction layer (HAL) structure for
 * the SPI interface.
 * @param rx_buf Pointer to the SPI buffer structure to which the data is
 * received.
 */
static int spi_mchp_fast_rx(const hal_mchp_spi_t *hal, const struct spi_buf *rx_buf)
{
	uint8_t *rx_data_ptr = rx_buf->buf;
	size_t len = rx_buf->len;
	uint8_t dummy_data = 0U;
	int err;

	if (len == 0) {
		return 0;
	}

	while (len != 0) {

		/* Write a dummy data to receive data */
		hal_mchp_spi_write_data(hal, dummy_data);
		len--;

		while (hal_mchp_spi_is_rx_comp(hal) != true) {
			/* Wait for completion, and read */
		};

		if (rx_buf->buf != NULL) {
			*rx_data_ptr = hal_mchp_spi_read_data(hal);
			rx_data_ptr++;
		} else {
			(void)hal_mchp_spi_read_data(hal);
		}
	}
	/* Finish the SPI transmission */
	err = spi_mchp_finish(hal);
	return err;
}

/**
 * @brief Perform fast SPI transmission and reception.
 *
 * This function transmits and receives data buffers of the same length
 * over SPI using the Microchip hardware abstraction layer.
 *
 * @param hal Pointer to the Microchip SPI hardware abstraction structure.
 * @param tx_buf Pointer to the SPI transmit buffer.
 * @param rx_buf Pointer to the SPI receive buffer.
 */
static int spi_mchp_fast_txrx(const hal_mchp_spi_t *hal, const struct spi_buf *tx_buf,
			      const struct spi_buf *rx_buf)
{
	const uint8_t *tx_data_ptr = tx_buf->buf;
	uint8_t *rx_data_ptr = rx_buf->buf;
	size_t len = rx_buf->len;
	uint8_t dummy_data = 0U;
	int err;

	if (len == 0) {
		return 0;
	}

	while (len > 0) {
		/* Send the next byte */
		if (tx_data_ptr != NULL) {
			hal_mchp_spi_write_data(hal, *tx_data_ptr);
			tx_data_ptr++;
		} else {
			hal_mchp_spi_write_data(hal, dummy_data);
		}

		/* Wait for completion */
		while (hal_mchp_spi_is_rx_comp(hal) != true) {
			/* Wait for completion */
		};

		/* Read received data */
		if (rx_data_ptr != NULL) {
			*rx_data_ptr = hal_mchp_spi_read_data(hal);
			rx_data_ptr++;
		} else {
			(void)hal_mchp_spi_read_data(hal);
		}
		len--;
	}
	/* Finish the SPI transmission */
	err = spi_mchp_finish(hal);
	return err;
}

/**
 * @brief Perform fast SPI transceive operation.
 *
 * This function transmits and receives data using SPI, ensuring that
 * overlapping TX and RX buffers have the same length. It processes
 * multiple buffers efficiently using Microchip's SPI HAL.
 *
 * @param dev Pointer to the SPI device.
 * @param config Pointer to the SPI configuration structure.
 * @param tx_bufs Pointer to the set of SPI transmit buffers.
 * @param rx_bufs Pointer to the set of SPI receive buffers.
 */
static int spi_mchp_fast_transceive(const struct device *dev, const struct spi_config *config,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs)
{
	const spi_mchp_dev_config_t *cfg = dev->config;
	const hal_mchp_spi_t *hal = &cfg->hal;
	size_t tx_count = 0;
	size_t rx_count = 0;
	const struct spi_buf *tx_data_ptr = NULL;
	const struct spi_buf *rx_data_ptr = NULL;
	int err;

	if (tx_bufs != NULL) {
		tx_data_ptr = tx_bufs->buffers;
		tx_count = tx_bufs->count;
	}

	if (rx_bufs != NULL) {
		rx_data_ptr = rx_bufs->buffers;
		rx_count = rx_bufs->count;
	} else {
		rx_data_ptr = NULL;
	}

	while (tx_count != 0 && rx_count != 0) {
		/* This function is called only if the count is equal*/
		err = spi_mchp_fast_txrx(hal, tx_data_ptr, rx_data_ptr);

		tx_data_ptr++;
		tx_count--;
		rx_data_ptr++;
		rx_count--;
	}

	/* Handle remaining transmit buffers */
	while (tx_count > 0) {
		err = spi_mchp_fast_tx(hal, tx_data_ptr);
		tx_data_ptr++;
		tx_count--;
	}

	/* Handle remaining receive buffers */
	while (rx_count > 0) {
		err = spi_mchp_fast_rx(hal, rx_data_ptr);
		rx_data_ptr++;
		rx_count--;
	}

	return err;
}

/**
 * @brief Check if SPI transmit and receive buffers have the same length.
 *
 * This function verifies whether all corresponding buffers in the
 * transmit and receive buffer sets have matching lengths.
 *
 * @param tx_bufs Pointer to the SPI transmit buffer set.
 * @param rx_bufs Pointer to the SPI receive buffer set.
 *
 * @return true if all corresponding TX and RX buffers have the same
 * length, false otherwise.
 */
static bool spi_mchp_is_same_len(const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs)
{
	const struct spi_buf *tx_data_ptr = NULL;
	const struct spi_buf *rx_data_ptr = NULL;
	size_t tx_count = 0;
	size_t rx_count = 0;

	if (tx_bufs != NULL) {
		tx_data_ptr = tx_bufs->buffers;
		tx_count = tx_bufs->count;
	}

	if (rx_bufs != NULL) {
		rx_data_ptr = rx_bufs->buffers;
		rx_count = rx_bufs->count;
	}

	while (tx_count != 0 && rx_count != 0) {
		/* Compare the length of each corresponding TX and RX buffer */
		if (tx_data_ptr->len != rx_data_ptr->len) {
			return false;
		}

		tx_data_ptr++;
		tx_count--;
		rx_data_ptr++;
		rx_count--;
	}

	return true;
}
#endif /* CONFIG_SPI_MCHP_INTERRUPT_DRIVEN*/

#if defined(CONFIG_SPI_MCHP_INTERRUPT_DRIVEN) || (CONFIG_SPI_MCHP_INTERRUPT_DRIVEN_ASYNC)
/**
 * @brief Perform SPI transceive operation using interrupts.
 *
 * This function initializes and starts an SPI transaction in
 * interrupt-driven mode. It sets up the SPI buffers, writes the first
 * byte, and enables relevant interrupts to handle data transfer
 * asynchronously.
 *
 * @param dev Pointer to the SPI device.
 * @param config Pointer to the SPI configuration structure.
 * @param tx_bufs Pointer to the set of SPI transmit buffers.
 * @param rx_bufs Pointer to the set of SPI receive buffers.
 *
 * @return 0 on success.
 */
static int spi_mchp_transceive_interrupt(const struct device *dev, const struct spi_config *config,
					 const struct spi_buf_set *tx_bufs,
					 const struct spi_buf_set *rx_bufs)
{
	const spi_mchp_dev_config_t *cfg = dev->config;
	const hal_mchp_spi_t *hal = &cfg->hal;
	spi_mchp_dev_data_t *const data = dev->data;
	uint8_t tx_data;

	/* Setup SPI buffers */
	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	/* Prepare first byte for transmission */
	if (spi_context_tx_buf_on(&data->ctx) == true) {
		tx_data = *(uint8_t *)(data->ctx.tx_buf);
	} else {
		tx_data = 0U;
	}

	/* Clear the Data Register */
	hal_mchp_spi_clr_data(hal);

	/* Get the dummysize */
	if ((data->ctx.rx_len) > (data->ctx.tx_len)) {
		data->dummysize = (data->ctx.rx_len) - (data->ctx.tx_len);
	}

	/* Write first data byte to the SPI data register */
	spi_context_update_tx(&data->ctx, 1, 1);
	hal_mchp_spi_write_data(hal, tx_data);

	/* Enable SPI interrupts for RX, TX completion, and data empty events */
	if (data->ctx.rx_len > 0) {
		hal_mchp_spi_enable_rxc_interrupt(hal);
	} else {
		hal_mchp_spi_enable_data_empty_interrupt(hal);
	}

#if defined(CONFIG_SPI_MCHP_INTERRUPT_DRIVEN)
	/* Wait for transaction to complete */
	k_sem_take(&data->ctx.sync, K_FOREVER);
#endif
	return 0;
}
#endif

/**
 * @brief Perform synchronous SPI transceive operation.
 *
 * This function performs an SPI transfer in a synchronous manner. It locks
 * the SPI context, configures the device, and selects the appropriate
 * transmission mode (interrupt-driven or polling). If the transfer is
 * interrupt-driven, it delegates to `spi_mchp_transceive_interrupt()`.
 * Otherwise, it handles the transfer using optimized routines or polling.
 *
 * @param dev Pointer to the SPI device.
 * @param config Pointer to the SPI configuration structure.
 * @param tx_bufs Pointer to the set of SPI transmit buffers.
 * @param rx_bufs Pointer to the set of SPI receive buffers.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int spi_mchp_transceive_sync(const struct device *dev, const struct spi_config *config,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs)
{
	const spi_mchp_dev_config_t *cfg = dev->config;
	const hal_mchp_spi_t *hal = &cfg->hal;
	spi_mchp_dev_data_t *data = dev->data;
	int err;

	ARG_UNUSED(hal);
	/* Lock SPI context */
	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	/* Configure SPI device */
	err = spi_mchp_configure(dev, config);
	if (err != 0) {
		/* Release SPI context */
		spi_context_release(&data->ctx, err);
		return err;
	}

	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
		/* Assert chip select (CS) */
		spi_context_cs_control(&data->ctx, true);
	}

#if CONFIG_SPI_MCHP_INTERRUPT_DRIVEN
	/* Use interrupt-driven SPI transceive */
	err = spi_mchp_transceive_interrupt(dev, config, tx_bufs, rx_bufs);
#else
	/* Use optimized fast path if TX and RX buffer lengths match */
	if (spi_mchp_is_same_len(tx_bufs, rx_bufs) == true) {
		spi_mchp_fast_transceive(dev, config, tx_bufs, rx_bufs);
	} else {
		/* Setup SPI buffers and process using polling */
		spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

		do {
			err = spi_mchp_poll_in(hal, data);
		} while (spi_mchp_transfer_in_progress(data) && err == 0);
	}
#endif

	/* Deassert chip select (CS) */
	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
		/* Assert chip select (CS) */
		spi_context_cs_control(&data->ctx, false);
	}

	/* Release SPI context */
	spi_context_release(&data->ctx, err);
	return err;
}

#if CONFIG_SPI_ASYNC
#if CONFIG_SPI_MCHP_DMA_DRIVEN
static void spi_mchp_dma_rx_done(const struct device *dma_dev, void *arg, uint32_t id,
				 int error_code);
/**
 * @brief Load data into the DMA controller for SPI TX transfer.
 *
 * This function configures and starts a DMA transaction to transfer data
 * from memory to the SPI peripheral. If no TX buffer is provided, it sends
 * dummy bytes (0U) instead.
 *
 * @param dev Pointer to the SPI device structure.
 * @param buf Pointer to the TX data buffer (can be NULL).
 * @param len Length of the data to be transmitted.
 *
 * @retval 0 on success.
 * @retval Negative error code on failure.
 */
static int spi_mchp_dma_tx_load(const struct device *dev, const uint8_t *buf, size_t len)
{
	const spi_mchp_dev_config_t *cfg = dev->config;
	const hal_mchp_spi_t *hal = &cfg->hal;

	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};
	int retval;

	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->spi_dma.tx_dma_request;

	dma_blk.block_size = len;

	if (buf != NULL) {
		dma_blk.source_address = (uint32_t)buf;
	} else {
		static const uint8_t dummy_data;

		dma_blk.source_address = (uint32_t)&dummy_data;
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	dma_blk.dest_address = (uint32_t)(hal_mchp_spi_get_dma_dest_addr(hal));
	dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	retval = dma_config(cfg->spi_dma.dma_dev, cfg->spi_dma.tx_dma_channel, &dma_cfg);

	if (retval != 0) {
		return retval;
	}

	return dma_start(cfg->spi_dma.dma_dev, cfg->spi_dma.tx_dma_channel);
}

/**
 * @brief Configure and start DMA for SPI RX (receive).
 *
 * This function sets up the DMA configuration for receiving SPI data and
 * starts the DMA transfer. If a valid buffer is provided, the received
 * data is stored there. Otherwise, a dummy buffer is used.
 *
 * @param dev Pointer to the SPI device.
 * @param buf Pointer to the receive buffer, or NULL if the received data
 * is not needed.
 * @param len Length of data to be received.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int spi_mchp_dma_rx_load(const struct device *dev, uint8_t *buf, size_t len)
{
	const spi_mchp_dev_config_t *cfg = dev->config;
	const hal_mchp_spi_t *hal = &cfg->hal;
	spi_mchp_dev_data_t *data = dev->data;

	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};
	int retval;

	/* Configure DMA settings */
	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.user_data = data;
	dma_cfg.dma_callback = spi_mchp_dma_rx_done;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->spi_dma.rx_dma_request;

	/* Set block size */
	dma_blk.block_size = len;

	/* Configure destination address */
	if (buf != NULL) {
		dma_blk.dest_address = (uint32_t)buf;
	} else {
		/* Use a static dummy variable if no buffer is provided */
		static uint8_t dummy_data;

		dma_blk.dest_address = (uint32_t)&dummy_data;
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	/* Configure source address */
	dma_blk.source_address = (uint32_t)(hal_mchp_spi_get_dma_src_addr(hal));
	dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	/* Apply DMA configuration */
	retval = dma_config(cfg->spi_dma.dma_dev, cfg->spi_dma.rx_dma_channel, &dma_cfg);
	if (retval != 0) {
		return retval;
	}

	/* Start DMA transfer */
	return dma_start(cfg->spi_dma.dma_dev, cfg->spi_dma.rx_dma_channel);
}

/**
 * @brief Determines the next SPI DMA transfer segment length.
 *
 * This function selects the shortest valid buffer length from TX and RX
 * buffers to determine the next DMA transfer segment. It ensures that
 * the segment length does not exceed the maximum allowable DMA transfer
 * size.
 *
 * @param dev Pointer to the SPI device structure.
 *
 * @retval true if there is a valid segment to transfer.
 * @retval false if there is no data left to transfer.
 */
static bool spi_mchp_dma_select_segment(const struct device *dev)
{
	spi_mchp_dev_data_t *data = dev->data;
	uint32_t segment_len;

	/* Pick the shorter buffer of ones that have an actual length */
	if (data->ctx.rx_len != 0) {
		segment_len = data->ctx.rx_len;
		if (data->ctx.tx_len != 0) {
			segment_len = MIN(segment_len, data->ctx.tx_len);
		}
	} else {
		segment_len = data->ctx.tx_len;
	}

	if (segment_len == 0) {
		return false;
	}

	/* Ensure the segment length does not exceed the max allowed value
	 */
	segment_len = MIN(segment_len, 65535);

	data->dma_segment_len = segment_len;
	return true;
}

/**
 * @brief Advances the SPI DMA buffers for the next segment.
 *
 * This function loads the next DMA segment into the RX and TX buffers.
 * It first loads the RX buffer to ensure data can be received before
 * initiating transmission. Then, it loads the TX buffer, which starts
 * the SPI clocking process.
 *
 * @param dev Pointer to the SPI device structure.
 *
 * @retval 0 on success.
 * @retval -EINVAL if the DMA segment length is zero.
 * @retval Negative error code if DMA loading fails.
 */
static int spi_mchp_dma_setup_buffers(const struct device *dev)
{
	spi_mchp_dev_data_t *data = dev->data;
	int retval;

	if (data->dma_segment_len == 0) {
		return -EINVAL;
	}

	/* Load receive buffer first to prepare for incoming data */
	if (data->ctx.rx_len != 0U) {
		retval = spi_mchp_dma_rx_load(dev, data->ctx.rx_buf, data->dma_segment_len);
	} else {
		retval = spi_mchp_dma_rx_load(dev, NULL, data->dma_segment_len);
	}

	if (retval != 0) {
		return retval;
	}

	/* Load transmit buffer, which starts SPI bus clocking */
	if (data->ctx.tx_len != 0U) {
		retval = spi_mchp_dma_tx_load(dev, data->ctx.tx_buf, data->dma_segment_len);
	} else {
		retval = spi_mchp_dma_tx_load(dev, NULL, data->dma_segment_len);
	}

	if (retval != 0) {
		return retval;
	}

	return 0;
}

/**
 * @brief SPI DMA RX completion callback.
 *
 * This function is called when a DMA transfer for SPI RX completes.
 * It updates the SPI context with the received data, checks if more
 * data segments need to be transferred, and either continues the
 * transfer or completes the SPI transaction.
 *
 * @param dma_dev Pointer to the DMA device structure.
 * @param arg Pointer to the SPI device data structure.
 * @param id DMA transaction ID (unused).
 * @param error_code Error code from the DMA transfer (unused).
 */
static void spi_mchp_dma_rx_done(const struct device *dma_dev, void *arg, uint32_t id,
				 int error_code)
{
	spi_mchp_dev_data_t *data = arg;
	const struct device *dev = data->dev;
	const spi_mchp_dev_config_t *cfg = dev->config;
	int retval;

	ARG_UNUSED(id);
	ARG_UNUSED(error_code);

	/* Update TX and RX context with the completed DMA segment */
	spi_context_update_tx(&data->ctx, 1, data->dma_segment_len);
	spi_context_update_rx(&data->ctx, 1, data->dma_segment_len);

	/* Check if more segments need to be transferred */
	if (spi_mchp_dma_select_segment(dev) == false) {
		/* Transmission complete */
		spi_context_cs_control(&data->ctx, false);
		spi_context_complete(&data->ctx, dev, 0);
		return;
	}

	/* Load the next DMA segment */
	retval = spi_mchp_dma_setup_buffers(dev);
	if (retval != 0) {
		/* Stop DMA and terminate the SPI transaction in case of failure */
		dma_stop(cfg->spi_dma.dma_dev, cfg->spi_dma.tx_dma_channel);
		dma_stop(cfg->spi_dma.dma_dev, cfg->spi_dma.rx_dma_channel);
		spi_context_cs_control(&data->ctx, false);
		spi_context_complete(&data->ctx, dev, retval);
		return;
	}
}
#endif /* CONFIG_SPI_MCHP_DMA_DRIVEN */

/**
 * @brief Perform an asynchronous SPI transceive operation.
 *
 * This function initiates an asynchronous SPI data transfer using DMA.
 * It configures the SPI peripheral, sets up DMA buffers, and starts
 * the transmission. The function returns immediately, and the transfer
 * completes via a callback.
 *
 * @param dev Pointer to the SPI device structure.
 * @param config Pointer to the SPI configuration structure.
 * @param tx_bufs Pointer to the SPI buffer set for transmission.
 * @param rx_bufs Pointer to the SPI buffer set for reception.
 * @param spi_callback Callback function to be called on completion.
 * @param userdata Pointer to user data to be passed to the callback.
 *
 * @retval 0 if the transfer is successfully initiated.
 * @retval -ENOTSUP if DMA channels are not properly configured.
 * @retval Other negative error codes in case of configuration or DMA setup
 * failures.
 */
static int spi_mchp_transceive_async(const struct device *dev, const struct spi_config *config,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs, spi_callback_t spi_callback,
				     void *userdata)
{
	const spi_mchp_dev_config_t *cfg = dev->config;
	spi_mchp_dev_data_t *data = dev->data;
	int retval;

	ARG_UNUSED(cfg);
/*
 * Transmit clocks the output, and we use receive to
 * determine when the transmit is done, so we
 * always need both TX and RX DMA channels.
 */
#if CONFIG_SPI_MCHP_DMA_DRIVEN
	if (cfg->spi_dma.tx_dma_channel == 0xFF || cfg->spi_dma.rx_dma_channel == 0xFF) {
		return -ENOTSUP;
	}
#endif

	/* Lock SPI context for exclusive access */
	spi_context_lock(&data->ctx, true, spi_callback, userdata, config);

	/* Configure SPI based on the provided configuration */
	retval = spi_mchp_configure(dev, config);
	if (retval != 0) {
		/* Release SPI context lock */
		spi_context_release(&data->ctx, retval);
		return retval;
	}

	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
		/* Assert chip select */
		spi_context_cs_control(&data->ctx, true);
	}

	/* Set up TX and RX buffers */
	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

/* Prepare and start DMA transfers */
#if CONFIG_SPI_MCHP_DMA_DRIVEN
	spi_mchp_dma_select_segment(dev);
	retval = spi_mchp_dma_setup_buffers(dev);
#else
	retval = spi_mchp_transceive_interrupt(dev, config, tx_bufs, rx_bufs);
#endif

	if (retval != 0) {
		/* Stop DMA transfers in case of failure */
#if CONFIG_SPI_MCHP_DMA_DRIVEN
		dma_stop(cfg->spi_dma.dma_dev, cfg->spi_dma.tx_dma_channel);
		dma_stop(cfg->spi_dma.dma_dev, cfg->spi_dma.rx_dma_channel);
#endif

		/* Deassert chip select */
		if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
			spi_context_cs_control(&data->ctx, false);
		}

		/* Release SPI context lock */
		spi_context_release(&data->ctx, retval);
	} else {
		retval = 0;
	}
	return retval;
}
#endif /* CONFIG_SPI_ASYNC */

/**
 * @brief Release the SPI bus.
 *
 * This function unlocks the SPI context, allowing other transactions to
 * proceed. It is typically called when a SPI transaction is complete or
 * aborted.
 *
 * @param dev Pointer to the SPI device structure.
 * @param config Pointer to the SPI configuration structure (unused).
 *
 * @retval 0 Always returns 0 as this function does not fail.
 */
static int spi_mchp_release(const struct device *dev, const struct spi_config *config)
{
	spi_mchp_dev_data_t *data = dev->data;

	/* Unlock the SPI context to allow other operations */
	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}
#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_MCHP_INTERRUPT_DRIVEN)

/**
 * SPI interrupt service routine for Microchip SPI controller.
 *
 * This ISR handles SPI data transmission and reception. It reads incoming
 * data, writes outgoing data when available, and manages interrupts based
 * on the SPI transfer state. It also releases semaphores to notify waiting
 * threads when using interrupt-driven or asynchronous modes.
 *
 * @param dev Pointer to the SPI device structure.
 *
 * @note This function is enabled only if CONFIG_SPI_ASYNC or
 *       CONFIG_SPI_MCHP_INTERRUPT_DRIVEN is defined.
 */
static void spi_mchp_isr(const struct device *dev)
{
	/* Device driver runtime data */
	spi_mchp_dev_data_t *data = dev->data;

	/* Device driver configuration */
	const spi_mchp_dev_config_t *cfg = dev->config;

	/* HAL structure for SPI registers */
	const hal_mchp_spi_t *hal = &cfg->hal;

	/* Dummy data for padding SPI transactions */
	uint8_t dummy_data = 0U;

	/* SPI status flag for last byte. */
	bool last_byte = false;

	/* Transmit and receive data placeholders */
	uint8_t tx_data = 0U;
	uint8_t rx_data = 0U;

	/* Check if the transmit buffer is empty and send the next byte */
	if (hal_mchp_spi_is_interrupt_set(hal) == true) {
		/* Check if data is available in the receive buffer */
		if (hal_mchp_spi_is_rx_comp(hal) == true) {
			if (spi_context_rx_buf_on(&data->ctx) == true) {
				rx_data = hal_mchp_spi_read_data(hal);
				*(uint8_t *)(data->ctx.rx_buf) = rx_data;
				spi_context_update_rx(&data->ctx, 1, 1);
			}
		}
		if (hal_mchp_spi_is_data_empty(hal) == true) {
			hal_mchp_spi_disable_data_empty_interrupt(hal);
			if (spi_context_tx_on(&data->ctx) == true) {
				tx_data = *(uint8_t *)(data->ctx.tx_buf);
				hal_mchp_spi_write_data(hal, tx_data);
				spi_context_update_tx(&data->ctx, 1, 1);
			} else if (data->dummysize > 0) {
				hal_mchp_spi_write_data(hal, dummy_data);
				data->dummysize--;
			} else {
				/* Do Nothing */
			}
			if ((data->dummysize == 0) && (spi_context_tx_on(&data->ctx) == false)) {
				last_byte = true;
			} else if (spi_context_rx_on(&data->ctx) == false) {
				hal_mchp_spi_enable_data_empty_interrupt(hal);
				hal_mchp_spi_disable_rxc_interrupt(hal);
			} else {
				/* Do Nothing */
			}
		}
		if ((hal_mchp_spi_is_tx_comp(hal) == true) && (last_byte == true)) {
			if (spi_context_rx_on(&data->ctx) == false) {
				hal_mchp_spi_disable_rxc_interrupt(hal);
				hal_mchp_spi_disable_txc_interrupt(hal);
				hal_mchp_spi_disable_data_empty_interrupt(hal);
				last_byte = false;
				if (spi_context_is_slave(&data->ctx) == true) {
					/* Control chip select for SPI slave mode */
					spi_context_cs_control(&data->ctx, false);
				}
#if CONFIG_SPI_MCHP_INTERRUPT_DRIVEN
				/* Release the semaphore to unblock waiting threads */
				k_sem_give(&data->ctx.sync);
#elif CONFIG_SPI_MCHP_INTERRUPT_DRIVEN_ASYNC
				/* Release the semaphore and invoke the callback */
				spi_context_complete(&data->ctx, dev, 0);
#endif
			}
		}
		if (last_byte == true) {
			hal_mchp_spi_enable_txc_interrupt(hal);
		}
	}
}
#endif /* CONFIG_SPI_ASYNC || CONFIG_SPI_MCHP_INTERRUPT_DRIVEN */

/**
 * @brief Initialize the Microchip SPI controller.
 *
 * This function configures the SPI hardware by enabling the clock,
 * disabling interrupts, setting the pin control state, and ensuring
 * the SPI device is ready for transactions.
 *
 * The actual SPI configuration and enabling occur during transceive calls.
 *
 * @param dev Pointer to the SPI device structure.
 *
 * @return 0 on success, negative error code on failure.
 */
static int spi_mchp_init(const struct device *dev)
{
	int err;
	const spi_mchp_dev_config_t *cfg = dev->config;
	const hal_mchp_spi_t *hal = &cfg->hal;
	spi_mchp_dev_data_t *const data = dev->data;

	/* Enable the SPI clock */
	SPI_MCHP_ENABLE_CLOCK(dev);

	/* Disable all SPI interrupts initially */
	hal_mchp_spi_disable_interrupts(hal);

	/* Apply the default pin configuration */
	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	/* Ensure the SPI context is unlocked */
	spi_context_unlock_unconditionally(&data->ctx);

	/* SPI will be configured and enabled during transceive */
	return 0;
}

/**
 * @brief SPI device API structure for Microchip SPI driver.
 *
 * This structure defines the function pointers for SPI operations,
 * including synchronous and optional asynchronous transceive functions.
 */
static const struct spi_driver_api spi_mchp_driver_api = {
	/* Synchronous transceive function */
	.transceive = spi_mchp_transceive_sync,

#if CONFIG_SPI_ASYNC
	/* Asynchronous transceive function */
	.transceive_async = spi_mchp_transceive_async,
#endif

#if CONFIG_SPI_RTIO
	/* RTIO-based IO submission */
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif

	/* Release the SPI bus */
	.release = spi_mchp_release,
};

/**
 * @brief Do the peripheral interrupt related configuration
 */
#if CONFIG_SPI_MCHP_INTERRUPT_DRIVEN || CONFIG_SPI_ASYNC

/**
 * @brief Macro to configure and enable SPI IRQ
 *
 * @param n Instance number
 * @param m IRQ index
 */
#define MCHP_SPI_IRQ_CONNECT(n, m)                                                                 \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, m, irq), DT_INST_IRQ_BY_IDX(n, m, priority),     \
			    spi_mchp_isr, DEVICE_DT_INST_GET(n), 0);                               \
		irq_enable(DT_INST_IRQ_BY_IDX(n, m, irq));                                         \
	} while (false)

/**
 * @brief Macro to declare an IRQ handler function
 *
 * @param n Instance number
 */
#define SPI_MCHP_IRQ_HANDLER_DECL(n) static void spi_mchp_irq_config_##n(const struct device *dev)
#define SPI_MCHP_IRQ_HANDLER_FUNC(n) .irq_config_func = spi_mchp_irq_config_##n,
#else
#define SPI_MCHP_IRQ_HANDLER_DECL(n)
#define SPI_MCHP_IRQ_HANDLER_FUNC(n)
#endif /* CONFIG_SPI_MCHP_INTERRUPT_DRIVEN || CONFIG_SPI_ASYNC */

#if CONFIG_SPI_MCHP_DMA_DRIVEN
/**
 * @brief Macro to define DMA channels for SPI
 *
 * @param n Instance number
 */
#define SPI_MCHP_DMA_CHANNELS(n)                                                                   \
	.spi_dma.dma_dev = DEVICE_DT_GET(MCHP_DT_INST_DMA_CTLR(n, tx)),                            \
	.spi_dma.tx_dma_request = MCHP_DT_INST_DMA_TRIGSRC(n, tx),                                 \
	.spi_dma.tx_dma_channel = MCHP_DT_INST_DMA_CHANNEL(n, tx),                                 \
	.spi_dma.rx_dma_request = MCHP_DT_INST_DMA_TRIGSRC(n, rx),                                 \
	.spi_dma.rx_dma_channel = MCHP_DT_INST_DMA_CHANNEL(n, rx),
#else
#define SPI_MCHP_DMA_CHANNELS(n)
#endif /* CONFIG_SPI_MCHP_DMA_DRIVEN */

/**
 * @brief Initial Configuration for the SPI Peripheral
 */

/**
 * @brief Macro to define SPI device configuration
 *
 * @param n Instance number
 */
#define SPI_MCHP_CONFIG_DEFN(n)                                                                    \
	static const spi_mchp_dev_config_t spi_mchp_config_##n = {                                 \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		SPI_MCHP_HAL_DEFN(n) SPI_MCHP_IRQ_HANDLER_FUNC(n) SPI_MCHP_DMA_CHANNELS(n)         \
			SPI_MCHP_CLOCK_DEFN(n)}

/**
 * @brief Macro to initialize SPI device
 *
 * @param n Instance number
 */
#define SPI_MCHP_DEVICE_INIT(n)                                                                    \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	SPI_MCHP_IRQ_HANDLER_DECL(n);                                                              \
	SPI_MCHP_CONFIG_DEFN(n);                                                                   \
	static spi_mchp_dev_data_t spi_mchp_data_##n = {                                           \
		SPI_CONTEXT_INIT_LOCK(spi_mchp_data_##n, ctx),                                     \
		SPI_CONTEXT_INIT_SYNC(spi_mchp_data_##n, ctx),                                     \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             \
	DEVICE_DT_INST_DEFINE(n, spi_mchp_init, NULL, &spi_mchp_data_##n, &spi_mchp_config_##n,    \
			      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, &spi_mchp_driver_api);        \
	SPI_MCHP_IRQ_HANDLER(n)

/**
 * @brief Initialize SPI devices for all compatible instances
 */
DT_INST_FOREACH_STATUS_OKAY(SPI_MCHP_DEVICE_INIT)
