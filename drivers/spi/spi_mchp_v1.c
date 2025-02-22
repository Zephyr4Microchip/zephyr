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

	/* Clock device */
	const struct device *clock_dev;

#ifdef CONFIG_SPI_MCHP_DMA_DRIVEN
	/* DMA device */
	const struct device *dma_dev;
	mchp_spi_dma_t spi_dma;
#endif

#if CONFIG_SPI_ASYNC
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
	/* SPI status flag for last byte. */
	uint8_t last_byte;
#endif
#ifdef CONFIG_SPI_MCHP_DMA_DRIVEN
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
	if (spi_context_configured(&data->ctx, config)) {
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
		/* Slave mode is not implemented */
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
		err = spi_context_cs_configure_all(&data->ctx);
		if (err < 0) {
			return err;
		}
	}

	/*Set the Data out and Pin out Configuration*/
	if ((config->operation & SPI_MODE_LOOP) != 0U) {

		/* Set MISO and MOSI on the same pad */
		hal_mchp_spi_mode_loop(hal);
	} else {
		/* Set the Pads */
		hal_mchp_spi_config_pinout(hal);
	}

	/* Set the Clock Polarity */
	if (((config->operation & SPI_MODE_CPOL))) {
		hal_mchp_spi_cpol_idle_high(hal);
	} else {
		hal_mchp_spi_cpol_idle_low(hal);
	}

	/* Set the Clock Phase */
	if (((config->operation & SPI_MODE_CPHA))) {
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
	if (config->operation & SPI_HALF_DUPLEX) {
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

#if CONFIG_SPI_ASYNC
	data->last_byte = false;
#if CONFIG_SPI_MCHP_INTERRUPT_DRIVEN
	cfg->irq_config_func(dev);
#endif
#if CONFIG_SPI_MCHP_DMA_DRIVEN
	if (!device_is_ready(cfg->dma_dev)) {
		return -ENODEV;
	}
	data->dev = dev;
#endif
#endif

	/* Save the config into the context */
	data->ctx.config = config;
	return 0;
}

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
static void spi_mchp_poll_in(const hal_mchp_spi_t *hal, spi_mchp_dev_data_t *data)
{
	uint8_t tx;
	uint8_t rx;

	/* Check if there is data to transmit */
	if (spi_context_tx_buf_on(&data->ctx)) {
		tx = *(uint8_t *)(data->ctx.tx_buf);
	} else {
		tx = 0U;
	}

	/* Indicate that the SPI data register is empty */
	hal_mchp_spi_data_empty(hal);

	/* Write data to the SPI data register */
	hal_mchp_spi_write_data(hal, tx);

	/* Update the SPI context for transmission */
	spi_context_update_tx(&data->ctx, 1, 1);

	/* Wait for the reception to complete */
	hal_mchp_spi_rx_comp(hal);

	/* Read the received data from the SPI data register */
	rx = hal_mchp_spi_read_data(hal);

	/* Check if there is a buffer to store received data */
	if (spi_context_rx_buf_on(&data->ctx)) {
		*data->ctx.rx_buf = rx;
	}

	/* Update the SPI context for reception */
	spi_context_update_rx(&data->ctx, 1, 1);
}

/**
 * @brief Finish any ongoing writes and drop any remaining read data.
 *
 * This function ensures that any ongoing SPI transmissions are completed
 * and clears any remaining data in the SPI buffer.
 *
 * @param hal Pointer to the hal_mchp_spi_t structure.
 */
static void spi_mchp_finish(const hal_mchp_spi_t *hal)
{
	/* Wait until transmit complete */
	hal_mchp_spix_comp(hal);

	/* Clear the data */
	hal_mchp_spi_clr_data(hal);
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
static void spi_mchp_fast_tx(const hal_mchp_spi_t *hal, const struct spi_buf *tx_buf)
{
	const uint8_t *p = tx_buf->buf;
	const uint8_t *pend = (uint8_t *)tx_buf->buf + tx_buf->len;
	uint8_t ch;

	/* Transmit each byte in the buffer */
	while (p != pend) {
		ch = *p++;
		hal_mchp_spi_data_empty(hal);
		hal_mchp_spi_write_data(hal, ch);
	}

	/* Finish the SPI transmission */
	spi_mchp_finish(hal);
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
static void spi_mchp_fast_rx(const hal_mchp_spi_t *hal, const struct spi_buf *rx_buf)
{
	uint8_t *rx = rx_buf->buf;
	int len = rx_buf->len;

	if (len <= 0) {
		return;
	}

	while (len) {
		static const uint8_t dummy_data = 0xFF;

		/* Write a dummy data to receive data */
		hal_mchp_spi_write_data(hal, dummy_data);
		len--;

		/* Wait for completion, and read */
		hal_mchp_spi_rx_comp(hal);

		*rx++ = hal_mchp_spi_read_data(hal);
	}
	/* Finish the SPI transmission */
	spi_mchp_finish(hal);
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
static void spi_mchp_fast_txrx(const hal_mchp_spi_t *hal, const struct spi_buf *tx_buf,
			       const struct spi_buf *rx_buf)
{
	const uint8_t *tx = tx_buf->buf;
	const uint8_t *txend = (uint8_t *)tx_buf->buf + tx_buf->len;
	uint8_t *rx = rx_buf->buf;
	size_t len = rx_buf->len;

	if (len == 0) {
		return;
	}

	while (tx != txend) {
		/* Send the next byte */
		hal_mchp_spi_write_data(hal, *tx++);

		/* Wait for completion */
		hal_mchp_spi_rx_comp(hal);

		/* Read received data */
		*rx++ = hal_mchp_spi_read_data(hal);
	}
	/* Finish the SPI transmission */
	spi_mchp_finish(hal);
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
static void spi_mchp_fast_transceive(const struct device *dev, const struct spi_config *config,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs)
{
	const spi_mchp_dev_config_t *cfg = dev->config;
	const hal_mchp_spi_t *hal = &cfg->hal;
	size_t tx_count = 0;
	size_t rx_count = 0;

	const struct spi_buf *tx = NULL;
	const struct spi_buf *rx = NULL;

	if (tx_bufs) {
		tx = tx_bufs->buffers;
		tx_count = tx_bufs->count;
	}

	if (rx_bufs) {
		rx = rx_bufs->buffers;
		rx_count = rx_bufs->count;
	} else {
		rx = NULL;
	}

	while (tx_count != 0 && rx_count != 0) {
		/* Process receive-only buffer */
		if (tx->buf == NULL) {
			spi_mchp_fast_rx(hal, rx);
		}
		/* Process transmit-only buffer */
		else if (rx->buf == NULL) {
			spi_mchp_fast_tx(hal, tx);
		}
		/* Process simultaneous transmit and receive */
		else {
			spi_mchp_fast_txrx(hal, tx, rx);
		}

		tx++;
		tx_count--;
		rx++;
		rx_count--;
	}

	/* Handle remaining transmit buffers */
	for (; tx_count != 0; tx_count--) {
		spi_mchp_fast_tx(hal, tx++);
	}

	/* Handle remaining receive buffers */
	for (; rx_count != 0; rx_count--) {
		spi_mchp_fast_rx(hal, rx++);
	}
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
	const struct spi_buf *tx = NULL;
	const struct spi_buf *rx = NULL;
	size_t tx_count = 0;
	size_t rx_count = 0;

	if (tx_bufs) {
		tx = tx_bufs->buffers;
		tx_count = tx_bufs->count;
	}

	if (rx_bufs) {
		rx = rx_bufs->buffers;
		rx_count = rx_bufs->count;
	}

	while (tx_count != 0 && rx_count != 0) {
		/* Compare the length of each corresponding TX and RX buffer */
		if (tx->len != rx->len) {
			return false;
		}

		tx++;
		tx_count--;
		rx++;
		rx_count--;
	}

	return true;
}
#if CONFIG_SPI_MCHP_INTERRUPT_DRIVEN
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
	const spi_mchp_dev_data_t *data = dev->data;
	uint8_t tx;
	uint8_t rx;

	/* Setup SPI buffers */
	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	/* Prepare first byte for transmission */
	if (spi_context_tx_buf_on(&data->ctx)) {
		tx = *(uint8_t *)(data->ctx.tx_buf);
	} else {
		tx = 0U;
	}

	/* Write first data byte to the SPI data register */
	hal_mchp_spi_write_data(hal, tx);
	spi_context_update_tx(&data->ctx, 1, 1);

	/* Enable SPI interrupts for RX, TX completion, and data empty events */
	hal_mchp_spi_enable_rxc_interrupt(hal);
	hal_mchp_spi_enable_txc_interrupt(hal);
	hal_mchp_spi_enable_data_empty_interrupt(hal);

	/* Wait for transaction to complete */
	k_sem_take(&data->ctx.sync, K_FOREVER);

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

	/* Lock SPI context */
	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	/* Configure SPI device */
	err = spi_mchp_configure(dev, config);
	if (err != 0) {
		goto done;
	}

#if !CONFIG_SPI_SLAVE
	/* Assert chip select (CS) */
	spi_context_cs_control(&data->ctx, true);
#endif

#if CONFIG_SPI_MCHP_INTERRUPT_DRIVEN
	/* Use interrupt-driven SPI transceive */
	err = spi_mchp_transceive_interrupt(dev, config, tx_bufs, rx_bufs);
#else
	/* Use optimized fast path if TX and RX buffer lengths match */
	if (spi_mchp_is_same_len(tx_bufs, rx_bufs)) {
		spi_mchp_fast_transceive(dev, config, tx_bufs, rx_bufs);
	} else {
		/* Setup SPI buffers and process using polling */
		spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

		do {
			spi_mchp_poll_in(hal, data);
		} while (spi_mchp_transfer_in_progress(data));
	}
#endif

	/* Deassert chip select (CS) */
	spi_context_cs_control(&data->ctx, false);

done:
	/* Release SPI context */
	spi_context_release(&data->ctx, err);
	return err;
}

#ifdef CONFIG_SPI_ASYNC
#ifdef CONFIG_SPI_MCHP_DMA_DRIVEN
/**
 * @brief Load data into the DMA controller for SPI TX transfer.
 *
 * This function configures and starts a DMA transaction to transfer data
 * from memory to the SPI peripheral. If no TX buffer is provided, it sends
 * dummy bytes (0xFF) instead.
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
	spi_mchp_dev_data_t *data = dev->data;

	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};
	int retval;

	dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->tx_dma_request;

	dma_blk.block_size = len;

	if (buf != NULL) {
		dma_blk.source_address = (uint32_t)buf;
	} else {
		static const uint8_t dummy_data = 0xFF;

		dma_blk.source_address = (uint32_t)&dummy_data;
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	dma_blk.dest_address = (uint32_t)(hal_mchp_spi_get_dma_dest_addr(hal));
	dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	retval = dma_config(cfg->dma_dev, cfg->tx_dma_channel, &dma_cfg);

	if (retval != 0) {
		return retval;
	}

	return dma_start(cfg->dma_dev, cfg->tx_dma_channel);
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
static bool spi_mchp_dma_advance_segment(const struct device *dev)
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
static int spi_mchp_dma_advance_buffers(const struct device *dev)
{
	spi_mchp_dev_data_t *data = dev->data;
	int retval;

	if (data->dma_segment_len == 0) {
		return -EINVAL;
	}

	/* Load receive buffer first to prepare for incoming data */
	if (data->ctx.rx_len) {
		retval = spi_mchp_dma_rx_load(dev, data->ctx.rx_buf, data->dma_segment_len);
	} else {
		retval = spi_mchp_dma_rx_load(dev, NULL, data->dma_segment_len);
	}

	if (retval != 0) {
		return retval;
	}

	/* Load transmit buffer, which starts SPI bus clocking */
	if (data->ctx.tx_len) {
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
	const struct spi_mchp_config *cfg = dev->config;
	int retval;

	ARG_UNUSED(id);
	ARG_UNUSED(error_code);

	/* Update TX and RX context with the completed DMA segment */
	spi_context_update_tx(&data->ctx, 1, data->dma_segment_len);
	spi_context_update_rx(&data->ctx, 1, data->dma_segment_len);

	/* Check if more segments need to be transferred */
	if (!spi_mchp_dma_advance_segment(dev)) {
		/* Transmission complete */
		spi_context_cs_control(&data->ctx, false);
		spi_context_complete(&data->ctx, dev, 0);
		return;
	}

	/* Load the next DMA segment */
	retval = spi_mchp_dma_advance_buffers(dev);
	if (retval != 0) {
		/* Stop DMA and terminate the SPI transaction in case of failure */
		dma_stop(cfg->dma_dev, cfg->tx_dma_channel);
		dma_stop(cfg->dma_dev, cfg->rx_dma_channel);
		spi_context_cs_control(&data->ctx, false);
		spi_context_complete(&data->ctx, dev, retval);
		return;
	}
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
	dma_cfg.dma_slot = cfg->rx_dma_request;

	/* Set block size */
	dma_blk.block_size = len;

	/* Configure destination address */
	if (buf != NULL) {
		dma_blk.dest_address = (uint32_t)buf;
	} else {
		/* Use a static dummy variable if no buffer is provided */
		static uint8_t dummy_data = 0xFF;

		dma_blk.dest_address = (uint32_t)&dummy_data;
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	/* Configure source address */
	dma_blk.source_address = (uint32_t)(hal_mchp_spi_get_dma_src_addr(hal));
	dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	/* Apply DMA configuration */
	retval = dma_config(cfg->dma_dev, cfg->rx_dma_channel, &dma_cfg);
	if (retval != 0) {
		return retval;
	}

	/* Start DMA transfer */
	return dma_start(cfg->dma_dev, cfg->rx_dma_channel);
}
#endif
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
 * @param cb Callback function to be called on completion.
 * @param userdata Pointer to user data to be passed to the callback.
 *
 * @retval 0 if the transfer is successfully initiated.
 * @retval -ENOTSUP if DMA channels are not properly configured.
 * @retval Other negative error codes in case of configuration or DMA setup
 * failures.
 */
static int spi_mchp_transceive_async(const struct device *dev, const struct spi_config *config,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs, spi_callback_t cb,
				     void *userdata)
{
	const spi_mchp_dev_config_t *cfg = dev->config;
	const hal_mchp_spi_t *hal = &cfg->hal;
	spi_mchp_dev_data_t *data = dev->data;
	int retval;

/*
 * Transmit clocks the output, and we use receive to
 * determine when the transmit is done, so we
 * always need both TX and RX DMA channels.
 */
#ifdef CONFIG_SPI_MCHP_DMA_DRIVEN
	if (cfg->tx_dma_channel == 0xFF || cfg->rx_dma_channel == 0xFF) {
		return -ENOTSUP;
	}
#endif

	/* Lock SPI context for exclusive access */
	spi_context_lock(&data->ctx, true, cb, userdata, config);

	/* Configure SPI based on the provided configuration */
	retval = spi_mchp_configure(dev, config);
	if (retval != 0) {
		goto err_unlock;
	}

#ifdef !CONFIG_SPI_SLAVE
	/* Assert chip select */
	spi_context_cs_control(&data->ctx, true);
#endif

	/* Set up TX and RX buffers */
	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

/* Prepare and start DMA transfers */
#ifdef CONFIG_SPI_MCHP_DMA_DRIVEN
	spi_mchp_dma_advance_segment(dev);
	retval = spi_mchp_dma_advance_buffers(dev);
#endif
	if (retval != 0) {
		goto err_cs;
	}
	return 0;

err_cs:
/* Stop DMA transfers in case of failure */
#ifdef CONFIG_SPI_MCHP_DMA_DRIVEN
	dma_stop(cfg->dma_dev, cfg->tx_dma_channel);
	dma_stop(cfg->dma_dev, cfg->rx_dma_channel);
#endif

/* Deassert chip select */
#ifdef !CONFIG_SPI_SLAVE
	spi_context_cs_control(&data->ctx, false);
#endif

err_unlock:
	/* Release SPI context lock */
	spi_context_release(&data->ctx, retval);
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
	uint8_t dummy_data = 0xFF;

	/* Transmit and receive data placeholders */
	uint8_t tx = 0U;
	uint8_t rx = 0U;

	/* Error status variable */
	int err = 0;

	/* Check if data is available in the receive buffer */
	if (hal_mchp_spi_is_rx_comp_set(hal)) {
		if (spi_context_rx_buf_on(&data->ctx)) {
			rx = hal_mchp_spi_read_data(hal);
			*(uint8_t *)(data->ctx.rx_buf) = rx;
			spi_context_update_rx(&data->ctx, 1, 1);
		}
	}

	/* Check if the transmit buffer is empty and send the next byte */
	if (hal_mchp_spi_is_data_empty_set(hal)) {
		hal_mchp_spi_disable_data_empty_interrupt(hal);

		if (spi_context_tx_buf_on(&data->ctx)) {
			tx = *(uint8_t *)(data->ctx.tx_buf);
			hal_mchp_spi_write_data(hal, tx);
			spi_context_update_tx(&data->ctx, 1, 1);
		} else if ((data->ctx.rx_len) - (data->ctx.tx_len) > 0) {
			hal_mchp_spi_write_data(hal, dummy_data);
		} else {
			data->last_byte = true;
		}

		/* Re-enable data empty interrupt if RX buffer is full */
		if (*(data->ctx.rx_buf) == (data->ctx.rx_len)) {
			hal_mchp_spi_enable_data_empty_interrupt(hal);
		}
	} else {
		/* Check if transmission is complete and disable interrupts
		 */
		if (hal_mchp_spi_is_tx_comp_set(hal) && (data->last_byte == true)) {
			hal_mchp_spi_disable_rxc_interrupt(hal);
			hal_mchp_spi_disable_txc_interrupt(hal);
			hal_mchp_spi_disable_data_empty_interrupt(hal);

#if !CONFIG_SPI_SLAVE
			/* Control chip select for SPI slave mode */
			spi_context_cs_control(&data->ctx, false);
#endif

#if CONFIG_SPI_MCHP_INTERRUPT_DRIVEN
			/* Release the semaphore to unblock waiting threads
			 */
			k_sem_give(&data->ctx.sync);
#endif

#if CONFIG_SPI_MCHP_INTERRUPT_DRIVEN_ASYNC
			/* Release the semaphore and invoke the callback */
			k_sem_give(&data->ctx.sync);
			spi_context_release(&data->ctx, err);
#endif

			/* Reset last byte flag */
			data->last_byte = false;
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

#ifdef CONFIG_SPI_ASYNC
	/* Asynchronous transceive function */
	.transceive_async = spi_mchp_transceive_async,
#endif

#ifdef CONFIG_SPI_RTIO
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
	.spi_dma.dma_dev = DEVICE_DT_GET(MICROCHIP_SAME54_DT_INST_DMA_CTLR(n, tx)),                \
	.spi_dma.hal.tx_dma_request = MICROCHIP_SAME54_DT_INST_DMA_TRIGSRC(n, tx),                 \
	.spi_dma.hal.tx_dma_channel = MICROCHIP_SAME54_DT_INST_DMA_CHANNEL(n, tx),                 \
	.spi_dma.hal.rx_dma_request = MICROCHIP_SAME54_DT_INST_DMA_TRIGSRC(n, rx),                 \
	.spi_dma.hal.rx_dma_channel = MICROCHIP_SAME54_DT_INST_DMA_CHANNEL(n, rx),
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
