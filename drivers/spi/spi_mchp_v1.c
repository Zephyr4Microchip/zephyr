/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <zephyr/drivers/clock_control/clock_control_mchp_v1.h>
#include <zephyr/irq.h>
#include <soc.h>
#include "spi_mchp_v1.h"

/**
 * Device constant configuration parameters
 * SPI configuration structure
 */

struct spi_mchp_dev_config_t {
	struct hal_mchp_spi_t hal;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clock_dev;

#ifdef CONFIG_SPI_MCHP_DMA_DRIVEN
	const struct device *dma_dev;
#endif

#if CONFIG_SPI_MCHP_INTERRUPT_DRIVEN_ASYNC || CONFIG_SPI_MCHP_DMA_DRIVEN
	void (*irq_config_func)(const struct device *dev);
#endif
};

/* SPI run time data */

struct spi_mchp_dev_data {
	struct spi_context ctx;
#ifdef CONFIG_SPI_MCHP_DMA_DRIVEN
	const struct device *dev;
	uint32_t dma_segment_len;
#endif
};

/* This is a internal fuction which is used to configure the spi peripheral. */

static int spi_mchp_configure(const struct device *dev, const struct spi_config *config)
{
	const struct spi_mchp_dev_config_t *cfg = dev->config;
	const struct hal_mchp_spi_t *hal = &cfg->hal;
	struct spi_mchp_dev_data *const data = dev->data;
	int err;
	uint32_t clock_rate;

	/**
	 * Disbale the SPI for Configuration
	 */
	hal_mchp_spi_disable(hal);

	/**
	 * Check if SPI is already configured
	 */
	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	/**
	 * Select the Character Size
	 */
	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		/**
		 * Char size 9 bit mode is not supported
		 */
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

	/**
	 * Enable the Receiver in SPI Peripheral
	 */
	hal_mchp_spi_rx_enable(hal);

	SPI_MCHP_GET_CLOCK_FREQ(dev, clock_rate);

	/**
	 * Select the Baudrate
	 */
	hal_mchp_spi_set_baudrate(hal, config, clock_rate);

	/**
	 * Set the SPI in Operation Mode
	 */
	if (SPI_OP_MODE_GET(config->operation) != SPI_OP_MODE_MASTER) {
		/**
		 * Slave mode is not implemented
		 */
		err = hal_mchp_spi_slave_mode(hal);
		if (err != 0) {
			return -ENOTSUP;
		}
	} else {
		err = hal_mchp_spi_master_mode(hal);
		if (err != 0) {
			return -ENOTSUP;
		}

		/**
		 * Set the CS line
		 */
		err = spi_context_cs_configure_all(&data->ctx);
		if (err < 0) {
			return err;
		}
	}

	/**
	 * Set the Data out and Pin out Configuration
	 */
	if ((config->operation & SPI_MODE_LOOP) != 0U) {
		/**
		 * Set MISO and MOSI on the same pad
		 */
		hal_mchp_spi_mode_loop(hal);
	} else {
		/**
		 * Set the Pads
		 */
		hal_mchp_spi_config_pinout(hal);
	}

	/**
	 * Set the Clock Polarity
	 */
	if (((config->operation & SPI_MODE_CPOL))) {
		hal_mchp_spi_cpol_idle_high(hal);
	} else {
		hal_mchp_spi_cpol_idle_low(hal);
	}

	/**
	 * Set the Clock Phase
	 */
	if (((config->operation & SPI_MODE_CPHA))) {
		hal_mchp_spi_cpha_lead_edge(hal);
	} else {
		hal_mchp_spi_cpha_trail_edge(hal);
	}

	/**
	 * Set the Data Order
	 */
	if ((config->operation & SPI_TRANSFER_LSB) != 0U) {
		hal_mchp_spi_lsb_first(hal);
	} else {
		hal_mchp_spi_msb_first(hal);
	}

	/**
	 * Set SPI Duplex Mode
	 */
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

	/**
	 * Enable the SPI Peripheral
	 */
	hal_mchp_spi_enable(hal);

#if CONFIG_SPI_ASYNC
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

	/**
	 * Save the config into the context
	 */
	data->ctx.config = config;
	return 0;
}

/**
 *  polarion transfer ongoing
 */
static bool spi_mchp_transfer_in_progress(struct spi_mchp_dev_data *data)
{
	return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}
/**
 * Polarion Shift master to poll in
 */
static void spi_mchp_poll_in(const struct hal_mchp_spi_t *hal, struct spi_mchp_dev_data *data)
{
	uint8_t tx;
	uint8_t rx;

	if (spi_context_tx_buf_on(&data->ctx)) {
		tx = *(uint8_t *)(data->ctx.tx_buf);
	} else {
		tx = 0U;
	}

	hal_mchp_spi_data_empty(hal);

	hal_mchp_spi_write_data(hal, tx);

	spi_context_update_tx(&data->ctx, 1, 1);

	hal_mchp_spi_rx_comp(hal);

	rx = hal_mchp_spi_read_data(hal);

	if (spi_context_rx_buf_on(&data->ctx)) {
		*data->ctx.rx_buf = rx;
	}
	spi_context_update_rx(&data->ctx, 1, 1);
}

/* Finish any ongoing writes and drop any remaining read data */
static void spi_mchp_finish(const struct hal_mchp_spi_t *hal)
{
	/**
	 * Wait until transmit complete
	 */
	hal_mchp_spi_tx_comp(hal);

	/**
	 * Clear the data
	 */
	hal_mchp_spi_clr_data(hal);
}

/**
 * Fast path that transmits a buf
 */
static void spi_mchp_fast_tx(const struct hal_mchp_spi_t *hal, const struct spi_buf *tx_buf)
{
	const uint8_t *p = tx_buf->buf;
	const uint8_t *pend = (uint8_t *)tx_buf->buf + tx_buf->len;
	uint8_t ch;

	while (p != pend) {
		ch = *p++;
		/**
		 * Wait until the Data empty
		 */
		hal_mchp_spi_data_empty(hal);

		/**
		 * Write data
		 */
		hal_mchp_spi_write_data(hal, ch);
	}

	spi_mchp_finish(hal);
}

/* Fast path that reads into a buf */
static void spi_mchp_fast_rx(const struct hal_mchp_spi_t *hal, const struct spi_buf *rx_buf)
{
	uint8_t *rx = rx_buf->buf;
	int len = rx_buf->len;

	if (len <= 0) {
		return;
	}

	while (len) {
		int dummy_data = 0;

		/* Write a dummy data to receive data */
		hal_mchp_spi_write_data(hal, dummy_data);
		len--;

		/* Wait for completion, and read */
		hal_mchp_spi_rx_comp(hal);

		*rx++ = hal_mchp_spi_read_data(hal);
	}

	spi_mchp_finish(hal);
}

/* Fast path that writes and reads bufs of the same length */
static void spi_mchp_fast_txrx(const struct hal_mchp_spi_t *hal, const struct spi_buf *tx_buf,
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

		/* Wait for completion*/
		hal_mchp_spi_rx_comp(hal);

		/*Read Data */
		*rx++ = hal_mchp_spi_read_data(hal);
	}
	spi_mchp_finish(hal);
}

/* Fast path where every overlapping tx and rx buffer is the same length */
static void spi_mchp_fast_transceive(const struct device *dev, const struct spi_config *config,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs)
{
	const struct spi_mchp_dev_config_t *cfg = dev->config;
	const struct hal_mchp_spi_t *hal = &cfg->hal;
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
		if (tx->buf == NULL) {
			spi_mchp_fast_rx(hal, rx);
		} else if (rx->buf == NULL) {
			spi_mchp_fast_tx(hal, tx);
		} else {
			spi_mchp_fast_txrx(hal, tx, rx);
		}

		tx++;
		tx_count--;
		rx++;
		rx_count--;
	}

	for (; tx_count != 0; tx_count--) {
		spi_mchp_fast_tx(hal, tx++);
	}

	for (; rx_count != 0; rx_count--) {
		spi_mchp_fast_rx(hal, rx++);
	}
}

/* Returns true if the request is suitable for the fast
 * path. Specifically, the bufs are a sequence of:
 *
 * - Zero or more RX and TX buf pairs where each is the same length.
 * - Zero or more trailing RX only bufs
 * - Zero or more trailing TX only bufs
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
static int spi_mchp_transceive_interrupt(const struct device *dev, const struct spi_config *config,
					 const struct spi_buf_set *tx_bufs,
					 const struct spi_buf_set *rx_bufs)
{
	const struct spi_mchp_dev_config_t *cfg = dev->config;
	const struct hal_mchp_spi_t *hal = &cfg->hal;
	const struct spi_mchp_dev_data *data = dev->data;
	uint8_t tx;
	uint8_t rx;

	/* Setup SPI buffers */
	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	if (spi_context_tx_buf_on(&data->ctx)) {
		tx = *(uint8_t *)(data->ctx.tx_buf);
	} else {
		tx = 0U;
	}
	/* Write first data byte to the SPI data register
	 */
	hal_mchp_spi_write_data(hal, tx);
	spi_context_update_tx(&data->ctx, 1, 1);

	hal_mchp_spi_enable_rxc_interrupt(hal);
	hal_mchp_spi_enable_txc_interrupt(hal);
	hal_mchp_spi_enable_data_empty_interrupt(hal);

	k_sem_take(&data->ctx.sync, K_FOREVER);
	return 0;
}
#endif

static int spi_mchp_transceive_sync(const struct device *dev, const struct spi_config *config,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs)
{
	const struct spi_mchp_dev_config_t *cfg = dev->config;
	const struct hal_mchp_spi_t *hal = &cfg->hal;
	struct spi_mchp_dev_data *data = dev->data;
	int err;

	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	err = spi_mchp_configure(dev, config);
	if (err != 0) {
		goto done;
	}

#if !CONFIG_SPI_SLAVE
	spi_context_cs_control(&data->ctx, true);
#endif

#if CONFIG_SPI_MCHP_INTERRUPT_DRIVEN
	err = spi_mchp_transceive_interrupt(dev, config, tx_bufs, rx_bufs);
#else
	/* This driver special cases the common send only, receive
	 * only, and transmit then receive operations.	This special
	 * casing is 4x faster than the spi_context() routines
	 * and allows the transmit and receive to be interleaved.
	 */
	if (spi_mchp_is_same_len(tx_bufs, rx_bufs)) {
		spi_mchp_fast_transceive(dev, config, tx_bufs, rx_bufs);
	} else {
		spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

		do {
			spi_mchp_poll_in(hal, data);
		} while (spi_mchp_transfer_in_progress(data));
	}
#endif
	spi_context_cs_control(&data->ctx, false);

done:
	spi_context_release(&data->ctx, err);
	return err;
}

#ifdef CONFIG_SPI_ASYNC

static void spi_mchp_dma_rx_done(const struct device *dma_dev, void *arg, uint32_t id,
				 int error_code);

static int spi_mchp_dma_rx_load(const struct device *dev, uint8_t *buf, size_t len)
{
	const struct spi_mchp_dev_config_t *cfg = dev->config;
	const struct hal_mchp_spi_t *hal = &cfg->hal;
	struct spi_mchp_dev_data *data = dev->data;

	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};
	int retval;

	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.user_data = data;
	dma_cfg.dma_callback = spi_mchp_dma_rx_done;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->rx_dma_request;

	dma_blk.block_size = len;

	if (buf != NULL) {
		dma_blk.dest_address = (uint32_t)buf;
	} else {
		static uint8_t dummy;

		dma_blk.dest_address = (uint32_t)&dummy;
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	dma_blk.source_address = (uint32_t)(&(GET_DMA_SRC_ADDR));
	dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	retval = dma_config(cfg->dma_dev, cfg->rx_dma_channel, &dma_cfg);
	if (retval != 0) {
		return retval;
	}

	return dma_start(cfg->dma_dev, cfg->rx_dma_channel);
}

static int spi_mchp_dma_tx_load(const struct device *dev, const uint8_t *buf, size_t len)
{
	const struct spi_mchp_dev_config_t *cfg = dev->config;
	const struct hal_mchp_spi_t *hal = &cfg->hal;
	struct spi_mchp_dev_data *data = dev->data;

	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};
	int retval;

	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->tx_dma_request;

	dma_blk.block_size = len;

	if (buf != NULL) {
		dma_blk.source_address = (uint32_t)buf;
	} else {
		static const uint8_t dummy;

		dma_blk.source_address = (uint32_t)&dummy;
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	dma_blk.dest_address = (uint32_t)(&(GET_DMA_DEST_ADDR));
	dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	retval = dma_config(cfg->dma_dev, cfg->tx_dma_channel, &dma_cfg);

	if (retval != 0) {
		return retval;
	}

	return dma_start(cfg->dma_dev, cfg->tx_dma_channel);
}

static bool spi_mchp_dma_advance_segment(const struct device *dev)
{
	struct spi_mchp_dev_data *data = dev->data;
	uint32_t segment_len;

	/* Pick the shorter buffer of ones that have an
	 * actual length
	 */
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

	segment_len = MIN(segment_len, 65535);

	data->dma_segment_len = segment_len;
	return true;
}

static int spi_mchp_dma_advance_buffers(const struct device *dev)
{
	struct spi_mchp_dev_data *data = dev->data;
	int retval;

	if (data->dma_segment_len == 0) {
		return -EINVAL;
	}

	/* Load receive first, so it can accept transmit
	 * data
	 */
	if (data->ctx.rx_len) {
		retval = spi_mchp_dma_rx_load(dev, data->ctx.rx_buf, data->dma_segment_len);
	} else {
		retval = spi_mchp_dma_rx_load(dev, NULL, data->dma_segment_len);
	}

	if (retval != 0) {
		return retval;
	}

	/* Now load the transmit, which starts the actual
	 * bus clocking
	 */
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

static void spi_mchp_dma_rx_done(const struct device *dma_dev, void *arg, uint32_t id,
				 int error_code)
{
	struct spi_mchp_dev_data *data = arg;
	const struct device *dev = data->dev;
	const struct spi_mchp_config *cfg = dev->config;
	int retval;

	ARG_UNUSED(id);
	ARG_UNUSED(error_code);

	spi_context_update_tx(&data->ctx, 1, data->dma_segment_len);
	spi_context_update_rx(&data->ctx, 1, data->dma_segment_len);

	if (!spi_mchp_dma_advance_segment(dev)) {
		/* Done */
		spi_context_cs_control(&data->ctx, false);
		spi_context_complete(&data->ctx, dev, 0);
		return;
	}

	retval = spi_mchp_dma_advance_buffers(dev);
	if (retval != 0) {
		dma_stop(cfg->dma_dev, cfg->tx_dma_channel);
		dma_stop(cfg->dma_dev, cfg->rx_dma_channel);
		spi_context_cs_control(&data->ctx, false);
		spi_context_complete(&data->ctx, dev, retval);
		return;
	}
}

static int spi_mchp_transceive_async(const struct device *dev, const struct spi_config *config,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs, spi_callback_t cb,
				     void *userdata)
{
	const struct spi_mchp_config *cfg = dev->config;
	struct spi_mchp_dev_data *data = dev->data;
	int retval;

	/*
	 * Transmit clocks the output and we use receive to
	 * determine when the transmit is done, so we
	 * always need both
	 */
	if (cfg->tx_dma_channel == 0xFF || cfg->rx_dma_channel == 0xFF) {
		return -ENOTSUP;
	}

	spi_context_lock(&data->ctx, true, cb, userdata, config);

	retval = spi_mchp_configure(dev, config);
	if (retval != 0) {
		goto err_unlock;
	}

	spi_context_cs_control(&data->ctx, true);

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	spi_mchp_dma_advance_segment(dev);
	retval = spi_mchp_dma_advance_buffers(dev);
	if (retval != 0) {
		goto err_cs;
	}

	return 0;

err_cs:
	dma_stop(cfg->dma_dev, cfg->tx_dma_channel);
	dma_stop(cfg->dma_dev, cfg->rx_dma_channel);

	spi_context_cs_control(&data->ctx, false);

err_unlock:
	spi_context_release(&data->ctx, retval);
	return retval;
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_mchp_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_mchp_dev_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static void spi_mchp_isr(const struct device *dev)
{
	struct spi_mchp_dev_data *data = dev->data;
	const struct spi_mchp_dev_config_t *cfg = dev->config;
	const struct hal_mchp_spi_t *hal = &cfg->hal;
	uint8_t dummy_data = 0U;
	uint8_t last_byte = false;
	uint8_t tx = 0U;
	uint8_t rx = 0U;

	/**
	 *Read received data
	 */
	if (hal_mchp_spi_is_rx_comp_set(hal)) {
		if (spi_context_rx_buf_on(&data->ctx)) {
			rx = hal_mchp_spi_read_data(hal);

			*(uint8_t *)(data->ctx.rx_buf) = rx;
			spi_context_update_rx(&data->ctx, 1, 1);
		}
	}

	/* Write next byte if available */
	if (hal_mchp_spi_is_data_empty_set(hal)) {
		hal_mchp_spi_disable_data_empty_interrupt(hal);
		if (spi_context_tx_buf_on(&data->ctx)) {
			tx = *(uint8_t *)(data->ctx.tx_buf);
			hal_mchp_spi_write_data(hal, tx);
			spi_context_update_tx(&data->ctx, 1, 1);
		} else if ((data->ctx.rx_len) - (data->ctx.tx_len) > 0) {
			hal_mchp_spi_write_data(hal, dummy_data);
		} else {
			last_byte = true;
		}
		if (*(data->ctx.rx_buf) == (data->ctx.rx_len)) {
			hal_mchp_spi_enable_data_empty_interrupt(hal);
		}
	} else {
		(hal_mchp_spi_is_tx_comp_set(hal) && (last_byte == true));
		{
			/* No more bytes to send, disable
			 * interrupts
			 */
			hal_mchp_spi_disable_rxc_interrupt(hal);
			hal_mchp_spi_disable_txc_interrupt(hal);
			hal_mchp_spi_disable_data_empty_interrupt(hal);
#if CONFIG_SPI_SLAVE
			spi_context_cs_control(&data->ctx, false);

#endif
#if CONFIG_SPI_MCHP_INTERRUPT_DRIVEN
			/* Release the semaphore to unblock
			 * the waiting thread
			 */
			k_sem_give(&data->ctx.sync);
#endif
#if CONFIG_SPI_MCHP_INTERRUPT_DRIVEN_ASYNC
			/* Release the semaphore to unblock
			 * the waiting thread
			 */
			k_sem_give(&data->ctx.sync);
			spi_context_release(&data->ctx, err);
			/**
			 * Invoke the callback
			 */
#endif
		}
	}
}

static int spi_mchp_init(const struct device *dev)
{
	int err;
	const struct spi_mchp_dev_config_t *cfg = dev->config;
	const struct hal_mchp_spi_t *hal = &cfg->hal;
	struct spi_mchp_dev_data *const data = dev->data;

	/**
	 * Enable the Clock
	 */
	SPI_MCHP_ENABLE_CLOCK(dev);

	/**
	 *  Disable the SPI
	 */
	hal_mchp_spi_disable(hal);

	/* Disable all SPI interrupts */
	hal_mchp_spi_disable_interrupts(hal);

	/* Set the pinctrl states */
	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	/* The device will be configured and enabled when
	 * transceive is called.
	 */

	return 0;
}

static DEVICE_API(spi, spi_mchp_driver_api) = {
	.transceive = spi_mchp_transceive_sync,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_mchp_transceive_async,
#endif
#ifdef CONFIG_SPI_RTIO
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif
	.release = spi_mchp_release,
};

/**
 * @brief Do the peripheral interrupt related configuration
 */

#if CONFIG_SPI_MCHP_INTERRUPT_DRIVEN || CONFIG_SPI_ASYNC

#define MCHP_SPI_IRQ_CONNECT(n, m) do {
IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, m, irq), DT_INST_IRQ_BY_IDX(n, m, priority), spi_mchp_isr,
	    DEVICE_DT_INST_GET(n), 0);
irq_enable(DT_INST_IRQ_BY_IDX(n, m, irq));
}
while (false) {
}

#define SPI_MCHP_IRQ_HANDLER_DECL(n) static void spi_mchp_irq_config_##n(const struct device *dev)
#define SPI_MCHP_IRQ_HANDLER_FUNC(n) .irq_config_func = spi_mchp_irq_config_##n,

#else
#define SPI_MCHP_IRQ_HANDLER_DECL(n)
#define SPI_MCHP_IRQ_HANDLER_FUNC(n)
#endif

#if CONFIG_SPI_MCHP_DMA_DRIVEN
#define SPI_MCHP_DMA_CHANNELS(n) .dma_dev = DEVICE_DT_GET(MICROCHIP_SAME54_DT_INST_DMA_CTLR(n, tx)),
#else
#define SPI_MCHP_DMA_CHANNELS(n)
#endif

/**
 * This is the Intial Configuration for the SPI Perpiheral
 */

#define SPI_MCHP_CONFIG_DEFN(n)                                                             \
	static const struct spi_mchp_dev_config_t spi_mchp_config_##n = {                       \
		SPI_MCHP_HAL_DEFN(n),																\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                   						\
		.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                                   	\
		SPI_MCHP_IRQ_HANDLER_FUNC(n) SPI_MCHP_DMA_CHANNELS(n)}

#define SPI_MCHP_DEVICE_INIT(n)                                                             \
	PINCTRL_DT_INST_DEFINE(n);                                                              \
	SPI_MCHP_CONFIG_DEFN(n);                                                                \
	static struct spi_mchp_dev_data spi_mchp_data_##n = {                                   \
		SPI_CONTEXT_INIT_LOCK(spi_mchp_data_##n, ctx),                         				\
		SPI_CONTEXT_INIT_SYNC(spi_mchp_data_##n, ctx),                                     	\
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             	\
	SPI_DEVICE_DT_INST_DEFINE(n, spi_mchp_init, NULL, &spi_mchp_data_##n,                   \
				  &spi_mchp_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,     			\
				  &spi_mchp_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_MCHP_DEVICE_INIT)
