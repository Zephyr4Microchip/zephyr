/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file uart_mchp_v1.c
 * @brief UART driver implementation for Microchip devices.
 */

#include <zephyr/device.h>
#include <errno.h>
#include <zephyr/init.h>
#include <zephyr/sys/__assert.h>
#include <soc.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <string.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/clock_control/mchp_clock_sam_d5x_e5x.h>
#include "uart_mchp_v1.h"

/**
 * @brief DMA configuration structure for the UART.
 *
 * This structure contains the DMA configuration parameters for the UART
 * peripheral.
 */
typedef struct mchp_uart_dma {
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
} mchp_uart_dma_t;

/**
 * @brief UART device constant configuration structure.
 */
typedef struct uart_mchp_dev_cfg {
	/* Baud rate for UART communication */
	uint32_t baudrate;
	uint8_t data_bits;
	uint8_t parity;
	uint8_t stop_bits;

	/* HAL UART structure */
	hal_mchp_uart_t hal;

#if CONFIG_UART_MCHP_ASYNC
	mchp_uart_dma_t uart_dma;
#endif

#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_MCHP_ASYNC
	/* IRQ configuration function */
	void (*irq_config_func)(const struct device *dev);
#endif
	/* Clock configuration */
	mchp_uart_clock_t uart_clock;
	/* Pin control configuration */
	const struct pinctrl_dev_config *pcfg;

} uart_mchp_dev_cfg_t;

/**
 * @brief UART device runtime data structure.
 */
typedef struct uart_mchp_dev_data {
	/* Cached UART configuration */
	struct uart_config config_cache;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	/* IRQ callback function */
	uart_irq_callback_user_data_t cb;
	/* IRQ callback user data */
	void *cb_data;
	/* Cached status of TX completion */
	bool is_tx_completed_cache;
#endif

#if CONFIG_UART_MCHP_ASYNC
	/* Device structure */
	const struct device *dev;
	/* Device configuration */
	const uart_mchp_dev_cfg_t *cfg;

	/* Asynchronous callback function */
	uart_callback_t async_cb;
	/* Asynchronous callback user data */
	void *async_cb_data;

	/* TX timeout work structure */
	struct k_work_delayable tx_timeout_work;
	/* TX buffer */
	const uint8_t *tx_buf;
	/* TX buffer length */
	size_t tx_len;

	/* RX timeout work structure */
	struct k_work_delayable rx_timeout_work;
	/* RX timeout time */
	size_t rx_timeout_time;
	/* RX timeout chunk */
	size_t rx_timeout_chunk;
	/* RX timeout start time */
	uint32_t rx_timeout_start;
	/* RX buffer */
	uint8_t *rx_buf;
	/* RX buffer length */
	size_t rx_len;
	/* RX processed length */
	size_t rx_processed_len;
	/* Next RX buffer */
	uint8_t *rx_next_buf;
	/* Next RX buffer length */
	size_t rx_next_len;
	/* RX waiting for IRQ flag */
	bool rx_waiting_for_irq;
	/* RX timeout from ISR flag */
	bool rx_timeout_from_isr;
#endif
} uart_mchp_dev_data_t;

#if CONFIG_UART_MCHP_ASYNC
/**
 * @brief TX timeout handler.
 *
 * @param work Work structure.
 */
static void uart_mchp_tx_timeout(struct k_work *work);

/**
 * @brief RX timeout handler.
 *
 * @param work Work structure.
 */
static void uart_mchp_rx_timeout(struct k_work *work);

/**
 * @brief DMA TX done callback.
 *
 * @param dma_dev DMA device.
 * @param arg User argument.
 * @param id DMA channel ID.
 * @param error_code Error code.
 */
static void uart_mchp_dma_tx_done(const struct device *dma_dev, void *arg, uint32_t id,
				  int error_code);

/**
 * @brief DMA RX done callback.
 *
 * @param dma_dev DMA device.
 * @param arg User argument.
 * @param id DMA channel ID.
 * @param error_code Error code.
 */
static void uart_mchp_dma_rx_done(const struct device *dma_dev, void *arg, uint32_t id,
				  int error_code);
#endif

#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_MCHP_ASYNC

/**
 * @brief UART ISR handler.
 *
 * @param dev Device structure.
 */
static void uart_mchp_isr(const struct device *dev)
{
	uart_mchp_dev_data_t *const dev_data = dev->data;

#if CONFIG_UART_INTERRUPT_DRIVEN
	if (dev_data->cb) {
		dev_data->cb(dev, dev_data->cb_data);
	}
#endif

#if CONFIG_UART_MCHP_ASYNC
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	const hal_mchp_uart_t *const hal = &cfg->hal;

	if (dev_data->tx_len && hal_mchp_uart_is_tx_complete(hal)) {
		hal_mchp_uart_enable_tx_complete_interrupt(hal, false);
		k_work_cancel_delayable(&dev_data->tx_timeout_work);

		unsigned int key = irq_lock();

		struct uart_event evt = {
			.type = UART_TX_DONE,
			.data.tx =
				{
					.buf = dev_data->tx_buf,
					.len = dev_data->tx_len,
				},
		};

		dev_data->tx_buf = NULL;
		dev_data->tx_len = 0U;

		if (dev_data->async_cb) {
			dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
		}

		irq_unlock(key);
	}

	if (dev_data->rx_len && hal_mchp_uart_is_rx_complete(hal) && dev_data->rx_waiting_for_irq) {
		dev_data->rx_waiting_for_irq = false;
		hal_mchp_uart_enable_rx_interrupt(hal, false);

		/* Receive started, so request the next buffer */
		if ((dev_data->rx_next_len == 0U) && dev_data->async_cb) {
			struct uart_event evt = {
				.type = UART_RX_BUF_REQUEST,
			};

			dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
		}

		/*
		 * If we have a timeout, restart the time remaining whenever
		 * we see data.
		 */
		if (dev_data->rx_timeout_time != SYS_FOREVER_US) {
			dev_data->rx_timeout_from_isr = true;
			dev_data->rx_timeout_start = k_uptime_get_32();
			k_work_reschedule(&dev_data->rx_timeout_work,
					  K_USEC(dev_data->rx_timeout_chunk));
		}

		/* DMA will read the currently ready byte out */
		dma_start(cfg->uart_dma.dma_dev, cfg->uart_dma.rx_dma_channel);
	}
#endif
}

#endif

/**
 * @brief Initialize the UART device.
 *
 * @param dev Device structure.
 * @return 0 on success, negative error code on failure.
 */
static int uart_mchp_init(const struct device *dev)
{
	int retval;
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	uart_mchp_dev_data_t *const dev_data = dev->data;
	const hal_mchp_uart_t *const hal = &cfg->hal;

	/* Enable the GCLK */
	UART_MCHP_ENABLE_CLOCK(dev);

	hal_mchp_uart_disable_interrupts(hal);

	dev_data->config_cache.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;

	hal_mchp_uart_config_data_bits(hal, cfg->data_bits);
	dev_data->config_cache.data_bits = cfg->data_bits;

	hal_mchp_uart_config_parity(hal, cfg->parity);
	dev_data->config_cache.parity = cfg->parity;

	hal_mchp_uart_config_stop_bits(hal, cfg->stop_bits);
	dev_data->config_cache.stop_bits = cfg->stop_bits;

	hal_mchp_uart_config_pinout(hal);
	hal_mchp_uart_set_clock_polarity(hal, false);
	hal_mchp_uart_set_clock_source(hal);
	hal_mchp_uart_set_lsb_first(hal, true);

	/* Enable PINMUX based on PINCTRL */
	retval = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		return retval;
	}

	/* Enable receiver and transmitter */
	hal_mchp_uart_rx_enable(hal, true);
	hal_mchp_uart_tx_enable(hal, true);

	uint32_t clock_rate;
	UART_MCHP_GET_CLOCK_FREQ(dev, clock_rate);
	retval = hal_mchp_uart_set_baudrate(hal, cfg->baudrate, clock_rate);
	if (retval != 0) {
		return retval;
	}
	dev_data->config_cache.baudrate = cfg->baudrate;

#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_MCHP_ASYNC
	cfg->irq_config_func(dev);
#endif

#ifdef CONFIG_UART_MCHP_ASYNC
	dev_data->dev = dev;
	dev_data->cfg = cfg;
	if (!device_is_ready(cfg->uart_dma.dma_dev)) {
		return -ENODEV;
	}

	k_work_init_delayable(&dev_data->tx_timeout_work, uart_mchp_tx_timeout);
	k_work_init_delayable(&dev_data->rx_timeout_work, uart_mchp_rx_timeout);

	if (cfg->uart_dma.tx_dma_channel != 0xFFU) {
		struct dma_config dma_cfg = {0};
		struct dma_block_config dma_blk = {0};

		dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
		dma_cfg.source_data_size = 1;
		dma_cfg.dest_data_size = 1;
		dma_cfg.user_data = dev_data;
		dma_cfg.dma_callback = uart_mchp_dma_tx_done;
		dma_cfg.block_count = 1;
		dma_cfg.head_block = &dma_blk;
		dma_cfg.dma_slot = cfg->uart_dma.tx_dma_request;

		dma_blk.block_size = 1;
		dma_blk.dest_address = (uint32_t)(hal_mchp_uart_get_dma_dest_addr(hal));
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

		retval = dma_config(cfg->uart_dma.dma_dev, cfg->uart_dma.tx_dma_channel, &dma_cfg);
		if (retval != 0) {
			return retval;
		}
	}

	if (cfg->uart_dma.rx_dma_channel != 0xFFU) {
		struct dma_config dma_cfg = {0};
		struct dma_block_config dma_blk = {0};

		dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		dma_cfg.source_data_size = 1;
		dma_cfg.dest_data_size = 1;
		dma_cfg.user_data = dev_data;
		dma_cfg.dma_callback = uart_mchp_dma_rx_done;
		dma_cfg.block_count = 1;
		dma_cfg.head_block = &dma_blk;
		dma_cfg.dma_slot = cfg->uart_dma.rx_dma_request;

		dma_blk.block_size = 1;
		dma_blk.source_address = (uint32_t)(hal_mchp_uart_get_dma_source_addr(hal));
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

		retval = dma_config(cfg->uart_dma.dma_dev, cfg->uart_dma.rx_dma_channel, &dma_cfg);
		if (retval != 0) {
			return retval;
		}
	}

#endif
	hal_mchp_uart_enable(hal, true);

	return 0;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
/**
 * @brief Configure the UART device.
 *
 * @param dev Device structure.
 * @param new_cfg New UART configuration.
 * @return 0 on success, negative error code on failure.
 */
static int uart_mchp_configure(const struct device *dev, const struct uart_config *new_cfg)
{
	int retval;
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	uart_mchp_dev_data_t *const dev_data = dev->data;
	const hal_mchp_uart_t *const hal = &cfg->hal;

	hal_mchp_uart_enable(hal, false);

	if (new_cfg->flow_ctrl != UART_CFG_FLOW_CTRL_NONE) {
		/* Flow control not yet supported though in principle possible
		 * on this soc family.
		 */
		return -ENOTSUP;
	}
	dev_data->config_cache.flow_ctrl = new_cfg->flow_ctrl;

	switch (new_cfg->parity) {
	case UART_CFG_PARITY_NONE:
		hal_mchp_uart_config_parity(hal, HAL_MCHP_UART_CFG_PARITY_NONE);
		break;
	case UART_CFG_PARITY_ODD:
		hal_mchp_uart_config_parity(hal, HAL_MCHP_UART_CFG_PARITY_ODD);
		break;
	case UART_CFG_PARITY_EVEN:
		hal_mchp_uart_config_parity(hal, HAL_MCHP_UART_CFG_PARITY_EVEN);
		break;
	default:
		return -ENOTSUP;
	}
	dev_data->config_cache.parity = new_cfg->parity;

	unsigned int stop_bit_count;
	switch (new_cfg->stop_bits) {
	case UART_CFG_STOP_BITS_1:
		stop_bit_count = 1;
		break;
	case UART_CFG_STOP_BITS_2:
		stop_bit_count = 2;
		break;
	default:
		return -ENOTSUP;
	}
	retval = hal_mchp_uart_config_stop_bits(hal, stop_bit_count);
	if (retval != 0) {
		return -ENOTSUP;
	}
	dev_data->config_cache.stop_bits = new_cfg->stop_bits;

	unsigned int data_bit_count;
	switch (new_cfg->data_bits) {
	case UART_CFG_DATA_BITS_5:
		data_bit_count = 5;
		break;
	case UART_CFG_DATA_BITS_6:
		data_bit_count = 6;
		break;
	case UART_CFG_DATA_BITS_7:
		data_bit_count = 7;
		break;
	case UART_CFG_DATA_BITS_8:
		data_bit_count = 8;
		break;
	case UART_CFG_DATA_BITS_9:
		data_bit_count = 9;
		break;
	default:
		return -ENOTSUP;
	}
	retval = hal_mchp_uart_config_data_bits(hal, data_bit_count);
	if (retval != 0) {
		return -ENOTSUP;
	}
	dev_data->config_cache.data_bits = new_cfg->data_bits;

	uint32_t clock_rate;
	UART_MCHP_GET_CLOCK_FREQ(dev, clock_rate);
	retval = hal_mchp_uart_set_baudrate(hal, new_cfg->baudrate, clock_rate);
	if (retval != 0) {
		return retval;
	}
	dev_data->config_cache.baudrate = new_cfg->baudrate;

	hal_mchp_uart_enable(hal, true);

	return 0;
}

/**
 * @brief Get the current UART configuration.
 *
 * @param dev Device structure.
 * @param out_cfg Output configuration structure.
 * @return 0 on success.
 */
static int uart_mchp_config_get(const struct device *dev, struct uart_config *out_cfg)
{
	uart_mchp_dev_data_t *const dev_data = dev->data;

	memcpy(out_cfg, &(dev_data->config_cache), sizeof(dev_data->config_cache));

	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

/**
 * @brief Poll the UART device for input.
 *
 * @param dev Device structure.
 * @param data Pointer to store the received data.
 * @return 0 on success, negative error code on failure.
 */
static int uart_mchp_poll_in(const struct device *dev, unsigned char *data)
{
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	const hal_mchp_uart_t *const hal = &cfg->hal;

#ifdef CONFIG_UART_MCHP_ASYNC
	uart_mchp_dev_data_t *const dev_data = dev->data;

	if (dev_data->rx_len != 0U) {
		return -EBUSY;
	}
#endif /* CONFIG_UART_MCHP_ASYNC */

	if (!hal_mchp_uart_is_rx_complete(hal)) {
		return -1;
	}

	*data = hal_mchp_uart_get_received_char(hal);
	return 0;
}

/**
 * @brief Output a character via UART.
 *
 * @param dev Device structure.
 * @param data Character to send.
 */
static void uart_mchp_poll_out(const struct device *dev, unsigned char data)
{
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	const hal_mchp_uart_t *const hal = &cfg->hal;

	while (!hal_mchp_uart_is_tx_ready(hal)) {
	}

	/* send a character */
	hal_mchp_uart_tx_char(hal, data);
}

/**
 * @brief Check for UART errors.
 *
 * @param dev Device structure.
 * @return Error code.
 */
static int uart_mchp_err_check(const struct device *dev)
{
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	const hal_mchp_uart_t *const hal = &cfg->hal;
	uint32_t err = 0U;

	if (hal_mchp_uart_is_err_buffer_overflow(hal)) {
		err |= UART_ERROR_OVERRUN;
	}

	if (hal_mchp_uart_is_err_frame(hal)) {
		err |= UART_ERROR_FRAMING;
	}

	if (hal_mchp_uart_is_err_parity(hal)) {
		err |= UART_ERROR_PARITY;
	}

	if (hal_mchp_uart_is_err_autobaud_sync(hal)) {
		err |= UART_BREAK;
	}

	if (hal_mchp_uart_is_err_collision(hal)) {
		err |= UART_ERROR_COLLISION;
	}

	/* Clear all errors */
	hal_mchp_uart_err_clear_all(hal);

	return err;
}

#if CONFIG_UART_INTERRUPT_DRIVEN

/**
 * @brief Fill the UART FIFO with data.
 *
 * This function fills the UART FIFO with data from the provided buffer.
 *
 * @param dev Pointer to the device structure.
 * @param tx_data Pointer to the data buffer to be transmitted.
 * @param len Length of the data buffer.
 * @return Number of bytes written to the FIFO.
 */
static int uart_mchp_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	const hal_mchp_uart_t *const hal = &cfg->hal;

	if (hal_mchp_uart_is_tx_ready(hal) && (len >= 1)) {
		hal_mchp_uart_tx_char(hal, tx_data[0]); /* Transmit the first character */
		return 1;
	} else {
		return 0;
	}
}

/**
 * @brief Enable UART TX interrupt.
 *
 * This function enables the UART TX ready and TX complete interrupts.
 *
 * @param dev Pointer to the device structure.
 */
static void uart_mchp_irq_tx_enable(const struct device *dev)
{
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	const hal_mchp_uart_t *const hal = &cfg->hal;

	unsigned int key = irq_lock();
	hal_mchp_uart_enable_tx_ready_interrupt(hal, true);
	hal_mchp_uart_enable_tx_complete_interrupt(hal, true);
	irq_unlock(key);
}

/**
 * @brief Disable UART TX interrupt.
 *
 * This function disables the UART TX ready and TX complete interrupts.
 *
 * @param dev Pointer to the device structure.
 */
static void uart_mchp_irq_tx_disable(const struct device *dev)
{
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	const hal_mchp_uart_t *const hal = &cfg->hal;

	hal_mchp_uart_enable_tx_ready_interrupt(hal, false);
	hal_mchp_uart_enable_tx_complete_interrupt(hal, false);
}

/**
 * @brief Check if UART TX is ready.
 *
 * This function checks if the UART TX is ready to transmit data.
 *
 * @param dev Pointer to the device structure.
 * @return 1 if TX is ready, 0 otherwise.
 */
static int uart_mchp_irq_tx_ready(const struct device *dev)
{
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	const hal_mchp_uart_t *const hal = &cfg->hal;

	return (hal_mchp_uart_is_tx_ready(hal) && hal_mchp_uart_is_tx_interrupt_enabled(hal));
}

/**
 * @brief Check if UART TX is complete.
 *
 * This function checks if the UART TX has completed transmission.
 *
 * @param dev Pointer to the device structure.
 * @return 1 if TX is complete, 0 otherwise.
 */
static int uart_mchp_irq_tx_complete(const struct device *dev)
{
	uart_mchp_dev_data_t *const dev_data = dev->data;

	if (dev_data->is_tx_completed_cache) {
		return 1;
	} else {
		return 0;
	}
}

/**
 * @brief Enable UART RX interrupt.
 *
 * This function enables the UART RX interrupt.
 *
 * @param dev Pointer to the device structure.
 */
static void uart_mchp_irq_rx_enable(const struct device *dev)
{
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	const hal_mchp_uart_t *const hal = &cfg->hal;

	hal_mchp_uart_enable_rx_interrupt(hal, true);
}

/**
 * @brief Disable UART RX interrupt.
 *
 * This function disables the UART RX interrupt.
 *
 * @param dev Pointer to the device structure.
 */
static void uart_mchp_irq_rx_disable(const struct device *dev)
{
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	const hal_mchp_uart_t *const hal = &cfg->hal;

	hal_mchp_uart_enable_rx_interrupt(hal, false);
}

/**
 * @brief Check if UART RX is ready.
 *
 * This function checks if the UART RX has received data.
 *
 * @param dev Pointer to the device structure.
 * @return 1 if RX is ready, 0 otherwise.
 */
static int uart_mchp_irq_rx_ready(const struct device *dev)
{
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	const hal_mchp_uart_t *const hal = &cfg->hal;

	return (hal_mchp_uart_is_rx_complete(hal));
}

/**
 * @brief Read data from UART FIFO.
 *
 * This function reads data from the UART FIFO into the provided buffer.
 *
 * @param dev Pointer to the device structure.
 * @param rx_data Pointer to the buffer to store received data.
 * @param size Size of the buffer.
 * @return Number of bytes read from the FIFO.
 */
static int uart_mchp_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	const hal_mchp_uart_t *const hal = &cfg->hal;

	if (hal_mchp_uart_is_rx_complete(hal)) {
		uint8_t ch = hal_mchp_uart_get_received_char(hal); /* Get the received character */

		if (size >= 1) {
			*rx_data = ch; /* Store the received character in the buffer */
			return 1;
		} else {
			return -EINVAL;
		}
	}
	return 0;
}

/**
 * @brief Check if UART interrupt is pending.
 *
 * This function checks if there is any pending UART interrupt.
 *
 * @param dev Pointer to the device structure.
 * @return 1 if an interrupt is pending, 0 otherwise.
 */
static int uart_mchp_irq_is_pending(const struct device *dev)
{
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	const hal_mchp_uart_t *const hal = &cfg->hal;

	if (hal_mchp_uart_is_interrupt_pending(hal)) {
		return 1;
	} else {
		return 0;
	}
}

/**
 * @brief Enable UART error interrupt.
 *
 * This function enables the UART error interrupt.
 *
 * @param dev Pointer to the device structure.
 */
static void uart_mchp_irq_err_enable(const struct device *dev)
{
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	const hal_mchp_uart_t *const hal = &cfg->hal;

	hal_mchp_uart_enable_err_interrupt(hal, true);
}

/**
 * @brief Disable UART error interrupt.
 *
 * This function disables the UART error interrupt.
 *
 * @param dev Pointer to the device structure.
 */
static void uart_mchp_irq_err_disable(const struct device *dev)
{
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	const hal_mchp_uart_t *const hal = &cfg->hal;

	hal_mchp_uart_enable_err_interrupt(hal, false);
}

/**
 * @brief Update UART interrupt status.
 *
 * This function clears sticky interrupts and updates the TX complete cache.
 *
 * @param dev Pointer to the device structure.
 * @return Always returns 1.
 */
static int uart_mchp_irq_update(const struct device *dev)
{
	/* Clear sticky interrupts */
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	uart_mchp_dev_data_t *const dev_data = dev->data;
	const hal_mchp_uart_t *const hal = &cfg->hal;

	/*
	 * Cache the TXC flag, and use this cached value to clear the interrupt
	 * if we do not use the cached value, there is a chance TXC will set
	 * after caching...this will cause TXC to never be cached.
	 */
	dev_data->is_tx_completed_cache = hal_mchp_uart_is_tx_complete(hal);
	hal_mchp_uart_clear_interrupts(hal);

	return 1;
}

/**
 * @brief Set UART interrupt callback.
 *
 * This function sets the callback function for UART interrupts.
 *
 * @param dev Pointer to the device structure.
 * @param cb Callback function.
 * @param cb_data User data to be passed to the callback function.
 */
static void uart_mchp_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
				       void *cb_data)
{
	uart_mchp_dev_data_t *const dev_data = dev->data;

	dev_data->cb = cb;
	dev_data->cb_data = cb_data;

#if defined(CONFIG_UART_MCHP_ASYNC) && defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS)
	dev_data->async_cb = NULL;
	dev_data->async_cb_data = NULL;
#endif
}
#endif

#ifdef CONFIG_UART_MCHP_ASYNC

/**
 * @brief Halt UART transmission.
 *
 * This function halts the UART transmission and stops the DMA transfer.
 *
 * @param dev_data Pointer to the UART device data structure.
 * @return 0 on success, negative error code on failure.
 */
static int uart_mchp_tx_halt(uart_mchp_dev_data_t *dev_data)
{
	size_t tx_active;
	struct dma_status dma_stat;
	const uart_mchp_dev_cfg_t *const cfg = dev_data->cfg;

	unsigned int key = irq_lock();

	struct uart_event evt = {
		.type = UART_TX_ABORTED,
		.data.tx =
			{
				.buf = dev_data->tx_buf,
				.len = 0U,
			},
	};

	tx_active = dev_data->tx_len;
	dev_data->tx_buf = NULL;
	dev_data->tx_len = 0U;

	dma_stop(cfg->uart_dma.dma_dev, cfg->uart_dma.tx_dma_channel);

	irq_unlock(key);

	if (dma_get_status(cfg->uart_dma.dma_dev, cfg->uart_dma.tx_dma_channel, &dma_stat) == 0) {
		evt.data.tx.len = tx_active - dma_stat.pending_length;
	}

	if (tx_active) {
		if (dev_data->async_cb) {
			dev_data->async_cb(dev_data->dev, &evt, dev_data->async_cb_data);
		}
	} else {
		return -EINVAL;
	}

	return 0;
}

/**
 * @brief Notify UART RX processed.
 *
 * This function notifies that UART RX data has been processed.
 *
 * @param dev_data Pointer to the UART device data structure.
 * @param processed Number of bytes processed.
 */
static void uart_mchp_notify_rx_processed(uart_mchp_dev_data_t *dev_data, size_t processed)
{
	if (!dev_data->async_cb) {
		return;
	}

	if (dev_data->rx_processed_len == processed) {
		return;
	}

	struct uart_event evt = {
		.type = UART_RX_RDY,
		.data.rx =
			{
				.buf = dev_data->rx_buf,
				.offset = dev_data->rx_processed_len,
				.len = processed - dev_data->rx_processed_len,
			},
	};

	dev_data->rx_processed_len = processed;

	dev_data->async_cb(dev_data->dev, &evt, dev_data->async_cb_data);
}

/**
 * @brief UART TX timeout handler.
 *
 * This function handles UART TX timeout events.
 *
 * @param work Pointer to the work structure.
 */
static void uart_mchp_tx_timeout(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	uart_mchp_dev_data_t *dev_data = CONTAINER_OF(dwork, uart_mchp_dev_data_t, tx_timeout_work);

	uart_mchp_tx_halt(dev_data);
}

/**
 * @brief UART RX timeout handler.
 *
 * This function handles UART RX timeout events.
 *
 * @param work Pointer to the work structure.
 */
static void uart_mchp_rx_timeout(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	uart_mchp_dev_data_t *dev_data = CONTAINER_OF(dwork, uart_mchp_dev_data_t, rx_timeout_work);
	const uart_mchp_dev_cfg_t *const cfg = dev_data->cfg;
	const hal_mchp_uart_t *const hal = &cfg->hal;
	struct dma_status dma_stat;
	unsigned int key = irq_lock();

	if (dev_data->rx_len == 0U) {
		irq_unlock(key);
		return;
	}

	/*
	 * Stop the DMA transfer and restart the interrupt read
	 * component (so the timeout restarts if there's still data).
	 * However, just ignore it if the transfer has completed (nothing
	 * pending) that means the DMA ISR is already pending, so just let
	 * it handle things instead when we re-enable IRQs.
	 */
	dma_stop(cfg->uart_dma.dma_dev, cfg->uart_dma.rx_dma_channel);
	if ((dma_get_status(cfg->uart_dma.dma_dev, cfg->uart_dma.rx_dma_channel, &dma_stat) == 0) &&
	    (dma_stat.pending_length == 0U)) {
		irq_unlock(key);
		return;
	}

	uint8_t *rx_dma_start = dev_data->rx_buf + dev_data->rx_len - dma_stat.pending_length;
	size_t rx_processed = rx_dma_start - dev_data->rx_buf;

	/*
	 * We know we still have space, since the above will catch the
	 * empty buffer, so always restart the transfer.
	 */
	dma_reload(cfg->uart_dma.dma_dev, cfg->uart_dma.rx_dma_channel,
		   (uint32_t)(hal_mchp_uart_get_dma_source_addr(hal)), (uint32_t)rx_dma_start,
		   dev_data->rx_len - rx_processed);

	dev_data->rx_waiting_for_irq = true;
	hal_mchp_uart_enable_rx_interrupt(hal, true);

	/*
	 * Never do a notify on a timeout started from the ISR: timing
	 * granularity means the first timeout can be in the middle
	 * of reception but still have the total elapsed time exhausted.
	 * So we require a timeout chunk with no data seen at all
	 * (i.e. no ISR entry).
	 */
	if (dev_data->rx_timeout_from_isr) {
		dev_data->rx_timeout_from_isr = false;
		k_work_reschedule(&dev_data->rx_timeout_work, K_USEC(dev_data->rx_timeout_chunk));
		irq_unlock(key);
		return;
	}

	uint32_t now = k_uptime_get_32();
	uint32_t elapsed = now - dev_data->rx_timeout_start;

	if (elapsed >= dev_data->rx_timeout_time) {
		/*
		 * No time left, so call the handler, and let the ISR
		 * restart the timeout when it sees data.
		 */
		uart_mchp_notify_rx_processed(dev_data, rx_processed);
	} else {
		/*
		 * Still have time left, so start another timeout.
		 */
		uint32_t remaining =
			MIN(dev_data->rx_timeout_time - elapsed, dev_data->rx_timeout_chunk);

		k_work_reschedule(&dev_data->rx_timeout_work, K_USEC(remaining));
	}

	irq_unlock(key);
}

/**
 * @brief UART DMA TX done handler.
 *
 * This function handles the completion of UART DMA TX transfer.
 *
 * @param dma_dev Pointer to the DMA device structure.
 * @param arg User argument.
 * @param id DMA channel ID.
 * @param error_code Error code.
 */
static void uart_mchp_dma_tx_done(const struct device *dma_dev, void *arg, uint32_t id,
				  int error_code)
{
	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);
	ARG_UNUSED(error_code);

	uart_mchp_dev_data_t *const dev_data = (uart_mchp_dev_data_t *const)arg;
	const uart_mchp_dev_cfg_t *const cfg = dev_data->cfg;

	const hal_mchp_uart_t *const hal = &cfg->hal;

	hal_mchp_uart_enable_tx_complete_interrupt(hal, true);
}

/**
 * @brief DMA RX done callback for UART.
 *
 * This function is called when the DMA transfer for UART RX is completed.
 *
 * @param dma_dev Pointer to the DMA device.
 * @param arg Pointer to the UART device data.
 * @param id DMA channel ID.
 * @param error_code Error code from the DMA transfer.
 */
static void uart_mchp_dma_rx_done(const struct device *dma_dev, void *arg, uint32_t id,
				  int error_code)
{
	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);
	ARG_UNUSED(error_code);

	uart_mchp_dev_data_t *const dev_data = (uart_mchp_dev_data_t *const)arg;
	const struct device *dev = dev_data->dev;
	const uart_mchp_dev_cfg_t *const cfg = dev_data->cfg;
	const hal_mchp_uart_t *const hal = &cfg->hal;
	unsigned int key = irq_lock();

	if (dev_data->rx_len == 0U) {
		irq_unlock(key);
		return;
	}

	uart_mchp_notify_rx_processed(dev_data, dev_data->rx_len);

	if (dev_data->async_cb) {
		struct uart_event evt = {
			.type = UART_RX_BUF_RELEASED,
			.data.rx_buf =
				{
					.buf = dev_data->rx_buf,
				},
		};

		dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
	}

	/* No next buffer, so end the transfer */
	if (!dev_data->rx_next_len) {
		dev_data->rx_buf = NULL;
		dev_data->rx_len = 0U;

		if (dev_data->async_cb) {
			struct uart_event evt = {
				.type = UART_RX_DISABLED,
			};

			dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
		}

		irq_unlock(key);
		return;
	}

	dev_data->rx_buf = dev_data->rx_next_buf;
	dev_data->rx_len = dev_data->rx_next_len;
	dev_data->rx_next_buf = NULL;
	dev_data->rx_next_len = 0U;
	dev_data->rx_processed_len = 0U;

	dma_reload(cfg->uart_dma.dma_dev, cfg->uart_dma.rx_dma_channel,
		   (uint32_t)(hal_mchp_uart_get_dma_source_addr(hal)), (uint32_t)dev_data->rx_buf,
		   dev_data->rx_len);

	/*
	 * If there should be a timeout, handle starting the DMA in the
	 * ISR, since reception resets it and DMA completion implies
	 * reception.  This also catches the case of DMA completion during
	 * timeout handling.
	 */
	if (dev_data->rx_timeout_time != SYS_FOREVER_US) {
		dev_data->rx_waiting_for_irq = true;
		hal_mchp_uart_enable_rx_interrupt(hal, true);
		irq_unlock(key);
		return;
	}

	/* Otherwise, start the transfer immediately. */
	dma_start(cfg->uart_dma.dma_dev, cfg->uart_dma.rx_dma_channel);

	struct uart_event evt = {
		.type = UART_RX_BUF_REQUEST,
	};

	dev_data->async_cb(dev, &evt, dev_data->async_cb_data);

	irq_unlock(key);
}

/**
 * @brief Set UART callback function.
 *
 * This function sets the callback function for UART events.
 *
 * @param dev Pointer to the UART device.
 * @param callback Callback function to be set.
 * @param user_data User data to be passed to the callback function.
 *
 * @return 0 on success, negative error code on failure.
 */
static int uart_mchp_callback_set(const struct device *dev, uart_callback_t callback,
				  void *user_data)
{
	uart_mchp_dev_data_t *const dev_data = dev->data;

	dev_data->async_cb = callback;
	dev_data->async_cb_data = user_data;

#if defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS)
	dev_data->cb = NULL;
	dev_data->cb_data = NULL;
#endif

	return 0;
}

/**
 * @brief Transmit data over UART.
 *
 * This function transmits data over UART using DMA.
 *
 * @param dev Pointer to the UART device.
 * @param buf Pointer to the data buffer to be transmitted.
 * @param len Length of the data buffer.
 * @param timeout Timeout for the transmission.
 *
 * @return 0 on success, negative error code on failure.
 */
static int uart_mchp_tx(const struct device *dev, const uint8_t *buf, size_t len, int32_t timeout)
{
	uart_mchp_dev_data_t *const dev_data = dev->data;
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	const hal_mchp_uart_t *const hal = &cfg->hal;
	int retval;

	if (cfg->uart_dma.tx_dma_channel == 0xFFU) {
		return -ENOTSUP;
	}

	if (len > 0xFFFFU) {
		return -EINVAL;
	}

	unsigned int key = irq_lock();

	if (dev_data->tx_len != 0U) {
		retval = -EBUSY;
		irq_unlock(key);
		return retval;
	}

	dev_data->tx_buf = buf;
	dev_data->tx_len = len;

	irq_unlock(key);

	retval = dma_reload(cfg->uart_dma.dma_dev, cfg->uart_dma.tx_dma_channel, (uint32_t)buf,
			    (uint32_t)(hal_mchp_uart_get_dma_dest_addr(hal)), len);
	if (retval != 0U) {
		return retval;
	}

	if (timeout != SYS_FOREVER_US) {
		k_work_reschedule(&dev_data->tx_timeout_work, K_USEC(timeout));
	}

	retval = dma_start(cfg->uart_dma.dma_dev, cfg->uart_dma.tx_dma_channel);
	if (retval != 0U) {
		return retval;
	}

	return 0;
}

/**
 * @brief Abort UART transmission.
 *
 * This function aborts the ongoing UART transmission.
 *
 * @param dev Pointer to the UART device.
 *
 * @return 0 on success, negative error code on failure.
 */
static int uart_mchp_tx_abort(const struct device *dev)
{
	uart_mchp_dev_data_t *const dev_data = dev->data;
	const uart_mchp_dev_cfg_t *const cfg = dev->config;

	if (cfg->uart_dma.tx_dma_channel == 0xFFU) {
		return -ENOTSUP;
	}

	k_work_cancel_delayable(&dev_data->tx_timeout_work);

	return uart_mchp_tx_halt(dev_data);
}

/**
 * @brief Provide a new RX buffer for UART.
 *
 * This function provides a new buffer for UART RX.
 *
 * @param dev Pointer to the UART device.
 * @param buf Pointer to the new RX buffer.
 * @param len Length of the new RX buffer.
 *
 * @return 0 on success, negative error code on failure.
 */
static int uart_mchp_rx_buf_rsp(const struct device *dev, uint8_t *buf, size_t len)
{
	if (len > 0xFFFFU) {
		return -EINVAL;
	}

	uart_mchp_dev_data_t *const dev_data = dev->data;
	unsigned int key = irq_lock();
	int retval = 0;

	if (dev_data->rx_len == 0U) {
		retval = -EACCES;
		irq_unlock(key);
		return retval;
	}

	if (dev_data->rx_next_len != 0U) {
		retval = -EBUSY;
		irq_unlock(key);
		return retval;
	}

	dev_data->rx_next_buf = buf;
	dev_data->rx_next_len = len;

	irq_unlock(key);
	return 0;
}

/**
 * @brief Enable UART RX.
 *
 * This function enables UART RX and sets up the RX buffer and timeout.
 *
 * @param dev Pointer to the UART device.
 * @param buf Pointer to the RX buffer.
 * @param len Length of the RX buffer.
 * @param timeout Timeout for the RX operation.
 *
 * @return 0 on success, negative error code on failure.
 */
static int uart_mchp_rx_enable(const struct device *dev, uint8_t *buf, size_t len, int32_t timeout)
{
	uart_mchp_dev_data_t *const dev_data = dev->data;
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	const hal_mchp_uart_t *const hal = &cfg->hal;
	int retval;

	if (cfg->uart_dma.rx_dma_channel == 0xFFU) {
		return -ENOTSUP;
	}

	if (len > 0xFFFFU) {
		return -EINVAL;
	}

	unsigned int key = irq_lock();

	if (dev_data->rx_len != 0U) {
		retval = -EBUSY;
		irq_unlock(key);
		return retval;
	}

	/* Read off anything that was already there */
	while (hal_mchp_uart_is_rx_complete(hal)) {
		char discard = hal_mchp_uart_get_received_char(hal);
		(void)discard;
	}

	retval = dma_reload(cfg->uart_dma.dma_dev, cfg->uart_dma.rx_dma_channel,
			    (uint32_t)(hal_mchp_uart_get_dma_source_addr(hal)), (uint32_t)buf, len);
	if (retval != 0) {
		irq_unlock(key);
		return retval;
	}

	dev_data->rx_buf = buf;
	dev_data->rx_len = len;
	dev_data->rx_processed_len = 0U;
	dev_data->rx_waiting_for_irq = true;
	dev_data->rx_timeout_from_isr = true;
	dev_data->rx_timeout_time = timeout;
	dev_data->rx_timeout_chunk = MAX(timeout / 4U, 1);

	hal_mchp_uart_enable_rx_interrupt(hal, true);

	irq_unlock(key);
	return 0;
}

/**
 * @brief Disable UART RX.
 *
 * This function disables UART RX and stops the DMA transfer.
 *
 * @param dev Pointer to the UART device.
 *
 * @return 0 on success, negative error code on failure.
 */
static int uart_mchp_rx_disable(const struct device *dev)
{
	uart_mchp_dev_data_t *const dev_data = dev->data;
	const uart_mchp_dev_cfg_t *const cfg = dev->config;
	const hal_mchp_uart_t *const hal = &cfg->hal;
	struct dma_status dma_stat;

	k_work_cancel_delayable(&dev_data->rx_timeout_work);

	unsigned int key = irq_lock();

	if (dev_data->rx_len == 0U) {
		irq_unlock(key);
		return -EFAULT;
	}

	hal_mchp_uart_enable_rx_interrupt(hal, false);
	dma_stop(cfg->uart_dma.dma_dev, cfg->uart_dma.rx_dma_channel);

	if ((dma_get_status(cfg->uart_dma.dma_dev, cfg->uart_dma.rx_dma_channel, &dma_stat) == 0) &&
	    (dma_stat.pending_length != 0U)) {
		size_t rx_processed = dev_data->rx_len - dma_stat.pending_length;

		uart_mchp_notify_rx_processed(dev_data, rx_processed);
	}

	struct uart_event evt = {
		.type = UART_RX_BUF_RELEASED,
		.data.rx_buf =
			{
				.buf = dev_data->rx_buf,
			},
	};

	dev_data->rx_buf = NULL;
	dev_data->rx_len = 0U;

	if (dev_data->async_cb) {
		dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
	}

	if (dev_data->rx_next_len) {
		struct uart_event next_evt = {
			.type = UART_RX_BUF_RELEASED,
			.data.rx_buf =
				{
					.buf = dev_data->rx_next_buf,
				},
		};

		dev_data->rx_next_buf = NULL;
		dev_data->rx_next_len = 0U;

		if (dev_data->async_cb) {
			dev_data->async_cb(dev, &next_evt, dev_data->async_cb_data);
		}
	}

	evt.type = UART_RX_DISABLED;
	if (dev_data->async_cb) {
		dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
	}

	irq_unlock(key);

	return 0;
}

#endif

static const struct uart_driver_api uart_mchp_driver_api = {
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_mchp_configure,
	.config_get = uart_mchp_config_get,
#endif

	.poll_in = uart_mchp_poll_in,
	.poll_out = uart_mchp_poll_out,
	.err_check = uart_mchp_err_check,

#if CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_mchp_fifo_fill,
	.fifo_read = uart_mchp_fifo_read,
	.irq_tx_enable = uart_mchp_irq_tx_enable,
	.irq_tx_disable = uart_mchp_irq_tx_disable,
	.irq_tx_ready = uart_mchp_irq_tx_ready,
	.irq_tx_complete = uart_mchp_irq_tx_complete,
	.irq_rx_enable = uart_mchp_irq_rx_enable,
	.irq_rx_disable = uart_mchp_irq_rx_disable,
	.irq_rx_ready = uart_mchp_irq_rx_ready,
	.irq_is_pending = uart_mchp_irq_is_pending,
	.irq_err_enable = uart_mchp_irq_err_enable,
	.irq_err_disable = uart_mchp_irq_err_disable,
	.irq_update = uart_mchp_irq_update,
	.irq_callback_set = uart_mchp_irq_callback_set,
#endif

#if CONFIG_UART_MCHP_ASYNC
	.callback_set = uart_mchp_callback_set,
	.tx = uart_mchp_tx,
	.tx_abort = uart_mchp_tx_abort,
	.rx_enable = uart_mchp_rx_enable,
	.rx_buf_rsp = uart_mchp_rx_buf_rsp,
	.rx_disable = uart_mchp_rx_disable,
#endif
};

#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_MCHP_ASYNC

#define MCHP_UART_IRQ_CONNECT(n, m)                                                                \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, m, irq), DT_INST_IRQ_BY_IDX(n, m, priority),     \
			    uart_mchp_isr, DEVICE_DT_INST_GET(n), 0);                              \
		irq_enable(DT_INST_IRQ_BY_IDX(n, m, irq));                                         \
	} while (false)

#define UART_MCHP_IRQ_HANDLER_DECL(n) static void uart_mchp_irq_config_##n(const struct device *dev)
#define UART_MCHP_IRQ_HANDLER_FUNC(n) .irq_config_func = uart_mchp_irq_config_##n,

#else
#define UART_MCHP_IRQ_HANDLER_DECL(n)
#define UART_MCHP_IRQ_HANDLER_FUNC(n)
#endif

#if CONFIG_UART_MCHP_ASYNC
#define UART_MCHP_DMA_CHANNELS(n)                                                                  \
	.uart_dma.dma_dev = DEVICE_DT_GET(MCHP_DT_INST_DMA_CTLR(n, tx)),                           \
	.uart_dma.tx_dma_request = MCHP_DT_INST_DMA_TRIGSRC(n, tx),                                \
	.uart_dma.tx_dma_channel = MCHP_DT_INST_DMA_CHANNEL(n, tx),                                \
	.uart_dma.rx_dma_request = MCHP_DT_INST_DMA_TRIGSRC(n, rx),                                \
	.uart_dma.rx_dma_channel = MCHP_DT_INST_DMA_CHANNEL(n, rx),
#else
#define UART_MCHP_DMA_CHANNELS(n)
#endif /* CONFIG_UART_MCHP_ASYNC */

#define UART_MCHP_CONFIG_DEFN(n)                                                                   \
	static const uart_mchp_dev_cfg_t uart_mchp_config_##n = {                                  \
		.baudrate = DT_INST_PROP(n, current_speed),                                        \
		.data_bits = DT_INST_ENUM_IDX_OR(n, data_bits, UART_CFG_DATA_BITS_8),              \
		.parity = DT_INST_ENUM_IDX_OR(n, parity, UART_CFG_PARITY_NONE),                    \
		.stop_bits = DT_INST_ENUM_IDX_OR(n, stop_bits, UART_CFG_STOP_BITS_1),              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		UART_MCHP_HAL_DEFN(n) UART_MCHP_IRQ_HANDLER_FUNC(n) UART_MCHP_DMA_CHANNELS(n)      \
			UART_MCHP_CLOCK_DEFN(n)}

#define UART_MCHP_DEVICE_INIT(n)                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	UART_MCHP_IRQ_HANDLER_DECL(n);                                                             \
	UART_MCHP_CONFIG_DEFN(n);                                                                  \
	static uart_mchp_dev_data_t uart_mchp_data_##n;                                            \
	DEVICE_DT_INST_DEFINE(n, uart_mchp_init, NULL, &uart_mchp_data_##n, &uart_mchp_config_##n, \
			      PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY, &uart_mchp_driver_api);   \
	UART_MCHP_IRQ_HANDLER(n)

DT_INST_FOREACH_STATUS_OKAY(UART_MCHP_DEVICE_INIT)
