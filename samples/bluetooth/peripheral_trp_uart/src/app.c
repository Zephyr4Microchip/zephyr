/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/uart.h>

#include "app.h"
#include <zephyr/bluetooth/services/trs/trs_server.h>
#include "app_ble.h"
#include "app_ble_handler.h"
#include "app_trsps_handler.h"

LOG_MODULE_REGISTER(app);

#define UART_TX_BUF_SIZE       244
#define RING_BUF_SIZE          512
#define UART_TX_RETRY_DELAY_MS 10U

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

/* ISR is the sole producer (ring_buf_put_claim/finish); work queue is the
 * sole consumer (ring_buf_get).  Safe on single-core without an explicit lock. */
RING_BUF_DECLARE(uart_to_ble_ring, RING_BUF_SIZE);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* [work-queue context] UART→BLE notify pump. */
static struct k_work uart_to_ble_work;
static struct k_work_delayable uart_to_ble_retry_work;
/* Pending retry chunk — work-queue context only, no lock needed. */
static uint8_t uart_to_ble_pending[UART_TX_BUF_SIZE];
static size_t uart_to_ble_pending_len;

/* [work-queue context]: BLE notify APIs are not ISR-safe. */
static void uart_to_ble_work_handler(struct k_work *work)
{
	struct bt_conn *conn;
	int err;
	int conn_err;
	uint16_t tx_mtu = 0U;
	size_t bytes_read = 0;
	size_t payload_mtu;
	uint8_t tx_buff[UART_TX_BUF_SIZE] = {0U};

	ARG_UNUSED(work);

	/* Acquires a reference; conn must be released at 'out' on every exit path. */
	conn_err = app_ble_handler_conn_ref_get(&conn);
	if (conn_err != 0) {
		return;
	}

	trs_server_get_tx_mtu(conn, &tx_mtu);

	/* ATT notification header is 3 bytes; guard against zero or negative
	 * payload.  MTU may still be at the default 23 if exchange has not
	 * completed yet — retry after a short delay. */
	if (tx_mtu <= 4U) {
		(void)k_work_reschedule(&uart_to_ble_retry_work, K_MSEC(UART_TX_RETRY_DELAY_MS));
		goto out;
	}

	payload_mtu = MIN((size_t)(tx_mtu - 4U), sizeof(tx_buff));

	/* Send any previously failed chunk first to preserve UART→BLE byte ordering
	 * before consuming new data from the ring buffer. */
	if (uart_to_ble_pending_len > 0U) {
		err = trs_server_send_data(conn, (uint16_t)uart_to_ble_pending_len,
					   uart_to_ble_pending);
		if (err < 0) {
			/* -ENOBUFS/-ENOTSUP/-EAGAIN are transient BLE stack conditions; retry. */
			if ((err == -ENOBUFS) || (err == -ENOTSUP) || (err == -EAGAIN)) {
				(void)k_work_reschedule(&uart_to_ble_retry_work,
							K_MSEC(UART_TX_RETRY_DELAY_MS));
			}
			goto out;
		}

		uart_to_ble_pending_len = 0U;
	}

	while ((bytes_read = ring_buf_get(&uart_to_ble_ring, &tx_buff[0], payload_mtu)) > 0U) {
		err = trs_server_send_data(conn, (uint16_t)bytes_read, &tx_buff[0]);
		if (err < 0) {
			/* Save the failed chunk so the retry handler can resend it
			 * without losing UART data. */
			if (bytes_read <= sizeof(uart_to_ble_pending)) {
				(void)memcpy(uart_to_ble_pending, tx_buff, bytes_read);
				uart_to_ble_pending_len = bytes_read;
			}

			if ((err == -ENOBUFS) || (err == -ENOTSUP) || (err == -EAGAIN)) {
				(void)k_work_reschedule(&uart_to_ble_retry_work,
							K_MSEC(UART_TX_RETRY_DELAY_MS));
			} else {
				LOG_ERR("UART->BLE notify failed (err %d)", err);
			}
			break;
		}
	}

out:
	bt_conn_unref(conn);
}

static void uart_to_ble_retry_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	k_work_submit(&uart_to_ble_work);
}

/* TX channel open or CBFC credits granted: start UART→BLE pump. */
static void on_ble_tx_channel_ready(void *context)
{
	ARG_UNUSED(context);
	k_work_submit(&uart_to_ble_work);
}

static void app_ble_state_changed(bool connected, uint8_t reason, void *context)
{
	ARG_UNUSED(reason);
	ARG_UNUSED(context);

	if (connected) {
		/* Drain any UART data buffered before the connection was established. */
		k_work_submit(&uart_to_ble_work);
		return;
	}

	/* Discard pending UART→BLE retry data and stop the timer; the connection
	 * the data was queued for no longer exists. */
	uart_to_ble_pending_len = 0U;
	(void)k_work_cancel_delayable(&uart_to_ble_retry_work);

	/* Schedule deferred advertising restart so it runs after the BT stack
	 * has fully released the connection resources (avoids -ENOMEM). */
	app_ble_adv_restart();
}

static void uart_read_to_ringbuf(void)
{
	uint8_t buf[32U] = {0};
	uint8_t *ptr;
	size_t claimed = 0;
	int bytes_read;
	int bytes_to_store;

	bytes_read = uart_fifo_read(uart_dev, buf, sizeof(buf));
	if (bytes_read > 0) {
		/* ring_buf_put_claim returns a single contiguous window and may
		 * return less than requested at the wrap-around boundary.
		 * Excess bytes are dropped; the overflow log indicates data loss. */
		claimed = ring_buf_put_claim(&uart_to_ble_ring, &ptr, (uint32_t)bytes_read);
		if (claimed > 0U) {
			bytes_to_store = (int)MIN(claimed, (size_t)bytes_read);
			(void)memcpy(ptr, buf, (size_t)bytes_to_store);
			ring_buf_put_finish(&uart_to_ble_ring, (uint32_t)bytes_to_store);
			if (bytes_read > bytes_to_store) {
				LOG_ERR("uart_to_ble_ring overflow: UART data dropped");
			}
		} else {
			LOG_ERR("uart_to_ble_ring full: UART data dropped");
		}
	}
}

/* [ISR context] UART RX/TX interrupt dispatcher. */
static void uart_isr(const struct device *unused, void *user_data)
{
	ARG_UNUSED(unused);
	ARG_UNUSED(user_data);

	/* Loop until all pending flags are cleared.  A single IRQ entry can
	 * represent simultaneous RX-ready and TX-empty events on most controllers;
	 * processing both in one pass reduces re-entry overhead. */
	while (uart_irq_update(uart_dev) && uart_irq_is_pending(uart_dev)) {
		if (!(uart_irq_rx_ready(uart_dev) || uart_irq_tx_ready(uart_dev))) {
			LOG_INF("spurious interrupt");
		}

		/* BLE→UART direction: feed queued BLE data into the UART TX FIFO. */
		if (uart_irq_tx_ready(uart_dev)) {
			app_trsps_handler_ble_to_uart_service();
		}

		/* UART→BLE direction: drain UART RX FIFO into ring buffer. */
		if (uart_irq_rx_ready(uart_dev)) {
			uart_read_to_ringbuf();
			/* k_work_submit and app_ble_handler_is_connected() are both ISR-safe. */
			if (app_ble_handler_is_connected()) {
				k_work_submit(&uart_to_ble_work);
			}
		}
	}
}

static int uart_init(void)
{
	if (!device_is_ready(uart_dev)) {
		return -ENODEV;
	}

	/* Disable both interrupts before registering the callback to prevent a
	 * stray IRQ firing with a NULL handler if a bootloader left them enabled. */
	uart_irq_rx_disable(uart_dev);
	uart_irq_tx_disable(uart_dev);

	uart_irq_callback_set(uart_dev, uart_isr);
	uart_irq_rx_enable(uart_dev);
	/* TX interrupt enabled on-demand by ble_to_uart_enqueue() only when
	 * BLE→UART data is ready, preventing a continuous empty-FIFO IRQ loop. */

	return 0;
}

int app_init(void)
{
	int err;

	/* Work items must be initialized before uart_init() arms the RX interrupt.
	 * A byte arriving immediately after uart_irq_rx_enable() would call
	 * k_work_submit() on an uninitialized struct, causing undefined behavior. */
	k_work_init(&uart_to_ble_work, uart_to_ble_work_handler);
	k_work_init_delayable(&uart_to_ble_retry_work, uart_to_ble_retry_handler);

	err = uart_init();
	if (err != 0) {
		return err;
	}

	/* BLE connection callbacks must be registered before bt_enable() so no
	 * connect or disconnect event is missed during the startup window. */
	err = app_ble_handler_init(app_ble_state_changed, NULL);
	if (err != 0) {
		LOG_ERR("BLE handler init failed (err %d)", err);
		return err;
	}

	/* TRS server callback must be registered before bt_enable() so the first
	 * GATT write after connection is not silently dropped. */
	err = app_trsps_handler_init(uart_dev, on_ble_tx_channel_ready, NULL);
	if (err != 0) {
		LOG_ERR("TRSPS handler init failed (err %d)", err);
		return err;
	}

	/* bt_enable() is asynchronous; advertising starts from app_ble_ready_cb()
	 * in the BT RX thread after this function has already returned. */
	err = app_ble_init();
	if (err != 0) {
		LOG_ERR("BLE init failed (err %d)", err);
		return err;
	}

	return 0;
}
