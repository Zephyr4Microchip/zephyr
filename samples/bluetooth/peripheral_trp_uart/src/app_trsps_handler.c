/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/check.h>

#include <zephyr/bluetooth/services/trs/trs_server.h>
#include "app_trsps_handler.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_trsps_handler);

#define APP_TRSPS_BUF_POOL_SIZE      CONFIG_TRS_SERVER_BUF_POOL_SIZE
#define APP_TRSPS_BUF_DATA_SIZE      CONFIG_TRS_SERVER_BUF_DATA_SIZE
#define APP_TRSPS_RX_CONN_QUEUE_LEN  (CONFIG_BT_MAX_CONN * 4U)

/* Pool sized to match TRS_SERVER_CREDIT so each in-flight CBFC credit slot
 * can hold one buffer.  K_FOREVER allocs in the work queue will block if all
 * buffers are held by a stalled UART TX path, stalling all other work items. */
NET_BUF_POOL_DEFINE(ble_to_uart_buf_pool, APP_TRSPS_BUF_POOL_SIZE,
		    APP_TRSPS_BUF_DATA_SIZE, 0, NULL);

/* BLE→UART direction:
 * Producer: work-queue context (ble_to_uart_enqueue, protected by irq_lock).
 * Consumer: ISR context (app_trsps_handler_ble_to_uart_service, K_NO_WAIT). */
static K_FIFO_DEFINE(ble_to_uart_fifo);

/* Passes bt_conn references from BT RX thread to the BLE→UART work queue.
 * Each entry holds a ref that the work-queue consumer must release. */
K_MSGQ_DEFINE(ble_rx_conn_msgq, sizeof(struct bt_conn *),
	      APP_TRSPS_RX_CONN_QUEUE_LEN, sizeof(void *));

/* [work-queue context] BLE→UART drain work. */
static struct k_work ble_to_uart_work;
static const struct device *app_trsps_uart_dev;
static app_trsps_uart_to_ble_ready_cb_t uart_to_ble_ready_cb;
static void *uart_to_ble_ready_cb_ctx;
static bool app_trsps_initialized;

/* [BT RX thread] Invoke UART→BLE ready callback. */
static void trigger_uart_to_ble_pump(void)
{
	if (uart_to_ble_ready_cb != NULL) {
		uart_to_ble_ready_cb(uart_to_ble_ready_cb_ctx);
	}
}

/* Enqueue BLE→UART buffer; enable TX IRQ on empty→non-empty transition. [work-queue context] */
static void ble_to_uart_enqueue(struct net_buf *buf)
{
	unsigned int key;
	bool was_empty;

	if (buf == NULL) {
		return;
	}

	/* irq_lock() rather than spinlock: the consumer runs in ISR context.
	 * The critical section covers the empty check and tx_enable together
	 * to prevent enabling the IRQ before the buffer is visible to the ISR. */
	key = irq_lock();
	was_empty = k_fifo_is_empty(&ble_to_uart_fifo);
	k_fifo_put(&ble_to_uart_fifo, buf);
	if (was_empty && (app_trsps_uart_dev != NULL)) {
		/* Enable TX interrupt only on the empty→non-empty transition.
		 * The ISR disables it again when the FIFO drains to avoid a busy loop. */
		uart_irq_tx_enable(app_trsps_uart_dev);
	}
	irq_unlock(key);
}

/* [work-queue context] Drain BLE receive queue for conn to UART. */
static void ble_to_uart_drain_conn(struct bt_conn *conn)
{
	while (true) {
		uint16_t data_len;
		int copied_len;
		struct net_buf *buf;

		trs_server_get_data_len(conn, &data_len);
		if (data_len == 0U) {
			return;
		}

		/* K_FOREVER: blocks the work queue until a buffer is available.
		 * If the UART TX path is stalled, this will stall all other work items
		 * on the system queue.  Pool depth matches CBFC credit count to bound
		 * the maximum outstanding allocation. */
		buf = net_buf_alloc_len(&ble_to_uart_buf_pool, (size_t)data_len, K_FOREVER);
		if (buf == NULL) {
			LOG_ERR("BLE->UART: no buffer for %u-byte payload", data_len);
			return;
		}

		/* net_buf_add → write pointer; trs_server_get_data copies and updates CBFC credits. */
		copied_len = trs_server_get_data(conn, net_buf_add(buf, (size_t)data_len));
		if (copied_len <= 0) {
			net_buf_unref(buf);
			return;
		}

		ble_to_uart_enqueue(buf);
	}
}

static void ble_to_uart_work_handler(struct k_work *work)
{
	struct bt_conn *conn;

	ARG_UNUSED(work);

	while (k_msgq_get(&ble_rx_conn_msgq, &conn, K_NO_WAIT) == 0) {
		ble_to_uart_drain_conn(conn);
		/* Release the ref taken in trs_event_handler(). */
		bt_conn_unref(conn);
	}
}

/* [BT RX thread] TRS server event dispatcher. */
static void trs_event_handler(struct trs_server_event *event)
{
	struct bt_conn *conn_ref;

	if (event == NULL) {
		return;
	}

	switch (event->event_id) {
	case TRS_SERVER_EVT_RECEIVE_DATA:
		/* BLE→UART direction: BLE Central wrote data; queue conn ref for work-queue processing. */
		if (event->event_field.conn == NULL) {
			break;
		}

		/* Take a ref before posting to the queue; the event pointer is
		 * valid only for the duration of this callback. */
		conn_ref = bt_conn_ref(event->event_field.conn);
		if (k_msgq_put(&ble_rx_conn_msgq, &conn_ref, K_NO_WAIT) != 0) {
			/* Queue full: release the ref immediately to avoid a leak.
			 * The receive event is dropped; BLE data for this packet is lost. */
			bt_conn_unref(conn_ref);
			LOG_WRN("BLE->UART: ble_rx_conn_msgq full, dropping receive event");
			break;
		}

		k_work_submit(&ble_to_uart_work);
		break;

	case TRS_SERVER_EVT_CTRL_STATUS:
		LOG_INF("Control point status: %s",
			(event->event_field.ctrl_status.status == TRS_SERVER_STATUS_CTRL_OPENED)
				? "opened"
				: "disabled");
		break;

	case TRS_SERVER_EVT_TX_STATUS:
		/* UART→BLE channel state changed (Central wrote TX CCCD). */
		LOG_INF("UART->BLE TX channel: %s",
			(event->event_field.tx_status.status == TRS_SERVER_STATUS_TX_OPENED)
				? "opened"
				: "disabled");
		if (event->event_field.tx_status.status == TRS_SERVER_STATUS_TX_OPENED) {
			trigger_uart_to_ble_pump();
		}
		break;

	case TRS_SERVER_EVT_CBFC_ENABLED:
		LOG_INF("CBFC BLE->UART Rx enabled");
		break;

	case TRS_SERVER_EVT_CBFC_CREDIT:
		/* UART→BLE direction: peer granted more TX credits; resume sending. */
		LOG_INF("CBFC UART->BLE Tx credit updated");
		trigger_uart_to_ble_pump();
		break;

	case TRS_SERVER_EVT_VENDOR_CMD:
		LOG_INF("Vendor command received (length = %d)",
			event->event_field.vendor_cmd.length);
		break;

	case TRS_SERVER_EVT_ERR_UNSPECIFIED:
		LOG_INF("TRS unspecified error");
		break;

	case TRS_SERVER_EVT_ERR_NO_MEM:
		LOG_INF("TRS no memory");
		break;

	default:
		break;
	}
}

static struct trs_server_cb trs_evt_cb = {
	.trs_server_event = trs_event_handler,
};

int app_trsps_handler_init(const struct device *uart_dev,
			   app_trsps_uart_to_ble_ready_cb_t uart_to_ble_ready_cb_arg,
			   void *uart_to_ble_ready_cb_ctx_arg)
{
	int err;

	/* NULL uart_dev would crash uart_irq_tx_enable() in the TX ISR path. */
	if (uart_dev == NULL) {
		return -EINVAL;
	}

	/* trs_server_cb_register() must not be called twice; node insertion is not idempotent. */
	if (app_trsps_initialized) {
		return 0;
	}

	app_trsps_uart_dev       = uart_dev;
	uart_to_ble_ready_cb     = uart_to_ble_ready_cb_arg;
	uart_to_ble_ready_cb_ctx = uart_to_ble_ready_cb_ctx_arg;

	/* Submitted later by trs_event_handler() on each BLE packet arrival. */
	k_work_init(&ble_to_uart_work, ble_to_uart_work_handler);

	err = trs_server_cb_register(&trs_evt_cb);
	if (err != 0) {
		return err;
	}

	app_trsps_initialized = true;
	return 0;
}

/* [ISR context] Service UART TX: drain ble_to_uart_fifo one net_buf at a time. */
void app_trsps_handler_ble_to_uart_service(void)
{
	/* static: persists across ISR calls so a partially-written buffer
	 * is resumed on the next TX-ready interrupt without re-queuing. */
	static struct net_buf *ble_to_uart_buf;
	int len;

	if (app_trsps_uart_dev == NULL) {
		return;
	}

	if (ble_to_uart_buf == NULL) {
		ble_to_uart_buf = k_fifo_get(&ble_to_uart_fifo, K_NO_WAIT);
		if (ble_to_uart_buf == NULL) {
			/* FIFO is empty; disable TX interrupt until next BLE→UART data arrives.
			 * ble_to_uart_enqueue() re-enables it on next data. */
			uart_irq_tx_disable(app_trsps_uart_dev);
			return;
		}
	}

	len = uart_fifo_fill(app_trsps_uart_dev, ble_to_uart_buf->data, ble_to_uart_buf->len);
	net_buf_pull(ble_to_uart_buf, len);
	if (ble_to_uart_buf->len == 0U) {
		/* Buffer fully consumed; return it to the pool.  This may unblock
		 * a K_FOREVER alloc waiting in ble_to_uart_drain_conn(). */
		net_buf_unref(ble_to_uart_buf);
		ble_to_uart_buf = NULL;
	}
}
