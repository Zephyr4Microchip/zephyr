/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

#include "app_ble_handler.h"
#include "app_ble_discovery.h"
#include "app_ble_policy.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_ble_handler);

static struct bt_conn *app_ble_conn;
/* Spinlock rather than mutex: app_ble_handler_is_connected() is called from
 * uart_isr() in ISR context where a mutex cannot be acquired. */
static struct k_spinlock app_ble_lock;
static app_ble_conn_state_cb_t app_ble_state_cb;
static void *app_ble_state_cb_ctx;
static bool app_ble_initialized;

static void app_ble_notify_state(bool connected, uint8_t reason)
{
	if (app_ble_state_cb != NULL) {
		app_ble_state_cb(connected, reason, app_ble_state_cb_ctx);
	}
}

static void app_ble_connected(struct bt_conn *conn, uint8_t err)
{
	bool accepted = false;

	if (err != 0U) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	{
		k_spinlock_key_t key = k_spin_lock(&app_ble_lock);

		if (app_ble_conn == NULL) {
			/* Ref held until app_ble_disconnected(); released there via bt_conn_unref(). */
			app_ble_conn = bt_conn_ref(conn);
			accepted = true;
		}

		/* Lock released before calling policy and notify to avoid
		 * holding the spinlock across BT-stack re-entrant calls. */
		k_spin_unlock(&app_ble_lock, key);
	}

	if (!accepted) {
		/* Only one connection is supported (CONFIG_BT_MAX_CONN=1).
		 * The extra connection is ignored; it will time out naturally. */
		LOG_WRN("Additional BLE connection ignored");
		return;
	}

	LOG_INF("Connected");
	app_ble_policy_on_connected(conn);
	app_ble_notify_state(true, 0U);
}

static void app_ble_disconnected(struct bt_conn *conn, uint8_t reason)
{
	struct bt_conn *to_unref = NULL;

	{
		k_spinlock_key_t key = k_spin_lock(&app_ble_lock);

		if ((app_ble_conn != NULL) && (app_ble_conn == conn)) {
			/* Defer bt_conn_unref() to after the spinlock is released.
			 * Calling it under the lock risks re-entrant BT-stack locking. */
			to_unref = app_ble_conn;
			app_ble_conn = NULL;
		}

		k_spin_unlock(&app_ble_lock, key);
	}

	if (to_unref != NULL) {
		app_ble_discovery_on_disconnected(conn, reason);
		app_ble_policy_on_disconnected(conn, reason);
		bt_conn_unref(to_unref);
		app_ble_notify_state(false, reason);
	}

	LOG_INF("Disconnected, reason %u %s", reason, bt_hci_err_to_str(reason));
}

static struct bt_conn_cb app_ble_conn_cbs = {
	.connected = app_ble_connected,
	.disconnected = app_ble_disconnected,
};

static void app_ble_auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Pairing cancelled: %s", addr);
}

static struct bt_conn_auth_cb app_ble_auth_cb = {
	.cancel = app_ble_auth_cancel,
};

int app_ble_handler_init(app_ble_conn_state_cb_t state_cb, void *state_cb_ctx)
{
	int rc;

	app_ble_state_cb = state_cb;
	app_ble_state_cb_ctx = state_cb_ctx;

	if (app_ble_initialized) {
		return 0;
	}

	bt_conn_cb_register(&app_ble_conn_cbs);

	rc = bt_conn_auth_cb_register(&app_ble_auth_cb);
	if ((rc != 0) && (rc != -EALREADY)) {
		LOG_ERR("Failed to register auth callback: %d", rc);
		return rc;
	}

	rc = app_ble_policy_init();
	if (rc != 0) {
		LOG_ERR("Failed to initialize BLE policy: %d", rc);
		return rc;
	}

	rc = app_ble_discovery_init(NULL, NULL);
	if (rc != 0) {
		LOG_ERR("Failed to initialize BLE discovery: %d", rc);
		return rc;
	}

	app_ble_initialized = true;
	return 0;
}

/* Safe to call from ISR context; uses a spinlock and never blocks. */
bool app_ble_handler_is_connected(void)
{
	bool connected;
	k_spinlock_key_t key = k_spin_lock(&app_ble_lock);

	connected = (app_ble_conn != NULL);

	k_spin_unlock(&app_ble_lock, key);
	return connected;
}

/* Caller must call bt_conn_unref() when done.
 * Returns -ENOTCONN if no connection is active, -EINVAL if conn is NULL. */
int app_ble_handler_conn_ref_get(struct bt_conn **conn)
{
	if (conn == NULL) {
		return -EINVAL;
	}

	*conn = NULL;

	{
		k_spinlock_key_t key = k_spin_lock(&app_ble_lock);

		if (app_ble_conn == NULL) {
			k_spin_unlock(&app_ble_lock, key);
			return -ENOTCONN;
		}

		/* Ref incremented under lock to prevent release between NULL check and bt_conn_ref(). */
		*conn = bt_conn_ref(app_ble_conn);

		k_spin_unlock(&app_ble_lock, key);
	}

	return 0;
}
