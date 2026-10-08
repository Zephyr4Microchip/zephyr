/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/bluetooth/services/trs/trs.h>
#include "app_ble_discovery.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_ble_discovery);

static struct bt_uuid_128 app_ble_discovery_uuid_trs = BT_UUID_INIT_128(BT_UUID_TRS_VAL);
static struct bt_uuid_128 app_ble_discovery_uuid_tx = BT_UUID_INIT_128(BT_UUID_TX_CHAR_VAL);
static struct bt_uuid_128 app_ble_discovery_uuid_rx = BT_UUID_INIT_128(BT_UUID_RX_CHAR_VAL);
static struct bt_uuid_128 app_ble_discovery_uuid_cp = BT_UUID_INIT_128(BT_UUID_CP_CHAR_VAL);

static struct k_spinlock app_ble_discovery_lock;
static struct bt_conn *app_ble_discovery_conn;
/* Must remain valid for the full discovery sequence; bt_gatt_discover() holds
 * a pointer to this struct until the callback signals completion. */
static struct bt_gatt_discover_params app_ble_discover_params;
static struct app_ble_discovery_result app_ble_discovery_result;
static app_ble_discovery_cb_t app_ble_discovery_cb;
static void *app_ble_discovery_cb_ctx;
static bool app_ble_discovery_initialized;
static bool app_ble_discovery_in_progress;
static bool app_ble_discovery_completed;
static int app_ble_discovery_status;

static void app_ble_discovery_reset_result(void)
{
	(void)memset(&app_ble_discovery_result, 0, sizeof(app_ble_discovery_result));
}

static bool app_ble_discovery_has_trs_handles(void)
{
	return (app_ble_discovery_result.tx_value_handle != 0U) &&
	       (app_ble_discovery_result.rx_value_handle != 0U) &&
	       (app_ble_discovery_result.cp_value_handle != 0U);
}

static void app_ble_discovery_notify(struct bt_conn *conn, int status)
{
	app_ble_discovery_cb_t cb;
	void *cb_ctx;
	struct app_ble_discovery_result result_snapshot;

	{
		k_spinlock_key_t key = k_spin_lock(&app_ble_discovery_lock);

		cb = app_ble_discovery_cb;
		cb_ctx = app_ble_discovery_cb_ctx;
		result_snapshot = app_ble_discovery_result;

		k_spin_unlock(&app_ble_discovery_lock, key);
	}

	if (cb != NULL) {
		cb(conn, &result_snapshot, status, cb_ctx);
	}
}

static void app_ble_discovery_complete(struct bt_conn *conn, int status)
{
	{
		k_spinlock_key_t key = k_spin_lock(&app_ble_discovery_lock);

		app_ble_discovery_in_progress = false;
		app_ble_discovery_completed = true;
		app_ble_discovery_status = status;
		(void)memset(&app_ble_discover_params, 0, sizeof(app_ble_discover_params));

		k_spin_unlock(&app_ble_discovery_lock, key);
	}

	app_ble_discovery_notify(conn, status);
}

/* GATT discovery callback: NULL attr signals end of this discovery phase. */
static uint8_t app_ble_discover_func(struct bt_conn *conn,
				     const struct bt_gatt_attr *attr,
				     struct bt_gatt_discover_params *params)
{
	uint8_t discover_type;

	if (params == NULL) {
		return BT_GATT_ITER_STOP;
	}

	discover_type = params->type;

	if (attr == NULL) {
		if (discover_type == BT_GATT_DISCOVER_PRIMARY) {
			LOG_INF("TRS discovery complete: service not found");
			app_ble_discovery_complete(conn, -ENOENT);
		} else if (discover_type == BT_GATT_DISCOVER_CHARACTERISTIC) {
			int status = app_ble_discovery_has_trs_handles() ? 0 : -ENOENT;

			if (status == 0) {
				LOG_INF("TRS discovery complete: TX=0x%04x RX=0x%04x CP=0x%04x",
					app_ble_discovery_result.tx_value_handle,
					app_ble_discovery_result.rx_value_handle,
					app_ble_discovery_result.cp_value_handle);
			} else {
				LOG_WRN("TRS discovery incomplete: missing one or more characteristic handles");
			}
			app_ble_discovery_complete(conn, status);
		}

		return BT_GATT_ITER_STOP;
	}

	if (discover_type == BT_GATT_DISCOVER_PRIMARY) {
		const struct bt_gatt_service_val *svc = attr->user_data;
		int err;

		{
			k_spinlock_key_t key = k_spin_lock(&app_ble_discovery_lock);

			app_ble_discovery_result.svc_start_handle = attr->handle;
			app_ble_discovery_result.svc_end_handle = (svc != NULL)
								      ? svc->end_handle
								      : BT_ATT_LAST_ATTRIBUTE_HANDLE;

			k_spin_unlock(&app_ble_discovery_lock, key);
		}

		params->uuid = NULL;
		params->start_handle = attr->handle + 1U;
		params->end_handle = (svc != NULL) ? svc->end_handle : BT_ATT_LAST_ATTRIBUTE_HANDLE;
		params->type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, params);
		if (err != 0) {
			LOG_ERR("TRS characteristic discovery failed to start (%d)", err);
			app_ble_discovery_complete(conn, err);
		}

		return BT_GATT_ITER_STOP;
	}

	if (discover_type == BT_GATT_DISCOVER_CHARACTERISTIC) {
		const struct bt_gatt_chrc *chrc = attr->user_data;

		if (chrc == NULL) {
			return BT_GATT_ITER_CONTINUE;
		}

		{
			k_spinlock_key_t key = k_spin_lock(&app_ble_discovery_lock);

			if (!bt_uuid_cmp(chrc->uuid, &app_ble_discovery_uuid_tx.uuid)) {
				app_ble_discovery_result.tx_value_handle = chrc->value_handle;
			} else if (!bt_uuid_cmp(chrc->uuid, &app_ble_discovery_uuid_rx.uuid)) {
				app_ble_discovery_result.rx_value_handle = chrc->value_handle;
			} else if (!bt_uuid_cmp(chrc->uuid, &app_ble_discovery_uuid_cp.uuid)) {
				app_ble_discovery_result.cp_value_handle = chrc->value_handle;
			}

			k_spin_unlock(&app_ble_discovery_lock, key);
		}

		return BT_GATT_ITER_CONTINUE;
	}

	return BT_GATT_ITER_STOP;
}

/* Registers the optional result callback; idempotent on repeated calls. */
int app_ble_discovery_init(app_ble_discovery_cb_t cb, void *cb_ctx)
{
	{
		k_spinlock_key_t key = k_spin_lock(&app_ble_discovery_lock);

		app_ble_discovery_cb = cb;
		app_ble_discovery_cb_ctx = cb_ctx;

		if (app_ble_discovery_initialized) {
			k_spin_unlock(&app_ble_discovery_lock, key);
			return 0;
		}

		app_ble_discovery_status = -EAGAIN;
		app_ble_discovery_initialized = true;

		k_spin_unlock(&app_ble_discovery_lock, key);
	}

	return 0;
}

/* Starts TRS primary service and characteristic discovery on conn.
 * Returns -EINVAL (NULL conn), -EALREADY (in progress), -EACCES (not initialized). */
int app_ble_discovery_start(struct bt_conn *conn)
{
	struct bt_conn *old_conn = NULL;
	int err;

	if (conn == NULL) {
		return -EINVAL;
	}

	{
		k_spinlock_key_t key = k_spin_lock(&app_ble_discovery_lock);

		if (!app_ble_discovery_initialized) {
			k_spin_unlock(&app_ble_discovery_lock, key);
			return -EACCES;
		}

		if (app_ble_discovery_in_progress) {
			k_spin_unlock(&app_ble_discovery_lock, key);
			return -EALREADY;
		}

		old_conn = app_ble_discovery_conn;
		app_ble_discovery_conn = bt_conn_ref(conn);
		app_ble_discovery_in_progress = true;
		app_ble_discovery_completed = false;
		app_ble_discovery_status = -EINPROGRESS;
		app_ble_discovery_reset_result();

		(void)memset(&app_ble_discover_params, 0, sizeof(app_ble_discover_params));
		app_ble_discover_params.uuid = &app_ble_discovery_uuid_trs.uuid;
		app_ble_discover_params.func = app_ble_discover_func;
		app_ble_discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
		app_ble_discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
		app_ble_discover_params.type = BT_GATT_DISCOVER_PRIMARY;

		k_spin_unlock(&app_ble_discovery_lock, key);
	}

	/* Unref the stale connection outside the spinlock to avoid calling
	 * bt_conn_unref() while holding the lock. */
	if (old_conn != NULL) {
		bt_conn_unref(old_conn);
	}

	err = bt_gatt_discover(conn, &app_ble_discover_params);
	if (err != 0) {
		LOG_ERR("TRS service discovery failed to start (%d)", err);
		app_ble_discovery_complete(conn, err);
		return err;
	}

	return 0;
}

/* Copies discovered handles to result.
 * Returns -EINPROGRESS (still running), -ENOTCONN (no conn), -ENOENT (conn mismatch). */
int app_ble_discovery_result_get(struct bt_conn *conn,
				 struct app_ble_discovery_result *result)
{
	int status;

	if (result == NULL) {
		return -EINVAL;
	}

	{
		k_spinlock_key_t key = k_spin_lock(&app_ble_discovery_lock);

		if (app_ble_discovery_conn == NULL) {
			k_spin_unlock(&app_ble_discovery_lock, key);
			return -ENOTCONN;
		}

		if ((conn != NULL) && (conn != app_ble_discovery_conn)) {
			k_spin_unlock(&app_ble_discovery_lock, key);
			return -ENOENT;
		}

		if (!app_ble_discovery_completed) {
			k_spin_unlock(&app_ble_discovery_lock, key);
			return -EINPROGRESS;
		}

		*result = app_ble_discovery_result;
		status = app_ble_discovery_status;

		k_spin_unlock(&app_ble_discovery_lock, key);
	}

	return status;
}

/* Cleans up discovery state for conn; no-op if conn is not tracked. */
void app_ble_discovery_on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	struct bt_conn *to_unref = NULL;

	ARG_UNUSED(reason);

	if (conn == NULL) {
		return;
	}

	{
		k_spinlock_key_t key = k_spin_lock(&app_ble_discovery_lock);

		if (app_ble_discovery_conn == conn) {
			to_unref = app_ble_discovery_conn;
			app_ble_discovery_conn = NULL;
			app_ble_discovery_in_progress = false;
			app_ble_discovery_completed = false;
			app_ble_discovery_status = -ENOTCONN;
			app_ble_discovery_reset_result();
			(void)memset(&app_ble_discover_params, 0, sizeof(app_ble_discover_params));
		}

		k_spin_unlock(&app_ble_discovery_lock, key);
	}

	if (to_unref != NULL) {
		bt_conn_unref(to_unref);
	}
}
