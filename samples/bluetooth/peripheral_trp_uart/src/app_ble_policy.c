/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/sys/util.h>

#include "app_ble_policy.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_ble_policy);

/* BT_SECURITY_L2: unauthenticated pairing with encryption (Just Works). */
#define APP_BLE_POLICY_SEC_LEVEL            BT_SECURITY_L2
#define APP_BLE_POLICY_CONN_INTERVAL_MIN    0x0006U
#define APP_BLE_POLICY_CONN_INTERVAL_MAX    0x0C80U
#define APP_BLE_POLICY_CONN_LATENCY_MAX     0x01F3U
#define APP_BLE_POLICY_CONN_TIMEOUT_MIN     0x000AU
#define APP_BLE_POLICY_CONN_TIMEOUT_MAX     0x0C80U

static bool app_ble_policy_initialized;

/* Reject conn param updates outside defined bounds. */
static bool app_ble_policy_le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	ARG_UNUSED(conn);

	if (param == NULL) {
		return false;
	}

	if ((param->interval_min < APP_BLE_POLICY_CONN_INTERVAL_MIN) ||
	    (param->interval_max > APP_BLE_POLICY_CONN_INTERVAL_MAX) ||
	    (param->interval_min > param->interval_max) ||
	    (param->latency > APP_BLE_POLICY_CONN_LATENCY_MAX) ||
	    (param->timeout < APP_BLE_POLICY_CONN_TIMEOUT_MIN) ||
	    (param->timeout > APP_BLE_POLICY_CONN_TIMEOUT_MAX)) {
		LOG_WRN("Rejecting conn params (itv %u-%u, lat %u, to %u)",
			param->interval_min, param->interval_max, param->latency,
			param->timeout);
		return false;
	}

	return true;
}

static void app_ble_policy_le_param_updated(struct bt_conn *conn, uint16_t interval,
					    uint16_t latency, uint16_t timeout)
{
	ARG_UNUSED(conn);

	LOG_INF("Conn params updated (itv %u, lat %u, to %u)", interval, latency, timeout);
}

#if defined(CONFIG_BT_SMP) || defined(CONFIG_BT_CLASSIC)
static void app_ble_policy_security_changed(struct bt_conn *conn, bt_security_t level,
					    enum bt_security_err err)
{
	ARG_UNUSED(conn);

	if (err == BT_SECURITY_ERR_SUCCESS) {
		LOG_INF("Security changed: level %u", level);
		return;
	}

	LOG_WRN("Security change failed: level %u, err %u", level, err);
}
#endif

static struct bt_conn_cb app_ble_policy_conn_cbs = {
	.le_param_req = app_ble_policy_le_param_req,
	.le_param_updated = app_ble_policy_le_param_updated,
#if defined(CONFIG_BT_SMP) || defined(CONFIG_BT_CLASSIC)
	.security_changed = app_ble_policy_security_changed,
#endif
};

#if defined(CONFIG_BT_SMP)
static void app_ble_policy_pairing_complete(struct bt_conn *conn, bool bonded)
{
	ARG_UNUSED(conn);

	LOG_INF("Pairing complete (bonded=%u)", bonded ? 1U : 0U);
}

static void app_ble_policy_pairing_failed(struct bt_conn *conn,
					  enum bt_security_err reason)
{
	ARG_UNUSED(conn);

	LOG_WRN("Pairing failed (reason=%u)", reason);
}

static void app_ble_policy_bond_deleted(uint8_t id, const bt_addr_le_t *peer)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (peer == NULL) {
		return;
	}

	bt_addr_le_to_str(peer, addr, sizeof(addr));
	LOG_INF("Bond deleted (id=%u, peer=%s)", id, addr);
}

static struct bt_conn_auth_info_cb app_ble_policy_auth_info_cb = {
	.pairing_complete = app_ble_policy_pairing_complete,
	.pairing_failed = app_ble_policy_pairing_failed,
	.bond_deleted = app_ble_policy_bond_deleted,
};
#endif

int app_ble_policy_init(void)
{
	int rc;

	if (app_ble_policy_initialized) {
		return 0;
	}

	bt_conn_cb_register(&app_ble_policy_conn_cbs);

#if defined(CONFIG_BT_SMP)
	rc = bt_conn_auth_info_cb_register(&app_ble_policy_auth_info_cb);
	if ((rc != 0) && (rc != -EALREADY)) {
		LOG_ERR("Failed to register auth-info callbacks: %d", rc);
		return rc;
	}
#endif

	app_ble_policy_initialized = true;
	return 0;
}

void app_ble_policy_on_connected(struct bt_conn *conn)
{
#if defined(CONFIG_BT_SMP)
	int rc;

	if (conn == NULL) {
		return;
	}

	rc = bt_conn_set_security(conn, APP_BLE_POLICY_SEC_LEVEL);
	if ((rc != 0) && (rc != -EALREADY)) {
		LOG_WRN("Security request failed: %d", rc);
	}
#else
	ARG_UNUSED(conn);
#endif
}

void app_ble_policy_on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(reason);
}
