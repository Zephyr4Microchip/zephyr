/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/settings/settings.h>

#include "app_ble.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_ble);

#define APP_BLE_DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define APP_BLE_DEVICE_NAME_LEN (sizeof(APP_BLE_DEVICE_NAME) - 1)

/* 50 ms gives the BT stack time to fully release connection resources
 * before bt_le_adv_start() is called again; avoids -ENOMEM from the runner. */
#define APP_BLE_ADV_RESTART_DELAY_MS  50U

static const struct bt_data app_ble_ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_SVC_DATA16, 0xDA, 0xFE, 0xFF, 0x01),
	BT_DATA(BT_DATA_NAME_COMPLETE, APP_BLE_DEVICE_NAME, APP_BLE_DEVICE_NAME_LEN),
};

/* Guards against calling bt_enable() more than once; the BT stack does not
 * support re-initialization. */
static bool app_ble_init_requested;
static bool app_ble_ready;

static struct k_work_delayable app_ble_adv_restart_dwork;

static int app_ble_adv_start(void)
{
	int err;

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, app_ble_ad, ARRAY_SIZE(app_ble_ad), NULL, 0);
	if (err == -EALREADY) {
		return 0;
	}
	if (err != 0) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return err;
	}

	LOG_INF("Configuration mode: waiting connections...");
	return 0;
}

/* [work-queue context] bt_le_adv_start() succeeds here after the stack releases the connection slot. */
static void app_ble_adv_restart_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	(void)app_ble_adv_start();
}

void app_ble_adv_restart(void)
{
	(void)k_work_reschedule(&app_ble_adv_restart_dwork,
				K_MSEC(APP_BLE_ADV_RESTART_DELAY_MS));
}

/* [BT RX thread] bt_le_adv_start() is thread-safe here. */
static void app_ble_ready_cb(int err)
{
	int rc;

	if (err != 0) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}

	app_ble_ready = true;
	LOG_INF("Bluetooth initialized");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		rc = settings_load();
		if (rc != 0) {
			LOG_WRN("settings_load failed (err %d)", rc);
		}
	}

	(void)app_ble_adv_start();
}

int app_ble_init(void)
{
	int err;

	if (app_ble_init_requested) {
		return 0;
	}

	k_work_init_delayable(&app_ble_adv_restart_dwork, app_ble_adv_restart_work_handler);

	app_ble_init_requested = true;

	err = bt_enable(app_ble_ready_cb);
	if (err != 0) {
		app_ble_init_requested = false;
		LOG_ERR("Bluetooth enable failed (err %d)", err);
		return err;
	}

	return 0;
}

bool app_ble_is_ready(void)
{
	return app_ble_ready;
}
