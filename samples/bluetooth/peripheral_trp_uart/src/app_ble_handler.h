/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APP_BLE_HANDLER_H_
#define APP_BLE_HANDLER_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/bluetooth/conn.h>

typedef void (*app_ble_conn_state_cb_t)(bool connected, uint8_t reason, void *context);

int app_ble_handler_init(app_ble_conn_state_cb_t state_cb, void *state_cb_ctx);

bool app_ble_handler_is_connected(void);

int app_ble_handler_conn_ref_get(struct bt_conn **conn);

#endif /* APP_BLE_HANDLER_H_ */
