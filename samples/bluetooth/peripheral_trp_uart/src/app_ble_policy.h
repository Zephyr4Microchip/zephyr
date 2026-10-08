/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APP_BLE_POLICY_H_
#define APP_BLE_POLICY_H_

#include <stdint.h>

#include <zephyr/bluetooth/conn.h>

int app_ble_policy_init(void);

void app_ble_policy_on_connected(struct bt_conn *conn);

void app_ble_policy_on_disconnected(struct bt_conn *conn, uint8_t reason);

#endif /* APP_BLE_POLICY_H_ */
