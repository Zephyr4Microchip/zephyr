/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APP_BLE_DISCOVERY_H_
#define APP_BLE_DISCOVERY_H_

#include <stdint.h>

#include <zephyr/bluetooth/conn.h>

struct app_ble_discovery_result {
    uint16_t svc_start_handle;
    uint16_t svc_end_handle;
    uint16_t tx_value_handle;
    uint16_t rx_value_handle;
    uint16_t cp_value_handle;
};

typedef void (*app_ble_discovery_cb_t)(struct bt_conn *conn,
                                        const struct app_ble_discovery_result *result,
                                        int status,
                                        void *context);

int app_ble_discovery_init(app_ble_discovery_cb_t cb, void *cb_ctx);

int app_ble_discovery_start(struct bt_conn *conn);

int app_ble_discovery_result_get(struct bt_conn *conn,
                                 struct app_ble_discovery_result *result);

void app_ble_discovery_on_disconnected(struct bt_conn *conn, uint8_t reason);

#endif /* APP_BLE_DISCOVERY_H_ */
