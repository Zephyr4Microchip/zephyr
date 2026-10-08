/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APP_BLE_H_
#define APP_BLE_H_

#include <stdbool.h>

int app_ble_init(void);

bool app_ble_is_ready(void);

/* Schedules a deferred advertising restart from the system work queue.
 * Must be used instead of calling bt_le_adv_start() directly from a BT
 * disconnect callback to avoid -ENOMEM while the stack is still releasing
 * connection resources. */
void app_ble_adv_restart(void);

#endif /* APP_BLE_H_ */
