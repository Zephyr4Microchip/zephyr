/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APP_TRSPS_HANDLER_H_
#define APP_TRSPS_HANDLER_H_

#include <zephyr/drivers/uart.h>

/** Callback invoked when the UART→BLE channel becomes ready to accept data.
 *  Fired when the BLE Central enables TX notifications (CCCD) or grants CBFC credits. */
typedef void (*app_trsps_uart_to_ble_ready_cb_t)(void *context);

int app_trsps_handler_init(const struct device *uart_dev,
                           app_trsps_uart_to_ble_ready_cb_t uart_to_ble_ready_cb,
                           void *uart_to_ble_ready_cb_ctx);

/** Services the BLE→UART UART TX path from within the UART TX-ready ISR. */
void app_trsps_handler_ble_to_uart_service(void);

#endif /* APP_TRSPS_HANDLER_H_ */
