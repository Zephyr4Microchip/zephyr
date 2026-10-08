/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>

#include "app.h"

LOG_MODULE_REGISTER(main);

int main(void)
{
    int err;

    err = app_init();
    if (err != 0) {
        LOG_ERR("Application init failed (err %d)", err);
    }

    return 0;
}
