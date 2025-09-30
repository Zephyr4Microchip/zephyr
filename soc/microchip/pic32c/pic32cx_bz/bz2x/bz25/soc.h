/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MICROCHIP_BZ25_SOC_H_
#define MICROCHIP_BZ25_SOC_H_

#include <zephyr/types.h>
#include <mchp_dt_helper.h>

#if defined(CONFIG_SOC_PIC32CX1012BZ25048)
#include <pic32cx1012bz25048.h>
#else
#error Library does not support the specified device.
#endif

#endif /* MICROCHIP_BZ25_SOC_H_ */
