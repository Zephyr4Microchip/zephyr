/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MICROCHIP_BZ3_SOC_H_
#define MICROCHIP_BZ3_SOC_H_

#include <zephyr/types.h>
#include <mchp_dt_helper.h>

#if defined(CONFIG_SOC_PIC32CX5109BZ31048)
#include <pic32cx5109bz31048.h>
#else
#error Library does not support the specified device.
#endif

#include "pic32cx_bz.h"

#endif /* MICROCHIP_BZ3_SOC_H_ */
