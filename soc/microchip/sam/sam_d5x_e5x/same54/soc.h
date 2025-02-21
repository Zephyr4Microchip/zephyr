/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MICROCHIP_SAME54_SOC_H_
#define MICROCHIP_SAME54_SOC_H_

#ifndef _ASMLANGUAGE

#define DONT_USE_CMSIS_INIT

#include <zephyr/types.h>
#include <mchp_dt_helper.h>

#if defined(CONFIG_SOC_ATSAME54N19A)
#include <same54n19a.h>
#elif defined(CONFIG_SOC_ATSAME54N20A)
#include <same54n20a.h>
#elif defined(CONFIG_SOC_ATSAME54P19A)
#include <same54p19a.h>
#elif defined(CONFIG_SOC_ATSAME54P20A)
#include <same54p20a.h>
#else
#error Library does not support the specified device.
#endif

#endif /* _ASMLANGUAGE */

#endif /* MICROCHIP_SAME54_SOC_H_ */
