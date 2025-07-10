/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SOC_MICROCHIP_SAM_D5X_E5X_SAME53_SOC_H_
#define SOC_MICROCHIP_SAM_D5X_E5X_SAME53_SOC_H_

#ifndef _ASMLANGUAGE

#define DONT_USE_CMSIS_INIT

#include <zephyr/types.h>

#if defined(CONFIG_SOC_ATSAME53J18A)
#include <same53j18a.h>
#elif defined(CONFIG_SOC_ATSAME53J19A)
#include <same53j19a.h>
#elif defined(CONFIG_SOC_ATSAME53J20A)
#include <same53j20a.h>
#elif defined(CONFIG_SOC_ATSAME53N19A)
#include <same53n19a.h>
#elif defined(CONFIG_SOC_ATSAME53N20A)
#include <same53n20a.h>
#else
#error Library does not support the specified device.
#endif

#endif /* _ASMLANGUAGE */

#endif /* SOC_MICROCHIP_SAM_D5X_E5X_SAME53_SOC_H_ */
