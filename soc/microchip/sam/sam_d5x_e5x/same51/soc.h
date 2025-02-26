/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MICROCHIP_SAME51_SOC_H_
#define MICROCHIP_SAME51_SOC_H_

#ifndef _ASMLANGUAGE

#define DONT_USE_CMSIS_INIT

#include <zephyr/types.h>
#include <mchp_dt_helper.h>

#if defined(CONFIG_SOC_ATSAME51J18A)
#include <same51j18a.h>
#elif defined(CONFIG_SOC_ATSAME51J19A)
#include <same51j19a.h>
#elif defined(CONFIG_SOC_ATSAME51J20A)
#include <same51j20a.h>
#elif defined(CONFIG_SOC_ATSAME51N19A)
#include <same51n19a.h>
#elif defined(CONFIG_SOC_ATSAME51N20A)
#include <same51n20a.h>
#else
#error Library does not support the specified device.
#endif

#endif /* _ASMLANGUAGE */

#endif /* MICROCHIP_SAME51_SOC_H_ */
