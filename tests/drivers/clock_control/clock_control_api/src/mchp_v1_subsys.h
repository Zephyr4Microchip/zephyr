/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "device_subsys.h"
#include <zephyr/drivers/clock_control/mchp_clock_sam_d5x_e5x.h>
#include <zephyr/dt-bindings/clock/mchp_sam_d5x_e5x_clock.h>

static const struct device_subsys_data subsys_data[] = {
	{
		.subsys = (void *)CLOCK_MCHP_MCLKPERIPH_ID_APBB_SERCOM3,
		.startup_us = IS_ENABLED(SOC_FAMILY_MICROCHIP_SAM_D5X_E5X) ? 3000 : 500,
	},
	{
		.subsys = (void *)CLOCK_MCHP_GCLKPERIPH_ID_SERCOM4_CORE,
		.startup_us = IS_ENABLED(SOC_FAMILY_MICROCHIP_SAM_D5X_E5X) ? 3000 : 500,
	},
	{
		.subsys = (void *)CLOCK_MCHP_XOSC_ID_XOSC1,
		.startup_us = IS_ENABLED(SOC_FAMILY_MICROCHIP_SAM_D5X_E5X) ? 3000 : 500,
	},
	{
		.subsys = (void *)CLOCK_MCHP_XOSC32K_ID_XOSC32K,
		.startup_us = IS_ENABLED(SOC_FAMILY_MICROCHIP_SAM_D5X_E5X) ? 3000 : 500,
	}

};

static const struct device_data devices[] = {{.dev = DEVICE_DT_GET(DT_NODELABEL(clock)),
					      .subsys_data = subsys_data,
					      .subsys_cnt = ARRAY_SIZE(subsys_data)}};
