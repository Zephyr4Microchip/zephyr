/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_sam0.h>

static int clock_control_sam0_gclk_on(const struct device *dev, clock_control_subsys_t sys)
{
	clock_control_sam0_sub_sys gclk_type = (clock_control_sam0_sub_sys)sys;
	if (gclk_type <= CLOCK_CONTROL_SAM0_SUB_SYS_PERIPH_BEGIN ||
	    gclk_type >= CLOCK_CONTROL_SAM0_SUB_SYS_PERIPH_END) {
		return -1; // -error no (No clock selected)
	}
	GCLK->PCHCTRL[gclk_type].bit.CHEN = 1; // Enable channel
	printf("GCLK: clock_control_sam0_gclk_off %08x\n", (uint32_t)GCLK->PCHCTRL[gclk_type].reg);
	return 0;
}

static int clock_control_sam0_gclk_off(const struct device *dev, clock_control_subsys_t sys)
{
	clock_control_sam0_sub_sys gclk_type = (clock_control_sam0_sub_sys)sys;
	if (gclk_type <= CLOCK_CONTROL_SAM0_SUB_SYS_PERIPH_BEGIN ||
	    gclk_type >= CLOCK_CONTROL_SAM0_SUB_SYS_PERIPH_END) {
		return -1; // -error no (No clock selected)
	}
	GCLK->PCHCTRL[gclk_type].bit.CHEN = 0; // Disable channel
	// TODO Wait for SYNC or not?
	printf("GCLK: clock_control_sam0_gclk_off\n");
	return 0;
}

static int clock_control_sam0_gclk_async_on(const struct device *dev, clock_control_subsys_t sys,
					    clock_control_cb_t cb, void *user_data)
{
	// TODO
	printf("GCLK: clock_control_sam0_gclk_async_on\n");
	return 0;
}

static int clock_control_sam0_gclk_get_rate(const struct device *dev, clock_control_subsys_t sys,
					    uint32_t *rate)
{
	// TODO
	clock_control_sam0_sub_sys gclk_type = (clock_control_sam0_sub_sys)sys;
	if (gclk_type <= CLOCK_CONTROL_SAM0_SUB_SYS_PERIPH_BEGIN ||
	    gclk_type >= CLOCK_CONTROL_SAM0_SUB_SYS_PERIPH_END) {
		return -1; // -error no (No clock selected)
	}
	printf("GCLK: clock_control_sam0_gclk_get_rate\n");
	return 0;
}

static enum clock_control_status clock_control_sam0_gclk_get_status(const struct device *dev,
								    clock_control_subsys_t sys)
{
	// TODO
	clock_control_sam0_sub_sys gclk_type = (clock_control_sam0_sub_sys)sys;
	if (gclk_type <= CLOCK_CONTROL_SAM0_SUB_SYS_PERIPH_BEGIN ||
	    gclk_type >= CLOCK_CONTROL_SAM0_SUB_SYS_PERIPH_END) {
		return -1; // -error no (No clock selected)
	}
	printf("GCLK: clock_control_sam0_gclk_get_status: %d\n", gclk_type);
	return CLOCK_CONTROL_STATUS_UNKNOWN;
}

static int clock_control_sam0_gclk_set_rate(const struct device *dev, clock_control_subsys_t sys,
					    clock_control_subsys_rate_t rate)
{
	// TODO
	clock_control_sam0_sub_sys gclk_type = (clock_control_sam0_sub_sys)sys;
	if (gclk_type <= CLOCK_CONTROL_SAM0_SUB_SYS_PERIPH_BEGIN ||
	    gclk_type >= CLOCK_CONTROL_SAM0_SUB_SYS_PERIPH_END) {
		return -1; // -error no (No clock selected)
	}
	printf("GCLK: clock_control_sam0_gclk_set_rate: %d\n", gclk_type);
	return 0;
}

static int clock_control_sam0_gclk_configure(const struct device *dev, clock_control_subsys_t sys,
					     void *data)
{
	// TODO
	struct clock_control_sam0_gclk_data *gclk_data =
		(struct clock_control_sam0_gclk_data *)data;
	clock_control_sam0_sub_sys gclk_type = (clock_control_sam0_sub_sys)sys;
	if (gclk_type <= CLOCK_CONTROL_SAM0_SUB_SYS_PERIPH_BEGIN ||
	    gclk_type >= CLOCK_CONTROL_SAM0_SUB_SYS_PERIPH_END) {
		return -1; // -error no (No clock selected)
	}
	printf("	Clock Type: GCLK%d\n", gclk_type);
	//printf("	Clock input_source: %u\n", gclk_data->input_source);
	//printf("	Clock Divider: %u\n", gclk_data->divider);
	return 0;
}

static const struct clock_control_driver_api clock_control_sam0_driver_api = {
	.on = clock_control_sam0_on,
	.off = clock_control_sam0_off,
	.async_on = clock_control_sam0_async_on,
	.get_rate = clock_control_sam0_get_rate,
	.get_status = clock_control_sam0_get_status,
	.set_rate = clock_control_sam0_set_rate,
	.configure = clock_control_sam0_configure,
};

static int clock_control_sam0_gclk_init(const struct device *dev)
{
	// TODO
	// This mostly will deal with Kconfig options

	return 0;
}

DEVICE_DT_DEFINE(DT_NODELABEL(mclk), clock_control_sam0_gclk_init, NULL, NULL, NULL, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_sam0_driver_gclk_api);
