/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_atmel_sam0.h>

#define DT_DRV_COMPAT atmel_samd5x_mclk

#ifdef CONFIG_CLOCK_CONTROL_SAME54_MCLK_CPU_CPU_CLOCK_DIV_1
	#define CLOCK_CONTROL_SAME54_MCLK_CPU_CLOCK_DIV 1
#elif CONFIG_CLOCK_CONTROL_SAME54_MCLK_CPU_CLOCK_DIV_2
	#define CLOCK_CONTROL_SAME54_MCLK_CPU_CLOCK_DIV 2
#elif CONFIG_CLOCK_CONTROL_SAME54_MCLK_CPU_CLOCK_DIV_4
	#define CLOCK_CONTROL_SAME54_MCLK_CPU_CLOCK_DIV 4
#elif CONFIG_CLOCK_CONTROL_SAME54_MCLK_CPU_CLOCK_DIV_8
	#define CLOCK_CONTROL_SAME54_MCLK_CPU_CLOCK_DIV 8
#elif CONFIG_CLOCK_CONTROL_SAME54_MCLK_CPU_CLOCK_DIV_16
	#define CLOCK_CONTROL_SAME54_MCLK_CPU_CLOCK_DIV 16
#elif CONFIG_CLOCK_CONTROL_SAME54_MCLK_CPU_CLOCK_DIV_32
	#define CLOCK_CONTROL_SAME54_MCLK_CPU_CLOCK_DIV 32
#elif CONFIG_CLOCK_CONTROL_SAME54_MCLK_CPU_CLOCK_DIV_64
	#define CLOCK_CONTROL_SAME54_MCLK_CPU_CLOCK_DIV 64
#elif CONFIG_CLOCK_CONTROL_SAME54_MCLK_CPU_CLOCK_DIV_128
	#define CLOCK_CONTROL_SAME54_MCLK_CPU_CLOCK_DIV 128
#else
	// TODO Nothing Selected (Default)
	#define CLOCK_CONTROL_SAME54_MCLK_CPU_CLOCK_DIV 1
#endif

#ifdef CONFIG_CLOCK_CONTROL_SAME54_MCLK_HS_CLOCK_DIV_1
	#define CLOCK_CONTROL_SAME54_MCLK_HS_CLOCK_DIV 1
#else
	#define CLOCK_CONTROL_SAME54_MCLK_HS_CLOCK_DIV 1
#endif

static int _clock_control_xosc_init(void)
{
	// TODO Check whether this exeeds maximum CPU frequency
	return 0;
}

static int _clock_control_xosc32_init(void)
{
	// TODO Check whether this exeeds maximum CPU frequency
	return 0;
}

static int _clock_control_cpu_div(void)
{
	// TODO Check whether this exeeds maximum CPU frequency
	MCLK->CPUDIV.reg = (uint32_t)CLOCK_CONTROL_SAME54_MCLK_CPU_CLOCK_DIV;
	return 0;
}

static int _clock_control_hs_div(void)
{
	// TODO Check whether this exeeds maximum HS frequency
	//MCLK->HSDIV.reg = (uint32_t)CLOCK_CONTROL_SAME54_MCLK_HS_CLOCK_DIV;
	return 0;
}

static int clock_control_sam0_mclk_on(const struct device *dev, clock_control_subsys_t sys)
{
	// TODO
	//MCLK->AHBMASK.reg = (uint32_t)CLOCK_CONTROL_SAME54_MAIN_CLOCK_DIV;
	//MCLK->APBMASK.reg = (uint32_t)CLOCK_CONTROL_SAME54_MAIN_CLOCK_DIV;
	printf("MCLK: clock_control_sam0_mclk_on\n");
	return 0;
}

static int clock_control_sam0_mclk_off(const struct device *dev, clock_control_subsys_t sys)
{
	// TODO
	//MCLK->AHBMASK.reg = (uint32_t)CLOCK_CONTROL_SAME54_MAIN_CLOCK_DIV;
	//MCLK->APBMASK.reg = (uint32_t)CLOCK_CONTROL_SAME54_MAIN_CLOCK_DIV;
	printf("MCLK: clock_control_sam0_mclk_off\n");
	return 0;
}

static int clock_control_sam0_mclk_async_on(const struct device *dev, clock_control_subsys_t sys,
					    clock_control_cb_t cb, void *user_data)
{
	// TODO
	printf("MCLK: clock_control_sam0_mclk_async_on\n");
	return 0;
}

static int clock_control_sam0_mclk_get_rate(const struct device *dev, clock_control_subsys_t sys,
					    uint32_t *rate)
{
	// TODO
	*rate = 10000000;
	printf("MCLK: clock_control_sam0_mclk_get_rate\n");
	return 0;
}

static enum clock_control_status clock_control_sam0_mclk_get_status(const struct device *dev,
								    clock_control_subsys_t sys)
{
	// TODO
	printf("MCLK: clock_control_sam0_mclk_get_status\n");
	return CLOCK_CONTROL_STATUS_UNKNOWN;
}

static int clock_control_sam0_mclk_set_rate(const struct device *dev, clock_control_subsys_t sys,
					    clock_control_subsys_rate_t rate)
{
	// TODO
	printf("MCLK: clock_control_sam0_mclk_set_rate\n");
	return 0;
}

static int clock_control_sam0_mclk_configure(const struct device *dev, clock_control_subsys_t sys,
					     void *data)
{
	// TODO
	printf("MCLK: clock_control_sam0_mclk_configure\n");
	return 0;
}

static const struct clock_control_driver_api clock_control_sam0_driver_mclk_api = {
	.on = clock_control_sam0_mclk_on,
	.off = clock_control_sam0_mclk_off,
	.async_on = clock_control_sam0_mclk_async_on,
	.get_rate = clock_control_sam0_mclk_get_rate,
	.get_status = clock_control_sam0_mclk_get_status,
	.set_rate = clock_control_sam0_mclk_set_rate,
	.configure = clock_control_sam0_mclk_configure,
};

static int clock_control_sam0_mclk_init(const struct device *dev)
{
	_clock_control_xosc_init();
	_clock_control_xosc32_init();
	_clock_control_cpu_div();
	_clock_control_hs_div();

	// TODO
	return 0;
}

DEVICE_DT_INST_DEFINE(0,
	clock_control_sam0_mclk_init,
	NULL, NULL, NULL,
	PRE_KERNEL_1,
	CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
	&clock_control_sam0_driver_mclk_api);
