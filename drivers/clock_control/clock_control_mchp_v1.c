/*
 * Copyright (c) [2024] [Microchip Technology Inc.]
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_control/clock_control_mchp_v1.h>
#include "clock_control_mchp_v1.h"

/* Get subsys device function */
#define CLOCK_CONTROL_MCHP_SUBSYS_GET(node_id) DEVICE_DT_GET(node_id),

/* Get subsys register address function */
#define CLOCK_CONTROL_MCHP_SUBSYS_GET_REG(node_id) DT_REG_ADDR(node_id),

/* define subsys device function */
#define CLOCK_CONTROL_MCHP_SUBSYS_DEFINE(node_id)                                                  \
	DEVICE_DT_DEFINE(node_id, NULL, NULL, NULL, NULL, PRE_KERNEL_1,                            \
			 CONFIG_CLOCK_CONTROL_INIT_PRIORITY, NULL);

/* Get number of ranges, which is equal to number of subnodes */
#define CLOCK_CONTROL_MCHP_SUBSYS_COUNT DT_NUM_RANGES(DT_NODELABEL(clock))

struct clock_control_mchp_cb_work_data_t {
	struct k_work work;
	clock_control_cb_t cb;
	const struct device *dev;
	clock_control_subsys_t sys;
	void *user_data;
};

struct clock_control_mchp_data_t {
	uint32_t async_base;
	uint32_t async_id;
	struct clock_control_mchp_cb_work_data_t cb_work_data;
};

struct clock_control_mchp_config_t {
	uint32_t subsys_count;
	const struct device *subsys_devs[CLOCK_CONTROL_MCHP_SUBSYS_COUNT];
	const uint32_t subsys_regs[CLOCK_CONTROL_MCHP_SUBSYS_COUNT];
	struct clock_control_mchp_user_frequency_t user_frequency;
};

static void clock_control_mchp_invoke_cb(struct k_work *async_work)
{
	struct clock_control_mchp_cb_work_data_t *cb_work_data =
		CONTAINER_OF(async_work, struct clock_control_mchp_cb_work_data_t, work);
	cb_work_data->cb(cb_work_data->dev, cb_work_data->sys, cb_work_data->user_data);
}

static int clock_control_mchp_get_idx(const struct device *dev, const struct device *clock_dev)
{
	int subsys_idx = -1;
	const struct clock_control_mchp_config_t *config = dev->config;
	uint8_t index;
	for (index = 0; index < CLOCK_CONTROL_MCHP_SUBSYS_COUNT; index++) {
		if (clock_dev == config->subsys_devs[index]) {
			subsys_idx = index;
			break;
		}
	}
	return subsys_idx;
}

static void clock_control_mchp_isr(const struct device *dev)
{
	struct clock_control_mchp_data_t *data = dev->data;

	hal_mchp_clock_clear_interrupt(data->async_base, data->async_id);
	hal_mchp_clock_disable_interrupt(data->async_base, data->async_id);

	if (data->cb_work_data.cb == NULL) {
		/* no callback provided by caller */
	} else {
		k_work_submit(&data->cb_work_data.work);
	}
}

static int clock_control_mchp_async_on(const struct device *dev, clock_control_subsys_t sys,
				       clock_control_cb_t cb, void *user_data)
{
	int ret_val;
	struct clock_control_mchp_subsys_t *subsys;
	enum clock_control_mchp_state_t state;
	enum clock_control_mchp_state_t status;
	int subsys_idx;
	const struct clock_control_mchp_config_t *config = dev->config;
	struct clock_control_mchp_data_t *data = dev->data;

	if (sys == CLOCK_CONTROL_SUBSYS_ALL) {
		/* Clock Async On for all clock is not supported */
		ret_val = -ENOTSUP;
	} else {
		subsys = (struct clock_control_mchp_subsys_t *)sys;
		subsys_idx = clock_control_mchp_get_idx(dev, subsys->dev);
		if (subsys_idx < 0) {
			/* no matching subsys */
			ret_val = -ENOTSUP;
		} else {
			status = hal_mchp_clock_status(config->subsys_regs[subsys_idx], subsys->id);
			if (status == CLOCK_CONTROL_MCHP_STATE_ON) {
				/* Clock is already ON */
				ret_val = -EALREADY;
			} else if (status == CLOCK_CONTROL_MCHP_STATE_STARTING) {
				/* Clock is already STARTING */
				ret_val = -EALREADY;
			} else if (status == CLOCK_CONTROL_MCHP_STATE_OFF) {
				/* Clock is Off. Turn it On */
				state = hal_mchp_clock_on(config->subsys_regs[subsys_idx],
							  subsys->id);
				if (state == CLOCK_CONTROL_MCHP_STATE_ON) {
					if (hal_mchp_clock_enable_interrupt(
						    config->subsys_regs[subsys_idx], subsys->id) ==
					    CLOCK_CONTROL_MCHP_STATE_OK) {
						data->async_base = config->subsys_regs[subsys_idx];
						data->async_id = subsys->id;
						data->cb_work_data.cb = cb;
						data->cb_work_data.sys = sys;
						data->cb_work_data.user_data = user_data;
						k_work_init(&data->cb_work_data.work,
							    clock_control_mchp_invoke_cb);
						ret_val = 0;
					} else {
						/* subsys is not supported */
						ret_val = -ENOTSUP;
					}
				} else {
					/* subsys is not supported */
					ret_val = -ENOTSUP;
				}
			} else {
				/* Any other values */
				ret_val = -ENOTSUP;
			}
		}
	}
	return ret_val;
}

static int clock_control_mchp_configure(const struct device *dev, clock_control_subsys_t sys,
					void *req_configuration)
{
	int ret_val;
	struct clock_control_mchp_subsys_t *subsys;
	enum clock_control_mchp_state_t state;
	uint8_t index;
	int subsys_idx;
	const struct clock_control_mchp_config_t *config = dev->config;

	if (sys == CLOCK_CONTROL_SUBSYS_ALL) {
		state = CLOCK_CONTROL_MCHP_STATE_NO_SUPPORT;
		for (index = 0; index < CLOCK_CONTROL_MCHP_SUBSYS_COUNT; index++) {
			state = hal_mchp_clock_configure(config->subsys_regs[index], 0,
							 req_configuration);
		}
		if (state == CLOCK_CONTROL_MCHP_STATE_OK) {
			ret_val = 0;
		} else {
			ret_val = -ENOTSUP;
		}
	} else {
		subsys = (struct clock_control_mchp_subsys_t *)sys;
		subsys_idx = clock_control_mchp_get_idx(dev, subsys->dev);
		if (subsys_idx < 0) {
			/* no matching subsys */
			ret_val = -ENOTSUP;
		} else {
			state = hal_mchp_clock_configure(config->subsys_regs[subsys_idx],
							 subsys->id, req_configuration);
			if (state == CLOCK_CONTROL_MCHP_STATE_OK) {
				ret_val = 0;
			} else {
				ret_val = -ENOTSUP;
			}
		}
	}
	return ret_val;
}

static int clock_control_mchp_get_rate(const struct device *dev, clock_control_subsys_t sys,
				       uint32_t *rate)
{
	int ret_val;
	struct clock_control_mchp_subsys_t *subsys;
	enum clock_control_mchp_state_t state;
	int subsys_idx;
	const struct clock_control_mchp_config_t *config = dev->config;

	if (sys == CLOCK_CONTROL_SUBSYS_ALL) {
		/* get rate for all clock is not supported */
		ret_val = -ENOTSUP;
	} else {
		subsys = (struct clock_control_mchp_subsys_t *)sys;
		subsys_idx = clock_control_mchp_get_idx(dev, subsys->dev);
		if (subsys_idx < 0) {
			/* no matching subsys */
			ret_val = -ENOTSUP;
		} else {
			state = hal_mchp_clock_get_rate(config->subsys_regs[subsys_idx], subsys->id,
							rate, config->user_frequency);
			if (state == CLOCK_CONTROL_MCHP_STATE_OK) {
				ret_val = 0;
			} else if (state == CLOCK_CONTROL_MCHP_STATE_NO_RATE) {
				ret_val = -EAGAIN;
			} else {
				ret_val = -ENOTSUP;
			}
		}
	}
	return ret_val;
}

static enum clock_control_status clock_control_mchp_get_status(const struct device *dev,
							       clock_control_subsys_t sys)
{
	enum clock_control_status ret_status;
	struct clock_control_mchp_subsys_t *subsys;
	enum clock_control_mchp_state_t status;
	int subsys_idx;
	const struct clock_control_mchp_config_t *config = dev->config;

	if (sys == CLOCK_CONTROL_SUBSYS_ALL) {
		/* Clock status for all clock is not supported */
		ret_status = CLOCK_CONTROL_STATUS_UNKNOWN;
	} else {
		subsys = (struct clock_control_mchp_subsys_t *)sys;
		subsys_idx = clock_control_mchp_get_idx(dev, subsys->dev);
		if (subsys_idx < 0) {
			/* no matching subsys */
			ret_status = CLOCK_CONTROL_STATUS_UNKNOWN;
		} else {
			status = hal_mchp_clock_status(config->subsys_regs[subsys_idx], subsys->id);
			if (status == CLOCK_CONTROL_MCHP_STATE_OFF) {
				ret_status = CLOCK_CONTROL_STATUS_OFF;
			} else if (status == CLOCK_CONTROL_MCHP_STATE_ON) {
				ret_status = CLOCK_CONTROL_STATUS_ON;
			} else if (status == CLOCK_CONTROL_MCHP_STATE_STARTING) {
				ret_status = CLOCK_CONTROL_STATUS_STARTING;
			} else {
				ret_status = CLOCK_CONTROL_STATUS_UNKNOWN;
			}
		}
	}
	return ret_status;
}

static int clock_control_mchp_off(const struct device *dev, clock_control_subsys_t sys)
{
	int ret_val;
	struct clock_control_mchp_subsys_t *subsys;
	enum clock_control_mchp_state_t state;
	enum clock_control_mchp_state_t status;
	int subsys_idx;
	const struct clock_control_mchp_config_t *config = dev->config;

	if (sys == CLOCK_CONTROL_SUBSYS_ALL) {
		/* Clock Off for all clock is not supported */
		ret_val = -ENOTSUP;
	} else {
		subsys = (struct clock_control_mchp_subsys_t *)sys;
		subsys_idx = clock_control_mchp_get_idx(dev, subsys->dev);
		if (subsys_idx < 0) {
			/* no matching subsys */
			ret_val = -ENOTSUP;
		} else {
			status = hal_mchp_clock_status(config->subsys_regs[subsys_idx], subsys->id);
			if (status == CLOCK_CONTROL_MCHP_STATE_OFF) {
				/* Clock is already OFF */
				ret_val = -EALREADY;
			} else {
				/* Clock is On/Starting. Turn it Off. if sub_sys not valid
				 * or does not support OFF operation error will be returned
				 */
				state = hal_mchp_clock_off(config->subsys_regs[subsys_idx],
							   subsys->id);
				if (state == CLOCK_CONTROL_MCHP_STATE_OFF) {
					ret_val = 0;
				} else {
					ret_val = -ENOTSUP;
				}
			}
		}
	}
	return ret_val;
}

static int clock_control_mchp_on(const struct device *dev, clock_control_subsys_t sys)
{
	int ret_val;
	struct clock_control_mchp_subsys_t *subsys;
	enum clock_control_mchp_state_t state;
	enum clock_control_mchp_state_t status;
	enum clock_control_mchp_state_t on_status;
	int subsys_idx;
	const struct clock_control_mchp_config_t *config = dev->config;

	if (sys == CLOCK_CONTROL_SUBSYS_ALL) {
		/* Clock On for all clock is not supported */
		ret_val = -ENOTSUP;
	} else {
		subsys = (struct clock_control_mchp_subsys_t *)sys;
		subsys_idx = clock_control_mchp_get_idx(dev, subsys->dev);
		if (subsys_idx < 0) {
			/* no matching subsys */
			ret_val = -ENOTSUP;
		} else {
			status = hal_mchp_clock_status(config->subsys_regs[subsys_idx], subsys->id);
			if (status == CLOCK_CONTROL_MCHP_STATE_ON) {
				/* Clock is already ON */
				ret_val = -EALREADY;
			} else if (status == CLOCK_CONTROL_MCHP_STATE_STARTING) {
				/* Clock is already STARTING */
				ret_val = -EALREADY;
			} else if (status == CLOCK_CONTROL_MCHP_STATE_OFF) {
				/* Clock is Off. Turn it On */
				state = hal_mchp_clock_on(config->subsys_regs[subsys_idx],
							  subsys->id);
				if (state == CLOCK_CONTROL_MCHP_STATE_ON) {
					while (true) {
						on_status = hal_mchp_clock_status(
							config->subsys_regs[subsys_idx],
							subsys->id);
						if (on_status == CLOCK_CONTROL_MCHP_STATE_ON) {
							ret_val = 0;
							break;
						} else {
							k_sleep(K_MSEC(1));
						}
					}
				} else {
					/* subsys is not supported */
					ret_val = -ENOTSUP;
				}
			} else {
				/* Any other values */
				ret_val = -ENOTSUP;
			}
		}
	}
	return ret_val;
}

static int clock_control_mchp_set_rate(const struct device *dev, clock_control_subsys_t sys,
				       clock_control_subsys_rate_t rate)
{
	int ret_val;
	struct clock_control_mchp_subsys_t *subsys;
	enum clock_control_mchp_state_t state;
	int subsys_idx;
	const struct clock_control_mchp_config_t *config = dev->config;

	if (sys == CLOCK_CONTROL_SUBSYS_ALL) {
		/* set rate for all clock is not supported */
		ret_val = -ENOTSUP;
	} else {
		subsys = (struct clock_control_mchp_subsys_t *)sys;
		subsys_idx = clock_control_mchp_get_idx(dev, subsys->dev);
		if (subsys_idx < 0) {
			/* no matching subsys */
			ret_val = -ENOTSUP;
		} else {
			state = hal_mchp_clock_set_rate(config->subsys_regs[subsys_idx], subsys->id,
							rate);
			if (state == CLOCK_CONTROL_MCHP_STATE_OK) {
				ret_val = 0;
			} else if (state == CLOCK_CONTROL_MCHP_STATE_SAME_RATE) {
				ret_val = -EALREADY;
			} else {
				ret_val = -ENOTSUP;
			}
		}
	}
	return ret_val;
}

static DEVICE_API(clock_control, clock_control_mchp_driver_api) = {
	.on = clock_control_mchp_on,
	.off = clock_control_mchp_off,
	.async_on = clock_control_mchp_async_on,
	.get_rate = clock_control_mchp_get_rate,
	.get_status = clock_control_mchp_get_status,
	.set_rate = clock_control_mchp_set_rate,
	.configure = clock_control_mchp_configure,
};

static int clock_control_mchp_init(const struct device *dev)
{
	CLOCK_CONTROL_MCHP_IRQ_CONNECT_ENABLE_DEFN;
	return 0;
}

/* Device data configuration parameters */
static struct clock_control_mchp_data_t clock_control_mchp_data = {
	.async_base = 0,
	.async_id = 0,
};

/* Device constant configuration parameters */
static const struct clock_control_mchp_config_t clock_control_mchp_config = {
	.subsys_count = CLOCK_CONTROL_MCHP_SUBSYS_COUNT,
	.subsys_devs = {DT_FOREACH_CHILD(DT_NODELABEL(clock), CLOCK_CONTROL_MCHP_SUBSYS_GET)},
	.subsys_regs = {DT_FOREACH_CHILD(DT_NODELABEL(clock), CLOCK_CONTROL_MCHP_SUBSYS_GET_REG)},
	.user_frequency = {
		.dfll = DT_PROP_OR(DT_NODELABEL(oscctrl), dfll_frequency, 48000000),
		.xosc = DT_PROP_OR(DT_NODELABEL(oscctrl), xosc_frequency, 0),
		.xosc32k = DT_PROP_OR(DT_NODELABEL(osc32kctrl), xosc32k_frequency, 32768),
		.gclk_in = DT_PROP_OR(DT_NODELABEL(gclk), in_frequency, (0, 0, 0, 0, 0, 0, 0, 0)),
	}};

DEVICE_DT_DEFINE(DT_NODELABEL(clock), clock_control_mchp_init, NULL, &clock_control_mchp_data,
		 &clock_control_mchp_config, PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		 &clock_control_mchp_driver_api);

DT_FOREACH_CHILD(DT_NODELABEL(clock), CLOCK_CONTROL_MCHP_SUBSYS_DEFINE)
