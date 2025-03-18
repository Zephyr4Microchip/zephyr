/**
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>

#include <zephyr/drivers/watchdog.h>
#define LOG_LEVEL CONFIG_WDT_LOG_LEVEL
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <stdio.h>
#include "wdt_mchp_v1.h"


LOG_MODULE_REGISTER(wdt_mchp_v1);

/**
 * @enum wdt_mchp_enable_disable_t
 * @brief Enumeration for enabling or disabling the WDT.
 */
typedef enum {
	WDT_DISABLE = 0, /**< Disable the WDT */
	WDT_ENABLE = 1,  /**< Enable the WDT */
} wdt_mchp_enable_disable_t;

/**
 * @struct wdt_mchp_channel_data
 * @brief Structure to hold WDT channel data.
 */
typedef struct wdt_mchp_channel_data {
	struct wdt_window window; /**< WDT window configuration */
} wdt_mchp_channel_data_t;

/**
 * @struct wdt_mchp_dev_data
 * @brief Structure to hold WDT device data.
 */
typedef struct wdt_mchp_dev_data {
	wdt_callback_t cb;             /**< Callback function for WDT */
	bool interrupt_enabled;        /**< Flag to indicate if interrupt is enabled */
	bool window_mode;              /**< Flag to indicate if window mode is enabled */
	uint8_t installed_timeout_cnt; /**< Number of installed timeouts */
	wdt_mchp_channel_data_t channel_data[MAX_INSTALLABLE_TIMEOUT_COUNT]; /**< Channel data */
} wdt_mchp_dev_data_t;

/**
 * @struct wdt_mchp_dev_cfg
 * @brief Structure to hold WDT device configuration.
 */
typedef struct wdt_mchp_dev_cfg {
	hal_mchp_wdt_t hal;
	void (*irq_config_func)(const struct device *dev); /**< Function to configure IRQ */
	mchp_wdt_clock_t wdt_clock;
} wdt_mchp_dev_cfg_t;

/**
 * @brief WDT interrupt service routine.
 *
 * @param dev Pointer to the device structure.
 */
static void wdt_mchp_isr(const struct device *dev)
{
	wdt_mchp_dev_data_t *mchp_wdt_data = dev->data;
	const wdt_mchp_dev_cfg_t *const mchp_wdt_cfg = dev->config;
	const hal_mchp_wdt_t *const hal = &mchp_wdt_cfg->hal;

	hal_mchp_wdt_interrupt_flag_clear(hal);
	if (mchp_wdt_data->cb != NULL) {
		mchp_wdt_data->cb(dev, 0);
	}
}

/**
 * @brief Setup the WDT.
 *
 * @param dev Pointer to the device structure.
 * @param options WDT options.
 * @return 0 on success, negative error code on failure.
 */
static int wdt_mchp_setup(const struct device *dev, uint8_t options)
{
	wdt_mchp_dev_data_t *mchp_wdt_data = dev->data;
	const wdt_mchp_dev_cfg_t *const mchp_wdt_cfg = dev->config;
	const hal_mchp_wdt_t *const hal = &mchp_wdt_cfg->hal;
	int ret = 0;

	if (hal_mchp_wdt_is_enabled(hal) != 0) {
		LOG_ERR("Watchdog already setup");
		return -EBUSY;
	}

	if (mchp_wdt_data->installed_timeout_cnt == 0) {
		LOG_ERR("No valid timeout installed");
		return -EINVAL;
	}
	ret = hal_mchp_wdt_apply_options(options);
	if (ret != 0) {
		LOG_ERR("ret val apply = %d", ret);
		return ret;
	}

	ret = hal_mchp_wdt_enable(hal, WDT_ENABLE);
	DBG_WDT("watchdog enabled : 0x%x\n\r", hal_mchp_wdt_is_enabled(hal));
	return 0;
}
/**
 * @brief Disable the WDT.
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, negative error code on failure.
 */
static int wdt_mchp_disable(const struct device *dev)
{
	wdt_mchp_dev_data_t *mchp_wdt_data = dev->data;
	const wdt_mchp_dev_cfg_t *const mchp_wdt_cfg = dev->config;
	const hal_mchp_wdt_t *const hal = &mchp_wdt_cfg->hal;
	int ret = MCHP_SUCCESS;

	mchp_wdt_data->installed_timeout_cnt = 0;
	/*if watchdog is not enabled, then return fault*/
	if (hal_mchp_wdt_is_enabled(hal) == 0) {
		return -EFAULT;
	}
	ret = hal_mchp_wdt_enable(hal, WDT_DISABLE);
	if (ret != 0) {
		LOG_ERR("ret wdg was not disabled = %d", ret);
	}
	return ret;
}

/**
 * @brief Install a timeout for the WDT.
 *
 * @param dev Pointer to the device structure.
 * @param cfg Pointer to the WDT timeout configuration.
 * @return Channel ID on success, negative error code on failure.
 */
static int wdt_mchp_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *cfg)
{
	wdt_mchp_dev_data_t *mchp_wdt_data = dev->data;
	const wdt_mchp_dev_cfg_t *const mchp_wdt_cfg = dev->config;
	const hal_mchp_wdt_t *const hal = &mchp_wdt_cfg->hal;
	wdt_mchp_channel_data_t *channel_data = mchp_wdt_data->channel_data;
	hal_wdt_mchp_channel_data_t actual_set_timeout = {0};
	int ret = 0;

	mchp_wdt_data->cb = cfg->callback;
	mchp_wdt_data->window_mode = (cfg->window.min) > 0;
	mchp_wdt_data->interrupt_enabled = mchp_wdt_data->cb != NULL;
	/* CONFIG is enable protected, error out if already enabled */
	if (hal_mchp_wdt_is_enabled(hal) != 0) {
		LOG_ERR("Watchdog already setup");
		return -EBUSY;
	}
#if defined(WDT_FLAG_ONLY_ONE_TIMEOUT_VALUE_SUPPORTED)
	hal_wdt_mchp_channel_data_t new_set_timeout = {0};
	/*Check whether the new timeout is different from the already existing timeout */
	if (mchp_wdt_data->installed_timeout_cnt != 0) {
		new_set_timeout =
			hal_mchp_wdt_get_available_timeout_val(cfg->window.min, cfg->window.max);
		if ((new_set_timeout.min != channel_data[0].window.min) ||
		    (new_set_timeout.max != channel_data[0].window.max)) {
			return -EINVAL;
		}
	}
#endif

	/*if more installable timeouts are not available the count will be
	 * MAX_INSTALLABLE_TIMEOUT_COUNT
	 */
	if (mchp_wdt_data->installed_timeout_cnt == MAX_INSTALLABLE_TIMEOUT_COUNT) {
		LOG_ERR("No more timeouts available");
		return -ENOMEM;
	}
	/*Set the behaviour of the watcdog peripheral based on the flags supplied */
	ret = hal_mchp_wdt_reset_type_set(cfg->flags);
	if (ret != 0) {
		return ret;
	}
	/*validate the timeout window to be in the range available for the peripheral*/
	ret = hal_mchp_wdt_validate_window(cfg->window.min, cfg->window.max);
	if (ret != 0) {
		LOG_ERR("timeout out of range");
		return ret;
	}
	/*register the provided callback and enable the interrupt*/
	if (mchp_wdt_data->interrupt_enabled != 0) {
		mchp_wdt_data->cb = cfg->callback;
		ret = hal_mchp_wdt_interrupt_enable(hal);
		if (ret == -ENOTSUP) {
			LOG_ERR("Interrupt is not supported for this peripeheral");
		}
	}

	switch (mchp_wdt_data->window_mode) {

	case NORMAL_MODE:
		hal_mchp_wdt_window_enable(hal, WDT_DISABLE);
		break;

	case WINDOW_MODE:
		hal_mchp_wdt_window_enable(hal, WDT_ENABLE);
		break;
	}
	actual_set_timeout = hal_mchp_wdt_set_timeout(hal, cfg->window.min, cfg->window.max);
	/*Update the channel_data structure with the window parameters of each channel*/
	channel_data[mchp_wdt_data->installed_timeout_cnt].window.max = actual_set_timeout.max;
	channel_data[mchp_wdt_data->installed_timeout_cnt].window.min = actual_set_timeout.min;

	LOG_ERR("Rounded off timeout max to %d", actual_set_timeout.max);
	LOG_ERR("Rounded off timeout min to %d", actual_set_timeout.min);

	/*this will return the channel id and then increment the
	 * count which will then be used for the next channel.
	 */
	return mchp_wdt_data->installed_timeout_cnt++;
}
/**
 * @brief Feed the WDT.
 *
 * @param dev Pointer to the device structure.
 * @param channel_id Channel ID to feed.
 * @return 0 on success, negative error code on failure.
 */
static int wdt_mchp_feed(const struct device *dev, int channel_id)
{
	wdt_mchp_dev_data_t *mchp_wdt_data = dev->data;
	const wdt_mchp_dev_cfg_t *const mchp_wdt_cfg = dev->config;
	const hal_mchp_wdt_t *const hal = &mchp_wdt_cfg->hal;

	if (hal_mchp_wdt_is_enabled(hal) == 0) {
		LOG_ERR("Watchdog not setup");
		return -EINVAL;
	}
	if ((channel_id < 0) || (channel_id >= (mchp_wdt_data->installed_timeout_cnt))) {
		LOG_ERR("Invalid channel selected");
		return -EINVAL;
	}
	if (mchp_wdt_data->installed_timeout_cnt == 0) {
		LOG_ERR("No valid timeout installed");
		return -EINVAL;
	}
	hal_mchp_wdt_reset_timer(hal, channel_id);
	return 0;
}
/**
 * @brief WDT driver API structure.
 */
static const struct wdt_driver_api wdt_mchp_driver_api = {
	.setup = wdt_mchp_setup,
	.disable = wdt_mchp_disable,
	.install_timeout = wdt_mchp_install_timeout,
	.feed = wdt_mchp_feed,
};

/**
 * @brief Initialize the WDT.
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, negative error code on failure.
 */
static int wdt_mchp_init(const struct device *dev)
{
	wdt_mchp_dev_data_t *mchp_wdt_data = dev->data;
	const wdt_mchp_dev_cfg_t *const mchp_wdt_cfg = dev->config;

#if defined(CONFIG_WDT_DISABLE_AT_BOOT)
	int ret = wdt_mchp_disable(dev);

	if (ret != 0) {
		LOG_ERR("Watchdog could not be disabled on startup");
	}
#endif
	MCHP_WDT_ENABLE_CLOCK(dev)

	mchp_wdt_data->installed_timeout_cnt = 0;
	mchp_wdt_cfg->irq_config_func(dev);

	return 0;
}

/**
 * @brief Define the WDT IRQ handler.
 *
 * @param n Instance number.
 */
#define WDT_MCHP_IRQ_HANDLER(n)                                                                    \
	static void wdt_mchp_irq_config_##n(const struct device *dev)                              \
	{                                                                                          \
		MCHP_WDT_IRQ_CONNECT(n, 0);                                                        \
	}
/**
 * @brief Connect the WDT IRQ.
 *
 * @param n Instance number.
 * @param m IRQ index.
 */
#define MCHP_WDT_IRQ_CONNECT(n, m)                                                                 \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, m, irq), DT_INST_IRQ_BY_IDX(n, m, priority),     \
			    wdt_mchp_isr, DEVICE_DT_INST_GET(n), 0);                               \
		irq_enable(DT_INST_IRQ_BY_IDX(n, m, irq));                                         \
	} while (false);

/**
 * @brief Declare the WDT IRQ handler.
 *
 * @param n Instance number.
 */
#define WDT_MCHP_IRQ_HANDLER_DECL(n) static void wdt_mchp_irq_config_##n(const struct device *dev)

/**
 * @brief Define the WDT IRQ handler function.
 *
 * @param n Instance number.
 */
#define WDT_MCHP_IRQ_HANDLER_FUNC(n) .irq_config_func = wdt_mchp_irq_config_##n,
/**
 * @brief Define the WDT configuration.
 *
 * @param n Instance number.
 */
#define WDT_MCHP_CONFIG_DEFN(n)                                                                    \
	static const wdt_mchp_dev_cfg_t wdt_mchp_config_##n = {                                    \
		WDT_MCHP_HAL_DEFN(n) WDT_MCHP_IRQ_HANDLER_FUNC(n) WDT_MCHP_CLOCK_DEFN(n)}

/**
 * @brief Initialize the WDT device.
 *
 * @param n Instance number.
 */
#define WDT_MCHP_DEVICE_INIT(n)                                                                    \
	WDT_MCHP_IRQ_HANDLER_DECL(n);                                                              \
	WDT_MCHP_CONFIG_DEFN(n);                                                                   \
	static wdt_mchp_dev_data_t wdt_mchp_data_##n;                                              \
	DEVICE_DT_INST_DEFINE(n, wdt_mchp_init, NULL, &wdt_mchp_data_##n, &wdt_mchp_config_##n,    \
			      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                    \
			      &wdt_mchp_driver_api);                                               \
	WDT_MCHP_IRQ_HANDLER(n)

/**
 * @brief Initialize all WDT instances.
 */
DT_INST_FOREACH_STATUS_OKAY(WDT_MCHP_DEVICE_INIT)
