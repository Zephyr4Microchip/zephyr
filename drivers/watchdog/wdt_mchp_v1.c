/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file wdt_mchp_v1.c
 * @brief WDT driver implementation for Microchip Technology Inc. devices
 *
 * This file contains the implementation of the WDT driver for Microchip
 * Technology Inc. devices. It includes initialization, configuration, and
 * setup functions.
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
	wdt_callback_t callback;       /**< Callback function for WDT */
	bool interrupt_enabled;        /**< Flag to indicate if interrupt is enabled */
	bool window_mode;              /**< Flag to indicate if window mode is enabled */
	uint8_t installed_timeout_cnt; /**< Number of installed timeouts */
	wdt_mchp_channel_data_t channel_data[MAX_INSTALLABLE_TIMEOUT_COUNT]; /**< Channel data */
	MCHP_WDT_LOCK_TYPE lock;
} wdt_mchp_dev_data_t;

/**
 * @struct wdt_mchp_dev_cfg
 * @brief Structure to hold WDT device configuration.
 */
typedef struct wdt_mchp_dev_cfg {
	hal_mchp_wdt_t hal_wdt;
	void (*irq_config_func)(const struct device *dev); /**< Function to configure IRQ */
	mchp_wdt_clock_t wdt_clock;
} wdt_mchp_dev_cfg_t;

/**
 * @brief Set the Watchdog Timer (WDT) reset type.
 *
 * This function sets the type of reset that the Watchdog Timer (WDT) will trigger
 * based on the provided flag. The function will call the appropriate hardware
 * abstraction layer (HAL) function to set the reset type.
 *
 * @param flag The reset type flag. This can be one of the following:
 *             - WDT_FLAG_RESET_NONE: No reset.
 *             - WDT_FLAG_RESET_CPU_CORE: Reset the CPU core.
 *             - WDT_FLAG_RESET_SOC: Reset the entire system on chip (SoC).
 *
 * @return int Returns WDT_MCHP_SUCCESS on success, or -ENOTSUP if the flag is not supported.
 */
static inline int wdt_mchp_reset_type_set(const hal_mchp_wdt_t *hal, uint8_t flag)
{
	int hal_ret = WDT_MCHP_SUCCESS;
	switch (flag) {
	case WDT_FLAG_RESET_NONE:
		hal_ret = hal_mchp_wdt_reset_none(hal);
		break;
	case WDT_FLAG_RESET_CPU_CORE:
		hal_ret = hal_mchp_wdt_reset_cpu_core(hal);
		break;
	case WDT_FLAG_RESET_SOC:
		hal_ret = hal_mchp_wdt_reset_soc(hal);
		break;
	default:
		hal_ret = -EINVAL;
		break;
	}

	return (hal_ret < 0) ? -ENOTSUP : WDT_MCHP_SUCCESS;
}

/**
 * @brief WDT interrupt service routine.
 *
 * @param dev Pointer to the device structure.
 */
static void wdt_mchp_isr(const struct device *dev)
{
	wdt_mchp_dev_data_t *mchp_wdt_data = dev->data;
	const wdt_mchp_dev_cfg_t *const mchp_wdt_cfg = dev->config;
	const hal_mchp_wdt_t *const hal = &mchp_wdt_cfg->hal_wdt;

	hal_mchp_wdt_interrupt_flag_clear(hal);
	if (mchp_wdt_data->callback != NULL) {
		mchp_wdt_data->callback(dev, 0);
	}
}

/**
 * @brief Apply Watchdog Timer (WDT) options.
 *
 * This function applies the specified options to the Watchdog Timer (WDT).
 * The options can be combined using bitwise OR. The function will call the
 * appropriate hardware abstraction layer (HAL) functions to apply the options.
 *
 * @param options The options to apply. This can be a combination of the following:
 *                - WDT_OPT_PAUSE_IN_SLEEP: Pause the WDT when the system is in sleep mode.
 *                - WDT_OPT_PAUSE_HALTED_BY_DBG: Pause the WDT when the system is halted by a
 * debugger.
 *
 * @return int Returns WDT_MCHP_SUCCESS on success, or -ENOTSUP if applying any option fails.
 */
static inline int wdt_mchp_apply_options(const hal_mchp_wdt_t *hal, uint8_t options)
{
	int hal_ret = WDT_MCHP_SUCCESS;
	if ((options & WDT_OPT_PAUSE_IN_SLEEP) != 0) {
		hal_ret = hal_mchp_wdt_pause_in_sleep(hal);
	}
	if ((hal_ret >= 0) && (options & WDT_OPT_PAUSE_HALTED_BY_DBG) != 0) {
		hal_ret = hal_mchp_wdt_pause_halted_by_debug(hal);
	}

	return (hal_ret < 0) ? -ENOTSUP : WDT_MCHP_SUCCESS;
}

/**
 * @brief Setup the Microchip Watchdog Timer (WDT).
 *
 * This function sets up the Microchip Watchdog Timer (WDT) with the specified options.
 * It ensures that the WDT is not already enabled and that a valid timeout is installed before
 * applying the options and enabling the watchdog.
 *
 * @param dev Pointer to the device structure.
 * @param options The setup options for the WDT.
 *
 * @return int Returns WDT_MCHP_SUCCESS on success, -EBUSY if the WDT is already enabled, -EINVAL
 * if no valid timeout is installed, or -ENOTSUP if applied option is not supported
 */
static int wdt_mchp_setup(const struct device *dev, uint8_t options)
{
	wdt_mchp_dev_data_t *mchp_wdt_data = dev->data;
	const wdt_mchp_dev_cfg_t *const mchp_wdt_cfg = dev->config;
	const hal_mchp_wdt_t *const hal = &mchp_wdt_cfg->hal_wdt;
	int ret = WDT_MCHP_SUCCESS;
	MCHP_WDT_DATA_LOCK(&mchp_wdt_data->lock);
	do {
		if (hal_mchp_wdt_is_enabled(hal) == true) {
			LOG_ERR("Watchdog already setup");
			ret = -EBUSY;
			break;
		}

		if (mchp_wdt_data->installed_timeout_cnt == 0) {
			LOG_ERR("No valid timeout installed");
			ret = -EINVAL;
			break;
		}
		ret = wdt_mchp_apply_options(hal, options);
		if (ret < 0) {
			LOG_ERR("ret val apply = %d", ret);
			break;
		}
		hal_mchp_wdt_enable(hal, true);
		DBG_WDT("watchdog enabled : 0x%x\n\r", hal_mchp_wdt_is_enabled(hal));
	} while (0);
	MCHP_WDT_DATA_UNLOCK(&mchp_wdt_data->lock);
	return ret;
}

/**
 * @brief Disable the Microchip Watchdog Timer (WDT).
 *
 * This function disables the Microchip Watchdog Timer (WDT). It ensures that the WDT is currently
 * enabled before attempting to disable it. If the WDT is not enabled, it returns an error.
 *
 * @param dev Pointer to the device structure.
 *
 * @return int Returns WDT_MCHP_SUCCESS on success, -EFAULT if the WDT is not enabled, or -EPERM if
 * disabling the WDT fails.
 */
static int wdt_mchp_disable(const struct device *dev)
{
	wdt_mchp_dev_data_t *mchp_wdt_data = dev->data;
	const wdt_mchp_dev_cfg_t *const mchp_wdt_cfg = dev->config;
	const hal_mchp_wdt_t *const hal = &mchp_wdt_cfg->hal_wdt;
	int ret = WDT_MCHP_SUCCESS;
	uint32_t irq_key = 0;

	irq_key = irq_lock();
	do {
		mchp_wdt_data->installed_timeout_cnt = 0;
		/*if watchdog is not enabled, then return fault*/
		if (hal_mchp_wdt_is_enabled(hal) == false) {
			ret = -EFAULT;
			break;
		}
		ret = hal_mchp_wdt_enable(hal, false);
		if (ret < 0) {
			ret = -EPERM;
			LOG_ERR("wdg was not disabled = %d", ret);
		}
	} while (0);
	irq_unlock(irq_key);

	return ret;
}

/**
 * @brief Install a timeout configuration for the Microchip Watchdog Timer (WDT).
 *
 * This function installs a timeout configuration for the Microchip Watchdog Timer (WDT).
 * It ensures that the WDT is not already enabled and that the timeout configuration is valid.
 * It also sets the behavior of the WDT based on the provided flags and enables the interrupt if a
 * callback is provided.
 *
 * @param dev Pointer to the device structure.
 * @param cfg Pointer to the wdt_timeout_cfg structure containing the timeout configuration.
 *
 * @return int Returns the channel ID on success, -EBUSY if the WDT is already enabled, -EINVAL if
 * the timeout configuration is invalid, -ENOMEM if no more timeouts are available, or -ENOTSUP if
 * the interrupt is not supported.
 */
static int wdt_mchp_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *cfg)
{
	wdt_mchp_dev_data_t *mchp_wdt_data = dev->data;
	const wdt_mchp_dev_cfg_t *const mchp_wdt_cfg = dev->config;
	const hal_mchp_wdt_t *const hal = &mchp_wdt_cfg->hal_wdt;
	wdt_mchp_channel_data_t *channel_data = mchp_wdt_data->channel_data;
	hal_wdt_mchp_channel_data_t actual_set_timeout = {0};
	int ret = WDT_MCHP_SUCCESS;
	uint32_t irq_key = 0;

	/*Lock the API to prevent other thread from accessing shared resources */
	MCHP_WDT_DATA_LOCK(&mchp_wdt_data->lock);
	do {
		mchp_wdt_data->callback = cfg->callback;
		if (hal_mchp_wdt_win_mode_supported(hal) == WDT_MCHP_SUCCESS) {
			mchp_wdt_data->window_mode = (((cfg->window.min) > 0) ? true : false);
		}
		mchp_wdt_data->interrupt_enabled =
			((mchp_wdt_data->callback != NULL) ? true : false);
		/* CONFIG is enable protected, error out if already enabled */
		if (hal_mchp_wdt_is_enabled(hal) != 0) {
			LOG_ERR("Watchdog already setup");
			ret = -EBUSY;
			break;
		}
#if defined(WDT_FLAG_ONLY_ONE_TIMEOUT_VALUE_SUPPORTED)
		hal_wdt_mchp_channel_data_t new_set_timeout = {0};
		/*Check whether the new timeout is different from the already existing timeout */
		if (mchp_wdt_data->installed_timeout_cnt != 0) {
			new_set_timeout =
				hal_mchp_wdt_get_timeout_val(cfg->window.min, cfg->window.max);
			if ((new_set_timeout.min != channel_data[0].window.min) ||
			    (new_set_timeout.max != channel_data[0].window.max)) {
				ret = -EINVAL;
				break;
			}
		}
#endif

		/*if more installable timeouts are not available the count will be
		 * MAX_INSTALLABLE_TIMEOUT_COUNT
		 */
		if (mchp_wdt_data->installed_timeout_cnt == MAX_INSTALLABLE_TIMEOUT_COUNT) {
			LOG_ERR("No more timeouts available");
			ret = -ENOMEM;
			break;
		}
		/*Set the behaviour of the watchdog peripheral based on the flags supplied */
		ret = wdt_mchp_reset_type_set(hal, cfg->flags);
		if (ret < 0) {
			break;
		}
		/*validate the timeout window to be in the range available for the peripheral*/
		ret = hal_mchp_wdt_validate_window(cfg->window.min, cfg->window.max);
		if (ret < 0) {
			LOG_ERR("timeout out of range");
			ret = -EINVAL;
			break;
		}
		/*register the provided callback and enable the interrupt*/
		if (mchp_wdt_data->interrupt_enabled != 0) {
			mchp_wdt_data->callback = cfg->callback;
			ret = hal_mchp_wdt_interrupt_enable(hal);
			if (ret < 0) {
				LOG_ERR("Interrupt is not supported for this peripeheral");
				ret = -ENOTSUP;
				break;
			}
		}

		switch (mchp_wdt_data->window_mode) {
		case NORMAL_MODE:
			hal_mchp_wdt_window_enable(hal, false);
			break;

		case WINDOW_MODE:
			hal_mchp_wdt_window_enable(hal, true);
			break;
		}
		actual_set_timeout =
			hal_mchp_wdt_set_timeout(hal, cfg->window.min, cfg->window.max);
		/*Update the channel_data structure with the window parameters of each channel*/
		channel_data[mchp_wdt_data->installed_timeout_cnt].window.max =
			actual_set_timeout.max;
		channel_data[mchp_wdt_data->installed_timeout_cnt].window.min =
			actual_set_timeout.min;

		LOG_ERR("Rounded off timeout min to %d", actual_set_timeout.min);
		LOG_ERR("Rounded off timeout max to %d", actual_set_timeout.max);

		/*this will return the channel id and then increment the
		 * count which will then be used for the next channel.
		 */
		irq_key = irq_lock();
		ret = mchp_wdt_data->installed_timeout_cnt++;
		irq_unlock(irq_key);
	} while (0);
	MCHP_WDT_DATA_UNLOCK(&mchp_wdt_data->lock);
	return ret;
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
	const hal_mchp_wdt_t *const hal = &mchp_wdt_cfg->hal_wdt;
	int ret = WDT_MCHP_SUCCESS;
	/*Lock mutex only if feed called from a thread */
	if (k_is_in_isr() == false) {
		MCHP_WDT_DATA_LOCK(&mchp_wdt_data->lock);
	}
	do {
		if (hal_mchp_wdt_is_enabled(hal) == 0) {
			LOG_ERR("Watchdog not setup");
			ret = -EINVAL;
			break;
		}
		if ((channel_id < 0) || (channel_id >= (mchp_wdt_data->installed_timeout_cnt))) {
			LOG_ERR("Invalid channel selected");
			ret = -EINVAL;
			break;
		}
		if (mchp_wdt_data->installed_timeout_cnt == 0) {
			LOG_ERR("No valid timeout installed");
			ret = -EINVAL;
			break;
		}
		hal_mchp_wdt_reset_timer(hal, channel_id);
	} while (0);
	if (k_is_in_isr() == false) {
		MCHP_WDT_DATA_UNLOCK(&mchp_wdt_data->lock);
	}
	return ret;
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
	MCHP_WDT_DATA_LOCK_INIT(&mchp_wdt_data->lock);
#if defined(CONFIG_WDT_DISABLE_AT_BOOT)
	int ret = wdt_mchp_disable(dev);

	if (ret < 0) {
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
