/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file wdt_mchp_g2.c
 * @brief WDT driver implementation for Microchip Watchdog g2 peripheral
 *
 * This file contains the implementation of the WDT driver for Microchip
 * Technology Inc. devices. It includes initialization, configuration, and
 * setup functions.
 */

#include <soc.h>
#include <stdio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>

/****************************************************************************
 * @brief Devicetree definitions
 ****************************************************************************/
#define DT_DRV_COMPAT microchip_wdt_g2

/****************************************************************************
 * Const and Macro Defines
 ****************************************************************************/
LOG_MODULE_REGISTER(wdt_mchp_g2, CONFIG_WDT_LOG_LEVEL);

/**
 * @brief Type definition for the WDT lock.
 *
 * This macro defines the type of the lock used to protect access to the WDT APIs.
 */
#define WDT_LOCK_TYPE struct k_mutex

/**
 * @brief Timeout duration for acquiring the WDT lock.
 *
 * This macro defines the timeout duration for acquiring the WDT lock.
 * The timeout is specified in milliseconds.
 */
#define WDT_LOCK_TIMEOUT K_MSEC(10)

/**
 * @brief Initialize the WDT lock.
 *
 * This macro initializes the WDT lock.
 *
 * @param p_lock Pointer to the lock to be initialized.
 */
#define WDT_DATA_LOCK_INIT(p_lock) k_mutex_init(p_lock)

/**
 * @brief Acquire the WDT lock.
 *
 * This macro acquires the WDT lock. If the lock is not available, the
 * function will wait for the specified timeout duration.
 *
 * @param p_lock Pointer to the lock to be acquired.
 * @return 0 if the lock was successfully acquired, or a negative error code.
 */
#define WDT_DATA_LOCK(p_lock) k_mutex_lock(p_lock, WDT_LOCK_TIMEOUT)

/**
 * @brief Release the WDT lock.
 *
 * This macro releases the WDT lock.
 *
 * @param p_lock Pointer to the lock to be released.
 * @return 0 if the lock was successfully released, or a negative error code.
 */
#define WDT_DATA_UNLOCK(p_lock) k_mutex_unlock(p_lock)

#define MAX_TIMEOUT_POST_SCALE 1048576 /** Macro indicating max post scaler.*/

#define WDT_LPRC_CLOCK_FREQ 32768 /** Macro indicating LPRC frequency.*/

#define WDT_MCHP_SUCCESS 0 /** Macro indicating successful operation.*/

/***********************************
 * Typedefs and Enum Declarations
 **********************************
 */

/**
 * @enum wdt_mode_t
 * @brief Enumeration for WDT modes.
 */
typedef enum {
	NORMAL_MODE = 0, /**< Normal mode for WDT */
	WINDOW_MODE = 1, /**< Window mode for WDT */
} wdt_mode_t;

/**
 * @struct mchp_wdt_clock
 * @brief Structure to hold device clock configuration.
 */
typedef struct wdt_mchp_clock {
	/* Clock driver */
	const struct device *clock_dev;

	/* Watchdog clock subsystem. */
	clock_control_subsys_t wdtclk_sys;
} mchp_wdt_clock_t;

/**
 * @struct wdt_mchp_dev_data
 * @brief Structure to hold WDT device data.
 */
typedef struct wdt_mchp_dev_data {
	struct wdt_window window; /**< WDT window configuration */
	WDT_LOCK_TYPE lock;
} wdt_mchp_dev_data_t;

/**
 * @struct wdt_mchp_dev_cfg
 * @brief Structure to hold WDT device configuration.
 */
typedef struct wdt_mchp_dev_cfg {
	wdt_registers_t *regs; /**< Pointer to WDT registers */
	mchp_wdt_clock_t wdt_clock;
} wdt_mchp_dev_cfg_t;

/***********************************
 * Internal functions
 **********************************
 */
/**
 * Function to check if the watchdog timer is enabled.
 *
 * This function checks the control register of the watchdog timer to see if
 * either the enable bit or the always-on bit is set. If either bit is set,
 * the function returns true, indicating that the watchdog timer is enabled.
 */
static inline bool wdt_is_enabled(const wdt_registers_t *regs)
{
	return ((regs->WDT_WDTCON & WDT_WDTCON_ON_Msk) == WDT_WDTCON_ON_Msk ? true : false);
}

/**
 * Function to enable or disable the watchdog timer.
 *
 * This function enables or disables the watchdog timer based on the input
 * parameter.
 *
 * It won't be able to disable if the always on bit is turned on
 */
static int wdt_enable(wdt_registers_t *regs, bool enable)
{
	int ret = WDT_MCHP_SUCCESS;

	/* enable watchdog peripheral bit in the ctrl register*/
	if (enable != 0) {
		regs->WDT_WDTCONSET = WDT_WDTCON_ON_Msk;
	} else {
		regs->WDT_WDTCONCLR = WDT_WDTCON_ON_Msk;
	}

	LOG_DBG("ctrl reg = 0x%x\n", regs->WDT_WDTCON);

	return ret;
}
/**
 * Function to get the period index for a given timeout value.
 *
 * It determines the appropriate period index based on the number of clock cycles.
 * The function returns the period index.
 */
static uint32_t wdt_get_period_idx(uint32_t timeout)
{
	uint32_t next_period = 0;
	uint32_t ret_val = 0;

	/* Round up to next period and calculate the register value */
	next_period = (1ULL << 32) >> __builtin_clz(timeout - 1);
	ret_val = find_msb_set(next_period >> 4);
	ret_val = 32 - __builtin_clz(timeout - 1);

	LOG_DBG("%s: %d, %d, %x, %x\n", __func__, timeout, __builtin_clz(timeout - 1), next_period,
		ret_val);

	return ret_val;
}

/**
 * Function to validate the watchdog timer window configuration.
 *
 * This function converts the minimum and maximum timeout values in ms to period scaler index.
 * Then apply it to HW and reply actual timeout.
 * If any of the checks fail, the function returns failure.
 */
static int wdt_set_timeout(const struct device *wdt_dev, const struct wdt_timeout_cfg *cfg,
			   struct wdt_window *actual_set_timeout)
{
	int ret = WDT_MCHP_SUCCESS;
	uint32_t ps_idx, window_idx, window_start, ps, freq_scale, freq;
	wdt_mchp_dev_data_t *mchp_wdt_data = wdt_dev->data;
	const wdt_mchp_dev_cfg_t *const mchp_wdt_cfg = wdt_dev->config;

	const double window_sz[] = {0.75, 0.5, 0.375, 0.25};

	clock_control_get_rate(mchp_wdt_cfg->wdt_clock.clock_dev,
			       mchp_wdt_cfg->wdt_clock.wdtclk_sys, &freq);

	/* Period selection: the expected timeout counter.
	 * Counter 1 is 1 ms based on a 32 kHz (nominal) input clock.
	 * So the timeout value need to be multiplied by freq_scale for other clock source.
	 */
	freq_scale = freq / WDT_LPRC_CLOCK_FREQ;
	if (freq % WDT_LPRC_CLOCK_FREQ) {
		freq_scale++;
	}

	ps = cfg->window.max * freq_scale;
	LOG_DBG("%s: max:%d, freq:%d , freq_scale:%d\n", __func__, cfg->window.max, freq,
		freq_scale);

	if (ps > MAX_TIMEOUT_POST_SCALE) {
		LOG_ERR("Exceed max. timeout value");
		return -EINVAL;
	}

	ps_idx = wdt_get_period_idx(ps);
	ps = (1 << ps_idx) / freq_scale;

	if (cfg->window.min > 0) {
		for (window_idx = 0; window_idx < ARRAY_SIZE(window_sz); window_idx++) {
			window_start = ps * (1 - window_sz[window_idx]);
			if (window_start >= cfg->window.min) {
				break;
			}
		}

		if (window_idx == ARRAY_SIZE(window_sz)) {
			LOG_ERR("window too small");
			return -EINVAL;
		}
	} else {
		window_start = 0;
	}

	/*Check whether the new timeout is different from the already existing
	 *timeout
	 */
	if (mchp_wdt_data->window.max != 0) {
		if ((window_start != mchp_wdt_data->window.min) ||
		    (ps != mchp_wdt_data->window.max)) {
			LOG_ERR("Multiple timeouts values");
			return -EINVAL;
		}
	}

	/*Timeout already installed
	 */
	if (mchp_wdt_data->window.max != 0) {
		LOG_ERR("No more timeouts available");
		return -ENOMEM;
	}

	if (window_start > 0) {
		CFG_REGS->CFG_CFGCON2CLR = CFG_CFGCON2_WDTPSR_Msk | CFG_CFGCON2_WDTWINSZ_Msk;
		CFG_REGS->CFG_CFGCON2SET = CFG_CFGCON2_WDTPSR(ps_idx);
		CFG_REGS->CFG_CFGCON2SET = CFG_CFGCON2_WDTWINSZ(window_idx);
	} else {
		CFG_REGS->CFG_CFGCON2CLR = CFG_CFGCON2_WDTPSR_Msk;
		CFG_REGS->CFG_CFGCON2SET = CFG_CFGCON2_WDTPSR(ps_idx);
	}

	actual_set_timeout->max = ps;
	actual_set_timeout->min = window_start;

	return ret;
}
/**
 * @brief Set the Watchdog Timer (WDT) reset type.
 *
 * This function sets the type of reset that the Watchdog Timer (WDT) will trigger
 * based on the provided flag.
 *
 * @param flag The reset type flag. This can be one of the following:
 *             - WDT_FLAG_RESET_NONE: No reset.
 *             - WDT_FLAG_RESET_CPU_CORE: Reset the CPU core.
 *             - WDT_FLAG_RESET_SOC: Reset the entire system on chip (SoC).
 *
 * @return int Returns WDT_MCHP_SUCCESS on success, or -ENOTSUP if the flag is not supported.
 */
static int wdt_reset_type_set(uint8_t flag)
{
	int ret_val = WDT_MCHP_SUCCESS;

	switch (flag) {
	case WDT_FLAG_RESET_NONE:
		ret_val = -ENOTSUP;
		break;
	case WDT_FLAG_RESET_CPU_CORE:
		break;
	case WDT_FLAG_RESET_SOC:
		break;
	default:
		ret_val = -EINVAL;
		break;
	}

	return ret_val;
}

/**
 * Function to enable or disable the watchdog timer window mode.
 *
 * This function enables or disables the watchdog timer window mode based on the
 * input parameter.
 */
static void wdt_window_enable(wdt_registers_t *regs, bool enable)
{
	/* enable the window in ctrl register*/
	if (enable) {
		regs->WDT_WDTCONSET = WDT_WDTCON_WDTWINEN_Msk;
	} else {
		regs->WDT_WDTCONCLR = WDT_WDTCON_WDTWINEN_Msk;
	}
	LOG_DBG("ctrl reg = 0x%x\n", regs->WDT_WDTCON);
}

/**
 * @brief Apply Watchdog Timer (WDT) options.
 *
 * This function applies the specified options to the Watchdog Timer (WDT).
 * The options can be combined using bitwise OR.
 *
 * @param options The options to apply. This can be a combination of the following:
 *                - WDT_OPT_PAUSE_IN_SLEEP: Pause the WDT when the system is in sleep mode.
 *                - WDT_OPT_PAUSE_HALTED_BY_DBG: Pause the WDT when the system is halted by
 * a debugger.
 *
 * @return int Returns WDT_MCHP_SUCCESS on success, or -ENOTSUP if applying any option
 * fails.
 */
static int wdt_apply_options(wdt_registers_t *regs, uint8_t options)
{
	int ret_val = WDT_MCHP_SUCCESS;

	/* WDT_OPT_PAUSE_HALTED_BY_DBG is supported by default by the peripheral */
	if ((options & WDT_OPT_PAUSE_IN_SLEEP) == 0) {
		ret_val = -ENOTSUP;
	}

	return ret_val;
}
/***********************************
 * Zephyr APIs
 **********************************
 */

/**
 * @brief Setup the Microchip Watchdog Timer (WDT).
 *
 * This function sets up the Microchip Watchdog Timer (WDT) with the specified options.
 * It ensures that the WDT is not already enabled and that a valid timeout is installed
 * before applying the options and enabling the watchdog.
 *
 * @param dev Pointer to the device structure.
 * @param options The setup options for the WDT.
 *
 * @return int Returns WDT_MCHP_SUCCESS on success, -EBUSY if the WDT is already enabled,
 * -EINVAL if no valid timeout is installed, or -ENOTSUP if applied option is not supported
 */
static int wdt_mchp_setup(const struct device *wdt_dev, uint8_t options)
{
	wdt_mchp_dev_data_t *mchp_wdt_data = wdt_dev->data;
	const wdt_mchp_dev_cfg_t *const mchp_wdt_cfg = wdt_dev->config;
	wdt_registers_t *regs = mchp_wdt_cfg->regs;
	int ret = WDT_MCHP_SUCCESS;

	WDT_DATA_LOCK(&mchp_wdt_data->lock);
	do {
		if (wdt_is_enabled(regs) == true) {
			LOG_ERR("Watchdog already setup");
			ret = -EBUSY;
			break;
		}

		if (mchp_wdt_data->window.max == 0) {
			LOG_ERR("No valid timeout installed");
			ret = -EINVAL;
			break;
		}
		ret = wdt_apply_options(regs, options);
		if (ret < 0) {
			LOG_ERR("ret val apply = %d", ret);
			break;
		}
		ret = wdt_enable(regs, true);
		if (ret < 0) {
			LOG_ERR("wdt_enable failed %d", ret);
			break;
		}
		LOG_DBG("watchdog enabled : 0x%x\n\r", wdt_is_enabled(regs));
	} while (0);
	WDT_DATA_UNLOCK(&mchp_wdt_data->lock);

	return ret;
}

/**
 * @brief Disable the Microchip Watchdog Timer (WDT).
 *
 * This function disables the Microchip Watchdog Timer (WDT). It ensures that the WDT is
 * currently enabled before attempting to disable it. If the WDT is not enabled, it returns
 * an error.
 *
 * @param dev Pointer to the device structure.
 *
 * @return int Returns WDT_MCHP_SUCCESS on success, -EFAULT if the WDT is not enabled, or
 * -EPERM if disabling the WDT fails.
 */
static int wdt_mchp_disable(const struct device *wdt_dev)
{
	wdt_mchp_dev_data_t *mchp_wdt_data = wdt_dev->data;
	const wdt_mchp_dev_cfg_t *const mchp_wdt_cfg = wdt_dev->config;
	wdt_registers_t *regs = mchp_wdt_cfg->regs;
	int ret = WDT_MCHP_SUCCESS;
	uint32_t irq_key = 0;

	irq_key = irq_lock();
	do {
		mchp_wdt_data->window.max = 0;

		/*if watchdog is not enabled, then return fault*/
		if (wdt_is_enabled(regs) == false) {
			ret = -EFAULT;
			break;
		}
		ret = wdt_enable(regs, false);
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
 * It ensures that the WDT is not already enabled and that the timeout configuration is
 * valid. It also sets the behavior of the WDT based on the provided flags and enables the
 * interrupt if a callback is provided.
 *
 * @param dev Pointer to the device structure.
 * @param cfg Pointer to the wdt_timeout_cfg structure containing the timeout configuration.
 *
 * @return int Returns the channel ID on success, -EBUSY if the WDT is already enabled,
 * -EINVAL if the timeout configuration is invalid, -ENOMEM if no more timeouts are
 * available, or -ENOTSUP if the interrupt is not supported.
 */
static int wdt_mchp_install_timeout(const struct device *wdt_dev, const struct wdt_timeout_cfg *cfg)
{
	wdt_mchp_dev_data_t *mchp_wdt_data = wdt_dev->data;
	const wdt_mchp_dev_cfg_t *const mchp_wdt_cfg = wdt_dev->config;
	wdt_registers_t *regs = mchp_wdt_cfg->regs;
	struct wdt_window actual_set_timeout = {0};
	int ret = WDT_MCHP_SUCCESS;
	uint32_t irq_key = 0;

	/*Lock the API to prevent other thread from accessing shared resources */
	WDT_DATA_LOCK(&mchp_wdt_data->lock);
	do {
		if (cfg->callback != NULL) {
			LOG_ERR("Callback is not supported");
			ret = -ENOTSUP;
			break;
		}

		if ((cfg->window.max == 0) || (cfg->window.max <= cfg->window.min)) {
			LOG_ERR("Invalid timeout value");
			return -EINVAL;
		}

		/* CONFIG is enable protected, error out if already enabled */
		if (wdt_is_enabled(regs) != 0) {
			LOG_ERR("Watchdog already setup");
			ret = -EBUSY;
			break;
		}

		/*Set the behaviour of the watchdog peripheral based on the flags supplied
		 */
		ret = wdt_reset_type_set(cfg->flags);
		if (ret < 0) {
			break;
		}

		/*validate the timeout window to be in the range available for the
		 *peripheral
		 */
		ret = wdt_set_timeout(wdt_dev, cfg, &actual_set_timeout);
		if (ret < 0) {
			break;
		}

		wdt_window_enable(regs, cfg->window.min != 0);

		/*Update the channel_data structure with the window parameters of each
		 *channel
		 */
		mchp_wdt_data->window.max = actual_set_timeout.max;
		mchp_wdt_data->window.min = actual_set_timeout.min;

		LOG_DBG("Rounded off timeout min to %d", actual_set_timeout.min);
		LOG_DBG("Rounded off timeout max to %d", actual_set_timeout.max);

		/*this will return the channel id and then increment the
		 * count which will then be used for the next channel.
		 */
		irq_key = irq_lock();

		irq_unlock(irq_key);
	} while (0);
	WDT_DATA_UNLOCK(&mchp_wdt_data->lock);

	return ret;
}
/**
 * @brief Feed the WDT.
 *
 * @param dev Pointer to the device structure.
 * @param channel_id Channel ID to feed.
 * @return 0 on success, negative error code on failure.
 */
static int wdt_mchp_feed(const struct device *wdt_dev, int channel_id)
{
	wdt_mchp_dev_data_t *mchp_wdt_data = wdt_dev->data;
	const wdt_mchp_dev_cfg_t *const mchp_wdt_cfg = wdt_dev->config;
	wdt_registers_t *regs = mchp_wdt_cfg->regs;
	int ret = WDT_MCHP_SUCCESS;

	/*Lock mutex only if feed called from a thread */
	if (false == k_is_in_isr()) {
		WDT_DATA_LOCK(&mchp_wdt_data->lock);
	}
	do {
		if (wdt_is_enabled(regs) == false) {
			LOG_ERR("Watchdog not setup");
			ret = -EINVAL;
			break;
		}
		if ((channel_id != 0) || (mchp_wdt_data->window.max == 0)) {
			LOG_ERR("Invalid channel selected");
			ret = -EINVAL;
			break;
		}

		/* Writing specific value to only upper 16 bits of WDTCON register clears WDT
		 * counter. Only write to the upper 16 bits of the register when clearing.
		 * WDTCLRKEY = 0x5743
		 */
		const uint32_t WDT_CLR_REG_ADDRESS = (uint32_t)&regs->WDT_WDTCON + 2U;
		*((volatile uint16_t *)WDT_CLR_REG_ADDRESS) = 0x5743U;
	} while (0);
	if (k_is_in_isr() == false) {
		WDT_DATA_UNLOCK(&mchp_wdt_data->lock);
	}

	return ret;
}
/**
 * @brief WDT driver API structure.
 */
static DEVICE_API(wdt, wdt_mchp_api) = {
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
static int wdt_mchp_init(const struct device *wdt_dev)
{
	wdt_mchp_dev_data_t *mchp_wdt_data = wdt_dev->data;
	const wdt_mchp_dev_cfg_t *const mchp_wdt_cfg = wdt_dev->config;
	int ret_val = 0;

	WDT_DATA_LOCK_INIT(&mchp_wdt_data->lock);
	do {
#if defined(CONFIG_WDT_DISABLE_AT_BOOT)
		ret_val = wdt_mchp_disable(wdt_dev);
		if (ret_val < 0) {
			LOG_ERR("Watchdog could not be disabled on startup");
			break;
		}
#endif /* CONFIG_WDT_DISABLE_AT_BOOT */
		ret_val = clock_control_on(mchp_wdt_cfg->wdt_clock.clock_dev,
					   mchp_wdt_cfg->wdt_clock.wdtclk_sys);
		if ((ret_val < 0) && (ret_val != -EALREADY)) {
			LOG_ERR("Clock control on failed %d", ret_val);
			break;
		}

		mchp_wdt_data->window.max = 0;

	} while (0);
	ret_val = (ret_val == -EALREADY) ? 0 : ret_val;

	return ret_val;
}

/**
 * @brief Define the WDT configuration.
 *
 * @param n Instance number.
 */
#define WDT_MCHP_CONFIG_DEFN(n)                                                                    \
	static const wdt_mchp_dev_cfg_t wdt_mchp_config_##n = {                                    \
		.regs = (wdt_registers_t *)DT_INST_REG_ADDR(n),                                    \
		.wdt_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                         \
		.wdt_clock.wdtclk_sys =                                                            \
			(void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, wdtclk, subsystem))}

/**
 * @brief Initialize the WDT device.
 *
 * @param n Instance number.
 */
#define WDT_MCHP_DEVICE_INIT(n)                                                                    \
	WDT_MCHP_CONFIG_DEFN(n);                                                                   \
	static wdt_mchp_dev_data_t wdt_mchp_data_##n;                                              \
	DEVICE_DT_INST_DEFINE(n, wdt_mchp_init, NULL, &wdt_mchp_data_##n, &wdt_mchp_config_##n,    \
			      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &wdt_mchp_api);

/**
 * @brief Initialize all WDT instances.
 */
DT_INST_FOREACH_STATUS_OKAY(WDT_MCHP_DEVICE_INIT)
