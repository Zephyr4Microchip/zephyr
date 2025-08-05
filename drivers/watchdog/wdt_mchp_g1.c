/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file wdt_mchp_g1.c
 * @brief WDT driver implementation for Microchip Watchdog g1 peripheral
 *
 * This file contains the implementation of the WDT driver for Microchip
 * Technology Inc. devices. It includes initialization, configuration, and
 * setup functions.
 */

#include <soc.h>
#include <stdio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>

/****************************************************************************
 * @brief Devicetree definitions
 ****************************************************************************/
#define DT_DRV_COMPAT microchip_wdt_g1

/****************************************************************************
 * Const and Macro Defines
 ****************************************************************************/
LOG_MODULE_REGISTER(wdt_mchp_g1, CONFIG_WDT_LOG_LEVEL);

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

/**
 * @def MAX_INSTALLABLE_TIMEOUT_COUNT
 * @brief Macro to get the maximum installable timeout count from the device
 * tree.
 */
#define MAX_INSTALLABLE_TIMEOUT_COUNT (DT_PROP(DT_NODELABEL(wdog), max_installable_timeout_count))

/**
 * @def MAX_TIMEOUT_WINDOW
 * @brief Macro to get the maximum timeout window from the device tree.
 */
#define MAX_TIMEOUT_WINDOW (DT_PROP(DT_NODELABEL(wdog), max_timeout_window))

/**
 * @def MAX_TIMEOUT_WINDOW_MODE
 * @brief Macro to get the maximum timeout window mode from the device tree.
 */
#define MAX_TIMEOUT_WINDOW_MODE (DT_PROP(DT_NODELABEL(wdog), max_timeout_window_mode))

/**
 * @def MIN_WINDOW_LIMIT
 * @brief Macro to get the min window limit from the device tree.
 */
#define MIN_WINDOW_LIMIT (DT_PROP(DT_NODELABEL(wdog), min_window_limit))

/**
 * @def WDT_FLAG_ONLY_ONE_TIMEOUT_VALUE_SUPPORTED
 * @brief Macro to get the value of "only_one_timeout_val_supported_flag" from
 * the device tree.
 */
#define WDT_FLAG_ONLY_ONE_TIMEOUT_VALUE_SUPPORTED                                                  \
	(DT_PROP(DT_NODELABEL(wdog), only_one_timeout_val_supported_flag))

/**< Either of these bits will be set in case watchdog is turned on */
#define WDT_ENABLED_BITS_POS (WDT_CTRLA_ENABLE(1) | WDT_CTRLA_ALWAYSON(1))

#define WDT_MCHP_SUCCESS 0 /** Macro indicating successful operation.*/

#define WDT_MCHP_FAIL -1 /** Macro indicating failed operation. */

/**
 * @brief Macro to calculate the period value.
 *
 * This macro takes an integer input `n` and calculates the period value by
 * left-shifting the number 8 by `n` positions.
 *
 * @param n The number of positions to left-shift the number 8.
 * @return The calculated period value.
 *
 * @note The result of this macro is 8 * 2^n.
 */
#define PERIOD_VALUE(n) (8 << n)

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

	/* Main clock subsystem. */
	clock_control_subsys_t mclk_sys;
} mchp_wdt_clock_t;

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
	WDT_LOCK_TYPE lock;
} wdt_mchp_dev_data_t;

/**
 * @struct wdt_mchp_dev_cfg
 * @brief Structure to hold WDT device configuration.
 */
typedef struct wdt_mchp_dev_cfg {
	wdt_registers_t *regs;                             /**< Pointer to WDT registers */
	void (*irq_config_func)(const struct device *dev); /**< Function to configure IRQ */
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
	/* the significance of 0x82 is that it
	 * denotes the always on bit and the enable bit
	 */
	return ((regs->WDT_CTRLA & WDT_ENABLED_BITS_POS) != 0) ? true : false;
}
/**
 * Function to wait for the watchdog timer synchronization.
 *
 * This function waits until the synchronization busy bit in the watchdog timer
 * control register is cleared, indicating that the synchronization is complete.
 */
static void wdt_sync_wait(const wdt_registers_t *regs)
{
	while (regs->WDT_SYNCBUSY != 0) {
	}
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
		regs->WDT_CTRLA |= WDT_CTRLA_ENABLE(1);
	} else {
		/*check always on bit here*/
		if (0 == (regs->WDT_CTRLA & WDT_CTRLA_ALWAYSON(1))) {
			regs->WDT_CTRLA &= ~WDT_CTRLA_ENABLE(1);
		} else {
			ret = WDT_MCHP_FAIL;
		}
	}
	wdt_sync_wait(regs);
	LOG_DBG("ctrl reg = 0x%x\n", regs->WDT_CTRLA);

	return ret;
}
/**
 * Function to get the period index for a given timeout value.
 *
 * This function calculates the number of clock cycles required for the given
 * timeout value in milliseconds. It then determines the appropriate period
 * index based on the number of clock cycles. The function returns the period
 * index.
 */
static uint32_t wdt_get_period_idx(uint32_t timeout_ms)
{
	uint32_t next_period = 0;
	uint32_t cycles = 0;
	uint32_t ret_val = 0;

	do {
		/* Calculate number of clock cycles @ 1.024 kHz input clock */
		cycles = (timeout_ms * 1024U) / 1000;

		/* Minimum wdt period is 8 clock cycles (register value 0) */
		if (cycles <= MIN_WINDOW_LIMIT) {
			break;
		}

		/* Round up to next period and calculate the register value */
		next_period = (1ULL << 32) >> __builtin_clz(cycles - 1);
		ret_val = find_msb_set(next_period >> 4);
	} while (0);

	return ret_val;
}

/**
 * Function to get the watchdog timer timeout values.
 *
 * This function calculates the minimum and maximum timeout values for the
 * watchdog timer based on the input parameters. It calculates the appropriate
 * period indices for the window closed time and the maximum timeout value. The
 * function returns the calculated timeout values.
 *
 */
static wdt_mchp_channel_data_t wdt_get_timeout_val(uint32_t window_closed_time,
						   uint32_t timeout_max)
{
	wdt_mchp_channel_data_t new_timeout = {0};
	uint8_t window_index = wdt_get_period_idx(window_closed_time);
	uint8_t per_index = wdt_get_period_idx(timeout_max - window_closed_time);

	new_timeout.window.min = (window_closed_time ? PERIOD_VALUE(window_index) : 0);
	new_timeout.window.max =
		(window_closed_time ? (PERIOD_VALUE(per_index) + PERIOD_VALUE(window_index))
				    : PERIOD_VALUE(per_index));

	return new_timeout;
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
 * Function to validate the watchdog timer window configuration.
 *
 * This function checks the validity of the minimum and maximum timeout values
 * for the watchdog timer window mode. It performs various checks to ensure that
 * the timeout values are within the allowed range and that the window
 * configuration is valid. If any of the checks fail, the function returns
 * failure.
 */
static inline int wdt_validate_window(uint32_t timeout_min, uint32_t timeout_max)
{
	int ret = WDT_MCHP_SUCCESS;

	switch (0) {
	case 0:
		/*Check whether the timeout max is greater
		 *than the maximum possible value in window mode
		 */
		if ((timeout_max >= MAX_TIMEOUT_WINDOW_MODE) && (timeout_min != 0)) {
			ret = WDT_MCHP_FAIL;
			break;
		}
	case 1:
		/*check whether the timeout max given is zero*/
		if (timeout_max == 0) {
			ret = WDT_MCHP_FAIL;
			break;
		}
	case 2:
		/*Check whether the timeout min is not
		 * less than the minimum possible window
		 */
		if ((timeout_min < PERIOD_VALUE(0)) && (timeout_min != 0)) {
			ret = WDT_MCHP_FAIL;
			break;
		}
	case 3:
		/*Ensure that a window is available)*/
		if (timeout_min > (timeout_max >> 1)) {
			ret = WDT_MCHP_FAIL;
			break;
		}
	case 4:
		/*this will ensure that the timeout range is within
		 * the limit for both normal mode and window mode
		 */
		if ((timeout_max - timeout_min) > MAX_TIMEOUT_WINDOW) {
			ret = WDT_MCHP_FAIL;
			break;
		}
	}

	return ret;
}
/**
 * Function to enable watchdog timer interrupts.
 *
 * This function always returns failure, indicating that enabling watchdog timer
 * interrupts is not supported.
 */
static inline int wdt_interrupt_enable(wdt_registers_t *regs)
{
	ARG_UNUSED(regs);

	return WDT_MCHP_FAIL;
}
/**
 * Function to clear the watchdog timer interrupt flag.
 *
 * This function always returns failure, indicating that clearing the watchdog
 * timer interrupt flag is not supported.
 */
static inline int wdt_interrupt_flag_clear(wdt_registers_t *regs)
{
	ARG_UNUSED(regs);

	return WDT_MCHP_FAIL;
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
	if (enable != 0) {
		regs->WDT_CTRLA |= WDT_CTRLA_WEN(1);
	} else {
		regs->WDT_CTRLA &= ~WDT_CTRLA_WEN(1);
	}
	wdt_sync_wait(regs);
	LOG_DBG("ctrl reg = 0x%x\n", regs->WDT_CTRLA);
}

/**
 * Function to set the watchdog timer timeout values.
 *
 * This function sets the minimum and maximum timeout values for the watchdog
 * timer based on the input parameters. It calculates the appropriate period
 * indices for the window closed time and the maximum timeout value. The
 * function then sets the window and period values in the watchdog timer
 * configuration register.
 */
static wdt_mchp_channel_data_t wdt_set_timeout(wdt_registers_t *regs, uint32_t window_closed_time,
					       uint32_t timeout_max)
{

	wdt_mchp_channel_data_t set_timeout = {0};
	uint8_t window = wdt_get_period_idx(window_closed_time);

	/* The difference is taken as the total time of WDT
	 * defined by the CONFIG.window + CONFIG.per register value
	 */
	uint8_t per = wdt_get_period_idx(timeout_max - window_closed_time);

	LOG_DBG("window = %d : 0x%x per = %d : 0x%x\n\r", window, WDT_CONFIG_WINDOW(window), per,
		WDT_CONFIG_PER(per));

	set_timeout.window.min = window_closed_time ? PERIOD_VALUE(window) : 0;

	/*Based on the mode the window mode or normal mode, timeout max is returned
	 */
	set_timeout.window.max = (window_closed_time ? (PERIOD_VALUE(per) + PERIOD_VALUE(window))
						     : PERIOD_VALUE(per));
	regs->WDT_CONFIG = WDT_CONFIG_WINDOW(window) | WDT_CONFIG_PER(per);
	wdt_sync_wait(regs);
	LOG_DBG("wdt_config = 0x%x\n\r", regs->WDT_CONFIG);

	return set_timeout;
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
	if ((options & WDT_OPT_PAUSE_IN_SLEEP) != 0) {
		ret_val = -ENOTSUP;
	}

	return ret_val;
}
/***********************************
 * Zephyr APIs
 **********************************
 */
/**
 * @brief WDT interrupt service routine.
 *
 * @param dev Pointer to the device structure.
 */
static void wdt_mchp_isr(const struct device *wdt_dev)
{
	wdt_mchp_dev_data_t *mchp_wdt_data = wdt_dev->data;
	const wdt_mchp_dev_cfg_t *const mchp_wdt_cfg = wdt_dev->config;

	wdt_interrupt_flag_clear(mchp_wdt_cfg->regs);
	if (mchp_wdt_data->callback != NULL) {
		mchp_wdt_data->callback(wdt_dev, 0);
	}
}

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

		if (mchp_wdt_data->installed_timeout_cnt == 0) {
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
		mchp_wdt_data->installed_timeout_cnt = 0;

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
	wdt_mchp_channel_data_t *channel_data = mchp_wdt_data->channel_data;
	wdt_mchp_channel_data_t actual_set_timeout = {0};
	int ret = WDT_MCHP_SUCCESS;
	uint32_t irq_key = 0;

	/*Lock the API to prevent other thread from accessing shared resources */
	WDT_DATA_LOCK(&mchp_wdt_data->lock);
	do {
		mchp_wdt_data->callback = cfg->callback;
		mchp_wdt_data->window_mode = (((cfg->window.min) > 0) ? true : false);
		mchp_wdt_data->interrupt_enabled =
			((mchp_wdt_data->callback != NULL) ? true : false);

		/* CONFIG is enable protected, error out if already enabled */
		if (wdt_is_enabled(regs) != 0) {
			LOG_ERR("Watchdog already setup");
			ret = -EBUSY;
			break;
		}
#if defined(WDT_FLAG_ONLY_ONE_TIMEOUT_VALUE_SUPPORTED)
		wdt_mchp_channel_data_t new_set_timeout = {0};

		/*Check whether the new timeout is different from the already existing
		 *timeout
		 */
		if (mchp_wdt_data->installed_timeout_cnt != 0) {
			new_set_timeout = wdt_get_timeout_val(cfg->window.min, cfg->window.max);
			if ((new_set_timeout.window.min != channel_data[0].window.min) ||
			    (new_set_timeout.window.max != channel_data[0].window.max)) {
				ret = -EINVAL;
				break;
			}
		}
#endif /* WDT_FLAG_ONLY_ONE_TIMEOUT_VALUE_SUPPORTED */

		/*if more installable timeouts are not available the count will be
		 * MAX_INSTALLABLE_TIMEOUT_COUNT
		 */
		if (mchp_wdt_data->installed_timeout_cnt == MAX_INSTALLABLE_TIMEOUT_COUNT) {
			LOG_ERR("No more timeouts available");
			ret = -ENOMEM;
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
		ret = wdt_validate_window(cfg->window.min, cfg->window.max);
		if (ret < 0) {
			LOG_ERR("timeout out of range");
			ret = -EINVAL;
			break;
		}

		/*register the provided callback and enable the interrupt*/
		if (mchp_wdt_data->interrupt_enabled != 0) {
			mchp_wdt_data->callback = cfg->callback;
			ret = wdt_interrupt_enable(regs);
			if (ret < 0) {
				LOG_ERR("Interrupt is not supported for this peripeheral");
				ret = -ENOTSUP;
				break;
			}
		}

		switch (mchp_wdt_data->window_mode) {
		case NORMAL_MODE:
			wdt_window_enable(regs, false);
			break;
		case WINDOW_MODE:
			wdt_window_enable(regs, true);
			break;
		}
		actual_set_timeout = wdt_set_timeout(regs, cfg->window.min, cfg->window.max);

		/*Update the channel_data structure with the window parameters of each
		 *channel
		 */
		channel_data[mchp_wdt_data->installed_timeout_cnt].window.max =
			actual_set_timeout.window.max;
		channel_data[mchp_wdt_data->installed_timeout_cnt].window.min =
			actual_set_timeout.window.min;

		LOG_ERR("Rounded off timeout min to %d", actual_set_timeout.window.min);
		LOG_ERR("Rounded off timeout max to %d", actual_set_timeout.window.max);

		/*this will return the channel id and then increment the
		 * count which will then be used for the next channel.
		 */
		irq_key = irq_lock();
		ret = (mchp_wdt_data->installed_timeout_cnt)++;
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

		/* Clear the watchdog timer */
		regs->WDT_CLEAR = WDT_CLEAR_CLEAR_KEY_Val;
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
					   mchp_wdt_cfg->wdt_clock.mclk_sys);
		if ((ret_val < 0) && (ret_val != -EALREADY)) {
			LOG_ERR("Clock control on failed for MCLK %d", ret_val);
			break;
		}

		mchp_wdt_data->installed_timeout_cnt = 0;
		mchp_wdt_cfg->irq_config_func(wdt_dev);
	} while (0);
	ret_val = (ret_val == -EALREADY) ? 0 : ret_val;

	return ret_val;
}

/**
 * @brief Define the WDT IRQ handler.
 *
 * @param n Instance number.
 */
#define WDT_MCHP_IRQ_HANDLER(n)                                                                    \
	static void wdt_mchp_irq_config_##n(const struct device *wdt_dev)                          \
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
#define WDT_MCHP_IRQ_HANDLER_DECL(n)                                                               \
	static void wdt_mchp_irq_config_##n(const struct device *wdt_dev)

/**
 * @brief Define the WDT configuration.
 *
 * @param n Instance number.
 */
#define WDT_MCHP_CONFIG_DEFN(n)                                                                    \
	static const wdt_mchp_dev_cfg_t wdt_mchp_config_##n = {                                    \
		.regs = (wdt_registers_t *)DT_INST_REG_ADDR(n),                                    \
		.irq_config_func = wdt_mchp_irq_config_##n,                                        \
		.wdt_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                         \
		.wdt_clock.mclk_sys = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, subsystem))}

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
			      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &wdt_mchp_api);    \
	WDT_MCHP_IRQ_HANDLER(n)

/**
 * @brief Initialize all WDT instances.
 */
DT_INST_FOREACH_STATUS_OKAY(WDT_MCHP_DEVICE_INIT)
