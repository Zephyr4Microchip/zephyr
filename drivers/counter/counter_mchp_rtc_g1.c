/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * @file counter_mchp_rtc_g1.c
 * @brief Counter driver implementation for Microchip devices.
 */

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>

/*****************************************************************************
 * Devicetree definitions
 *****************************************************************************/
#define DT_DRV_COMPAT microchip_rtc_g1_counter

LOG_MODULE_REGISTER(counter_mchp_rtc_g1, CONFIG_COUNTER_LOG_LEVEL);

/* All timer/counter synchronization bits set. */
#define ALL_RTC_SYNC_BITS ((uint32_t)UINT32_MAX)

/* Return value for failed operation. */
#define COUNTER_RET_FAILED (-1)

/* Return value for successful operation. */
#define COUNTER_RET_PASSED (0)

/* Synchronization time-out in us */
#define RTC_SYNCHRONIZATION_TIMEOUT_IN_US (5000)

#define DELAY_US (10)

/*
 * @section:  Datatypes
 *
 * This section includes necessary user defined data-types used to implement
 * the counter driver for microchip devices
 */

/*
 * @brief Clock configuration structure for the Counter.
 *
 * This structure contains the clock configuration parameters for the Counter
 * peripheral.
 */
typedef struct mchp_counter_clock {
	/* Clock driver */
	const struct device *clock_dev;

	/* Main clock subsystem. */
	clock_control_subsys_t host_core_sync_clk;

	/* Generic clock subsystem. */
	clock_control_subsys_t periph_async_clk;

} counter_mchp_clock_t;

typedef struct rtc_counter_irq_map {
	/* Overflow channel */
	uint32_t ovf_irq_line;

	/* Alarm interrupt number */
	uint32_t comp_irq_line;
} rtc_counter_irq_map_t;

/*
 * @brief Structure to hold channel-specific data for the counter.
 *
 * This structure holds the callback function and user data for a specific channel of the counter.
 */
typedef struct counter_mchp_ch_data {
	/* Callback function for the channel alarm */
	counter_alarm_callback_t callback;

	/* Compare value	*/
	uint32_t compare_value;

	/* User data to be passed to the callback function */
	void *user_data;

} counter_mchp_ch_data_t;

/*
 * @brief Structure to hold data for the counter.
 *
 * This structure holds the top callback function, user data, and channel-specific data for the
 * counter.
 */
typedef struct counter_mchp_dev_data {
	/* Callback function for the top event */
	counter_top_callback_t top_cb;

	/* User data to be passed to the top callback function */
	void *top_user_data;

	/* Late alarm flag	  */
	bool late_alarm_flag;

	/* Late alarm channel	 */
	uint8_t late_alarm_channel;

	/* Guard period */
	uint32_t guard_period;

	/* Channel-specific data */
	counter_mchp_ch_data_t *channel_data;

} counter_mchp_dev_data_t;

/*
 * @brief Structure to hold configuration for the counter.
 *
 * This structure holds the configuration information, register pointers, pin control configuration,
 * clock settings, prescaler value, and IRQ configuration function for the counter.
 */
typedef struct counter_mchp_dev_config {
	/* General counter configuration information */
	struct counter_config_info info;

	/* Register address */
	void *regs;

	/* Pin control configuration */
	const struct pinctrl_dev_config *pcfg;

	/* Clock configuration */
	counter_mchp_clock_t counter_clock;

	/* Channel IRQ map */
	uint32_t irq_line;

	/* Maximum bit-width */
	uint32_t max_bit_width;

	/* Prescaler value */
	uint16_t prescaler;

	/* IRQ configuration function */
	void (*irq_config_func)(const struct device *dev);

} counter_mchp_dev_cfg_t;

/* Enumeration for counter bit modes */
typedef enum {
	COUNTER_BIT_MODE_8 = 8,
	COUNTER_BIT_MODE_16 = 16,
	COUNTER_BIT_MODE_32 = 32,
} rtc_counter_modes_t;

/* this function will ge the value of the prescale index from prescaler val in dts */
inline uint8_t get_rtc_prescale_index(uint16_t prescaler)
{
	return __builtin_ctz(prescaler) + 1;
}

/*
 * @brief Wait for SYNCBUSY register bit to clear with 5ms timeout.
 *
 * @param sync_reg_addr Pointer to SYNCBUSY register address.
 * @param bit_mask Bit mask to check for synchronization.
 */
static void rtc_counter_wait_sync(const volatile uint32_t *sync_reg_addr, uint32_t bit_mask)
{
	/* WAIT_FOR returns true if condition met before timeout, false otherwise */
	bool success = WAIT_FOR((((*sync_reg_addr) & bit_mask) == 0U),
				RTC_SYNCHRONIZATION_TIMEOUT_IN_US, k_busy_wait(DELAY_US));

	if (!success) {
		LOG_ERR("%s : Synchronization time-out occurred", __func__);
	}
}

/*
 * @brief Wait until the counter value differs from the RTC_COUNT register, with 5ms timeout.
 *
 * This function polls the RTC_COUNT register and waits until its value differs
 * from the value pointed to by @p counter_value. If the value does not change
 * within 5 milliseconds, the function returns a timeout error.
 *
 * @param counter_value Pointer to the value to compare against RTC_COUNT.
 * @param p_regs Pointer to the RTC register structure containing RTC_COUNT.
 */
static void rtc_counter_wait_count_change(const void *regs, const uint32_t *counter_value,
					  const uint32_t max_bit_width)
{
	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		rtc_mode0_registers_t *const p_regs =
			(rtc_mode0_registers_t *const)(&(((rtc_registers_t *)regs)->MODE0));

		if ((p_regs->RTC_CTRLA & RTC_MODE0_CTRLA_ENABLE_Msk) != 0) {
			bool success =
				WAIT_FOR((*counter_value != p_regs->RTC_COUNT),
					 RTC_SYNCHRONIZATION_TIMEOUT_IN_US, k_busy_wait(DELAY_US));
			if (!success) {
				LOG_ERR("%s : Synchronization time-out occurred %d", __func__,
					max_bit_width);
			}
		}
		break;
	}
	case COUNTER_BIT_MODE_16: {
		rtc_mode1_registers_t *const p_regs =
			(rtc_mode1_registers_t *const)(&(((rtc_registers_t *)regs)->MODE1));

		if ((p_regs->RTC_CTRLA & RTC_MODE0_CTRLA_ENABLE_Msk) != 0) {
			bool success =
				WAIT_FOR((*counter_value != p_regs->RTC_COUNT),
					 RTC_SYNCHRONIZATION_TIMEOUT_IN_US, k_busy_wait(DELAY_US));
			if (!success) {
				LOG_ERR("%s : Synchronization time-out occurred %d", __func__,
					max_bit_width);
			}
		}
		break;
	}
	default:
		LOG_ERR("%s : Unsupported Counter mode %d", __func__, max_bit_width);
		break;
	}
}

/*
 * @brief Initialize the RTC counter peripheral.
 *
 * This function configures the RTC counter with the specified prescaler and bit width.
 * It performs the following steps:
 *	- Disables and resets the counter.
 *	- Configures the counter for either 16-bit or 32-bit mode, based on max_bit_width.
 *	- Sets the counter to count up in free-running mode.
 *	- Sets the period and compare values to their maximum.
 *	- Sets CTRLA.MATCHCLR to use channel 0 to control top value.
 *		(This configures the RTC so that when the counter value matches the value
 *		in compare channel 0, the counter is automatically cleared to zero. This
 *		allows for set and control top value)
 *
 * @param regs         Pointer to the RTC peripheral registers.
 * @param prescaler    Prescaler value for the counter.
 * @param max_bit_width Counter bit width (must be 16 or 32).
 *
 * @retval COUNTER_RET_PASSED   Counter initialized successfully.
 * @retval COUNTER_RET_FAILED  Invalid bit width specified.
 */
static int32_t rtc_counter_init(const void *regs, uint32_t prescaler, const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		rtc_mode0_registers_t *const p_regs =
			(rtc_mode0_registers_t *const)(&(((rtc_registers_t *)regs)->MODE0));

		p_regs->RTC_CTRLA &= ~RTC_MODE0_CTRLA_ENABLE_Msk;
		rtc_counter_wait_sync(&p_regs->RTC_SYNCBUSY, RTC_MODE0_SYNCBUSY_ENABLE_Msk);

		p_regs->RTC_CTRLA = RTC_MODE0_CTRLA_SWRST_Msk;
		rtc_counter_wait_sync(&p_regs->RTC_SYNCBUSY, RTC_MODE0_SYNCBUSY_SWRST_Msk);

		p_regs->RTC_CTRLA = RTC_MODE0_CTRLA_MODE(0U) | RTC_MODE0_CTRLA_MATCHCLR(1U) |
				    RTC_MODE0_CTRLA_COUNTSYNC(1U) |
				    RTC_MODE0_CTRLA_PRESCALER(get_rtc_prescale_index(prescaler));

		p_regs->RTC_COMP[0U] = UINT32_MAX;
		p_regs->RTC_COMP[1U] = UINT32_MAX;
		p_regs->RTC_INTFLAG = RTC_MODE0_INTFLAG_Msk;
		rtc_counter_wait_sync(&p_regs->RTC_SYNCBUSY, ALL_RTC_SYNC_BITS);
		break;
	}
	case COUNTER_BIT_MODE_16: {
		rtc_mode1_registers_t *const p_regs =
			(rtc_mode1_registers_t *const)(&(((rtc_registers_t *)regs)->MODE1));

		p_regs->RTC_CTRLA &= ~RTC_MODE1_CTRLA_ENABLE_Msk;
		rtc_counter_wait_sync(&p_regs->RTC_SYNCBUSY, RTC_MODE1_SYNCBUSY_ENABLE_Msk);

		p_regs->RTC_CTRLA = RTC_MODE1_CTRLA_SWRST_Msk;
		rtc_counter_wait_sync(&p_regs->RTC_SYNCBUSY, RTC_MODE1_SYNCBUSY_SWRST_Msk);

		p_regs->RTC_CTRLA = RTC_MODE1_CTRLA_MODE(1U) | RTC_MODE1_CTRLA_COUNTSYNC(1U) |
				    RTC_MODE1_CTRLA_PRESCALER(get_rtc_prescale_index(prescaler));

		p_regs->RTC_PER = UINT16_MAX;
		p_regs->RTC_COMP[0U] = UINT16_MAX;
		p_regs->RTC_COMP[1U] = UINT16_MAX;
		p_regs->RTC_COMP[2U] = UINT16_MAX;
		p_regs->RTC_COMP[3U] = UINT16_MAX;
		p_regs->RTC_INTFLAG = RTC_MODE1_INTFLAG_Msk;
		rtc_counter_wait_sync(&p_regs->RTC_SYNCBUSY, ALL_RTC_SYNC_BITS);
		break;
	}
	default:
		LOG_ERR("%s : Unsupported Counter mode %d", __func__, max_bit_width);
		return_value = COUNTER_RET_FAILED;
		break;
	}

	return return_value;
}

/*
 * @brief Disable the RTC counter peripheral.
 *
 * Clears the enable bit for the RTC counter in the specified mode.
 *
 * @param regs			Pointer to the RTC peripheral registers.
 * @param max_bit_width Maximum bit width (16 or 32) to select the counter mode.
 *
 * @retval COUNTER_RET_PASSED   Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t rtc_counter_disable(const void *regs, const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		rtc_mode0_registers_t *const p_regs =
			(rtc_mode0_registers_t *const)(&(((rtc_registers_t *)regs)->MODE0));
		p_regs->RTC_CTRLA &= ~RTC_MODE0_CTRLA_ENABLE_Msk;
		rtc_counter_wait_sync(&p_regs->RTC_SYNCBUSY, RTC_MODE0_SYNCBUSY_ENABLE_Msk);
		break;
	}
	case COUNTER_BIT_MODE_16: {
		rtc_mode1_registers_t *const p_regs =
			(rtc_mode1_registers_t *const)(&(((rtc_registers_t *)regs)->MODE1));
		p_regs->RTC_CTRLA &= ~RTC_MODE1_CTRLA_ENABLE_Msk;
		rtc_counter_wait_sync(&p_regs->RTC_SYNCBUSY, RTC_MODE1_SYNCBUSY_ENABLE_Msk);
		break;
	}
	default:
		LOG_ERR("%s : Unsupported Counter mode %d", __func__, max_bit_width);
		return_value = COUNTER_RET_FAILED;
		break;
	}

	return return_value;
}

/*
 * @brief Start the RTC counter.
 *
 * Start the counter.
 *
 * @param regs			Pointer to the RTC peripheral registers.
 * @param max_bit_width Maximum bit width (16 or 32) to select the counter mode.
 *
 * @retval COUNTER_RET_PASSED   Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t rtc_counter_start(const void *regs, const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		rtc_mode0_registers_t *const p_regs =
			(rtc_mode0_registers_t *const)(&(((rtc_registers_t *)regs)->MODE0));
		p_regs->RTC_CTRLA |= RTC_MODE0_CTRLA_ENABLE_Msk;
		rtc_counter_wait_sync(&p_regs->RTC_SYNCBUSY, RTC_MODE0_SYNCBUSY_ENABLE_Msk);
	}
	case COUNTER_BIT_MODE_16: {
		rtc_mode1_registers_t *const p_regs =
			(rtc_mode1_registers_t *const)(&(((rtc_registers_t *)regs)->MODE1));
		p_regs->RTC_CTRLA |= RTC_MODE1_CTRLA_ENABLE_Msk;
		rtc_counter_wait_sync(&p_regs->RTC_SYNCBUSY, RTC_MODE1_SYNCBUSY_ENABLE_Msk);
		break;
	}
	default:
		LOG_ERR("%s : Unsupported Counter mode %d", __func__, max_bit_width);
		return_value = COUNTER_RET_FAILED;
		break;
	}

	return return_value;
}

/*
 * @brief Stop the RTC counter.
 *
 * Alias for rtc_counter_disable().
 *
 * @param regs			Pointer to the RTC peripheral registers.
 * @param max_bit_width Maximum bit width (16 or 32) to select the counter mode.
 *
 * @retval COUNTER_RET_PASSED   Success.
 * @retval COUNTER_RET_FAILED   Invalid bit width.
 */
static inline int32_t rtc_counter_stop(const void *regs, const uint32_t max_bit_width)
{
	return rtc_counter_disable(regs, max_bit_width);
}

/*
 * @brief Retrigger the RTC counter.
 *
 * Resets the counter value to zero and restarts the counter.
 *
 * @param regs			Pointer to the RTC peripheral registers.
 * @param max_bit_width Maximum bit width (16 or 32) to select the counter mode.
 *
 * @retval COUNTER_RET_PASSED  Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t rtc_counter_retrigger(const void *regs, const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		rtc_mode0_registers_t *const p_regs =
			(rtc_mode0_registers_t *const)(&(((rtc_registers_t *)regs)->MODE0));
		p_regs->RTC_COUNT = 0u;
		p_regs->RTC_CTRLA |= RTC_MODE0_CTRLA_ENABLE_Msk;
		rtc_counter_wait_sync(&p_regs->RTC_SYNCBUSY, ALL_RTC_SYNC_BITS);
		break;
	}
	case COUNTER_BIT_MODE_16: {
		rtc_mode1_registers_t *const p_regs =
			(rtc_mode1_registers_t *const)(&(((rtc_registers_t *)regs)->MODE1));
		p_regs->RTC_COUNT = 0u;
		p_regs->RTC_CTRLA |= RTC_MODE1_CTRLA_ENABLE_Msk;
		rtc_counter_wait_sync(&p_regs->RTC_SYNCBUSY, ALL_RTC_SYNC_BITS);
		break;
	}
	default:
		LOG_ERR("%s : Unsupported Counter mode %d", __func__, max_bit_width);
		return_value = COUNTER_RET_FAILED;
		break;
	}

	return return_value;
}

/*
 * @brief Get the current RTC counter value.
 *
 * Reads and returns the current count value of the RTC counter.
 *
 * @param regs Pointer to the RTC peripheral registers.
 * @param counter_value	Pointer to store counter value.
 * @param max_bit_width Maximum bit width (16 or 32) to select the counter mode.
 *
 * @retval COUNTER_RET_PASSED  Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static uint32_t rtc_counter_get_count(const void *regs, uint32_t *const counter_value,
				      const uint32_t max_bit_width)
{
	uint32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		rtc_mode0_registers_t *const p_regs =
			(rtc_mode0_registers_t *const)(&(((rtc_registers_t *)regs)->MODE0));

		if ((p_regs->RTC_CTRLA & RTC_MODE0_CTRLA_COUNTSYNC_Msk) == 0u) {
			p_regs->RTC_CTRLA |= RTC_MODE0_CTRLA_COUNTSYNC_Msk;
			rtc_counter_wait_sync(&p_regs->RTC_SYNCBUSY,
					      RTC_MODE0_SYNCBUSY_COUNTSYNC_Msk);
		}
		rtc_counter_wait_sync(&p_regs->RTC_SYNCBUSY, RTC_MODE0_SYNCBUSY_COUNT_Msk);

		*counter_value = p_regs->RTC_COUNT;
		/* RTC Errata: TMR102-19
		 *   When COUNTSYNC is enabled, the first COUNT value is not correctly
		 *   synchronized and thus it is a wrong value.
		 * Workaround
		 *   After enabling COUNTSYNC, read the COUNT register until its
		 *   value is changed when compared to its first value read. After this,
		 *   all consequent value read from the COUNT register is valid.
		 */
		rtc_counter_wait_count_change(regs, counter_value, max_bit_width);
		*counter_value = p_regs->RTC_COUNT;

		break;
	}
	case COUNTER_BIT_MODE_16: {
		rtc_mode1_registers_t *const p_regs =
			(rtc_mode1_registers_t *const)(&(((rtc_registers_t *)regs)->MODE1));
		*counter_value = p_regs->RTC_COUNT;

		/* RTC Errata: TMR102-19
		 *   When COUNTSYNC is enabled, the first COUNT value is not correctly
		 *   synchronized and thus it is a wrong value.
		 * Workaround
		 *   After enabling COUNTSYNC, read the COUNT register until its
		 *   value is changed when compared to its first value read. After this,
		 *   all consequent value read from the COUNT register is valid.
		 */
		rtc_counter_wait_count_change(regs, counter_value, max_bit_width);
		*counter_value = p_regs->RTC_COUNT;

		break;
	}
	default:
		LOG_ERR("%s : Unsupported Counter mode %d", __func__, max_bit_width);
		return_value = COUNTER_RET_FAILED;
		break;
	}

	return return_value;
}
/*
 * @brief Set the period value.
 *
 * Sets the period value of the RTC peripheral.
 *
 * @param regs			Pointer to the RTC peripheral registers.
 * @param period		The period value to set.
 * @param max_bit_width Maximum bit width (16 or 32) to select the counter mode.
 *
 * @retval COUNTER_RET_PASSED  Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t rtc_counter_set_period(const void *regs, const uint32_t period,
				      const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		rtc_mode0_registers_t *const p_regs =
			(rtc_mode0_registers_t *const)(&(((rtc_registers_t *)regs)->MODE0));
		p_regs->RTC_COMP[0] = period;
		rtc_counter_wait_sync(&p_regs->RTC_SYNCBUSY, RTC_MODE0_SYNCBUSY_COMP0_Msk);
		break;
	}
	case COUNTER_BIT_MODE_16: {
		rtc_mode1_registers_t *const p_regs =
			(rtc_mode1_registers_t *const)(&(((rtc_registers_t *)regs)->MODE1));
		p_regs->RTC_PER = period;
		rtc_counter_wait_sync(&p_regs->RTC_SYNCBUSY, RTC_MODE1_SYNCBUSY_PER_Msk);
		break;
	}
	default:
		LOG_ERR("%s : Unsupported Counter mode %d", __func__, max_bit_width);
		return_value = COUNTER_RET_FAILED;
		break;
	}

	return return_value;
}

/*
 * @brief Get the current period value.
 *
 * Gets the current period value of the RTC peripheral.
 *
 * @param regs	Pointer to the RTC peripheral registers.
 * @param period	Pointer to store period value.
 * @param max_bit_width Maximum bit width (16 or 32) to select the counter mode.
 *
 * @retval COUNTER_RET_PASSED  Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t rtc_counter_get_period(const void *regs, uint32_t *period,
				      const uint32_t max_bit_width)
{
	uint32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		rtc_mode0_registers_t *const p_regs =
			(rtc_mode0_registers_t *const)(&(((rtc_registers_t *)regs)->MODE0));
		*period = p_regs->RTC_COMP[0];
		break;
	}
	case COUNTER_BIT_MODE_16: {
		rtc_mode1_registers_t *const p_regs =
			(rtc_mode1_registers_t *const)(&(((rtc_registers_t *)regs)->MODE1));
		*period = p_regs->RTC_PER;
		break;
	}
	default:
		LOG_ERR("%s : Unsupported Counter mode %d", __func__, max_bit_width);
		return_value = COUNTER_RET_FAILED;
		break;
	}

	return return_value;
}

/*
 * @brief Set the compare value.
 *
 * Sets the compare value for a specified channel of the RTC peripheral.
 *
 * @param regs			Pointer to the RTC peripheral registers.
 * @param chan_id		Compare channel index.
 * @param compare_value Value to set for the compare channel.
 * @param max_bit_width Maximum bit width (16 or 32) to select the counter mode.
 *
 * @retval COUNTER_RET_PASSED  Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t rtc_counter_set_compare(const void *regs, const uint32_t chan_id,
				       const uint32_t compare_value, const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		rtc_mode0_registers_t *const p_regs =
			(rtc_mode0_registers_t *const)(&(((rtc_registers_t *)regs)->MODE0));

		p_regs->RTC_COMP[1u] = compare_value;
		rtc_counter_wait_sync(&p_regs->RTC_SYNCBUSY, ALL_RTC_SYNC_BITS);
		break;
	}
	case COUNTER_BIT_MODE_16: {
		rtc_mode1_registers_t *const p_regs =
			(rtc_mode1_registers_t *const)(&(((rtc_registers_t *)regs)->MODE1));

		p_regs->RTC_COMP[chan_id] = compare_value;
		rtc_counter_wait_sync(&p_regs->RTC_SYNCBUSY, ALL_RTC_SYNC_BITS);
		break;
	}
	default:
		LOG_ERR("%s : Unsupported Counter mode %d", __func__, max_bit_width);
		return_value = COUNTER_RET_FAILED;
		break;
	}

	return return_value;
}

/*
 * @brief Check if an interrupt is pending.
 *
 * Checks if any interrupt is pending for the RTC peripheral.
 *
 * @param regs			Pointer to the RTC peripheral registers.
 * @param max_bit_width Maximum bit width (16 or 32) to select the counter mode.
 * @return Interrupt flag register value
 */
static int32_t rtc_counter_get_pending_irqs(const void *regs, const uint32_t max_bit_width)
{
	int32_t ret_status = 0;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		rtc_mode0_registers_t *const p_regs =
			(rtc_mode0_registers_t *const)(&(((rtc_registers_t *)regs)->MODE0));

		ret_status = p_regs->RTC_INTFLAG;
		break;
	}
	case COUNTER_BIT_MODE_16: {
		rtc_mode1_registers_t *const p_regs =
			(rtc_mode1_registers_t *const)(&(((rtc_registers_t *)regs)->MODE1));

		ret_status = p_regs->RTC_INTFLAG;
		break;
	}
	default:
		LOG_ERR("%s : Unsupported Counter mode %d", __func__, max_bit_width);
		break;
	}

	return ret_status;
}

/*
 * @brief Enable alarm interrupt for a specific channel.
 *
 * Enables the alarm interrupt for a specified channel of the RTC peripheral.
 *
 * @param regs			Pointer to the RTC peripheral registers.
 * @param channel_id	The channel ID to enable the alarm interrupt for.
 * @param max_bit_width Maximum bit width (16 or 32) to select the counter mode.
 *
 * @retval COUNTER_RET_PASSED  Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t rtc_counter_alarm_irq_enable(const void *regs, const uint32_t channel_id,
					    const uint32_t max_bit_width)
{
	int32_t ret_status = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		rtc_mode0_registers_t *const p_regs =
			(rtc_mode0_registers_t *const)(&(((rtc_registers_t *)regs)->MODE0));

		if (channel_id == 0u) {
			p_regs->RTC_INTENSET = RTC_MODE0_INTFLAG_CMP1_Msk;
		} else {
			ret_status = COUNTER_RET_FAILED;
		}
		break;
	}
	case COUNTER_BIT_MODE_16: {
		rtc_mode1_registers_t *const p_regs =
			(rtc_mode1_registers_t *const)(&(((rtc_registers_t *)regs)->MODE1));

		p_regs->RTC_INTENSET = RTC_MODE0_INTFLAG_CMP0_Msk << channel_id;

		break;
	}
	default:
		LOG_ERR("%s : Unsupported Counter mode %d", __func__, max_bit_width);
		ret_status = COUNTER_RET_FAILED;
		break;
	}

	return ret_status;
}

/*
 * @brief Disable alarm interrupt for a specific channel.
 *
 * Disables the alarm interrupt for a specified channel of the RTC peripheral.
 *
 * @param regs			Pointer to the RTC peripheral registers.
 * @param channel_id	The channel ID to disable the alarm interrupt for.
 * @param max_bit_width Maximum bit width (16 or 32) to select the counter mode.
 *
 * @retval COUNTER_RET_PASSED  Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t rtc_counter_alarm_irq_disable(const void *regs, const uint32_t channel_id,
					     const uint32_t max_bit_width)
{
	int32_t ret_status = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		rtc_mode0_registers_t *const p_regs =
			(rtc_mode0_registers_t *const)(&(((rtc_registers_t *)regs)->MODE0));

		if (channel_id == 0u) {
			p_regs->RTC_INTENCLR = RTC_MODE0_INTFLAG_CMP1_Msk;
		} else {
			ret_status = COUNTER_RET_FAILED;
		}
		break;
	}
	case COUNTER_BIT_MODE_16: {
		rtc_mode1_registers_t *const p_regs =
			(rtc_mode1_registers_t *const)(&(((rtc_registers_t *)regs)->MODE1));

		p_regs->RTC_INTENCLR = 1U << ((uint32_t)RTC_MODE1_INTFLAG_CMP0_Msk + channel_id);
	}
	default:
		LOG_ERR("%s : Unsupported Counter mode %d", __func__, max_bit_width);
		ret_status = COUNTER_RET_FAILED;
		break;
	}

	return ret_status;
}

/*
 * @brief Clear alarm interrupt for a specific channel.
 *
 * Clears the alarm interrupt for a specified channel of the RTC peripheral.
 *
 * @param regs			Pointer to the RTC peripheral registers.
 * @param channel_id	The channel ID to clear the alarm interrupt for.
 * @param max_bit_width Maximum bit width (16 or 32) to select the counter mode.
 *
 * @retval COUNTER_RET_PASSED  Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t rtc_counter_alarm_irq_clear(const void *regs, const uint32_t channel_id,
					   const uint32_t max_bit_width)
{
	int32_t ret_status = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		rtc_mode0_registers_t *const p_regs =
			(rtc_mode0_registers_t *const)(&(((rtc_registers_t *)regs)->MODE0));

		if (channel_id == 0u) {
			p_regs->RTC_INTFLAG = RTC_MODE0_INTFLAG_CMP1_Msk;
		} else {
			ret_status = COUNTER_RET_FAILED;
		}
		break;
	}
	case COUNTER_BIT_MODE_16: {
		rtc_mode1_registers_t *const p_regs =
			(rtc_mode1_registers_t *const)(&(((rtc_registers_t *)regs)->MODE1));

		p_regs->RTC_INTFLAG = RTC_MODE0_INTFLAG_CMP0_Msk << channel_id;
		break;
	}
	default:
		LOG_ERR("%s : Unsupported Counter mode %d", __func__, max_bit_width);
		ret_status = COUNTER_RET_FAILED;
		break;
	}

	return ret_status;
}

/*
 * @brief Check the alarm interrupt status for a specific channel.
 *
 * Checks the alarm interrupt status for a specified channel of the RTC peripheral.
 *
 * @param pending_irq_status  Interrupt flag status.
 * @param channel_id	The channel ID to check the alarm interrupt status for.
 * @param max_bit_width Maximum bit width (16 or 32) to select the counter mode.
 *
 * @retval COUNTER_RET_PASSED  Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static bool rtc_counter_alarm_irq_status(const uint32_t pending_irq_status,
					 const uint32_t channel_id, const uint32_t max_bit_width)
{
	int32_t ret_status = true;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		if (channel_id == 0u) {
			ret_status = (bool)((pending_irq_status & RTC_MODE0_INTFLAG_CMP1_Msk) ==
					    RTC_MODE0_INTFLAG_CMP1_Msk);
		} else {
			ret_status = false;
		}
		break;
	}
	case COUNTER_BIT_MODE_16: {
		uint32_t channel_mask = RTC_MODE1_INTFLAG_CMP0_Msk << channel_id;

		ret_status =
			(bool)((pending_irq_status & channel_mask) == RTC_MODE1_INTFLAG_CMP0_Msk);
		break;
	}
	default:
		LOG_ERR("%s : Unsupported Counter mode %d", __func__, max_bit_width);
		ret_status = false;
		break;
	}

	return ret_status;
}

/*
 * @brief Enable overflow interrupt.
 *
 * Enables the overflow interrupt for the RTC peripheral.
 *
 * @param regs			Pointer to the RTC peripheral registers.
 * @param max_bit_width Maximum bit width (16 or 32) to select the counter mode.
 *
 * @retval COUNTER_RET_PASSED  Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t rtc_counter_top_irq_enable(const void *regs, const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		rtc_mode0_registers_t *const p_regs =
			(rtc_mode0_registers_t *const)(&(((rtc_registers_t *)regs)->MODE0));
		p_regs->RTC_INTENSET = RTC_MODE0_INTFLAG_CMP0_Msk;
		break;
	}
	case COUNTER_BIT_MODE_16: {
		rtc_mode1_registers_t *const p_regs =
			(rtc_mode1_registers_t *const)(&(((rtc_registers_t *)regs)->MODE1));
		p_regs->RTC_INTENSET = RTC_MODE1_INTFLAG_OVF_Msk;
		break;
	}
	default:
		LOG_ERR("%s : Unsupported Counter mode %d", __func__, max_bit_width);
		return_value = COUNTER_RET_FAILED;
		break;
	}

	return return_value;
}
/*
 * @brief Disable overflow interrupt.
 *
 * Disables the overflow interrupt for the RTC peripheral.
 *
 * @param regs			Pointer to the RTC peripheral registers.
 * @param max_bit_width Maximum bit width (16 or 32) to select the counter mode.
 *
 * @retval COUNTER_RET_PASSED  Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t rtc_counter_top_irq_disable(const void *regs, const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		rtc_mode0_registers_t *const p_regs =
			(rtc_mode0_registers_t *const)(&(((rtc_registers_t *)regs)->MODE0));

		p_regs->RTC_INTENCLR = RTC_MODE0_INTFLAG_CMP0_Msk;
		break;
	}
	case COUNTER_BIT_MODE_16: {
		rtc_mode1_registers_t *const p_regs =
			(rtc_mode1_registers_t *const)(&(((rtc_registers_t *)regs)->MODE1));

		p_regs->RTC_INTENCLR = RTC_MODE1_INTFLAG_OVF_Msk;
		break;
	}
	default:
		LOG_ERR("%s : Unsupported Counter mode %d", __func__, max_bit_width);
		return_value = COUNTER_RET_FAILED;
		break;
	}

	return return_value;
}

/*
 * @brief Clear overflow interrupt.
 *
 * Clears the overflow interrupt for the RTC peripheral.
 *
 * @param regs			Pointer to the RTC peripheral registers.
 * @param max_bit_width Maximum bit width (16 or 32) to select the counter mode.
 *
 * @retval COUNTER_RET_PASSED  Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t rtc_counter_top_irq_clear(const void *regs, const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		rtc_mode0_registers_t *const p_regs =
			(rtc_mode0_registers_t *const)(&(((rtc_registers_t *)regs)->MODE0));

		p_regs->RTC_INTFLAG = RTC_MODE0_INTFLAG_CMP0_Msk;
		break;
	}
	case COUNTER_BIT_MODE_16: {
		rtc_mode1_registers_t *const p_regs =
			(rtc_mode1_registers_t *const)(&(((rtc_registers_t *)regs)->MODE1));

		p_regs->RTC_INTFLAG = RTC_MODE1_INTFLAG_OVF_Msk;
		break;
	}
	default:
		LOG_ERR("%s : Unsupported Counter mode %d", __func__, max_bit_width);
		return_value = COUNTER_RET_FAILED;
		break;
	}

	return return_value;
}

/*
 * @brief Check the overflow interrupt status.
 *
 * Checks the overflow interrupt status for the RTC peripheral.
 *
 * @param pending_irq_status  Interrupt flag status.
 * @param max_bit_width Maximum bit width (16 or 32) to select the counter mode.
 * @return true if the interrupt is pending, false otherwise
 */
static bool rtc_counter_top_irq_status(const uint32_t pending_irq_status,
				       const uint32_t max_bit_width)
{
	bool return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		return_value = (bool)((pending_irq_status & RTC_MODE0_INTFLAG_CMP0_Msk) ==
				      RTC_MODE0_INTFLAG_CMP0_Msk);
		break;
	}
	case COUNTER_BIT_MODE_16: {
		return_value = (bool)((pending_irq_status & RTC_MODE1_INTFLAG_OVF_Msk) ==
				      RTC_MODE1_INTFLAG_OVF_Msk);
		break;
	}
	default:
		LOG_ERR("%s : Unsupported Counter mode %d", __func__, max_bit_width);
		return_value = false;
		break;
	}

	return return_value;
}

/*
 * @brief Computes the difference between two tick values considering wraparound.
 *
 * This function calculates `val - old`, handling cases where the counter
 * has wrapped around. If `top` is a power of two minus one, bitwise AND is used
 * for efficiency.
 *
 * @param[in] val  Newer tick value.
 * @param[in] old  Older tick value.
 * @param[in] top  Maximum counter value before wraparound.
 *
 * @return Difference `(val - old) % (top + 1)`, ensuring proper wraparound handling.
 */
static uint32_t rtc_counter_ticks_sub(uint32_t val, uint32_t old, uint32_t top)
{
	if (true == likely(IS_BIT_MASK(top))) {
		return (val - old) & top;
	}

	/* If top is not 2^n - 1, handle general wraparound case */
	return (val >= old) ? (val - old) : val + top + 1U - old;
}

/*
 * @brief Adds two tick values considering counter wraparound.
 *
 * This function adds `val1` and `val2` while ensuring proper wraparound
 * behavior when the counter reaches `top`.
 *
 * @param[in] val1 First tick value.
 * @param[in] val2 Second tick value.
 * @param[in] top  Maximum counter value before wraparound.
 *
 * @return Result of (val1 + val2) modulo `top`.
 */
static uint32_t rtc_counter_ticks_add(uint32_t val1, uint32_t val2, uint32_t top)
{
	uint64_t sum = (uint64_t)(val1 + val2);

	if (true == likely(IS_BIT_MASK(top))) {
		return sum & top;
	}

	return (uint32_t)(sum % top);
}

/*
 * @brief Computes absolute difference between two counter values with wraparound.
 *
 * This handles wraparound by computing the minimum distance between the two
 * values on a circular counter that resets at `top`.
 *
 * @param a	  First counter value
 * @param b	  Second counter value
 * @param top Maximum counter value before wraparound
 * @return Absolute difference in ticks (always positive)
 */
static uint32_t rtc_counter_ticks_diff(uint32_t a, uint32_t b, uint32_t top)
{
	uint32_t diff = (a > b) ? (a - b) : (b - a);
	uint32_t wrap_diff = top - diff;

	return (diff < wrap_diff) ? diff : wrap_diff;
}

/*
 * @brief rtc_counter_start the counter.
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, negative error code on failure.
 */
static int32_t counter_mchp_start(const struct device *const dev)
{
	const counter_mchp_dev_cfg_t *const cfg = dev->config;

	rtc_counter_start(cfg->regs, cfg->max_bit_width);

	return 0;
}

/*
 * @brief Stop the counter.
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, negative error code on failure.
 */
static int32_t counter_mchp_stop(const struct device *const dev)
{
	const counter_mchp_dev_cfg_t *const cfg = dev->config;

	rtc_counter_stop(cfg->regs, cfg->max_bit_width);

	return 0;
}

/*
 * @brief Get the current counter value.
 *
 * @param dev Pointer to the device structure.
 * @param ticks Pointer to store the current counter value.
 * @return 0 on success, negative error code on failure.
 */
static int32_t counter_mchp_get_value(const struct device *const dev, uint32_t *const ticks)
{
	const counter_mchp_dev_cfg_t *const cfg = dev->config;

	return rtc_counter_get_count(cfg->regs, ticks, cfg->max_bit_width);
}

/*
 * @brief Set an alarm.
 *
 * @param dev Pointer to the device structure.
 * @param chan_id Channel ID for the alarm.
 * @param alarm_cfg Pointer to the alarm configuration structure.
 * @return 0 on success, negative error code on failure.
 */
static int32_t counter_mchp_set_alarm(const struct device *const dev, const uint8_t chan_id,
				      const struct counter_alarm_cfg *const alarm_cfg)
{
	bool irq_on_late;
	uint32_t absolute;
	int32_t ret_status = 0;
	uint32_t count_value = 0u;
	uint32_t furthest_count_value = 0u;
	int32_t count_diff = 0u;
	uint32_t top_value = 0u;
	uint32_t ticks = alarm_cfg->ticks;

	counter_mchp_dev_data_t *const data = dev->data;
	const counter_mchp_dev_cfg_t *const cfg = dev->config;

	/* Check for valid channel  */
	__ASSERT(chan_id < counter_get_num_of_channels(dev), "Invalid channel ID: %u (max %u)",
		 chan_id, counter_get_num_of_channels(dev));

	/* Get top value */
	rtc_counter_get_period(cfg->regs, &top_value, cfg->max_bit_width);
	__ASSERT_NO_MSG(data->guard_period < top_value);

	/* Check if the requested tick value is less than top (period) value */
	if (ticks > top_value) {
		return -EINVAL;
	}

	if (NULL != data->channel_data[chan_id].callback) {
		return -EBUSY;
	}

	/* First take care of a risk of an event coming from CC being set to
	 * next tick. Reconfigure CC to future ( current counter value minus guard period
	 * is the furthestfuture).
	 */
	(void)rtc_counter_get_count(cfg->regs, &count_value, cfg->max_bit_width);
	furthest_count_value = rtc_counter_ticks_sub(count_value, data->guard_period, top_value);

	rtc_counter_set_compare(cfg->regs, chan_id, furthest_count_value, cfg->max_bit_width);
	rtc_counter_alarm_irq_clear(cfg->regs, chan_id, cfg->max_bit_width);

	/* Update new callback functions */
	data->channel_data[chan_id].callback = alarm_cfg->callback;
	data->channel_data[chan_id].user_data = alarm_cfg->user_data;

	/* Check if "Absolute Alarm" flag is set */
	absolute = alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE;

	/* Check if counter has exceeded the alarm count in absolute alarm configuration */
	if (absolute != 0) {
		count_diff = rtc_counter_ticks_diff(count_value, ticks, top_value);
		if (count_diff <= data->guard_period) {
			ret_status = -ETIME;
			irq_on_late = alarm_cfg->flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE;
			if (irq_on_late != 0) {
				data->late_alarm_flag = true;
				data->late_alarm_channel = chan_id;

				/* Update compare value*/
				data->channel_data[chan_id].compare_value = ticks;

				/* Enable interrupt and trigger immediately */
				NVIC_SetPendingIRQ(cfg->irq_line);
			} else {
				data->channel_data[chan_id].callback = NULL;
				data->channel_data[chan_id].user_data = NULL;
			}
		} else {
			/* Enable interrupt at compare match */
			rtc_counter_alarm_irq_enable(cfg->regs, chan_id, cfg->max_bit_width);

			/* Update compare value*/
			data->channel_data[chan_id].compare_value = ticks;
			rtc_counter_set_compare(cfg->regs, chan_id, ticks, cfg->max_bit_width);
		}
	} else {
		ticks = rtc_counter_ticks_add(count_value, ticks, top_value);

		/* Enable interrupt at compare match */
		rtc_counter_alarm_irq_enable(cfg->regs, chan_id, cfg->max_bit_width);

		/* Update compare value*/
		data->channel_data[chan_id].compare_value = ticks;
		rtc_counter_set_compare(cfg->regs, chan_id, ticks, cfg->max_bit_width);
	}

	return ret_status;
}

/*
 * @brief Cancel an alarm.
 *
 * @param dev Pointer to the device structure.
 * @param chan_id Channel ID for the alarm to be canceled.
 * @return 0 on success, negative error code on failure.
 */
static int32_t counter_mchp_cancel_alarm(const struct device *const dev, uint8_t chan_id)
{
	counter_mchp_dev_data_t *const data = dev->data;
	const counter_mchp_dev_cfg_t *const cfg = dev->config;

	/* Check for valid channel  */
	__ASSERT(chan_id < counter_get_num_of_channels(dev), "Invalid channel ID: %u (max %u)",
		 chan_id, counter_get_num_of_channels(dev));

	/* Clear, and disable interrupt flags */
	rtc_counter_alarm_irq_disable(cfg->regs, chan_id, cfg->max_bit_width);
	rtc_counter_alarm_irq_clear(cfg->regs, chan_id, cfg->max_bit_width);

	/* Set the callback function to NULL */
	data->channel_data[chan_id].callback = NULL;

	return 0;
}

/*
 * @brief Set the top value of the counter.
 *
 * @param dev Pointer to the device structure.
 * @param top_cfg Pointer to the top value configuration structure.
 * @return 0 on success, negative error code on failure.
 */
static int32_t counter_mchp_set_top_value(const struct device *const dev,
					  const struct counter_top_cfg *const top_cfg)
{
	int32_t ret_status = 0;
	counter_mchp_dev_data_t *const data = dev->data;
	const counter_mchp_dev_cfg_t *const cfg = dev->config;

	/* Check if alarm callback function is already in progress */
	for (uint8_t i = 0; i < counter_get_num_of_channels(dev); i++) {
		if (NULL != data->channel_data[i].callback) {
			return -EBUSY;
		}
	}

	/* Check if top callback function is already in progress */
	rtc_counter_top_irq_disable(cfg->regs, cfg->max_bit_width);
	rtc_counter_top_irq_clear(cfg->regs, cfg->max_bit_width);

	/* Update callback functions */
	data->top_cb = top_cfg->callback;
	data->top_user_data = top_cfg->user_data;

	/* Update the counter period based on top configuration data */
	rtc_counter_set_period(cfg->regs, top_cfg->ticks, cfg->max_bit_width);

	if (top_cfg->flags & COUNTER_TOP_CFG_DONT_RESET) {
		/*
		 * Top trigger is on equality of the rising edge only, so
		 * manually reset it if the counter has missed the new top.
		 */
		uint32_t count_value = 0;
		(void)rtc_counter_get_count(cfg->regs, &count_value, cfg->max_bit_width);
		if (count_value >= top_cfg->ticks) {
			ret_status = -ETIME;
			if ((top_cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) ==
			    COUNTER_TOP_CFG_RESET_WHEN_LATE) {
				rtc_counter_retrigger(cfg->regs, cfg->max_bit_width);
			}
		}
	} else {
		rtc_counter_retrigger(cfg->regs, cfg->max_bit_width);
	}

	/* Enable top IRQ */
	if (NULL != top_cfg->callback) {
		rtc_counter_top_irq_enable(cfg->regs, cfg->max_bit_width);
	}

	return ret_status;
}

/*
 * @brief Get the pending interrupt status.
 *
 * @param dev Pointer to the device structure.
 * @return Pending interrupt status.
 */
static uint32_t counter_mchp_get_pending_int(const struct device *const dev)
{
	const counter_mchp_dev_cfg_t *const cfg = dev->config;

	rtc_counter_get_pending_irqs(cfg->regs, cfg->max_bit_width);

	return 0;
}

/*
 * @brief Get the top value of the counter.
 *
 * @param dev Pointer to the device structure.
 * @return Top value of the counter.
 */
static uint32_t counter_mchp_get_top_value(const struct device *const dev)
{
	uint32_t period_value = 0u;
	const counter_mchp_dev_cfg_t *const cfg = dev->config;

	rtc_counter_get_period(cfg->regs, &period_value, cfg->max_bit_width);

	return period_value;
}

/*
 * @brief Gets the current guard period value for the counter device.
 *
 * This function retrieves the guard period value from the device's data structure.
 *
 * @param dev Pointer to the device structure.
 * @param flags Flags for configuration (currently not used in this function).
 *
 * @return The current guard period value for the counter device.
 */
static uint32_t counter_mchp_get_guard_period(const struct device *dev, uint32_t flags)
{
	counter_mchp_dev_data_t *const data = dev->data;

	return data->guard_period;
}

/*
 * @brief Sets the guard period value for the counter device.
 *
 * This function sets the guard period for the counter device. If the provided
 * guard period exceeds the maximum allowable value (top value), the function
 * will return an error.
 *
 * @param dev Pointer to the device structure.
 * @param guard The guard period value to be set.
 * @param flags Flags for configuration (currently not used in this function).
 *
 * @return 0 if the guard period is successfully set, -EINVAL if the guard period
 *		   exceeds the maximum allowable value.
 */
static int32_t counter_mchp_set_guard_period(const struct device *dev, uint32_t guard,
					     uint32_t flags)
{
	uint32_t period_value = 0u;
	int32_t ret_status = -EINVAL;
	counter_mchp_dev_data_t *const data = dev->data;
	const counter_mchp_dev_cfg_t *const cfg = dev->config;

	/* Get period value */
	rtc_counter_get_period(cfg->regs, &period_value, cfg->max_bit_width);
	if (guard < period_value) {
		data->guard_period = guard;
		ret_status = 0;
	}

	return ret_status;
}

/*
 * @brief Get the frequency based on the source clock rate.
 *
 * This function retrieves the source clock frequency and updates the provided
 * frequency pointer with the calculated frequency based on the device's prescaler.
 *
 * @param[in] dev Pointer to the device structure.
 * @return Frequency
 */
static uint32_t counter_mchp_get_frequency(const struct device *const dev)
{
	uint32_t source_clk_freq = 0u;
	const counter_mchp_dev_cfg_t *const cfg = dev->config;
	counter_mchp_clock_t *const clk = (counter_mchp_clock_t *const)&cfg->counter_clock;

	/* Get source clock frequency */
	clock_control_get_rate(clk->clock_dev, clk->periph_async_clk, &source_clk_freq);

	/* Update the info field for clock frequency based on clock rate */
	source_clk_freq = source_clk_freq / cfg->prescaler;

	return source_clk_freq;
}

/*
 * @brief Initialize the counter device.
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, negative error code on failure.
 */
static int32_t counter_mchp_init(const struct device *const dev)
{
	int32_t ret_status = 0;
	const counter_mchp_dev_cfg_t *const cfg = dev->config;
	counter_mchp_clock_t *const clk = (counter_mchp_clock_t *const)&cfg->counter_clock;
	uint32_t max_counter_val = 0u;

	max_counter_val = (uint32_t)((1ULL << cfg->max_bit_width) - 1u);
	if (max_counter_val != cfg->info.max_top_value) {
		LOG_ERR("%s : Maximum bit width not allowed", __func__);
		ret_status = COUNTER_RET_FAILED;
	}

	/* Enable host synchronous core clock */
	ret_status = clock_control_on(clk->clock_dev, clk->host_core_sync_clk);
	if ((ret_status < 0) && (ret_status != -EALREADY)) {
		LOG_ERR("%s : Unable to initialize host clock", __func__);
		ret_status = COUNTER_RET_FAILED;
	}

	/* Enable peripheral asynchronous clock */
	if (ret_status != COUNTER_RET_FAILED) {
		ret_status = clock_control_on(clk->clock_dev, clk->periph_async_clk);
		if ((ret_status < 0) && (ret_status != -EALREADY)) {
			LOG_ERR("%s : Unable to initialize peripheral clock", __func__);
			ret_status = COUNTER_RET_FAILED;
		}
	}

	/* Initialize counter peripheral */
	if (ret_status != COUNTER_RET_FAILED) {
		ret_status = rtc_counter_init(cfg->regs, cfg->prescaler, cfg->max_bit_width);
		if (ret_status < 0) {
			LOG_ERR("%s : Counter failed to initialize", __func__);
			ret_status = COUNTER_RET_FAILED;
		} else {
			/* Initialize interrupt */
			cfg->irq_config_func(dev);
		}
	}

	return ret_status;
}

/*
 * @brief Alarm interrupt handler.
 *
 * @param dev Pointer to the device structure.
 */
static void counter_mchp_alarm_irq_handler(const struct device *const dev)
{
	uint32_t cc_value = 0u;
	uint32_t alarm_status = 0u;
	uint32_t pending_irq_status = 0u;
	counter_mchp_dev_data_t *const data = dev->data;
	const counter_mchp_dev_cfg_t *const cfg = dev->config;

	/* Clear pending interrupt  */
	NVIC_ClearPendingIRQ(cfg->irq_line);

	/* Get pending IRQs  */
	pending_irq_status = rtc_counter_get_pending_irqs(cfg->regs, cfg->max_bit_width);

	/* Check for immediate interrupt trigger */
	if (1u == data->late_alarm_flag) {
		data->late_alarm_flag = false;
		counter_alarm_callback_t cb = NULL;

		cb = data->channel_data[data->late_alarm_channel].callback;
		if (cb != NULL) {
			data->channel_data[data->late_alarm_channel].callback = NULL;
			cc_value = data->channel_data[data->late_alarm_channel].compare_value;

			/* Execute callback function  */
			cb(dev, data->late_alarm_channel, cc_value,
			   data->channel_data[data->late_alarm_channel].user_data);
		}
		return;
	}

	for (uint8_t chan_id = 0; chan_id < counter_get_num_of_channels(dev); chan_id++) {
		alarm_status = rtc_counter_alarm_irq_status(pending_irq_status, chan_id,
							    cfg->max_bit_width);
		if (false != alarm_status) {
			/* Clear NVIC Flag to avoid retrigger */
			rtc_counter_alarm_irq_clear(cfg->regs, chan_id, cfg->max_bit_width);

			if (NULL != data->channel_data[chan_id].callback) {
				counter_alarm_callback_t cb = data->channel_data[chan_id].callback;

				data->channel_data[chan_id].callback = NULL;

				/* Execute callback function  */
				cc_value = data->channel_data[chan_id].compare_value;
				cb(dev, chan_id, cc_value, data->channel_data[chan_id].user_data);
			}
		}
	}
}

/*
 * @brief Top interrupt handler.
 *
 * @param dev Pointer to the device structure.
 */
static void counter_mchp_top_irq_handler(const struct device *const dev)
{
	counter_mchp_dev_data_t *const data = dev->data;
	const counter_mchp_dev_cfg_t *const cfg = dev->config;
	uint32_t pending_irq_status = rtc_counter_get_pending_irqs(cfg->regs, cfg->max_bit_width);

	/* Clear pending IRQ Flags */
	NVIC_ClearPendingIRQ(cfg->irq_line);

	if (false != rtc_counter_top_irq_status(pending_irq_status, cfg->max_bit_width)) {
		rtc_counter_top_irq_clear(cfg->regs, cfg->max_bit_width);

		if (NULL != data->top_cb) {
			data->top_cb(dev, data->top_user_data);
		}
	}
}

/*
 * @brief Counter interrupt service routine.
 *
 * @param dev Pointer to the device structure.
 */
static void counter_mchp_interrupt_handler(const struct device *const dev)
{
	/* Alarm handler */
	counter_mchp_alarm_irq_handler(dev);

	/* Top handler */
	counter_mchp_top_irq_handler(dev);
}

/*
 * @brief Counter driver API structure.
 *
 * This structure contains pointers to the functions that implement the
 * counter driver API.
 */
static DEVICE_API(counter, counter_mchp_api) = {
	.start = counter_mchp_start,
	.stop = counter_mchp_stop,
	.get_freq = counter_mchp_get_frequency,
	.get_value = counter_mchp_get_value,
	.set_alarm = counter_mchp_set_alarm,
	.cancel_alarm = counter_mchp_cancel_alarm,
	.set_top_value = counter_mchp_set_top_value,
	.get_pending_int = counter_mchp_get_pending_int,
	.get_top_value = counter_mchp_get_top_value,
	.get_guard_period = counter_mchp_get_guard_period,
	.set_guard_period = counter_mchp_set_guard_period,
};

/*
 * @brief Macro to determine the number of compare/capture channels
 * for Microchip RTC g1.
 *
 * This macro evaluates the `max_bit_width` property of the device instance
 * and returns 1 if the bit width is 32, otherwise it returns 4.
 *
 * @param n Instance number (not used in this macro).
 */
/* clang-format off */
#define COUNTER_MCHP_CC_NUMS(n) \
	((DT_INST_PROP(n, max_bit_width) == 32) ? 1 : 4)
/* clang-format on */

/*
 * @brief Retrieve the maximum bit width for the RTC counter.
 *
 * This macro retrieves the maximum bit width for an RTC (Real-Time Counter) instance
 * from the device tree. The value is determined by the `max_bit_width` property of the
 * corresponding device tree node.
 *
 * @param n	 Counter instance number (not used in this macro).
 *
 * @return The maximum bit width as specified in the device tree.
 *
 * @note This macro directly uses the `DT_INST_PROP` macro to fetch the `max_bit_width`
 *		 property for the given instance.
 */
/* clang-format off */
#define COUNTER_MCHP_MAX_BIT_WIDTH(n) (DT_INST_PROP(n, max_bit_width))
/* clang-format on */

/*
 * @brief Retrieve the prescaler value for a Microchip counter device.
 *
 * This macro retrieves the prescaler value for the specified instance of a Microchip
 * counter device. If the device tree node for the instance has a `prescaler` property,
 * the macro uses its value. Otherwise, it defaults to a prescaler value of 1.
 *
 * @param n Instance number of the device.
 *
 * @return The prescaler value from the device tree or the default value (1).
 *
 * @note This macro relies on the `DT_INST_NODE_HAS_PROP` and `DT_INST_PROP` macros
 *		 to check for and retrieve the `prescaler` property from the device tree.
 */
/* clang-format off */
#define COUNTER_MCHP_PRESCALER(n)	 \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, prescaler),	\
				(DT_INST_PROP(n, prescaler)), (1))
/* clang-format on */

/*
 * @brief Assign clock configurations for the Counter peripheral.
 *
 * This macro sets up the clock configurations for the Counter peripheral, including
 * host core synchronous clock, client core synchronous clock (if applicable), and
 * peripheral asynchronous clock. It uses device tree properties to determine the
 * appropriate clock sources and configurations.
 *
 * @param n Counter instance number.
 *
 * @note This macro conditionally includes client core synchronous clock handling
 *		 and peripheral asynchronous clock configurations based on the presence
 *		 of relevant device tree properties.
 */
/* clang-format off */
#define COUNTER_MCHP_CLOCK_ASSIGN(n)  \
	.counter_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),	\
	.counter_clock.host_core_sync_clk =	 \
		(void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, subsystem)),	\
	\
	COND_CODE_1(  \
		DT_NODE_HAS_PROP(DT_DRV_INST(n), client),  \
		(.counter_clock.client_core_sync_clk =	\
			(void *)DT_CLOCKS_CELL_BY_NAME(	 \
				DT_PHANDLE(DT_DRV_INST(n), client), mclk, subsystem),),	 \
		()	\
	)  \
   \
	COND_CODE_1(  \
		DT_NODE_EXISTS(DT_INST_CLOCKS_CTLR_BY_NAME(n, rtcclk)),	 \
		(.counter_clock.periph_async_clk =	\
			(void *)DT_INST_CLOCKS_CELL_BY_NAME(n, rtcclk, subsystem),),  \
		()	\
	)
/* clang-format on */

/*
 * @brief Define the configuration structure for a Microchip counter device.
 *
 * This macro defines a static configuration structure for the specified instance
 * of a Microchip counter device. The structure includes:
 * - Device information (e.g., maximum top value, frequency, flags, and channels).
 * - Clock assignments.
 * - IRQ mapping.
 * - Maximum bit width.
 * - Prescaler value.
 * - IRQ configuration function.
 * - Pin control configuration.
 *
 * @param n Instance number of the device.
 *
 * @note This macro relies on several other macros to retrieve device-specific
 *		 properties and configurations, including,
 *		 `COUNTER_MCHP_CLOCK_ASSIGN`, and `COUNTER_MCHP_MAX_BIT_WIDTH`.
 */
/* clang-format off */
#define COUNTER_MCHP_CONFIG_VAR(n)	  \
	static const counter_mchp_dev_cfg_t counter_mchp_dev_config_##n = {	   \
		.info = {	\
			.max_top_value =	\
				((uint32_t)((1ULL << COUNTER_MCHP_MAX_BIT_WIDTH(n)) -  1)),	  \
			.freq = 0u,	\
			.flags = COUNTER_CONFIG_INFO_COUNT_UP,	\
			.channels = COUNTER_MCHP_CC_NUMS(n) },	\
		.regs = (void *)DT_INST_REG_ADDR(n),	\
		COUNTER_MCHP_CLOCK_ASSIGN(n)	\
		.irq_line =	DT_INST_IRQ_BY_IDX(n, 0, irq),	\
		.max_bit_width = DT_INST_PROP(n, max_bit_width),	\
		.prescaler = COUNTER_MCHP_PRESCALER(n),	\
		.irq_config_func = &counter_mchp_config_##n,	\
	};
/* clang-format on */

/*
 * @brief Configure the IRQ handler for the Microchip TC U2249 counter.
 *
 * This macro sets up the IRQ handler for the specified instance of the
 * Microchip TC U2249 counter. It connects the first IRQ line (index 0)
 * for the counter peripheral.
 *
 * @param n Instance number of the counter.
 *
 * @note This macro assumes that the TC U2249 counter has a single IRQ line.
 */
/* clang-format off */
#define COUNTER_MCHP_IRQ_HANDLER(n)	\
	static void counter_mchp_config_##n(const struct device *dev)	 \
	{	 \
		MCHP_COUNTER_IRQ_CONNECT(n, 0);	\
	}
/* clang-format on */

/*
 * @brief Define the channel data structure for a Microchip counter device.
 *
 * This macro creates a static array of channel data structures for the specified
 * instance of a Microchip counter device. The size of the array is determined
 * by the number of compare/capture channels (`CC_NUMS`) for the device instance.
 *
 * @param n Instance number of the device.
 */
/* clang-format off */
#define COUNTER_MCHP_CHANNEL_DATA_VAR(n)	\
	static counter_mchp_ch_data_t	 \
		counter_mchp_channel_data_##n[COUNTER_MCHP_CC_NUMS(n)];
/* clang-format on */

/*
 * @brief Define the device data structure for a Microchip counter device.
 *
 * This macro creates a static device data structure for the specified instance
 * of a Microchip counter device. The structure includes a reference to the
 * channel data array, which stores runtime information for each channel of the
 * counter device.
 *
 * @param n Instance number of the device.
 *
 * @note The `channel_data` field in the device data structure is initialized
 *		 using the `COUNTER_MCHP_CHANNEL_DATA_VAR` macro, which defines the
 *		 channel data array for the device instance.
 */
/* clang-format off */
#define COUNTER_MCHP_TOP_DATA_VAR(n)	\
	static counter_mchp_dev_data_t counter_mchp_dev_data_##n = {	\
		.channel_data = counter_mchp_channel_data_##n};
/* clang-format on */

/*
 * @brief Connect and enable an IRQ for a Microchip counter device.
 *
 * This macro connects an IRQ handler to the specified IRQ line for a Microchip
 * counter device instance and enables the IRQ. The IRQ number and priority are
 * retrieved from the device tree.
 *
 * @param n Instance number of the device.
 * @param m Index of the IRQ line in the device tree.
 */
/* clang-format off */
#define MCHP_COUNTER_IRQ_CONNECT(n, m)	\
	COND_CODE_1(DT_IRQ_HAS_IDX(DT_DRV_INST(n), m),	\
	(	\
		do {	\
			IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, m, irq),	\
				DT_INST_IRQ_BY_IDX(n, m, priority),	\
				counter_mchp_interrupt_handler, DEVICE_DT_INST_GET(n), 0);	\
			irq_enable(DT_INST_IRQ_BY_IDX(n, m, irq));	\
		} while (false);	\
	),	\
	()	\
	)
/* clang-format on */

/*
 * @brief Define the initialization function for a Microchip counter device.
 *
 * This macro generates a static initialization function for a Microchip counter device.
 * The function performs the following tasks:
 * - Defines the IRQ handler for the device instance.
 * - Configures the device instance.
 * - Defines channel data and top-level data structures for the device instance.
 *
 * @param n Instance number of the device.
 */
/* clang-format off */
#define COUNTER_MCHP_DEVICE_INIT_FUNC(n)	\
	COUNTER_MCHP_IRQ_HANDLER(n)	\
	COUNTER_MCHP_CONFIG_VAR(n);	   \
	COUNTER_MCHP_CHANNEL_DATA_VAR(n)	\
	COUNTER_MCHP_TOP_DATA_VAR(n)
/* clang-format on */

/*
 * @brief Initialize a Microchip counter device.
 *
 * This macro generates the initialization function and device structure for a
 * Microchip counter device instance. It uses the `COUNTER_MCHP_DEVICE_INIT_FUNC`
 * macro to define the initialization function and registers the device with the
 * Zephyr device model.
 *
 * @param n Instance number of the device.
 */

#define COUNTER_MCHP_DEVICE_INIT(n)                                                                \
	COUNTER_MCHP_DEVICE_INIT_FUNC(n)                                                           \
	DEVICE_DT_INST_DEFINE(n, counter_mchp_init, NULL, &counter_mchp_dev_data_##n,              \
			      &counter_mchp_dev_config_##n, POST_KERNEL,                           \
			      CONFIG_COUNTER_INIT_PRIORITY, &counter_mchp_api);

/*
 * @brief Initialize Microchip RTC g1 counter devices.
 *
 * This block checks for the presence of RTC g1 counter devices in the device tree
 * and initializes them using the `COUNTER_MCHP_DEVICE_INIT` macro.
 */
DT_INST_FOREACH_STATUS_OKAY(COUNTER_MCHP_DEVICE_INIT)
