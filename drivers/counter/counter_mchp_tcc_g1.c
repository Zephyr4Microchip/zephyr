/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * @file counter_mchp_tcc_g1.c
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
#define DT_DRV_COMPAT microchip_tcc_g1_counter

/*
 * @section Macros
 * User-specific macros for the Microchip counter driver.
 */
LOG_MODULE_REGISTER(counter_mchp_tcc_g1, CONFIG_COUNTER_LOG_LEVEL);

/* All timer/counter synchronization bits set. */
#define ALL_TCC_SYNC_BITS ((uint32_t)UINT32_MAX)

/* Return value for failed operation. */
#define COUNTER_RET_FAILED (-1)

/* Return value for successful operation. */
#define COUNTER_RET_PASSED (0)

/* @brief Synchronization time-out in us */
#define TCC_SYNCHRONIZATION_TIMEOUT_IN_US (5)

/* Delay time in us */
#define DELAY_US (1)

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

/*
 * @section:  Datatypes
 *
 * This section includes necessary user defined data-types used to implement
 * the counter driver for microchip devices
 */

typedef struct tcc_counter_irq_map {
	/* Overflow interrupt number */
	const uint32_t ovf_irq_line;

	/* Channel interrupt number */
	const uint32_t comp_irq_line[6u];

} tcc_counter_irq_map_t;

/*
 * @brief Structure to hold channel-specific data for the counter.
 *
 * This structure holds the callback function and user data for a specific channel of the counter.
 */
typedef struct counter_mchp_ch_data {
	/* Callback function for the channel alarm */
	counter_alarm_callback_t callback;

	/* Compare value */
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

	/* Late alarm flag */
	bool late_alarm_flag;

	/* Late alarm channel */
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

	/* Register address  */
	tcc_registers_t *regs;

	/* Clock configuration */
	counter_mchp_clock_t counter_clock;

	/* Channel IRQ map */
	tcc_counter_irq_map_t *channel_irq_map;

	/* Maximum bit-width */
	uint32_t max_bit_width;

	/* Prescaler value */
	uint16_t prescaler;

	/* IRQ configuration function */
	void (*irq_config_func)(const struct device *dev);

} counter_mchp_dev_cfg_t;

/* Enumeration for counter reset modes */
typedef enum {
	/* Reload or reset the counter on the next generic clock */
	TCC_GCLK_RESET_ON_GENERIC_CLOCK = 0x0,

	/* Reload or reset the counter on the next prescaler clock */
	TCC_PRESC_RESET_ON_PRESCALER_CLOCK = 0x1,

	/* Reload or reset the counter on the next generic clock, and reset the prescaler
	 * counter
	 */
	TCC_RESYNC_RESET_ON_GENERIC_CLOCK = 0x2,

} tcc_counter_prescaler_sync_mode_t;

/* Enumeration for event control modes */
typedef enum {
	/* Event action disabled. */
	TCC_EVENT_CONTROL_MODE_OFF = 0x0,

	/* Count on event. */
	TCC_EVENT_CONTROL_MODE_COUNT = 0x2,
} tcc_counter_evt_control_mode_t;

/* Maximum number of supported TCC channels */
#define TCC_CHANNEL_COUNT 6

/* find the prescale index based on the value present in the device tree */
uint8_t get_tcc_prescale_index(uint16_t prescaler)
{
	uint8_t prescale_index = 0u;

	switch (prescaler) {
	case 64:
		prescale_index = TCC_CTRLA_PRESCALER_DIV64_Val;
		break;
	case 256:
		prescale_index = TCC_CTRLA_PRESCALER_DIV256_Val;
		break;
	case 1024:
		prescale_index = TCC_CTRLA_PRESCALER_DIV1024_Val;
		break;
	default:
		if ((prescaler >= 1) && (prescaler <= 16)) {
			prescale_index = __builtin_ctz(prescaler);
		}
		break;
	}
	return prescale_index;
}
/*
 * @brief Waits for synchronization of a register.
 *
 * Waits until the specified bit(s) in the SYNCBUSY register are cleared,
 * indicating that synchronization is complete.
 *
 * @param[in] sync_reg_addr Pointer to the SYNCBUSY register.
 * @param[in] bit_mask Bit mask for the bit(s) to monitor.
 */
static void tcc_counter_wait_sync(const volatile uint32_t *sync_reg_addr, uint32_t bit_mask)
{
	bool success = WAIT_FOR(((*sync_reg_addr) & bit_mask) == 0u,
				TCC_SYNCHRONIZATION_TIMEOUT_IN_US, k_busy_wait(DELAY_US));

	if (!success) {
		LOG_ERR("%s : Synchronization time-out occurred", __func__);
	}
}

/*
 * @brief Initializes the counter peripheral with specified settings.
 *
 * Configures the counter peripheral with the provided prescaler and bit width.
 * Disables capture, on-demand, standby, one-shot, and output events/interrupts.
 * Sets 32-bit mode, counting up, and waveform generation to MFRQ.
 *
 * @param[in] regs Pointer to the counter peripheral registers.
 * @param[in] prescaler Prescaler value to set.
 * @param[in] max_bit_width Maximum bit width for the counter.
 *
 * @retval COUNTER_RET_PASSED Initialization successful.
 */
static int32_t tcc_counter_init(tcc_registers_t *const p_regs, uint32_t prescaler,
				const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;
	uint32_t max_counter_value = 0U;

	/* Reset TCC */
	p_regs->TCC_CTRLA |= TCC_CTRLA_SWRST_Msk;
	tcc_counter_wait_sync(&p_regs->TCC_SYNCBUSY, TCC_SYNCBUSY_SWRST_Msk);

	/* Configure counter mode & prescaler */
	p_regs->TCC_CTRLA = TCC_CTRLA_CPTEN0(0U) | TCC_CTRLA_CPTEN1(0U) | TCC_CTRLA_CPTEN2(0U) |
			    TCC_CTRLA_CPTEN3(0U) | TCC_CTRLA_CPTEN4(0U) | TCC_CTRLA_CPTEN5(0U) |
			    TCC_CTRLA_MSYNC(0U) |
			    TCC_CTRLA_PRESCALER(get_tcc_prescale_index(prescaler)) |
			    TCC_CTRLA_PRESCSYNC(TCC_GCLK_RESET_ON_GENERIC_CLOCK) |
			    TCC_CTRLA_RUNSTDBY(0U) | TCC_CTRLA_RESOLUTION(0U);

	/* Configure waveform generation mode */
	p_regs->TCC_WAVE = TCC_WAVE_WAVEGEN_NFRQ;

	/* Configure timer one shot mode & direction */
	p_regs->TCC_CTRLBSET = TCC_CTRLBCLR_ONESHOT(0U) | TCC_CTRLBCLR_DIR(0U);

	/* Configure drive control register */
	p_regs->TCC_DRVCTRL = TCC_DRVCTRL_INVEN(0U);

	/* Set the period register to maximum value */
	max_counter_value = (uint32_t)((uint64_t)1U << max_bit_width) - 1U;

	p_regs->TCC_PER = max_counter_value;

	for (uint8_t index = 0u; index < TCC_CHANNEL_COUNT; index++) {
		p_regs->TCC_CC[index] = max_counter_value;
	}

	/* Clear all interrupt flags */
	p_regs->TCC_INTFLAG = TCC_INTFLAG_Msk;

	/* Event control register */
	p_regs->TCC_EVCTRL = TCC_EVCTRL_EVACT0(0U) | TCC_EVCTRL_EVACT1(0U) | TCC_EVCTRL_TCINV(0U) |
			     TCC_EVCTRL_TCEI(0U) | TCC_EVCTRL_OVFEO(0U) | TCC_EVCTRL_MCEO0(0U) |
			     TCC_EVCTRL_MCEO1(0U) | TCC_EVCTRL_MCEO2(0U) | TCC_EVCTRL_MCEO3(0U) |
			     TCC_EVCTRL_MCEO4(0U) | TCC_EVCTRL_MCEO5(0U);

	tcc_counter_wait_sync(&p_regs->TCC_SYNCBUSY, ALL_TCC_SYNC_BITS);

	return return_value;
}

/*
 * @brief Starts the counter peripheral.
 *
 * Enables the counter and ensures synchronization.
 *
 * @param[in] regs Pointer to the counter peripheral registers.
 *
 * @retval COUNTER_RET_PASSED Counter started successfully.
 */
static int32_t tcc_counter_start(tcc_registers_t *const p_regs)
{
	int32_t return_value = COUNTER_RET_PASSED;

	/* Set enable bit */
	p_regs->TCC_CTRLA |= TCC_CTRLA_ENABLE_Msk;

	/* Wait for synchronization */
	tcc_counter_wait_sync(&p_regs->TCC_SYNCBUSY, TCC_SYNCBUSY_ENABLE_Msk);

	/* Write command to force COUNT register read synchronization */
	p_regs->TCC_CTRLBSET |= TCC_CTRLBSET_CMD_RETRIGGER;

	/* Wait for synchronization */
	tcc_counter_wait_sync(&p_regs->TCC_SYNCBUSY, TCC_SYNCBUSY_CTRLB_Msk);

	/* Wait for CMD to become zero */
	while ((p_regs->TCC_CTRLBSET & TCC_CTRLBSET_CMD_Msk) != 0U) {
	}

	return return_value;
}

/*
 * @brief Stops the counter peripheral.
 *
 * Issues a stop command and waits for synchronization.
 *
 * @param[in] regs Pointer to the counter peripheral registers.
 *
 * @retval COUNTER_RET_PASSED Counter stopped successfully.
 */
static int32_t tcc_counter_stop(tcc_registers_t *const p_regs)
{
	int32_t return_value = COUNTER_RET_PASSED;

	/* Write command to force COUNT register read synchronization */
	p_regs->TCC_CTRLBSET |= TCC_CTRLBSET_CMD_STOP;

	/* Wait for synchronization */
	tcc_counter_wait_sync(&p_regs->TCC_SYNCBUSY, TCC_SYNCBUSY_CTRLB_Msk);

	/* Wait for CMD to become zero */
	while ((p_regs->TCC_CTRLBSET & TCC_CTRLBSET_CMD_Msk) != 0U) {
	}

	return return_value;
}

/*
 * @brief Retriggers the counter peripheral.
 *
 * Issues a retrigger command and waits for synchronization.
 *
 * @param[in] regs Pointer to the counter peripheral registers.
 *
 * @retval COUNTER_RET_PASSED Counter retriggered successfully.
 */
static int32_t tcc_counter_retrigger(tcc_registers_t *const p_regs)
{
	int32_t return_value = COUNTER_RET_PASSED;

	/* Write command to force COUNT register read synchronization */
	p_regs->TCC_CTRLBSET |= TCC_CTRLBSET_CMD_RETRIGGER;

	/* Wait for synchronization */
	tcc_counter_wait_sync(&p_regs->TCC_SYNCBUSY, TCC_SYNCBUSY_CTRLB_Msk);

	/* Wait for CMD to become zero */
	while ((p_regs->TCC_CTRLBSET & TCC_CTRLBSET_CMD_Msk) != 0U) {
	}

	return return_value;
}

/*
 * @brief Gets the current count value of the counter.
 *
 * Reads the current value of the counter register.
 *
 * @param[in] regs Pointer to the counter peripheral registers.
 * @param[out] counter_value Pointer to store the current count value.
 *
 * @retval 0 Success.
 */
static int32_t tcc_counter_get_count(tcc_registers_t *const p_regs, uint32_t *const counter_value)
{
	int32_t ret_status = 0;

	/* Write command to force COUNT register read synchronization */
	p_regs->TCC_CTRLBSET |= TCC_CTRLBSET_CMD_READSYNC;

	/* Wait for synchronization */
	tcc_counter_wait_sync(&p_regs->TCC_SYNCBUSY, TCC_SYNCBUSY_CTRLB_Msk);

	/* Wait for CMD to become zero */
	while ((p_regs->TCC_CTRLBSET & TCC_CTRLBSET_CMD_Msk) != 0U) {
	}

	/* Read current count value */
	*counter_value = p_regs->TCC_COUNT;

	return ret_status;
}

/*
 * @brief Sets the period value of the counter.
 *
 * Updates the period register and waits for synchronization.
 *
 * @param[in] regs Pointer to the counter peripheral registers.
 * @param[in] period Period value to set.
 *
 * @retval COUNTER_RET_PASSED Period set successfully.
 */
static int32_t tcc_counter_set_period(tcc_registers_t *const p_regs, const uint32_t period)
{
	int32_t return_value = COUNTER_RET_PASSED;

	/* Set the period value */
	p_regs->TCC_PER = period;

	/* Wait for synchronization */
	tcc_counter_wait_sync(&p_regs->TCC_SYNCBUSY, TCC_SYNCBUSY_PER_Msk);

	return return_value;
}

/*
 * @brief Gets the current period value of the counter.
 *
 * @param[in] regs Pointer to the counter peripheral registers.
 *
 * @return Current period value.
 */
static uint32_t tcc_counter_get_period(tcc_registers_t *const p_regs)
{
	return p_regs->TCC_PER;
}

/*
 * @brief Sets the compare value for a specific channel.
 *
 * @param[in] regs Pointer to the counter peripheral registers.
 * @param[in] chan_id Channel ID to set the compare value for.
 * @param[in] compare_value Compare value to set.
 *
 * @retval COUNTER_RET_PASSED Compare value set successfully.
 */
static int32_t tcc_counter_set_compare(tcc_registers_t *const p_regs, const uint32_t chan_id,
				       const uint32_t compare_value)
{
	int32_t return_value = COUNTER_RET_PASSED;

	/* Set the period value */
	p_regs->TCC_CC[chan_id] = compare_value;

	/* Wait for synchronization */
	tcc_counter_wait_sync(&p_regs->TCC_SYNCBUSY, ALL_TCC_SYNC_BITS);

	return return_value;
}

/*
 * @brief Gets the pending interrupt flags.
 *
 * @param[in] regs Pointer to the counter peripheral registers.
 *
 * @return Pending interrupt flags.
 */
static int32_t tcc_counter_get_pending_irqs(tcc_registers_t *const p_regs)
{
	/* Check interrupt flags */
	return p_regs->TCC_INTFLAG;
}

/*
 * @brief Enables the alarm interrupt for a specific channel.
 *
 * @param[in] regs Pointer to the counter peripheral registers.
 * @param[in] channel_id Channel ID to enable the alarm interrupt for.
 *
 * @retval COUNTER_RET_PASSED Success.
 * @retval COUNTER_RET_FAILED Invalid channel.
 */
static int32_t tcc_counter_alarm_irq_enable(tcc_registers_t *const p_regs,
					    const uint32_t channel_id)
{
	if (channel_id >= TCC_CHANNEL_COUNT) {
		return COUNTER_RET_FAILED;
	}
	p_regs->TCC_INTENSET = TCC_INTFLAG_MC0_Msk << channel_id;

	return COUNTER_RET_PASSED;
}

/*
 * @brief Disables the alarm interrupt for a specific channel.
 *
 * @param[in] regs Pointer to the counter peripheral registers.
 * @param[in] channel_id Channel ID to disable the alarm interrupt for.
 *
 * @retval COUNTER_RET_PASSED Success.
 * @retval COUNTER_RET_FAILED Invalid channel.
 */
static int32_t tcc_counter_alarm_irq_disable(tcc_registers_t *const p_regs,
					     const uint32_t channel_id)
{
	if (channel_id >= TCC_CHANNEL_COUNT) {
		return COUNTER_RET_FAILED;
	}
	p_regs->TCC_INTENCLR = TCC_INTFLAG_MC0_Msk << channel_id;

	return COUNTER_RET_PASSED;
}

/*
 * @brief Clears the alarm interrupt for a specific channel.
 *
 * @param[in] regs Pointer to the counter peripheral registers.
 * @param[in] channel_id Channel ID to clear the alarm interrupt for.
 *
 * @retval COUNTER_RET_PASSED Success.
 * @retval COUNTER_RET_FAILED Invalid channel.
 */
static int32_t tcc_counter_alarm_irq_clear(tcc_registers_t *const p_regs, const uint32_t channel_id)
{
	if (channel_id >= TCC_CHANNEL_COUNT) {
		return COUNTER_RET_FAILED;
	}
	p_regs->TCC_INTFLAG = TCC_INTFLAG_MC0_Msk << channel_id;

	return COUNTER_RET_PASSED;
}

/*
 * @brief Enables the overflow interrupt.
 *
 * @param[in] regs Pointer to the counter peripheral registers.
 *
 * @retval COUNTER_RET_PASSED Success.
 */
static int32_t tcc_counter_top_irq_enable(tcc_registers_t *const p_regs)
{
	int32_t return_value = COUNTER_RET_PASSED;

	/* Enable overflow interrupt */
	p_regs->TCC_INTENSET = TCC_INTFLAG_OVF_Msk;

	return return_value;
}

/*
 * @brief Disables the overflow interrupt.
 *
 * @param[in] regs Pointer to the counter peripheral registers.
 *
 * @retval COUNTER_RET_PASSED Success.
 */
static int32_t tcc_counter_top_irq_disable(tcc_registers_t *const p_regs)
{
	int32_t return_value = COUNTER_RET_PASSED;

	/* Disable overflow interrupt */
	p_regs->TCC_INTENCLR = TCC_INTFLAG_OVF_Msk;

	return return_value;
}

/*
 * @brief Clears the overflow interrupt.
 *
 * @param[in] regs Pointer to the counter peripheral registers.
 *
 * @retval COUNTER_RET_PASSED Success.
 */
static int32_t tcc_counter_top_irq_clear(tcc_registers_t *const p_regs)
{
	/* Clear overflow interrupt */
	p_regs->TCC_INTFLAG = TCC_INTFLAG_OVF_Msk;

	return COUNTER_RET_PASSED;
}

/*
 * @brief Computes the difference between two tick values, handling wraparound.
 *
 * @param[in] val Newer tick value.
 * @param[in] old Older tick value.
 * @param[in] top Maximum counter value before wraparound.
 *
 * @return Difference (val - old) modulo (top + 1).
 */
static uint32_t tcc_counter_ticks_sub(uint32_t val, uint32_t old, uint32_t top)
{
	if (true == likely(IS_BIT_MASK(top))) {
		return (val - old) & top;
	}

	/* If top is not 2^n - 1, handle general wraparound case */
	return (val >= old) ? (val - old) : val + top + 1U - old;
}

/*
 * @brief Adds two tick values, handling counter wraparound.
 *
 * @param[in] val1 First tick value.
 * @param[in] val2 Second tick value.
 * @param[in] top Maximum counter value before wraparound.
 *
 * @return Sum (val1 + val2) modulo (top + 1).
 */
static uint32_t tcc_counter_ticks_add(uint32_t val1, uint32_t val2, uint32_t top)
{
	uint32_t to_top;

	if (true == likely(IS_BIT_MASK(top))) {
		return (val1 + val2) & top;
	}

	to_top = top - val1;

	return (val2 <= to_top) ? val1 + val2 : val2 - to_top - 1U;
}

/*
 * @brief Computes the absolute difference between two counter values with wraparound.
 *
 * @param[in] a First counter value.
 * @param[in] b Second counter value.
 * @param[in] top Maximum counter value before wraparound.
 *
 * @return Absolute difference in ticks.
 */
static uint32_t tcc_counter_ticks_diff(uint32_t a, uint32_t b, uint32_t top)
{
	uint32_t diff = (a > b) ? (a - b) : (b - a);
	uint32_t wrap_diff = top - diff;

	return (diff < wrap_diff) ? diff : wrap_diff;
}

/*
 * @brief Starts the counter device.
 *
 * @param[in] dev Pointer to the device structure.
 *
 * @retval 0 Success.
 * @retval Negative error code on failure.
 */
static int32_t counter_mchp_start(const struct device *const dev)
{
	const counter_mchp_dev_cfg_t *const cfg = dev->config;

	tcc_counter_start(cfg->regs);

	return 0;
}

/*
 * @brief Stops the counter device.
 *
 * @param[in] dev Pointer to the device structure.
 *
 * @retval 0 Success.
 * @retval Negative error code on failure.
 */
static int32_t counter_mchp_stop(const struct device *const dev)
{
	const counter_mchp_dev_cfg_t *const cfg = dev->config;

	tcc_counter_stop(cfg->regs);

	return 0;
}

/*
 * @brief Gets the current counter value.
 *
 * @param[in] dev Pointer to the device structure.
 * @param[out] ticks Pointer to store the current counter value.
 *
 * @retval 0 Success.
 * @retval Negative error code on failure.
 */
static int32_t counter_mchp_get_value(const struct device *const dev, uint32_t *const ticks)
{
	const counter_mchp_dev_cfg_t *const cfg = dev->config;

	return tcc_counter_get_count(cfg->regs, ticks);
}

/*
 * @brief Sets an alarm for the counter device.
 *
 * @param[in] dev Pointer to the device structure.
 * @param[in] chan_id Channel ID for the alarm.
 * @param[in] alarm_cfg Pointer to the alarm configuration structure.
 *
 * @retval 0 Success.
 * @retval -EINVAL Invalid argument.
 * @retval -EBUSY Channel is busy.
 * @retval -ETIME Alarm is late.
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
	top_value = tcc_counter_get_period(cfg->regs);
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
	(void)tcc_counter_get_count(cfg->regs, &count_value);
	furthest_count_value = tcc_counter_ticks_sub(count_value, data->guard_period, top_value);

	tcc_counter_set_compare(cfg->regs, chan_id, furthest_count_value);
	tcc_counter_alarm_irq_clear(cfg->regs, chan_id);

	/* Update new callback functions */
	data->channel_data[chan_id].callback = alarm_cfg->callback;
	data->channel_data[chan_id].user_data = alarm_cfg->user_data;

	/* Check if "Absolute Alarm" flag is set */
	absolute = alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE;

	/* Check if counter has exceeded the alarm count in absolute alarm configuration */
	if (absolute != 0) {
		count_diff = tcc_counter_ticks_diff(count_value, ticks, top_value);
		if (count_diff <= data->guard_period) {
			ret_status = -ETIME;
			irq_on_late = alarm_cfg->flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE;
			if (irq_on_late != 0) {
				data->late_alarm_flag = true;
				data->late_alarm_channel = chan_id;

				/* Update compare value*/
				data->channel_data[chan_id].compare_value = ticks;

				/* Enable interrupt and trigger immediately */
				NVIC_SetPendingIRQ(cfg->channel_irq_map->comp_irq_line[chan_id]);
			} else {
				data->channel_data[chan_id].callback = NULL;
				data->channel_data[chan_id].user_data = NULL;
			}
		} else {
			/* Update compare value*/
			data->channel_data[chan_id].compare_value = ticks;
			tcc_counter_set_compare(cfg->regs, chan_id, ticks);

			/* Enable interrupt at compare match */
			tcc_counter_alarm_irq_enable(cfg->regs, chan_id);
		}
	} else {
		ticks = tcc_counter_ticks_add(count_value, ticks, top_value);

		/* Update compare value*/
		data->channel_data[chan_id].compare_value = ticks;
		tcc_counter_set_compare(cfg->regs, chan_id, ticks);

		/* Enable interrupt at compare match */
		tcc_counter_alarm_irq_enable(cfg->regs, chan_id);
	}

	return ret_status;
}

/*
 * @brief Cancels an alarm for the specified channel.
 *
 * @param[in] dev Pointer to the device structure.
 * @param[in] chan_id Channel ID for the alarm to be canceled.
 *
 * @retval 0 Success.
 */
static int32_t counter_mchp_cancel_alarm(const struct device *const dev, uint8_t chan_id)
{
	counter_mchp_dev_data_t *const data = dev->data;
	const counter_mchp_dev_cfg_t *const cfg = dev->config;

	/* Check for valid channel  */
	__ASSERT(chan_id < counter_get_num_of_channels(dev), "Invalid channel ID: %u (max %u)",
		 chan_id, counter_get_num_of_channels(dev));

	/* Set the callback function to NULL */
	data->channel_data[chan_id].callback = NULL;

	/* Clear, and disable interrupt flags */
	tcc_counter_alarm_irq_disable(cfg->regs, chan_id);
	tcc_counter_alarm_irq_clear(cfg->regs, chan_id);

	/* Clear NVIC Flag to avoid retrigger */
	NVIC_ClearPendingIRQ(cfg->channel_irq_map->comp_irq_line[chan_id]);

	return 0;
}

/*
 * @brief Cancels an alarm for the specified channel.
 *
 * @param[in] dev Pointer to the device structure.
 * @param[in] chan_id Channel ID for the alarm to be canceled.
 *
 * @retval 0 Success.
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
	tcc_counter_top_irq_disable(cfg->regs);
	tcc_counter_top_irq_clear(cfg->regs);

	/* Update callback functions */
	data->top_cb = top_cfg->callback;
	data->top_user_data = top_cfg->user_data;

	/* Update the counter period based on top configuration data */
	tcc_counter_set_period(cfg->regs, top_cfg->ticks);

	if (top_cfg->flags & COUNTER_TOP_CFG_DONT_RESET) {
		/*
		 * Top trigger is on equality of the rising edge only, so
		 * manually reset it if the counter has missed the new top.
		 */
		uint32_t counter_value = 0u;

		tcc_counter_get_count(cfg->regs, &counter_value);
		if (counter_value >= top_cfg->ticks) {
			ret_status = -ETIME;
			if ((top_cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) ==
			    COUNTER_TOP_CFG_RESET_WHEN_LATE) {
				tcc_counter_retrigger(cfg->regs);
			}
		}
	} else {
		tcc_counter_retrigger(cfg->regs);
	}

	/* Enable top IRQ */
	if (NULL != top_cfg->callback) {
		tcc_counter_top_irq_enable(cfg->regs);
	}

	return ret_status;
}

/*
 * @brief Gets the pending interrupt status.
 *
 * @param[in] dev Pointer to the device structure.
 *
 * @return Pending interrupt status.
 */
static uint32_t counter_mchp_get_pending_int(const struct device *const dev)
{
	const counter_mchp_dev_cfg_t *const cfg = dev->config;

	tcc_counter_get_pending_irqs(cfg->regs);

	return 0;
}

/*
 * @brief Gets the top value of the counter.
 *
 * @param[in] dev Pointer to the device structure.
 *
 * @return Top value of the counter.
 */
static uint32_t counter_mchp_get_top_value(const struct device *const dev)
{
	const counter_mchp_dev_cfg_t *const cfg = dev->config;

	return tcc_counter_get_period(cfg->regs);
}

/*
 * @brief Gets the current guard period value for the counter device.
 *
 * @param[in] dev Pointer to the device structure.
 * @param[in] flags Flags for configuration (currently unused).
 *
 * @return Current guard period value.
 */
static uint32_t counter_mchp_get_guard_period(const struct device *dev, uint32_t flags)
{
	counter_mchp_dev_data_t *const data = dev->data;

	return data->guard_period;
}

/*
 * @brief Sets the guard period value for the counter device.
 *
 * @param[in] dev Pointer to the device structure.
 * @param[in] guard Guard period value to set.
 * @param[in] flags Flags for configuration (currently unused).
 *
 * @retval 0 Success.
 * @retval -EINVAL Guard period exceeds maximum allowable value.
 */
static int32_t counter_mchp_set_guard_period(const struct device *dev, uint32_t guard,
					     uint32_t flags)
{
	counter_mchp_dev_data_t *const data = dev->data;
	const counter_mchp_dev_cfg_t *const cfg = dev->config;

	if (guard > tcc_counter_get_period(cfg->regs)) {
		return -EINVAL;
	}

	data->guard_period = guard;

	return 0;
}

/*
 * @brief Gets the frequency based on the source clock rate.
 *
 * Retrieves the source clock frequency and calculates the counter frequency
 * based on the device's prescaler.
 *
 * @param[in] dev Pointer to the device structure.
 *
 * @return Counter frequency in Hz.
 */
static uint32_t counter_mchp_get_frequency(const struct device *const dev)
{
	uint32_t source_clk_freq = 0u;
	const counter_mchp_dev_cfg_t *const cfg = dev->config;
	counter_mchp_clock_t *const clk = (counter_mchp_clock_t *const)&cfg->counter_clock;

	/* Get the clock source */
	clock_control_get_rate(clk->clock_dev, clk->periph_async_clk, &source_clk_freq);

	/* Update the info field for clock frequency based on clock rate */
	source_clk_freq = source_clk_freq / cfg->prescaler;

	return source_clk_freq;
}

/*
 * @brief Initializes the counter device.
 *
 * Initializes the counter peripheral, sets the top value, and disables all interrupts.
 *
 * @param[in] dev Pointer to the device structure.
 *
 * @retval 0 Success.
 * @retval Negative error code on failure.
 */
static int32_t counter_mchp_init(const struct device *const dev)
{
	int32_t ret_status = 0;
	const counter_mchp_dev_cfg_t *const cfg = dev->config;
	const counter_mchp_clock_t *const clk = &cfg->counter_clock;
	uint32_t max_counter_val = 0u;

	max_counter_val = (uint32_t)((1ULL << cfg->max_bit_width) - 1u);
	if (max_counter_val != cfg->info.max_top_value) {
		LOG_ERR("%s : Maximum bit width not allowed", __func__);
		ret_status = COUNTER_RET_FAILED;
	}

	if (ret_status != COUNTER_RET_FAILED) {
		/* Enable host synchronous core clock */
		ret_status = clock_control_on(clk->clock_dev, clk->host_core_sync_clk);
		if ((ret_status < 0) && (ret_status != -EALREADY)) {
			LOG_ERR("%s : Unable to initialize host clock", __func__);
			ret_status = COUNTER_RET_FAILED;
		}
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
		ret_status = tcc_counter_init(cfg->regs, cfg->prescaler, cfg->max_bit_width);
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
 * @brief Counter overflow interrupt service routine.
 *
 * Handles the overflow interrupt.
 * This is mapped to irq-0 (overflow IRQ).
 *
 * @param[in] dev Pointer to the device structure.
 */
static inline void counter_mchp_irq_0_handle(const struct device *const dev)
{
	counter_mchp_dev_data_t *const data = dev->data;
	const counter_mchp_dev_cfg_t *const cfg = dev->config;

	NVIC_ClearPendingIRQ(cfg->channel_irq_map->ovf_irq_line);
	tcc_counter_top_irq_clear(cfg->regs);

	if (data->top_cb) {
		data->top_cb(dev, data->top_user_data);
	}
}

/*
 * @brief Generic counter channel compare interrupt service routine.
 *
 * Handles the compare (alarm) interrupt for the specified channel.
 *
 * @param[in] dev Pointer to the device structure.
 * @param[in] channel Channel index (0-based)
 */
static inline void counter_mchp_channel_irq_handle(const struct device *const dev, uint8_t channel)
{
	uint32_t cc_value = 0u;
	counter_mchp_dev_data_t *const data = dev->data;
	const counter_mchp_dev_cfg_t *const cfg = dev->config;
	counter_alarm_callback_t cb = data->channel_data[channel].callback;

	/* Immediate interrupt trigger */
	if (data->late_alarm_flag && data->late_alarm_channel == channel) {
		data->late_alarm_flag = false;
	} else {
		tcc_counter_alarm_irq_clear(cfg->regs, channel);
	}

	data->channel_data[channel].callback = NULL;
	cc_value = data->channel_data[channel].compare_value;
	if (cb != NULL) {
		cb(dev, channel, cc_value, data->channel_data[channel].user_data);
	}
}

/*
 * @brief Channel 0 compare interrupt service routine.
 *
 * Wrapper for channel 0 compare (alarm) interrupt.
 * This is mapped to irq-1 (channel 0 IRQ).
 *
 * @param[in] dev Pointer to the device structure.
 */
static inline void counter_mchp_irq_1_handle(const struct device *const dev)
{
	counter_mchp_channel_irq_handle(dev, 0);
}

/*
 * @brief Channel 1 compare interrupt service routine.
 *
 * Wrapper for channel 1 compare (alarm) interrupt.
 * This is mapped to irq-2 (channel 1 IRQ).
 *
 * @param[in] dev Pointer to the device structure.
 */
static inline void counter_mchp_irq_2_handle(const struct device *const dev)
{
	counter_mchp_channel_irq_handle(dev, 1);
}

/*
 * @brief Channel 2 compare interrupt service routine.
 *
 * Wrapper for channel 2 compare (alarm) interrupt.
 * This is mapped to irq-3 (channel 2 IRQ).
 *
 * @param[in] dev Pointer to the device structure.
 */
static inline void counter_mchp_irq_3_handle(const struct device *const dev)
{
	counter_mchp_channel_irq_handle(dev, 2);
}

/*
 * @brief Channel 3 compare interrupt service routine.
 *
 * Wrapper for channel 3 compare (alarm) interrupt.
 * This is mapped to irq-4 (channel 3 IRQ).
 *
 * @param[in] dev Pointer to the device structure.
 */
static inline void counter_mchp_irq_4_handle(const struct device *const dev)
{
	counter_mchp_channel_irq_handle(dev, 3);
}

/*
 * @brief Channel 4 compare interrupt service routine.
 *
 * Wrapper for channel 4 compare (alarm) interrupt.
 * This is mapped to irq-5 (channel 4 IRQ).
 *
 * @param[in] dev Pointer to the device structure.
 */
static inline void counter_mchp_irq_5_handle(const struct device *const dev)
{
	counter_mchp_channel_irq_handle(dev, 4);
}

/*
 * @brief Channel 5 compare interrupt service routine.
 *
 * Wrapper for channel 5 compare (alarm) interrupt.
 * This is mapped to irq-6 (channel 5 IRQ).
 *
 * @param[in] dev Pointer to the device structure.
 */
static inline void counter_mchp_irq_6_handle(const struct device *const dev)
{
	counter_mchp_channel_irq_handle(dev, 5);
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
#define COUNTER_MCHP_CLOCK_ASSIGN(n)	\
	.counter_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),	\
	.counter_clock.host_core_sync_clk =	\
		(void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, subsystem)),	\
	\
	.counter_clock.periph_async_clk =	\
		(void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, subsystem)),
/* clang-format on */

/*
 * @brief Configure the IRQ handler for the Microchip TCC g1 counter.
 *
 * This macro sets up the IRQ handler for the specified instance of the
 * Microchip TCC g1 counter. It connects the appropriate IRQs based on
 * the number of IRQs defined for the device in the device tree.
 *
 * @param n Instance number of the counter.
 *
 * @note The macro iterates through the IRQs defined in the device tree and
 *		 connects them sequentially. It supports up to 6 IRQs for the TCC g1 counter.
 */
/* clang-format off */
#define COUNTER_MCHP_IRQ_HANDLER(n)	\
	static void counter_mchp_config_##n(const struct device *dev)	\
	{	\
		if (DT_NUM_IRQS(DT_DRV_INST(n)) > 0) {	\
			MCHP_COUNTER_IRQ_CONNECT(n, 0);	\
		}	\
		if (DT_NUM_IRQS(DT_DRV_INST(n)) > 1) {	\
			MCHP_COUNTER_IRQ_CONNECT(n, 1);	\
		}	\
		if (DT_NUM_IRQS(DT_DRV_INST(n)) > 2) {	\
			MCHP_COUNTER_IRQ_CONNECT(n, 2);	\
		}	\
		if (DT_NUM_IRQS(DT_DRV_INST(n)) > 3) {	\
			MCHP_COUNTER_IRQ_CONNECT(n, 3);	\
		}	\
		if (DT_NUM_IRQS(DT_DRV_INST(n)) > 4) {	\
			MCHP_COUNTER_IRQ_CONNECT(n, 4);	\
		}	\
		if (DT_NUM_IRQS(DT_DRV_INST(n)) > 5) {	\
			MCHP_COUNTER_IRQ_CONNECT(n, 5);	\
		}	\
		if (DT_NUM_IRQS(DT_DRV_INST(n)) > 6) {	\
			MCHP_COUNTER_IRQ_CONNECT(n, 6);	\
		}	\
	}
/* clang-format on */

/*
 * @brief Macro to determine the number of IRQs for Microchip TCC g1.
 *
 * This macro calculates the number of IRQs for the given instance of the Microchip TCC g1
 * counter by subtracting 1 from the total number of IRQs ( First IRQ is for OVF).
 *
 * @param n Instance number.
 */
/* clang-format off */
#define COUNTER_MCHP_CC_NUMS(n) (DT_NUM_IRQS(DT_DRV_INST(n)) - 1u)
/* clang-format on */

/*
 * @brief Retrieve the maximum bit width for the TCC counter.
 *
 * This macro retrieves the maximum bit width for a TCC (Timer/Counter for Control) instance
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
 * counter device from the device tree. If the device tree node for the instance has a
 * `prescaler` property, the macro uses its value; otherwise, it defaults to 1.
 *
 * @param n Instance number of the device.
 * @return The prescaler value from the device tree, or 1 if the property is not present.
 *
 * @note This macro uses the `DT_INST_PROP_OR` macro to provide a default value if the
 *		 `prescaler` property is not defined in the device tree.
 */
/* clang-format off */
#define COUNTER_MCHP_PRESCALER(n) DT_INST_PROP_OR(n, prescaler, 1)
/* clang-format on */

/*
 * @brief Declare the IRQ mapping structure for a Microchip counter device.
 *
 * This macro declares a static IRQ mapping structure for the specified instance
 * of a Microchip counter device. The structure maps IRQs to channels based on
 * the number of compare/capture channels (`CC_NUMS`) for the device instance.
 *
 * @param n Instance number of the device.
 */
/* clang-format off */
#define COUNTER_MCHP_IRQ_MAP_VAR(n) \
	static tcc_counter_irq_map_t counter_mchp_irq_map_##n = { \
		.ovf_irq_line = DT_INST_IRQ_BY_IDX(n, 0, irq), \
		.comp_irq_line = { \
			COND_CODE_1( \
				DT_INST_IRQ_HAS_IDX(n, 1), \
				(DT_INST_IRQ_BY_IDX(n, 1, irq)), \
				(0) \
			), \
			COND_CODE_1( \
				DT_INST_IRQ_HAS_IDX(n, 2), \
				(DT_INST_IRQ_BY_IDX(n, 2, irq)), \
				(0) \
			), \
			COND_CODE_1( \
			DT_INST_IRQ_HAS_IDX(n, 3), \
				(DT_INST_IRQ_BY_IDX(n, 3, irq)), \
				(0) \
			), \
			COND_CODE_1( \
				DT_INST_IRQ_HAS_IDX(n, 4), \
				(DT_INST_IRQ_BY_IDX(n, 4, irq)), \
				(0) \
			), \
			COND_CODE_1( \
				DT_INST_IRQ_HAS_IDX(n, 5), \
				(DT_INST_IRQ_BY_IDX(n, 5, irq)), \
				(0) \
			), \
			COND_CODE_1( \
				DT_INST_IRQ_HAS_IDX(n, 6), \
				(DT_INST_IRQ_BY_IDX(n, 6, irq)), \
				(0) \
			) \
		} \
	}
/* clang-format on */

/*
 * @brief Define the configuration structure for a Microchip counter device.
 *
 * This macro defines a static configuration structure for the specified instance
 * of a Microchip counter device. The structure includes:
 * - Device information (e.g., maximum top value, frequency, flags, and channels).
 * - Register assignments.
 * - Clock assignments.
 * - Register interfaces.
 * - IRQ mapping.
 * - Maximum bit width.
 * - Prescaler value.
 * - IRQ configuration function.
 * - Pin control configuration.
 *
 * @param n Instance number of the device.
 *
 * @note This macro relies on several other macros to retrieve device-specific
 *		 properties and configurations, `COUNTER_MCHP_CLOCK_ASSIGN`,
 *		 and `COUNTER_MCHP_MAX_BIT_WIDTH`.
 */
/* clang-format off */
#define COUNTER_MCHP_CONFIG_VAR(n)	\
	static const counter_mchp_dev_cfg_t counter_mchp_dev_config_##n = {	\
		.info = {.max_top_value =	\
				 ((uint32_t)((1ULL << COUNTER_MCHP_MAX_BIT_WIDTH(n)) - 1)),	\
			 .freq = 0u,	\
			 .flags = COUNTER_CONFIG_INFO_COUNT_UP,	\
			 .channels = COUNTER_MCHP_CC_NUMS(n)},	\
		.regs = (tcc_registers_t *)DT_INST_REG_ADDR(n),	\
		COUNTER_MCHP_CLOCK_ASSIGN(n)	\
		.channel_irq_map = &counter_mchp_irq_map_##n,	\
		.max_bit_width = COUNTER_MCHP_MAX_BIT_WIDTH(n),	\
		.prescaler = COUNTER_MCHP_PRESCALER(n),	\
		.irq_config_func = &counter_mchp_config_##n,	\
	};
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
	static counter_mchp_ch_data_t counter_mchp_channel_data_##n[COUNTER_MCHP_CC_NUMS(n)];
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
				DT_INST_IRQ_BY_IDX(n, m, priority),		\
				counter_mchp_irq_##m##_handle, DEVICE_DT_INST_GET(n), 0);	\
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
	/* Map IRQs for the device instance. */	\
	COUNTER_MCHP_IRQ_MAP_VAR(n);	\
	\
	/* Function for IRQ Handler */	\
	COUNTER_MCHP_IRQ_HANDLER(n)	\
	\
	/* Define the configuration for the device instance. */	\
	COUNTER_MCHP_CONFIG_VAR(n);	\
	\
	/* Define channel data for the device instance. */	\
	COUNTER_MCHP_CHANNEL_DATA_VAR(n)	\
	\
	/* Define top data for the device instance. */	\
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
/* clang-format off */
#define COUNTER_MCHP_DEVICE_INIT(n)	\
	COUNTER_MCHP_DEVICE_INIT_FUNC(n)	\
	DEVICE_DT_INST_DEFINE(n, counter_mchp_init, NULL, &counter_mchp_dev_data_##n,	\
			      &counter_mchp_dev_config_##n, POST_KERNEL,	\
			      CONFIG_COUNTER_INIT_PRIORITY, &counter_mchp_api);
/* clang-format on */

/*
 * @brief Initialize Microchip TCC g1 counter devices.
 *
 * This block checks for the presence of TCC g1 counter devices in the device tree
 * and initializes them using the `COUNTER_MCHP_DEVICE_INIT` macro.
 */

DT_INST_FOREACH_STATUS_OKAY(COUNTER_MCHP_DEVICE_INIT)
