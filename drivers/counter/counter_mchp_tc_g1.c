/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * @file counter_mchp_tc_g1.c
 * @brief Counter driver implementation for Microchip devices.
 */

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>

/*****************************************************************************
 * Devicetree definitions
 *****************************************************************************/
#define DT_DRV_COMPAT microchip_tc_g1_counter

/*
 * @section Macros
 * User-specific macros for the Microchip counter driver.
 */
LOG_MODULE_REGISTER(counter_mchp_tc_g1, CONFIG_COUNTER_LOG_LEVEL);

/* All timer/counter synchronization bits set. */
#define ALL_TC_SYNC_BITS ((uint32_t)UINT32_MAX)

/* Return value for failed operation. */
#define COUNTER_RET_FAILED (-1)

/* Return value for successful operation. */
#define COUNTER_RET_PASSED (0)

/*
 * @brief Macro functions.
 */
#define COUNTER_MCHP_GET_TC_PRESCALE_INDEX(prescaler)                                              \
	({                                                                                         \
		uint8_t prescale_index = 0u;                                                       \
		if ((prescaler) == 1u) {                                                           \
			prescale_index = 0u;                                                       \
		} else if ((prescaler) == 2u) {                                                    \
			prescale_index = 1u;                                                       \
		} else if ((prescaler) == 4u) {                                                    \
			prescale_index = 2u;                                                       \
		} else if ((prescaler) == 8u) {                                                    \
			prescale_index = 3u;                                                       \
		} else if ((prescaler) == 16u) {                                                   \
			prescale_index = 4u;                                                       \
		} else if ((prescaler) == 64u) {                                                   \
			prescale_index = 5u;                                                       \
		} else if ((prescaler) == 256u) {                                                  \
			prescale_index = 6u;                                                       \
		} else if ((prescaler) == 1024u) {                                                 \
			prescale_index = 7u;                                                       \
		}                                                                                  \
		prescale_index;                                                                    \
	})

/* Synchronization time-out in us */
#define TC_SYNCHRONIZATION_TIMEOUT_IN_US (5)

/* Delay time in us */
#define DELAY_US (2)

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
	clock_control_subsys_t host_mclk;
	clock_control_subsys_t client_mclk;

	/* Generic clock subsystem. */
	clock_control_subsys_t host_gclk;

} counter_mchp_clock_t;

/*
 * @brief Structure to hold channel-specific data for the counter.
 *
 * This structure holds the callback function and user data for a specific channel of the counter.
 */
typedef struct counter_mchp_ch_data {
	/* Callback function for the channel alarm */
	counter_alarm_callback_t callback;

	/* Compare value	 */
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

	/* Late alarm flag   */
	bool late_alarm_flag;

	/* Late alarm channel	  */
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

	/* Clock configuration */
	counter_mchp_clock_t counter_clock;

	/* Common IRQ line */
	uint32_t irq_line;

	/* Maximum bit-width */
	uint32_t max_bit_width;

	/* Prescaler value */
	uint16_t prescaler;

	/* IRQ configuration function */
	void (*irq_config_func)(const struct device *dev);

} counter_mchp_dev_cfg_t;

/* Enumeration for counter bit modes. */
typedef enum {
	COUNTER_BIT_MODE_8 = 8,
	COUNTER_BIT_MODE_16 = 16,
	COUNTER_BIT_MODE_32 = 32,
} tc_counter_modes_t;

/* Enumeration for counter reset modes. */
typedef enum {
	/* Reload or reset the counter on the next generic clock */
	TC_GCLK_RESET_ON_GENERIC_CLOCK = 0x0,

	/* Reload or reset the counter on the next prescaler clock */
	TC_PRESC_RESET_ON_PRESCALER_CLOCK = 0x1,

	/* Reload or reset the counter on the next generic clock,
	 *  and reset the prescaler counter
	 */
	TC_RESYNC_RESET_ON_GENERIC_CLOCK = 0x2
} tc_counter_tc_prescaler_sync_mode_t;

/* Enumeration for event control modes. */
typedef enum {
	TC_EVENT_CONTROL_MODE_OFF = 0x0,   /* Event action disabled. */
	TC_EVENT_CONTROL_MODE_COUNT = 0x2, /* Count on event. */
} tc_counter_tc_evt_control_mode_t;

/* find the prescale index based on the value present in the device tree */
uint8_t get_tc_prescale_index(uint16_t prescaler)
{
	uint8_t prescale_index = 0u;

	switch (prescaler) {
	case 64:
		prescale_index = TC_CTRLA_PRESCALER_DIV64_Val;
		break;
	case 256:
		prescale_index = TC_CTRLA_PRESCALER_DIV256_Val;
		break;
	case 1024:
		prescale_index = TC_CTRLA_PRESCALER_DIV1024_Val;
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
 * @brief Wait for synchronization.
 *
 * This function waits for the SYNCBUSY register to be cleared.
 *
 * @param sync_reg_addr Pointer to the SYNCBUSY register.
 * @param bit_mask Bit mask for the bit to be monitored for synchronization.
 */
static void tc_counter_wait_sync(const volatile uint32_t *sync_reg_addr, uint32_t bit_mask)
{
	bool success = WAIT_FOR((((*sync_reg_addr) & bit_mask) == 0u),
				TC_SYNCHRONIZATION_TIMEOUT_IN_US, k_busy_wait(DELAY_US));

	if (success == false) {
		LOG_ERR("%s : Synchronization time-out occurred", __func__);
	}
}

/*
 * @brief Initialize the TC counter peripheral with the specified settings.
 *
 * This function configures the TC counter peripheral for the specified bit width (8, 16, or 32).
 * It sets up the prescaler, disables capture and event features, configures the waveform,
 * and clears all interrupt flags.
 *
 * @param[in] tc_regs		 Pointer to the TC peripheral register base.
 * @param[in] prescaler		 Prescaler value to configure.
 * @param[in] max_bit_width	 Maximum bit width of the counter (8, 16, or 32).
 *
 * @retval COUNTER_RET_PASSED   Counter initialized successfully.
 * @retval COUNTER_RET_FAILED  Invalid bit width specified.
 */
static int32_t tc_counter_init(const void *tc_regs, uint32_t prescaler,
			       const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	uint32_t ctrla_reg_value = TC_CTRLA_CAPTEN0(0u) | TC_CTRLA_CAPTEN1(0u) |
				   TC_CTRLA_COPEN0(0u) | TC_CTRLA_COPEN1(0u) |
				   TC_CTRLA_PRESCALER(get_tc_prescale_index(prescaler)) |
				   TC_CTRLA_PRESCSYNC(TC_GCLK_RESET_ON_GENERIC_CLOCK) |
				   TC_CTRLA_ONDEMAND(0u) | TC_CTRLA_RUNSTDBY(0u);

	uint32_t evctrl_reg_value = TC_EVCTRL_EVACT(TC_EVENT_CONTROL_MODE_OFF) |
				    (uint16_t)TC_EVCTRL_TCINV(0u) | (uint16_t)TC_EVCTRL_TCEI(0u) |
				    (uint16_t)TC_EVCTRL_OVFEO(0u) | (uint16_t)TC_EVCTRL_MCEO0(0u) |
				    (uint16_t)TC_EVCTRL_MCEO1(0u);

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		tc_count32_registers_t *const p_regs = &(((tc_registers_t *)tc_regs)->COUNT32);

		if (TC_STATUS_Msk != (p_regs->TC_STATUS & TC_STATUS_Msk)) {
			p_regs->TC_CTRLA &= ~TC_CTRLA_ENABLE_Msk;
			tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_ENABLE_Msk);
			p_regs->TC_CTRLA |= TC_CTRLA_SWRST_Msk;
			tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_SWRST_Msk);

			p_regs->TC_CTRLA = ctrla_reg_value | TC_CTRLA_MODE(2u);
			p_regs->TC_WAVE = TC_WAVE_WAVEGEN_MFRQ;
			p_regs->TC_CTRLBSET = TC_CTRLBCLR_ONESHOT(0u) | TC_CTRLBCLR_DIR(0u);
			p_regs->TC_DRVCTRL = TC_DRVCTRL_INVEN(0u);
			p_regs->TC_CC[0u] = UINT32_MAX;
			p_regs->TC_CC[1u] = UINT32_MAX;
			p_regs->TC_INTFLAG = TC_INTFLAG_Msk;
			p_regs->TC_EVCTRL = evctrl_reg_value;
			tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, ALL_TC_SYNC_BITS);
		} else {
			return_value = COUNTER_RET_FAILED;
		}
		break;
	}
	case COUNTER_BIT_MODE_16: {
		tc_count16_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT16);

		if (TC_STATUS_Msk != (p_regs->TC_STATUS & TC_STATUS_Msk)) {
			p_regs->TC_CTRLA &= ~TC_CTRLA_ENABLE_Msk;
			tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_ENABLE_Msk);
			p_regs->TC_CTRLA |= TC_CTRLA_SWRST_Msk;
			tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_SWRST_Msk);

			p_regs->TC_CTRLA = ctrla_reg_value;
			p_regs->TC_WAVE = TC_WAVE_WAVEGEN_MFRQ;
			p_regs->TC_CTRLBSET = TC_CTRLBCLR_ONESHOT(0u) | TC_CTRLBCLR_DIR(0u);
			p_regs->TC_DRVCTRL = TC_DRVCTRL_INVEN(0u);
			p_regs->TC_CC[0u] = UINT16_MAX;
			p_regs->TC_CC[1u] = UINT16_MAX;
			p_regs->TC_INTFLAG = TC_INTFLAG_Msk;
			p_regs->TC_EVCTRL = evctrl_reg_value;
			tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, ALL_TC_SYNC_BITS);
		} else {
			return_value = COUNTER_RET_FAILED;
		}
		break;
	}
	case COUNTER_BIT_MODE_8: {
		tc_count8_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT8);

		if (TC_STATUS_Msk != (p_regs->TC_STATUS & TC_STATUS_Msk)) {
			p_regs->TC_CTRLA &= ~TC_CTRLA_ENABLE_Msk;
			tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_ENABLE_Msk);
			p_regs->TC_CTRLA |= TC_CTRLA_SWRST_Msk;
			tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_SWRST_Msk);

			p_regs->TC_CTRLA = ctrla_reg_value | TC_CTRLA_MODE(1u);
			p_regs->TC_WAVE = TC_WAVE_WAVEGEN_NFRQ;
			p_regs->TC_CTRLBSET = TC_CTRLBCLR_ONESHOT(0u) | TC_CTRLBCLR_DIR(0u);
			p_regs->TC_DRVCTRL = TC_DRVCTRL_INVEN(0u);
			p_regs->TC_CC[0u] = UINT8_MAX;
			p_regs->TC_CC[1u] = UINT8_MAX;
			p_regs->TC_PER = UINT8_MAX;
			p_regs->TC_INTFLAG = TC_INTFLAG_Msk;
			p_regs->TC_EVCTRL = evctrl_reg_value;
			tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, ALL_TC_SYNC_BITS);
		} else {
			return_value = COUNTER_RET_FAILED;
		}
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
 * @brief Start the TC peripheral.
 *
 * This function starts the TC peripheral by issuing a retrigger command for the specified bit
 * width. It waits for synchronization and for the command to complete.
 *
 * @param[in] tc_regs		 Pointer to the TC peripheral register base.
 * @param[in] max_bit_width	 Maximum bit width of the counter (8, 16, or 32).
 *
 * @retval COUNTER_RET_PASSED   Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t tc_counter_start(const void *tc_regs, const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		tc_count32_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT32);

		p_regs->TC_CTRLA |= TC_CTRLA_ENABLE_Msk;
		tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_ENABLE_Msk);

		p_regs->TC_CTRLBSET |= TC_CTRLBSET_CMD_RETRIGGER;
		tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_CTRLB_Msk);
		while ((p_regs->TC_CTRLBSET & TC_CTRLBSET_CMD_Msk) != 0U) {
		}
		break;
	}
	case COUNTER_BIT_MODE_16: {
		tc_count16_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT16);

		p_regs->TC_CTRLA |= TC_CTRLA_ENABLE_Msk;
		tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_ENABLE_Msk);

		p_regs->TC_CTRLBSET |= TC_CTRLBSET_CMD_RETRIGGER;
		tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_CTRLB_Msk);
		while ((p_regs->TC_CTRLBSET & TC_CTRLBSET_CMD_Msk) != 0U) {
		}
		break;
	}
	case COUNTER_BIT_MODE_8: {
		tc_count8_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT8);

		p_regs->TC_CTRLA |= TC_CTRLA_ENABLE_Msk;
		tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_ENABLE_Msk);

		p_regs->TC_CTRLBSET |= TC_CTRLBSET_CMD_RETRIGGER;
		tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_CTRLB_Msk);
		while ((p_regs->TC_CTRLBSET & TC_CTRLBSET_CMD_Msk) != 0U) {
		}
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
 * @brief Stop the TC peripheral.
 *
 * This function stops the TC peripheral by issuing a stop command for the specified bit width.
 * It waits for synchronization and for the command to complete.
 *
 * @param[in] tc_regs		 Pointer to the TC peripheral register base.
 * @param[in] max_bit_width	 Maximum bit width of the counter (8, 16, or 32).
 *
 * @retval COUNTER_RET_PASSED   Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t tc_counter_stop(const void *tc_regs, const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		tc_count32_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT32);

		p_regs->TC_CTRLBSET |= TC_CTRLBSET_CMD_STOP;
		tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_CTRLB_Msk);
		while ((p_regs->TC_CTRLBSET & TC_CTRLBSET_CMD_Msk) != 0U) {
		}
		break;
	}
	case COUNTER_BIT_MODE_16: {
		tc_count16_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT16);

		p_regs->TC_CTRLBSET |= TC_CTRLBSET_CMD_STOP;
		tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_CTRLB_Msk);
		while ((p_regs->TC_CTRLBSET & TC_CTRLBSET_CMD_Msk) != 0U) {
		}
		break;
	}
	case COUNTER_BIT_MODE_8: {
		tc_count8_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT8);

		p_regs->TC_CTRLBSET |= TC_CTRLBSET_CMD_STOP;
		tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_CTRLB_Msk);
		while ((p_regs->TC_CTRLBSET & TC_CTRLBSET_CMD_Msk) != 0U) {
		}
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
 * @brief Retrigger the TC peripheral.
 *
 * This function retriggers the TC peripheral by issuing a retrigger command for the specified bit
 * width. It waits for synchronization and for the command to complete.
 *
 * @param[in] tc_regs		 Pointer to the TC peripheral register base.
 * @param[in] max_bit_width	 Maximum bit width of the counter (8, 16, or 32).
 *
 * @retval COUNTER_RET_PASSED   Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t tc_counter_retrigger(const void *tc_regs, const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		tc_count32_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT32);

		p_regs->TC_CTRLBSET |= TC_CTRLBSET_CMD_RETRIGGER;
		tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_CTRLB_Msk);
		while ((p_regs->TC_CTRLBSET & TC_CTRLBSET_CMD_Msk) != 0U) {
		}
		break;
	}
	case COUNTER_BIT_MODE_16: {
		tc_count16_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT16);

		p_regs->TC_CTRLBSET |= TC_CTRLBSET_CMD_RETRIGGER;
		tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_CTRLB_Msk);
		while ((p_regs->TC_CTRLBSET & TC_CTRLBSET_CMD_Msk) != 0U) {
		}
		break;
	}
	case COUNTER_BIT_MODE_8: {
		tc_count8_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT8);

		p_regs->TC_CTRLBSET |= TC_CTRLBSET_CMD_RETRIGGER;
		tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_CTRLB_Msk);
		while ((p_regs->TC_CTRLBSET & TC_CTRLBSET_CMD_Msk) != 0U) {
		}
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
 * @brief Get the current count value.
 *
 * This function gets the current count value of the TC peripheral for the specified bit width (8,
 * 16, or 32). It issues a read synchronization command before reading the value.
 *
 * @param[in] tc_regs		 Pointer to the TC peripheral register base.
 * @param[in] max_bit_width	 Maximum bit width of the counter (8, 16, or 32).
 *
 * @retval COUNTER_RET_PASSED   Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static inline int32_t tc_counter_get_count(const void *tc_regs, uint32_t *const counter_value,
					   const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		tc_count32_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT32);

		p_regs->TC_CTRLBSET |= TC_CTRLBSET_CMD_READSYNC;
		tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_CTRLB_Msk);
		while ((p_regs->TC_CTRLBSET & TC_CTRLBSET_CMD_Msk) != 0U) {
		}
		*counter_value = p_regs->TC_COUNT;
		break;
	}
	case COUNTER_BIT_MODE_16: {
		tc_count16_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT16);

		p_regs->TC_CTRLBSET |= TC_CTRLBSET_CMD_READSYNC;
		tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_CTRLB_Msk);
		while ((p_regs->TC_CTRLBSET & TC_CTRLBSET_CMD_Msk) != 0U) {
		}
		*counter_value = p_regs->TC_COUNT;
		break;
	}
	case COUNTER_BIT_MODE_8: {
		tc_count8_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT8);

		p_regs->TC_CTRLBSET |= TC_CTRLBSET_CMD_READSYNC;
		tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_CTRLB_Msk);
		while ((p_regs->TC_CTRLBSET & TC_CTRLBSET_CMD_Msk) != 0U) {
		}
		*counter_value = p_regs->TC_COUNT;
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
 * This function sets the period value of the TC peripheral for the specified bit width (8, 16, or
 * 32).
 *
 * @param[in] tc_regs		 Pointer to the TC peripheral register base.
 * @param[in] period		 The period value to set.
 * @param[in] max_bit_width	 Maximum bit width of the counter (8, 16, or 32).
 *
 * @retval COUNTER_RET_PASSED   Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t tc_counter_set_period(const void *tc_regs, const uint32_t period,
				     const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		tc_count32_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT32);

		p_regs->TC_CC[0u] = period;
		tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_CC0_Msk);
		break;
	}
	case COUNTER_BIT_MODE_16: {
		tc_count16_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT16);

		p_regs->TC_CC[0u] = period;
		tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_CC0_Msk);
		break;
	}
	case COUNTER_BIT_MODE_8: {
		tc_count8_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT8);

		p_regs->TC_PER = period;
		tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_CC0_Msk);
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
 * This function gets the current period value of the TC peripheral for the specified bit width (8,
 * 16, or 32).
 *
 * @param[in] tc_regs		 Pointer to the TC peripheral register base.
 * @param[in] period		 Period value
 * @param[in] max_bit_width	 Maximum bit width of the counter (8, 16, or 32).
 *
 * @retval COUNTER_RET_PASSED   Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t tc_counter_get_period(const void *tc_regs, uint32_t *const period,
				     const uint32_t max_bit_width)
{
	uint32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		tc_count32_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT32);

		*period = p_regs->TC_CC[0u];
		break;
	}
	case COUNTER_BIT_MODE_16: {
		tc_count16_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT16);

		*period = p_regs->TC_CC[0u];
		break;
	}
	case COUNTER_BIT_MODE_8: {
		tc_count8_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT8);

		*period = p_regs->TC_PER;
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
 * @brief Set the compare value for a channel.
 *
 * This function sets the compare value for a specified channel of the TC peripheral for the
 * specified bit width (8, 16, or 32).
 *
 * @param[in] tc_regs		 Pointer to the TC peripheral register base.
 * @param[in] chan_id		 Channel ID (ignored for 32/16-bit).
 * @param[in] compare_value	 The compare value to set.
 * @param[in] max_bit_width	 Maximum bit width of the counter (8, 16, or 32).
 *
 * @retval COUNTER_RET_PASSED   Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t tc_counter_set_compare(const void *tc_regs, const uint32_t chan_id,
				      const uint32_t compare_value, const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		(void)chan_id;
		tc_count32_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT32);

		p_regs->TC_CC[1u] = compare_value;
		tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_CC1_Msk);
		break;
	}
	case COUNTER_BIT_MODE_16: {
		(void)chan_id;
		tc_count16_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT16);

		p_regs->TC_CC[1u] = compare_value;
		tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_CC1_Msk);
		break;
	}
	case COUNTER_BIT_MODE_8: {
		tc_count8_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT8);

		p_regs->TC_CC[chan_id] = compare_value;
		if (chan_id == 0u) {
			tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_CC0_Msk);
		} else {
			tc_counter_wait_sync(&p_regs->TC_SYNCBUSY, TC_SYNCBUSY_CC1_Msk);
		}
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
 * @brief Get pending interrupt flags.
 *
 * This function returns the current interrupt flags of the TC peripheral for the specified bit
 * width (8, 16, or 32).
 *
 * @param[in] tc_regs		 Pointer to the TC peripheral register base.
 * @param[in] max_bit_width	 Maximum bit width of the counter (8, 16, or 32).
 *
 * @return Interrupt flags value
 */
static int32_t tc_counter_get_pending_irqs(const void *tc_regs, const uint32_t max_bit_width)
{
	int32_t ret_status = 0;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		tc_count32_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT32);

		ret_status = p_regs->TC_INTFLAG;
	} break;
	case COUNTER_BIT_MODE_16: {
		tc_count16_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT16);

		ret_status = p_regs->TC_INTFLAG;
	} break;
	case COUNTER_BIT_MODE_8: {
		tc_count8_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT8);

		ret_status = p_regs->TC_INTFLAG;
	} break;
	default:
		LOG_ERR("%s : Unsupported Counter mode %d", __func__, max_bit_width);
		break;
	}

	return ret_status;
}

/*
 * @brief Enable alarm interrupt for a specific channel.
 *
 * This function enables the alarm interrupt for a specified channel of the TC peripheral.
 *
 * @param[in] tc_regs		 Pointer to the TC peripheral register base.
 * @param[in] channel_id	 The channel ID to enable the alarm interrupt for.
 * @param[in] max_bit_width	 Maximum bit width of the counter (8, 16, or 32).
 *
 * @retval COUNTER_RET_PASSED   Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t tc_counter_alarm_irq_enable(const void *tc_regs, const uint32_t channel_id,
					   const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		tc_count32_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT32);

		if (channel_id == 0u) {
			p_regs->TC_INTENSET = TC_INTFLAG_MC1_Msk;
		} else {
			return_value = COUNTER_RET_FAILED;
		}
		break;
	}
	case COUNTER_BIT_MODE_16: {
		tc_count16_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT16);

		if (channel_id == 0u) {
			p_regs->TC_INTENSET = TC_INTFLAG_MC1_Msk;
		} else {
			return_value = COUNTER_RET_FAILED;
		}
		break;
	}
	case COUNTER_BIT_MODE_8: {
		tc_count8_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT8);

		if (channel_id == 0u) {
			p_regs->TC_INTENSET = TC_INTFLAG_MC0_Msk;
		} else if (channel_id == 1u) {
			p_regs->TC_INTENSET = TC_INTFLAG_MC1_Msk;
		} else {
			return_value = COUNTER_RET_FAILED;
		}
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
 * @brief Disable alarm interrupt for a specific channel.
 *
 * This function disables the alarm interrupt for a specified channel of the TC peripheral.
 *
 * @param[in] tc_regs		 Pointer to the TC peripheral register base.
 * @param[in] channel_id	 The channel ID to disable the alarm interrupt for.
 * @param[in] max_bit_width	 Maximum bit width of the counter (8, 16, or 32).
 *
 * @retval COUNTER_RET_PASSED   Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t tc_counter_alarm_irq_disable(const void *tc_regs, const uint32_t channel_id,
					    const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		tc_count32_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT32);

		if (channel_id == 0u) {
			p_regs->TC_INTENCLR = TC_INTFLAG_MC1_Msk;
		} else {
			return_value = COUNTER_RET_FAILED;
		}
		break;
	}
	case COUNTER_BIT_MODE_16: {
		tc_count16_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT16);

		if (channel_id == 0u) {
			p_regs->TC_INTENCLR = TC_INTFLAG_MC1_Msk;
		} else {
			return_value = COUNTER_RET_FAILED;
		}
		break;
	}
	case COUNTER_BIT_MODE_8: {
		tc_count8_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT8);

		if (channel_id == 0u) {
			p_regs->TC_INTENCLR = TC_INTFLAG_MC0_Msk;
		} else if (channel_id == 1u) {
			p_regs->TC_INTENCLR = TC_INTFLAG_MC1_Msk;
		} else {
			return_value = COUNTER_RET_FAILED;
		}
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
 * @brief Clear alarm interrupt for a specific channel.
 *
 * This function clears the alarm interrupt for a specified channel of the TC peripheral.
 *
 * @param[in] tc_regs		 Pointer to the TC peripheral register base.
 * @param[in] channel_id	 The channel ID to clear the alarm interrupt for.
 * @param[in] max_bit_width	 Maximum bit width of the counter (8, 16, or 32).
 *
 * @retval COUNTER_RET_PASSED   Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t tc_counter_alarm_irq_clear(const void *tc_regs, const uint32_t channel_id,
					  const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		tc_count32_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT32);

		if (channel_id == 0u) {
			p_regs->TC_INTFLAG = TC_INTFLAG_MC1_Msk;
		} else {
			return_value = COUNTER_RET_FAILED;
		}
		break;
	}
	case COUNTER_BIT_MODE_16: {
		tc_count16_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT16);

		if (channel_id == 0u) {
			p_regs->TC_INTFLAG = TC_INTFLAG_MC1_Msk;
		} else {
			return_value = COUNTER_RET_FAILED;
		}
		break;
	}
	case COUNTER_BIT_MODE_8: {
		tc_count8_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT8);

		if (channel_id == 0u) {
			p_regs->TC_INTFLAG = TC_INTFLAG_MC0_Msk;
		} else if (channel_id == 1u) {
			p_regs->TC_INTFLAG = TC_INTFLAG_MC1_Msk;
		} else {
			return_value = COUNTER_RET_FAILED;
		}
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
 * @brief Check the alarm interrupt status for a specific channel.
 *
 * This function checks the alarm interrupt status for a specified channel of the TC peripheral.
 *
 * @param[in] tc_regs		 Pointer to the TC peripheral register base.
 * @param[in] channel_id	 The channel ID to check the alarm interrupt status for.
 * @param[in] max_bit_width	 Maximum bit width of the counter (8, 16, or 32).
 *
 * @return true if the interrupt is pending, false otherwise
 */
static bool tc_counter_alarm_irq_status(const uint32_t pending_irq_status,
					const uint32_t channel_id, const uint32_t max_bit_width)
{
	bool ret_status = false;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		if (channel_id == 0u) {
			ret_status = (bool)((pending_irq_status & (uint8_t)TC_INTFLAG_MC1_Msk) ==
					    TC_INTFLAG_MC1_Msk);
		} else {
			ret_status = false;
		}
		break;
	}
	case COUNTER_BIT_MODE_16: {
		if (channel_id == 0u) {
			ret_status = (bool)((pending_irq_status & (uint8_t)TC_INTFLAG_MC1_Msk) ==
					    TC_INTFLAG_MC1_Msk);
		} else {
			ret_status = false;
		}
		break;
	}
	case COUNTER_BIT_MODE_8: {
		if (channel_id == 0u) {
			ret_status = (bool)((pending_irq_status & (uint8_t)TC_INTFLAG_MC0_Msk) ==
					    TC_INTFLAG_MC0_Msk);
		} else if (channel_id == 1u) {
			ret_status = (bool)((pending_irq_status & (uint8_t)TC_INTFLAG_MC1_Msk) ==
					    TC_INTFLAG_MC1_Msk);
		} else {
			ret_status = false;
		}
		break;
	}
	default:
		LOG_ERR("%s : Unsupported Counter mode %d", __func__, max_bit_width);
		break;
	}

	return ret_status;
}

/*
 * @brief Enable overflow interrupt.
 *
 * This function enables the overflow interrupt for the TC peripheral for the specified bit width
 * (8, 16, or 32).
 *
 * @param[in] tc_regs		 Pointer to the TC peripheral register base.
 * @param[in] max_bit_width	 Maximum bit width of the counter (8, 16, or 32).
 *
 * @retval COUNTER_RET_PASSED   Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t tc_counter_top_irq_enable(const void *tc_regs, const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		tc_count32_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT32);

		p_regs->TC_INTENSET = TC_INTFLAG_MC0_Msk;
		break;
	}
	case COUNTER_BIT_MODE_16: {
		tc_count16_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT16);

		p_regs->TC_INTENSET = TC_INTFLAG_MC0_Msk;
		break;
	}
	case COUNTER_BIT_MODE_8: {
		tc_count8_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT8);

		p_regs->TC_INTENSET = TC_INTFLAG_OVF_Msk;
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
 * This function disables the overflow interrupt for the TC peripheral for the specified bit width
 * (8, 16, or 32).
 *
 * @param[in] tc_regs		 Pointer to the TC peripheral register base.
 * @param[in] max_bit_width	 Maximum bit width of the counter (8, 16, or 32).
 *
 * @retval COUNTER_RET_PASSED   Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t tc_counter_top_irq_disable(const void *tc_regs, const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		tc_count32_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT32);

		p_regs->TC_INTENCLR = TC_INTFLAG_MC0_Msk;
		break;
	}
	case COUNTER_BIT_MODE_16: {
		tc_count16_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT16);

		p_regs->TC_INTENCLR = TC_INTFLAG_MC0_Msk;
		break;
	}
	case COUNTER_BIT_MODE_8: {
		tc_count8_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT8);

		p_regs->TC_INTENCLR = TC_INTFLAG_OVF_Msk;
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
 * This function clears the overflow interrupt for the TC peripheral for the specified bit width (8,
 * 16, or 32).
 *
 * @param[in] tc_regs		 Pointer to the TC peripheral register base.
 * @param[in] max_bit_width	 Maximum bit width of the counter (8, 16, or 32).
 *
 * @retval COUNTER_RET_PASSED   Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t tc_counter_top_irq_clear(const void *tc_regs, const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		tc_count32_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT32);

		p_regs->TC_INTFLAG = TC_INTFLAG_MC0_Msk;
		break;
	}
	case COUNTER_BIT_MODE_16: {
		tc_count16_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT16);

		p_regs->TC_INTFLAG = TC_INTFLAG_MC0_Msk;
		break;
	}
	case COUNTER_BIT_MODE_8: {
		tc_count8_registers_t *p_regs = &(((tc_registers_t *)tc_regs)->COUNT8);

		p_regs->TC_INTFLAG = TC_INTFLAG_OVF_Msk;
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
 * This function checks the overflow interrupt status for the TC peripheral for the specified bit
 * width (8, 16, or 32).
 *
 * @param[in] tc_regs		 Pointer to the TC peripheral register base.
 * @param[in] max_bit_width	 Maximum bit width of the counter (8, 16, or 32).
 *
 * @retval COUNTER_RET_PASSED   Success.
 * @retval COUNTER_RET_FAILED  Invalid bit width.
 */
static int32_t tc_counter_top_irq_status(const uint32_t pending_irq_status,
					 const uint32_t max_bit_width)
{
	int32_t return_value = COUNTER_RET_PASSED;

	switch (max_bit_width) {
	case COUNTER_BIT_MODE_32: {
		return_value =
			((pending_irq_status & (uint8_t)TC_INTFLAG_MC0_Msk) == TC_INTFLAG_MC0_Msk);
		break;
	}
	case COUNTER_BIT_MODE_16: {
		return_value =
			((pending_irq_status & (uint8_t)TC_INTFLAG_MC0_Msk) == TC_INTFLAG_MC0_Msk);
		break;
	}
	case COUNTER_BIT_MODE_8: {
		return_value =
			((pending_irq_status & (uint8_t)TC_INTFLAG_OVF_Msk) == TC_INTFLAG_OVF_Msk);
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
static uint32_t tc_counter_ticks_sub(uint32_t val, uint32_t old, uint32_t top)
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
static uint32_t tc_counter_ticks_add(uint32_t val1, uint32_t val2, uint32_t top)
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
static uint32_t tc_counter_ticks_diff(uint32_t a, uint32_t b, uint32_t top)
{
	uint32_t diff = (a > b) ? (a - b) : (b - a);
	uint32_t wrap_diff = top - diff;

	return (diff < wrap_diff) ? diff : wrap_diff;
}

/*
 * @brief tc_counter_start the counter.
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, negative error code on failure.
 */
static int32_t counter_mchp_start(const struct device *const dev)
{
	const counter_mchp_dev_cfg_t *const cfg = dev->config;

	tc_counter_start(cfg->regs, cfg->max_bit_width);

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

	tc_counter_stop(cfg->regs, cfg->max_bit_width);

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

	return tc_counter_get_count(cfg->regs, ticks, cfg->max_bit_width);
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
	tc_counter_get_period(cfg->regs, &top_value, cfg->max_bit_width);
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
	(void)tc_counter_get_count(cfg->regs, &count_value, cfg->max_bit_width);
	furthest_count_value = tc_counter_ticks_sub(count_value, data->guard_period, top_value);

	tc_counter_set_compare(cfg->regs, chan_id, furthest_count_value, cfg->max_bit_width);
	tc_counter_alarm_irq_clear(cfg->regs, chan_id, cfg->max_bit_width);

	/* Update new callback functions */
	data->channel_data[chan_id].callback = alarm_cfg->callback;
	data->channel_data[chan_id].user_data = alarm_cfg->user_data;

	/* Check if "Absolute Alarm" flag is set */
	absolute = alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE;

	/* Check if counter has exceeded the alarm count in absolute alarm configuration */
	if (absolute != 0) {
		count_diff = tc_counter_ticks_diff(count_value, ticks, top_value);
		if (count_diff <= data->guard_period) {
			ret_status = -ETIME;
			irq_on_late = alarm_cfg->flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE;
			if (0 != irq_on_late) {
				data->late_alarm_flag = true;
				data->late_alarm_channel = chan_id;

				/* Update compare value*/
				data->channel_data[chan_id].compare_value = ticks;

				/* Trigger interrupt immediately */
				NVIC_SetPendingIRQ(cfg->irq_line);
			} else {
				data->channel_data[chan_id].callback = NULL;
				data->channel_data[chan_id].user_data = NULL;
			}
		} else {
			/* Update compare value*/
			data->channel_data[chan_id].compare_value = ticks;
			tc_counter_set_compare(cfg->regs, chan_id, ticks, cfg->max_bit_width);

			/* Enable interrupt at compare match */
			tc_counter_alarm_irq_enable(cfg->regs, chan_id, cfg->max_bit_width);
		}
	} else {
		ticks = tc_counter_ticks_add(count_value, ticks, top_value);

		/* Update compare value*/
		data->channel_data[chan_id].compare_value = ticks;
		tc_counter_set_compare(cfg->regs, chan_id, ticks, cfg->max_bit_width);

		/* Enable interrupt at compare match */
		tc_counter_alarm_irq_enable(cfg->regs, chan_id, cfg->max_bit_width);
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
	tc_counter_alarm_irq_disable(cfg->regs, chan_id, cfg->max_bit_width);
	tc_counter_alarm_irq_clear(cfg->regs, chan_id, cfg->max_bit_width);

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
	tc_counter_top_irq_disable(cfg->regs, cfg->max_bit_width);
	tc_counter_top_irq_clear(cfg->regs, cfg->max_bit_width);

	/* Update callback functions */
	data->top_cb = top_cfg->callback;
	data->top_user_data = top_cfg->user_data;

	/* Update the counter period based on top configuration data */
	tc_counter_set_period(cfg->regs, top_cfg->ticks, cfg->max_bit_width);

	if (top_cfg->flags & COUNTER_TOP_CFG_DONT_RESET) {
		/*
		 * Top trigger is on equality of the rising edge only, so
		 * manually reset it if the counter has missed the new top.
		 */
		uint32_t counter_value = 0u;

		tc_counter_get_count(cfg->regs, &counter_value, cfg->max_bit_width);
		if (counter_value >= top_cfg->ticks) {
			ret_status = -ETIME;
			if ((top_cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) ==
			    COUNTER_TOP_CFG_RESET_WHEN_LATE) {
				tc_counter_retrigger(cfg->regs, cfg->max_bit_width);
			}
		}
	} else {
		tc_counter_retrigger(cfg->regs, cfg->max_bit_width);
	}

	/* Enable top IRQ */
	if (NULL != top_cfg->callback) {
		tc_counter_top_irq_enable(cfg->regs, cfg->max_bit_width);
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

	tc_counter_get_pending_irqs(cfg->regs, cfg->max_bit_width);

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

	(void)tc_counter_get_period(cfg->regs, &period_value, cfg->max_bit_width);

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
	tc_counter_get_period(cfg->regs, &period_value, cfg->max_bit_width);
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
	clock_control_get_rate(clk->clock_dev, clk->host_gclk, &source_clk_freq);

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
static int32_t counter_init(const struct device *const dev)
{
	int32_t ret_status = 0;
	const counter_mchp_dev_cfg_t *const cfg = dev->config;
	uint32_t max_counter_val = 0u;

	do {
		max_counter_val = (uint32_t)((1ULL << cfg->max_bit_width) - 1u);
		if (max_counter_val != cfg->info.max_top_value) {
			LOG_ERR("%s : Maximum bit width not allowed", __func__);
			ret_status = COUNTER_RET_FAILED;
			break;
		}

		/* Initialize counter peripheral */
		ret_status = tc_counter_init(cfg->regs, cfg->prescaler, cfg->max_bit_width);
		if (ret_status < 0) {
			LOG_ERR("%s : Counter failed to initialize", __func__);
		} else {
			/* Initialize interrupt */
			cfg->irq_config_func(dev);
		}
	} while (0);
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
	counter_mchp_dev_data_t *const data = dev->data;
	const counter_mchp_dev_cfg_t *const cfg = dev->config;
	uint32_t pending_irq_status = 0u;

	/* Clear NVIC Flag to avoid retrigger */
	NVIC_ClearPendingIRQ(cfg->irq_line);

	/* Get pending IRQs  */
	pending_irq_status = tc_counter_get_pending_irqs(cfg->regs, cfg->max_bit_width);

	/* Check for immediate interrupt trigger */
	if (true == data->late_alarm_flag) {
		data->late_alarm_flag = false;
		if (NULL != data->channel_data[data->late_alarm_channel].callback) {
			counter_alarm_callback_t cb = NULL;

			cb = data->channel_data[data->late_alarm_channel].callback;
			if (cb != NULL) {
				data->channel_data[data->late_alarm_channel].callback = NULL;
				cc_value =
					data->channel_data[data->late_alarm_channel].compare_value;

				/* Execute callback function  */
				cb(dev, data->late_alarm_channel, cc_value,
				   data->channel_data[data->late_alarm_channel].user_data);
			}
		}

		return;
	}

	for (uint8_t chan_id = 0; chan_id < counter_get_num_of_channels(dev); chan_id++) {
		if (false !=
		    tc_counter_alarm_irq_status(pending_irq_status, chan_id, cfg->max_bit_width)) {
			tc_counter_alarm_irq_clear(cfg->regs, chan_id, cfg->max_bit_width);

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
	uint32_t pending_irq_status = 0u;

	/* Clear NVIC Flag to avoid retrigger */
	NVIC_ClearPendingIRQ(cfg->irq_line);

	/* Get pending IRQs  */
	pending_irq_status = tc_counter_get_pending_irqs(cfg->regs, cfg->max_bit_width);

	if (0 != tc_counter_top_irq_status(pending_irq_status, cfg->max_bit_width)) {
		tc_counter_top_irq_clear(cfg->regs, cfg->max_bit_width);

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
 * @brief Macro to determine the number of compare/capture channels for Microchip TC g1.
 *
 * This macro evaluates the `max_bit_width` property of the device instance and returns
 * 1 if the bit width is either 32 or 16, otherwise it returns 2.
 *
 * @param n Instance number (not used in this macro).
 */
/* clang-format off */
#define COUNTER_MCHP_CC_NUMS(n) \
	((DT_INST_PROP(n, max_bit_width) >= 16) ? 1 : 2)
/* clang-format on */

#define COUNTER_MCHP_MAX_BIT_WIDTH(n) (DT_INST_PROP(n, max_bit_width))

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
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, prescaler),   \
				(DT_INST_PROP(n, prescaler)), (1))
/* clang-format on */
/* clang-format off */
#define GET_THE_CLIENT_MCLOCK_IF_AVAILABLE(n)						\
	COND_CODE_1(DT_INST_CLOCKS_HAS_NAME(n, client_mclk),				\
	((void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, client_mclk, subsystem))),		\
	NULL)
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
#define COUNTER_MCHP_CLOCK_ASSIGN(n)								\
	.counter_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),				\
	.counter_clock.host_mclk = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, subsystem)),	\
	.counter_clock.host_gclk = (void *)DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, subsystem),	\
	.counter_clock.client_mclk = GET_THE_CLIENT_MCLOCK_IF_AVAILABLE(n)
/* clang-format on */
static int32_t counter_mchp_init(const struct device *dev)
{
	int32_t ret_status = COUNTER_RET_PASSED;

	/*< Enable the clock for the device instance. */
	ret_status = COUNTER_RET_PASSED;
	const counter_mchp_dev_cfg_t *cfg = (const counter_mchp_dev_cfg_t *)(dev->config);
	counter_mchp_clock_t *const clk = (counter_mchp_clock_t *const)&cfg->counter_clock;

	/* Enable host synchronous core clock */
	ret_status = clock_control_on(clk->clock_dev, clk->host_mclk);
	if ((ret_status < 0) && (ret_status != -EALREADY)) {
		LOG_ERR("%s : Unable to initialize host clock", __func__);
		ret_status = COUNTER_RET_FAILED;
	}

	/* Conditionally enable client synchronous core clock */
	if (ret_status != COUNTER_RET_FAILED) {
		if (cfg->max_bit_width == 32) {
			if (clk->client_mclk != NULL) {
				ret_status = clock_control_on(clk->clock_dev, clk->client_mclk);
				if ((ret_status < 0) && (ret_status != -EALREADY)) {
					LOG_ERR("%s : Unable to initialize client clock", __func__);
					ret_status = COUNTER_RET_FAILED;
				}
			} else {
				LOG_ERR("Peripheral does not support 32 bit mode");
				ret_status = COUNTER_RET_FAILED;
			}
		}
	}

	if (ret_status != COUNTER_RET_FAILED) { /* Enable peripheral asynchronous clock */
		ret_status = clock_control_on(clk->clock_dev, clk->host_gclk);
		if ((ret_status < 0) && (ret_status != -EALREADY)) {
			LOG_ERR("%s : Unable to initialize peripheral clock", __func__);
			ret_status = COUNTER_RET_FAILED;
		}
	}
	if (ret_status != COUNTER_RET_FAILED) {
		ret_status = counter_init(dev);
	} else {
		ret_status = COUNTER_RET_FAILED;
	}
	return ret_status;
}
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
#define COUNTER_MCHP_CONFIG_VAR(n)								\
	static const counter_mchp_dev_cfg_t counter_mchp_dev_config_##n = {			\
		.info = {									\
			.max_top_value =							\
				((uint32_t)((1ULL << COUNTER_MCHP_MAX_BIT_WIDTH(n)) -  1)),	\
			.freq = 0u,								\
			.flags = COUNTER_CONFIG_INFO_COUNT_UP,					\
			.channels = COUNTER_MCHP_CC_NUMS(n) },					\
		.regs = (void *)DT_INST_REG_ADDR(n),						\
		COUNTER_MCHP_CLOCK_ASSIGN(n),							\
		.irq_line =	DT_INST_IRQ_BY_IDX(n, 0, irq),					\
		.max_bit_width = DT_INST_PROP(n, max_bit_width),				\
		.prescaler = COUNTER_MCHP_PRESCALER(n),						\
		.irq_config_func = &counter_mchp_config_##n,					\
	};
/* clang-format on */

/*
 * @brief Configure the IRQ handler for the Microchip TC g1 counter.
 *
 * This macro sets up the IRQ handler for the specified instance of the
 * Microchip TC g1 counter. It connects the first IRQ line (index 0)
 * for the counter peripheral.
 *
 * @param n Instance number of the counter.
 *
 * @note This macro assumes that the TC g1 counter has a single IRQ line.
 */
/* clang-format off */
#define COUNTER_MCHP_IRQ_HANDLER(n)					\
	static void counter_mchp_config_##n(const struct device *dev)	\
	{								\
		MCHP_COUNTER_IRQ_CONNECT(n, 0);				\
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
#define COUNTER_MCHP_CHANNEL_DATA_VAR(n)				\
	static counter_mchp_ch_data_t					\
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
#define COUNTER_MCHP_TOP_DATA_VAR(n)					\
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
#define MCHP_COUNTER_IRQ_CONNECT(n, m)							\
	COND_CODE_1(DT_IRQ_HAS_IDX(DT_DRV_INST(n), m),					\
	(										\
		do {									\
			IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, m, irq),			\
			    DT_INST_IRQ_BY_IDX(n, m, priority),				\
			    counter_mchp_interrupt_handler, DEVICE_DT_INST_GET(n), 0);	\
			irq_enable(DT_INST_IRQ_BY_IDX(n, m, irq));			\
		} while (false);							\
	),										\
	()										\
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
 * - Enables the clock for the device instance.
 * - Calls the main initialization function for the device.
 *
 * @param n Instance number of the device.
 */
/* clang-format off */
#define COUNTER_MCHP_DEVICE_INIT_FUNC(n)					\
	/* Function for IRQ Handler */						\
	COUNTER_MCHP_IRQ_HANDLER(n)						\
	/* Define the configuration for the device instance. */			\
	COUNTER_MCHP_CONFIG_VAR(n);						\
	/* Define channel data for the device instance. */			\
	COUNTER_MCHP_CHANNEL_DATA_VAR(n)					\
	/* Define top data for the device instance. */				\
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
#define COUNTER_MCHP_DEVICE_INIT(n)							\
	COUNTER_MCHP_DEVICE_INIT_FUNC(n)						\
	DEVICE_DT_INST_DEFINE(n, counter_mchp_init, NULL,				\
				  &counter_mchp_dev_data_##n,				\
				  &counter_mchp_dev_config_##n, POST_KERNEL,		\
				  CONFIG_COUNTER_INIT_PRIORITY, &counter_mchp_api);
/* clang-format on */

/*
 * @brief Initialize Microchip TC g1 counter devices.
 *
 * This block checks for the presence of TC g1 counter devices in the device tree
 * and initializes them using the `COUNTER_MCHP_DEVICE_INIT` macro.
 */

DT_INST_FOREACH_STATUS_OKAY(COUNTER_MCHP_DEVICE_INIT)
