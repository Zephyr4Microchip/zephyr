/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file pwm_mchp_tc_u2249.c
 * @brief PWM driver for Microchip devices.
 *
 * This file provides the implementation of pwm functions
 * for Microchip TC U2249 peripheral
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>

/*******************************************
 * Const and Macro Defines
 ******************************************
 */
#define DT_DRV_COMPAT microchip_pwm_tc_u2249

LOG_MODULE_REGISTER(pwm_mchp_tc_u2249, CONFIG_PWM_LOG_LEVEL);

#define PWM_MODE8(pwm_reg)  ((tc_count8_registers_t *)&(((tc_registers_t *)(pwm_reg))->COUNT8))
#define PWM_MODE16(pwm_reg) ((tc_count16_registers_t *)&(((tc_registers_t *)(pwm_reg))->COUNT16))
#define PWM_MODE32(pwm_reg) ((tc_count32_registers_t *)&(((tc_registers_t *)(pwm_reg))->COUNT32))

#define MCHP_PWM_FAIL    -1
#define MCHP_PWM_SUCCESS 0

/**
 * @brief Type definition for the PWM lock.
 *
 * This macro defines the type of the lock used to protect access to the PWM APIs.
 */
#define MCHP_PWM_LOCK_TYPE struct k_mutex

/**
 * @brief Timeout duration for acquiring the PWM lock.
 *
 * This macro defines the timeout duration for acquiring the PWM lock.
 * The timeout is specified in milliseconds.
 */
#define MCHP_PWM_LOCK_TIMEOUT K_MSEC(10)

/**
 * @brief Initialize the PWM lock.
 *
 * This macro initializes the PWM lock.
 *
 * @param p_lock Pointer to the lock to be initialized.
 */
#define MCHP_PWM_DATA_LOCK_INIT(p_lock) k_mutex_init(p_lock)

/**
 * @brief Acquire the PWM lock.
 *
 * This macro acquires the PWM lock. If the lock is not available, the
 * function will wait for the specified timeout duration.
 *
 * @param p_lock Pointer to the lock to be acquired.
 * @return 0 if the lock was successfully acquired, or a negative error code.
 */
#define MCHP_PWM_DATA_LOCK(p_lock) k_mutex_lock(p_lock, MCHP_PWM_LOCK_TIMEOUT)

/**
 * @brief Release the PWM lock.
 *
 * This macro releases the PWM lock.
 *
 * @param p_lock Pointer to the lock to be released.
 * @return 0 if the lock was successfully released, or a negative error code.
 */
#define MCHP_PWM_DATA_UNLOCK(p_lock) k_mutex_unlock(p_lock)

/** This represents the highest peripheral ID that supports chaining with the client peripheral in
 * 32-bit mode.
 */
#define PERIPH_ID_MAX 2

/***********************************
 * Typedefs and Enum Declarations
 **********************************
 */
typedef enum {
	BIT_MODE_8 = 8,
	BIT_MODE_16 = 16,
	BIT_MODE_24 = 24,
	BIT_MODE_32 = 32,
} pwm_counter_modes_t;

typedef enum {
	PWM_PRESCALE_1 = 1,
	PWM_PRESCALE_2 = 2,
	PWM_PRESCALE_4 = 4,
	PWM_PRESCALE_8 = 8,
	PWM_PRESCALE_16 = 16,
	PWM_PRESCALE_32 = 32,
	PWM_PRESCALE_64 = 64,
	PWM_PRESCALE_128 = 128,
	PWM_PRESCALE_256 = 256,
	PWM_PRESCALE_512 = 512,
	PWM_PRESCALE_1024 = 1024
} pwm_prescale_modes_t;

/**
 * @brief Structure for managing flags for the pwm.
 */
typedef enum {
	PWM_MCHP_FLAGS_CAPTURE_TYPE_PERIOD,
	PWM_MCHP_FLAGS_CAPTURE_TYPE_PULSE,
	PWM_MCHP_FLAGS_CAPTURE_TYPE_BOTH,
	PWM_MCHP_FLAGS_CAPTURE_MODE_SINGLE,
	PWM_MCHP_FLAGS_CAPTURE_MODE_CONTINUOUS,
} pwm_mchp_flags_t;

/**
 * @brief Structure to hold PWM data specific to Microchip hardware.
 */
typedef struct {
	MCHP_PWM_LOCK_TYPE lock; /* Lock type for PWM configuration */
} pwm_mchp_data_t;

typedef struct mchp_counter_clock {
	/* Clock driver */
	const struct device *clock_dev;

	/* Main clock subsystem. */
	clock_control_mchp_subsys_t host_core_sync_clk;
	clock_control_mchp_subsys_t client_core_sync_clk;

	/* Generic clock subsystem. */
	clock_control_mchp_subsys_t periph_async_clk;
} pwm_mchp_clock_t;

/**
 * @brief Structure to hold the configuration for Microchip PWM.
 */
typedef struct {
	void *regs; /*Pointer to PWM peripheral registers */
	/* Used for finding out the mode
	 * at which tc peripheral is configured
	 */
	uint32_t max_bit_width;
	pwm_mchp_clock_t pwm_clock;                      /* PWM clock configuration */
	const struct pinctrl_dev_config *pinctrl_config; /* Pin control configuration */
	uint16_t prescaler;                              /* Prescaler value for PWM */
	uint8_t id;                                      /* The id of the pwm peripheral */
	uint8_t channels;                                /* Number of PWM channels */
	uint32_t freq;                                   /* Frequency of the PWM signal */
} pwm_mchp_config_t;

/***********************************
 * Internal functions
 **********************************
 */
/*
 *This function maps a given prescaler constant to its corresponding numerical
 *value. If the prescaler does not match any predefined constants, it returns 0.
 */
static uint32_t tc_get_prescale_val(uint32_t prescaler)
{
	uint32_t prescaler_val = 0;

	switch (prescaler) {
	case PWM_PRESCALE_1:
		prescaler_val = TC_CTRLA_PRESCALER_DIV1;
		break;
	case PWM_PRESCALE_2:
		prescaler_val = TC_CTRLA_PRESCALER_DIV2;
		break;
	case PWM_PRESCALE_4:
		prescaler_val = TC_CTRLA_PRESCALER_DIV4;
		break;
	case PWM_PRESCALE_8:
		prescaler_val = TC_CTRLA_PRESCALER_DIV8;
		break;
	case PWM_PRESCALE_16:
		prescaler_val = TC_CTRLA_PRESCALER_DIV16;
		break;
	case PWM_PRESCALE_64:
		prescaler_val = TC_CTRLA_PRESCALER_DIV64;
		break;
	case PWM_PRESCALE_256:
		prescaler_val = TC_CTRLA_PRESCALER_DIV256;
		break;
	case PWM_PRESCALE_1024:
		prescaler_val = TC_CTRLA_PRESCALER_DIV1024;
		break;
	default:
		prescaler_val = TC_CTRLA_PRESCALER_DIV1; /*  Default fallback */
		LOG_ERR("Unsupported prescaler specified in dts. Initialising with default "
			"prescaler of DIV1");
		break;
	}
	return prescaler_val;
}

/**
 * This function will check whether the tc periferal is in slave mode or not.
 * This is for ensuring that the tc peripheral will not be configured if it is chained to a host tc
 * peripheral to achieve 32 bit mode.
 */

static bool check_slave_status(const void *pwm_reg)
{
	bool ret = false;

	ret = PWM_MODE8(pwm_reg)->TC_STATUS & TC_STATUS_SLAVE_Msk;
	LOG_DBG("%s", (ret ? "tc is a slave" : "tc is not a slave"));
	return ret;
}
/*
 * This function waits for the synchronization of the TC (Timer/Counter)
 * registers to complete. It takes a pointer to a the pwm peripheral base address and max bit width
 * as input. Depending on the max_bit_width of the PWM, it determines the appropriate register
 * address:
 * - For 8-bit mode, it uses the COUNT8 register.
 * - For 16- mode, it uses the COUNT16 register.
 * - For 32-bit mode, it uses the COUNT32 register.
 * It then enters a while loop, continuously checking the TC_SYNCBUSY bit until
 * it is cleared, indicating synchronization is complete. If the max_bit_width is
 * not supported, it logs an error message.
 */
static void tc_sync_wait(const void *pwm_reg, const uint32_t max_bit_width)
{
	switch (max_bit_width) {
	case BIT_MODE_8:
		while (0 != (PWM_MODE8(pwm_reg)->TC_SYNCBUSY))
			;
		break;

	case BIT_MODE_16:
		while (0 != (PWM_MODE16(pwm_reg)->TC_SYNCBUSY))
			;
		break;

	case BIT_MODE_32:
		while (0 != (PWM_MODE32(pwm_reg)->TC_SYNCBUSY))
			;
		break;

	default:
		LOG_ERR("%s : Unsupported PWM mode %d", __func__, max_bit_width);
		break;
	}
}
/*
 *This function resets the TC registers for the given PWM.
 *It sets the TC_CTRLA register to initiate a software reset based on the
 *max_bit_width. After setting the reset, it waits for synchronization to
 *complete.
 */
static int tc_reset_regs(const void *pwm_reg, const uint32_t max_bit_width)

{
	int32_t ret_val = MCHP_PWM_SUCCESS;

	do {
		ret_val = check_slave_status(pwm_reg);
		if (ret_val != 0) {
			ret_val = MCHP_PWM_FAIL;
			break;
		}

		switch (max_bit_width) {
		case BIT_MODE_8:
			PWM_MODE8(pwm_reg)->TC_CTRLA = TC_CTRLA_SWRST(1);
			break;

		case BIT_MODE_16:
			PWM_MODE16(pwm_reg)->TC_CTRLA = TC_CTRLA_SWRST(1);
			break;

		case BIT_MODE_32:
			PWM_MODE32(pwm_reg)->TC_CTRLA = TC_CTRLA_SWRST(1);
			break;

		default:
			LOG_ERR("%s : Unsupported PWM mode %d", __func__, max_bit_width);
			ret_val = MCHP_PWM_FAIL;
			break;
		}
		LOG_DBG("%s invoked %d", __func__, max_bit_width);
		tc_sync_wait(pwm_reg, max_bit_width);
	} while (0);
	return ret_val;
}

/**
 * This function enables or disables the TC based on the enable parameter.
 * It sets or clears the TC_CTRLA_ENABLE bit in the TC_CTRLA register based on
 * the max_bit_width. After setting or clearing the enable bit, it waits for
 * synchronization to complete.
 */
static int32_t tc_enable(const void *pwm_reg, const uint32_t max_bit_width, bool enable)
{
	int32_t ret_val = MCHP_PWM_SUCCESS;

	do {
		ret_val = check_slave_status(pwm_reg);
		if (ret_val != 0) {
			LOG_ERR("tc is in slave mode");
			ret_val = MCHP_PWM_FAIL;
			break;
		}
		switch (max_bit_width) {
		case BIT_MODE_8:
			if (enable != 0) {
				PWM_MODE8(pwm_reg)->TC_CTRLA |= TC_CTRLA_ENABLE(1);
			} else {
				PWM_MODE8(pwm_reg)->TC_CTRLA &= ~TC_CTRLA_ENABLE(1);
			}
			LOG_DBG("%s %d invoked 0x%x", __func__, enable,
				PWM_MODE8(pwm_reg)->TC_CTRLA);
			break;

		case BIT_MODE_16:
			if (enable != 0) {
				PWM_MODE16(pwm_reg)->TC_CTRLA |= TC_CTRLA_ENABLE(1);
			} else {
				PWM_MODE16(pwm_reg)->TC_CTRLA &= ~TC_CTRLA_ENABLE(1);
			}
			break;

		case BIT_MODE_32:
			if (enable != 0) {
				PWM_MODE32(pwm_reg)->TC_CTRLA |= TC_CTRLA_ENABLE(1);
			} else {
				PWM_MODE32(pwm_reg)->TC_CTRLA &= ~TC_CTRLA_ENABLE(1);
			}
			break;

		default:
			LOG_ERR("%s : Unsupported PWM mode %d", __func__, max_bit_width);
			ret_val = MCHP_PWM_FAIL;
			break;
		}

		tc_sync_wait(pwm_reg, max_bit_width);
	} while (0);
	return ret_val;
}

/*
 * This function sets the mode of the TC based on the max_bit_width.
 * It clears the current mode bits in the TC_CTRLA register and sets the
 * appropriate mode bits. After setting the mode, it waits for synchronization to
 * complete. Returns 0 on success, or -1 if the max_bit_width is unsupported.
 */
static int32_t tc_set_mode(const void *pwm_reg, const uint32_t max_bit_width)
{
	int32_t ret_val = MCHP_PWM_SUCCESS;

	do {
		ret_val = check_slave_status(pwm_reg);
		if (ret_val != 0) {
			LOG_ERR("tc is in slave mode");
			ret_val = MCHP_PWM_FAIL;
			break;
		}
		switch (max_bit_width) {
		case BIT_MODE_8:
			PWM_MODE8(pwm_reg)->TC_CTRLA &= (~TC_CTRLA_MODE_Msk);
			LOG_DBG("CTRLA = 0x%x\n", PWM_MODE8(pwm_reg)->TC_CTRLA);
			PWM_MODE8(pwm_reg)->TC_CTRLA |= TC_CTRLA_MODE(TC_CTRLA_MODE_COUNT8_Val);
			LOG_DBG("CTRLA = 0x%x\n", PWM_MODE8(pwm_reg)->TC_CTRLA);
			break;

		case BIT_MODE_16:
			PWM_MODE16(pwm_reg)->TC_CTRLA &= (~TC_CTRLA_MODE_Msk);
			PWM_MODE16(pwm_reg)->TC_CTRLA |= TC_CTRLA_MODE(TC_CTRLA_MODE_COUNT16_Val);
			break;

		case BIT_MODE_32:
			PWM_MODE32(pwm_reg)->TC_CTRLA &= (~TC_CTRLA_MODE_Msk);
			PWM_MODE32(pwm_reg)->TC_CTRLA |= TC_CTRLA_MODE(TC_CTRLA_MODE_COUNT32_Val);
			break;

		default:
			LOG_ERR("%s : Unsupported PWM mode %d", __func__, max_bit_width);
			ret_val = MCHP_PWM_FAIL;
			break;
		}
		tc_sync_wait(pwm_reg, max_bit_width);
		LOG_DBG("Mode set = %x\n", TC_CTRLA_MODE(TC_CTRLA_MODE_COUNT8_Val));
	} while (0);
	return ret_val;
}

/*
 * This function sets the pulse width for the specified channel based on the
 * max_bit_width. It writes the pulse value to the appropriate TC_CC register.
 * Logs the pulse value for debugging purposes.
 *
 * In 16-bit/32-bit mode, the pulse value is written to CC[1]. This is because they are in MPWM
 * mode. In MPWM mode, the wave output will can be observed in WO[1] and a negative spike can  be
 * observed in each overflow of the counter(at the beginning of each period).
 */
static int32_t tc_set_pulse(const void *pwm_reg, uint32_t max_bit_width, uint32_t channel,
			    uint32_t pulse)
{
	int32_t ret_val = 0;

	do {
		ret_val = check_slave_status(pwm_reg);
		if (ret_val != 0) {
			ret_val = MCHP_PWM_FAIL;
			break;
		}
		switch (max_bit_width) {
		case BIT_MODE_8:
			PWM_MODE8(pwm_reg)->TC_CC[channel] = TC_COUNT8_COUNT_COUNT(pulse);
			LOG_DBG("m_tc_set_pulse invoked 8: 0x%x", TC_COUNT8_COUNT_COUNT(pulse));
			break;

		case BIT_MODE_16:
			PWM_MODE16(pwm_reg)->TC_CC[1] = TC_COUNT16_COUNT_COUNT(pulse);
			LOG_DBG("m_tc_set_pulse invoked 16: 0x%x", TC_COUNT16_COUNT_COUNT(pulse));
			break;

		case BIT_MODE_32:
			PWM_MODE32(pwm_reg)->TC_CC[1] = TC_COUNT32_COUNT_COUNT(pulse);
			LOG_DBG("m_tc_set_pulse invoked 32 : 0x%x", TC_COUNT32_COUNT_COUNT(pulse));
			break;

		default:
			LOG_ERR("%s : Unsupported PWM mode %d", __func__, max_bit_width);
			ret_val = MCHP_PWM_FAIL;
			break;
		}
	} while (0);
	return ret_val;
}

/*
 * This function sets the period value for the TC based on the max_bit_width.
 * It writes the period value to the appropriate register (TC_PER or TC_CC[0]).
 * Logs the period value and max_bit_width for debugging purposes.
 * After setting the period, it waits for synchronization to complete.
 */
static int32_t tc_set_period(const void *pwm_reg, const uint32_t max_bit_width,
			     const uint32_t period)
{
	int32_t ret_val = MCHP_PWM_SUCCESS;

	do {
		ret_val = check_slave_status(pwm_reg);
		if (ret_val != 0) {
			LOG_ERR("tc is in slave mode");
			ret_val = MCHP_PWM_FAIL;
			break;
		}
		switch (max_bit_width) {
		case BIT_MODE_8:
			/** Set the period value */
			PWM_MODE8(pwm_reg)->TC_PER = (uint8_t)period;
			break;

		case BIT_MODE_16:
			/** Set the period value */
			PWM_MODE16(pwm_reg)->TC_CC[0u] = (uint16_t)period;
			break;

		case BIT_MODE_32:
			/** Set the period value */
			PWM_MODE32(pwm_reg)->TC_CC[0u] = period;
			break;

		default:
			LOG_ERR("%s : Unsupported PWM mode %d", __func__, max_bit_width);
			ret_val = MCHP_PWM_FAIL;
			break;
		}
		LOG_DBG("period %d set to %x", max_bit_width, period);
		tc_sync_wait(pwm_reg, max_bit_width);
	} while (0);
	return ret_val;
}

/**
 * This function sets the invert mode for the specified channel based on the
 * max_bit_width. It first disables the TC, waits for synchronization, and then
 * sets the invert mask in the TC_DRVCTRL register. After setting the invert
 * mask, it re-enables the TC and waits for synchronization again.
 */
static int32_t tc_set_invert(const void *pwm_reg, const uint32_t max_bit_width, uint32_t channel)
{
	int32_t ret_val = MCHP_PWM_SUCCESS;

	do {
		ret_val = check_slave_status(pwm_reg);
		if (ret_val != 0) {
			ret_val = MCHP_PWM_FAIL;
			break;
		}
		uint32_t invert_mask = 1 << (channel + TC_DRVCTRL_INVEN0_Pos);

		tc_enable(pwm_reg, max_bit_width, false);
		tc_sync_wait(pwm_reg, max_bit_width);

		switch (max_bit_width) {
		case BIT_MODE_8:

			PWM_MODE8(pwm_reg)->TC_DRVCTRL |= invert_mask;
			LOG_DBG("tc set invert 0x%x invoked", invert_mask);
			break;

		case BIT_MODE_16:

			PWM_MODE16(pwm_reg)->TC_DRVCTRL |= invert_mask;
			break;

		case BIT_MODE_32:

			PWM_MODE32(pwm_reg)->TC_DRVCTRL |= invert_mask;
			break;

		default:
			LOG_ERR("%s : Unsupported PWM mode %d", __func__, max_bit_width);
			ret_val = MCHP_PWM_FAIL;
			break;
		}
		tc_enable(pwm_reg, max_bit_width, true);
		tc_sync_wait(pwm_reg, max_bit_width);
	} while (0);
	return ret_val;
}

/**
 * This function retrieves the invert status for the specified channel based on
 * the max_bit_width. It reads the invert status from the TC_DRVCTRL register and
 * checks if the invert mask is set. Returns true if the invert status is not
 * set, otherwise returns false.
 */
static bool tc_get_invert_status(const void *pwm_reg, const uint32_t max_bit_width,
				 uint32_t channel)
{
	uint32_t invert_status = 0;
	uint32_t invert_mask = 1 << (channel + TC_DRVCTRL_INVEN0_Pos);

	LOG_DBG("mchp_pwm_get_invert_status 0x%x invoked", invert_mask);
	switch (max_bit_width) {
	case BIT_MODE_8:
		invert_status = PWM_MODE8(pwm_reg)->TC_DRVCTRL & invert_mask;
		break;

	case BIT_MODE_16:
		invert_status = PWM_MODE16(pwm_reg)->TC_DRVCTRL & invert_mask;
		break;

	case BIT_MODE_32:
		invert_status = PWM_MODE32(pwm_reg)->TC_DRVCTRL & invert_mask;
		break;

	default:
		LOG_ERR("%s : Unsupported PWM mode %d", __func__, max_bit_width);
		break;
	}
	return (invert_status == 0) ? true : false;
}

/**
 * This function sets the prescaler value for the TC based on the max_bit_width.
 * It writes the prescaler value to the TC_CTRLA register.
 * After setting the prescaler, it waits for synchronization to complete.
 */
static int32_t tc_set_prescaler(const void *pwm_reg, const uint32_t max_bit_width,
				uint32_t prescaler)
{
	int32_t ret_val = MCHP_PWM_SUCCESS;

	do {
		ret_val = check_slave_status(pwm_reg);
		if (ret_val != 0) {
			LOG_ERR("tc is in slave mode");
			ret_val = MCHP_PWM_FAIL;
			break;
		}
		prescaler = tc_get_prescale_val(prescaler);
		switch (max_bit_width) {
		case BIT_MODE_8:
			PWM_MODE8(pwm_reg)->TC_CTRLA |= prescaler;
			break;

		case BIT_MODE_16:
			PWM_MODE16(pwm_reg)->TC_CTRLA |= prescaler;
			break;

		case BIT_MODE_32:
			PWM_MODE32(pwm_reg)->TC_CTRLA |= prescaler;
			break;

		default:
			LOG_ERR("%s : Unsupported PWM mode %d", __func__, max_bit_width);
			ret_val = MCHP_PWM_FAIL;
			break;
		}
		tc_sync_wait(pwm_reg, max_bit_width);
	} while (0);
	return ret_val;
}

/**
 * This function sets the wave generation type for the TC based on the
 * max_bit_width. It writes the appropriate wave generation value to the TC_WAVE
 * register. After setting the wave type, it waits for synchronization to
 * complete.
 *
 * In 16-bit/32-bit mode, the PWM wave type is set as MPWM mode as default. This is because the MAX
 * value of the counter can be controlled in that mode only for setting proper period.
 */
static int32_t tc_set_wave_type(const void *pwm_reg, const uint32_t max_bit_width,
				uint32_t wave_type)
{
	int32_t ret_val = MCHP_PWM_SUCCESS;

	do {
		ret_val = check_slave_status(pwm_reg);
		if (ret_val != 0) {
			LOG_ERR("tc is in slave mode");
			ret_val = MCHP_PWM_FAIL;
			break;
		}
		switch (max_bit_width) {
		case BIT_MODE_8:
			PWM_MODE8(pwm_reg)->TC_WAVE = TC_WAVE_WAVEGEN(wave_type);
			break;

		case BIT_MODE_16:
			PWM_MODE16(pwm_reg)->TC_WAVE = TC_WAVE_WAVEGEN(TC_WAVE_WAVEGEN_MPWM);
			break;

		case BIT_MODE_32:
			PWM_MODE32(pwm_reg)->TC_WAVE = TC_WAVE_WAVEGEN(TC_WAVE_WAVEGEN_MPWM);
			break;

		default:
			LOG_ERR("%s : Unsupported PWM mode %d", __func__, max_bit_width);
			ret_val = MCHP_PWM_FAIL;
			break;
		}
		tc_sync_wait(pwm_reg, max_bit_width);
		LOG_DBG("%s invoked", __func__);
	} while (0);
	return ret_val;
}

/**
 * Initializes the TC for PWM by performing the following steps:
 * 1. Resets the TC registers.
 * 2. Sets the TC mode.
 * 3. Sets the prescaler value.
 * 4. Sets the wave generation type to NPWM.
 * 5. Sets the period to 0.
 * 6. Enables the TC.
 */
static int tc_init(const pwm_mchp_config_t *const mchp_pwm_cfg)
{
	const void *pwm_reg = mchp_pwm_cfg->regs;
	const uint32_t max_bit_width = mchp_pwm_cfg->max_bit_width;
	int ret = MCHP_PWM_SUCCESS;

	do {
		ret = tc_reset_regs(pwm_reg, max_bit_width);
		if (ret != MCHP_PWM_SUCCESS) {
			break;
		}

		tc_sync_wait(pwm_reg, max_bit_width);
		ret = tc_set_mode(pwm_reg, max_bit_width);
		if (ret != MCHP_PWM_SUCCESS) {
			break;
		}

		ret = tc_set_prescaler(pwm_reg, max_bit_width, mchp_pwm_cfg->prescaler);
		if (ret != MCHP_PWM_SUCCESS) {
			break;
		}

		ret = tc_set_wave_type(pwm_reg, max_bit_width, TC_WAVE_WAVEGEN_NPWM);
		if (ret != MCHP_PWM_SUCCESS) {
			break;
		}

		ret = tc_set_period(pwm_reg, max_bit_width, 0);
		if (ret != MCHP_PWM_SUCCESS) {
			break;
		}

		ret = tc_enable(pwm_reg, max_bit_width, true);
		if (ret != MCHP_PWM_SUCCESS) {
			break;
		}

	} while (0);
	return ret;
}
/***********************************
 * Zephyr APIs
 **********************************
 */
/**
 * @brief Set the PWM cycles for a specific channel.
 *
 * This function sets the PWM period and pulse width for a specified channel. It also handles the
 * polarity inversion if required.
 *
 * @param pwm_dev Pointer to the PWM device structure.
 * @param channel PWM channel number.
 * @param period PWM period in cycles.
 * @param pulse PWM pulse width in cycles.
 * @param flags PWM flags (e.g., polarity inversion).
 *
 * @return 0 on success, -EINVAL if the channel is invalid or the period/pulse is out of range.
 */
static int pwm_mchp_set_cycles(const struct device *pwm_dev, uint32_t channel, uint32_t period,
			       uint32_t pulse, pwm_flags_t flags)
{

	const pwm_mchp_config_t *const mchp_pwm_cfg = pwm_dev->config;

	pwm_mchp_data_t *mchp_pwm_data = pwm_dev->data;
	const void *pwm_reg = mchp_pwm_cfg->regs;
	const uint32_t max_bit_width = mchp_pwm_cfg->max_bit_width;
	int ret_val = 0;
	uint32_t top = ((uint32_t)(1 << (max_bit_width)) - 1);

	MCHP_PWM_DATA_LOCK(&mchp_pwm_data->lock);
	do {
		bool invert_flag_set = ((flags & PWM_POLARITY_INVERTED) != 0);
		bool not_inverted = tc_get_invert_status(pwm_reg, max_bit_width, channel);

		if ((invert_flag_set == true) && (not_inverted == true)) {
			ret_val = tc_set_invert(pwm_reg, max_bit_width, channel);
			if (ret_val < 0) {
				LOG_ERR("PWM peripheral busy");
				ret_val = -EBUSY;
				break;
			}
		}

		if (channel >= mchp_pwm_cfg->channels) {
			LOG_ERR("channel %d is invalid", channel);
			ret_val = -EINVAL;
			break;
		}

		if ((period > top) || (pulse > top)) {
			LOG_ERR("period or pulse is out of range");
			ret_val = -EINVAL;
			break;
		}
		ret_val = tc_set_pulse(pwm_reg, max_bit_width, channel, pulse);
		if (ret_val < 0) {
			LOG_ERR("PWM peripheral busy");
			ret_val = -EBUSY;
			break;
		}
		ret_val = tc_set_period(pwm_reg, max_bit_width, period);
		if (ret_val < 0) {
			LOG_ERR("PWM peripheral busy");
			ret_val = -EBUSY;
			break;
		}
	} while (0);
	MCHP_PWM_DATA_UNLOCK(&mchp_pwm_data->lock);
	return ret_val;
}

/**
 * @brief Get the number of PWM cycles per second for a specific channel.
 *
 * This function retrieves the frequency of the PWM signal in cycles per second for a specified
 * channel.
 *
 * @param pwm_dev Pointer to the PWM device structure.
 * @param channel PWM channel number.
 * @param cycles Pointer to store the number of cycles per second.
 *
 * @return 0 on success, -EINVAL if the channel is invalid.
 */

static int pwm_mchp_get_cycles_per_sec(const struct device *pwm_dev, uint32_t channel,
				       uint64_t *cycles)
{
	const pwm_mchp_config_t *const mchp_pwm_cfg = pwm_dev->config;
	pwm_mchp_data_t *mchp_pwm_data = pwm_dev->data;
	uint32_t periph_clk_freq = 0;
	int ret_val = 0;

	MCHP_PWM_DATA_LOCK(&mchp_pwm_data->lock);
	do {
		if (channel >= (mchp_pwm_cfg->channels)) {
			LOG_ERR("channel %d is invalid", channel);
			ret_val = -EINVAL;
			break;
		}
		/* clang-format off */
		clock_control_get_rate(
			mchp_pwm_cfg->pwm_clock.clock_dev,
			(clock_control_subsys_t)&(mchp_pwm_cfg->pwm_clock.periph_async_clk),
			&periph_clk_freq);
		/* clang-format on */
		*cycles = periph_clk_freq / mchp_pwm_cfg->prescaler;
	} while (0);
	MCHP_PWM_DATA_UNLOCK(&mchp_pwm_data->lock);
	return ret_val;
}

/**
 * @brief PWM driver API structure for the Microchip PWM device.
 *
 * This structure defines the API functions for the Microchip PWM driver, including setting PWM
 * cycles, getting the number of cycles per second, and optionally configuring, enabling, and
 * disabling PWM capture.
 */
static const struct pwm_driver_api pwm_mchp_driver_api = {
	/* Turn the clock on for the specified subsystem. */
	.set_cycles = pwm_mchp_set_cycles,
	.get_cycles_per_sec = pwm_mchp_get_cycles_per_sec,
#ifdef CONFIG_PWM_CAPTURE
	.configure_capture = pwm_mchp_configure_capture,
	.enable_capture = pwm_mchp_enable_capture,
	.disable_capture = pwm_mchp_disable_capture,
#endif /* CONFIG_PWM_CAPTURE */
};

/**
 * @brief Initialize the Microchip PWM device.
 *
 * This function initializes the Microchip PWM device by applying the pin control configuration and
 * initializing PWM peripheral with the specified prescaler.
 *
 * @param pwm_dev Pointer to the PWM device structure.
 *
 * @return 0 on success, negative error code on failure.
 */

static int pwm_mchp_init(const struct device *pwm_dev)
{
	int ret_val;
	const pwm_mchp_config_t *const mchp_pwm_cfg = pwm_dev->config;
	pwm_mchp_data_t *mchp_pwm_data = pwm_dev->data;

	MCHP_PWM_DATA_LOCK_INIT(&mchp_pwm_data->lock);
	do {
		ret_val = clock_control_on(
			mchp_pwm_cfg->pwm_clock.clock_dev,
			(clock_control_subsys_t)&mchp_pwm_cfg->pwm_clock.periph_async_clk);
		if (ret_val < 0) {
			LOG_ERR("Failed to enable the periph_async_clk for PWM: %d", ret_val);
			break;
		}
		ret_val = clock_control_on(
			mchp_pwm_cfg->pwm_clock.clock_dev,
			(clock_control_subsys_t)&mchp_pwm_cfg->pwm_clock.host_core_sync_clk);
		if (ret_val < 0) {
			LOG_ERR("Failed to enable the host_core_sync_clk for PWM: %d", ret_val);
			break;
		}
		/* If the mode is 32 bit the turn on the clock of the client peripheral as well.
		 * Need to ensure that the peripheral id of the current instance is less than the
		 * given limit. This is because only till PERIPH_ID_MAX is capable of chaining to a
		 * client peripheral for 32 bit mode
		 */
		if ((mchp_pwm_cfg->max_bit_width == BIT_MODE_32) &&
		    ((mchp_pwm_cfg->id) <= PERIPH_ID_MAX) && (((mchp_pwm_cfg->id) & 1) == 0)) {
			ret_val = clock_control_on(
				mchp_pwm_cfg->pwm_clock.clock_dev,
				(clock_control_subsys_t) &
					(mchp_pwm_cfg->pwm_clock.client_core_sync_clk));
			if (ret_val < 0) {
				LOG_ERR("Failed to enable the client_core_sync_clk for PWM: %d",
					ret_val);
				break;
			}
		} else if (mchp_pwm_cfg->max_bit_width == BIT_MODE_32) {
			LOG_ERR("The selected mode is not supported for the peripheral tc %d",
				mchp_pwm_cfg->id);
			ret_val = -EINVAL;
			break;
		}

		ret_val = pinctrl_apply_state(mchp_pwm_cfg->pinctrl_config, PINCTRL_STATE_DEFAULT);
		if (ret_val < 0) {
			LOG_ERR("pincontrol apply state failed: %d", ret_val);
			break;
		}
		ret_val = tc_init(mchp_pwm_cfg);
	} while (0);
	return ret_val;
}

/**                                                                                                \
 * @brief Macro to define the PWM data structure for a specific instance.                          \
 *                                                                                                 \
 * This macro defines the PWM data structure for a specific instance of the Microchip PWM device.  \
 *                                                                                                 \
 * @param n Instance number.                                                                       \
 */
#define PWM_MCHP_DATA_DEFN(n) static pwm_mchp_data_t pwm_mchp_data_##n

/**
 * @brief Macro to assign clock configurations for the Microchip PWM device.
 *
 * This macro assigns the clock configurations for the PWM device, including the
 * host core synchronous clock, client core synchronous clock (conditionally), and
 * peripheral asynchronous clock (conditionally).
 *
 * @param n Device tree node number.
 *
 * @note This macro conditionally includes client core synchronous clock handling
 *       and peripheral asynchronous clock configurations based on the presence
 *       of relevant device tree properties.
 */
/* clang-format off */
#define PWM_MCHP_CLOCK_ASSIGN(n)							\
	.pwm_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),			\
	.pwm_clock.host_core_sync_clk = {						\
		.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, mclk)),		\
		.id = DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, id)				\
	},										\
	COND_CODE_1(DT_NODE_HAS_PROP(DT_NODELABEL(tc##n), client),			\
	(.pwm_clock.client_core_sync_clk = {						\
	.dev = DEVICE_DT_GET(DT_CLOCKS_CTLR_BY_NAME(					\
	DT_PHANDLE(DT_NODELABEL(tc##n), client), mclk)),				\
	.id = DT_CLOCKS_CELL_BY_NAME(							\
	DT_PHANDLE(DT_NODELABEL(tc##n), client), mclk, id)				\
	},), ())									\
	COND_CODE_1(DT_NODE_EXISTS(DT_INST_CLOCKS_CTLR_BY_NAME(n, osc32kctrl)),		\
	(.pwm_clock.periph_async_clk = {						\
	.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, osc32kctrl)),		\
	.id = DT_INST_CLOCKS_CELL_BY_NAME(n, osc32kctrl, id)				\
	},), ())									\
	COND_CODE_1(DT_NODE_EXISTS(DT_INST_CLOCKS_CTLR_BY_NAME(n, gclk)),		\
	(.pwm_clock.periph_async_clk = {						\
		.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, gclk)),		\
		.id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, id)				\
	},), ())
/* clang-format on */

/**
 * @brief Macro to define the PWM configuration structure for a specific instance.
 *
 * This macro defines the PWM configuration structure for a specific instance of the Microchip PWM
 * device.
 *
 * @param n Instance number.
 */
#define PWM_MCHP_CONFIG_DEFN(n)                                                                    \
	static const pwm_mchp_config_t pwm_mchp_config_##n = {                                     \
		.prescaler = DT_INST_PROP(n, prescaler),                                           \
		.pinctrl_config = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                               \
		.channels = DT_INST_PROP(n, channels),                                             \
		.id = DT_INST_PROP(n, instance_id),                                                \
		.regs = (void *)DT_INST_REG_ADDR(n),                                               \
		.max_bit_width = DT_INST_PROP(n, max_bit_width),                                   \
		PWM_MCHP_CLOCK_ASSIGN(n)}

/**
 * @brief Macro to define the device structure for a specific instance of the PWM device.
 *
 * This macro defines the device structure for a specific instance of the Microchip PWM device.
 * It uses the DEVICE_DT_INST_DEFINE macro to create the device instance with the specified
 * initialization function, data structure, configuration structure, and driver API.
 *
 * @param n Instance number.
 */
#define PWM_MCHP_DEVICE_DT_DEFN(n)                                                                 \
	DEVICE_DT_INST_DEFINE(n, pwm_mchp_init, NULL, &pwm_mchp_data_##n, &pwm_mchp_config_##n,    \
			      POST_KERNEL, CONFIG_PWM_INIT_PRIORITY, &pwm_mchp_driver_api)

/**
 * Initialize the PWM device with pin control, data, and configuration definitions.
 */
#define PWM_MCHP_DEVICE_INIT(n)                                                                    \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	PWM_MCHP_DATA_DEFN(n);                                                                     \
	PWM_MCHP_CONFIG_DEFN(n);                                                                   \
	PWM_MCHP_DEVICE_DT_DEFN(n);

/* Run init macro for each pwm-generic node */
DT_INST_FOREACH_STATUS_OKAY(PWM_MCHP_DEVICE_INIT)
