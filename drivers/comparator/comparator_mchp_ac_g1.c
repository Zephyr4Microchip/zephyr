/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file comparator_mchp_ac_g1.c
 * @brief Microchip analog comparator driver for G1 Peripherals
 *
 * Implements Zephyr's analog comparator API for the AC G1 peripherals.
 */

#include <soc.h>
#include <zephyr/drivers/comparator.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>

/******************************************************************************
 * @brief Devicetree definitions
 *****************************************************************************/
#define DT_DRV_COMPAT microchip_ac_g1_comparator

/******************************************************************************
 * @brief Macro definitions
 *****************************************************************************/
LOG_MODULE_REGISTER(comparator_mchp_ac_g1, CONFIG_COMPARATOR_LOG_LEVEL);

/* Get device configuration structure from device pointer */
#define DEV_CFG(dev) ((const comparator_mchp_dev_config_t *)(dev)->config)

/*
 * Shortcut macro to access AC (Analog Comparator) registers from the device configuration.
 *
 * We `#undef` AC_REGS first because some Microchip SoC headers may already define `AC_REGS`
 * as a fixed base address macro. Since we need an instance-specific (per-device) version
 * for Zephyr's device model, we redefine it here based on `dev->config`.
 */
#undef AC_REGS
#define AC_REGS (((const comparator_mchp_dev_config_t *)(dev)->config)->regs)

#define TIMEOUT_VALUE_US 1000
#define DELAY_US         2

#define COMPARATOR_INT_STATUS_NONE (-1)

/*******************************************
 * Data Types and Static Configuration
 *******************************************/
/** @brief Positive input mux selection (MUXPOS) */
typedef enum {
	MCHP_COMP_POS_INPUT_PIN0 = 0,
	MCHP_COMP_POS_INPUT_PIN1,
	MCHP_COMP_POS_INPUT_PIN2,
	MCHP_COMP_POS_INPUT_PIN3,
	MCHP_COMP_POS_INPUT_VSCALE,
} mchp_comp_pos_input_t;

/** @brief Negative input mux selection (MUXNEG) */
typedef enum {
	MCHP_COMP_NEG_INPUT_PIN0 = 0,
	MCHP_COMP_NEG_INPUT_PIN1,
	MCHP_COMP_NEG_INPUT_PIN2,
	MCHP_COMP_NEG_INPUT_PIN3,
	MCHP_COMP_NEG_INPUT_GND,
	MCHP_COMP_NEG_INPUT_VSCALE,
	MCHP_COMP_NEG_INPUT_BANDGAP,
	MCHP_COMP_NEG_INPUT_DAC,
} mchp_comp_neg_input_t;

/** @brief Comparator output routing mode */
typedef enum {
	MCHP_COMP_OUTPUT_OFF = 0,
	MCHP_COMP_OUTPUT_ASYNC,
	MCHP_COMP_OUTPUT_SYNC,
} mchp_comp_output_mode_t;

/** @brief Comparator interrupt selection mode */
typedef enum {
	MCHP_COMP_INT_TOGGLE = 0,
	MCHP_COMP_INT_RISING,
	MCHP_COMP_INT_FALLING,
	MCHP_COMP_INT_EOC,
} mchp_comp_interrupt_selection_t;

/** @brief Comparator filter mode */
typedef enum {
	MCHP_COMP_FILTER_OFF = 0,
	MCHP_COMP_FILTER_MAJ3,
	MCHP_COMP_FILTER_MAJ5,
} mchp_comp_filter_t;

/** @brief Comparator hysteresis level */
typedef enum {
	MCHP_COMP_HYST_50MV = 0,
	MCHP_COMP_HYST_100MV,
	MCHP_COMP_HYST_150MV,
} mchp_comp_hysteresis_t;

/**
 * @brief Configuration structure for a single Microchip analog comparator channel.
 *
 * This structure holds the configuration parameters parsed from the devicetree
 * for an individual comparator channel inside the AC peripheral.
 */
typedef struct comparator_mchp_channel_cfg {

	/* Comparator channel index (0 or 1) */
	uint8_t channel_id;

	/* Positive input mux selection */
	mchp_comp_pos_input_t pos_input;

	/* Negative input mux selection */
	mchp_comp_neg_input_t neg_input;

	/* Output routing mode (off, async, sync) */
	mchp_comp_output_mode_t output_mode;

	/* Interrupt condition (toggle, rising, falling, eoc) */
	mchp_comp_interrupt_selection_t interrupt_selection;

	/* Output filter mode (off, maj3, maj5) */
	mchp_comp_filter_t filter_length;

	/* Hysteresis voltage level */
	mchp_comp_hysteresis_t hysteresis_level;

	/* vddana-scale-value */
	uint8_t vddana_scale_value;

	/* Enable interrupt generation */
	bool interrupt_enable;

	/* Enable single-shot mode */
	bool single_shot_mode;

	/* Enable hysteresis */
	bool hysteresis_enable;

	/* Keep comparator running in standby mode */
	bool run_standby;

	/* Enable event input */
	bool event_input_enable;

	/* Enable event output */
	bool event_output_enable;

	/* Swap and invert comparator inputs */
	bool swap_inputs;
} comparator_mchp_channel_cfg_t;

/**
 * @brief Runtime data for the Microchip comparator driver.
 */
typedef struct comparator_mchp_dev_data {

	/* Configured interrupt trigger type */
	uint32_t interrupt_mask;

	/* Last interrupt status or edge detected */
	uint32_t interrupt_status;

	/* Application-provided callback function */
	comparator_callback_t callback;

	/* Context data passed to the callback */
	void *user_data;
} comparator_mchp_dev_data_t;

/**
 * @brief Comparator clock configuration.
 */
typedef struct comparator_mchp_clock {

	/* Clock controller device */
	const struct device *clock_dev;

	/* Main clock subsystem */
	clock_control_subsys_t mclk_sys;

	/* Generic clock subsystem */
	clock_control_subsys_t gclk_sys;
} comparator_mchp_clock_t;

/**
 * @brief Comparator device configuration structure.
 */
typedef struct comparator_mchp_dev_config {

	/* Base address of AC peripheral registers */
	ac_registers_t *regs;

	/* Pin control configuration */
	const struct pinctrl_dev_config *pcfg;

	/* Comparator clock configuration */
	comparator_mchp_clock_t comparator_clock;

	/* Pointer to device-specific config/init function */
	void (*config_func)(const struct device *dev);

	/* Static configuration for the comparator channel */
	comparator_mchp_channel_cfg_t channel_config;
} comparator_mchp_dev_config_t;

/*******************************************
 * Static Helper Functions
 *******************************************/
/* Print channel configuration */
static void comparator_mchp_print_channel_cfg(const comparator_mchp_channel_cfg_t *cfg)
{
	static const char *const pos_input_names[] = {"PIN0", "PIN1", "PIN2", "PIN3", "VSCALE"};
	static const char *const neg_input_names[] = {"PIN0", "PIN1",   "PIN2",    "PIN3",
						      "GND",  "VSCALE", "BANDGAP", "DAC"};
	static const char *const output_mode_names[] = {"OFF", "ASYNC", "SYNC"};
	static const char *const interrupt_selection_names[] = {"TOGGLE", "RISING", "FALLING",
								"EOC"};
	static const char *const filter_names[] = {"OFF", "MAJ3", "MAJ5"};
	static const char *const hysteresis_names[] = {"HYST50", "HYST100", "HYST150"};

	LOG_DBG("=== Comparator Channel Configuration ===\n");
	LOG_DBG("Channel ID           : %u\n", cfg->channel_id);
	LOG_DBG("Positive Input       : %s\n", pos_input_names[cfg->pos_input]);
	LOG_DBG("Negative Input       : %s\n", neg_input_names[cfg->neg_input]);
	LOG_DBG("Output Mode          : %s\n", output_mode_names[cfg->output_mode]);
	LOG_DBG("Interrupt Selection  : %s\n", interrupt_selection_names[cfg->interrupt_selection]);
	LOG_DBG("Filter Length        : %s\n", filter_names[cfg->filter_length]);
	LOG_DBG("Hysteresis Enabled   : %s\n", cfg->hysteresis_enable ? "Yes" : "No");
	LOG_DBG("Hysteresis Level     : %s\n", hysteresis_names[cfg->hysteresis_level]);
	LOG_DBG("Single-shot Mode     : %s\n", cfg->single_shot_mode ? "Yes" : "No");
	LOG_DBG("Interrupt Enabled    : %s\n", cfg->interrupt_enable ? "Yes" : "No");
	LOG_DBG("Run in Standby       : %s\n", cfg->run_standby ? "Yes" : "No");
	LOG_DBG("Swap Inputs          : %s\n", cfg->swap_inputs ? "Yes" : "No");
	LOG_DBG("Event Input Enabled  : %s\n", cfg->event_input_enable ? "Yes" : "No");
	LOG_DBG("Event Output Enabled : %s\n", cfg->event_output_enable ? "Yes" : "No");
	LOG_DBG("========================================\n");
}

/* Print all the comparator register values */
static void comparator_print_reg(const struct device *dev)
{
	LOG_DBG("=============== Comparator Registers ===============\r\n");
	LOG_DBG("%-20s: 0x%02x\r\n", "AC_CTRLA", AC_REGS->AC_CTRLA);
	LOG_DBG("%-20s: 0x%02x\r\n", "AC_CTRLB", AC_REGS->AC_CTRLB);
	LOG_DBG("%-20s: 0x%04x\r\n", "AC_EVCTRL", AC_REGS->AC_EVCTRL);
	LOG_DBG("%-20s: 0x%02x\r\n", "AC_INTENCLR", AC_REGS->AC_INTENCLR);
	LOG_DBG("%-20s: 0x%02x\r\n", "AC_INTENSET", AC_REGS->AC_INTENSET);
	LOG_DBG("%-20s: 0x%02x\r\n", "AC_INTFLAG", AC_REGS->AC_INTFLAG);
	LOG_DBG("%-20s: 0x%02x\r\n", "AC_STATUSA", AC_REGS->AC_STATUSA);
	LOG_DBG("%-20s: 0x%02x\r\n", "AC_STATUSB", AC_REGS->AC_STATUSB);
	LOG_DBG("%-20s: 0x%02x\r\n", "AC_DBGCTRL", AC_REGS->AC_DBGCTRL);
	LOG_DBG("%-20s: 0x%02x\r\n", "AC_WINCTRL", AC_REGS->AC_WINCTRL);
	LOG_DBG("%-20s: 0x%02x\r\n", "AC_SCALER[0]", AC_REGS->AC_SCALER[0]);
	LOG_DBG("%-20s: 0x%02x\r\n", "AC_SCALER[1]", AC_REGS->AC_SCALER[1]);
	LOG_DBG("%-20s: 0x%08x\r\n", "AC_COMPCTRL[0]", (uint32_t)AC_REGS->AC_COMPCTRL[0]);
	LOG_DBG("%-20s: 0x%08x\r\n", "AC_COMPCTRL[1]", (uint32_t)AC_REGS->AC_COMPCTRL[1]);
	LOG_DBG("%-20s: 0x%08x\r\n", "AC_SYNCBUSY", (uint32_t)AC_REGS->AC_SYNCBUSY);
	LOG_DBG("%-20s: 0x%04x\r\n", "AC_CALIB", AC_REGS->AC_CALIB);
	LOG_DBG("===================================================\r\n");
}

/* Wait for synchronization */
static inline void ac_wait_sync(ac_registers_t *ac_reg, uint32_t mask)
{
	if (!WAIT_FOR(((ac_reg->AC_SYNCBUSY & mask) != mask), TIMEOUT_VALUE_US,
		      k_busy_wait(DELAY_US))) {
		LOG_ERR("Timeout waiting for AC_SYNCBUSY bits to clear (mask=0x%X)", mask);
	}
}

/* AC utility functions for enabling, configuring, and syncing the analog comparator */
static inline void ac_enable(ac_registers_t *ac_reg, bool enable)
{
	if (enable == true) {
		ac_reg->AC_CTRLA |= AC_CTRLA_ENABLE_Msk;
	} else {
		ac_reg->AC_CTRLA &= ~AC_CTRLA_ENABLE_Msk;
	}

	ac_wait_sync(ac_reg, AC_SYNCBUSY_ENABLE_Msk);
}

/* Enable or disable a specific comparator channel and wait for sync */
static inline void ac_channel_enable(ac_registers_t *ac_reg, uint8_t channel_id, bool enable)
{
	if (enable == true) {
		ac_reg->AC_COMPCTRL[channel_id] |= AC_COMPCTRL_ENABLE_Msk;
	} else {
		ac_reg->AC_COMPCTRL[channel_id] &= ~AC_COMPCTRL_ENABLE_Msk;
	}

	/* Only 2 possible values for channel_id: 0 or 1 */
	if (channel_id == 0) {
		ac_wait_sync(ac_reg, AC_SYNCBUSY_COMPCTRL0_Msk);
	} else {
		ac_wait_sync(ac_reg, AC_SYNCBUSY_COMPCTRL1_Msk);
	}
}

/* Enable or disable interrupt for the given comparator channel */
static inline void ac_enable_interrupt(ac_registers_t *ac_reg, uint8_t channel_id, bool enable)
{
	if (enable == true) {
		if (channel_id == 0) {
			ac_reg->AC_INTENSET = AC_INTENSET_COMP0_Msk;
		} else {
			ac_reg->AC_INTENSET = AC_INTENSET_COMP1_Msk;
		}
	} else {
		if (channel_id == 0) {
			ac_reg->AC_INTENSET &= ~AC_INTENSET_COMP0_Msk;
		} else {
			ac_reg->AC_INTENSET &= ~AC_INTENSET_COMP1_Msk;
		}
	}
}

/* Set the AC interrupt selection based on the desired trigger type */
int ac_interrupt_selection(ac_registers_t *ac_reg, uint8_t channel_id,
			   enum comparator_trigger trigger_type)
{
	uint32_t int_sel_val = AC_COMPCTRL_INTSEL_EOC_Val;

	switch (trigger_type) {
	case COMPARATOR_TRIGGER_NONE:
		int_sel_val = AC_COMPCTRL_INTSEL_EOC_Val;
		break;
	case COMPARATOR_TRIGGER_RISING_EDGE:
		int_sel_val = AC_COMPCTRL_INTSEL_RISING_Val;
		break;
	case COMPARATOR_TRIGGER_FALLING_EDGE:
		int_sel_val = AC_COMPCTRL_INTSEL_FALLING_Val;
		break;
	case COMPARATOR_TRIGGER_BOTH_EDGES:
		int_sel_val = AC_COMPCTRL_INTSEL_TOGGLE_Val;
		break;
	default:
		LOG_WRN("Invalid interrupt selection for comparator, Default to "
			"COMPARATOR_TRIGGER_NONE");
		break;
	}

	/* Clear existing INTSEL bits */
	ac_reg->AC_COMPCTRL[channel_id] &= ~AC_COMPCTRL_INTSEL_Msk;

	ac_reg->AC_COMPCTRL[channel_id] |= AC_COMPCTRL_INTSEL(int_sel_val);

	return 0;
}

/* Trigger a single-shot comparison for the specified channel */
static inline void ac_start_conversion(ac_registers_t *ac_reg, uint8_t channel_id)
{
	ac_reg->AC_CTRLB |= (AC_CTRLB_START0_Msk << channel_id);
}

/* Wait until the comparator result for the specified channel is ready */
static inline int ac_wait_for_conversion(ac_registers_t *ac_reg, uint8_t channel_id)
{
	uint32_t ready_mask = AC_STATUSB_READY0_Msk << channel_id;

	if (!WAIT_FOR(((ac_reg->AC_STATUSB & ready_mask) == ready_mask), TIMEOUT_VALUE_US,
		      k_busy_wait(DELAY_US))) {
		LOG_ERR("Timeout waiting for AC_STATUSB ready (channel=%u)", channel_id);
		return -ETIMEDOUT;
	}

	return 0;
}

/* Get the current comparator output state for the specified channel
 * Returns true if output is HIGH, false if LOW
 */
static inline bool ac_get_result(ac_registers_t *ac_reg, uint8_t channel_id)
{
	uint32_t state_mask = AC_STATUSA_STATE0_Msk << channel_id;

	return (ac_reg->AC_STATUSA & state_mask) != 0;
}

/* Configure comparator channel: inputs, mode, hysteresis, output, interrupt, scaler, etc. */
int ac_configure_channel(const struct device *dev)
{
	/* Get device config */
	const comparator_mchp_dev_config_t *const dev_cfg = DEV_CFG(dev);
	const comparator_mchp_channel_cfg_t *channel_config = &dev_cfg->channel_config;
	uint8_t channel_id = channel_config->channel_id;

	/* Reset COMPCTRL before new config */
	AC_REGS->AC_COMPCTRL[channel_id] = 0;

	/* Set single-shot or continuous mode */
	AC_REGS->AC_COMPCTRL[channel_id] = AC_COMPCTRL_SINGLE(channel_config->single_shot_mode);

	/* Set MUX inputs */
	AC_REGS->AC_COMPCTRL[channel_id] |= AC_COMPCTRL_MUXPOS(channel_config->pos_input);
	AC_REGS->AC_COMPCTRL[channel_id] |= AC_COMPCTRL_MUXNEG(channel_config->neg_input);

	/* Configure VDDANA scaler if used */
	if (channel_config->neg_input == MCHP_COMP_NEG_INPUT_VSCALE ||
	    channel_config->pos_input == MCHP_COMP_POS_INPUT_VSCALE) {
		AC_REGS->AC_SCALER[channel_id] =
			AC_SCALER_VALUE(channel_config->vddana_scale_value);
	}

	/* Set interrupt selection */
	AC_REGS->AC_COMPCTRL[channel_id] |= AC_COMPCTRL_INTSEL(channel_config->interrupt_selection);

	/* Set Output mode */
	AC_REGS->AC_COMPCTRL[channel_id] |= AC_COMPCTRL_OUT(channel_config->output_mode);

	/* Set Filter length */
	AC_REGS->AC_COMPCTRL[channel_id] |= AC_COMPCTRL_FLEN(channel_config->filter_length);

	if (channel_config->single_shot_mode == false) {
		/* Enable hysterisis */
		AC_REGS->AC_COMPCTRL[channel_id] |=
			AC_COMPCTRL_HYSTEN(channel_config->hysteresis_enable ? 1 : 0);

		/* Set hysterisis */
		if (channel_config->hysteresis_enable == true) {
			AC_REGS->AC_COMPCTRL[channel_id] |=
				AC_COMPCTRL_HYST(channel_config->hysteresis_level);
		}
	}

	/* Set Comparator speed */
	AC_REGS->AC_COMPCTRL[channel_id] |= AC_COMPCTRL_SPEED(AC_COMPCTRL_SPEED_HIGH_Val);

	/* Run in standby if enabled */
	if (channel_config->run_standby == true) {
		AC_REGS->AC_COMPCTRL[channel_id] |= AC_COMPCTRL_RUNSTDBY(1);
	}

	LOG_DBG("Configuration done AC_REGS->AC_COMPCTRL[0] : 0x%x\n",
		AC_REGS->AC_COMPCTRL[channel_id]);

	/* Interrupt Enable */
	ac_enable_interrupt(AC_REGS, channel_id, true);

	return 0;
}

/*******************************************
 * Interrupt Service Routines (ISRs)
 *******************************************/
static void comparator_mchp_isr(const struct device *dev)
{
	comparator_mchp_dev_data_t *const dev_data = (comparator_mchp_dev_data_t *const)dev->data;

	/* Store comparator status (latched value from AC_STATUSA) */
	dev_data->interrupt_status = AC_REGS->AC_STATUSA;

	/* Clear interrupt flags (write 1 to clear) */
	AC_REGS->AC_INTFLAG = AC_INTFLAG_Msk;

	/* Call registered callback if present */
	if (dev_data->callback != NULL) {
		dev_data->callback(dev, dev_data->user_data);
	}
}

/*******************************************
 * Zephyr Driver API Implementations
 *******************************************/
/**
 * @brief Get the output state of the comparator.
 *
 * For single-shot mode, this triggers a conversion. Waits for the conversion result
 * and returns the comparator's output state (HIGH or LOW).
 *
 * @param dev Pointer to the comparator device.
 *
 * @retval 1 Comparator output is HIGH.
 * @retval 0 Comparator output is LOW.
 * @retval -ETIMEDOUT Conversion did not complete within the timeout period.
 */
static int comparator_mchp_get_output(const struct device *dev)
{
	const comparator_mchp_dev_config_t *const dev_cfg = DEV_CFG(dev);
	const comparator_mchp_channel_cfg_t *channel_config = &dev_cfg->channel_config;
	uint8_t channel_id = channel_config->channel_id;

	int ret = 0;
	bool result = false;

	/* Optional debug */
	comparator_mchp_print_channel_cfg(channel_config);

	/* Trigger conversion in single-shot mode */
	if (channel_config->single_shot_mode == true) {
		ac_start_conversion(AC_REGS, channel_id);
	}

	/* Wait for result to become ready */
	ret = ac_wait_for_conversion(AC_REGS, channel_id);
	if (ret == 0) {

		/* Optional debug */
		comparator_print_reg(dev);

		/* Get result */
		result = ac_get_result(AC_REGS, channel_id);
		ret = result ? 1 : 0;

		LOG_DBG("AC comparator result: %s", result ? "HIGH" : "LOW");
	}

	return ret;
}

/**
 * @brief Configure comparator interrupt trigger mode.
 *
 * Sets the trigger condition for the comparator (rising, falling, both edges, or none).
 * Internally maps the trigger enum to the appropriate INTSEL field value and enables
 * or disables interrupts accordingly.
 *
 * @param dev Pointer to the comparator device.
 * @param trigger Trigger mode to configure.
 *
 * @retval 0 Always returns success.
 */
static int comparator_mchp_set_trigger(const struct device *dev, enum comparator_trigger trigger)
{
	const comparator_mchp_dev_config_t *const dev_cfg = DEV_CFG(dev);
	comparator_mchp_dev_data_t *const dev_data = (comparator_mchp_dev_data_t *)dev->data;
	const comparator_mchp_channel_cfg_t *channel_config = &dev_cfg->channel_config;
	uint8_t channel_id = channel_config->channel_id;

	uint8_t intsel_val;

	LOG_INF("Setting comparator trigger mode: %d", trigger);

	/* Disable comparator before reconfig */
	ac_channel_enable(AC_REGS, channel_id, false);

	/* Map trigger enum to hardware INTSEL value */
	switch (trigger) {
	case COMPARATOR_TRIGGER_NONE:
		intsel_val = AC_COMPCTRL_INTSEL_EOC_Val;
		break;
	case COMPARATOR_TRIGGER_RISING_EDGE:
		intsel_val = AC_COMPCTRL_INTSEL_RISING_Val;
		break;
	case COMPARATOR_TRIGGER_FALLING_EDGE:
		intsel_val = AC_COMPCTRL_INTSEL_FALLING_Val;
		break;
	case COMPARATOR_TRIGGER_BOTH_EDGES:
		intsel_val = AC_COMPCTRL_INTSEL_TOGGLE_Val;
		break;
	default:
		LOG_WRN("Invalid comparator trigger: %d, defaulting to NONE", trigger);
		intsel_val = AC_COMPCTRL_INTSEL_EOC_Val;
		trigger = COMPARATOR_TRIGGER_NONE;
		break;
	}

	/* Update INTSEL field */
	AC_REGS->AC_COMPCTRL[channel_id] &= ~AC_COMPCTRL_INTSEL_Msk;
	AC_REGS->AC_COMPCTRL[channel_id] |= AC_COMPCTRL_INTSEL(intsel_val);

	/* Enable or disable interrupt */
	ac_enable_interrupt(AC_REGS, channel_id, trigger != COMPARATOR_TRIGGER_NONE);

	LOG_INF("Trigger mode: %d, INTSEL set to: %d", trigger, intsel_val);

	/* Store trigger mode (not INTSEL value!) */
	dev_data->interrupt_mask = trigger;

	/* Re-enable comparator */
	ac_channel_enable(AC_REGS, channel_id, true);

	return 0;
}

/**
 * @brief Set a user callback for comparator trigger events.
 *
 * Registers a callback function that will be invoked when a comparator trigger occurs.
 *
 * @param dev Pointer to the comparator device.
 * @param callback Function to call on comparator trigger (can be NULL to unregister).
 * @param user_data User-supplied data passed to the callback.
 *
 * @retval 0 Always returns success.
 */
static int comparator_mchp_set_trigger_callback(const struct device *dev,
						comparator_callback_t callback, void *user_data)
{
	comparator_mchp_dev_data_t *const dev_data = (comparator_mchp_dev_data_t *)dev->data;

	dev_data->callback = callback;
	dev_data->user_data = user_data;

	LOG_INF("Comparator callback registered");

	return 0;
}

/**
 * @brief Check and clear a pending comparator trigger (polling mode).
 *
 * This function checks whether a comparator interrupt trigger is pending,
 * based on the configured edge selection. Only applicable if no callback is registered.
 *
 * @param dev Pointer to the comparator device.
 *
 * @retval 1 Trigger is pending and acknowledged.
 * @retval 0 No trigger is pending.
 */
static int comparator_mchp_trigger_is_pending(const struct device *dev)
{
	const comparator_mchp_dev_config_t *const dev_cfg = DEV_CFG(dev);

	/* Retrieve device runtime data */
	comparator_mchp_dev_data_t *const dev_data =
		((comparator_mchp_dev_data_t *const)(dev)->data);
	const comparator_mchp_channel_cfg_t *channel_config = &dev_cfg->channel_config;
	uint8_t channel_id = channel_config->channel_id;
	int ret = 0;
	uint32_t state_mask = AC_STATUSA_STATE0_Msk << channel_id;

	LOG_INF("Checking if comparator trigger is pending...");

	/* Only process in polling mode (no callback handler registered) */
	if (dev_data->callback == NULL &&
	    dev_data->interrupt_status != COMPARATOR_INT_STATUS_NONE) {
		switch (dev_data->interrupt_mask) {
		case COMPARATOR_TRIGGER_RISING_EDGE:
			if ((dev_data->interrupt_status & state_mask) != 0) {
				LOG_INF("Rising edge trigger is pending");
				ret = 1;
			}
			break;
		case COMPARATOR_TRIGGER_FALLING_EDGE:
			if ((dev_data->interrupt_status & state_mask) == 0) {
				LOG_INF("Falling edge trigger is pending");
				ret = 1;
			}
			break;
		case COMPARATOR_TRIGGER_BOTH_EDGES:
			LOG_INF("Both edge trigger is pending");
			ret = 1;
			break;
		default:
			break;
		}
		/* Mark as consumed */
		dev_data->interrupt_status = COMPARATOR_INT_STATUS_NONE;
	}

	/* Return true if the comparator result is ready */
	return ret;
}

/*******************************************
 * Driver Initialization Function
 *******************************************/

static int comparator_mchp_init(const struct device *dev)
{
	const comparator_mchp_dev_config_t *const dev_cfg = DEV_CFG(dev);
	comparator_mchp_dev_data_t *const dev_data = dev->data;

	int ret = 0;
	uint8_t channel_id = dev_cfg->channel_config.channel_id;

	dev_data->interrupt_status = COMPARATOR_INT_STATUS_NONE;

	do {
		/* Turn on GCLK for AC */
		ret = clock_control_on(dev_cfg->comparator_clock.clock_dev,
				       dev_cfg->comparator_clock.gclk_sys);
		if (ret < 0 && ret != -EALREADY) {
			LOG_ERR("Failed to enable GCLK for COMP: %d", ret);
			break;
		}

		/* Turn on MCLK for AC */
		ret = clock_control_on(dev_cfg->comparator_clock.clock_dev,
				       dev_cfg->comparator_clock.mclk_sys);
		if (ret < 0 && ret != -EALREADY) {
			LOG_ERR("Failed to enable MCLK for COMP: %d", ret);
			break;
		}

		/* Apply pinctrl default state */
		ret = pinctrl_apply_state(dev_cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			LOG_ERR("Failed to apply pinctrl state: %d", ret);
			break;
		}

		/* Reset the comparator peripheral */
		AC_REGS->AC_CTRLA = AC_CTRLA_SWRST_Msk;
		ac_wait_sync(AC_REGS, AC_SYNCBUSY_SWRST_Msk);

		/* Configure calibration and ISR */
		dev_cfg->config_func(dev);

		/* Configure comparator channel */
		ac_configure_channel(dev);

		/* Enable comparator channel */
		ac_channel_enable(AC_REGS, channel_id, true);

		comparator_print_reg(dev);

		/* Enable AC peripheral */
		ac_enable(AC_REGS, true);
		ac_wait_sync(AC_REGS, AC_SYNCBUSY_ENABLE_Msk);

	} while (0);

	/* If everything is OK but the clocks are already enabled, return 0. */
	return (ret == -EALREADY) ? 0 : ret;
}

/*******************************************
 * Zephyr Driver API Structure
 *******************************************/
static DEVICE_API(comparator, comparator_mchp_api) = {
	.get_output = comparator_mchp_get_output,
	.set_trigger = comparator_mchp_set_trigger,
	.set_trigger_callback = comparator_mchp_set_trigger_callback,
	.trigger_is_pending = comparator_mchp_trigger_is_pending,
};

/*******************************************
 * Driver Instantiation
 *******************************************/

#define COMPARATOR_MCHP_DATA_DEFN(n) static comparator_mchp_dev_data_t comparator_mchp_data_##n;

/* Connect a single IRQ for instance n at index idx */
#define COMPARATOR_MCHP_IRQ_CONNECT(idx, n)                                                        \
	IF_ENABLED(DT_INST_IRQ_HAS_IDX(n, idx),\
		(                           \
			IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, idx, irq),         \
						DT_INST_IRQ_BY_IDX(n, idx, priority),             \
						comparator_mchp_isr,                         \
						DEVICE_DT_INST_GET(n), 0);                        \
			irq_enable(DT_INST_IRQ_BY_IDX(n, idx, irq));                    \
		)                                                                    \
	)

/* Generate config function for instance n */
#define COMPARATOR_MCHP_DEFINE_CONFIG_FUNC(n)                                                      \
	static void comparator_mchp_config_##n(const struct device *dev)                           \
	{                                                                                          \
		/* Connect all IRQs declared in devicetree */                                      \
		LISTIFY(DT_NUM_IRQS(DT_DRV_INST(n)),                                   \
				COMPARATOR_MCHP_IRQ_CONNECT,                                   \
				(), n);             \
		/* Load AC calibration from fuse and write to hardware register */                 \
		((comparator_mchp_dev_config_t *)dev->config)->regs->AC_CALIB =                    \
			(uint8_t)((((fuses_sw0_fuses_registers_t *)SW0_ADDR)->FUSES_SW0_WORD_0 &   \
				   FUSES_SW0_WORD_0_AC_BIAS0_Msk) >>                               \
				  FUSES_SW0_WORD_0_AC_BIAS0_Pos);                                  \
	}

/*
 * Define comparator device configuration structure for instance 'n' using devicetree properties.
 * Includes register base, pinctrl, clock subsystems, and static channel configuration.
 */
#define COMPARATOR_MCHP_CONFIG_DEFN(n)                                                             \
	static void comparator_mchp_config_##n(const struct device *dev);                          \
	static const comparator_mchp_dev_config_t comparator_mchp_cfg_##n = {                      \
		.regs = (ac_registers_t *)DT_INST_REG_ADDR(n),                                     \
		.config_func = comparator_mchp_config_##n,                                         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.comparator_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                  \
		.comparator_clock.mclk_sys =                                                       \
			(void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, subsystem)),                 \
		.comparator_clock.gclk_sys =                                                       \
			(void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, subsystem)),                 \
		.channel_config = {                                                                \
			.channel_id = DT_PROP_OR(DT_DRV_INST(n), comparator_channel, 0),           \
			.pos_input = DT_ENUM_IDX_OR(DT_DRV_INST(n), positive_mux_input, 0),        \
			.neg_input = DT_ENUM_IDX_OR(DT_DRV_INST(n), negative_mux_input, 0),        \
			.output_mode = DT_ENUM_IDX_OR(DT_DRV_INST(n), output_mode, 0),             \
			.interrupt_selection =                                                     \
				DT_ENUM_IDX_OR(DT_DRV_INST(n), interrupt_selection, 0),            \
			.filter_length = DT_ENUM_IDX_OR(DT_DRV_INST(n), filter_length, 0),         \
			.hysteresis_level = DT_ENUM_IDX_OR(DT_DRV_INST(n), hysteresis_level, 0),   \
			.vddana_scale_value = DT_PROP_OR(DT_DRV_INST(n), vddana_scale_value, 0),   \
			.interrupt_enable = DT_PROP_OR(DT_DRV_INST(n), interrupt_enable, 0),       \
			.single_shot_mode = DT_PROP_OR(DT_DRV_INST(n), single_shot_mode, 0),       \
			.hysteresis_enable = DT_PROP_OR(DT_DRV_INST(n), hysteresis_enable, 0),     \
			.run_standby = DT_PROP_OR(DT_DRV_INST(n), run_standby, 0),                 \
			.event_input_enable = DT_PROP_OR(DT_DRV_INST(n), event_input_enable, 0),   \
			.event_output_enable = DT_PROP_OR(DT_DRV_INST(n), event_output_enable, 0), \
			.swap_inputs = DT_PROP_OR(DT_DRV_INST(n), swap_inputs, 0),                 \
		}}

/*
 * Instantiate the comparator driver for device instance 'n'.
 * Sets up pinctrl, configuration, data, and registers it with Zephyr's device model.
 */
#define COMPARATOR_MCHP_DEVICE_INIT(n)                                                             \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	COMPARATOR_MCHP_CONFIG_DEFN(n);                                                            \
	COMPARATOR_MCHP_DATA_DEFN(n);                                                              \
	DEVICE_DT_INST_DEFINE(n, comparator_mchp_init, NULL, &comparator_mchp_data_##n,            \
			      &comparator_mchp_cfg_##n, POST_KERNEL,                               \
			      CONFIG_COMPARATOR_INIT_PRIORITY, &comparator_mchp_api);              \
	COMPARATOR_MCHP_DEFINE_CONFIG_FUNC(n);

DT_INST_FOREACH_STATUS_OKAY(COMPARATOR_MCHP_DEVICE_INIT);
