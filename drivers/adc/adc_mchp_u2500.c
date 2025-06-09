/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file adc_mchp_u2500.c
 * @brief ADC driver for Microchip ADC_U2500 peripheral.
 *
 * Implements Zephyr ADC API support with interrupt-driven sampling,
 * hardware gain/reference configuration, and context-managed reads.
 *
 * Supported SoC Families:
 * - SOC_FAMILY_MCHP_SAM_D5X_E5X
 */

#include <soc.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

/*******************************************
 * Constants and Macro Definitions
 *******************************************/

/* Compatibility string for device tree matching */
#define DT_DRV_COMPAT microchip_adc_u2500

/* Register a logging module for ADC driver with configured log level */
LOG_MODULE_REGISTER(adc_mchp_u2500, CONFIG_ADC_LOG_LEVEL);

/* Reference voltage of the ADC in millivolts, obtained from device tree */
#define ADC_MCHP_VREF_MV DT_INST_PROP(0, vref_mv)

/* Get device configuration structure from device pointer */
#define DEV_CFG(dev) ((const adc_mchp_dev_config_t *)(dev)->config)

/* Shortcut to access ADC registers from device configuration */
#define ADC_REGS ((const adc_mchp_dev_config_t *)(dev)->config)->regs

/* Calculate ADC sample length in ADC clock cycles given sampling time in nanoseconds,
 * ADC clock frequency in Hz, and prescaler value.
 * Formula: ((sampling_time_ns * adc_clk_hz) / (prescaler * 1,000,000,000)) - 1
 */
#define ADC_CALC_SAMPLEN_NS(sampling_time_ns, gclk_adc_hz, prescaler_val)                          \
	((((uint64_t)(sampling_time_ns) * (gclk_adc_hz)) / ((prescaler_val) * 1000000000ULL)) - 1)

/* Gain correction constants scaled by 2048 for fixed-point arithmetic */
#define ADC_GAIN_CORR_1_2 1024             /* 0.5 gain scaled */
#define ADC_GAIN_CORR_2_3 ((2 * 2048) / 3) /* 0.666... gain scaled */
#define ADC_GAIN_CORR_4_5 ((4 * 2048) / 5) /* 0.8 gain scaled */
#define ADC_GAIN_CORR_1   2048             /* 1.0 gain scaled */

/* ADC resolution options (in bits) */
#define ADC_RESOLUTION_8BIT  8
#define ADC_RESOLUTION_10BIT 10
#define ADC_RESOLUTION_12BIT 12

/* ADC clock prescaler division factors */
#define ADC_PRESCALER_DIV_2   2
#define ADC_PRESCALER_DIV_4   4
#define ADC_PRESCALER_DIV_8   8
#define ADC_PRESCALER_DIV_16  16
#define ADC_PRESCALER_DIV_32  32
#define ADC_PRESCALER_DIV_64  64
#define ADC_PRESCALER_DIV_128 128
#define ADC_PRESCALER_DIV_256 256

/* If the SUPC API support is not available, define direct access
 * to SUPC registers and shortcuts for voltage reference register
 */
#ifndef MCHP_SUPC_API_SUPPORT_AVAILABLE
static supc_registers_t *SUPC = (supc_registers_t *)SUPC_REGS;
#define SUPC_VREF (SUPC->SUPC_VREF)
#endif

/*******************************************
 * Enum and typedefs
 *******************************************/

/**
 * @brief ADC channel configuration.
 *
 * This structure holds the configuration parameters for an individual
 * ADC channel, including input selections, reference, and mode.
 */
typedef struct adc_mchp_channel_cfg {
	/** True if this ADC channel has been initialized. */
	bool initialized;

	/** Zephyr ADC channel configuration. */
	struct adc_channel_cfg channel_cfg;

} adc_mchp_channel_cfg_t;

/**
 * @brief Runtime driver data for ADC driver.
 *
 * This structure holds context and buffers required for ADC operation,
 * including the Zephyr ADC context, device reference, and sample buffers.
 */
typedef struct adc_mchp_dev_data {
	/** ADC context structure for managing sampling state and synchronization. */
	struct adc_context ctx;

	/** Pointer to the ADC device instance. */
	const struct device *dev;

	/** Pointer to the current position in the buffer where ADC samples are stored. */
	int16_t *buffer;

	/** Used to reset buffer position if needed after sampling advances. */
	int16_t *repeat_buffer;

	/** Configured channels (bitmask). */
	uint32_t channels;

	/** Currently selected channel ID. */
	uint8_t channel_id;

	/** Channel configuration array for all available ADC input channels. */
	adc_mchp_channel_cfg_t *channel_config;

} adc_mchp_dev_data_t;

/**
 * @brief Clock configuration structure for the ADC driver.
 *
 * This structure contains the clock configuration parameters for the ADC
 * peripheral.
 */
typedef struct mchp_adc_clock {

	/* Clock driver */
	const struct device *clock_dev;

	/* Main clock subsystem. */
	clock_control_mchp_subsys_t mclk_sys;

	/* Generic clock subsystem. */
	clock_control_mchp_subsys_t gclk_sys;

} mchp_adc_clock_t;

/**
 * @brief Static configuration structure for the ADC driver.
 *
 * Holds constant configuration data such as register base addresses,
 * pin control configuration, clock settings, frequency, prescaler,
 * channel count, and initialization function pointer.
 */
typedef struct adc_mchp_dev_config {

	/** ADC Registers */
	adc_registers_t *regs;

	/** Pointer to the pin control configuration for the ADC device. */
	const struct pinctrl_dev_config *pcfg;

	/** Clock configuration for the ADC peripheral. */
	mchp_adc_clock_t adc_clock;

	/** ADC operating frequency in Hz. After div */
	uint32_t freq;

	/** Clock prescaler value for the ADC input clock. */
	uint16_t prescaler;

	/** Maximum number od ADC channels. */
	uint8_t num_channels;

	/**
	 * @brief Function pointer for ADC peripheral initialization.
	 *
	 * This is typically set during device instantiation to configure
	 * peripheral-specific interrupts and calibration values.
	 *
	 * @param dev Pointer to the ADC device instance.
	 */
	void (*config_func)(const struct device *dev);
} adc_mchp_dev_config_t;

/*******************************************
 * Helper functions
 *******************************************/

/* Wait for synchronization */
static inline void adc_wait_synchronization(adc_registers_t *adc_reg)
{
	while ((adc_reg->ADC_SYNCBUSY & ADC_SYNCBUSY_Msk) != 0) {
	}
}

/* Enable ADC interrupt. */
static inline void adc_interrupt_enable(adc_registers_t *adc_reg, uint8_t interrupt_flag)
{
	/* Enable the RESRDY Interrupt */
	adc_reg->ADC_INTENSET |= ADC_INTENSET_RESRDY(1);
}

/* Clear interrupt. */
static inline void adc_interrupt_clear(adc_registers_t *adc_reg)
{
	/* Clear the interrupt */
	adc_reg->ADC_INTFLAG = ADC_INTFLAG_RESRDY_Msk;
}

/* Enable or disable the ADC controller. */
static inline void adc_controller_enable(adc_registers_t *adc_reg, bool enable)
{
	if (enable == true) {
		adc_reg->ADC_CTRLA |= ADC_CTRLA_ENABLE(1);
	} else {
		adc_reg->ADC_CTRLA &= ~ADC_CTRLA_ENABLE_Msk;
	}
}

/* ADC Correction enable */
static inline void adc_correction_enable(adc_registers_t *adc_reg)
{
	adc_reg->ADC_CTRLB |= ADC_CTRLB_CORREN(1);
	adc_wait_synchronization(adc_reg);
}

/* Set ADC Correction OFFSET */
static inline void adc_set_offset_correction(adc_registers_t *adc_reg, int16_t offset_corr)
{
	adc_reg->ADC_OFFSETCORR = ADC_OFFSETCORR_OFFSETCORR(offset_corr);
	adc_wait_synchronization(adc_reg);
}

/* Trigger an ADC conversion. */
static inline void adc_trigger_conversion(adc_registers_t *adc_reg)
{
	adc_reg->ADC_SWTRIG |= ADC_SWTRIG_START(1);
}

/* Read the result of the ADC conversion. */
static inline uint16_t adc_get_conversion_result(adc_registers_t *adc_reg)
{
	return adc_reg->ADC_RESULT;
}

/**
 * @brief Set ADC acquisition time in terms of sample length (clock cycles).
 *
 * This function configures the ADC sample control register to control the
 * acquisition time. The sample length must be within the valid range [0, 63].
 *
 * @param[in,out] adc_reg Pointer to ADC registers.
 * @param[in] sample_length Sample length in ADC clock cycles (max 63).
 *
 * @retval 0 on success.
 * @retval -EINVAL if sample_length is out of valid range.
 */
static inline int8_t adc_set_acq_time(adc_registers_t *adc_reg, uint16_t sample_length)
{
	int ret = 0;

	/* Valid sample length is in the range [0, 63] */
	if (sample_length >= 64U) {
		ret = -EINVAL;
	}

	if (ret == 0) {
		adc_reg->ADC_SAMPCTRL = ADC_SAMPCTRL_SAMPLEN(sample_length);
		adc_wait_synchronization(adc_reg);
	}
	return ret;
}

/**
 * @brief Validate ADC channel configuration parameters.
 *
 * This function checks the validity of ADC gain, reference, sample length,
 * and input channel selections for both positive and negative inputs.
 *
 * @param gain           ADC gain setting (enum adc_gain).
 *                       Valid values: ADC_GAIN_1_2, ADC_GAIN_2_3, ADC_GAIN_4_5, ADC_GAIN_1.
 * @param reference      ADC reference voltage source (enum adc_reference).
 *                       Valid range: ADC_REF_VDD_1 to ADC_REF_EXTERNAL1.
 * @param sample_length  Sampling time length. Valid range: 0 to 63.
 * @param input_positive Positive ADC input channel.
 *                       Valid: ≤ 0x1E and not in the range 0x10 to 0x17 (reserved).
 * @param input_negative Negative ADC input channel.
 *                       Valid: 0 to 7.
 *
 * @return 0 on success, -EINVAL if any parameter is invalid.
 */
static int8_t adc_validate_channel_params(enum adc_gain gain, enum adc_reference reference,
					  uint16_t sample_length, uint8_t input_positive,
					  uint8_t input_negative)
{
	int8_t ret = 0;

	/* validate gain */
	if (gain != ADC_GAIN_1_2 && gain != ADC_GAIN_2_3 && gain != ADC_GAIN_4_5 &&
	    gain != ADC_GAIN_1) {
		ret = -EINVAL;
	}
	/* Validate reference */
	if (reference < ADC_REF_VDD_1 || reference > ADC_REF_EXTERNAL1) {
		ret = -EINVAL;
	}
	/* Valid sample length is in the range [0, 63] */
	if (sample_length >= 64U) {
		ret = -EINVAL;
	}
	/*
	 * Validate input_positive:
	 * - Must be <= 0x1E (highest valid value is DAC0)
	 * - Values 0x10 to 0x17 are reserved and must not be used
	 */
	if (input_positive > 0x1E || (input_positive >= 0x10 && input_positive <= 0x17)) {
		ret = -EINVAL;
	}
	/* Validate the input_negative range: must be within 0–7 */
	if (input_negative > 7) {
		ret = -EINVAL;
	}

	return ret;
}

/**
 * @brief Set the ADC reference source.
 *
 * Configures the ADC_REFCTRL register based on the selected reference and enables internal
 * references if needed.
 *
 * @param adc_reg Pointer to ADC registers.
 * @param reference Reference selection.
 * @return 0 on success, -EINVAL if the reference is invalid.
 */
static int8_t adc_set_reference(adc_registers_t *adc_reg, enum adc_reference reference)
{
	uint8_t refctrl = 0;
	int8_t ret = 0;

	switch (reference) {
	case ADC_REF_VDD_1:
		refctrl = ADC_REFCTRL_REFSEL_INTVCC1 | ADC_REFCTRL_REFCOMP(1);
		break;
	case ADC_REF_VDD_1_2:
		refctrl = ADC_REFCTRL_REFSEL_INTVCC0 | ADC_REFCTRL_REFCOMP(1);
		break;
	case ADC_REF_INTERNAL:
		refctrl = ADC_REFCTRL_REFSEL_INTREF | ADC_REFCTRL_REFCOMP(1);
		break;
	case ADC_REF_EXTERNAL0:
		refctrl = ADC_REFCTRL_REFSEL_AREFA;
		break;
	case ADC_REF_EXTERNAL1:
		refctrl = ADC_REFCTRL_REFSEL_AREFB;
		break;
	case ADC_REF_VDD_1_3:
		ret = -EINVAL;
		break;
	case ADC_REF_VDD_1_4:
		ret = -EINVAL;
		break;
	default:
		LOG_ERR("ADC Selected reference is not valid : %d\n", reference);
		ret = -EINVAL;
	}
	/* Assign the value to the ADC_REFCTRL Register */
	if (ret == 0) {
		adc_reg->ADC_REFCTRL = refctrl;
		adc_wait_synchronization(adc_reg);
	}

#ifndef MCHP_SUPC_API_SUPPORT_AVAILABLE
	/* Enable internal references manually if SUPC API support is not available */
	/* Enable Bandgap reference and select 2.5V output */
	SUPC_VREF |= SUPC_VREF_SEL_2V4;
#else
	/* Use MCHP SUPC API to enable and select the internal bandgap voltage reference */
	/* It may done in the user application, in that case don't do anything here */
#endif

	return ret;
}

/**
 * @brief Configure the ADC gain and apply gain correction if needed.
 *
 * Sets the gain correction registers based on the selected gain value.
 *
 * @param adc_reg Pointer to ADC registers.
 * @param gain ADC gain selection.
 * @return 0 on success, -EINVAL if the gain is invalid.
 */
static int8_t adc_set_gain(adc_registers_t *adc_reg, enum adc_gain gain)
{
	int8_t ret = 0;
	uint16_t gain_corr = 0;

	/* Check if the gain is ADC_GAIN_1 (no correction needed) */
	if (gain == ADC_GAIN_1) {
		/* No need to enable Gain correction as the gain is 1 */
	} else {
		/* Enable Gain correction */
		adc_correction_enable(adc_reg);

		/* Set default offset correction value (zero offset) */
		adc_set_offset_correction(adc_reg, 0x0);

		/* Set the correct gain correction value based on the selected gain */
		switch (gain) {
		case ADC_GAIN_1_2:
			/* For 1/2 gain, the correction factor is 1024 */
			gain_corr = ADC_GAIN_CORR_1_2;
			break;
		case ADC_GAIN_2_3:
			/* For 2/3 gain, calculate the correction factor: (2 * 2048) / 3 */
			gain_corr = ADC_GAIN_CORR_2_3;
			break;
		case ADC_GAIN_4_5:
			/* For 4/5 gain, calculate the correction factor: (4 * 2048) / 5 */
			gain_corr = ADC_GAIN_CORR_4_5;
			break;
		case ADC_GAIN_1:
			/* Default case (gain of 1), the correction factor is 2048 */
			gain_corr = ADC_GAIN_CORR_1;
			break;
		default:
			/* Invalid gain value provided, return error code */
			ret = -EINVAL;
		}

		/* Apply the calculated gain correction if no error occurred */
		if (ret == 0) {
			/* Set the gain correction value in the ADC register */
			adc_reg->ADC_GAINCORR = ADC_GAINCORR_GAINCORR(gain_corr);

			/* Wait for the gain correction to be synchronized */
			adc_wait_synchronization(adc_reg);
		}
	}

	return ret;
}

/**
 * @brief Set the ADC positive input channel.
 *
 * Validates and configures the ADC positive input multiplexer and
 * manages SUPC internal reference settings if applicable.
 *
 * @param adc_reg Pointer to ADC registers.
 * @param input_positive Positive input channel selection.
 * @return 0 on success, -EINVAL if the input is invalid.
 */
static int8_t adc_set_input_positive(adc_registers_t *adc_reg, uint8_t input_positive)
{
	int8_t ret = 0;

	/*
	 * Validate input_positive:
	 * - Must be <= 0x1E (highest valid value is DAC0)
	 * - Values 0x10 to 0x17 are reserved and must not be used
	 */
	if (input_positive > 0x1E || (input_positive >= 0x10 && input_positive <= 0x17)) {
		ret = -EINVAL;
	}

	/* Set the MUXPOS field in ADC_INPUTCTRL register to select positive input channel */
	adc_reg->ADC_INPUTCTRL = ADC_INPUTCTRL_MUXPOS(input_positive);

#ifndef MCHP_SUPC_API_SUPPORT_AVAILABLE
	/* If no SUPC API, configure internal references manually when applicable */
	switch (input_positive) {
#ifdef ADC_INPUTCTRL_MUXPOS_TEMP_Val
	case ADC_INPUTCTRL_MUXPOS_TEMP_Val:
		/* Temperature sensor – no explicit SUPC config required */
		break;
#endif

#ifdef ADC_INPUTCTRL_MUXPOS_PTAT_Val
	case ADC_INPUTCTRL_MUXPOS_PTAT_Val:
		/* Enable TSEN and select PTAT */
		SUPC_VREF |= SUPC_VREF_TSEN(1);
		SUPC_VREF &= ~SUPC_VREF_TSSEL_Msk; /* TSSEL = 0 selects PTAT */
		break;
#endif

#ifdef ADC_INPUTCTRL_MUXPOS_CTAT_Val
	case ADC_INPUTCTRL_MUXPOS_CTAT_Val:
		/* Enable TSEN and select CTAT */
		SUPC_VREF |= SUPC_VREF_TSEN(1);
		SUPC_VREF |= SUPC_VREF_TSSEL(1); /* TSSEL = 1 selects CTAT */
		break;
#endif

	case ADC_INPUTCTRL_MUXPOS_BANDGAP_Val:
		/* Bandgap reference selected.
		 * Enable 2.4V bandgap output via SUPC.
		 */
		SUPC_VREF |= SUPC_VREF_VREFOE(1); /* Enable VREF output */
		SUPC_VREF |= SUPC_VREF_SEL_2V4;   /* Select 2.4V output */
		break;

	default:
		/* No SUPC configuration needed for other inputs */
		break;
	}
#else
	/* SUPC configuration is handled via MCHP SUPC API externally */
#endif

	return ret;
}

/**
 * @brief Configure the ADC negative input channel and differential mode.
 *
 * Validates the negative input and sets the ADC to differential or single-ended mode.
 *
 * @param adc_reg Pointer to ADC registers.
 * @param input_negative Negative input channel selection.
 * @param differential Enable differential mode if true; else single-ended mode.
 * @return 0 on success, -EINVAL if the input is invalid.
 */
static int8_t adc_set_input_negative(adc_registers_t *adc_reg, uint8_t input_negative,
				     bool differential)
{
	int8_t ret = 0;

	/* Validate input_negative range (only valid for differential mode) */
	if (input_negative > 7) {
		ret = -EINVAL;
	}
	/* Set the ADC_INPUTCTRL Register */
	if (ret == 0) {
		if (differential == true) {
			/* Enable differential mode and set specified negative input */
			adc_reg->ADC_INPUTCTRL |= ADC_INPUTCTRL_DIFFMODE(1);
			adc_reg->ADC_INPUTCTRL |= ADC_INPUTCTRL_MUXNEG(input_negative);
		} else {
			/* In single-ended mode, connect negative input to GND */
			adc_reg->ADC_INPUTCTRL |=
				ADC_INPUTCTRL_MUXNEG(ADC_INPUTCTRL_MUXNEG_GND_Val);
		}
	}

	return ret;
}

/**
 * @brief Configure ADC oversampling ratio.
 *
 * Sets the number of samples to average per conversion for improved resolution.
 *
 * @param adc_reg Pointer to ADC registers.
 * @param oversampling Oversampling setting (0 to 10, corresponding to 1 to 1024 samples).
 * @return 0 on success, -EINVAL if the oversampling value is invalid.
 */
static int8_t adc_set_oversampling(adc_registers_t *adc_reg, uint8_t oversampling)
{
	int8_t ret = 0;

	/*
	 * Oversampling configuration:
	 * 0x0 = 1 sample
	 * 0x1 = 2 samples
	 * 0x2 = 4 samples
	 * 0x3 = 8 samples
	 * 0x4 = 16 samples
	 * 0x5 = 32 samples
	 * 0x6 = 64 samples
	 * 0x7 = 128 samples
	 * 0x8 = 256 samples
	 * 0x9 = 512 samples
	 * 0xA = 1024 samples
	 *
	 * Valid range: 0 to 10 (inclusive)
	 */
	if (oversampling <= 10) {
		adc_reg->ADC_SWTRIG = ADC_AVGCTRL_SAMPLENUM(oversampling);
		adc_wait_synchronization(adc_reg);
	} else {
		ret = -EINVAL;
	}
	return ret;
}

/**
 * @brief Set ADC resolution considering oversampling.
 *
 * Configures ADC resolution and validates compatibility with oversampling settings.
 *
 * @param adc_reg Pointer to ADC registers.
 * @param resolution Desired ADC resolution (8, 10, or 12-bit).
 * @param oversampling Oversampling setting; affects resolution options.
 * @return 0 on success, -EINVAL if parameters are invalid.
 */
static int8_t adc_set_resolution(adc_registers_t *adc_reg, uint8_t resolution, uint8_t oversampling)
{
	int8_t ret = 0;
	uint16_t resolution_val = 0;

	switch (resolution) {
	case ADC_RESOLUTION_8BIT:
		if (oversampling != 0) {
			ret = -EINVAL;
		} else {
			resolution_val = ADC_CTRLB_RESSEL_8BIT;
		}
		break;
	case ADC_RESOLUTION_10BIT:
		if (oversampling != 0) {
			ret = -EINVAL;
		} else {
			resolution_val = ADC_CTRLB_RESSEL_10BIT;
		}
		break;
		break;
	case ADC_RESOLUTION_12BIT:
		if (oversampling) {
			resolution_val = ADC_CTRLB_RESSEL_16BIT;
		} else {
			resolution_val = ADC_CTRLB_RESSEL_12BIT;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}
	if (ret == 0) {
		adc_reg->ADC_CTRLB = resolution_val;
		adc_wait_synchronization(adc_reg);
	}

	return ret;
}

/**
 * @brief Configure the ADC clock prescaler.
 *
 * Sets the prescaler to divide the input clock, with fallback to default if invalid.
 *
 * @param adc_reg Pointer to ADC registers.
 * @param prescaler Prescaler division factor (must be power of 2 between 2 and 256).
 * @return 0 always (fallback applied if invalid).
 */
static int8_t adc_set_prescalar(adc_registers_t *adc_reg, uint16_t prescaler)
{
	int8_t ret = 0;
	uint16_t prescaler_val = 0;

	switch (prescaler) {
	case ADC_PRESCALER_DIV_2:
		prescaler_val = ADC_CTRLA_PRESCALER_DIV2;
		break;
	case ADC_PRESCALER_DIV_4:
		prescaler_val = ADC_CTRLA_PRESCALER_DIV4;
		break;
	case ADC_PRESCALER_DIV_8:
		prescaler_val = ADC_CTRLA_PRESCALER_DIV8;
		break;
	case ADC_PRESCALER_DIV_16:
		prescaler_val = ADC_CTRLA_PRESCALER_DIV16;
		break;
	case ADC_PRESCALER_DIV_32:
		prescaler_val = ADC_CTRLA_PRESCALER_DIV32;
		break;
	case ADC_PRESCALER_DIV_64:
		prescaler_val = ADC_CTRLA_PRESCALER_DIV64;
		break;
	case ADC_PRESCALER_DIV_128:
		prescaler_val = ADC_CTRLA_PRESCALER_DIV128;
		break;
	case ADC_PRESCALER_DIV_256:
		prescaler_val = ADC_CTRLA_PRESCALER_DIV256;
		break;
	default:
		/*
		 * Fall back to default prescaler if the provided value is invalid.
		 * Acceptable values must be powers of 2 within the range 1–256.
		 * (prescaler <= 0 || prescaler > 256 || (prescaler & (prescaler - 1)))
		 * Notify user of fallback to ensure the ADC doesn't fail to initialize.
		 */
		prescaler_val = ADC_CTRLA_PRESCALER_DIV2;
		LOG_WRN("Warning: Invalid ADC prescaler value %d, using default (DIV2).\n",
			prescaler);
		break;
	}
	if (ret == 0) {
		adc_reg->ADC_CTRLA = prescaler_val;
		adc_wait_synchronization(adc_reg);
	}

	return ret;
}

/**
 * @brief Select and configure ADC input channels.
 *
 * This function sets the positive and negative input channels for the ADC
 * based on the configuration provided in @p channel_config.
 *
 * It configures the MUX settings required for both single-ended and
 * differential ADC modes as needed.
 *
 * @param[in] dev Pointer to the ADC device structure.
 * @param[in] channel_config Pointer to the ADC channel configuration structure.
 *
 * @retval 0 If the input channels were configured successfully.
 * @retval -EINVAL If an invalid input configuration is detected.
 */
static int8_t adc_select_channels(const struct device *dev, struct adc_channel_cfg *channel_config)
{

	int8_t ret = 0;

	do {
		/* Set channels */
		ret = adc_set_input_positive(ADC_REGS, channel_config->input_positive);
		if (ret != 0) {
			LOG_ERR("Invalid input_positive : %d\n", channel_config->input_positive);
			break;
		}

		/* Set channels */
		ret = adc_set_input_negative(ADC_REGS, channel_config->input_negative,
					     channel_config->differential);
		if (ret != 0) {
			LOG_ERR("Invalid input_negative : %d\n", channel_config->input_negative);
			break;
		}
	} while (0);

	return ret;
}

/**
 * @brief Calculate the ADC sample length based on acquisition time and clock settings.
 *
 * This function computes the sample length in ADC clock cycles given the acquisition time,
 * ADC clock frequency, and prescaler value. The acquisition time unit can be in ticks,
 * microseconds, or nanoseconds.
 *
 * @param[in] acq_time Acquisition time encoded with unit and value.
 * @param[in] adc_clk ADC clock frequency in Hz.
 * @param[in] prescalar ADC prescaler value.
 *
 * @return The sample length in ADC clock cycles.
 *         Returns 0 if the acquisition time unit is unsupported or default.
 */
static int adc_get_sample_length(uint16_t acq_time, uint32_t adc_clk, uint8_t prescalar)
{
	uint16_t sample_length = 0;

	switch (ADC_ACQ_TIME_UNIT(acq_time)) {
	case ADC_ACQ_TIME_TICKS:
		sample_length = ADC_ACQ_TIME_VALUE(acq_time) - 1;
		break;
	case ADC_ACQ_TIME_MICROSECONDS:
		sample_length = (uint16_t)ADC_CALC_SAMPLEN_NS(
			((ADC_ACQ_TIME_VALUE(acq_time)) * 1000), adc_clk, prescalar);
		break;
	case ADC_ACQ_TIME_NANOSECONDS:
		sample_length = (uint16_t)ADC_CALC_SAMPLEN_NS(ADC_ACQ_TIME_VALUE(acq_time), adc_clk,
							      prescalar);
		break;
	default:
		/* Unsupported acquisition time unit or ADC_ACQ_TIME_DEFAULT */
		sample_length = 0;
		break;
	}

	return sample_length;
}

/**
 * @brief Apply ADC channel-specific configuration.
 *
 * This function configures acquisition time, gain, and reference voltage
 * settings for a given ADC channel.
 *
 * @param[in] dev Pointer to the ADC device structure.
 * @param[in] channel_config Pointer to the ADC channel configuration structure.
 *
 * @retval 0 on success.
 * @retval -EINVAL if any of the settings are invalid.
 *
 * @note This function assumes that the hardware supports per-channel configuration.
 *       Validation of such support should be done before calling this function.
 */
static int8_t adc_apply_channel_config(const struct device *dev,
				       struct adc_channel_cfg *channel_config)
{
	int8_t ret = 0;
	/* ADC Device config */
	const adc_mchp_dev_config_t *const dev_cfg = DEV_CFG(dev);

	do {
		/* Set Acquisition time */
		uint16_t sample_length = adc_get_sample_length(channel_config->acquisition_time,
							       dev_cfg->freq, dev_cfg->prescaler);

		/* Set sample length*/
		ret = adc_set_acq_time(ADC_REGS, sample_length);
		if (ret < 0) {
			LOG_ERR("Selected ADC acquisition time is not valid");
			break;
		}

		/* Set gain */
		ret = adc_set_gain(ADC_REGS, channel_config->gain);
		if (ret < 0) {
			LOG_ERR("Invalid gain value: %d", channel_config->gain);
			break;
		}

		/* Set Reference */
		ret = adc_set_reference(ADC_REGS, channel_config->reference);
		if (ret < 0) {
			LOG_ERR("Invalid reference: %d", channel_config->reference);
			break;
		}

	} while (0);

	return ret;
}

/**
 * @brief Start a single ADC conversion.
 *
 * This function triggers a software start for an ADC conversion on the
 * specified device. It does not wait for the conversion to complete,
 * assuming that subsequent operations will handle synchronization.
 *
 * @param dev Pointer to the ADC device structure.
 */
static void adc_start_channel(const struct device *dev)
{
	/* Retrieve ADC device-specific runtime data */
	adc_mchp_dev_data_t *dev_data = dev->data;
	adc_mchp_channel_cfg_t *channel_config = NULL;

	/* Determine the next channel to process by finding the least significant bit set */
	dev_data->channel_id = find_lsb_set(dev_data->channels) - 1;

	/* Get the configuration for the selected channel */
	channel_config = &dev_data->channel_config[dev_data->channel_id];

	/* Channel configuration has already been validated earlier,
	 * so no need for error checking here.
	 */

	/* Apply configuration for the selected channel */
	adc_apply_channel_config(dev, &channel_config->channel_cfg);

	/* Select the channel(s) to be used in this conversion */
	adc_select_channels(dev, &channel_config->channel_cfg);

	/* Start the ADC conversion */
	adc_trigger_conversion(ADC_REGS);
}

/**
 * @brief Check if the provided buffer is large enough for the ADC sequence.
 *
 * This function calculates the required buffer size based on the number of
 * active ADC channels and any extra samplings specified in the sequence options.
 * It verifies that the user-provided buffer is sufficient to store all expected samples.
 *
 * @param[in] sequence Pointer to the ADC sequence structure that describes the sampling
 * configuration.
 * @param[in] active_channels Number of active ADC channels that will be sampled.
 *
 * @retval 0 If the provided buffer size is sufficient.
 * @retval -ENOMEM If the provided buffer is too small.
 */
static int adc_check_buffer_size(const struct adc_sequence *sequence, uint8_t active_channels)
{
	size_t needed_buffer_size;

	needed_buffer_size = active_channels * sizeof(uint16_t);
	if (sequence->options) {
		needed_buffer_size *= (1U + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed_buffer_size) {
		LOG_ERR("Provided buffer is too small (%u/%u)", sequence->buffer_size,
			needed_buffer_size);
		return -ENOMEM;
	}
	return 0;
}

/**
 * @brief Start an ADC read operation
 *
 * This function initiates a read of a single ADC input channel as defined in
 * the provided `adc_sequence` structure. It configures oversampling, resolution,
 * and validates the input parameters.
 *
 * Key actions performed:
 * - Configures oversampling settings via `AVGCTRL` register
 * - Adjusts result resolution based on configuration
 * - Validates channel selection (single channel only supported)
 * - Sets up internal buffer references for result storage
 * - Starts the ADC context read and waits for completion
 *
 * Limitations:
 * - Only one ADC channel can be sampled per call (channel scanning is not supported)
 * - Oversampling is only supported with 12-bit resolution
 *
 * @param dev Pointer to the device structure for the ADC instance
 * @param sequence Pointer to the adc_sequence structure defining the read operation
 *
 * @retval 0 If successful
 * @retval -EINVAL If configuration values are invalid (e.g., unsupported resolution or
 * oversampling)
 * @retval -ENOTSUP If channel scanning is attempted
 * @retval Negative errno code on buffer validation or context wait failure
 *
 * @note The actual sampling and conversion are done asynchronously using the ADC
 * context APIs, but the function blocks until the results are ready.
 */
static int adc_start_read(const struct device *dev, const struct adc_sequence *sequence)
{
	/* ADC Device config */
	const adc_mchp_dev_config_t *const dev_cfg = DEV_CFG(dev);
	/* ADC Device data*/
	adc_mchp_dev_data_t *dev_data = dev->data;
	int ret = 0;
	uint32_t channels = 0, channel_count = 0, index = 0;

	do {
		if (sequence->channels == 0) {
			LOG_ERR("No chennels selected!\n");
			ret = -EINVAL;
			break;
		}

		/* Set oversampling */
		ret = adc_set_oversampling(ADC_REGS, sequence->oversampling);
		if (ret != 0) {
			LOG_ERR("Invalid oversampling : %d\n", sequence->oversampling);
			break;
		}

		/* Set Resolution */
		ret = adc_set_resolution(ADC_REGS, sequence->resolution, sequence->oversampling);
		if (ret != 0) {
			LOG_ERR("Invalid resolution : %d\n", sequence->resolution);
			break;
		}

		/* Verify all requested channels are initialized and store resolution */
		channels = sequence->channels;
		channel_count = 0;
		while (channels) {
			/* Iterate through all channels and check if they are initialized */
			index = find_lsb_set(channels) - 1;
			if (index >= dev_cfg->num_channels) {
				LOG_ERR("Invalid channel number : %d", index);
				ret = -EINVAL;
				break;
			}
			/* If the channels is not initialized return invalid */
			if (dev_data->channel_config[index].initialized == false) {
				LOG_ERR("Channel is not initialized");
				ret = -EINVAL;
				break;
			}
			channel_count++;
			channels &= ~BIT(index);
		}
		/* Break if any error occurred */
		if (ret != 0) {
			break;
		}

		/* Check buffer */
		ret = adc_check_buffer_size(sequence, channel_count);
		if (ret != 0) {
			LOG_ERR("Check buffer size invalid\n");
			break;
		}

		/* Store buffer references for use during sampling */
		dev_data->buffer = sequence->buffer;
		dev_data->repeat_buffer = sequence->buffer;

		/* At this point we allow the scheduler to do other things while
		 * we wait for the conversions to complete. This is provided by the
		 * adc_context functions. However, the caller of this function is
		 * blocked until the results are in.
		 */
		adc_context_start_read(&dev_data->ctx, sequence);

		/* Wait for all ADC conversions to complete before returning, if it's a synchronous
		 * call
		 */
		ret = adc_context_wait_for_completion(&dev_data->ctx);

	} while (0);

	return ret;
}

/**
 * @brief Start ADC sampling using the ADC context.
 *
 * This function retrieves the device-specific data structure from the
 * provided ADC context and initiates a single ADC conversion.
 *
 * @param ctx Pointer to the ADC context structure.
 */
static void adc_context_start_sampling(struct adc_context *ctx)
{
	adc_mchp_dev_data_t *dev_data = CONTAINER_OF(ctx, adc_mchp_dev_data_t, ctx);

	dev_data->channels = ctx->sequence.channels;

	adc_start_channel(dev_data->dev);
}

/**
 * @brief Update the ADC buffer pointer for repeated sampling.
 *
 * If repeated sampling is enabled, this function resets the buffer pointer
 * to the beginning of the repeat buffer for the next sampling cycle.
 *
 * @param ctx Pointer to the ADC context structure.
 * @param repeat_sampling Boolean indicating whether sampling is repeated.
 */
static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	adc_mchp_dev_data_t *data = CONTAINER_OF(ctx, adc_mchp_dev_data_t, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

/*******************************************
 * ADC ISR Handler
 *******************************************/

/**
 * @brief Interrupt Service Routine for ADC conversion complete.
 *
 * This function handles the ADC interrupt triggered when a conversion result
 * is ready. It reads the result from the hardware register and stores it into
 * the sample buffer.
 *
 * @param dev Pointer to the ADC device structure.
 */
static void adc_mchp_isr(const struct device *dev)
{
	/* Get ADC Device Data */
	adc_mchp_dev_data_t *dev_data = dev->data;

	uint16_t result = 0;

	adc_interrupt_clear(ADC_REGS);

	result = adc_get_conversion_result(ADC_REGS);

	*dev_data->buffer++ = result;

	dev_data->channels &= ~BIT(dev_data->channel_id);

	if (dev_data->channels != 0) {
		/* If multiple channels are configured, continue sampling the next channel */
		adc_start_channel(dev);
	} else {
		/* If no additional channels, notify that sampling is complete */
		adc_context_on_sampling_done(&dev_data->ctx, dev);
	}
}

/*******************************************
 * Implementation of Zephyr APIs
 *******************************************/

/**
 * @brief Configure an ADC channel
 *
 * This function sets up an ADC input channel according to the specified
 * configuration parameters, including acquisition time, reference voltage,
 * gain, and input pins (positive/negative). It also handles internal references
 * and temperature sensor enablement if needed.
 *
 * The configuration includes:
 * - Acquisition time conversion to sample clocks
 * - Reference voltage selection (internal, external, or VDD-based)
 * - Gain configuration for the input signal
 * - Differential or single-ended mode setup
 * - Internal reference enabling (e.g., bandgap, temperature sensors)
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param channel_cfg Pointer to the structure containing channel configuration
 *
 * @retval 0 If successful
 * @retval -EINVAL If an invalid configuration is provided
 * @retval Negative errno code on failure
 *
 * @note This function includes hardware synchronization points to ensure proper
 * register access. It also manages ADC enable/disable during protected register
 * writes if required by the hardware (e.g., for reference switching).
 */
static int adc_mchp_channel_setup(const struct device *dev,
				  const struct adc_channel_cfg *channel_cfg)
{
	/* ADC Device data*/
	adc_mchp_dev_data_t *dev_data = dev->data;
	/* Get device config */
	const adc_mchp_dev_config_t *const dev_cfg = DEV_CFG(dev);
	/* ADC Channel config */
	adc_mchp_channel_cfg_t *channel_config = NULL;
	int8_t ret = 0;
	uint16_t sample_length = 0;

	do {
		/* Validate channel id */
		if (channel_cfg->channel_id >= dev_cfg->num_channels) {
			LOG_ERR("Invalid Channel id : %d\n", channel_cfg->channel_id);
			ret = -EINVAL;
			break;
		}
		/* Update the channel configuration */
		channel_config = &dev_data->channel_config[channel_cfg->channel_id];
		channel_config->initialized = false;

		/* Calculate sample length in ADC clock cycles from acquisition time */
		sample_length = adc_get_sample_length(channel_cfg->acquisition_time, dev_cfg->freq,
						      dev_cfg->prescaler);

		/*
		 * If the device supports individual channel configuration,
		 * it can be configured during channel sequencing.
		 * Validate the channel configuration parameters accordingly.
		 */
		ret = adc_validate_channel_params(channel_cfg->gain, channel_cfg->reference,
						  sample_length, channel_cfg->input_positive,
						  channel_cfg->input_negative);

		if (ret != 0) {
			LOG_ERR("Invalid ADC channel config");
			break;
		}

		/* Add the channel config to the channel_config array */
		channel_config->channel_cfg = *channel_cfg;
		channel_config->initialized = true;

	} while (0);

	return ret;
}

/**
 * @brief Initiate an ADC read operation
 *
 * This function serves as the main entry point for performing ADC conversions.
 * It wraps the lower-level `adc_start_read()` function with proper ADC context
 * locking and releasing mechanisms to ensure safe and synchronized access
 * to the ADC hardware.
 *
 * @param dev Pointer to the device structure for the ADC instance
 * @param sequence Pointer to the adc_sequence structure that defines the read parameters
 *
 * @retval 0 If the read operation completed successfully
 * @retval Negative errno code in case of an error during the read
 *
 * @note This function locks the ADC context before starting the read, and
 * releases it once the operation is complete.
 */
static int adc_mchp_read(const struct device *dev, const struct adc_sequence *sequence)
{
	adc_mchp_dev_data_t *data = dev->data;
	int ret = 0;

	adc_context_lock(&data->ctx, false, NULL);
	ret = adc_start_read(dev, sequence);
	adc_context_release(&data->ctx, ret);

	return ret;
}

#ifdef CONFIG_ADC_ASYNC
/**
 * @brief Perform an asynchronous ADC read operation.
 *
 * This function initiates a non-blocking ADC read using the provided
 * sequence and signals completion using a kernel poll signal.
 *
 * It locks the ADC context, starts the read operation, and then
 * releases the context with the result.
 *
 * @param dev Pointer to the ADC device.
 * @param sequence Pointer to the ADC sequence configuration.
 * @param async Pointer to a kernel poll signal for asynchronous notification.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int adc_mchp_read_async(const struct device *dev, const struct adc_sequence *sequence,
			       struct k_poll_signal *async)
{
	adc_mchp_dev_data_t *data = dev->data;

	int ret = 0;

	adc_context_lock(&data->ctx, true, async);
	ret = adc_start_read(dev, sequence);
	adc_context_release(&data->ctx, ret);

	return ret;
}
#endif

/**
 * @brief Initialize the ADC peripheral for MCHP devices.
 *
 * This function sets up the clocking, pin configuration, interrupt settings,
 * and hardware registers needed to enable the ADC. It is typically called
 * during system initialization by the device driver framework.
 *
 * @param dev Pointer to the ADC device structure.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int adc_mchp_init(const struct device *dev)
{
	/* Get device config */
	const adc_mchp_dev_config_t *const dev_cfg = DEV_CFG(dev);
	/* Get device data */
	adc_mchp_dev_data_t *dev_data = dev->data;
	int8_t ret = 0;

	/* Enable the ADC Clock */
	LOG_DBG("Clock dev: %p, gclk id: %d, mclk id: %d", dev_cfg->adc_clock.clock_dev,
		dev_cfg->adc_clock.gclk_sys.id, dev_cfg->adc_clock.mclk_sys.id);
	do {
		/* On Global clock for ADC */
		ret = clock_control_on(dev_cfg->adc_clock.clock_dev,
				       (clock_control_subsys_t)&dev_cfg->adc_clock.gclk_sys);
		if (ret != 0) {
			LOG_ERR("Failed to enable the GCLK for ADC: %d", ret);
			break;
		}
		/* On Main clock for ADC */
		ret = clock_control_on(dev_cfg->adc_clock.clock_dev,
				       (clock_control_subsys_t)&dev_cfg->adc_clock.mclk_sys);
		if (ret != 0) {
			LOG_ERR("Failed to enable the MCLK for ADC: %d", ret);
			break;
		}

		/* Get ADC Clock Frequency */
		ret = clock_control_get_rate(
			((const adc_mchp_dev_config_t *)(dev->config))->adc_clock.clock_dev,
			&(((adc_mchp_dev_config_t *)(dev->config))->adc_clock.mclk_sys),
			(uint32_t *)&dev_cfg->freq);

		if (ret != 0) {
			LOG_ERR("Failed to get the clock rate for ADC: %d", ret);
			break;
		}

		/* Set prescaler */
		adc_set_prescalar(ADC_REGS, dev_cfg->prescaler);

		/* Apply pinctrl state */
		pinctrl_apply_state(dev_cfg->pcfg, PINCTRL_STATE_DEFAULT);

		/* Clear the interrupts */
		adc_interrupt_clear(ADC_REGS);

		/* IRQ connect and set factory calibration value */
		dev_cfg->config_func(dev);

		/* Enable the Result Ready interrupt */
		adc_interrupt_enable(ADC_REGS, 1);

		/* Enable the ADC Controller */
		adc_controller_enable(ADC_REGS, true);

		dev_data->dev = dev;

		adc_context_unlock_unconditionally(&dev_data->ctx);
	} while (0);

	return ret;
}

/*******************************************
 * Zephyr ADC driver API Structure
 *******************************************/

/**
 * @brief Microchip ADC driver API structure.
 *
 * Provides function pointers to implement the ADC driver interface in Zephyr.
 */
static const struct adc_driver_api adc_mchp_api = {
	.channel_setup = adc_mchp_channel_setup,
	.read = adc_mchp_read,
	.ref_internal = ADC_MCHP_VREF_MV,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_mchp_read_async,
#endif
};

/*******************************************
 * Macros and structure initialization for driver instantiation
 *******************************************/

/* Macro to Write the calibration values */
#define ADC_CALIB_WRITE(sw0_word, n, field)                                                        \
	do {                                                                                       \
		uint8_t val_##field = (((sw0_word) & FUSES_SW0_WORD_0_ADC##n##_##field##_Msk) >>   \
				       FUSES_SW0_WORD_0_ADC##n##_##field##_Pos);                   \
		ADC_REGS->ADC_CALIB |= ADC_CALIB_##field(val_##field);                             \
	} while (0)

/**
 * @brief Initialize the ADC factory calibration values.
 *
 * This function reads the NVM Software Calibration Area at address 0x00800080 and extracts
 * calibration values for the specified ADC (either ADC0 or ADC1) using the provided index `n`.
 * It then applies the corresponding calibration values (BIASCOMP, BIASR2R, and BIASREFBUF)
 * to the ADC calibration register.
 *
 * @param dev Pointer to the ADC device structure.
 * @param n Index for the ADC (0 for ADC0, 1 for ADC1).
 *
 * @note The NVM Software Calibration Area can be read at address 0x00800080.
 */
static inline void adc_init_factory_calib_value(const struct device *dev, int n)
{
	const volatile fuses_sw0_fuses_registers_t *fuse_reg =
		(const volatile fuses_sw0_fuses_registers_t *)SW0_ADDR;

	uint32_t sw0_word = (fuse_reg->FUSES_SW0_WORD_0);

	if (n == 0) {
		ADC_CALIB_WRITE(sw0_word, 0, BIASCOMP);
		ADC_CALIB_WRITE(sw0_word, 0, BIASR2R);
		ADC_CALIB_WRITE(sw0_word, 0, BIASREFBUF);
	} else if (n == 1) {
		ADC_CALIB_WRITE(sw0_word, 1, BIASCOMP);
		ADC_CALIB_WRITE(sw0_word, 1, BIASR2R);
		ADC_CALIB_WRITE(sw0_word, 1, BIASREFBUF);
	}
}

/**
 * @brief Defines the ADC configuration function for instance @p n.
 *
 * Used to configure interrupts and calibration for the ADC instance.
 */
#define ADC_MCHP_DEFINE_CONFIG_FUNC(n)                                                             \
	static void adc_mchp_config_##n(const struct device *dev)                                  \
	{                                                                                          \
		/* Placeholder for IRQ and calibration configuration */                            \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, resrdy, irq),                                   \
			    DT_INST_IRQ_BY_NAME(n, resrdy, priority), adc_mchp_isr,                \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQ_BY_NAME(n, resrdy, irq));                                   \
		adc_init_factory_calib_value(dev, n);                                              \
		return;                                                                            \
	}

/**
 * @brief Defines and initializes the ADC driver data structure for instance @p n.
 *
 * Initializes synchronization, locking, and timing context for ADC operations.
 */
#define ADC_MCHP_DATA_DEFN(n)                                                                      \
	static adc_mchp_channel_cfg_t adc_channel_config_##n[DT_INST_PROP(n, num_channels)];       \
	static adc_mchp_dev_data_t adc_mchp_data_##n = {                                           \
		ADC_CONTEXT_INIT_TIMER(adc_mchp_data_##n, ctx),                                    \
		ADC_CONTEXT_INIT_LOCK(adc_mchp_data_##n, ctx),                                     \
		ADC_CONTEXT_INIT_SYNC(adc_mchp_data_##n, ctx),                                     \
		.channel_config = adc_channel_config_##n,                                          \
	}

/**
 * @brief Defines the ADC driver configuration structure for instance @p n.
 *
 * Sets the configuration function, pin control, register mappings,
 * and clock definitions for the ADC instance.
 */
#define ADC_MCHP_CONFIG_DEFN(n)                                                                    \
	static void adc_mchp_config_##n(const struct device *dev);                                 \
	static adc_mchp_dev_config_t adc_mchp_cfg_##n = {                                          \
		.regs = (adc_registers_t *)DT_INST_REG_ADDR(n),                                    \
		.config_func = adc_mchp_config_##n,                                                \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.freq = 0,                                                                         \
		.prescaler = DT_INST_PROP(n, prescaler),                                           \
		.num_channels = DT_INST_PROP(n, num_channels),                                     \
		.adc_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                         \
		.adc_clock.mclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, mclk)), \
				       .id = DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, id)},            \
		.adc_clock.gclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, gclk)), \
				       .id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, id)}}

/**
 * @brief Instantiates the ADC device for instance @p n.
 *
 * This macro handles configuration, data definition, and registration
 * of the ADC device in the Zephyr device model.
 */
#define ADC_MCHP_DEVICE(n)                                                                         \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	ADC_MCHP_CONFIG_DEFN(n);                                                                   \
	ADC_MCHP_DATA_DEFN(n);                                                                     \
	DEVICE_DT_INST_DEFINE(n, adc_mchp_init, NULL, &adc_mchp_data_##n, &adc_mchp_cfg_##n,       \
			      POST_KERNEL, CONFIG_ADC_INIT_PRIORITY, &adc_mchp_api);               \
	ADC_MCHP_DEFINE_CONFIG_FUNC(n);

DT_INST_FOREACH_STATUS_OKAY(ADC_MCHP_DEVICE)
