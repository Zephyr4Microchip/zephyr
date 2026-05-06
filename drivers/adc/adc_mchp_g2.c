/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file adc_mchp_g2.c
 * @brief ADC driver for Microchip ADC G2 peripherals.
 *
 * Implements Zephyr ADC API support with interrupt-driven sampling,
 * hardware gain/reference configuration, and context-managed reads.
 */

#include <soc.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

/* Register a logging module for ADC driver with configured log level */
LOG_MODULE_REGISTER(adc_mchp_g2, CONFIG_ADC_LOG_LEVEL);

/******************************************************************************
 * @brief Devicetree definitions
 *****************************************************************************/
#define DT_DRV_COMPAT microchip_adc_g2

/******************************************************************************
 * @brief Macro definitions
 *****************************************************************************/
/* Reference voltage of the ADC in millivolts, obtained from device tree */
#define ADC_MCHP_VREF_MV DT_INST_PROP(0, vref_mv)

/* Get device configuration structure from device pointer */
#define DEV_CFG(dev) ((const adc_mchp_dev_config_t *)(dev)->config)

/* Shortcut to access ADC registers from device configuration */
#define ADC_REGS ((const adc_mchp_dev_config_t *)(dev)->config)->regs

/* Calculate ADC sample length in ADC clock cycles given sampling time in nanoseconds,
 * ADC clock frequency in Hz, and conclkdiv value.
 * Formula: ((sampling_time_ns * adc_clk_hz) / (conclkdiv * 1,000,000,000)) - 1
 */
#define ADC_CALC_SAMPLEN_NS(sampling_time_ns, gclk_adc_hz, conclkdiv_val)                          \
	((((uint64_t)(sampling_time_ns) * (gclk_adc_hz)) / ((conclkdiv_val) * 1000000000ULL)) - 1)

/* ADC resolution options (in bits) */
#define ADC_RESOLUTION_6BIT  6
#define ADC_RESOLUTION_8BIT  8
#define ADC_RESOLUTION_10BIT 10
#define ADC_RESOLUTION_12BIT 12

#define ADC_DEFAULT_SAMPLEN 4

#if defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ2) ||                                            \
	defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ3)

/* Channels which overlaps JTAG pins (AN0 to AN3)*/
#define JTAG_CHANNEL_MASK 0x0F

#elif defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ6)

/* Channels which overlaps JTAG pins (AN1 and AN8)*/
#define JTAG_CHANNEL_MASK 0x0102

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
 * @brief Static configuration structure for the ADC driver.
 *
 * Holds constant configuration data such as register base addresses,
 * pin control configuration, clock settings, frequency, conclkdiv,
 * channel count, and initialization function pointer.
 */
typedef struct adc_mchp_dev_config {

	/** ADC Registers */
	adchs_registers_t *regs;

	/** ADC operating frequency in Hz. After div */
	uint32_t freq;

	/** Clock configuration for the ADC peripheral. */
	uint32_t adcsel: 2;

	/** Clock conclkdiv value for the ADC input clock. */
	uint32_t conclkdiv: 6;

	/** Clock adcdiv value for the shared SAR ADC input clock. */
	uint32_t adcdiv: 7;

	/** Fractional data output format. */
	uint32_t fract: 1;

	/** Using signed data output mode. */
	uint32_t sign: 1;

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

/* Enable or disable the result interrupt of a channel. */
static inline void adc_channel_result_int_enable(const adc_mchp_dev_config_t *const dev_cfg,
						 uint8_t channel, bool enable)
{
	if ((uint32_t)channel < dev_cfg->num_channels) {
		if (enable) {
			dev_cfg->regs->ADCHS_ADCGIRQEN1 |= 0x01UL << (uint32_t)channel;
		} else {
			dev_cfg->regs->ADCHS_ADCGIRQEN1 &= ~(0x01UL << (uint32_t)channel);
		}
	}
}

/* Enable or disable the ADC controller. */
static inline void adc_controller_enable(adchs_registers_t *adc_reg, bool enable)
{
	if (enable == true) {
		adc_reg->ADCHS_ADCCON3 |= ADCHS_ADCCON3_DIGEN7_Msk;
	} else {
		adc_reg->ADCHS_ADCCON3 &= ~ADCHS_ADCCON3_DIGEN7_Msk;
	}
}

/* Trigger an ADC conversion. */
static inline void adc_trigger_conversion(adchs_registers_t *adc_reg)
{
	adc_reg->ADCHS_ADCCON3 |= ADCHS_ADCCON3_GSWTRG_Msk;
}

/* Read the result of the ADC conversion. */
static inline uint16_t adc_get_conversion_result(adchs_registers_t *adc_reg, uint8_t channel)
{
	uint32_t channel_addr =
		(uint32_t)adc_reg + ADCHS_ADCDATA0_REG_OFST + ((uint32_t)channel << 4U);
	return (uint16_t)(*(uint32_t *)channel_addr);
}

/**
 * @brief Set ADC acquisition time in terms of sample length (clock cycles).
 *
 * This function configures the ADC sample control register to control the
 * acquisition time. The sample length must be within the valid range [2, 1025].
 *
 * @param[in] adc_reg Pointer to ADC registers.
 * @param[in] sample_length Sample length in ADC clock cycles.
 *
 * @retval 0 on success.
 * @retval -EINVAL if sample_length is out of valid range.
 */
static inline int8_t adc_set_acq_time(adchs_registers_t *adc_reg, uint16_t sample_length)
{
	int ret = 0;

	/* Valid sample length is in the range [2, 1025] */
	if (sample_length > 1025U || sample_length < 2U) {
		ret = -EINVAL;
	}

	if (ret == 0) {
		adc_reg->ADCHS_ADCCON2 &= ~(ADCHS_ADCCON2_SAMC_Msk);
		adc_reg->ADCHS_ADCCON2 |= ADCHS_ADCCON2_SAMC(sample_length - 2);
	}
	return ret;
}

/**
 * @brief Validate ADC channel configuration parameters.
 *
 * This function checks the validity of ADC gain, reference and sample length.
 *
 * @param gain           ADC gain setting (enum adc_gain).
 *                       Valid values: ADC_GAIN_1.
 * @param reference      ADC reference voltage source (enum adc_reference).
 *                       Valid range: ADC_REF_INTERNAL.
 * @param sample_length  Sampling time length. Valid range: 2 to 1025.
 *
 * @return 0 on success, -EINVAL if any parameter is invalid.
 */
static int8_t adc_validate_channel_params(enum adc_gain gain, enum adc_reference reference,
					  uint16_t sample_length)
{
	int8_t ret = 0;

	/* validate gain */
	if (gain != ADC_GAIN_1) {
		ret = -EINVAL;
	}

	/* Validate reference */
	if (reference != ADC_REF_INTERNAL) {
		ret = -EINVAL;
	}

	/* Valid sample length is in the range [2, 1025] */
	if (sample_length > 1025U || sample_length < 2U) {
		ret = -EINVAL;
	}

	return ret;
}

/**
 * @brief Configure ADC oversampling ratio.
 *
 * Sets the number of samples to average per conversion for improved resolution.
 *
 * @param adc_reg Pointer to ADC registers.
 * @param oversampling Oversampling setting (0 to 8, corresponding to 1 to 256 samples).
 * @return 0 on success, -EINVAL if the oversampling value is invalid.
 */
static int8_t adc_set_oversampling(adchs_registers_t *adc_reg, uint8_t oversampling)
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
	 *
	 * Valid range: 0 to 10 (inclusive)
	 */
	if (0 < oversampling && oversampling <= 8) {
		adc_reg->ADCHS_ADCFLTR1 =
			ADCHS_ADCFLTR1_OVRSAM0(oversampling - 1) | ADCHS_ADCFLTR1_ADFMODE0(1);
		adc_reg->ADCHS_ADCFLTR2 = 0;
	} else if (oversampling == 0) {
		adc_reg->ADCHS_ADCFLTR1 = 0;
		adc_reg->ADCHS_ADCFLTR2 = 0;
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
 * @param resolution Desired ADC resolution (6, 8, 10, or 12-bit).
 * @param oversampling Oversampling setting; affects resolution options.
 * @return 0 on success, -EINVAL if parameters are invalid.
 */
static int8_t adc_set_resolution(adchs_registers_t *adc_reg, uint8_t resolution,
				 uint8_t oversampling)
{
	int8_t ret = 0;
	uint16_t resolution_val = 0;

	switch (resolution) {
	case ADC_RESOLUTION_6BIT:
		if (oversampling != 0) {
			ret = -EINVAL;
		} else {
			resolution_val = ADCHS_ADCCON1_SELRES_6_BITS_Val;
		}
		break;
	case ADC_RESOLUTION_8BIT:
		if (oversampling != 0) {
			ret = -EINVAL;
		} else {
			resolution_val = ADCHS_ADCCON1_SELRES_8_BITS_Val;
		}
		break;
	case ADC_RESOLUTION_10BIT:
		if (oversampling != 0) {
			ret = -EINVAL;
		} else {
			resolution_val = ADCHS_ADCCON1_SELRES_10_BITS_Val;
		}
		break;
		break;
	case ADC_RESOLUTION_12BIT:
		resolution_val = ADCHS_ADCCON1_SELRES_12_BITS_Val;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	if (ret == 0) {
		adc_reg->ADCHS_ADCCON1 &= ~ADCHS_ADCCON1_SELRES_Msk;
		adc_reg->ADCHS_ADCCON1 |= ADCHS_ADCCON1_SELRES(resolution_val);
	}

	return ret;
}

/**
 * @brief Select and configure ADC input channels.
 *
 * This function enabless the of specified channel for the ADC
 * based on the configuration provided in @p channel_config.
 *
 * It configures the MUX settings required for both single-ended and
 * differential ADC modes as needed.
 *
 * @param[in] dev Pointer to the ADC device structure.
 * @param[in] channel_config Pointer to the ADC channel configuration structure.
 *
 */
static void adc_select_channels(const struct device *dev, struct adc_channel_cfg *channel_config)
{

	uint8_t value;

	do {
		/* Enable result interrupt */
		adc_channel_result_int_enable(DEV_CFG(dev), channel_config->channel_id, true);

		/* Configure differential and signed data format */
		value = channel_config->differential << 1 | DEV_CFG(dev)->sign;
		ADC_REGS->ADCHS_ADCIMCON1 &= ~(0x3 << (uint32_t)channel_config->channel_id);
		ADC_REGS->ADCHS_ADCIMCON1 |= value << (uint32_t)channel_config->channel_id;
	} while (0);
}

/**
 * @brief Calculate the ADC sample length based on acquisition time and clock settings.
 *
 * This function computes the sample length in ADC clock cycles given the acquisition time,
 * ADC clock frequency, and conclkdiv value. The acquisition time unit can be in ticks,
 * microseconds, or nanoseconds.
 *
 * @param[in] acq_time Acquisition time encoded with unit and value.
 * @param[in] adc_clk ADC clock frequency in Hz.
 * @param[in] conclkdiv ADC conclkdiv value.
 *
 * @return The sample length in ADC clock cycles.
 *         Returns 0 if the acquisition time unit is unsupported or default.
 */
static int adc_get_sample_length(uint16_t acq_time, uint32_t adc_clk, uint8_t conclkdiv)
{
	uint16_t sample_length = 0;

	switch (ADC_ACQ_TIME_UNIT(acq_time)) {
	case ADC_ACQ_TIME_TICKS:
		sample_length = ADC_ACQ_TIME_VALUE(acq_time) - 1;
		break;
	case ADC_ACQ_TIME_MICROSECONDS:
		sample_length = (uint16_t)ADC_CALC_SAMPLEN_NS(
			((ADC_ACQ_TIME_VALUE(acq_time)) * 1000), adc_clk, conclkdiv);
		break;
	case ADC_ACQ_TIME_NANOSECONDS:
		sample_length = (uint16_t)ADC_CALC_SAMPLEN_NS(ADC_ACQ_TIME_VALUE(acq_time), adc_clk,
							      conclkdiv);
		break;
	default:
		/* Unsupported acquisition time unit or ADC_ACQ_TIME_DEFAULT */
		sample_length = ADC_DEFAULT_SAMPLEN;
		break;
	}

	return sample_length;
}

/**
 * @brief Apply ADC channel-specific configuration.
 *
 * This function configures acquisition time settings for a given ADC channel.
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
							       dev_cfg->freq, dev_cfg->conclkdiv);

		/* Set sample length*/
		ret = adc_set_acq_time(ADC_REGS, sample_length);
		if (ret < 0) {
			LOG_ERR("Selected ADC acquisition time is not valid");
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
 * - Configures oversampling settings via `ADC_REG` register
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

/**
 * @brief Check if need to disable jtag.
 *
 * This function check if JTAG is the default functionality on these channel pins.
 * Then disable JTAG function.
 *
 * @param channel_id The channel id.
 */
static void adc_disable_jtag(uint8_t channel_id)
{
	uint32_t mask;

	mask = BIT(channel_id);

	if ((mask & JTAG_CHANNEL_MASK) != 0) {
		/* If any JTAG pins are being repurposed, disable JTAG globally */
		CFG_REGS->CFG_CFGCON0CLR = CFG_CFGCON0_JTAGEN_Msk;
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

	/* interrupt will be cleared when the associated data register is read */
	result = adc_get_conversion_result(ADC_REGS, dev_data->channel_id);

	*dev_data->buffer++ = result;

	dev_data->channels &= ~BIT(dev_data->channel_id);

	adc_channel_result_int_enable(DEV_CFG(dev), dev_data->channel_id, false);

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
 * configuration parameters.
 *
 * The configuration includes:
 * - Acquisition time conversion to sample clocks
 * - Differential or single-ended mode setup
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
						      dev_cfg->conclkdiv);

		/*
		 * If the device supports individual channel configuration,
		 * it can be configured during channel sequencing.
		 * Validate the channel configuration parameters accordingly.
		 */
		ret = adc_validate_channel_params(channel_cfg->gain, channel_cfg->reference,
						  sample_length);

		if (ret != 0) {
			LOG_ERR("Invalid ADC channel config");
			break;
		}

		adc_disable_jtag(channel_cfg->channel_id);

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

static int adc_get_clock_freq(const adc_mchp_dev_config_t *const dev_cfg)
{
	uint32_t subsys;
	int ret;

	switch (dev_cfg->adcsel) {
	case ADCHS_ADCCON3_ADCSEL_PBCLK1_Val:
		subsys = CLOCK_MCHP_PBCLK_ID_1;
		break;
	case ADCHS_ADCCON3_ADCSEL_FRC_Val:
		subsys = CLOCK_MCHP_FRC_ID;
		break;
	case ADCHS_ADCCON3_ADCSEL_REFCLK3_Val:
		subsys = CLOCK_MCHP_REFCLK_ID_3;
		break;
	case ADCHS_ADCCON3_ADCSEL_SYSCLK_Val:
		subsys = CLOCK_MCHP_SYSCLK_ID;
		break;
	default:
		return -EINVAL;
	}
	ret = clock_control_get_rate(DEVICE_DT_GET(DT_NODELABEL(clock)),
				     (clock_control_subsys_t)subsys, (uint32_t *)&dev_cfg->freq);

	if (ret != 0) {
		LOG_ERR("Failed to get the clock rate for ADC: %d", ret);
		return -EFAULT;
	}

	return 0;
}

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

	do {
#if defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ2)
		CFG_REGS->CFG_PMD1 &= ~(CFG_PMD1_ADCMD_Msk | CFG_PMD1_ADCSARMD_Msk);
#elif defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ3)
		CFG_REGS->CFG_PMD1 &=
			~(CFG_PMD1_ADCMD_Msk | CFG_PMD1_ADCSARMD_Msk | CFG_PMD1_CVDMD_Msk);
#elif defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ6)
		CFG_REGS->CFG_PMD1 &= ~(CFG_PMD1_ADCMD_Msk | CFG_PMD1_ADCSARMD_Msk |
					CFG_PMD1_CVDMD_Msk | CFG_PMD1_ADCSARSHRMD_Msk);
#endif

		/* clear and disable ADC */
		ADC_REGS->ADCHS_ADCCON1 = 0;

		/* fractional data output format */
		ADC_REGS->ADCHS_ADCCON1 = ADCHS_ADCCON1_FRACT(dev_cfg->fract);

		/* division ratio for the shared SAR ADC core clock */
		ADC_REGS->ADCHS_ADCCON2 = ADCHS_ADCCON2_ADCDIV(dev_cfg->adcdiv);

		/* analog-to-digital control clock (TQ) divider and analog-to-digital clock Source
		 * (TCLK)
		 */
		ADC_REGS->ADCHS_ADCCON3 = ADCHS_ADCCON3_CONCLKDIV(dev_cfg->conclkdiv) |
					  ADCHS_ADCCON3_ADCSEL(dev_cfg->adcsel);

		/* trigger source, set to sw edge */
		ADC_REGS->ADCHS_ADCTRG1 =
			ADCHS_ADCTRG1_TRGSRC0_SW_EDGE | ADCHS_ADCTRG1_TRGSRC1_SW_EDGE |
			ADCHS_ADCTRG1_TRGSRC2_SW_EDGE | ADCHS_ADCTRG1_TRGSRC3_SW_EDGE;
		ADC_REGS->ADCHS_ADCTRG2 =
#if defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ2)
			ADCHS_ADCTRG2_TRGSRC4_SW_EDGE | ADCHS_ADCTRG2_TRGSRC5_SW_EDGE;
#else
			ADCHS_ADCTRG2_TRGSRC4_SW_EDGE | ADCHS_ADCTRG2_TRGSRC5_SW_EDGE |
			ADCHS_ADCTRG2_TRGSRC6_SW_EDGE | ADCHS_ADCTRG2_TRGSRC7_SW_EDGE;
#endif

		/* ADC Trigger Level/Edge Sensitivity Register */
		ADC_REGS->ADCHS_ADCTRGSNS = 0x0;

		/* ADC Input Mode Control, select between single-ended and differential operation as
		 * well as select between signed and unsigned data format.
		 */
		ADC_REGS->ADCHS_ADCIMCON1 = 0x0;

		/* analog inputs to be scanned by the common scan trigger. */
		ADC_REGS->ADCHS_ADCCSS1 = 0x0;

		/* Get ADC Clock Frequency */
		ret = adc_get_clock_freq(dev_cfg);
		if (ret != 0) {
			LOG_ERR("Failed to get the clock rate for ADC: %d", ret);
			break;
		}

		/* IRQ connect and set factory calibration value */
		dev_cfg->config_func(dev);

		/* Turn ON ADC */
		ADC_REGS->ADCHS_ADCCON1 |= ADCHS_ADCCON1_ON_Msk;
		while ((ADC_REGS->ADCHS_ADCCON2 & ADCHS_ADCCON2_BGVRRDY_Msk) ==
		       ADCHS_ADCCON2_BGVRRDY_Msk) {
			/* Wait until the reference voltage is ready */
		}

		while ((ADC_REGS->ADCHS_ADCCON2 & ADCHS_ADCCON2_REFFLT_Msk) ==
		       ADCHS_ADCCON2_REFFLT_Msk) {
			/* Wait if there is a fault with the reference voltage */
		}

		/* ADC 7 */
		ADC_REGS->ADCHS_ADCANCON |=
			ADCHS_ADCANCON_ANEN7_Msk; /* Enable the clock to analog bias */
		while (((dev_cfg->regs->ADCHS_ADCANCON & ADCHS_ADCANCON_WKRDY7_Msk)) ==
		       0U) { /* Wait until ADC is ready */
			     /* Nothing to do */
		}

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
static DEVICE_API(adc, adc_mchp_api) = {
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

/**
 * @brief Defines the ADC configuration function for instance @p n.
 *
 * Used to configure interrupts and calibration for the ADC instance.
 */
#define ADC_MCHP_DEFINE_CONFIG_FUNC(n)                                                             \
	static void adc_mchp_config_##n(const struct device *dev)                                  \
	{                                                                                          \
		/* Placeholder for IRQ and calibration configuration */                            \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, 0, irq), DT_INST_IRQ_BY_IDX(n, 0, priority),     \
			    adc_mchp_isr, DEVICE_DT_INST_GET(n), 0);                               \
		irq_enable(DT_INST_IRQ_BY_IDX(n, 0, irq));                                         \
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
		.regs = (adchs_registers_t *)DT_INST_REG_ADDR(n),                                  \
		.config_func = adc_mchp_config_##n,                                                \
		.freq = 0,                                                                         \
		.adcsel = DT_ENUM_IDX(DT_NODELABEL(adc), adcsel),                                  \
		.conclkdiv = DT_INST_PROP(n, conclkdiv) - 1,                                       \
		.adcdiv = DT_INST_PROP(n, adcdiv) >> 1,                                            \
		.fract = DT_ENUM_IDX(DT_NODELABEL(adc), fract),                                    \
		.sign = DT_INST_PROP(n, sign),                                                     \
		.num_channels = DT_INST_PROP(n, num_channels)}

/**
 * @brief Instantiates the ADC device for instance @p n.
 *
 * This macro handles configuration, data definition, and registration
 * of the ADC device in the Zephyr device model.
 */
#define ADC_MCHP_DEVICE(n)                                                                         \
	ADC_MCHP_CONFIG_DEFN(n);                                                                   \
	ADC_MCHP_DATA_DEFN(n);                                                                     \
	DEVICE_DT_INST_DEFINE(n, adc_mchp_init, NULL, &adc_mchp_data_##n, &adc_mchp_cfg_##n,       \
			      POST_KERNEL, CONFIG_ADC_INIT_PRIORITY, &adc_mchp_api);               \
	ADC_MCHP_DEFINE_CONFIG_FUNC(n);

DT_INST_FOREACH_STATUS_OKAY(ADC_MCHP_DEVICE)
