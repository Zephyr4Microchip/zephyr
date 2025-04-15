/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file gpio_mchp_v1.c
 * @brief GPIO driver implementation for Microchip devices.
 */

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/gpio/microchip-sam-gpio.h>
#include <soc.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include "gpio_mchp_v1.h"

/**
 * @brief Configuration structure for MCHP GPIO driver
 */
struct gpio_mchp_config {
	/* Common GPIO driver configuration */
	struct gpio_driver_config common;
	/* Pointer to port group registers */
	hal_gpio_port_reg *hal_regs;
};

/**
 * @brief Runtime data structure for MCHP GPIO driver
 */
struct gpio_mchp_data {
	/* Common GPIO driver data */
	struct gpio_driver_data common;
	/* Pointer to device structure */
	const struct device *dev;
};

/**
 * @brief Configure a GPIO pin
 *
 * @param dev Pointer to the device structure
 * @param pin Pin number to configure
 * @param flags Configuration flags
 * @retval 0 on success
 * @retval ENOTSUP If any of the configuration options is not supported
 */
static int gpio_mchp_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_mchp_config *config = dev->config;
	hal_gpio_port_reg *hal_gpio = config->hal_regs;
	int retval = 0;
	gpio_flags_t io_flags = 0;

	io_flags = flags & (GPIO_INPUT | GPIO_OUTPUT);
	if (io_flags == GPIO_DISCONNECTED) {
		/* Disconnect the gpio if the feature is supported, and exit */
		retval = hal_mchp_gpio_disconnect(hal_gpio, pin);
		return (retval < 0) ? -ENOTSUP : 0;
	}

	/* Check for single-ended mode configuration */
	if (flags & GPIO_SINGLE_ENDED) {
		retval = (flags & GPIO_LINE_OPEN_DRAIN)
				 ? hal_mchp_gpio_port_set_open_drain(hal_gpio, pin)
				 : hal_mchp_gpio_port_set_open_source(hal_gpio, pin);

		if (retval < 0) {
			return -ENOTSUP;
		}
	}

	/* Configure the pin as input and output if requested */
	if (io_flags == (GPIO_INPUT | GPIO_OUTPUT)) {
		retval = hal_mchp_gpio_set_dir_input_output(hal_gpio, pin);

		if (retval < 0) {
			return -ENOTSUP;
		}

		/* Set initial output state if specified */
		if (flags & GPIO_OUTPUT_INIT_LOW) {
			hal_mchp_gpio_outclr(hal_gpio, pin);
		} else if (flags & GPIO_OUTPUT_INIT_HIGH) {
			hal_mchp_gpio_outset(hal_gpio, pin);
		}
	} else if ((flags & GPIO_INPUT) != 0) {
		/* Configure the pin as input if requested */
		hal_mchp_gpio_set_dir_input(hal_gpio, pin);

		/* Configure pull-up or pull-down if requested */
		if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) != 0) {
			retval = hal_mchp_gpio_enable_pullup(hal_gpio, pin);
			if (retval < 0) {
				return -ENOTSUP;
			}

			if ((flags & GPIO_PULL_UP) != 0) {
				hal_mchp_gpio_outset(hal_gpio, pin);
			} else {
				hal_mchp_gpio_outclr(hal_gpio, pin);
			}
		}
	} else if ((flags & GPIO_OUTPUT) != 0) {
		/* Output is incompatible with pull-up or pull-down */
		if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) != 0) {
			return -ENOTSUP;
		}
		/* Set initial output state if specified */
		if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			hal_mchp_gpio_outclr(hal_gpio, pin);
		} else if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			hal_mchp_gpio_outset(hal_gpio, pin);
		}
		/* Set the pin as output */
		hal_mchp_gpio_set_dir_output(hal_gpio, pin);
	}

	return 0;
}

/**
 * @brief Get raw port value
 *
 * @param dev Pointer to the device structure
 * @param value Pointer to store the port value
 * @retval 0 on success
 */
static int gpio_mchp_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	const struct gpio_mchp_config *config = dev->config;
	hal_gpio_port_reg *hal_gpio = config->hal_regs;
	/* Read the input value of the port */
	hal_mchp_gpio_port_get(hal_gpio, value);

	return 0;
}

/**
 * @brief Set masked raw port value
 *
 * @param dev Pointer to the device structure
 * @param mask Mask of pins to set
 * @param value Value to set
 * @retval 0 on success
 */
static int gpio_mchp_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					 gpio_port_value_t value)
{
	const struct gpio_mchp_config *config = dev->config;
	hal_gpio_port_reg *hal_gpio = config->hal_regs;
	/* Set the output value of the port with the specified mask */
	hal_mchp_gpio_port_outset_masked(hal_gpio, mask, value);

	return 0;
}

/**
 * @brief Set bits in raw port value
 *
 * @param dev Pointer to the device structure
 * @param pins Pins to set
 * @retval 0 on success
 */
static int gpio_mchp_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_mchp_config *config = dev->config;
	hal_gpio_port_reg *hal_gpio = config->hal_regs;
	/* Set the specified pins in the output register */
	hal_mchp_gpio_port_set_pins_high(hal_gpio, pins);

	return 0;
}

/**
 * @brief Clear bits in raw port value
 *
 * @param dev Pointer to the device structure
 * @param pins Pins to clear
 * @retval 0 on success
 */
static int gpio_mchp_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_mchp_config *config = dev->config;
	hal_gpio_port_reg *hal_gpio = config->hal_regs;
	/* Clear the specified pins in the output register */
	hal_mchp_gpio_port_set_pins_low(hal_gpio, pins);

	return 0;
}

/**
 * @brief Toggle bits in raw port value
 *
 * @param dev Pointer to the device structure
 * @param pins Pins to toggle
 * @retval 0 on success
 */
static int gpio_mchp_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_mchp_config *config = dev->config;
	hal_gpio_port_reg *hal_gpio = config->hal_regs;
	/* Toggle the specified pins in the output register */
	hal_mchp_gpio_port_toggle_pins(hal_gpio, pins);

	return 0;
}

#ifdef CONFIG_GPIO_GET_CONFIG
/**
 * @brief Get the configuration of a specific GPIO pin
 *
 * This function retrieves the configuration flags of a specified GPIO pin.
 *
 * @param dev Pointer to the device structure
 * @param pin The pin number to get the configuration for
 * @param out_flags Pointer to store the retrieved configuration flags
 * @retval 0 on success
 */
static int gpio_mchp_pin_get_config(const struct device *dev, gpio_pin_t pin,
				    gpio_flags_t *out_flags)
{
	const struct gpio_mchp_config *config = dev->config;
	hal_gpio_port_reg *hal_gpio = config->hal_regs;
	struct gpio_mchp_data *data = dev->data;
	gpio_flags_t flags = 0;

	/* flag to check if the pin is configured as an output */
	bool is_output = hal_mchp_gpio_is_pin_output(hal_gpio, pin);
	/* flag to check if pull-up or pull-down resistors are enabled */
	bool is_pull_enabled = hal_mchp_gpio_is_pullup(hal_gpio, pin);
	/* flag to check if the output is set to high */
	bool is_output_high = hal_mchp_gpio_is_pin_high(hal_gpio, pin);
	/* Check if the pin is configured as active low */
	bool is_active_low = data->common.invert & (gpio_port_pins_t)BIT(pin);
	/* Check if the pin is configured as open drain */
	bool is_open_drain = hal_mchp_gpio_is_pin_open_drain(hal_gpio, pin);

	/* Check if the pin is configured as an output */
	if (is_output) {
		flags |= GPIO_OUTPUT;
		flags |= is_output_high ? GPIO_OUTPUT_INIT_HIGH : GPIO_OUTPUT_INIT_LOW;
	} else {
		flags |= GPIO_INPUT;
		/* Check if pull-up or pull-down resistors are enabled */
		if (is_pull_enabled) {
			flags |= is_output_high ? GPIO_PULL_UP : GPIO_PULL_DOWN;
		}
	}

	/* Check if the pin is configured as open drain */
	flags |= is_open_drain ? GPIO_LINE_OPEN_DRAIN : GPIO_LINE_OPEN_SOURCE;
	/* Check if the pin is configured as active low */
	flags |= is_active_low ? GPIO_ACTIVE_LOW : GPIO_ACTIVE_HIGH;

	*out_flags = flags;
	return 0;
}
#endif /* CONFIG_GPIO_GET_CONFIG */

#ifdef CONFIG_GPIO_GET_DIRECTION
/**
 * @brief Get the direction of GPIO pins in a port.
 *
 * This function retrieves the direction (input or output) of the specified GPIO pins in a port.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param map Bitmask representing the pins to check.
 * @param inputs Pointer to store the bitmask of input pins (can be NULL).
 * @param outputs Pointer to store the bitmask of output pins (can be NULL).
 *
 * @retval 0 on success
 */
static int gpio_mchp_port_get_direction(const struct device *dev, gpio_port_pins_t map,
					gpio_port_pins_t *inputs, gpio_port_pins_t *outputs)
{
	/* Get the device configuration */
	const struct gpio_mchp_config *config = dev->config;
	/* Get the HAL GPIO register base address*/
	hal_gpio_port_reg *hal_gpio = config->hal_regs;

	map &= config->common.port_pin_mask;

	if (inputs != NULL) {
		/* Get the input pins */
		*inputs = map & (hal_mchp_gpio_port_get_input_pins(hal_gpio));
	}

	if (outputs != NULL) {
		/* Get the output pins */
		*outputs = map & (hal_mchp_gpio_port_get_output_pins(hal_gpio));
	}

	return 0;
}

#endif /* CONFIG_GPIO_GET_DIRECTION */

/**
 * @brief GPIO driver API structure
 */
static const struct gpio_driver_api gpio_mchp_api = {
	.pin_configure = gpio_mchp_configure,
	.port_get_raw = gpio_mchp_port_get_raw,
	.port_set_masked_raw = gpio_mchp_port_set_masked_raw,
	.port_set_bits_raw = gpio_mchp_port_set_bits_raw,
	.port_clear_bits_raw = gpio_mchp_port_clear_bits_raw,
	.port_toggle_bits = gpio_mchp_port_toggle_bits,
#ifdef CONFIG_GPIO_GET_CONFIG
	.pin_get_config = gpio_mchp_pin_get_config,
#endif
#ifdef CONFIG_GPIO_GET_DIRECTION
	.port_get_direction = gpio_mchp_port_get_direction,
#endif
};

/**
 * @brief Initialize the GPIO driver
 *
 * @param dev Pointer to the device structure
 * @retval 0 on success
 */
static int gpio_mchp_init(const struct device *dev)
{
	return 0;
}

/* Define GPIO port configuration macro */
#define GPIO_PORT_CONFIG(idx)                                                                      \
	static const struct gpio_mchp_config gpio_mchp_config_##idx = {                            \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(idx),             \
			},                                                                         \
		.hal_regs = (hal_gpio_port_reg *)DT_INST_REG_ADDR(idx),                            \
	};                                                                                         \
	static struct gpio_mchp_data gpio_mchp_data_##idx;                                         \
	DEVICE_DT_DEFINE(DT_INST(idx, DT_DRV_COMPAT), gpio_mchp_init, NULL, &gpio_mchp_data_##idx, \
			 &gpio_mchp_config_##idx, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,         \
			 &gpio_mchp_api);

/* Use DT_INST_FOREACH_STATUS_OKAY to iterate over GPIO instances */
DT_INST_FOREACH_STATUS_OKAY(GPIO_PORT_CONFIG)
