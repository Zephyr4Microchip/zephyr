/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file gpio_mchp_v1.c
 * @brief GPIO driver implementation for Microchip devices.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/gpio/microchip-sam-gpio.h>
#include <zephyr/drivers/interrupt_controller/intc_mchp_eic_u2254.h>
#include <soc.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include "gpio_mchp_v1.h"
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_mchp_v1, CONFIG_GPIO_LOG_LEVEL);

/**
 * @brief Configuration structure for MCHP GPIO driver
 */
struct gpio_mchp_config {
	/* Common GPIO driver configuration */
	struct gpio_driver_config common;
	/* Pointer to port group registers */
	hal_gpio_port_reg *hal_regs;
	/*Contains the ID of the gpio port*/
	uint8_t gpio_port_id;
};

/**
 * @brief Runtime data structure for MCHP GPIO driver
 */
struct gpio_mchp_data {
	/* Common GPIO driver data */
	struct gpio_driver_data common;
	/* Pointer to device structure */
	const struct device *dev;
	gpio_port_pins_t debounce;
#ifdef CONFIG_MCHP_EIC_U2254
	/* provided by gpio_utils
	 * callbacks are stored here for each pins
	 */
	sys_slist_t callbacks;
#endif /*CONFIG_MCHP_EIC_U2254*/
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
	struct gpio_mchp_data *gpio_data = dev->data;
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
	/* Preserve debounce flag for interrupt configuration. */
	WRITE_BIT(gpio_data->debounce, pin, ((flags & MCHP_GPIO_DEBOUNCE) != 0));
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

#ifdef CONFIG_MCHP_EIC_U2254
/**
 * @brief Convert a GPIO trigger mode to a Microchip EIC trigger type.
 *
 * This function maps a generic GPIO interrupt trigger mode to the corresponding
 * Microchip EIC trigger type enumeration value.
 *
 * @param trigger_mode The GPIO trigger mode (e.g., edge/level, rising/falling/both).
 *                    Should be one of the GPIO_INT_* macros.
 *
 * @return The corresponding @ref mchp_eic_trigger_t value for the given trigger mode.
 *         If the trigger mode is not recognized, returns 0.
 *
 */
static mchp_eic_trigger_t get_eic_trig_type(uint32_t trigger_mode)
{
	mchp_eic_trigger_t trig_type = 0;

	switch (trigger_mode) {
	case GPIO_INT_EDGE_BOTH:
		trig_type = MCHP_EIC_BOTH;
		LOG_DBG("both edge");
		break;
	case GPIO_INT_EDGE_RISING:
		trig_type = MCHP_EIC_RISING;
		LOG_DBG("rising edge");
		break;
	case GPIO_INT_EDGE_FALLING:
		trig_type = MCHP_EIC_FALLING;
		LOG_DBG("falling edge");
		break;

	case GPIO_INT_LEVEL_HIGH:
		trig_type = MCHP_EIC_HIGH;
		LOG_DBG("level high");
		break;
	case GPIO_INT_LEVEL_LOW:
		trig_type = MCHP_EIC_LOW;
		LOG_DBG("level low");
		break;
	default:
		LOG_ERR("Unknown trigger mode 0x%x", trigger_mode);
		break;
	}
	return trig_type;
}

/**
 * @brief GPIO interrupt service routine (ISR).
 *
 * This function is called when a GPIO interrupt occurs. It logs the interrupt event
 * and invokes registered GPIO callbacks for the triggered pins.
 *
 * @param pins Bitmask indicating which GPIO pins triggered the interrupt.
 * @param arg  Pointer to a @ref gpio_mchp_data structure containing callback and device
 * information.
 *
 */
static void gpio_mchp_isr(uint32_t pins, void *arg)
{
	struct gpio_mchp_data *const data = (struct gpio_mchp_data *)arg;

	gpio_fire_callbacks(&data->callbacks, data->dev, pins);
}
/**
 * @brief Configure interrupt for a specific GPIO pin.
 *
 * This function configures the interrupt mode and trigger type for a given GPIO pin
 * on a Microchip GPIO controller. It sets up the necessary parameters and enables or
 * disables the interrupt as requested.
 *
 * @param[in] dev   Pointer to the device structure for the GPIO controller.
 * @param[in] pin   Pin number to configure.
 * @param[in] mode  Interrupt mode (e.g., disabled, edge, level).
 * @param[in] trig  Interrupt trigger type (e.g., rising, falling, both).
 *
 * @retval 0        On success.
 * @retval -EINVAL  If an invalid trigger mode is specified.
 * @retval <0       Other negative values may be returned from lower-level functions.
 *
 * @note This function relies on the device's configuration and data structures,
 *       and uses the EIC (External Interrupt Controller) for interrupt management.
 */
static int gpio_mchp_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
					     enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	int ret_val = 0;
	const struct gpio_mchp_config *gpio_config = dev->config;
	struct gpio_mchp_data *gpio_data = dev->data;
	eic_config_params_t eic_pin_config = {0};
	uint32_t trigger_mode =
		(mode == GPIO_INT_MODE_DISABLED) ? GPIO_INT_MODE_DISABLED : (mode | trig);

	gpio_data->dev = dev;
	/* initialise the config params structure */
	eic_pin_config.port_id = gpio_config->gpio_port_id;
	eic_pin_config.pin_num = pin;
	eic_pin_config.debounce = (gpio_data->debounce & BIT(pin)) ? true : false;
	eic_pin_config.port_addr = gpio_config->hal_regs;
	eic_pin_config.eic_line_callback = gpio_mchp_isr;
	eic_pin_config.gpio_data = gpio_data;

	LOG_DBG("trigger mode : 0x%x mode = 0x%x trig = 0x%x\nport address : %p", trigger_mode,
		mode, trig, eic_pin_config.port_addr);

	/*
	 * Handle GPIO interrupt configuration based on the trigger mode.
	 */
	switch (trigger_mode) {
	case GPIO_INT_MODE_DISABLED:
		ret_val = eic_mchp_disable_interrupt(&eic_pin_config);
		break;
	case GPIO_INT_EDGE_RISING:
	case GPIO_INT_EDGE_FALLING:
	case GPIO_INT_EDGE_BOTH:
	case GPIO_INT_LEVEL_HIGH:
	case GPIO_INT_LEVEL_LOW:
		eic_pin_config.trig_type = get_eic_trig_type(trigger_mode);
		ret_val = eic_mchp_config_interrupt(&eic_pin_config);
		break;
	default:
		ret_val = -EINVAL;
		LOG_ERR("Invalid trigger mode for interrupt");
		break;
	}
	LOG_DBG("retval = %d", ret_val);
	return ret_val;
}

/**
 * @brief Manage (add or remove) a GPIO callback for the MCHP GPIO driver.
 *
 * This function adds or removes a user-specified callback from the GPIO
 * callback syslist associated with the device.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param callback Pointer to the GPIO callback structure to add or remove.
 * @param set Boolean flag indicating whether to add (true) or remove (false) the callback.
 *
 * @retval 0 On success.
 * @retval -EINVAL If the callback is invalid or not found (when removing).
 *
 * @note This function is typically called by higher-level GPIO API functions
 *       to manage interrupt callbacks.
 */
static int gpio_mchp_manage_callback(const struct device *dev, struct gpio_callback *callback,
				     bool set)
{
	struct gpio_mchp_data *const data = dev->data;
	/**Adds or remove user specified callback on to the syslist */
	return gpio_manage_callback(&data->callbacks, callback, set);
}

/**
 * @brief Get pending GPIO interrupt status for the MCHP GPIO driver.
 *
 * This function retrieves the pending interrupt status for the specified
 * GPIO device. It is typically used to check if any GPIO interrupts are
 * pending, for example after waking up from low power mode.
 *
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @return A bitmask indicating the pin and port numbers of pending interrupts.
 *         Each bit set corresponds to a pending interrupt on a specific pin.
 *
 * @note This function calls the EIC (External Interrupt Controller) API to
 *       obtain the pending interrupt status.
 */
static uint32_t gpio_mchp_get_pending_int(const struct device *dev)
{
	const struct gpio_mchp_config *config = dev->config;

	return eic_mchp_interrupt_pending(config->gpio_port_id);
}

#endif /*CONFIG_MCHP_EIC_U2254*/

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
#ifdef CONFIG_MCHP_EIC_U2254
	.pin_interrupt_configure = gpio_mchp_pin_interrupt_configure,
	.manage_callback = gpio_mchp_manage_callback,
	.get_pending_int = gpio_mchp_get_pending_int,
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
		.gpio_port_id = MCHP_PORT_ID##idx};                                                \
	static struct gpio_mchp_data gpio_mchp_data_##idx;                                         \
	DEVICE_DT_DEFINE(DT_INST(idx, DT_DRV_COMPAT), gpio_mchp_init, NULL, &gpio_mchp_data_##idx, \
			 &gpio_mchp_config_##idx, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,         \
			 &gpio_mchp_api);

/* Use DT_INST_FOREACH_STATUS_OKAY to iterate over GPIO instances */
DT_INST_FOREACH_STATUS_OKAY(GPIO_PORT_CONFIG)
