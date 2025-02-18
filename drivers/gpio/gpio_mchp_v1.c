/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_same54_gpio

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/gpio/microchip-sam-gpio.h>
#include <soc.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

#ifndef PORT_PMUX_PMUXE_A_Val
#define PORT_PMUX_PMUXE_A_Val (0)
#endif

/**
 * @brief Configuration structure for MCHP GPIO driver
 */
struct gpio_mchp_config {
	/* Common GPIO driver configuration */
	struct gpio_driver_config common;
	/* Pointer to port group registers */
	port_group_registers_t *regs;
};

/**
 * @brief Runtime data structure for MCHP GPIO driver
 */
struct gpio_mchp_data {
	/* Common GPIO driver data */
	struct gpio_driver_data common;
	/* Pointer to device structure */
	const struct device *dev;
	/* Debounce configuration for pins */
	gpio_port_pins_t debounce;
};

/**
 * @brief Configure a GPIO pin
 *
 * @param dev Pointer to the device structure
 * @param pin Pin number to configure
 * @param flags Configuration flags
 * @return 0 on success, negative error code on failure
 */
static int gpio_mchp_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_mchp_config *config = dev->config;
	struct gpio_mchp_data *data = dev->data;
	port_group_registers_t *regs = config->regs;

	/* Check if single-ended mode is requested, which is not supported */
	if ((flags & GPIO_SINGLE_ENDED) != 0) {
		return -ENOTSUP;
	}

	/* Configure the pin as input if requested */
	if ((flags & GPIO_INPUT) != 0) {
		regs->PORT_PINCFG[pin] |= PORT_PINCFG_INEN(1);
	}

	/* Configure the pin as output if requested */
	if ((flags & GPIO_OUTPUT) != 0) {
		/* Output is incompatible with pull-up or pull-down */
		if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) != 0) {
			return -ENOTSUP;
		}
		/* Set initial output state if specified */
		if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			regs->PORT_OUTCLR |= (1 << pin);
		} else if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			regs->PORT_OUTSET |= (1 << pin);
		}
		/* Set the pin as output */
		regs->PORT_DIRSET |= (1 << pin);
	} else {
		/* If not output, configure as input */
		regs->PORT_DIRCLR |= (1 << pin);

		/* Configure pull-up or pull-down if requested */
		if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) != 0) {
			regs->PORT_PINCFG[pin] |= PORT_PINCFG_PULLEN(1);
			if ((flags & GPIO_PULL_UP) != 0) {
				regs->PORT_OUTSET |= (1 << pin);
			} else {
				regs->PORT_OUTCLR |= (1 << pin);
			}
		}
	}

	/* Preserve debounce flag for interrupt configuration */
	WRITE_BIT(data->debounce, pin,
		  ((flags & MCHP_GPIO_DEBOUNCE) != 0) && ((flags & GPIO_INPUT) != 0));

	return 0;
}

/**
 * @brief Get raw port value
 *
 * @param dev Pointer to the device structure
 * @param value Pointer to store the port value
 * @return 0 on success, negative error code on failure
 */
static int gpio_mchp_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	const struct gpio_mchp_config *config = dev->config;

	/* Read the input value of the port */
	*value = config->regs->PORT_IN;

	return 0;
}

/**
 * @brief Set masked raw port value
 *
 * @param dev Pointer to the device structure
 * @param mask Mask of pins to set
 * @param value Value to set
 * @return 0 on success, negative error code on failure
 */
static int gpio_mchp_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					 gpio_port_value_t value)
{
	const struct gpio_mchp_config *config = dev->config;
	uint32_t out = config->regs->PORT_OUT;

	/* Set the output value of the port with the specified mask */
	config->regs->PORT_OUT = (out & ~mask) | (value & mask);

	return 0;
}

/**
 * @brief Set bits in raw port value
 *
 * @param dev Pointer to the device structure
 * @param pins Pins to set
 * @return 0 on success, negative error code on failure
 */
static int gpio_mchp_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_mchp_config *config = dev->config;

	/* Set the specified pins in the output register */
	config->regs->PORT_OUTSET = pins;

	return 0;
}

/**
 * @brief Clear bits in raw port value
 *
 * @param dev Pointer to the device structure
 * @param pins Pins to clear
 * @return 0 on success, negative error code on failure
 */
static int gpio_mchp_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_mchp_config *config = dev->config;

	/* Clear the specified pins in the output register */
	config->regs->PORT_OUTCLR = pins;

	return 0;
}

/**
 * @brief Toggle bits in raw port value
 *
 * @param dev Pointer to the device structure
 * @param pins Pins to toggle
 * @return 0 on success, negative error code on failure
 */
static int gpio_mchp_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_mchp_config *config = dev->config;

	/* Toggle the specified pins in the output register */
	config->regs->PORT_OUTTGL = pins;

	return 0;
}

/**
 * @brief GPIO driver API structure
 */
static const struct gpio_driver_api gpio_mchp_api = {
	.pin_configure = gpio_mchp_config,
	.port_get_raw = gpio_mchp_port_get_raw,
	.port_set_masked_raw = gpio_mchp_port_set_masked_raw,
	.port_set_bits_raw = gpio_mchp_port_set_bits_raw,
	.port_clear_bits_raw = gpio_mchp_port_clear_bits_raw,
	.port_toggle_bits = gpio_mchp_port_toggle_bits,
};

/**
 * @brief Initialize the GPIO driver
 *
 * @param dev Pointer to the device structure
 * @return 0 on success, negative error code on failure
 */
static int gpio_mchp_init(const struct device *dev)
{
	/* Initialization code can be added here if needed */
	return 0;
}

/* Port A */
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(porta))

/**
 * @brief Configuration for Port A
 */
static const struct gpio_mchp_config gpio_mchp_config_0 = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(0),
	},
	.regs = (port_group_registers_t *)DT_REG_ADDR(DT_NODELABEL(porta)),
};

/**
 * @brief Runtime data for Port A
 */
static struct gpio_mchp_data gpio_mchp_data_0;

DEVICE_DT_DEFINE(DT_NODELABEL(porta), gpio_mchp_init, NULL, &gpio_mchp_data_0, &gpio_mchp_config_0,
		 PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY, &gpio_mchp_api);
#endif

/* Port B */
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(portb))

/**
 * @brief Configuration for Port B
 */
static const struct gpio_mchp_config gpio_mchp_config_1 = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(1),
	},
	.regs = (port_group_registers_t *)DT_REG_ADDR(DT_NODELABEL(portb)),
};

/**
 * @brief Runtime data for Port B
 */
static struct gpio_mchp_data gpio_mchp_data_1;

DEVICE_DT_DEFINE(DT_NODELABEL(portb), gpio_mchp_init, NULL, &gpio_mchp_data_1, &gpio_mchp_config_1,
		 PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY, &gpio_mchp_api);
#endif

/* Port C */
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(portc))

/**
 * @brief Configuration for Port C
 */
static const struct gpio_mchp_config gpio_mchp_config_2 = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(2),
	},
	.regs = (port_group_registers_t *)DT_REG_ADDR(DT_NODELABEL(portc)),
};

/**
 * @brief Runtime data for Port C
 */
static struct gpio_mchp_data gpio_mchp_data_2;

DEVICE_DT_DEFINE(DT_NODELABEL(portc), gpio_mchp_init, NULL, &gpio_mchp_data_2, &gpio_mchp_config_2,
		 PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY, &gpio_mchp_api);
#endif

/* Port D */
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(portd))

/**
 * @brief Configuration for Port D
 */
static const struct gpio_mchp_config gpio_mchp_config_3 = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(3),
	},
	.regs = (port_group_registers_t *)DT_REG_ADDR(DT_NODELABEL(portd)),
};

/**
 * @brief Runtime data for Port D
 */
static struct gpio_mchp_data gpio_mchp_data_3;

DEVICE_DT_DEFINE(DT_NODELABEL(portd), gpio_mchp_init, NULL, &gpio_mchp_data_3, &gpio_mchp_config_3,
		 PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY, &gpio_mchp_api);
#endif
