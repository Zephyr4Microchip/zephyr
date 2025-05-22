/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file intc_mchp_eic_u2254.h
 * @brief EIC driver header file for Microchip eic u2254 peripheral.
 * This can be used to access the APIs implemented for the eic driver.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_INTC_MCHP_EIC_U2254_H_
#define ZEPHYR_INCLUDE_DRIVERS_INTC_MCHP_EIC_U2254_H_

#include <soc.h>
#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>

/* callback for EIC interrupt */
typedef void (*mchp_eic_callback_t)(uint32_t pins, void *data);

/**
 * @brief EIC trigger condition
 */
typedef enum mchp_eic_trigger {
	/* Rising edge */
	MCHP_EIC_RISING = 1,
	/* Falling edge */
	MCHP_EIC_FALLING,
	/* Both edges */
	MCHP_EIC_BOTH,
	/* High level detection */
	MCHP_EIC_HIGH,
	/* Low level detection */
	MCHP_EIC_LOW,

} mchp_eic_trigger_t;

/**
 * @enum mchp_gpio_port_id
 * @brief Enumeration of Microchip GPIO port identifiers.
 *
 * This enum defines the possible port IDs for Microchip GPIO ports.
 */
typedef enum mchp_gpio_port_id {
	MCHP_PORT_ID0 = 0,
	MCHP_PORT_ID1,
	MCHP_PORT_ID2,
	MCHP_PORT_ID3,
	MCHP_PORT_ID_MAX,
} mchp_gpio_port_id_t;

/**
 * @enum mchp_eic_line
 * @brief Enumeration of Microchip EIC (External Interrupt Controller) line numbers.
 *
 * This enum defines the available EIC lines for the Microchip EIC peripheral.
 */
typedef enum mchp_eic_line {
	EIC_LINE_0 = 0,
	EIC_LINE_1,
	EIC_LINE_2,
	EIC_LINE_3,
	EIC_LINE_4,
	EIC_LINE_5,
	EIC_LINE_6,
	EIC_LINE_7,
	EIC_LINE_8,
	EIC_LINE_9,
	EIC_LINE_10,
	EIC_LINE_11,
	EIC_LINE_12,
	EIC_LINE_13,
	EIC_LINE_14,
	EIC_LINE_15,
	EIC_LINE_MAX,
} mchp_eic_line_t;
/**
 * @struct eic_config_params
 * @brief Configuration parameters for a Microchip EIC (External Interrupt Controller) line.
 *
 * This structure contains all the configuration parameters required to set up an EIC line,
 * including trigger type, pin and port information, debounce setting, port register address,
 * callback function, and user data.
 */
typedef struct eic_config_params {
	mchp_eic_trigger_t trig_type; /* trigger type */
	uint8_t pin_num;              /* pin number */
	uint8_t port_id;              /*This denotes the port id */
	bool debounce;                /* Denotes whether debouncing is enabled for the pin or not */
	void *gpio_data;              /* This contains the pointer to the `data`structure of gpio */
	port_group_registers_t *port_addr; /* Contains address of the port register of the pin */
	mchp_eic_callback_t eic_line_callback; /**It contains the address of gpio isr function */
} eic_config_params_t;

/**
 * @brief Configure and enable an EIC interrupt for a given pin.
 *
 * This function configures the EIC interrupt for the specified pin, including setting up
 * the trigger type, enabling input, configuring debounce if required,
 * and updating the internal data structures to reflect the new assignment of pin to an eic line.
 *
 * @param eic_pin_config Pointer to the EIC pin configuration structure.
 *
 * @return 0 on success, or a negative error code on failure:
 *         - -ENOTSUP if no EIC line is available for the pin.
 *         - -EBUSY if the EIC line is already in use.
 */
int eic_mchp_config_interrupt(eic_config_params_t *eic_pin_config);

/**
 * @brief Get the pending EIC interrupts for a given port.
 *
 * This function checks which pins in the specified port have pending EIC interrupts.
 * It returns a bitmask where each set bit corresponds to a pin with a pending interrupt.
 *
 * @param port_id The identifier of the port to check for pending interrupts.
 *
 * @return A bitmask indicating which pins in the port have pending interrupts.
 *         Each bit set in the return value corresponds to a pin number with a pending interrupt.
 *
 */
uint32_t eic_mchp_interrupt_pending(uint8_t port_id);

/**
 * @brief Disables an EIC interrupt for a given pin configuration.
 *
 * This function disables the EIC interrupt associated with the specified pin configuration.
 * It checks if the pin is currently assigned to an EIC line, disables the interrupt line if so,
 * and updates the internal data structures to mark the line as free.
 *
 * @param eic_pin_config Pointer to the EIC pin configuration structure.
 *
 * @return 0 on success, or a negative error code on failure.
 */
int eic_mchp_disable_interrupt(eic_config_params_t *eic_pin_config);

#endif /*ZEPHYR_INCLUDE_DRIVERS_INTC_MCHP_EIC_U2254_H_*/
