/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file intc_mchp_eic_g2.h
 * @brief EIC driver header file for Microchip eic g2 peripheral.
 * This can be used to access the APIs implemented for the eic driver.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_INTC_MCHP_EIC_G2_H_
#define ZEPHYR_INCLUDE_DRIVERS_INTC_MCHP_EIC_G2_H_

#include <soc.h>
#include <zephyr/types.h>

/* callback for EIC interrupt */
typedef void (*mchp_eic_callback_t)(void *data);

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
 * @enum mchp_eic_pin
 * @brief Enumeration of Microchip EIC (External Interrupt Controller) pin numbers.
 *
 * This enum defines the available EIC pins for the Microchip EIC peripheral.
 */
typedef enum mchp_eic_pin {
	EIC_PIN_0 = 0,
	EIC_PIN_1,
	EIC_PIN_2,
	EIC_PIN_3,
	EIC_PIN_MAX,
} mchp_eic_pin_t;

/**
 * @struct eic_config_params
 * @brief Configuration parameters for a Microchip EIC (External Interrupt Controller) line.
 *
 * This structure contains all the configuration parameters required to set up an EIC line,
 * including trigger type, pin number, debounce setting, callback function, and user data.
 */
typedef struct eic_config_params {
	mchp_eic_trigger_t trig_type; /* trigger type */
	mchp_eic_pin_t pin_num;       /* pin number */
	bool debounce;                /* Denotes whether debouncing is enabled for the pin or not */
	void *data; /* This contains the pointer to the data pass to callback function */
	mchp_eic_callback_t eic_pin_callback; /**It contains the address of callback function */
} eic_config_params_t;

/**
 * @brief Configure and enable an EIC interrupt for a given pin.
 *
 * This function configures the EIC interrupt for the specified pin, including setting up
 * the trigger type, enabling input, configuring debounce if required,
 * and updating the internal data structures to reflect the new assignment of pin to an eic pin.
 *
 * @param eic_pin_config Pointer to the EIC pin configuration structure.
 *
 * @return 0 on success.
 */
int eic_mchp_config_interrupt(eic_config_params_t *eic_pin_config);

/**
 * @brief Enables an EIC interrupt for a given pin number.
 *
 * This function enables the EIC interrupt associated with the specified pin configuration.
 *
 * @param pin_num pin number to be enabled.
 *
 * @return 0 on success, or a negative error code on failure.
 */
int eic_mchp_enable_interrupt(mchp_eic_pin_t pin_num);

/**
 * @brief Disables an EIC interrupt for a given pin number.
 *
 * This function disables the EIC interrupt associated with the specified pin configuration.
 *
 * @param pin_num pin number to be disabled.
 *
 * @return 0 on success, or a negative error code on failure.
 */
int eic_mchp_disable_interrupt(mchp_eic_pin_t pin_num);

#endif /*ZEPHYR_INCLUDE_DRIVERS_INTC_MCHP_EIC_G2_H_*/
