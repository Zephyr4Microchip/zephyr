/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file intc_mchp_eic_u2254.c
 * @brief EIC driver implementation for Microchip eic u2254 peripheral
 */

#include <soc.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include <zephyr/drivers/interrupt_controller/intc_mchp_eic_u2254.h>

LOG_MODULE_REGISTER(intc_mchp_eic_u2254, CONFIG_INTC_LOG_LEVEL);
/*******************************************
 * Const and Macro Defines
 *******************************************/
/* Device Tree Driver Compatibility */
#define DT_DRV_COMPAT         microchip_eic_u2254
/* This is used for checking whether the eic line is free or not */
#define INTC_LINE_FREE        0XFF
/* This is the default value of the pin in the `mchp_eic_line_assignment_t`*/
#define INTC_PIN_DEFAULT_VAL  0x1f
/* This is the default value of the port in the `mchp_eic_line_assignment_t`*/
#define INTC_PORT_DEFAULT_VAL 0X7

/* Type definition for the EIC lock. */
#define EIC_LOCK_TYPE uint32_t

/* Acquire the EIC lock */
#define EIC_DATA_LOCK(p_lock) p_lock = irq_lock()

/*Release the EIC lock.*/
#define EIC_DATA_UNLOCK(p_lock) irq_unlock(p_lock)

/* created this by listing down the pin mux table available in the datasheet DS60001507M */
/* Port A */
#define PORTA_UNSUPPORTED_PINS    (BIT(8) | BIT(9) | BIT(26) | BIT(28) | BIT(29))
/* Port B */
/* The special pins need an offset when calculating the eic line */
#define PORTB_SPECIAL_PINS        (BIT(26) | BIT(27) | BIT(28) | BIT(29))
/* Port C */
#define PORTC_UNSUPPORTED_PINS    (BIT(8) | BIT(9) | BIT(14) | BIT(29))
/* The special pins need an offset when calculating the eic line */
#define PORTC_SPECIAL_PINS        (BIT(7))
#define PORTC_SPECIAL_PINS_OFFSET 2
/* Port D */
#define PORTD_SUPPORTED_PINS                                                                       \
	(BIT(0) | BIT(1) | BIT(8) | BIT(9) | BIT(10) | BIT(11) | BIT(12) | BIT(20) | BIT(21))
/* The special pins need an offset when calculating the eic line */
#define PORTD_SPECIAL_PINS_1        (BIT(8) | BIT(9) | BIT(10) | BIT(11) | BIT(12))
#define PORTD_SPECIAL_PINS_2        (BIT(20) | BIT(21))
#define PORTD_SPECIAL_PINS_1_OFFSET 5
#define PORTD_SPECIAL_PINS_2_OFFSET 6

/***********************************
 * Typedefs and Enum Declarations
 ***********************************/
/**
 * @struct mchp_EIC_clock
 * @brief Structure to hold device clock configuration.
 */
typedef struct mchp_eic_clock {

	/* Clock driver */
	const struct device *clock_dev;

	/* Main clock subsystem. */
	clock_control_mchp_subsys_t mclk_sys;

	/* Generic clock subsystem. */
	clock_control_mchp_subsys_t gclk_sys;

} mchp_eic_clock_t;

/*
 * Structure for assigning a pin and port to an EIC line.
 */
typedef struct mchp_eic_line_assignment {
	uint8_t pin: 5;
	uint8_t port: 3;
} mchp_eic_line_assignment_t;

/*
 * Configuration structure for EIC device settings and options.
 */

typedef struct eic_mchp_dev_cfg {
	eic_registers_t *regs;
	mchp_eic_clock_t eic_clock;
	/* Function pointer for configuring EIC interrupts. */
	void (*irq_config)(void);
	/* Enable the low power mode to use the ULP32K clock for EIC */
	bool low_power_mode;
} eic_mchp_dev_cfg_t;

/* Device data structure for containing housekeeping structures */
typedef struct eic_mchp_dev_data {

	/* This is used for checking whether a line is in busy or not.Each bit correspond to an eic
	 * line
	 */
	uint16_t line_busy;

	/* It contains the address of gpio isr function */
	mchp_eic_callback_t eic_line_callback;

	/* This will contain the address of data structure of gpio peripheral. It is used
	 * for callback related functions.
	 */
	void *gpio_data;

	/* Each bit of this uin16_t denotes an eic line. There are MCHP_PORT_ID_MAX number of such
	 * variables. Whenever a eic_line is assigned to a particular port, it is to be updated in
	 * this. This is later used in checking the pending irq
	 */
	mchp_eic_line_assignment_t lines[EIC_LINE_MAX];

	/*Each line will have its own structure
	 *for keeping its data
	 */
	uint16_t port_assigned_line[MCHP_PORT_ID_MAX];
	EIC_LOCK_TYPE lock;
} eic_mchp_dev_data_t;

/***********************************
 * Internal functions
 ***********************************/
/**
 * Find the EIC (External Interrupt Controller) line corresponding to a given port and pin.
 *
 * This function determines the EIC line number for a specified port and pin combination.
 * It takes into account unsupported and special pins for each port, and applies the necessary
 * offsets or marks the line as free if the pin is not supported.
 *
 * It returns the EIC line number corresponding to the given port and pin.
 *         If the pin is unsupported, returns INTC_LINE_FREE.
 *
 * @note The function relies on several macros and constants:
 *       - BIT(pin): Macro to generate a bitmask for the pin.
 *       - MCHP_PORT_ID0, MCHP_PORT_ID1, etc.: Port identifiers.
 *       - PORTA_UNSUPPORTED_PINS, PORTB_SPECIAL_PINS, etc.: Pin masks for special/unsupported pins.
 *       - INTC_LINE_FREE: Value indicating the line is not available.
 *       - PORTD_SPECIAL_PINS_1_OFFSET, PORTD_SPECIAL_PINS_2_OFFSET: Offsets for special pins.
 */
uint8_t find_eic_line_from_pin(int port, int pin)
{
	uint8_t eic_line = pin % 16;
	uint32_t pin_mask = BIT(pin);

	switch (port) {
	case MCHP_PORT_ID0:
		if ((PORTA_UNSUPPORTED_PINS & pin_mask) != 0) {
			eic_line = INTC_LINE_FREE;
		}
		break;
	case MCHP_PORT_ID1:
		if ((PORTB_SPECIAL_PINS & pin_mask) != 0) {
			eic_line += 2;
		}
		break;
	case MCHP_PORT_ID2:
		if ((PORTC_UNSUPPORTED_PINS & pin_mask) != 0) {
			eic_line = INTC_LINE_FREE;
		} else if ((PORTC_SPECIAL_PINS & pin_mask) != 0) {
			eic_line += PORTC_SPECIAL_PINS_OFFSET;
		}
		break;
	case MCHP_PORT_ID3:
		if ((PORTD_SUPPORTED_PINS & pin_mask) == 0) {
			eic_line = INTC_LINE_FREE;
		} else if ((PORTD_SPECIAL_PINS_2 & pin_mask) != 0) {
			eic_line += PORTD_SPECIAL_PINS_2_OFFSET;
		} else if ((PORTD_SPECIAL_PINS_1 & pin_mask) != 0) {
			eic_line -= PORTD_SPECIAL_PINS_1_OFFSET;
		}
		break;
	}
	return eic_line;
}

/*
 * Waits until EIC synchronization is complete.
 */
static inline void eic_sync_wait(eic_registers_t *eic_reg)
{
	LOG_DBG("Reached in sync wait");
	while (eic_reg->EIC_SYNCBUSY != 0) {
	}
}

/*
 *Enables or disables the EIC peripheral.
 */
static inline void eic_enable(eic_registers_t *regs, bool enable)
{
	if (enable == true) {
		regs->EIC_CTRLA |= EIC_CTRLA_ENABLE_Msk;
	} else {
		regs->EIC_CTRLA &= ~EIC_CTRLA_ENABLE_Msk;
	}
}
/***********************************
 * Zephyr APIs
 ***********************************/

/**
 * @brief Enable the EIC interrupt for a specific port and pin combination
 *
 * @param port port index (A=0, etc)
 * @param pin pin in the port
 */
static void enable_interrupt_line(eic_registers_t *regs, uint8_t eic_line, bool enable)
{
	uint16_t pin_mask = BIT(eic_line);

	if (enable == true) {
		regs->EIC_INTFLAG = pin_mask;
		regs->EIC_INTENSET |= pin_mask;
	} else {
		regs->EIC_INTENCLR |= pin_mask;
	}
}
/**
 * This function disables the EIC interrupt associated with the specified pin configuration.
 * It checks if the pin is currently assigned to an EIC line, disables the interrupt line if so,
 * and updates the internal data structures to mark the line as free.
 */
int eic_mchp_disable_interrupt(eic_config_params_t *eic_pin_config)
{
	const struct device *const dev = DEVICE_DT_INST_GET(0);
	const eic_mchp_dev_cfg_t *eic_cfg = dev->config;
	eic_mchp_dev_data_t *eic_data = dev->data;
	int ret_val = 0;

	EIC_DATA_LOCK(eic_data->lock);
	LOG_DBG("port = %p pint = %d", eic_pin_config->port_addr, eic_pin_config->pin_num);
	do {
		/*Check whether the pin was assigned to an eic line.*/
		uint8_t eic_line =
			find_eic_line_from_pin(eic_pin_config->port_id, eic_pin_config->pin_num);

		if ((eic_data->line_busy & BIT(eic_line)) != 0) {
			enable_interrupt_line(eic_cfg->regs, eic_line, false);
		} else {
			LOG_ERR("EIC Line is already free");
			break;
		}
		/*Remove the connection from EIC peripheral*/
		eic_pin_config->port_addr->PORT_PINCFG[eic_pin_config->pin_num] &=
			(~PORT_PINCFG_PMUXEN(1));
		/*Clear the pin number and port number from the structure which holds the status of
		 * each eic line and make it free
		 */
		eic_data->line_busy &= ~BIT(eic_line);
		/* Remove the assigned pin and port from lines structure */
		eic_data->lines[eic_line].pin = INTC_PIN_DEFAULT_VAL;
		eic_data->lines[eic_line].port = INTC_PORT_DEFAULT_VAL;
		/*Remove the assigned EIC line from the port data*/
		eic_data->port_assigned_line[eic_pin_config->port_id] &= ~BIT(eic_line);
	} while (0);
	EIC_DATA_UNLOCK(eic_data->lock);
	return ret_val;
}
/**
 * This function checks which pins in the specified port have pending EIC interrupts.
 * It returns a bitmask where each set bit corresponds to a pin with a pending interrupt.
 */
uint32_t eic_mchp_interrupt_pending(uint8_t port_id)
{
	const struct device *const dev = DEVICE_DT_INST_GET(0);
	const eic_mchp_dev_cfg_t *eic_cfg = dev->config;
	eic_mchp_dev_data_t *eic_data = dev->data;
	uint8_t pin = 0;
	uint8_t eic_line = 0;
	/* Contains the bit mask where each set bit corresponds to a pending interrupt */
	uint32_t ret_val = 0;

	EIC_DATA_LOCK(eic_data->lock);
	do {
		if (port_id >= MCHP_PORT_ID_MAX) {
			LOG_ERR("Invalid port id passed");
			break;
		}
		uint16_t port_flagged_lines = eic_data->port_assigned_line[port_id];
		/* Gets the interrupt pending flags which are relevant to the give port id */
		port_flagged_lines = eic_cfg->regs->EIC_INTFLAG & port_flagged_lines;
		/* Get the trailing zeroes to find the eic_line number.
		 * Use the eic_line to get the pin number
		 * then use the BIT(n) function to create a mask with each bit corresponding to a
		 * pin of the port and return it to the user
		 */
		while (port_flagged_lines != 0) {
			eic_line = __builtin_ctz(port_flagged_lines);
			port_flagged_lines &= (port_flagged_lines - 1);
			pin = eic_data->lines[eic_line].pin;
			ret_val |= BIT(pin);
		}
	} while (0);
	EIC_DATA_UNLOCK(eic_data->lock);
	return ret_val;
}

/**
 * This function configures the EIC interrupt for the specified pin, including setting up
 * the trigger type, enabling input, configuring debounce if required,
 * and updating the internal data structures to reflect the new assignment of pin to an eic line.
 */
int eic_mchp_config_interrupt(eic_config_params_t *eic_pin_config)
{
	const struct device *const dev = DEVICE_DT_INST_GET(0);
	const eic_mchp_dev_cfg_t *eic_cfg = dev->config;
	eic_mchp_dev_data_t *eic_data = dev->data;
	uint8_t pin = eic_pin_config->pin_num;
	uint8_t pmux_offset = pin / 2;
	int eic_line = INTC_LINE_FREE;
	int ret_val = 0;

	eic_data->gpio_data = eic_pin_config->gpio_data;
	eic_data->eic_line_callback = eic_pin_config->eic_line_callback;
	EIC_DATA_LOCK(eic_data->lock);
	do {
		/* Find the eic line of a given pin of the given port. If no eic line associated or
		 * eic line is busy, return failure
		 */
		eic_line = find_eic_line_from_pin(eic_pin_config->port_id, pin);
		LOG_DBG("eic line of port %d pin %d = %d", eic_pin_config->port_id, pin, eic_line);

		if (eic_line == INTC_LINE_FREE) {
			LOG_ERR("no associated eic line found");
			ret_val = -ENOTSUP;
			break;
		}
		/* If the line is not free then return failure */
		if ((eic_data->line_busy & BIT(eic_line)) != 0) {
			LOG_ERR("EIC Line for port %d : %d is busy", eic_pin_config->port_id, pin);
			ret_val = -EBUSY;
			break;
		}
		eic_enable(eic_cfg->regs, false);
		/* Configure the pin as input and connect it to eic peripheral */
		eic_pin_config->port_addr->PORT_PINCFG[pin] =
			PORT_PINCFG_PMUXEN(1) | PORT_PINCFG_INEN(1);
		eic_pin_config->port_addr->PORT_PMUX[pmux_offset] = 0;

		/* - The bit position for respective eic line in the config register is calculated
		 *   and written into the respective config register
		 * - The `eic_line>>3` will find out which config register(0 OR 1) to write the
		 *   trigger type to.
		 * - `eic_line%8` is required to find the offset of the eic line inside the config
		 *   register
		 */

		eic_cfg->regs->EIC_CONFIG[(eic_line >> 3)] = (eic_pin_config->trig_type)
							     << (4 * (eic_line % 8));
		/* Set the debouncing feature of the eic line if required */
		if (eic_pin_config->debounce != 0) {
			eic_cfg->regs->EIC_DEBOUNCEN = BIT(eic_line);
		}
		LOG_DBG("%s",
			eic_pin_config->debounce ? "debouncing enabled" : "debouncing disabled");

		enable_interrupt_line(eic_cfg->regs, eic_line, true);

		eic_enable(eic_cfg->regs, true);

		/*House keeping */
		eic_data->line_busy |= BIT(eic_line);
		eic_data->lines[eic_line].pin = pin;
		eic_data->lines[eic_line].port = eic_pin_config->port_id;
		eic_data->port_assigned_line[eic_pin_config->port_id] |= BIT(eic_line);
	} while (0);
	EIC_DATA_UNLOCK(eic_data->lock);
	return ret_val;
}

/**
 * @brief Initialize the Microchip EIC peripheral.
 *
 * This function performs all necessary hardware and software initialization for the EIC,
 * including clock setup, software reset, low power mode configuration, and enabling the EIC.
 *
 * @param dev Pointer to the device structure for the EIC instance.
 *
 * @return 0 on success.
 *
 * @note
 * - The function performs a software reset and waits for synchronization before enabling the EIC.
 * - If low power mode is enabled, the EIC is configured to use the ULP32K clock.
 */
static int eic_mchp_init(const struct device *dev)
{
	const eic_mchp_dev_cfg_t *eic_cfg = dev->config;
	int ret_val = 0;

	do {
		ret_val = clock_control_on(eic_cfg->eic_clock.clock_dev,
					   (clock_control_subsys_t)&eic_cfg->eic_clock.mclk_sys);
		if ((ret_val < 0) && (ret_val != -EALREADY)) {
			LOG_ERR("Clock control on failed for mclk %d", ret_val);
			break;
		}
		ret_val = clock_control_on(eic_cfg->eic_clock.clock_dev,
					   (clock_control_subsys_t)&eic_cfg->eic_clock.gclk_sys);
		if ((ret_val < 0) && (ret_val != -EALREADY)) {
			LOG_ERR("Clock control on failed for gclk %d", ret_val);
			break;
		}
		eic_cfg->irq_config();

		eic_cfg->regs->EIC_CTRLA = EIC_CTRLA_SWRST(1);
		eic_sync_wait(eic_cfg->regs);

		if (eic_cfg->low_power_mode == true) {
			eic_cfg->regs->EIC_CTRLA = EIC_CTRLA_CKSEL(EIC_CTRLA_CKSEL_CLK_ULP32K);
		}
		eic_cfg->regs->EIC_CTRLA = EIC_CTRLA_ENABLE(1);
		eic_sync_wait(eic_cfg->regs);

		LOG_DBG("EIC initialisation done 0x%p", eic_cfg->regs);
	} while (0);
	ret_val = (ret_val == -EALREADY) ? 0 : ret_val;
	return ret_val;
}

/**
 * @brief EIC interrupt service routine for a specific EIC line.
 *
 * This ISR clears the interrupt flag for the specified EIC line and invokes the registered
 * callback, if any, passing the pin mask and user data.
 *
 *
 * @param dev Pointer to the device structure for the EIC instance.
 *
 * - The interrupt flag is cleared by writing to the EIC_INTFLAG register.
 * - The callback is called only if it is not NULL.
 */
#define EIC_MCHP_CB_INIT(eic_line, _)                                                              \
	static void eic_mchp_isr_##eic_line(const struct device *dev)                              \
	{                                                                                          \
		const eic_mchp_dev_cfg_t *eic_cfg = dev->config;                                   \
		eic_mchp_dev_data_t *eic_data = dev->data;                                         \
                                                                                                   \
		eic_cfg->regs->EIC_INTFLAG = BIT(EIC_LINE_##eic_line);                             \
		if (eic_data->eic_line_callback != NULL) {                                         \
			uint32_t pins = BIT(eic_data->lines[EIC_LINE_##eic_line].pin);             \
			eic_data->eic_line_callback(pins, eic_data->gpio_data);                    \
		}                                                                                  \
	}

#define EIC_MCHP_IRQ_CONNECT(eic_line, inst)                                                       \
	IF_ENABLED(DT_INST_IRQ_HAS_IDX(inst, eic_line), (  \
	do {\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, eic_line, irq),  \
			    DT_INST_IRQ_BY_IDX(inst, eic_line, priority), eic_mchp_isr_##eic_line,\
			    DEVICE_DT_INST_GET(inst), inst);\
		irq_enable(DT_INST_IRQ_BY_IDX(inst, eic_line, irq));\
	} while (false);\
			))

#define EIC_MCHP_DATA_DEFN(n) static eic_mchp_dev_data_t eic_mchp_data_##n

/**
 * @brief Macro to define the EIC clock configuration.
 *
 * This macro defines the clock configuration for the External Interrupt Controller (EIC) for a
 * given instance. It initializes the clock device and the main clock system (MCLK) for the EIC.
 *
 * @param n The instance number of the EIC.
 */
#define EIC_MCHP_CLOCK_DEFN(n)                                                                     \
	.eic_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                                 \
	.eic_clock.mclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, mclk)),         \
			       .id = DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, id)},                    \
	.eic_clock.gclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, gclk)),         \
			       .id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, id)}

/*
 * Define the EIC device configuration for instance n.
 */
#define EIC_MCHP_CFG_DEFN(n)                                                                       \
	static const eic_mchp_dev_cfg_t eic_mchp_dev_cfg_##n = {                                   \
		.regs = (eic_registers_t *)DT_INST_REG_ADDR(n),                                    \
		EIC_MCHP_CLOCK_DEFN(n),                                                            \
		.irq_config = eic_irq_connect_##n,                                                 \
		.low_power_mode = DT_INST_PROP(n, low_power_mode)}

/**
 * @brief Declare the EIC IRQ connection handler for a specific instance.
 *
 * This macro defines a static function prototype for connecting
 * the EIC interrupt handlers of a given EIC instance.
 *
 * @param n Instance index of the EIC controller.
 */
#define EIC_MCHP_IRQ_HANDLER_DECL(n) static void eic_irq_connect_##n(void)

/**
 * @brief Define the EIC IRQ handler function for a given instance.
 *
 * This macro generates a function to connect all IRQ lines of a EIC instance
 * by calling EIC_MCHP_IRQ_CONNECT for each available IRQ in the instance.
 *
 * @param n The EIC instance number.
 */
#define EIC_MCHP_IRQ_HANDLER(n)                                                                    \
	static void eic_irq_connect_##n(void)                                                      \
	{                                                                                          \
		/** Connect all IRQs for this instance */                                          \
		LISTIFY(\
			DT_NUM_IRQS(DT_DRV_INST(n)), \
			EIC_MCHP_IRQ_CONNECT, \
			(), \
			n\
		)                                                                          \
	}
/*
 * Create interrupt handlers for each IRQ of the EIC instance. Listify
 */
#define EIC_MCHP_CREATE_HANDLERS(n) LISTIFY(DT_NUM_IRQS(DT_DRV_INST(n)), EIC_MCHP_CB_INIT, (;),)

/*
 * Initialize EIC device instance and related handlers, data, and config.
 */
#define EIC_MCHP_DEVICE_INIT(n)                                                                    \
	EIC_MCHP_CREATE_HANDLERS(n);                                                               \
	EIC_MCHP_IRQ_HANDLER_DECL(n);                                                              \
	EIC_MCHP_DATA_DEFN(n);                                                                     \
	EIC_MCHP_CFG_DEFN(n);                                                                      \
	DEVICE_DT_INST_DEFINE(n, eic_mchp_init, NULL, &eic_mchp_data_##n, &eic_mchp_dev_cfg_##n,   \
			      PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY, NULL);                      \
	EIC_MCHP_IRQ_HANDLER(n)

/**
 * @brief Initialize all EIC instances.
 */
DT_INST_FOREACH_STATUS_OKAY(EIC_MCHP_DEVICE_INIT)
