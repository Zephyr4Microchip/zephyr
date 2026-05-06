/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file intc_mchp_eic_g2.c
 * @brief EIC driver implementation for Microchip eic g2 peripheral
 */
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/interrupt_controller/intc_mchp_eic_g2.h>

/******************************************************************************
 * @brief Devicetree definitions
 *****************************************************************************/
#define DT_DRV_COMPAT microchip_eic_g2_intc

LOG_MODULE_REGISTER(intc_mchp_eic_g2, CONFIG_INTC_LOG_LEVEL);

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

	/* Generic clock subsystem. */
	clock_control_subsys_t gclk_sys;

} mchp_eic_clock_t;

/*
 * Configuration structure for EIC device settings and options.
 */

typedef struct eic_mchp_dev_cfg {
	eic_registers_t *regs;
	mchp_eic_clock_t eic_clock;
	const struct pinctrl_dev_config *pcfg;
	/* Function pointer for configuring EIC interrupts. */
	void (*irq_config)(void);
	/* Enable the low power mode to use the ULP32K clock for EIC */
	bool low_power_mode;
} eic_mchp_dev_cfg_t;

typedef struct eic_mchp_cb_obj {
	/* External Interrupt Pin Callback Handler */
	mchp_eic_callback_t callback;

	/* External Interrupt Pin Client context */
	void *context;
} eic_mchp_cb_obj_t;

/* Device data structure for containing housekeeping structures */
typedef struct eic_mchp_dev_data {
	eic_mchp_cb_obj_t pin_data[EIC_PIN_MAX];
} eic_mchp_dev_data_t;

/***********************************
 * Internal functions
 ***********************************/

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

/***********************************
 * Zephyr APIs
 ***********************************/
/**
 * This function enables the EIC interrupt associated with the specified pin number.
 */
int eic_mchp_enable_interrupt(mchp_eic_pin_t pin_num)
{
	const struct device *const dev = DEVICE_DT_INST_GET(0);
	const eic_mchp_dev_cfg_t *eic_cfg = dev->config;
	int ret_val = 0;
	uint32_t key;

	key = irq_lock();
	LOG_DBG("pin = %d", pin_num);
	do {
		if (pin_num < EIC_PIN_MAX) {
			enable_interrupt_line(eic_cfg->regs, pin_num, true);
		} else {
			LOG_ERR("cannot find eic line for the specified pin");
			ret_val = -EINVAL;
			break;
		}
	} while (0);
	irq_unlock(key);
	return ret_val;
}

/**
 * This function disables the EIC interrupt associated with the specified pin number.
 */
int eic_mchp_disable_interrupt(mchp_eic_pin_t pin_num)
{
	const struct device *const dev = DEVICE_DT_INST_GET(0);
	const eic_mchp_dev_cfg_t *eic_cfg = dev->config;
	int ret_val = 0;
	uint32_t key;

	key = irq_lock();
	LOG_DBG("pin = %d", pin_num);
	do {
		if (pin_num < EIC_PIN_MAX) {
			enable_interrupt_line(eic_cfg->regs, pin_num, false);
		} else {
			LOG_ERR("cannot find eic line for the specified pin");
			ret_val = -EINVAL;
			break;
		}
	} while (0);
	irq_unlock(key);
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
	uint32_t key;

	eic_data->pin_data[eic_pin_config->pin_num].context = eic_pin_config->data;
	eic_data->pin_data[eic_pin_config->pin_num].callback = eic_pin_config->eic_pin_callback;
	key = irq_lock();
	do {
		eic_enable(eic_cfg->regs, false);

		eic_cfg->regs->EIC_CONFIG |= (eic_pin_config->trig_type | 0x8)
					     << (4 * eic_pin_config->pin_num);

		/* Set the debouncing feature of the eic line if required */
		if (eic_pin_config->debounce != 0) {
			eic_cfg->regs->EIC_DEBOUNCEN = BIT(eic_pin_config->pin_num);
		}
		LOG_DBG("%s",
			eic_pin_config->debounce ? "debouncing enabled" : "debouncing disabled");

		enable_interrupt_line(eic_cfg->regs, eic_pin_config->pin_num, true);

		eic_enable(eic_cfg->regs, true);
	} while (0);
	irq_unlock(key);
	return 0;
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
static void eic_mchp_isr(const struct device *dev)
{
	const eic_mchp_dev_cfg_t *eic_cfg = dev->config;
	eic_mchp_dev_data_t *eic_data = dev->data;
	uint8_t currentChannel;
	uint32_t eicIntFlagStatus;

	/* Find any triggered channels, run associated callback handlers */
	for (currentChannel = 0U; currentChannel < EIC_PIN_MAX; currentChannel++) {
		/* Read the interrupt flag status */
		eicIntFlagStatus = eic_cfg->regs->EIC_INTFLAG & (1UL << currentChannel);

		if (0U != eicIntFlagStatus) {
			/* Find any associated callback entries in the callback table */
			if (eic_data->pin_data[currentChannel].callback != NULL) {
				eic_data->pin_data[currentChannel].callback(
					eic_data->pin_data[currentChannel].context);
			}

			/* Clear interrupt flag */
			eic_cfg->regs->EIC_INTFLAG = (1UL << currentChannel);
		}
	}
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
		ret_val =
			clock_control_on(eic_cfg->eic_clock.clock_dev, eic_cfg->eic_clock.gclk_sys);
		if ((ret_val < 0) && (ret_val != -EALREADY)) {
			LOG_ERR("Clock control on failed for gclk %d", ret_val);
			break;
		}
		eic_cfg->irq_config();

		eic_cfg->regs->EIC_CTRLA |= EIC_CTRLA_SWRST(1);
		eic_sync_wait(eic_cfg->regs);

		if (eic_cfg->low_power_mode == true) {
			eic_cfg->regs->EIC_CTRLA |= EIC_CTRLA_CKSEL_Msk;
		}
		eic_cfg->regs->EIC_CTRLA |= EIC_CTRLA_ENABLE(1);
		eic_sync_wait(eic_cfg->regs);

		ret_val = pinctrl_apply_state(eic_cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret_val < 0) {
			LOG_ERR("pinctrl on failed for eic %d", ret_val);
			return ret_val;
		}

		LOG_DBG("EIC initialisation done 0x%p", eic_cfg->regs);
	} while (0);
	ret_val = (ret_val == -EALREADY) ? 0 : ret_val;
	return ret_val;
}

#define EIC_MCHP_IRQ_CONNECT(eic_line, inst)                                                       \
	IF_ENABLED(DT_INST_IRQ_HAS_IDX(inst, eic_line), (  \
	do {\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, eic_line, irq),  \
				DT_INST_IRQ_BY_IDX(inst, eic_line, priority), eic_mchp_isr,\
				DEVICE_DT_INST_GET(inst), inst);\
		irq_enable(DT_INST_IRQ_BY_IDX(inst, eic_line, irq));\
	} while (false);\
			))

#define EIC_MCHP_DATA_DEFN(n) static eic_mchp_dev_data_t eic_mchp_data_##n

/**
 * @brief Macro to define the EIC clock configuration.
 *
 * This macro defines the clock configuration for the External Interrupt Controller (EIC) for a
 * given instance.
 *
 * @param n The instance number of the EIC.
 */
#define EIC_MCHP_CLOCK_DEFN(n)                                                                     \
	.eic_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                                 \
	.eic_clock.gclk_sys = (void *)DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, subsystem)

/*
 * Define the EIC device configuration for instance n.
 */
#define EIC_MCHP_CFG_DEFN(n)                                                                       \
	static const eic_mchp_dev_cfg_t eic_mchp_dev_cfg_##n = {                                   \
		.regs = (eic_registers_t *)DT_INST_REG_ADDR(n),                                    \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),                                         \
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
 * Initialize EIC device instance and related handlers, data, and config.
 */
#define EIC_MCHP_DEVICE_INIT(n)                                                                    \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
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
