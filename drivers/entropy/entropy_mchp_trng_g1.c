/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file entropy_mchp_trng_g1.c
 * @brief Microchip g1 True Random Number Generator (TRNG) Entropy driver for Zephyr
 *
 * This file implements the driver for the Microchip g1 TRNG hardware,
 * providing entropy services to the Zephyr OS entropy subsystem.
 *
 */
#include <soc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>

/*******************************************
 * @brief Devicetree definitions
 *******************************************/
#define DT_DRV_COMPAT microchip_trng_g1_entropy

/*******************************************
 * Const and Macro Defines
 *******************************************/
LOG_MODULE_REGISTER(entropy_mchp_trng_g1, CONFIG_ENTROPY_LOG_LEVEL);

/** @brief Macro indicating successful operation. */
#define ENTROPY_MCHP_SUCCESS 0

/** @brief Timeout value for TRNG readiness polling. */
#define TRNG_TIMEOUT 1000000

/** @brief Use busy-waiting to get entropy data. */
#define ENTROPY_BLOCKING ENTROPY_BUSYWAIT

/** @brief Do not block when getting entropy data. */
#define ENTROPY_NON_BLOCKING 0

/** @brief Macro to access the TRNG register block from the device config. */
#define ENTROPY_REGS ((const entropy_mchp_config_t *)(dev)->config)->regs

/**
 * @brief Type for the entropy data ready semaphore.
 *
 * Defines the semaphore type used for synchronizing entropy operations.
 */
#define ENTROPY_DATA_RDY_LOCK_TYPE struct k_sem

/**
 * @brief Timeout for acquiring the entropy data ready semaphore.
 *
 * Specifies the timeout (in milliseconds) to wait for the semaphore.
 */
#define ENTROPY_DATA_RDY_SEM_TIMEOUT K_MSEC(1)

/**
 * @brief Initial count for the entropy data ready semaphore.
 *
 * The semaphore starts with a count of 0, so the first take will block until released.
 */
#define ENTROPY_DATA_RDY_SEM_INIT_COUNT 0

/**
 * @brief Maximum count for the entropy data ready semaphore.
 *
 * Sets the maximum number of times the semaphore can be given.
 */
#define ENTROPY_DATA_RDY_SEM_LIMIT 1

/**
 * @brief Initialize the entropy data ready semaphore.
 *
 * Initializes the semaphore with the defined initial count and limit.
 *
 * @param p_sem Pointer to the semaphore to initialize.
 */
#define ENTROPY_DATA_RDY_SEM_INIT(p_sem)                                                           \
	k_sem_init(p_sem, ENTROPY_DATA_RDY_SEM_INIT_COUNT, ENTROPY_DATA_RDY_SEM_LIMIT)

/**
 * @brief Take the entropy data ready semaphore.
 *
 * Attempts to take the semaphore, blocking for the defined timeout.
 *
 * @param p_sem Pointer to the semaphore to take.
 * @return 0 on success, negative error code on failure or timeout.
 */
#define ENTROPY_DATA_RDY_SEM_TAKE(p_sem)                                                           \
	k_sem_take((p_sem), k_is_in_isr() ? K_NO_WAIT : ENTROPY_DATA_RDY_SEM_TIMEOUT)

/**
 * @brief Give the entropy data ready semaphore.
 *
 * Releases the semaphore to signal that entropy data is ready.
 *
 * @param p_sem Pointer to the semaphore to give.
 */
#define ENTROPY_DATA_RDY_SEM_GIVE(p_sem) k_sem_give(p_sem)

/*******************************************
 * @brief Data type definitions
 ******************************************/
/**
 * @enum entropy_mchp_runstandby_t
 * @brief Run standby mode configuration for TRNG peripheral.
 *
 * This enumeration defines the possible states for enabling or disabling
 * run standby mode in the TRNG peripheral.
 */
typedef enum {
	ENTROPY_RUNSTANDBY_DISABLED = 0,
	ENTROPY_RUNSTANDBY_ENABLED
} entropy_mchp_runstandby_t;

/**
 * @struct entropy_mchp_clock
 * @brief Clock configuration structure for entropy (TRNG) peripheral.
 *
 * This structure defines the clock configuration for the entropy (TRNG)
 *  peripheral, including the clock device and subsystem.
 */
#if defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ2)
typedef struct entropy_mchp_clock {
	const struct device *clock_dev;
} entropy_mchp_clock_t;
#else
typedef struct entropy_mchp_clock {
	const struct device *clock_dev;
	clock_control_subsys_t mclk_sys;
} entropy_mchp_clock_t;
#endif

/**
 * @struct entropy_mchp_config_t
 * @brief Entropy configuration structure.
 *
 * This structure defines the configuration for the entropy (TRNG) peripheral.
 */
typedef struct entropy_mchp_config {

	/** Pointer to the TRNG registers */
	trng_registers_t *regs;

	/** Clock control configuration for the TRNG subsystem */
	entropy_mchp_clock_t entropy_clock;

	/**< Function to configure IRQ */
	void (*irq_config_func)(const struct device *dev);

	/* Enable peripheral operation in standby sleep mode. */
	uint8_t run_in_standby;

} entropy_mchp_config_t;

/**
 * @struct entropy_mchp_dev_data
 * @brief Structure to hold entropy device data.
 */
typedef struct entropy_mchp_dev_data {

	/**< Pointer to the entropy device instance. */
	const struct device *dev;

	/**< Semaphore lock for entropy interrupt operations */
	ENTROPY_DATA_RDY_LOCK_TYPE entropy_data_rdy_sem;

	/**< Array of pointers to NVM data registers */
	volatile uint32_t trng_data;

} entropy_mchp_dev_data_t;

/*******************************************
 * @brief Helper functions
 ******************************************/
/**
 * @brief Enable the TRNG Data Ready interrupt.
 *
 * This function enables the Data Ready interrupt for the TRNG peripheral.
 *
 * @param dev Pointer to the device structure.
 */
static void entropy_trng_interrupt_enable(const struct device *dev)
{
	/* Enable the Data Ready interrupt */
	ENTROPY_REGS->TRNG_INTENSET = TRNG_INTENSET_DATARDY(1);
}

/**
 * @brief Disable the TRNG Data Ready interrupt.
 *
 * This function disables the Data Ready interrupt for the TRNG peripheral.
 *
 * @param dev Pointer to the device structure.
 */
static void entropy_trng_interrupt_disable(const struct device *dev)
{
	/* Disable the Data Ready interrupt */
	ENTROPY_REGS->TRNG_INTENCLR = TRNG_INTENCLR_DATARDY(1);
}

/**
 * @brief Enable the TRNG module.
 *
 * This function enables the TRNG peripheral module.
 *
 * @param dev Pointer to the device structure.
 */
static void entropy_trng_enable(const struct device *dev)
{
	/* Enable TRNG module */
	ENTROPY_REGS->TRNG_CTRLA |= TRNG_CTRLA_ENABLE(1);
}

/**
 * @brief Check if TRNG data is ready.
 *
 * This function checks if the TRNG has generated new random data.
 *
 * @param dev Pointer to the device structure for the TRNG peripheral.
 *
 * @retval 0 if data is ready
 * @retval 1 if not ready
 */
static inline uint8_t entropy_ready(const struct device *dev)
{
	entropy_mchp_dev_data_t *mchp_entropy_data = dev->data;

	return ENTROPY_DATA_RDY_SEM_TAKE(&mchp_entropy_data->entropy_data_rdy_sem);
}

/**
 * @brief Enable or disable run in standby mode for the TRNG peripheral.
 *
 * This function sets the run in standby mode for the TRNG peripheral based on the device
 * configuration.
 *
 * @param dev Pointer to the device structure.
 */
static void entropy_runstandby_enable(const struct device *dev)
{
	const entropy_mchp_config_t *const entropy_cfg = dev->config;

	if (entropy_cfg->run_in_standby == ENTROPY_RUNSTANDBY_ENABLED) {
		ENTROPY_REGS->TRNG_CTRLA |= TRNG_CTRLA_RUNSTDBY(1);
	} else {
		ENTROPY_REGS->TRNG_CTRLA &= ~TRNG_CTRLA_RUNSTDBY(1);
	}
}

/**
 * @brief Wait for TRNG to be ready.
 *
 * This function waits for the True Random Number Generator (TRNG) to be ready
 * by checking the readiness status. It will timeout if the TRNG is not ready
 * within a specified period.
 *
 * @param dev Pointer to the device structure for the TRNG peripheral.
 *
 * @retval 0 on success
 * @retval -ETIMEDOUT on timeout
 */
static int entropy_wait_ready(const struct device *dev)
{
	/* Wait for the TRNG to be ready */
	int timeout = TRNG_TIMEOUT; /* Timeout value */
	int ret = 0;

	while (entropy_ready(dev) != 0) {
		if (timeout == 0) {
			ret = -ETIMEDOUT;
			break;
		}
		timeout--;
	}

	return ret;
}

/**
 * @brief Read entropy data from the TRNG.
 *
 * This function reads random data from the TRNG hardware into the provided buffer.
 * It supports both blocking and non-blocking modes.
 *
 * @param dev Pointer to the device structure for the TRNG peripheral.
 * @param buffer Pointer to the buffer to store random data.
 * @param length Number of bytes to read.
 * @param flags Flags to control blocking/non-blocking behavior.
 *
 * @retval Number of bytes read on success
 * @retval -EAGAIN if data is not ready (non-blocking)
 * @retval -ETIMEDOUT if TRNG is not ready in time (blocking)
 * @retval -EINVAL on invalid arguments
 */
static int entropy_read(const struct device *dev, uint8_t *buffer, uint16_t length, uint32_t flags)
{
	int ret = -EINVAL;
	uint16_t cnt = length;
	entropy_mchp_dev_data_t *mchp_entropy_data = dev->data;

	while (length > 0) {

		size_t to_copy = 0;

		/* Enable DATARDY interrupt */
		entropy_trng_interrupt_enable(dev);

		if ((flags & ENTROPY_BUSYWAIT) != 0U) {
			int wait = entropy_wait_ready(dev);

			if (wait < 0) {
				ret = wait;
				break;
			}
		} else {
			if (entropy_ready(dev) != 0) {
				ret = -EAGAIN;
				break;
			}
			ret = (int)(cnt - length);
		}

		to_copy = MIN(length, sizeof(mchp_entropy_data->trng_data));

		uint8_t *src = (uint8_t *)&mchp_entropy_data->trng_data;
		uint8_t *dst = buffer;

		for (size_t i = 0; i < to_copy; i++) {
			*dst++ = *src++;
		}

		buffer += to_copy;
		length -= to_copy;
	}

	if (length == 0) {
		ret = (int)cnt;
	}

	return ret;
}

/******************************************************************************
 * @brief API functions
 *****************************************************************************/
/**
 * @brief Get entropy (random data) from the TRNG (blocking).
 *
 * This function retrieves random data from the TRNG in a blocking manner.
 *
 * @param dev Pointer to the device structure for the TRNG peripheral.
 * @param buffer Pointer to the buffer to store random data.
 * @param length Number of bytes to read.
 *
 * @retval 0 on success
 * @retval -EINVAL on invalid arguments or failure
 */
static int entropy_mchp_get_entropy(const struct device *dev, uint8_t *buffer, uint16_t length)
{
	int ret = -EINVAL;

	if (length > 0) {

		/* Always blocking in this API */
		int bk_ret = entropy_read(dev, buffer, length, ENTROPY_BLOCKING);

		if (bk_ret == length) {
			ret = 0;
		}
	} else {
		LOG_DBG("Invalid length:error %d", ret);
	}

	return ret;
}

/**
 * @brief Get entropy (random data) from the TRNG (ISR context).
 *
 * This function retrieves random data from the TRNG, supporting both blocking
 * and non-blocking modes, suitable for use in interrupt service routines.
 *
 * @param dev Pointer to the device structure for the TRNG peripheral.
 * @param buffer Pointer to the buffer to store random data.
 * @param length Number of bytes to read.
 * @param flags Flags to control blocking/non-blocking behavior.
 *
 * @retval Number of bytes read on success
 * @retval -EAGAIN if data is not ready (non-blocking)
 * @retval -EINVAL on invalid arguments
 */
static int entropy_mchp_get_entropy_isr(const struct device *dev, uint8_t *buffer, uint16_t length,
					uint32_t flags)
{
	int ret = -EINVAL;

	if (length > 0) {
		if ((flags & ENTROPY_BUSYWAIT) == 0U) {

			/* Non-blocking: only read if ready */
			ret = entropy_read(dev, buffer, length, ENTROPY_NON_BLOCKING);

		} else {

			/* Blocking: wait until ready */
			ret = entropy_read(dev, buffer, length, ENTROPY_BLOCKING);
		}
	}

	return ret;
}

/**
 * @brief Initialize the entropy device.
 *
 * This function initializes the entropy device.
 *
 * @param dev Pointer to the device structure.
 *
 * @return 0 on success.
 */
static int entropy_mchp_init(const struct device *dev)
{
	const entropy_mchp_config_t *entropy_cfg = dev->config;
	entropy_mchp_dev_data_t *mchp_entropy_data = dev->data;
	int ret = ENTROPY_MCHP_SUCCESS;

#if defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ2)
	CFG_REGS->CFG_PMD3 &= (~0x40);
#else
	ret = clock_control_on(entropy_cfg->entropy_clock.clock_dev,
			       entropy_cfg->entropy_clock.mclk_sys);
#endif

	if ((ret == ENTROPY_MCHP_SUCCESS) || (ret == -EALREADY)) {

		ENTROPY_DATA_RDY_SEM_INIT(&mchp_entropy_data->entropy_data_rdy_sem);

		entropy_cfg->irq_config_func(dev);

		entropy_runstandby_enable(dev);

		entropy_trng_enable(dev);

		ret = ENTROPY_MCHP_SUCCESS;
	}

	return ret;
}

/**
 * @brief ISR for Microchip TRNG peripheral.
 *
 * Handles TRNG interrupts by signaling data ready and reading new entropy data.
 *
 * @param dev Pointer to the entropy device structure.
 */
static void entropy_mchp_isr(const struct device *dev)
{
	/* Disable DATARDY interrupt */
	entropy_trng_interrupt_disable(dev);

	entropy_mchp_dev_data_t *mchp_entropy_data = dev->data;

	/* Read random generated data */
	mchp_entropy_data->trng_data = ENTROPY_REGS->TRNG_DATA;

	ENTROPY_DATA_RDY_SEM_GIVE(&mchp_entropy_data->entropy_data_rdy_sem);
}

/******************************************************************************
 * @brief Zephyr driver instance creation
 *****************************************************************************/
/** Entropy driver API structure. */
static DEVICE_API(entropy, entropy_mchp_api) = {.get_entropy = entropy_mchp_get_entropy,
						.get_entropy_isr = entropy_mchp_get_entropy_isr};

/**
 * @brief Declare the ENTROPY IRQ handler.
 *
 * @param n Instance number.
 */
#define ENTROPY_MCHP_IRQ_HANDLER_DECL(n)                                                           \
	static void entropy_mchp_irq_config_##n(const struct device *dev)

/**
 * @brief Define and connect the ENTROPY IRQ handler for a given instance.
 *
 * This macro defines the IRQ configuration function for the specified ENTROPY instance,
 * connects the ENTROPY interrupt to its handler, and enables the IRQ.
 *
 * @param n Instance number.
 */
#define ENTROPY_MCHP_IRQ_HANDLER(n)                                                                \
	static void entropy_mchp_irq_config_##n(const struct device *dev)                          \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, 0, irq), DT_INST_IRQ_BY_IDX(n, 0, priority),     \
			    entropy_mchp_isr, DEVICE_DT_INST_GET(n), 0);                           \
		irq_enable(DT_INST_IRQ_BY_IDX(n, 0, irq));                                         \
	}

/**
 * @brief Initialize entropy device instances.
 *
 * This macro initializes instances of the entropy device for each compatible
 * device tree node.
 *
 * @param n Device instance number.
 */
#if defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ2)
#define ENTROPY_MCHP_CONFIG_DEFN(n)                                                                \
	static const entropy_mchp_config_t entropy_mchp_config_##n = {                             \
		.regs = (trng_registers_t *)DT_INST_REG_ADDR(n),                                   \
		.irq_config_func = entropy_mchp_irq_config_##n,                                    \
		.run_in_standby = DT_INST_PROP(n, run_in_standby_en)}
#else
#define ENTROPY_MCHP_CONFIG_DEFN(n)                                                                \
	static const entropy_mchp_config_t entropy_mchp_config_##n = {                             \
		.regs = (trng_registers_t *)DT_INST_REG_ADDR(n),                                   \
		.entropy_clock = {.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                 \
				  .mclk_sys = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk,        \
										   subsystem))},   \
		.irq_config_func = entropy_mchp_irq_config_##n,                                    \
		.run_in_standby = DT_INST_PROP(n, run_in_standby_en)}
#endif
/**
 * @brief Macro to define the entropy data structure for a specific instance.
 *
 * This macro defines the entropy data structure for a specific instance of
 * the entropy device.
 *
 * @param n Instance number.
 */
#define ENTROPY_MCHP_DATA_DEFN(n) static entropy_mchp_dev_data_t entropy_mchp_data_##n

/**
 * @brief Macro to define the device structure for a specific instance of the entropy device.
 *
 * This macro defines the device structure for a specific instance of the entropy device.
 * It uses the DEVICE_DT_INST_DEFINE macro to create the device instance with the specified
 * initialization function, data structure, configuration structure, and driver API.
 *
 * @param n Instance number.
 */
#define ENTROPY_MCHP_DEVICE_DT_DEFN(n)                                                             \
	DEVICE_DT_INST_DEFINE(n, entropy_mchp_init,     /* init function */                        \
			      NULL,                     /* PM device (or NULL) */                  \
			      &entropy_mchp_data_##n,   /* data (or pointer to data struct) */     \
			      &entropy_mchp_config_##n, /* config (or pointer to config struct) */ \
			      POST_KERNEL,              /* initialization level */                 \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, /* initialization priority */    \
			      &entropy_mchp_api                   /* API struct */                 \
	)
/**
 * @brief Initialize entropy device instances.
 *
 * This macro initializes instances of the entropy device for each compatible
 * device tree node.
 *
 * @param n Device instance number.
 */
#define ENTROPY_DEVICE_INIT(n)                                                                     \
	ENTROPY_MCHP_IRQ_HANDLER_DECL(n);                                                          \
	ENTROPY_MCHP_CONFIG_DEFN(n);                                                               \
	ENTROPY_MCHP_DATA_DEFN(n);                                                                 \
	ENTROPY_MCHP_DEVICE_DT_DEFN(n);                                                            \
	ENTROPY_MCHP_IRQ_HANDLER(n);

DT_INST_FOREACH_STATUS_OKAY(ENTROPY_DEVICE_INIT)
