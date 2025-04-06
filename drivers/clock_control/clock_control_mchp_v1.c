/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file clock_control_mchp_v1.c
 * @brief Clock control driver for Microchip devices.
 *
 * This file provides the implementation of clock control functions
 * for Microchip-based systems.
 */

#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include "clock_control_mchp_v1.h"
#include "zephyr/syscall.h"
#include <stdbool.h>

/**
 * @brief Get the subsys device for a given node_id.
 *
 * This macro retrieves the device associated with the specified node_id using the
 * DEVICE_DT_GET function.
 *
 * @param node_id The device tree node identifier.
 * @return The device corresponding to the given node_id.
 */
#define CLOCK_CONTROL_MCHP_SUBSYS_GET(node_id) DEVICE_DT_GET(node_id)

/**
 * @brief Get the register address for a given node_id.
 *
 * This macro retrieves the register address associated with the specified node_id
 * using the DT_REG_ADDR function.
 *
 * @param node_id The device tree node identifier.
 * @return The register address for the device corresponding to the node_id.
 */
#define CLOCK_CONTROL_MCHP_SUBSYS_GET_REG(node_id) DT_REG_ADDR(node_id)

/**
 * @brief Define the subsys device for a given node_id.
 *
 * This macro defines a device using the DEVICE_DT_DEFINE function for the
 * specified node_id. It sets the device to initialize at the PRE_KERNEL_1 stage
 * with the specified initialization priority.
 *
 * @param node_id The device tree node identifier.
 */
#define CLOCK_CONTROL_MCHP_SUBSYS_DEFINE(node_id)                                                  \
	DEVICE_DT_DEFINE(node_id, NULL, NULL, NULL, NULL, PRE_KERNEL_1,                            \
			 CONFIG_CLOCK_CONTROL_INIT_PRIORITY, NULL)

/**
 * @brief Get the number of ranges (subnodes) for the "clock" node.
 *
 * This macro calculates the number of subnodes for the "clock" node using
 * the DT_NUM_RANGES function, which indicates the number of subnodes (ranges)
 * defined under the "clock" node.
 *
 * @return The number of subnodes under the "clock" node.
 */
#define CLOCK_CONTROL_MCHP_SUBSYS_COUNT DT_NUM_RANGES(DT_NODELABEL(clock))

/**
 * @brief Structure holding data for the clock control mechanism.
 *
 * This structure contains information related to asynchronous clock control
 * operations, including the base address and identifier for asynchronous operations,
 * as well as the callback work data for clock control.
 */
typedef struct {
	/* Register address for asynchronous operations related to clock control. */
	uint32_t async_reg;
	/* Identifier for the asynchronous clock control operation. */
	uint32_t async_id;
	/* Callback function to be executed when the work is triggered. */
	clock_control_cb_t async_cb;
	/* Device associated with the callback function. */
	const struct device *async_dev;
	/* Clock control subsystem associated with the callback. */
	clock_control_subsys_t async_sys;
	/* User-specific data that will be passed to the callback function. */
	void *async_user_data;
	/* Flag indicating if an asynchronous operation is in progress. */
	bool is_async_in_progress;
} clock_control_mchp_data_t;

/**
 * @brief Structure holding the configuration for clock control.
 *
 * This structure stores configuration data for clock control, including the count of
 * subsystems, devices and registers associated with each subsystem, and user-defined
 * frequency settings.
 */
typedef struct {
	/* The number of subsystems. */
	uint32_t subsys_count;
	/* Array of pointers to subsystem devices. */
	const struct device *subsys_devs[CLOCK_CONTROL_MCHP_SUBSYS_COUNT];
	/* Array of registers for each subsystem. */
	const uint32_t subsys_regs[CLOCK_CONTROL_MCHP_SUBSYS_COUNT];
	/* User-defined frequency configuration. */
	struct clock_control_mchp_user_frequency user_frequency;
	/* Timeout in milliseconds to wait for clock to turn on */
	uint32_t on_timeout_ms;
} clock_control_mchp_config_t;

/**
 * @brief Get the index of a subsystem device in the configuration.
 *
 * This function searches for the specified `clock_dev` in the list of subsystem devices
 * for the given `dev`. If found, it returns the index of the subsystem device.
 *
 * @param dev Pointer to the device for which the subsystem index is being searched.
 * @param clock_dev Pointer to the clock device to find in the subsystem device list.
 *
 * @return The index of the found subsystem device, or -1 if not found.
 */
static int clock_control_mchp_get_idx(const struct device *dev, const struct device *clock_dev)
{
	/* Initialize the subsystem index to -1 (not found). */
	int subsys_idx = -1;
	/* Pointer to the clock control configuration structure. */
	const clock_control_mchp_config_t *config = dev->config;
	/* Index variable for looping through the subsystem devices. */
	uint8_t index;

	/* Loop through each subsystem device to find a match with clock_dev. */
	for (index = 0; index < CLOCK_CONTROL_MCHP_SUBSYS_COUNT; index++) {
		if (clock_dev == config->subsys_devs[index]) {
			/* Set the found subsystem index and break the loop. */
			subsys_idx = index;
			break;
		}
	}

	/* Return the found subsystem index or -1 if not found. */
	return subsys_idx;
}

/**
 * @brief Clock control interrupt service routine (ISR).
 *
 * This ISR clears and disables the clock interrupt, then checks if a callback
 * function is provided. If a callback is provided, the associated work is submitted.
 *
 * @param dev Pointer to the device structure representing the clock control.
 */
static void clock_control_mchp_isr(const struct device *dev)
{
	/* Pointer to the clock control data associated with the device. */
	clock_control_mchp_data_t *data = dev->data;

	/* Clear the interrupt for the specified async base and ID. */
	hal_mchp_clock_clear_interrupt(data->async_reg, data->async_id);

	/* Disable the interrupt for the specified async base and ID. */
	hal_mchp_clock_disable_interrupt(data->async_reg, data->async_id);

	/* Check if a callback function is provided by the caller. */
	if (data->async_cb != NULL) {
		/* Invoke the callback function with the device, subsystem, and user data. */
		data->async_cb(data->async_dev, data->async_sys, data->async_user_data);
	}

	/* Change flag to indicate async operation not in progress */
	data->is_async_in_progress = false;
}

/**
 * @brief Start an asynchronous clock control operation for a given subsystem.
 *
 * This function attempts to turn on the clock for a specified subsystem asynchronously.
 * It checks the current clock state, and if the clock is off, it turns it on and
 * enables an interrupt. If a callback is provided, it is set up to be invoked when
 * the operation completes.
 *
 * @param dev Pointer to the device structure representing the clock control.
 * @param sys The subsystem for which the clock is to be turned on.
 * @param cb Callback function to be invoked once the clock is on.
 * @param user_data User data to be passed to the callback function.
 *
 * @return 0 if the operation is successful, otherwise an error code.
 */
static int clock_control_mchp_async_on(const struct device *dev, clock_control_subsys_t sys,
				       clock_control_cb_t cb, void *user_data)
{
	/* Return value for the operation status. */
	int ret_val;
	/* Subsystem index for identifying the clock device. */
	int subsys_idx;
	/* Pointer to the subsystem structure. */
	clock_control_mchp_subsys_t *subsys;
	/* Status of the clock. */
	clock_control_mchp_state_t status;
	/* Pointer to the clock control configuration data. */
	const clock_control_mchp_config_t *config = dev->config;
	/* Pointer to the clock control data associated with the device. */
	clock_control_mchp_data_t *data = dev->data;

	/* Check if an async operation is already in progress. */
	if (data->is_async_in_progress == true) {
		/* Return busy to indicate async is already in progress. */
		return -EBUSY;
	}

	/* Check if the clock control for all subsystems is requested. */
	if (sys == CLOCK_CONTROL_SUBSYS_ALL) {
		/* Clock Async On for all clocks is not supported. */
		ret_val = -ENOTSUP;
	} else {
		/* Get the subsystem pointer. */
		subsys = (clock_control_mchp_subsys_t *)sys;

		/* Get the index of the subsystem. */
		subsys_idx = clock_control_mchp_get_idx(dev, subsys->dev);

		/* Check if the subsystem was found. */
		if (subsys_idx < 0) {
			/* No matching subsystem found. */
			ret_val = -ENOTSUP;
		} else {
			/* Get the current status of the clock. */
			status = hal_mchp_clock_status(config->subsys_regs[subsys_idx], subsys->id);

			/* Check if the clock is already on or starting. */
			if (status == CLOCK_CONTROL_MCHP_STATE_ON) {
				/* Clock is already ON. */
				ret_val = -EALREADY;
			} else if (status == CLOCK_CONTROL_MCHP_STATE_STARTING) {
				/* Clock is already STARTING. */
				ret_val = -EALREADY;
			} else if (status == CLOCK_CONTROL_MCHP_STATE_OFF) {
				/* Clear the interrupt before enabling */
				hal_mchp_clock_clear_interrupt(config->subsys_regs[subsys_idx],
							       subsys->id);

				/* Clock is off, attempt to enable interrupt */
				if (hal_mchp_clock_enable_interrupt(config->subsys_regs[subsys_idx],
								    subsys->id) ==
				    CLOCK_CONTROL_MCHP_STATE_OK) {
					/* Store async data to context. */
					data->async_reg = config->subsys_regs[subsys_idx];
					data->async_id = subsys->id;
					data->async_cb = cb;
					data->async_sys = sys;
					data->async_user_data = user_data;
					/* Set flag to indicate async operation is in progress. */
					data->is_async_in_progress = true;
					/* Clock interrupt is enabled, attempt to turn it on. */
					if (hal_mchp_clock_on(config->subsys_regs[subsys_idx],
							      subsys->id) ==
					    CLOCK_CONTROL_MCHP_STATE_OK) {
						/* Successfully initialized */
						ret_val = 0;
					} else {
						/* Subsystem does not support on operation. */
						ret_val = -ENOTSUP;
					}
				} else {
					/* Subsystem does not support interrupt. */
					ret_val = -ENOTSUP;
				}
			} else {
				/* Any other status value is unsupported. */
				ret_val = -ENOTSUP;
			}
		}
	}

	/* Return the result of the operation. */
	return ret_val;
}

/**
 * @brief Configure the clock for a specified subsystem.
 *
 * This function configures the clock for the given subsystem. If `CLOCK_CONTROL_SUBSYS_ALL`
 * is specified, it attempts to configure all subsystems. It returns 0 if the configuration
 * is successful, or an error code otherwise.
 *
 * @param dev Pointer to the device structure representing the clock control.
 * @param sys The subsystem to configure, or `CLOCK_CONTROL_SUBSYS_ALL` for all subsystems.
 * @param req_configuration Pointer to the requested configuration for the clock.
 *
 * @return 0 if the configuration is successful, otherwise an error code.
 */
static int clock_control_mchp_configure(const struct device *dev, clock_control_subsys_t sys,
					void *req_configuration)
{
	/* Return value for the operation status. */
	int ret_val;
	/* Subsystem index for identifying the clock device. */
	int subsys_idx;
	/* Pointer to the subsystem structure. */
	clock_control_mchp_subsys_t *subsys;
	/* State of the clock after configuration. */
	clock_control_mchp_state_t state;
	/* Index variable for looping through subsystems. */
	uint8_t index;
	/* Pointer to the clock control configuration data. */
	const clock_control_mchp_config_t *config = dev->config;
	/* Flag to indicate one or more subsystems is missing configuration data. */
	bool is_config_missing = false;

	/* Check if requested configuration is NULL. */
	if (req_configuration == NULL) {
		/* Return not support to indicate configuration data is not present. */
		ret_val = -ENOTSUP;
		/* Check if configuration for all subsystems is requested. */
	} else if (sys == CLOCK_CONTROL_SUBSYS_ALL) {
		/* Initialize return value as 0 (success). */
		ret_val = 0;
		/* Initialize state as no support and configure all subsystems. */
		state = CLOCK_CONTROL_MCHP_STATE_NO_SUPPORT;
		for (index = 0; index < CLOCK_CONTROL_MCHP_SUBSYS_COUNT; index++) {
			/* Configure the clock for each subsystem. */
			state = hal_mchp_clock_configure(config->subsys_regs[index], 0,
							 req_configuration);

			/* Check if the configuration for current subsystem was not supported. */
			if (state == CLOCK_CONTROL_MCHP_STATE_NO_SUPPORT) {
				/* Configuration failed for current subsystem. */
				ret_val = -ENOTSUP;
				break;
			} else if (state == CLOCK_CONTROL_MCHP_STATE_NO_CONFIG) {
				/* Configuration missing for current subsystem. */
				is_config_missing = true;
			}
		}

		/* Check whether one or more configuration is not valid. */
		if ((ret_val == 0) && (is_config_missing == true)) {
			/* One or more subsytem configuration is not valid. */
			ret_val = -EINVAL;
		}
	} else {
		/* Get the subsystem pointer for a specific subsystem. */
		subsys = (clock_control_mchp_subsys_t *)sys;

		/* Get the index of the specific subsystem. */
		subsys_idx = clock_control_mchp_get_idx(dev, subsys->dev);

		/* Check if the specified subsystem was found. */
		if (subsys_idx < 0) {
			/* No matching subsystem found. */
			ret_val = -ENOTSUP;
		} else {
			/* Configure the clock for the specified subsystem. */
			state = hal_mchp_clock_configure(config->subsys_regs[subsys_idx],
							 subsys->id, req_configuration);

			/* Check if the configuration for the subsystem was successful. */
			if (state == CLOCK_CONTROL_MCHP_STATE_OK) {
				ret_val = 0;
			} else {
				/* Configuration failed for the subsystem. */
				ret_val = -ENOTSUP;
			}
		}
	}

	/* Return the result of the configuration operation. */
	return ret_val;
}

/**
 * @brief Get the rate of the clock for a specified subsystem.
 *
 * This function retrieves the clock frequency (Hz) for the given subsystem. If
 * `CLOCK_CONTROL_SUBSYS_ALL` is specified, this operation is not supported. The function returns 0
 * if the rate is successfully retrieved, or an error code otherwise.
 *
 * @param dev Pointer to the device structure representing the clock control.
 * @param sys The subsystem for which the clock rate is to be retrieved.
 * @param frequency Pointer to store the retrieved clock rate.
 *
 * @return 0 if the rate is successfully retrieved, otherwise an error code.
 */
static int clock_control_mchp_get_rate(const struct device *dev, clock_control_subsys_t sys,
				       uint32_t *frequency)
{
	/* Return value for the operation status. */
	int ret_val;
	/* Subsystem index for identifying the clock device. */
	int subsys_idx;
	/* Pointer to the subsystem structure. */
	clock_control_mchp_subsys_t *subsys;
	/* State of the subsystem clock. */
	clock_control_mchp_state_t state;
	/* Pointer to the clock control configuration data. */
	const clock_control_mchp_config_t *config = dev->config;

	/* Check if retrieving rate for all subsystems is requested. */
	if (sys == CLOCK_CONTROL_SUBSYS_ALL) {
		/* Getting rate for all clocks is not supported. */
		ret_val = -ENOTSUP;
	} else if (frequency == NULL) {
		/* Getting rate pointer is NULL. */
		ret_val = -ENOTSUP;
	} else {
		/* Get the subsystem pointer for a specific subsystem. */
		subsys = (clock_control_mchp_subsys_t *)sys;

		/* Get the index of the specific subsystem. */
		subsys_idx = clock_control_mchp_get_idx(dev, subsys->dev);

		/* Check if the specified subsystem was found. */
		if (subsys_idx < 0) {
			/* No matching subsystem found. */
			ret_val = -ENOTSUP;
		} else {
			/* Get the rate of the clock for the specified subsystem. */
			state = hal_mchp_clock_get_rate(config->subsys_regs[subsys_idx], subsys->id,
							frequency, config->user_frequency);

			/* Check if the rate retrieval was successful. */
			if (state == CLOCK_CONTROL_MCHP_STATE_OK) {
				ret_val = 0;
			} else if (state == CLOCK_CONTROL_MCHP_STATE_NO_RATE) {
				/* Rate is unavailable, retry later. */
				ret_val = -EAGAIN;
			} else {
				/* An unsupported state was encountered. */
				ret_val = -ENOTSUP;
			}
		}
	}

	/* Return the result of the rate retrieval operation. */
	return ret_val;
}

/**
 * @brief Get the status of the clock for a specified subsystem.
 *
 * This function retrieves the current status of the clock for a given subsystem.
 * If `CLOCK_CONTROL_SUBSYS_ALL` is specified, the operation is not supported.
 * The function returns the current clock status (e.g., off, on, starting, or unknown).
 *
 * @param dev Pointer to the device structure representing the clock control.
 * @param sys The subsystem for which the clock status is to be retrieved.
 *
 * @return The current status of the clock for the specified subsystem.
 */
static enum clock_control_status clock_control_mchp_get_status(const struct device *dev,
							       clock_control_subsys_t sys)
{
	/* Variable to store the returned status to indicate stae is not yet known. */
	enum clock_control_status ret_status = CLOCK_CONTROL_STATUS_UNKNOWN;
	/* Pointer to the subsystem structure. */
	clock_control_mchp_subsys_t *subsys;
	/* State of the clock for the given subsystem. */
	clock_control_mchp_state_t status;
	/* Subsystem index for identifying the clock device. */
	int subsys_idx;
	/* Pointer to the clock control configuration data. */
	const clock_control_mchp_config_t *config = dev->config;
	/* Pointer to the clock control data associated with the device. */
	clock_control_mchp_data_t *data = dev->data;

	/* Check if clock status for all subsystems is not requested. */
	if (sys != CLOCK_CONTROL_SUBSYS_ALL) {
		/* Get the subsystem pointer for a specific subsystem. */
		subsys = (clock_control_mchp_subsys_t *)sys;

		/* Get the index of the specific subsystem. */
		subsys_idx = clock_control_mchp_get_idx(dev, subsys->dev);

		/* Check if the specified subsystem was found. */
		if (subsys_idx >= 0) {
			/* Check whether an async operation is initiated for this clock */
			if ((data->is_async_in_progress == true) &&
			    (data->async_id == subsys->id) &&
			    (data->async_reg == config->subsys_regs[subsys_idx])) {
				/* The clock async operation is in progress. */
				status = CLOCK_CONTROL_MCHP_STATE_STARTING;
			} else {
				/* Retrieve the current clock status for the subsystem. */
				status = hal_mchp_clock_status(config->subsys_regs[subsys_idx],
							       subsys->id);
			}

			/* Map the status directly to the return status */
			switch (status) {
			case CLOCK_CONTROL_MCHP_STATE_OFF:
				ret_status = CLOCK_CONTROL_STATUS_OFF;
				break;
			case CLOCK_CONTROL_MCHP_STATE_ON:
				ret_status = CLOCK_CONTROL_STATUS_ON;
				break;
			case CLOCK_CONTROL_MCHP_STATE_STARTING:
				ret_status = CLOCK_CONTROL_STATUS_STARTING;
				break;
			default:
				/* If status is unknown, it stays as CLOCK_CONTROL_STATUS_UNKNOWN */
				break;
			}
		}
	}

	/* Return the status of the clock for the specified subsystem. */
	return ret_status;
}

/**
 * @brief Turn off the clock for a specified subsystem.
 *
 * This function attempts to turn off the clock for the given subsystem.
 * If `CLOCK_CONTROL_SUBSYS_ALL` is specified, this operation is not supported.
 * The function returns 0 if the clock is successfully turned off,
 * or an error code (e.g., `-ENOTSUP`, `-EALREADY`) if the operation fails.
 *
 * @param dev Pointer to the device structure representing the clock control.
 * @param sys The subsystem for which the clock is to be turned off.
 *
 * @return 0 if the clock is successfully turned off, otherwise an error code.
 */
static int clock_control_mchp_off(const struct device *dev, clock_control_subsys_t sys)
{
	/* Return value for the operation status. */
	int ret_val;
	/* Subsystem index for identifying the clock device. */
	int subsys_idx;
	/* Pointer to the subsystem structure. */
	clock_control_mchp_subsys_t *subsys;
	/* State of the clock for the given subsystem. */
	clock_control_mchp_state_t state;
	/* Current status of the clock for the given subsystem. */
	clock_control_mchp_state_t status;
	/* Pointer to the clock control configuration data. */
	const clock_control_mchp_config_t *config = dev->config;
	/* Pointer to the clock control data associated with the device. */
	clock_control_mchp_data_t *data = dev->data;

	/* Check if turning off all clocks is requested. */
	if (sys == CLOCK_CONTROL_SUBSYS_ALL) {
		/* Turning off all clocks is not supported. */
		ret_val = -ENOTSUP;
	} else {
		/* Get the subsystem pointer for a specific subsystem. */
		subsys = (clock_control_mchp_subsys_t *)sys;

		/* Get the index of the specific subsystem. */
		subsys_idx = clock_control_mchp_get_idx(dev, subsys->dev);

		/* Check if the specified subsystem was found. */
		if (subsys_idx < 0) {
			/* No matching subsystem found. */
			ret_val = -ENOTSUP;
		} else {
			/* Check whether an async operation is initiated for this clock */
			if ((data->is_async_in_progress == true) &&
			    (data->async_id == subsys->id) &&
			    (data->async_reg == config->subsys_regs[subsys_idx])) {
				/* The clock is starting. disable interrupts */
				hal_mchp_clock_disable_interrupt(config->subsys_regs[subsys_idx],
								 subsys->id);
				data->is_async_in_progress = false;
				status = CLOCK_CONTROL_MCHP_STATE_STARTING;

			} else { /* Retrieve the current clock status for the subsystem. */
				status = hal_mchp_clock_status(config->subsys_regs[subsys_idx],
							       subsys->id);
			}

			if (status == CLOCK_CONTROL_MCHP_STATE_OFF) {
				/* The clock is already OFF. */
				ret_val = 0;
			} else if ((status == CLOCK_CONTROL_MCHP_STATE_STARTING) ||
				   (status == CLOCK_CONTROL_MCHP_STATE_ON)) {
				/* Clock is On or Starting. Turn it OFF. */
				state = hal_mchp_clock_off(config->subsys_regs[subsys_idx],
							   subsys->id);
				/* Check if the clock off operation was successful. */
				if (state == CLOCK_CONTROL_MCHP_STATE_OK) {
					ret_val = 0;
				} else {
					/* The clock does not support Off operation. */
					ret_val = -ENOTSUP;
				}
			} else {
				/* The clock does not support Status operation. */
				ret_val = -ENOTSUP;
			}
		}
	}

	/* Return the result of the clock off operation. */
	return ret_val;
}

/**
 * @brief Turn on the clock for a specified subsystem.
 *
 * This function attempts to turn on the clock for the given subsystem.
 * If `CLOCK_CONTROL_SUBSYS_ALL` is specified, this operation is not supported.
 * The function returns 0 if the clock is successfully turned on,
 * or an error code (e.g., `-ENOTSUP`, `-EALREADY`) if the operation fails.
 *
 * @param dev Pointer to the device structure representing the clock control.
 * @param sys The subsystem for which the clock is to be turned on.
 *
 * @return 0 if the clock is successfully turned on, otherwise an error code.
 */
static int clock_control_mchp_on(const struct device *dev, clock_control_subsys_t sys)
{
	/* Return value for the operation status. */
	int ret_val;
	/* Subsystem index for identifying the clock device. */
	int subsys_idx;
	/* Pointer to the subsystem structure. */
	clock_control_mchp_subsys_t *subsys;
	/* State of the clock for the given subsystem. */
	clock_control_mchp_state_t state;
	/* Current status of the clock for the given subsystem. */
	clock_control_mchp_state_t status;
	/* Pointer to the clock control configuration data. */
	const clock_control_mchp_config_t *config = dev->config;
	/* Whether to wait or not if clock is supported */
	bool is_wait = false;
	/* Initialise clock on timeout variable */
	uint32_t on_timeout_ms = 0;

	/* Check if turning on all clocks is requested. */
	if (sys == CLOCK_CONTROL_SUBSYS_ALL) {
		/* Turning on all clocks is not supported. */
		ret_val = -ENOTSUP;
	} else {
		/* Get the subsystem pointer for a specific subsystem. */
		subsys = (clock_control_mchp_subsys_t *)sys;

		/* Get the index of the specific subsystem. */
		subsys_idx = clock_control_mchp_get_idx(dev, subsys->dev);

		/* Check if the specified subsystem was found. */
		if (subsys_idx < 0) {
			/* No matching subsystem found. */
			ret_val = -ENOTSUP;
		} else {
			/* Retrieve the current clock status for the subsystem. */
			status = hal_mchp_clock_status(config->subsys_regs[subsys_idx], subsys->id);

			/* Check if the clock is already on or starting. */
			if (status == CLOCK_CONTROL_MCHP_STATE_ON) {
				/* The clock is already ON. */
				ret_val = -EALREADY;
			} else if (status == CLOCK_CONTROL_MCHP_STATE_STARTING) {
				/* The clock is already STARTING. */
				ret_val = -EALREADY;
			} else if (status == CLOCK_CONTROL_MCHP_STATE_OFF) {
				/* The clock is OFF, attempt to turn it ON. */
				state = hal_mchp_clock_on(config->subsys_regs[subsys_idx],
							  subsys->id);

				/* Check if the clock on operation was successful. */
				if (state == CLOCK_CONTROL_MCHP_STATE_OK) {
					is_wait = true;
				} else {
					/* The subsystem does not support on operation. */
					ret_val = -ENOTSUP;
				}
			} else {
				/* Any other clock state is unsupported. */
				ret_val = -ENOTSUP;
			}
		}
	}

	/* Check whether to wait for clock to become ON. */
	if (is_wait == true) {
		/* Wait until the clock state becomes ON. */
		while (true) {
			status = hal_mchp_clock_status(config->subsys_regs[subsys_idx], subsys->id);
			if (status == CLOCK_CONTROL_MCHP_STATE_ON) {
				/* Successfully turned on clock. */
				ret_val = 0;
				break;
			}
			if (on_timeout_ms < config->on_timeout_ms) {
				/* Thread is not available while booting. */
				if ((k_is_pre_kernel() == false) && (k_current_get() != NULL)) {
					/* Sleep before checking again. */
					k_sleep(K_MSEC(1));
				}
				/* Increment clock on timeout value */
				on_timeout_ms++;
			} else {
				/* Clock on timeout occurred */
				ret_val = -ETIMEDOUT;
				break;
			}
		}
	}

	/* Return the result of the clock on operation. */
	return ret_val;
}

/**
 * @brief Set the rate for the specified clock subsystem.
 *
 * This function attempts to set the rate for a given subsystem's clock.
 * If `CLOCK_CONTROL_SUBSYS_ALL` is specified, this operation is not supported.
 * The function returns 0 if the rate is successfully set, or an error code (e.g., `-ENOTSUP`,
 * `-EALREADY`) if the operation fails.
 *
 * @param dev Pointer to the device structure representing the clock control.
 * @param sys The subsystem for which the clock rate is to be set.
 * @param rate The desired clock rate for the specified subsystem.
 *
 * @return 0 if the rate is successfully set, otherwise an error code.
 */
static int clock_control_mchp_set_rate(const struct device *dev, clock_control_subsys_t sys,
				       clock_control_subsys_rate_t rate)
{
	/* Return value for the operation status. */
	int ret_val;
	/* Subsystem index for identifying the clock device. */
	int subsys_idx;
	/* Pointer to the subsystem structure. */
	clock_control_mchp_subsys_t *subsys;
	/* State of the clock for the given subsystem. */
	clock_control_mchp_state_t state;
	/* Pointer to the clock control configuration data. */
	const clock_control_mchp_config_t *config = dev->config;

	/* Check if setting rate for all clocks is requested. */
	if (sys == CLOCK_CONTROL_SUBSYS_ALL) {
		/* Setting rate for all clocks is not supported. */
		ret_val = -ENOTSUP;
	} else if (rate == NULL) {
		/* Subsystem rate setting is NULL. */
		ret_val = -ENOTSUP;
	} else {
		/* Get the subsystem pointer for a specific subsystem. */
		subsys = (clock_control_mchp_subsys_t *)sys;

		/* Get the index of the specific subsystem. */
		subsys_idx = clock_control_mchp_get_idx(dev, subsys->dev);

		/* Check if the specified subsystem was found. */
		if (subsys_idx < 0) {
			/* No matching subsystem found. */
			ret_val = -ENOTSUP;
		} else {
			/* Attempt to set the clock rate for the subsystem. */
			state = hal_mchp_clock_set_rate(config->subsys_regs[subsys_idx], subsys->id,
							rate);

			/* Check if the clock rate was successfully set. */
			if (state == CLOCK_CONTROL_MCHP_STATE_OK) {
				/* Successfully set the clock rate. */
				ret_val = 0;
			} else if (state == CLOCK_CONTROL_MCHP_STATE_SAME_RATE) {
				/* The rate is already the same as the requested rate. */
				ret_val = -EALREADY;
			} else {
				/* The rate setting operation is not supported. */
				ret_val = -ENOTSUP;
			}
		}
	}

	/* Return the result of the rate setting operation. */
	return ret_val;
}

/**
 * @brief Clock control API for the Microchip driver.
 *
 * This structure defines the functions that are part of the clock control API
 * for the Microchip driver. It provides operations for controlling the clock
 * of a device, including turning it on/off, setting the rate, and configuring
 * the clock subsystem.
 *
 * @note The `clock_control_mchp_driver_api` API is implemented by the driver
 *       to allow external interactions with the device's clock control functions.
 */
static const struct clock_control_driver_api clock_control_mchp_driver_api = {
	/* Turn the clock on for the specified subsystem. */
	.on = clock_control_mchp_on,
	/* Turn the clock off for the specified subsystem. */
	.off = clock_control_mchp_off,
	/* Asynchronously turn the clock on for the specified subsystem. */
	.async_on = clock_control_mchp_async_on,
	/* Get the clock rate for the specified subsystem. */
	.get_rate = clock_control_mchp_get_rate,
	/* Get the clock status for the specified subsystem. */
	.get_status = clock_control_mchp_get_status,
	/* Set the clock rate for the specified subsystem. */
	.set_rate = clock_control_mchp_set_rate,
	/* Configure the clock settings for the specified subsystem. */
	.configure = clock_control_mchp_configure,
};

/**
 * @brief Initialize the clock control subsystem for the Microchip driver.
 *
 * This function initializes the clock control subsystem by enabling the
 * interrupt connection required for the driver to operate. It must be called
 * during system initialization to set up the clock control functionality.
 *
 * @param dev Pointer to the device structure representing the clock control.
 *
 * @return 0 on successful initialization.
 */
static int clock_control_mchp_init(const struct device *dev)
{
	/* Enable the interrupt connection for the clock control subsystem. */
	CLOCK_CONTROL_MCHP_IRQ_CONNECT_ENABLE_DEFN();

	/* Return 0 indicating successful initialization. */
	return 0;
}

/**
 * @brief Device data configuration for the Microchip clock control driver.
 *
 * This structure holds the data configuration for the clock control
 * subsystem of the Microchip driver, including the base address and ID
 * for asynchronous operations. It is initialized with default values.
 */
static clock_control_mchp_data_t clock_control_mchp_data = {
	/* Register address for asynchronous clock operations. */
	.async_reg = 0,
	/* ID for the asynchronous clock operation. */
	.async_id = 0,
	/* Initialize the flag to indicate no asynchronous operation is in progress.
	 */
	.is_async_in_progress = false,
};

/**
 * @brief Device constant configuration for the Microchip clock control driver.
 *
 * This structure holds the constant configuration parameters for the
 * clock control subsystem, including the count of subsystems, their
 * respective devices and registers, and the user-defined frequency settings
 * for the clock control system. It is initialized with values retrieved
 * from device tree properties or defaults.
 */
static const clock_control_mchp_config_t clock_control_mchp_config = {
	/* Number of clock subsystems. */
	.subsys_count = CLOCK_CONTROL_MCHP_SUBSYS_COUNT,

	/* clang-format off */
	/* Array of devices for each clock subsystem. */
	.subsys_devs = {DT_FOREACH_CHILD_SEP(DT_NODELABEL(clock), CLOCK_CONTROL_MCHP_SUBSYS_GET,
					     (,))},
	/* Array of register addresses for each clock subsystem. */
	.subsys_regs = {DT_FOREACH_CHILD_SEP(DT_NODELABEL(clock), CLOCK_CONTROL_MCHP_SUBSYS_GET_REG,
					 (,))},
	/* clang-format on */

	/* User-defined frequency settings for various clock sources. */
	.user_frequency = CLOCK_CONTROL_MCHP_USER_FREQUENCY_DEFN,

	/* Timeout in milliseconds to wait for clock to turn on */
	.on_timeout_ms = DT_PROP_OR(DT_NODELABEL(clock), on_timeout_ms, 5),
};

/**
 * @brief Define the clock control device for the Microchip clock control driver.
 *
 * This macro registers the clock control device, initializing it with
 * the necessary data, configuration, and driver API. It also sets up
 * the initialization function and priority to run before the kernel
 * starts in the pre-kernel phase.
 */
DEVICE_DT_DEFINE(
	/* Device tree node label for clock */
	DT_NODELABEL(clock),
	/* Initialization function for the clock control */
	clock_control_mchp_init,
	/* Optional device power management function (not used here) */
	NULL,
	/* Pointer to the device data structure */
	&clock_control_mchp_data,
	/* Pointer to the device configuration structure */
	&clock_control_mchp_config,
	/* Initialization phase (before kernel starts) */
	PRE_KERNEL_1,
	/* Initialization priority */
	CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
	/* Pointer to the device API */
	&clock_control_mchp_driver_api);

/**
 * @brief Iterate over all child nodes of the `clock` node and define subsystems.
 *
 * This macro iterates over each child of the `clock` node in the device tree
 * and calls `CLOCK_CONTROL_MCHP_SUBSYS_DEFINE` for each child, defining
 * the corresponding subsystem devices.
 */
DT_FOREACH_CHILD(
	/* Device tree node label for clock */
	DT_NODELABEL(clock),
	/* Macro to define each subsystem device */
	CLOCK_CONTROL_MCHP_SUBSYS_DEFINE);
