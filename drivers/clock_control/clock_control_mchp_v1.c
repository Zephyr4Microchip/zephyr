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

#include <zephyr/drivers/clock_control/clock_control_mchp_v1.h>
#include "clock_control_mchp_v1.h"

/**
 * @brief Get the subsys device for a given node_id.
 *
 * This macro retrieves the device associated with the specified node_id using the
 * DEVICE_DT_GET function.
 *
 * @param node_id The device tree node identifier.
 * @return The device corresponding to the given node_id.
 */
#define CLOCK_CONTROL_MCHP_SUBSYS_GET(node_id) DEVICE_DT_GET(node_id),

/**
 * @brief Get the register address for a given node_id.
 *
 * This macro retrieves the register address associated with the specified node_id
 * using the DT_REG_ADDR function.
 *
 * @param node_id The device tree node identifier.
 * @return The register address for the device corresponding to the node_id.
 */
#define CLOCK_CONTROL_MCHP_SUBSYS_GET_REG(node_id) DT_REG_ADDR(node_id),

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
			 CONFIG_CLOCK_CONTROL_INIT_PRIORITY, NULL);

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
 * @brief Structure holding callback work data for the clock control.
 *
 * This structure is used to store data associated with the clock control callback
 * and the work queued for processing in the system. It includes the work object,
 * the callback function, the device the callback is associated with, the subsystem
 * for the clock, and any user data that is passed with the callback.
 */
struct clock_control_mchp_cb_work_data {
	/* Work object for clock control callback processing. */
	struct k_work work;

	/* Callback function to be executed when the work is triggered. */
	clock_control_cb_t cb;

	/* Device associated with the callback function. */
	const struct device *dev;

	/* Clock control subsystem associated with the callback. */
	clock_control_subsys_t sys;

	/* User-specific data that will be passed to the callback function. */
	void *user_data;
};

/**
 * @brief Structure holding data for the clock control mechanism.
 *
 * This structure contains information related to asynchronous clock control
 * operations, including the base address and identifier for asynchronous operations,
 * as well as the callback work data for clock control.
 */
struct clock_control_mchp_data {
	/* Base address for asynchronous operations related to clock control. */
	uint32_t async_base;

	/* Identifier for the asynchronous clock control operation. */
	uint32_t async_id;

	/* Callback work data for clock control operations. */
	struct clock_control_mchp_cb_work_data cb_work_data;
};

/**
 * @brief Structure holding the configuration for clock control.
 *
 * This structure stores configuration data for clock control, including the count of
 * subsystems, devices and registers associated with each subsystem, and user-defined
 * frequency settings.
 */
struct clock_control_mchp_config {
	/* The number of subsystems. */
	uint32_t subsys_count;

	/* Array of pointers to subsystem devices. */
	const struct device *subsys_devs[CLOCK_CONTROL_MCHP_SUBSYS_COUNT];

	/* Array of registers for each subsystem. */
	const uint32_t subsys_regs[CLOCK_CONTROL_MCHP_SUBSYS_COUNT];

	/* User-defined frequency configuration. */
	struct clock_control_mchp_user_frequency user_frequency;
};

/**
 * @brief Invoke the clock control callback function.
 *
 * This function is called to invoke the callback function associated with a
 * specific asynchronous clock control work. It retrieves the callback data
 * and calls the user-defined callback with the appropriate parameters.
 *
 * @param async_work Pointer to the work structure that contains the callback data.
 */
static void clock_control_mchp_invoke_cb(struct k_work *async_work)
{
	/* Retrieve the callback work data from the work structure. */
	struct clock_control_mchp_cb_work_data *cb_work_data =
		CONTAINER_OF(async_work, struct clock_control_mchp_cb_work_data, work);

	/* Invoke the callback function with the device, subsystem, and user data. */
	cb_work_data->cb(cb_work_data->dev, cb_work_data->sys, cb_work_data->user_data);
}

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
	const struct clock_control_mchp_config *config = dev->config;

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
	struct clock_control_mchp_data *data = dev->data;

	/* Clear the interrupt for the specified async base and ID. */
	hal_mchp_clock_clear_interrupt(data->async_base, data->async_id);

	/* Disable the interrupt for the specified async base and ID. */
	hal_mchp_clock_disable_interrupt(data->async_base, data->async_id);

	/* Check if a callback function is provided by the caller. */
	if (data->cb_work_data.cb == NULL) {
		/* No callback provided by the caller. */
	} else {
		/* Submit the work to be processed by the callback function. */
		k_work_submit(&data->cb_work_data.work);
	}
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

	/* Pointer to the subsystem structure. */
	clock_control_mchp_subsys_t *subsys;

	/* State of the subsystem clock. */
	enum clock_control_mchp_state state;

	/* Status of the clock. */
	enum clock_control_mchp_state status;

	/* Subsystem index for identifying the clock device. */
	int subsys_idx;

	/* Pointer to the clock control configuration data. */
	const struct clock_control_mchp_config *config = dev->config;

	/* Pointer to the clock control data associated with the device. */
	struct clock_control_mchp_data *data = dev->data;

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
				/* Clock is off, attempt to turn it on. */
				state = hal_mchp_clock_on(config->subsys_regs[subsys_idx],
							  subsys->id);

				/* Check if the clock on operation is success. */
				if (state == CLOCK_CONTROL_MCHP_STATE_OK) {
					/* Enable interrupt for the subsystem */
					if (hal_mchp_clock_enable_interrupt(
						    config->subsys_regs[subsys_idx], subsys->id) ==
					    CLOCK_CONTROL_MCHP_STATE_OK) {
						/* Store data and initialize work. */
						data->async_base = config->subsys_regs[subsys_idx];
						data->async_id = subsys->id;
						data->cb_work_data.cb = cb;
						data->cb_work_data.sys = sys;
						data->cb_work_data.user_data = user_data;
						k_work_init(&data->cb_work_data.work,
							    clock_control_mchp_invoke_cb);

						/* Successfully initialized */
						ret_val = 0;
					} else {
						/* Subsystem does not support interrupt. */
						ret_val = -ENOTSUP;
					}
				} else {
					/* Subsystem does not support on operation. */
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

	/* Pointer to the subsystem structure. */
	clock_control_mchp_subsys_t *subsys;

	/* State of the clock after configuration. */
	enum clock_control_mchp_state state;

	/* Index variable for looping through subsystems. */
	uint8_t index;

	/* Subsystem index for identifying the clock device. */
	int subsys_idx;

	/* Pointer to the clock control configuration data. */
	const struct clock_control_mchp_config *config = dev->config;

	/* Check if configuration for all subsystems is requested. */
	if (sys == CLOCK_CONTROL_SUBSYS_ALL) {
		/* Initialize state as no support and configure all subsystems. */
		state = CLOCK_CONTROL_MCHP_STATE_NO_SUPPORT;
		for (index = 0; index < CLOCK_CONTROL_MCHP_SUBSYS_COUNT; index++) {
			/* Configure the clock for each subsystem. */
			state = hal_mchp_clock_configure(config->subsys_regs[index], 0,
							 req_configuration);
		}

		/* Check if the configuration for all subsystems was successful. */
		if (state == CLOCK_CONTROL_MCHP_STATE_OK) {
			ret_val = 0;
		} else {
			/* Configuration failed for all subsystems. */
			ret_val = -ENOTSUP;
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
 * This function retrieves the clock rate for the given subsystem. If `CLOCK_CONTROL_SUBSYS_ALL`
 * is specified, this operation is not supported. The function returns 0 if the rate is successfully
 * retrieved, or an error code otherwise.
 *
 * @param dev Pointer to the device structure representing the clock control.
 * @param sys The subsystem for which the clock rate is to be retrieved.
 * @param rate Pointer to store the retrieved clock rate.
 *
 * @return 0 if the rate is successfully retrieved, otherwise an error code.
 */
static int clock_control_mchp_get_rate(const struct device *dev, clock_control_subsys_t sys,
				       uint32_t *rate)
{
	/* Return value for the operation status. */
	int ret_val;

	/* Pointer to the subsystem structure. */
	clock_control_mchp_subsys_t *subsys;

	/* State of the subsystem clock. */
	enum clock_control_mchp_state state;

	/* Subsystem index for identifying the clock device. */
	int subsys_idx;

	/* Pointer to the clock control configuration data. */
	const struct clock_control_mchp_config *config = dev->config;

	/* Check if retrieving rate for all subsystems is requested. */
	if (sys == CLOCK_CONTROL_SUBSYS_ALL) {
		/* Getting rate for all clocks is not supported. */
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
							rate, config->user_frequency);

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
	/* Variable to store the returned status. */
	enum clock_control_status ret_status;

	/* Pointer to the subsystem structure. */
	clock_control_mchp_subsys_t *subsys;

	/* State of the clock for the given subsystem. */
	enum clock_control_mchp_state status;

	/* Subsystem index for identifying the clock device. */
	int subsys_idx;

	/* Pointer to the clock control configuration data. */
	const struct clock_control_mchp_config *config = dev->config;

	/* Check if clock status for all subsystems is requested. */
	if (sys == CLOCK_CONTROL_SUBSYS_ALL) {
		/* Clock status for all clocks is not supported. */
		ret_status = CLOCK_CONTROL_STATUS_UNKNOWN;
	} else {
		/* Get the subsystem pointer for a specific subsystem. */
		subsys = (clock_control_mchp_subsys_t *)sys;

		/* Get the index of the specific subsystem. */
		subsys_idx = clock_control_mchp_get_idx(dev, subsys->dev);

		/* Check if the specified subsystem was found. */
		if (subsys_idx < 0) {
			/* No matching subsystem found. */
			ret_status = CLOCK_CONTROL_STATUS_UNKNOWN;
		} else {
			/* Retrieve the current clock status for the subsystem. */
			status = hal_mchp_clock_status(config->subsys_regs[subsys_idx], subsys->id);

			/* Map the clock state to the corresponding status. */
			if (status == CLOCK_CONTROL_MCHP_STATE_OFF) {
				ret_status = CLOCK_CONTROL_STATUS_OFF;
			} else if (status == CLOCK_CONTROL_MCHP_STATE_ON) {
				ret_status = CLOCK_CONTROL_STATUS_ON;
			} else if (status == CLOCK_CONTROL_MCHP_STATE_STARTING) {
				ret_status = CLOCK_CONTROL_STATUS_STARTING;
			} else {
				/* Unknown clock state encountered. */
				ret_status = CLOCK_CONTROL_STATUS_UNKNOWN;
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

	/* Pointer to the subsystem structure. */
	clock_control_mchp_subsys_t *subsys;

	/* State of the clock for the given subsystem. */
	enum clock_control_mchp_state state;

	/* Current status of the clock for the given subsystem. */
	enum clock_control_mchp_state status;

	/* Subsystem index for identifying the clock device. */
	int subsys_idx;

	/* Pointer to the clock control configuration data. */
	const struct clock_control_mchp_config *config = dev->config;

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
			/* Retrieve the current clock status for the subsystem. */
			status = hal_mchp_clock_status(config->subsys_regs[subsys_idx], subsys->id);

			/* Check if the clock is already turned off. */
			if (status == CLOCK_CONTROL_MCHP_STATE_OFF) {
				/* The clock is already OFF. */
				ret_val = -EALREADY;
			} else {
				/* Clock is ON or STARTING. Turn it OFF. */
				state = hal_mchp_clock_off(config->subsys_regs[subsys_idx],
							   subsys->id);

				/* Check if the clock off operation was successful. */
				if (state == CLOCK_CONTROL_MCHP_STATE_OK) {
					ret_val = 0;
				} else {
					/* Subsystem does not support off operation. */
					ret_val = -ENOTSUP;
				}
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

	/* Pointer to the subsystem structure. */
	clock_control_mchp_subsys_t *subsys;

	/* State of the clock for the given subsystem. */
	enum clock_control_mchp_state state;

	/* Current status of the clock for the given subsystem. */
	enum clock_control_mchp_state status;

	/* Subsystem index for identifying the clock device. */
	int subsys_idx;

	/* Pointer to the clock control configuration data. */
	const struct clock_control_mchp_config *config = dev->config;

	/* Whether to wait or not if clock is supported */
	bool is_wait = false;

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
			/* Sleep before checking again. */
			k_sleep(K_MSEC(1));
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

	/* Pointer to the subsystem structure. */
	clock_control_mchp_subsys_t *subsys;

	/* State of the clock for the given subsystem. */
	enum clock_control_mchp_state state;

	/* Subsystem index for identifying the clock device. */
	int subsys_idx;

	/* Pointer to the clock control configuration data. */
	const struct clock_control_mchp_config *config = dev->config;

	/* Check if setting rate for all clocks is requested. */
	if (sys == CLOCK_CONTROL_SUBSYS_ALL) {
		/* Setting rate for all clocks is not supported. */
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
	CLOCK_CONTROL_MCHP_IRQ_CONNECT_ENABLE_DEFN;

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
static struct clock_control_mchp_data clock_control_mchp_data_instance = {
	/* Base address for asynchronous clock operations. */
	.async_base = 0,

	/* ID for the asynchronous clock operation. */
	.async_id = 0,
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
static const struct clock_control_mchp_config clock_control_mchp_config_instance = {
	/* Number of clock subsystems. */
	.subsys_count = CLOCK_CONTROL_MCHP_SUBSYS_COUNT,

	/* Array of devices for each clock subsystem. */
	.subsys_devs = {DT_FOREACH_CHILD(DT_NODELABEL(clock), CLOCK_CONTROL_MCHP_SUBSYS_GET)},

	/* Array of register addresses for each clock subsystem. */
	.subsys_regs = {DT_FOREACH_CHILD(DT_NODELABEL(clock), CLOCK_CONTROL_MCHP_SUBSYS_GET_REG)},

	/* User-defined frequency settings for various clock sources. */
	.user_frequency = CLOCK_CONTROL_MCHP_USER_FREQUENCY_DEFN,
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
	&clock_control_mchp_data_instance,
	/* Pointer to the device configuration structure */
	&clock_control_mchp_config_instance,
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
	CLOCK_CONTROL_MCHP_SUBSYS_DEFINE)
