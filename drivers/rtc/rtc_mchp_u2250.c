/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/************************************************************************
 * @file        rtc_mchp_2250.c
 * @brief       Real-Time Clock (RTC) driver implementation for Microchip
				RTC U2250 Peripheral.
 *
 * @details
 * This file contains the implementation of the RTC driver functions for
 * Microchip hardware. It provides functionalities to initialize the RTC,
 * set and get time and alarm, and handle RTC interrupts.
 *
 ************************************************************************/

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include "rtc_utils.h"

/* Define compatible string */
#define DT_DRV_COMPAT microchip_rtc_u2250

/* Define the alarms for the RTC (Real-Time Clock) module */
#define RTC_MCHP_ALARM_1 (0)
#define RTC_MCHP_ALARM_2 (1)

/* This macro defines the reference start year used by the RTC module: 1st, Jan, 1900 */
#define RTC_TM_REFERENCE_YEAR (1900U)

/* Reference Year */
#define RTC_REFERENCE_YEAR (1996U)

/* Adjust user month */
#define RTC_ADJUST_MONTH(month) (month + 1U)

/* define the RTC alarm count supported macros */
#define RTC_ALARM_COUNT DT_PROP(DT_NODELABEL(rtc), alarms_count)

/* define the RTC calibration PPB count */
#define RTC_CALIB_PARTS_PER_BILLION (1000000000)

/* The maximum value is set to 127 ppb */
#define RTC_CALIBRATE_PPB_MAX (127)

/* This macro defines the type of the lock used to protect access to the RTC APIs. */
#define RTC_LOCK_TYPE struct k_mutex

/* This macro initializes the RTC lock. */
#define RTC_DATA_LOCK_INIT(p_lock) k_mutex_init(p_lock)

/* This macro acquires the RTC lock */
#define RTC_DATA_LOCK(p_lock) k_mutex_lock(p_lock, K_FOREVER)

/* This macro releases the RTC lock. */
#define RTC_DATA_UNLOCK(p_lock) k_mutex_unlock(p_lock)

/**
 * @brief Structure defining various time parameters for Microchip RTC driver.
 *
 * This structure defining the various time parameters supported by Microchip RTC driver.
 */
typedef struct rtc_mchp_time {
	uint32_t second;        /* Second value */
	uint32_t minute;        /* Minute value */
	uint32_t hour;          /* Hour value */
	uint32_t date_of_month; /* Day value */
	uint32_t month;         /* Month value */
	uint32_t year;          /* Year value */

} rtc_mchp_time_t;

/* Do the peripheral interrupt related configuration */
#if defined(CONFIG_RTC_ALARM)
#define RTC_MCHP_IRQ_HANDLER(n)                                                                    \
	static void rtc_mchp_irq_config_##n(const struct device *dev)                              \
	{                                                                                          \
		RTC_MCHP_IRQ_CONNECT(n, 0);                                                        \
	}

#endif

/**
 * @brief Clock configuration structure for RTC.
 *
 * This structure contains the clock configuration parameters for RTC
 * peripheral.
 */
typedef struct mchp_rtc_clock {

	/* Clock driver */
	const struct device *clock_dev;

	/* Main clock subsystem. */
	clock_control_mchp_subsys_t mclk_sys;

	/* Generic clock subsystem. */
	clock_control_mchp_subsys_t osc32k_sys;

} mchp_rtc_clock_t;

/* Enumeration for RTC (Real-Time Clock) alarm mask selection */
typedef enum {
	/* Alarm mask is off */
	RTC_MCHP_ALARM_MASK_SEL_OFF = 0x0,

	/* Alarm mask includes seconds */
	RTC_MCHP_ALARM_MASK_SEL_SS = 0x1,

	/* Alarm mask includes minutes and seconds */
	RTC_MCHP_ALARM_MASK_SEL_MMSS = 0x2,

	/* Alarm mask includes hours, minutes, and seconds */
	RTC_MCHP_ALARM_MASK_SEL_HHMMSS = 0x3,

	/* Alarm mask includes days, hours, minutes, and seconds */
	RTC_MCHP_ALARM_MASK_SEL_DDHHMMSS = 0x4,

	/* Alarm mask includes months, days, hours, minutes, and seconds */
	RTC_MCHP_ALARM_MASK_SEL_MMDDHHMMSS = 0x5,

	/* Alarm mask includes years, months, days, hours, minutes, and seconds */
	RTC_MCHP_ALARM_MASK_SEL_YYMMDDHHMMSS = 0x6,

} rtc_mchp_alarm_mask_sel_t;

typedef struct rtc_mchp_dev_config {

	/* Pointer to the RTC hardware registers structure */
	rtc_registers_t *regs;

	/* RTC clock configuration */
	mchp_rtc_clock_t rtc_clock;

	/* Prescaler value for the RTC clock */
	uint16_t prescaler;

	/* Function to configure the IRQ during initialization. */
	void (*irq_config_func)(const struct device *dev);

#ifdef CONFIG_RTC_CALIBRATION
	int32_t cal_constant;
#endif /* CONFIG_RTC_CALIBRATION */

#ifdef CONFIG_RTC_ALARM
	/* Number of alarms supported by the RTC */
	uint8_t alarms_count;
#endif /* CONFIG_RTC_ALARM */
} rtc_mchp_dev_config_t;

#ifdef CONFIG_RTC_ALARM
/* This structure holds the callback information for RTC alarm events. */
typedef struct rtc_mchp_data_cb {
	/* Indicates if an alarm is currently pending */
	bool is_alarm_pending;

	/* Callback function to be called when the alarm triggers */
	rtc_alarm_callback alarm_cb;

	/* User data to be passed to the callback function */
	void *alarm_user_data;
} rtc_mchp_data_cb_t;
#endif /* CONFIG_RTC_ALARM */

typedef struct rtc_mchp_dev_data {
	/* Mutex for protecting access to the RTC driver data. */
	RTC_LOCK_TYPE lock;

#ifdef CONFIG_RTC_ALARM
	rtc_mchp_data_cb_t alarms[RTC_ALARM_COUNT];
#endif /* CONFIG_RTC_ALARM */

} rtc_mchp_dev_data_t;

LOG_MODULE_REGISTER(rtc_mchp_u2250, CONFIG_RTC_LOG_LEVEL);

/**
 * @brief Wait for RTC synchronization to complete.
 *
 * This function waits until the specified synchronization flag is cleared,
 * indicating that the RTC synchronization is complete.
 *
 * @param regs Pointer to the RTC hardware registers structure.
 * @param sync_flag Synchronization flag to wait for.
 */
static inline void rtc_sync_busy(const rtc_registers_t *regs, uint32_t sync_flag)
{
	while ((regs->MODE2.RTC_SYNCBUSY & sync_flag) == sync_flag) {
		/* Wait for synchronization after Software Reset */
	}
}

/**
 * @brief Perform a software reset on the RTC module.
 *
 * This function sets the software reset bit in the RTC control register and waits for the reset to
 * complete.
 *
 * @param regs Pointer to the RTC hardware registers structure.
 */
static inline void rtc_swrst(rtc_registers_t *regs)
{
	regs->MODE2.RTC_CTRLA |= RTC_MODE2_CTRLA_SWRST(1);

	/* Wait for the software reset to complete */
	rtc_sync_busy(regs, RTC_MODE2_SYNCBUSY_SWRST_Msk);
}

/**
 * @brief Enable or disable the RTC module.
 *
 * This function enables or disables the RTC (Real-Time Clock) module based on the provided flag.
 * It sets or clears the enable bit in the RTC control register and waits for the synchronization to
 * complete.
 *
 * @param regs Pointer to the RTC hardware registers structure.
 * @param enable Boolean flag to enable (true) or disable (false) the RTC module.
 */
static inline void rtc_enable(rtc_registers_t *regs, bool enable)
{
	if (enable == true) {
		regs->MODE2.RTC_CTRLA |= RTC_MODE2_CTRLA_ENABLE(1);
	} else {
		regs->MODE2.RTC_CTRLA &= ~RTC_MODE2_CTRLA_ENABLE(1);
	}

	/* Wait for the enable/disable operation to complete */
	rtc_sync_busy(regs, RTC_MODE2_SYNCBUSY_ENABLE_Msk);
}

/**
 * @brief Enable the RTC module in clock and calendar mode.
 *
 * This function configures the RTC module to operate in clock and calendar mode.
 * It sets the appropriate bits in the RTC control register and waits for the synchronization to
 * complete.
 *
 * @param  regs Pointer to the RTC hardware registers structure.
 */
static void rtc_enable_clock_calendar_mode(rtc_registers_t *regs)
{
	regs->MODE2.RTC_CTRLA = ((regs->MODE2.RTC_CTRLA &
				  ~(RTC_MODE2_CTRLA_MODE_Msk | RTC_MODE2_CTRLA_CLOCKSYNC_Msk)) |
				 (RTC_MODE2_CTRLA_MODE(2UL) | RTC_MODE2_CTRLA_CLOCKSYNC(1)));

	/* Wait for the clock synchronization to complete */
	rtc_sync_busy(regs, RTC_MODE2_SYNCBUSY_CLOCKSYNC_Msk);
}

/**
 * @brief Set the prescaler value for the RTC module.
 *
 * This function configures the prescaler value for the RTC (Real-Time Clock) module.
 * It updates the prescaler bits in the RTC control register.
 *
 * @param regs Pointer to the RTC hardware registers structure.
 * @param prescaler_value The desired prescaler value to be set.
 */
static inline void rtc_set_prescaler(rtc_registers_t *regs, uint16_t prescaler_value)
{
	regs->MODE2.RTC_CTRLA = ((regs->MODE2.RTC_CTRLA & ~(RTC_MODE2_CTRLA_PRESCALER_Msk)) |
				 RTC_MODE2_CTRLA_PRESCALER(prescaler_value + 1));
}

/**
 * @brief Set the clock time for the RTC module.
 *
 * This function sets the clock time for the RTC (Real-Time Clock) module.
 * It updates the RTC_CLOCK register with the provided time values.
 *
 * @param regs Pointer to the RTC hardware registers structure.
 * @param rtc_set_time Pointer to the structure containing the time values to be set.
 */
static void rtc_set_clock_time(rtc_registers_t *regs, rtc_mchp_time_t *rtc_set_time)
{
	regs->MODE2.RTC_CLOCK =
		(uint32_t)((((RTC_TM_REFERENCE_YEAR + rtc_set_time->year) - RTC_REFERENCE_YEAR)
			    << RTC_MODE2_CLOCK_YEAR_Pos) |
			   ((RTC_ADJUST_MONTH(rtc_set_time->month)) << RTC_MODE2_CLOCK_MONTH_Pos) |
			   (rtc_set_time->date_of_month << RTC_MODE2_CLOCK_DAY_Pos) |
			   (rtc_set_time->hour << RTC_MODE2_CLOCK_HOUR_Pos) |
			   (rtc_set_time->minute << RTC_MODE2_CLOCK_MINUTE_Pos) |
			   (rtc_set_time->second << RTC_MODE2_CLOCK_SECOND_Pos));

	/* Synchronization after writing value to CLOCK Register */
	rtc_sync_busy(regs, RTC_MODE2_SYNCBUSY_CLOCKSYNC_Msk);
}

/**
 * @brief Get the current clock time from the RTC module.
 *
 * This function retrieves the current clock time from the RTC (Real-Time Clock) module.
 * It reads the RTC_CLOCK register and updates the provided time structure with the current time
 * values.
 *
 * @param regs Pointer to the RTC hardware registers structure.
 * @param rtc_get_time Pointer to the structure where the current time values will be stored.
 */
static void rtc_get_clock_time(const rtc_registers_t *regs, rtc_mchp_time_t *rtc_get_time)
{
	uint32_t dataClockCalendar = 0U;

	/* Synchronization before reading value from CLOCK Register */
	rtc_sync_busy(regs, RTC_MODE2_SYNCBUSY_CLOCKSYNC_Msk);

	dataClockCalendar = regs->MODE2.RTC_CLOCK;

	rtc_get_time->hour =
		(dataClockCalendar & RTC_MODE2_CLOCK_HOUR_Msk) >> RTC_MODE2_CLOCK_HOUR_Pos;

	rtc_get_time->minute =
		(dataClockCalendar & RTC_MODE2_CLOCK_MINUTE_Msk) >> RTC_MODE2_CLOCK_MINUTE_Pos;

	rtc_get_time->second =
		(dataClockCalendar & RTC_MODE2_CLOCK_SECOND_Msk) >> RTC_MODE2_CLOCK_SECOND_Pos;

	rtc_get_time->month =
		(((dataClockCalendar & RTC_MODE2_CLOCK_MONTH_Msk) >> RTC_MODE2_CLOCK_MONTH_Pos) -
		 1);

	rtc_get_time->year =
		(((dataClockCalendar & RTC_MODE2_CLOCK_YEAR_Msk) >> RTC_MODE2_CLOCK_YEAR_Pos) +
		 RTC_REFERENCE_YEAR) -
		RTC_TM_REFERENCE_YEAR;

	rtc_get_time->date_of_month =
		(dataClockCalendar & RTC_MODE2_CLOCK_DAY_Msk) >> RTC_MODE2_CLOCK_DAY_Pos;
}

/**
 * @brief Get the supported alarm mask for the RTC.
 *
 * This function retrieves the supported alarm mask for the given RTC instance.
 *
 * @param regs Pointer to the RTC hardware registers structure.
 * @return The supported alarm mask as a 16-bit unsigned integer.
 */
static inline uint16_t rtc_get_alarm_supported_mask(const rtc_registers_t *regs)
{
	uint16_t get_supported_mask = (RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE |
				       RTC_ALARM_TIME_MASK_HOUR | RTC_ALARM_TIME_MASK_MONTHDAY |
				       RTC_ALARM_TIME_MASK_MONTH | RTC_ALARM_TIME_MASK_YEAR);
	return get_supported_mask;
}

/**
 * @brief Set the alarm mask for the specified alarm ID in the RTC module.
 *
 * This function sets the alarm mask for the specified alarm ID in the RTC (Real-Time Clock) module.
 * It writes the provided alarm mask value to the corresponding alarm mask register.
 *
 * @param regs Pointer to the RTC hardware registers structure.
 * @param alarm_id The ID of the alarm to set the mask for.
 * @param alarm_mask The mask value to be set for the specified alarm.
 */
static void rtc_set_alarm_mask(rtc_registers_t *regs, uint16_t alarm_id, uint16_t alarm_mask)
{
	uint16_t set_mask = alarm_mask;

	if (alarm_id == RTC_MCHP_ALARM_1) {
		regs->MODE2.RTC_MASK0 = (uint8_t)((regs->MODE2.RTC_MASK0 & ~RTC_MODE2_MASK0_Msk) |
						  RTC_MODE2_MASK0_SEL(set_mask));

		/* Synchronization after writing value to MASK Register */
		rtc_sync_busy(regs, RTC_MODE2_SYNCBUSY_MASK0_Msk);
	}

	if (alarm_id == RTC_MCHP_ALARM_2) {
		regs->MODE2.RTC_MASK1 = (uint8_t)((regs->MODE2.RTC_MASK1 & ~RTC_MODE2_MASK1_Msk) |
						  RTC_MODE2_MASK1_SEL(set_mask));

		/* Synchronization after writing value to MASK Register */
		rtc_sync_busy(regs, RTC_MODE2_SYNCBUSY_MASK1_Msk);
	}
}

/**
 * @brief Get the alarm mask for the specified alarm ID from the RTC module.
 *
 * This function retrieves the alarm mask for the specified alarm ID from the RTC (Real-Time Clock)
 * module. It reads the alarm mask register and returns the mask value.
 *
 * @param regs Pointer to the RTC hardware registers structure.
 * @param alarm_id The ID of the alarm to retrieve the mask for.
 * @return The mask value of the specified alarm.
 */
static uint16_t rtc_get_alarm_mask(const rtc_registers_t *regs, uint16_t alarm_id)
{
	uint16_t get_mask = 0;

	if (alarm_id == RTC_MCHP_ALARM_1) {
		get_mask = (uint16_t)regs->MODE2.RTC_MASK0;

		/* Synchronization after writing value to MASK Register */
		rtc_sync_busy(regs, RTC_MODE2_SYNCBUSY_MASK0_Msk);
	} else if (alarm_id == RTC_MCHP_ALARM_2) {
		get_mask = (uint16_t)regs->MODE2.RTC_MASK1;

		/* Synchronization after writing value to MASK Register */
		rtc_sync_busy(regs, RTC_MODE2_SYNCBUSY_MASK1_Msk);
	}

	return get_mask;
}

/**
 * @brief Set the alarm time for the specified alarm ID in the RTC module.
 *
 * This function sets the alarm time for the specified alarm ID in the RTC (Real-Time Clock) module.
 * It writes the provided alarm time value to the corresponding alarm time register.
 *
 * @param regs Pointer to the RTC hardware registers structure.
 * @param alarm_id The ID of the alarm to set the time for.
 * @param rtc_set_alarm Pointer to the structure containing the alarm time to be set.
 */
static void rtc_set_alarm_time(rtc_registers_t *regs, uint16_t alarm_id,
			       rtc_mchp_time_t *rtc_set_alarm)
{
	if (alarm_id == RTC_MCHP_ALARM_1) {
		regs->MODE2.RTC_ALARM0 =
			(uint32_t)((((RTC_TM_REFERENCE_YEAR + rtc_set_alarm->year) -
				     RTC_REFERENCE_YEAR)
				    << RTC_MODE2_CLOCK_YEAR_Pos) |
				   ((RTC_ADJUST_MONTH(rtc_set_alarm->month))
				    << RTC_MODE2_CLOCK_MONTH_Pos) |
				   (rtc_set_alarm->date_of_month << RTC_MODE2_CLOCK_DAY_Pos) |
				   (rtc_set_alarm->hour << RTC_MODE2_CLOCK_HOUR_Pos) |
				   (rtc_set_alarm->minute << RTC_MODE2_CLOCK_MINUTE_Pos) |
				   (rtc_set_alarm->second << RTC_MODE2_CLOCK_SECOND_Pos));
	}

	if (alarm_id == RTC_MCHP_ALARM_2) {
		regs->MODE2.RTC_ALARM1 =
			(uint32_t)((((RTC_TM_REFERENCE_YEAR + rtc_set_alarm->year) -
				     RTC_REFERENCE_YEAR)
				    << RTC_MODE2_CLOCK_YEAR_Pos) |
				   ((RTC_ADJUST_MONTH(rtc_set_alarm->month))
				    << RTC_MODE2_CLOCK_MONTH_Pos) |
				   (rtc_set_alarm->date_of_month << RTC_MODE2_CLOCK_DAY_Pos) |
				   (rtc_set_alarm->hour << RTC_MODE2_CLOCK_HOUR_Pos) |
				   (rtc_set_alarm->minute << RTC_MODE2_CLOCK_MINUTE_Pos) |
				   (rtc_set_alarm->second << RTC_MODE2_CLOCK_SECOND_Pos));
	}

	/* Synchronization after writing value to CLOCK Register */
	rtc_sync_busy(regs, RTC_MODE2_SYNCBUSY_CLOCKSYNC_Msk);
}

/**
 * @brief Get the alarm time for the specified alarm ID from the RTC module.
 *
 * This function retrieves the alarm time for the specified alarm ID from the RTC (Real-Time Clock)
 * module. It reads the alarm time registers and populates the provided rtc_mchp_time_t structure
 * with the alarm time.
 *
 * @param regs Pointer to the RTC hardware registers structure.
 * @param alarm_id The ID of the alarm to retrieve the time for.
 * @param rtc_get_time Pointer to the structure where the retrieved alarm time will be stored.
 */
static void rtc_get_alarm_time(const rtc_registers_t *regs, uint16_t alarm_id,
			       rtc_mchp_time_t *rtc_get_time)
{
	uint32_t dataClockCalendar = 0U;

	/* Synchronization before reading value from CLOCK Register */
	rtc_sync_busy(regs, RTC_MODE2_SYNCBUSY_CLOCKSYNC_Msk);

	if (alarm_id == RTC_MCHP_ALARM_1) {
		dataClockCalendar = regs->MODE2.RTC_ALARM0;
	} else if (alarm_id == RTC_MCHP_ALARM_2) {
		dataClockCalendar = regs->MODE2.RTC_ALARM1;
	}
	rtc_get_time->hour =
		(dataClockCalendar & RTC_MODE2_CLOCK_HOUR_Msk) >> RTC_MODE2_CLOCK_HOUR_Pos;

	rtc_get_time->minute =
		(dataClockCalendar & RTC_MODE2_CLOCK_MINUTE_Msk) >> RTC_MODE2_CLOCK_MINUTE_Pos;

	rtc_get_time->second =
		(dataClockCalendar & RTC_MODE2_CLOCK_SECOND_Msk) >> RTC_MODE2_CLOCK_SECOND_Pos;

	rtc_get_time->month =
		(((dataClockCalendar & RTC_MODE2_CLOCK_MONTH_Msk) >> RTC_MODE2_CLOCK_MONTH_Pos) -
		 1);

	rtc_get_time->year =
		(((dataClockCalendar & RTC_MODE2_CLOCK_YEAR_Msk) >> RTC_MODE2_CLOCK_YEAR_Pos) +
		 RTC_REFERENCE_YEAR) -
		RTC_TM_REFERENCE_YEAR;

	rtc_get_time->date_of_month =
		(dataClockCalendar & RTC_MODE2_CLOCK_DAY_Msk) >> RTC_MODE2_CLOCK_DAY_Pos;
}

/**
 * @brief Enable the specified interrupt for the RTC module.
 *
 * This function enables the specified interrupt for the RTC (Real-Time Clock) module.
 * It sets the corresponding bit in the RTC interrupt enable register.
 *
 * @param regs Pointer to the RTC hardware registers structure.
 * @param alarm_id The ID of the alarm interrupt to be enabled.
 */
static void rtc_enable_interrupt(rtc_registers_t *regs, uint16_t alarm_id)
{
	uint16_t alarm_int = 0;

	if (alarm_id == RTC_MCHP_ALARM_1) {
		alarm_int |= RTC_MODE2_INTENSET_ALARM0(1);
	}
	if (alarm_id == RTC_MCHP_ALARM_2) {
		alarm_int |= RTC_MODE2_INTENSET_ALARM1(1);
	}

	regs->MODE2.RTC_INTENSET = alarm_int;
}

/**
 * @brief Disable the specified interrupt for the RTC module.
 *
 * This function disables the specified interrupt for the RTC (Real-Time Clock) module.
 * It clears the corresponding bit in the RTC interrupt enable register.
 *
 * @param regs Pointer to the RTC hardware registers structure.
 * @param alarm_id The ID of the alarm interrupt to be disabled.
 */
static void rtc_disable_interrupt(rtc_registers_t *regs, uint16_t alarm_id)
{
	uint16_t alarm_int = 0;

	if (alarm_id == RTC_MCHP_ALARM_1) {
		alarm_int |= RTC_MODE2_INTENCLR_ALARM0(1);
	}
	if (alarm_id == RTC_MCHP_ALARM_2) {
		alarm_int |= RTC_MODE2_INTENCLR_ALARM1(1);
	}

	regs->MODE2.RTC_INTENCLR = alarm_int;
}

/**
 * @brief Get the interrupt flags for the RTC.
 *
 * This function retrieves the interrupt flags for the Real-Time Clock (RTC) hardware.
 * The flags indicate which interrupt events have occurred.
 *
 * @param regs Pointer to the RTC hardware registers structure.
 * @param alarm_id Pointer to a variable where the alarm ID will be stored.
 * @return The interrupt flags as a 16-bit unsigned integer.
 */
static uint16_t rtc_get_interrupt_flags(const rtc_registers_t *regs, uint16_t *alarm_id)
{
	uint16_t int_status = regs->MODE2.RTC_INTFLAG;

	if ((int_status & RTC_MODE2_INTFLAG_ALARM0_Msk) == RTC_MODE2_INTFLAG_ALARM0_Msk) {
		*alarm_id = RTC_MCHP_ALARM_1;
	} else if ((int_status & RTC_MODE2_INTFLAG_ALARM1_Msk) == RTC_MODE2_INTFLAG_ALARM1_Msk) {
		*alarm_id = RTC_MCHP_ALARM_2;
	}

	return int_status;
}

/**
 * @brief Clear the interrupt flags for the RTC.
 *
 * This function clears the interrupt flags for the Real-Time Clock (RTC) hardware.
 * The interrupt flags indicate which interrupt events have occurred and need to be cleared.
 *
 * @param regs Pointer to the RTC hardware registers structure.
 * @param alarm_id The ID of the alarm whose interrupt flags need to be cleared.
 */
static void rtc_clear_interrupt_flags(rtc_registers_t *regs, uint16_t alarm_id)
{
	uint16_t alarm_status = 0;

	if (alarm_id == RTC_MCHP_ALARM_1) {
		alarm_status |= RTC_MODE2_INTFLAG_ALARM0_Msk;
	} else if (alarm_id == RTC_MCHP_ALARM_2) {
		alarm_status |= RTC_MODE2_INTFLAG_ALARM1_Msk;
	}

	regs->MODE2.RTC_INTFLAG = alarm_status;
}

/**
 * @brief Get the supported alarm interrupt flags for the RTC.
 *
 * This function retrieves the supported alarm interrupt flags for the Real-Time Clock (RTC)
 * hardware. The flags indicate which alarm interrupt features are supported by the RTC.
 *
 * @param regs Pointer to the RTC hardware registers structure.
 * @return The supported alarm interrupt flags as a 16-bit unsigned integer.
 */
static inline uint16_t rtc_supported_alarm_int_flags(const rtc_registers_t *regs)
{
	uint16_t supported_flags = (RTC_MODE2_INTFLAG_ALARM0_Msk | RTC_MODE2_INTFLAG_ALARM1_Msk);

	return supported_flags;
}

/**
 * @brief Set the calibration value for the RTC module.
 *
 * This function sets the calibration value for the RTC (Real-Time Clock) module.
 * It writes the provided calibration value and correction sign to the RTC_FREQCORR register.
 *
 * @param regs Pointer to the RTC hardware registers structure.
 * @param calib The calibration value to be set.
 * @param correction The correction sign to be set.
 */
static void mchp_rtc_set_calibration(rtc_registers_t *regs, uint32_t calib, uint32_t correction)
{
	/* Combine the calibration value and correction sign */
	uint32_t freqcorr_value =
		((calib & RTC_FREQCORR_VALUE_Msk) | (correction & RTC_FREQCORR_SIGN_Msk));

	/* Write the combined value to the RTC_FREQCORR register */
	regs->MODE2.RTC_FREQCORR = (uint8_t)freqcorr_value;

	/* Synchronization after writing value to RTC_FREQCORR Register */
	rtc_sync_busy(regs, RTC_MODE2_SYNCBUSY_FREQCORR_Msk);
}

/**
 * @brief Get the current calibration value from the RTC module.
 *
 * This function retrieves the current calibration value from the RTC (Real-Time Clock) module.
 * It reads the RTC_FREQCORR register and updates the provided calibration value and correction
 * sign.
 *
 * @param regs Pointer to the RTC hardware registers structure.
 * @param calib Pointer to the variable where the calibration value will be stored.
 * @param corr_sign Pointer to the variable where the correction sign will be stored.
 */
static void mchp_rtc_get_calibration(const rtc_registers_t *regs, uint32_t *calib,
				     uint8_t *corr_sign)
{
	/* Read the calibration value from the RTC_FREQCORR register */
	*calib = (uint32_t)(regs->MODE2.RTC_FREQCORR & RTC_FREQCORR_VALUE_Msk);

	/* Read the correction sign from the RTC_FREQCORR register */
	if ((regs->MODE2.RTC_FREQCORR & RTC_FREQCORR_SIGN_Msk) == RTC_FREQCORR_SIGN_Msk) {
		*corr_sign = 1;
	}
}

/***********************************
 * Zephyr APIs
 ***********************************/

/**
 * @brief RTC interrupt service routine.
 *
 * This function handles the RTC interrupts, clears the interrupt flags, and calls the appropriate
 * alarm callback functions if they are set.
 *
 * @param dev Pointer to the device structure representing the RTC.
 */

#if defined(CONFIG_RTC_ALARM)

static void rtc_mchp_isr(const struct device *dev)
{
	rtc_mchp_dev_data_t *data = dev->data;
	const rtc_mchp_dev_config_t *const cfg = dev->config;
	uint16_t alarm_id = -1;

	/* Get the interrupt flags and the alarm ID and clear the interrupt flags */
	uint16_t rtc_int_flag = rtc_get_interrupt_flags(cfg->regs, &alarm_id);

	rtc_clear_interrupt_flags(cfg->regs, alarm_id);

#if defined(CONFIG_RTC_ALARM)

	for (uint8_t alarm = 0; alarm <= alarm_id; alarm++) {
		if ((rtc_int_flag & rtc_supported_alarm_int_flags(cfg->regs)) != 0) {
			if (data->alarms[alarm].alarm_cb != NULL) {
				data->alarms[alarm].alarm_cb(dev, alarm,
							     data->alarms[alarm].alarm_user_data);
				data->alarms[alarm].is_alarm_pending = false;
			} else {
				data->alarms[alarm].is_alarm_pending = true;
			}
		}
	}

#endif /* CONFIG_RTC_ALARM */
}
#endif

#ifdef CONFIG_RTC_ALARM

/**
 * @brief Get the supported fields for RTC alarm.
 *
 * This function retrieves the supported fields for the RTC alarm and sets the mask accordingly.
 *
 * @param dev Pointer to the device structure representing the RTC.
 * @param id The ID of the alarm to check.
 * @param mask Pointer to the variable where the supported fields mask will be stored.
 *
 * @return 0 on success.
 */

static int rtc_mchp_get_alarm_supported_fields(const struct device *dev, uint16_t id,
					       uint16_t *mask)
{
	const rtc_mchp_dev_config_t *const cfg = dev->config;

	ARG_UNUSED(id);

	*mask = rtc_get_alarm_supported_mask(cfg->regs);

	return 0;
}

/**
 * @brief Check if a specific alarm is pending in the RTC device.
 *
 * This function checks if the alarm specified by alarm_id is pending.
 * If the alarm is pending, it disables the interrupt for the alarm and
 * clears the pending status.
 *
 * @param dev Pointer to the device structure representing the RTC.
 * @param alarm_id The ID of the alarm to check.
 * @return 0 if no alarm is pending, non-zero if an alarm is pending,
 *         -EINVAL if the alarm_id is out of range.
 */
static int rtc_mchp_alarm_is_pending(const struct device *dev, uint16_t alarm_id)
{
	rtc_mchp_dev_data_t *data = dev->data;
	const rtc_mchp_dev_config_t *const cfg = dev->config;
	int retval = 0;

	/* Check if the alarm ID is within the valid range */
	if (alarm_id >= cfg->alarms_count) {
		LOG_ERR("RTC Alarm id is out of range");
		retval = -EINVAL;
	} else {
		/* Lock interrupts to ensure setting of callback*/
		unsigned int key = irq_lock();

		if (data->alarms[alarm_id].is_alarm_pending == true) {

			retval = data->alarms[alarm_id].is_alarm_pending;
			/* Clear the pending status of the alarm. */
			data->alarms[alarm_id].is_alarm_pending = false;
		}
		/* Lock interrupts after alarm pending check */
		irq_unlock(key);
	}

	return retval;
}
/**
 * @brief Convert the provided alarm mask to the hardware-specific format.
 *
 * This function converts the provided alarm mask into a format that the hardware RTC can
 * understand.
 *
 * @param alarm_mask The provided alarm mask in Zephyr format.
 *
 * @return The hardware-specific alarm mask to write to the register.
 */
static uint16_t rtc_mchp_alarm_mask(uint16_t alarm_mask)
{
	uint16_t rtc_alarm_enable_mask = 0;

	if (alarm_mask & RTC_ALARM_TIME_MASK_SECOND) {
		rtc_alarm_enable_mask = RTC_MCHP_ALARM_MASK_SEL_SS;
	}
	if (alarm_mask & RTC_ALARM_TIME_MASK_MINUTE) {
		rtc_alarm_enable_mask = RTC_MCHP_ALARM_MASK_SEL_MMSS;
	}
	if (alarm_mask & RTC_ALARM_TIME_MASK_HOUR) {
		rtc_alarm_enable_mask = RTC_MCHP_ALARM_MASK_SEL_HHMMSS;
	}
	if (alarm_mask & RTC_ALARM_TIME_MASK_MONTHDAY) {
		rtc_alarm_enable_mask = RTC_MCHP_ALARM_MASK_SEL_DDHHMMSS;
	}
	if (alarm_mask & RTC_ALARM_TIME_MASK_MONTH) {
		rtc_alarm_enable_mask = RTC_MCHP_ALARM_MASK_SEL_MMDDHHMMSS;
	}
	if (alarm_mask & RTC_ALARM_TIME_MASK_YEAR) {
		rtc_alarm_enable_mask = RTC_MCHP_ALARM_MASK_SEL_YYMMDDHHMMSS;
	}

	return rtc_alarm_enable_mask;
}

/**
 * @brief Convert the mask value from hardware-specific format to expected zephyr mask format.
 *
 * This function converts the read mask from rtc registers into a format that zephyr
 * expects.
 *
 * @param mask The mask value read from RTC mask registers.
 *
 * @return The converted alarm mask.
 */
static uint16_t rtc_mchp_mask_from_alarm_msk(uint16_t mask)
{
	uint16_t alarm_mask = 0;

	switch (mask) {
	case RTC_MCHP_ALARM_MASK_SEL_YYMMDDHHMMSS:
		alarm_mask |= RTC_ALARM_TIME_MASK_YEAR;
		__fallthrough;
	case RTC_MCHP_ALARM_MASK_SEL_MMDDHHMMSS:
		alarm_mask |= RTC_ALARM_TIME_MASK_MONTH;
		__fallthrough;
	case RTC_MCHP_ALARM_MASK_SEL_DDHHMMSS:
		alarm_mask |= RTC_ALARM_TIME_MASK_MONTHDAY;
		__fallthrough;
	case RTC_MCHP_ALARM_MASK_SEL_HHMMSS:
		alarm_mask |= RTC_ALARM_TIME_MASK_HOUR;
		__fallthrough;
	case RTC_MCHP_ALARM_MASK_SEL_MMSS:
		alarm_mask |= RTC_ALARM_TIME_MASK_MINUTE;
		__fallthrough;
	case RTC_MCHP_ALARM_MASK_SEL_SS:
		alarm_mask |= RTC_ALARM_TIME_MASK_SECOND;
		break;
	default:
		break;
	}

	return alarm_mask;
}

/**
 * @brief Set the RTC alarm time.
 *
 * This function sets the alarm time for the specified alarm ID.
 *
 * @param dev Pointer to the device structure.
 * @param alarm_id ID of the alarm to set.
 * @param alarm_mask Mask specifying which fields to set for alarm.
 * @param timeptr Pointer to the rtc_time structure containing the time to set.
 *
 * @return 0 on success, -EINVAL on invalid parameters.
 */
static int rtc_mchp_set_alarm_time(const struct device *dev, uint16_t alarm_id, uint16_t alarm_mask,
				   const struct rtc_time *timeptr)
{
	const rtc_mchp_dev_config_t *const cfg = dev->config;
	rtc_mchp_time_t rtc_time;
	uint16_t supported_mask;
	uint16_t set_mask = 0;
	int retval = 0;

	/* Get the supported alarm fields */
	rtc_mchp_get_alarm_supported_fields(dev, 0, &supported_mask);

	do {
		/* Check if the provided alarm mask is valid */
		if (alarm_mask & ~supported_mask) {
			LOG_ERR("Invalid RTC alarm mask");
			retval = -EINVAL;
			break;
		}
		/* Check if the alarm ID is within the valid range */
		if (alarm_id >= cfg->alarms_count) {
			LOG_ERR("RTC Alarm id is out of range");
			retval = -EINVAL;
			break;
		}
		/* Check if the time pointer is provided when the alarm mask is not zero */
		if ((timeptr == NULL) && (alarm_mask != 0)) {
			LOG_ERR("No pointer is provided to set RTC alarm");
			retval = -EINVAL;
			break;
		}

		/* Validate the provided RTC time */
		if (timeptr && !rtc_utils_validate_rtc_time(timeptr, supported_mask)) {
			retval = -EINVAL;
			break;
		}

		/* Lock interrupts to ensure setting of callback */
		unsigned int key = irq_lock();

		/* Disable the interrupt for the specified alarm ID */
		rtc_disable_interrupt(cfg->regs, alarm_id);

		if (alarm_mask == 0) {
			/* If the alarm mask is zero, turn off the alarm */
			rtc_set_alarm_mask(cfg->regs, alarm_id, RTC_MCHP_ALARM_MASK_SEL_OFF);
		} else {

			/* If validation passed, set the RTC ALARM time */
			rtc_time.second = timeptr->tm_sec;
			rtc_time.minute = timeptr->tm_min;
			rtc_time.hour = timeptr->tm_hour;
			rtc_time.month = timeptr->tm_mon;
			rtc_time.date_of_month = timeptr->tm_mday;
			rtc_time.year = timeptr->tm_year;

			/* Set the alarm time */
			rtc_set_alarm_time(cfg->regs, alarm_id, &rtc_time);

			/* Enable the interrupt for the specified alarm ID */
			set_mask = rtc_mchp_alarm_mask(alarm_mask);

			rtc_set_alarm_mask(cfg->regs, alarm_id, set_mask);

			/* Enable the interrupt for the specified alarm ID */
			rtc_enable_interrupt(cfg->regs, alarm_id);
		}

		/* Unlock the IRQ after completion of setting callback */
		irq_unlock(key);

	} while (0);

	return retval;
}

/**
 * @brief Get the alarm time for a specific alarm in the RTC device.
 *
 * This function retrieves the time and mask for the specified alarm.
 * If the alarm ID is out of range or the time pointer is NULL, it returns an error.
 *
 * @param dev Pointer to the device structure representing the RTC.
 * @param alarm_id The ID of the alarm to get the time for.
 * @param alarm_mask Pointer to store the mask of fields enabled in the alarm time.
 * @param timeptr Pointer to the structure to store the retrieved alarm time.
 * @return 0 if the alarm time was retrieved successfully, -EINVAL if the alarm_id is out of range
 *         or the time pointer is NULL.
 */
static int rtc_mchp_get_alarm_time(const struct device *dev, uint16_t alarm_id,
				   uint16_t *alarm_mask, struct rtc_time *timeptr)
{
	rtc_mchp_dev_data_t *data = dev->data;
	const rtc_mchp_dev_config_t *const cfg = dev->config;
	rtc_mchp_time_t rtc_alarm_time = {0};
	int retval = 0;
	uint16_t mask;

	/* Check if the alarm ID is within the valid range */
	if (alarm_id >= cfg->alarms_count) {
		LOG_ERR("RTC Alarm id is out of range");
		retval = -EINVAL;
	} else {

		/* Check if the time pointer is provided when the alarm mask is not zero */
		if (timeptr == NULL) {
			LOG_ERR("No pointer is provided to get RTC alarm");
			retval = -EINVAL;
		} else {
			/* Lock the mutex before accessing the RTC. */
			RTC_DATA_LOCK(&data->lock);

			/* get the rtc alarm time from id */
			rtc_get_alarm_time(cfg->regs, alarm_id, &rtc_alarm_time);

			/* Get the mask of fields which are enabled in the alarm time */
			mask = rtc_get_alarm_mask(cfg->regs, alarm_id);

			*alarm_mask = rtc_mchp_mask_from_alarm_msk(mask);

			/* Populate the rtc_time structure with the retrieved values. */
			timeptr->tm_sec = (int)rtc_alarm_time.second;
			timeptr->tm_min = (int)rtc_alarm_time.minute;
			timeptr->tm_hour = (int)rtc_alarm_time.hour;
			timeptr->tm_mon = (int)rtc_alarm_time.month;
			timeptr->tm_mday = (int)rtc_alarm_time.date_of_month;
			timeptr->tm_year = (int)rtc_alarm_time.year;

			/* Unlock the mutex before returning. */
			RTC_DATA_UNLOCK(&data->lock);
		}
	}

	return retval;
}
/**
 * @brief Set a callback function for a specific alarm in the RTC device.
 *
 * This function sets a callback function that will be called when the specified alarm
 * triggers. If the alarm ID is out of range or the alarm is not pending, it returns an
 * error.
 *
 * @param dev Pointer to the device structure representing the RTC.
 * @param alarm_id The ID of the alarm to set the callback for.
 * @param callback The callback function to be called when the alarm triggers.
 * @param user_data Pointer to user data that will be passed to the callback function.
 * @return 0 if the callback was set successfully, -EINVAL if the alarm_id is out of range,
 *         the alarm is not pending, or the callback is NULL.
 */
static int rtc_mchp_set_alarm_callback(const struct device *dev, uint16_t alarm_id,
				       rtc_alarm_callback callback, void *user_data)
{
	rtc_mchp_dev_data_t *data = dev->data;
	const rtc_mchp_dev_config_t *const cfg = dev->config;
	int retval = 0;

	/* Check if the alarm ID is within the valid range */
	if (alarm_id >= cfg->alarms_count) {
		LOG_ERR("RTC Alarm id is out of range");
		retval = -EINVAL;
	} else {
		/* Lock interrupts to ensure setting of callback */
		unsigned int key = irq_lock();

		if ((callback == NULL) && (user_data == NULL)) {
			/* Disable the interrupt for the specified alarm ID */
			rtc_disable_interrupt(cfg->regs, alarm_id);
		} else {
			/* Enable the interrupt for the specified alarm ID */
			rtc_enable_interrupt(cfg->regs, alarm_id);
		}
		if (retval == 0) {
			/* Set the callback function for the alarm and user data. */
			data->alarms[alarm_id].alarm_cb = callback;
			data->alarms[alarm_id].alarm_user_data = user_data;
		}

		/* Unlock the IRQ after completion of setting callback */
		irq_unlock(key);
	}

	return retval;
}

#endif

/**
 * @brief Validate the RTC clock time.
 *
 * This function checks if the provided RTC clock time values are within valid ranges.
 *
 * @param rtc_clock_time Pointer to the rtc_time structure containing the time values to
 * validate.
 * @return true if all time values are within valid ranges, false otherwise.
 */
static bool rtc_mchp_validate_rtc_clock_time(const struct rtc_time *rtc_clock_time)
{
	bool retval = true;

	/* Validate seconds (0-59) */
	if (rtc_clock_time->tm_sec < 0 || rtc_clock_time->tm_sec > 59) {
		retval = false;
	}

	/* Validate minutes (0-59) */
	if (rtc_clock_time->tm_min < 0 || rtc_clock_time->tm_min > 59) {
		retval = false;
	}

	/* Validate hours (0-23) */
	if (rtc_clock_time->tm_hour < 0 || rtc_clock_time->tm_hour > 23) {
		retval = false;
	}

	/* Validate months (0-11, where 0 = January) */
	if (rtc_clock_time->tm_mon < 0 || rtc_clock_time->tm_mon > 11) {
		retval = false;
	}

	/* Validate days of the month (1-31) */
	if (rtc_clock_time->tm_mday < 1 || rtc_clock_time->tm_mday > 31) {
		retval = false;
	}

	return retval;
}

/**
 * @brief Set the RTC clock time.
 *
 * This function sets the RTC clock time based on the provided time structure.
 *
 * @param dev Pointer to the device structure.
 * @param timeptr Pointer to the rtc_time structure containing the time to set.
 *
 * @return 0 on success, -EINVAL if the time parameters are invalid.
 */

static int rtc_mchp_set_clock_time(const struct device *dev, const struct rtc_time *timeptr)
{
	rtc_mchp_dev_data_t *data = dev->data;
	const rtc_mchp_dev_config_t *const cfg = dev->config;
	rtc_mchp_time_t rtc_time;
	int retval = 0;

	/* Check if rtc_time structure not null */
	if (timeptr == NULL) {
		retval = -EINVAL;

	} else {
		/* Validate the provided RTC time parameters */
		if (rtc_mchp_validate_rtc_clock_time(timeptr) == false) {
			retval = -EINVAL;
		}

		if (retval == 0) {
			/* lock the mutex before setting the RTC. */
			RTC_DATA_LOCK(&data->lock);

			/* If validation passed, set the RTC time */
			rtc_time.second = timeptr->tm_sec;
			rtc_time.minute = timeptr->tm_min;
			rtc_time.hour = timeptr->tm_hour;
			rtc_time.month = timeptr->tm_mon;
			rtc_time.date_of_month = timeptr->tm_mday;
			rtc_time.year = timeptr->tm_year;

			rtc_set_clock_time(cfg->regs, &rtc_time);

			/* Unlock the mutex before returning. */
			RTC_DATA_UNLOCK(&data->lock);
		}
	}

	return retval;
}

/**
 * @brief Get the current clock time from the RTC.
 *
 * This function retrieves the current time from the RTC and populates the
 * provided `rtc_time` structure with the retrieved values. It ensures thread
 * safety by locking a mutex before accessing the RTC and unlocking it after
 * the operation is complete.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param timeptr Pointer to the `rtc_time` structure to be populated with the current time.
 *
 * @return 0 on success
 */

static int rtc_mchp_get_clock_time(const struct device *dev, struct rtc_time *timeptr)
{
	rtc_mchp_dev_data_t *data = dev->data;
	const rtc_mchp_dev_config_t *const cfg = dev->config;
	rtc_mchp_time_t rtc_current_time;

	/* Lock the mutex before accessing the RTC. */
	RTC_DATA_LOCK(&data->lock);

	/* Retrieve the current time from the RTC. */
	rtc_get_clock_time(cfg->regs, &rtc_current_time);

	/* Populate the rtc_time structure with the retrieved values. */
	timeptr->tm_sec = (int)rtc_current_time.second;
	timeptr->tm_min = (int)rtc_current_time.minute;
	timeptr->tm_hour = (int)rtc_current_time.hour;
	timeptr->tm_mon = (int)rtc_current_time.month;
	timeptr->tm_mday = (int)rtc_current_time.date_of_month;
	timeptr->tm_year = (int)rtc_current_time.year;

	/* Unlock the mutex before returning. */
	RTC_DATA_UNLOCK(&data->lock);

	return 0;
}

#ifdef CONFIG_RTC_CALIBRATION

/**
 * @brief Set the RTC calibration value.
 *
 * This function sets the calibration value for the RTC to adjust for any drift.
 * The calibration value is divided by a constant to get the correction value,
 * which is then applied to the RTC's frequency correction register.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param calibration The calibration value to be set.
 *
 * @return 0 on success, -EINVAL if the calibration value is out of range.
 */
static int rtc_mchp_set_calibration(const struct device *dev, int32_t calibration)
{
	const rtc_mchp_dev_config_t *const cfg = dev->config;
	int retval = 0;

	/* Calculate the calibration value in parts per billion */
	int32_t correction = calibration / (RTC_CALIB_PARTS_PER_BILLION / cfg->cal_constant);
	uint32_t abs_correction = abs(correction);

	LOG_DBG("Correction: %d, Absolute: %d, Calibration: %d", correction, abs_correction,
		calibration);

	/* Check if correction value is 0 */
	if (abs_correction == 0) {
		mchp_rtc_set_calibration(cfg->regs, 0, 0);

	} else {

		/* Check if correction value out of range */
		if (abs_correction > RTC_CALIBRATE_PPB_MAX) {
			LOG_ERR("The RTC calibration %d result in an out of range value %d",
				calibration, abs_correction);
			retval = -EINVAL;
		} else {

			/* Set the correction value into RTC calib register */
			mchp_rtc_set_calibration(cfg->regs, abs_correction, correction);
		}
	}

	return retval;
}

/**
 * @brief Get the calibration value for the RTC device.
 *
 * This function retrieves the current calibration value for the RTC device.
 * If the calibration pointer is NULL, it returns an error.
 *
 * @param dev Pointer to the device structure representing the RTC.
 * @param calibration Pointer to store the retrieved calibration value.
 * @return 0 if the calibration value was retrieved successfully, -EINVAL if the calibration pointer
 * is NULL.
 */
static int rtc_mchp_get_calibration(const struct device *dev, int32_t *calibration)
{
	const rtc_mchp_dev_config_t *const cfg = dev->config;
	int retval = 0;
	int32_t correction;
	uint8_t correction_sign = 0;

	/* Check if the calibration pointer is NULL */
	if (calibration == NULL) {
		retval = -EINVAL;
	} else {
		/* Retrieve the correction value from the hardware register */
		mchp_rtc_get_calibration(cfg->regs, &correction, &correction_sign);

		/* Calculate the calibration value based on the correction value */
		if (correction == 0) {
			*calibration = 0;
		} else {
			*calibration = ((int64_t)correction * RTC_CALIB_PARTS_PER_BILLION) /
				       cfg->cal_constant;
		}
		/* Adjust the calibration value based on the sign bit */
		if (correction_sign == 1) {
			*calibration *= -1;
		}
	}

	return retval;
}

#endif /* CONFIG_RTC_CALIBRATION */

/*
 * @brief Initialize the RTC (Real-Time Clock) for the Microchip device.
 *
 * This function initializes the RTC hardware by enabling the clock, performing a software
 * reset, setting the prescaler, enabling the clock calendar mode, and enabling the RTC. If
 * RTC alarm configuration is enabled, it also configures the IRQ for the RTC peripheral.
 *
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @return Returns 0 when clock initialization success.
 */

static int rtc_mchp_init(const struct device *dev)
{
	rtc_mchp_dev_data_t *data = dev->data;
	const rtc_mchp_dev_config_t *const cfg = dev->config;
	int16_t ret = 0;

	do {
		/* On Oscillator clock for RTC */
		ret = clock_control_on(cfg->rtc_clock.clock_dev,
				       (clock_control_subsys_t)&cfg->rtc_clock.osc32k_sys);
		if ((ret != 0) && (ret != -EALREADY)) {
			LOG_ERR("Failed to enable the osc32k clock for RTC: %d", ret);
			break;
		}

		/* On Main clock for RTC */
		ret = clock_control_on(cfg->rtc_clock.clock_dev,
				       (clock_control_subsys_t)&cfg->rtc_clock.mclk_sys);
		if ((ret != 0) && (ret != -EALREADY)) {
			LOG_ERR("Failed to enable the MCLK for RTC: %d", ret);
			break;
		}
		/* Initialize mutex for RTC data structure */
		RTC_DATA_LOCK_INIT(&data->lock);

		/* Perform a software reset on the RTC peripheral */
		rtc_swrst(cfg->regs);

		/* Set the prescaler for the RTC peripheral */
		rtc_set_prescaler(cfg->regs, cfg->prescaler);

		/* Enable the clock calendar mode for the RTC peripheral */
		rtc_enable_clock_calendar_mode(cfg->regs);

		/* Enable the RTC peripheral */
		rtc_enable(cfg->regs, true);

#if defined(CONFIG_RTC_ALARM)
		/* Configure the IRQ for the RTC peripheral */
		cfg->irq_config_func(dev);
#endif

	} while (0);

	if (ret == -EALREADY) {
		ret = 0;
	}

	return ret;
}

static const struct rtc_driver_api rtc_mchp_driver_api = {
	.set_time = rtc_mchp_set_clock_time,
	.get_time = rtc_mchp_get_clock_time,
#ifdef CONFIG_RTC_ALARM
	.alarm_get_supported_fields = rtc_mchp_get_alarm_supported_fields,
	.alarm_is_pending = rtc_mchp_alarm_is_pending,
	.alarm_set_time = rtc_mchp_set_alarm_time,
	.alarm_get_time = rtc_mchp_get_alarm_time,
	.alarm_set_callback = rtc_mchp_set_alarm_callback,
#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_CALIBRATION
	.set_calibration = rtc_mchp_set_calibration,
	.get_calibration = rtc_mchp_get_calibration,
#endif /* CONFIG_RTC_CALIBRATION */
};

/**
 * @brief Defines the RTC interrupt configurations @p n.
 *
 * Used to configure interrupts for the RTC peripheral.
 */
#if defined(CONFIG_RTC_ALARM)
#define RTC_MCHP_IRQ_CONNECT(n, m)                                                                 \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, m, irq), DT_INST_IRQ_BY_IDX(n, m, priority),     \
			    rtc_mchp_isr, DEVICE_DT_INST_GET(n), 0);                               \
		irq_enable(DT_INST_IRQ_BY_IDX(n, m, irq));                                         \
	} while (false)

#endif

/**
 * @brief Defines the RTC driver cclock onfiguration @p n.
 *
 * Sets the clock configuration clock dev, main clock and oscillator clock systems for the RTC
 * instance.
 */
#define RTC_MCHP_CLOCK_DEFN(n)                                                                     \
	.rtc_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                                 \
	.rtc_clock.mclk_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, mclk)),         \
			       .id = DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, id)},                    \
	.rtc_clock.osc32k_sys = {.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, osc32kctrl)), \
				 .id = DT_INST_CLOCKS_CELL_BY_NAME(n, osc32kctrl, id)},

/* Do the peripheral interrupt related configuration */
#if defined(CONFIG_RTC_ALARM)
#define RTC_MCHP_IRQ_HANDLER(n)                                                                    \
	static void rtc_mchp_irq_config_##n(const struct device *dev)                              \
	{                                                                                          \
		RTC_MCHP_IRQ_CONNECT(n, 0);                                                        \
	}

#endif

/**
 * @brief Defines the RTC driver configuration structure for instance @p n.
 *
 * Sets the configuration function, pin control, register mappings,
 * and clock definitions for the RTC instance.
 */
/* clang-format off */
#define RTC_MCHP_CONFIG_DEFN(n)        \
	static const rtc_mchp_dev_config_t rtc_mchp_dev_config_##n = {		\
		.regs = (rtc_registers_t *)DT_INST_REG_ADDR(n),		\
		.prescaler = DT_INST_ENUM_IDX(n, prescaler),		\
	IF_ENABLED(CONFIG_RTC_ALARM, (		\
		.alarms_count = DT_INST_PROP(n, alarms_count),	\
		.irq_config_func = &rtc_mchp_irq_config_##n,))		\
	IF_ENABLED(CONFIG_RTC_CALIBRATION, (		\
		.cal_constant = DT_INST_PROP(n, cal_constant),))		\
	RTC_MCHP_CLOCK_DEFN(n)     \
	}
/* clang-format on */
/**
 * @brief Instantiates the RTC device @p n.
 *
 * This macro handles configuration, data definition, and registration
 * of the RTC device in the Zephyr device model.
 */
#define RTC_MCHP_DEVICE_INIT(n)                                                                    \
	IF_ENABLED(CONFIG_RTC_ALARM, (						\
	static void rtc_mchp_irq_config_##n(const struct device *dev);		\
))                                                     \
	RTC_MCHP_CONFIG_DEFN(n);                                                                   \
	static rtc_mchp_dev_data_t rtc_mchp_dev_data_##n;                                          \
	DEVICE_DT_INST_DEFINE(n, rtc_mchp_init, NULL, &rtc_mchp_dev_data_##n,                      \
			      &rtc_mchp_dev_config_##n, POST_KERNEL, CONFIG_RTC_INIT_PRIORITY,     \
			      &rtc_mchp_driver_api);                                               \
	IF_ENABLED(CONFIG_RTC_ALARM, (						\
					RTC_MCHP_IRQ_HANDLER(n)			\
				))

DT_INST_FOREACH_STATUS_OKAY(RTC_MCHP_DEVICE_INIT);
