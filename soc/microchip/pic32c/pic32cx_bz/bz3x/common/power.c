/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file power.c
 * @brief This file contains the Power Management (PM) hook function.
 */

/******************************************************************************
 * @brief Included Files
 *****************************************************************************/
#include <zephyr/kernel.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/pm/pm.h>
#include <zephyr/drivers/counter.h>
#include <soc.h>
#include <pmu_system.h>
#if defined(CONFIG_BT)
#include <bt_sys.h>
#endif
#include "device_sleep.h"

/******************************************************************************
 * @brief Macro definitions
 *****************************************************************************/
BUILD_ASSERT(DT_SAME_NODE(DT_CHOSEN(zephyr_cortex_m_idle_timer), DT_NODELABEL(rtc)),
	     "Pic32 CX BZ series needs RTC as an additional IDLE timer for power management");

#if !defined(CONFIG_BT)
#error "Pic32 CX BZ series needs BT to be enabled for power management"
#endif

#if !DT_HAS_CHOSEN(zephyr_bt_hci)
#error "Pic32 CX BZ series needs DT chosen property for HCI to enable power management"
#endif

#if defined(CONFIG_CORTEX_M_SYSTICK_LPM_TIMER_COUNTER)
static const struct device *idle_timer = DEVICE_DT_GET(DT_CHOSEN(zephyr_cortex_m_idle_timer));
#endif

/******************************************************************************
 * @brief Global Variables
 *****************************************************************************/
static PMU_Mode_T pmu_mode;

#ifdef CONFIG_HPA_SUPPORTED
/* Bypass pin control for HPA module */
#define HPA_BYPASS_SET_LOW() (GPIOB_REGS->GPIO_LATCLR = (1U << 1U))
#endif

/******************************************************************************
 * @brief Functions
 *****************************************************************************/
void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:

#if defined(CONFIG_CORTEX_M_SYSTICK_LPM_TIMER_COUNTER)
		counter_start(idle_timer);

		uint32_t idle_timer_count;
		uint32_t idle_timer_freq;

		if (counter_get_value(idle_timer, &idle_timer_count) != 0) {
			break;
		}
		idle_timer_freq = counter_get_frequency(idle_timer);
		if (BT_SYS_AllowSystemSleep(idle_timer_freq, idle_timer_count) == false) {
			break;
		}
#endif

		/* Back up PMU mode */
		pmu_mode = PMU_Get_Mode();

		/* Set PMU as BUCK PSM mode if it's not in MLDO mode.
		 * If it's in MLDO mode, do not perform mode switch to PSM.
		 */
		if (pmu_mode != PMU_MODE_MLDO) {
			PMU_Set_Mode(PMU_MODE_BUCK_PSM);

			/* Disable current sensor to improve current consumption. */
			PMU_ConfigCurrentSensor(false);
		}
#ifdef CONFIG_HPA_SUPPORTED
		/* Pull low bypass pin for HPA module  */
		HPA_BYPASS_SET_LOW();
#endif

		device_enter_sleep_mode();

		k_cpu_idle();

		break;

	default:
		break;
	}
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:

		if (BT_SYS_GetSleepMode() == false) {
			break;
		}

		/* Exit system sleep mode */
		device_exit_sleep_mode();

		/* Enable current sensor */
		PMU_ConfigCurrentSensor(true);

		/* Restore PMU mode */
		PMU_Set_Mode(pmu_mode);

		__set_PRIMASK(0);
		barrier_dsync_fence_full(); /* __DSB(); */
		barrier_isync_fence_full(); /* __ISB(); */

		break;
	default:
		break;
	}

	irq_unlock(0);
}
