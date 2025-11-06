/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#ifdef CONFIG_ENABLE_PMU_BUCK_MODE
#include <pmu_system.h>
#endif
#include <zephyr/kernel.h>

#if defined(CONFIG_BT) || defined(CONFIG_IEEE802154)
#include <rf_system.h>
#endif

#ifdef CONFIG_ARM_ON_ENTER_CPU_IDLE_PREPARE_HOOK
#ifdef CONFIG_BT
#include <bt_sys.h>
#endif
#endif

#define CLK_READY_RETRIES 8000U
#define BTZB_XTAL_NOT_READY                                                                        \
	((BTZBSYS_REGS->BTZBSYS_SUBSYS_STATUS_REG1 &                                               \
	  BTZBSYS_SUBSYS_STATUS_REG1_xtal_ready_out_Msk) !=                                        \
	 BTZBSYS_SUBSYS_STATUS_REG1_xtal_ready_out_Msk)
#define BTZB_PLL_NOT_LOCKED                                                                        \
	((BTZBSYS_REGS->BTZBSYS_SUBSYS_STATUS_REG1 &                                               \
	  BTZBSYS_SUBSYS_CNTRL_REG1_subsys_dbg_bus_sel_top_Msk) !=                                 \
	 BTZBSYS_SUBSYS_CNTRL_REG1_subsys_dbg_bus_sel_top_Msk)

#ifdef CONFIG_ARM_ON_ENTER_CPU_IDLE_PREPARE_HOOK
void z_arm_on_enter_cpu_idle_prepare(void)
{
	uint32_t key;
	bool rf_need_cal = RF_NeedCal();

#ifdef CONFIG_BT
	uint8_t bt_rf_suspended = 0;

	key = irq_lock();
	bt_rf_suspended = BT_SYS_RfSuspendReq(1);
	irq_unlock(key);

	if ((rf_need_cal == true) && (bt_rf_suspended == BT_SYS_RF_SUSPENDED_NO_SLEEP)) {
		RF_Timer_Cal(WSS_ENABLE_BLE);
	}
	BT_SYS_RfSuspendReq(0);
#endif
}
#endif

#if defined(CONFIG_BT) || defined(CONFIG_IEEE802154)
static __ramfunc void pche_setup(void)
{
	/* Set Flash Wait states and enable pre-fetch */
	/* clear PFMWS and ADRWS */
	PCHE_REGS->PCHE_CHECON =
		(PCHE_REGS->PCHE_CHECON &
		 (~(PCHE_CHECON_PFMWS_Msk | PCHE_CHECON_ADRWS_Msk | PCHE_CHECON_PREFEN_Msk))) |
		(PCHE_CHECON_PFMWS(1) | PCHE_CHECON_PREFEN(1));
	/* write completion delay */
	for (int i = 1; i < 10; i++) {
		__asm__ volatile("NOP");
	}
}
#endif

void soc_early_init_hook(void)
{
#if defined(CONFIG_BT) || defined(CONFIG_IEEE802154)
	SYS_ClkGen_Config();
	pche_setup();
#endif

	/* Peripheral Module Disable Configuration */
	CFG_REGS->CFG_PMD1 = 0x200101cfU;
	CFG_REGS->CFG_PMD3 = 0x7fffU;

	PCHE_REGS->PCHE_CHECON =
		(PCHE_REGS->PCHE_CHECON &
		 (~(PCHE_CHECON_PFMWS_Msk | PCHE_CHECON_ADRWS_Msk | PCHE_CHECON_PREFEN_Msk))) |
		(PCHE_CHECON_PFMWS(1) | PCHE_CHECON_PREFEN(1));

	/* CM4 SW Reset Enabled */
	CFG_REGS->CFG_MISCSTAT |= 0x8000;

#ifdef CONFIG_ENABLE_PMU_BUCK_MODE
	/* Set Power mode of the system */
	PMU_Set_Mode(PMU_MODE_BUCK_PWM);
#endif
}
