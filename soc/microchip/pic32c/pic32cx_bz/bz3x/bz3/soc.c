/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/kernel.h>

#ifdef CONFIG_BT
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

void soc_early_init_hook(void)
{
#ifdef CONFIG_BT
	SYS_ClkGen_Config();
#endif

	/* Peripheral Module Disable Configuration */
	CFG_REGS->CFG_PMD1 = 0x208103cfU;
	CFG_REGS->CFG_PMD3 = 0x7fffU;

	/* Configure Prefetch, Wait States by calling the ROM function whose address is available at
	 * address 0xF2D0
	 */
	typedef void (*FUNC_PCHE_SETUP)(uint32_t setup);
	(void)((FUNC_PCHE_SETUP)(*(uint32_t *)0xF2D0))(
		(PCHE_REGS->PCHE_CHECON &
		 (~(PCHE_CHECON_PFMWS_Msk | PCHE_CHECON_ADRWS_Msk | PCHE_CHECON_PREFEN_Msk))) |
		(PCHE_CHECON_PFMWS(2) | PCHE_CHECON_PREFEN(1) | PCHE_CHECON_ADRWS(0)));

	/* CM4 SW Reset Enabled */
	CFG_REGS->CFG_MISCSTAT |= 0x8000;
}
