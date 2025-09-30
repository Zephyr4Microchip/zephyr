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

static void clock_rf_Write_reg(uint8_t addr, uint16_t value)
{
	BLE_REGS->BLE_SPI_REG_ADDR = addr;
	BLE_REGS->BLE_SPI_WR_DATA = value;
	BLE_REGS->BLE_RFPWRMGMT |= 0x00100000U;
	while ((BLE_REGS->BLE_RFPWRMGMT & 0x00100000U) != 0U) {
		/* Do nothing */
	}
}

static void clock_initialize(void)
{
	/* check CLDO ready */
	while ((CFG_REGS->CFG_MISCSTAT & CFG_MISCSTAT_CLDORDY_Msk) == 0U) {
		/* Nothing to do */
	}
	clock_rf_Write_reg(0x27U, 0x2078U);

	/* Current Oscillator is 8MHz FRC or 16MHz POSC */
	if ((CRU_REGS->CRU_OSCCON & CRU_OSCCON_COSC_Msk) != CRU_OSCCON_COSC_SPLL) {
		/* Setup 128MHz PLL */
		clock_rf_Write_reg(0x2EU, 0x4328U);

		/* MISRAC 2012 deviation block start
		 * MISRA C-2012 Rule 11.1 deviated 1 time.
		 *     Deviation record ID -  H3_MISRAC_2012_R_11_1_DR_1
		 * Configure Prefetch, Wait States by calling the ROM function whose address is
		 *    available at address 0xF2D0
		 */
#if (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC == 64000000)
		typedef void (*FUNC_PCHE_SETUP)(uint32_t setup);
		(void)((FUNC_PCHE_SETUP)(*(uint32_t *)0xF2D0))(
			(PCHE_REGS->PCHE_CHECON & (~(PCHE_CHECON_PFMWS_Msk | PCHE_CHECON_ADRWS_Msk |
						     PCHE_CHECON_PREFEN_Msk))) |
			(PCHE_CHECON_PFMWS(1) | PCHE_CHECON_PREFEN(1) | PCHE_CHECON_ADRWS(1)));
#elif (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC == 128000000)
		typedef void (*FUNC_PCHE_SETUP)(uint32_t setup);
		(void)((FUNC_PCHE_SETUP)(*(uint32_t *)0xF2D0))(
			(PCHE_REGS->PCHE_CHECON & (~(PCHE_CHECON_PFMWS_Msk | PCHE_CHECON_ADRWS_Msk |
						     PCHE_CHECON_PREFEN_Msk))) |
			(PCHE_CHECON_PFMWS(4) | PCHE_CHECON_PREFEN(1) | PCHE_CHECON_ADRWS(1)));
#else
#error "SYS_CLOCK_HW_CYCLES_PER_SEC value is not supported"
#endif
	}
	/* programming 4ms delay -  programming subsys_xtal_ready_delay */
	/* check xtal spec for delay required */
	/* wait for crystal ready */
	while ((BTZBSYS_REGS->BTZBSYS_SUBSYS_STATUS_REG1 &
		BTZBSYS_SUBSYS_STATUS_REG1_xtal_ready_out_Msk) !=
	       BTZBSYS_SUBSYS_STATUS_REG1_xtal_ready_out_Msk) {
		/* Nothing to do */
	}
	CFG_REGS->CFG_SYSKEY = 0x00000000U; /* Write junk to lock it if it is already unlocked */
	CFG_REGS->CFG_SYSKEY = 0xAA996655U;
	CFG_REGS->CFG_SYSKEY = 0x556699AAU;
	CRU_REGS->CRU_OSCCON = 0x200U; /* switch to XO */

	/* Enable oscillator switch from COSC to NOSC */
	CRU_REGS->CRU_OSCCONSET = CRU_OSCCON_OSWEN_Msk;

	/* wait for successful clock change before continuing */
	while ((CRU_REGS->CRU_OSCCON & CRU_OSCCON_OSWEN_Msk) != 0U) {
		/* Nothing to do */
	}

	/* set PLL_disable */
	BLE_REGS->BLE_DPLL_RG2 |= 0x02U;

	/* set PLL_enable */
	BLE_REGS->BLE_DPLL_RG2 &= ((uint8_t)~(0x02U));

	/* Set MISC[24]=0, CLKGEN_PLLRST = 0 */
	CFG_REGS->CFG_MISCSTAT &= 0x00FFFFFFU;
	/* programming delay for pll lock - 500 us */
	/* 32 us steps - check pll spec for final value */
	BTZBSYS_REGS->BTZBSYS_SUBSYS_CNTRL_REG3 =
		((BTZBSYS_REGS->BTZBSYS_SUBSYS_CNTRL_REG3 &
		  ~BTZBSYS_SUBSYS_CNTRL_REG3_subsys_pll_ready_delay_Msk) |
		 ((0x02UL) << BTZBSYS_SUBSYS_CNTRL_REG3_subsys_pll_ready_delay_Pos));

	/* Unlock system for clock configuration */
	CFG_REGS->CFG_SYSKEY = 0x00000000U;
	CFG_REGS->CFG_SYSKEY = 0xAA996655U;
	CFG_REGS->CFG_SYSKEY = 0x556699AAU;

#if (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC == 64000000)
	/* SPLLPWDN     = 0x1     */
	/* SPLLFLOCK    = 0x0    */
	/* SPLLRST      = 0x0      */
	/* SPLLPOSTDIV1 = 2 */
	/* SPLLPOSTDIV2 = 0x1 */
	/* SPLL_BYP     = 0x3     */
	CRU_REGS->CRU_SPLLCON = 0xc0010208U;
#elif (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC == 128000000)
	/* SPLLPWDN     = 0x1     */
	/* SPLLFLOCK    = 0x0    */
	/* SPLLRST      = 0x0      */
	/* SPLLPOSTDIV1 = 1 */
	/* SPLLPOSTDIV2 = 0x1 */
	/* SPLL_BYP     = 0x3     */
	CRU_REGS->CRU_SPLLCON = 0xc0010108U;
#else
#error "SYS_CLOCK_HW_CYCLES_PER_SEC value is not supported"
#endif

	/* wait for PLL Lock */
	while ((BTZBSYS_REGS->BTZBSYS_SUBSYS_STATUS_REG1 & 0x03U) != 0x03U) {
		/* Nothing to do */
	}

	/* OSWEN    = SWITCH    */
	/* SOSCEN   = OFF   */
	/* CF       = NO_FAILDET       */
	/* SLPEN    = IDLE    */
	/* CLKLOCK  = UNLOCKED  */
	/* NOSC     = SPLL     */
	/* WAKE2SPD = FRC */
	/* DRMEN    = NO_EFFECT    */
	/* FRCDIV   = DIV_1   */
	CRU_REGS->CRU_OSCCON = 0x200101;

	CRU_REGS->CRU_OSCCONSET = CRU_OSCCON_OSWEN_Msk; /* request oscillator switch to occur */

	/* wait for indication of successful clock change before proceeding */
	while ((CRU_REGS->CRU_OSCCON & CRU_OSCCON_OSWEN_Msk) != 0U) {
		/* Nothing to do */
	}
	/* Peripheral Bus 3 is by default enabled, set its divisor */
	/* PBDIV = 10 */
	CRU_REGS->CRU_PB3DIV = CRU_PB3DIV_PBDIVON_Msk | CRU_PB3DIV_PBDIV(9U);

	/* Set up Reference Clock 1 */
	/* REFO1CON register */
	/* ROSEL =  SPLL1 */
	/* DIVSWEN = 1 */
	/* RSLP = false */
	/* SIDL = false */
	/* RODIV = 0 */
	CRU_REGS->CRU_REFO1CON = 0x201U;

	/* Enable oscillator (ON bit) */
	CRU_REGS->CRU_REFO1CONSET = 0x00008000U;

	/* Peripheral Clock Generators */
	CFG_REGS->CFG_CFGPCLKGEN1 = 0x0U;
	CFG_REGS->CFG_CFGPCLKGEN2 = 0x0U;
	CFG_REGS->CFG_CFGPCLKGEN3 = 0x0U;
	CFG_REGS->CFG_CFGPCLKGEN4 = 0x0U;

	/* Peripheral Module Disable Configuration */

	CFG_REGS->CFG_PMD1 = 0x200107e0U;
	CFG_REGS->CFG_PMD2 = 0x0U;
	CFG_REGS->CFG_PMD3 = 0xf73ffffU;

	/* Change src_clk source to PLL CLK */
	BTZBSYS_REGS->BTZBSYS_SUBSYS_CNTRL_REG1 |= 0x00000010U;

	/* set aclb_reset_n[24] */
	BTZBSYS_REGS->BTZBSYS_SUBSYS_CNTRL_REG0 |= 0x01000000U;

#ifdef CONFIG_BT
	/* set bt_en_main_clk[20] */
	BTZBSYS_REGS->BTZBSYS_SUBSYS_CNTRL_REG0 |= BTZBSYS_SUBSYS_CNTRL_REG0_bt_en_main_clk(1U);
#endif

	/* Lock system since done with clock configuration */
	CFG_REGS->CFG_SYSKEY = 0x33333333U;
}

void soc_early_init_hook(void)
{
#ifdef CONFIG_BT
	SYS_ClkGen_Config();
#endif

	clock_initialize();

	/* CM4 SW Reset Enabled */
	CFG_REGS->CFG_MISCSTAT |= 0x8000;

	/* For the case: ROM code runs as 128Mhz already,
	 * but application code runs a lower system clock
	 */
#if (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC == 64000000)
	typedef void (*FUNC_PCHE_SETUP)(uint32_t setup);
	(void)((FUNC_PCHE_SETUP)(*(uint32_t *)0xF2D0))(
		(PCHE_REGS->PCHE_CHECON &
		 (~(PCHE_CHECON_PFMWS_Msk | PCHE_CHECON_ADRWS_Msk | PCHE_CHECON_PREFEN_Msk))) |
		(PCHE_CHECON_PFMWS(1) | PCHE_CHECON_PREFEN(1) | PCHE_CHECON_ADRWS(1)));
#endif

#ifdef CONFIG_ENABLE_PMU_BUCK_MODE
	/* Set Power mode of the system */
	PMU_Set_Mode(PMU_MODE_BUCK_PWM);
#endif
}
