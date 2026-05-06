/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file device_sleep.c
 * @brief This file contains the Device Sleep functions.
 */

/******************************************************************************
 * @brief Included Files
 *****************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <soc.h>
#include "device_sleep.h"
#include "sleep_system.h"

/******************************************************************************
 * @brief Macro definitions
 *****************************************************************************/
#define CLK_READY_RETRIES 8000U
#define BTZB_XTAL_NOT_READY                                                                        \
	((BTZBSYS_REGS->BTZBSYS_SUBSYS_STATUS_REG1 &                                               \
	  BTZBSYS_SUBSYS_STATUS_REG1_xtal_ready_out_Msk) !=                                        \
	 BTZBSYS_SUBSYS_STATUS_REG1_xtal_ready_out_Msk)
#define BTZB_PLL_NOT_LOCKED                                                                        \
	((BTZBSYS_REGS->BTZBSYS_SUBSYS_STATUS_REG1 &                                               \
	  BTZBSYS_SUBSYS_CNTRL_REG1_subsys_dbg_bus_sel_top_Msk) !=                                 \
	 BTZBSYS_SUBSYS_CNTRL_REG1_subsys_dbg_bus_sel_top_Msk)

/* The action ID for enter/exit sleep. */
typedef enum {
	DEVICE_SLEEP_ENTER_SLEEP, /**< Enter sleep. */
	DEVICE_SLEEP_EXIT_SLEEP   /**< Exit sleep. */
} device_sleep_action_id_t;

/* The definition of clock source  */
typedef enum {
	DEVICE_NO_CLK = 0x00, /* No clock is selected. */
	DEVICE_CLK_REFO1,     /* REFO1 is selected. */
	DEVICE_CLK_REFO2,     /* REFO2 is selected. */
	DEVICE_CLK_REFO3,     /* REFO3 is selected. */
	DEVICE_CLK_REFO4,     /* REFO4 is selected. */
	DEVICE_CLK_REFO5,     /* REFO5 is selected. */
	DEVICE_CLK_REFO6,     /* REFO6 is selected. */
	DEVICE_CLK_LPCLK,     /* Low power clock is selected. */
	DEVICE_CLK_END
} device_clk_src_id_t;

/* Peripheral enable/disable settings of PMD3. */
typedef struct {
	unsigned int sercom1: 1;   /* SERCOM 1. */
	unsigned int sercom2: 1;   /* SERCOM 2. */
	unsigned int sercom3: 1;   /* SERCOM 3. */
	unsigned int sercom4: 1;   /* SERCOM 4. */
	unsigned int tc0: 1;       /* TC 0. */
	unsigned int tc1: 1;       /* TC 1. */
	unsigned int tc2: 1;       /* TC 2. */
	unsigned int tc3: 1;       /* TC 3. */
	unsigned int tcc0: 1;      /* TCC 0. */
	unsigned int tcc1: 1;      /* TCC 1. */
	unsigned int tcc2: 1;      /* TCC 2. */
	unsigned int reserved: 21; /* Reserved. */
} device_pmd3_reg_t;

struct device_reg_backup {
	uint32_t pmd1; /* PMD1 register backup */
	uint32_t pmd2; /* PMD2 register backup */
	uint32_t pmd3; /* PMD3 register backup */

	uint32_t refo1; /* REFO1 register backup */
	uint32_t refo2; /* REFO2 register backup */
	uint32_t refo3; /* REFO3 register backup */
	uint32_t refo4; /* REFO4 register backup */
	uint32_t refo5; /* REFO5 register backup */
	uint32_t refo6; /* REFO6 register backup */

	uint32_t pche;    /* Prefetch Cache Module Control register backup */
	uint32_t pb1_div; /* PB1 Clock Divisor Control register backup */
};

/******************************************************************************
 * @brief Global Variables
 *****************************************************************************/
static struct device_reg_backup bk_reg;

/******************************************************************************
 * @brief Functions
 *****************************************************************************/
static void device_delay(uint32_t value)
{
	uint32_t i;

	for (i = 0; i < value; i++) {
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
	}
}

/* Unlock system for clock configuration */
static void device_sys_unlock(void)
{
	CFG_REGS->CFG_SYSKEY = 0x00000000U;
	CFG_REGS->CFG_SYSKEY = 0xAA996655U;
	CFG_REGS->CFG_SYSKEY = 0x556699AAU;
}

static void device_sys_lock(void)
{
	CFG_REGS->CFG_SYSKEY = 0x33333333U;
}

/* Configure Reference Oscillator Control Register */
static void device_config_ref_osc_reg(device_sleep_action_id_t action)
{
	if (action == DEVICE_SLEEP_ENTER_SLEEP) {
		bk_reg.refo1 = 0;
		bk_reg.refo2 = 0;
		bk_reg.refo3 = 0;
		bk_reg.refo4 = 0;
		bk_reg.refo5 = 0;
		bk_reg.refo6 = 0;

		/* REFO1CON */
		if (((CRU_REGS->CRU_REFO1CON & CRU_REFO1CON_ON_Msk) != 0U) &&
		    ((CRU_REGS->CRU_REFO1CON & CRU_REFO1CON_RSLP_Msk) == 0U)) {
			/* Disable REFOxCON in sleep mode */

			/* Backup REFOxCON */
			bk_reg.refo1 = CRU_REGS->CRU_REFO1CON;

			/* Can't update REFOxCON register if REFOxCON.ACTIVE != REFOxCON.ON */
			while ((CRU_REGS->CRU_REFO1CON & CRU_REFO1CON_ACTIVE_Msk) == 0U) {
				/* Do Nothing */
			}
			CRU_REGS->CRU_REFO1CONCLR = CRU_REFO1CON_ON_Msk; /* Disable REFOxCON */

			while ((CRU_REGS->CRU_REFO1CON & CRU_REFO1CON_ACTIVE_Msk) != 0U) {
				/* Do Nothing */
			}
			CRU_REGS->CRU_REFO1CONCLR =
				0xFFFFFEFFU; /* Clear REFOxCON (Bit 8 is read only) */
		}

		/* REFO2CON */
		if (((CRU_REGS->CRU_REFO2CON & CRU_REFO2CON_ON_Msk) != 0U) &&
		    ((CRU_REGS->CRU_REFO2CON & CRU_REFO2CON_RSLP_Msk) == 0U)) {
			/* Disable REFOxCON in sleep mode */

			/* Backup REFOxCON */
			bk_reg.refo2 = CRU_REGS->CRU_REFO2CON;

			/* Can't update REFOxCON register if REFOxCON.ACTIVE != REFOxCON.ON */
			while ((CRU_REGS->CRU_REFO2CON & CRU_REFO2CON_ACTIVE_Msk) == 0U) {
				/* Do Nothing */
			}
			CRU_REGS->CRU_REFO2CONCLR = CRU_REFO2CON_ON_Msk; /* Disable REFOxCON */

			while ((CRU_REGS->CRU_REFO2CON & CRU_REFO2CON_ACTIVE_Msk) != 0U) {
				/* Do Nothing */
			}
			CRU_REGS->CRU_REFO2CONCLR =
				0xFFFFFEFFU; /* Clear REFOxCON (Bit 8 is read only) */
		}

		/* REFO3CON */
		if (((CRU_REGS->CRU_REFO3CON & CRU_REFO3CON_ON_Msk) != 0U) &&
		    ((CRU_REGS->CRU_REFO3CON & CRU_REFO3CON_RSLP_Msk) == 0U)) {
			/* Disable REFOxCON in sleep mode */

			/* Backup REFOxCON */
			bk_reg.refo3 = CRU_REGS->CRU_REFO3CON;

			/* Can't update REFOxCON register if REFOxCON.ACTIVE != REFOxCON.ON */
			while ((CRU_REGS->CRU_REFO3CON & CRU_REFO3CON_ACTIVE_Msk) == 0U) {
				/* Do Nothing */
			}
			CRU_REGS->CRU_REFO3CONCLR = CRU_REFO3CON_ON_Msk; /* Disable REFOxCON */

			while ((CRU_REGS->CRU_REFO3CON & CRU_REFO3CON_ACTIVE_Msk) != 0U) {
				/* Do Nothing */
			}
			CRU_REGS->CRU_REFO3CONCLR =
				0xFFFFFEFFU; /* Clear REFOxCON (Bit 8 is read only) */
		}

		/* REFO4CON */
		if (((CRU_REGS->CRU_REFO4CON & CRU_REFO4CON_ON_Msk) != 0U) &&
		    ((CRU_REGS->CRU_REFO4CON & CRU_REFO4CON_RSLP_Msk) == 0U)) {
			/* Disable REFOxCON in sleep mode */

			/* Backup REFOxCON */
			bk_reg.refo4 = CRU_REGS->CRU_REFO4CON;

			/* Can't update REFOxCON register if REFOxCON.ACTIVE != REFOxCON.ON */
			while ((CRU_REGS->CRU_REFO4CON & CRU_REFO4CON_ACTIVE_Msk) == 0U) {
				/* Do Nothing */
			}
			CRU_REGS->CRU_REFO4CONCLR = CRU_REFO4CON_ON_Msk; /* Disable REFOxCON */

			while ((CRU_REGS->CRU_REFO4CON & CRU_REFO4CON_ACTIVE_Msk) != 0U) {
				/* Do Nothing */
			}
			CRU_REGS->CRU_REFO4CONCLR =
				0xFFFFFEFFU; /* Clear REFOxCON (Bit 8 is read only) */
		}

		/* REFO5CON */
		if (((CRU_REGS->CRU_REFO5CON & CRU_REFO5CON_ON_Msk) != 0U) &&
		    ((CRU_REGS->CRU_REFO5CON & CRU_REFO5CON_RSLP_Msk) == 0U)) {
			/* Disable REFOxCON in sleep mode */

			/* Backup REFOxCON */
			bk_reg.refo5 = CRU_REGS->CRU_REFO5CON;

			/* Can't update REFOxCON register if REFOxCON.ACTIVE != REFOxCON.ON */
			while ((CRU_REGS->CRU_REFO5CON & CRU_REFO5CON_ACTIVE_Msk) == 0U) {
				/* Do Nothing */
			}
			CRU_REGS->CRU_REFO5CONCLR = CRU_REFO5CON_ON_Msk; /* Disable REFOxCON */

			while ((CRU_REGS->CRU_REFO5CON & CRU_REFO5CON_ACTIVE_Msk) != 0U) {
				/* Do Nothing */
			}
			CRU_REGS->CRU_REFO5CONCLR =
				0xFFFFFEFFU; /* Clear REFOxCON (Bit 8 is read only) */
		}

		/* REFO6CON */
		if (((CRU_REGS->CRU_REFO6CON & CRU_REFO6CON_ON_Msk) != 0U) &&
		    ((CRU_REGS->CRU_REFO6CON & CRU_REFO6CON_RSLP_Msk) == 0U)) {
			/* Disable REFOxCON in sleep mode */

			/* Backup REFOxCON */
			bk_reg.refo6 = CRU_REGS->CRU_REFO6CON;

			/* Can't update REFOxCON register if REFOxCON.ACTIVE != REFOxCON.ON */
			while ((CRU_REGS->CRU_REFO6CON & CRU_REFO6CON_ACTIVE_Msk) == 0U) {
				/* Do Nothing */
			}
			CRU_REGS->CRU_REFO6CONCLR = CRU_REFO6CON_ON_Msk; /* Disable REFOxCON */

			while ((CRU_REGS->CRU_REFO6CON & CRU_REFO6CON_ACTIVE_Msk) != 0U) {
				/* Do Nothing */
			}
			CRU_REGS->CRU_REFO6CONCLR =
				0xFFFFFEFFU; /* Clear REFOxCON (Bit 8 is read only) */
		}
	} else {
		/* Check if it needs to restore REFOx */
		/* REFO1CON */
		if (((bk_reg.refo1 & CRU_REFO1CON_ON_Msk) != 0U) &&
		    ((bk_reg.refo1 & CRU_REFO1CON_RSLP_Msk) == 0U)) {
			/* Exclude output enable and active(read only) bit */
			bk_reg.refo1 &= ~(CRU_REFO1CON_ON_Msk | CRU_REFO1CON_ACTIVE_Msk);

			/* Restore REFOx setting */
			CRU_REGS->CRU_REFO1CON = bk_reg.refo1;

			/* Enable oscillator (ON bit) */
			CRU_REGS->CRU_REFO1CONSET = CRU_REFO1CON_ON_Msk;
		}

		/* REFO2CON */
		if (((bk_reg.refo2 & CRU_REFO2CON_ON_Msk) != 0U) &&
		    ((bk_reg.refo2 & CRU_REFO2CON_RSLP_Msk) == 0U)) {
			/* Exclude output enable and active(read only) bit */
			bk_reg.refo2 &= ~(CRU_REFO2CON_ON_Msk | CRU_REFO2CON_ACTIVE_Msk);

			/* Restore REFOx setting */
			CRU_REGS->CRU_REFO2CON = bk_reg.refo2;

			/* Enable oscillator (ON bit) */
			CRU_REGS->CRU_REFO2CONSET = CRU_REFO2CON_ON_Msk;
		}

		/* REFO3CON */
		if (((bk_reg.refo3 & CRU_REFO3CON_ON_Msk) != 0U) &&
		    ((bk_reg.refo3 & CRU_REFO3CON_RSLP_Msk) == 0U)) {
			/* Exclude output enable and active(read only) bit */
			bk_reg.refo3 &= ~(CRU_REFO3CON_ON_Msk | CRU_REFO3CON_ACTIVE_Msk);

			/* Restore REFOx setting */
			CRU_REGS->CRU_REFO3CON = bk_reg.refo3;

			/* Enable oscillator (ON bit) */
			CRU_REGS->CRU_REFO3CONSET = CRU_REFO3CON_ON_Msk;
		}

		/* REFO4CON */
		if (((bk_reg.refo4 & CRU_REFO4CON_ON_Msk) != 0U) &&
		    ((bk_reg.refo4 & CRU_REFO4CON_RSLP_Msk) == 0U)) {
			/* Exclude output enable and active(read only) bit */
			bk_reg.refo4 &= ~(CRU_REFO4CON_ON_Msk | CRU_REFO4CON_ACTIVE_Msk);

			/* Restore REFOx setting */
			CRU_REGS->CRU_REFO4CON = bk_reg.refo4;

			/* Enable oscillator (ON bit) */
			CRU_REGS->CRU_REFO4CONSET = CRU_REFO4CON_ON_Msk;
		}

		/* REFO5CON */
		if (((bk_reg.refo5 & CRU_REFO5CON_ON_Msk) != 0U) &&
		    ((bk_reg.refo5 & CRU_REFO5CON_RSLP_Msk) == 0U)) {
			/* Exclude output enable and active(read only) bit */
			bk_reg.refo5 &= ~(CRU_REFO5CON_ON_Msk | CRU_REFO5CON_ACTIVE_Msk);

			/* Restore REFOx setting */
			CRU_REGS->CRU_REFO5CON = bk_reg.refo5;

			/* Enable oscillator (ON bit) */
			CRU_REGS->CRU_REFO5CONSET = CRU_REFO5CON_ON_Msk;
		}

		/* REFO6CON */
		if (((bk_reg.refo6 & CRU_REFO6CON_ON_Msk) != 0U) &&
		    ((bk_reg.refo6 & CRU_REFO6CON_RSLP_Msk) == 0U)) {
			/* Exclude output enable and active(read only) bit */
			bk_reg.refo6 &= ~(CRU_REFO6CON_ON_Msk | CRU_REFO6CON_ACTIVE_Msk);

			/* Restore REFOx setting */
			CRU_REGS->CRU_REFO6CON = bk_reg.refo6;

			/* Enable oscillator (ON bit) */
			CRU_REGS->CRU_REFO6CONSET = CRU_REFO6CON_ON_Msk;
		}
	}
}

/* Check the if peripheral can be keep running in sleep mode. Return true means it can be keep
 * running.
 */
static bool device_chk_peripheral(device_clk_src_id_t select)
{
	uint32_t pmd2_val;

	pmd2_val = CFG_REGS->CFG_PMD2;

	if ((select == DEVICE_CLK_REFO1) &&
	    ((pmd2_val & CFG_PMD2_REFO1MD_Msk) == 0U)) { /* REFO1 is not disabled in sleep mode */
		return true;
	} else if ((select == DEVICE_CLK_REFO2) &&
		   ((pmd2_val & CFG_PMD2_REFO2MD_Msk) == 0U)) { /* REFO2 is not disabled */
		return true;
	} else if ((select == DEVICE_CLK_REFO3) &&
		   ((pmd2_val & CFG_PMD2_REFO3MD_Msk) == 0U)) { /* REFO3 is not disabled */
		return true;
	} else if ((select == DEVICE_CLK_REFO4) &&
		   ((pmd2_val & CFG_PMD2_REFO4MD_Msk) == 0U)) { /* REFO4 is not disabled */
		return true;
	} else if ((select == DEVICE_CLK_REFO5) &&
		   ((pmd2_val & CFG_PMD2_REFO5MD_Msk) == 0U)) { /* REFO5 is not disabled */
		return true;
	} else if ((select == DEVICE_CLK_REFO6) &&
		   ((pmd2_val & CFG_PMD2_REFO6MD_Msk) == 0U)) { /* REFO6 is not disabled */
		return true;
	} else if (select == DEVICE_CLK_LPCLK) { /* LPCLK is used */
		return true;
	} else {
		return false;
	}
}

/* Configure PMD Register */
static void device_config_pmd_reg(device_sleep_action_id_t action)
{
	if (action == DEVICE_SLEEP_ENTER_SLEEP) {

		device_pmd3_reg_t pmd_reg;
		device_clk_src_id_t select;
		uint32_t pmd2_val, pmd3_val;
		uint8_t temp_val;

		(void)memset((uint8_t *)&pmd_reg, 0, sizeof(device_pmd3_reg_t));

		/* For PMD1, disable all PMD except RTC */
		CFG_REGS->CFG_PMD1 = 0xFFFEFFFFU; /* bit 16: RTC */

		/* For PMD2 */
		/* Check if RSLP bit is set, do not disable this REFOx */
		pmd2_val = 0xFFFFFFFFU;

		if (((CRU_REGS->CRU_REFO1CON & CRU_REFO1CON_ON_Msk) != 0U) &&
		    ((CRU_REGS->CRU_REFO1CON & CRU_REFO1CON_RSLP_Msk) != 0U)) {
			pmd2_val &= ~CFG_PMD2_REFO1MD_Msk;
		}

		if (((CRU_REGS->CRU_REFO2CON & CRU_REFO2CON_ON_Msk) != 0U) &&
		    ((CRU_REGS->CRU_REFO2CON & CRU_REFO2CON_RSLP_Msk) != 0U)) {
			pmd2_val &= ~CFG_PMD2_REFO2MD_Msk;
		}

		if (((CRU_REGS->CRU_REFO3CON & CRU_REFO3CON_ON_Msk) != 0U) &&
		    ((CRU_REGS->CRU_REFO3CON & CRU_REFO3CON_RSLP_Msk) != 0U)) {
			pmd2_val &= ~CFG_PMD2_REFO3MD_Msk;
		}

		if (((CRU_REGS->CRU_REFO4CON & CRU_REFO4CON_ON_Msk) != 0U) &&
		    ((CRU_REGS->CRU_REFO4CON & CRU_REFO4CON_RSLP_Msk) != 0U)) {
			pmd2_val &= ~CFG_PMD2_REFO4MD_Msk;
		}

		if (((CRU_REGS->CRU_REFO5CON & CRU_REFO5CON_ON_Msk) != 0U) &&
		    ((CRU_REGS->CRU_REFO5CON & CRU_REFO5CON_RSLP_Msk) != 0U)) {
			pmd2_val &= ~CFG_PMD2_REFO5MD_Msk;
		}

		if (((CRU_REGS->CRU_REFO6CON & CRU_REFO6CON_ON_Msk) != 0U) &&
		    ((CRU_REGS->CRU_REFO6CON & CRU_REFO6CON_RSLP_Msk) != 0U)) {
			pmd2_val &= ~CFG_PMD2_REFO6MD_Msk;
		}

		CFG_REGS->CFG_PMD2 = pmd2_val;

		/*
		 * For PMD3, check if the peripheral is enabled
		 * bit 0~3 of PMD3 for SERCOM 0~3
		 * bit 8~11 of PMD3 for TC0~3
		 * bit 12~14 of PMD3 for TCC0~TCC2
		 */

		pmd3_val = 0xFFFF;

		if ((CFG_REGS->CFG_PMD3 & CFG_PMD3_SER1MD_Msk) == 0U) {
			pmd_reg.sercom1 = 1;
		}

		if ((CFG_REGS->CFG_PMD3 & CFG_PMD3_SER2MD_Msk) == 0U) {
			pmd_reg.sercom2 = 1;
		}

#ifdef CONFIG_SOC_SERIES_PIC32CX_BZ25
		if ((CFG_REGS->CFG_PMD3 & CFG_PMD3_SER3MD_Msk) == 0U) {
			pmd_reg.sercom3 = 1;
		}

		if ((CFG_REGS->CFG_PMD3 & CFG_PMD3_SER4MD_Msk) == 0U) {
			pmd_reg.sercom4 = 1;
		}
#endif

		if ((CFG_REGS->CFG_PMD3 & CFG_PMD3_TC0MD_Msk) == 0U) {
			pmd_reg.tc0 = 1;
		}

		if ((CFG_REGS->CFG_PMD3 & CFG_PMD3_TC1MD_Msk) == 0U) {
			pmd_reg.tc1 = 1;
		}

		if ((CFG_REGS->CFG_PMD3 & CFG_PMD3_TC2MD_Msk) == 0U) {
			pmd_reg.tc2 = 1;
		}

		if ((CFG_REGS->CFG_PMD3 & CFG_PMD3_TC3MD_Msk) == 0U) {
			pmd_reg.tc3 = 1;
		}

		if ((CFG_REGS->CFG_PMD3 & CFG_PMD3_TCC0MD_Msk) == 0U) {
			pmd_reg.tcc0 = 1;
		}

		if ((CFG_REGS->CFG_PMD3 & CFG_PMD3_TCC1MD_Msk) == 0U) {
			pmd_reg.tcc1 = 1;
		}

		if ((CFG_REGS->CFG_PMD3 & CFG_PMD3_TCC2MD_Msk) == 0U) {
			pmd_reg.tcc2 = 1;
		}

		/*
		 * Check CFGPCLKGEN1~3 to check if the peripheral clock is enabled and check its
		 * clock source Do not turn the peripheral off if RSLP is set (s_refoxBackup = 0) or
		 * CLK SRC is set as LP CLK
		 */
		if ((pmd_reg.sercom1 != 0U) || (pmd_reg.sercom2 != 0U)) {
			/* Check CFGCLKGEN1 bit 15,
			 * if enabled, check the clock source by bit 12~14
			 */
			if ((CFG_REGS->CFG_CFGPCLKGEN1 & CFG_CFGPCLKGEN1_S01CD_Msk) != 0U) {
				temp_val = (uint8_t)((CFG_REGS->CFG_CFGPCLKGEN1 &
						      CFG_CFGPCLKGEN1_SERCOM01CSEL_Msk) >>
						     CFG_CFGPCLKGEN1_SERCOM01CSEL_Pos);
				select = (device_clk_src_id_t)temp_val;

				if (device_chk_peripheral(select)) {
					if (pmd_reg.sercom1 != 0U) {
						pmd3_val &= ~CFG_PMD3_SER1MD_Msk;
					}

					if (pmd_reg.sercom2 != 0U) {
						pmd3_val &= ~CFG_PMD3_SER2MD_Msk;
					}
				}
			}
		}

#ifdef CONFIG_SOC_SERIES_PIC32CX_BZ25
		if ((pmd_reg.sercom3 != 0U) || (pmd_reg.sercom4 != 0U)) {
			/* Check CFGCLKGEN1 bit 19,
			 * if enabled, check the clock source by bit 16~18
			 */
			if ((CFG_REGS->CFG_CFGPCLKGEN1 & CFG_CFGPCLKGEN1_S23CD_Msk) != 0U) {
				temp_val = (uint8_t)((CFG_REGS->CFG_CFGPCLKGEN1 &
						      CFG_CFGPCLKGEN1_SERCOM23CSEL_Msk) >>
						     CFG_CFGPCLKGEN1_SERCOM23CSEL_Pos);
				select = (device_clk_src_id_t)temp_val;

				if (device_chk_peripheral(select)) {
					if (pmd_reg.sercom3 != 0U) {
						pmd3_val &= ~CFG_PMD3_SER3MD_Msk;
					}

					if (pmd_reg.sercom4 != 0U) {
						pmd3_val &= ~CFG_PMD3_SER4MD_Msk;
					}
				}
			}
		}
#endif

		if (pmd_reg.tc0 != 0U) {
			/* Check CFGCLKGEN3 bit 27, if enabled,
			 * check the clock source by bit 24~26
			 */
			if ((CFG_REGS->CFG_CFGPCLKGEN3 & CFG_CFGPCLKGEN3_TC0CD_Msk) != 0U) {
				temp_val = (uint8_t)((CFG_REGS->CFG_CFGPCLKGEN3 &
						      CFG_CFGPCLKGEN3_TC0CSEL_Msk) >>
						     CFG_CFGPCLKGEN3_TC0CSEL_Pos);
				select = (device_clk_src_id_t)temp_val;

				if (device_chk_peripheral(select)) {
					pmd3_val &= ~CFG_PMD3_TC0MD_Msk;
				}
			}
		}

		if (pmd_reg.tc1 != 0U) {
			/* Check CFGCLKGEN3 bit 31, if enabled,
			 * check the clock source by bit 28~30
			 */
			if ((CFG_REGS->CFG_CFGPCLKGEN3 & CFG_CFGPCLKGEN3_TC1CD_Msk) != 0U) {
				temp_val = (uint8_t)((CFG_REGS->CFG_CFGPCLKGEN3 &
						      CFG_CFGPCLKGEN3_TC1CSEL_Msk) >>
						     CFG_CFGPCLKGEN3_TC1CSEL_Pos);
				select = (device_clk_src_id_t)temp_val;

				if (device_chk_peripheral(select)) {
					pmd3_val &= ~CFG_PMD3_TC1MD_Msk;
				}
			}
		}

		if ((pmd_reg.tc2 != 0U) || (pmd_reg.tc3 != 0U)) {
			/* Check CFGCLKGEN1 bit 27, if enabled,
			 * check the clock source by bit 24~26
			 */
			if ((CFG_REGS->CFG_CFGPCLKGEN1 & CFG_CFGPCLKGEN1_TC23CD_Msk) != 0U) {
				temp_val = (uint8_t)((CFG_REGS->CFG_CFGPCLKGEN1 &
						      CFG_CFGPCLKGEN1_TC23CSEL_Msk) >>
						     CFG_CFGPCLKGEN1_TC23CSEL_Pos);
				select = (device_clk_src_id_t)temp_val;

				if (device_chk_peripheral(select)) {
					if (pmd_reg.tc2 != 0U) {
						pmd3_val &= ~CFG_PMD3_TC2MD_Msk;
					}

					if (pmd_reg.tc3 != 0U) {
						pmd3_val &= ~CFG_PMD3_TC3MD_Msk;
					}
				}
			}
		}

		if (pmd_reg.tcc0 != 0U) {
			/* Check CFGCLKGEN3 bit 23, if enabled,
			 * check the clock source by bit 20~22
			 */
			if ((CFG_REGS->CFG_CFGPCLKGEN3 & CFG_CFGPCLKGEN3_TCC0CD_Msk) != 0U) {
				temp_val = (uint8_t)((CFG_REGS->CFG_CFGPCLKGEN3 &
						      CFG_CFGPCLKGEN3_TCC0CSEL_Msk) >>
						     CFG_CFGPCLKGEN3_TCC0CSEL_Pos);
				select = (device_clk_src_id_t)temp_val;

				if (device_chk_peripheral(select)) {
					pmd3_val &= ~CFG_PMD3_TCC0MD_Msk;
				}
			}
		}

		if ((pmd_reg.tcc1 != 0U) || (pmd_reg.tcc2 != 0U)) {
			/* Check CFGCLKGEN1 bit 23, if enabled,
			 * check the clock source by bit 20~22
			 */
			if ((CFG_REGS->CFG_CFGPCLKGEN1 & CFG_CFGPCLKGEN1_TCC12CD_Msk) != 0U) {
				temp_val = (uint8_t)((CFG_REGS->CFG_CFGPCLKGEN1 &
						      CFG_CFGPCLKGEN1_TCC12CSEL_Msk) >>
						     CFG_CFGPCLKGEN1_TCC12CSEL_Pos);
				select = (device_clk_src_id_t)temp_val;

				if (device_chk_peripheral(select)) {
					if (pmd_reg.tcc1 != 0U) {
						pmd3_val &= ~CFG_PMD3_TCC1MD_Msk;
					}

					if (pmd_reg.tcc2 != 0U) {
						pmd3_val &= ~CFG_PMD3_TCC2MD_Msk;
					}
				}
			}
		}

		CFG_REGS->CFG_PMD3 = pmd3_val;

	} else {
		CFG_REGS->CFG_PMD1 = bk_reg.pmd1;
		CFG_REGS->CFG_PMD2 = bk_reg.pmd2;
		CFG_REGS->CFG_PMD3 = bk_reg.pmd3;
	}
}

void device_enter_sleep_mode(void)
{
	/* unlock key sequence */
	device_sys_unlock();

	/* Step 1 : For connected sleep case, if SOSC clock is available on the module, set 32K CLK
	 * source to SOSC CLK in CRU and set MLPCLK_MOD to 1 to divide 32.768kHz clock to 32kHz
	 * clock to save power. Otherwise, set 32K CLK source to POSC CLK in CRU to ensure accuracy
	 * of 32K CLK
	 *          For unconnected sleep case, set 32K CLK source to LPRC CLK in CRU to save power
	 *          since accuracy requirement does not need to be met
	 * step 1 has moved to CLK_Initialize()
	 * Some steps are executed within library
	 */

	/* Step 6 : Disable bt_zb_dbg bus toggling */
	DEVICE_SLEEP_DisableDebugBus();

	/* Step 14 : Disable PCHE Cache, which is proposed by SOC team for low power optimization */
	bk_reg.pche = PCHE_REGS->PCHE_CHECON;
	PCHE_REGS->PCHE_CHECON = 0xf;

	/* Step 15 : Set PB1 CLK to SYS_CLK/5, which is proposed by SOC team for low power */
	/* optimization */
	bk_reg.pb1_div = CRU_REGS->CRU_PB1DIV;
	CRU_REGS->CRU_PB1DIV = 0x8804;

	/* Step 16 : set REFOx registers to 0, combining step 17 to de-assert external PLL request
	 * since we don't know how and when plib_clock will be changed. so we will backup all 6 sets
	 * REFOx registers. Ensure the Reference Clock Out Module is enabled, REFO1 Clock is
	 * selected by CFG_CFGPCLKGEN1
	 */

	/* Check PMD lock bit */
	if ((CFG_REGS->CFG_CFGCON0 & CFG_CFGCON0_PMDLOCK_Msk) != 0U) {
		/* PMD lock is enabled, check CFGCLOCK */
		if ((CFG_REGS->CFG_CFGCON0 & CFG_CFGCON0_CFGCLOCK_Msk) == 0U) {
			/* Disable PMD lock */
			CFG_REGS->CFG_CFGCON0 &= ~CFG_CFGCON0_PMDLOCK_Msk;
		}
	}

	if ((CFG_REGS->CFG_PMD2 & CFG_PMD2_REFO1MD_Msk) != 0U) {
		/* Enable PMD2_REFO1 */
		CFG_REGS->CFG_PMD2 &= ~CFG_PMD2_REFO1MD_Msk;
	}

	/* Store all PMD setting */
	bk_reg.pmd1 = CFG_REGS->CFG_PMD1;
	bk_reg.pmd2 = CFG_REGS->CFG_PMD2;
	bk_reg.pmd3 = CFG_REGS->CFG_PMD3;

	/* Backup and Configure REFOxCON register */
	device_config_ref_osc_reg(DEVICE_SLEEP_ENTER_SLEEP);

	/* Configure PMD register */
	device_config_pmd_reg(DEVICE_SLEEP_ENTER_SLEEP);

	/* Step 17 : Change SYS CLK source in CRU from SPLL1 CLK to POSC CLK */
	CRU_REGS->CRU_OSCCON &= ~((uint32_t)0xf01U);
	CRU_REGS->CRU_OSCCON |= (uint32_t)0x200U;

	/* Request oscillator switch to occur */
	CRU_REGS->CRU_OSCCONSET = CRU_OSCCON_OSWEN_Msk;

	/* Wait for indication of successful clock change before proceeding */
	while ((CRU_REGS->CRU_OSCCON & CRU_OSCCON_OSWEN_Msk) != 0U) {
		/* Do nothing */
	}

	/* Step 18 : Set subsys_clk_src_sel to 0 to
	 * change subsys clock source from PLL CLK to XTAL
	 */
	/* CLK */
	BTZBSYS_REGS->BTZBSYS_SUBSYS_CNTRL_REG1 &= ~((uint32_t)0x30U);

	/* Step 19 :Disable CLKGEN Clock enable in RF */
	DEVICE_SLEEP_ConfigRfClk(false);

	/* Step 19.1 : If XTAL clock is OFF, set subsys_bypass_pll_lock to 0 via subsys config */
	/* register */
	if ((CFG_REGS->CFG_CFGCON4 & CFG_CFGCON4_VBKP_32KCSEL_Msk) ==
	    0x2000U) { /* SOSC : XTAL_OFF */
		DEVICE_SLEEP_ConfigSubSysPllLock(false);
	}

	/* Step 20 : Set die_BENXOANA_ovrd_en to 1 to switch LDO_PLL to CLDO */
	BTZBSYS_REGS->BTZBSYS_SUBSYS_CNTRL_REG0 |= (uint32_t)0x800U;

	/* Step 21 : Set EN_RFLDO_ovrd_en to 1 */
	BTZBSYS_REGS->BTZBSYS_SUBSYS_CNTRL_REG0 |= (uint32_t)0x8000U;

	/* Step 22 : Combining step 21,
	 * de-assert external PLL power request to disable CLKGEN LDO
	 */
	/* enable in RF */
	CRU_REGS->CRU_SPLLCON |= (uint32_t)0x08U;

	/* Step 23 : set KEEP_ACLB_CLOCKS_ON to 1 to enable ACLB clocks for SPI access */
	DEVICE_SLEEP_ConfigAclbClk(true);

	/* Step 24 : Turn off MBS in RF */
	DEVICE_SLEEP_ConfigRfMbs(false);

	/* Step 25 : set KEEP_ACLB_CLOCKS_ON to 0 to disable ACLB clocks for SPI access */
	DEVICE_SLEEP_ConfigAclbClk(false);

	/* Step 26 : set bt_en_main_clk to 0 to disable BT main clock */
	BTZBSYS_REGS->BTZBSYS_SUBSYS_CNTRL_REG0 &= ~((uint32_t)0x00100000U);

	/* Step 27 : change CLK source in CRU from POSC CLK to FRC CLK */
	CRU_REGS->CRU_OSCCON &= ~((uint32_t)0xf01U);

	/* Request oscillator switch to occur */
	CRU_REGS->CRU_OSCCONSET = CRU_OSCCON_OSWEN_Msk;

	/* Wait for indication of successful clock change before proceeding */
	while ((CRU_REGS->CRU_OSCCON & CRU_OSCCON_OSWEN_Msk) != 0U) {
		/* Do nothing */
	}

	/* If XTAL clock is OFF */
	if ((CFG_REGS->CFG_CFGCON4 & CFG_CFGCON4_VBKP_32KCSEL_Msk) ==
	    0x2000U) { /* SOSC : XTAL_OFF */
		/* Step 27.1 : If XTAL clock is OFF when bt_zb_subsys enters into sleep mode,
		 * set subsys_clk_src_sel to 1 via subsys config register
		 * (SUBSYS_CNTRL_REG1_ADDR[4]) to select PLL CLK as SRC clock
		 */
		BTZBSYS_REGS->BTZBSYS_SUBSYS_CNTRL_REG1 |= (uint32_t)0x10U;

		/* Step 28 : If XTAL clock is OFF when bt_zb_subsys enters into sleep mode, set
		 * BXTLEN to 0 via BT config register to disable XTAL
		 */
		device_delay(4); /* add 2us delay */
		DEVICE_SLEEP_ConfigRfXtal(false);

		/* Step 28.1 : If XTAL clock is OFF when bt_zb_subsys enters into sleep mode, set
		 * subsys_bypass_xtal_ready to 0 via subsys config register
		 */
		DEVICE_SLEEP_ConfigSubSysXtalReady(false);
	}
	/* Step 29 : set deep sleep enable to 0 */
	DSCON_REGS->DSCON_DSCON &= ~((uint32_t)0x8000U);

	/* Step 30 : set sleep enable to 1, make CPU into sleep */
	CRU_REGS->CRU_OSCCON |= (uint32_t)0x10U;

	/* Lock system since done with clock configuration */
	device_sys_lock();
}

void device_exit_sleep_mode(void)
{
	/* unlock key sequence */
	device_sys_unlock();

	/* step 1: Clear sleep flag */
	RCON_REGS->RCON_RCON &= (~RCON_RCON_SLEEP_Msk);

	/* If XTAL clock is off */
	if ((CFG_REGS->CFG_CFGCON4 & CFG_CFGCON4_VBKP_32KCSEL_Msk) ==
	    0x2000U) { /* SOSC : XTAL_OFF */
		/* Select PCLK(FRC) instead of XTAL(POSC) as the XTAL lock counter clock source */
		BTZBSYS_REGS->BTZBSYS_SUBSYS_CNTRL_REG1 |= (0x1 << 24);

		/* Step 2 : If XTAL clock is off when bt_zb_subsys enters into low power mode, wait
		 * for xtal_ready_out_sync Enable bit 7 to creates one clk_lp_cycle wide pulse on
		 * ZBT Subsystem.external_NMI0 pin
		 */
		CFG_REGS->CFG_CFGCON1 |= CFG_CFGCON1_ZBTWKSYS_Msk;

		/* Wait for xtal_ready */
		uint32_t clk_ready_tries = 0;

		do {
			clk_ready_tries++;
		} while (BTZB_XTAL_NOT_READY && (clk_ready_tries < CLK_READY_RETRIES));

		if ((clk_ready_tries >= CLK_READY_RETRIES) && BTZB_XTAL_NOT_READY) {
			DEVICE_SLEEP_ConfigSubSysXtalReady(true);

			while (BTZB_XTAL_NOT_READY) {
				/* Do nothing */
			}
		}
		/* Step 2.1 :
		 * If XTAL clock is OFF when bt_zb_subsys enters into sleep mode,
		 * set subsys_clk_src_sel to 0 via subsys config register
		 */
		/* (SUBSYS_CNTRL_REG1_ADDR[4]) to select XTAL CLK as SRC clock */
		BTZBSYS_REGS->BTZBSYS_SUBSYS_CNTRL_REG1 &= ~((uint32_t)0x10U);
	}

	/* Step 3 : Change CLK source in CRU from FRC CLK to POSC CLK */
	CRU_REGS->CRU_OSCCON &= ~((uint32_t)0xf01U);
	CRU_REGS->CRU_OSCCON |= (uint32_t)0x200U;

	/* Request oscillator switch to occur */
	CRU_REGS->CRU_OSCCONSET = CRU_OSCCON_OSWEN_Msk;

	/* Wait for indication of successful clock change before proceeding */
	while ((CRU_REGS->CRU_OSCCON & CRU_OSCCON_OSWEN_Msk) != 0U) {
		/* Do nothing */
	}

	/* Step 4 : set bt_en_main_clk to 1 to enable BT main clock */
	BTZBSYS_REGS->BTZBSYS_SUBSYS_CNTRL_REG0 |= (uint32_t)0x00100000U;

	/* Step 5 : set KEEP_ACLB_CLOCKS_ON to 1 via BT config register */
	DEVICE_SLEEP_ConfigAclbClk(true);

	/* Step 6 : Turn on MBS in RF */
	DEVICE_SLEEP_ConfigRfMbs(true);

	/* Step 7 : Wait for MBS settling time. Settling time value is 35us */
	device_delay(4);

	/* Step 8 : Set KEEP_ACLB_CLOCKS_ON to 0 via BT config register */
	DEVICE_SLEEP_ConfigAclbClk(false);

	/* Step 9 : Assert external PLL power request */
	CRU_REGS->CRU_SPLLCON &= ~((uint32_t)0x08U);

	/* Step 10 : Combining step 9,
	 * set EN_RFLDO_ovrd_en to 0 to enable CLKGEN LDO enable in RF
	 */
	BTZBSYS_REGS->BTZBSYS_SUBSYS_CNTRL_REG0 &= ~((uint32_t)0x8000U);

	/* Step 11 : Wait for LDO settling time. Settling time value is 10us */
	device_delay(1);

	/* Step 12 : Set die_BENXOANA_ovrd_en to 0 to switch CLDO to LDO_PLL */
	BTZBSYS_REGS->BTZBSYS_SUBSYS_CNTRL_REG0 &= ~((uint32_t)0x800U);

	/* Step 13 : Enable CLKGEN CLK enable in RF */
	DEVICE_SLEEP_ConfigRfClk(true);

	/* Step 14 : Poll pll_lock_out_sync to wait until pll_lock_out_sync is set */
	/* wait for PLL Lock */
	uint32_t clk_ready_tries = 0;

	do {
		clk_ready_tries++;
	} while (BTZB_PLL_NOT_LOCKED && (clk_ready_tries < CLK_READY_RETRIES));

	if ((clk_ready_tries >= CLK_READY_RETRIES) && BTZB_PLL_NOT_LOCKED) {
		DEVICE_SLEEP_ConfigSubSysPllLock(true);

		while (BTZB_PLL_NOT_LOCKED) {
			/* Do nothing */
		}
	}

	/* Step 15 : Restore REFOx registers programming for run mode */

	/* Check PMD lock bit */
	if ((CFG_REGS->CFG_CFGCON0 & CFG_CFGCON0_PMDLOCK_Msk) != 0U) {
		/* PMD lock is enabled, check CFGCLOCK */
		if ((CFG_REGS->CFG_CFGCON0 & CFG_CFGCON0_CFGCLOCK_Msk) == 0U) {
			/* Disable PMD lock */
			CFG_REGS->CFG_CFGCON0 &= ~CFG_CFGCON0_PMDLOCK_Msk;
		}
	}

	/* Restore all PMD setting */
	device_config_pmd_reg(DEVICE_SLEEP_EXIT_SLEEP);

	/* Restore the setting of REFOxCON register */
	device_config_ref_osc_reg(DEVICE_SLEEP_EXIT_SLEEP);

	/* Step 16 : Set PB1 CLK to SYS_CLK to restore its clock rate for run mode */
	CRU_REGS->CRU_PB1DIV = bk_reg.pb1_div;

	/* Step 17 : Restore PCHE Cache programming for run mode */
	PCHE_REGS->PCHE_CHECON = bk_reg.pche;

	/* Step 18 : Change CLK source in CRU from POSC CLK to SPLL1 CLK */
	CRU_REGS->CRU_OSCCON &= ~((uint32_t)0xf01U);
	CRU_REGS->CRU_OSCCON |= (uint32_t)0x100U;

	/* Request oscillator switch to occur */
	CRU_REGS->CRU_OSCCONSET = CRU_OSCCON_OSWEN_Msk;

	/* Wait for indication of successful clock change before proceeding */
	while ((CRU_REGS->CRU_OSCCON & CRU_OSCCON_OSWEN_Msk) != 0U) {
		/* Do nothing */
	}

	/* Step 19 : Set subsys_clk_src_sel to 1 to
	 * change subsys clock source from XTAL CLK to PLL
	 */
	/* CLK */
	BTZBSYS_REGS->BTZBSYS_SUBSYS_CNTRL_REG1 &= ~((uint32_t)0x30U);
	BTZBSYS_REGS->BTZBSYS_SUBSYS_CNTRL_REG1 |= (uint32_t)0x10U;

	/* Remove sleep enable */
	/* not match to word! word should have CRU_OSCCON */
	CRU_REGS->CRU_OSCCON &= ~((uint32_t)0x10U);

	/* Lock system since done with clock configuration */
	device_sys_lock();
}
