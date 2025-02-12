/*
 * Copyright (c) [2025] [Microchip Technology Inc.]
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>

void soc_reset_hook(void)
{
	/*
	 * Force Cortex M Cache Controller disabled
	 *
	 * It is not clear if regular Cortex-M instructions can be used to
	 * perform cache maintenance or this is a proprietary cache controller
	 * that require special SoC support.
	 */
	CMCC_REGS->CMCC_CTRL &= ~(CMCC_CTRL_CEN_Msk);
}
