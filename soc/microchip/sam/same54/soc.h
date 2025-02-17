/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_MICROCHIP_SAME54_SOC_H_
#define _SOC_MICROCHIP_SAME54_SOC_H_

#ifndef _ASMLANGUAGE

#define DONT_USE_CMSIS_INIT

#include <zephyr/types.h>

#if defined(CONFIG_SOC_ATSAME54N19A)
#include <same54n19a.h>
#elif defined(CONFIG_SOC_ATSAME54N20A)
#include <same54n20a.h>
#elif defined(CONFIG_SOC_ATSAME54P19A)
#include <same54p19a.h>
#elif defined(CONFIG_SOC_ATSAME54P20A)
#include <same54p20a.h>
#else
#error Library does not support the specified device.
#endif

#endif /* _ASMLANGUAGE */

/* Helper macros for use with MICROCHIP SAM0 DMAC controller
 * return 0xff as default value if there is no 'dmas' property
 */
#define MICROCHIP_SAME54_DT_INST_DMA_CELL(n, name, cell)                                           \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, dmas),					\
			(DT_INST_DMAS_CELL_BY_NAME(n, name, cell)),			\
			(0xff))
#define MICROCHIP_SAME54_DT_INST_DMA_TRIGSRC(n, name)                                              \
	MICROCHIP_SAME54_DT_INST_DMA_CELL(n, name, trigsrc)
#define MICROCHIP_SAME54_DT_INST_DMA_CHANNEL(n, name)                                              \
	MICROCHIP_SAME54_DT_INST_DMA_CELL(n, name, channel)
#define MICROCHIP_SAME54_DT_INST_DMA_CTLR(n, name)                                                 \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, dmas),					\
				(DT_INST_DMAS_CTLR_BY_NAME(n, name)),			\
				(DT_INVALID_NODE))

#endif /* _SOC_MICROCHIP_SAME54_SOC_H_ */
