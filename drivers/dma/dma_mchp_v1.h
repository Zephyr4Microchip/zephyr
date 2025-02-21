/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file dma_mchp_v1.h
 * @brief DMA driver configuration for Microchip SAME54 series.
 *
 * This file contains the configuration macros and structures for the DMA
 * driver specific to the Microchip SAME54 series.
 */

#ifndef _DMA_MCHP_V1_H_
#define _DMA_MCHP_V1_H_

#if defined(CONFIG_SOC_FAMILY_MCHP_SAM_D5X_E5X)
/**
 * @file
 * @brief Include required header files
 */

/**
 * Include HAL file, Specific to the peripheral
 */
#include "dmac/hal_mchp_dma_dmac_u2503.h"

#elif defined(CONFIG_SOC_SERIES_SAM71)
/**
 * @brief Added Samv71 related configuration here.
 */
#else
#define DMA_MCHP_HAL_DEFN(n)
#endif

#endif /* _DMA_MCHP_V1_H_ */
