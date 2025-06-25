/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file dma_mchp_v1.h
 * @brief DMA driver configuration for Microchip MCUs.
 *
 * This file contains the configuration macros and structures for the DMA
 * driver specific to the Microchip MCUs.
 */

#ifndef MICROCHIP_DMA_MCHP_V1_H_
#define MICROCHIP_DMA_MCHP_V1_H_

#if defined(CONFIG_SOC_FAMILY_MCHP_SAM_D5X_E5X)

#define DT_DRV_COMPAT microchip_dmac_u2503

/**
 * @struct hal_dma_mchp_block_config_t
 * @brief DMA block configuration structure.
 *
 * This structure defines the configuration for a single DMA transfer block.
 */
typedef struct hal_dma_mchp_block_config {
	/** Block starting address at the source */
	uint32_t source_address;
	/** Block starting address at the destination */
	uint32_t dest_address;
	/** Number of bytes to be transferred for this block */
	uint32_t block_size;
	/**
	 * Source address adjustment option.
	 *
	 * - 0b00: Increment
	 * - 0b01: Decrement
	 * - 0b10: No change
	 */
	uint16_t source_addr_adj: 2;
	/**
	 * Destination address adjustment option.
	 *
	 * - 0b00: Increment
	 * - 0b01: Decrement
	 * - 0b10: No change
	 */
	uint16_t dest_addr_adj: 2;
} hal_dma_mchp_block_config_t;

/**
 * @struct hal_dma_mchp_status_t
 * @brief DMA runtime status structure.
 *
 * This structure provides the runtime status of a DMA channel.
 */
typedef struct hal_dma_mchp_status {
	/** Indicates whether the current DMA transfer is busy or idle */
	bool busy;
	/** Pending length to be transferred in bytes (HW specific) */
	uint32_t pending_length;
} hal_dma_mchp_status_t;

/**
 * @brief Copies block configuration data from a public Zephyr block structure
 *        to an internal HAL block configuration structure.
 *
 * This macro is used to transfer block settings from a Zephyr DMA block
 * configuration (public structure) to an internal HAL block configuration
 * (private/internal structure).
 *
 * @param hal_block Destination HAL block configuration structure.
 *                  This is an internal structure used by the HAL driver.
 * @param block Source Zephyr block structure containing the DMA block settings.
 *              This structure is part of the Zephyr public API.
 */
#define HAL_DMA_MCHP_COPY_BLOCK_CONFIG(hal_block, block)                                           \
	hal_block.block_size = block->block_size;                                                  \
	hal_block.dest_addr_adj = block->dest_addr_adj;                                            \
	hal_block.dest_address = block->dest_address;                                              \
	hal_block.source_addr_adj = block->source_addr_adj;                                        \
	hal_block.source_address = block->source_address;

/**
 * @brief Copies DMA status data from an internal HAL status structure
 *        to a public Zephyr status structure.
 *
 * This macro transfers DMA status information from the internal HAL structure
 * to the public Zephyr structure, ensuring that the Zephyr driver can access
 * the current DMA status in a standardized format.
 *
 * @param status Destination Zephyr DMA status structure (public structure).
 *               This structure is used by the Zephyr DMA API to report status.
 * @param hal_status Source HAL DMA status structure (internal structure).
 *                   This structure holds the internal DMA status information
 *                   as managed by the HAL.
 */
#define HAL_DMA_MCHP_COPY_STATUS(status, hal_status)                                               \
	status->busy = hal_status.busy;                                                            \
	status->pending_length = hal_status.pending_length;

/**
 * @brief Macro to define the HAL (Hardware Abstraction Layer) configuration for DMA.
 *
 * This macro initializes the DMA HAL structure with register base addresses,
 * and IRQ count.
 *
 * @param n DMA instance index.
 */
#define DMA_MCHP_HAL_DEFN(n)                                                                       \
	.hal_dma.regs = ((dmac_registers_t *)DT_INST_REG_ADDR(n)),                                 \
	.hal_dma.num_irq = DT_NUM_IRQS(DT_DRV_INST(n)),                                            \
	.hal_dma.mclk_sys = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, subsystem))

/**
 * @brief Enable the clock for the Microchip DMA controller.
 *
 * This macro turns on the clock for the DMA peripheral using the clock control subsystem.
 *
 * @param dev Pointer to the DMA device structure.
 */
#define DMA_MCHP_ENABLE_CLOCK(dev)                                                                 \
	clock_control_on(((const dma_mchp_dev_config_t *)(dev->config))->clock_dev,                \
			 (((dma_mchp_dev_config_t *)(dev->config))->hal_dma.mclk_sys))

/**
 * @brief Structure representing the DMA hardware abstraction layer (HAL).
 *
 * This structure contains the necessary fields for interacting with the DMA controller
 * and related peripherals, including registers, interrupts, and clock management.
 */
typedef struct hal_mchp_dma {

	/* Pointer to the DMA registers. */
	dmac_registers_t *regs;

	/* This field stores the number of interrupts associated with the DMA. */
	uint8_t num_irq;

	/* Contains the clock control configuration for the DMA subsystem. */
	clock_control_subsys_t mclk_sys;

	/* Reference to DMA HAL data */
	void *data;

} hal_mchp_dma_t;

/**
 * Include HAL file, Specific to the peripheral
 */
#include <dmac/hal_mchp_dma_dmac_u2503.h>

#endif

#endif /* MICROCHIP_DMA_MCHP_V1_H_ */
