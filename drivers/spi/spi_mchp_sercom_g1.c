/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file spi_mchp_sercom_g1.c
 * @brief SPI driver for Microchip Technology Inc. devices
 *
 * This file contains the implementation of the SPI driver for Microchip
 * Technology Inc. devices. It includes initialization, configuration, and
 * data transfer functions.
 */

#include <soc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include <mchp_dt_helper.h>

/******************************************************************************
 * @brief Devicetree definitions
 *****************************************************************************/
#define DT_DRV_COMPAT microchip_sercom_g1_spi

/*******************************************
 * Const and Macro Defines
 *******************************************/
/**
 * @brief Set the logging level for the SPI driver.
 *
 * This macro assigns the SPI driver's logging level based on the
 * `CONFIG_SPI_LOG_LEVEL` Kconfig option. It controls the verbosity of log
 * messages (e.g., error, warning, info, debug) emitted by the driver.
 */
#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL

/**
 * @brief Register SPI MCHP G1 driver with logging subsystem.
 */
LOG_MODULE_REGISTER(spi_mchp_sercom_g1);

/**
 * @brief Including header here to avoid compilation error
 *
 * As spi_context.h header file usage Log messages, LOG_LEVEL has to be defined before.
 */
#include "spi_context.h"

/**
 * @brief Maximum supported SPI transfer size in bytes.
 *
 * This defines the upper limit for a single buffer's transfer size
 * for the Microchip SERCOM SPI controller.
 */
#define SPI_MCHP_MAX_XFER_SIZE 65535

/**
 * @brief Supported SPI word size in bits.
 *
 * This driver currently supports only 8-bit SPI data transfers.
 */
#define SUPPORTED_SPI_WORD_SIZE 8

/**
 * @brief Min pins to be defined in the pinctrl-x in dtsi
 */
#define SPI_PIN_CNT 4

/**
 * @brief Macro for handling SPI peripheral clock configuration.
 *
 * This macro retrieve the clock frequency for the SPI peripheral.
 */
#define GET_SPI_CLOCK_FREQ(dev, rate)                                                              \
	clock_control_get_rate(                                                                    \
		((const spi_mchp_dev_config_t *)(dev->config))->spi_clock.clock_dev,               \
		(((spi_mchp_dev_config_t *)(dev->config))->spi_clock.gclk_sys), &rate);

#if defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ2)
#define ENABLE_SPI_MODULE(regs)                                                                    \
	if (regs == SERCOM1_REGS) {                                                                \
		CFG_REGS->CFG_PMD3 &= ~CFG_PMD3_SER2MD_Msk;                                        \
	} else if (regs == SERCOM2_REGS) {                                                         \
		CFG_REGS->CFG_PMD3 &= ~CFG_PMD3_SER3MD_Msk;                                        \
	}
#elif defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ6)
#define ENABLE_SPI_MODULE(regs)                                                                    \
	if (regs == SERCOM4_REGS) {                                                                \
		CFG_REGS->CFG_PMD3 &= ~CFG_PMD3_SER4MD_Msk;                                        \
	} else if (regs == SERCOM5_REGS) {                                                         \
		CFG_REGS->CFG_PMD3 &= ~CFG_PMD3_SER5MD_Msk;                                        \
	}
#endif
/***********************************
 * Typedefs and Enum Declarations
 ***********************************/
/**
 * @brief Hardware-specific configuration for a Microchip SERCOM SPI instance.
 *
 * This structure holds the register base address and pad configuration
 * for a SERCOM SPI peripheral. It may also be extended to other hardware related configuration.
 */
typedef struct mchp_spi_reg_config {
	/* Pointer to SPI master registers */
	sercom_registers_t *regs;

	/* SPI pad configuration */
	uint32_t pads;

} mchp_spi_reg_config_t;

/**
 * @brief Clock configuration structure for the SPI.
 *
 * This structure contains the clock configuration parameters for the SPI
 * peripheral.
 */
#if defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ2) ||                                            \
	defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ6)
typedef struct mchp_spi_clock {
	/* Clock driver */
	const struct device *clock_dev;

	/* Generic clock subsystem. */
	clock_control_subsys_t gclk_sys;
} mchp_spi_clock_t;
#else
typedef struct mchp_spi_clock {
	/* Clock driver */
	const struct device *clock_dev;

	/* Main clock subsystem. */
	clock_control_subsys_t mclk_sys;

	/* Generic clock subsystem. */
	clock_control_subsys_t gclk_sys;
} mchp_spi_clock_t;
#endif
/**
 * @brief DMA configuration structure for the SPI
 *
 * This structure contains the DMA configuration parameters for the SPI
 * peripheral.
 */
typedef struct mchp_spi_dma {
	/* DMA driver for asynchronous operations */
	const struct device *dma_dev;

	/* TX DMA request line. */
	uint8_t tx_dma_request;

	/* TX DMA channel. */
	uint8_t tx_dma_channel;

	/* RX DMA request line. */
	uint8_t rx_dma_request;

	/* RX DMA channel. */
	uint8_t rx_dma_channel;
} mchp_spi_dma_t;

/**
 * @brief Device constant configuration parameters
 *
 * This structure holds the constant configuration parameters for the SPI
 * device.
 */
typedef struct spi_mchp_dev_config {
	/* reg configuration for SPI */
	mchp_spi_reg_config_t reg_cfg;

	/* Pin control device configuration */
	const struct pinctrl_dev_config *pcfg;

#if CONFIG_SPI_MCHP_DMA_DRIVEN
	/* DMA device */
	mchp_spi_dma_t spi_dma;
#endif

#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_MCHP_INTERRUPT_DRIVEN)
	/*IRQ configuration function */
	void (*irq_config_func)(const struct device *dev);
#endif
	/* Clock configuration */
	mchp_spi_clock_t spi_clock;

} spi_mchp_dev_config_t;

/**
 * @brief SPI run time data structure.
 *
 * This structure holds the runtime data for the SPI driver.
 */
typedef struct spi_mchp_dev_data {
	/* SPI context structure. */
	struct spi_context ctx;
#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_MCHP_INTERRUPT_DRIVEN)
	/* Variable to store the dummysize for tx and rx. */
	uint8_t dummysize;
#endif
#if CONFIG_SPI_MCHP_DMA_DRIVEN
	/* Pointer to the device structure. */
	const struct device *dev;

	/* Length of the DMA segment. */
	uint32_t dma_segment_len;
#endif
} spi_mchp_dev_data_t;

/***********************************
 * Internal functions
 ***********************************/

/*Wait for synchronization*/
static inline void spi_wait_sync(const mchp_spi_reg_config_t *spi_reg_cfg, uint32_t sync_flag)
{
	/* Wait for synchronization */
	while ((spi_reg_cfg->regs->SPIM.SERCOM_SYNCBUSY & sync_flag) != 0U) {
		/* Do nothing */
	}
}

/*Enable the SPI peripheral*/
static inline void spi_enable(const mchp_spi_reg_config_t *spi_reg_cfg, spi_operation_t op)
{
	spi_wait_sync(spi_reg_cfg, SERCOM_SPIM_SYNCBUSY_ENABLE_Msk);
	if (SPI_OP_MODE_GET(op) == SPI_OP_MODE_MASTER) {
		spi_reg_cfg->regs->SPIM.SERCOM_CTRLA |= SERCOM_SPIM_CTRLA_ENABLE_Msk;
	} else {
		spi_reg_cfg->regs->SPIS.SERCOM_CTRLA |= SERCOM_SPIS_CTRLA_ENABLE_Msk;
	}
	spi_wait_sync(spi_reg_cfg, SERCOM_SPIM_SYNCBUSY_ENABLE_Msk);
}

/*Disable the SPI peripheral*/
static inline void spi_disable(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_wait_sync(spi_reg_cfg, SERCOM_SPIM_SYNCBUSY_ENABLE_Msk);
	spi_reg_cfg->regs->SPIM.SERCOM_CTRLA &= ~SERCOM_SPIM_CTRLA_ENABLE_Msk;
	spi_wait_sync(spi_reg_cfg, SERCOM_SPIM_SYNCBUSY_ENABLE_Msk);
}

/*Set the SPI Master Mode*/
static inline int spi_master_mode(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	/* Clear the MODE bit field and set it to SPI Master mode */
	spi_reg_cfg->regs->SPIM.SERCOM_CTRLA =
		(spi_reg_cfg->regs->SPIM.SERCOM_CTRLA & ~SERCOM_SPIM_CTRLA_MODE_Msk) |
		SERCOM_SPIM_CTRLA_MODE_SPI_MASTER;

	return 0;
}

/*Set the SPI Slave Mode*/
static inline int spi_slave_mode(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	/* Clear the MODE bit field and set it to SPI Slave mode */
#if defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ2) ||                                            \
	defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ6)
	spi_reg_cfg->regs->SPIS.SERCOM_CTRLA =
		(spi_reg_cfg->regs->SPIS.SERCOM_CTRLA & ~SERCOM_SPIS_CTRLA_MODE_Msk) |
		SERCOM_SPIS_CTRLA_MODE_SPI_SLAVE | SERCOM_SPIS_CTRLA_RUNSTDBY_Msk;
#else
	spi_reg_cfg->regs->SPIS.SERCOM_CTRLA =
		(spi_reg_cfg->regs->SPIS.SERCOM_CTRLA & ~SERCOM_SPIS_CTRLA_MODE_Msk) |
		SERCOM_SPIS_CTRLA_MODE_SPI_SLAVE;
#endif

	return 0;
}

/*Set the SPI Data Order, MSB first*/
static inline void spi_msb_first(const mchp_spi_reg_config_t *spi_reg_cfg, spi_operation_t op)
{
	/* Clear the DORD bit field and set it to MSB first */
	if (SPI_OP_MODE_GET(op) == SPI_OP_MODE_MASTER) {
		spi_reg_cfg->regs->SPIM.SERCOM_CTRLA =
			(spi_reg_cfg->regs->SPIM.SERCOM_CTRLA & ~SERCOM_SPIM_CTRLA_DORD_Msk) |
			SERCOM_SPIM_CTRLA_DORD_MSB;
	} else {
		spi_reg_cfg->regs->SPIS.SERCOM_CTRLA =
			(spi_reg_cfg->regs->SPIS.SERCOM_CTRLA & ~SERCOM_SPIS_CTRLA_DORD_Msk) |
			SERCOM_SPIS_CTRLA_DORD_MSB;
	}
}

/*Set the SPI Data Order,LSB first*/
static inline void spi_lsb_first(const mchp_spi_reg_config_t *spi_reg_cfg, spi_operation_t op)
{
	/* Clear the DORD bit field and set it to LSB first */
	if (SPI_OP_MODE_GET(op) == SPI_OP_MODE_MASTER) {
		spi_reg_cfg->regs->SPIM.SERCOM_CTRLA =
			(spi_reg_cfg->regs->SPIM.SERCOM_CTRLA & ~SERCOM_SPIM_CTRLA_DORD_Msk) |
			SERCOM_SPIM_CTRLA_DORD_LSB;
	} else {
		spi_reg_cfg->regs->SPIS.SERCOM_CTRLA =
			(spi_reg_cfg->regs->SPIS.SERCOM_CTRLA & ~SERCOM_SPIS_CTRLA_DORD_Msk) |
			SERCOM_SPIS_CTRLA_DORD_LSB;
	}
}

/*Set the SPI Clock Polarity Idle Low*/
static inline void spi_cpol_idle_low(const mchp_spi_reg_config_t *spi_reg_cfg, spi_operation_t op)
{
	/* Clear the CPOL bit field and set clock polarity to Idle Low */
	if (SPI_OP_MODE_GET(op) == SPI_OP_MODE_MASTER) {
		spi_reg_cfg->regs->SPIM.SERCOM_CTRLA =
			(spi_reg_cfg->regs->SPIM.SERCOM_CTRLA & ~SERCOM_SPIM_CTRLA_CPOL_Msk) |
			SERCOM_SPIM_CTRLA_CPOL_IDLE_LOW;
	} else {
		spi_reg_cfg->regs->SPIS.SERCOM_CTRLA =
			(spi_reg_cfg->regs->SPIS.SERCOM_CTRLA & ~SERCOM_SPIS_CTRLA_CPOL_Msk) |
			SERCOM_SPIS_CTRLA_CPOL_IDLE_LOW;
	}
}

/*Set the SPI Clock Polarity Idle High*/
static inline void spi_cpol_idle_high(const mchp_spi_reg_config_t *spi_reg_cfg, spi_operation_t op)
{
	/* Clear the CPOL bit field and set clock polarity to Idle High */
	if (SPI_OP_MODE_GET(op) == SPI_OP_MODE_MASTER) {
		spi_reg_cfg->regs->SPIM.SERCOM_CTRLA =
			(spi_reg_cfg->regs->SPIM.SERCOM_CTRLA & ~SERCOM_SPIM_CTRLA_CPOL_Msk) |
			SERCOM_SPIM_CTRLA_CPOL_IDLE_HIGH;
	} else {
		spi_reg_cfg->regs->SPIS.SERCOM_CTRLA =
			(spi_reg_cfg->regs->SPIS.SERCOM_CTRLA & ~SERCOM_SPIS_CTRLA_CPOL_Msk) |
			SERCOM_SPIS_CTRLA_CPOL_IDLE_HIGH;
	}
}

/*Set the SPI Clock Phase leading Edge*/
static inline void spi_cpha_lead_edge(const mchp_spi_reg_config_t *spi_reg_cfg, spi_operation_t op)
{
	/* Clear the CPHA bit field and set clock phase to Leading Edge */
	if (SPI_OP_MODE_GET(op) == SPI_OP_MODE_MASTER) {
		spi_reg_cfg->regs->SPIM.SERCOM_CTRLA =
			(spi_reg_cfg->regs->SPIM.SERCOM_CTRLA & ~SERCOM_SPIM_CTRLA_CPHA_Msk) |
			SERCOM_SPIM_CTRLA_CPHA_LEADING_EDGE;
	} else {
		spi_reg_cfg->regs->SPIS.SERCOM_CTRLA =
			(spi_reg_cfg->regs->SPIS.SERCOM_CTRLA & ~SERCOM_SPIS_CTRLA_CPHA_Msk) |
			SERCOM_SPIS_CTRLA_CPHA_LEADING_EDGE;
	}
}

/*Set the SPI Clock Phase Trailing Edge*/
static inline void spi_cpha_trail_edge(const mchp_spi_reg_config_t *spi_reg_cfg, spi_operation_t op)
{
	/* Clear the CPHA bit field and set clock phase to Trailing Edge */
	if (SPI_OP_MODE_GET(op) == SPI_OP_MODE_MASTER) {
		spi_reg_cfg->regs->SPIM.SERCOM_CTRLA =
			(spi_reg_cfg->regs->SPIM.SERCOM_CTRLA & ~SERCOM_SPIM_CTRLA_CPHA_Msk) |
			SERCOM_SPIM_CTRLA_CPHA_TRAILING_EDGE;
	} else {
		spi_reg_cfg->regs->SPIS.SERCOM_CTRLA =
			(spi_reg_cfg->regs->SPIS.SERCOM_CTRLA & ~SERCOM_SPIS_CTRLA_CPHA_Msk) |
			SERCOM_SPIS_CTRLA_CPHA_TRAILING_EDGE;
	}
}

/*Set the SPI Half Duplex Mode*/
static inline int spi_half_duplex_mode(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	LOG_ERR("SPI half-duplex mode is not supported");

	return -1;
}

/*Set the SPI Full Duplex Mode. Since the device is full duplex mode by default this API returns
 *success
 */
static inline int spi_full_duplex_mode(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	return 0;
}

/*Set the pads for the SPI Transmission*/
static inline void spi_slave_config_pinout(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	/* Clear the DIPO and DOPO bit fields and apply the new pad configuration */
	spi_reg_cfg->regs->SPIS.SERCOM_CTRLA =
		(spi_reg_cfg->regs->SPIS.SERCOM_CTRLA &
		 ~(SERCOM_SPIS_CTRLA_DIPO_Msk | SERCOM_SPIS_CTRLA_DOPO_Msk)) |
		spi_reg_cfg->pads;
}

/*Set the pads for the SPI Transmission*/
static inline void spi_master_config_pinout(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	/* Clear the DIPO and DOPO bit fields and apply the new pad configuration */
	spi_reg_cfg->regs->SPIM.SERCOM_CTRLA =
		(spi_reg_cfg->regs->SPIM.SERCOM_CTRLA &
		 ~(SERCOM_SPIM_CTRLA_DIPO_Msk | SERCOM_SPIM_CTRLA_DOPO_Msk)) |
		spi_reg_cfg->pads;
}

/*Set the pads for the SPI Transmission for loopback mode*/
static inline void spi_mode_loopback(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	/* Clear the DIPO and DOPO bit fields and set them to PAD0 */
	spi_reg_cfg->regs->SPIM.SERCOM_CTRLA =
		(spi_reg_cfg->regs->SPIM.SERCOM_CTRLA &
		 ~(SERCOM_SPIM_CTRLA_DIPO_Msk | SERCOM_SPIM_CTRLA_DOPO_Msk)) |
		(SERCOM_SPIM_CTRLA_DIPO_PAD0 | SERCOM_SPIM_CTRLA_DOPO_PAD0);
}

/*Enable the Receiver in SPI peripheral*/
static inline void spi_rx_enable(const mchp_spi_reg_config_t *spi_reg_cfg, spi_operation_t op)
{
	if (SPI_OP_MODE_GET(op) == SPI_OP_MODE_MASTER) {
		spi_wait_sync(spi_reg_cfg, SERCOM_SPIM_SYNCBUSY_CTRLB_Msk);
		/* Clear the RXEN bit field and enable Receiver */
		spi_reg_cfg->regs->SPIM.SERCOM_CTRLB =
			(spi_reg_cfg->regs->SPIM.SERCOM_CTRLB & ~SERCOM_SPIM_CTRLB_RXEN_Msk) |
			SERCOM_SPIM_CTRLB_RXEN_Msk;
		spi_wait_sync(spi_reg_cfg, SERCOM_SPIM_SYNCBUSY_CTRLB_Msk);
	} else {
		spi_wait_sync(spi_reg_cfg, SERCOM_SPIS_SYNCBUSY_CTRLB_Msk);
		/* Clear the RXEN bit field and enable Receiver */
		spi_reg_cfg->regs->SPIS.SERCOM_CTRLB =
			(spi_reg_cfg->regs->SPIS.SERCOM_CTRLB & ~SERCOM_SPIS_CTRLB_RXEN_Msk) |
			SERCOM_SPIS_CTRLB_RXEN_Msk;
		spi_wait_sync(spi_reg_cfg, SERCOM_SPIS_SYNCBUSY_CTRLB_Msk);
	}
}

/*Set the 8 BIT Character Size in SPI peripheral*/
static inline void spi_8bit_ch_size(const mchp_spi_reg_config_t *spi_reg_cfg, spi_operation_t op)
{
	if (SPI_OP_MODE_GET(op) == SPI_OP_MODE_MASTER) {
		/* Clear the CHSIZE bit field and set character size to 8-bit */
		spi_reg_cfg->regs->SPIM.SERCOM_CTRLB =
			(spi_reg_cfg->regs->SPIM.SERCOM_CTRLB & ~SERCOM_SPIM_CTRLB_CHSIZE_Msk) |
			SERCOM_SPIM_CTRLB_CHSIZE_8_BIT;
	} else {
		/* Clear the CHSIZE bit field and set character size to 8-bit */
		spi_reg_cfg->regs->SPIS.SERCOM_CTRLB =
			(spi_reg_cfg->regs->SPIS.SERCOM_CTRLB & ~SERCOM_SPIS_CTRLB_CHSIZE_Msk) |
			SERCOM_SPIS_CTRLB_CHSIZE_8_BIT;
	}
}

/*Set the BAUD Rate value for SPI peripheral*/
static inline void spi_set_baudrate(const mchp_spi_reg_config_t *spi_reg_cfg,
				    const struct spi_config *config, uint32_t clk_freq_hz)
{
	if (config->frequency != 0) {
		uint32_t divisor = 2U * config->frequency;

		/* Use the requested or next highest possible frequency */
		uint32_t baud_value = (clk_freq_hz / divisor) - 1;

		if ((clk_freq_hz % divisor) >= (divisor / 2U)) {
			/* Round up the baud_value to ensures SPI clock is as close as possible to
			 * the requested frequency
			 */
			baud_value += 1U;
		}

		baud_value = CLAMP(baud_value, 0, UINT8_MAX);

		if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
			spi_reg_cfg->regs->SPIM.SERCOM_BAUD = baud_value;
		} else {
			spi_reg_cfg->regs->SPIS.SERCOM_BAUD = baud_value;
		}
	}
}

/*Set the Inter character dpacing*/
static inline void spi_set_icspace(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_reg_cfg->regs->SPIM.SERCOM_CTRLC |=
		SERCOM_SPIM_CTRLC_ICSPACE(CONFIG_SPI_MCHP_INTER_CHARACTER_SPACE);
}

/*Disable all SPI Interrupt*/
static inline void spi_disable_interrupts(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_reg_cfg->regs->SPIM.SERCOM_INTENCLR = SERCOM_SPIM_INTENCLR_Msk;
}

/*Write Data into DATA register*/
static inline void spi_write_data(const mchp_spi_reg_config_t *spi_reg_cfg, uint8_t data)
{
	spi_reg_cfg->regs->SPIM.SERCOM_DATA = data;
}

/*Read Data from the SPI MASTER DATA register*/
static inline uint8_t spi_read_data(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	return (uint8_t)spi_reg_cfg->regs->SPIM.SERCOM_DATA;
}

/*Read Data from the SPI SLAVE DATA register*/
static inline uint8_t spi_slave_read_data(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	return (uint8_t)spi_reg_cfg->regs->SPIS.SERCOM_DATA;
}

/*Return true if receive complete flag is set*/
static inline bool spi_is_rx_comp(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	return (spi_reg_cfg->regs->SPIM.SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_RXC_Msk) ==
	       SERCOM_SPIM_INTFLAG_RXC_Msk;
}

/*Return true if transmit complete flag is set*/
static inline bool spi_is_tx_comp(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	return ((spi_reg_cfg->regs->SPIM.SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_TXC_Msk) ==
		SERCOM_SPIM_INTFLAG_TXC_Msk);
}

/*Clear the DATA register*/
static inline void spi_clr_data(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	while ((spi_reg_cfg->regs->SPIM.SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_RXC_Msk) ==
	       SERCOM_SPIM_INTFLAG_RXC_Msk) {
		/*Clear the DATA register until the RXC flag is cleared*/
		(void)spi_reg_cfg->regs->SPIM.SERCOM_DATA;
	};
}

/*Return true if data register empty flag is set*/
static inline bool spi_is_data_empty(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	return (spi_reg_cfg->regs->SPIM.SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_DRE_Msk) ==
	       SERCOM_SPIM_INTFLAG_DRE_Msk;
}

/*Enable the Receive Complete Interrupt*/
static inline void spi_enable_rxc_interrupt(const mchp_spi_reg_config_t *spi_reg_cfg,
					    spi_operation_t op)
{
	if (SPI_OP_MODE_GET(op) == SPI_OP_MODE_MASTER) {
		spi_reg_cfg->regs->SPIM.SERCOM_INTENSET = SERCOM_SPIM_INTENSET_RXC_Msk;
	} else {
		spi_reg_cfg->regs->SPIS.SERCOM_INTENSET = SERCOM_SPIS_INTENSET_RXC_Msk;
	}
}

/*Enable the Transmit Complete Interrupt*/
static inline void spi_enable_txc_interrupt(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_reg_cfg->regs->SPIM.SERCOM_INTENSET = SERCOM_SPIM_INTENSET_TXC_Msk;
}

/*Enable the Data Register Empty Interrupt*/
static inline void spi_enable_data_empty_interrupt(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_reg_cfg->regs->SPIM.SERCOM_INTENSET = SERCOM_SPIM_INTENSET_DRE_Msk;
}

/*Disable the Receive Complete Interrupt*/
static inline void spi_disable_rxc_interrupt(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_reg_cfg->regs->SPIM.SERCOM_INTENCLR = SERCOM_SPIM_INTENCLR_RXC_Msk;
}

/*Disable the Transmit Complete Interrupt*/
static inline void spi_disable_txc_interrupt(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_reg_cfg->regs->SPIM.SERCOM_INTENCLR = SERCOM_SPIM_INTENCLR_TXC_Msk;
}

/*Disable the Data Register Empty Interrupt*/
static inline void spi_disable_data_empty_interrupt(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_reg_cfg->regs->SPIM.SERCOM_INTENCLR = SERCOM_SPIM_INTENCLR_DRE_Msk;
}

/*Return the source address. The same data register is used for both master and slave. Hence this
 * can be used in slave related APIs as well
 */
static inline void *spi_get_dma_src_addr(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	return ((void *)&(spi_reg_cfg->regs->SPIM.SERCOM_DATA));
}

/*Return the destination address*/
static inline void *spi_get_dma_dest_addr(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	return ((void *)&(spi_reg_cfg->regs->SPIM.SERCOM_DATA));
}

/*Return true if any of the interrupt is enabled is set*/
static inline bool spi_is_interrupt_set(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	return (spi_reg_cfg->regs->SPIM.SERCOM_INTENSET != 0);
}

/* Enable the preload slave data*/
static inline void spi_slave_preload_enable(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_reg_cfg->regs->SPIS.SERCOM_CTRLB =
		(spi_reg_cfg->regs->SPIS.SERCOM_CTRLB & ~SERCOM_SPIS_CTRLB_PLOADEN_Msk) |
		SERCOM_SPIS_CTRLB_PLOADEN_Msk;
}

/* Enable the slave select detection*/
static inline void spi_slave_select_low_enable(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_reg_cfg->regs->SPIS.SERCOM_CTRLB =
		(spi_reg_cfg->regs->SPIS.SERCOM_CTRLB & ~SERCOM_SPIS_CTRLB_SSDE_Msk) |
		SERCOM_SPIS_CTRLB_SSDE_Msk;
}

/* Enable the Immediate buffer overflow*/
static inline void spi_immediate_buf_overflow(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_reg_cfg->regs->SPIS.SERCOM_CTRLA =
		(spi_reg_cfg->regs->SPIS.SERCOM_CTRLA & ~SERCOM_SPIS_CTRLA_IBON_Msk) |
		SERCOM_SPIS_CTRLA_IBON_Msk;
}

/* Enable slave select line interrupt */
static inline void spi_slave_select_line_enable(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_reg_cfg->regs->SPIS.SERCOM_INTENSET =
		(spi_reg_cfg->regs->SPIS.SERCOM_INTENSET & ~SERCOM_SPIS_INTENSET_SSL_Msk) |
		SERCOM_SPIS_INTENSET_SSL_Msk;
}

/* Return the slave select line status*/
static inline bool spi_slave_select_line(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	return ((spi_reg_cfg->regs->SPIS.SERCOM_INTFLAG & SERCOM_SPIS_INTFLAG_SSL_Msk) ==
		SERCOM_SPIS_INTFLAG_SSL_Msk);
}

/* Clear the slave select line interrupt */
static inline void spi_slave_clr_slave_select_line(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_reg_cfg->regs->SPIS.SERCOM_INTFLAG = SERCOM_SPIS_INTFLAG_SSL_Msk;
}

/* Return buffer overflow flag */
static inline bool spi_slave_buf_overflow(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	return ((spi_reg_cfg->regs->SPIS.SERCOM_STATUS & SERCOM_SPIS_STATUS_BUFOVF_Msk) ==
		SERCOM_SPIS_STATUS_BUFOVF_Msk);
}

/* Clear buffer overflow flag */
static inline void spi_slave_clr_buf_overflow(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_reg_cfg->regs->SPIS.SERCOM_STATUS = SERCOM_SPIS_STATUS_BUFOVF_Msk;
}

/* Set the Hardware slave select*/
static inline void spi_slave_select_enable(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_wait_sync(spi_reg_cfg, SERCOM_SPIM_SYNCBUSY_CTRLB_Msk);

	/* Clear the MSSEN bit field and enable Master Slave Select */
	spi_reg_cfg->regs->SPIM.SERCOM_CTRLB =
		(spi_reg_cfg->regs->SPIM.SERCOM_CTRLB & ~SERCOM_SPIM_CTRLB_MSSEN_Msk) |
		SERCOM_SPIM_CTRLB_MSSEN_Msk;
	spi_wait_sync(spi_reg_cfg, SERCOM_SPIM_SYNCBUSY_CTRLB_Msk);
}

/* Enable the Transmit Complete Interrupt */
static inline void spi_slave_enable_txc_interrupt(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_reg_cfg->regs->SPIS.SERCOM_INTENSET = SERCOM_SPIS_INTENSET_TXC_Msk;
}

/* Clear the DATA register */
static inline void spi_slave_clr_data(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	while ((spi_reg_cfg->regs->SPIS.SERCOM_INTFLAG & SERCOM_SPIS_INTFLAG_RXC_Msk) ==
	       SERCOM_SPIS_INTFLAG_RXC_Msk) {
		/*Clear the DATA register until the RXC flag is cleared*/
		(void)spi_reg_cfg->regs->SPIS.SERCOM_DATA;
	};
}

/*Clear the Error Interrupt Flag */
static inline void spi_slave_clr_error_int_flag(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_reg_cfg->regs->SPIS.SERCOM_INTFLAG = (uint8_t)SERCOM_SPIS_INTFLAG_ERROR_Msk;
}

/*Return true if receive complete flag is set*/
static inline bool spi_slave_is_rx_comp(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	return (spi_reg_cfg->regs->SPIS.SERCOM_INTFLAG & SERCOM_SPIS_INTFLAG_RXC_Msk) ==
	       SERCOM_SPIS_INTFLAG_RXC_Msk;
}

/*Return true if data register empty flag is set*/
static inline bool spi_slave_is_data_empty(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	return (spi_reg_cfg->regs->SPIS.SERCOM_INTFLAG & SERCOM_SPIS_INTFLAG_DRE_Msk) ==
	       SERCOM_SPIS_INTFLAG_DRE_Msk;
}

/*Return true if transmit complete flag is set*/
static inline bool spi_slave_is_tx_comp(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	return ((spi_reg_cfg->regs->SPIS.SERCOM_INTFLAG & SERCOM_SPIS_INTFLAG_TXC_Msk) ==
		SERCOM_SPIS_INTFLAG_TXC_Msk);
}

/*Write Data into DATA register*/
static inline void spi_slave_write_data(const mchp_spi_reg_config_t *spi_reg_cfg, uint8_t data)
{
	spi_reg_cfg->regs->SPIS.SERCOM_DATA = data;
}

/*Disable DRE interrupt*/
static inline void spi_slave_disable_dre_int(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_reg_cfg->regs->SPIS.SERCOM_INTENCLR = (uint8_t)SERCOM_SPIS_INTENCLR_DRE_Msk;
}

/*Clear transmit complete flag is set*/
static inline void spi_slave_clr_tx_comp_flag(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_reg_cfg->regs->SPIS.SERCOM_INTFLAG = SERCOM_SPIS_INTFLAG_TXC_Msk;
}

/*Disable all SPI Interrupt*/
static inline void spi_slave_disable_interrupts(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_reg_cfg->regs->SPIS.SERCOM_INTENCLR = SERCOM_SPIS_INTENCLR_Msk;
}

/*Clear all SPI Interrupt*/
static inline void spi_slave_clr_interrupts(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_reg_cfg->regs->SPIS.SERCOM_INTFLAG = SERCOM_SPIS_INTFLAG_Msk;
}

/*Enable the Data Register Empty Interrupt*/
static inline void spi_slave_enable_data_empty_interrupt(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	spi_reg_cfg->regs->SPIS.SERCOM_INTENSET = SERCOM_SPIS_INTENSET_DRE_Msk;
}

/**
 * @brief Configure the Microchip SERCOM SPI peripheral.
 *
 * This function configures the SPI peripheral registers and hardware settings
 * based on the provided SPI configuration structure. It supports both master
 * and slave modes, handling character size, baud rate, clock polarity and phase,
 * data order, duplex mode, pin configuration, and interrupts/DMA setup.
 *
 * The SPI peripheral is disabled during configuration and re-enabled after setup.
 * The current configuration is saved into the SPI context for tracking.
 *
 * @param dev Pointer to the device structure for the SPI driver instance.
 * @param config Pointer to the SPI configuration parameters.
 *
 * @return 0 on success, or a negative error code on failure or unsupported options.
 */
static int spi_mchp_configure(const struct device *dev, const struct spi_config *config)
{
	const spi_mchp_dev_config_t *cfg = dev->config;
	const mchp_spi_reg_config_t *spi_reg_cfg = &cfg->reg_cfg;
	spi_mchp_dev_data_t *const data = dev->data;
	int err;
	uint32_t clock_rate;

	/* Disable the SPI for Configuration */
	spi_disable(spi_reg_cfg);

	/* Check if SPI is already configured */
	if (spi_context_configured(&data->ctx, config) == true) {
		spi_enable(spi_reg_cfg, config->operation);
		return 0;
	}

	/* Select the Character Size */
	if (SPI_WORD_SIZE_GET(config->operation) != SUPPORTED_SPI_WORD_SIZE) {
		LOG_ERR("Unsupported SPI word size: %d bits. Only 8-bit transfers are supported.",
			SPI_WORD_SIZE_GET(config->operation));
		return -ENOTSUP;
	}
	spi_8bit_ch_size(spi_reg_cfg, config->operation);

	/* Enable the Receiver in SPI Peripheral */
	spi_rx_enable(spi_reg_cfg, config->operation);

#if CONFIG_SPI_SLAVE
	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_SLAVE) {
		/*Below configurations are relevant only during slave mode.*/

		/* Enable the preload slave data*/
		spi_slave_preload_enable(spi_reg_cfg);

		/* Enable the slave select detection*/
		spi_slave_select_low_enable(spi_reg_cfg);

		/* Enable the Immediate buffer overflow*/
		spi_immediate_buf_overflow(spi_reg_cfg);

		/* Set the SPI Operation Mode */
		err = spi_slave_mode(spi_reg_cfg);
		if (err != 0) {
			return -ENOTSUP;
		}
	}
#endif

	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {

		/* Set inter character spacing*/
		spi_set_icspace(spi_reg_cfg);

		/* Get the Clock Frequency*/
		GET_SPI_CLOCK_FREQ(dev, clock_rate);

		/* Select the Baudrate */
		if (clock_rate >= (2 * config->frequency)) {
			spi_set_baudrate(spi_reg_cfg, config, clock_rate);
		} else {
			return -ENOTSUP;
		}

		/* Set the SPI Operation Mode */
		err = spi_master_mode(spi_reg_cfg);
		if (err != 0) {
			return -ENOTSUP;
		}

		/*Set the CS line*/
		if (data->ctx.num_cs_gpios != 0) {
			err = spi_context_cs_configure_all(&data->ctx);
			if (err < 0) {
				return err;
			}
		} else if (cfg->pcfg->states->pin_cnt == SPI_PIN_CNT) {
			spi_slave_select_enable(spi_reg_cfg);
		} else {
			/* Handled by user */
		}
	}

	if ((config->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		LOG_ERR("Only single line mode is supported");
		return -ENOTSUP;
	}

	/*Set the Data out and Pin out Configuration*/
	if ((config->operation & SPI_MODE_LOOP) != 0U) {

		if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_SLAVE) {
			LOG_ERR("For slave Loopback mode is not supported");
			return -ENOTSUP;
		}
		/* Set MISO and MOSI on the same pad */
		spi_mode_loopback(spi_reg_cfg);
	} else {
		/* Set the Pads */
		if (SPI_OP_MODE_GET(config->operation) != SPI_OP_MODE_MASTER) {
			spi_slave_config_pinout(spi_reg_cfg);
		} else {
			spi_master_config_pinout(spi_reg_cfg);
		}
	}

	/* Set the Clock Polarity */
	if (((config->operation & SPI_MODE_CPOL)) != 0U) {
		spi_cpol_idle_high(spi_reg_cfg, config->operation);
	} else {
		spi_cpol_idle_low(spi_reg_cfg, config->operation);
	}

	/* Set the Clock Phase */
	if (((config->operation & SPI_MODE_CPHA)) != 0U) {
		spi_cpha_trail_edge(spi_reg_cfg, config->operation);
	} else {
		spi_cpha_lead_edge(spi_reg_cfg, config->operation);
	}

	/* Set the Data Order */
	if ((config->operation & SPI_TRANSFER_LSB) != 0U) {
		spi_lsb_first(spi_reg_cfg, config->operation);
	} else {
		spi_msb_first(spi_reg_cfg, config->operation);
	}

	/* Set SPI Duplex Mode */
	if ((config->operation & SPI_HALF_DUPLEX) != 0U) {
		err = spi_half_duplex_mode(spi_reg_cfg);
		if (err != 0) {
			return -ENOTSUP;
		}
	} else {
		err = spi_full_duplex_mode(spi_reg_cfg);
		if (err != 0) {
			return -ENOTSUP;
		}
	}

	/* Enable the SPI Peripheral */
	spi_enable(spi_reg_cfg, config->operation);

#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_MCHP_INTERRUPT_DRIVEN)
	cfg->irq_config_func(dev);
#endif
#if CONFIG_SPI_MCHP_DMA_DRIVEN
	if (device_is_ready(cfg->spi_dma.dma_dev) != true) {
		return -ENODEV;
	}
	data->dev = dev;
#endif

	/* Save the config into the context */
	data->ctx.config = config;

	return 0;
}

/**
 * @brief Validate buffer lengths in an SPI buffer set.
 *
 * This function checks whether any buffer in the provided spi_buf_set
 * exceeds the maximum supported SPI transfer size (SPI_MCHP_MAX_XFER_SIZE).
 * It safely handles NULL inputs and skips validation if the buffer set is empty.
 *
 * @param buf_set Pointer to the SPI buffer set (either TX or RX).
 *
 * @return 0 on success (all buffer lengths are within limits),
 *         -EINVAL if any buffer length exceeds the allowed size.
 */
static int spi_mchp_check_buf_len(const struct spi_buf_set *buf_set)
{
	int ret_val = 0;

	do {
		if ((buf_set == 0) || (buf_set->buffers == 0)) {
			break;
		}

		for (size_t i = 0; i < buf_set->count; i++) {
			if (buf_set->buffers[i].len > SPI_MCHP_MAX_XFER_SIZE) {
				LOG_ERR("SPI buffer length (%u) exceeds max allowed (%u)",
					buf_set->buffers[i].len, SPI_MCHP_MAX_XFER_SIZE);
				ret_val = -EINVAL;
				break;
			}
		}
	} while (0);

	return ret_val;
}

#ifndef CONFIG_SPI_MCHP_INTERRUPT_DRIVEN
/**
 * @brief Check if an SPI transfer is in progress.
 *
 * This function checks the SPI context to determine if there is an ongoing
 * transmit or receive operation.
 *
 * @param data Pointer to the SPI device data structure.
 *
 * @return true if a transfer is in progress, false otherwise.
 */
static bool spi_mchp_transfer_in_progress(spi_mchp_dev_data_t *data)
{
	return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

/**
 * @brief Finish any ongoing writes and drop any remaining read data.
 *
 * This function ensures that any ongoing SPI transmissions are completed
 * and clears any remaining data in the SPI buffer.
 *
 * @param spi_reg_cfg Pointer to the mchp_spi_reg_config_t structure.
 */
static int spi_mchp_finish(const mchp_spi_reg_config_t *spi_reg_cfg)
{
	while (spi_is_tx_comp(spi_reg_cfg) != true) {
		/* Wait until transmit complete */
	};

	/* Clear the data */
	spi_clr_data(spi_reg_cfg);

	return 0;
}

/**
 * @brief Polls the SPI interface for incoming data and handles
 * transmission.
 *
 * This function polls the SPI interface for incoming data, transmits data
 * if available, and updates the SPI context for both transmission and
 * reception.
 *
 * @param spi_reg_cfg Pointer to the mchp_spi_reg_config_t structure.
 * @param data Pointer to the SPI device data structure.
 */
static int spi_mchp_poll_in(const mchp_spi_reg_config_t *spi_reg_cfg, spi_mchp_dev_data_t *data)
{
	uint8_t tx_data;
	uint8_t rx_data;

	/* Check if there is data to transmit */
	if (spi_context_tx_buf_on(&data->ctx) == true) {
		tx_data = *(uint8_t *)(data->ctx.tx_buf);
	} else {
		tx_data = 0U;
	}

	while (spi_is_data_empty(spi_reg_cfg) != true) {
		/* wait until the SPI data is empty */
	};

	/* Write data to the SPI data */
	spi_write_data(spi_reg_cfg, tx_data);

	/* Update the SPI context for transmission */
	spi_context_update_tx(&data->ctx, 1, 1);

	while (spi_is_rx_comp(spi_reg_cfg) != true) {
		/* Wait for the reception to complete */
	};

	/* Read the received data from the SPI data register */
	rx_data = spi_read_data(spi_reg_cfg);

	/* Check if there is a buffer to store received data */
	if (spi_context_rx_buf_on(&data->ctx) == true) {
		*data->ctx.rx_buf = rx_data;
	}

	/* Update the SPI context for reception */
	spi_context_update_rx(&data->ctx, 1, 1);
	return 0;
}

/**
 * @brief Transmits data quickly over the SPI interface.
 *
 * This function transmits data from the provided buffer over the SPI
 * interface in a fast manner by directly writing each byte to the SPI data
 * register.
 *
 * @param spi_reg_cfg Pointer to the mchp_spi_reg_config_t structure for
 * the SPI interface.
 * @param tx_buf Pointer to the SPI buffer structure containing the data to
 * be transmitted.
 */
static int spi_mchp_fast_tx(const mchp_spi_reg_config_t *spi_reg_cfg, const struct spi_buf *tx_buf)
{
	const uint8_t *tx_data_ptr = tx_buf->buf;
	uint8_t tx_data;
	size_t len = tx_buf->len;
	uint8_t dummy_data = 0U;
	int err;

	/* Transmit each byte in the buffer */
	while (len != 0) {
		if (tx_buf->buf != NULL) {
			tx_data = *tx_data_ptr++;
		} else {
			tx_data = dummy_data;
		}
		while (spi_is_data_empty(spi_reg_cfg) != true) {
			/* Wait until the tramist is complete */
		}
		spi_write_data(spi_reg_cfg, tx_data);
		len--;
	}

	/* Finish the SPI transmission */
	err = spi_mchp_finish(spi_reg_cfg);
	return err;
}

/**
 * @brief Receives data quickly over the SPI interface.
 *
 * This function receives data to the provided buffer over the SPI
 * interface in a fast manner by directly reading each byte to the SPI data
 * register.
 *
 * @param spi_reg_cfg Pointer to the mchp_spi_reg_config_t structure for
 * the SPI interface.
 * @param rx_buf Pointer to the SPI buffer structure to which the data is
 * received.
 */
static int spi_mchp_fast_rx(const mchp_spi_reg_config_t *spi_reg_cfg, const struct spi_buf *rx_buf)
{
	uint8_t *rx_data_ptr = rx_buf->buf;
	size_t len = rx_buf->len;
	uint8_t dummy_data = 0U;
	int err;

	if (len == 0) {
		return 0;
	}

	while (len != 0) {

		/* Write a dummy data to receive data */
		spi_write_data(spi_reg_cfg, dummy_data);
		len--;

		while (spi_is_rx_comp(spi_reg_cfg) != true) {
			/* Wait for completion, and read */
		};

		if (rx_buf->buf != NULL) {
			*rx_data_ptr = spi_read_data(spi_reg_cfg);
			rx_data_ptr++;
		} else {
			(void)spi_read_data(spi_reg_cfg);
		}
	}

	/* Finish the SPI transmission */
	err = spi_mchp_finish(spi_reg_cfg);

	return err;
}

/**
 * @brief Perform fast SPI transmission and reception.
 *
 * This function transmits and receives data buffers of the same length
 * over SPI using the Microchip hardware abstraction layer.
 *
 * @param spi_reg_cfg Pointer to the mchp_spi_reg_config_t structure.
 * @param tx_buf Pointer to the SPI transmit buffer.
 * @param rx_buf Pointer to the SPI receive buffer.
 */
static int spi_mchp_fast_txrx(const mchp_spi_reg_config_t *spi_reg_cfg,
			      const struct spi_buf *tx_buf, const struct spi_buf *rx_buf)
{
	const uint8_t *tx_data_ptr = tx_buf->buf;
	uint8_t *rx_data_ptr = rx_buf->buf;
	size_t len = rx_buf->len;
	uint8_t dummy_data = 0U;
	int err;

	if (len == 0) {
		return 0;
	}

	while (len > 0) {
		/* Send the next byte */
		if (tx_data_ptr != NULL) {
			spi_write_data(spi_reg_cfg, *tx_data_ptr);
			tx_data_ptr++;
		} else {
			spi_write_data(spi_reg_cfg, dummy_data);
		}

		/* Wait for completion */
		while (spi_is_rx_comp(spi_reg_cfg) != true) {
			/* Wait for completion */
		};

		/* Read received data */
		if (rx_data_ptr != NULL) {
			*rx_data_ptr = spi_read_data(spi_reg_cfg);
			rx_data_ptr++;
		} else {
			(void)spi_read_data(spi_reg_cfg);
		}
		len--;
	}

	/* Finish the SPI transmission */
	err = spi_mchp_finish(spi_reg_cfg);

	return err;
}

/**
 * @brief Perform fast SPI transceive operation.
 *
 * This function transmits and receives data using SPI, ensuring that
 * overlapping TX and RX buffers have the same length. It processes
 * multiple buffers efficiently using direct register access.
 *
 * @param dev Pointer to the SPI device.
 * @param config Pointer to the SPI configuration structure.
 * @param tx_bufs Pointer to the set of SPI transmit buffers.
 * @param rx_bufs Pointer to the set of SPI receive buffers.
 */
static int spi_mchp_fast_transceive(const struct device *dev, const struct spi_config *config,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs)
{
	const spi_mchp_dev_config_t *cfg = dev->config;
	const mchp_spi_reg_config_t *spi_reg_cfg = &cfg->reg_cfg;
	size_t tx_count = 0;
	size_t rx_count = 0;
	const struct spi_buf *tx_data_ptr = NULL;
	const struct spi_buf *rx_data_ptr = NULL;
	int err;

	if (tx_bufs != NULL) {
		tx_data_ptr = tx_bufs->buffers;
		tx_count = tx_bufs->count;
	}

	if (rx_bufs != NULL) {
		rx_data_ptr = rx_bufs->buffers;
		rx_count = rx_bufs->count;
	} else {
		rx_data_ptr = NULL;
	}

	while (tx_count != 0 && rx_count != 0) {
		/* This function is called only if the count is equal*/
		err = spi_mchp_fast_txrx(spi_reg_cfg, tx_data_ptr, rx_data_ptr);

		tx_data_ptr++;
		tx_count--;
		rx_data_ptr++;
		rx_count--;
	}

	/* Handle remaining transmit buffers */
	while (tx_count > 0) {
		err = spi_mchp_fast_tx(spi_reg_cfg, tx_data_ptr);
		tx_data_ptr++;
		tx_count--;
	}

	/* Handle remaining receive buffers */
	while (rx_count > 0) {
		err = spi_mchp_fast_rx(spi_reg_cfg, rx_data_ptr);
		rx_data_ptr++;
		rx_count--;
	}

	return err;
}

/**
 * @brief Check if SPI transmit and receive buffers have the same length.
 *
 * This function verifies whether all corresponding buffers in the
 * transmit and receive buffer sets have matching lengths.
 *
 * @param tx_bufs Pointer to the SPI transmit buffer set.
 * @param rx_bufs Pointer to the SPI receive buffer set.
 *
 * @return true if all corresponding TX and RX buffers have the same
 * length, false otherwise.
 */
static bool spi_mchp_is_same_len(const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs)
{
	const struct spi_buf *tx_data_ptr = NULL;
	const struct spi_buf *rx_data_ptr = NULL;
	size_t tx_count = 0;
	size_t rx_count = 0;

	if (tx_bufs != NULL) {
		tx_data_ptr = tx_bufs->buffers;
		tx_count = tx_bufs->count;
	}

	if (rx_bufs != NULL) {
		rx_data_ptr = rx_bufs->buffers;
		rx_count = rx_bufs->count;
	}

	while (tx_count != 0 && rx_count != 0) {
		/* Compare the length of each corresponding TX and RX buffer */
		if (tx_data_ptr->len != rx_data_ptr->len) {
			return false;
		}

		tx_data_ptr++;
		tx_count--;
		rx_data_ptr++;
		rx_count--;
	}

	return true;
}
#endif /* CONFIG_SPI_MCHP_INTERRUPT_DRIVEN*/

#if defined(CONFIG_SPI_MCHP_INTERRUPT_DRIVEN) || (CONFIG_SPI_MCHP_INTERRUPT_DRIVEN_ASYNC)
/**
 * @brief Perform SPI master transceive operation using interrupts.
 *
 * This function initializes and starts an SPI transaction in
 * interrupt-driven mode. It sets up the SPI buffers, writes the first
 * byte, and enables relevant interrupts to handle data transfer.
 *
 * @param dev Pointer to the SPI device.
 * @param config Pointer to the SPI configuration structure.
 * @param tx_bufs Pointer to the set of SPI transmit buffers.
 * @param rx_bufs Pointer to the set of SPI receive buffers.
 *
 * @return 0 on success.
 */
static int spi_mchp_transceive_interrupt(const struct device *dev, const struct spi_config *config,
					 const struct spi_buf_set *tx_bufs,
					 const struct spi_buf_set *rx_bufs)
{
	const spi_mchp_dev_config_t *cfg = dev->config;
	const mchp_spi_reg_config_t *spi_reg_cfg = &cfg->reg_cfg;
	spi_mchp_dev_data_t *const data = dev->data;
	uint8_t tx_data;

	/* Setup SPI buffers */
	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	/* Prepare first byte for transmission */
	if (spi_context_tx_buf_on(&data->ctx) == true) {
		tx_data = *(uint8_t *)(data->ctx.tx_buf);
	} else {
		tx_data = 0U;
	}

	/* Clear the Data Register */
	spi_clr_data(spi_reg_cfg);

	/* Get the dummysize */
	if ((data->ctx.rx_len) > (data->ctx.tx_len)) {
		data->dummysize = (data->ctx.rx_len) - (data->ctx.tx_len);
	}

	/* Write first data byte to the SPI data register */
	spi_context_update_tx(&data->ctx, 1, 1);
	spi_write_data(spi_reg_cfg, tx_data);

	/* Enable SPI interrupts for RX, TX completion, and data empty events */
	if (data->ctx.rx_len > 0) {
		spi_enable_rxc_interrupt(spi_reg_cfg, config->operation);
	} else {
		spi_enable_data_empty_interrupt(spi_reg_cfg);
	}

#if defined(CONFIG_SPI_MCHP_INTERRUPT_DRIVEN)
	/* Wait for transaction to complete */
	spi_context_wait_for_completion(&data->ctx);
#endif
	return 0;
}

#if CONFIG_SPI_SLAVE
/**
 * @brief Perform SPI transmit operation using interrupts in slave mode.
 *
 * This function initializes and starts an SPI transaction in
 * interrupt-driven mode. It writes the initial bytes in Data register
 * and enables relevant interrupt to handle data transfer.
 *
 * @param dev Pointer to the SPI device.
 *
 * @return None.
 */
static void spi_mchp_slave_write(const struct device *dev)
{
	const spi_mchp_dev_config_t *cfg = dev->config;
	const mchp_spi_reg_config_t *spi_reg_cfg = &cfg->reg_cfg;
	uint8_t tx_data;
	uint8_t dummy_data = 0U;
	spi_mchp_dev_data_t *const data = dev->data;
	bool write_ready;

	/* Prepare initial bytes for transmission */
	if (spi_context_tx_buf_on(&data->ctx) == true) {
		write_ready = spi_context_tx_buf_on(&data->ctx);
		write_ready = write_ready && (spi_slave_is_data_empty(spi_reg_cfg) == true);
		while (write_ready) {
			tx_data = *(uint8_t *)(data->ctx.tx_buf);
			spi_slave_write_data(spi_reg_cfg, tx_data);

			/* Write data byte to the SPI data register */
			spi_context_update_tx(&data->ctx, 1, 1);
			write_ready = spi_context_tx_buf_on(&data->ctx);
			write_ready = write_ready && (spi_slave_is_data_empty(spi_reg_cfg) == true);
		}
	} else {
		write_ready = (spi_slave_is_data_empty(spi_reg_cfg));
		while (write_ready) {
			tx_data = dummy_data;
			spi_slave_write_data(spi_reg_cfg, tx_data);
			write_ready = (spi_slave_is_data_empty(spi_reg_cfg));
		}
	}
	spi_slave_enable_data_empty_interrupt(spi_reg_cfg);
}

/**
 * @brief Perform SPI transceive operation using interrupts in slave mode.
 *
 * This function initializes and starts an SPI transaction in
 * interrupt-driven mode. It sets up the SPI buffers, initiate writing data
 * into Data register, and enables relevant interrupts to handle data transfer.
 *
 * @param dev Pointer to the SPI device.
 * @param config Pointer to the SPI configuration structure.
 * @param tx_bufs Pointer to the set of SPI transmit buffers.
 * @param rx_bufs Pointer to the set of SPI receive buffers.
 *
 * @return number of bytes received.
 */
static int spi_mchp_slave_transceive_interrupt(const struct device *dev,
					       const struct spi_config *config,
					       const struct spi_buf_set *tx_bufs,
					       const struct spi_buf_set *rx_bufs)
{
	const spi_mchp_dev_config_t *cfg = dev->config;
	const mchp_spi_reg_config_t *spi_reg_cfg = &cfg->reg_cfg;
	spi_mchp_dev_data_t *const data = dev->data;
	int ret = 0;

	/* Setup SPI buffers */
	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	if (spi_context_tx_on(&data->ctx) == true) {
		/* Prepare for transmission */
		spi_mchp_slave_write(dev);
	}

	/* Enable receive interrupt */
	spi_enable_rxc_interrupt(spi_reg_cfg, config->operation);

	/* Enable slave select line interrupt */
	spi_slave_select_line_enable(spi_reg_cfg);
#if defined(CONFIG_SPI_MCHP_INTERRUPT_DRIVEN)
	ret = spi_context_wait_for_completion(&data->ctx);
#endif
	return ret;
}

#endif
#endif

/**
 * @brief Perform synchronous SPI transceive operation.
 *
 * This function executes a complete SPI transaction in synchronous mode using
 * the SPI context locking mechanism. It configures the device, asserts the chip
 * select if required, and selects the appropriate transmission method based on
 * the configuration:
 *
 * - In **interrupt-driven** mode (`CONFIG_SPI_MCHP_INTERRUPT_DRIVEN`):
 *   - If operating as SPI master, it uses `spi_mchp_transceive_interrupt()`.
 *   - If operating as SPI slave (`CONFIG_SPI_SLAVE`), it uses
 *     `spi_mchp_slave_transceive_interrupt()`.
 *
 * - In **polling mode** (when interrupts are disabled):
 *   - If TX and RX buffer lengths match, it uses the optimized
 *     `spi_mchp_fast_transceive()` path.
 *   - Otherwise, it performs a byte-wise polling-based transceive operation.
 *
 * After the transfer, the function handles chip select deassertion and releases
 * the SPI context.
 *
 * @param dev Pointer to the SPI device.
 * @param config Pointer to the SPI configuration structure.
 * @param tx_bufs Pointer to the transmit buffer set.
 * @param rx_bufs Pointer to the receive buffer set.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int spi_mchp_transceive_sync(const struct device *dev, const struct spi_config *config,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs)
{
	const spi_mchp_dev_config_t *cfg = dev->config;
	const mchp_spi_reg_config_t *spi_reg_cfg = &cfg->reg_cfg;
	spi_mchp_dev_data_t *data = dev->data;
	int ret;

	ret = spi_mchp_check_buf_len(tx_bufs);
	if (ret < 0) {
		return ret;
	}

	ret = spi_mchp_check_buf_len(rx_bufs);
	if (ret < 0) {
		return ret;
	}

	ARG_UNUSED(spi_reg_cfg);

	/* Lock SPI context */
	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	/* Configure SPI device */
	ret = spi_mchp_configure(dev, config);
	if (ret != 0) {
		/* Release SPI context */
		spi_context_release(&data->ctx, ret);
		return ret;
	}

	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
		/* Assert chip select (CS) */
		spi_context_cs_control(&data->ctx, true);
	}

#if CONFIG_SPI_MCHP_INTERRUPT_DRIVEN
#if CONFIG_SPI_SLAVE
	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_SLAVE) {
		ret = spi_mchp_slave_transceive_interrupt(dev, config, tx_bufs, rx_bufs);
	}
#endif
	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
		/* Use interrupt-driven SPI transceive */
		ret = spi_mchp_transceive_interrupt(dev, config, tx_bufs, rx_bufs);
	}
#else
	/* Use optimized fast path if TX and RX buffer lengths match */
	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
		if (spi_mchp_is_same_len(tx_bufs, rx_bufs) == true) {
			spi_mchp_fast_transceive(dev, config, tx_bufs, rx_bufs);
		} else {
			/* Setup SPI buffers and process using polling */
			spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

			do {
				ret = spi_mchp_poll_in(spi_reg_cfg, data);
			} while (spi_mchp_transfer_in_progress(data) && ret == 0);
		}
	}

#if CONFIG_SPI_SLAVE
	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_SLAVE) {
		/* Release SPI context */
		spi_context_release(&data->ctx, ret);
		return -ENOTSUP;
	}
#endif
#endif

	/* Deassert chip select (CS) */
	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
		spi_context_cs_control(&data->ctx, false);
	}

	/* Release SPI context */
	spi_context_release(&data->ctx, ret);

	return ret;
}

#if CONFIG_SPI_ASYNC
#if CONFIG_SPI_MCHP_DMA_DRIVEN
static void spi_mchp_dma_rx_done(const struct device *dma_dev, void *arg, uint32_t id,
				 int error_code);
/**
 * @brief Load data into the DMA controller for SPI TX transfer.
 *
 * This function configures and starts a DMA transaction to transfer data
 * from memory to the SPI peripheral. If no TX buffer is provided, it sends
 * dummy bytes (0U) instead.
 *
 * @param dev Pointer to the SPI device structure.
 * @param buf Pointer to the TX data buffer (can be NULL).
 * @param len Length of the data to be transmitted.
 *
 * @retval 0 on success.
 * @retval Negative error code on failure.
 */
static int spi_mchp_dma_tx_load(const struct device *dev, const uint8_t *buf, size_t len)
{
	const spi_mchp_dev_config_t *cfg = dev->config;
	const mchp_spi_reg_config_t *spi_reg_cfg = &cfg->reg_cfg;

	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};
	int retval;

	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->spi_dma.tx_dma_request;

	dma_blk.block_size = len;

	if (buf != NULL) {
		dma_blk.source_address = (uint32_t)buf;
	} else {
		static const uint8_t dummy_data;

		dma_blk.source_address = (uint32_t)&dummy_data;
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	dma_blk.dest_address = (uint32_t)(spi_get_dma_dest_addr(spi_reg_cfg));
	dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	retval = dma_config(cfg->spi_dma.dma_dev, cfg->spi_dma.tx_dma_channel, &dma_cfg);

	if (retval != 0) {
		return retval;
	}

	return dma_start(cfg->spi_dma.dma_dev, cfg->spi_dma.tx_dma_channel);
}

/**
 * @brief Configure and start DMA for SPI RX (receive).
 *
 * This function sets up the DMA configuration for receiving SPI data and
 * starts the DMA transfer. If a valid buffer is provided, the received
 * data is stored there. Otherwise, a dummy buffer is used.
 *
 * @param dev Pointer to the SPI device.
 * @param buf Pointer to the receive buffer, or NULL if the received data
 * is not needed.
 * @param len Length of data to be received.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int spi_mchp_dma_rx_load(const struct device *dev, uint8_t *buf, size_t len)
{
	const spi_mchp_dev_config_t *cfg = dev->config;
	const mchp_spi_reg_config_t *spi_reg_cfg = &cfg->reg_cfg;
	spi_mchp_dev_data_t *data = dev->data;

	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};
	int retval;

	/* Configure DMA settings */
	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.user_data = data;
	dma_cfg.dma_callback = spi_mchp_dma_rx_done;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->spi_dma.rx_dma_request;

	/* Set block size */
	dma_blk.block_size = len;

	/* Configure destination address */
	if (buf != NULL) {
		dma_blk.dest_address = (uint32_t)buf;
	} else {
		/* Use a static dummy variable if no buffer is provided */
		static uint8_t dummy_data;

		dma_blk.dest_address = (uint32_t)&dummy_data;
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	/* Configure source address */
	dma_blk.source_address = (uint32_t)(spi_get_dma_src_addr(spi_reg_cfg));
	dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	/* Apply DMA configuration */
	retval = dma_config(cfg->spi_dma.dma_dev, cfg->spi_dma.rx_dma_channel, &dma_cfg);
	if (retval != 0) {
		return retval;
	}

	/* Start DMA transfer */
	return dma_start(cfg->spi_dma.dma_dev, cfg->spi_dma.rx_dma_channel);
}

/**
 * @brief Determines the next SPI DMA transfer segment length.
 *
 * This function selects the shortest valid buffer length from TX and RX
 * buffers to determine the next DMA transfer segment. It ensures that
 * the segment length does not exceed the maximum allowable DMA transfer
 * size.
 *
 * @param dev Pointer to the SPI device structure.
 *
 * @retval true if there is a valid segment to transfer.
 * @retval false if there is no data left to transfer.
 */
static bool spi_mchp_dma_select_segment(const struct device *dev)
{
	spi_mchp_dev_data_t *data = dev->data;
	uint32_t segment_len;

	/* Pick the shorter buffer of ones that have an actual length */
	if (data->ctx.rx_len != 0) {
		segment_len = data->ctx.rx_len;
		if (data->ctx.tx_len != 0) {
			segment_len = MIN(segment_len, data->ctx.tx_len);
		}
	} else {
		segment_len = data->ctx.tx_len;
	}

	if (segment_len == 0) {
		return false;
	}

	/* Ensure the segment length does not exceed the max allowed value
	 */
	segment_len = MIN(segment_len, 65535);

	data->dma_segment_len = segment_len;

	return true;
}

/**
 * @brief Advances the SPI DMA buffers for the next segment.
 *
 * This function loads the next DMA segment into the RX and TX buffers.
 * It first loads the RX buffer to ensure data can be received before
 * initiating transmission. Then, it loads the TX buffer, which starts
 * the SPI clocking process in case of master.
 *
 * @param dev Pointer to the SPI device structure.
 *
 * @retval 0 on success.
 * @retval -EINVAL if the DMA segment length is zero.
 * @retval Negative error code if DMA loading fails.
 */
static int spi_mchp_dma_setup_buffers(const struct device *dev)
{
	spi_mchp_dev_data_t *data = dev->data;
	int retval;

	if (data->dma_segment_len == 0) {
		return -EINVAL;
	}

	/* Load receive buffer first to prepare for incoming data */
	if (data->ctx.rx_len != 0U) {
		retval = spi_mchp_dma_rx_load(dev, data->ctx.rx_buf, data->dma_segment_len);
	} else {
		retval = spi_mchp_dma_rx_load(dev, NULL, data->dma_segment_len);
	}

	if (retval != 0) {
		return retval;
	}

	/* Load transmit buffer, which starts SPI bus clocking */
	if (data->ctx.tx_len != 0U) {
		retval = spi_mchp_dma_tx_load(dev, data->ctx.tx_buf, data->dma_segment_len);
	} else {
		retval = spi_mchp_dma_tx_load(dev, NULL, data->dma_segment_len);
	}

	if (retval != 0) {
		return retval;
	}

	return 0;
}

/**
 * @brief SPI DMA RX completion callback.
 *
 * This function is called when a DMA transfer for SPI RX completes.
 * It updates the SPI context with the received data, checks if more
 * data segments need to be transferred, and either continues the
 * transfer or completes the SPI transaction.
 *
 * @param dma_dev Pointer to the DMA device structure.
 * @param arg Pointer to the SPI device data structure.
 * @param id DMA transaction ID (unused).
 * @param error_code Error code from the DMA transfer (unused).
 */
static void spi_mchp_dma_rx_done(const struct device *dma_dev, void *arg, uint32_t id,
				 int error_code)
{
	spi_mchp_dev_data_t *data = arg;
	const struct device *dev = data->dev;
	const spi_mchp_dev_config_t *cfg = dev->config;
	int retval;

	ARG_UNUSED(id);
	ARG_UNUSED(error_code);

	/* Update TX and RX context with the completed DMA segment */
	spi_context_update_tx(&data->ctx, 1, data->dma_segment_len);
	spi_context_update_rx(&data->ctx, 1, data->dma_segment_len);

	/* Check if more segments need to be transferred */
	if (spi_mchp_dma_select_segment(dev) == false) {
		if (spi_context_is_slave(&data->ctx) == false) {
			spi_context_cs_control(&data->ctx, false);
		}
		/* Transmission complete */
		spi_context_complete(&data->ctx, dev, 0);
		return;
	}

	/* Load the next DMA segment */
	retval = spi_mchp_dma_setup_buffers(dev);
	if (retval != 0) {
		/* Stop DMA and terminate the SPI transaction in case of failure */
		dma_stop(cfg->spi_dma.dma_dev, cfg->spi_dma.tx_dma_channel);
		dma_stop(cfg->spi_dma.dma_dev, cfg->spi_dma.rx_dma_channel);
		if (spi_context_is_slave(&data->ctx) == false) {
			spi_context_cs_control(&data->ctx, false);
		}
		spi_context_complete(&data->ctx, dev, retval);
		return;
	}
}
#endif /* CONFIG_SPI_MCHP_DMA_DRIVEN */

/**
 * @brief Perform an asynchronous SPI transceive operation.
 *
 * This function initiates a non-blocking (asynchronous) SPI transfer. It supports
 * DMA-driven transfers when `CONFIG_SPI_MCHP_DMA_DRIVEN` is enabled and falls back
 * to interrupt-driven mode otherwise. The function locks the SPI context, configures
 * the peripheral, sets up DMA or interrupt paths, and returns immediately. Completion
 * is signaled via a user-provided callback.
 *
 * Key behaviors:
 * - If DMA is enabled and channels are configured, it starts the DMA-based transfer.
 * - If DMA is not enabled, it uses interrupt-based transceive for master or slave.
 * - The function ensures chip select is asserted for master mode and releases it
 *   appropriately on error.
 *
 * @param dev Pointer to the SPI device structure.
 * @param config Pointer to the SPI configuration structure.
 * @param tx_bufs Pointer to the SPI transmit buffer set.
 * @param rx_bufs Pointer to the SPI receive buffer set.
 * @param spi_callback Callback to invoke upon completion.
 * @param userdata User-defined pointer passed to the callback.
 *
 * @retval 0 Transfer successfully initiated.
 * @retval -ENOTSUP DMA not supported or improperly configured.
 * @retval Negative error code if configuration or transfer setup fails.
 */
static int spi_mchp_transceive_async(const struct device *dev, const struct spi_config *config,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs, spi_callback_t spi_callback,
				     void *userdata)
{
	const spi_mchp_dev_config_t *cfg = dev->config;
	spi_mchp_dev_data_t *data = dev->data;
	int retval;

	retval = spi_mchp_check_buf_len(tx_bufs);
	if (retval < 0) {
		return retval;
	}

	retval = spi_mchp_check_buf_len(rx_bufs);
	if (retval < 0) {
		return retval;
	}

	ARG_UNUSED(cfg);
/*
 * Transmit clocks the output, and we use receive to
 * determine when the transmit is done, so we
 * always need both TX and RX DMA channels.
 */
#if CONFIG_SPI_MCHP_DMA_DRIVEN
	if (cfg->spi_dma.tx_dma_channel == 0xFF || cfg->spi_dma.rx_dma_channel == 0xFF) {
		return -ENOTSUP;
	}
#endif

	/* Lock SPI context for exclusive access */
	spi_context_lock(&data->ctx, true, spi_callback, userdata, config);

	/* Configure SPI based on the provided configuration */
	retval = spi_mchp_configure(dev, config);
	if (retval != 0) {
		/* Release SPI context lock */
		spi_context_release(&data->ctx, retval);
		return retval;
	}

	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
		/* Assert chip select */
		spi_context_cs_control(&data->ctx, true);
	}

	/* Set up TX and RX buffers */
	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

/* Prepare and start DMA transfers */
#if CONFIG_SPI_MCHP_DMA_DRIVEN
	spi_mchp_dma_select_segment(dev);
	retval = spi_mchp_dma_setup_buffers(dev);
#else
	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
		retval = spi_mchp_transceive_interrupt(dev, config, tx_bufs, rx_bufs);
	}
#if CONFIG_SPI_SLAVE
	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_SLAVE) {
		retval = spi_mchp_slave_transceive_interrupt(dev, config, tx_bufs, rx_bufs);
	}
#endif
#endif

	if (retval != 0) {
		/* Stop DMA transfers in case of failure */
#if CONFIG_SPI_MCHP_DMA_DRIVEN
		dma_stop(cfg->spi_dma.dma_dev, cfg->spi_dma.tx_dma_channel);
		dma_stop(cfg->spi_dma.dma_dev, cfg->spi_dma.rx_dma_channel);
#endif

		/* Deassert chip select */
		if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
			spi_context_cs_control(&data->ctx, false);
		}

		/* Release SPI context lock */
		spi_context_release(&data->ctx, retval);
	}

	return retval;
}
#endif /* CONFIG_SPI_ASYNC */

/**
 * @brief Release the SPI bus.
 *
 * This function unlocks the SPI context, allowing other transactions to
 * proceed. It is typically called when a SPI transaction is complete or
 * aborted.
 *
 * @param dev Pointer to the SPI device structure.
 * @param config Pointer to the SPI configuration structure (unused).
 *
 * @retval 0 Always returns 0 as this function does not fail.
 */
static int spi_mchp_release(const struct device *dev, const struct spi_config *config)
{
	spi_mchp_dev_data_t *data = dev->data;

	/* Unlock the SPI context to allow other operations */
	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}
#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_MCHP_INTERRUPT_DRIVEN)

/**
 * SPI interrupt service routine for Microchip SPI controller.
 *
 * This ISR handles SPI data transmission and reception. It reads incoming
 * data, writes outgoing data when available, and manages interrupts based
 * on the SPI transfer state. It also releases semaphores to notify waiting
 * threads when using interrupt-driven or asynchronous modes.
 *
 * @param dev Pointer to the SPI device structure.
 *
 * @note This function is enabled only if CONFIG_SPI_ASYNC or
 *       CONFIG_SPI_MCHP_INTERRUPT_DRIVEN is defined.
 */
static void spi_mchp_isr(const struct device *dev)
{
	/* Device driver runtime data */
	spi_mchp_dev_data_t *data = dev->data;

	/* Device driver configuration */
	const spi_mchp_dev_config_t *cfg = dev->config;

	/* HAL structure for SPI registers */
	const mchp_spi_reg_config_t *spi_reg_cfg = &cfg->reg_cfg;

	/* Dummy data for padding SPI transactions */
	uint8_t dummy_data = 0U;

	/* SPI status flag for last byte. */
	bool last_byte = false;

	/* Transmit and receive data placeholders */
	uint8_t tx_data = 0U;
	uint8_t rx_data = 0U;

#if (CONFIG_SPI_SLAVE)
	uint8_t intFlag = spi_reg_cfg->regs->SPIS.SERCOM_INTFLAG;
	static bool transaction_complete;

	if (spi_context_is_slave(&data->ctx) == true) {
		if ((spi_context_tx_buf_on(&data->ctx) == true) ||
		    (spi_context_rx_buf_on(&data->ctx) == true)) {
			transaction_complete = false;
		}

		/* Check if data empty bit is set*/
		if (spi_slave_is_data_empty(spi_reg_cfg) == true) {
			tx_data = *(uint8_t *)(data->ctx.tx_buf);
			if (spi_slave_is_tx_comp(spi_reg_cfg) == true) {
				intFlag = (uint8_t)SERCOM_SPIS_INTFLAG_TXC_Msk;
			}
			spi_slave_write_data(spi_reg_cfg, tx_data);
			if (spi_context_tx_on(&data->ctx) == true) {
				spi_context_update_tx(&data->ctx, 1, 1);
			} else {
				/* Disable DRE interrupt. The last byte sent by the master will be
				 * shifted out automatically
				 */
				spi_slave_disable_dre_int(spi_reg_cfg);
			}
		}

		/* Check if slave select bit is set*/
		if (spi_slave_select_line(spi_reg_cfg) == true) {
			spi_slave_clr_slave_select_line(spi_reg_cfg);
			spi_slave_enable_txc_interrupt(spi_reg_cfg);
		}

		/* Check if buffer overflow error bit is set*/
		if (spi_slave_buf_overflow(spi_reg_cfg)) {

			/* Clear the status register */
			spi_slave_clr_buf_overflow(spi_reg_cfg);

			/* Flush out the received data until RXC flag is set */
			spi_slave_clr_data(spi_reg_cfg);

			/* Clear the Error Interrupt Flag */
			spi_slave_clr_error_int_flag(spi_reg_cfg);
		}

		/* Check if data is available in the receive buffer */
		if (spi_slave_is_rx_comp(spi_reg_cfg) == true) {
			rx_data = spi_slave_read_data(spi_reg_cfg);
			if (spi_context_rx_buf_on(&data->ctx) == true) {
				*(uint8_t *)(data->ctx.rx_buf) = rx_data;
				spi_context_update_rx(&data->ctx, 1, 1);
			}
		}

		if ((intFlag & SERCOM_SPIS_INTFLAG_TXC_Msk) == SERCOM_SPIS_INTFLAG_TXC_Msk) {
			intFlag = 0;
			spi_slave_clr_tx_comp_flag(spi_reg_cfg);
			if ((spi_context_rx_on(&data->ctx) == false) &&
			    (spi_context_tx_on(&data->ctx) == false)) {
				spi_slave_disable_interrupts(spi_reg_cfg);
				spi_slave_clr_interrupts(spi_reg_cfg);
				/* Release the semaphore to unblock waiting threads */
				if (transaction_complete == false) {
					spi_context_complete(&data->ctx, dev, 0);
					transaction_complete = true;
				}
			}
		}
	}
#endif

	if (spi_context_is_slave(&data->ctx) == false) {
		/* Check if the transmit buffer is empty and send the next byte */
		if (spi_is_interrupt_set(spi_reg_cfg) == true) {
			/* Check if data is available in the receive buffer */
			if (spi_is_rx_comp(spi_reg_cfg) == true) {
				if (spi_context_rx_buf_on(&data->ctx) == true) {
					rx_data = spi_read_data(spi_reg_cfg);
					*(uint8_t *)(data->ctx.rx_buf) = rx_data;
					spi_context_update_rx(&data->ctx, 1, 1);
				}
			}
			if (spi_is_data_empty(spi_reg_cfg) == true) {
				spi_disable_data_empty_interrupt(spi_reg_cfg);
				if (spi_context_tx_on(&data->ctx) == true) {
					tx_data = *(uint8_t *)(data->ctx.tx_buf);
					spi_write_data(spi_reg_cfg, tx_data);
					spi_context_update_tx(&data->ctx, 1, 1);
				} else if (data->dummysize > 0) {
					spi_write_data(spi_reg_cfg, dummy_data);
					data->dummysize--;
				} else {
					/* Do Nothing */
				}
				if ((data->dummysize == 0) &&
				    (spi_context_tx_on(&data->ctx) == false)) {
					last_byte = true;
				} else if (spi_context_rx_on(&data->ctx) == false) {
					spi_enable_data_empty_interrupt(spi_reg_cfg);
					spi_disable_rxc_interrupt(spi_reg_cfg);
				} else {
					/* Do Nothing */
				}
			}
			if ((spi_is_tx_comp(spi_reg_cfg) == true) && (last_byte == true)) {
				if (spi_context_rx_on(&data->ctx) == false) {
					spi_disable_rxc_interrupt(spi_reg_cfg);
					spi_disable_txc_interrupt(spi_reg_cfg);
					spi_disable_data_empty_interrupt(spi_reg_cfg);
					last_byte = false;
					if (spi_context_is_slave(&data->ctx) == false) {
						/* Control chip select for SPI slave mode */
						spi_context_cs_control(&data->ctx, false);
					}
					/* Release the semaphore to unblock waiting threads */
					spi_context_complete(&data->ctx, dev, 0);
				}
			}
			if (last_byte == true) {
				spi_enable_txc_interrupt(spi_reg_cfg);
			}
		}
	}
}
#endif /* CONFIG_SPI_ASYNC || CONFIG_SPI_MCHP_INTERRUPT_DRIVEN */

/**
 * @brief Initialize the Microchip SPI controller.
 *
 * This function configures the SPI hardware by enabling the clock,
 * disabling interrupts, setting the pin control state, and ensuring
 * the SPI device is ready for transactions.
 *
 * The actual SPI configuration and enabling occur during transceive calls.
 *
 * @param dev Pointer to the SPI device structure.
 *
 * @return 0 on success, negative error code on failure.
 */
static int spi_mchp_init(const struct device *dev)
{
	int err;
	const spi_mchp_dev_config_t *cfg = dev->config;
	const mchp_spi_reg_config_t *spi_reg_cfg = &cfg->reg_cfg;
	spi_mchp_dev_data_t *const data = dev->data;

	do {
		/* Enable the SPI clock */
		err = clock_control_on(cfg->spi_clock.clock_dev, cfg->spi_clock.gclk_sys);

		if ((err < 0) && (err != -EALREADY)) {
			LOG_ERR("Failed to enable the gclk_sys for SPI: %d", err);
			break;
		}

#if defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ2) ||                                            \
	defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ6)
		ENABLE_SPI_MODULE(spi_reg_cfg->regs);
#else
		err = clock_control_on(cfg->spi_clock.clock_dev, cfg->spi_clock.mclk_sys);

		if ((err < 0) && (err != -EALREADY)) {
			LOG_ERR("Failed to enable the mclk_sys for SPI: %d", err);
			break;
		}
#endif

		/* Disable all SPI interrupts initially */
		spi_disable_interrupts(spi_reg_cfg);

		/* Apply the default pin configuration */
		err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (err < 0) {
			LOG_ERR("pinctrl_apply_state Failed for SPI: %d", err);
			break;
		}

		/* Ensure the SPI context is unlocked */
		spi_context_unlock_unconditionally(&data->ctx);
	} while (0);

	err = (err == -EALREADY) ? 0 : err;

	return err;
}

/******************************************************************************
 * @brief Zephyr driver instance creation
 *****************************************************************************/
/**
 * @brief SPI device API structure for Microchip SPI driver.
 *
 * This structure defines the function pointers for SPI operations,
 * including synchronous and optional asynchronous transceive functions.
 */
static DEVICE_API(spi, spi_mchp_api) = {
	/* Synchronous transceive function */
	.transceive = spi_mchp_transceive_sync,

#if CONFIG_SPI_ASYNC
	/* Asynchronous transceive function */
	.transceive_async = spi_mchp_transceive_async,
#endif

#if CONFIG_SPI_RTIO
	/* RTIO-based IO submission */
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif

	/* Release the SPI bus */
	.release = spi_mchp_release,
};

/**
 * @brief Retrieve the SPI pads configuration from the device tree.
 *
 * @param n Instance number from the device tree.
 * @return Configured SERCOM SPI pads.
 */
#define SPI_MCHP_SERCOM_PADS(n)                                                                    \
	SERCOM_SPIM_CTRLA_DIPO(DT_INST_PROP(n, dipo)) |                                            \
		SERCOM_SPIM_CTRLA_DOPO(DT_INST_PROP(n, dopo))

/**
 * @brief Define peripheral IP-specific features
 * based on the selected configuration.
 *
 * If SPI is configured for asynchronous operation with DMA,
 * additional DMA parameters are included.
 * Otherwise, only standard register addresses
 * and clock configurations are defined.
 */
#define SPI_MCHP_REG_CFG_DEFN(n)                                                                   \
	.reg_cfg.regs = (sercom_registers_t *)DT_INST_REG_ADDR(n),                                 \
	.reg_cfg.pads = SPI_MCHP_SERCOM_PADS(n),

/**
 * @brief Define interrupt configuration macros based on device
 * capabilities.
 *
 * If the device has multiple interrupt indices, all of them are
 * configured. Otherwise, only a single interrupt is set up.
 */
/**
 * @brief Configure SPI interrupts for instance n.
 *
 * This function sets up the interrupt handlers for the SPI instance
 * specified by n.
 *
 * @param dev Pointer to the device structure for the driver instance.
 */
#if CONFIG_SPI_MCHP_INTERRUPT_DRIVEN || CONFIG_SPI_ASYNC
#if DT_INST_IRQ_HAS_IDX(0, 3)
#define SPI_MCHP_IRQ_HANDLER(n)                                                                    \
	static void spi_mchp_irq_config_##n(const struct device *dev)                              \
	{                                                                                          \
		MCHP_SPI_IRQ_CONNECT(n, 0);                                                        \
		MCHP_SPI_IRQ_CONNECT(n, 1);                                                        \
		MCHP_SPI_IRQ_CONNECT(n, 2);                                                        \
		MCHP_SPI_IRQ_CONNECT(n, 3);                                                        \
	}
#else
#define SPI_MCHP_IRQ_HANDLER(n)                                                                    \
	static void spi_mchp_irq_config_##n(const struct device *dev)                              \
	{                                                                                          \
		MCHP_SPI_IRQ_CONNECT(n, 0);                                                        \
	}
#endif
#else
#define SPI_MCHP_IRQ_HANDLER(n)
#endif

/* Do the peripheral clock related configuration */
#if defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ2) ||                                            \
	defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ6)
#define SPI_MCHP_CLOCK_DEFN(n)                                                                     \
	.spi_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                                 \
	.spi_clock.gclk_sys = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, subsystem))
#else
#define SPI_MCHP_CLOCK_DEFN(n)                                                                     \
	.spi_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                                 \
	.spi_clock.mclk_sys = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, subsystem)),           \
	.spi_clock.gclk_sys = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, subsystem))
#endif

/**
 * @brief Do the peripheral interrupt related configuration
 */
#if CONFIG_SPI_MCHP_INTERRUPT_DRIVEN || CONFIG_SPI_ASYNC

/**
 * @brief Macro to configure and enable SPI IRQ
 *
 * @param n Instance number
 * @param m IRQ index
 */
#define MCHP_SPI_IRQ_CONNECT(n, m)                                                                 \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, m, irq), DT_INST_IRQ_BY_IDX(n, m, priority),     \
			    spi_mchp_isr, DEVICE_DT_INST_GET(n), 0);                               \
		irq_enable(DT_INST_IRQ_BY_IDX(n, m, irq));                                         \
	} while (false)

/**
 * @brief Macro to declare an IRQ handler function
 *
 * @param n Instance number
 */
#define SPI_MCHP_IRQ_HANDLER_DECL(n) static void spi_mchp_irq_config_##n(const struct device *dev)
#define SPI_MCHP_IRQ_HANDLER_FUNC(n) .irq_config_func = spi_mchp_irq_config_##n,
#else
#define SPI_MCHP_IRQ_HANDLER_DECL(n)
#define SPI_MCHP_IRQ_HANDLER_FUNC(n)
#endif /* CONFIG_SPI_MCHP_INTERRUPT_DRIVEN || CONFIG_SPI_ASYNC */

#if CONFIG_SPI_MCHP_DMA_DRIVEN
/**
 * @brief Macro to define DMA channels for SPI
 *
 * @param n Instance number
 */
#define SPI_MCHP_DMA_CHANNELS(n)                                                                   \
	.spi_dma.dma_dev = DEVICE_DT_GET(MCHP_DT_INST_DMA_CTLR(n, tx)),                            \
	.spi_dma.tx_dma_request = MCHP_DT_INST_DMA_TRIGSRC(n, tx),                                 \
	.spi_dma.tx_dma_channel = MCHP_DT_INST_DMA_CHANNEL(n, tx),                                 \
	.spi_dma.rx_dma_request = MCHP_DT_INST_DMA_TRIGSRC(n, rx),                                 \
	.spi_dma.rx_dma_channel = MCHP_DT_INST_DMA_CHANNEL(n, rx),
#else
#define SPI_MCHP_DMA_CHANNELS(n)
#endif /* CONFIG_SPI_MCHP_DMA_DRIVEN */

/**
 * @brief Initial Configuration for the SPI Peripheral
 */

/**
 * @brief Macro to define SPI device configuration
 *
 * @param n Instance number
 */
#define SPI_MCHP_CONFIG_DEFN(n)                                                                    \
	static const spi_mchp_dev_config_t spi_mchp_config_##n = {                                 \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		SPI_MCHP_REG_CFG_DEFN(n) SPI_MCHP_IRQ_HANDLER_FUNC(n) SPI_MCHP_DMA_CHANNELS(n)     \
			SPI_MCHP_CLOCK_DEFN(n)}

/**
 * @brief Macro to initialize SPI device
 *
 * @param n Instance number
 */
#define SPI_MCHP_DEVICE_INIT(n)                                                                    \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	SPI_MCHP_IRQ_HANDLER_DECL(n);                                                              \
	SPI_MCHP_CONFIG_DEFN(n);                                                                   \
	static spi_mchp_dev_data_t spi_mchp_data_##n = {                                           \
		SPI_CONTEXT_INIT_LOCK(spi_mchp_data_##n, ctx),                                     \
		SPI_CONTEXT_INIT_SYNC(spi_mchp_data_##n, ctx),                                     \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             \
	DEVICE_DT_INST_DEFINE(n, spi_mchp_init, NULL, &spi_mchp_data_##n, &spi_mchp_config_##n,    \
			      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, &spi_mchp_api);               \
	SPI_MCHP_IRQ_HANDLER(n)

/**
 * @brief Initialize SPI devices for all compatible instances
 */
DT_INST_FOREACH_STATUS_OKAY(SPI_MCHP_DEVICE_INIT)
