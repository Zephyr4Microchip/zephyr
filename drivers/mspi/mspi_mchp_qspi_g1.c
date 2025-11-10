/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* *
 * @file mspi_mchp_qspi_g1.c
 * @brief Zephyr MSPI driver for Microchip G1 peripheral.
 *
 * This driver provides an implementation of the Zephyr MSPI API for the
 * Microchip QSPI peripheral (MSPI Group 1).
 *
 */

#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>

#include <soc.h>

#include "mspi_mchp_qspi_g1.h"

#define DT_DRV_COMPAT microchip_qspi_g1_mspi
#include <zephyr/devicetree.h>

#include <errno.h>

LOG_MODULE_REGISTER(microchip_qspi_g1_mspi, CONFIG_MSPI_LOG_LEVEL);

/* ===== Devicetree pinctrl hook for the QSPI node ===== */
#define CLOCK_NODE DT_NODELABEL(clock)

typedef void (*irq_config_func_t)(const struct device *dev);
static int mspi_mchp_qspi_init(const struct device *controller);

/* *
 * @brief Clock configuration structure for the QSPI.
 *
 * This structure contains the clock configuration parameters for the QSPI
 * peripheral.
 */
typedef struct mchp_mspi_clock {
	/* Clock driver */
	const struct device *clock_dev_apb;
	const struct device *clock_dev_ahb;
	const struct device *clock_dev_ahb2x;

	/* Main clock subsystem. */
	clock_control_subsys_t mclk_apb;
	clock_control_subsys_t mclk_ahb;
	clock_control_subsys_t mclk_ahb2x;

	/* Generic clock subsystem.(not used currently) */
	clock_control_subsys_t gclk_sys;
} mchp_mspi_clock_t;

/*
 * @brief Description of a child device connected to the QSPI peripheral.
 *
 * This structure contains the static configuration parameters for a child
 * device connected to the QSPI peripheral.
 */
typedef struct mspi_child_desc {
	uint16_t cs;     /* Chip-select index */
	bool is_spi_nor; /* JEDEC SPI-NOR compatible */
			 /* Add more fields if needed */
} mspi_child_desc_t;

/*
 * @brief A ready-to-apply config derived from DT (maps to your mspi_dev_cfg)
 */
typedef struct {
	struct mspi_dev_id id;
	struct mspi_dev_cfg cfg;
} mspi_child_cfg_t;

/* *
 * @brief Run-time data structure for the QSPI.
 *
 * This structure contains the Run-time data parameters for the QSPI
 * peripheral.
 */
typedef struct {
	const struct mspi_dev_id *owner;
	struct mspi_xfer xfer;
	int packets_left;
	int packets_done;
	mspi_callback_handler_t callback;
	struct mspi_callback_context *callback_ctx;
	bool asynchronous;
	struct k_sem lock;
} mspi_context_t;

/* *
 * @brief Device PIO transfer structure for the QSPI.
 *
 * This structure contains the Run-time data parameters for the QSPI
 * peripheral.
 */
typedef struct {
	/* Number of bytes to transfer. */
	uint32_t ui32_num_bytes;

	/* Enable scrambling. */
	bool bScrambling;

	/* Enable DQS. */
	bool dcx_enabled;

	/* Transfer direction (Transmit / Receive). */
	enum mspi_xfer_direction direction;

	/* Send device address. */
	bool send_addr;

	/* Device address length (bytes). */
	uint8_t addrlen;

	/* Device address. */
	uint32_t ui32_device_Addr;

	/* Send device instruction. */
	bool send_instr;

	/* Device instruction. */
	uint8_t ui8_device_instr;

	/* Enable turnaround between address write and data read (rx dummy). */
	bool rx_dummy_en;

	/* Enable write latency counter (tx dummy). */
	bool tx_dummy_en;

	/* Number of RX dummy cycles. */
	uint8_t rx_dummy;

	/* Number of TX dummy cycles. */
	uint8_t tx_dummy;

	/* Paired-Quad command enabled. */
	bool quad_cmd_enabled;

	/* Continuation CRMODE. */
	bool continue_crmode;

	/* Pointer to data buffer. */
	uint32_t *pui32_buffer;

} mchp_hal_mspi_pio_transfer_t;

typedef struct {
	/* SPI mode */
	void *tx_buffer;
	void *rx_buffer;
	uint8_t addrlen;
	size_t txSize;
	size_t rxSize;
	size_t dummy_size;
	size_t rx_count;
	size_t tx_count;
	size_t blocks;
	size_t pre_dummy_bytes;
	size_t rx_drop;
	bool transfer_is_busy;
} mchp_hal_spi_pio_transfer_t;

/* *
 * @brief Static configuration structure for the QSPI.
 *
 * This structure contains the static configuration parameters for the QSPI
 * peripheral.
 */
typedef struct { /* Also refer to struct mspi_ambiq_config in drivers\mspi\mspi_ambiq_ap3.c */
	mchp_qspi_reg_config_t reg_cfg;
	uint32_t reg_size;
	struct mspi_cfg mspicfg;
	const struct pinctrl_dev_config *pcfg;

	irq_config_func_t irq_config_func;
	mchp_mspi_clock_t mspi_clock;

	/* children of QSPI module */
	const mspi_child_desc_t *child_desc;
	uint16_t num_children;
	const mspi_child_cfg_t *child_cfg;
} mspi_sam_qspi_cfg_t;

/* *
 * @brief Run-time data structure for the QSPI.
 *
 * This structure contains the Run-time data parameters for the QSPI
 * peripheral.
 */
typedef struct { /* Also refer to struct mspi_ambiq_data in drivers\mspi\mspi_ambiq_ap3.c */
	/* Commented out fields currently unused */

	/* maybe add a microchip specific config struct(refer to struct mspi_ambiq_data in */
	/* drivers\mspi\mspi_ambiq_ap3.c) */
	struct mspi_dev_id *dev_id;
	struct k_mutex lock_init;
	struct k_mutex lock_dev;
	mspi_callback_handler_t cbs[MSPI_BUS_EVENT_MAX];
	struct mspi_callback_context *cb_ctxs[MSPI_BUS_EVENT_MAX];
	mspi_context_t ctx;
	/* for SPI mode */
	mchp_hal_spi_pio_transfer_t qspiObj;
	/* use clock_control_get_rate */
	/* uint32_t ahb_hz;        CLK_QSPI_AHB (divider source for SCK) */
	/* uint32_t ahb2x_hz;      CLK_QSPI2X_AHB (DDR reads) */

} mspi_sam_qspi_data_t;

/* *
 * @brief Reset qspiObj to safe defaults (clear buffer pointers, counters and busy flag).
 * @return void
 */

void qspi_obj_init(mchp_hal_spi_pio_transfer_t *qspiObj)
{
	qspiObj->tx_buffer = NULL;
	qspiObj->rx_buffer = NULL;
	qspiObj->addrlen = 0;
	qspiObj->txSize = 0;
	qspiObj->rxSize = 0;
	qspiObj->dummy_size = 0;
	qspiObj->rx_count = 0;
	qspiObj->tx_count = 0;
	qspiObj->blocks = 0;
	qspiObj->transfer_is_busy = false;
}

static int qspi_set_enabled(qspi_registers_t *q, bool enable)
{
	uint32_t want = 0;

	if (enable == true) {
		want = QSPI_STATUS_ENABLE_Msk;
		q->QSPI_CTRLA |= QSPI_STATUS_ENABLE_Msk;
	} else {
		want = 0;
		q->QSPI_CTRLA &= ~QSPI_CTRLA_ENABLE_Msk;
	}

	if (!WAIT_FOR(((q->QSPI_STATUS & QSPI_STATUS_ENABLE_Msk) == want), TIMEOUT_VALUE_US,
		      k_busy_wait(DELAY_US))) {
		LOG_ERR("QSPI %s flag setting timed out", enable ? "enable" : "disable");
		return -ETIMEDOUT;
	}
	return 0;
}

/* *
 * @brief Enable the QSPI controller.
 *
 * @details
 * Sets the ENABLE bit in CTRLA and busy-waits until the STATUS
 * reflects the enabled state.
 *
 * @param q Pointer to the QSPI register block.
 */
static inline int qspi_enable(qspi_registers_t *q)
{
	return qspi_set_enabled(q, true);
}

/* *
 * @brief Disable the QSPI controller.
 *
 * @details
 * Clears the ENABLE bit in CTRLA and busy-waits until the STATUS
 * reflects the disabled state.
 *
 * @param q Pointer to the QSPI register block.
 */
static inline int qspi_disable(qspi_registers_t *q)
{
	return qspi_set_enabled(q, false);
}

/* *
 * @brief Find if child node is on SPI mode
 * or serial mode
 *
 * @param children Pointer to the MSPI child descriptor structure.
 * @param dev_idx Chip select index/register address of child node
 * @param num_of_children total number of children for the MSPI controller
 *
 * @return 0 on success, negative error on failure
 */
static bool find_serial_mode_of_child(const mspi_child_desc_t *children, uint16_t dev_idx,
				      uint8_t num_of_children)
{
	for (uint8_t i = 0; i < num_of_children; i++) {
		if (children[i].cs == dev_idx) {
			return children[i].is_spi_nor;
		}
	}
	LOG_ERR("No child node found for dev_idx %d", dev_idx);
	return false;
}

/**
 * @brief Validate the chip-select (CE) index against controller limits.
 *
 * When enabled through the mask, checks that the device chip-select index
 * does not exceed the number of chip-select GPIOs supported by the MSPI
 * controller. This also handles the case where no external CE GPIOs are
 * defined.
 *
 * @param mask    Bitmask indicating whether CE index validation should
 *                be performed (MSPI_DEVICE_CONFIG_CE_NUM).
 * @param dev_id  Pointer to device identifier containing the chip-select index.
 * @param ccfg    Pointer to controller configuration providing the number
 *                of supported CE GPIOs.
 *
 * @return 0 on success, or -EINVAL if the chip-select index is invalid.
 */
static int check_cs_index_limit(uint32_t mask, const struct mspi_dev_id *dev_id,
				const mspi_sam_qspi_cfg_t *ccfg)
{
	if ((mask & MSPI_DEVICE_CONFIG_CE_NUM) != 0) {
		/* greater than max CE lines */
		if (dev_id->dev_idx > ccfg->mspicfg.num_ce_gpios) {
			LOG_ERR("Invalid CE number");
			return -EINVAL;
		}
		/* ccfg->mspicfg.num_ce_gpios will show 0 as "ce-gpios" not present in parent node
		 */
		LOG_INF("Device id = %d, num_ce_gpios = %d", dev_id->dev_idx,
			ccfg->mspicfg.num_ce_gpios);
	}
	return 0;
}

/**
 * @brief Configure the MSPI baud rate (serial clock frequency).
 *
 * Retrieves the QSPI AHB clock frequency, validates the requested device
 * frequency, computes the corresponding baud divider, and programs the
 * QSPI BAUD register when frequency configuration is enabled.
 *
 * @param mask  Bitmask indicating whether frequency configuration should
 *              be applied (MSPI_DEVICE_CONFIG_FREQUENCY).
 * @param cfg   Pointer to device configuration containing the target
 *              MSPI clock frequency.
 * @param ccfg  Pointer to controller configuration providing clock and
 *              maximum frequency information.
 * @param q     Pointer to the QSPI hardware register block used to apply
 *              the baud rate configuration.
 *
 * @return 0 on success; a negative error code on failure:
 *         -EINVAL for invalid frequency,
 *         -ETIMEDOUT if clock rate retrieval times out,
 *         or a propagated error from the clock control driver.
 */
static int set_mspi_baud(uint32_t mask, const struct mspi_dev_cfg *cfg,
			 const mspi_sam_qspi_cfg_t *ccfg, qspi_registers_t *q)
{
	uint8_t ret = 0;
	uint8_t baud = 0;
	uint32_t ahb_clk_freq = 0;

	/* wait for QSPI_AHB to get its frequency from clock driver */
	uint32_t tries = 200;

	do {
		ret = clock_control_get_rate(DEVICE_DT_GET(CLOCK_NODE), ccfg->mspi_clock.mclk_ahb,
					     &ahb_clk_freq);
		if (ret != 0) {
			return ret;
		}
		k_sleep(K_MSEC(100));
		if (--tries == 0) {
			LOG_INF("Timed out getting frequency of clock AHB");
			return -ETIMEDOUT;
		}
	} while (ahb_clk_freq == 0);

	if ((mask & MSPI_DEVICE_CONFIG_FREQUENCY) != 0) {
		if (cfg->freq == 0 || cfg->freq > ccfg->mspicfg.max_freq) {
			LOG_ERR("Invalid frequency");
			return -EINVAL;
		}
		/* get baud rate */
		baud = (uint8_t)((ahb_clk_freq / cfg->freq) - 1);
		/* even if it is ahb2x its clock freq is 120Mhz */
		LOG_INF("QSPI controller clock freq = %d divided by 2", ahb_clk_freq);
		LOG_INF("BAUD = %u", baud);
		LOG_INF("QSPI device freq set to %d", cfg->freq);
		qspi_set_baud(q, baud);
	}
	return 0;
}

/**
 * @brief Configure the MSPI data bus width (I/O mode).
 *
 * Applies the requested MSPI I/O mode when enabled through the mask,
 * translates it to the controller-specific width encoding, and programs
 * the WIDTH field in the QSPI INSTRFRAME register. Unsupported I/O modes
 * return an error.
 *
 * @param mask  Bitmask indicating whether I/O mode configuration should
 *              be applied (MSPI_DEVICE_CONFIG_IO_MODE).
 * @param cfg   Pointer to device configuration containing the MSPI I/O mode.
 * @param q     Pointer to the QSPI hardware register block used to apply
 *              the width configuration.
 *
 * @return 0 on success, or -EINVAL if the requested I/O mode is unsupported.
 */
static int set_mspi_width(uint32_t mask, const struct mspi_dev_cfg *cfg, qspi_registers_t *q)
{
	uint8_t width = 0;

	if ((mask & MSPI_DEVICE_CONFIG_IO_MODE) != 0) {
		switch (cfg->io_mode) {
		case MSPI_IO_MODE_SINGLE:
			width = 0;
			break;

		case MSPI_IO_MODE_DUAL_1_1_2:
			width = 1;
			break;

		case MSPI_IO_MODE_QUAD_1_1_4:
			width = 2;
			break;

		case MSPI_IO_MODE_DUAL_1_2_2:
			width = 3;
			break;

		case MSPI_IO_MODE_QUAD_1_4_4:
			width = 4;
			break;

		case MSPI_IO_MODE_DUAL:
			width = 5;
			break;

		case MSPI_IO_MODE_QUAD:
			width = 6;
			break;

		case MSPI_IO_MODE_OCTAL:
		case MSPI_IO_MODE_OCTAL_1_1_8:
		case MSPI_IO_MODE_OCTAL_1_8_8:
		case MSPI_IO_MODE_HEX:
		case MSPI_IO_MODE_HEX_8_8_16:
		case MSPI_IO_MODE_HEX_8_16_16:
		default:
			LOG_ERR("MCU does not support above quad IO mode");
			return -EINVAL;
		}
		LOG_INF("IO mode set by user = %d", cfg->io_mode);
		LOG_INF("Width bits set to %d", width);
		/* set WIDTH bitfields in INSTRFRAME register */
		set_width_bits(q, width);
	}
	return 0;
}

/**
 * @brief Configure QSPI clock polarity and phase (CPOL/CPHA).
 *
 * Applies the CPP (Clock Polarity/Phase) mode from the device
 * configuration to the QSPI_BAUD register when the corresponding
 * mask bit (MSPI_DEVICE_CONFIG_CPP) is set.
 *
 * @param mask  Bitmask indicating which MSPI configuration fields
 *              should be applied (e.g., MSPI_DEVICE_CONFIG_CPP).
 * @param cfg   Pointer to device configuration containing the CPP mode.
 * @param q     Pointer to QSPI register block (qspi_registers_t).
 *
 * @return 0 on success, or -EINVAL if an invalid CPP mode is provided.
 */
static int set_clock_pol_pha(uint32_t mask, const struct mspi_dev_cfg *cfg, qspi_registers_t *q)
{
	if ((mask & MSPI_DEVICE_CONFIG_CPP) != 0) {
		switch (cfg->cpp) {
		/* Polarity = 0,Phase = 0 */
		case MSPI_CPP_MODE_0:
			q->QSPI_BAUD |=
				(q->QSPI_BAUD & ~(QSPI_BAUD_CPOL_Msk | QSPI_BAUD_CPHA_Msk)) |
				(QSPI_BAUD_CPOL(0) | QSPI_BAUD_CPHA(0));
			break;

		/* Polarity = 0,Phase = 1 */
		case MSPI_CPP_MODE_1:
			q->QSPI_BAUD |=
				(q->QSPI_BAUD & ~(QSPI_BAUD_CPOL_Msk | QSPI_BAUD_CPHA_Msk)) |
				(QSPI_BAUD_CPOL(0) | QSPI_BAUD_CPHA(1));
			break;

		/* Polarity = 1,Phase = 0 */
		case MSPI_CPP_MODE_2:
			q->QSPI_BAUD |=
				(q->QSPI_BAUD & ~(QSPI_BAUD_CPOL_Msk | QSPI_BAUD_CPHA_Msk)) |
				(QSPI_BAUD_CPOL(1) | QSPI_BAUD_CPHA(0));
			break;

		/* Polarity = 1,Phase = 1 */
		case MSPI_CPP_MODE_3:
			q->QSPI_BAUD |=
				(q->QSPI_BAUD & ~(QSPI_BAUD_CPOL_Msk | QSPI_BAUD_CPHA_Msk)) |
				(QSPI_BAUD_CPOL(1) | QSPI_BAUD_CPHA(1));
			break;

		default:
			LOG_ERR("Invalid CPP mode");
			return -EINVAL;
		}
		LOG_INF("CPP mode set by user = %d", cfg->cpp);
	}
	return 0;
}

/**
 * @brief Validate and report the chip-select (CE) polarity configuration.
 *
 * Checks whether CE polarity should be applied based on the mask and
 * verifies that the provided polarity value is valid (0 = active low,
 * 1 = active high).
 *
 * @param mask  Bitmask indicating which MSPI configuration fields are active
 *              (e.g., MSPI_DEVICE_CONFIG_CE_POL).
 * @param cfg   Pointer to device configuration containing the CE polarity value.
 *
 * @return 0 on success, or -EINVAL if the CE polarity is invalid.
 */
static int check_cs_polarity(uint32_t mask, const struct mspi_dev_cfg *cfg)
{
	if ((mask & MSPI_DEVICE_CONFIG_CE_POL) != 0) {
		if (cfg->ce_polarity > 1) {
			LOG_ERR("Invalid CE polarity");
			return -EINVAL;
		}
		/* active low */
		if (cfg->ce_polarity == 0) {
			LOG_INF("CE polarity active low");
		}
		/* active high */
		else {
			LOG_INF("CE polarity active high");
		}
		LOG_INF("CE polarity set by user = %d", cfg->ce_polarity);
	}
	return 0;
}

/**
 * @brief Check whether DQS mode is requested and report unsupported usage.
 *
 * Validates the configuration mask and returns an error if DQS
 * (Data Strobe) mode is requested, as this MCU does not support it.
 *
 * @param mask  Bitmask indicating which MSPI configuration options
 *              are being applied (e.g., MSPI_DEVICE_CONFIG_DQS).
 *
 * @return 0 if DQS is not requested, or -EINVAL if DQS mode is unsupported.
 */
static int check_dqs_support(uint32_t mask)
{
	if (mask & MSPI_DEVICE_CONFIG_DQS) {
		LOG_INF("DQS mode not supported by MCU");
		return -EINVAL;
	}
	return 0;
}

/**
 * @brief Validate and apply TX/RX dummy cycle configuration.
 *
 * Verifies that TX and RX dummy cycle values are within the supported
 * 5-bit range (0–31). When enabled through the mask, the function programs
 * the dummy cycle bits in the QSPI register block for both TX and RX paths.
 *
 * @param mask  Bitmask indicating which dummy-cycle configuration fields
 *              should be applied (MSPI_DEVICE_CONFIG_TX_DUMMY and/or
 *              MSPI_DEVICE_CONFIG_RX_DUMMY).
 * @param cfg   Pointer to device configuration containing TX and RX
 *              dummy cycle values.
 * @param q     Pointer to the QSPI hardware register block used to apply
 *              the dummy cycle settings.
 *
 * @return 0 on success, or -EINVAL if a dummy cycle value exceeds 5-bit range.
 */
static int check_rx_tx_dummy(uint32_t mask, const struct mspi_dev_cfg *cfg, qspi_registers_t *q)
{
	if ((mask & MSPI_DEVICE_CONFIG_TX_DUMMY) != 0) {
		/* max 5-bit value */
		if (cfg->tx_dummy > 0x1F) {
			LOG_ERR("Invalid TX dummy cycles");
			return -EINVAL;
		}
		/* same bitfields for both tx and rx dummy cycles(tx dummy value can be accessed */
		/* anytime from .cfg) */
		set_dummy_bits(q, cfg->tx_dummy);
		LOG_INF("TX dummy cycles set by user = %d", cfg->tx_dummy);
	}
	if ((mask & MSPI_DEVICE_CONFIG_RX_DUMMY) != 0) {
		/* max 5-bit value */
		if (cfg->rx_dummy > 0x1F) {
			LOG_ERR("Invalid RX dummy cycles");
			return -EINVAL;
		}
		/* same bitfields for both tx and rx dummy cycles(rx dummy value can be accessed */
		/* anytime from .cfg) */
		set_dummy_bits(q, cfg->rx_dummy);
		LOG_INF("RX dummy cycles set by user = %d", cfg->rx_dummy);
	}
	return 0;
}

/**
 * @brief Validate read and write command opcodes.
 *
 * Ensures that the configured read and write commands are valid 1-byte
 * opcodes when the corresponding mask bits are set. Logs the selected
 * commands after successful validation.
 *
 * @param mask  Bitmask indicating which opcode fields should be validated
 *              (MSPI_DEVICE_CONFIG_READ_CMD and/or
 *               MSPI_DEVICE_CONFIG_WRITE_CMD).
 * @param cfg   Pointer to device configuration containing the read and
 *              write command values.
 *
 * @return 0 on success, or -EINVAL if any opcode exceeds the 1-byte range.
 */
static int check_read_write_cmd(uint32_t mask, const struct mspi_dev_cfg *cfg)
{
	/* ******Read Command input sanity check****** */
	if ((mask & MSPI_DEVICE_CONFIG_READ_CMD) != 0) {
		/* 1 byte command */
		if (cfg->read_cmd > 0xFF) {
			LOG_ERR("Invalid read command");
			return -EINVAL;
		}
		LOG_INF("Read command set by user = 0x%X", cfg->read_cmd);
	}

	/* ******Write Command input sanity check****** */
	if ((mask & MSPI_DEVICE_CONFIG_WRITE_CMD) != 0) {
		/* 1 byte command */
		if (cfg->write_cmd > 0xFF) {
			LOG_ERR("Invalid write command");
			return -EINVAL;
		}
		LOG_INF("Write command set by user = 0x%X", cfg->write_cmd);
	}
	return 0;
}

/**
 * @brief Validate the SPI command length.
 *
 * Checks whether the command length configuration is enabled in the mask
 * and verifies that the provided length does not exceed one byte.
 *
 * @param mask  Bitmask indicating whether command-length configuration
 *              should be applied (MSPI_DEVICE_CONFIG_CMD_LEN).
 * @param cfg   Pointer to device configuration containing the command length.
 *
 * @return 0 on success, or -EINVAL if the command length is greater than 1 byte.
 */
static int check_cmd_length(uint32_t mask, const struct mspi_dev_cfg *cfg)
{
	if ((mask & MSPI_DEVICE_CONFIG_CMD_LEN) != 0) {
		/* max 1 byte */
		if (cfg->cmd_length > 1) {
			LOG_ERR("Invalid command length");
			return -EINVAL;
		}
		LOG_INF("Command length set by user = %d byte(s)", cfg->cmd_length);
	}
	return 0;
}

/**
 * @brief Validate and apply the SPI address length configuration.
 *
 * Ensures that the address length is either 3 or 4 bytes when the
 * corresponding mask bit is set. Converts the byte value into the
 * controller-specific format and programs the ADDRLEN field in the
 * INSTRFRAME register.
 *
 * @param mask  Bitmask indicating whether address-length configuration
 *              should be applied (MSPI_DEVICE_CONFIG_ADDR_LEN).
 * @param cfg   Pointer to device configuration containing the address length
 *              in bytes.
 * @param q     Pointer to the QSPI hardware register block used to update
 *              the address-length field.
 *
 * @return 0 on success, or -EINVAL if the address length is invalid.
 */
static int check_addr_length(uint32_t mask, const struct mspi_dev_cfg *cfg, qspi_registers_t *q)
{
	if ((mask & MSPI_DEVICE_CONFIG_ADDR_LEN) != 0) {
		/* max 4 bytes,normally 3 or 4 bytes address length(when only instruction to be */
		/* sent, then address length = 0) */
		if (cfg->addr_length < 3 || cfg->addr_length > 4) {
			LOG_ERR("Invalid address length- Address length can be either 3 or 4 "
				"bytes");
		}
		int addr_len = cfg->addr_length;

		if (cfg->addr_length == 3) {
			addr_len = 0;
		} else if (cfg->addr_length == 4) {
			addr_len = 1;
		}
		/* configure ADDRLEN bitfield in INSTRFRAME register */
		set_address_len(q, addr_len);
		LOG_INF("Address length set by user = %d byte(s)", cfg->addr_length);
	}
	return 0;
}

/**
 * @brief Validate the configured memory boundary.
 *
 * When enabled through the mask, verifies that the memory boundary falls
 * within the supported valid range (0 to 16 MB). Logs the configured
 * boundary on success.
 *
 * @param mask  Bitmask indicating whether memory-boundary configuration
 *              should be applied (MSPI_DEVICE_CONFIG_MEM_BOUND).
 * @param cfg   Pointer to device configuration containing the memory
 *              boundary value in bytes.
 *
 * @return 0 on success, or -EINVAL if the boundary is outside the valid range.
 */
static int check_mem_bound(uint32_t mask, const struct mspi_dev_cfg *cfg)
{
	if ((mask & MSPI_DEVICE_CONFIG_MEM_BOUND) != 0) {
		/* max 16MB */
		if (cfg->mem_boundary < 0 || cfg->mem_boundary > 0x01000000) {
			LOG_ERR("Invalid memory boundary");
			return -EINVAL;
		}
		LOG_INF("Memory boundary set by user = %d bytes", cfg->mem_boundary);
	}
	return 0;
}

/* *
 * @brief MSPI interrupt handler for the Microchip QSPI peripheral - services QSPI INTFLAG events
 *        (RXC/DRE/TXC/INSTREND/ERROR), moves data between hardware and packet buffers, updates
 *        qspiObj transfer state, and calls the transfer completion callback when done.
 *
 * @param arg Pointer to the Zephyr device instance (const struct device *). The handler casts
 *            this to access the instance config and runtime data.
 */

static void mspi_mchp_isr(const void *arg)
{
	const struct device *dev = arg;
	const mspi_sam_qspi_cfg_t *cfg = dev->config;
	mspi_sam_qspi_data_t *data = dev->data;

	mspi_context_t *ctx = &data->ctx;
	const struct mspi_xfer *xfer = &data->ctx.xfer;

	/* using this packet fields to process further */
	const struct mspi_xfer_packet *packet = ctx->callback_ctx->mspi_evt.evt_data.packet;

	qspi_registers_t *q = cfg->reg_cfg.regs;

	bool send_addr = (xfer->addr_length != 0);
	uint32_t flags = q->QSPI_INTFLAG;

	/* ***SPI implementation variables*** */
	uint32_t receivedData;
	static bool isLastByteTransferInProgress;
	mchp_hal_spi_pio_transfer_t *qspiObj = &data->qspiObj;

	size_t rx_count = qspiObj->rx_count;
	size_t tx_count = qspiObj->tx_count;

	uint16_t pre_dummy_bytes_tmp = 0;

	pre_dummy_bytes_tmp = qspiObj->pre_dummy_bytes;
	/* ***SPI implementation variables*** */

	if ((q->QSPI_CTRLB & QSPI_CTRLB_MODE_Msk) != 0) {
		/* One shot: mask INSTREND immediately */
		if ((flags & QSPI_INTFLAG_INSTREND_Msk) != 0) {
			/* clear INSTREND */
			q->QSPI_INTFLAG = flags;
		}

		/* if event type matches call the callback function */
		if (ctx->callback_ctx->mspi_evt.evt_type == MSPI_BUS_XFER_COMPLETE) {
			/* clear status field so that next call does not get stuck in its while loop
			 */
			ctx->callback_ctx->mspi_evt.evt_data.status = 0;
			ctx->callback(ctx->callback_ctx, send_addr);
		}
	} else {
		/* clear error flags if any */
		if (q->QSPI_INTFLAG & QSPI_INTFLAG_ERROR_Msk) {
			/* write 1 to clear */
			q->QSPI_INTFLAG = QSPI_INTFLAG_ERROR_Msk;
			/* flush once */
			(void)q->QSPI_RXDATA;
		}

		/* wait for received data to come in RXDATA from shifter */
		if ((q->QSPI_INTFLAG & QSPI_INTFLAG_RXC_Msk) == QSPI_INTFLAG_RXC_Msk) {
			/* reading RXDATA clears RXC flag */
			receivedData =
				((q->QSPI_RXDATA & QSPI_RXDATA_DATA_Msk) >> QSPI_RXDATA_DATA_Pos);

			if (qspiObj->rx_drop != 0) {
				qspiObj->rx_drop--;
			} else {
				if ((packet->dir == MSPI_RX) && (rx_count < packet->num_bytes)) {
					packet->data_buf[rx_count] = receivedData;
					rx_count++;
					qspiObj->rx_count = rx_count;
				}
			}
		}

		/* wait for data to move from TXDATA to shifter */
		if ((q->QSPI_INTFLAG & QSPI_INTFLAG_DRE_Msk) == QSPI_INTFLAG_DRE_Msk) {
			/* Disable the DRE interrupt(so that it doesn't keep firing again and again
			 * after DRE = 1 when opcode leaves TXDATA for shifter). This will be
			 * enabled back if more than one byte is pending to be transmitted
			 */
			q->QSPI_INTENCLR = QSPI_INTENCLR_DRE_Msk;

			if (qspiObj->addrlen > 0) {
				q->QSPI_TXDATA = (uint8_t)(((packet->address &
							     (0x000000FF
							      << (8 * (qspiObj->addrlen - 1))))) >>
							   (8 * (qspiObj->addrlen - 1)));
				qspiObj->addrlen--;
				qspiObj->blocks++;
			}
			/* for dummy cycles if needed before read in or clock in data */
			else if (pre_dummy_bytes_tmp > 0) {
				q->QSPI_TXDATA = (uint8_t)0xFF;
				pre_dummy_bytes_tmp--;
				qspiObj->pre_dummy_bytes = pre_dummy_bytes_tmp;
				qspiObj->blocks++;
			} else if (qspiObj->dummy_size > 0) {
				q->QSPI_TXDATA = (uint8_t)0xFF;
				qspiObj->dummy_size--;
				qspiObj->blocks++;
			} else if ((tx_count < packet->num_bytes) && (packet->dir == MSPI_TX)) {
				q->QSPI_TXDATA = packet->data_buf[tx_count];
				tx_count++;
				qspiObj->blocks++;
			} else {
				/* nothing to send out of TXDATA */
			}
			qspiObj->tx_count = tx_count;

			if (qspiObj->blocks != xfer->cmd_length + xfer->addr_length +
						       qspiObj->pre_dummy_bytes +
						       packet->num_bytes) {
				/* keep DRE enabled till sending out of TXDATA */
				q->QSPI_INTENSET = QSPI_INTENSET_RXC_Msk | QSPI_INTENSET_DRE_Msk;
			}

			if ((qspiObj->blocks == xfer->cmd_length + xfer->addr_length +
							qspiObj->pre_dummy_bytes +
							packet->num_bytes) &&
			    (isLastByteTransferInProgress != true)) {
				/* We've just queued the last real byte to send, and there are no
				 * dummy clocks left. The hardware may still be shifting that last
				 * byte out on the wire, so the transfer isn't truly finished yet.
				 *
				 * So the code:
				 *
				 * 1) Marks that we're in the "last byte is in flight" phase
				 * (isLastByteTransferInProgress = true).
				 *
				 * 2) (Important) It does not immediately enable the "TX complete"
				 * interrupt here; that's armed later so we don't accidentally fire
				 * the completion callback too early (a race that can happen at high
				 * baud rates). When the hardware finally reports TXC (shifter truly
				 * empty), the ISR runs once more, then we clean up and call the
				 * user's callback.
				 */

				/* At higher baud rates, the data in the shift register can be
				 * shifted out and TXC flag can get set resulting in a
				 * callback been given to the application with the SPI interrupt
				 * pending with the application. This will then result in the
				 * interrupt handler being called again with nothing to transmit.
				 * To avoid this, a software flag is set, but
				 * the TXC interrupt is not enabled until the very end.
				 */
				isLastByteTransferInProgress = true;

				/* Set Last transfer to deassert CS after the last byte written in
				 * TDR has been transferred and TXC flag is set.
				 */
				/* Setting Last transfer just after writing last byte tends to
				 * corrupt last byte read(Only tested with CSMODE = 1(LASTXFER))
				 */
				/* QSPI_EndTransfer(q); */
			}
			/* if MSPI_RX && blocks == cmd_length + addr_length + pre-dummy + dummy +
			 * data
			 */
			else if (((rx_count == packet->num_bytes) && (packet->dir == MSPI_RX)) ||
				 ((qspiObj->rx_drop == 0) && (packet->dir == MSPI_TX))) {
				/* Enable DRE interrupt as all the requested bytes are received
				 * and can now make use of the internal transmit shift register.
				 */

				/* 1) We already got all the bytes the user wanted to read (rx_count
				 * == rxSize). So stop listening for more receive interrupts:
				 * INTENCLR_RXC.
				 *
				 * 2) But the SPI transaction isn't truly over yet.
				 * In SPI, you only receive while you keep sending clocks. Even
				 * after the last wanted RX byte arrives, the controller may still
				 * need to:
				 *
				 * a) push any remaining dummy bytes (to keep clocks going),
				 * b) let the last byte finish shifting out (basically give
				 * some more time for bytes to get out of shifter),
				 * c) and then reach the hardware's TX complete (TXC) state so
				 * CS can be cleanly released.
				 */
				q->QSPI_INTENCLR = QSPI_INTENCLR_RXC_Msk;
				q->QSPI_INTENSET = QSPI_INTENSET_DRE_Msk;
			} else {
				/* No action required */
			}
		}

		/* See if Exchange is complete */
		if (((q->QSPI_INTFLAG & QSPI_INTFLAG_TXC_Msk) == QSPI_INTFLAG_TXC_Msk) &&
		    (isLastByteTransferInProgress == true)) {
			if (((rx_count == packet->num_bytes) && (packet->dir == MSPI_RX)) ||
			    ((qspiObj->blocks == xfer->cmd_length + xfer->addr_length +
							 qspiObj->pre_dummy_bytes +
							 packet->num_bytes) &&
			     (packet->dir == MSPI_TX))) {
				qspiObj->transfer_is_busy = false;

				/* Set Last transfer to deassert CS after the last byte written in
				 * TDR has been transferred.
				 */
				QSPI_EndTransfer(q);
				/* Disable DRE, RXC and TXC interrupts so isr is not fired after
				 * transfer is complete
				 */
				q->QSPI_INTENCLR = QSPI_INTENCLR_DRE_Msk | QSPI_INTENCLR_RXC_Msk |
						   QSPI_INTENCLR_TXC_Msk;

				isLastByteTransferInProgress = false;
				if (ctx->callback_ctx->mspi_evt.evt_type ==
				    MSPI_BUS_XFER_COMPLETE) {
					/* Call the below callback function, if desired */
					/* ctx->callback(ctx->callback_ctx); */
					/* Callback area,status set to 0 */
					ctx->callback_ctx->mspi_evt.evt_data.status = 0;
				}
			}
		}

		if (isLastByteTransferInProgress == true) {
			/* For the last byte transfer, the TDRE interrupt is already disabled.
			 * Enable TXC interrupt to ensure no data is present in the shift
			 * register before application callback is called.
			 */
			q->QSPI_INTENSET = QSPI_INTENSET_TXC_Msk;
		}
	}
}

/* *
 * @brief Disable/De-initialize features of QSPI.
 * This function Disable QSPI moduke
 * and in future may disable power control
 *
 * @param controller Pointer to the MSPI device structure.
 *
 * @return 0 on success, negative error on failure
 */
static int qspi_deinit(const struct device *controller)
{
	const mspi_sam_qspi_cfg_t *cfg = controller->config;
	int ret = 0;

	qspi_registers_t *q = cfg->reg_cfg.regs;

	/* disable QSPI(try to call fn for this and use data->mspiHandle as argument in that) */
	ret = qspi_disable(q);
	if (ret != 0) {
		return ret;
	}
	LOG_INF("QSPI disabled");

	/* power control and de-init here -->refer to static int mspi_ambiq_deinit() */
	return 0;
}

/* *
 * @brief Register an MSPI bus event callback for a controller / target pair.
 *
 * @details
 * Associates a user callback with the specified MSPI controller and device ID
 * so it is invoked when the given bus event type occurs (e.g., transfer
 * completion, error, or status change).
 *
 * The driver stores @p cb and the
 * optional @p ctx pointer and will pass them back to the handler when the
 * event fires. Handlers may be called from ISR or driver thread context
 * depending on the backend; they must be fast and non-blocking. If a callback
 * was previously registered for the same {controller, dev_id, evt_type}, the
 * new one replaces it. Returns 0 on success or a negative errno on error
 * (e.g., -EINVAL for bad arguments or unknown event, -ENOSPC if the callback
 * table is full, -ENOTSUP if callbacks are not supported for @p evt_type).
 *
 * @param controller  MSPI controller device handle (`struct device`).
 * @param dev_id      Target device identifier on the MSPI bus.
 * @param evt_type    Event to subscribe to (e.g., transfer complete, error).
 * @param cb          Callback function to invoke on @p evt_type (must be non-NULL).
 * @param ctx         Optional user context passed back to @p cb (may be NULL).
 *
 * @return 0 on success;
 *         -ENOTSUP if callbacks are not supported for @p evt_type or backend;
 */
static int mspi_mchp_register_callback(const struct device *controller,
				       const struct mspi_dev_id *dev_id,
				       const enum mspi_bus_event evt_type,
				       mspi_callback_handler_t cb,
				       struct mspi_callback_context *ctx)
{
	mspi_sam_qspi_data_t *data = controller->data;
	int ret = 0;

	if (evt_type != MSPI_BUS_XFER_COMPLETE) {
		LOG_ERR("callback types not supported.");
		return -ENOTSUP;
	}
	if ((controller == NULL) || (controller->config == NULL) || (controller->data == NULL) ||
	    (dev_id == NULL) || (ctx == NULL)) {
		return -ENOTSUP;
	}

	/* Support other events and other callbacks as well after basic support */

	data->cbs[evt_type] = cb;
	data->cb_ctxs[evt_type] = ctx;

	return ret;
}

/* *
 * @brief Basic configuration for QSPI module.
 *
 * This function brings up QSPi module by
 * enabling clocks,
 * pin control,
 * clears any pending interrupt flags
 * disables interrupt
 * enables QSPI module
 *
 * @param spec Pointer to the MSPI device tree specification.
 *
 * @return 0 on success, negative error code on failure.
 */
/* ========== Controller-level re-init at runtime: mspi_config() ========== */
static int mspi_mchp_config(const struct mspi_dt_spec *spec)
{
	const struct mspi_cfg *config = &spec->config;
	const mspi_sam_qspi_cfg_t *cfg = spec->bus->config;
	mspi_sam_qspi_data_t *data = spec->bus->data;

	qspi_registers_t *q = cfg->reg_cfg.regs;

	bool locked = false;
	int ret = 0;

	/* Arguments sanity checks */
	if (config->channel_num > 1) {
		return -ENOTSUP;
	}
	if (config->op_mode != MSPI_OP_MODE_CONTROLLER) {
		return -ENOTSUP;
	}
	if (config->max_freq > MSPI_MCHP_CONTROLLER_MAX_FREQ) {
		return -ENOTSUP;
	}
	if (config->dqs_support == true) {
		return -ENOTSUP;
	} /* no DQS */
	if (config->duplex != MSPI_HALF_DUPLEX) {
		return -ENOTSUP;
	} /* duplex not relevant for serial memory mode */

	k_mutex_lock(&data->lock_init, K_FOREVER);
	locked = true;

	/* interrupt flags clear and disable interrupt */
	clear_all_interrupts(q);

	/* interrupt disable */
	disable_all_interrupts(q);

	if (config->re_init == true) {
		/* disable qspi and re-init*/
		ret = qspi_deinit(spec->bus);
		if (ret != 0) {
			if (locked == true) {
				k_mutex_unlock(&data->lock_init);
				LOG_ERR("Failed to de-initialize MSPI controller");
			}
			return ret;
		}
		ret = mspi_mchp_qspi_init(spec->bus);
		if (ret != 0) {
			if (locked == true) {
				k_mutex_unlock(&data->lock_init);
				LOG_ERR("Failed to re-initialize MSPI controller");
			}
			return ret;
		}
	}

	/* enable QSPI module */
	ret = qspi_enable(q);
	if (ret != 0) {
		if (locked == true) {
			k_mutex_unlock(&data->lock_init);
		}
		return ret;
	}
	LOG_INF("QSPI enabled");

	/* Connects the hardware interrupt to your ISR
	 * Enables the interrupt in the NVIC / interrupt controller
	 * (Actually calls mspi_mchp_irq_config_##inst function)
	 */
	cfg->irq_config_func(spec->bus);

	if (locked == true) {
		k_mutex_unlock(&data->lock_init);
	}
	return ret;
}

/* *
 * @brief Device(connected to QSPI) specific configuration for QSPI module.
 *
 * This function configures QSPi module by
 * setting device specific parameters such as
 * frequency/baud rate,
 * width of instruction,address,data
 * data rate,
 * clock polarity,
 * clock phase,
 * chip select polarity,
 * dummy cycles
 * read/write commands
 * command/address length
 * memory boundary
 * time to break up a transfer
 *
 * @param controller Pointer to the MSPI device structure.
 * @param dev_id Pointer to the MSPI device ID structure.
 * @param param_mask Bitmask of parameters to configure.
 * @param cfg Pointer to the MSPI device configuration structure.
 *
 * @return 0 on success, negative error code on failure.
 */
/* ========== Device specific config: mspi_dev_config() ========== */
static int mspi_mchp_dev_config(const struct device *controller, const struct mspi_dev_id *dev_id,
				const enum mspi_dev_cfg_mask param_mask,
				const struct mspi_dev_cfg *cfg)
{
	int ret = 0;
	bool locked = false;

	uint32_t mask = (uint32_t)param_mask;
	const mspi_sam_qspi_cfg_t *ccfg = controller->config;
	mspi_sam_qspi_data_t *cdata = controller->data;
	qspi_registers_t *q = ccfg->reg_cfg.regs;

	if (controller == NULL || (controller->config == NULL) || (controller->data == NULL) ||
	    (cfg == NULL) || (dev_id == NULL)) {
		LOG_ERR("Invalid parameters");
		return -EINVAL;
	}

	ret = check_cs_index_limit(mask, dev_id, ccfg);
	if (ret != 0) {
		return ret;
	}

	/* *****Set baud rate***** */
	ret = set_mspi_baud(mask, cfg, ccfg, q);
	if (ret != 0) {
		return ret;
	}

	/* ******Set WIDTH******* */
	ret = set_mspi_width(mask, cfg, q);
	if (ret != 0) {
		return ret;
	}

	/* ******Set DATA RATE****** */
	uint8_t baud = 0;
	uint32_t ahb2x_clk_freq = 0;
	uint32_t tries = 200;

	if ((mask & MSPI_DEVICE_CONFIG_DATA_RATE) != 0) {
		switch (cfg->data_rate) {
		case MSPI_DATA_RATE_SINGLE:
			clear_double_data_rate(q);
			break;

		case MSPI_DATA_RATE_S_S_D:
			/* only in serial memory mode */
			if (find_serial_mode_of_child(ccfg->child_desc, dev_id->dev_idx,
						      ccfg->num_children) == true) {
				/* CLK_QSPI2X_AHB clock must be enabled in mspi_config() before
				 * enabling the DDREN bit.
				 */
				ret = clock_control_on(DEVICE_DT_GET(CLOCK_NODE),
						       ccfg->mspi_clock.mclk_ahb2x);
				if ((ret < 0) && (ret != -EALREADY)) {
					LOG_ERR("MCLK AHB2X not ready");
					return ret;
				}

				/* wait for QSPI_AHB2X to get its frequency from clock driver */
				tries = 200;
				do {
					ret = clock_control_get_rate(DEVICE_DT_GET(CLOCK_NODE),
								     ccfg->mspi_clock.mclk_ahb2x,
								     &ahb2x_clk_freq);
					if (ret != 0) {
						return ret;
					}
					k_sleep(K_MSEC(100));
					if (--tries == 0) {
						LOG_INF("Timed out getting frequency of clock "
							"AHB2X");
						return -ETIMEDOUT;
					}
				} while (ahb2x_clk_freq == 0);
				LOG_INF("AHB2X clock freq %d", ahb2x_clk_freq);

				/* set DDREN bit when AHB2X clock is up and running */
				if (clock_control_get_status(DEVICE_DT_GET(CLOCK_NODE),
							     ccfg->mspi_clock.mclk_ahb2x) ==
				    CLOCK_CONTROL_STATUS_ON) {
					set_double_data_rate(q);
				}

				/* set baud rate accordingly */
				/* get baud rate */
				baud = (uint8_t)((ahb2x_clk_freq / cfg->freq) - 1);
				/* even if it is ahb2x its clock freq is 120Mhz */
				LOG_INF("QSPI controller clock freq = %d divided by 2",
					ahb2x_clk_freq);
				LOG_INF("BAUD = %u", baud);
				LOG_INF("QSPI device freq set to %d", cfg->freq);
				qspi_set_baud(q, baud);
			} else {
				LOG_ERR("S_S_D mode only for serial memory mode");
				return -EINVAL;
			}
			/* only for readmemory instruction */
		case MSPI_DATA_RATE_S_D_D:
		case MSPI_DATA_RATE_DUAL:
		default:
			LOG_ERR("MCU does not support above quad data rate");
			return -EINVAL;
		}
		LOG_INF("Data rate set by user = %d", cfg->data_rate);
	}

	/* ******Set Clock Polarity and Phase****** */
	ret = set_clock_pol_pha(mask, cfg, q);
	if (ret != 0) {
		return ret;
	}

	/* ******Chip Select Polarity Sanity check****** */
	ret = check_cs_polarity(mask, cfg);
	if (ret != 0) {
		return ret;
	}

	/* Check Data Strobe support */
	ret = check_dqs_support(mask);
	if (ret != 0) {
		return ret;
	}

	/* ******Dummy Cycles input sanity check****** */
	ret = check_rx_tx_dummy(mask, cfg, q);
	if (ret != 0) {
		return ret;
	}

	ret = check_read_write_cmd(mask, cfg);
	if (ret != 0) {
		return ret;
	}

	/* ******Command Length input sanity check****** */
	ret = check_cmd_length(mask, cfg);
	if (ret != 0) {
		return ret;
	}

	/* ******Set Address & its Length input sanity check****** */
	ret = check_addr_length(mask, cfg, q);
	if (ret != 0) {
		return ret;
	}

	/* ******Memory Boundary input sanity check****** */
	ret = check_mem_bound(mask, cfg);
	if (ret != 0) {
		return ret;
	}

	/* ******Time to break input sanity check(will do it later)****** */
	k_mutex_lock(&cdata->lock_dev, K_FOREVER);
	locked = true;
	ret = qspi_disable(q);
	if (ret != 0) {
		if (locked == true) {
			k_mutex_unlock(&cdata->lock_dev);
		}
		return ret;
	}

	if (find_serial_mode_of_child(ccfg->child_desc, dev_id->dev_idx, ccfg->num_children) ==
	    true) {
		if (get_mode(q) != 0) {
			if (locked == true) {
				k_mutex_unlock(&cdata->lock_dev);
			}
			LOG_ERR("Only Mode 0 allowed in serial mode");
			return -EINVAL;
		}
		set_serial_memory_mode(q);
		LOG_INF("Serial memory mode enabled");
	}
	/* if communicating with SPI device(not serial memory)
	 * 1 - set to SPI mode
	 * 2 - set CSMODE to 1 to foolproof any CS de-asserts when next byte transfer begins
	 */
	else {
		q->QSPI_CTRLB =
			(q->QSPI_CTRLB & ~((QSPI_CTRLB_CSMODE_Msk) | (QSPI_CTRLB_MODE_Msk))) |
			(QSPI_CTRLB_MODE(0) | QSPI_CTRLB_CSMODE(1));
		LOG_INF("SPI mode.......");
	}
	/* CTRLB register */
	/* DLYCS and DLYBCT to be set by mspi_timing_config() API -current default 0 */
	/* DATALEN defaults to 0 - 8 bits (1 byte) */
	/* CSMODE defaults to 0 - CS de-asserts after every transfer */
	/* SMEMREG defaults to Serial Memory Registers written via AHB access (QSPI memory map) */
	/* Loopback disabled by default */

	/* BAUD register */
	/* DLYBS set by mspi_timing_config() API */
	/* CPOL, CPHA already set */
	/* BAUD already set */
	/* No interrupts being enabled */

	/* TDATA, RXDATA registers not to be modified currently */

	/* INTENCLR register */
	/* Interrupts already disabled in mspi_config() */

	/* INTENSET register (trying polling method initially) */
	/* No interrupts being enabled */

	/* INTFLAG register */
	/* Interrupt flags already cleared in mspi_config() */

	/* STATUS register */
	if (is_cs_deassert(q) == false) {
		if (locked == true) {
			k_mutex_unlock(&cdata->lock_dev);
		}
		LOG_ERR("CS line not de-asserted");
		return -EIO;
	}

	/* INSTRADDR register */
	/* ONLY USED In serial memory mode AND only if address and no data */
	/* to be configured in mspi_transceive() */

	/* INSTRCTRL register */
	/* ONLY USED in Serial Memory mode for storing commands and option code */
	/* To be configured in mspi_transceive() */

	/* INSTRFRAME register */
	/* WIDTH already set */
	/* Instruction, address, option and data enable bits to be set in mspi_transceive() based on
	 * the instruction
	 */
	/* OPTCODELEN left to default 0 - to be configured in mspi_transceive() */
	/* ADDRLEN initialized here, to be set in mspi_transceive() (24 or 32 bits) */
	/* TFRTYPE to be set in mspi_transceive() based on instruction */
	/* CRMODE (Continuous Read mode) defaults to 0 (disabled) - configured in mspi_transceive()
	 * based on instruction
	 */
	/* DDREN already configured */
	/* DUMMYLEN defaults to 0 - to be set in mspi_transceive() based on instruction */

	/* SCRAMBCTRL and SCRAMBKEY configured in mspi_scramble_config() */

	ret = qspi_enable(q);
	if (ret != 0) {
		if (locked == true) {
			k_mutex_unlock(&cdata->lock_dev);
		}
		return ret;
	}

	if (locked == true) {
		k_mutex_unlock(&cdata->lock_dev);
	}
	return ret;
}

/*
 * @brief 32-bit memcpy.
 *
 * This function copies memory in 32-bit chunks.
 * @param dest Pointer to the destination memory.
 * @param src Pointer to the source memory.
 * @param len Length of data to copy in number of 32-bit words.
 */
static void qspi_memcpy_32bit(uint32_t *dest, uint32_t *src, uint32_t len)
{
	while (len > 0) {
		*dest = *src;
		dest++;
		src++;
		len--;
	}
}

/*
 * @brief 8-bit memcpy.
 *
 * This function copies memory in 8-bit chunks.
 * @param dest Pointer to the destination memory.
 * @param src Pointer to the source memory.
 * @param len Length of data to copy in number of bytes.
 */

static void qspi_memcpy_8bit(uint8_t *dest, uint8_t *src, uint32_t len)
{
	while (len > 0) {
		*dest = *src;
		dest++;
		src++;
		len--;
	}
}

/*
 * @brief Setup QSPI transfer.
 *
 * This function sets up a QSPI transfer.
 * @param trans Pointer to the transfer structure.
 * @param tfr_type Transfer type.
 * @param q Pointer to the QSPI registers.
 *
 *
 * @return true on successful configuration of the QSPI INSTR/INSTRFRAME registers; false on
 * failure.
 */
static bool qspi_setup_transfer(mchp_hal_mspi_pio_transfer_t *trans, uint32_t tfr_type,
				qspi_registers_t *q)
{
	uint32_t mask = 0;

	/* Set instruction address register if address and no data */
	/* address is defined by QSPI memory map address if address and data both are present */
	q->QSPI_INSTRADDR = QSPI_INSTRADDR_ADDR(trans->ui32_device_Addr);

	/* Set Instruction code register */
	q->QSPI_INSTRCTRL = (QSPI_INSTRCTRL_INSTR((uint32_t)trans->ui8_device_instr));
	/* maybe add an opcode field by parsing MSB bits of packet->cmd */
	LOG_INF("Writing command: 0x%X", trans->ui8_device_instr);

	/* Set Instruction Frame register */

	if (trans->send_addr == true) {
		if (trans->addrlen == 3) {
			mask |= (uint32_t)QSPI_INSTRFRAME_ADDRLEN_24BITS_Val;
		}
		/* addrlen = 4 */
		else {
			mask |= (uint32_t)QSPI_INSTRFRAME_ADDRLEN_32BITS_Val;
		}
	}

	/* Add option code later
	 * parse MSB bits of packet->cmd to extract opcode
	 * mask |= QSPI_INSTRFRAME_OPTCODEEN_Msk; and QSPI_INSTRFRAME_OPTCODELEN(value)
	 * }
	 */
	/* continuous read mode */
	if (trans->continue_crmode == true) {
		mask |= QSPI_INSTRFRAME_CRMODE_Msk;
	}

	if (trans->direction == MSPI_RX) {
		mask |= QSPI_INSTRFRAME_DUMMYLEN((uint32_t)trans->rx_dummy);
	} else {
		mask |= QSPI_INSTRFRAME_DUMMYLEN((uint32_t)trans->tx_dummy);
	}

	mask |= QSPI_INSTRFRAME_INSTREN_Msk | QSPI_INSTRFRAME_ADDREN_Msk |
		QSPI_INSTRFRAME_DATAEN_Msk;

	mask |= QSPI_INSTRFRAME_TFRTYPE(tfr_type);

	mask |= (q->QSPI_INSTRFRAME >> QSPI_INSTRFRAME_WIDTH_Pos) & (QSPI_INSTRFRAME_WIDTH_Msk);

	q->QSPI_INSTRFRAME = mask;

	/* To synchronize APB and AHB accesses */
	(uint32_t)q->QSPI_INSTRFRAME;

	return true;
}

/*
 * @brief Write command to QSPI register.
 *
 * This function writes a command to a QSPI register.
 *
 * @param packet Pointer to the transfer packet containing the cmd, address ,data buffer,no. of
 * bytes and callback mask
 * @param xfer   Pointer to the transfer descriptor (cmd/addr lengths, dummy lengths, async flag,
 * direction).
 * @param q      Pointer to the QSPI register block (qspi_registers_t) used to program the hardware.
 *
 * @return       true if the command was successfully issued (or queued for async); false on error
 * or timeout.
 */
static bool QSPI_CommandWrite(const struct mspi_xfer_packet *packet, const struct mspi_xfer *xfer,
			      qspi_registers_t *q)
{
	uint32_t mask = 0;
	bool async = xfer->async;

	/* Configure address */
	if (xfer->addr_length != 0) {
		q->QSPI_INSTRADDR = QSPI_INSTRADDR_ADDR(packet->address);

		mask |= QSPI_INSTRFRAME_ADDREN_Msk;
		if (xfer->addr_length == 3) {
			mask |= (uint32_t)QSPI_INSTRFRAME_ADDRLEN_24BITS_Val;
		}
		/* addrlen = 4 */
		else {
			mask |= (uint32_t)QSPI_INSTRFRAME_ADDRLEN_32BITS_Val;
		}
	}

	/* Configure instruction */
	q->QSPI_INSTRCTRL = (QSPI_INSTRCTRL_INSTR((uint32_t)packet->cmd));
	LOG_INF("Writing command: 0x%X", packet->cmd);

	/* Configure instruction frame */
	mask |= QSPI_INSTRFRAME_INSTREN_Msk;
	mask |= QSPI_INSTRFRAME_TFRTYPE(QSPI_INSTRFRAME_TFRTYPE_READ_Val);

	mask |= (q->QSPI_INSTRFRAME >> QSPI_INSTRFRAME_WIDTH_Pos) & (QSPI_INSTRFRAME_WIDTH_Msk);

	q->QSPI_INSTRFRAME = mask;

	if (async == false) {
		if (!WAIT_FOR(((q->QSPI_INTFLAG & QSPI_INTFLAG_INSTREND_Msk) ==
			       QSPI_INTFLAG_INSTREND_Msk),
			      10 * (TIMEOUT_VALUE_US), k_busy_wait(DELAY_US))) {
			/* Poll Status register to know status if instruction has end */
			LOG_ERR("QSPI command write timeout");
			return false;
		}

		q->QSPI_INTFLAG |= QSPI_INTFLAG_INSTREND_Msk;
	}

	return true;
}

/*
 * @brief Read data from QSPI register.
 *
 * This function reads data from a QSPI register.
 *
 * @param packet Pointer to the transfer packet containing the cmd, address ,data buffer,no. of
 * bytes and callback mask
 * @param xfer   Pointer to the transfer descriptor (cmd/addr lengths, dummy lengths, async flag,
 * direction).
 * @param q Pointer to the QSPI registers.
 *
 * @return       true if the command was successfully issued (or queued for async); false on error
 * or timeout.
 */
static bool QSPI_RegisterRead(const struct mspi_xfer_packet *packet, const struct mspi_xfer *xfer,
			      qspi_registers_t *q)
{
	uint32_t *qspi_buffer = (uint32_t *)QSPI_ADDR;
	uint32_t mask = 0;
	bool async = xfer->async;

	/* Configure Instruction */
	q->QSPI_INSTRCTRL = (QSPI_INSTRCTRL_INSTR((uint32_t)packet->cmd));
	LOG_INF("Writing command: 0x%X", packet->cmd);

	/* Configure Instruction Frame */
	if (packet->dir == MSPI_RX) {
		mask |= QSPI_INSTRFRAME_DUMMYLEN((uint32_t)xfer->rx_dummy);
	} else {
		mask |= QSPI_INSTRFRAME_DUMMYLEN((uint32_t)xfer->tx_dummy);
	}

	mask |= QSPI_INSTRFRAME_INSTREN_Msk | QSPI_INSTRFRAME_DATAEN_Msk;

	mask |= QSPI_INSTRFRAME_TFRTYPE(QSPI_INSTRFRAME_TFRTYPE_READ_Val);

	mask |= (q->QSPI_INSTRFRAME >> QSPI_INSTRFRAME_WIDTH_Pos) & (QSPI_INSTRFRAME_WIDTH_Msk);

	q->QSPI_INSTRFRAME = mask;

	/* To synchronize APB and AHB accesses */
	(uint32_t)q->QSPI_INSTRFRAME;

	/* Read the register content */
	qspi_memcpy_8bit((uint8_t *)packet->data_buf, (uint8_t *)qspi_buffer, packet->num_bytes);

	__DSB();
	__ISB();

	QSPI_EndTransfer(q);

	/* not interrupt driven */
	if (async == false) {
		if (!WAIT_FOR(((q->QSPI_INTFLAG & QSPI_INTFLAG_INSTREND_Msk) ==
			       QSPI_INTFLAG_INSTREND_Msk),
			      10 * (TIMEOUT_VALUE_US), k_busy_wait(DELAY_US))) {
			/* Poll Status register to know status if instruction has end */
			LOG_ERR("QSPI register read timeout");
			return false;
		}

		/* clear flag */
		q->QSPI_INTFLAG |= QSPI_INTFLAG_INSTREND_Msk;
	}

	return true;
}

/*
 * @brief WRITE data to QSPI register.
 *
 * This function writes data to a QSPI register.
 *
 * @param packet Pointer to the transfer packet containing the cmd, address ,data buffer,no. of
 * bytes and callback mask
 * @param xfer   Pointer to the transfer descriptor (cmd/addr lengths, dummy lengths, async flag,
 * direction).
 * @param q Pointer to the QSPI registers.
 *
 * @return       true if the command was successfully issued (or queued for async); false on error
 * or timeout.
 */
static bool QSPI_RegisterWrite(const struct mspi_xfer_packet *packet, const struct mspi_xfer *xfer,
			       qspi_registers_t *q)
{
	uint32_t *qspi_buffer = (uint32_t *)QSPI_ADDR;
	uint32_t mask = 0;
	bool async = xfer->async;

	/* Configure Instruction */
	q->QSPI_INSTRCTRL = (QSPI_INSTRCTRL_INSTR((uint32_t)packet->cmd));
	LOG_INF("Writing command: 0x%X", packet->cmd);

	/* Configure Instruction Frame */
	if (packet->dir == MSPI_RX) {
		mask |= QSPI_INSTRFRAME_DUMMYLEN((uint32_t)xfer->rx_dummy);
	} else {
		mask |= QSPI_INSTRFRAME_DUMMYLEN((uint32_t)xfer->tx_dummy);
	}

	mask |= QSPI_INSTRFRAME_INSTREN_Msk | QSPI_INSTRFRAME_DATAEN_Msk;

	mask |= QSPI_INSTRFRAME_TFRTYPE(QSPI_INSTRFRAME_TFRTYPE_WRITE_Val);

	mask |= (q->QSPI_INSTRFRAME >> QSPI_INSTRFRAME_WIDTH_Pos) & (QSPI_INSTRFRAME_WIDTH_Msk);

	q->QSPI_INSTRFRAME = mask;

	/* To synchronize APB and AHB accesses */
	(uint32_t)q->QSPI_INSTRFRAME;

	/* Read the register content */
	qspi_memcpy_8bit((uint8_t *)qspi_buffer, (uint8_t *)packet->data_buf, packet->num_bytes);

	__DSB();
	__ISB();

	QSPI_EndTransfer(q);

	/* not interrupt driven */
	if (async == false) {
		if (!WAIT_FOR(((q->QSPI_INTFLAG & QSPI_INTFLAG_INSTREND_Msk) ==
			       QSPI_INTFLAG_INSTREND_Msk),
			      10 * (TIMEOUT_VALUE_US), k_busy_wait(DELAY_US))) {
			/* Poll Status register to know status if instruction has end */
			LOG_ERR("QSPI register write timeout");
			return false;
		}

		/* clear flag */
		q->QSPI_INTFLAG |= QSPI_INTFLAG_INSTREND_Msk;
	}

	return true;
}

/*
 * @brief Write data to memory-mapped QSPI flash.
 *
 * This function writes data to a memory-mapped QSPI flash device.
 *
 * @param packet Pointer to the transfer packet containing the cmd, address ,data buffer,no. of
 * bytes and callback mask
 * @param xfer   Pointer to the transfer descriptor (cmd/addr lengths, dummy lengths, async flag,
 * direction).
 * @param q Pointer to the QSPI registers.
 *
 * @return       true if the command was successfully issued (or queued for async); false on error
 * or timeout.
 */
static bool QSPI_MemoryWrite(const struct mspi_xfer_packet *packet, const struct mspi_xfer *xfer,
			     qspi_registers_t *q)
{
	uint32_t *qspi_mem = (uint32_t *)(QSPI_ADDR | packet->address);
	uint32_t *current_buffer_ptr = (uint32_t *)packet->data_buf;
	uint32_t length_32bit, length_8bit;

	bool memory_write_status = false;
	bool async = xfer->async;

	mchp_hal_mspi_pio_transfer_t trans = {0};

	/* "trans" containing "packet" and transfer descriptor "xfer" info */
	trans.ui8_device_instr = packet->cmd;
	trans.send_addr = (xfer->addr_length != 0);
	if (trans.send_addr == true) {
		trans.addrlen = xfer->addr_length;
		trans.ui32_device_Addr = packet->address;
	}
	trans.direction = packet->dir;
	trans.rx_dummy = xfer->rx_dummy;
	trans.tx_dummy = xfer->tx_dummy;

	if (qspi_setup_transfer(&trans, QSPI_INSTRFRAME_TFRTYPE_WRITEMEMORY_Val, q) == true) {
		/* Write to serial flash memory */
		LOG_INF("Writing data to memory");
		length_32bit = (packet->num_bytes) / 4UL;
		length_8bit = packet->num_bytes & 0x03U;

		/* Copy in 32-bit chunks for faster write */
		if (length_32bit > 0U) {
			qspi_memcpy_32bit(qspi_mem, current_buffer_ptr, length_32bit);
		}
		current_buffer_ptr = current_buffer_ptr + length_32bit;
		qspi_mem = qspi_mem + length_32bit;

		/* left over buffer data copy in 8-bit chunks */
		if (length_8bit > 0U) {
			qspi_memcpy_8bit((uint8_t *)qspi_mem, (uint8_t *)current_buffer_ptr,
					 length_8bit);
		}
		__DSB();
		__ISB();

		QSPI_EndTransfer(q);

		/* not interrupt driven */
		if (async == false) {
			if (!WAIT_FOR(((q->QSPI_INTFLAG & QSPI_INTFLAG_INSTREND_Msk) ==
				       QSPI_INTFLAG_INSTREND_Msk),
				      10 * (TIMEOUT_VALUE_US), k_busy_wait(DELAY_US))) {
				/* Poll Status register to know status if instruction has end */
				LOG_ERR("QSPI memory write timeout");
				return false;
			}

			/* clear instruction end flag */
			q->QSPI_INTFLAG |= QSPI_INTFLAG_INSTREND_Msk;
		}

		memory_write_status = true;
		/* Write to serial flash memory */
	}
	return memory_write_status;
}

/*
 * @brief Read data from memory-mapped QSPI flash.
 *
 * This function reads data from a memory-mapped QSPI flash device.
 * @param packet Pointer to the transfer packet containing the cmd, address ,data buffer,no. of
 * bytes and callback mask
 * @param xfer   Pointer to the transfer descriptor (cmd/addr lengths, dummy lengths, async flag,
 * direction).
 * @param q Pointer to the QSPI registers.
 *
 * @return       true if the command was successfully issued (or queued for async); false on error
 * or timeout.
 */
static bool QSPI_MemoryRead(const struct mspi_xfer_packet *packet, const struct mspi_xfer *xfer,
			    qspi_registers_t *q)
{
	uint32_t *qspi_mem = (uint32_t *)(QSPI_ADDR | packet->address);
	uint32_t *current_buffer_ptr = (uint32_t *)packet->data_buf;
	uint32_t length_32bit, length_8bit;

	bool memory_read_status = false;
	bool async = xfer->async;

	mchp_hal_mspi_pio_transfer_t trans = {0};

	/* "trans" containing "packet" and transfer descriptor "xfer" info */
	trans.ui8_device_instr = packet->cmd;
	trans.send_addr = (xfer->addr_length != 0);
	if (trans.send_addr == true) {
		trans.addrlen = xfer->addr_length;
		trans.ui32_device_Addr = packet->address;
	}
	trans.direction = packet->dir;
	trans.rx_dummy = xfer->rx_dummy;
	trans.tx_dummy = xfer->tx_dummy;

	if (qspi_setup_transfer(&trans, QSPI_INSTRFRAME_TFRTYPE_READMEMORY_Val, q) == true) {
		/* Read from serial flash memory */
		LOG_INF("Reading data from memory");
		length_32bit = (packet->num_bytes) / 4UL;
		length_8bit = packet->num_bytes & 0x03U;

		/* Copy in 32-bit chunks for faster write */
		if (length_32bit > 0U) {
			qspi_memcpy_32bit(current_buffer_ptr, qspi_mem, length_32bit);
		}
		current_buffer_ptr = current_buffer_ptr + length_32bit;
		qspi_mem = qspi_mem + length_32bit;

		/* left over buffer data copy in 8-bit chunks */
		if (length_8bit > 0U) {
			qspi_memcpy_8bit((uint8_t *)current_buffer_ptr, (uint8_t *)qspi_mem,
					 length_8bit);
		}

		/* Dummy Read to clear QSPI_SR.INSTRE and QSPI_SR.CSR */
		(uint32_t)q->QSPI_INTFLAG;

		__DSB();
		__ISB();

		QSPI_EndTransfer(q);

		/* not interrupt driven */
		if (async == false) {
			if (!WAIT_FOR(((q->QSPI_INTFLAG & QSPI_INTFLAG_INSTREND_Msk) ==
				       QSPI_INTFLAG_INSTREND_Msk),
				      10 * (TIMEOUT_VALUE_US), k_busy_wait(DELAY_US))) {
				/* Poll Status register to know status if instruction has end */
				LOG_ERR("QSPI memory read timeout");
				return false;
			}

			q->QSPI_INTFLAG |= QSPI_INTFLAG_INSTREND_Msk;
		}

		memory_read_status = true;
		/* Read from serial flash memory */
	}
	return memory_read_status;
}

/* *
 * @brief Release the MSPI context: clear its owner and give the context semaphore to allow another
 * user.
 */
static inline void mspi_context_release(mspi_context_t *ctx)
{
	ctx->owner = NULL;
	k_sem_give(&ctx->lock);
}

/*
 * @brief Lock MSPI context.
 *
 * This function locks the MSPI context for a specific device and transfer.
 *
 * @param ctx Pointer to the MSPI context structure.
 * @param dev_id Pointer to the device ID structure.
 * @param xfer Pointer to the MSPI transfer structure.
 * @param callback Callback function to be called upon transfer completion.
 * @param callback_ctx Context to be passed to the callback function.
 * @param lockon Boolean flag indicating whether to lock the context.
 *
 * @return 0 on success, negative error code on failure.
 */
static int mspi_context_lock(mspi_context_t *ctx, const struct mspi_dev_id *dev_id,
			     const struct mspi_xfer *xfer, mspi_callback_handler_t callback,
			     struct mspi_callback_context *callback_ctx, bool lockon)
{
	int ret = 1;

	if ((k_sem_count_get(&ctx->lock) == 0) && (lockon == false) && (ctx->owner == dev_id)) {
		LOG_ERR("MSPI context already in use by same owner.");
		return -EBUSY;
	}

	/* Lock the semaphore from data##n */
	if (k_sem_take(&ctx->lock, K_FOREVER) != 0) {
		return -EBUSY;
	}

	/* if previous transfer is async, compare it with current xfer */
	if (ctx->xfer.async == true) {
		/* if previous async transfer`s(ctx->xfer) all packets have been queued */
		if (ctx->packets_left == 0) {
			/* if there is a callback for previous async transfer ctx */
			if (ctx->callback_ctx) {
				volatile struct mspi_event_data *evt_data;

				evt_data = &ctx->callback_ctx->mspi_evt.evt_data;
				if (!WAIT_FOR((evt_data->status == 0), TIMEOUT_VALUE_TWO_SEC,
					      k_busy_wait(DELAY_US))) {
					LOG_ERR("Timed out waiting for xfer`s packet to return "
						"from callback");
					return -ETIMEDOUT;
				}
			}
		}
		/* return -Error for current transfer if previous transfer stil has packets
		 * remaining to be queued and dummy,instr,cmd length is not the same for current and
		 * previous transfers
		 */
		else {
			return -EIO;
		}
	}

	/* populate mspi_context ctx with user info	*/
	ctx->owner = dev_id;
	/* ctx->xfer will contain transfer info of the previous transfer which
	 * can be compared to the current transfer 3rd argument
	 */
	ctx->xfer = *xfer;
	ctx->packets_done = 0;
	ctx->packets_left = ctx->xfer.num_packet;
	ctx->callback = callback;
	ctx->callback_ctx = callback_ctx;

	return ret;
}

/*
 * @brief Perform a blocking MSPI transfer.
 *
 * This function handles a blocking transfer over the MSPI interface.
 *
 * @param controller Pointer to the MSPI device structure.
 * @param packet Pointer to the transfer packet containing the cmd, address ,data buffer,no. of
 * bytes and callback mask
 * @param xfer   Pointer to the transfer descriptor (cmd/addr lengths, dummy lengths, async flag,
 * direction).
 *
 * @return 0 on success, negative error code on failure.
 */
static int mchp_hal_mspi_transfer(const struct device *controller,
				  const struct mspi_xfer_packet *packet,
				  const struct mspi_xfer *xfer)
{
	const mspi_sam_qspi_cfg_t *cfg = controller->config;

	int ret = 0;

	qspi_registers_t *q = cfg->reg_cfg.regs;

	/* Sanity checks */
	if ((packet->dir == MSPI_TX) && (xfer->cmd_length != 0) && (packet->num_bytes == 0) &&
	    (packet->data_buf == NULL)) {
		/* Command write only -TX command with instruction,with/without address, and no data
		 */
		QSPI_CommandWrite(packet, xfer, q);
	} else if ((packet->dir == MSPI_RX) && (xfer->cmd_length != 0) &&
		   (xfer->addr_length == 0) && (packet->num_bytes != 0) &&
		   (packet->data_buf != NULL)) {
		/* size of rx_data shud be changed according to size of register being read */
		QSPI_RegisterRead(packet, xfer, q);
	} else if ((packet->dir == MSPI_TX) && (xfer->cmd_length != 0) &&
		   (xfer->addr_length == 0) && (packet->num_bytes != 0) &&
		   (packet->data_buf != NULL)) {
		/* size of tx_data shud be changed according to size of register being read */
		QSPI_RegisterWrite(packet, xfer, q);
	} else if ((packet->dir == MSPI_TX) && (xfer->cmd_length != 0) &&
		   (xfer->addr_length != 0) && (packet->num_bytes != 0) &&
		   (packet->data_buf != NULL)) {
		/* Memory write only - TX command with instruction,address and data */
		QSPI_MemoryWrite(packet, xfer, q);
	} else if ((packet->dir == MSPI_RX) && (xfer->cmd_length != 0) &&
		   (xfer->addr_length != 0) && (packet->num_bytes != 0) &&
		   (packet->data_buf != NULL)) {
		/* Memory read only - TX command with instruction,address and no data */
		QSPI_MemoryRead(packet, xfer, q);
	} else {
		LOG_ERR("Wrong packet configuration");
		ret = -EINVAL;
	}

	return ret;
}

/*
 * @brief Perform a PIO-based MSPI transfer.
 *
 * This function handles a PIO-based transfer over the MSPI interface.
 *
 * @param controller Pointer to the MSPI device structure.
 * @param xfer Pointer to the MSPI transfer structure.
 * @param cb Callback function to be called upon transfer completion.
 * @param cb_ctx Context to be passed to the callback function.
 *
 * @return 0 on success, negative error code on failure.
 */
static int mspi_pio_serial_transceive(const struct device *controller, const struct mspi_xfer *xfer,
				      mspi_callback_handler_t cb,
				      struct mspi_callback_context *cb_ctx)
{
	const mspi_sam_qspi_cfg_t *cfg = controller->config;
	mspi_sam_qspi_data_t *data = controller->data;
	qspi_registers_t *q = cfg->reg_cfg.regs;

	/* data->ctx(.callback, .callback_ctx) is initialized in init function with 0s(zeroes) &
	 * rest of fields are filled in mspi_context_lock and in else async part with
	 * "ctx->callback_ctx->mspi_evt"
	 */
	mspi_context_t *ctx = &data->ctx;
	const struct mspi_xfer_packet *packet;

	uint32_t packet_idx;

	int ret = 0;
	int cfg_flag = 0;
	bool locked = false;

	if (xfer->num_packet == 0 || (xfer->packets == NULL) ||
	    (xfer->timeout > CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE)) {
		return -EFAULT;
	}

	/* Lock the semaphore from data##n and populate mspi_context ctx with user info */
	locked = true;
	cfg_flag = mspi_context_lock(ctx, data->dev_id, xfer, cb, cb_ctx, true);
	if (cfg_flag != 1) {
		if (locked == true) {
			mspi_context_release(ctx);
		}
	}

	while (ctx->packets_left > 0) {
		packet_idx = ctx->xfer.num_packet - ctx->packets_left;
		packet = &ctx->xfer.packets[packet_idx];
		if (ctx->xfer.async == false) {
			ret = mchp_hal_mspi_transfer(controller, packet, xfer);
		} else { /* async transfer */
			/* enable INSTREND interrupt */
			enable_instrend_interrupt(q);

			if ((ctx->callback != NULL) &&
			    (packet->cb_mask == MSPI_BUS_XFER_COMPLETE_CB)) {
				/* populate fields of ctx->callback_ctx */
				ctx->callback_ctx->mspi_evt.evt_type = MSPI_BUS_XFER_COMPLETE;
				ctx->callback_ctx->mspi_evt.evt_data.controller = controller;
				ctx->callback_ctx->mspi_evt.evt_data.dev_id = data->dev_id;
				ctx->callback_ctx->mspi_evt.evt_data.packet = packet;
				ctx->callback_ctx->mspi_evt.evt_data.packet_idx = packet_idx;
				ctx->callback_ctx->mspi_evt.evt_data.status = ~0;
			}

			/* Do a non-blocking transfer using only trans argument */
			ret = mchp_hal_mspi_transfer(controller, packet, xfer);

			disable_instrend_interrupt(q);
		}
		ctx->packets_left--;
		if (ret != 0) {
			if (locked == true) {
				LOG_ERR("Serial transceive failed!");
				mspi_context_release(ctx);
			}
			return -EIO;
		}
	}

	if (locked == true) {
		mspi_context_release(ctx);
	}
	return ret;
}

/* *
 * @brief Perform a blocking SPI transceive using polling on the Microchip QSPI peripheral.
 *
 * Handles sending command/address/dummy bytes and reading/writing payload bytes by
 * polling QSPI status (RXC) and moving data between the peripheral and the packet buffer.
 *
 * @param controller Pointer to the Zephyr device instance for the QSPI controller.
 * @param packet     Pointer to the transfer packet containing the cmd, address ,data buffer,no. of
 * bytes and callback mask
 * @param xfer       Pointer to the transfer descriptor (cmd/addr lengths, dummy lengths, timeout,
 * async flag).
 *
 * @return 0 on success, negative errno on failure (e.g. -EIO, -EINVAL, -EFAULT).
 */

static int spi_transceive_poll(const struct device *controller,
			       const struct mspi_xfer_packet *packet, const struct mspi_xfer *xfer)
{
	uint8_t ret = 0;
	bool cmdwriteonly = false;

	const mspi_sam_qspi_cfg_t *cfg = controller->config;
	qspi_registers_t *q = cfg->reg_cfg.regs;

	uint32_t dummyData;
	uint16_t dummyLen = packet->num_bytes;
	uint16_t pre_dummy_bytes = 0;
	uint16_t pre_dummy_bytes_tmp = 0;
	uint8_t *tx_temp = (uint8_t *)packet->data_buf;
	uint8_t *rx_temp = (uint8_t *)packet->data_buf;

	uint32_t rx_drop = 0;
	uint8_t addressLen = xfer->addr_length;

	uint32_t receivedData;
	uint32_t packets = packet->num_bytes;
	uint32_t blocks = 0;

	uint32_t index = 0;

	if (packet->dir == MSPI_RX) {
		pre_dummy_bytes = (xfer->rx_dummy + MSPI_BITS_ROUND_MASK) / MSPI_BITS_PER_BYTE;
	} else {
		pre_dummy_bytes = (xfer->tx_dummy + MSPI_BITS_ROUND_MASK) / MSPI_BITS_PER_BYTE;
	}
	pre_dummy_bytes_tmp = pre_dummy_bytes;
	rx_drop = xfer->cmd_length + xfer->addr_length + pre_dummy_bytes;

	/* Flush out any unread data in SPI read buffer */
	dummyData = (q->QSPI_RXDATA & QSPI_RXDATA_DATA_Msk) >> QSPI_RXDATA_DATA_Pos;

	if (xfer->cmd_length != 0) {
		/* command write only , just break out of loop */
		if ((packet->dir == MSPI_TX) && (xfer->addr_length == 0) &&
		    (packet->num_bytes == 0) && (packet->data_buf == NULL)) {
			LOG_INF("Command write only");
			cmdwriteonly = true;
		}
		LOG_INF("packet cmd = 0x%X", packet->cmd);
		/* initiate the transfer */
		q->QSPI_TXDATA = (uint8_t)packet->cmd;
	}

	while (blocks <
	       (xfer->cmd_length + xfer->addr_length + packet->num_bytes + pre_dummy_bytes)) {
		blocks++;
		if (!WAIT_FOR(((q->QSPI_INTFLAG & QSPI_INTFLAG_RXC_Msk) == QSPI_INTFLAG_RXC_Msk),
			      TIMEOUT_VALUE_US, k_busy_wait(DELAY_US))) {
			LOG_ERR("Wait for received data to come in RXDATA from shifter timed out");
			return -ETIMEDOUT;
		}

		/* reading RXDATA clears RXC flag */
		receivedData = ((q->QSPI_RXDATA & QSPI_RXDATA_DATA_Msk) >> QSPI_RXDATA_DATA_Pos);
		/* drop cmd,address,pre-dummy cycles */
		if (rx_drop != 0) {
			rx_drop--;
			LOG_INF("rx_drop = %u", rx_drop);
			if (cmdwriteonly == true) {
				QSPI_EndTransfer(q);
				break;
			}
		} else {
			if (packet->dir == MSPI_RX) {
				rx_temp[index] = receivedData;
				index++;
				LOG_INF("receivedData = %u", receivedData);
				if ((blocks == (xfer->cmd_length + xfer->addr_length +
						packet->num_bytes + pre_dummy_bytes)) ||
				    (dummyLen == 0)) {
					QSPI_EndTransfer(q);
					break;
				}
			}
			/* for MSPI_TX if addresses and data has been sent , end and exit */
			else if ((addressLen == 0) && (packets == 0)) {
				/* dont go in DRE check if address byte and data bytes are 0 */
				QSPI_EndTransfer(q);
				LOG_INF("All address and data packets sent.");
				break;
			}
		}

		if (!WAIT_FOR(((q->QSPI_INTFLAG & QSPI_INTFLAG_DRE_Msk) == QSPI_INTFLAG_DRE_Msk),
			      TIMEOUT_VALUE_US, k_busy_wait(DELAY_US))) {
			LOG_ERR("Wait for data to move from TXDATA to shifter QSPI enable flag "
				"setting timed out");
			return -ETIMEDOUT;
		}

		/* set LASTXFER before sending out last byte in TXDATA(compare blocks or dummylen(if
		 * address bytes have been sent and last dummy byte remains))
		 */
		if ((blocks == (xfer->cmd_length + xfer->addr_length + packet->num_bytes +
				pre_dummy_bytes)) &&
		    (packet->dir == MSPI_TX)) {
			QSPI_EndTransfer(q);
			break;
		}

		if (addressLen > 0) {
			q->QSPI_TXDATA = (uint8_t)(((packet->address &
						     (0x000000FF << (8 * (addressLen - 1))))) >>
						   (8 * (addressLen - 1)));
			addressLen--;
		}
		/* for dummy cycles if needed before transmitting bogus 0xFF to read in or clock
		 * in data
		 */
		else if (pre_dummy_bytes_tmp > 0) {
			q->QSPI_TXDATA = (uint8_t)0xFF;
			pre_dummy_bytes_tmp--;
		} else if (dummyLen > 0 && packet->dir == MSPI_RX) {
			q->QSPI_TXDATA = (uint8_t)0xFF;
			dummyLen--;
		} else {
			q->QSPI_TXDATA = tx_temp[index];
			index++;
			packets--;
		}
	}

	LOG_INF("Write Read Completed!");
	return ret;
}

/* *
 * @brief Perform a interrupt-driven SPI transceive on the Microchip QSPI peripheral.
 *
 * initiates the SPI transceive process by sending the first byte out of TXDATA
 * Then mspi_mchp_isr(interrupt handler) takes over Handling the sending command/address/dummy bytes
 * and reading/writing payload bytes and moving data between the peripheral and the packet buffer.
 *
 * @param controller Pointer to the Zephyr device instance for the QSPI controller.
 * @param packet     Pointer to the transfer packet containing the cmd, address ,data buffer,no. of
 * bytes and callback mask.
 * @param xfer       Pointer to the transfer descriptor (cmd/addr lengths, dummy lengths, timeout,
 * async flag).
 *
 * @return 0 on success, negative errno on failure (e.g. -EIO, -EINVAL, -EFAULT).
 */

static int spi_transceive_async(const struct device *controller,
				const struct mspi_xfer_packet *packet, const struct mspi_xfer *xfer)
{
	uint8_t ret = 0;

	const mspi_sam_qspi_cfg_t *cfg = controller->config;
	mspi_sam_qspi_data_t *data = controller->data;
	qspi_registers_t *q = cfg->reg_cfg.regs;

	uint32_t dummyData;
	uint16_t pre_dummy_bytes_tmp = 0;

	/* all fields of qspiObj cleared/initialized in mspi_mchp_qspi_init function */
	qspi_obj_init(&data->qspiObj);
	mchp_hal_spi_pio_transfer_t *qspiObj = &data->qspiObj;

	if (packet->dir == MSPI_RX) {
		qspiObj->pre_dummy_bytes =
			(xfer->rx_dummy + MSPI_BITS_ROUND_MASK) / MSPI_BITS_PER_BYTE;
		qspiObj->dummy_size = packet->num_bytes;
	} else {
		qspiObj->pre_dummy_bytes =
			(xfer->tx_dummy + MSPI_BITS_ROUND_MASK) / MSPI_BITS_PER_BYTE;
	}
	pre_dummy_bytes_tmp = qspiObj->pre_dummy_bytes;

	qspiObj->rx_count = 0;
	qspiObj->tx_count = 0;

	qspiObj->transfer_is_busy = false;
	qspiObj->addrlen = xfer->addr_length;
	LOG_INF("qspiObj->addrlen= %u", qspiObj->addrlen);

	if (packet->dir == MSPI_TX) {
		qspiObj->rx_drop = xfer->cmd_length + xfer->addr_length + qspiObj->pre_dummy_bytes +
				   packet->num_bytes;
		LOG_INF("qspiObj->rx_drop = %u", qspiObj->rx_drop);
	} else {
		qspiObj->rx_drop = xfer->cmd_length + xfer->addr_length + qspiObj->pre_dummy_bytes;
	}

	if (qspiObj->transfer_is_busy == false) {
		/* Flush out any unread data in SPI read buffer */
		dummyData = (q->QSPI_RXDATA & QSPI_RXDATA_DATA_Msk) >> QSPI_RXDATA_DATA_Pos;

		if (xfer->cmd_length != 0) {
			/* initiate the transfer with cmd/opcode */
			q->QSPI_TXDATA = packet->cmd;
			LOG_INF("packet->cmd = 0x%X", packet->cmd);
			qspiObj->blocks++;
		}

		/* ********Usually unused************* */
		else if (xfer->addr_length != 0) {
			/* initiate the transfer with address */
			q->QSPI_TXDATA = (uint8_t)((packet->address &
						    (0x000000FF << (8 * (qspiObj->addrlen - 1)))) >>
						   (8 * (qspiObj->addrlen - 1)));
			qspiObj->addrlen--;
			qspiObj->blocks++;
		} else if (pre_dummy_bytes_tmp > 0) {
			q->QSPI_TXDATA = (uint8_t)0xFF;
			pre_dummy_bytes_tmp--;
			qspiObj->pre_dummy_bytes = pre_dummy_bytes_tmp;
			qspiObj->blocks++;
		} else if (qspiObj->dummy_size > 0) {
			/* send dummy byte to read data */
			q->QSPI_TXDATA = (uint8_t)0xFF;
			qspiObj->dummy_size--;
			qspiObj->blocks++;
		}
		/* initiate transfer directly with data */
		else if (packet->dir == MSPI_TX) {
			q->QSPI_TXDATA = packet->data_buf[qspiObj->tx_count];
			qspiObj->blocks++;
		}
		/* *******Usually unused************* */

		if (packet->dir == MSPI_RX) {
			/* enable RXC interrupt to get into mspi_mchp_isr */
			enable_rxc_interrupt(q);
		} else if ((packet->dir == MSPI_TX) && (packet->num_bytes > 0)) {
			/* maybe both interrupts dont need to be enabled */
			LOG_INF("enabled rxc and dre interrupts");
			enable_rxc_dre_interrupt(q);
		} else if (packet->dir == MSPI_TX) {
			/* enable DRE interrupt to get into mspi_mchp_isr */
			enable_dre_interrupt(q);
		}
	} else {
		LOG_ERR("QSPI transfer is busy or invalid packet");
	}

	return ret;
}

/* *
 * @brief PIO-mode SPI transceive for the MSPI controller - process all packets in @p xfer using
 * SPI-mode PIO.
 *
 * Locks the MSPI context, iterates the packets in @p xfer and performs per-packet transfers
 * in SPI mode (either blocking polling or non-blocking/interrupt-driven), and releases the context.
 *
 * @param controller Pointer to the MSPI controller device (const struct device *).
 * @param xfer       Pointer to the transfer descriptor (const struct mspi_xfer *) containing
 * packets, mode and timeout.
 * @param cb         Transfer-completion callback (mspi_callback_handler_t); may be NULL if not
 * used.
 * @param cb_ctx     Context passed to @p cb (struct mspi_callback_context *); may be NULL.
 *
 * @return 0 on success; negative errno on failure (e.g. -EINVAL, -EFAULT, -EIO).
 */

static int mspi_pio_spi_transceive(const struct device *controller, const struct mspi_xfer *xfer,
				   mspi_callback_handler_t cb, struct mspi_callback_context *cb_ctx)
{
	mspi_sam_qspi_data_t *data = controller->data;
	int ret = 0;

	/* data->ctx(.callback, .callback_ctx) is initialized in init function with 0s(zeroes) &
	 * rest of fields are filled in mspi_context_lock and in else async part with
	 * "ctx->callback_ctx->mspi_evt"
	 */
	mspi_context_t *ctx = &data->ctx;

	const struct mspi_xfer_packet *packet = NULL;

	uint32_t packet_idx;

	int cfg_flag = 0;
	bool locked = false;

	if (xfer->num_packet == 0 || (xfer->packets == NULL) ||
	    (xfer->timeout > CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE)) {
		return -EFAULT;
	}

	/* Lock the semaphore from data##n and populate mspi_context ctx with user info */
	locked = true;
	cfg_flag = mspi_context_lock(ctx, data->dev_id, xfer, cb, cb_ctx, true);
	if (cfg_flag != 1) {
		if (locked == true) {
			mspi_context_release(ctx);
		}
	}

	/* each mspi_xfer_packet packet from "xfer" getting processed in SPI mode,either by:
	 * a) polling mode
	 * b) interrupt-driven mode
	 */
	while (ctx->packets_left > 0) {
		packet_idx = ctx->xfer.num_packet - ctx->packets_left;
		packet = &ctx->xfer.packets[packet_idx];

		if (ctx->xfer.async == false) {
			/* Do a blocking transfer using only trans argument */
			ret = spi_transceive_poll(controller, packet, xfer);
			LOG_INF("ctx->packets_left = %u", ctx->packets_left);
		} else {
			LOG_INF("ctx->packets_left = %u", ctx->packets_left);
			if ((ctx->callback != NULL) &&
			    (packet->cb_mask == MSPI_BUS_XFER_COMPLETE_CB)) {
				/* populate fields of ctx->callback_ctx */
				ctx->callback_ctx->mspi_evt.evt_type = MSPI_BUS_XFER_COMPLETE;
				ctx->callback_ctx->mspi_evt.evt_data.controller = controller;
				ctx->callback_ctx->mspi_evt.evt_data.dev_id = data->dev_id;
				ctx->callback_ctx->mspi_evt.evt_data.packet = packet;
				ctx->callback_ctx->mspi_evt.evt_data.packet_idx = packet_idx;
				ctx->callback_ctx->mspi_evt.evt_data.status = ~0;
			}
			/* interrupt implementation in SPI mode */
			LOG_INF("spi_transceive_async...");
			ret = spi_transceive_async(controller, packet, xfer);
		}

		ctx->packets_left--;
		if (ret != 0) {
			if (locked == true) {
				mspi_context_release(ctx);
			}
			LOG_ERR("spi_transceive failed!");
			return -EIO;
		}
	}

	if (locked == true) {
		mspi_context_release(ctx);
	}
	return ret;
}

/*
 * @brief Transceive data over MSPI.
 *
 * This function initiates a data transfer over the MSPI interface.
 *
 * @param controller Pointer to the MSPI device structure.
 * @param dev_id Pointer to the MSPI device ID structure.
 * @param xfer Pointer to the MSPI transfer structure.
 *
 * @return 0 on success, negative error code on failure.
 */
static int mspi_mchp_transceive(const struct device *controller, const struct mspi_dev_id *dev_id,
				const struct mspi_xfer *xfer)
{
	const mspi_sam_qspi_cfg_t *cfg = controller->config;
	mspi_sam_qspi_data_t *data = controller->data;

	mspi_callback_handler_t cb = data->cbs[MSPI_BUS_XFER_COMPLETE];
	struct mspi_callback_context *cb_ctx = data->cb_ctxs[MSPI_BUS_XFER_COMPLETE];

	qspi_registers_t *q = cfg->reg_cfg.regs;

	int ret = 0;

	if (controller == NULL || (controller->config == NULL) || (dev_id == NULL) ||
	    (xfer == NULL)) {
		return -EINVAL;
	}

	/* wait for 150us for device to be ready to read/write etc.(can also be ran from app code)
	 */
	k_busy_wait(150);

	/* Clear all interrupt flags */
	clear_all_interrupts(q);

	if (((q->QSPI_STATUS & QSPI_STATUS_CSSTATUS_Msk) == 0) ||
	    ((q->QSPI_STATUS & QSPI_STATUS_ENABLE_Msk) == 0)) {
		LOG_ERR("QSPI module not enabled or CS line not de-asserted");
		return -EIO;
	}

	/* *********Based on user input start config the registers below******** */

	/* CSMODE is 0 */
	/* SMEMREG is 0 */

	if (/* (xfer->async == false) && */ (xfer->xfer_mode == MSPI_PIO)) {
		/* decide whether to choose SPI mode or serial memory mode */
		if (find_serial_mode_of_child(cfg->child_desc, dev_id->dev_idx,
					      cfg->num_children) == false) {
			/* SPI mode implementation */
			LOG_INF("Implementing in SPI mode");
			mspi_pio_spi_transceive(controller, xfer, cb, cb_ctx);
		} else {
			LOG_INF("Implementing in MSPI mode");
			ret = mspi_pio_serial_transceive(controller, xfer, cb, cb_ctx);
			if (ret != 0) {
				LOG_ERR("mspi_pio_serial_transceive failed");
				return ret;
			}
		}
	}
	/* DMA transfers to be handled later */
	return ret;
}

/* ===== Driver API table (add other hooks in your full driver) ===== */
static const struct mspi_driver_api mspi_sam_qspi_api = {
	.config = mspi_mchp_config,
	.dev_config = mspi_mchp_dev_config,
	.transceive = mspi_mchp_transceive,
	.register_callback = mspi_mchp_register_callback,
};

/* *
 * @brief Initialize QSPI module.
 *
 * This function Disable QSPI module
 * and in future may disable power control
 *
 * @param controller Pointer to the MSPI device structure.
 *
 * @return 0 on success, negative error code on failure.
 */
static int mspi_mchp_qspi_init(const struct device *controller)
{
	mspi_sam_qspi_data_t *data = controller->data;
	const mspi_sam_qspi_cfg_t *cfg = controller->config;
	const mspi_child_cfg_t *child_cfg_ptr = cfg->child_cfg;

	qspi_registers_t *q = cfg->reg_cfg.regs;

	const struct device *device_clk = DEVICE_DT_GET(CLOCK_NODE);

	int ret = 0;
	bool locked = false;

	k_mutex_init(&data->lock_init);
	k_mutex_init(&data->lock_dev);
	k_sem_init(&data->ctx.lock, 1, 1);

	k_mutex_lock(&data->lock_init, K_FOREVER);
	locked = true;

	data->ctx.owner = NULL;

	/* init the data->ctx.xfer values */
	data->ctx.xfer.async = false;
	data->ctx.xfer.xfer_mode = MSPI_PIO;
	data->ctx.xfer.tx_dummy = 0;
	data->ctx.xfer.rx_dummy = 0;
	data->ctx.xfer.cmd_length = 0;
	data->ctx.xfer.addr_length = 0;
	data->ctx.xfer.priority = 0;
	data->ctx.xfer.packets = NULL;
	data->ctx.xfer.num_packet = 0;

	data->ctx.packets_left = -1;
	data->ctx.packets_done = -1;

	data->ctx.callback = NULL;
	data->ctx.callback_ctx = NULL;

	data->ctx.asynchronous = false;

	/* enable clocks */
	if ((device_is_ready(device_clk)) == false) {
		if (locked == true) {
			k_mutex_unlock(&data->lock_init);
		}
		LOG_WRN("Clocks not ready");
		return -ENODEV;
	}

	ret = clock_control_on(device_clk, cfg->mspi_clock.mclk_apb);
	if ((ret < 0) && (ret != -EALREADY)) {
		if (locked == true) {
			k_mutex_unlock(&data->lock_init);
		}
		LOG_ERR("MCLK APB not ready");
		return ret;
	}
	ret = clock_control_on(device_clk, cfg->mspi_clock.mclk_ahb);
	if ((ret < 0) && (ret != -EALREADY)) {
		if (locked == true) {
			k_mutex_unlock(&data->lock_init);
		}
		LOG_ERR("MCLK AHB not ready");
		return ret;
	}

	LOG_INF("MCLK APB, AHB enabled");

	/* apply pin config */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		if (locked == true) {
			k_mutex_unlock(&data->lock_init);
		}
		LOG_ERR("Pin control apply state failed=%d", ret);
		return ret;
	}

	/* Reset the QSPI module */
	qspi_swrst(q);
	LOG_INF("QSPI software reset done");

	/* iterate over all children and configure each child device */
	LOG_INF("Number of children = %d", cfg->num_children);
	for (uint8_t i = 0; i < cfg->num_children; i++) {
		ret = mspi_mchp_dev_config(controller, &child_cfg_ptr[i].id,
					   (MSPI_DEVICE_CONFIG_ALL & ~(MSPI_DEVICE_CONFIG_DQS)),
					   &child_cfg_ptr[i].cfg);
		if (ret != 0) {
			if (locked == true) {
				k_mutex_unlock(&data->lock_init);
			}
			LOG_ERR("Device config failed for dev_idx %d", child_cfg_ptr[i].id.dev_idx);
			return ret;
		}
		LOG_INF("Address length = %d bytes", child_cfg_ptr[i].cfg.addr_length);
	}

	if (locked == true) {
		k_mutex_unlock(&data->lock_init);
	}
	return ret;
}

/* *
 * @brief One entry per enabled child under this controller instance
 *
 * @param child Instance number
 */
#define CHILD_DESC(child)                                                                          \
	{                                                                                          \
		.cs = (uint8_t)DT_REG_ADDR(child),                                                 \
		.is_spi_nor = DT_NODE_HAS_COMPAT(child, jedec_mspi_nor),                           \
	}

/* *
 * @brief Build child config (id + cfg)
 *
 * @param node Instance number
 */
#define MSPI_MCHP_CHILD_CFG(node)                                                                  \
	{                                                                                          \
		.id =                                                                              \
			{                                                                          \
				.dev_idx = (uint8_t)DT_REG_ADDR(node),                             \
				.ce = {0}, /* currently unused */                                  \
			},                                                                         \
		.cfg = {.freq = DT_PROP_OR(node, mspi_max_frequency, 0),                           \
			.io_mode = DT_ENUM_IDX_OR(node, mspi_io_mode, 0),                          \
			.data_rate = DT_ENUM_IDX_OR(node, mspi_data_rate, 0),                      \
			.cpp = DT_ENUM_IDX_OR(node, mspi_cpp_mode, 0),                             \
			.ce_polarity = DT_ENUM_IDX_OR(node, mspi_ce_polarity, 0),                  \
			.dqs_enable = false,                                                       \
			.cmd_length = DT_ENUM_IDX_OR(node, command_length, 1),                     \
			.addr_length = DT_ENUM_IDX_OR(node, address_length, 3),                    \
			.read_cmd = DT_PROP_OR(node, read_command, 0x0B),                          \
			.write_cmd = DT_PROP_OR(node, write_command, 0x02),                        \
			.rx_dummy = DT_PROP_OR(node, rx_dummy, 8),                                 \
			.tx_dummy = DT_PROP_OR(node, tx_dummy, 0),                                 \
			.mem_boundary = DT_PROP_BY_IDX(node, ce_break_config, 0),                  \
			.time_to_break = DT_PROP_BY_IDX(node, ce_break_config, 1),                 \
		}                                                                                  \
	}

/* *
 * @brief  Controller instantiation from DT(is meant for mspi_config() if user doesn't call
 * mspi_config())
 *
 * @param inst Instance number
 */
#define MSPI_MCHP_CONFIG(inst)                                                                     \
	{                                                                                          \
		.channel_num = 0,                                                                  \
		.op_mode = DT_ENUM_IDX_OR(DT_DRV_INST(inst), op_mode, MSPI_OP_MODE_CONTROLLER),    \
		.duplex = DT_ENUM_IDX_OR(DT_DRV_INST(inst), duplex, MSPI_HALF_DUPLEX),             \
		.max_freq = DT_INST_PROP_OR(                                                       \
			inst, clock_frequency,                                                     \
			120000000), /* 120Mhz for CLK_QSPI_AHB not CLK_QSPI_AHB2X */               \
		.dqs_support = DT_INST_PROP_OR(inst, dqs_support, false),                          \
		.num_periph = DT_INST_CHILD_NUM(inst),                                             \
		.sw_multi_periph = DT_INST_PROP(inst, software_multiperipheral),                   \
	}

/* *
 * @brief  Generate child configuration arrays per controller instance
 *
 * @param inst Instance number
 */
#define MSPI_MCHP_GEN_CHILD_TABLES(inst)                                                           \
	static const mspi_child_desc_t mspi_child_desc_##inst[] = {                                \
		DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(inst), CHILD_DESC)};                      \
	static const mspi_child_cfg_t mspi_child_cfg_##inst[] = {                                  \
		DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(inst), MSPI_MCHP_CHILD_CFG)};

/* *
 * @brief Macro to configure and enable MSPI IRQ
 *
 * @param inst Instance number
 * @param m IRQ index
 */
#define MCHP_MSPI_IRQ_CONNECT(inst, m)                                                             \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, m, irq),                                      \
			    DT_INST_IRQ_BY_IDX(inst, m, priority), mspi_mchp_isr,                  \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQ_BY_IDX(inst, m, irq));                                      \
	} while (false)

/* *
 * @brief Configure MSPI interrupts for instance inst.
 *
 * This function sets up the interrupt handlers for the MSPI instance
 * specified by inst.
 *
 * @param dev Pointer to the device structure for the driver instance.
 */
#define MSPI_MCHP_IRQ_HANDLER(inst)                                                                \
	static void mspi_mchp_irq_config_##inst(const struct device *dev)                          \
	{                                                                                          \
		MCHP_MSPI_IRQ_CONNECT(inst, 0);                                                    \
	}

/* *
 * @brief Macro to declare an IRQ handler function
 *
 * @param inst Instance number
 */
#define MSPI_MCHP_IRQ_HANDLER_DECL(inst)                                                           \
	static void mspi_mchp_irq_config_##inst(const struct device *dev);
#define MSPI_MCHP_IRQ_HANDLER_FUNC(inst) .irq_config_func = mspi_mchp_irq_config_##inst,

/* *
 * @brief Macro that declares and initializes a Microchip MSPI device instance:
 * allocates/initializes driver data, configures hardware resources and registers the device with
 * the OS.
 */
#define MSPI_MCHP_DEVICE_INIT(inst)                                                                \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	MSPI_MCHP_IRQ_HANDLER_DECL(inst)                                                           \
	static mspi_sam_qspi_data_t mspi_sam_qspi_data_##inst = {                                  \
		.dev_id = NULL,                                                                    \
		.lock_init = Z_MUTEX_INITIALIZER(mspi_sam_qspi_data_##inst.lock_init),             \
		.lock_dev = Z_MUTEX_INITIALIZER(mspi_sam_qspi_data_##inst.lock_dev),               \
		.qspiObj = {0},                                                                    \
	};                                                                                         \
	static const mspi_sam_qspi_cfg_t mspi_sam_qspi_cfg_##inst = {                              \
		.reg_cfg = {.regs = (qspi_registers_t *)DT_INST_REG_ADDR(0), .pads = 0},           \
		.reg_size = DT_INST_REG_SIZE(inst),                                                \
		.mspicfg = MSPI_MCHP_CONFIG(inst),                                                 \
		.mspicfg.re_init = false,                                                          \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		MSPI_MCHP_IRQ_HANDLER_FUNC(inst).mspi_clock =                                      \
			{                                                                          \
				.clock_dev_apb = NULL,                                             \
				.clock_dev_ahb = NULL,                                             \
				.clock_dev_ahb2x = NULL,                                           \
				.mclk_apb = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(   \
					inst, apb, subsystem),                                     \
				.mclk_ahb = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(   \
					inst, ahb, subsystem),                                     \
				.mclk_ahb2x = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME( \
					inst, ahb2x, subsystem),                                   \
			},                                                                         \
		.child_desc = mspi_child_desc_##inst,                                              \
		.num_children = ARRAY_SIZE(mspi_child_desc_##inst),                                \
		.child_cfg = mspi_child_cfg_##inst,                                                \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, mspi_mchp_qspi_init, NULL, &mspi_sam_qspi_data_##inst,         \
			      &mspi_sam_qspi_cfg_##inst, POST_KERNEL,                              \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &mspi_sam_qspi_api);             \
	MSPI_MCHP_IRQ_HANDLER(inst)

DT_INST_FOREACH_STATUS_OKAY(MSPI_MCHP_GEN_CHILD_TABLES)
DT_INST_FOREACH_STATUS_OKAY(MSPI_MCHP_DEVICE_INIT)
