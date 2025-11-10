#include <soc.h>
#include <stdint.h>
#include <stdbool.h>

/* Timeout values for WAIT_FOR macro */
#define TIMEOUT_VALUE_US      1000
#define TIMEOUT_VALUE_TWO_SEC 2000000
#define DELAY_US              2

/* Dummy clock cycle macros */
#define MSPI_BITS_PER_BYTE   8
#define MSPI_BITS_ROUND_MASK (MSPI_BITS_PER_BYTE - 1)

#define MSPI_MCHP_CONTROLLER_MAX_FREQ DT_PROP(DT_NODELABEL(qspi0), clock_frequency)

typedef enum {
	MCHP_HAL_MSPI_XIP_READ_ONLY = 0,
	MCHP_HAL_MSPI_XIP_READ_WRITE = 1,
} mspi_xip_permit_e;

typedef enum {
	MCHP_HAL_MSPI_DATA_RATE_SDR = 0,
	MCHP_HAL_MSPI_DATA_RATE_DDR = 1,
} mspi_data_rate_e;

typedef enum {
	MCHP_HAL_MSPI_CMD_1_BYTE = 1,
	MCHP_HAL_MSPI_CMD_2_BYTE = 2
} mchp_hal_mspi_instr_e;

typedef enum {
	MCHP_HAL_MSPI_ADDR_3_BYTE = 3,
	MCHP_HAL_MSPI_ADDR_4_BYTE = 4
} mchp_hal_mspi_addr_e;

/**
 * @brief Register configuration for the Microchip QSPI peripheral.
 *
 * @details
 * Holds the base register pointer and pad configuration used to
 * program the MCUs QSPI controller. This is a pure hardware
 * mapping container-no ownership or lifetime semantics.
 */
typedef struct mchp_qspi_reg_config {
	/* Pointer to SPI master registers */
	qspi_registers_t *regs;

	/* SPI pad configuration */
	uint32_t pads;

} mchp_qspi_reg_config_t;

/**
 * @brief Timing and access policy for an MSPI/XIP transaction path.
 *
 * @details
 * Packs the SPI inter-transfer delays and dummy cycles together with the
 * effective data rate and XIP write permission. The numeric fields map
 * directly to the device registers shown in parentheses below; unless stated
 * otherwise, values are interpreted in SPI master clock periods.
 */
struct mchp_qspi_timing_cfg {
	uint8_t dlybs;        /* 0..255  (BAUD.DLYBS) */
	uint8_t dlybct;       /* 0..255  (CTRLB.DLYBCT)  - ignored in MODE=1 */
	uint8_t dlycs;        /* 0..255  (CTRLB.DLYCS)   */
	uint8_t dummy_cycles; /* 0..31   (INSTRFRAME.DUMMYLEN) */

	mspi_data_rate_e data_rate;   /* SDR / DDR */
	mspi_xip_permit_e permission; /* READ_ONLY / READ_WRITE               */
};

/**
 * @brief Issue a software reset to the QSPI controller.
 *
 * @details
 * Sets SWRST in CTRLA.
 *
 * @param q Pointer to the QSPI register block.
 */
static inline void qspi_swrst(qspi_registers_t *q)
{
	q->QSPI_CTRLA |= QSPI_CTRLA_SWRST_Msk;
}

/**
 * @brief Program the serial clock baud divider.
 *
 * @details
 * Writes the BAUD field. The effective SCK is derived from the QSPI
 * source clock and this divider per the datasheet.
 *
 * @param q     Pointer to the QSPI register block.
 * @param baud  Encoded divider value for QSPI_BAUD.BAUD.
 */
static inline void qspi_set_baud(qspi_registers_t *q, uint32_t baud)
{
	q->QSPI_BAUD |= (q->QSPI_BAUD & ~(QSPI_BAUD_BAUD_Msk)) | QSPI_BAUD_BAUD(baud);
}

/**
 * @brief Set bus width (IO lines) for the instruction frame.
 *
 * @details
 * Updates INSTRFRAME.WIDTH to select single/dual/quad line transactions.
 *
 * @param q      Pointer to the QSPI register block.
 * @param width  Encoded width value for QSPI_INSTRFRAME.WIDTH.
 */
static inline void set_width_bits(qspi_registers_t *q, uint32_t width)
{
	q->QSPI_INSTRFRAME |=
		(q->QSPI_INSTRFRAME & ~QSPI_INSTRFRAME_WIDTH_Msk) | QSPI_INSTRFRAME_WIDTH(width);
}

/**
 * @brief Get phase and polarity.
 *
 * @details
 * Get phase and polarity(mode) from BAUD register.
 *
 * @param q      Pointer to the QSPI register block.
 * @return mode  Bitfield [1:0] from BAUD register.
 */
static inline uint8_t get_mode(qspi_registers_t *q)
{
	uint8_t phase = 5;    /* dummy value */
	uint8_t polarity = 5; /* dummy value */

	phase = (q->QSPI_BAUD & QSPI_BAUD_CPHA_Msk) >> QSPI_BAUD_CPHA_Msk;
	polarity = (q->QSPI_BAUD & QSPI_BAUD_CPOL_Msk) >> QSPI_BAUD_CPOL_Msk;

	return (uint8_t)(polarity | (phase << 1));
}

/**
 * @brief Disable Double Data Rate (DDR) for instruction/data phases.
 *
 * @param q Pointer to the QSPI register block.
 */
static inline void clear_double_data_rate(qspi_registers_t *q)
{
	q->QSPI_INSTRFRAME &= ~QSPI_INSTRFRAME_DDREN_Msk;
}

/**
 * @brief Enable Double Data Rate (DDR) for instruction/data phases.
 *
 * @param q Pointer to the QSPI register block.
 */
static inline void set_double_data_rate(qspi_registers_t *q)
{
	q->QSPI_INSTRFRAME |= QSPI_INSTRFRAME_DDREN_Msk;
}

/**
 * @brief Set the number of dummy cycles for the instruction frame.
 *
 * @details
 * Programs INSTRFRAME.DUMMYLEN which controls the number of dummy
 * cycles inserted before the data phase (e.g., FAST READ).
 *
 * @param q      Pointer to the QSPI register block.
 * @param dummy  Encoded dummy length for QSPI_INSTRFRAME.DUMMYLEN.
 */
static inline void set_dummy_bits(qspi_registers_t *q, uint32_t dummy)
{
	q->QSPI_INSTRFRAME |= (q->QSPI_INSTRFRAME & ~QSPI_INSTRFRAME_DUMMYLEN_Msk) |
			      QSPI_INSTRFRAME_DUMMYLEN(dummy);
}

/**
 * @brief Set the address length (bytes) for the instruction frame.
 *
 * @details
 * Programs INSTRFRAME.ADDRLEN to match the target device addressing
 * (e.g., 24-bit or 32-bit).
 *
 * @param q         Pointer to the QSPI register block.
 * @param addr_len  Encoded address length for QSPI_INSTRFRAME.ADDRLEN.
 */
static inline void set_address_len(qspi_registers_t *q, uint32_t addr_len)
{
	q->QSPI_INSTRFRAME |= (q->QSPI_INSTRFRAME & ~QSPI_INSTRFRAME_ADDRLEN_Msk) |
			      QSPI_INSTRFRAME_ADDRLEN(addr_len);
}

/**
 * @brief Put the controller into Serial Memory Mode.
 *
 * @details
 * Sets CTRLB.MODE accordingly so that array accesses use the AHB
 * memory window (XIP-like behavior per datasheet).
 *
 * @param q Pointer to the QSPI register block.
 */
static inline void set_serial_memory_mode(qspi_registers_t *q)
{
	q->QSPI_CTRLB = (q->QSPI_CTRLB & ~((QSPI_CTRLB_CSMODE_Msk) | (QSPI_CTRLB_MODE_Msk))) |
			(QSPI_CTRLB_MODE(QSPI_CTRLB_MODE_MEMORY_Val) |
			 QSPI_CTRLB_CSMODE(QSPI_CTRLB_CSMODE_NORELOAD_Val));
}

/**
 * @brief Check if chip-select is deasserted.
 *
 * @details
 * Reads STATUS.CSSTATUS to determine whether the hardware reports CS high.
 *
 * @param q Pointer to the QSPI register block.
 * @return true if CS is deasserted (high), false otherwise.
 */
static inline bool is_cs_deassert(qspi_registers_t *q)
{
	return ((q->QSPI_STATUS & QSPI_STATUS_CSSTATUS_Msk) != 0);
}

/**
 * @brief Clear all QSPI interrupt flags.
 *
 * @details
 * Writes to INTFLAG to acknowledge pending events (RXC, DRE, TXC, ERROR,
 * CSRISE, INSTREND).
 *
 * @param q Pointer to the QSPI register block.
 */
static inline void clear_all_interrupts(qspi_registers_t *q)
{
	q->QSPI_INTFLAG = QSPI_INTFLAG_RXC_Msk | QSPI_INTFLAG_DRE_Msk | QSPI_INTFLAG_TXC_Msk |
			  QSPI_INTFLAG_ERROR_Msk | QSPI_INTFLAG_CSRISE_Msk |
			  QSPI_INTFLAG_INSTREND_Msk;
}

/**
 * @brief Disable all QSPI interrupts.
 *
 * @details
 * Writes to INTENCLR to mask RXC, DRE, TXC, ERROR, CSRISE, and INSTREND.
 *
 * @param q Pointer to the QSPI register block.
 */
static inline void disable_all_interrupts(qspi_registers_t *q)
{
	q->QSPI_INTENCLR =
		(QSPI_INTENCLR_RXC_Msk | QSPI_INTENCLR_DRE_Msk | QSPI_INTENCLR_TXC_Msk |
		 QSPI_INTENCLR_ERROR_Msk | QSPI_INTENCLR_CSRISE_Msk | QSPI_INTENCLR_INSTREND_Msk);
}

static inline void enable_instrend_interrupt(qspi_registers_t *q)
{
	q->QSPI_INTENSET |= QSPI_INTENSET_INSTREND(1);
}

static inline void disable_instrend_interrupt(qspi_registers_t *q)
{
	q->QSPI_INTENCLR |= QSPI_INTENCLR_INSTREND(1);
}

static inline void enable_dre_interrupt(qspi_registers_t *q)
{
	q->QSPI_INTENSET = QSPI_INTENSET_DRE_Msk;
}

static inline void enable_rxc_interrupt(qspi_registers_t *q)
{
	q->QSPI_INTENSET = QSPI_INTENSET_RXC_Msk;
}

static inline void enable_rxc_dre_interrupt(qspi_registers_t *q)
{
	q->QSPI_INTENSET = QSPI_INTENSET_RXC_Msk | QSPI_INTENSET_DRE_Msk;
}

/*
 * @brief End QSPI transfer.
 *
 * This function ends a QSPI transfer.
 * setting LASTXFER bit in CSMODE = 0 is not required
 * (present there because if user decides to make CSMODE = 1)
 * @param q Pointer to the QSPI registers.
 */
static inline void QSPI_EndTransfer(qspi_registers_t *q)
{
	q->QSPI_CTRLA = QSPI_CTRLA_ENABLE_Msk | QSPI_CTRLA_LASTXFER_Msk;
}
