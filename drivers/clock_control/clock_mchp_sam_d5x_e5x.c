/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file clock_mchp_sam_d5x_e5x.c
 * @brief Clock control driver for sam_d5x_e5x family devices.
 */

#include <stdbool.h>
#include <string.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/clock_control/mchp_clock_control.h>

/******************************************************************************
 * @brief Macro definitions
 *****************************************************************************/
#define DT_DRV_COMPAT microchip_sam_d5x_e5x_clock
LOG_MODULE_REGISTER(clock_mchp_sam_d5x_e5x, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

#define CLOCK_NODE DT_NODELABEL(clock)

#define CLOCK_SUCCESS 0

#define FREQ_32KHZ      32768
#define FREQ_1KHZ       1024
#define FREQ_DFLL_48MHZ 48000000

/* maximum value for gclk pin I/O channel, 0 - 7 */
#define GCLK_IO_MAX 7

/* gclk peripheral channel max, 0 - 47 */
#define GPH_MAX 47

/* maximum value for mask bit position, 0 - 31 */
#define MMASK_MAX 31

/* maximum value for div, when div_select is clock source frequency divided by 2^(N+1) */
#define GCLKGEN_POWER_DIV_MAX 29

/* init iteration count, so that, the source clocks are initialized before executing init */
#define CLOCK_INIT_ITERATION_COUNT 3

/* mclkbus Not Applicable for a clock subsystem ID */
#define MBUS_NA  (0x3f)
/* mclkmaskbit Not Applicable for a clock subsystem ID */
#define MMASK_NA (0x3f)
/* gclkperiph Not Applicable for a clock subsystem ID */
#define GPH_NA   (0x3f)

/* Clock subsystem Types */
#define SUBSYS_TYPE_XOSC       (0)
#define SUBSYS_TYPE_DFLL       (1)
#define SUBSYS_TYPE_FDPLL      (2)
#define SUBSYS_TYPE_RTC        (3)
#define SUBSYS_TYPE_OSC32K     (4)
#define SUBSYS_TYPE_GCLKGEN    (5)
#define SUBSYS_TYPE_GCLKPERIPH (6)
#define SUBSYS_TYPE_MCLKCPU    (7)
#define SUBSYS_TYPE_MCLKPERIPH (8)
#define SUBSYS_TYPE_MAX        (8)

/* mclk bus */
#define MBUS_AHB  (0)
#define MBUS_APBA (1)
#define MBUS_APBB (2)
#define MBUS_APBC (3)
#define MBUS_APBD (4)
#define MBUS_MAX  (4)

/* XOSC instances */
#define INST_XOSC0 0
#define INST_XOSC1 1

/* FDPLL instances */
#define INST_FDPLL0 0
#define INST_FDPLL1 1

/* OSC32K instances */
#define INST_OSC32K_OSCULP1K  0
#define INST_OSC32K_OSCULP32K 1
#define INST_OSC32K_XOSC1K    2
#define INST_OSC32K_XOSC32K   3

/******************************************************************************
 * @brief Data type definitions
 *****************************************************************************/
/** @brief Clock subsystem definition.
 *
 * Structure which can be used as a sys argument in the clock_control API.
 * Encode clock type, mclk bus, mclk mask bit, gclk pch and instance number,
 * to clock subsystem.
 *
 * - 00..07 (8 bits): inst
 * - 08..13 (6 bits): gclkperiph
 * - 14..19 (6 bits): mclkmaskbit
 * - 20..25 (6 bits): mclkbus
 * - 26..31 (6 bits): type
 */
typedef union {
	uint32_t val;
	struct {
		uint32_t inst: 8;
		uint32_t gclkperiph: 6;
		uint32_t mclkmaskbit: 6;
		uint32_t mclkbus: 6;
		uint32_t type: 6;
	} bits;
} clock_mchp_subsys_t;

#if CONFIG_CLOCK_CONTROL_MCHP_CONFIG_BOOTUP
/** @brief XOSC initialization structure. */
typedef struct {
	clock_mchp_subsys_t subsys;
	uint32_t frequency;
	uint8_t clock_switch_en;            /* XOSCCTRL */
	uint8_t clock_failure_detection_en; /* XOSCCTRL */
	uint8_t automatic_loop_control_en;  /* XOSCCTRL */
	uint8_t low_buffer_gain_en;         /* XOSCCTRL */
	uint8_t on_demand_en;               /* XOSCCTRL */
	uint8_t run_in_standby_en;          /* XOSCCTRL */
	uint8_t xtal_en;                    /* XOSCCTRL */
	uint16_t startup_time;              /* XOSCCTRL */
	uint8_t enable;                     /* XOSCCTRL */
} clock_xosc_init_t;

/** @brief DFLL initialization structure. */
typedef struct {
	uint8_t src_gclk; /* PCHCTRL */

	uint8_t closed_loop_en;        /* DFLLCTRLB */
	uint8_t wait_lock_en;          /* DFLLCTRLB */
	uint8_t bypass_coarse_lock_en; /* DFLLCTRLB */
	uint8_t quick_lock_dis;        /* DFLLCTRLB */
	uint8_t chill_cycle_dis;       /* DFLLCTRLB */
	uint8_t usb_recovery_en;       /* DFLLCTRLB */
	uint8_t lose_lock_en;          /* DFLLCTRLB */
	uint8_t stable_freq_en;        /* DFLLCTRLB */

	uint8_t coarse_max_step;  /* DFLLMUL */
	uint8_t fine_max_step;    /* DFLLMUL */
	uint16_t multiply_factor; /* DFLLMUL */

	uint8_t on_demand_en;      /* DFLLCTRLA */
	uint8_t run_in_standby_en; /* DFLLCTRLA */
	uint8_t enable;            /* DFLLCTRLA */
} clock_dfll_init_t;

/** @brief FDPLL initialization structure. */
typedef struct {
	clock_mchp_subsys_t subsys;
	uint8_t dco_filter_select;   /* DPLLCTRLB */
	uint8_t src;                 /* DPLLCTRLB */
	uint8_t pi_filter_type;      /* DPLLCTRLB */
	uint8_t dco_en;              /* DPLLCTRLB */
	uint8_t lock_bypass_en;      /* DPLLCTRLB */
	uint8_t wakeup_fast_en;      /* DPLLCTRLB */
	uint16_t xosc_clock_divider; /* DPLLCTRLB */

	uint8_t divider_ratio_frac; /* DPLLRATIO */
	uint16_t divider_ratio_int; /* DPLLRATIO */

	uint8_t on_demand_en;      /* DPLLCTRLA */
	uint8_t run_in_standby_en; /* DPLLCTRLA */
	uint8_t enable;            /* DPLLCTRLA */
} clock_fdpll_init_t;

/** @brief OSC32K initialization structure. */
typedef struct {
	uint8_t cf_backup_divideby2_en; /* CFDCTRL */
	uint8_t switch_back_en;         /* CFDCTRL */
	uint8_t cfd_en;                 /* CFDCTRL */

	uint8_t osculp32k_write_lock_en; /* OSCULP32K */
	uint8_t osculp32k_1khz_en;       /* OSCULP32K */
	uint8_t osculp32k_32khz_en;      /* OSCULP32K */

	uint8_t gain_mode;         /* XOSC32K */
	uint8_t write_lock_en;     /* XOSC32K */
	uint8_t on_demand_en;      /* XOSC32K */
	uint8_t run_in_standby_en; /* XOSC32K */
	uint8_t osc32k_1khz_en;    /* XOSC32K */
	uint8_t osc32k_32khz_en;   /* XOSC32K */
	uint8_t xtal_en;           /* XOSC32K */
	uint16_t startup_time;     /* XOSC32K */
	uint8_t enable;            /* XOSC32K */
} clock_osc32k_init_t;

/** @brief GCLKGEN initialization structure. */
typedef struct {
	clock_mchp_subsys_t subsys;
	uint8_t div_select;
	uint8_t pin_output_off_val;
	uint8_t src;
	uint8_t run_in_standby_en;
	uint8_t pin_output_en;
	uint8_t duty_50_50_en;
	uint16_t div_factor;
	uint8_t enable;
	uint32_t pin_src_freq;
} clock_gclkgen_init_t;
#endif /* CONFIG_CLOCK_CONTROL_MCHP_CONFIG_BOOTUP */

/** @brief clock driver configuration structure. */
typedef struct {
	oscctrl_registers_t *oscctrl_regs;
	osc32kctrl_registers_t *osc32kctrl_regs;
	gclk_registers_t *gclk_regs;
	mclk_registers_t *mclk_regs;

	/* Timeout in milliseconds to wait for clock to turn on */
	uint32_t on_timeout_ms;
} clock_mchp_config_t;

/*
 * - 00..14 (15 bits): clock_mchp_gclkgen_t/clock_mchp_fdpll_src_clock_t
 * - 15..23 (9 bits): CLOCK_MCHP_FDPLL_SRC_MAX+1+clock_mchp_gclk_src_clock_t
 */
typedef enum {
	ON_BITPOS_GCLK0 = CLOCK_MCHP_FDPLL_SRC_GCLK0,
	ON_BITPOS_GCLK1 = CLOCK_MCHP_FDPLL_SRC_GCLK1,
	ON_BITPOS_GCLK2 = CLOCK_MCHP_FDPLL_SRC_GCLK2,
	ON_BITPOS_GCLK3 = CLOCK_MCHP_FDPLL_SRC_GCLK3,
	ON_BITPOS_GCLK4 = CLOCK_MCHP_FDPLL_SRC_GCLK4,
	ON_BITPOS_GCLK5 = CLOCK_MCHP_FDPLL_SRC_GCLK5,
	ON_BITPOS_GCLK6 = CLOCK_MCHP_FDPLL_SRC_GCLK6,
	ON_BITPOS_GCLK7 = CLOCK_MCHP_FDPLL_SRC_GCLK7,
	ON_BITPOS_GCLK8 = CLOCK_MCHP_FDPLL_SRC_GCLK8,
	ON_BITPOS_GCLK9 = CLOCK_MCHP_FDPLL_SRC_GCLK9,
	ON_BITPOS_GCLK10 = CLOCK_MCHP_FDPLL_SRC_GCLK10,
	ON_BITPOS_GCLK11 = CLOCK_MCHP_FDPLL_SRC_GCLK11,
	ON_BITPOS_XOSC32K_ = CLOCK_MCHP_FDPLL_SRC_XOSC32K,
	ON_BITPOS_XOSC0_ = CLOCK_MCHP_FDPLL_SRC_XOSC0,
	ON_BITPOS_XOSC1_ = CLOCK_MCHP_FDPLL_SRC_XOSC1,
	ON_BITPOS_XOSC0 = CLOCK_MCHP_FDPLL_SRC_MAX + 1 + CLOCK_MCHP_GCLK_SRC_XOSC0,
	ON_BITPOS_XOSC1 = CLOCK_MCHP_FDPLL_SRC_MAX + 1 + CLOCK_MCHP_GCLK_SRC_XOSC1,
	ON_BITPOS_GCLKPIN = CLOCK_MCHP_FDPLL_SRC_MAX + 1 + CLOCK_MCHP_GCLK_SRC_GCLKPIN,
	ON_BITPOS_GCLKGEN1 = CLOCK_MCHP_FDPLL_SRC_MAX + 1 + CLOCK_MCHP_GCLK_SRC_GCLKGEN1,
	ON_BITPOS_OSCULP32K = CLOCK_MCHP_FDPLL_SRC_MAX + 1 + CLOCK_MCHP_GCLK_SRC_OSCULP32K,
	ON_BITPOS_XOSC32K = CLOCK_MCHP_FDPLL_SRC_MAX + 1 + CLOCK_MCHP_GCLK_SRC_XOSC32K,
	ON_BITPOS_DFLL = CLOCK_MCHP_FDPLL_SRC_MAX + 1 + CLOCK_MCHP_GCLK_SRC_DFLL,
	ON_BITPOS_FDPLL0 = CLOCK_MCHP_FDPLL_SRC_MAX + 1 + CLOCK_MCHP_GCLK_SRC_FDPLL0,
	ON_BITPOS_FDPLL1 = CLOCK_MCHP_FDPLL_SRC_MAX + 1 + CLOCK_MCHP_GCLK_SRC_FDPLL1
} clock_mchp_on_bitpos_t;

/** @brief clock driver data structure. */
typedef struct {
#if CONFIG_CLOCK_CONTROL_MCHP_ASYNC_ON
	/* To indicate if async on is triggered and interrupt is yet to occur */
	bool is_async_in_progress;

	/* subsystem for which async on is triggered */
	clock_mchp_subsys_t async_subsys;

	/* async on call back function and user argument */
	clock_control_cb_t async_cb;
	void *async_cb_user_data;
#endif /* CONFIG_CLOCK_CONTROL_MCHP_ASYNC_ON */

	uint32_t xosc_crystal_freq[CLOCK_MCHP_XOSC_ID_MAX + 1];
	uint32_t gclkpin_freq[GCLK_IO_MAX + 1];

	/*
	 * - 00..14 (15 bits): clock_mchp_gclkgen_t/clock_mchp_fdpll_src_clock_t
	 * - 15..23 (9 bits): CLOCK_MCHP_FDPLL_SRC_MAX+1+clock_mchp_gclk_src_clock_t
	 */
	uint32_t src_on_status;
	clock_mchp_gclk_src_clock_t gclk0_src;
} clock_mchp_data_t;

/******************************************************************************
 * @brief Function forward declarations
 *****************************************************************************/
#if CONFIG_CLOCK_CONTROL_MCHP_GET_RATE
static int clock_get_rate_dfll(const struct device *dev, uint32_t *freq);
static int clock_get_rate_fdpll(const struct device *dev, uint8_t fdpll_id, uint32_t *freq);
#endif /* CONFIG_CLOCK_CONTROL_MCHP_GET_RATE */
static enum clock_control_status clock_mchp_get_status(const struct device *dev,
						       clock_control_subsys_t sys);

/******************************************************************************
 * @brief Helper functions
 *****************************************************************************/
/**
 * @brief check if subsystem type and id are valid.
 */
static int clock_check_subsys(clock_mchp_subsys_t subsys)
{
	int ret_val;
	uint32_t inst_max, gclkperiph_max, mclkbus_max, mclkmaskbit_max;

	ret_val = -EINVAL;
	inst_max = 0;
	gclkperiph_max = GPH_NA;
	mclkbus_max = MBUS_NA;
	mclkmaskbit_max = MMASK_NA;
	do {
		/* Check if turning on all clocks is requested. */
		if (subsys.val == (uint32_t)CLOCK_CONTROL_SUBSYS_ALL) {
			break;
		}

		/* Check if the specified subsystem was found. */
		if (subsys.bits.type > SUBSYS_TYPE_MAX) {
			break;
		}

		switch (subsys.bits.type) {
		case SUBSYS_TYPE_XOSC:
			inst_max = CLOCK_MCHP_XOSC_ID_MAX;
			break;

		case SUBSYS_TYPE_DFLL:
			inst_max = CLOCK_MCHP_DFLL_MAX;
			gclkperiph_max = CLOCK_MCHP_DFLL_MAX;
			break;

		case SUBSYS_TYPE_FDPLL:
			inst_max = CLOCK_MCHP_FDPLL_ID_MAX;
			gclkperiph_max = CLOCK_MCHP_FDPLL_ID_MAX;
			break;

		case SUBSYS_TYPE_RTC:
			inst_max = CLOCK_MCHP_RTC_ID_MAX;
			break;

		case SUBSYS_TYPE_OSC32K:
			inst_max = CLOCK_MCHP_OSC32K_ID_MAX;
			break;

		case SUBSYS_TYPE_GCLKGEN:
			inst_max = CLOCK_MCHP_GCLKGEN_ID_MAX;
			break;

		case SUBSYS_TYPE_GCLKPERIPH:
			inst_max = CLOCK_MCHP_GCLKPERIPH_ID_MAX;
			gclkperiph_max = GPH_MAX;
			break;

		case SUBSYS_TYPE_MCLKCPU:
			inst_max = CLOCK_MCHP_MCLKCPU_MAX;
			break;

		case SUBSYS_TYPE_MCLKPERIPH:
			inst_max = CLOCK_MCHP_MCLKPERIPH_ID_MAX;
			mclkbus_max = MBUS_MAX;
			mclkmaskbit_max = MMASK_MAX;
			break;

		default:
			LOG_ERR("Unsupported SUBSYS_TYPE");
		}

		/* Check if the specified id is valid. */
		if ((subsys.bits.inst > inst_max) || (subsys.bits.gclkperiph > gclkperiph_max) ||
		    (subsys.bits.mclkbus > mclkbus_max) ||
		    (subsys.bits.mclkmaskbit > mclkmaskbit_max)) {
			break;
		}

		ret_val = CLOCK_SUCCESS;
	} while (0);

	return ret_val;
}

/**
 * @brief get the address of mclk mask register.
 */
__IO uint32_t *get_mclkbus_mask_reg(mclk_registers_t *mclk_regs, uint32_t bus)
{
	__IO uint32_t *reg32 = NULL;

	switch (bus) {
	case MBUS_AHB:
		reg32 = &mclk_regs->MCLK_AHBMASK;
		break;
	case MBUS_APBA:
		reg32 = &mclk_regs->MCLK_APBAMASK;
		break;
	case MBUS_APBB:
		reg32 = &mclk_regs->MCLK_APBBMASK;
		break;
	case MBUS_APBC:
		reg32 = &mclk_regs->MCLK_APBCMASK;
		break;
	case MBUS_APBD:
		reg32 = &mclk_regs->MCLK_APBDMASK;
		break;
	default:
		LOG_ERR("Unsupported mclkbus");
		break;
	}
	return reg32;
}

#if CONFIG_CLOCK_CONTROL_MCHP_ASYNC_ON
/**
 * @brief disable clock ready interrupts.
 */
void clock_disable_interrupt(const clock_mchp_config_t *config, const clock_mchp_subsys_t subsys)
{
	oscctrl_registers_t *oscctrl_regs = config->oscctrl_regs;

	switch (subsys.bits.type) {
	case SUBSYS_TYPE_XOSC:
		oscctrl_regs->OSCCTRL_INTENCLR = (subsys.bits.inst == INST_XOSC0)
							 ? OSCCTRL_INTENCLR_XOSCRDY0_Msk
							 : OSCCTRL_INTENCLR_XOSCRDY1_Msk;
		break;

	case SUBSYS_TYPE_FDPLL:
		oscctrl_regs->OSCCTRL_INTENCLR = (subsys.bits.inst == INST_FDPLL0)
							 ? OSCCTRL_INTENCLR_DPLL0LCKR_Msk
							 : OSCCTRL_INTENCLR_DPLL1LCKR_Msk;
		break;

	case SUBSYS_TYPE_DFLL:
		oscctrl_regs->OSCCTRL_INTENCLR = OSCCTRL_INTENCLR_DFLLRDY_Msk;
		break;

	case SUBSYS_TYPE_OSC32K:
		config->osc32kctrl_regs->OSC32KCTRL_INTENCLR |= OSC32KCTRL_INTENCLR_XOSC32KRDY_Msk;
		break;

	default:
		break;
	}
}

/**
 * @brief clear clock ready interrupts.
 */
void clock_clear_interrupt(const clock_mchp_config_t *config, const clock_mchp_subsys_t subsys)
{
	oscctrl_registers_t *oscctrl_regs = config->oscctrl_regs;

	switch (subsys.bits.type) {
	case SUBSYS_TYPE_XOSC:
		oscctrl_regs->OSCCTRL_INTFLAG = (subsys.bits.inst == INST_XOSC0)
							? OSCCTRL_INTFLAG_XOSCRDY0_Msk
							: OSCCTRL_INTFLAG_XOSCRDY1_Msk;
		break;

	case SUBSYS_TYPE_FDPLL:
		oscctrl_regs->OSCCTRL_INTFLAG = (subsys.bits.inst == INST_FDPLL0)
							? OSCCTRL_INTFLAG_DPLL0LCKR_Msk
							: OSCCTRL_INTFLAG_DPLL1LCKR_Msk;
		break;

	case SUBSYS_TYPE_DFLL:
		oscctrl_regs->OSCCTRL_INTFLAG = OSCCTRL_INTFLAG_DFLLRDY_Msk;
		break;

	case SUBSYS_TYPE_OSC32K:
		config->osc32kctrl_regs->OSC32KCTRL_INTFLAG |= OSC32KCTRL_INTFLAG_XOSC32KRDY_Msk;
		break;

	default:
		break;
	}
}

/**
 * @brief enable clock ready interrupts.
 */
void clock_enable_interrupt(const clock_mchp_config_t *config, const clock_mchp_subsys_t subsys)
{
	oscctrl_registers_t *oscctrl_regs = config->oscctrl_regs;

	switch (subsys.bits.type) {
	case SUBSYS_TYPE_XOSC:
		oscctrl_regs->OSCCTRL_INTENSET = (subsys.bits.inst == INST_XOSC0)
							 ? OSCCTRL_INTENSET_XOSCRDY0_Msk
							 : OSCCTRL_INTENSET_XOSCRDY1_Msk;
		break;

	case SUBSYS_TYPE_FDPLL:
		oscctrl_regs->OSCCTRL_INTENSET = (subsys.bits.inst == INST_FDPLL0)
							 ? OSCCTRL_INTENSET_DPLL0LCKR_Msk
							 : OSCCTRL_INTENSET_DPLL1LCKR_Msk;
		break;

	case SUBSYS_TYPE_DFLL:
		oscctrl_regs->OSCCTRL_INTENSET = OSCCTRL_INTENSET_DFLLRDY_Msk;
		break;

	case SUBSYS_TYPE_OSC32K:
		config->osc32kctrl_regs->OSC32KCTRL_INTENSET |= OSC32KCTRL_INTENSET_XOSC32KRDY_Msk;
		break;

	default:
		break;
	}
}
#endif /* CONFIG_CLOCK_CONTROL_MCHP_ASYNC_ON */

/**
 * @brief function to set/clear clock subsystem enable bit.
 */
static int clock_on_off(const clock_mchp_config_t *config, const clock_mchp_subsys_t subsys,
			bool on)
{
	oscctrl_registers_t *oscctrl_regs = config->oscctrl_regs;
	osc32kctrl_registers_t *osc32kctrl_regs = config->osc32kctrl_regs;
	gclk_registers_t *gclk_regs = config->gclk_regs;

	int ret_val;
	__IO uint32_t *reg32 = NULL;
	uint32_t reg32_val;
	__IO uint16_t *reg16 = NULL;
	uint16_t reg16_val;
	__IO uint8_t *reg8 = NULL;
	uint8_t reg8_val, inst;

	ret_val = CLOCK_SUCCESS;
	inst = subsys.bits.inst;
	switch (subsys.bits.type) {
	case SUBSYS_TYPE_XOSC:
		reg32 = &oscctrl_regs->OSCCTRL_XOSCCTRL[inst];
		reg32_val = OSCCTRL_XOSCCTRL_ENABLE_Msk;
		break;
	case SUBSYS_TYPE_DFLL:
		reg8 = &oscctrl_regs->OSCCTRL_DFLLCTRLA;
		reg8_val = OSCCTRL_DFLLCTRLA_ENABLE_Msk;
		break;
	case SUBSYS_TYPE_FDPLL:
		reg8 = &oscctrl_regs->DPLL[inst].OSCCTRL_DPLLCTRLA;
		reg8_val = OSCCTRL_DPLLCTRLA_ENABLE_Msk;
		break;
	case SUBSYS_TYPE_OSC32K:
		if (on == true) {
			osc32kctrl_regs->OSC32KCTRL_XOSC32K |= OSC32KCTRL_XOSC32K_ENABLE_Msk;
		}
		switch (inst) {
		case INST_OSC32K_OSCULP1K:
			reg32 = &osc32kctrl_regs->OSC32KCTRL_OSCULP32K;
			reg32_val = OSC32KCTRL_OSCULP32K_EN1K_Msk;
			break;
		case INST_OSC32K_OSCULP32K:
			reg32 = &osc32kctrl_regs->OSC32KCTRL_OSCULP32K;
			reg32_val = OSC32KCTRL_OSCULP32K_EN32K_Msk;
			break;
		case INST_OSC32K_XOSC1K:
			reg16 = &osc32kctrl_regs->OSC32KCTRL_XOSC32K;
			reg16_val = OSC32KCTRL_XOSC32K_EN1K_Msk;
			break;
		case INST_OSC32K_XOSC32K:
			reg16 = &osc32kctrl_regs->OSC32KCTRL_XOSC32K;
			reg16_val = OSC32KCTRL_XOSC32K_EN32K_Msk;
			break;
		default:
			break;
		}
		break;
	case SUBSYS_TYPE_GCLKGEN:
		/* Check if the clock is GCLKGEN0, which is always on */
		if (inst == CLOCK_MCHP_GCLKGEN_GEN0) {
			/* GCLK GEN0 is always on */
			break;
		}
		/* Enable the clock generator by setting the GENEN bit */
		reg32 = &gclk_regs->GCLK_GENCTRL[inst];
		reg32_val = GCLK_GENCTRL_GENEN_Msk;
		break;
	case SUBSYS_TYPE_GCLKPERIPH:
		reg32 = &gclk_regs->GCLK_PCHCTRL[subsys.bits.gclkperiph];
		reg32_val = GCLK_PCHCTRL_CHEN_Msk;
		break;
	case SUBSYS_TYPE_MCLKPERIPH:
		reg32 = get_mclkbus_mask_reg(config->mclk_regs, subsys.bits.mclkbus);
		reg32_val = 1 << subsys.bits.mclkmaskbit;
		break;
	default:
		ret_val = -ENOTSUP;
		break;
	}

	if (ret_val == CLOCK_SUCCESS) {
		if (reg32 != NULL) {
			if (on == true) {
				*reg32 |= reg32_val;
			} else {
				*reg32 &= ~reg32_val;
			}
		} else if (reg16 != NULL) {
			if (on == true) {
				*reg16 |= reg16_val;
			} else {
				*reg16 &= ~reg16_val;
			}
		} else if (reg8 != NULL) {
			if (on == true) {
				*reg8 |= reg8_val;
			} else {
				*reg8 &= ~reg8_val;
			}
		}
	}

	return ret_val;
}

#if CONFIG_CLOCK_CONTROL_MCHP_GET_RATE
/**
 * @brief get rate of gclk generator in Hz.
 */
static int clock_get_rate_gclkgen(const struct device *dev, clock_mchp_gclkgen_t gclkgen_id,
				  clock_mchp_gclk_src_clock_t gclkgen_called_src, uint32_t *freq)
{
	const clock_mchp_config_t *config = dev->config;
	gclk_registers_t *gclk_regs = config->gclk_regs;
	clock_mchp_data_t *data = dev->data;

	int ret_val;
	clock_mchp_gclk_src_clock_t gclkgen_src;
	uint32_t gclkgen_src_freq;
	uint16_t gclkgen_div;
	bool power_div;

	ret_val = CLOCK_SUCCESS;
	power_div = (((gclk_regs->GCLK_GENCTRL[gclkgen_id] & GCLK_GENCTRL_DIVSEL_Msk) >>
		      GCLK_GENCTRL_DIVSEL_Pos) == GCLK_GENCTRL_DIVSEL_DIV1_Val)
			    ? false
			    : true;
	do {
		/* Return rate as 0, if clock is not on */
		if (clock_mchp_get_status(dev, (clock_control_subsys_t)MCHP_CLOCK_DERIVE_ID(
						       SUBSYS_TYPE_GCLKGEN, MBUS_NA, MMASK_NA,
						       GPH_NA, gclkgen_id)) !=
		    CLOCK_CONTROL_STATUS_ON) {
			*freq = 0;
			break;
		}

		/* get source for gclk generator from gclkgen registers */
		gclkgen_src = (gclk_regs->GCLK_GENCTRL[gclkgen_id] & GCLK_GENCTRL_SRC_Msk) >>
			      GCLK_GENCTRL_SRC_Pos;
		if (gclkgen_called_src == gclkgen_src) {
			ret_val = -ELOOP;
			break;
		}

		switch (gclkgen_src) {
		case CLOCK_MCHP_GCLK_SRC_XOSC0:
			gclkgen_src_freq = data->xosc_crystal_freq[INST_XOSC0];
			break;

		case CLOCK_MCHP_GCLK_SRC_XOSC1:
			gclkgen_src_freq = data->xosc_crystal_freq[INST_XOSC1];
			break;

		case CLOCK_MCHP_GCLK_SRC_DFLL:
			ret_val = clock_get_rate_dfll(dev, &gclkgen_src_freq);
			break;

		case CLOCK_MCHP_GCLK_SRC_FDPLL0:
			ret_val = clock_get_rate_fdpll(dev, INST_FDPLL0, &gclkgen_src_freq);
			break;

		case CLOCK_MCHP_GCLK_SRC_FDPLL1: {
			ret_val = clock_get_rate_fdpll(dev, INST_FDPLL1, &gclkgen_src_freq);
			break;
		}
		case CLOCK_MCHP_GCLK_SRC_OSCULP32K:
		case CLOCK_MCHP_GCLK_SRC_XOSC32K:
			gclkgen_src_freq = FREQ_32KHZ;
			break;

		case CLOCK_MCHP_GCLK_SRC_GCLKPIN:
			if (gclkgen_id <= GCLK_IO_MAX) {
				gclkgen_src_freq = data->xosc_crystal_freq[gclkgen_id];
			} else {
				ret_val = -ENOTSUP;
			}
			break;

		case CLOCK_MCHP_GCLK_SRC_GCLKGEN1:
			ret_val = (gclkgen_id == CLOCK_MCHP_GCLKGEN_GEN1)
					  ? -ELOOP
					  : clock_get_rate_gclkgen(dev, CLOCK_MCHP_GCLKGEN_GEN1,
								   CLOCK_MCHP_GCLK_SRC_MAX + 1,
								   &gclkgen_src_freq);
			break;
		default:
			break;
		}
		if (ret_val != CLOCK_SUCCESS) {
			break;
		}
		/* get gclk generator clock divider*/
		gclkgen_div = (gclk_regs->GCLK_GENCTRL[gclkgen_id] & GCLK_GENCTRL_DIV_Msk) >>
			      GCLK_GENCTRL_DIV_Pos;
		/*
		 * For gclk1, 8 division factor bits - DIV[7:0]
		 * others, 16 division factor bits - DIV[15:0]
		 */
		if (gclkgen_id != CLOCK_MCHP_GCLKGEN_GEN1) {
			gclkgen_div = gclkgen_div & 0xFF;
		}

		if (power_div == true) {
			if (gclkgen_div > GCLKGEN_POWER_DIV_MAX) {
				gclkgen_div = GCLKGEN_POWER_DIV_MAX;
			}
			gclkgen_div = 1 << (gclkgen_div + 1);
		} else {
			/* if DIV value is 0, has same effect as DIV value 1 */
			if (gclkgen_div == 0) {
				gclkgen_div = 1;
			}
		}
		*freq = gclkgen_src_freq / gclkgen_div;
	} while (0);
	return ret_val;
}

/**
 * @brief get rate of DFLL in Hz.
 */
static int clock_get_rate_dfll(const struct device *dev, uint32_t *freq)
{
	const clock_mchp_config_t *config = dev->config;
	oscctrl_registers_t *oscctrl_regs = config->oscctrl_regs;
	int ret_val;
	uint32_t multiply_factor, gclkgen_freq;
	clock_mchp_gclkgen_t src_gclkgen;

	ret_val = CLOCK_SUCCESS;
	if ((oscctrl_regs->OSCCTRL_STATUS & OSCCTRL_STATUS_DFLLRDY_Msk) == 0) {
		/* Return rate as 0, if clock is not on */
		*freq = 0;
	} else if ((oscctrl_regs->OSCCTRL_DFLLCTRLB & OSCCTRL_DFLLCTRLB_MODE_Msk) == 0) {
		/* in open loop mode*/
		*freq = FREQ_DFLL_48MHZ;
	} else {
		/* in closed loop mode*/
		multiply_factor = (oscctrl_regs->OSCCTRL_DFLLMUL & OSCCTRL_DFLLMUL_MUL_Msk) >>
				  OSCCTRL_DFLLMUL_MUL_Pos;
		/* PCHCTRL_0 is for DFLL*/
		src_gclkgen = (config->gclk_regs->GCLK_PCHCTRL[0] & GCLK_PCHCTRL_GEN_Msk) >>
			      GCLK_PCHCTRL_GEN_Pos;

		ret_val = clock_get_rate_gclkgen(dev, src_gclkgen, CLOCK_MCHP_GCLK_SRC_DFLL,
						 &gclkgen_freq);
		if (ret_val == CLOCK_SUCCESS) {
			*freq = multiply_factor * gclkgen_freq;
		}
	}
	return ret_val;
}

/**
 * @brief get rate of FDPLL in Hz.
 */
static int clock_get_rate_fdpll(const struct device *dev, uint8_t fdpll_id, uint32_t *freq)
{
	const clock_mchp_config_t *config = dev->config;
	oscctrl_registers_t *oscctrl_regs = config->oscctrl_regs;
	clock_mchp_data_t *data = dev->data;
	int ret_val;
	uint32_t src_freq, div, mult_int, mult_frac, frac_mult_max;
	clock_mchp_gclkgen_t src_gclkgen;
	uint8_t ref_clk_type;
	bool div_en;

	ret_val = CLOCK_SUCCESS;
	do {
		/* Return rate as 0, if clock is not on */
		if (clock_mchp_get_status(dev, (clock_control_subsys_t)MCHP_CLOCK_DERIVE_ID(
						       SUBSYS_TYPE_FDPLL, MBUS_NA, MMASK_NA,
						       fdpll_id + 1, fdpll_id)) !=
		    CLOCK_CONTROL_STATUS_ON) {
			*freq = 0;
			break;
		}

		ref_clk_type = (oscctrl_regs->DPLL[fdpll_id].OSCCTRL_DPLLCTRLB &
				OSCCTRL_DPLLCTRLB_REFCLK_Msk) >>
			       OSCCTRL_DPLLCTRLB_REFCLK_Pos;
		div_en = false;

		switch (ref_clk_type) {
		case OSCCTRL_DPLLCTRLB_REFCLK_GCLK_Val:
			src_gclkgen = (config->gclk_regs->GCLK_PCHCTRL[fdpll_id + 1] &
				       GCLK_PCHCTRL_GEN_Msk) >>
				      GCLK_PCHCTRL_GEN_Pos;
			ret_val = clock_get_rate_gclkgen(
				dev, src_gclkgen, CLOCK_MCHP_GCLK_SRC_FDPLL0 + fdpll_id, &src_freq);
			break;

		case OSCCTRL_DPLLCTRLB_REFCLK_XOSC32_Val:
			src_freq = FREQ_32KHZ;
			break;

		case OSCCTRL_DPLLCTRLB_REFCLK_XOSC0_Val:
			src_freq = data->xosc_crystal_freq[0];
			div_en = true;
			break;

		case OSCCTRL_DPLLCTRLB_REFCLK_XOSC1_Val:
			src_freq = data->xosc_crystal_freq[1];
			div_en = true;
			break;
		}
		if (ret_val != CLOCK_SUCCESS) {
			break;
		}
		if (div_en == true) {
			div = (oscctrl_regs->DPLL[fdpll_id].OSCCTRL_DPLLCTRLB &
			       OSCCTRL_DPLLCTRLB_DIV_Msk) >>
			      OSCCTRL_DPLLCTRLB_DIV_Pos;
			src_freq = src_freq / (2 * (div + 1));
		}
		mult_int = (oscctrl_regs->DPLL[fdpll_id].OSCCTRL_DPLLRATIO &
			    OSCCTRL_DPLLRATIO_LDR_Msk) >>
			   OSCCTRL_DPLLRATIO_LDR_Pos;
		mult_frac = (oscctrl_regs->DPLL[fdpll_id].OSCCTRL_DPLLRATIO &
			     OSCCTRL_DPLLRATIO_LDRFRAC_Msk) >>
			    OSCCTRL_DPLLRATIO_LDRFRAC_Pos;

		frac_mult_max = OSCCTRL_DPLLRATIO_LDRFRAC_Msk >> OSCCTRL_DPLLRATIO_LDRFRAC_Pos;
		*freq = (src_freq * (((mult_int + 1) * (frac_mult_max + 1)) + mult_frac)) /
			(frac_mult_max + 1);
	} while (0);

	return ret_val;
}
#endif /* CONFIG_CLOCK_CONTROL_MCHP_GET_RATE */

/******************************************************************************
 * @brief API functions
 *****************************************************************************/
#if CONFIG_CLOCK_CONTROL_MCHP_ASYNC_ON
/**
 * @brief Clock control interrupt service routine (ISR).
 *
 * @param dev Pointer to the  clock device structure.
 */
static void clock_mchp_isr(const struct device *dev)
{
	const clock_mchp_config_t *config = dev->config;
	clock_mchp_data_t *data = dev->data;

	/* Clear and disable the interrupt for the specified async subsystem. */
	clock_clear_interrupt(config, data->async_subsys);
	clock_disable_interrupt(config, data->async_subsys);

	if (data->async_cb != NULL) {
		data->async_cb(dev, (clock_control_subsys_t)data->async_subsys.val,
			       data->async_cb_user_data);
	}

	data->is_async_in_progress = false;
}
#endif /* CONFIG_CLOCK_CONTROL_MCHP_ASYNC_ON */

/**
 * @brief Turn on the clock for a specified subsystem, can be blocking.
 *
 * @param dev Pointer to the clock device structure
 * @param sys Clock subsystem
 *
 * @return 0 if the clock is successfully turned on
 * @return -ENOTSUP If the requested operation is not supported.
 * @return -ETIMEDOUT If the requested operation is timedout.
 * @return -EALREADY If clock is already on.
 */
static int clock_mchp_on(const struct device *dev, clock_control_subsys_t sys)
{
	const clock_mchp_config_t *config = dev->config;
	clock_mchp_subsys_t subsys;

	int ret_val;
	enum clock_control_status status;
	bool is_wait = false;
	uint32_t on_timeout_ms = 0;

	subsys.val = (uint32_t)sys;
	ret_val = -ENOTSUP;
	do {
		/* Validate subsystem. */
		if (CLOCK_SUCCESS != clock_check_subsys(subsys)) {
			break;
		}

		status = clock_mchp_get_status(dev, sys);
		if (status == CLOCK_CONTROL_STATUS_ON) {
			/* clock is already on. */
			ret_val = -EALREADY;
			break;
		}

		/* Check if the clock on operation is successful. */
		if (clock_on_off(config, subsys, true) == CLOCK_SUCCESS) {
			is_wait = true;
		}
	} while (0);

	/* Wait until the clock state becomes ON. */
	while (is_wait == true) {
		status = clock_mchp_get_status(dev, sys);
		if (status == CLOCK_CONTROL_STATUS_ON) {
			/* Successfully turned on clock. */
			ret_val = CLOCK_SUCCESS;
			break;
		}
		if (on_timeout_ms < config->on_timeout_ms) {
			/* Thread is not available while booting. */
			if ((k_is_pre_kernel() == false) && (k_current_get() != NULL)) {
				/* Sleep before checking again. */
				k_sleep(K_MSEC(1));
				on_timeout_ms++;
			}
		} else {
			/* Clock on timeout occurred */
			ret_val = -ETIMEDOUT;
			break;
		}
	}

	return ret_val;
}

/**
 * @brief Turn off the clock for a specified subsystem.
 *
 * @param dev Pointer to the clock device structure
 * @param sys Clock subsystem
 *
 * @return 0 if the clock is successfully turned off
 * @return -ENOTSUP If the requested operation is not supported.
 * @return -EALREADY If clock is already off.
 */
static int clock_mchp_off(const struct device *dev, clock_control_subsys_t sys)
{
	const clock_mchp_config_t *config = dev->config;
	clock_mchp_subsys_t subsys;

	/* Return value for the operation status. */
	int ret_val;
	enum clock_control_status status;

	subsys.val = (uint32_t)sys;
	ret_val = -ENOTSUP;
	do {
		/* Validate subsystem. */
		if (CLOCK_SUCCESS != clock_check_subsys(subsys)) {
			break;
		}
		status = clock_mchp_get_status(dev, sys);
		if (status == CLOCK_CONTROL_STATUS_OFF) {
			/* clock is already off. */
			ret_val = -EALREADY;
			break;
		}

#if CONFIG_CLOCK_CONTROL_MCHP_ASYNC_ON
		clock_mchp_data_t *data = dev->data;

		/* Check whether an async operation is initiated for this clock */
		if ((data->is_async_in_progress == true) &&
		    (data->async_subsys.bits.type == subsys.bits.type) &&
		    (data->async_subsys.bits.inst == subsys.bits.inst)) {
			/* The clock is starting. disable interrupts */
			clock_disable_interrupt(config, subsys);
			data->is_async_in_progress = false;
		}
#endif /* CONFIG_CLOCK_CONTROL_MCHP_ASYNC_ON */

		ret_val = clock_on_off(config, subsys, false);
	} while (0);

	return ret_val;
}

/**
 * @brief Get the status of clock, for a specified subsystem.
 *
 * This function retrieves the current status of the clock for a given subsystem.
 *
 * @param dev Pointer to the clock device structure.
 * @param sys The clock subsystem.
 *
 * @return The current status of clock for the subsystem (e.g., off, on, starting, or unknown).
 */
static enum clock_control_status clock_mchp_get_status(const struct device *dev,
						       clock_control_subsys_t sys)
{
	const clock_mchp_config_t *config = dev->config;
	oscctrl_registers_t *oscctrl_regs = config->oscctrl_regs;
	osc32kctrl_registers_t *osc32kctrl_regs = config->osc32kctrl_regs;
	gclk_registers_t *gclk_regs = config->gclk_regs;
	clock_mchp_subsys_t subsys;
	uint32_t mask;
	uint8_t inst;

	__IO uint32_t *reg32;
	/* Variable to store the returned status to indicate state is not yet known. */
	enum clock_control_status ret_status = CLOCK_CONTROL_STATUS_UNKNOWN;

	subsys.val = (uint32_t)sys;

	do {
		/* Validate subsystem. */
		if (CLOCK_SUCCESS != clock_check_subsys(subsys)) {
			break;
		}

		inst = subsys.bits.inst;
#if CONFIG_CLOCK_CONTROL_MCHP_ASYNC_ON
		clock_mchp_data_t *data = dev->data;

		/* Check whether an async operation is initiated for this clock */
		if ((data->is_async_in_progress == true) &&
		    (data->async_subsys.bits.type == subsys.bits.type) &&
		    (data->async_subsys.bits.inst == inst)) {
			/* The clock async operation is in progress. */
			ret_status = CLOCK_CONTROL_STATUS_STARTING;
			break;
		}
#endif /* CONFIG_CLOCK_CONTROL_MCHP_ASYNC_ON */

		switch (subsys.bits.type) {
		case SUBSYS_TYPE_XOSC:
			/* Check if XOSC is enabled */
			if ((oscctrl_regs->OSCCTRL_XOSCCTRL[inst] & OSCCTRL_XOSCCTRL_ENABLE_Msk) !=
			    0) {
				mask = (inst == INST_XOSC0) ? OSCCTRL_STATUS_XOSCRDY0_Msk
							    : OSCCTRL_STATUS_XOSCRDY1_Msk;
				/* Check if ready bit is set */
				ret_status = ((oscctrl_regs->OSCCTRL_STATUS & mask) == 0)
						     ? CLOCK_CONTROL_STATUS_STARTING
						     : CLOCK_CONTROL_STATUS_ON;
			} else {
				ret_status = CLOCK_CONTROL_STATUS_OFF;
			}

			break;
		case SUBSYS_TYPE_DFLL:
			/* Check if DFLL is enabled */
			if ((oscctrl_regs->OSCCTRL_DFLLCTRLA & OSCCTRL_DFLLCTRLA_ENABLE_Msk) != 0) {
				/* Check if sync is complete and ready bit is set */
				ret_status = ((oscctrl_regs->OSCCTRL_DFLLSYNC != 0) ||
					      ((oscctrl_regs->OSCCTRL_STATUS &
						OSCCTRL_STATUS_DFLLRDY_Msk) == 0))
						     ? CLOCK_CONTROL_STATUS_STARTING
						     : CLOCK_CONTROL_STATUS_ON;
			} else {
				ret_status = CLOCK_CONTROL_STATUS_OFF;
			}

			break;
		case SUBSYS_TYPE_FDPLL:
			/* Check if DPLL is enabled */
			if ((oscctrl_regs->DPLL[inst].OSCCTRL_DPLLCTRLA &
			     OSCCTRL_DPLLCTRLA_ENABLE_Msk) != 0) {
				mask = OSCCTRL_DPLLSTATUS_LOCK_Msk | OSCCTRL_DPLLSTATUS_CLKRDY_Msk;
				/* Check if sync is complete and ready bit is set */
				ret_status =
					((oscctrl_regs->DPLL[inst].OSCCTRL_DPLLSYNCBUSY != 0) ||
					 ((oscctrl_regs->DPLL[inst].OSCCTRL_DPLLSTATUS & mask) !=
					  mask))
						? CLOCK_CONTROL_STATUS_STARTING
						: CLOCK_CONTROL_STATUS_ON;
			} else {
				ret_status = CLOCK_CONTROL_STATUS_OFF;
			}
			break;
		case SUBSYS_TYPE_RTC:
			ret_status = CLOCK_CONTROL_STATUS_ON;
			break;
		case SUBSYS_TYPE_OSC32K:
			switch (inst) {
			case INST_OSC32K_OSCULP1K:
				ret_status = ((osc32kctrl_regs->OSC32KCTRL_OSCULP32K &
					       OSC32KCTRL_OSCULP32K_EN1K_Msk) != 0)
						     ? CLOCK_CONTROL_STATUS_ON
						     : CLOCK_CONTROL_STATUS_OFF;
				break;
			case INST_OSC32K_OSCULP32K:
				ret_status = ((osc32kctrl_regs->OSC32KCTRL_OSCULP32K &
					       OSC32KCTRL_OSCULP32K_EN32K_Msk) != 0)
						     ? CLOCK_CONTROL_STATUS_ON
						     : CLOCK_CONTROL_STATUS_OFF;
				break;
			case INST_OSC32K_XOSC1K:
				ret_status = ((osc32kctrl_regs->OSC32KCTRL_XOSC32K &
					       OSC32KCTRL_XOSC32K_EN1K_Msk) != 0)
						     ? CLOCK_CONTROL_STATUS_ON
						     : CLOCK_CONTROL_STATUS_OFF;
				break;
			case INST_OSC32K_XOSC32K:
				/* Check if XOSC32K is enabled */
				if ((osc32kctrl_regs->OSC32KCTRL_XOSC32K &
				     OSC32KCTRL_XOSC32K_EN32K_Msk) != 0) {
					/* Check if ready bit is set */
					ret_status = ((osc32kctrl_regs->OSC32KCTRL_STATUS &
						       OSC32KCTRL_STATUS_XOSC32KRDY_Msk) == 0)
							     ? CLOCK_CONTROL_STATUS_STARTING
							     : CLOCK_CONTROL_STATUS_ON;
				} else {
					ret_status = CLOCK_CONTROL_STATUS_OFF;
				}
				break;
			}
			break;
		case SUBSYS_TYPE_GCLKGEN:
			ret_status = CLOCK_CONTROL_STATUS_OFF;
			if ((gclk_regs->GCLK_GENCTRL[inst] & GCLK_GENCTRL_GENEN_Msk) != 0) {
				/* Generator is on, check if it's starting or fully on */
				ret_status = ((gclk_regs->GCLK_SYNCBUSY &
					       (1 << (GCLK_SYNCBUSY_GENCTRL_Pos + inst))) != 0)
						     ? CLOCK_CONTROL_STATUS_STARTING
						     : CLOCK_CONTROL_STATUS_ON;
			}
			break;
		case SUBSYS_TYPE_GCLKPERIPH:
			/* Check if the peripheral clock is enabled */
			ret_status = ((gclk_regs->GCLK_PCHCTRL[subsys.bits.gclkperiph] &
				       GCLK_PCHCTRL_CHEN_Msk) != 0)
					     ? CLOCK_CONTROL_STATUS_ON
					     : CLOCK_CONTROL_STATUS_OFF;
			break;
		case SUBSYS_TYPE_MCLKCPU:
			ret_status = CLOCK_CONTROL_STATUS_ON;
			break;
		case SUBSYS_TYPE_MCLKPERIPH:
			reg32 = get_mclkbus_mask_reg(config->mclk_regs, subsys.bits.mclkbus);
			mask = 1 << subsys.bits.mclkmaskbit;
			ret_status = ((*reg32 & mask) != 0) ? CLOCK_CONTROL_STATUS_ON
							    : CLOCK_CONTROL_STATUS_OFF;

			break;
		}
	} while (0);

	/* Return the status of the clock for the specified subsystem. */
	return ret_status;
}

#if CONFIG_CLOCK_CONTROL_MCHP_ASYNC_ON
/**
 * @brief Turn on the clock for a specified subsystem, without blocking.
 *
 * @param dev Pointer to the clock device structure
 * @param sys Clock subsystem
 * @param cb Callback function
 * @param user_data User data to be passed in callback
 *
 * @return 0 if the clock is successfully turned on
 * @return -ENOTSUP If the requested operation is not supported.
 * @return -EBUSY If an async call is already in progress.
 * @return -EALREADY If clock is already on, or starting.
 */
static int clock_mchp_async_on(const struct device *dev, clock_control_subsys_t sys,
			       clock_control_cb_t cb, void *user_data)
{
	const clock_mchp_config_t *config = dev->config;
	clock_mchp_data_t *data = dev->data;
	clock_mchp_subsys_t subsys;

	/* Return value for the operation status. */
	int ret_val;
	enum clock_control_status status;

	subsys.val = (uint32_t)sys;
	ret_val = -ENOTSUP;
	do {
		/* Check if an async operation is already in progress. */
		if (data->is_async_in_progress == true) {
			ret_val = -EBUSY;
			break;
		}
		/* Validate subsystem. */
		if (CLOCK_SUCCESS != clock_check_subsys(subsys)) {
			break;
		}

		/* Get the current status of the clock. */
		status = clock_mchp_get_status(dev, sys);
		/* Check if the clock is already on or starting. */
		if ((status == CLOCK_CONTROL_STATUS_ON) ||
		    (status == CLOCK_CONTROL_STATUS_STARTING)) {
			ret_val = -EALREADY;
			break;
		}

		/* Check if interrupt is supported by this clock subsystem */
		switch (subsys.bits.type) {
		case SUBSYS_TYPE_XOSC:
		case SUBSYS_TYPE_FDPLL:
		case SUBSYS_TYPE_DFLL:
		case SUBSYS_TYPE_OSC32K:
			/* Clear the interrupt before enabling */
			clock_clear_interrupt(config, subsys);
			clock_enable_interrupt(config, subsys);
			/* Store async data to context. */
			data->async_subsys.bits.type = subsys.bits.type;
			data->async_subsys.bits.inst = subsys.bits.inst;
			data->async_cb = cb;
			data->async_cb_user_data = user_data;
			/* Set flag to indicate async operation is in progress. */
			data->is_async_in_progress = true;
			/* Clock interrupt is enabled, attempt to turn it on. */
			ret_val = clock_on_off(config, subsys, true);
			break;

		default:
			/* if interrupt is not supported, turn on clock and invoke callback here */
			ret_val = clock_on_off(config, subsys, true);
			if ((ret_val == CLOCK_SUCCESS) && (cb != NULL)) {
				cb(dev, (clock_control_subsys_t)subsys.val, user_data);
			}
			break;
		}

	} while (0);

	return ret_val;
}
#endif /* CONFIG_CLOCK_CONTROL_MCHP_ASYNC_ON */

#if CONFIG_CLOCK_CONTROL_MCHP_GET_RATE
/**
 * @brief Get the rate of the clock for a specified subsystem.
 *
 * This function retrieves the clock frequency (Hz) for the given subsystem.
 *
 * @param dev Pointer to clock device structure.
 * @param sys The clock subsystem.
 * @param frequency Pointer to store the retrieved clock rate.
 *
 * @return 0 if the rate is successfully retrieved.
 * @return -ENOTSUP If the requested operation is not supported.
 */
static int clock_mchp_get_rate(const struct device *dev, clock_control_subsys_t sys, uint32_t *freq)
{
	const clock_mchp_config_t *config = dev->config;
	clock_mchp_data_t *data = dev->data;
	int ret_val;
	uint8_t rtc_src, cpu_div, inst;
	uint32_t gclkgen_src_freq;
	clock_mchp_gclkgen_t gclkperiph_src;
	clock_mchp_subsys_t subsys;

	subsys.val = (uint32_t)sys;
	ret_val = CLOCK_SUCCESS;
	inst = subsys.bits.inst;
	do {
		/* Validate subsystem. */
		if (CLOCK_SUCCESS != clock_check_subsys(subsys)) {
			ret_val = -ENOTSUP;
			break;
		}

		/* Return rate as 0, if clock is not on */
		if (clock_mchp_get_status(dev, sys) != CLOCK_CONTROL_STATUS_ON) {
			*freq = 0;
			break;
		}

		switch (subsys.bits.type) {
		case SUBSYS_TYPE_XOSC:
			*freq = data->xosc_crystal_freq[inst];
			break;

		case SUBSYS_TYPE_DFLL:
			ret_val = clock_get_rate_dfll(dev, freq);
			break;

		case SUBSYS_TYPE_FDPLL:
			ret_val = clock_get_rate_fdpll(dev, inst, freq);
			break;

		case SUBSYS_TYPE_RTC:
			/* get rtc source clock*/
			rtc_src = config->osc32kctrl_regs->OSC32KCTRL_RTCCTRL &
				  OSC32KCTRL_RTCCTRL_RTCSEL_Msk;
			if ((rtc_src == OSC32KCTRL_RTCCTRL_RTCSEL_ULP1K_Val) ||
			    (rtc_src == OSC32KCTRL_RTCCTRL_RTCSEL_XOSC1K_Val)) {
				*freq = FREQ_1KHZ;

			} else if ((rtc_src == OSC32KCTRL_RTCCTRL_RTCSEL_ULP32K_Val) ||
				   (rtc_src == OSC32KCTRL_RTCCTRL_RTCSEL_XOSC32K_Val)) {
				*freq = FREQ_32KHZ;
			} else {
				ret_val = -ENOTSUP;
			}
			break;

		case SUBSYS_TYPE_OSC32K:
			if ((inst == INST_OSC32K_OSCULP1K) || (inst == INST_OSC32K_XOSC1K)) {
				*freq = FREQ_1KHZ;

			} else if ((inst == INST_OSC32K_OSCULP32K) ||
				   (inst == INST_OSC32K_XOSC32K)) {
				*freq = FREQ_32KHZ;
			}
			break;

		case SUBSYS_TYPE_GCLKGEN:
			ret_val = clock_get_rate_gclkgen(dev, inst, CLOCK_MCHP_GCLK_SRC_MAX + 1,
							 freq);
			break;

		case SUBSYS_TYPE_GCLKPERIPH:
			gclkperiph_src = (config->gclk_regs->GCLK_PCHCTRL[subsys.bits.gclkperiph] &
					  GCLK_PCHCTRL_GEN_Msk) >>
					 GCLK_PCHCTRL_GEN_Pos;
			ret_val = clock_get_rate_gclkgen(dev, gclkperiph_src,
							 CLOCK_MCHP_GCLK_SRC_MAX + 1, freq);
			break;

		case SUBSYS_TYPE_MCLKCPU:
		case SUBSYS_TYPE_MCLKPERIPH:
			/* source for mclk is always gclk0 */
			ret_val = clock_get_rate_gclkgen(dev, 0, CLOCK_MCHP_GCLK_SRC_MAX + 1,
							 &gclkgen_src_freq);
			if (ret_val == CLOCK_SUCCESS) {
				cpu_div = (config->mclk_regs->MCLK_CPUDIV & MCLK_CPUDIV_DIV_Msk) >>
					  MCLK_CPUDIV_DIV_Pos;
				if (cpu_div != 0) {
					*freq = gclkgen_src_freq / cpu_div;
				}
			}
			break;
		}
	} while (0);

	return ret_val;
}

#if CONFIG_CLOCK_CONTROL_MCHP_SET_RATE
/**
 * @brief Set the rate for the specified clock subsystem.
 *
 * This function attempts to set the rate for a given subsystem's clock.
 * Only the parameters in respective clock blocks are modified to get the rate.
 * Parameters of source clocks are not considered to modify.
 *
 * @param dev Pointer to the clock device structure.
 * @param sys The clock subsystem.
 * @param rate The desired clock rate in Hz.
 *
 * @return 0 if the rate is successfully set.
 * @return -ENOTSUP If the requested operation is not supported.
 */
static int clock_mchp_set_rate(const struct device *dev, clock_control_subsys_t sys,
			       clock_control_subsys_rate_t rate_arg)
{
	const clock_mchp_config_t *config = dev->config;
	oscctrl_registers_t *oscctrl_regs = config->oscctrl_regs;
	gclk_registers_t *gclk_regs = config->gclk_regs;
	clock_mchp_data_t *data = dev->data;
	clock_mchp_subsys_t subsys;

	int ret_val;
	uint32_t src_freq, calc, calc_freq_in, div, mult_int, mult_frac, int_mult_max,
		frac_mult_max, div_max;
	uint32_t rate = *(uint32_t *)rate_arg;
	clock_mchp_gclkgen_t src_gclkgen;
	uint8_t ref_clk_type, inst;
	bool div_en, power_div;

	subsys.val = (uint32_t)sys;
	ret_val = -ENOTSUP;

	/* Validate subsystem. */
	if (CLOCK_SUCCESS != clock_check_subsys(subsys)) {
		/* Has to return from here, else compliance check give many leading tabs error */
		return ret_val;
	}

	/* check if rate is 0 */
	if (rate == 0) {
		/* Has to return from here, else compliance check give many leading tabs error */
		return ret_val;
	}

	inst = subsys.bits.inst;
	switch (subsys.bits.type) {
	case SUBSYS_TYPE_XOSC:
	case SUBSYS_TYPE_RTC:
	case SUBSYS_TYPE_OSC32K:
	case SUBSYS_TYPE_MCLKPERIPH:
	case SUBSYS_TYPE_GCLKPERIPH:
		break;
	case SUBSYS_TYPE_DFLL:
		if ((oscctrl_regs->OSCCTRL_DFLLCTRLB & OSCCTRL_DFLLCTRLB_MODE_Msk) != 0) {
			/* in closed loop mode*/
			/* PCHCTRL_0 is for DFLL*/
			src_gclkgen = (gclk_regs->GCLK_PCHCTRL[0] & GCLK_PCHCTRL_GEN_Msk) >>
				      GCLK_PCHCTRL_GEN_Pos;

			if (CLOCK_SUCCESS == clock_get_rate_gclkgen(dev, src_gclkgen,
								    CLOCK_MCHP_GCLK_SRC_DFLL,
								    &src_freq)) {
				if (src_freq == 0) {
					break;
				}
				mult_int = rate / src_freq;
				if (((rate % src_freq) == 0) && (mult_int <= 0xFFFF)) {
					oscctrl_regs->OSCCTRL_DFLLMUL &= ~OSCCTRL_DFLLMUL_MUL_Msk;
					oscctrl_regs->OSCCTRL_DFLLMUL |=
						OSCCTRL_DFLLMUL_MUL(mult_int);
					ret_val = CLOCK_SUCCESS;
				}
			}
		} /* else in open loop mode & have fixed rate */
		break;

	case SUBSYS_TYPE_FDPLL:
		ref_clk_type = (oscctrl_regs->DPLL[inst].OSCCTRL_DPLLCTRLB &
				OSCCTRL_DPLLCTRLB_REFCLK_Msk) >>
			       OSCCTRL_DPLLCTRLB_REFCLK_Pos;
		div_en = false;
		if (ref_clk_type == OSCCTRL_DPLLCTRLB_REFCLK_GCLK_Val) {
			src_gclkgen = (gclk_regs->GCLK_PCHCTRL[inst + 1] & GCLK_PCHCTRL_GEN_Msk) >>
				      GCLK_PCHCTRL_GEN_Pos;
			if (clock_get_rate_gclkgen(dev, src_gclkgen,
						   CLOCK_MCHP_GCLK_SRC_FDPLL0 + inst,
						   &src_freq) != CLOCK_SUCCESS) {
				break;
			}
		} else if (ref_clk_type == OSCCTRL_DPLLCTRLB_REFCLK_XOSC32_Val) {
			src_freq = FREQ_32KHZ;
		} else if (ref_clk_type == OSCCTRL_DPLLCTRLB_REFCLK_XOSC0_Val) {
			src_freq = data->xosc_crystal_freq[0];
			div_en = true;
		} else if (ref_clk_type == OSCCTRL_DPLLCTRLB_REFCLK_XOSC1_Val) {
			src_freq = data->xosc_crystal_freq[1];
			div_en = true;
		} else {
			break;
		}

		/* Range of values to write in register is from 0, which have "+ 1" effect
		 */
		int_mult_max = OSCCTRL_DPLLRATIO_LDR_Msk >> OSCCTRL_DPLLRATIO_LDR_Pos;
		frac_mult_max = OSCCTRL_DPLLRATIO_LDRFRAC_Msk >> OSCCTRL_DPLLRATIO_LDRFRAC_Pos;
		div_max = OSCCTRL_DPLLCTRLB_DIV_Msk >> OSCCTRL_DPLLCTRLB_DIV_Pos;

		div = 0;
		do {
			calc_freq_in = (div_en == true) ? (src_freq / (2 * (div + 1))) : src_freq;
			/* iterate to find correct mult_int and mult_frac */
			for (mult_int = 0; mult_int <= int_mult_max; mult_int++) {
				if (((calc_freq_in * (mult_int + 1)) > rate) ||
				    (ret_val == CLOCK_SUCCESS)) {
					/* break if value is above required freq */
					break;
				}
				for (mult_frac = 0; mult_frac <= frac_mult_max; mult_frac++) {
					calc = calc_freq_in *
					       (((mult_int + 1) * (frac_mult_max + 1)) +
						mult_frac) /
					       (frac_mult_max + 1);
					if ((calc == rate) && (div_en == true)) {
						oscctrl_regs->DPLL[inst].OSCCTRL_DPLLCTRLB &=
							~OSCCTRL_DPLLCTRLB_DIV_Msk;
						oscctrl_regs->DPLL[inst].OSCCTRL_DPLLCTRLB |=
							OSCCTRL_DPLLCTRLB_DIV(div);
					}
					if (calc == rate) {
						/* found matching values */
						oscctrl_regs->DPLL[inst].OSCCTRL_DPLLRATIO =
							OSCCTRL_DPLLRATIO_LDR(mult_int) |
							OSCCTRL_DPLLRATIO_LDRFRAC(mult_frac);
						ret_val = CLOCK_SUCCESS;
						break;
					}
				}
			}
			div++;
		} while ((div_en == true) && (div <= div_max) && (ret_val != CLOCK_SUCCESS));
		break;

	case SUBSYS_TYPE_GCLKGEN:
		if (clock_get_rate_gclkgen(dev, inst, CLOCK_MCHP_GCLK_SRC_MAX + 1, &src_freq) !=
		    CLOCK_SUCCESS) {
			break;
		}
		power_div = (((gclk_regs->GCLK_GENCTRL[inst] & GCLK_GENCTRL_DIVSEL_Msk) >>
			      GCLK_GENCTRL_DIVSEL_Pos) == GCLK_GENCTRL_DIVSEL_DIV1_Val)
				    ? false
				    : true;
		div = (gclk_regs->GCLK_GENCTRL[inst] & GCLK_GENCTRL_DIV_Msk) >>
		      GCLK_GENCTRL_DIV_Pos;
		if (power_div == true) {
			src_freq = src_freq << (div + 1);
		} else {
			if (div == 0) {
				div++;
			}
			src_freq *= div;
		}

		div = src_freq / rate;
		div_max = GCLK_GENCTRL_DIV_Msk >> GCLK_GENCTRL_DIV_Pos;
		/*
		 * For gclk1, 8 division factor bits - DIV[7:0]
		 * others, 16 division factor bits - DIV[15:0]
		 */
		if (inst != CLOCK_MCHP_GCLKGEN_GEN1) {
			div_max = div_max & 0xFF;
		}
		if (power_div == false) {
			if (((src_freq % rate) == 0) && (div <= div_max)) {
				gclk_regs->GCLK_GENCTRL[inst] &= ~GCLK_GENCTRL_DIV_Msk;
				gclk_regs->GCLK_GENCTRL[inst] |= GCLK_GENCTRL_DIV(div);
				ret_val = CLOCK_SUCCESS;
			}
		} else {
			/* Check if div is power of 2 */
			if (((src_freq % rate) == 0) && ((div & (div - 1)) == 0)) {
				calc = 0;
				while (div > 1) {
					div >>= 1;
					calc++;
				}
				gclk_regs->GCLK_GENCTRL[inst] &= ~GCLK_GENCTRL_DIV_Msk;
				gclk_regs->GCLK_GENCTRL[inst] |= GCLK_GENCTRL_DIV(calc - 1);
				ret_val = CLOCK_SUCCESS;
			}
		}
		break;

	case SUBSYS_TYPE_MCLKCPU:
		/* source for mclk is always gclk0 */
		if (clock_get_rate_gclkgen(dev, 0, CLOCK_MCHP_GCLK_SRC_MAX + 1, &src_freq) !=
		    CLOCK_SUCCESS) {
			break;
		}
		div = src_freq / rate;
		if ((src_freq % rate) == 0) {
			switch (div) {
			case MCLK_CPUDIV_DIV_DIV1_Val:
			case MCLK_CPUDIV_DIV_DIV2_Val:
			case MCLK_CPUDIV_DIV_DIV4_Val:
			case MCLK_CPUDIV_DIV_DIV8_Val:
			case MCLK_CPUDIV_DIV_DIV16_Val:
			case MCLK_CPUDIV_DIV_DIV32_Val:
			case MCLK_CPUDIV_DIV_DIV64_Val:
			case MCLK_CPUDIV_DIV_DIV128_Val:
				config->mclk_regs->MCLK_CPUDIV = MCLK_CPUDIV_DIV(div);
				ret_val = CLOCK_SUCCESS;
				break;
			default:
			}
		}
		break;
	}

	return ret_val;
}
#endif /* CONFIG_CLOCK_CONTROL_MCHP_SET_RATE */
#endif /* CONFIG_CLOCK_CONTROL_MCHP_GET_RATE */

#if CONFIG_CLOCK_CONTROL_MCHP_CONFIG_RUNTIME
/**
 * @brief Configure the clock for a specified subsystem.
 *
 * req_config is typecasted to corresponding structure type, according to the clock
 * subsystem.
 *
 * @param dev Pointer to clock device structure.
 * @param sys The clock subsystem.
 * @param req_config Pointer to the requested configuration for the clock.
 *
 * @return 0 if the configuration is successful.
 * @return -EINVAL if req_config is not a valid value.
 * @return -ENOTSUP If the requested operation is not supported.
 */
static int clock_mchp_configure(const struct device *dev, clock_control_subsys_t sys,
				void *req_config)
{
	const clock_mchp_config_t *config = dev->config;
	oscctrl_registers_t *oscctrl_regs = config->oscctrl_regs;
	osc32kctrl_registers_t *osc32kctrl_regs = config->osc32kctrl_regs;
	gclk_registers_t *gclk_regs = config->gclk_regs;
	clock_mchp_subsys_t subsys;

	int ret_val;
	uint32_t val, reg_val;
	uint16_t inst;

	subsys.val = (uint32_t)sys;
	val = 0;
	inst = subsys.bits.inst;

	ret_val = CLOCK_SUCCESS;
	do {
		if (req_config == NULL) {
			ret_val = -EINVAL;
			break;
		}
		/* Validate subsystem. */
		if (CLOCK_SUCCESS != clock_check_subsys(subsys)) {
			ret_val = -ENOTSUP;
			break;
		}

		switch (subsys.bits.type) {
		case SUBSYS_TYPE_XOSC:
			clock_mchp_subsys_xosc_config_t *xosc_config =
				(clock_mchp_subsys_xosc_config_t *)req_config;
			reg_val = oscctrl_regs->OSCCTRL_XOSCCTRL[inst];
			reg_val &= ~(OSCCTRL_XOSCCTRL_RUNSTDBY_Msk | OSCCTRL_XOSCCTRL_ONDEMAND_Msk);
			val |= ((xosc_config->run_in_standby_en != 0) ? OSCCTRL_XOSCCTRL_RUNSTDBY(1)
								      : 0);
			val |= ((xosc_config->on_demand_en != 0) ? OSCCTRL_XOSCCTRL_ONDEMAND(1)
								 : 0);
			reg_val |= val;
			oscctrl_regs->OSCCTRL_XOSCCTRL[inst] = reg_val;
			break;

		case SUBSYS_TYPE_DFLL:
			clock_mchp_subsys_dfll_config_t *dfll_config =
				(clock_mchp_subsys_dfll_config_t *)req_config;
			/* GCLK_PCHCTRL[0] is for DFLL48 input clock source */
			reg_val = gclk_regs->GCLK_PCHCTRL[0];
			reg_val &= ~GCLK_PCHCTRL_GEN_Msk;
			reg_val |= GCLK_PCHCTRL_GEN(dfll_config->src);
			gclk_regs->GCLK_PCHCTRL[0] = reg_val;

			if (dfll_config->closed_loop_en == true) {
				reg_val = oscctrl_regs->OSCCTRL_DFLLMUL;
				reg_val &= ~OSCCTRL_DFLLMUL_MUL_Msk;
				reg_val |= OSCCTRL_DFLLMUL_MUL(dfll_config->multiply_factor);
				oscctrl_regs->OSCCTRL_DFLLMUL = reg_val;

				reg_val = oscctrl_regs->OSCCTRL_DFLLCTRLB;
				reg_val &= ~OSCCTRL_DFLLCTRLB_MODE_Msk;
				reg_val |= OSCCTRL_DFLLCTRLB_MODE(1);
				oscctrl_regs->OSCCTRL_DFLLCTRLB = reg_val;
			}

			reg_val = oscctrl_regs->OSCCTRL_DFLLCTRLA;
			reg_val &=
				~(OSCCTRL_DFLLCTRLA_RUNSTDBY_Msk | OSCCTRL_DFLLCTRLA_ONDEMAND_Msk);
			val |= ((dfll_config->run_in_standby_en != 0)
					? OSCCTRL_DFLLCTRLA_RUNSTDBY(1)
					: 0);
			val |= ((dfll_config->on_demand_en != 0) ? OSCCTRL_DFLLCTRLA_ONDEMAND(1)
								 : 0);
			reg_val |= val;
			oscctrl_regs->OSCCTRL_DFLLCTRLA = reg_val;
			break;

		case SUBSYS_TYPE_FDPLL:
			clock_mchp_subsys_fdpll_config_t *fdpll_config =
				(clock_mchp_subsys_fdpll_config_t *)req_config;
			if (fdpll_config->src <= CLOCK_MCHP_FDPLL_SRC_XOSC1) {
				switch (fdpll_config->src) {
				case CLOCK_MCHP_FDPLL_SRC_XOSC32K:
					val |= OSCCTRL_DPLLCTRLB_REFCLK_XOSC32;
					break;

				case CLOCK_MCHP_FDPLL_SRC_XOSC0:
					val |= OSCCTRL_DPLLCTRLB_REFCLK_XOSC0;
					break;

				case CLOCK_MCHP_FDPLL_SRC_XOSC1:
					val |= OSCCTRL_DPLLCTRLB_REFCLK_XOSC1;
					break;

				default:
					val |= OSCCTRL_DPLLCTRLB_REFCLK_GCLK;
					/* source is gclk*/
					gclk_regs->GCLK_PCHCTRL[inst + 1] &= ~GCLK_PCHCTRL_GEN_Msk;
					gclk_regs->GCLK_PCHCTRL[inst + 1] |=
						GCLK_PCHCTRL_GEN(fdpll_config->src);
					break;
				}
				reg_val = oscctrl_regs->DPLL[inst].OSCCTRL_DPLLCTRLB;
				reg_val &= ~OSCCTRL_DPLLCTRLB_REFCLK_Msk;
				reg_val |= val;
				oscctrl_regs->DPLL[inst].OSCCTRL_DPLLCTRLB = reg_val;
			}

			reg_val = oscctrl_regs->DPLL[inst].OSCCTRL_DPLLCTRLB;
			reg_val &= ~OSCCTRL_DPLLCTRLB_DIV_Msk;
			reg_val |= OSCCTRL_DPLLCTRLB_DIV(fdpll_config->xosc_clock_divider);
			oscctrl_regs->DPLL[inst].OSCCTRL_DPLLCTRLB = reg_val;

			/* DPLLRATIO */
			val = 0;
			val |= OSCCTRL_DPLLRATIO_LDR(fdpll_config->divider_ratio_int);
			val |= OSCCTRL_DPLLRATIO_LDRFRAC(fdpll_config->divider_ratio_frac);
			oscctrl_regs->DPLL[inst].OSCCTRL_DPLLRATIO = val;

			/* DPLLCTRLA */
			val = 0;
			reg_val = oscctrl_regs->DPLL[inst].OSCCTRL_DPLLCTRLA;
			reg_val &=
				~(OSCCTRL_DPLLCTRLA_RUNSTDBY_Msk | OSCCTRL_DPLLCTRLA_ONDEMAND_Msk);
			val |= ((fdpll_config->run_in_standby_en != 0)
					? OSCCTRL_DPLLCTRLA_RUNSTDBY(1)
					: 0);
			val |= ((fdpll_config->on_demand_en != 0) ? OSCCTRL_DPLLCTRLA_ONDEMAND(1)
								  : 0);
			reg_val |= val;
			oscctrl_regs->DPLL[inst].OSCCTRL_DPLLCTRLA = reg_val;
			break;

		case SUBSYS_TYPE_RTC:
			clock_mchp_subsys_rtc_config_t *rtc_config =
				(clock_mchp_subsys_rtc_config_t *)req_config;
			osc32kctrl_regs->OSC32KCTRL_RTCCTRL =
				OSC32KCTRL_RTCCTRL_RTCSEL(rtc_config->src);
			break;

		case SUBSYS_TYPE_OSC32K:
			clock_mchp_subsys_osc32k_config_t *osc32k_config =
				(clock_mchp_subsys_osc32k_config_t *)req_config;

			reg_val = osc32kctrl_regs->OSC32KCTRL_XOSC32K;
			reg_val &= ~(OSC32KCTRL_XOSC32K_RUNSTDBY_Msk |
				     OSC32KCTRL_XOSC32K_ONDEMAND_Msk);
			val |= ((osc32k_config->run_in_standby_en != 0)
					? OSC32KCTRL_XOSC32K_RUNSTDBY(1)
					: 0);
			val |= ((osc32k_config->on_demand_en != 0) ? OSC32KCTRL_XOSC32K_ONDEMAND(1)
								   : 0);
			reg_val |= val;
			osc32kctrl_regs->OSC32KCTRL_XOSC32K = reg_val;
			break;

		case SUBSYS_TYPE_GCLKGEN:
			clock_mchp_subsys_gclkgen_config_t *gclkgen_config =
				(clock_mchp_subsys_gclkgen_config_t *)req_config;

			reg_val = gclk_regs->GCLK_GENCTRL[inst];
			reg_val &= ~(GCLK_GENCTRL_RUNSTDBY_Msk | GCLK_GENCTRL_SRC_Msk |
				     GCLK_GENCTRL_DIV_Msk);
			val |= ((gclkgen_config->run_in_standby_en != 0) ? GCLK_GENCTRL_RUNSTDBY(1)
									 : 0);

			val |= GCLK_GENCTRL_SRC(gclkgen_config->src);
			/* check range for div_factor, gclk1: 0 - 65535, others: 0 - 255 */
			if ((inst == CLOCK_MCHP_GCLKGEN_GEN1) ||
			    (gclkgen_config->div_factor <= 0xFF)) {
				val |= GCLK_GENCTRL_DIV(gclkgen_config->div_factor);
			}
			reg_val |= val;

			gclk_regs->GCLK_GENCTRL[inst] = reg_val;
			break;

		case SUBSYS_TYPE_GCLKPERIPH:
			clock_mchp_subsys_gclkperiph_config_t *gclkperiph_config =
				(clock_mchp_subsys_gclkperiph_config_t *)req_config;
			reg_val = gclk_regs->GCLK_PCHCTRL[subsys.bits.gclkperiph];
			reg_val &= ~GCLK_PCHCTRL_GEN_Msk;
			reg_val |= GCLK_PCHCTRL_GEN(gclkperiph_config->src);
			gclk_regs->GCLK_PCHCTRL[subsys.bits.gclkperiph] = reg_val;
			break;

		case SUBSYS_TYPE_MCLKCPU:
			clock_mchp_subsys_mclkcpu_config_t *mclkcpu_config =
				(clock_mchp_subsys_mclkcpu_config_t *)req_config;
			config->mclk_regs->MCLK_CPUDIV =
				MCLK_CPUDIV_DIV(mclkcpu_config->division_factor);
			break;

		default:
			ret_val = -ENOTSUP;
			break;
		}
	} while (0);

	return ret_val;
}
#endif /* CONFIG_CLOCK_CONTROL_MCHP_CONFIG_RUNTIME */

#if CONFIG_CLOCK_CONTROL_MCHP_CONFIG_BOOTUP
/******************************************************************************
 * @brief Internal initialization functions
 *****************************************************************************/
/**
 * @brief initialize XOSC from device tree node.
 */
void clock_xosc_init(const struct device *dev, clock_xosc_init_t *xosc_init)
{
	const clock_mchp_config_t *config = dev->config;
	oscctrl_registers_t *oscctrl_regs = config->oscctrl_regs;
	clock_mchp_data_t *data = dev->data;

	int inst;
	uint32_t val;
	uint32_t rdy_mask;

	inst = xosc_init->subsys.bits.inst;
	do {
		if ((data->src_on_status & (1 << (CLOCK_MCHP_FDPLL_SRC_MAX + 1 + inst))) != 0) {
			break;
		}

		data->xosc_crystal_freq[inst] = xosc_init->frequency;

		/* XOSCCTRL */
		val = 0;
		val |= ((xosc_init->clock_switch_en != 0) ? OSCCTRL_XOSCCTRL_SWBEN(1) : 0);
		val |= ((xosc_init->clock_failure_detection_en != 0) ? OSCCTRL_XOSCCTRL_CFDEN(1)
								     : 0);
		val |= ((xosc_init->automatic_loop_control_en != 0) ? OSCCTRL_XOSCCTRL_ENALC(1)
								    : 0);
		val |= ((xosc_init->low_buffer_gain_en != 0) ? OSCCTRL_XOSCCTRL_LOWBUFGAIN(1) : 0);
		val |= ((xosc_init->run_in_standby_en != 0) ? OSCCTRL_XOSCCTRL_RUNSTDBY(1) : 0);
		val |= ((xosc_init->xtal_en != 0) ? OSCCTRL_XOSCCTRL_XTALEN(1) : 0);
		val |= OSCCTRL_XOSCCTRL_STARTUP(xosc_init->startup_time);
		val |= OSCCTRL_XOSCCTRL_IMULT(4U) | OSCCTRL_XOSCCTRL_IPTAT(3U);
		val |= ((xosc_init->enable != 0) ? OSCCTRL_XOSCCTRL_ENABLE(1) : 0);

		/* Important: Initializing it with 1, along with clock enabled, can lead to
		 * indefinite wait for the clock to be on, if there is no peripheral request for the
		 * clock in the sequence of clock Initialization. If required, better to turn on the
		 * clock using API, instead of enabling both (on_demand_en & enable) during startup.
		 */
		val |= ((xosc_init->on_demand_en != 0) ? OSCCTRL_XOSCCTRL_ONDEMAND(1) : 0);

		oscctrl_regs->OSCCTRL_XOSCCTRL[inst] = val;
		if (xosc_init->enable != 0) {
			rdy_mask = (inst == INST_XOSC0) ? OSCCTRL_STATUS_XOSCRDY0_Msk
							: OSCCTRL_STATUS_XOSCRDY1_Msk;
			while ((oscctrl_regs->OSCCTRL_STATUS & rdy_mask) == 0) {
			}
			data->src_on_status |=
				(1 << (ON_BITPOS_XOSC0 + inst)) | (1 << (ON_BITPOS_XOSC0_ + inst));
		}
	} while (0);
}

/**
 * @brief initialize DFLL from device tree node.
 */
void clock_dfll_init(const struct device *dev, clock_dfll_init_t *dfll_init)
{
	const clock_mchp_config_t *config = dev->config;
	clock_mchp_data_t *data = dev->data;
	oscctrl_registers_t *oscctrl_regs = config->oscctrl_regs;
	gclk_registers_t *gclk_regs = config->gclk_regs;

	int gclkgen_index;
	uint8_t val8;
	uint32_t val32;

	do {
		if ((data->src_on_status & (1 << ON_BITPOS_DFLL)) != 0) {
			break;
		}

		gclkgen_index = dfll_init->src_gclk;
		if ((data->src_on_status & (1 << gclkgen_index)) == 0) {
			break;
		}

		/* To avoid changing dfll, while gclk0 is driven by it. Else will affect CPU */
		if (data->gclk0_src == CLOCK_MCHP_GCLK_SRC_DFLL) {
			break;
		}

		/* GCLK_PCHCTRL[0] is for DFLL48 input clock source */
		gclk_regs->GCLK_PCHCTRL[0] &= ~(GCLK_PCHCTRL_GEN_Msk);
		gclk_regs->GCLK_PCHCTRL[0] |=
			(GCLK_PCHCTRL_GEN(gclkgen_index) | GCLK_PCHCTRL_CHEN_Msk);

		if (dfll_init->closed_loop_en == true) {
			/* DFLLCTRLB */
			val8 = 0;
			val8 |= ((dfll_init->wait_lock_en != 0) ? OSCCTRL_DFLLCTRLB_WAITLOCK(1)
								: 0);
			val8 |= ((dfll_init->bypass_coarse_lock_en != 0)
					 ? OSCCTRL_DFLLCTRLB_BPLCKC(1)
					 : 0);
			val8 |= ((dfll_init->quick_lock_dis != 0) ? OSCCTRL_DFLLCTRLB_QLDIS(1) : 0);
			val8 |= ((dfll_init->chill_cycle_dis != 0) ? OSCCTRL_DFLLCTRLB_CCDIS(1)
								   : 0);
			val8 |= ((dfll_init->usb_recovery_en != 0) ? OSCCTRL_DFLLCTRLB_USBCRM(1)
								   : 0);
			val8 |= ((dfll_init->lose_lock_en != 0) ? OSCCTRL_DFLLCTRLB_LLAW(1) : 0);
			val8 |= ((dfll_init->stable_freq_en != 0) ? OSCCTRL_DFLLCTRLB_STABLE(1)
								  : 0);
			val8 |= OSCCTRL_DFLLCTRLB_MODE(1);

			oscctrl_regs->OSCCTRL_DFLLCTRLB = val8;
			while (oscctrl_regs->OSCCTRL_DFLLSYNC) {
			}

			/* DFLLMUL */
			val32 = 0;
			val32 |= OSCCTRL_DFLLMUL_CSTEP(dfll_init->coarse_max_step);
			val32 |= OSCCTRL_DFLLMUL_FSTEP(dfll_init->fine_max_step);
			val32 |= OSCCTRL_DFLLMUL_MUL(dfll_init->multiply_factor);

			oscctrl_regs->OSCCTRL_DFLLMUL = val32;
			while (oscctrl_regs->OSCCTRL_DFLLSYNC) {
			}
		}

		/* DFLLCTRLA */
		val8 = 0;
		val8 |= ((dfll_init->run_in_standby_en != 0) ? OSCCTRL_DFLLCTRLA_RUNSTDBY(1) : 0);
		val8 |= ((dfll_init->enable != 0) ? OSCCTRL_DFLLCTRLA_ENABLE(1) : 0);

		/* Important: Initializing it with 1, along with clock enabled, can lead to
		 * indefinite wait for the clock to be on, if there is no peripheral request for the
		 * clock in the sequence of clock Initialization. If required, better to turn on the
		 * clock using API, instead of enabling both (on_demand_en & enable) during startup.
		 */
		val8 |= ((dfll_init->on_demand_en != 0) ? OSCCTRL_DFLLCTRLA_ONDEMAND(1) : 0);

		oscctrl_regs->OSCCTRL_DFLLCTRLA = val8;
		while (oscctrl_regs->OSCCTRL_DFLLSYNC) {
		}
		if (dfll_init->enable != 0) {
			while ((oscctrl_regs->OSCCTRL_STATUS & OSCCTRL_STATUS_DFLLRDY_Msk) == 0) {
			}
			data->src_on_status |= (1 << ON_BITPOS_DFLL);
		}
	} while (0);
}

/**
 * @brief initialize FDPLL from device tree node.
 */
void clock_fdpll_init(const struct device *dev, clock_fdpll_init_t *fdpll_init)
{
	const clock_mchp_config_t *config = dev->config;
	clock_mchp_data_t *data = dev->data;
	oscctrl_registers_t *oscctrl_regs = config->oscctrl_regs;
	gclk_registers_t *gclk_regs = config->gclk_regs;

	uint8_t val8;
	int inst, src;
	uint32_t val32, mask;

	inst = fdpll_init->subsys.bits.inst;
	do {
		if (data->src_on_status & (1 << (ON_BITPOS_FDPLL0 + inst))) {
			break;
		}

		src = fdpll_init->src;
		if ((data->src_on_status & (1 << src)) == 0) {
			break;
		}

		/* program gclkph if source is gclk & enable */
		if (src <= CLOCK_MCHP_FDPLL_SRC_GCLK11) {
			gclk_regs->GCLK_PCHCTRL[inst + 1] |=
				(GCLK_PCHCTRL_GEN(src) | GCLK_PCHCTRL_CHEN_Msk);
			while ((gclk_regs->GCLK_PCHCTRL[inst + 1] & GCLK_PCHCTRL_CHEN_Msk) == 0) {
			}
		}

		/* DPLLCTRLB */
		val32 = 0;
		val32 |= OSCCTRL_DPLLCTRLB_DCOFILTER(fdpll_init->dco_filter_select);
		val32 |= OSCCTRL_DPLLCTRLB_REFCLK((src > CLOCK_MCHP_FDPLL_SRC_GCLK11)
							  ? (src - CLOCK_MCHP_FDPLL_SRC_GCLK11)
							  : 0);
		val32 |= OSCCTRL_DPLLCTRLB_FILTER(fdpll_init->pi_filter_type);
		val32 |= ((fdpll_init->dco_en != 0) ? OSCCTRL_DPLLCTRLB_DCOEN(1) : 0);
		val32 |= ((fdpll_init->lock_bypass_en != 0) ? OSCCTRL_DPLLCTRLB_LBYPASS(1) : 0);
		val32 |= ((fdpll_init->wakeup_fast_en != 0) ? OSCCTRL_DPLLCTRLB_WUF(1) : 0);
		val32 |= OSCCTRL_DPLLCTRLB_DIV(fdpll_init->xosc_clock_divider);

		oscctrl_regs->DPLL[inst].OSCCTRL_DPLLCTRLB = val32;

		/* DPLLRATIO */
		val32 = 0;
		val32 |= OSCCTRL_DPLLRATIO_LDR(fdpll_init->divider_ratio_int);
		val32 |= OSCCTRL_DPLLRATIO_LDRFRAC(fdpll_init->divider_ratio_frac);

		oscctrl_regs->DPLL[inst].OSCCTRL_DPLLRATIO = val32;
		while (oscctrl_regs->DPLL[inst].OSCCTRL_DPLLSYNCBUSY) {
		}

		/* DPLLCTRLA */
		val8 = 0;
		val8 |= ((fdpll_init->run_in_standby_en != 0) ? OSCCTRL_DPLLCTRLA_RUNSTDBY(1) : 0);
		val8 |= ((fdpll_init->enable != 0) ? OSCCTRL_DPLLCTRLA_ENABLE(1) : 0);

		/* Important: Initializing it with 1, along with clock enabled, can lead to
		 * indefinite wait for the clock to be on, if there is no peripheral request for the
		 * clock in the sequence of clock Initialization. If required, better to turn on the
		 * clock using API, instead of enabling both (on_demand_en & enable) during startup.
		 */
		val8 |= ((fdpll_init->on_demand_en != 0) ? OSCCTRL_DPLLCTRLA_ONDEMAND(1) : 0);

		oscctrl_regs->DPLL[inst].OSCCTRL_DPLLCTRLA = val8;
		while (oscctrl_regs->DPLL[inst].OSCCTRL_DPLLSYNCBUSY) {
		}
		if (fdpll_init->enable != 0) {
			mask = OSCCTRL_DPLLSTATUS_LOCK_Msk | OSCCTRL_DPLLSTATUS_CLKRDY_Msk;
			while ((oscctrl_regs->DPLL[inst].OSCCTRL_DPLLSTATUS & mask) != mask) {
			}
			data->src_on_status |= (1 << (ON_BITPOS_FDPLL0 + inst));
		}
	} while (0);
}

/**
 * @brief initialize rtc clock source from device tree node.
 */
void clock_rtc_init(const struct device *dev, uint8_t rtc_src)
{
	const clock_mchp_config_t *config = dev->config;

	config->osc32kctrl_regs->OSC32KCTRL_RTCCTRL = OSC32KCTRL_RTCCTRL_RTCSEL(rtc_src);
}

/**
 * @brief initialize osc32k clocks from device tree node.
 */
void clock_osc32k_init(const struct device *dev, clock_osc32k_init_t *osc32k_init)
{
	const clock_mchp_config_t *config = dev->config;
	clock_mchp_data_t *data = dev->data;
	osc32kctrl_registers_t *osc32kctrl_regs = config->osc32kctrl_regs;

	uint8_t val8;
	uint16_t val16;
	uint32_t val32;

	do {
		if ((data->src_on_status &
		     ((1 << ON_BITPOS_XOSC32K) | (1 << ON_BITPOS_OSCULP32K))) != 0) {
			break;
		}

		/* CFDCTRL */
		val8 = 0;
		val8 |= ((osc32k_init->cf_backup_divideby2_en != 0) ? OSC32KCTRL_CFDCTRL_CFDPRESC(1)
								    : 0);
		val8 |= ((osc32k_init->switch_back_en != 0) ? OSC32KCTRL_CFDCTRL_SWBACK(1) : 0);
		val8 |= ((osc32k_init->cfd_en != 0) ? OSC32KCTRL_CFDCTRL_CFDEN(1) : 0);

		osc32kctrl_regs->OSC32KCTRL_CFDCTRL = val8;

		/* OSCULP32K */
		val32 = 0;
		val32 |= ((osc32k_init->osculp32k_write_lock_en != 0)
				  ? OSC32KCTRL_OSCULP32K_WRTLOCK(1)
				  : 0);
		val32 |= ((osc32k_init->osculp32k_1khz_en != 0) ? OSC32KCTRL_OSCULP32K_EN1K(1) : 0);
		val32 |= ((osc32k_init->osculp32k_32khz_en != 0) ? OSC32KCTRL_OSCULP32K_EN32K(1)
								 : 0);

		osc32kctrl_regs->OSC32KCTRL_OSCULP32K = val32;
		if (osc32k_init->osculp32k_32khz_en != 0) {
			data->src_on_status |= (1 << ON_BITPOS_OSCULP32K);
		}

		/* XOSC32K */
		val16 = 0;
		if (osc32k_init->gain_mode == 0) { /* standard */
			val16 |= OSC32KCTRL_XOSC32K_CGM(OSC32KCTRL_XOSC32K_CGM_XT_Val);
		} else { /* highspeed */
			val16 |= OSC32KCTRL_XOSC32K_CGM(OSC32KCTRL_XOSC32K_CGM_HS_Val);
		}
		val16 |= ((osc32k_init->write_lock_en != 0) ? OSC32KCTRL_XOSC32K_WRTLOCK(1) : 0);
		val16 |= ((osc32k_init->run_in_standby_en != 0) ? OSC32KCTRL_XOSC32K_RUNSTDBY(1)
								: 0);
		val16 |= ((osc32k_init->osc32k_1khz_en != 0) ? OSC32KCTRL_XOSC32K_EN1K(1) : 0);
		val16 |= ((osc32k_init->osc32k_32khz_en != 0) ? OSC32KCTRL_XOSC32K_EN32K(1) : 0);
		val16 |= ((osc32k_init->xtal_en != 0) ? OSC32KCTRL_XOSC32K_XTALEN(1) : 0);
		val16 |= OSC32KCTRL_XOSC32K_STARTUP(osc32k_init->startup_time);
		val16 |= ((osc32k_init->enable != 0) ? OSC32KCTRL_XOSC32K_ENABLE(1) : 0);

		/* Important: Initializing it with 1, along with clock enabled, can lead to
		 * indefinite wait for the clock to be on, if there is no peripheral request for the
		 * clock in the sequence of clock Initialization. If required, better to turn on the
		 * clock using API, instead of enabling both (on_demand_en & enable) during startup.
		 */
		val16 |= ((osc32k_init->on_demand_en != 0) ? OSC32KCTRL_XOSC32K_ONDEMAND(1) : 0);

		osc32kctrl_regs->OSC32KCTRL_XOSC32K = val16;
		if (osc32k_init->osc32k_32khz_en != 0) {
			while ((osc32kctrl_regs->OSC32KCTRL_STATUS &
				OSC32KCTRL_STATUS_XOSC32KRDY_Msk) == 0) {
			}
			data->src_on_status |= (1 << ON_BITPOS_XOSC32K);
			data->src_on_status |= (1 << ON_BITPOS_XOSC32K_);
		}
	} while (0);
}

/**
 * @brief initialize gclk generator from device tree node.
 */
void clock_gclkgen_init(const struct device *dev, clock_gclkgen_init_t *gclkgen_init)
{
	const clock_mchp_config_t *config = dev->config;
	clock_mchp_data_t *data = dev->data;

	int inst;
	uint32_t val;

	inst = gclkgen_init->subsys.bits.inst;
	do {
		if ((data->src_on_status & (1 << inst)) != 0) {
			/* Already initialized */
			break;
		}
		if ((data->src_on_status & (1 << (ON_BITPOS_XOSC0 + gclkgen_init->src))) == 0) {
			/* Source is not on */
			break;
		}

		if (inst <= GCLK_IO_MAX) {
			data->gclkpin_freq[inst] = gclkgen_init->pin_src_freq;
		}

		/* GENCTRL */
		val = 0;
		if (gclkgen_init->div_select == 0) { /* div-factor */
			val |= GCLK_GENCTRL_DIVSEL(GCLK_GENCTRL_DIVSEL_DIV1_Val);
		} else { /* div-factor-power */
			val |= GCLK_GENCTRL_DIVSEL(GCLK_GENCTRL_DIVSEL_DIV2_Val);
		}
		val |= GCLK_GENCTRL_OOV(gclkgen_init->pin_output_off_val);
		val |= GCLK_GENCTRL_SRC(gclkgen_init->src);
		val |= ((gclkgen_init->run_in_standby_en != 0) ? GCLK_GENCTRL_RUNSTDBY(1) : 0);
		val |= ((gclkgen_init->pin_output_en != 0) ? GCLK_GENCTRL_OE(1) : 0);
		val |= ((gclkgen_init->duty_50_50_en != 0) ? GCLK_GENCTRL_IDC(1) : 0);
		/* check range for div_factor, gclk1: 0 - 65535, others: 0 - 255 */
		if ((inst == 1) || (gclkgen_init->div_factor <= 0xFF)) {
			val |= GCLK_GENCTRL_DIV(gclkgen_init->div_factor);
		}
		val |= ((gclkgen_init->enable != 0) ? GCLK_GENCTRL_GENEN(1) : 0);

		config->gclk_regs->GCLK_GENCTRL[inst] = val;
		while (config->gclk_regs->GCLK_SYNCBUSY != 0) {
		}

		/* To avoid changing dfll, while gclk0 is driven by it. Else will affect CPU */
		if (inst == CLOCK_MCHP_GCLKGEN_GEN0) {
			data->gclk0_src = gclkgen_init->src;
		}

		data->src_on_status |= (1 << inst);
		if (inst == CLOCK_MCHP_GCLKGEN_GEN0) {
			data->src_on_status |= (1 << ON_BITPOS_GCLKGEN1);
		}
	} while (0);
}

/**
 * @brief initialize peripheral gclk from device tree node.
 */
void clock_gclkperiph_init(const struct device *dev, uint32_t subsys_val, uint8_t pch_src,
			   uint8_t enable)
{
	const clock_mchp_config_t *config = dev->config;
	clock_mchp_subsys_t subsys;
	uint32_t val;

	subsys.val = subsys_val;

	/* PCHCTRL */
	val = 0;
	val |= ((enable != 0) ? GCLK_PCHCTRL_CHEN(1) : 0);
	val |= GCLK_PCHCTRL_GEN(pch_src);

	config->gclk_regs->GCLK_PCHCTRL[subsys.bits.gclkperiph] = val;
}

/**
 * @brief initialize cpu mclk from device tree node.
 */
void clock_mclkcpu_init(const struct device *dev, uint8_t cpu_div)
{
	const clock_mchp_config_t *config = dev->config;

	config->mclk_regs->MCLK_CPUDIV = MCLK_CPUDIV_DIV(cpu_div);
}

/**
 * @brief initialize peripheral mclk from device tree node.
 */
void clock_mclkperiph_init(const struct device *dev, uint32_t subsys_val, uint8_t enable)
{
	const clock_mchp_config_t *config = dev->config;
	clock_mchp_subsys_t subsys;

	uint32_t mask;
	__IO uint32_t *mask_reg;

	subsys.val = subsys_val;
	mask = 1 << subsys.bits.mclkmaskbit;
	mask_reg = get_mclkbus_mask_reg(config->mclk_regs, subsys.bits.mclkbus);

	if (mask_reg != NULL) {
		if (enable == true) {
			*mask_reg |= mask;

		} else {
			*mask_reg &= ~mask;
		}
	}
}

#define CLOCK_MCHP_ITERATE_XOSC(child)                                                             \
	xosc_init.subsys.val = DT_PROP(child, subsystem);                                          \
	xosc_init.frequency = DT_PROP(child, xosc_frequency);                                      \
	xosc_init.startup_time = DT_ENUM_IDX(child, xosc_startup_time);                            \
	xosc_init.clock_switch_en = DT_PROP(child, xosc_clock_switch_en);                          \
	xosc_init.clock_failure_detection_en = DT_PROP(child, xosc_clock_failure_detection_en);    \
	xosc_init.automatic_loop_control_en = DT_PROP(child, xosc_automatic_loop_control_en);      \
	xosc_init.low_buffer_gain_en = DT_PROP(child, xosc_low_buffer_gain_en);                    \
	xosc_init.on_demand_en = DT_PROP(child, xosc_on_demand_en);                                \
	xosc_init.run_in_standby_en = DT_PROP(child, xosc_run_in_standby_en);                      \
	xosc_init.xtal_en = DT_PROP(child, xosc_xtal_en);                                          \
	xosc_init.enable = DT_PROP(child, xosc_en);                                                \
	clock_xosc_init(dev, &xosc_init);

#define CLOCK_MCHP_PROCESS_DFLL(node)                                                              \
	dfll_init.on_demand_en = DT_PROP(node, dfll_on_demand_en);                                 \
	dfll_init.run_in_standby_en = DT_PROP(node, dfll_run_in_standby_en);                       \
	dfll_init.wait_lock_en = DT_PROP(node, dfll_wait_lock_en);                                 \
	dfll_init.bypass_coarse_lock_en = DT_PROP(node, dfll_bypass_coarse_lock_en);               \
	dfll_init.quick_lock_dis = DT_PROP(node, dfll_quick_lock_dis);                             \
	dfll_init.chill_cycle_dis = DT_PROP(node, dfll_chill_cycle_dis);                           \
	dfll_init.usb_recovery_en = DT_PROP(node, dfll_usb_recovery_en);                           \
	dfll_init.lose_lock_en = DT_PROP(node, dfll_lose_lock_en);                                 \
	dfll_init.stable_freq_en = DT_PROP(node, dfll_stable_freq_en);                             \
	dfll_init.closed_loop_en = DT_PROP(node, dfll_closed_loop_en);                             \
	dfll_init.coarse_max_step = DT_PROP(node, dfll_coarse_max_step);                           \
	dfll_init.fine_max_step = DT_PROP(node, dfll_fine_max_step);                               \
	dfll_init.multiply_factor = DT_PROP(node, dfll_multiply_factor);                           \
	dfll_init.src_gclk = DT_ENUM_IDX(node, dfll_src_gclk);                                     \
	dfll_init.enable = DT_PROP(node, dfll_en);                                                 \
	clock_dfll_init(dev, &dfll_init);

#define CLOCK_MCHP_ITERATE_FDPLL(child)                                                            \
	fdpll_init.subsys.val = DT_PROP(child, subsystem);                                         \
	fdpll_init.on_demand_en = DT_PROP(child, fdpll_on_demand_en);                              \
	fdpll_init.run_in_standby_en = DT_PROP(child, fdpll_run_in_standby_en);                    \
	fdpll_init.divider_ratio_int = DT_PROP(child, fdpll_divider_ratio_int);                    \
	fdpll_init.divider_ratio_frac = DT_PROP(child, fdpll_divider_ratio_frac);                  \
	fdpll_init.xosc_clock_divider = DT_PROP(child, fdpll_xosc_clock_divider);                  \
	fdpll_init.dco_en = DT_PROP(child, fdpll_dco_en);                                          \
	fdpll_init.dco_filter_select = DT_ENUM_IDX(child, fdpll_dco_filter_select);                \
	fdpll_init.lock_bypass_en = DT_PROP(child, fdpll_lock_bypass_en);                          \
	fdpll_init.src = DT_ENUM_IDX(child, fdpll_src);                                            \
	fdpll_init.wakeup_fast_en = DT_PROP(child, fdpll_wakeup_fast_en);                          \
	fdpll_init.pi_filter_type = DT_ENUM_IDX(child, fdpll_pi_filter_type);                      \
	fdpll_init.enable = DT_PROP(child, fdpll_en);                                              \
	clock_fdpll_init(dev, &fdpll_init);

#define CLOCK_MCHP_PROCESS_RTC(node) clock_rtc_init(dev, DT_PROP(node, rtc_src));

#define CLOCK_MCHP_PROCESS_OSC32K(node)                                                            \
	osc32k_init.gain_mode = DT_ENUM_IDX(node, osc32k_gain_mode);                               \
	osc32k_init.write_lock_en = DT_PROP(node, osc32k_write_lock_en);                           \
	osc32k_init.startup_time = DT_ENUM_IDX(node, osc32k_startup_time);                         \
	osc32k_init.on_demand_en = DT_PROP(node, osc32k_on_demand_en);                             \
	osc32k_init.run_in_standby_en = DT_PROP(node, osc32k_run_in_standby_en);                   \
	osc32k_init.osc32k_1khz_en = DT_PROP(node, osc32k_1khz_en);                                \
	osc32k_init.osc32k_32khz_en = DT_PROP(node, osc32k_32khz_en);                              \
	osc32k_init.xtal_en = DT_PROP(node, osc32k_xtal_en);                                       \
	osc32k_init.cf_backup_divideby2_en = DT_PROP(node, osc32k_cf_backup_divideby2_en);         \
	osc32k_init.switch_back_en = DT_PROP(node, osc32k_switch_back_en);                         \
	osc32k_init.cfd_en = DT_PROP(node, osc32k_cfd_en);                                         \
	osc32k_init.osculp32k_write_lock_en = DT_PROP(node, osculp32k_write_lock_en);              \
	osc32k_init.osculp32k_1khz_en = DT_PROP(node, osculp32k_1khz_en);                          \
	osc32k_init.osculp32k_32khz_en = DT_PROP(node, osculp32k_32khz_en);                        \
	osc32k_init.enable = DT_PROP(node, osc32k_en);                                             \
	clock_osc32k_init(dev, &osc32k_init);

#define CLOCK_MCHP_ITERATE_GCLKGEN(child)                                                          \
	gclkgen_init.subsys.val = DT_PROP(child, subsystem);                                       \
	gclkgen_init.div_factor = DT_PROP(child, gclkgen_div_factor);                              \
	gclkgen_init.run_in_standby_en = DT_PROP(child, gclkgen_run_in_standby_en);                \
	gclkgen_init.div_select = DT_ENUM_IDX(child, gclkgen_div_select);                          \
	gclkgen_init.pin_output_en = DT_PROP(child, gclkgen_pin_output_en);                        \
	gclkgen_init.pin_output_off_val = DT_ENUM_IDX(child, gclkgen_pin_output_off_val);          \
	gclkgen_init.duty_50_50_en = DT_PROP(child, gclkgen_duty_50_50_en);                        \
	gclkgen_init.src = DT_ENUM_IDX(child, gclkgen_src);                                        \
	gclkgen_init.enable = DT_PROP(child, gclkgen_en);                                          \
	gclkgen_init.pin_src_freq = DT_PROP(child, gclkgen_pin_src_freq);                          \
	clock_gclkgen_init(dev, &gclkgen_init);

#define CLOCK_MCHP_ITERATE_GCLKPERIPH(child)                                                       \
	clock_gclkperiph_init(dev, DT_PROP(child, subsystem), DT_ENUM_IDX(child, gclkperiph_src),  \
			      DT_PROP(child, gclkperiph_en));

#define CLOCK_MCHP_PROCESS_MCLKCPU(node) clock_mclkcpu_init(dev, DT_PROP(node, mclk_cpu_div));

#define CLOCK_MCHP_ITERATE_MCLKPERIPH(child)                                                       \
	clock_mclkperiph_init(dev, DT_PROP(child, subsystem), DT_PROP(child, mclk_en));

#endif /* CONFIG_CLOCK_CONTROL_MCHP_CONFIG_BOOTUP */

#if CONFIG_CLOCK_CONTROL_MCHP_ASYNC_ON
#define CLOCK_MCHP_IRQ_CONNECT_ENABLE(node, idx)                                                   \
	IRQ_CONNECT(DT_IRQ_BY_IDX(node, idx, irq), DT_IRQ_BY_IDX(node, idx, priority),             \
		    clock_mchp_isr, DEVICE_DT_GET(DT_NODELABEL(clock)), 0);                        \
	irq_enable(DT_IRQ_BY_IDX(node, idx, irq))
#endif /* CONFIG_CLOCK_CONTROL_MCHP_ASYNC_ON */

/**
 * @brief clock driver initialization function.
 */
static int clock_mchp_init(const struct device *dev)
{
#if CONFIG_CLOCK_CONTROL_MCHP_ASYNC_ON
	/* Enable the interrupt connection for the clock control subsystem. */
	CLOCK_MCHP_IRQ_CONNECT_ENABLE(DT_NODELABEL(clock), 0);
	CLOCK_MCHP_IRQ_CONNECT_ENABLE(DT_NODELABEL(clock), 1);
	CLOCK_MCHP_IRQ_CONNECT_ENABLE(DT_NODELABEL(clock), 2);
	CLOCK_MCHP_IRQ_CONNECT_ENABLE(DT_NODELABEL(clock), 3);
	CLOCK_MCHP_IRQ_CONNECT_ENABLE(DT_NODELABEL(clock), 4);
	CLOCK_MCHP_IRQ_CONNECT_ENABLE(DT_NODELABEL(clock), 5);
	CLOCK_MCHP_IRQ_CONNECT_ENABLE(DT_NODELABEL(clock), 6);
#endif /* CONFIG_CLOCK_CONTROL_MCHP_ASYNC_ON */

#if CONFIG_CLOCK_CONTROL_MCHP_CONFIG_BOOTUP
	const clock_mchp_config_t *config = dev->config;
	clock_mchp_data_t *data = dev->data;

	/* iteration-1 */
	{
		clock_xosc_init_t xosc_init = {0};

		DT_FOREACH_CHILD(DT_NODELABEL(xosc), CLOCK_MCHP_ITERATE_XOSC);
	}
	{
		clock_osc32k_init_t osc32k_init = {0};

		CLOCK_MCHP_PROCESS_OSC32K(DT_NODELABEL(osc32k));
	}

	config->gclk_regs->GCLK_CTRLA = GCLK_CTRLA_SWRST(1);
	while (config->gclk_regs->GCLK_SYNCBUSY != 0) {
	}

	/* To avoid changing dfll, while gclk0 is driven by it. Else will affect CPU */
	data->gclk0_src = CLOCK_MCHP_GCLK_SRC_DFLL;

	for (int i = 0; i < CLOCK_INIT_ITERATION_COUNT; i++) {
		{
			clock_gclkgen_init_t gclkgen_init = {0};

			DT_FOREACH_CHILD(DT_NODELABEL(gclkgen), CLOCK_MCHP_ITERATE_GCLKGEN);
		}
		{
			clock_dfll_init_t dfll_init = {0};

			CLOCK_MCHP_PROCESS_DFLL(DT_NODELABEL(dfll));
		}
		{
			clock_fdpll_init_t fdpll_init = {0};

			DT_FOREACH_CHILD(DT_NODELABEL(fdpll), CLOCK_MCHP_ITERATE_FDPLL);
		}
	}

	CLOCK_MCHP_PROCESS_RTC(DT_NODELABEL(rtcclock));
	DT_FOREACH_CHILD(DT_NODELABEL(gclkperiph), CLOCK_MCHP_ITERATE_GCLKPERIPH);
	DT_FOREACH_CHILD(DT_NODELABEL(mclkperiph), CLOCK_MCHP_ITERATE_MCLKPERIPH);

	CLOCK_MCHP_PROCESS_MCLKCPU(DT_NODELABEL(mclkcpu));
#endif /* CONFIG_CLOCK_CONTROL_MCHP_CONFIG_BOOTUP */

	/* Return CLOCK_SUCCESS indicating successful initialization. */
	return CLOCK_SUCCESS;
}

/******************************************************************************
 * @brief Zephyr driver instance creation
 *****************************************************************************/
static const struct clock_control_driver_api clock_mchp_driver_api = {
	.on = clock_mchp_on,
	.off = clock_mchp_off,
	.get_status = clock_mchp_get_status,
#if CONFIG_CLOCK_CONTROL_MCHP_ASYNC_ON
	.async_on = clock_mchp_async_on,
#endif /* CONFIG_CLOCK_CONTROL_MCHP_ASYNC_ON */
#if CONFIG_CLOCK_CONTROL_MCHP_GET_RATE
	.get_rate = clock_mchp_get_rate,
#if CONFIG_CLOCK_CONTROL_MCHP_SET_RATE
	.set_rate = clock_mchp_set_rate,
#endif /* CONFIG_CLOCK_CONTROL_MCHP_SET_RATE */
#endif /* CONFIG_CLOCK_CONTROL_MCHP_GET_RATE */
#if CONFIG_CLOCK_CONTROL_MCHP_CONFIG_RUNTIME
	.configure = clock_mchp_configure,
#endif /* CONFIG_CLOCK_CONTROL_MCHP_CONFIG_RUNTIME */
};

#define CLOCK_MCHP_CONFIG_DEFN()                                                                   \
	static const clock_mchp_config_t clock_mchp_config = {                                     \
		.on_timeout_ms = DT_PROP_OR(CLOCK_NODE, on_timeout_ms, 5),                         \
		.mclk_regs = (mclk_registers_t *)DT_REG_ADDR_BY_IDX(CLOCK_NODE, 0),                \
		.oscctrl_regs = (oscctrl_registers_t *)DT_REG_ADDR_BY_IDX(CLOCK_NODE, 1),          \
		.osc32kctrl_regs = (osc32kctrl_registers_t *)DT_REG_ADDR_BY_IDX(CLOCK_NODE, 2),    \
		.gclk_regs = (gclk_registers_t *)DT_REG_ADDR_BY_IDX(CLOCK_NODE, 3)}

#define CLOCK_MCHP_DATA_DEFN() static clock_mchp_data_t clock_mchp_data;

#define CLOCK_MCHP_DEVICE_INIT(n)                                                                  \
	CLOCK_MCHP_CONFIG_DEFN();                                                                  \
	CLOCK_MCHP_DATA_DEFN();                                                                    \
	DEVICE_DT_INST_DEFINE(n, clock_mchp_init, NULL, &clock_mchp_data, &clock_mchp_config,      \
			      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,                    \
			      &clock_mchp_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CLOCK_MCHP_DEVICE_INIT)
