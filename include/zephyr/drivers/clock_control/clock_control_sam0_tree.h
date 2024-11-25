/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_ATMEL_SAM0_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_ATMEL_SAM0_H_

#include "clock_control_sam0_sys.h"
#include "clock_control_sam0_index.h"

/** @brief Clock data structure for OSCCTRL Module.
 *
 * Structure which defines all available configurations for OSCCTRL module.
 */
struct clock_control_sam0_oscctrl_data {
	bool is_crystal;         // Crystal / External
	int32_t frequency;       // (8 – 48 MHz)
	bool is_advanced_config; // Enable Advanced config
	struct {
		bool is_automatic_loop_control;   // (Enable / Disable)
		bool is_low_buffer_gain;          // (Enable / Disable)
		bool is_clock_switch;             // XOSC Clock Switch (Enable / Disable)
		bool is_oscillator_on_demand;     // (Enable / Disable)
		uint32_t oscillator_startup_time; // 31us to 1000000us
		bool is_run_in_standby;           // (Enable / Disable)
		bool is_clock_failure_detection;  // (Enable / Disable)
		uint32_t safe_clock_frequency;    // 48MHz - 0.3125MHz
	} advanced_config;
};

struct clock_control_sam0_oscctrl_dfll_data {
	bool is_open_loop;       // Open Loop / Closed Loop
	int8_t gclk_source_num;  // Closed Loop (GCLK 0 -11)
	int16_t mul_factor;      // DFLL Multiply Factor (1 – 65535)
	bool is_advanced_config; // Enable Advanced config
	struct {
		bool is_oscillator_on_demand; // (Enable / Disable)
		bool is_run_in_standby;       // (Enable / Disable)
		bool is_usb_recovery;         // USB Clock Recovery Mode (Enable / Disable)
		bool is_wait_for_lock;        // Wait for DFLL lock (Enable / Disable)
		bool is_bypass_coarse_lock;   // Bypass Coarse Lock (Enable / Disable)
		bool is_quick_lock;           // Quick lock (Enable / Disable)
		bool is_chill_cycle;          // Chill Cycle (Enable / Disable)
		bool is_lose_lock_after_wake; // Lose Lock After Wake (Enable / Disable)
		bool is_stable_freq;          // Stable DFLL Frequency (Enable / Disable)
		uint8_t coarse_max_step;      // Coarse Maximum Step (0 – 31)
		uint8_t fine_max_step;        // Fine Maximum Step (0 – 255)
	} advanced_config;
};

struct clock_control_sam0_oscctrl_fdpll_data {
	uint8_t source;                // xosc32k, Gclk0-11, xosc0, xosc1
	uint16_t xosc_div;             // (2 – 4096)
	uint16_t loop_divider_integer; // Loop Divider Ratio Integer Part (1-4096)
	uint8_t loop_divider_frac;     // Loop Divider Ratio Fractional Part (1-32)
	bool is_advanced_config;       // Enable Advanced config
	struct {
		bool is_oscillator_on_demand; // (Enable / Disable)
		bool is_run_in_standby;       // (Enable / Disable)
		bool is_bypass_fdpll_lock;    // Bypass FDPLL lock (Enable / Disable)
		bool is_dco_filter;           // DCO Filter (Enable / Disable)
		uint8_t sigma_delta_filter;   // Capacitor 0.5pF – 4pF
		bool is_fast_output_mode;     // Fast Output Mode (Enable / Disable)
		uint8_t prop_integral_filter; // Integral Filter Selection BW 23.2kHz – 185kHz
	} advanced_config;
};

/** @brief Clock data structure for OSC32KCTRL Module.
 *
 * Structure which defines all available configurations for OSC32KCTRL module.
 */
struct clock_control_sam0_osc32kctrl_data {
	bool is_crystal;         // Source is Crystal (32KHz output is enabled)
	bool is_xosc1k;          // XOSC1K (Enable / Disable)
	bool is_external;        // External (32kHz and 1kHz output is disabled)
	bool is_advanced_config; // Enable Advanced config
	struct {
		bool is_oscillator_on_demand;          // (Enable / Disable)
		bool is_run_in_standby;                // (Enable / Disable)
		uint8_t control_gain_mode;             // Low, Standard, High
		bool is_clock_failure_detection;       // Clock Failure Detection (Enable / Disable)
		bool is_clock_failure_backup_div_by_2; // Div by 2 (Enable / Disable)
		bool is_clock_switch_back;             // Clock Switch Back (Enable / Disable)
		uint32_t start_up_time;                // Start-Up Time 62592us - 8000092us
	} advanced_config;
};

struct clock_control_sam0_osc32kctrl_rtc_data {
	uint8_t rtc_source; // OSCULP1K, OSCULP32K, XOSC1K, XOSC32K
};

/** @brief Clock data structure for GCLK Module.
 *
 * Structure which defines all available configurations for GCLK module.
 */
struct clock_control_sam0_gclk_data {
	uint8_t source;          // XOSC0 – GCLK1
	uint16_t divider;        // (1-512)
	bool is_advanced_config; // Enable Advanced config
	struct {
		bool is_run_in_standby;     // (Enable / Disable)
		bool is_output_on_io;       // GCLK signal on IO pin (Enable / Disable)
		bool is_output_off_low;     // Output OFF value logic 0 / 1
		bool is_divide_high_enable; // Divide by 2^(Divider +1) (Enable / Disable)
		bool is_improve_duty;       // Improve Duty Cycle (Enable / Disable)
	} advanced_config;
};

/** @brief Clock data structure for MCLK Module.
 *
 * Structure which defines all available configurations for MCLK module.
 */
struct clock_control_sam0_mclk_data {
	uint8_t cpu_div;           // 1 – 128
	uint8_t backup_clk_div;    // 1 - 128
	uint8_t low_power_clk_div; // 1 – 128
	uint8_t hs_div;            // 1
};

/** @brief Clock data structure for Peripheral Clocks.
 *
 * Structure which defines all available configurations for Peripheral clocks.
 */
struct clock_control_sam0_periph_data {
	uint8_t source; // GCLK0 – GCLK11
};

/** @brief Clock data structure for AHB Clock.
 *
 * Structure which defines all available configurations for AHB Clock.
 */
/*struct clock_control_sam0_ahb_data ahb_data {
	// TODO // No need to configure
};*/

/** @brief Clock data structure for APB Clock.
 *
 * Structure which defines all available configurations for APB Clock.
 */
/*struct clock_control_sam0_apb_data apb_data {
	// TODO
	// on, off, get_rate, set_rate
};*/

/** @brief Clock data structure in common to all Clocks.
 *
 * Structure which can be used with configure_fn in clock_control API.
 * Keep unused subsys pointers as NULL
 * subsys is identified by index
 */
struct clock_control_sam0_data {
	// XOSC0, XOSC1 (2 sub_sys)
	struct clock_control_sam0_oscctrl_data *xosc_data[2];

	// DFLL (1 sub_sys)
	struct clock_control_sam0_oscctrl_dfll_data *dfll_data;

	// FDPLL0, FDPLL1 (2 sub_sys)
	struct clock_control_sam0_oscctrl_fdpll_data *fdpll_data[2];

	// XOSC32K & XOSC1K (1 sub_sys)
	struct clock_control_sam0_osc32kctrl_data *xosc32k_data;

	// RTC (1 sub_sys)
	struct clock_control_sam0_osc32kctrl_data *rtc_data;

	// GCLK0 - 11 (12 sub_sys)
	struct clock_control_sam0_gclk_data *gclk_data[12];

	// All Periph Clocks (48 sub_sys)
	struct clock_control_sam0_periph_data *periph_data[48];

	// CPU_DIV, HS_DIV,LS_DIV, BCKUP_DIV (4 sub_sys)
	struct clock_control_sam0_mclk_data *mclk_data[4];

	// only on, off, get_rate, set_rate
	// struct clock_control_sam0_ahb_data ahb_data;

	// only on, off, get_rate, set_rate
	// struct clock_control_sam0_apb_data apb_data;
};

struct clock_control_sam0_data_element {
	// Identify clock sub system
	clock_control_sam0_sub_sys sub_sys_id;

	// contains the specified sub_sys data
	void* sub_sys_data;
};

struct clock_control_sam0_data_config {
	// Number of clock sub system to configure
	uint8_t element_count;

	// contains the specified sub_sys data table
	struct clock_control_sam0_data_element* sub_sys_table;
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_ATMEL_SAM0_H_ */
