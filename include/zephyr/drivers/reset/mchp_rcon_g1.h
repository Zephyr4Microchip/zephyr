/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file mchp_reset_g1.h
 * @brief Microchip RCON G1 reset controller header
 *
 * This header includes the Microchip RCON G1 macro definitions.
 */

#ifndef INCLUDE_ZEPHYR_DRIVERS_RESET_MCHP_RCON_G1_H_
#define INCLUDE_ZEPHYR_DRIVERS_RESET_MCHP_RCON_G1_H_

/**
 * @enum rcon_g1_rcause_t
 * @brief Reset cause flags for Microchip RCON G1.
 *
 * This enumeration defines the possible reset causes as indicated by the
 * RCON_RCAUSE register in the Microchip RCON G1 reset controller.
 */
typedef enum {
	RCON_G1_RCAUSE_POR = 0,       /* Power-on Reset */
	RCON_G1_RCAUSE_BOR = 1,       /* Brown-out Reset */
	RCON_G1_RCAUSE_IDLE = 2,      /* Wake from Idle */
	RCON_G1_RCAUSE_SLEEP = 3,     /* Wake from Sleep */
	RCON_G1_RCAUSE_WDTO = 4,      /* Watchdog Timer Time-Out */
	RCON_G1_RCAUSE_DMTO = 5,      /* Deadman Timer Time-Out */
	RCON_G1_RCAUSE_SWR = 6,       /* Software Reset */
	RCON_G1_RCAUSE_EXTR = 7,      /* External Reset */
	RCON_G1_RCAUSE_CMR = 9,       /* Configuration Mismatch */
	RCON_G1_RCAUSE_DPSLP = 10,    /* DPSLP Deep Sleep Mode */
	RCON_G1_RCAUSE_NVMEOL = 24,   /* NVM End of Life */
	RCON_G1_RCAUSE_NVMLTA = 25,   /* NVM Life Time Alert */
	RCON_G1_RCAUSE_BCFGFAIL = 26, /* BCFG Failure */
	RCON_G1_RCAUSE_BCFGERR = 27,  /* BCFG Error */
	RCON_G1_RCAUSE_PORCORE = 30,  /* Core Voltage POR */
	RCON_G1_RCAUSE_PORIO = 31     /* IO Voltage POR */
} rcon_g1_rcause_t;
#endif /* INCLUDE_ZEPHYR_DRIVERS_RESET_MCHP_RCON_G1_H_ */
