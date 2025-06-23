/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file mchp_reset_u2239.h
 * @brief Microchip RSTC U2239 reset controller header
 *
 * This header includes the  Microchip RSTC u2239 macro definitions.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_RESET_MCHP_RSTC_U2239_H_
#define ZEPHYR_INCLUDE_DRIVERS_RESET_MCHP_RSTC_U2239_H_

/**
 * @enum rstc_u2239_rcause_t
 * @brief Reset cause flags for Microchip RSTC U2239.
 *
 * This enumeration defines the possible reset causes as indicated by the
 * RSTC_RCAUSE register in the Microchip RSTC U2239 reset controller.
 */
typedef enum {
	RSTC_U2239_RCAUSE_POR = 0,   /* Power-on Reset */
	RSTC_U2239_RCAUSE_BOD12 = 1, /* Brown-Out 1.2V Detector Reset */
	RSTC_U2239_RCAUSE_BOD33 = 2, /* Brown-Out 3.3V Detector Reset */
	RSTC_U2239_RCAUSE_NVM = 3,   /* NVM Reset */
	RSTC_U2239_RCAUSE_EXT = 4,   /* External Reset */
	RSTC_U2239_RCAUSE_WDT = 5,   /* Watchdog Reset */
	RSTC_U2239_RCAUSE_SYST = 6,  /* System Reset Request */
	RSTC_U2239_RCAUSE_BACKUP = 7 /* Backup Reset */
} rstc_u2239_rcause_t;
#endif /* ZEPHYR_INCLUDE_DRIVERS_RESET_MCHP_RSTC_U2239_H_ */
