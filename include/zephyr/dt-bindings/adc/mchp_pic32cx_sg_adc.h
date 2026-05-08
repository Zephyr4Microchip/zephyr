/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file mchp_pic32cx_sg.h
 * @brief ADC input selection definitions for PIC32CX_SG devices.
 *
 */

#ifndef INCLUDE_ZEPHYR_DT_BINDINGS_ADC_PIC32CX_SG_ADC_H_
#define INCLUDE_ZEPHYR_DT_BINDINGS_ADC_PIC32CX_SG_ADC_H_

/* External analog inputs */
#define MCHP_ADC_AIN0  0x00
#define MCHP_ADC_AIN1  0x01
#define MCHP_ADC_AIN2  0x02
#define MCHP_ADC_AIN3  0x03
#define MCHP_ADC_AIN4  0x04
#define MCHP_ADC_AIN5  0x05
#define MCHP_ADC_AIN6  0x06
#define MCHP_ADC_AIN7  0x07
#define MCHP_ADC_AIN8  0x08
#define MCHP_ADC_AIN9  0x09
#define MCHP_ADC_AIN10 0x0A
#define MCHP_ADC_AIN11 0x0B
#define MCHP_ADC_AIN12 0x0C
#define MCHP_ADC_AIN13 0x0D
#define MCHP_ADC_AIN14 0x0E
#define MCHP_ADC_AIN15 0x0F

/* 0x10 – 0x17: Reserved */

/* Internal ADC sources */
#define MCHP_ADC_SCALEDCOREVCC 0x18 /* 1/4 scaled core supply */
#define MCHP_ADC_SCALEDVBAT    0x19 /* 1/4 scaled VBAT */
#define MCHP_ADC_SCALEDIOVCC   0x1A /* 1/4 scaled I/O supply */
#define MCHP_ADC_BANDGAP       0x1B /* Bandgap voltage */
#define MCHP_ADC_PTAT          0x1C /* PTAT temperature sensor */
#define MCHP_ADC_CTAT          0x1D /* CTAT temperature sensor */
#define MCHP_ADC_DAC0          0x1E /* DAC0 output */

/* ADC positive input (MUXPOS) valid values in bits */
#define MCHP_ADC_MUXPOS_VALID_MASK (BIT(0x1E) | GENMASK(0x1B, 0x18) | GENMASK(0x0F, 0x00))

/* ADC negative input (MUXNEG) valid values in bits*/
#define MCHP_ADC_MUXNEG_VALID_MASK (BIT(0x18) | GENMASK(0x07, 0x00))

/* ADC reference selection */
#define MCHP_ADC_REF_INTREF  0x00 /* Internal bandgap reference (SUPC.VREF.SEL) */
#define MCHP_ADC_REF_INTVCC0 0x02 /* 1/2 VDDANA (VDDANA > 2.0 V) */
#define MCHP_ADC_REF_INTVCC1 0x03 /* VDDANA */
#define MCHP_ADC_REF_AREFA   0x04 /* External reference A */
#define MCHP_ADC_REF_AREFB   0x05 /* External reference B */
#define MCHP_ADC_REF_AREFC   0x06 /* External reference C (ADC1 only) */

#endif /* INCLUDE_ZEPHYR_DT_BINDINGS_ADC_PIC32CX_SG_ADC_H_ */
