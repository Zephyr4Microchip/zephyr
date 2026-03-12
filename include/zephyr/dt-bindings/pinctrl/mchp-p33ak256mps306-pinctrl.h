/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_MCHP_P33AK256MPS306_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_MCHP_P33AK256MPS306_PINCTRL_H_

#include <zephyr/dt-bindings/dt-util.h>

/* Masks & Shifts */
#define DSPIC33_PORT_MASK  0x3U /* 2 bits for 0-3 */
#define DSPIC33_PORT_SHIFT 30

#define DSPIC33_PIN_MASK  0x3FU /* 4 bits for 0-11 */
#define DSPIC33_PIN_SHIFT 24

#define DSPIC33_FUNC_MASK  0xFFFFFFU /* 24 bits for 0-511 */
#define DSPIC33_FUNC_SHIFT 0

/* Encode: pack port, pin, function */
#define DSPIC33_PINMUX(port, pin, func)                                                            \
	((((port) & DSPIC33_PORT_MASK) << DSPIC33_PORT_SHIFT) |                                    \
	 (((pin) & DSPIC33_PIN_MASK) << DSPIC33_PIN_SHIFT) |                                       \
	 (((func) & DSPIC33_FUNC_MASK) << DSPIC33_FUNC_SHIFT))

/* Decode: extract port, pin, function */
#define DSPIC33_PINMUX_PORT(pinmux) (((pinmux) >> DSPIC33_PORT_SHIFT) & DSPIC33_PORT_MASK)
#define DSPIC33_PINMUX_PIN(pinmux)  (((pinmux) >> DSPIC33_PIN_SHIFT) & DSPIC33_PIN_MASK)
#define DSPIC33_PINMUX_FUNC(pinmux) (((pinmux) >> DSPIC33_FUNC_SHIFT) & DSPIC33_FUNC_MASK)

#define OFFSET_RPOR  0x3150
#define OFFSET_RPIN  0x30D4
#define OFFSET_LATCH 0x0004
#define OFFSET_TRIS  0x0008
#define OFFSET_ANSEL 0x3440

/* Port definitions */
#define PORT_A 0
#define PORT_B 1
#define PORT_C 2
#define PORT_D 3


/* Input Function Macros (for RPINRx register configuration) */
#define INT1     0x32D5
#define INT2     0x32D6
#define INT3     0x32D7
#define INT4     0x32D8
#define T1CK     0x32D9
#define T2CK     0x32DA
#define T3CK     0x32DB
#define TCKI1    0x32DC
#define ICM1     0x32DD
#define TCKI2    0x32DE
#define ICM2     0x32DF
#define TCKI3    0x32E0
#define ICM3     0x32E1
#define TCKI4    0x32E2
#define ICM4     0x32E3
#define TCKI5    0x32E4
#define ICM5     0x32E5
#define OCFAR    0x32F0
#define OCFBR    0x32F1
#define OCFCR    0x32F2
#define OCFDR    0x32F3
#define PCI8     0x32F4
#define PCI9     0x32F5
#define PCI10    0x32F6
#define PCI11    0x32F7
#define QEA1     0x32F8
#define QEB1     0x32F9
#define INDX1    0x32FA
#define HOME1    0x32FB
#define U1RX     0x3308
#define U1DSR    0x3309
#define U2RX     0x330A
#define U2DSR    0x330B
#define U3RX     0x330C
#define U3DSR    0x330D
#define SDI1     0x330E
#define SDK1     0x330F
#define SS1      0x3310
#define SDI2     0x3311
#define SDK2     0x3312
#define SS2      0x3313
#define SDI3     0x3314
#define SDK3     0x3315
#define SS3      0x3316
#define SDI4     0x3317
#define SDK4     0x3318
#define SS4      0x3319
#define CAN1RX   0x331A
#define SENT1    0x331C
#define SENT2    0x331D
#define REFI1    0x331E
#define REFI2    0x331F
#define PCI12    0x3320
#define PCI13    0x3321
#define PCI14    0x3322
#define PCI15    0x3323
#define PCI16    0x3324
#define PCI17    0x3325
#define PCI18    0x3326
#define CLCINA   0x3327
#define CLCINB   0x3328
#define CLCINC   0x3329
#define CLCIND   0x332A
#define CLCINE   0x332B
#define CLCINF   0x332C
#define CLCING   0x332D
#define CLCINH   0x332E
#define CLCINI   0x332F
#define CLCINJ   0x3330
#define ADTRG    0x3331
#define U1CTS    0x3332
#define U2CTS    0x3333
#define U3CTS    0x3334
#define BISS1SL  0x3335
#define BISS1GS  0x3336
#define IOM0     0x3337
#define IOM1     0x3338
#define IOM2     0x3339
#define IOM3     0x333A
#define PCI19    0x333F
#define PCI20    0x3340
#define PCI21    0x3341
#define PCI22    0x3342
#define U4RX     0x3344
#define U4DSR    0x3345

/* Output Function Macros (for RPnR register configuration) */
#define DEFAULT_PORT   0x00
#define PWM1H          0x01
#define PWM1L          0x02
#define PWM2H          0x03
#define PWM2L          0x04
#define PWM3H          0x05
#define PWM3L          0x06
#define PWM4H          0x07
#define PWM4L          0x08
#define CAN1TX         0x09
#define U1TX           0x0A
#define U1RTS          0x0B
#define U2TX           0x0C
#define U2RTS          0x0D
#define U3TX           0x0E
#define U3RTS          0x0F
#define U4TX           0x10
#define U4RTS          0x11
#define SDO1           0x12
#define SCK1           0x13
#define SS1            0x14
#define SDO2           0x15
#define SCK2           0x16
#define SS2            0x17
#define SDO3           0x18
#define SCK3           0x19
#define SS3            0x1A
#define REFO1          0x1B
#define REFO2          0x1C
#define OCM1           0x1D
#define OCM2           0x1E
#define OCM3           0x1F
#define OCM4           0x20
#define MCCP9A         0x21
#define MCCP9B         0x22
#define MCCP9C         0x23
#define MCCP9D         0x24
#define MCCP9E         0x25
#define MCCP9F         0x26
#define CMP1           0x27
#define CMP2           0x28
#define CMP3           0x29
#define CMP4           0x2A
#define CMP5           0x2B
#define PEVTA          0x2C
#define PEVTB          0x2D
#define PEVTC          0x2E
#define PEVTD          0x2F
#define PWME           0x30
#define PWMF           0x31
#define QEICMP1        0x32
#define CLC1OUT        0x33
#define CLC2OUT        0x34
#define CLC3OUT        0x35
#define CLC4OUT        0x36
#define PTGTRG24       0x37
#define PTGTRG25       0x38
#define SENT1OUT       0x39
#define SENT2OUT       0x3A
#define BISSMO1        0x3B
#define BISSMA1        0x3C
#define U1DTRn         0x3D
#define U2DTRn         0x3E
#define U3DTRn         0x3F
#define U4DTRn         0x40
#define RDCEXC         0x41
#define RDCEXCI        0x42
#define PTGTRG18       0x43
#define PTGTRG19       0x44
#define PTGTRG20       0x45


#endif
