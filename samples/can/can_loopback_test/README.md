# CAN Loopback Test — PIC32WM BZ6204 Curiosity

Validates the CAN1 and CAN0 controllers on the PIC32WM BZ6204 Curiosity board
using the Bosch M_CAN v3.2.3 IP core through the Zephyr CAN bus API.

---

## Overview

The PIC32CX2051BZ62132 SoC provides two CAN controllers:

| Controller | Base Address | IRQ | Default Board Pins
|------------|-------------|-----|-------------------
| **CAN1**   | 0x46002400  | 52  | PA3 (TX), PA2 (RX)  
| **CAN0**   | 0x46002000  | 51  | PA8 (TX), PA9 (RX)    

By default this sample targets **CAN1**, which is fully enabled in the board
device tree (`pic32wm_bz6204_curiosity.dts`). To test **CAN0**, a device tree
overlay is required. Both procedures are described below.

---

## Test Modes

### Mode 1 — Internal Loopback (`prj.conf`, default)

The M_CAN controller loops TX back to RX entirely in silicon
(`CCCR.TEST=1, TEST.LBCK=1`). No wire, transceiver, or external hardware is
required.

Runs six test cases:

| Test | Description |
|------|-------------|
| TC1 | Standard frame (11-bit ID), 8-byte payload |
| TC2 | Extended frame (29-bit ID), 8-byte payload |
| TC3 | Standard frame, 1-byte payload |
| TC4 | Zero-length data frame (DLC = 0) |
| TC5 | Five sequential frames (stress test) |
| TC6 | Hardware filter selectivity (accepted vs. rejected IDs) |

### Mode 2 — Internal Loopback + GPIO Wire Presence (`prj_wire.conf`)

Adds **TC0**, which temporarily reconfigures the CAN TX pin as a GPIO output
and reads it back on the CAN RX pin. This confirms a jumper wire is physically
connected between the two pins before CAN tests run.

Additional test case:

| Test | Description | Requirement |
|------|-------------|-------------|
| TC0 | GPIO wire presence — drives TX pin HIGH/LOW, reads RX pin | Jumper wire: TX → RX |

---

## Hardware Setup

### CAN1 (default, no extra wiring needed for internal loopback)

No physical connections are required for **Mode 1**.

For **Mode 2** wire presence test:

```
PA3 (CAN1 TX) ──── jumper wire ──── PA2 (CAN1 RX)
```

### CAN0 (overlay required — see Section below)

No physical connections are required for **Mode 1** (internal loopback).

For **Mode 2** wire presence test with CAN0:

```
PA8 (CAN0 TX) ──── jumper wire ──── PA9 (CAN0 RX)
```

---

## Building and Testing CAN1 (Default)

### Step 1 — Activate the build environment

```cmd
cd C:\Zworkbench
activate-env.cmd
```

### Step 2 — Build

**Internal loopback (no wiring needed):**

```cmd
west build -b pic32wm_bz6204_curiosity -p always samples\can\can_loopback_test
```

**With wire presence test (jumper required PA3 → PA2):**

```cmd
west build -b pic32wm_bz6204_curiosity -p always samples\can\can_loopback_test ^
  -DCONF_FILE=prj_wire.conf
```

### Step 3 — Flash

```cmd
west flash
```

### Step 4 — Observe output

Open TeraTerm (or any terminal) on the PKOB4 virtual COM port at **115200 baud, 8N1**.

Expected output (all pass):

```
==============================================
 PIC32 BZ6 CAN Internal Loopback Test
 Board  : pic32wm_bz6204_curiosity
 Device : can@46002400 (M_CAN v3.2.3)
==============================================
Mode    : INTERNAL LOOPBACK (TEST=1, LBCK=1)
----------------------------------------------
Running 6 tests...
  [PASS] TC1: Standard frame 8-byte
  [PASS] TC2: Extended frame 8-byte
  [PASS] TC3: Standard frame 1-byte
  [PASS] TC4: Zero-DLC frame
  [PASS] TC5: Sequential frames
  [PASS] TC6: Filter selectivity
----------------------------------------------
Results : 6/6 tests passed
OVERALL : *** ALL TESTS PASSED ***
==============================================
```

**LED indicators:**

| LED   | Color | Meaning                  |
|-------|-------|--------------------------|
| LED1  | Green | All tests passed         |
| LED0  | Red   | One or more tests failed |

---

## Testing CAN0 Using a Device Tree Overlay

CAN0 is defined in the SoC device tree (`pic32cxxbz6x.dtsi`) but is
**disabled by default**. 

To enable and test CAN0, create a board overlay file that:
1. Enables the CAN0 node.
2. Assigns pinctrl for PA8 (TX) and PA9 (RX).
3. Redirects `zephyr,canbus` to `&can0`.

### Step 1 — Create the overlay file

Create a file named `pic32wm_bz6204_curiosity.overlay` in the sample directory
(`samples/can/can_loopback_test/`):

```dts
/*
 * Board overlay to enable CAN0 on the PIC32WM BZ6204 Curiosity board.
 *
 * CAN0 TX: PA8  (PA8_CAN0_TX_OUT — direct/gpio mode, not PPS)
 * CAN0 RX: PA9  (PA9_CAN0RX_IN  — direct/gpio mode, not PPS)
 *
 * Note: CAN0 uses dedicated (non-PPS) pin routing on the BZ62132
 * package. The pinmux definitions use func=gpio because the CAN0 signal
 * path is hardware-fixed and does not go through the PPS matrix.
 */

#include <dt-bindings/pic32c/pic32cx_bz/bz62/pic32cx2051bz62132-pinctrl.h>

/* Add CAN0 pinctrl group to the pinctrl node */
&pinctrl {
    can0_default: can0_default {
        group1 {
            pinmux = <PA8_CAN0_TX_OUT>,
                     <PA9_CAN0RX_IN>;
        };
    };
};

/* Enable CAN0 and assign pinctrl */
&can0 {
    status = "okay";
    pinctrl-0 = <&can0_default>;
    pinctrl-names = "default";
    bosch,mram-cfg = <0x0 8 8 64 0 0 4 4>;
};

/* Redirect the system CAN bus and alias to CAN0 */
/ {
    chosen {
        zephyr,canbus = &can0;
    };

    aliases {
        can0 = &can0;
    };
};
```

> **Note:** The sample application selects the CAN device using
> `DT_CHOSEN(zephyr_canbus)`. Changing `zephyr,canbus` to `&can0` in the
> overlay is all that is needed to switch the test target — no source code
> changes are required.

### Step 2 — Build with the overlay

Zephyr automatically discovers an overlay file named after the board in the
application directory. No extra CMake flags are needed:

```cmd
west build -b pic32wm_bz6204_curiosity -p always samples\can\can_loopback_test
```

To confirm the overlay was applied, check the build log for:

```
-- Found BOARD.overlay: .../can_loopback_test/pic32wm_bz6204_curiosity.overlay
```

### Step 3 — Flash and observe

Follow the same flash procedure as CAN1 (Steps 3–4 above).

Expected output with CAN0:

```
==============================================
 PIC32 BZ6 CAN Internal Loopback Test
 Board  : pic32wm_bz6204_curiosity
 Device : can@46002000 (M_CAN v3.2.3)
==============================================
```

The device name now shows `can@46002000` (CAN0 base address) instead of
`can@46002400` (CAN1). All six test cases run identically.

### Step 4 — Optional: CAN0 wire presence test

Connect a jumper wire between PA8 and PA9 on the board, then build with the
wire test configuration:

```cmd
west build -b pic32wm_bz6204_curiosity -p always samples\can\can_loopback_test ^
  -DCONF_FILE=prj_wire.conf
```

TC0 will verify the physical wire between PA8 (CAN0 TX) and PA9 (CAN0 RX).

---

## File Reference

```
samples/can/can_loopback_test/
├── CMakeLists.txt                          Application build definition
├── Kconfig                                 Application Kconfig (CAN_WIRE_TEST option)
├── prj.conf                                Default config — internal loopback, no wire
├── prj_wire.conf                           Wire test config — loopback + GPIO TC0
├── pic32wm_bz6204_curiosity.overlay        CAN0 overlay (create this to test CAN0)
└── src/
    └── main.c                              Test application source
```

---

## Troubleshooting

| Symptom | Likely Cause | Action |
|---------|-------------|--------|
| `CAN device not ready` | PMD3 module-disable bit not cleared | Check `soc_early_init_hook` clears CFG_PMD3 CAN bits |
| `can_start failed` | GCLKPERIPH clock not enabled | Verify `gclkperiph_can0`/`gclkperiph_can1` in DTS clock node |
| TC0 FAIL: `PA9 read 0 (expected 1)` | Jumper wire missing or wrong pins | Connect wire between TX and RX pins |
| All TCs FAIL after overlay | `zephyr,canbus` not updated | Confirm overlay sets `zephyr,canbus = &can0` |
| Build error: `PA8_CAN0_TX_OUT undeclared` | HAL pinctrl header not in include path | Verify `west update` has fetched `hal_microchip` |

---

## References

- SoC DTS: `dts/arm/microchip/pic32c/pic32cx_bz/bz6x/common/pic32cxxbz6x.dtsi`
- Board DTS: `boards/microchip/pic32wm/pic32wm_bz6204_curiosity/pic32wm_bz6204_curiosity.dts`
- Board pinctrl: `boards/microchip/pic32wm/pic32wm_bz6204_curiosity/pic32wm_bz6204_curiosity-pinctrl.dtsi`
- CAN driver: `drivers/can/can_mchp_mcan_g1.c`
- CAN DT binding: `dts/bindings/can/microchip,mcan-g1.yaml`
- PIC32CX BZ6 FRM Section 24 — CAN controller register map
