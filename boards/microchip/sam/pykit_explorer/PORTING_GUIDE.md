# PyKit Explorer — CircuitPython → Zephyr Port Guide
# Verified against: mpconfigboard.h + pins.c (microchip_curiosity_circuitpython)

---

## Board Files Generated

```
boards/microchip/pykit_explorer/
├── board.yml                        ← Board metadata
├── board.cmake                      ← Programmer config (pyocd / openocd / jlink)
├── CMakeLists.txt
├── Kconfig.board                    ← BOARD_PYKIT_EXPLORER symbol
├── Kconfig.defconfig                ← USB strings, clock defaults
├── pykit_explorer_defconfig         ← Kernel config defaults
├── pykit_explorer.dts               ← Full board device tree
├── pykit_explorer-pinctrl.dtsi      ← All pin mux assignments
└── support/
    └── openocd.cfg                  ← EDBG / CMSIS-DAP
```

Place the folder at `boards/microchip/pykit_explorer/` inside the Zephyr
tree, or declare it out-of-tree in your app's CMakeLists.txt:

```cmake
set(BOARD pykit_explorer)
set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})   # your local boards/ lives here
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
```

---

## Verified Pin Map

Every entry below was read directly from `mpconfigboard.h` and `pins.c`.

### GPIO

| CircuitPython name | Pin  | Zephyr DTS node / GPIO spec         | Notes                    |
|--------------------|------|--------------------------------------|--------------------------|
| `board.LED` / D13  | PB23 | `<&portb 23 GPIO_ACTIVE_LOW>`       | Status LED               |
| `board.NEOPIXEL`   | PB22 | see ws2812 section below             | WS2812 RGB               |
| `board.BLE_CLR`    | PA14 | `<&porta 14 GPIO_ACTIVE_HIGH>`      | RNBD451 HW reset         |
| `board.LCD_BL`     | PA06 | `<&porta 6 GPIO_ACTIVE_HIGH>`       | LCD backlight (or PWM)   |
| `board.LCD_CS`     | PA07 | `<&porta 7 GPIO_ACTIVE_LOW>`        | LCD chip select          |
| `board.SD_CS`      | PA19 | `<&porta 19 GPIO_ACTIVE_LOW>`       | SD card chip select      |
| `board.CS` (IMU)   | PB01 | `<&portb 1 GPIO_ACTIVE_LOW>`        | IMU SPI CS / addr select |
| `board.IMU_INT`    | PB00 | `<&portb 0 GPIO_ACTIVE_HIGH>`       | IMU interrupt (= MISO!)  |
| `board.CAN_STDBY`  | PB17 | `<&portb 17 GPIO_ACTIVE_LOW>`       | CAN transceiver stdby    |
| `board.VREF`       | PA03 | — (hardware VREF, no driver needed) | ADC voltage reference    |
| `board.DAC`        | PA02 | `&dac0` node                        | DAC VOUT0                |

### SERCOM Allocation

| SERCOM   | Peripheral        | Pins                           | MUX | txpo/rxpo or dopo/dipo |
|----------|-------------------|--------------------------------|-----|------------------------|
| SERCOM0  | LCD SPI           | PA04 (MOSI), PA05 (SCK)       | D   | dopo=0                 |
| SERCOM1  | SD SPI            | PA16/PA17/PA18 (MOSI/SCK/MISO)| C   | dopo=0, dipo=2         |
| SERCOM2  | BLE UART          | PA12 (TX), PA13 (RX)          | C   | txpo=0, rxpo=1         |
| SERCOM3  | Debug UART ← **console** | PA22 (TX), PA23 (RX)  | C   | txpo=0, rxpo=1         |
| SERCOM5  | Main SPI          | PB02/PB03/PB00 (MOSI/SCK/MISO)| D  | dopo=0, dipo=2         |
| SERCOM7  | I2C               | PB31 (SDA), PB30 (SCL)        | D   | standard I2C           |

SERCOM4 and SERCOM6 are unallocated — available for expansion.

### Digital / Analog Pins

| CircuitPython | Pin  | Zephyr resource              |
|---------------|------|------------------------------|
| D0            | PA15 | `<&porta 15 0>`              |
| D1            | PA20 | `<&porta 20 0>`              |
| D2            | PA21 | `<&porta 21 0>`              |
| D3            | PA27 | `<&porta 27 0>`              |
| D4            | PB14 | `<&portb 14 0>`              |
| D5            | PB15 | `<&portb 15 0>`              |
| D6            | PB16 | `<&portb 16 0>`              |
| A0            | PB04 | `&adc1` channel 6            |
| A1            | PB05 | `&adc1` channel 7            |
| A2            | PB06 | `&adc1` channel 8            |
| A3            | PB07 | `&adc1` channel 9            |
| A4            | PB08 | `&adc1` channel 0 (ADC1)     |
| A5            | PB09 | `&adc1` channel 1 (ADC1)     |

> **ADC channel numbers:** Verify against DS60001507 Table 45-3 (ADC1 AINx).
> PB04-PB09 map to ADC1 (not ADC0) because they are on Port B.

### CAN

| Signal    | Pin  | MUX | Peripheral     |
|-----------|------|-----|----------------|
| CAN_TX    | PB12 | H   | CAN1 (MCAN1)   |
| CAN_RX    | PB13 | H   | CAN1 (MCAN1)   |
| CAN_STDBY | PB17 | —   | GPIO (active-low to enable transceiver) |

---

## Known Design Detail: IMU_INT shares PB00 with SPI MISO

`pins.c` maps `IMU_INT`, `MISO`, and `D8` all to `PB00`. This is the SAME51
Curiosity Nano's IMU addressing scheme: the IMU can communicate via either
SPI or I2C, selected by the PB01 (IMU_ADDR) logic level.

**In SPI mode:** PB00 is SERCOM5/PAD[2] (MISO). The IMU interrupt is not
usable as an edge-triggered GPIO while SPI transfers are in progress.
Configure the IMU interrupt as a level-triggered input or poll status registers.

**In I2C mode:** PB00 is a dedicated GPIO interrupt line. Disable SERCOM5
pinctrl for PB00 and configure it as `GPIO_INPUT`.

---

## NeoPixel (PB22 — WS2812)

PB22 is **SERCOM5/PAD[2]** on MUX C. Since SERCOM5 is already used for
main SPI on MUX D, and MUX C is a different alternate function, PB22 can
be driven by a dedicated SPI instance for the WS2812 protocol. The simplest
approach in Zephyr is the `ws2812-spi` driver with a carefully chosen bit
rate to encode the 0/1 pulse widths.

The `worldsemi,ws2812-spi` binding requires an SPI bus; the output is the
MOSI line. Since WS2812 is one-way, no MISO or CS pins are needed. The
timing requires an SPI clock of 2.4 or 3.2 MHz (encoding each WS2812 bit as
3 SPI bits).

To enable:
1. Route PB22 to a free SERCOM as MOSI (e.g. if SERCOM4 is free and PB22
   maps to it — verify in Table 6-1 of the datasheet).
2. Add a `ws2812_spi` node on that SERCOM.
3. Enable `CONFIG_LED_STRIP=y` and `CONFIG_WS2812_SPI=y`.

---

## Enabling Peripherals

### I2C + ICM-20948 (your PID loop IMU)

In `pykit_explorer.dts`, inside `&sercom7`:
```dts
icm20948: icm20948@68 {
    compatible = "invensense,icm20948";
    reg = <0x68>;
    /* IMU_INT = PB00 — only usable as INT when not using SPI */
    int-gpios = <&portb 0 GPIO_ACTIVE_HIGH>;
};
```
In `prj.conf`:
```
CONFIG_I2C=y
CONFIG_SENSOR=y
```

### ST7789 LCD (your DisplayManager)

In `pykit_explorer.dts`, inside `&sercom0`:
```dts
st7789: st7789v@0 {
    compatible = "sitronix,st7789v";
    spi-max-frequency = <20000000>;
    reg = <0>;
    cs-gpios    = <&porta 7 GPIO_ACTIVE_LOW>;    /* LCD_CS */
    /* Add cmd-data and reset gpios if wired */
    width  = <240>;
    height = <240>;
};
```
In `prj.conf`:
```
CONFIG_SPI=y
CONFIG_DISPLAY=y
CONFIG_ST7789V=y
```
LCD backlight PWM (PA06) can be mapped to a TC/TCC output — verify which
timer function is available on PA06 MUX E or F in the datasheet.

### CAN Bus

In `prj.conf`:
```
CONFIG_CAN=y
CONFIG_CAN_SAM=y
```
Drive `CAN_STDBY` (PB17) low at startup to take the transceiver out of
standby. This is handled by the `regulator-fixed` node in the DTS which
asserts the GPIO at boot via `regulator-boot-on`.

---

## Concept Mapping: CircuitPython → Zephyr

| CircuitPython                        | Zephyr equivalent                            |
|--------------------------------------|----------------------------------------------|
| `board.LED` → `DigitalInOut`         | `gpio_pin_toggle_dt(&led0_spec)`             |
| `busio.UART(board.DEBUG_TX, ...)`    | `uart_tx()` on `DEVICE_DT_GET(DT_NODELABEL(sercom3))` |
| `busio.UART(board.BLE_TX, ...)`      | `uart_tx()` on `DEVICE_DT_GET(DT_NODELABEL(sercom2))` |
| `busio.I2C(board.SCL, board.SDA)`    | `i2c_write_dt()` / `i2c_read_dt()`          |
| `busio.SPI(board.SCK, ...)`          | `spi_transceive_dt()` on sercom5             |
| `board.LCD_SPI`                      | `spi_write_dt()` on sercom0                  |
| `board.SD_SPI`                       | `spi_transceive_dt()` on sercom1             |
| `analogio.AnalogIn(board.A0)`        | `adc_read()` on `&adc1` channel 6           |
| `analogio.AnalogOut(board.DAC)`      | `dac_write_value()` on `&dac0`              |
| `neopixel.NeoPixel(board.NEOPIXEL)`  | `led_strip_update_rgb()` (ws2812-spi)        |
| `asyncio.sleep(0)`                   | `k_yield()`                                  |
| `asyncio` task @ 200 Hz             | `k_thread_create()` + `k_sleep(K_USEC(5000))` |
| `supervisor.ticks_ms()`              | `k_uptime_get()`                             |
| `time.sleep(n)`                      | `k_sleep(K_MSEC(n * 1000))`                |
| `microcontroller.reset()`            | `sys_reboot(SYS_REBOOT_COLD)`               |

---

## Build & Flash

```bash
west build -b pykit_explorer samples/basic/blinky
west flash                        # via EDBG/CMSIS-DAP (no extra hardware)
screen /dev/ttyACM0 115200        # EDBG virtual COM = sercom3
```

---

## References

- SAM D5x/E5x Datasheet **DS60001507** — Table 6-1 (PORT multiplexing), Table 45-3 (ADC1 AINx)
- SAME51 Curiosity Nano User Guide **DS50002770**
- [Zephyr Board Porting Guide](https://docs.zephyrproject.org/latest/hardware/porting/board_porting.html)
- [Closest in-tree board: atsame54_xpro](https://github.com/zephyrproject-rtos/zephyr/tree/main/boards/arm/atsame54_xpro)
- [SAME51 SoC in Zephyr](https://github.com/zephyrproject-rtos/zephyr/tree/main/soc/arm/atmel_sam0/same51)
