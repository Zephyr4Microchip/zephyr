.. zephyr:board:: wbz451hpe_curiosity

Overview
*********

The **WBZ451HPE Curiosity Board** is a development platform designed for evaluating and prototyping with the WBZ451HPE multi-protocol wireless MCU module.

Based on the PIC32CX-BZ2 family of MCUs, it offers a modular, efficient, and feature-rich environment for rapid development and demonstration of wireless applications. The board supports BLE 5.2 and Zigbee, and includes:

* On-board debugger/programmer (PKOB4)
* mikroBUS™ socket for Click board™ expansion
* XPRO header for additional connectivity
* USB or Li-Po battery power options
* RGB LED and temperature sensor

WBZ451HPE supports the following drivers:

* UART
* Flash
* GPIO
* Pin Control
* Entropy
* DMA
* RTC
* I2C
* SPI
* PWM
* Clock Control
* Reset




Hardware
********

* **WBZ451HPE Module**: Integrated wireless MCU module with PCB antenna.
* **On-board Debugger/Programmer**: MPLAB PICkit On-Board 4 (PKOB4).
* **mikroBUS Socket**: For Click board™ expansion.
* **Power Supply**: USB or Li-Po battery powered; includes charger and management circuit.
* **Switches and LEDs**: RGB LED (PWM-controlled), user switch, reset switch.
* **USB Connectivity**: Micro-B USB connector for programming, debugging, and UART.
* **Other Peripherals**: Temperature sensor, QSPI serial flash, 32.768 kHz crystal oscillator.

**Design Files:**  
`WBZ451HPE Curiosity Board Product Page <https://ww1.microchip.com/downloads/aemDocuments/documents/WSG/ProductDocuments/BoardDesignFiles/WBZ451H-Curiosity-Board-Hardware-Design-Documentation.zip>`_


Supported Features
==================

.. zephyr:board-supported-hw::

For complete list of `WBZ451HPE Curiosity Board User’s Guide <https://ww1.microchip.com/downloads/aemDocuments/documents/WSG/ProductDocuments/UserGuides/WBZ451HPE-Curiosity-Board-User-Guide-DS50003681.pdf>`_


Connections and IOs
===================

User LED (RGB)
--------------

- RGB LED available on the WBZ451HPE Curiosity Board.
  
  - Connected to PWM-capable GPIOs.
  - Each color (Red, Green, Blue) can be controlled independently via PWM.
  - Useful for status indication and visual feedback.

Push Buttons
------------

The WBZ451HPE Curiosity Board includes two switches:

- **Reset Switch (SW1)**: Connected to **MCLR** pin. Pulls high in idle; pressing drives low and resets the module.
- **User Configurable Switch (SW2)**: Connected to **PB4**. Pulls high in idle; pressing drives low.

**Switches Description**

.. list-table::
   :header-rows: 1
   :widths: 25 25 50

   * - Switch Name
     - Pin on WBZ451HPE Curiosity Board
     - Description
   * - Reset (SW1)
     - MCLR
     - Reset switch connected to MCLR pin
   * - User Button (SW2)
     - PB4
     - User configurable switch

mikroBUS Socket
---------------

The mikroBUS™ socket supports Click board™ expansion and includes:

- **Communication Interfaces**: SPI, UART, I²C
- **Power Lines**: 3.3V/GND and 5V/GND
- **Additional Signals**: PWM, RST, INT, AN

**mikroBUS Socket Pinout**

.. list-table::
   :header-rows: 1
   :widths: 10 15 30 45

   * - Pin #
     - Pin Name
     - Pin on WBZ451HPE Curiosity Board
     - Description
   * - 1
     - AN
     - PB6_AN
     - ADC analog input
   * - 2
     - RST
     - PB2
     - General purpose I/O pin
   * - 3
     - CS
     - PB13_SERCOM0_PAD3
     - Client select pin for SPI or general purpose I/O
   * - 4
     - SCK
     - PB11_SERCOM0_PAD1
     - SPI clock
   * - 5
     - MISO
     - PB10_SERCOM0_PAD2
     - SPI host input client output
   * - 6
     - MOSI
     - PB12_SERCOM0_PAD0
     - SPI host output client input
   * - 7
     - +3.3V
     - +3.3V
     - +3.3V power supply
   * - 8
     - GND
     - GND
     - Ground
   * - 9
     - PWM
     - PB0
     - General purpose I/O pin
   * - 10
     - INT
     - PB1
     - General purpose I/O pin
   * - 11
     - TX
     - PA1_SERCOM1_PAD0
     - UART TX
   * - 12
     - RX
     - PA0_SERCOM1_PAD1
     - UART RX
   * - 13
     - SCL
     - PA3_SERCOM1_PAD1
     - I²C clock
   * - 14
     - SDA
     - PA2_SERCOM1_PAD0
     - I²C data
   * - 15
     - +5V
     - +5V
     - +5V power supply
   * - 16
     - GND
     - GND
     - Ground

For detailed schematics, pin assignments, and signal descriptions, see the  `WBZ451HPE Curiosity Board User’s Guide <https://ww1.microchip.com/downloads/aemDocuments/documents/WSG/ProductDocuments/UserGuides/WBZ451HPE-Curiosity-Board-User-Guide-DS50003681.pdf>`_


Programming and Debugging
*************************

This section describes how to flash and debug applications on the Microchip Wireless WBZ451 Curiosity board using Zephyr.

**Supported Debuggers**

.. list-table::
   :header-rows: 1
   :widths: 20 20 20 20 20

   * - 
     - Flash
     - Debug
     - Debug Server
     - Debug Tool
   * - Segger
     - ✓
     - ✓
     - ✓
     - J-Link
   * - OpenOCD
     - ✓
     - ✓
     - ✓
     - PKOB4, PICkit Basic

Flashing
========

Follow the steps below to build and flash your application:

1. Open a terminal and change to the Zephyr workspace directory:

   .. code-block:: console

      cd zephyr

2. Build the application using the following command:

   .. code-block:: console

      west build -p always -b wbz451hpe_curiosity .\samples\basic\blinky\

3. After a successful build, connect the WBZ451HPE device to your machine.

4. Flash the device using the `west flash` command:

   .. code-block:: console

      west flash

5. Ensure the flash process completes successfully. You should see confirmation messages in the terminal.

Debugging
=========

To debug the WBZ451HPE application using Visual Studio Code:

1. Ensure the application is built for the HPE board.
2. Install the `cortex-debug` extension in Visual Studio Code.
3. Open the workspace and click the **Run and Debug** icon on the left sidebar.
4. If `launch.json` and `tasks.json` files are already present, VS Code will automatically start the debug session.
5. If prompted to create a new `launch.json`, select the **Cortex Debug** debugger option.
6. Replace the contents of `launch.json` with:

   .. code-block:: json

      {
        "version": "2.0.0",
        "configurations": [
          {
            "name": "Debug WBZ451 HPE",
            "type": "cortex-debug",
            "request": "attach",
            "servertype": "openocd",
            "cwd": "C:\\developers\\zephyr\\",
            "executable": "<path to zephyr project>/build/zephyr/zephyr.elf",
            "device": "WBZ451HPE",
            "configFiles": [
              "interface/cmsis-dap.cfg",
              "target/wbz451h.cfg"
            ],
            "gdbPath": "<path to zephyr sdk>/arm-zephyr-eabi/bin/arm-zephyr-eabigdb.exe",
            "preLaunchTask": "flash_wbz451hpe_hex",
            "postRestartCommands": [
              "symbol-file <path to zephyr project>/build/zephyr/zephyr.elf",
              "monitor reset halt",
              "break main"
            ],
            "showDevDebugOutput": "none"
          }
        ]
      }

7. Create `tasks.json` inside `.vscode` with:

   .. code-block:: json

      {
        "version": "2.0.0",
        "tasks": [
          {
            "label": "flash_wbz451hpe_hex",
            "type": "shell",
            "command": "openocd",
            "args": [
              "-f", "interface/cmsis-dap.cfg",
              "-f", "target/wbz451h.cfg",
              "-c", "init",
              "-c", "reset halt",
              "-c", "program <path to zephyr project>/build/zephyr/zephyr.hex reset exit"
            ],
            "problemMatcher": [],
            "group": {
              "kind": "build",
              "isDefault": true
            }
          }
        ]
      }

8. Connect the WBZ451HPE Curiosity board.
9. Click the **Run and Debug** icon again and select the WBZ451HPE debug option.
10. Confirm that the debugger hits the breakpoint in `main.c`. Press **Continue** to proceed.

References
==========

- `WBZ451HPE Curiosity Board Product Page <https://www.microchip.com/en-us/development-tool/ev79y91a>`_
- `WBZ451HPE Curiosity Board User Guide <https://ww1.microchip.com/downloads/aemDocuments/documents/WSG/ProductDocuments/UserGuides/WBZ451HPE-Curiosity-Board-User-Guide-DS50003681.pdf>`_
- `Hardware Design Files <https://ww1.microchip.com/downloads/aemDocuments/documents/WSG/ProductDocuments/BoardDesignFiles/WBZ451H-Curiosity-Board-Hardware-Design-Documentation.zip>`_
- `PIC32CX-BZ2 and WBZ45 Family Data Sheet <https://ww1.microchip.com/downloads/aemDocuments/documents/WSG/ProductDocuments/DataSheets/PIC32CX-BZ2-and-WBZ45-Family-Data-Sheet-DS70005504.pdf>`_
- `mikroBUS Click Boards <https://www.mikroe.com/click>`_
- `Microchip Support Portal <http://support.microchip.com/>`_
- `Microchip Direct <https://www.microchipdirect.com/?srsltid=AfmBOop0KWt1byQZUafcD8wwzrgQX_iuCJLi6AmzTIzhI6Ez-D2IZr_M>`_
