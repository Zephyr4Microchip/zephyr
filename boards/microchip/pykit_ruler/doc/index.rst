.. zephyr:board:: pykit_ruler

Overview
********

Microchip Curiosity PyKit Explorer is a CircuitPython development kit
designed in the form factor of a 12-inch ruler, featuring a SAME51 MCU,
TFT display, IMU, sensors, Class D audio Amplifier with Speaker, BLE
and SD Card

Hardware
********

- 4MB of Flash for storing user programs and libraries
- 9-DoF IMU for motion and magnetic field sensing
- CAN bus support
- 1.14” TFT color display
- Class-D audio amplifier and 8-ohm speaker
- Five NeoPixels
- MicroSD slot
- Wireless connectivity via the Microchip RNBD451 BLE module
- STEMMA/QWIIC connector for adding I²C breakout boards

Supported Features
==================

.. zephyr:board-supported-hw::

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

Flash Using MPLAB IPE
=====================
Connect an external debugger/programmer like MPLAB Snap or PICKit Basic
to the debug port.

For instructions on flashing using MPLAB IPE see :ref:`microchip-mplab-ipe-flashing`.

#. Accessing Console -
   UART pins can be accessed using J9 on the Daughtercard or
   J2 on the Ruler. All Signals are at MCU's VCC level 3v3
   Pin 15 - Gnd
   Pin 17 - Tx
   Pin 18 - Rx

References
**********

PyKIT Ruler Product Blog:
    https://www.microchip.com/en-us/about/media-center/blog/2026/pykit-explorer-circuitpython-development-kit
