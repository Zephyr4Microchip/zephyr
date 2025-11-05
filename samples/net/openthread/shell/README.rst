.. zephyr:code-sample:: openthread-shell
   :name: OpenThread shell
   :relevant-api: net_stats

   Test Thread and IEEE 802.15.4 using the OpenThread shell.

Overview
********

This sample allows testing the Thread protocol and the underlying IEEE 802.15.4 drivers for various
boards using the OpenThread shell.

Building and Running
********************

Verify that the board and chip you are targeting provide IEEE 802.15.4 support.

There are configuration files for different boards and setups in the shell directory:

- :file:`prj.conf`
  Generic config file.

- :file:`overlay-ot-rcp-host-nxp.conf`
  This overlay config enables support of OpenThread RCP host running on NXP chips over IMU interface.

Build shell application like this:

.. zephyr-app-commands::
   :zephyr-app: samples/net/openthread/shell
   :board: <board to use>
   :conf: <config file to use>
   :goals: build
   :compact:

Example building for Nordic's nRF52840 DK.

.. zephyr-app-commands::
   :zephyr-app: samples/net/openthread/shell
   :board: nrf52840dk/nrf52840
   :conf: "prj.conf"
   :goals: build
   :compact:

Example building for NXP's RW612 FRDM (RCP host).

.. zephyr-app-commands::
   :zephyr-app: samples/net/openthread/shell
   :board: frdm_rw612
   :conf: "prj.conf overlay-ot-rcp-host-nxp.conf"
   :goals: build
   :compact:

Example building for Microchip's wbz450/wbz451/wbz451hpe curiosity boards.

.. zephyr-app-commands::
   :zephyr-app: samples/net/openthread/shell
   :board: <board to use>
   :conf: "prj.conf"
   :goals: build
   :compact:

Example building for Microchip's pic32wm_bz6204_curiosity board.

.. zephyr-app-commands::
   :zephyr-app: samples/net/openthread/shell
   :board: pic32wm_bz6204_curiosity
   :conf: "prj.conf"
   :gen-args: "-DEXTRA_CONF_FILE=boards/pic32wm_bz6204_curiosity.conf"
   :goals: build
   :compact:

Sample console interaction
==========================

.. code-block:: console

   uart:~$ ot scan
   | PAN  | MAC Address      | Ch | dBm | LQI |
   +------+------------------+----+-----+-----+
   | fe09 | abcdef1234567890 | 15 | -78 |  60 |
   Done
