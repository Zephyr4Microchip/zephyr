.. zephyr:code-sample:: ble_peripheral_trp_uart
   :name: Peripheral Transparent UART (TRP UART)
   :relevant-api: bluetooth

   Bidirectional BLE-to-UART bridge using Microchip Transparent Profile (TRP)
   with Credit Based Flow Control (CBFC).

Overview
********

This sample implements a **BLE Peripheral** that bridges a hardware UART to a
BLE Central over Microchip's proprietary **Transparent Profile (TRP)**. Data
written to the UART is forwarded to the connected Central as GATT TX
notifications; data written by the Central to the RX characteristic is
forwarded to the UART TX pin.

The application is structured in two reusable subsystem layers and a thin
application layer:

- **Service layer** (``subsys/bluetooth/services/trs/trs.c``) — owns the GATT
  attribute table (TX, RX, Control Point characteristics and their CCCDs) and
  exposes a callback-based raw API.
- **Server profile layer** (``subsys/bluetooth/services/trs/trs_server.c``) —
  sits above the service layer and adds per-connection state, Credit Based Flow
  Control (CBFC), a receive packet queue, MTU tracking, and named application
  events.
- **Application layer** (``src/``) — UART ISR, work-queue-based data pump,
  BLE advertising lifecycle, connection policy, and GATT discovery.

.. list-table:: Key facts
   :widths: 30 70
   :header-rows: 0

   * - Zephyr version
     - 4.2.0
   * - BLE role
     - Peripheral (GATT Server)
   * - Profile
     - Microchip Transparent Profile (TRP) — proprietary 128-bit UUIDs
   * - Flow control
     - Credit Based Flow Control (CBFC) over Control Point characteristic
   * - Advertising name
     - ``Microchip`` (``CONFIG_BT_DEVICE_NAME``)
   * - Max connections
     - 1 (``CONFIG_BT_MAX_CONN=1``)

Features
********

- Bidirectional transparent data bridge between a hardware UART and a BLE
  Central.
- Full CBFC flow control: the server grants credits to the Central and
  returns credits as received packets are consumed.
- MTU-aware segmentation: payload size adapts to the negotiated ATT MTU.
- Automatic advertising restart 50 ms after every disconnection (deferred
  work item avoids stack resource contention).
- GATT service and server profile registered as a Zephyr extra module; no
  in-tree modifications required.
- GATT discovery helper for the TRS service UUID and characteristic handles.
- Bond storage via Zephyr Settings over NVS flash.
- Per-connection retry logic for transient UART-to-BLE send failures.

Supported Hardware
******************

.. list-table::
   :widths: 50 50
   :header-rows: 1

   * - Board
     - Board Target
   * - PIC32WM BZ6204 Curiosity
     - ``pic32wm_bz6204_curiosity``
   * - WBZ451 Curiosity
     - ``wbz451_curiosity``
   * - WBZ351 Curiosity
     - ``wbz351_curiosity``


Prerequisites
*************

The following tools must be installed before building:

- **Zephyr 4.2.0 workspace** — initialised from the upstream manifest or a
  Microchip fork (``Zephyr4Microchip/zephyr``, branch ``mchp_pic32cxbz_v4.2.0``
  or later). The workspace must include ``west blobs fetch hal_microchip`` for
  BZ-family boards.
- **Zephyr SDK 0.17.4** — ARM Cortex-M toolchain
  (``arm-zephyr-eabi-gcc 12.2.0``).
- **west ≥ 0.14.0** and **CMake ≥ 3.20**.
- **Microchip custom OpenOCD** for flashing BZ-family boards. Install from
  `<https://github.com/MicrochipTech/openOCD-wireless>`_ or use a pre-built
  copy at a local path (referred to below as ``<openocd-path>``).
- **Serial terminal** at 115200 baud 8N1 (e.g. Tera Term, minicom).
- **BLE Central** — Microchip Bluetooth Data app or any generic BLE UART app
  (LightBlue)

Application Files
*****************

.. list-table::
   :widths: 42 58
   :header-rows: 1

   * - File or Directory
     - Purpose
   * - ``CMakeLists.txt``
     - Sample build entry point; lists ``src/`` source files explicitly.
   * - ``prj.conf``
     - Kconfig configuration; enables BT stack, TRS service + server,
       net_buf, NVS, settings, and MTU sizing.
   * - ``Kconfig``
     - Sample menu config; sources ``Kconfig.zephyr`` only — all TRS options
       are owned by the subsys module.
   * - ``sample.yaml``
     - West sample descriptor with supported board list.
   * - ``src/main.c``
     - Entry point: calls ``app_init()`` and returns (Zephyr idle loop runs).
   * - ``src/app.c`` / ``app.h``
     - UART initialisation, ring-buffer-based UART→BLE work queue, retry
       logic, and BLE connection state handler.
   * - ``src/app_ble.c`` / ``app_ble.h``
     - ``bt_enable()`` wrapper, advertising data, ``app_ble_adv_restart()``
       deferred work item.
   * - ``src/app_ble_handler.c`` / ``app_ble_handler.h``
     - ``bt_conn_cb`` implementation; stores/releases the active connection
       reference; dispatches connect and disconnect to the application.
   * - ``src/app_ble_policy.c`` / ``app_ble_policy.h``
     - Connection parameter validation, security level enforcement, pairing
       callbacks.
   * - ``src/app_ble_discovery.c`` / ``app_ble_discovery.h``
     - GATT primary service and characteristic discovery for TRS UUIDs;
       stores TX, RX, and CP value handles.
   * - ``src/app_trsps_handler.c`` / ``app_trsps_handler.h``
     - Registers TRS server event callback; routes BLE→UART data through a
       ``net_buf`` FIFO to the UART TX ISR path; triggers UART→BLE pump on
       TX channel open and CBFC credit events.

The subsystem files used by this sample are in a separate module:

.. list-table::
   :widths: 42 58
   :header-rows: 1

   * - File
     - Purpose
   * - ``subsys/bluetooth/services/trs/trs.c``
     - GATT attribute table, raw write handlers, notify API, runtime
       permission config.
   * - ``subsys/bluetooth/services/trs/trs.h``
     - Public service-layer API (UUIDs, callback struct, notify, register).
   * - ``subsys/bluetooth/services/trs/trs_server.c``
     - Server profile layer: per-connection state, CBFC, receive queue,
       MTU tracking, event dispatch.
   * - ``subsys/bluetooth/services/trs/trs_server.h``
     - Public server-layer API (events, send, get data, register callback).
   * - ``subsys/bluetooth/services/trs/Kconfig.trs``
     - Defines ``BT_TRS``, ``BT_TRS_SERVER``, and all ``TRS_SERVER_*``
       tuning options.
   * - ``subsys/bluetooth/services/trs/CMakeLists.txt``
     - Conditionally compiles ``trs.c`` and ``trs_server.c`` under
       ``CONFIG_BT_TRS`` and ``CONFIG_BT_TRS_SERVER``.
   * - ``CMakeLists.txt`` (module root)
     - Exposes ``include/`` to the build and adds the ``trs/`` subdirectory.
   * - ``Kconfig`` (module root)
     - Sources ``Kconfig.trs`` into the Zephyr Kconfig tree via ``rsource``.
   * - ``zephyr/module.yml``
     - West extra-module descriptor; points CMake and Kconfig to the
       module root.

**Generated build artefacts** (inside the ``-d <build-dir>`` directory):

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - File
     - Description
   * - ``zephyr/zephyr.elf``
     - ELF with full debug symbols; used by debuggers and size tools.
   * - ``zephyr/zephyr.hex``
     - Intel HEX image; passed to OpenOCD by ``west flash``.
   * - ``zephyr/zephyr_signed.hex``
     - Signed HEX produced by the board's post-build signing step; this is
       the image actually written to flash on BZ3 and BZ6 family boards.
   * - ``zephyr/mchp-image.bin``
     - Raw binary produced alongside the signed HEX.

Build the Application
*********************

The sample application is available in the following path: zephyr/samples/bluetooth/peripheral_trp_uart


West Build
*********************

.. code-block:: bash

   west build \
     -s <path-to-app> \
     -b <board-name> \
     -p always \
     -d <build-directory>


Flash the Application
*********************

.. code-block:: bash

   west flash \
      -d <build-directory>

The runner uses ``zephyr_signed.hex`` (the post-build signed image). A
successful flash ends with::

   wrote <N> bytes from file zephyr_signed.hex in <T>s (<rate> KiB/s)

Run and Verify the Application
*******************************

1. Connect a serial terminal to the board port at
   **115200 baud, 8N1, no flow control**.

2. Reset or power-cycle the board.

3. Open the **Microchip Bluetooth Data** app (or any BLE scanner) on a phone.
   Scan for the device named ``Microchip``.

4. Connect. Expected log:
  .. code-block:: none
  [00:00:xx.xxx] <inf> app_ble_handler: Connected

5. Type characters in the terminal. They appear in the Central's app (UART→BLE
   direction).

6. Send data from the Central. The bytes appear in the terminal output
   (BLE→UART direction).

7. Disconnect from the phone. Expected log approximately 50 ms later:

  .. code-block:: none

  [00:00:xx.xxx] <inf> trs: Tx notifications disabled

  [00:00:xx.xxx] <inf> trs: CP notifications disabled

  [00:00:xx.xxx] <inf> app_ble_handler: Disconnected, reason 19

  [00:00:xx.xxx] <inf> app_ble: Configuration mode: waiting connections...


8. The device is advertising again and ready for the next connection.

Services Integration
********************


Service files compiled
======================

.. list-table::
   :widths: 30 20 25 25
   :header-rows: 1

   * - Service File
     - Purpose
     - Included Through
     - Used By
   * - ``trs.c``
     - GATT attribute registration, raw write handlers, TX/CP notify API
     - ``CONFIG_BT_TRS=y`` → ``trs/CMakeLists.txt``
     - ``trs_server.c``, ``app_ble_discovery.c``
   * - ``trs_server.c``
     - CBFC, connection list, receive queue, event dispatch
     - ``CONFIG_BT_TRS_SERVER=y`` → ``trs/CMakeLists.txt``
     - ``app_trsps_handler.c``, ``app.c``
   * - ``trs.h``
     - Service UUIDs, callback struct, notify and register prototypes
     - ``include/zephyr/bluetooth/services/trs/``
     - ``trs_server.c``, ``app_ble_discovery.c``
   * - ``trs_server.h``
     - Server events, send/receive API, callback register
     - ``include/zephyr/bluetooth/services/trs/``
     - ``app_trsps_handler.c``, ``app.c``

Required Kconfig configuration
===============================

.. code-block:: ini

   # ── BLE core ───────────────────────────────────────────────────
   CONFIG_BT=y                        # Bluetooth stack (mandatory)
   CONFIG_BT_PERIPHERAL=y             # GATT server / peripheral role
   CONFIG_BT_SMP=y                    # Security Manager Protocol (pairing)
   CONFIG_BT_SETTINGS=y               # Bond storage via Settings
   CONFIG_BT_GATT_DYNAMIC_DB=y        # Required for trs.c SYS_INIT registration
   CONFIG_BT_DEVICE_NAME="Microchip"  # Advertising name

   # ── BLE buffer sizing ──────────────────────────────────────────
   CONFIG_BT_L2CAP_TX_MTU=247         # 244-byte payload + 3-byte ATT header
   CONFIG_BT_BUF_ACL_RX_SIZE=255
   CONFIG_BT_BUF_ACL_TX_SIZE=251
   CONFIG_BT_BUF_CMD_TX_SIZE=255
   CONFIG_BT_RX_STACK_SIZE=2048
   CONFIG_BT_MAX_CONN=1               # Single connection

   # ── Storage ────────────────────────────────────────────────────
   CONFIG_FLASH=y                     # Flash driver (bond storage)
   CONFIG_FLASH_MAP=y
   CONFIG_NVS=y                       # NVS backend for Settings
   CONFIG_SETTINGS=y

   # ── Net buf (required by trs_server.c receive queue) ──────────
   CONFIG_NET_BUF=y

   # ── TRS service layer ──────────────────────────────────────────
   CONFIG_BT_TRS=y                    # Enables trs.c (GATT attributes)

   # ── TRS server profile layer ───────────────────────────────────
   CONFIG_BT_TRS_SERVER=y             # Enables trs_server.c (CBFC, events)

   # ── TRS server tuning (override defaults if needed) ───────────
   CONFIG_TRS_SERVER_CREDIT=16        # Initial CBFC credit count; also sets queue depth
   CONFIG_TRS_SERVER_BUF_POOL_SIZE=16 # net_buf pool size (>= CREDIT × MAX_CONN)
   CONFIG_TRS_SERVER_BUF_DATA_SIZE=244 # Max payload per buffer (≤ ATT MTU − 3)

   # ── System work queue ──────────────────────────────────────────
   CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE=2048

   # ── Logging ────────────────────────────────────────────────────
   CONFIG_LOG=y

Dependency notes:

- ``CONFIG_BT_TRS_SERVER`` requires ``CONFIG_BT_TRS``.
- ``CONFIG_BT_TRS_SERVER`` selects ``CONFIG_NET_BUF`` automatically via
  ``Kconfig.trs``; setting it explicitly is recommended for clarity.
- ``CONFIG_BT_GATT_DYNAMIC_DB=y`` is required because ``trs.c`` registers
  the GATT service at ``APPLICATION`` init level via ``SYS_INIT``.

Integration sequence for new applications
==========================================

Follow this sequence to reuse the TRS service and server in a new Zephyr
application:

1. **Enable the Kconfig options** in ``prj.conf``:

   .. code-block:: ini

      CONFIG_BT_TRS=y
      CONFIG_BT_TRS_SERVER=y

2. **Include the headers** in application sources:

   .. code-block:: c

      #include <zephyr/bluetooth/services/trs/trs.h>       /* service API */
      #include <zephyr/bluetooth/services/trs/trs_server.h> /* server API */

3. **Register a TRS server callback** before ``bt_enable()``:

   .. code-block:: c

      static void my_trs_event(struct trs_server_event *evt)
      {
          if (evt->event_id == TRS_SERVER_EVT_RECEIVE_DATA) {
              /* pull data with trs_server_get_data() */
          }
      }

      static struct trs_server_cb my_cb = { .trs_server_event = my_trs_event };

      trs_server_cb_register(&my_cb);

4. **Send data** to the Central:

   .. code-block:: c

      trs_server_send_data(conn, len, data);

5. **No explicit service initialisation** is needed. ``trs.c`` and
   ``trs_server.c`` both register themselves via ``SYS_INIT`` at
   ``APPLICATION`` priority level, before ``main()`` runs.

Runtime sequence
================

.. code-block:: text

   Power on / reset
        │
        ▼
   Zephyr kernel + drivers initialise
        │
        ▼
   SYS_INIT (APPLICATION level):
     bt_trs_init()      → bt_trs_add() registers GATT attributes
     trs_server_init()  → registers conn + GATT callbacks, bt_trs_cb_register()
        │
        ▼
   main() → app_init()
     uart_init()              → UART ISR armed
     app_ble_handler_init()   → bt_conn_cb registered
     app_trsps_handler_init() → trs_server_cb_register(), work item init
     app_ble_init()           → bt_enable(app_ble_ready_cb)
        │
        ▼
   app_ble_ready_cb() [BT RX thread]
     settings_load()          → bonds restored from NVS
     bt_le_adv_start()        → advertising begins ("Microchip")
        │
        ▼
   Central connects
     app_ble_connected()      → connection ref stored
     bt_conn_set_security()   → pairing initiated (optional)
        │
        ▼
   Central enables TX CCCD
     uart_to_ble_ccc_changed() → sync_status_events()
     TRS_SERVER_EVT_TX_STATUS  → on_ble_tx_channel_ready()
     uart_to_ble_work submitted → UART ring buffer drained to BLE TX
        │
        ▼
   Data flows bidirectionally
     UART ISR → ring buffer → work queue → trs_server_send_data()
     Central writes RX char → trs_ble_to_uart_receive() → net_buf queue
     → ble_to_uart_work → uart_fifo_fill() ISR
        │
        ▼
   Central disconnects (reason 19 = remote user terminated)
     app_ble_disconnected()        → conn ref released
     app_ble_state_changed(false)  → cancel retry work
     app_ble_adv_restart()         → k_work_reschedule +50 ms
        │
        ▼ (50 ms later, system work queue)
   bt_le_adv_start()  → advertising restarts

References
**********

- `Zephyr Project Documentation <https://docs.zephyrproject.org/>`_
- `Zephyr BLE API Reference <https://docs.zephyrproject.org/latest/connectivity/bluetooth/api/index.html>`_
- `Zephyr BLE GATT API <https://docs.zephyrproject.org/latest/connectivity/bluetooth/api/gatt.html>`_
- `Microchip BLE Transparent Profile specification <https://onlinedocs.microchip.com/oxy/GUID-A5330D3A-9F51-4A26-B71D-8503A493DF9C-en-US-11/GUID-07EC83B7-CC28-4C55-8634-4B7F56A9DB36.html>`_
- `Microchip custom OpenOCD for wireless <https://github.com/MicrochipTech/openOCD-wireless>`_
- `PIC32WM BZ6204 Curiosity board <https://www.microchip.com/en-us/development-tool/EV31U42A>`_
- `WBZ451 Curiosity board <https://www.microchip.com/en-us/development-tool/EV96B94A>`_
- `WBZ351 Curiosity board <https://www.microchip.com/en-us/development-tool/EV19J06A>`_
