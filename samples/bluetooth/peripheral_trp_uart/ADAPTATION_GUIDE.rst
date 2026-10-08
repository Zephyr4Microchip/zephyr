Adapting the Transparent UART Sample for Your Application
##################################################

**Target audience:** developers who want to use this BLE peripheral
sample as a base for their own application — for example, sending sensor readings
or receiving motor control parameters over BLE.

Architecture Block Diagram
**************************

.. code-block:: text

   ┌───────────────────────────────────────────────────────────────────────────────┐
   │                              APPLICATION LAYER                                │
   │                                                                               │
   │   ┌──────────────────────────────┐   ┌─────────────────────────────────────┐  │
   │   │       YOUR DATA SOURCE       │   │       YOUR COMMAND HANDLER          │  │
   │   │  (sensor, motor, ADC, etc.)  │   │  (parse received bytes, set PWM,    │  │
   │   │                              │   │   write registers, etc.)            │  │
   │   │  Reads → calls               │   │                                     │  │
   │   │  trs_server_send_data()      │   │  Called from your event handler     │  │
   │   └──────────────┬───────────────┘   └───────────────────┬─────────────────┘  │
   │                  │ send                                   ▲ receive            │
   └──────────────────┼─────────────────────────────────────── ┼ ───────────────────┘
                      │                                         │
   ┌──────────────────▼─────────────────────────────────────── ┼ ───────────────────┐
   │                   TRS SERVER PROFILE LAYER (DO NOT MODIFY)│                    │
   │                  trs_server.c / trs_server.h               │                    │
   │                                                            │                   │
   │   trs_server_send_data()      ◄── you call this            │                   │
   │   trs_server_get_data()       ──► you call this ───────────┘                   │
   │   trs_server_get_data_len()   ──► you call this                                │
   │   trs_server_send_vendor_command() ◄── you call this (commands)               │
   │   trs_server_get_tx_mtu()    ──► you call this (sizing)                        │
   │                                                                                │
   │   Events fired to your callback:                                               │
   │     TRS_SERVER_EVT_TX_STATUS      → TX channel opened/closed                  │
   │     TRS_SERVER_EVT_RECEIVE_DATA   → incoming data ready to dequeue             │
   │     TRS_SERVER_EVT_CBFC_ENABLED   → CBFC started, credits exchanged           │
   │     TRS_SERVER_EVT_CBFC_CREDIT    → new TX credits granted by Central         │
   │     TRS_SERVER_EVT_VENDOR_CMD     → structured command received                │
   │     TRS_SERVER_EVT_CTRL_STATUS    → CP channel opened/closed                  │
   └────────────────────────────────────────────────────────────────────────────────┘
                                           │
   ┌───────────────────────────────────────▼────────────────────────────────────────┐
   │                    TRS SERVICE LAYER  (DO NOT MODIFY)                          │
   │                    trs.c / trs.h                                               │
   │                                                                                │
   │   GATT attribute table: TX characteristic + CCCD                               │
   │                         RX characteristic                                      │
   │                         CP (Control Point) characteristic + CCCD               │
   │   Raw BLE write handlers, bt_gatt_notify(), bt_conn callbacks                  │
   └────────────────────────────────────────────────────────────────────────────────┘
                                           │ BLE radio
                                 ┌─────────▼──────────┐
                                 │   BLE CENTRAL        │
                                 │  (phone / gateway)   │
                                 └──────────────────────┘

File Ownership
****************************************************

.. code-block:: text

   zephyr/
   │
   ├── subsys/bluetooth/services/trs/      ─── DO NOT MODIFY
   │   ├── trs.c                               GATT service implementation
   │   ├── trs.h                               Service API (UUIDs, callbacks)
   │   ├── trs_server.c                        Profile layer (CBFC, queue, events)
   │   └── trs_server.h                        Profile API (send, receive, events)
   │
   └── samples/bluetooth/peripheral_trp_uart/
       ├── prj.conf                         ── KEEP (add your Kconfig options here)
       ├── CMakeLists.txt                   ── KEEP + ADD your source files here
       │
       └── src/
           ├── main.c                       ── DO NOT MODIFY (calls app_init())
           ├── app_ble.c / app_ble.h        ── DO NOT MODIFY (BT init, advertising)
           ├── app_ble_handler.c / .h       ── DO NOT MODIFY (connection tracking)
           ├── app_ble_policy.c / .h        ── DO NOT MODIFY (security, conn params)
           ├── app_ble_discovery.c / .h     ── DO NOT MODIFY (GATT discovery)
           │
           ├── app_trsps_handler.c / .h     ── REPLACE with your own handler
           │   (current: routes BLE↔UART)      (new: routes BLE↔sensor/actuator)
           │
           ├── app.c / app.h                ── REPLACE with your own app logic
           │   (current: UART ISR + pump)       (new: sensor read + send trigger)
           │
           └── app_your_sensor.c / .h       ── ADD your new files here
               app_your_motor.c / .h

**Rule of thumb:**

- Everything in ``subsys/`` → **untouched, reused as-is**
- ``app_ble*.c`` files → **untouched**, they handle BLE lifecycle generically
- ``app_trsps_handler.c`` and ``app.c`` → **replace** with your application logic
- ``main.c`` → **untouched** (``main()`` only calls ``app_init()``)

Use Case A — Send Sensor Data to the Central
********************************************

Example: periodically read an ADC and send raw bytes to the phone.

APIs needed
===========

.. list-table::
   :widths: 40 25 35
   :header-rows: 1

   * - API
     - Header
     - What it does
   * - ``trs_server_send_data(conn, len, data)``
     - ``trs_server.h``
     - Sends ``len`` bytes as a GATT TX notification. Returns 0 on success.
   * - ``trs_server_get_tx_mtu(conn, &mtu)``
     - ``trs_server.h``
     - Gets the negotiated ATT MTU. Max payload = ``mtu - 4``.
   * - ``trs_server_cb_register(&cb)``
     - ``trs_server.h``
     - Registers your event callback to know when the TX channel is open.
   * - ``app_ble_handler_conn_ref_get(&conn)``
     - ``app_ble_handler.h``
     - Gets a ref-counted pointer to the active connection. Must call
       ``bt_conn_unref(conn)`` after use.

Event to watch
==============

.. code-block:: text

   TRS_SERVER_EVT_TX_STATUS  with  status == TRS_SERVER_STATUS_TX_OPENED

This fires when the Central enables TX notifications (CCCD write). You must not
call ``trs_server_send_data()`` before this event or it returns ``-ENOTSUP``.

Code template
=============

.. code-block:: c

   /* app_sensor.c — example sensor data sender */

   #include <zephyr/kernel.h>
   #include <zephyr/bluetooth/conn.h>
   #include <zephyr/bluetooth/services/trs/trs_server.h>
   #include "app_ble_handler.h"

   static bool tx_channel_open;
   static struct k_work_delayable sensor_send_work;

   /* Replace with your real sensor read */
   static void read_sensor(uint8_t *buf, uint16_t *len)
   {
       /* e.g. read ADC, fill buf with raw bytes */
       buf[0] = 0xAA;   /* placeholder */
       *len   = 1U;
   }

   static void sensor_send_work_handler(struct k_work *work)
   {
       struct bt_conn *conn = NULL;
       uint16_t mtu        = 0U;
       uint16_t max_payload;
       uint8_t  buf[244];
       uint16_t len = 0U;
       int      err;

       if (!tx_channel_open) {
           return;  /* Central not subscribed yet */
       }

       err = app_ble_handler_conn_ref_get(&conn);
       if (err != 0) {
           return;  /* No active connection */
       }

       trs_server_get_tx_mtu(conn, &mtu);
       if (mtu <= 4U) {
           goto out;  /* MTU not yet negotiated, retry later */
       }

       max_payload = mtu - 4U;
       read_sensor(buf, &len);

       if (len > max_payload) {
           len = max_payload;  /* clamp — or segment if needed */
       }

       err = trs_server_send_data(conn, len, buf);
       if (err == -ENOBUFS) {
           /* No CBFC TX credits — schedule a retry */
           k_work_reschedule(&sensor_send_work, K_MSEC(10));
       }

   out:
       bt_conn_unref(conn);

       /* Reschedule for periodic send (e.g. 100 ms) */
       k_work_reschedule(&sensor_send_work, K_MSEC(100));
   }

   /* Called from your TRS event handler when TX channel opens */
   void app_sensor_on_tx_channel_ready(void)
   {
       tx_channel_open = true;
       k_work_reschedule(&sensor_send_work, K_NO_WAIT);
   }

   /* Called from your TRS event handler on disconnect */
   void app_sensor_on_disconnect(void)
   {
       tx_channel_open = false;
       k_work_cancel_delayable(&sensor_send_work);
   }

   void app_sensor_init(void)
   {
       k_work_init_delayable(&sensor_send_work, sensor_send_work_handler);
   }

Use Case B — Receive Commands from the Central
**********************************************

Example: Central sends motor speed and direction; board sets PWM accordingly.

Two sub-options
===============

Option 1 — Raw bytes via RX characteristic (simple data pipe)
-------------------------------------------------------------

The Central writes raw bytes to the RX characteristic. Your handler receives them
via ``TRS_SERVER_EVT_RECEIVE_DATA`` and interprets the byte stream however you
define.

Option 2 — Vendor commands via CP characteristic (structured commands)
----------------------------------------------------------------------

The TRS server already decodes opcodes on the Control Point characteristic. Any
opcode ``>= CONFIG_TRS_SERVER_VENDOR_OPCODE_MIN`` (default ``0x20``) is delivered
as ``TRS_SERVER_EVT_VENDOR_CMD``. This is cleaner for command-response use cases.

APIs needed
===========

.. list-table::
   :widths: 45 25 30
   :header-rows: 1

   * - API
     - Header
     - What it does
   * - ``trs_server_cb_register(&cb)``
     - ``trs_server.h``
     - Registers your event handler.
   * - ``trs_server_get_data_len(conn, &len)``
     - ``trs_server.h``
     - Peeks at the byte count of the next queued RX packet.
   * - ``trs_server_get_data(conn, buf)``
     - ``trs_server.h``
     - Dequeues one RX packet into ``buf``; also returns CBFC credits.
   * - ``trs_server_send_vendor_command(conn, id, len, payload)``
     - ``trs_server.h``
     - Sends a vendor command response back over CP notify.

Events to watch
===============

.. code-block:: text

   TRS_SERVER_EVT_RECEIVE_DATA   → raw data arrived (RX characteristic)
   TRS_SERVER_EVT_VENDOR_CMD     → structured command arrived (CP characteristic)

Code template
=============

.. code-block:: c

   /* app_motor.c — example motor control command receiver */

   #include <zephyr/kernel.h>
   #include <zephyr/bluetooth/conn.h>
   #include <zephyr/bluetooth/services/trs/trs_server.h>

   /* Define your command opcode layout — must be >= TRS_SERVER_VENDOR_OPCODE_MIN (0x20) */
   #define CMD_SET_SPEED     0x20U   /* payload[0] = speed 0-100 */
   #define CMD_SET_DIRECTION 0x21U   /* payload[0] = 0 (fwd) / 1 (rev) */
   #define CMD_STATUS_RESP   0x22U   /* response opcode */

   /* Work queue item so we don't process BLE data in the BT RX thread */
   static struct k_work motor_rx_work;
   static struct bt_conn *motor_pending_conn;

   static void motor_apply_speed(uint8_t speed_pct)
   {
       /* TODO: call your PWM / motor driver API */
   }

   static void motor_apply_direction(uint8_t direction)
   {
       /* TODO: set direction GPIO */
   }

   /* Option 1: raw RX characteristic handler */
   static void drain_rx_queue(struct bt_conn *conn)
   {
       uint16_t data_len;
       uint8_t  buf[244];

       while (true) {
           trs_server_get_data_len(conn, &data_len);
           if (data_len == 0U) {
               break;
           }

           if (trs_server_get_data(conn, buf) <= 0) {
               break;
           }

           /* Interpret your own byte protocol here */
           /* e.g. buf[0] = command, buf[1..] = parameters */
           if (buf[0] == 0x01U) {
               motor_apply_speed(buf[1]);
           }
       }
   }

   static void motor_rx_work_handler(struct k_work *work)
   {
       ARG_UNUSED(work);
       if (motor_pending_conn != NULL) {
           drain_rx_queue(motor_pending_conn);
           bt_conn_unref(motor_pending_conn);
           motor_pending_conn = NULL;
       }
   }

   /* Option 2: vendor command CP handler (already decoded by trs_server) */
   static void handle_vendor_cmd(struct trs_server_event *evt)
   {
       struct bt_conn *conn    = evt->event_field.conn;
       uint8_t        *payload = evt->event_field.vendor_cmd.payload;
       uint16_t        length  = evt->event_field.vendor_cmd.length;
       uint8_t         status  = 0U;

       if (length < 1U || payload == NULL) {
           return;
       }

       switch (payload[0]) {
       case CMD_SET_SPEED:
           if (length >= 2U) {
               motor_apply_speed(payload[1]);
           }
           break;

       case CMD_SET_DIRECTION:
           if (length >= 2U) {
               motor_apply_direction(payload[1]);
           }
           break;

       default:
           status = 0xFFU;  /* unknown command */
           break;
       }

       /* Send acknowledgement back to Central via CP notify */
       trs_server_send_vendor_command(conn, CMD_STATUS_RESP, 1U, &status);
   }

   /* Unified TRS event handler */
   static void trs_event_handler(struct trs_server_event *evt)
   {
       if (evt == NULL) {
           return;
       }

       switch (evt->event_id) {

       case TRS_SERVER_EVT_RECEIVE_DATA:
           /* Defer to work queue — must not block BT RX thread */
           if (motor_pending_conn != NULL) {
               bt_conn_unref(motor_pending_conn);
           }
           motor_pending_conn = bt_conn_ref(evt->event_field.conn);
           k_work_submit(&motor_rx_work);
           break;

       case TRS_SERVER_EVT_VENDOR_CMD:
           handle_vendor_cmd(evt);
           break;

       case TRS_SERVER_EVT_TX_STATUS:
           if (evt->event_field.tx_status.status == TRS_SERVER_STATUS_TX_OPENED) {
               /* Central is ready to receive — start your sensor send pump here */
               app_sensor_on_tx_channel_ready();
           } else {
               app_sensor_on_disconnect();
           }
           break;

       case TRS_SERVER_EVT_CBFC_CREDIT:
           /* Central gave TX credits — resume sending if you were blocked */
           app_sensor_on_tx_channel_ready();
           break;

       case TRS_SERVER_EVT_ERR_NO_MEM:
           /* RX queue full — drop or handle gracefully */
           break;

       default:
           break;
       }
   }

   static struct trs_server_cb motor_trs_cb = {
       .trs_server_event = trs_event_handler,
   };

   void app_motor_init(void)
   {
       k_work_init(&motor_rx_work, motor_rx_work_handler);
       trs_server_cb_register(&motor_trs_cb);
   }

How to Wire It into app_init()
*******************************

Replace ``app_trsps_handler_init()`` in ``app.c`` with calls to your own modules.
The minimum required skeleton for ``app.c`` is:

.. code-block:: c

   /* app.c — minimal skeleton for sensor / motor application */

   #include "app_ble.h"
   #include "app_ble_handler.h"
   #include "app_sensor.h"   /* your new file */
   #include "app_motor.h"    /* your new file */

   int app_init(void)
   {
       int err;

       /* 1. Initialise your hardware (sensor, motor driver, etc.) */
       app_sensor_init();
       app_motor_init();           /* registers trs_server_cb internally */

       /* 2. Register BLE connection state callback (keep as-is) */
       err = app_ble_handler_init(app_ble_state_changed, NULL);
       if (err != 0) {
           return err;
       }

       /* 3. Start BLE (advertising begins in app_ble_ready_cb) */
       err = app_ble_init();
       if (err != 0) {
           return err;
       }

       return 0;
   }

   /* Connection state changes — keep advertising restart logic */
   static void app_ble_state_changed(bool connected, uint8_t reason, void *ctx)
   {
       ARG_UNUSED(ctx);

       if (!connected) {
           app_sensor_on_disconnect();
           app_ble_adv_restart();  /* restart advertising after disconnect */
       }
   }

Then update ``CMakeLists.txt`` to add your files:

.. code-block:: cmake

   target_sources(app PRIVATE
       src/main.c
       src/app.c
       src/app_ble.c
       src/app_ble_handler.c
       src/app_ble_policy.c
       src/app_ble_discovery.c
       src/app_sensor.c          # ADD
       src/app_motor.c           # ADD
       # app_trsps_handler.c is removed — replaced by your modules
   )

Event-to-API Reference Card
****************************

.. code-block:: text

   TRS_SERVER_EVT_TX_STATUS (status=TX_OPENED)
       └─► Start sending: call trs_server_send_data() from a k_work handler
                          to avoid blocking the BT RX thread.

   TRS_SERVER_EVT_TX_STATUS (status=TX_DISABLED)
       └─► Stop sending: cancel your delayable work, discard pending data.

   TRS_SERVER_EVT_CBFC_CREDIT
       └─► Resume sending: resubmit your send work if it was blocked by -ENOBUFS.

   TRS_SERVER_EVT_RECEIVE_DATA
       └─► Data ready: call trs_server_get_data_len() then trs_server_get_data()
                       from a k_work handler (not from this callback directly).

   TRS_SERVER_EVT_VENDOR_CMD
       └─► Structured command: evt->event_field.vendor_cmd.payload and .length.
           Call trs_server_send_vendor_command() to acknowledge.

   TRS_SERVER_EVT_CBFC_ENABLED
       └─► CBFC negotiated: both directions can now use flow-controlled writes.
           Informational — no action required unless you track this explicitly.

   TRS_SERVER_EVT_ERR_NO_MEM
       └─► RX queue full: the incoming packet was dropped. Log and handle.

Sizing Guide
************

.. list-table::
   :widths: 35 35 30
   :header-rows: 1

   * - Parameter
     - Formula
     - Example (MTU=247)
   * - Max single send payload
     - ``mtu - 4`` bytes
     - 243 bytes
   * - RX queue depth
     - ``CONFIG_TRS_SERVER_CREDIT`` packets
     - 16 packets
   * - Max RX packet size
     - ``CONFIG_TRS_SERVER_BUF_DATA_SIZE`` bytes
     - 244 bytes
   * - Total RX buffer memory
     - ``CREDIT × BUF_DATA_SIZE``
     - 16 × 244 = 3904 bytes

If your sensor produces frames larger than ``mtu - 4``, you must **segment** at
the application layer before calling ``trs_server_send_data()``. The TRS service
does not perform fragmentation.

If you expect high RX throughput, increase ``CONFIG_TRS_SERVER_CREDIT`` and
``CONFIG_TRS_SERVER_BUF_POOL_SIZE`` in ``prj.conf``. Both must stay equal.

Constraints to Respect
***********************

.. list-table::
   :widths: 50 50
   :header-rows: 1

   * - Constraint
     - Reason
   * - Never call ``trs_server_send_data()`` from the TRS event callback.
     - The callback runs in the BT RX thread. Sending requires stack resources
       that may deadlock if called re-entrantly. Always use ``k_work_submit()``.
   * - Never call ``trs_server_get_data()`` from the TRS event callback.
     - Same reason — defer to a work queue.
   * - Check ``TRS_SERVER_EVT_TX_STATUS`` before sending.
     - Returns ``-ENOTSUP`` if the Central has not enabled TX notifications.
   * - Handle ``-ENOBUFS`` from ``trs_server_send_data()``.
     - CBFC TX credits exhausted — retry after ``TRS_SERVER_EVT_CBFC_CREDIT``.
   * - Keep vendor command opcodes ``>= 0x20``.
     - Opcodes below ``CONFIG_TRS_SERVER_VENDOR_OPCODE_MIN`` are reserved for
       CBFC internal use.
   * - Payload must fit in ``mtu - 4`` bytes.
     - ``trs_server_send_data()`` returns ``-EINVAL`` if exceeded.
   * - Call ``bt_conn_unref(conn)`` after ``app_ble_handler_conn_ref_get()``.
     - Every successful ``conn_ref_get`` increments the reference count.
       Leaking refs eventually causes ``k_panic``.
