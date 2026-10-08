/** @file
 *  @brief GATT Transparent Service
 */

/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/init.h>
#include <zephyr/sys/__assert.h>
#include <stdbool.h>
#include <zephyr/types.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/sys/check.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/bluetooth/services/trs/trs.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(trs);

/**
 * @brief GATT ATT error used for application-level failures.
 */
#define BT_TRS_ATT_ERR_APPLICATION_ERROR 0x80

static sys_slist_t trs_cbs = SYS_SLIST_STATIC_INIT(&trs_cbs);
static bool trs_registered;

static const struct bt_uuid_128 trs_uuid = BT_UUID_INIT_128(BT_UUID_TRS_VAL);
static const struct bt_uuid_128 tx_char_uuid = BT_UUID_INIT_128(BT_UUID_TX_CHAR_VAL);
static const struct bt_uuid_128 rx_char_uuid = BT_UUID_INIT_128(BT_UUID_RX_CHAR_VAL);
static const struct bt_uuid_128 cp_char_uuid = BT_UUID_INIT_128(BT_UUID_CP_CHAR_VAL);

/* GATT write handler for the RX characteristic: Central→DUT (BLE→UART direction) [BT RX thread]. */
static ssize_t ble_to_uart_gatt_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    bool handled = false;
    struct bt_trs_cb *listener;

    ARG_UNUSED(attr);

    /* TRS requires complete writes; partial and long-write (Prepare Write)
     * transactions are not supported by the protocol design. */
    if (offset != 0U) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if ((flags & BT_GATT_WRITE_FLAG_PREPARE) != 0U) {
        return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
    }

    /* -ENOTSUP from a listener means "not handled by me, try the next one".
     * Any other non-zero error maps to ATT application error 0x80. */
    SYS_SLIST_FOR_EACH_CONTAINER(&trs_cbs, listener, _node) {
        if (listener->rx_write != NULL) {
            int err;

            err = listener->rx_write(conn, (uint8_t *)buf, len, flags);
            if ((err != 0) && (err != -ENOTSUP)) {
                LOG_ERR("TRS Rx write cb returned %d", err);
                return BT_GATT_ERR(BT_TRS_ATT_ERR_APPLICATION_ERROR);
            }

            if (err == 0) {
                handled = true;
            }
        }
    }

    if (!handled) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    return len;
}

/* GATT write handler for the Control Point characteristic: Central→DUT (CBFC and vendor commands) [BT RX thread]. */
static ssize_t ctrl_point_gatt_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    bool handled = false;
    struct bt_trs_cb *listener;

    ARG_UNUSED(attr);

    /* Control point also accepts only full writes starting at offset 0. */
    if (offset != 0U) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    /* Control point does not support Prepare Write transactions. */
    if ((flags & BT_GATT_WRITE_FLAG_PREPARE) != 0U) {
        return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
    }

    SYS_SLIST_FOR_EACH_CONTAINER(&trs_cbs, listener, _node) {
        if (listener->cp_write != NULL) {
            int err;

            err = listener->cp_write(conn, (uint8_t *)buf, len, flags);
            if ((err != 0) && (err != -ENOTSUP)) {
                LOG_ERR("TRS CP write cb returned %d", err);
                return BT_GATT_ERR(BT_TRS_ATT_ERR_APPLICATION_ERROR);
            }

            if (err == 0) {
                handled = true;
            }
        }
    }

    if (!handled) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    return len;
}

/* GATT write handler for the TX characteristic: Central writes to server's TX value.
 * Direction is uncertain — TX is primarily a server→Central notify path; Central
 * writing here is non-standard but permitted by characteristic properties. */
static ssize_t server_tx_gatt_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    struct bt_trs_cb *listener;

    ARG_UNUSED(attr);

    /* TX write path follows the same write contract as RX/CP. */
    if (offset != 0U) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if ((flags & BT_GATT_WRITE_FLAG_PREPARE) != 0U) {
        return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
    }

    SYS_SLIST_FOR_EACH_CONTAINER(&trs_cbs, listener, _node) {
        /* Call TX handler only when the listener provides one. */
        if (listener->tx_write != NULL) {
            int err;

            err = listener->tx_write(conn, (uint8_t *)buf, len, flags);
            if ((err != 0) && (err != -ENOTSUP)) {
                LOG_ERR("TRS Tx write cb returned %d", err);
                return BT_GATT_ERR(BT_TRS_ATT_ERR_APPLICATION_ERROR);
            }
        }
    }

    /*
     * H3 parity: TX writes are accepted even when no consumer handles them.
     * The value path is primarily used for notifications from server to client.
     */
    return len;
}

/* Called from BT RX thread when the Central writes the TX CCCD (UART→BLE notification toggle).
 * Triggers sync_status_events() in trs_server to emit TX_STATUS events. */
static void uart_to_ble_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    struct bt_trs_cb *listener;
    /* CCC value tells whether the peer enabled notifications for TX. */
    bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

    ARG_UNUSED(attr);

    LOG_INF("Tx notifications %s", notif_enabled ? "enabled" : "disabled");

    SYS_SLIST_FOR_EACH_CONTAINER(&trs_cbs, listener, _node) {
        if (listener->tx_ntf_changed != NULL) {
            listener->tx_ntf_changed(notif_enabled);
        }
    }
}

/* Called from BT RX thread when the Central writes the CP CCCD (control-point notification toggle). */
static void ctrl_point_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    struct bt_trs_cb *listener;
    /* CCC value tells whether the peer enabled notifications for the control point. */
    bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

    ARG_UNUSED(attr);

    LOG_INF("CP notifications %s", notif_enabled ? "enabled" : "disabled");

    SYS_SLIST_FOR_EACH_CONTAINER(&trs_cbs, listener, _node) {
        if (listener->cp_ntf_changed != NULL) {
            listener->cp_ntf_changed(notif_enabled);
        }
    }
}

static struct bt_gatt_attr trs_attrs[] = {
    BT_GATT_PRIMARY_SERVICE(&trs_uuid),
    /* TX characteristic: server→Central notify (UART→BLE); Central write path is non-standard. */
    BT_GATT_CHARACTERISTIC(&tx_char_uuid.uuid, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_WRITE, NULL, server_tx_gatt_write, NULL),
    BT_GATT_CCC(uart_to_ble_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    /* RX characteristic: Central→server write (BLE→UART direction). */
    BT_GATT_CHARACTERISTIC(&rx_char_uuid.uuid,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE, NULL, ble_to_uart_gatt_write, NULL),
    /* CP characteristic: Central→server control-point commands (CBFC, vendor). */
    BT_GATT_CHARACTERISTIC(&cp_char_uuid.uuid,
                           BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_WRITE |
                               BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_WRITE, NULL, ctrl_point_gatt_write, NULL),
    BT_GATT_CCC(ctrl_point_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
};

/* Compile-time guard: trs_attrs[] size must stay aligned with the H3 logical
 * handle span and the bt_trs_attr_index enum to prevent index/handle mismatch
 * when attributes are added or removed. */
BUILD_ASSERT(ARRAY_SIZE(trs_attrs) == (BT_TRS_END_HDL - BT_TRS_START_HDL + 1),
             "TRS attrs must stay aligned with H3 handle span");
BUILD_ASSERT(ARRAY_SIZE(trs_attrs) == BT_TRS_ATTR_INDEX_COUNT,
             "TRS attrs must stay aligned with bt_trs_attr_index");

static struct bt_gatt_service trs = BT_GATT_SERVICE(trs_attrs);

static int bt_trs_ensure_registered(void)
{
    int rc;

    /* Fast path: service is already in the GATT database. */
    if (trs_registered) {
        return 0;
    }

    /* Try to register now in case this API is used before init timing is complete. */
    rc = bt_trs_add();
    if ((rc != 0) && (rc != -EALREADY)) {
        LOG_WRN("TRS register pending, rc=%d", rc);
        return rc;
    }

    return 0;
}

int bt_trs_add(void)
{
    int rc;

    if (trs_registered) {
        return 0;
    }

    rc = bt_gatt_service_register(&trs);
    if (rc == 0) {
        trs_registered = true;
    }

    return rc;
}

static int bt_trs_permission_apply_by_index(uint16_t attr_index, uint16_t permissions)
{
    if (attr_index >= ARRAY_SIZE(trs_attrs)) {
        return -EINVAL;
    }

    trs_attrs[attr_index].perm |= permissions;
    return 0;
}

int bt_trs_permission_config(uint16_t attr_hdl, uint16_t permissions)
{
    /* Keep H3-like handle range checks before mutating permissions. */
    if ((attr_hdl < BT_TRS_START_HDL) || (attr_hdl > BT_TRS_END_HDL)) {
        return -EINVAL;
    }

    /*
     * This API intentionally uses H3 logical handles, not dynamic ATT handles.
     * It preserves migration compatibility for code that already targets BT_TRS_HDL_*
     * style values.
     */
    return bt_trs_permission_apply_by_index(attr_hdl - BT_TRS_START_HDL,
                                            permissions);
}

int bt_trs_permission_config_by_index(uint16_t attr_index, uint16_t permissions)
{
    return bt_trs_permission_apply_by_index(attr_index, permissions);
}

int bt_trs_permission_config_by_att_handle(uint16_t att_handle, uint16_t permissions)
{
    size_t i;
    int rc;

    if (att_handle == 0U) {
        return -EINVAL;
    }

    rc = bt_trs_ensure_registered();
    if (rc != 0) {
        return rc;
    }

    for (i = 0U; i < ARRAY_SIZE(trs_attrs); i++) {
        if (trs.attrs[i].handle == att_handle) {
            return bt_trs_permission_apply_by_index((uint16_t)i, permissions);
        }
    }

    return -ENOENT;
}

int bt_trs_attr_handle_get(uint16_t attr_index, uint16_t *att_handle)
{
    int rc;

    if (att_handle == NULL) {
        return -EINVAL;
    }

    rc = bt_trs_ensure_registered();
    if (rc != 0) {
        return rc;
    }

    if (attr_index >= ARRAY_SIZE(trs_attrs)) {
        return -EINVAL;
    }

    *att_handle = trs.attrs[attr_index].handle;
    return (*att_handle == 0U) ? -EAGAIN : 0;
}

static int bt_trs_init(void)
{
    (void)bt_trs_add();
    return 0;
}

/* Register the GATT service at boot so it is in the database before any
 * connection can arrive after bt_enable(). */
SYS_INIT(bt_trs_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

int bt_trs_tx_notify(struct bt_conn *conn, uint8_t *data, uint16_t len)
{
    int rc;

    rc = bt_trs_ensure_registered();
    if (rc != 0) {
        return rc;
    }

    if (data == NULL) {
        return -EINVAL;
    }

    /* Send TX characteristic notification to one connection (or all if conn is NULL). */
    rc = bt_gatt_notify(conn, &trs.attrs[BT_TRS_ATTR_INDEX_CHARVAL_TX], data, len);

    /* bt_gatt_notify returns -ENOTCONN when no peer has subscribed (CCC not set).
     * Treat as success: the caller cannot be expected to track CCC state externally. */
    return rc == -ENOTCONN ? 0 : rc;
}

int bt_trs_cp_notify(struct bt_conn *conn, uint8_t *data, uint16_t len)
{
    int rc;

    rc = bt_trs_ensure_registered();
    if (rc != 0) {
        return rc;
    }

    if (data == NULL) {
        return -EINVAL;
    }

    /* Send CP characteristic notification to one connection (or all if conn is NULL). */
    rc = bt_gatt_notify(conn, &trs.attrs[BT_TRS_ATTR_INDEX_CHARVAL_CTRL], data, len);

    return rc == -ENOTCONN ? 0 : rc;
}

bool bt_trs_tx_ccc_config_status(struct bt_conn *conn)
{
    if (bt_trs_ensure_registered() != 0) {
        return false;
    }

    bool is_subscribed = bt_gatt_is_subscribed(
        conn, &trs.attrs[BT_TRS_ATTR_INDEX_CHARVAL_TX], BT_GATT_CCC_NOTIFY);

    return is_subscribed;
}

bool bt_trs_cp_ccc_config_status(struct bt_conn *conn)
{
    if (bt_trs_ensure_registered() != 0) {
        return false;
    }

    bool is_subscribed = bt_gatt_is_subscribed(
        conn, &trs.attrs[BT_TRS_ATTR_INDEX_CHARVAL_CTRL], BT_GATT_CCC_NOTIFY);

    return is_subscribed;
}

int bt_trs_cb_register(struct bt_trs_cb *cb)
{
    int rc;

    CHECKIF(cb == NULL) {
        return -EINVAL;
    }

    rc = bt_trs_ensure_registered();
    if (rc != 0) {
        return rc;
    }

    /* Keep callback in a linked list so multiple modules can listen to TRS events. */
    sys_slist_append(&trs_cbs, &cb->_node);

    return 0;
}

int bt_trs_cb_unregister(struct bt_trs_cb *cb)
{
    CHECKIF(cb == NULL) {
        return -EINVAL;
    }

    if (!sys_slist_find_and_remove(&trs_cbs, &cb->_node)) {
        return -ENOENT;
    }

    return 0;
}

