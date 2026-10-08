/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TRANSPARENT_SERVICE_H_
#define TRANSPARENT_SERVICE_H_

/**
 * @brief Transparent Service (TRS)
 */
#include <stdint.h>
#include <stdbool.h>
#include <zephyr/sys/slist.h>
#include <zephyr/bluetooth/uuid.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 128-bit UUIDs for Service and Characteristics */
/**
 *  @brief Transparent Service UUID value
 */
#define BT_UUID_TRS_VAL 0x55,0xE4,0x05,0xD2,0xAF,0x9F,0xA9,0x8F,\
                        0xE5,0x4A,0x7D,0xFE,0x43,0x53,0x53,0x49
/**
 *  @brief TRS Characteristic Tx UUID value
 */
#define BT_UUID_TX_CHAR_VAL 0x16,0x96,0x24,0x47,0xC6,0x23,0x61,0xBA,\
                            0xD9,0x4B,0x4D,0x1E,0x43,0x53,0x53,0x49
/**
 *  @brief TRS Characteristic Rx UUID value
 */
#define BT_UUID_RX_CHAR_VAL 0xB3,0x9B,0x72,0x34,0xBE,0xEC,0xD4,0xA8,\
                            0xF4,0x43,0x41,0x88,0x43,0x53,0x53,0x49
/**
 *  @brief TRS Characteristic Control Point UUID value
 */
#define BT_UUID_CP_CHAR_VAL 0x7E,0x3B,0x07,0xFF,0x1C,0x51,0x49,0x2F,\
                            0xB3,0x39,0x8A,0x4C,0x43,0x53,0x53,0x49

/* H3 parity-style attribute handles used by bt_trs_permission_config(). */
#define BT_TRS_START_HDL 0x00A0U

enum bt_trs_attribute_handle {
    /* Primary TRS service declaration handle. */
    BT_TRS_HDL_SVC = BT_TRS_START_HDL,
    /* TX characteristic declaration handle. */
    BT_TRS_HDL_CHAR_TX,
    /* TX characteristic value handle. */
    BT_TRS_HDL_CHARVAL_TX,
    /* TX CCCD handle. */
    BT_TRS_HDL_CCCD_TX,
    /* RX characteristic declaration handle. */
    BT_TRS_HDL_CHAR_RX,
    /* RX characteristic value handle. */
    BT_TRS_HDL_CHARVAL_RX,
    /* Control Point characteristic declaration handle. */
    BT_TRS_HDL_CHAR_CTRL,
    /* Control Point characteristic value handle. */
    BT_TRS_HDL_CHARVAL_CTRL,
    /* Control Point CCCD handle. */
    BT_TRS_HDL_CCCD_CTRL,
};

#define BT_TRS_END_HDL BT_TRS_HDL_CCCD_CTRL

/* Zephyr-native attribute index map for bt_trs_permission_config_by_index(). */
enum bt_trs_attr_index {
    BT_TRS_ATTR_INDEX_SVC = 0,
    BT_TRS_ATTR_INDEX_CHAR_TX,
    BT_TRS_ATTR_INDEX_CHARVAL_TX,
    BT_TRS_ATTR_INDEX_CCCD_TX,
    BT_TRS_ATTR_INDEX_CHAR_RX,
    BT_TRS_ATTR_INDEX_CHARVAL_RX,
    BT_TRS_ATTR_INDEX_CHAR_CTRL,
    BT_TRS_ATTR_INDEX_CHARVAL_CTRL,
    BT_TRS_ATTR_INDEX_CCCD_CTRL,
    BT_TRS_ATTR_INDEX_COUNT,
};

/** @brief Transparent service callback structure */
struct bt_trs_cb {
    /**
     * @brief BLE→UART data write callback (Central writes to RX characteristic).
     *  Called when the BLE Central sends data destined for the UART TX pin.
     *
     * @param conn    Connection object.
     * @param data    Pointer to the received payload.
     * @param length  Payload length in bytes.
     * @param flags   GATT attribute write flags.
     *
     * @return 0 on successful handling.
     * @return -ENOTSUP to pass handling to the next registered listener.
     * @return Other negative error codes cause an ATT application error response.
     */
    int (*rx_write)(struct bt_conn *conn, uint8_t* data, uint16_t length, uint8_t flags);

    /**
     * @brief Control-point write callback (Central writes CBFC opcodes or vendor commands).
     *  Direction: Central→DUT; not a data bridge path.
     *
     * @param conn    Connection object.
     * @param data    Pointer to the control-point payload.
     * @param length  Payload length in bytes.
     * @param flags   GATT attribute write flags.
     *
     * @return 0 on successful handling.
     * @return -ENOTSUP to pass handling to the next registered listener.
     * @return Other negative error codes cause an ATT application error response.
     */
    int (*cp_write)(struct bt_conn *conn, uint8_t* data, uint16_t length, uint8_t flags);

    /**
     * @brief TX characteristic write callback (Central writes to server TX value).
     *  Direction is uncertain — TX is primarily a server→Central notify path (UART→BLE);
     *  Central writing here is non-standard but permitted by characteristic properties.
     *
     * @param conn    Connection object.
     * @param data    Pointer to Attribute data.
     * @param length  Attribute value length.
     * @param flags   GATT attribute write flags.
     *
     * @return 0 on successful handling.
     * @return -ENOTSUP to pass handling to the next registered listener.
     * @return Other negative error codes cause an ATT application error response.
     */
    int (*tx_write)(struct bt_conn *conn, uint8_t* data, uint16_t length, uint8_t flags);

    /** @brief UART→BLE TX notification state changed.
     *  Called when the Central enables or disables TX characteristic notifications (CCCD).
     *
     * @param enabled  true when notifications enabled; false when disabled.
     */
    void (*tx_ntf_changed)(bool enabled);

    /** @brief Control-point notification state changed.
     *  Called when the Central enables or disables CP characteristic notifications (CCCD).
     *
     * @param enabled  true when notifications enabled; false when disabled.
     */
    void (*cp_ntf_changed)(bool enabled);

    /** Internal member to form a list of callbacks */
    sys_snode_t _node;
};

/** @brief Notify transmit data.
 *
 * This will send a GATT notification to specific connection.
 *
 *  @param conn Connection object.
 *  @param data Pointer to Attribute data.
 *  @param len  Attribute value length.
 *
 *  @return Zero in case of success and error code in case of error.
 */
int bt_trs_tx_notify(struct bt_conn *conn, uint8_t* data, uint16_t len);

/** @brief Notify controle point data.
 *
 * This will send a GATT notification to specific connection.
 *
 *  @param conn Connection object.
 *  @param data Pointer to Attribute data.
 *  @param len  Attribute value length.
 *
 *  @return Zero in case of success and error code in case of error.
 */
int bt_trs_cp_notify(struct bt_conn *conn, uint8_t* data, uint16_t len);

/** @brief Register Transparent Service in GATT database.
 *
 * This is idempotent and returns success if already registered.
 *
 * @return 0 on success, negative error code on failure.
 */
int bt_trs_add(void);

/** @brief OR-in permissions for a TRS attribute.
 *
 * This provides H3-style runtime permission update semantics.
 *
 * @param attr_hdl One of @ref bt_trs_attribute_handle.
 * @param permissions Zephyr `BT_GATT_PERM_*` mask bits.
 *
 * @return 0 on success, -EINVAL for invalid handle.
 */
int bt_trs_permission_config(uint16_t attr_hdl, uint16_t permissions);

/** @brief OR-in permissions using Zephyr-native TRS attribute index.
 *
 * Use this in new Zephyr-first applications where fixed H3 logical handles are
 * not part of the app contract.
 *
 * @param attr_index One of @ref bt_trs_attr_index values.
 * @param permissions Zephyr `BT_GATT_PERM_*` mask bits.
 *
 * @return 0 on success, -EINVAL for invalid index.
 */
int bt_trs_permission_config_by_index(uint16_t attr_index, uint16_t permissions);

/** @brief OR-in permissions using runtime ATT handle.
 *
 * This is useful when your code tracks runtime ATT handles instead of logical
 * handles or fixed attribute indices.
 *
 * @param att_handle Runtime ATT handle assigned by Zephyr.
 * @param permissions Zephyr `BT_GATT_PERM_*` mask bits.
 *
 * @return 0 on success.
 * @return -EINVAL for invalid parameters.
 * @return -ENOENT if handle does not belong to TRS.
 * @return other negative error from registration path.
 */
int bt_trs_permission_config_by_att_handle(uint16_t att_handle, uint16_t permissions);

/** @brief Get runtime ATT handle for a TRS attribute index.
 *
 * @param attr_index One of @ref bt_trs_attr_index values.
 * @param att_handle Output pointer for runtime ATT handle.
 *
 * @return 0 on success, negative error code on failure.
 */
int bt_trs_attr_handle_get(uint16_t attr_index, uint16_t *att_handle);

/** @brief Get the Tx characteristic client configuration status.
 *
 * This will get the Tx characteristic client configuration status of 
 * a specific connection.
 *
 *  @param conn Connection object.
 *
 *  @return true if the attribute object has been subscribed.
 */
bool bt_trs_tx_ccc_config_status(struct bt_conn *conn);

/** @brief Get the CP characteristic client configuration status.
 *
 * This will get the CP characteristic client configuration status of 
 * a specific connection.
 *
 *  @param conn Connection object.
 *
 *  @return true if the attribute object has been subscribed.
 */
bool bt_trs_cp_ccc_config_status(struct bt_conn *conn);

/** @brief Transparent service callback register
 *
 * This function will register callbacks that will be called in
 * certain events related to Transparent Service.
 *
 * @param cb Pointer to callbacks structure. Must point to memory that remains valid
 * until unregistered.
 *
 * @return 0 on success
 * @return -EINVAL in case @p cb is NULL
 */
int bt_trs_cb_register(struct bt_trs_cb *cb);

/** @brief Transparent service callback unregister
 *
 * This function will unregister callback from Transparent service.
 *
 * @param cb Pointer to callbacks structure
 *
 * @return 0 on success
 * @return -EINVAL in case @p cb is NULL
 * @return -ENOENT in case the @p cb was not found in registered callbacks
 */
int bt_trs_cb_unregister(struct bt_trs_cb *cb);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* TRANSPARENT_SERVICE_H_ */
