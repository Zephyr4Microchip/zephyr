/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TRANSPARENT_SERVICE_SERVER_H_
#define TRANSPARENT_SERVICE_SERVER_H_

/**
 * @brief Transparent Service Server
 */
#include <stdint.h>
#include <zephyr/sys/slist.h>

struct bt_conn;

#ifdef __cplusplus
extern "C" {
#endif

enum trs_server_event_id {
    TRS_SERVER_EVT_RECEIVE_DATA= 1,
    TRS_SERVER_EVT_CTRL_STATUS,
    TRS_SERVER_EVT_TX_STATUS,
    TRS_SERVER_EVT_CBFC_ENABLED,
    TRS_SERVER_EVT_CBFC_CREDIT,
    TRS_SERVER_EVT_VENDOR_CMD,
    TRS_SERVER_EVT_ERR_UNSPECIFIED,
    TRS_SERVER_EVT_ERR_NO_MEM,
};

#define TRS_SERVER_STATUS_CTRL_DISABLED 0U
#define TRS_SERVER_STATUS_CTRL_OPENED   1U
#define TRS_SERVER_STATUS_TX_DISABLED   0U
#define TRS_SERVER_STATUS_TX_OPENED     1U

struct trs_server_vendor_cmd{
    uint16_t        length;
    uint8_t        *payload;
};

struct trs_server_status_field {
    struct bt_conn *conn;
    uint8_t status;
};

struct trs_server_event_field{
    struct bt_conn *conn;
    struct trs_server_vendor_cmd vendor_cmd;
    struct trs_server_status_field tx_status;
    struct trs_server_status_field ctrl_status;
};

struct  trs_server_event
{
    enum trs_server_event_id event_id;                 
    struct trs_server_event_field event_field;               
};

/** @brief Transparent service callback structure */
struct trs_server_cb {
    /**
     * @brief Transparent server callback
     *
     * @param event Pointer to the event structure containing details about the event.
     */
    void (*trs_server_event)(struct  trs_server_event *event);


    /** Internal member to form a list of callbacks */
    sys_snode_t _node;
};

/**
 * @brief Transparent service server callback register
 *
 * @param cb pointer to required callback
 *
 * @return 0 on success
 * @return negative error codes on failure
 */
int trs_server_cb_register(struct trs_server_cb *cb);

/** @brief Transparent service server callback unregister
 *
 * @param cb Pointer to callbacks structure
 *
 * @return 0 on success
 * @return -EINVAL in case @p cb is NULL
 * @return -ENOENT in case the @p cb was not found in registered callbacks
 */
int trs_server_cb_unregister(struct trs_server_cb *cb);

/** @brief Retrieves the negotiated ATT MTU used for UART→BLE notifications.
 *
 * @param conn    Connection object.
 * @param tx_mtu  Output pointer where the MTU value will be stored.
 *
 * @return none
 */
void trs_server_get_tx_mtu(struct bt_conn *conn , uint16_t *tx_mtu);

/** @brief Retrieves the byte length of the next queued BLE→UART packet for a connection.
 *
 * @param conn      Connection object.
 * @param data_len  Output pointer where the length will be stored; set to 0 if queue is empty.
 *
 * @return none
 */
void trs_server_get_data_len(struct bt_conn *conn, uint16_t *data_len);

/** @brief Dequeues one BLE→UART packet and copies it into the caller's buffer.
 *  Also returns CBFC credits to the peer when the RX queue slot is freed.
 *
 * @param conn  Connection object.
 * @param data  Caller-allocated buffer sized to the value returned by trs_server_get_data_len().
 *
 * @return Number of bytes copied on success.
 * @return -ENOTCONN if conn is not a tracked connection.
 * @return -ENODATA if the queue is empty.
 */
int trs_server_get_data(struct bt_conn *conn, uint8_t *data);

/** @brief Sends a vendor-specific command over BLE.
 *
 * @param conn Connection object.
 * @param command_id The identifier of the vendor-specific command.
 * @param command_length The length of the command payload.
 * @param command_payload Pointer to the command payload data.
 *
 * @return 0 on success
 * @return -ENOTCONN in case not a valid connection
 * @return -EINVAL Invalid parameters; incorrect commandID, commandLength.
 * @return -ENOTSUP in case the CCCD of TCP is not enabled.
 */
int trs_server_send_vendor_command(struct bt_conn *conn, uint8_t command_id, 
                                   uint16_t command_length, uint8_t *command_payload);

/** @brief Sends a UART→BLE data payload as a GATT notification on the TX characteristic.
 *  Len must not exceed att_mtu - 4 bytes.  Decrements the local CBFC TX credit on success.
 *
 * @param conn  Connection object.
 * @param len   Payload length in bytes; must be <= att_mtu - 4.
 * @param data  Pointer to the payload to notify.
 *
 * @return 0 on success.
 * @return -ENOTCONN  if conn is not a tracked connection.
 * @return -ENOTSUP   if the Central has not enabled TX notifications (CCCD).
 * @return -ENOBUFS   if no CBFC TX credits are available.
 * @return -EINVAL    if len exceeds the MTU payload limit or data is NULL.
 */
int trs_server_send_data(struct bt_conn *conn, uint16_t len, uint8_t *data);


#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* TRANSPARENT_SERVICE_SERVER_H_ */
