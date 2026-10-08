/** @file
 *  @brief GATT Transparent Service Server
 */

/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/init.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/check.h>
#include <string.h>

#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>

#include <zephyr/bluetooth/services/trs/trs.h>
#include <zephyr/bluetooth/services/trs/trs_server.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(trs_server);

/*Maximum number of concurrent connections supported by the device. */
#define TRS_SERVER_MAX_CONN_NBR                     CONFIG_BT_MAX_CONN

#define TRS_SERVER_ATT_DEFAULT_MTU                  (23U)

/*credit value for BLE Transparent server*/
#define TRS_SERVER_CREDIT                           CONFIG_TRS_SERVER_CREDIT
/*Maximum number of credits that can be returned to the peer device.*/
#define TRS_SERVER_MAX_RETURN_CREDIT                ((CONFIG_TRS_SERVER_CREDIT) - 3U)

/*Enable flag for CBFC (Credit Based Flow Control) transmission.*/
#define TRS_SERVER_CBFC_TX_ENABLED                  (1U)
/* Enable flag for CBFC reception.*/
#define TRS_SERVER_CBFC_RX_ENABLED                  (1U << 1U)

/*Opcode indicating the server has enabled CBFC.*/
#define TRS_SERVER_OPCODE_CBFC_ENABLED              CONFIG_TRS_SERVER_OPCODE_CBFC_ENABLED
/*Opcode for giving credit in CBFC.*/
#define TRS_SERVER_OPCODE_CBFC_GIVE_CREDIT          CONFIG_TRS_SERVER_OPCODE_CBFC_GIVE_CREDIT
/*Opcode indicating a successful operation in CBFC.*/
#define TRS_SERVER_OPCODE_CBFC_SUCCESS              (0U)

/*Minimum value for vendor-specific opcodes.*/
#define TRS_SERVER_VENDOR_OPCODE_MIN                CONFIG_TRS_SERVER_VENDOR_OPCODE_MIN

#define TRS_SERVER_BUF_POOL_SIZE                    CONFIG_TRS_SERVER_BUF_POOL_SIZE
#define TRS_SERVER_BUF_DATA_SIZE                    CONFIG_TRS_SERVER_BUF_DATA_SIZE

NET_BUF_POOL_DEFINE(trs_buff_pool, TRS_SERVER_BUF_POOL_SIZE, TRS_SERVER_BUF_DATA_SIZE, 0, NULL);

struct trs_packet_list{
    uint8_t   flags;
    struct net_buf *buf;
};

/* Circular receive queue; depth equals TRS_SERVER_CREDIT so each credit slot
 * maps directly to one packet slot.  Overflow is logged and the packet is dropped. */
struct trs_server_queue{
    uint8_t     used_num;
    uint8_t     write_index; 
    uint8_t     read_index; 
    struct trs_packet_list packet_list[TRS_SERVER_CREDIT];
};

struct trs_conn_list
{
    struct bt_conn  *conn;
    uint16_t    att_mtu;
    uint8_t     cbfc_enable;
    /* peer_credit: credits owed back to the peer for RX flow control.
     * local_credit: credits granted by the peer allowing us to send (TX flow control). */
    uint8_t     peer_credit;
    uint8_t     local_credit;
    uint8_t     tx_status;
    uint8_t     ctrl_status;
    struct trs_server_queue recv_pkt_queue;
};


/*An array to keep track of the connection list for BLE Transparent Service.*/
static struct trs_conn_list  conn_list[TRS_SERVER_MAX_CONN_NBR];
static sys_slist_t trs_server_cbs = SYS_SLIST_STATIC_INIT(&trs_server_cbs);

static void event_send(struct trs_server_event *event);

static void reset_recv_queue(struct trs_server_queue *queue)
{
    if (queue == NULL) {
        return;
    }

    while (queue->used_num > 0U) {
        struct trs_packet_list *entry = &queue->packet_list[queue->read_index];

        if (entry->buf != NULL) {
            net_buf_unref(entry->buf);
            entry->buf = NULL;
        }
        entry->flags = 0U;

        queue->read_index++;
        if (queue->read_index >= TRS_SERVER_CREDIT) {
            queue->read_index = 0U;
        }

        queue->used_num--;
    }

    queue->write_index = 0U;
    queue->read_index = 0U;
}

static void reset_conn_slot(int conn_index, bool unref_conn)
{
    if ((conn_index < 0) || (conn_index >= TRS_SERVER_MAX_CONN_NBR)) {
        return;
    }

    /* unref_conn=false on first connect: bt_conn_ref() has not been called yet.
     * unref_conn=true on disconnect: releases the ref taken in connected(). */
    if (unref_conn && (conn_list[conn_index].conn != NULL)) {
        bt_conn_unref(conn_list[conn_index].conn);
    }

    conn_list[conn_index].conn = NULL;
    conn_list[conn_index].att_mtu = TRS_SERVER_ATT_DEFAULT_MTU;
    conn_list[conn_index].cbfc_enable = 0U;
    conn_list[conn_index].peer_credit = 0U;
    conn_list[conn_index].local_credit = 0U;
    conn_list[conn_index].tx_status = TRS_SERVER_STATUS_TX_DISABLED;
    conn_list[conn_index].ctrl_status = TRS_SERVER_STATUS_CTRL_DISABLED;
    reset_recv_queue(&conn_list[conn_index].recv_pkt_queue);
}

static void emit_tx_status_event(struct bt_conn *conn, uint8_t status)
{
    struct trs_server_event event;

    (void)memset(&event, 0, sizeof(event));
    event.event_id = TRS_SERVER_EVT_TX_STATUS;
    event.event_field.conn = conn;
    event.event_field.tx_status.conn = conn;
    event.event_field.tx_status.status = status;
    event_send(&event);
}

static void emit_ctrl_status_event(struct bt_conn *conn, uint8_t status)
{
    struct trs_server_event event;

    (void)memset(&event, 0, sizeof(event));
    event.event_id = TRS_SERVER_EVT_CTRL_STATUS;
    event.event_field.conn = conn;
    event.event_field.ctrl_status.conn = conn;
    event.event_field.ctrl_status.status = status;
    event_send(&event);
}

/* Zephyr GATT CCC callbacks do not carry the connection object, so all active
 * connections must be polled to find which one changed subscription state. */
static void sync_status_events(void)
{
    for (int i = 0; i < TRS_SERVER_MAX_CONN_NBR; i++) {
        uint8_t new_tx_status;
        uint8_t new_ctrl_status;

        if (conn_list[i].conn == NULL) {
            continue;
        }

        new_tx_status = bt_trs_tx_ccc_config_status(conn_list[i].conn)
                            ? TRS_SERVER_STATUS_TX_OPENED
                            : TRS_SERVER_STATUS_TX_DISABLED;
        if (new_tx_status != conn_list[i].tx_status) {
            conn_list[i].tx_status = new_tx_status;
            emit_tx_status_event(conn_list[i].conn, new_tx_status);
        }

        new_ctrl_status = bt_trs_cp_ccc_config_status(conn_list[i].conn)
                              ? TRS_SERVER_STATUS_CTRL_OPENED
                              : TRS_SERVER_STATUS_CTRL_DISABLED;
        if (new_ctrl_status != conn_list[i].ctrl_status) {
            conn_list[i].ctrl_status = new_ctrl_status;
            emit_ctrl_status_event(conn_list[i].conn, new_ctrl_status);
        }
    }
}

/* Called from BT RX thread when the Central writes the TX CCCD (UART→BLE notification toggle). */
static void trs_uart_to_ble_ntf_changed(bool enabled)
{
    ARG_UNUSED(enabled);

    sync_status_events();
}

/* Called from BT RX thread when the Central writes the CP CCCD (control-point notification toggle). */
static void trs_ctrl_point_ntf_changed(bool enabled)
{
    ARG_UNUSED(enabled);

    sync_status_events();
}

/* Blocks a TX send if the peer's incoming credit threshold is reached.
 * Prevents sending while the remote RX queue is full under CBFC flow control. */
static bool check_queued_task(void)
{
    uint8_t i;
    struct bt_conn_info info;
    
    for (i = 0U; i < TRS_SERVER_MAX_CONN_NBR; i++) {
        if (conn_list[i].conn == NULL) {
            continue;
        }

        if ((bt_conn_get_info(conn_list[i].conn, &info) == 0) &&
            (info.state == BT_CONN_STATE_CONNECTED) &&
            (conn_list[i].peer_credit >= TRS_SERVER_MAX_RETURN_CREDIT)) {
            return true;
        }
    }

    return false;
}

static void event_send(struct trs_server_event *event)
{
    struct trs_server_cb *listener;

    if (event == NULL) {
        return;
    }

    SYS_SLIST_FOR_EACH_CONTAINER(&trs_server_cbs, listener, _node){
        if (listener->trs_server_event){
            listener->trs_server_event(event);
        }
    }
}

/* Returns accumulated peer credits via a CP notification.  Credits are batched
 * up to TRS_SERVER_MAX_RETURN_CREDIT to reduce notification traffic. */
static void server_return_credit(int conn_index)
{
    uint8_t notify_data[5U] = {0};
    uint16_t notify_len = 0U;

    if (conn_list[conn_index].peer_credit == 0U)
    {
        LOG_ERR("No credits available");
        return;
    };

    notify_data[notify_len] = TRS_SERVER_OPCODE_CBFC_SUCCESS;
    notify_len ++;
    notify_data[notify_len] = TRS_SERVER_OPCODE_CBFC_ENABLED;
    notify_len ++;
    notify_data[notify_len] = (uint8_t)(conn_list[conn_index].att_mtu>>8);
    notify_len ++;
    notify_data[notify_len] = (uint8_t)(conn_list[conn_index].att_mtu);
    notify_len ++;
    notify_data[notify_len] = conn_list[conn_index].peer_credit;
    notify_len ++;

    if (!bt_trs_cp_notify(conn_list[conn_index].conn, &notify_data[0], notify_len))
    {
        conn_list[conn_index].peer_credit = 0U;
    }
}

static int get_conn_list_index(struct bt_conn *conn)
{
    if (conn == NULL) {
        return -ENOTCONN;
    }

    for(int i=0; i<TRS_SERVER_MAX_CONN_NBR; i++)
    {
        if ((conn_list[i].conn != NULL) && 
            (conn_list[i].conn == conn))
        {
            return i;
        }
    }
    return -ENOTCONN;
}

static int get_free_conn_list_index(void)
{
    for(int i=0; i<TRS_SERVER_MAX_CONN_NBR; i++)
    {
        if (conn_list[i].conn == NULL)
        {
            return i;
        }
    }
    return -ENOTCONN;
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
    if (conn_err) {
        LOG_ERR("Connection failed, err 0x%02x %s", conn_err, bt_hci_err_to_str(conn_err));
    } else {
        int conn_index = get_free_conn_list_index();
        if(conn_index >= 0){
            reset_conn_slot(conn_index, false);
            conn_list[conn_index].conn = bt_conn_ref(conn);
        }
        else{
            struct trs_server_event event;
            event.event_id = TRS_SERVER_EVT_ERR_UNSPECIFIED;
            event.event_field.conn = conn;
            event_send(&event);
        }
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(reason);

    int conn_index = get_conn_list_index(conn);
    if (conn_index >= 0) {
        reset_conn_slot(conn_index, true);
    }
}

static struct bt_conn_cb conn_cbs = {
    .connected = connected,
    .disconnected = disconnected,
};

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
    ARG_UNUSED(rx);

    int conn_index = get_conn_list_index(conn);
    if(conn_index >= 0){
        conn_list[conn_index].att_mtu = tx;
    }
}

static struct bt_gatt_cb gatt_callbacks = {
    .att_mtu_updated = mtu_updated,
};

/* Called from BT RX thread (GATT write callback); must not block. */
/* Receives a BLE→UART data packet written by the Central to the RX characteristic [BT RX thread]. */
static int trs_ble_to_uart_receive(struct bt_conn *conn, uint8_t* data, uint16_t length, uint8_t flags)
{
    int conn_index = get_conn_list_index(conn);

    if(conn_index < 0){
        LOG_ERR("No connection found");
        return conn_index;
    }

    if ((length > 0U) && (data == NULL)) {
        return -EINVAL;
    }

    if (conn_list[conn_index].recv_pkt_queue.used_num < TRS_SERVER_CREDIT)
    {
        struct trs_server_event event;
        /* K_NO_WAIT: blocking allocation is not permitted in GATT callback context. */
        struct net_buf *buf = net_buf_alloc_len(&trs_buff_pool, (size_t)length, K_NO_WAIT);
        if (buf == NULL) {
            LOG_ERR("No memory for TRS Server Rx packet");
            event.event_id = TRS_SERVER_EVT_ERR_NO_MEM;
            event.event_field.conn = conn;
            event_send(&event);
            return -ENOBUFS;
        }
        memcpy(net_buf_add(buf, (size_t)length), data, (size_t)length);
        conn_list[conn_index].recv_pkt_queue.packet_list[conn_list[conn_index].recv_pkt_queue.write_index].flags = flags;
        conn_list[conn_index].recv_pkt_queue.packet_list[conn_list[conn_index].recv_pkt_queue.write_index].buf = buf;
        conn_list[conn_index].recv_pkt_queue.write_index++;
        if (conn_list[conn_index].recv_pkt_queue.write_index >= TRS_SERVER_CREDIT){
            conn_list[conn_index].recv_pkt_queue.write_index = 0;
        }
        conn_list[conn_index].recv_pkt_queue.used_num++;

        event.event_id = TRS_SERVER_EVT_RECEIVE_DATA;
        event.event_field.conn = conn;
        event_send(&event);
    } else {
        LOG_WRN("TRS server RX queue full, dropping packet");
    }

    return 0;
}

/* Receives a control-point command written by the Central (CBFC opcodes, vendor commands) [BT RX thread]. */
static int trs_ctrl_point_receive(struct bt_conn *conn, uint8_t* data, uint16_t length, uint8_t flags)
{
    struct trs_server_event event;
    int conn_index = get_conn_list_index(conn);

    ARG_UNUSED(flags);

    if(conn_index < 0){
        LOG_ERR("No connection found");
        return conn_index;
    }

    if ((length == 0U) || (data == NULL)) {
        return -EINVAL;
    }

    (void)memset((uint8_t *) &event, 0, sizeof(event));
    switch(data[0])
    {
        case TRS_SERVER_OPCODE_CBFC_ENABLED:
        {
            conn_list[conn_index].cbfc_enable |= TRS_SERVER_CBFC_RX_ENABLED;
            conn_list[conn_index].peer_credit = TRS_SERVER_CREDIT;
            server_return_credit(conn_index);
            event.event_id=TRS_SERVER_EVT_CBFC_ENABLED;
            event.event_field.conn = conn;
            event_send(&event);
        }
        break;
        case TRS_SERVER_OPCODE_CBFC_GIVE_CREDIT:
        {
            uint16_t updated_credit;

            if (length < 2U) {
                return -EINVAL;
            }

            conn_list[conn_index].cbfc_enable |= TRS_SERVER_CBFC_TX_ENABLED;
            updated_credit = (uint16_t)conn_list[conn_index].local_credit + data[1];
            conn_list[conn_index].local_credit =
                (updated_credit > UINT8_MAX) ? UINT8_MAX : (uint8_t)updated_credit;
            event.event_id=TRS_SERVER_EVT_CBFC_CREDIT;
            event.event_field.conn = conn;
            event_send(&event);
        }
        break;
        default:
        {
            if (data[0] >= TRS_SERVER_VENDOR_OPCODE_MIN)
            {
                event.event_id=TRS_SERVER_EVT_VENDOR_CMD;
                event.event_field.conn = conn;
                event.event_field.vendor_cmd.length = length;
                event.event_field.vendor_cmd.payload = data;
                event_send(&event);
            }
        }
        break;
    }

    return 0;
}

static struct bt_trs_cb trs_cb = {
    .rx_write       = trs_ble_to_uart_receive,      /* BLE Central → UART TX */
    .cp_write       = trs_ctrl_point_receive,        /* Control-point commands from Central */
    .tx_ntf_changed = trs_uart_to_ble_ntf_changed,  /* UART→BLE TX notification toggle */
    .cp_ntf_changed = trs_ctrl_point_ntf_changed,   /* Control-point notification toggle */
};

static int trs_server_init(void)
{
    int rc;

    bt_conn_cb_register(&conn_cbs);
    bt_gatt_cb_register(&gatt_callbacks);

    rc = bt_trs_cb_register(&trs_cb);
    if (rc != 0) {
        LOG_ERR("Failed to register TRS server callback: %d", rc);
        return rc;
    }

    return 0;
}

void trs_server_get_tx_mtu(struct bt_conn *conn , uint16_t *tx_mtu)
{
    if (tx_mtu == NULL) {
        return;
    }

    int conn_index = get_conn_list_index(conn);
    *tx_mtu = 0U;

    if (conn_index >= 0){
        *tx_mtu = conn_list[conn_index].att_mtu;
    }
}

void trs_server_get_data_len(struct bt_conn *conn, uint16_t *data_len)
{
    struct net_buf *buf;
    int conn_index = get_conn_list_index(conn);

    if (data_len == NULL) {
        return;
    }

    *data_len = 0U;

    if (conn_index < 0) {
        return;
    }
    
    if (conn_list[conn_index].recv_pkt_queue.used_num > 0U) {
        buf = conn_list[conn_index]
                  .recv_pkt_queue
                  .packet_list[conn_list[conn_index].recv_pkt_queue.read_index]
                  .buf;
        if (buf != NULL) {
            *data_len = buf->len;
        }
    }
}

int trs_server_get_data(struct bt_conn *conn, uint8_t *data)
{
    int ret = -ENODATA;
    uint8_t writeType = 0U;
    struct trs_packet_list *packet;
    struct net_buf *buf;
    int conn_index = get_conn_list_index(conn);

    if(conn_index < 0){
        return conn_index;
    }

    if (data == NULL) {
        return -EINVAL;
    }

    if (conn_list[conn_index].recv_pkt_queue.used_num > 0U)
    {
        packet = &conn_list[conn_index]
                      .recv_pkt_queue
                      .packet_list[conn_list[conn_index].recv_pkt_queue.read_index];
        buf = packet->buf;

        if ((buf != NULL) && (buf->data != NULL) && (buf->len > 0U))
        {
            ret = (int)(buf->len);
            (void)memcpy(data, buf->data, buf->len);
        }

        if (buf != NULL) {
            net_buf_unref(buf);
        }
        packet->buf = NULL;
        
        writeType = packet->flags;
        packet->flags = 0U;

        conn_list[conn_index].recv_pkt_queue.read_index++;
        if (conn_list[conn_index].recv_pkt_queue.read_index >= TRS_SERVER_CREDIT)
        {
            conn_list[conn_index].recv_pkt_queue.read_index = 0;
        }

        conn_list[conn_index].recv_pkt_queue.used_num --;
        
        /* Under CBFC: a Write Without Response (CMD flag) consumes one peer
         * credit.  Return accumulated credits when the threshold is reached. */
        if ((conn_list[conn_index].cbfc_enable & TRS_SERVER_CBFC_RX_ENABLED) && (writeType & BT_GATT_WRITE_FLAG_CMD))
        {
            conn_list[conn_index].peer_credit++;
            if (conn_list[conn_index].peer_credit >= TRS_SERVER_MAX_RETURN_CREDIT)
            {
                server_return_credit(conn_index);
            }
        }
    }
    
    return ret;
}

int trs_server_send_vendor_command(struct bt_conn *conn, uint8_t command_id, uint16_t command_length, uint8_t *command_payload)
{
    uint16_t max_payload;
    uint8_t send_buf[TRS_SERVER_BUF_DATA_SIZE] = {0};

    int conn_index = get_conn_list_index(conn);
    if(conn_index < 0){
        LOG_ERR("No connection found");
        return conn_index;
    }

    if(bt_trs_cp_ccc_config_status(conn) == false){
        LOG_ERR("Control Point notification not enabled");
        return -ENOTSUP;
    }

    if (command_id < TRS_SERVER_VENDOR_OPCODE_MIN){
        LOG_ERR("Not a valid command ID");
        return -EINVAL;
    }

    if (conn_list[conn_index].att_mtu <= 4U) {
        LOG_ERR("Invalid ATT MTU");
        return -EINVAL;
    }

    max_payload = conn_list[conn_index].att_mtu - 4U;

    if (command_length > max_payload){
        LOG_ERR("Data length is more than expected");
        return -EINVAL;
    }

    if ((command_length + 1U) > sizeof(send_buf)) {
        LOG_ERR("Command too large for configured TRS buffer");
        return -EINVAL;
    }

    if(command_length > 0U && command_payload == NULL){
        LOG_ERR("Command payload is NULL");
        return -ENODATA;
    }


    send_buf[0] = command_id;
    if(command_length > 0U){
        (void)memcpy(&send_buf[1], command_payload, command_length);
    }

    return bt_trs_cp_notify(conn_list[conn_index].conn, &send_buf[0],
                            (uint16_t)(command_length + 1U));
}

int trs_server_send_data(struct bt_conn *conn, uint16_t len, uint8_t *data)
{
    uint16_t max_payload;
    int result = 0;

    int conn_index = get_conn_list_index(conn);
    if(conn_index < 0){
        LOG_ERR("No connection found");
        return conn_index;
    }

    if(bt_trs_tx_ccc_config_status(conn) == false){
        LOG_ERR("Tx notification not enabled");
        return -ENOTSUP;
    }

    if(((conn_list[conn_index].cbfc_enable&TRS_SERVER_CBFC_TX_ENABLED)!=0U) 
        && (conn_list[conn_index].local_credit == 0U)){
        LOG_ERR("No credits available");
        return -ENOBUFS;
    }

    if (check_queued_task()){
        LOG_ERR("More packets in the queue to process");
        return -ENOBUFS;
    }

    if ((len > 0U) && (data == NULL)) {
        LOG_ERR("Data payload is NULL");
        return -ENODATA;
    }

    if (conn_list[conn_index].att_mtu <= 4U) {
        LOG_ERR("Invalid ATT MTU");
        return -EINVAL;
    }

    max_payload = conn_list[conn_index].att_mtu - 4U;

    if(len > max_payload){
        LOG_ERR("Data length is more than expected");
        return -EINVAL;
    }

    if (len > TRS_SERVER_BUF_DATA_SIZE) {
        LOG_ERR("Data too large for configured TRS buffer");
        return -EINVAL;
    }

    result = bt_trs_tx_notify(conn_list[conn_index].conn, data, len);
    if(!result){
        /* Deduct credit only after a confirmed successful send to avoid
         * under-counting when the notify call fails transiently. */
        if ((conn_list[conn_index].cbfc_enable&TRS_SERVER_CBFC_TX_ENABLED) != 0U){
            conn_list[conn_index].local_credit--;
        }
    }

    return result;
}

int trs_server_cb_register(struct trs_server_cb *cb)
{
    CHECKIF(cb == NULL) {
        return -EINVAL;
    }

    sys_slist_append(&trs_server_cbs, &cb->_node);

    return 0;
}

int trs_server_cb_unregister(struct trs_server_cb *cb)
{
    CHECKIF(cb == NULL){
        return -EINVAL;
    }

    if (!sys_slist_find_and_remove(&trs_server_cbs, &cb->_node)) {
        return -ENOENT;
    }

    return 0;
}

/* Runs before main() at APPLICATION init level, ensuring connection and GATT
 * callbacks are registered before bt_enable() is called from app_init(). */
SYS_INIT(trs_server_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
