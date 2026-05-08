/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/net/phy.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/net/mii.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include <zephyr/types.h>
#include "eth.h"

LOG_MODULE_REGISTER(eth_mchp_gmac_g1, CONFIG_ETHERNET_LOG_LEVEL);

#define DT_DRV_COMPAT microchip_gmac_g1_eth

#define MCHP_OUI_B0 0x00
#define MCHP_OUI_B1 0x04
#define MCHP_OUI_B2 0xA3

#define GMAC_PHY_CONN_TYPE_MII  0
#define GMAC_PHY_CONN_TYPE_RMII 1

#define GMAC_MTU            NET_ETH_MTU
#define GMAC_FRAME_SIZE_MAX (GMAC_MTU + 18)

/* Memory alignment of the RX/TX Buffer Descriptor List */
#define GMAC_DESC_ALIGNMENT 4

/* Total number of queues supported by GMAC hardware module */
#define GMAC_QUEUE_NUM          DT_INST_PROP(0, num_queues)
#define GMAC_PRIORITY_QUEUE_NUM (GMAC_QUEUE_NUM - 1)

#if (GMAC_PRIORITY_QUEUE_NUM >= 1)
BUILD_ASSERT(ARRAY_SIZE(GMAC->GMAC_TBQBAPQ) + 1 == GMAC_QUEUE_NUM,
	     "GMAC_QUEUE_NUM doesn't match soc header");
#endif /* (GMAC_PRIORITY_QUEUE_NUM >= 1) */

#define GMAC_ACTIVE_QUEUE_NUM          (GMAC_QUEUE_NUM)
#define GMAC_ACTIVE_PRIORITY_QUEUE_NUM (GMAC_ACTIVE_QUEUE_NUM - 1)
#define MAIN_QUEUE_RX_DESC_COUNT       (CONFIG_ETH_MCHP_BUF_RX_COUNT + 1)
#define MAIN_QUEUE_TX_DESC_COUNT       (CONFIG_NET_BUF_TX_COUNT + 1)

#define PRIORITY_QUEUE1_RX_DESC_COUNT 1
#define PRIORITY_QUEUE1_TX_DESC_COUNT 1

/* Receive buffer descriptor bit field definitions */
#define GMAC_RXW0_OWNERSHIP (0x1u)
#define GMAC_RXW0_WRAP      (0x2u)
#define GMAC_RXW0_ADDR      (0xFFFFFFFCu)

/* Receive frame length including FCS */
#define GMAC_RXW1_LEN               (0x1FFFu)
#define GMAC_RXW1_FCS_STATUS        (0x2000u)
#define GMAC_RXW1_SOF               (0x4000u)
#define GMAC_RXW1_EOF               (0x8000u)
#define GMAC_RXW1_CFI               (0x1u << 16)
#define GMAC_RXW1_VLANPRIORITY      (0x7u << 17)
#define GMAC_RXW1_PRIORITYDETECTED  (0x1u << 20)
#define GMAC_RXW1_VLANDETECTED      (0x1u << 21)
#define GMAC_RXW1_TYPEIDMATCH       (0x3u << 22)
#define GMAC_RXW1_TYPEIDFOUND       (0x1u << 24)
#define GMAC_RXW1_ADDRMATCH         (0x3u << 25)
#define GMAC_RXW1_ADDRFOUND         (0x1u << 27)
#define GMAC_RXW1_UNIHASHMATCH      (0x1u << 29)
#define GMAC_RXW1_MULTIHASHMATCH    (0x1u << 30)
#define GMAC_RXW1_BROADCASTDETECTED (0x1u << 31)

/* Transmit buffer descriptor bit field definitions */
#define GMAC_TXW1_LEN        (0x3FFFu << 0)
#define GMAC_TXW1_LASTBUFFER (0x1u << 15)
#define GMAC_TXW1_NOCRC      (0x1u << 16)
#define GMAC_TXW1_CHKSUMERR  (0x7u << 20)
#define GMAC_TXW1_LATECOLERR (0x1u << 26)
#define GMAC_TXW1_TRANSERR   (0x1u << 27)
#define GMAC_TXW1_RETRYEXC   (0x1u << 29)
#define GMAC_TXW1_WRAP       (0x40000000u)
#define GMAC_TXW1_USED       (0x80000000u)

/* Interrupt Status/Enable/Disable/Mask register bit field definitions */
#define GMAC_INT_RX_ERR_BITS (GMAC_IER_RXUBR_Msk | GMAC_IER_ROVR_Msk)
#define GMAC_INT_TX_ERR_BITS (GMAC_IER_TUR_Msk | GMAC_IER_RLEX_Msk | GMAC_IER_TFC_Msk)
#define GMAC_INT_EN_FLAGS                                                                          \
	(GMAC_IER_RCOMP_Msk | GMAC_INT_RX_ERR_BITS | GMAC_IER_TCOMP_Msk | GMAC_INT_TX_ERR_BITS |   \
	 GMAC_IER_HRESP_Msk)

#define GMAC_DMA_QUEUE_FLAGS (0)

enum queue_idx {
	GMAC_QUE_0, /* Main queue */
};

#if (DT_INST_PROP(0, max_frame_size) == 1518)
#define GMAC_MAX_FRAME_SIZE 0
#elif (DT_INST_PROP(0, max_frame_size) == 1536)
#define GMAC_MAX_FRAME_SIZE GMAC_NCFGR_MAXFS_Msk
#elif (DT_INST_PROP(0, max_frame_size) == 10240)
#define GMAC_MAX_FRAME_SIZE GMAC_NCFGR_JFRAME_Msk
#else
#error "GMAC_MAX_FRAME_SIZE is invalid, fix it at device tree."
#endif /* (DT_INST_PROP(0, max_frame_size) == 1518) */

#define ETH_MCHP_SUCCESS 0

#define ETH_MCHP_SHIFT_1_BYTE 8
#define ETH_MCHP_SHIFT_2_BYTE 16
#define ETH_MCHP_SHIFT_3_BYTE 24

#define ETH_MCHP_RX_PKT_WAIT_TIMEOUT_MS 500

#define ETH_MCHP_64_BYTE_ALIGNED_SIZE 0x3Fu

#define GMAC_RX_PROCESS_LIMIT 32u

struct gmac_desc {
	uint32_t addr_w0;
	uint32_t status_w1;
};

struct gmac_desc_list {
	struct gmac_desc *buf_desc;
	uint16_t len;
	uint16_t head;
	uint16_t tail;
};

struct frag_buffer {
	uint32_t *buf;
	uint16_t len;
	uint16_t head;
	uint16_t tail;
};

struct gmac_queue {
	struct gmac_desc_list rx_desc_list;
	struct gmac_desc_list tx_desc_list;
	struct k_mutex tx_mutex;
	struct k_sem tx_desc_sem;
	struct net_buf **rx_frag_list;
	struct frag_buffer tx_frag_list;
	volatile uint32_t err_rx_queue_frames_dropped;
	volatile uint32_t err_rx_queue_flushed_count;
	volatile uint32_t err_tx_queue_flushed_count;
	enum queue_idx que_idx;
};

struct gmac_dev_data {
	struct net_if *iface;
	uint8_t mac_addr[6];
	bool link_up;
	struct k_sem rx_int_sem;

	K_KERNEL_STACK_MEMBER(rx_thread_stack, CONFIG_ETH_MCHP_RX_THREAD_STACK_SIZE);
	struct k_thread rx_thread;
	struct gmac_queue *rx_queue;
	struct gmac_queue queue_list[GMAC_QUEUE_NUM];
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	struct net_stats_eth stats;
#endif /* CONFIG_NET_STATISTICS_ETHERNET */
};

struct gmac_clock_config {
	const struct device *clock_dev;
	clock_control_subsys_t mclk_apb_sys;
	clock_control_subsys_t mclk_ahb_sys;
};

struct gmac_dev_config {
	gmac_registers_t *const regs;
	uint32_t active_queues;
	const struct pinctrl_dev_config *pinctrl_cfg;
	void (*config_func)(void);
	const struct device *phy_dev;
	struct gmac_clock_config eth_clock_cfg;
};

struct gmac_frame_info {
	int16_t sof_index;
	int16_t eof_index;
	uint16_t num_descriptors;
};

#define ETH_MCHP_CLOCK_DEFN(n)                                                                     \
	.eth_clock_cfg.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                             \
	.eth_clock_cfg.mclk_apb_sys = (void *)DT_INST_CLOCKS_CELL_BY_NAME(n, mclk_apb, subsystem), \
	.eth_clock_cfg.mclk_ahb_sys = (void *)DT_INST_CLOCKS_CELL_BY_NAME(n, mclk_ahb, subsystem)

#if (CONFIG_NET_BUF_DATA_SIZE * CONFIG_ETH_MCHP_BUF_RX_COUNT) < GMAC_FRAME_SIZE_MAX
#error network data fragment not large enough to hold a full frame
#endif

#if (CONFIG_NET_BUF_DATA_SIZE * (CONFIG_NET_BUF_RX_COUNT - CONFIG_ETH_MCHP_BUF_RX_COUNT)) <        \
	GMAC_FRAME_SIZE_MAX
#error network data fragment are not large enough to hold a full frame
#endif

#if CONFIG_NET_BUF_DATA_SIZE & ETH_MCHP_64_BYTE_ALIGNED_SIZE
#error message "CONFIG_NET_BUF_DATA_SIZE should be a multiple of 64 bytes \
		due to the granularity of RX DMA"
#endif

#if ((CONFIG_ETH_MCHP_BUF_RX_COUNT + 1) * GMAC_ACTIVE_QUEUE_NUM) > CONFIG_NET_BUF_RX_COUNT
#error Not enough RX buffers to allocate descriptors for each HW queue
#endif

BUILD_ASSERT(DT_INST_ENUM_IDX(0, phy_connection_type) <= 1, "Invalid PHY connection");

static struct gmac_desc rx_desc_que0_eth_0[MAIN_QUEUE_RX_DESC_COUNT] __nocache
	__aligned(GMAC_DESC_ALIGNMENT);

static struct gmac_desc tx_desc_que0_eth_0[MAIN_QUEUE_TX_DESC_COUNT] __nocache
	__aligned(GMAC_DESC_ALIGNMENT);

static struct net_buf *rx_frag_list_que0_eth_0[MAIN_QUEUE_RX_DESC_COUNT];
static struct net_buf *tx_frag_list_que0_eth_0[MAIN_QUEUE_TX_DESC_COUNT];

#define MODULO_INC(val, max)                                                                       \
	{                                                                                          \
		val = (++val < max) ? val : 0;                                                     \
	}

#define MODULO_DEC(val, max)                                                                       \
	{                                                                                          \
		val = (--val == -1) ? (max - 1) : val;                                             \
	}

static void frag_buffer_reset(struct frag_buffer *rb)
{
	rb->head = 0U;
	rb->tail = 0U;
}

static uint32_t frag_buffer_get(struct frag_buffer *rb)
{
	uint32_t val = rb->buf[rb->tail];

	__ASSERT(rb->tail != rb->head, "retrieving data from empty ring buffer");
	MODULO_INC(rb->tail, rb->len);
	return val;
}

static void frag_buffer_put(struct frag_buffer *rb, uint32_t val)
{
	rb->buf[rb->head] = val;
	MODULO_INC(rb->head, rb->len);
	__ASSERT(rb->tail != rb->head, "ring buffer overflow");
}

static void gmac_free_rx_bufs(struct net_buf **rx_frag_list, uint16_t len)
{
	for (int i = 0; i < len; i++) {
		if (rx_frag_list[i] != NULL) {
			net_buf_unref(rx_frag_list[i]);
			rx_frag_list[i] = NULL;
		}
	}
}

static int gmac_rx_descriptors_init(gmac_registers_t *gmac_regs, struct gmac_queue *queue)
{
	struct gmac_desc_list *rx_desc_list = &queue->rx_desc_list;
	struct net_buf **rx_frag_list = queue->rx_frag_list;
	struct net_buf *rx_buf;
	uint8_t *rx_buf_addr;

	__ASSERT_NO_MSG(rx_frag_list);
	rx_desc_list->tail = 0U;
	for (int i = 0; i < rx_desc_list->len; i++) {
		rx_buf = net_pkt_get_reserve_rx_data(CONFIG_NET_BUF_DATA_SIZE, K_NO_WAIT);
		if (rx_buf == NULL) {
			gmac_free_rx_bufs(rx_frag_list, i);
			LOG_ERR("Failed to reserve data net buffers");
			return -ENOBUFS;
		}

		rx_frag_list[i] = rx_buf;
		rx_buf_addr = rx_buf->data;
		__ASSERT(((uint32_t)rx_buf_addr & ~GMAC_RXW0_ADDR) == 0,
			 "Misaligned RX buffer address");
		__ASSERT(rx_buf->size == CONFIG_NET_BUF_DATA_SIZE,
			 "Incorrect length of RX data buffer");
		rx_desc_list->buf_desc[i].addr_w0 = (uint32_t)rx_buf_addr & GMAC_RXW0_ADDR;
		rx_desc_list->buf_desc[i].status_w1 = 0U;
	}

	rx_desc_list->buf_desc[rx_desc_list->len - 1U].addr_w0 |= GMAC_RXW0_WRAP;
	gmac_regs->GMAC_RBQB = (uint32_t)queue->rx_desc_list.buf_desc;

	return ETH_MCHP_SUCCESS;
}

static void gmac_tx_descriptors_init(gmac_registers_t *gmac_regs, struct gmac_queue *queue)
{
	struct gmac_desc_list *tx_desc_list = &queue->tx_desc_list;

	tx_desc_list->head = 0U;
	tx_desc_list->tail = 0U;
	for (int i = 0; i < tx_desc_list->len; i++) {
		tx_desc_list->buf_desc[i].addr_w0 = 0U;
		tx_desc_list->buf_desc[i].status_w1 = GMAC_TXW1_USED;
	}

	tx_desc_list->buf_desc[tx_desc_list->len - 1U].status_w1 |= GMAC_TXW1_WRAP;
	frag_buffer_reset(&queue->tx_frag_list);
	gmac_regs->GMAC_TBQB = (uint32_t)queue->tx_desc_list.buf_desc;
}

static int gmac_queue_init(gmac_registers_t *gmac_regs, struct gmac_queue *queue)
{
	int result;

	__ASSERT_NO_MSG(queue->rx_desc_list.len > 0);
	__ASSERT_NO_MSG(queue->tx_desc_list.len > 0);
	__ASSERT(((uint32_t)queue->rx_desc_list.buf_desc & ~GMAC_RBQB_ADDR_Msk) == 0,
		 "RX descriptors have to be word aligned");
	__ASSERT(((uint32_t)queue->tx_desc_list.buf_desc & ~GMAC_TBQB_ADDR_Msk) == 0,
		 "TX descriptors have to be word aligned");

	result = gmac_rx_descriptors_init(gmac_regs, queue);
	if (result < 0) {
		return result;
	}

	gmac_tx_descriptors_init(gmac_regs, queue);
	k_mutex_init(&queue->tx_mutex);
	k_sem_init(&queue->tx_desc_sem, queue->tx_desc_list.len - 1, queue->tx_desc_list.len - 1);
	gmac_regs->GMAC_DCFGR =
		/* Receive Buffer Size (defined in multiples of 64 bytes) */
		GMAC_DCFGR_DRBS(CONFIG_NET_BUF_DATA_SIZE >> 6) | GMAC_DCFGR_RXBMS(3) |
		GMAC_DCFGR_FBLDO_INCR4 | GMAC_DMA_QUEUE_FLAGS;
	gmac_regs->GMAC_IER = GMAC_INT_EN_FLAGS;

	queue->err_rx_queue_frames_dropped = 0U;
	queue->err_rx_queue_flushed_count = 0U;
	queue->err_tx_queue_flushed_count = 0U;

	LOG_INF("Queue %d activated", queue->que_idx);

	return ETH_MCHP_SUCCESS;
}

static void gmac_mac_addr_set(gmac_registers_t *gmac, uint8_t index, uint8_t mac_addr[6])
{
	uint32_t top_addr = (mac_addr[5] << ETH_MCHP_SHIFT_1_BYTE) | (mac_addr[4]);
	uint32_t bottom_addr = (mac_addr[3] << ETH_MCHP_SHIFT_3_BYTE) |
			       (mac_addr[2] << ETH_MCHP_SHIFT_2_BYTE) |
			       (mac_addr[1] << ETH_MCHP_SHIFT_1_BYTE) | (mac_addr[0]);

	__ASSERT(index < 4, "index has to be in the range 0..3");
	gmac->SA[index].GMAC_SAB = GMAC_SAB_ADDR(bottom_addr);
	gmac->SA[index].GMAC_SAT = GMAC_SAT_ADDR(top_addr);
}

static void gmac_tx_completed(gmac_registers_t *gmac, struct gmac_queue *queue)
{
	struct gmac_desc_list *tx_desc_list = &queue->tx_desc_list;
	struct gmac_desc *tx_desc;
	struct net_buf *frag;

	__ASSERT(tx_desc_list->buf[tx_desc_list->tail].w1 & GMAC_TXW1_USED,
		 "first buffer of a frame is not marked as own by GMAC");
	while (tx_desc_list->tail != tx_desc_list->head) {
		tx_desc = &tx_desc_list->buf_desc[tx_desc_list->tail];
		MODULO_INC(tx_desc_list->tail, tx_desc_list->len);
		k_sem_give(&queue->tx_desc_sem);

		frag = UINT_TO_POINTER(frag_buffer_get(&queue->tx_frag_list));
		net_pkt_frag_unref(frag);
		LOG_DBG("Dropping frag %p", frag);

		if (tx_desc->status_w1 & GMAC_TXW1_LASTBUFFER) {
			tx_desc->status_w1 &= ~GMAC_TXW1_LASTBUFFER;
			break;
		}
	}
}

static void gmac_tx_error_handler(gmac_registers_t *gmac, struct gmac_queue *queue)
{
	struct net_buf *frag;
	struct frag_buffer *tx_frag_list = &queue->tx_frag_list;

	gmac->GMAC_NCR &= ~GMAC_NCR_TXEN_Msk;
	queue->err_tx_queue_flushed_count++;

	while (tx_frag_list->tail != tx_frag_list->head) {
		frag = UINT_TO_POINTER(tx_frag_list->buf[tx_frag_list->tail]);
		net_pkt_frag_unref(frag);
		LOG_DBG("Dropping frag %p", frag);
		MODULO_INC(tx_frag_list->tail, tx_frag_list->len);
	}

	/* Reinitialize TX descriptor list */
	k_sem_reset(&queue->tx_desc_sem);
	for (int i = 0; i < queue->tx_desc_list.len - 1; i++) {
		k_sem_give(&queue->tx_desc_sem);
	}

	gmac_tx_descriptors_init(gmac, queue);
	gmac->GMAC_NCR |= GMAC_NCR_TXEN_Msk;
}

static void gmac_rx_error_handler(gmac_registers_t *gmac, struct gmac_queue *queue)
{
	gmac->GMAC_NCR &= ~GMAC_NCR_RXEN_Msk;
	queue->err_rx_queue_flushed_count++;
	queue->rx_desc_list.tail = 0U;

	LOG_DBG("rx Err");

	for (int i = 0; i < queue->rx_desc_list.len; i++) {
		queue->rx_desc_list.buf_desc[i].status_w1 = 0U;
		queue->rx_desc_list.buf_desc[i].addr_w0 &= ~GMAC_RXW0_OWNERSHIP;
	}

	gmac->GMAC_RBQB = (uint32_t)queue->rx_desc_list.buf_desc;
	gmac->GMAC_NCR |= GMAC_NCR_RXEN_Msk;
}

static int gmac_set_phy_connection_type(gmac_registers_t *gmac)
{
	switch (DT_INST_ENUM_IDX(0, phy_connection_type)) {
	case GMAC_PHY_CONN_TYPE_MII:
		gmac->GMAC_UR = 0x1;
		break;
	case GMAC_PHY_CONN_TYPE_RMII:
		gmac->GMAC_UR = 0x0;
		break;
	default:
		LOG_ERR("The phy connection type is invalid");
		return -EINVAL;
	}

	return ETH_MCHP_SUCCESS;
}

static int gmac_init(gmac_registers_t *gmac)
{
	gmac->GMAC_NCFGR |= GMAC_NCFGR_MTIHEN_Msk | GMAC_NCFGR_LFERD_Msk | GMAC_NCFGR_RFCS_Msk |
			    GMAC_NCFGR_RXCOEN_Msk | GMAC_MAX_FRAME_SIZE;

	gmac->GMAC_NCR = GMAC_NCR_CLRSTAT_Msk | GMAC_NCR_MPE_Msk;
	gmac->GMAC_IDR = UINT32_MAX;
	(void)gmac->GMAC_ISR;
	gmac->GMAC_HRB = UINT32_MAX;
	gmac->GMAC_HRT = UINT32_MAX;
	gmac->GMAC_RSR = GMAC_RSR_RESETVALUE;

	return gmac_set_phy_connection_type(gmac);
}

static void gmac_link_configure(gmac_registers_t *gmac, bool full_duplex, bool speed_100M)
{
	uint32_t val = gmac->GMAC_NCFGR;

	val &= ~(GMAC_NCFGR_FD_Msk | GMAC_NCFGR_SPD_Msk);
	val |= (full_duplex) ? GMAC_NCFGR_FD_Msk : 0;
	val |= (speed_100M) ? GMAC_NCFGR_SPD_Msk : 0;
	gmac->GMAC_NCFGR = val;
	gmac->GMAC_NCR |= (GMAC_NCR_RXEN_Msk | GMAC_NCR_TXEN_Msk);
}

static void gmac_queue0_isr(gmac_registers_t *gmac, struct gmac_queue *queue)
{
	struct gmac_desc_list *rx_desc_list = &queue->rx_desc_list;
	struct gmac_desc_list *tx_desc_list = &queue->tx_desc_list;
	struct gmac_desc *tail_desc;
	uint32_t isr = gmac->GMAC_ISR;

	LOG_DBG("GMAC_ISR=0x%08x", isr);

	if ((isr & GMAC_INT_RX_ERR_BITS) != 0) {
		queue->err_rx_queue_flushed_count++;
	} else if (isr & GMAC_ISR_RCOMP_Msk) {
		struct gmac_dev_data *dev_data =
			CONTAINER_OF(queue, struct gmac_dev_data, queue_list[queue->que_idx]);

		tail_desc = &rx_desc_list->buf_desc[rx_desc_list->tail];
		LOG_DBG("rx.w1=0x%08x, tail=%d", tail_desc->status_w1, rx_desc_list->tail);
		dev_data->rx_queue = queue;
		k_sem_give(&dev_data->rx_int_sem);
	} else {
		if ((isr & GMAC_INT_TX_ERR_BITS) != 0) {
			gmac_tx_error_handler(gmac, queue);
		} else if (isr & GMAC_ISR_TCOMP_Msk) {
			tail_desc = &tx_desc_list->buf_desc[tx_desc_list->tail];
			LOG_DBG("tx.w1=0x%08x, tail=%d", tail_desc->status_w1, tx_desc_list->tail);
			gmac_tx_completed(gmac, queue);
		} else {
			/* To avoid Sonar Issue */
		}
	}

	if ((isr & GMAC_IER_HRESP_Msk) != 0) {
		LOG_DBG("IER HRESP");
	}
}

#if DT_INST_NODE_HAS_PROP(0, mac_eeprom)
static void gmac_get_mac_addr_from_i2c_eeprom(uint8_t mac_addr[6])
{
	uint32_t iaddr = CONFIG_ETH_MCHP_MAC_I2C_INT_ADDRESS;
	int ret;
	const struct i2c_dt_spec i2c = I2C_DT_SPEC_GET(DT_INST_PHANDLE(0, mac_eeprom));

	if (device_is_ready(i2c.bus) != true) {
		LOG_ERR("I2C Bus is not ready");
		return;
	}

	ret = i2c_write_read_dt(&i2c, &iaddr, CONFIG_ETH_MCHP_MAC_I2C_INT_ADDRESS_SIZE, mac_addr,
				6);
	if (ret != 0) {
		LOG_ERR("I2C: failed to read MAC addr");
	}
}
#endif /* DT_INST_NODE_HAS_PROP(0, mac_eeprom) */

static void gmac_mac_addr_generate(gmac_registers_t *gmac, uint8_t mac_addr[6])
{
#if DT_INST_NODE_HAS_PROP(0, mac_eeprom)
	gmac_get_mac_addr_from_i2c_eeprom(mac_addr);
#elif DT_INST_PROP(0, zephyr_random_mac_address)
	gen_random_mac(mac_addr, MCHP_OUI_B0, MCHP_OUI_B1, MCHP_OUI_B2);
#endif /* DT_INST_NODE_HAS_PROP(0, mac_eeprom) */
}

static void gmac_generate_set_mac(const struct device *dev, gmac_registers_t *gmac,
				  uint8_t mac_addr[6])
{
	struct gmac_dev_data *const dev_data = dev->data;

	gmac_mac_addr_generate(gmac, mac_addr);
	memcpy(dev_data->mac_addr, mac_addr, sizeof(dev_data->mac_addr));
	gmac_mac_addr_set(gmac, 0, mac_addr);
}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
static void gmac_get_stats(gmac_registers_t *gmac, struct net_stats_eth *eth_stats)
{
	uint32_t err_ae = gmac->GMAC_AE;    /* Alignment Error */
	uint32_t err_ofr = gmac->GMAC_OFR;  /* Over Length Frames */
	uint32_t err_fcs = gmac->GMAC_FCSE; /* FCS Error */
	uint32_t err_scf = gmac->GMAC_SCF;  /* Single Collision */
	uint32_t err_mcf = gmac->GMAC_MCF;  /* Multiple Collision */
	uint32_t err_ecf = gmac->GMAC_EC;   /* Excess Collision */

	eth_stats->collisions += err_scf + err_mcf + err_ecf;
	eth_stats->csum.rx_csum_offload_errors += gmac->GMAC_FCSE;
	eth_stats->csum.rx_csum_offload_good = 0;
	eth_stats->error_details.rx_align_errors += err_ae;
	eth_stats->error_details.rx_long_length_errors += err_ofr;
	eth_stats->error_details.rx_crc_errors += err_fcs;
	eth_stats->error_details.rx_length_errors += err_ofr;
	eth_stats->errors.rx += gmac->GMAC_UCE + gmac->GMAC_TCE + gmac->GMAC_IHCE + gmac->GMAC_ROE +
				gmac->GMAC_RRE + err_ae + gmac->GMAC_RSE + gmac->GMAC_LFFE +
				err_fcs + gmac->GMAC_JR + err_ofr + gmac->GMAC_UFR;
	eth_stats->errors.tx += err_scf + err_mcf + err_ecf + gmac->GMAC_TUR + gmac->GMAC_CSE;
	eth_stats->multicast.rx += gmac->GMAC_MFR;
	eth_stats->multicast.tx += gmac->GMAC_MFT;
	eth_stats->tx_dropped = 0;
	eth_stats->tx_restart_queue = 0;
	eth_stats->tx_timeout_count = 0;
	eth_stats->unknown_protocol = 0;

#ifdef CONFIG_NET_STATISTICS_ETHERNET_VENDOR
	eth_stats->vendor.key = NULL;
	eth_stats->vendor.value = 0;
#endif /* CONFIG_NET_STATISTICS_ETHERNET_VENDOR */
}
#endif /* CONFIG_NET_STATISTICS_ETHERNET */

static int gmac_check_queue_for_data(struct gmac_desc_list *rx_desc_list, struct gmac_desc *rx_desc)
{
	uint16_t tail = rx_desc_list->tail;

	while ((rx_desc->addr_w0 & GMAC_RXW0_OWNERSHIP) == 0) {
		MODULO_INC(tail, rx_desc_list->len);
		rx_desc = &rx_desc_list->buf_desc[tail];
		if (tail == rx_desc_list->tail) {
			return 0;
		}
	}

	rx_desc_list->tail = tail;
	return 1;
}

static int gmac_find_valid_frame(struct gmac_desc_list *rx_desc_list, struct gmac_desc *rx_desc,
				 struct gmac_frame_info *frame_info)
{
	uint16_t tail = rx_desc_list->tail;
	bool sof_found = false;

	frame_info->sof_index = -1;
	frame_info->eof_index = -1;
	frame_info->num_descriptors = 0;
	while (((rx_desc->addr_w0 & GMAC_RXW0_OWNERSHIP) != 0) && (frame_info->eof_index == -1)) {
		if (((uint8_t *)(rx_desc->addr_w0 & GMAC_RXW0_ADDR)) == 0) {
			rx_desc->addr_w0 &= ~GMAC_RXW0_OWNERSHIP;
			break;
		}

		if (rx_desc->status_w1 & GMAC_RXW1_SOF) {
			frame_info->sof_index = tail;
			sof_found = true;
		}

		if (rx_desc->status_w1 & GMAC_RXW1_EOF) {
			frame_info->eof_index = tail;
		}

		if (sof_found) {
			frame_info->num_descriptors++;
		}

		MODULO_INC(tail, rx_desc_list->len);
		rx_desc = &rx_desc_list->buf_desc[tail];
	}

	if ((frame_info->eof_index == -1) || (frame_info->sof_index == -1)) {
		return -EINVAL;
	}

	rx_desc_list->tail = frame_info->sof_index;
	rx_desc = &rx_desc_list->buf_desc[rx_desc_list->tail];
	return 0;
}

static struct net_pkt *gmac_extract_and_replace_buffers(struct gmac_queue *queue, int *errno,
							struct gmac_frame_info *frame_info)
{
	struct gmac_desc_list *rx_desc_list = &queue->rx_desc_list;
	struct net_buf **rx_frag_list = queue->rx_frag_list;
	struct net_pkt *rx_pkt = NULL;
	bool frame_is_complete = false;
	struct net_buf *frag;
	struct net_buf *new_frag;
	struct net_buf *last_frag = NULL;
	uint8_t *frag_data;
	uint32_t frag_len;
	uint32_t frame_len = 0U;
	uint16_t tail = rx_desc_list->tail;
	struct gmac_desc *rx_desc = &rx_desc_list->buf_desc[tail];

	rx_pkt = net_pkt_rx_alloc(K_NO_WAIT);
	if (rx_pkt == NULL) {
		for (int i = rx_desc_list->tail; (frame_is_complete == false);) {
			rx_desc = &rx_desc_list->buf_desc[i];
			frame_is_complete = (bool)(rx_desc->status_w1 & GMAC_RXW1_EOF);
			rx_desc->status_w1 = 0U;
			rx_desc->addr_w0 &= ~GMAC_RXW0_OWNERSHIP;
			MODULO_INC(i, rx_desc_list->len);
			rx_desc = &rx_desc_list->buf_desc[i];
		}

		queue->err_rx_queue_frames_dropped++;
		*errno = -ENOMEM;
		return NULL;
	}

	while (frame_is_complete == false) {
		frag = rx_frag_list[tail];
		frag_data = (uint8_t *)(rx_desc->addr_w0 & GMAC_RXW0_ADDR);
		__ASSERT(frag->data == frag_data, "RX descriptor and buffer list desynchronized");
		if (frame_info->eof_index == tail) {
			frame_is_complete = true;
			frag_len = (rx_desc->status_w1 & GMAC_RXW1_LEN) - frame_len;
		} else {
			frag_len = CONFIG_NET_BUF_DATA_SIZE;
		}

		frame_len += frag_len;
		new_frag = net_pkt_get_frag(rx_pkt, CONFIG_NET_BUF_DATA_SIZE, K_NO_WAIT);
		if (new_frag == NULL) {
			bool eof_flag = false;

			for (int i = rx_desc_list->tail; (eof_flag == false);) {
				rx_desc = &rx_desc_list->buf_desc[i];
				eof_flag = (bool)(rx_desc->status_w1 & GMAC_RXW1_EOF);
				rx_desc->status_w1 = 0U;
				rx_desc->addr_w0 &= ~GMAC_RXW0_OWNERSHIP;
				MODULO_INC(i, rx_desc_list->len);
				rx_desc = &rx_desc_list->buf_desc[i];
			}

			queue->err_rx_queue_frames_dropped++;
			net_pkt_unref(rx_pkt);
			*errno = -ENOMEM;
			return NULL;
		}

		net_buf_add(frag, frag_len);
		if (!last_frag) {
			net_pkt_frag_insert(rx_pkt, frag);
		} else {
			net_buf_frag_insert(last_frag, frag);
		}

		last_frag = frag;
		frag = new_frag;
		rx_frag_list[tail] = frag;
		rx_desc->status_w1 = 0U;
		rx_desc->addr_w0 &= (~GMAC_RXW0_ADDR) & (~GMAC_RXW0_OWNERSHIP);
		rx_desc->addr_w0 |= ((uint32_t)frag->data & GMAC_RXW0_ADDR);
		MODULO_INC(tail, rx_desc_list->len);
		rx_desc = &rx_desc_list->buf_desc[tail];
	}

	rx_desc_list->tail = tail;
	LOG_DBG("Frame complete: rx=%p, tail=%d", rx_pkt, tail);
	__ASSERT_NO_MSG(frame_is_complete);

	return rx_pkt;
}

static struct net_pkt *gmac_pkt_get(gmac_registers_t *gmac_regs, struct gmac_queue *queue,
				    int *errno)
{
	struct gmac_desc_list *rx_desc_list = &queue->rx_desc_list;
	struct net_pkt *rx_pkt = NULL;
	uint16_t tail = rx_desc_list->tail;
	struct gmac_desc *rx_desc = &rx_desc_list->buf_desc[tail];
	struct gmac_frame_info frame_info;

	/* if the packet not available in the first descriptor */
	if (gmac_check_queue_for_data(rx_desc_list, rx_desc) == 0) {
		*errno = -ENODATA;
		return NULL;
	}

	if (gmac_find_valid_frame(rx_desc_list, rx_desc, &frame_info) != 0) {
		gmac_rx_error_handler(gmac_regs, queue);
		*errno = -EINVAL;
		return NULL;
	}

	rx_pkt = gmac_extract_and_replace_buffers(queue, errno, &frame_info);

	return rx_pkt;
}

static void gmac_rx(gmac_registers_t *gmac_regs, struct gmac_queue *queue)
{
	int errno = 0;
	struct gmac_dev_data *dev_data =
		CONTAINER_OF(queue, struct gmac_dev_data, queue_list[queue->que_idx]);
	struct net_pkt *rx_pkt = NULL;
	uint32_t pkts_processed = 0;

	while (pkts_processed < GMAC_RX_PROCESS_LIMIT) {
		rx_pkt = gmac_pkt_get(gmac_regs, queue, &errno);
		if (rx_pkt == NULL) {
			if (errno == -ENODATA) {
				break;
			}

			if (errno == -ENOMEM) {
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
				dev_data->stats.error_details.rx_buf_alloc_failed++;
#endif /* CONFIG_NET_STATISTICS_ETHERNET */
				break;
			}

			if (errno == -EINVAL) {
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
				dev_data->stats.error_details.rx_frame_errors++;
#endif /* CONFIG_NET_STATISTICS_ETHERNET */
				continue;
			}
		}

		if (net_recv_data(dev_data->iface, rx_pkt) < 0) {
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
			dev_data->stats.error_details.rx_frame_errors++;
#endif /* CONFIG_NET_STATISTICS_ETHERNET */
			net_pkt_unref(rx_pkt);
		}
	}
}

static void gmac_rx_thread(void *arg1, void *unused1, void *unused2)
{
	int res;
	struct device *dev = (struct device *)arg1;
	struct gmac_dev_data *dev_data = dev->data;
	const struct gmac_dev_config *const cfg = dev->config;
	gmac_registers_t *gmac_regs = cfg->regs;

	while (1) {
		res = k_sem_take(&dev_data->rx_int_sem, K_MSEC(ETH_MCHP_RX_PKT_WAIT_TIMEOUT_MS));
		if (res == 0) {
			struct gmac_queue *queue = dev_data->rx_queue;

			gmac_rx(gmac_regs, queue);
		}
	}
}

static int eth_mchp_start(const struct device *dev)
{
	struct gmac_dev_data *const dev_data = dev->data;

	if (dev_data->link_up == false) {
		net_if_carrier_off(dev_data->iface);
	} else {
		net_eth_carrier_on(dev_data->iface);
	}

	return 0;
}

static int eth_mchp_stop(const struct device *dev)
{
	struct gmac_dev_data *const dev_data = dev->data;

	net_eth_carrier_off(dev_data->iface);
	return 0;
}

#if !defined(CONFIG_ETH_SAM_GMAC_FORCE_QUEUE) &&                                                   \
	((GMAC_ACTIVE_QUEUE_NUM != NET_TC_TX_COUNT) ||                                             \
	 ((NET_TC_TX_COUNT != NET_TC_RX_COUNT) && defined(CONFIG_NET_VLAN)))
static int eth_mchp_priority2queue(enum net_priority priority)
{
	return 0;
}
#endif

static int eth_mchp_send(const struct device *dev, struct net_pkt *pkt)
{
	const struct gmac_dev_config *const cfg = dev->config;
	struct gmac_dev_data *const dev_data = dev->data;
	gmac_registers_t *const gmac_regs = cfg->regs;
	struct gmac_queue *queue;
	struct gmac_desc_list *tx_desc_list;
	struct gmac_desc *tx_desc;
	struct gmac_desc *tx_first_desc;
	struct net_buf *frag;
	uint8_t *frag_data;
	uint16_t frag_len = 0;
	uint32_t err_tx_queue_flushed_count_at_entry;
	uint8_t pkt_prio;
	uint8_t pkt_buff_cnt = 0;
	uint8_t free_desc_cnt = 0;
	int8_t head_desc_index = 0;

	if (pkt == NULL) {
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
		dev_data->stats.errors.tx++;
#endif /* CONFIG_NET_STATISTICS_ETHERNET */
		return -EIO;
	}

	if (pkt->frags == NULL) {
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
		dev_data->stats.errors.tx++;
#endif /* CONFIG_NET_STATISTICS_ETHERNET */
		return -EIO;
	}

	pkt_prio = net_pkt_priority(pkt);
	queue = &dev_data->queue_list[eth_mchp_priority2queue(pkt_prio)];
	err_tx_queue_flushed_count_at_entry = queue->err_tx_queue_flushed_count;
	tx_desc_list = &queue->tx_desc_list;
	tx_first_desc = &tx_desc_list->buf_desc[tx_desc_list->head];
	frag = pkt->frags;

	k_mutex_lock(&queue->tx_mutex, K_FOREVER);
	free_desc_cnt = k_sem_count_get(&queue->tx_desc_sem);
	while (frag != NULL) {
		frag = frag->frags;
		pkt_buff_cnt++;
	}

	if (free_desc_cnt < pkt_buff_cnt) {
		k_mutex_unlock(&queue->tx_mutex);
		return -EIO;
	}

	tx_desc_list = &queue->tx_desc_list;
	tx_first_desc = &tx_desc_list->buf_desc[tx_desc_list->head];
	frag = pkt->frags;

	while (frag != NULL) {
		frag_data = frag->data;
		frag_len = frag->len;
		k_sem_take(&queue->tx_desc_sem, K_FOREVER);
		if (queue->err_tx_queue_flushed_count != err_tx_queue_flushed_count_at_entry) {
			k_mutex_unlock(&queue->tx_mutex);
			return -EIO;
		}

		tx_desc = &tx_desc_list->buf_desc[tx_desc_list->head];
		tx_desc->addr_w0 = (uint32_t)frag_data;
		tx_desc->status_w1 |= GMAC_TXW1_USED;

		if (frag->frags == NULL) {
			tx_desc->status_w1 |= GMAC_TXW1_LASTBUFFER;
		}

		tx_desc->status_w1 &= ~GMAC_TXW1_LEN;
		tx_desc->status_w1 |= (frag_len & GMAC_TXW1_LEN);
		MODULO_INC(tx_desc_list->head, tx_desc_list->len);
		__ASSERT(tx_desc_list->head != tx_desc_list->tail, "tx_desc_list overflow");
		frag_buffer_put(&queue->tx_frag_list, POINTER_TO_UINT(frag));
		net_pkt_frag_ref(frag);
		frag = frag->frags;
	}

	if (queue->err_tx_queue_flushed_count != err_tx_queue_flushed_count_at_entry) {
		gmac_regs->GMAC_NCR |= GMAC_NCR_TSTART_Msk;
		k_mutex_unlock(&queue->tx_mutex);
		return -EIO;
	}
	tx_desc_list->buf_desc[tx_desc_list->head].status_w1 |= GMAC_TXW1_USED;
	head_desc_index = tx_desc_list->head ? (tx_desc_list->head - 1) : (tx_desc_list->len - 1);
	frag = pkt->frags;
	while (frag != NULL) {
		tx_desc_list->buf_desc[head_desc_index].status_w1 &= ~GMAC_TXW1_USED;
		frag = frag->frags;
		MODULO_DEC(head_desc_index, tx_desc_list->len);
	}

	gmac_regs->GMAC_NCR |= GMAC_NCR_TSTART_Msk;
	k_mutex_unlock(&queue->tx_mutex);
	return 0;
}

static void eth_mchp_queue0_isr(const struct device *dev)
{
	const struct gmac_dev_config *const cfg = dev->config;
	struct gmac_dev_data *const dev_data = dev->data;
	gmac_registers_t *const gmac_regs = cfg->regs;
	struct gmac_queue *queue = &dev_data->queue_list[0];

	gmac_queue0_isr(gmac_regs, queue);
}

static int eth_mchp_initialize(const struct device *dev)
{
	const struct gmac_dev_config *const cfg = dev->config;
	int retval;

	cfg->config_func();

	retval = clock_control_on(cfg->eth_clock_cfg.clock_dev, cfg->eth_clock_cfg.mclk_apb_sys);
	if ((retval != 0) && (retval != -EALREADY)) {
		LOG_ERR("Failed to enable the MCLK APB for Ethernet: %d", retval);
		return retval;
	}

	retval = clock_control_on(cfg->eth_clock_cfg.clock_dev, cfg->eth_clock_cfg.mclk_ahb_sys);
	if ((retval != 0) && (retval != -EALREADY)) {
		LOG_ERR("Failed to enable the MCLK AHB for Ethernet: %d", retval);
		return retval;
	}

	retval = pinctrl_apply_state(cfg->pinctrl_cfg, PINCTRL_STATE_DEFAULT);
	if (retval != 0) {
		LOG_ERR("pinctrl_apply_state() Failed for Ethernet driver: %d", retval);
		return retval;
	}

	return retval;
}

static void eth_mchp_phy_link_state_changed(const struct device *pdev, struct phy_link_state *state,
					    void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	struct gmac_dev_data *const dev_data = dev->data;
	const struct gmac_dev_config *const cfg = dev->config;
	gmac_registers_t *const gmac_regs = cfg->regs;
	bool is_up;

	is_up = state->is_up;

	if ((is_up == true) && (dev_data->link_up == false)) {
		LOG_INF("Link up");
		dev_data->link_up = true;
		net_eth_carrier_on(dev_data->iface);
		gmac_link_configure(gmac_regs, PHY_LINK_IS_FULL_DUPLEX(state->speed),
				    PHY_LINK_IS_SPEED_100M(state->speed));
	} else if ((is_up == false) && (dev_data->link_up == true)) {
		LOG_INF("Link down");
		dev_data->link_up = false;
		net_eth_carrier_off(dev_data->iface);
	} else {
		/* To avoid sonar issue */
	}
}

static const struct device *eth_mchp_get_phy(const struct device *dev)
{
	const struct gmac_dev_config *const cfg = dev->config;

	return cfg->phy_dev;
}

static void eth_mchp_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct gmac_dev_data *const dev_data = dev->data;
	const struct gmac_dev_config *const cfg = dev->config;
	gmac_registers_t *const gmac_regs = cfg->regs;
	static bool init_done;
	int result;

	if (dev_data->iface == NULL) {
		dev_data->iface = iface;
	}

	ethernet_init(iface);

	if (init_done == true) {
		return;
	}

	result = gmac_init(gmac_regs);
	if (result < 0) {
		LOG_ERR("Unable to initialize ETH driver");
		return;
	}

	gmac_generate_set_mac(dev, gmac_regs, dev_data->mac_addr);

	LOG_INF("MAC: %02x:%02x:%02x:%02x:%02x:%02x", dev_data->mac_addr[0], dev_data->mac_addr[1],
		dev_data->mac_addr[2], dev_data->mac_addr[3], dev_data->mac_addr[4],
		dev_data->mac_addr[5]);

	net_if_set_link_addr(iface, dev_data->mac_addr, sizeof(dev_data->mac_addr),
			     NET_LINK_ETHERNET);

	for (int i = GMAC_QUE_0; i < GMAC_QUEUE_NUM; i++) {
		result = gmac_queue_init(gmac_regs, &dev_data->queue_list[i]);
		if (result < 0) {
			LOG_ERR("Unable to initialize ETH queue%d", i);
			return;
		}
	}

	if (device_is_ready(cfg->phy_dev) == true) {
		phy_link_callback_set(cfg->phy_dev, &eth_mchp_phy_link_state_changed, (void *)dev);
	} else {
		LOG_ERR("PHY device not ready");
	}

	k_sem_init(&dev_data->rx_int_sem, 0, K_SEM_MAX_LIMIT);
	k_thread_create(&dev_data->rx_thread, dev_data->rx_thread_stack,
			K_KERNEL_STACK_SIZEOF(dev_data->rx_thread_stack), gmac_rx_thread,
			(void *)dev, NULL, NULL,
			IS_ENABLED(CONFIG_NET_TC_THREAD_PREEMPTIVE)
				? K_PRIO_PREEMPT(CONFIG_ETH_MCHP_RX_THREAD_PRIORITY)
				: K_PRIO_COOP(CONFIG_ETH_MCHP_RX_THREAD_PRIORITY),
			0, K_NO_WAIT);
	k_thread_name_set(&dev_data->rx_thread, "mchp_eth_rx");

	init_done = true;
}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
static struct net_stats_eth *eth_mchp_get_stats(const struct device *dev)
{
	struct gmac_dev_data *dev_data = dev->data;
	const struct gmac_dev_config *const cfg = dev->config;
	gmac_registers_t *const gmac_regs = cfg->regs;

	gmac_get_stats(gmac_regs, &dev_data->stats);

	return &dev_data->stats;
}
#endif /* CONFIG_NET_STATISTICS_ETHERNET */

static enum ethernet_hw_caps eth_mchp_get_capabilities(const struct device *dev)
{
	enum ethernet_hw_caps hw_caps = ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE;

	ARG_UNUSED(dev);
#if defined(CONFIG_NET_VLAN)
	hw_caps |= ETHERNET_HW_VLAN;
#endif /* CONFIG_NET_VLAN */

	return hw_caps;
}

static int eth_mchp_set_config(const struct device *dev, enum ethernet_config_type type,
			       const struct ethernet_config *config)
{
	int result = 0;
	uint32_t val = 0;
	const struct gmac_dev_config *const cfg = dev->config;
	gmac_registers_t *const gmac_regs = cfg->regs;
	uint32_t chksum_support = ETHERNET_CHECKSUM_SUPPORT_IPV4_HEADER |
				  ETHERNET_CHECKSUM_SUPPORT_TCP | ETHERNET_CHECKSUM_SUPPORT_UDP;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS: {
		struct gmac_dev_data *const dev_data = dev->data;

		memcpy(dev_data->mac_addr, config->mac_address.addr, sizeof(dev_data->mac_addr));
		gmac_mac_addr_set(gmac_regs, 0, dev_data->mac_addr);
		LOG_INF("%s MAC set to %02x:%02x:%02x:%02x:%02x:%02x", dev->name,
			dev_data->mac_addr[0], dev_data->mac_addr[1], dev_data->mac_addr[2],
			dev_data->mac_addr[3], dev_data->mac_addr[4], dev_data->mac_addr[5]);
		result = net_if_set_link_addr(dev_data->iface, dev_data->mac_addr,
					      sizeof(dev_data->mac_addr), NET_LINK_ETHERNET);
	} break;

	case ETHERNET_CONFIG_TYPE_RX_CHECKSUM_SUPPORT: {
		if (config->chksum_support & ~chksum_support) {
			return -ENOTSUP;
		}

		val = gmac_regs->GMAC_NCFGR;
		val = val | GMAC_NCFGR_RXCOEN_Msk;
		gmac_regs->GMAC_NCFGR = val;
	} break;

	case ETHERNET_CONFIG_TYPE_TX_CHECKSUM_SUPPORT: {
		if (config->chksum_support & ~chksum_support) {
			return -ENOTSUP;
		}

		val = gmac_regs->GMAC_DCFGR;
		val = val | GMAC_DCFGR_TXCOEN_Msk;
		gmac_regs->GMAC_DCFGR = val;
	} break;

	default:
		return -ENOTSUP;
	}

	return result;
}

static int eth_mchp_get_config(const struct device *dev, enum ethernet_config_type type,
			       struct ethernet_config *config)
{
	const struct gmac_dev_config *const cfg = dev->config;
	gmac_registers_t *const gmac_regs = cfg->regs;
	uint32_t val;
	uint32_t retval = 0;
	uint32_t chksum_support = ETHERNET_CHECKSUM_SUPPORT_IPV4_HEADER |
				  ETHERNET_CHECKSUM_SUPPORT_TCP | ETHERNET_CHECKSUM_SUPPORT_UDP;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS: {
		struct gmac_dev_data *const dev_data = dev->data;

		memcpy(config->mac_address.addr, dev_data->mac_addr, sizeof(dev_data->mac_addr));
	} break;

	case ETHERNET_CONFIG_TYPE_RX_CHECKSUM_SUPPORT: {
		val = gmac_regs->GMAC_NCFGR;
		config->chksum_support = (val & GMAC_NCFGR_RXCOEN_Msk) ? chksum_support : 0;
	} break;

	case ETHERNET_CONFIG_TYPE_TX_CHECKSUM_SUPPORT: {
		val = gmac_regs->GMAC_DCFGR;
		config->chksum_support = (val & GMAC_DCFGR_TXCOEN_Msk) ? chksum_support : 0;
	} break;

	default:
		return -ENOTSUP;
	}

	return retval;
}

static const struct ethernet_api eth_mchp_api = {
	.iface_api.init = eth_mchp_iface_init,
	.start = eth_mchp_start,
	.stop = eth_mchp_stop,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats = eth_mchp_get_stats,
#endif /* CONFIG_NET_STATISTICS_ETHERNET */
	.get_capabilities = eth_mchp_get_capabilities,
	.set_config = eth_mchp_set_config,
	.get_config = eth_mchp_get_config,
	.get_phy = eth_mchp_get_phy,
	.send = eth_mchp_send,
};

static void eth_irq_config_0(void)
{
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, gmac, irq), DT_INST_IRQ_BY_NAME(0, gmac, priority),
		    eth_mchp_queue0_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_NAME(0, gmac, irq));
}

#define ETH_MCHP_CONFIG_DEFN(n)                                                                    \
	static const struct gmac_dev_config eth_config_##n = {                                     \
		.regs = (gmac_registers_t *)DT_INST_REG_ADDR(n),                                   \
		.active_queues = DT_INST_PROP(n, num_queues),                                      \
		.pinctrl_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                  \
		.config_func = eth_irq_config_##n,                                                 \
		ETH_MCHP_CLOCK_DEFN(n),                                                            \
		.phy_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, phy_handle)),                          \
	}

#define ETH_MCHP_DATA_DEFN(n)                                                                      \
	static struct gmac_dev_data eth_data_##n = {                                               \
		.mac_addr = DT_INST_PROP(n, local_mac_address),                                    \
		.queue_list = {{                                                                   \
			.que_idx = GMAC_QUE_0, /* clang-format off */                  \
		.rx_desc_list = {                       \
				.buf_desc = rx_desc_que0_eth_##n,       \
				.len = ARRAY_SIZE(rx_desc_que0_eth_##n),    \
			},                                  \
		.tx_desc_list = {                       \
				.buf_desc = tx_desc_que0_eth_##n,       \
				.len = ARRAY_SIZE(tx_desc_que0_eth_##n),    \
			}, /* clang-format on */                                           \
			.rx_frag_list = rx_frag_list_que0_eth_##n,                                 \
			.tx_frag_list =                                                            \
				{                                                                  \
					.buf = (uint32_t *)tx_frag_list_que0_eth_##n,              \
					.len = ARRAY_SIZE(tx_frag_list_que0_eth_##n),              \
				},                                                                 \
		}},                                                                                \
	}

#define ETH_MCHP_DEVICE_INIT(n)                                                                    \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	ETH_MCHP_DATA_DEFN(n);                                                                     \
	ETH_MCHP_CONFIG_DEFN(n);                                                                   \
	ETH_NET_DEVICE_DT_INST_DEFINE(n, eth_mchp_initialize, NULL, &eth_data_##n,                 \
				      &eth_config_##n, CONFIG_ETH_INIT_PRIORITY, &eth_mchp_api,    \
				      GMAC_MTU);

DT_INST_FOREACH_STATUS_OKAY(ETH_MCHP_DEVICE_INIT);
