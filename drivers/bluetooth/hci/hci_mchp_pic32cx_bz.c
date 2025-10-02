/*
 * Copyright (c) 2025 Microchip
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_types.h>
#include <bt_sys.h>
#include <rf_system.h>
#include <info_block.h>
#include <hci.h>

#define DT_DRV_COMPAT microchip_pic32cx_bz_bt_hci

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bt_hci_mchp_pic32cx_bz, CONFIG_BT_LOG_LEVEL);

struct hci_driver_data {
	bt_hci_recv_t recv;
};

struct hci_rx_data {
	void *reserved;
	struct net_buf *buf;
};

#define MCHP_BT_HCI_STACK_SIZE  2048
#define MCHP_BT_RECV_STACK_SIZE 2048

#define PREFER_ACL_TX_BUF_NUM (10)

static K_KERNEL_STACK_DEFINE(mchp_hci_thread_stack, MCHP_BT_HCI_STACK_SIZE);
static struct k_thread mchp_hci_thread;

static K_KERNEL_STACK_DEFINE(mchp_recv_thread_stack, MCHP_BT_RECV_STACK_SIZE);
static struct k_thread mchp_recv_thread;

static struct k_fifo mchp_recv_fifo;
static struct k_sem mchp_hci_sem;

#if defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ2)
#define EXT_COMMON_MEMORY_SIZE (28 * 1024)
#elif defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ6)
#define EXT_COMMON_MEMORY_SIZE (31 * 1024)
#endif

#if defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ2)
#define __mchp_bt_section Z_GENERIC_SECTION(.mchp_bt)
static uint8_t __mchp_bt_section s_btMem[EXT_COMMON_MEMORY_SIZE];

#elif defined(CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ6)
static uint8_t *s_btMem;

#else
#error "NOT supported SOC family or series"
#endif

typedef enum {
	OSAL_RESULT_NOT_IMPLEMENTED = -1,
	OSAL_RESULT_FALSE = 0,
	OSAL_RESULT_FAIL = 0,
	OSAL_RESULT_TRUE = 1,
	OSAL_RESULT_SUCCESS = 1,
} osal_result_t;

struct osal_api_list_type {
	uint32_t reserve1[3];

	osal_result_t (*osal_sem_pend)(struct k_sem *semID, k_timeout_t timeout);
	osal_result_t (*osal_sem_post)(struct k_sem *semID);
	osal_result_t (*osal_sem_postISR)(struct k_sem *semID);

	uint32_t reserve2[11];
};

struct osal_api_list_type osalAPIList;

static osal_result_t osal_sem_pend(struct k_sem *semID, k_timeout_t timeout)
{
	int ret;

	ret = k_sem_take(semID, timeout);

	if (ret == 0) {
		return OSAL_RESULT_TRUE;
	} else {
		return OSAL_RESULT_FALSE;
	}
}

static osal_result_t osal_sem_post(struct k_sem *semID)
{
	k_sem_give(semID);
	return OSAL_RESULT_TRUE;
}

static struct hci_rx_data *pack_rx_data(struct net_buf *buf)
{
	struct hci_rx_data *rx_data;

	rx_data = k_malloc(sizeof(struct hci_rx_data));
	if (rx_data == NULL) {
		return NULL;
	}

	rx_data->buf = buf;
	return rx_data;
}

static bool mchp_hci_event_handle(uint16_t length, uint8_t *p_packet)
{
	struct hci_rx_data *rx_data;
	struct net_buf *buf;

	buf = bt_buf_get_evt(p_packet[0], false, K_FOREVER);
	net_buf_add_mem(buf, p_packet, length);

	rx_data = pack_rx_data(buf);

	if (rx_data) {
		k_fifo_put(&mchp_recv_fifo, rx_data);
		return true;
	} else {
		return false;
	}
}

static bool mchp_acl_handle(uint16_t connHandle, uint16_t length, uint8_t *p_packet)
{
	struct hci_rx_data *rx_data;
	struct net_buf *buf;

	buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_FOREVER);

	net_buf_add_le16(buf, connHandle);
	net_buf_add_le16(buf, length);
	net_buf_add_mem(buf, p_packet, length);

	rx_data = pack_rx_data(buf);

	if (rx_data) {
		k_fifo_put(&mchp_recv_fifo, rx_data);
		return true;
	} else {
		return false;
	}
}

static void mchp_hci_thread_func(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	BM_Task(p1);
}

static void mchp_recv_thread_func(void *p1, void *p2, void *p3)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
	struct hci_driver_data *data = dev->data;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		struct hci_rx_data *rx_data = NULL;

		rx_data = k_fifo_get(&mchp_recv_fifo, K_FOREVER);

		if (rx_data) {
			data->recv(dev, rx_data->buf);
			k_free(rx_data);
		}
		k_yield();
	}
}

static int mchp_hci_drv_send(const struct device *dev, struct net_buf *buf)
{
	int ret = 0;

	ARG_UNUSED(dev);

	switch (net_buf_pull_u8(buf)) {
	case BT_HCI_H4_ACL:
		HCI_AclTx(buf->len, buf->data);
		break;
	case BT_HCI_H4_CMD:
		HCI_Cmd(buf->len, buf->data);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	net_buf_unref(buf);
	return ret;
}

static int mchp_hci_drv_open(const struct device *dev, bt_hci_recv_t recv)
{
	struct hci_driver_data *hci = dev->data;
	int ret = 0;

	IRQ_CONNECT(BT_INT0_IRQn, 4, BT_INT0_Handler, NULL, 0);
	IRQ_CONNECT(BT_INT1_IRQn, 4, BT_INT1_Handler, NULL, 0);
	IRQ_CONNECT(BT_LC_IRQn, 5, BT_LC_Handler, NULL, 0);

	k_thread_create(&mchp_hci_thread, mchp_hci_thread_stack,
			K_KERNEL_STACK_SIZEOF(mchp_hci_thread_stack), mchp_hci_thread_func, NULL,
			NULL, NULL, K_PRIO_COOP(CONFIG_BT_DRIVER_RX_HIGH_PRIO), 0, K_NO_WAIT);
	k_thread_name_set(&mchp_hci_thread, "BT RX pri");

	k_thread_create(&mchp_recv_thread, mchp_recv_thread_stack,
			K_KERNEL_STACK_SIZEOF(mchp_recv_thread_stack), mchp_recv_thread_func,
			(void *)dev, NULL, NULL, K_PRIO_COOP(CONFIG_BT_RX_PRIO), 0, K_NO_WAIT);
	k_thread_name_set(&mchp_recv_thread, "BT RX");
	hci->recv = recv;

	return ret;
}

static int mchp_hci_drv_close(const struct device *dev)
{
	struct hci_driver_data *data = dev->data;

	/* Abort prio RX thread */
	k_thread_abort(&mchp_hci_thread);

	/* Abort RX thread */
	k_thread_abort(&mchp_recv_thread);

	/* Clear the (host) receive callback */
	data->recv = NULL;

	return 0;
}

static int mchp_hci_drv_init(const struct device *dev)
{
	int ret = 0;
	BT_SYS_Cfg_T btSysCfg;
	BT_SYS_Option_T btOption;

	ARG_UNUSED(dev);

#ifdef CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ2
	/* Disable PMD lock */
	CFG_REGS->CFG_CFGCON0 &= ~CFG_CFGCON0_PMDLOCK_Msk;
	CFG_REGS->CFG_PMD1 &= ~CFG_PMD1_BTMD_Msk;
	CFG_REGS->CFG_PMD3 &= ~CFG_PMD3_AESMD_Msk;
#endif

	/* set bt_en_main_clk[20] */
	BTZBSYS_REGS->BTZBSYS_SUBSYS_CNTRL_REG0 |= BTZBSYS_SUBSYS_CNTRL_REG0_bt_en_main_clk(1U);
	/* Initialize the RF */
	SYS_Load_Cal(WSS_ENABLE_BLE);

	k_sem_init(&mchp_hci_sem, 0, 1);
	k_fifo_init(&mchp_recv_fifo);

	/* Retrieve BLE calibration data */
	(void)memset(&btSysCfg, 0, sizeof(BT_SYS_Cfg_T));
	btSysCfg.addrValid = IB_GetBdAddr(&btSysCfg.devAddr[0]);
	btSysCfg.rssiOffsetValid = IB_GetRssiOffset(&btSysCfg.rssiOffset);

	if (!IB_GetAntennaGain(&btSysCfg.antennaGain)) {
		btSysCfg.antennaGain = CONFIG_CUSTOM_ANTENNA_GAIN;
	}
#ifdef CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ6
	btSysCfg.adcTimingValid = IB_GetAdcTiming(&btSysCfg.adcTiming08, &btSysCfg.adcTiming51);
#endif

	osalAPIList.osal_sem_pend = osal_sem_pend;
	osalAPIList.osal_sem_post = osal_sem_post;
	osalAPIList.osal_sem_postISR = osal_sem_post;

#ifdef CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ6
	s_btMem = k_malloc(EXT_COMMON_MEMORY_SIZE);
	if (s_btMem == NULL) {
		LOG_ERR("EXT_COMMON_MEMORY k_malloc fail\n");
		return -ENOMEM;
	}
#endif

	/* Configure BLE option */
	(void)memset(&btOption, 0, sizeof(BT_SYS_Option_T));
	btOption.hciMode = true;
	btOption.cmnMemSize = EXT_COMMON_MEMORY_SIZE;
	btOption.p_cmnMemAddr = s_btMem;
	btOption.deFeatMask = 0;

	/* Initialize BLE Stack */
	BT_SYS_Init(&mchp_hci_sem, &osalAPIList, &btOption, &btSysCfg);

#ifdef CONFIG_SOC_FAMILY_MICROCHIP_PIC32CX_BZ2
	BT_SYS_UpgradeBleVersion(BT_SYS_BLE_VER_6_0);
#endif

	HCI_Init(PREFER_ACL_TX_BUF_NUM);
	HCI_EventRegister(mchp_hci_event_handle);
	HCI_AclRxRegister(mchp_acl_handle);

	LOG_DBG("Microchip BT HCI Initialized\n");

	HCI_AdvInit();            /* Advertising */
	HCI_ExtAdvInit();         /* Enable Extended Advertising */
	HCI_PeriodicAdvInit();    /* Enable Periodic Advertising */
	HCI_ScanInit();           /* Scan */
	HCI_ExtScanInit();        /* Enable Extended Scan */
	HCI_ConnPeripheralInit(); /* Peripheral */
	HCI_ConnCentralInit();    /* Central */
	HCI_ExtConnCntrlInit();   /* Enable Extended Central */
	HCI_SyncInit();           /* Enable Synchronization */

	return ret;
}

static const struct bt_hci_driver_api drv = {
	.open = mchp_hci_drv_open,
	.send = mchp_hci_drv_send,
	.close = mchp_hci_drv_close,
};

#define HCI_DEVICE_INIT(inst)                                                                      \
	static struct hci_driver_data hci_data_##inst = {};                                        \
	DEVICE_DT_INST_DEFINE(inst, mchp_hci_drv_init, NULL, &hci_data_##inst, NULL, POST_KERNEL,  \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &drv)

/* Only one instance supported right now */
HCI_DEVICE_INIT(0)
