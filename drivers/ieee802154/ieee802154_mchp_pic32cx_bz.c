/*
 * Copyright (c) 2025 Microchip
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* ieee802154_pic32cx_bz.c - Microchip PIC32CX_BZ IEEE 802.15.4 Driver */

#define DT_DRV_COMPAT microchip_pic32cx_bz_ieee802154

/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME ieee802154_pic32cx_bz
#define LOG_LEVEL       CONFIG_IEEE802154_DRIVER_LOG_LEVEL

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/debug/stack.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>

#include <string.h>
#include <zephyr/random/random.h>
#include <zephyr/linker/sections.h>
#include <zephyr/sys/atomic.h>

#include <zephyr/net/ieee802154_radio.h>

#include <info_block.h>
#include <rf_system.h>
#include "ieee_phy_const.h"
#include "bmm.h"
#include "stack_config.h"
#include "app_config.h"
#include "phy.h"
#include "phy_utils.h"
#include "ieee802154_mchp_pic32cx_bz.h"

#if defined(CONFIG_IEEE802154_PIC32CX_BZ_CRYPTO)
#include "sal.h"
#endif

#if defined(CONFIG_NET_L2_OPENTHREAD)
#include <zephyr/net/openthread.h>
#include <zephyr/net/ieee802154_radio_openthread.h>
#endif /* CONFIG_NET_L2_OPENTHREAD */

/* Convenience defines for RADIO */
#define PIC32CX_BZ_802154_DATA(dev) ((struct pic32cx_bz_context *const)(dev)->data)

#define XTAL_ACCURACY             40U
#define IEEE802154_PHY_FCS_LENGTH 2U

/* driver-allocated attribute memory - constant across all driver instances */
IEEE802154_DEFINE_PHY_SUPPORTED_CHANNELS(drv_attr, 11, 26);
K_KERNEL_STACK_DEFINE(ieee802154_radio_stack, 256);

/* Buffer holding ACK frames */
uint8_t sRxAckBuffer[128];
uint8_t sTxAckBuffer[128];
/* Key structure holding Prev, current, and Next Keys and key Ids */
struct ieee802154_local_key s_mac_keys[3];
#if defined(CONFIG_IEEE802154_PIC32CX_BZ_CRYPTO)
static struct Aes aes;
#endif

static struct pic32cx_bz_context gctx;
static PHY_Retval_t tx_status = PHY_FAILURE;
static frameInfo_t recFrameInfo;
static struct k_work_q iface_work_q;

extern void PHY_EnableEnhancedFramepending(bool enable);
extern PHY_Retval_t PHY_Tx2015Frame(PHY_FrameInfo_t *txFrame, PHY_CSMAMode_t csmaMode,
				    bool performFrameRetry);

const struct device *get_dev(void)
{
#if defined(CONFIG_IEEE802154_RAW_MODE)
	return gctx.dev;
#else
	return net_if_get_device(gctx.iface);
#endif
}

static uint64_t pic32cx_bz_802154_timer_current_time_get(void)
{
	int64_t ticks = k_uptime_ticks();

	return k_ticks_to_us_ceil64(ticks);
}

/* Get Radio Time */
static net_time_t pic32cx_bz_get_time(const struct device *dev)
{
	ARG_UNUSED(dev);
	return (net_time_t)(pic32cx_bz_802154_timer_current_time_get() * NSEC_PER_USEC);
}

#if defined(CONFIG_IEEE802154_PIC32CX_BZ_CRYPTO)
static void pic32cx_bz_set_mac_keys(struct ieee802154_key *mac_keys)
{
	for (uint8_t i = 0; mac_keys->key_value && i < NUM_MAC_KEYS; mac_keys++, i++) {
		memcpy((uint8_t *)&s_mac_keys[i].key_value, mac_keys->key_value, 16);
		s_mac_keys[i].key_frame_counter = mac_keys->key_frame_counter;
		s_mac_keys[i].frame_counter_per_key = mac_keys->frame_counter_per_key;
		s_mac_keys[i].key_id_mode = mac_keys->key_id_mode;
		memcpy((uint8_t *)&s_mac_keys[i].key_id, mac_keys->key_id, 1);
	};
}

static struct ieee802154_local_key *pic32cx_bz_get_curr_key(void)
{
	LOG_DBG("Key Value-");
	for (uint8_t i = 0; i < 16; i++) {
		LOG_DBG(" %x", (s_mac_keys[1].key_value[i]));
	}

	return &s_mac_keys[1];
}

static void pic32cx_bz_set_frame_counter(uint32_t mac_frame_counter)
{
	gctx.frame_counter = mac_frame_counter;
}

static void pic32cx_bz_set_frame_counter_if_larger(uint32_t mac_frame_counter)
{

	if (mac_frame_counter > gctx.frame_counter) {
		gctx.frame_counter = mac_frame_counter;
	}
}
#endif

#if defined(CONFIG_OPENTHREAD_CSL_RECEIVER)
static void pic32cx_bz_set_csl_period(uint32_t mac_csl_period)
{
	LOG_DBG("154 Radio: CSL Period %d", mac_csl_period);
	gctx.csl_period = mac_csl_period;
}

static void pic32cx_bz_set_csl_sample_time(net_time_t mac_csl_smaple_time)
{
	gctx.csl_sample_time = mac_csl_smaple_time;
}

/* Get current phase related to CSL period*/
static uint16_t __attribute__((optimize("O0"))) getCslPhase(void)
{
	uint32_t us;
	uint64_t currTime = 0;
	uint64_t cslPeriodInUs = 0;

	currTime = pic32cx_bz_802154_timer_current_time_get(); /* us */

	cslPeriodInUs = gctx.csl_period * RADIO_TEN_SYMBOLS_TIME;
	uint64_t sampleTime = (uint64_t)(gctx.csl_sample_time / NSEC_PER_USEC);

	if (currTime >= sampleTime) {
		uint32_t time_from_previous_window =
			(uint32_t)((currTime - sampleTime) % cslPeriodInUs);

		us = cslPeriodInUs - time_from_previous_window;
	} else {
		us = (uint32_t)((sampleTime - currTime) % cslPeriodInUs);
	}

	/* Round to the nearest integer when converting us to CSL units */
	uint32_t csl_phase = (us + (RADIO_TEN_SYMBOLS_TIME >> 1)) / RADIO_TEN_SYMBOLS_TIME;

	if (0 == csl_phase) {
		/* If the phase was rounded down to 0, increase it by one period. */
		csl_phase = gctx.csl_period;
	}
	return (uint16_t)csl_phase;
}
#endif

#if defined(CONFIG_IEEE802154_PIC32CX_BZ_CRYPTO)

static void createNonce(uint8_t *noncePtr, const uint8_t *aExtAddress, uint32_t frameCounter)
{
	uint8_t nonceIndex = 0;

	memcpy(noncePtr, aExtAddress, 8);
	nonceIndex += 8;

	noncePtr[nonceIndex++] = (frameCounter >> 24) & 0xff;
	noncePtr[nonceIndex++] = (frameCounter >> 16) & 0xff;
	noncePtr[nonceIndex++] = (frameCounter >> 8) & 0xff;
	noncePtr[nonceIndex++] = (frameCounter >> 0) & 0xff;

	noncePtr[nonceIndex] = 0x05;
}
/* Function to secure the Enhanced ACK frame to be transmitted */
static bool processTxAckSecurity(const struct device *dev, uint8_t *frame, uint8_t frameLen,
				 uint8_t hdrLen, struct ieee802154_local_key *key, uint32_t fc)
{
	int status = 0;
	uint8_t nonce[13] = {0};
	uint8_t macHdrLen = 0;
	uint8_t macPayloadLen = 0;
	uint8_t *macPayload = NULL;
	uint8_t *macTag = NULL;
	uint8_t macframeLen = 0;

	macHdrLen = hdrLen;
	macframeLen = frameLen;

	macPayloadLen = macframeLen - (macHdrLen + 4 + 2);

	macPayload = (uint8_t *)&frame[macHdrLen];
	macTag = (uint8_t *)&frame[macframeLen - (4 + 2)];

	createNonce(nonce, (uint8_t *)&gctx.ext_addr, fc);

	SAL_AesSetKey(&aes, (uint32_t *)&key->key_value, 16U);

	SAL_AesCcmEncrypt(&aes, nonce, NONCE_SIZE, (uint8_t *)&frame[0], (uint32_t)macHdrLen,
			  macPayload, (uint32_t)macPayloadLen, macPayload, macTag, MIC_SIZE);

	return status;
}
#endif

/*
 * PHY_GenerateEnhancedAck() is invoked upon reception of an ACK Requested v2 (2015) frame
 * to generate the enhanced ACK frame to be transmitted. The PHY layer then sends the ACK.
 */
void PHY_GenerateEnhancedAck(PHY_FrameInfo_t *rxFrame, PHY_FrameInfo_t *ackFramePtr)
{
	uint8_t ackHdrLen = 0;
	uint8_t ackFramelen = 0;

	const ie_data_t *ie_data = NULL;

	parseFrame(rxFrame->mpdu, &recFrameInfo);
	if (get_short_ack_ie_entry() > 0 || get_ext_ack_ie_entry() > 0) {
		if (((recFrameInfo.fcf.value & FCF_SRC_ADDR_MASK) == FCF_SRC_ADDR_SHORT)) {
			uint8_t shrAddr[2];

			sys_put_le16(recFrameInfo.srcAddr.srcShAddr, shrAddr);
			ie_data = get_short_ie(shrAddr);
		} else {
			ie_data = get_ext_ie((uint8_t *)&recFrameInfo.srcAddr.srcExtAddr);
		}
	}

	if (((recFrameInfo.fcf.value & FCF_SRC_ADDR_MASK) == FCF_SRC_ADDR_SHORT)) {
		gctx.sAckedwithFpb = searchShrtEntry(recFrameInfo.srcAddr.srcShAddr);
	} else {
		gctx.sAckedwithFpb = searchExtEntry((uint8_t *)&recFrameInfo.srcAddr.srcExtAddr);
	}

	if (ie_data != NULL) {
		ackHdrLen = buildEnhancedAckFrame(&recFrameInfo, &sTxAckBuffer[1], true,
						  ie_data->len, ie_data->p_data, gctx.sAckedwithFpb,
						  gctx.frame_counter);
	} else {
		bool iePresent = false;

		if (gctx.csl_period > 0) {
			iePresent = true;
		}

		ackHdrLen = buildEnhancedAckFrame(&recFrameInfo, &sTxAckBuffer[1], iePresent, 0,
						  NULL, gctx.sAckedwithFpb, gctx.frame_counter);
	}
#if defined(CONFIG_OPENTHREAD_CSL_RECEIVER)
	if (gctx.csl_period > 0) {
		uint16_t cslPhaseNow = getCslPhase();

		struct ieee802154_header_ie header_ie =
			IEEE802154_DEFINE_HEADER_IE_CSL_REDUCED(cslPhaseNow, gctx.csl_period);

		memcpy(&sTxAckBuffer[ackHdrLen + 1], &header_ie, 6);
		ackHdrLen += 6;
	}
#endif

	ackFramelen += ackHdrLen + 2; /* Add FCS Len */

#if defined(CONFIG_IEEE802154_PIC32CX_BZ_CRYPTO)
	/* Perform Encryption */
	if (recFrameInfo.fcf.security_enabled && recFrameInfo.securityControl.keyIdMode == 0x01) {
		uint8_t keyId = recFrameInfo.keyId.keyIndex;
		struct ieee802154_local_key key = s_mac_keys[1]; /* Initialize with current key */

		for (uint8_t i = 0; i < NUM_MAC_KEYS; i++) {
			if (keyId == (s_mac_keys[i].key_id)) {
				key = s_mac_keys[i];
				break;
			}
		}

		/* Update the local variables */
		gctx.sAckFrameCounter = gctx.frame_counter++;
		gctx.sAckKeyId = keyId;
		gctx.sAckedWithSecEnhAck = true;

		ackFramelen += 4; /* Add MIC Len */
		/* Secure the Enhanced ACK if the packet
		 * being received is Security enabled at MAC level
		 */
		processTxAckSecurity(get_dev(), (uint8_t *)&sTxAckBuffer[1], ackFramelen, ackHdrLen,
				     &key, gctx.sAckFrameCounter);

	} else {
		/* If received frame is not secured one clear
		 * the global variables related to ACK security material
		 */
		gctx.sAckFrameCounter = 0;
		gctx.sAckKeyId = 0;
		gctx.sAckedWithSecEnhAck = false;
    }
#else
	/* If received frame is not secured one clear
	 * the global variables related to ACK security material
	 */
	gctx.sAckFrameCounter = 0;
	gctx.sAckKeyId = 0;
	gctx.sAckedWithSecEnhAck = false;
#endif
	sTxAckBuffer[0] = ackFramelen;
	ackFramePtr->mpdu = &sTxAckBuffer[0];
}

#if defined(CONFIG_NET_L2_OPENTHREAD)

/* PHY_GenerateImmAck will be invoked by the PHY layer on the reception of frame
 * to generate the immediate acknowledgment - This will be called only when
 * enhanced frame pending is required
 */
void PHY_GenerateImmAck(PHY_FrameInfo_t *rxFrame, PHY_FrameInfo_t *ackFramePtr)
{
	/* Re-create ACK frame. */
	gctx.sAckFrame[1] = FCF_FRAMETYPE_ACK;
	if (true == gctx.sAckedwithFpb) {
		gctx.sAckFrame[1] |= FCF_FRAME_PENDING;
	}

	gctx.sAckFrame[2] = 0;
	gctx.sAckFrame[3] = rxFrame->mpdu[3];
	gctx.sAckFrame[0] = 5;

	ackFramePtr->mpdu = &(gctx.sAckFrame[0]);
}
#endif

bool PHY_IsFramePendingFromNextLayer(PHY_Addr_t *addr, uint8_t *addrMode)
{

	if (false == gctx.auto_fpb) {
		gctx.sAckedwithFpb = true;
	}
#if defined(CONFIG_NET_L2_OPENTHREAD)
	else if ((IEEE802154_FPB_ADDR_MATCH_THREAD == gctx.fpb_mode) && (true == gctx.auto_fpb)) {
		/* Search FPB for this address */
		uint8_t addrModeType = *addrMode;

		if (addrModeType == FCF_LONG_ADDR) {
			gctx.sAckedwithFpb = (searchExtEntry(addr->val));

		} else if (addrModeType == FCF_SHORT_ADDR) {
			gctx.sAckedwithFpb = searchShrtEntry((uint16_t)addr->shortAddr);
		}
	}
#endif
	return gctx.sAckedwithFpb;
}

/* PHY_RxStartCallback will be invoked by PHY Layer on the start of SFD reception */
void PHY_RxStartCallback(void)
{

	gctx.rx_timestamp =
		(net_time_t)(pic32cx_bz_802154_timer_current_time_get() * NSEC_PER_USEC);
}

/* PHY_RxFrameCallback will be invoked by PHY layer on the reception of the frame*/
void PHY_RxFrameCallback(PHY_FrameInfo_t *rxFrame)
{
	struct net_pkt *pkt;
	uint8_t frameLen = rxFrame->mpdu[0];

	pkt = net_pkt_rx_alloc_with_buffer(gctx.iface, frameLen, AF_UNSPEC, 0, K_FOREVER);

	if (0 > (net_pkt_write(pkt, rxFrame->mpdu + 1, frameLen))) {
		goto drop;
	}

	net_pkt_set_ieee802154_lqi(pkt, rxFrame->mpdu[frameLen + LQI_LEN]);
	net_pkt_set_ieee802154_rssi_dbm(
		pkt,
		(int16_t)(rxFrame->mpdu[frameLen + LQI_LEN + ED_VAL_LEN] + PHY_GetRSSIBaseVal()));
#if defined(CONFIG_NET_L2_OPENTHREAD)
	net_pkt_set_ieee802154_ack_fpb(pkt, gctx.sAckedwithFpb);
	net_pkt_set_ieee802154_ack_fc(pkt, gctx.sAckFrameCounter);
	net_pkt_set_ieee802154_ack_keyid(pkt, gctx.sAckKeyId);
	net_pkt_set_ieee802154_ack_seb(pkt, gctx.sAckedWithSecEnhAck);
#endif
	net_pkt_set_timestamp_ns(pkt, gctx.rx_timestamp);

	if (net_recv_data(gctx.iface, pkt) < 0) {
		LOG_ERR("RX Packet dropped by NET stack");
		net_pkt_unref(pkt);
		goto drop;
	}
drop:
	gctx.sAckedwithFpb = false;
	gctx.sAckedWithSecEnhAck = false;
	gctx.sAckFrameCounter = 0;
	gctx.sAckKeyId = 0;
	bmm_buffer_free((buffer_t *)rxFrame->buffer_header);
}

/* PHY_TxDoneCallback will be invoked by PHY layer to inform the status on the packet being
 * transmitted This will be called for the PHY_TxFrame() request
 */
void PHY_TxDoneCallback(PHY_Retval_t status, PHY_FrameInfo_t *frame)
{
	tx_status = status;
	/* Keep compiler happy. */
	frame = frame;

	k_sem_give(&gctx.txDone_fifo);
}

/* PHY_Tx2015FrameDoneCallback will be invoked by PHY layer to
 * inform the status on the packet being transmitted This will
 * be called for the PHY_Tx2015Frame() request
 */
void PHY_Tx2015FrameDoneCallback(PHY_Retval_t status, PHY_FrameInfo_t *frame,
				 PHY_FrameInfo_t *ackFramePtr)
{
	/* Keep compiler happy. */
	tx_status = status;
	frame = frame;

	if (ackFramePtr != NULL && ackFramePtr->mpdu != NULL) {
		/* Fill the sRxAckBuffer[] with enhanced ACK information */
		uint8_t frameLen = ackFramePtr->mpdu[0];

		memcpy(&sRxAckBuffer, &ackFramePtr->mpdu[0], frameLen + LQI_LEN + ED_VAL_LEN + 1);
	}

	k_sem_give(&gctx.txDone_fifo);
}

/* PHY_EdEndCallback will be called for the PHY_EdStart request to PHY layer */

void PHY_PostTask(bool isISRContext)
{
	k_sem_give(&(gctx.semPhyInternalHandler));
}

/* Get HW mac address */

static inline uint8_t *get_mac(const struct device *dev)
{
	LOG_DBG("154 Radio: Get HW mac address");

	struct pic32cx_bz_context *ctx = dev->data;

	if (!IB_GetMACAddr(&(ctx->mac_addr[0]))) {
		sys_rand_get(ctx->mac_addr, sizeof(ctx->mac_addr));

		ctx->mac_addr[0] = (ctx->mac_addr[0] & ~0x01) | 0x02;
	}

	return &(ctx->mac_addr[0]);
}

/* Attribute get */

static int pic32cx_bz_attr_get(const struct device *dev, enum ieee802154_attr attr,
			       struct ieee802154_attr_value *value)
{
	LOG_DBG("154 Radio: Attribute get : Supported Channels");
	if (ieee802154_attr_get_channel_page_and_range(
		    attr, IEEE802154_ATTR_PHY_CHANNEL_PAGE_ZERO_OQPSK_2450_BPSK_868_915,
		    &drv_attr.phy_supported_channels, value)) {
		return 0;
	}
	return 0;
}

/* Perform CCA */

static int pic32cx_bz_cca(const struct device *dev)
{
	int status = 0;

	PHY_Retval_t retVal = PHY_FAILURE;

	retVal = PHY_CCAPerform();

	if (PHY_CHANNEL_IDLE != retVal) {
		status = -EBUSY;
	}
	return status;
}

/* Set Channels */
static int pic32cx_bz_set_channel(const struct device *dev, uint16_t channel)
{
	int status = 0;
	PHY_TrxStatus_t prev_trx_state = PHY_TRX_OFF;

	LOG_DBG("154 Radio:Set Channel : %d", channel);

	prev_trx_state = PHY_GetTrxStatus();
	/* Disable Radio operation if not in TRX state */
	if (PHY_TRX_OFF != prev_trx_state) {
		PHY_RxEnable(PHY_STATE_TRX_OFF);
	}
	/* Set the Channel */
	status = PHY_PibSet(phyCurrentChannel, (PibValue_t *)&channel);

	if (PHY_TRX_OFF != prev_trx_state) {
		/* Restore the TRX state to Previous State */
		PHY_RxEnable(prev_trx_state);
	}

	return status;
}

/* Set filter attributes */

static int pic32cx_bz_filter(const struct device *dev, bool set, enum ieee802154_filter_type type,
			     const struct ieee802154_filter *filter)
{
	int status = 0;
	PHY_TrxStatus_t prev_trx_state = PHY_TRX_OFF;
	PibValue_t pibValue;

	LOG_DBG("154 Radio: filter %u", type);

	if (!set) {
		return -ENOTSUP;
	}
	prev_trx_state = PHY_GetTrxStatus();
	/* Disable Radio operation if not in TRX state */
	if (PHY_TRX_OFF != prev_trx_state) {
		PHY_RxEnable(PHY_STATE_TRX_OFF);
	}

	switch (type) {
	case IEEE802154_FILTER_TYPE_IEEE_ADDR: {
		memcpy(&pibValue.pib_value_64bit, filter->ieee_addr,
		       sizeof(pibValue.pib_value_64bit));
		for (uint8_t j = 0; j < 8; j++) {
			gctx.ext_addr[7 - j] = filter->ieee_addr[j];
		}
		status = PHY_PibSet(macIeeeAddress, (PibValue_t *)&pibValue);
	} break;
	case IEEE802154_FILTER_TYPE_SHORT_ADDR: {
		pibValue.pib_value_16bit = filter->short_addr;
		status = PHY_PibSet(macShortAddress, (PibValue_t *)&pibValue);
	} break;
	case IEEE802154_FILTER_TYPE_PAN_ID: {
		pibValue.pib_value_16bit = filter->pan_id;
		status = PHY_PibSet(macPANId, (PibValue_t *)&pibValue);
	} break;
	default:
		status = -ENOTSUP;
	}
	if (PHY_TRX_OFF != prev_trx_state) {
		/* Restore the TRX state to Previous State */
		PHY_RxEnable(prev_trx_state);
	}

	return status;
}

/* Set TX power */

static int pic32cx_bz_set_txpower(const struct device *dev, int16_t dbm)
{
	int status = 0;

	LOG_DBG("154 Radio: Set Tx power:  %d dbm", dbm);

	status = PHY_PibSet(phyTransmitPower, (PibValue_t *)&dbm);

	return status;
}

/* RX Start */

static int pic32cx_bz_start(const struct device *dev)
{
	PHY_RxEnable(PHY_RX_ON);
	return 0;
}

/* RX Stop */

static int pic32cx_bz_stop(const struct device *dev)
{
	LOG_DBG("154 Radio: in RX OFF");
	PHY_RxEnable(PHY_STATE_TRX_OFF);
	return 0;
}

/* CW TX */

#if defined(CONFIG_IEEE802154_PIC32CX_BZ_CARRIER_FUNCTIONS)
static int pic32cx_bz_continuous_carrier(const struct device *dev)
{
	ARG_UNUSED(dev);
	LOG_DBG("154 Radio:Starting Continuous wave transmission");
	return 0;
}
#endif

#if !defined(CONFIG_IEEE802154_RAW_MODE)
static int pic32cx_bz_handle_ack(struct pic32cx_bz_context *ctx, uint8_t sq_no, PHY_Retval_t status,
				 bool isEnhAck)
{
	int rstatus = 0;
	struct net_pkt *pic32cx_bz_ack_pkt = NULL;
	uint8_t pic32cx_bz_ack_psdu[ACK_PAYLOAD_LEN];

	if (isEnhAck) {
		LOG_DBG("2015ACK Packet.");
		pic32cx_bz_ack_pkt = net_pkt_rx_alloc_with_buffer(ctx->iface, aMaxPHYPacketSize,
								  AF_UNSPEC, 0, K_NO_WAIT);
	} else {
		LOG_DBG("Imm Acc Packet.");

		pic32cx_bz_ack_pkt = net_pkt_rx_alloc_with_buffer(ctx->iface, ACK_PAYLOAD_LEN,
								  AF_UNSPEC, 0, K_NO_WAIT);
	}

	if (NULL == pic32cx_bz_ack_pkt) {
		LOG_ERR("No free packet available.");
		rstatus = -ENOMSG;
		goto end;
	}
	if (isEnhAck) {
		uint8_t ackLen = sRxAckBuffer[0];

		/* Use some fake values for LQI and RSSI. */
		if ((net_pkt_write(pic32cx_bz_ack_pkt, (uint8_t *)&sRxAckBuffer[1], ackLen)) < 0) {
			LOG_ERR("Failed to write to a packet.");
			rstatus = -ENOMSG;
			goto end;
		}

		(void)net_pkt_set_ieee802154_lqi(pic32cx_bz_ack_pkt,
						 sRxAckBuffer[ackLen + LQI_LEN]);
		(void)net_pkt_set_ieee802154_rssi_dbm(
			pic32cx_bz_ack_pkt,
			(sRxAckBuffer[ackLen + LQI_LEN + ED_VAL_LEN] + PHY_GetRSSIBaseVal()));

	} else {
		/* Re-create ACK frame. */
		pic32cx_bz_ack_psdu[0] = FCF_FRAMETYPE_ACK;
		if (status == PHY_FRAME_PENDING) {
			pic32cx_bz_ack_psdu[0] |= FCF_FRAME_PENDING;
		}

		pic32cx_bz_ack_psdu[1] = 0;
		pic32cx_bz_ack_psdu[2] = sq_no;

		if (net_pkt_write(pic32cx_bz_ack_pkt, pic32cx_bz_ack_psdu,
				  sizeof(pic32cx_bz_ack_psdu)) < 0) {
			LOG_ERR("Failed to write to a packet.");
			rstatus = -ENOMSG;
			goto end;
		}

		/* Use some fake values for LQI and RSSI. */
		(void)net_pkt_set_ieee802154_lqi(pic32cx_bz_ack_pkt, 80);
		(void)net_pkt_set_ieee802154_rssi_dbm(pic32cx_bz_ack_pkt, -40);
	}

	net_pkt_set_timestamp_ns(pic32cx_bz_ack_pkt, pic32cx_bz_get_time(get_dev()));

	net_pkt_cursor_init(pic32cx_bz_ack_pkt);

	if (ieee802154_handle_ack(ctx->iface, pic32cx_bz_ack_pkt) != NET_OK) {
		LOG_WRN("ACK packet not handled - releasing.");
	}

end:
	if (pic32cx_bz_ack_pkt != NULL) {
		net_pkt_unref(pic32cx_bz_ack_pkt);
	}

	return rstatus;
}
#else
static int pic32cx_bz_handle_ack(struct pic32cx_bz_context *ctx, uint8_t sq_no, PHY_Retval_t status,
				 bool isEnhAck)
{
	return 0;
}
#endif

static int pic32cx_bz_tx(const struct device *dev, enum ieee802154_tx_mode mode,
			 struct net_pkt *pkt, struct net_buf *frag)
{
	bool is2015Frame = false;
	PHY_FrameInfo_t txFrame;
	int status = 0;
	struct pic32cx_bz_context *ctx = dev->data;

	uint8_t buffLen = frag->len;

	LOG_DBG("TX len %d", buffLen);
	LOG_DBG("154 Radio: TX");

	frameInfo_t txFrameInfo;

	if (ctx->tx_mode != mode) {
		switch (mode) {
		case IEEE802154_TX_MODE_DIRECT: {
			/* skip retries & csma/ca algorithm */
			ctx->csmaBackoffs = 0;
			ctx->maxRetries = 0;
			ctx->csmaMode = NO_CSMA_NO_IFS;
		} break;
		case IEEE802154_TX_MODE_CSMA_CA:
			/* backoff maxBE = 5, minBE = 3 */
			{
				ctx->csmaBackoffs = 4;
				ctx->maxRetries = 3;
				ctx->csmaMode = CSMA_UNSLOTTED;
			}
			break;
		case IEEE802154_TX_MODE_CCA:
			/* backoff period = 0 */
			{
				ctx->csmaBackoffs = 1;
				ctx->maxRetries = 0;
				ctx->csmaMode = CSMA_UNSLOTTED;
			}
			break;
		case IEEE802154_TX_MODE_TXTIME:
		case IEEE802154_TX_MODE_TXTIME_CCA:
		default:
			LOG_ERR("TX mode %d not supported", mode);
			return -ENOTSUP;
		}

		ctx->tx_mode = mode;
	}

	PHY_PibSet(macMaxCSMABackoffs, (PibValue_t *)&(ctx->csmaBackoffs));
	PHY_PibSet(macMaxFrameRetries, (PibValue_t *)&(ctx->maxRetries));

	gctx.txBuffer[0] = buffLen + IEEE802154_PHY_FCS_LENGTH;
	memcpy(&(gctx.txBuffer[1]), frag->data, buffLen);
	txFrame.mpdu = &(gctx.txBuffer[0]);

	uint8_t headerLen = parseFrame(txFrame.mpdu, &txFrameInfo);

	/* Handle 2015 frame */
	if (is2015FrameVersion(txFrameInfo.fcf.value)) {
		is2015Frame = true;
#if defined(CONFIG_OPENTHREAD_CSL_RECEIVER)
		if (!net_pkt_ieee802154_mac_hdr_rdy(pkt) && (gctx.csl_period > 0)) {

			uint16_t cslPhaseNow = getCslPhase();

			struct ieee802154_header_ie header_ie =
				IEEE802154_DEFINE_HEADER_IE_CSL_REDUCED(cslPhaseNow,
									gctx.csl_period);

			memcpy(&txFrame.mpdu[headerLen + 1], &header_ie, 6);
			headerLen += 8;
		}
#endif
	}

#if defined(CONFIG_IEEE802154_PIC32CX_BZ_CRYPTO)

	if (!net_pkt_ieee802154_frame_secured(pkt) && (txFrameInfo.fcf.security_enabled) &&
	    (txFrameInfo.securityControl.keyIdMode == 0x01)) {

		struct ieee802154_local_key *lkey = pic32cx_bz_get_curr_key();
		uint32_t fc_now = gctx.frame_counter++;
		uint8_t keyIdnow = lkey->key_id;
		uint8_t frameLen = txFrame.mpdu[0];

		memcpy(&txFrame.mpdu[txFrameInfo.frameCounter_offset], &(fc_now), 4);
		memcpy((uint8_t *)&frag->data[txFrameInfo.frameCounter_offset - 1], &(fc_now), 4);
		memcpy(&txFrame.mpdu[txFrameInfo.keyId_offset], &(keyIdnow), 1);
		memcpy((uint8_t *)&frag->data[txFrameInfo.keyId_offset - 1], &(keyIdnow), 1);

		/* Secure the Enhanced ACK if the packet
		 * being received is Security enabled at MAC level
		 */
		processTxAckSecurity(dev, (uint8_t *)&txFrame.mpdu[1], frameLen, headerLen, lkey,
				     fc_now);

		net_pkt_set_ieee802154_frame_secured(pkt, true);
	}
#endif

	if (is2015Frame) {
		tx_status = PHY_Tx2015Frame(&txFrame, ctx->csmaMode, (ctx->maxRetries != 0));
	} else {
		tx_status = PHY_TxFrame(&txFrame, ctx->csmaMode, (ctx->maxRetries != 0));
	}

	if (tx_status != PHY_SUCCESS) {
		goto update;
	}
	/* Wait for the TX callback */
	k_sem_take(&(gctx.txDone_fifo), K_FOREVER);

update:
	switch (tx_status) {
	case PHY_FAILURE:
		status = -EIO;
		break;
	case PHY_FRAME_PENDING:
	case PHY_SUCCESS: {
		if (isACkRequested(txFrameInfo.fcf.value)) {
			status = pic32cx_bz_handle_ack(ctx, frag->data[2], tx_status, is2015Frame);
		} else {
			status = 0;
		}
	} break;
	case PHY_CHANNEL_ACCESS_FAILURE:
		status = -EBUSY;
		break;
	case PHY_NO_ACK:
		status = -ENOMSG;
		break;
	default:
		status = -EIO;
		break;
	}

	tx_status = PHY_FAILURE;
	LOG_DBG("TX status %d", status);
	headerLen = 0;
	return status;
}

void start_ed_scan(struct k_work *ed_job)
{

	uint8_t edLevel;
	int8_t pwrDbm = INT8_MIN;
	uint32_t edEndtime;
	uint32_t duration = gctx.ed_duration;
	energy_scan_done_cb_t done_cb = gctx.ed_done_cb;

	edEndtime = k_ticks_to_ms_ceil32(k_uptime_ticks()) + (uint32_t)duration;

	do {
		edLevel = PHY_EdSample();
		pwrDbm = (pwrDbm > (int8_t)(edLevel + PHY_GetRSSIBaseVal()))
				 ? pwrDbm
				 : (int8_t)(edLevel + PHY_GetRSSIBaseVal());
	} while (edEndtime > k_ticks_to_ms_ceil32(k_uptime_ticks()));

	done_cb(get_dev(), (int16_t)pwrDbm);
}

static int pic32cx_bz_ed_scan(const struct device *dev, uint16_t duration,
			      energy_scan_done_cb_t done_cb)
{
	int status = 0;

	gctx.ed_done_cb = done_cb;
	gctx.ed_duration = duration;

	static K_WORK_DEFINE(ed_scan, start_ed_scan);

	if (!k_work_is_pending(&ed_scan)) {

		k_work_submit_to_queue(&iface_work_q, &ed_scan);
		status = 0;
	} else {
		status = -EBUSY;
	}

	return status;
}

/* Get Radio Clock Accuracy */

static uint8_t pic32cx_bz_get_acc(const struct device *dev)
{
	ARG_UNUSED(dev);
	return (XTAL_ACCURACY / 2);
}

static int pic32cx_bz_configure(const struct device *dev, enum ieee802154_config_type type,
				const struct ieee802154_config *config)
{
	int status = -EINVAL;
	PHY_Retval_t phy_status = PHY_FAILURE;
	struct pic32cx_bz_context *ctx = dev->data;

	LOG_DBG("154 Radio: Configure %d", type);

	switch (type) {
	case IEEE802154_CONFIG_AUTO_ACK_FPB: {
		gctx.auto_fpb = config->auto_ack_fpb.enabled;
		gctx.fpb_mode = config->auto_ack_fpb.mode;
		LOG_DBG("154 Radio: AUTO FPB %d and FPB mode %d", gctx.auto_fpb, gctx.fpb_mode);
	} break;
	case IEEE802154_CONFIG_ACK_FPB: {
		if (config->ack_fpb.enabled) {
			if (config->ack_fpb.extended) {
				uint8_t eAddr[8];

				memcpy(&eAddr, config->ack_fpb.addr, 8);
				status = addExtAddrEntry(eAddr);
			} else {
				uint16_t sAddr;

				memcpy(&sAddr, config->ack_fpb.addr, sizeof(uint16_t));
				status = addShrtAddrEntry(sAddr);
			}
		} else {
			if (config->ack_fpb.extended) {
				uint8_t eAddr[8];

				sys_memcpy_swap(eAddr, config->ack_fpb.addr, 8);
				status = rmvExtEntry(eAddr);
			} else {
				uint16_t sAddr;

				memcpy(&sAddr, config->ack_fpb.addr, sizeof(uint16_t));
				status = rmvShrtEntry(sAddr);
			}
		}
	} break;

	case IEEE802154_CONFIG_PAN_COORDINATOR: {
		ctx->iam_pan_coord = config->pan_coordinator;
		phy_status =
			PHY_PibSet(mac_i_pan_coordinator, (PibValue_t *)&(config->pan_coordinator));
		if (phy_status == PHY_SUCCESS) {
			status = 0;
		}
	} break;

	case IEEE802154_CONFIG_PROMISCUOUS: {
		phy_status = PHY_ConfigRxPromiscuousMode(config->promiscuous);
		if (phy_status == PHY_SUCCESS) {
			status = 0;
		}
	} break;
#if defined(CONFIG_IEEE802154_PIC32CX_BZ_CRYPTO)
	case IEEE802154_CONFIG_MAC_KEYS: {
		pic32cx_bz_set_mac_keys(config->mac_keys);
		status = 0;
	} break;

	case IEEE802154_CONFIG_FRAME_COUNTER: {
		pic32cx_bz_set_frame_counter(config->frame_counter);
		status = 0;
	} break;

	case IEEE802154_CONFIG_FRAME_COUNTER_IF_LARGER: {
		pic32cx_bz_set_frame_counter_if_larger(config->frame_counter);
		status = 0;
	} break;
#endif /* CONFIG_IEEE802154_PIC32CX_BZ_CRYPTO */
	case IEEE802154_CONFIG_ENH_ACK_HEADER_IE: {
		uint8_t ext_addr_ie[8];
		uint8_t short_addr_ie[2];
		uint8_t rec_elmnt_id;
		bool is_valid_ven_ie = false;

		if (config->ack_ie.purge_ie) {
			break;
		}

		if (config->ack_ie.short_addr == IEEE802154_BROADCAST_ADDRESS ||
		    config->ack_ie.ext_addr == NULL) {
			return -ENOTSUP;
		}

		sys_put_le16(config->ack_ie.short_addr, short_addr_ie);
		sys_memcpy_swap(ext_addr_ie, config->ack_ie.ext_addr, 8);

		if (config->ack_ie.header_ie == NULL || config->ack_ie.header_ie->length == 0) {
			if (config->ack_ie.short_addr != IEEE802154_NO_SHORT_ADDRESS_ASSIGNED) {
				clear_short_ie(short_addr_ie);
			}
			clear_ext_ie(ext_addr_ie);
		} else {
			rec_elmnt_id =
				ieee802154_header_ie_get_element_id(config->ack_ie.header_ie);

#if defined(CONFIG_NET_L2_OPENTHREAD)
			uint8_t ot_vendor_oui[IEEE802154_OPENTHREAD_VENDOR_OUI_LEN] =
				IEEE802154_OPENTHREAD_THREAD_IE_VENDOR_OUI;

			if (rec_elmnt_id == IEEE802154_HEADER_IE_ELEMENT_ID_VENDOR_SPECIFIC_IE &&
			    memcmp(config->ack_ie.header_ie->content.vendor_specific.vendor_oui,
				   ot_vendor_oui, sizeof(ot_vendor_oui)) == 0) {
				is_valid_ven_ie = true;
			}
#endif

			if (rec_elmnt_id != IEEE802154_HEADER_IE_ELEMENT_ID_CSL_IE &&
			    !is_valid_ven_ie) {
				return -ENOTSUP;
			}

#if defined(CONFIG_NET_L2_OPENTHREAD)
			if (rec_elmnt_id == IEEE802154_HEADER_IE_ELEMENT_ID_VENDOR_SPECIFIC_IE) {
				if (config->ack_ie.short_addr !=
				    IEEE802154_NO_SHORT_ADDRESS_ASSIGNED) {
					add_or_update_short_ie(
						short_addr_ie, (uint8_t *)config->ack_ie.header_ie,
						config->ack_ie.header_ie->length +
							IEEE802154_HEADER_IE_HEADER_LENGTH);
				}
				add_or_update_ext_ie(ext_addr_ie,
						     (uint8_t *)config->ack_ie.header_ie,
						     config->ack_ie.header_ie->length +
							     IEEE802154_HEADER_IE_HEADER_LENGTH);
			}
#endif
		}

		status = 0;
	} break;
#if defined(CONFIG_OPENTHREAD_CSL_RECEIVER)
	case IEEE802154_CONFIG_EXPECTED_RX_TIME: {
		pic32cx_bz_set_csl_sample_time(config->expected_rx_time);
		status = 0;
	} break;
	case IEEE802154_CONFIG_RX_SLOT: /* No support for ReceiveAt function in PHY */
		break;
	case IEEE802154_CONFIG_CSL_PERIOD: {
		pic32cx_bz_set_csl_period(config->csl_period);
		status = 0;
	} break;
#endif
	case IEEE802154_CONFIG_EVENT_HANDLER:
		break;
	case IEEE802154_CONFIG_RX_ON_WHEN_IDLE:
		break;
	default:
		break;
	}

	return status;
}

/* Task handler */
static void pic32cx_bz_task_handler(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct pic32cx_bz_context *ctx = p1;

	while (1) {
		k_sem_take(&ctx->semPhyInternalHandler, K_FOREVER);
		PHY_TaskHandler();
	}
}

/* RF Interface init */

static void pic32cx_bz_iface_init(struct net_if *iface)
{
	LOG_DBG("154 Radio: RF Interface init");
	const struct device *dev = net_if_get_device(iface);
	struct pic32cx_bz_context *ctx = dev->data;
	uint8_t *mac = get_mac(dev);

	ctx->iface = iface;

	net_if_set_link_addr(iface, mac, 8, NET_LINK_IEEE802154);

	ieee802154_init(iface);
}

/* Get HW capabilities */

static enum ieee802154_hw_caps pic32cx_bz_get_capabilities(const struct device *dev)
{
	LOG_DBG("154 Radio: Get HW capabilities");

	struct pic32cx_bz_context *ctx = dev->data;

	return ctx->hw_capabilities;
}

static int pic32cx_bz_init(const struct device *dev)
{

	struct pic32cx_bz_context *ctx = PIC32CX_BZ_802154_DATA(dev);
	PibValue_t pibValue;

	LOG_DBG("154 Radio: Initialize PIC32CX_BZ 15.4 PHY\n");

	/* set zb_en_main_clk[4] */
	BTZBSYS_REGS->BTZBSYS_SUBSYS_CNTRL_REG0 |= BTZBSYS_SUBSYS_CNTRL_REG0_zb_en_main_clk(1U);

	SYS_Load_Cal(WSS_ENABLE_ZB);

	PHY_Init();

	pibValue.pib_value_8bit = 3;
	PHY_PibSet(phyCCAMode, &pibValue);

	/* Initialize PHY data config */
	memset(&gctx, 0, sizeof(struct pic32cx_bz_context));
	gctx.auto_fpb = false;
	/* This is for default MAC FPB mode, which is set FPB to always '1' */
	gctx.fpb_mode = 0xFF;
#if defined(CONFIG_NET_L2_OPENTHREAD) && defined(CONFIG_OPENTHREAD_FTD)
	/* Enable Enhanced Frame Pending */
	PHY_EnableEnhancedFramepending(true);
#endif

	IRQ_CONNECT(ZB_INT0_IRQn, 1, ZB_INT0_Handler, NULL, 0);
	IRQ_CONNECT(ARBITER_IRQn, 1, ARBITER_Handler, NULL, 0);

#if defined(CONFIG_IEEE802154_RAW_MODE)
	gctx.dev = dev;
#endif
	LOG_DBG("154 Radio:  Init Done");
	/* HW Capabilities */
	ctx->hw_capabilities = IEEE802154_HW_ENERGY_SCAN | IEEE802154_HW_FCS |
			       IEEE802154_HW_PROMISC | IEEE802154_HW_FILTER | IEEE802154_HW_CSMA |
			       IEEE802154_HW_RETRANSMISSION | IEEE802154_HW_TX_RX_ACK |
			       IEEE802154_HW_RX_TX_ACK |
#if defined(CONFIG_IEEE802154_PIC32CX_BZ_CRYPTO)
			       IEEE802154_HW_TX_SEC |
#endif
			       IEEE802154_RX_ON_WHEN_IDLE;

	/* Create a FIFO queue for TX done handler */
	k_sem_init(&ctx->semPhyInternalHandler, 0, 20);
	k_sem_init(&ctx->txDone_fifo, 0, 20);
	gctx.semPhyInternalHandler = ctx->semPhyInternalHandler;
	gctx.txDone_fifo = ctx->txDone_fifo;

	/* 4. Create Thread and register Thread name */
	k_thread_create(&ctx->trx_thread, ctx->trx_stack,
			CONFIG_IEEE802154_PIC32CX_BZ_RX_STACK_SIZE, pic32cx_bz_task_handler, ctx,
			NULL, NULL, K_PRIO_COOP(5), 0, K_NO_WAIT);

	k_thread_name_set(&ctx->trx_thread, "pic32cx_bz_trx");

	LOG_DBG("nRF5 802154 radio initialized");
#if defined(CONFIG_IEEE802154_PIC32CX_BZ_CRYPTO)
	SAL_AesInit(&aes);
#endif

	k_work_queue_start(&iface_work_q, ieee802154_radio_stack,
			   K_KERNEL_STACK_SIZEOF(ieee802154_radio_stack), K_PRIO_COOP(5), NULL);

	return 0;
}

/* IEEE802154 driver APIs structure */
static const struct ieee802154_radio_api pic32cx_bz_radio_api = {
	.iface_api.init = pic32cx_bz_iface_init,
	.get_capabilities = pic32cx_bz_get_capabilities,
	.cca = pic32cx_bz_cca,
	.set_channel = pic32cx_bz_set_channel,
	.filter = pic32cx_bz_filter,
	.set_txpower = pic32cx_bz_set_txpower,
	.start = pic32cx_bz_start,
	.stop = pic32cx_bz_stop,
#if defined(CONFIG_IEEE802154_PIC32CX_BZ_CARRIER_FUNCTIONS)
	.continuous_carrier = pic32cx_bz_continuous_carrier,
#endif
	.tx = pic32cx_bz_tx,
	.ed_scan = pic32cx_bz_ed_scan,
	.get_time = pic32cx_bz_get_time,
	.get_sch_acc = pic32cx_bz_get_acc,
	.configure = pic32cx_bz_configure,
	.attr_get = pic32cx_bz_attr_get};

#if defined(CONFIG_NET_L2_IEEE802154)
#define L2          IEEE802154_L2
#define L2_CTX_TYPE NET_L2_GET_CTX_TYPE(IEEE802154_L2)
#define MTU         127
#elif defined(CONFIG_NET_L2_OPENTHREAD)
#define L2          OPENTHREAD_L2
#define L2_CTX_TYPE NET_L2_GET_CTX_TYPE(OPENTHREAD_L2)
#define MTU         1280
#endif

#if defined(CONFIG_IEEE802154_RAW_MODE)
DEVICE_DT_INST_DEFINE(0, pic32cx_bz_init, NULL, &gctx, NULL, POST_KERNEL,
		      CONFIG_IEEE802154_PIC32CX_BZ_INIT_PRIO, &pic32cx_bz_radio_api);
#else
NET_DEVICE_DT_INST_DEFINE(0, pic32cx_bz_init, NULL, &gctx, NULL,
			  CONFIG_IEEE802154_PIC32CX_BZ_INIT_PRIO, &pic32cx_bz_radio_api, L2,
			  L2_CTX_TYPE, MTU);
#endif
