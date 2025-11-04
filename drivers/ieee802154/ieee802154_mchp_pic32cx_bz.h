/*
 * Copyright (c) 2025 Microchip
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* ieee802154_pic32cx_bz.h - Microchip PIC32CX_BZ IEEE 802.15.4 Driver */

#ifndef ZEPHYR_DRIVERS_IEEE802154_IEEE802154_PIC32CX_BZ_H_
#define ZEPHYR_DRIVERS_IEEE802154_IEEE802154_PIC32CX_BZ_H_

#define NUM_MAC_KEYS 3

/**
 * Key configuration for transmit security offloading, see @ref
 * IEEE802154_CONFIG_MAC_KEYS.
 */
struct ieee802154_local_key {
	/** Key material */
	uint8_t key_value[16];
	/** Initial value of frame counter associated with the key, see section 9.4.3 */
	uint32_t key_frame_counter;
	/** Indicates if per-key frame counter should be used, see section 9.4.3 */
	bool frame_counter_per_key;
	/** Key Identifier Mode, see section 9.4.2.3, Table 9-7 */
	uint8_t key_id_mode;
	/** Key Identifier, see section 9.4.4 */
	uint8_t key_id;
};

struct pic32cx_bz_context {
	struct net_if *iface;
	const struct device *dev;
	struct k_thread trx_thread;

	K_KERNEL_STACK_MEMBER(trx_stack, CONFIG_IEEE802154_PIC32CX_BZ_RX_STACK_SIZE);

	struct k_sem semPhyInternalHandler;
	struct k_sem txDone_fifo;
	/* IEEE MAC Address */
	uint8_t mac_addr[8];
	uint8_t ext_addr[8];
	/* Capabilities of the network interface. */
	enum ieee802154_hw_caps hw_capabilities;
	enum ieee802154_tx_mode tx_mode;
	uint8_t csmaBackoffs;
	uint8_t maxRetries;
	PHY_CSMAMode_t csmaMode;
	uint8_t txBuffer[LARGE_BUFFER_SIZE];
	bool iam_pan_coord;
	bool auto_fpb;
	enum ieee802154_fpb_mode fpb_mode;
	uint8_t sAckFrame[6];
	bool sAckedwithFpb;
	uint32_t frame_counter;
	uint32_t csl_period;
	net_time_t csl_sample_time;
	uint32_t sAckFrameCounter;
	uint8_t sAckKeyId;
	bool sAckedWithSecEnhAck;
	net_time_t rx_timestamp;
	energy_scan_done_cb_t ed_done_cb;
	uint32_t ed_duration;
};

#endif /* ZEPHYR_DRIVERS_IEEE802154_IEEE802154_PIC32CX_BZ_H_ */
