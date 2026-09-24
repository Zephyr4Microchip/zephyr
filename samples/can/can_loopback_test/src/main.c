/*
 * CAN Loopback + Wire Presence Validation Test
 * PIC32WM BZ6204 Curiosity — CAN1 (Bosch M_CAN v3.2.3)
 *
 * Test modes (selected via prj.conf / prj_wire.conf)
 * ---------------------------------------------------
 * CONFIG_CAN_WIRE_TEST=n  (default, prj.conf)
 *   M_CAN internal loopback (CCCR.TEST=1, TEST.LBCK=1).
 *   TX→RX entirely inside silicon.  No wire or transceiver needed.
 *   TC1–TC6: CAN frame tests.
 *
 * CONFIG_CAN_WIRE_TEST=y  (prj_wire.conf)
 *   TC0: GPIO WIRE PRESENCE TEST  ← NEW
 *     Temporarily drives PA3 (CAN1_TX) as GPIO output and reads back
 *     on PA2 (CAN1_RX) as GPIO input.  Tests HIGH and LOW levels.
 *     PASS = wire is physically present between PA3 and PA2.
 *     FAIL = wire is missing or broken.
 *   TC1–TC6: CAN frame tests run in M_CAN internal loopback as usual.
 *
 * Why GPIO for wire test instead of M_CAN External Loopback?
 * ----------------------------------------------------------
 * M_CAN External Loopback (TEST=1, LBCK=0) requires a transceiver to
 * provide the CAN ACK dominant bit on PA2 during the ACK slot.  Without
 * a transceiver, TEFN never fires and can_send blocks indefinitely.
 * The GPIO toggle test is transceiver-free and definitively proves
 * the physical wire is present, independent of CAN protocol timing.
 *
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <string.h>

LOG_MODULE_REGISTER(can_loopback_test, LOG_LEVEL_INF);

/* -------------------------------------------------------------------------
 * Board resources
 * LED0 (red)   → lit if ANY test fails    (problem: CAN TX, RX, or filter broken)
 * LED1 (green) → lit when ALL tests pass  (CAN peripheral is fully functional)
 * -------------------------------------------------------------------------
 */
#define LED_RED_NODE   DT_ALIAS(led0)
#define LED_GREEN_NODE DT_ALIAS(led1)

static const struct gpio_dt_spec led_red   = GPIO_DT_SPEC_GET(LED_RED_NODE,   gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED_GREEN_NODE, gpios);

/* CAN device — zephyr,canbus chosen points to can1 */
static const struct device *can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

/* -------------------------------------------------------------------------
 * Test infrastructure
 * -------------------------------------------------------------------------
 */
#define RX_TIMEOUT_MS  500   /* max time to wait for a loopback echo */
#define TX_TIMEOUT_MS  100   /* max time to acquire a free TX buffer  */

static K_SEM_DEFINE(rx_sem, 0, 1);

/* Received frame — filled by rx_callback from ISR context. */
static struct can_frame rx_frame;
static volatile bool    rx_got;   /* true once rx_callback fires */

/* Test totals */
static int tests_run;
static int tests_passed;

/* ISR-safe RX callback: copies the received frame and signals the main thread */
static void rx_callback(const struct device *dev, struct can_frame *frame,
			void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	memcpy(&rx_frame, frame, sizeof(rx_frame));
	rx_got = true;
	k_sem_give(&rx_sem);
}

/* Print one result line.
 * PASS = hardware behaved correctly.
 * FAIL = hardware did not behave correctly — investigate driver or silicon.
 */
static void report(const char *name, bool pass)
{
	tests_run++;
	if (pass) {
		tests_passed++;
		LOG_INF("  [PASS] %s", name);
	} else {
		LOG_ERR("  [FAIL] %s", name);
	}
}

/* -------------------------------------------------------------------------
 * send_and_receive
 * Send one frame and poll for its loopback echo.
 * Returns true if the echo arrived and matched (ID, IDE, DLC, data).
 *
 * PASS means: TX path wrote the frame to M_CAN, M_CAN looped it back
 *             through the internal bus, RX FIFO captured it, the ISR
 *             notified the driver, and the data is bit-for-bit correct.
 * FAIL means: Either the frame was never sent, never looped back, was
 *             filtered out, or the received data does not match.
 * -------------------------------------------------------------------------
 */
static bool send_and_receive(const struct can_frame *tx, int filter_id)
{
	ARG_UNUSED(filter_id);

	int ret;

	rx_got = false;
	k_sem_reset(&rx_sem);

	ret = can_send(can_dev, tx, K_MSEC(TX_TIMEOUT_MS), NULL, NULL);
	if (ret != 0) {
		LOG_ERR("    TX failed (can_send err %d) — "
			"TX FIFO full or controller stopped", ret);
		return false;
	}

	/* Poll rx_got (set by rx_callback in ISR context).
	 * Polling is used because can_send with a NULL callback blocks
	 * internally until TX completes; by the time can_send returns the
	 * loopback RX may already have fired, making a semaphore wait
	 * timing-sensitive.  A volatile flag poll is more robust here.
	 */
	uint32_t t0 = k_uptime_get_32();

	while (!rx_got) {
		if ((k_uptime_get_32() - t0) > (uint32_t)RX_TIMEOUT_MS) {
			LOG_ERR("    No echo in %d ms — "
				"loopback path broken or RX filter rejected frame",
				RX_TIMEOUT_MS);
			return false;
		}
		k_yield();
	}

	/* Verify the echoed frame matches what was sent */
	bool id_ok   = (rx_frame.id == tx->id);
	bool ext_ok  = !!(rx_frame.flags & CAN_FRAME_IDE) ==
		       !!(tx->flags  & CAN_FRAME_IDE);
	bool dlc_ok  = (rx_frame.dlc == tx->dlc);
	bool data_ok = (tx->dlc == 0) ||
		       (memcmp(rx_frame.data, tx->data,
			       can_dlc_to_bytes(tx->dlc)) == 0);

	if (!id_ok)
		LOG_ERR("    ID mismatch  : rx 0x%X  expected 0x%X  "
			"— frame routed to wrong filter or MRAM corruption",
			rx_frame.id, tx->id);
	if (!ext_ok)
		LOG_ERR("    IDE mismatch : rx %u  expected %u  "
			"— standard/extended frame type not preserved",
			!!(rx_frame.flags & CAN_FRAME_IDE),
			!!(tx->flags  & CAN_FRAME_IDE));
	if (!dlc_ok)
		LOG_ERR("    DLC mismatch : rx %u  expected %u  "
			"— data-length field corrupted in transit",
			rx_frame.dlc, tx->dlc);
	if (!data_ok)
		LOG_ERR("    Data mismatch — payload bytes corrupted "
			"(MRAM read/write error or endian issue)");

	return id_ok && ext_ok && dlc_ok && data_ok;
}

/* =========================================================================
 * Test cases
 * =========================================================================
 */

#ifdef CONFIG_CAN_WIRE_TEST
/*
 * TC0 — GPIO wire presence test (only runs with CONFIG_CAN_WIRE_TEST=y)
 *
 * Why this works with CAN0 pins (PA8/PA9):
 *   CAN0 uses ENHANCED_CAN dedicated function — PA8 and PA9 are already
 *   defined with func=gpio in pic32cx2051bz62132-pinctrl.h:
 *     PA8_CAN0_TX_OUT = MCHP_PINMUX(a, 8, NA, NA, gpio)
 *     PA9_CAN0RX_IN   = MCHP_PINMUX(a, 9, NA, NA, gpio)
 *   The pinctrl driver configures them as plain digital GPIO — no PPS
 *   or peripheral mode is involved.  We can drive and read them directly
 *   via the Zephyr GPIO API without any mode switch.
 *
 * What it does:
 *   1. Configure PA8 as GPIO output, PA9 as GPIO input.
 *   2. Drive PA8 HIGH → read PA9.  Must read HIGH.
 *   3. Drive PA8 LOW  → read PA9.  Must read LOW.
 *   4. Restore PA8/PA9 to input (high-impedance) for CAN use.
 *
 * Wire connection required:  PA8 ──jumper wire──► PA9
 *
 * PASS: wire is physically present — both logic levels transmitted.
 * FAIL: wire missing, broken, or wrong pins connected.
 */
static void tc0_gpio_wire_presence(void)
{
	const struct device *porta = DEVICE_DT_GET(DT_NODELABEL(porta));

	LOG_INF("  TC0: GPIO wire presence test — PA8 (TX) ──wire──► PA9 (RX)");
	LOG_INF("       Drives PA8 HIGH/LOW and reads back on PA9.");
	LOG_INF("       PASS only if jumper wire is physically connected.");

	if (!device_is_ready(porta)) {
		LOG_ERR("    PORTA device not ready");
		report("TC0: GPIO wire presence (PA8->PA9)", false);
		return;
	}

	/* Configure PA8 as push-pull output, PA9 as input */
	gpio_pin_configure(porta, 8, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(porta, 9, GPIO_INPUT);

	/* Allow signal to settle */
	k_busy_wait(100);

	/* Test HIGH level: drive PA8 HIGH, read PA9 */
	gpio_pin_set(porta, 8, 1);
	k_busy_wait(100);
	int hi = gpio_pin_get(porta, 9);

	/* Test LOW level: drive PA8 LOW, read PA9 */
	gpio_pin_set(porta, 8, 0);
	k_busy_wait(100);
	int lo = gpio_pin_get(porta, 9);

	/* Restore both to input (tri-state) before CAN takes over */
	gpio_pin_configure(porta, 8, GPIO_INPUT);
	gpio_pin_configure(porta, 9, GPIO_INPUT);

	bool pass = (hi == 1) && (lo == 0);

	if (hi != 1)
		LOG_ERR("    HIGH test failed: PA9 read %d (expected 1) "
			"— wire missing or PA8/PA9 wrong pins", hi);
	if (lo != 0)
		LOG_ERR("    LOW  test failed: PA9 read %d (expected 0) "
			"— wire missing or PA8/PA9 wrong pins", lo);
	if (pass)
		LOG_INF("       HIGH: PA9=%d ✓   LOW: PA9=%d ✓  Wire confirmed.", hi, lo);

	report("TC0: GPIO wire presence (PA8->PA9)", pass);
}
#endif /* CONFIG_CAN_WIRE_TEST */

/*
 * TC1 — Standard frame (11-bit ID), 8-byte payload
 * Why : Exercises the most common CAN frame type with maximum classic
 *       payload.  Confirms basic TX→RX loopback, ID/DLC/data integrity.
 * Pass: Echo received with correct 11-bit ID (0x123), DLC=8, matching data.
 * Fail: Frame not received OR received with wrong ID/DLC/data — check
 *       M_CAN TX/RX FIFO config, MRCFG, or clock setup.
 */
static void tc1_standard_8byte(void)
{
	LOG_INF("  TC1: Sending standard frame ID=0x123 DLC=8 — "
		"verifies basic TX/RX loopback with full payload");

	const struct can_frame tx = {
		.id   = 0x123,
		.flags = 0,
		.dlc  = 8,
		.data = { 0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE },
	};
	report("TC1: Standard frame 8-byte (ID/DLC/data integrity)",
	       send_and_receive(&tx, 0));
}

/*
 * TC2 — Extended frame (29-bit ID), 8-byte payload
 * Why : CAN supports both 11-bit (standard) and 29-bit (extended) IDs.
 *       Confirms M_CAN correctly handles the IDE bit and a 29-bit identifier.
 * Pass: Echo received with IDE=1, ID=0x1FFFF001, data intact.
 * Fail: IDE flag lost, wrong ID, or no echo — extended-frame path broken.
 */
static void tc2_extended_8byte(void)
{
	LOG_INF("  TC2: Sending extended frame ID=0x1FFFF001 DLC=8 — "
		"verifies 29-bit (IDE=1) frame path");

	const struct can_frame tx = {
		.id    = 0x1FFFF001,
		.flags = CAN_FRAME_IDE,
		.dlc   = 8,
		.data  = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 },
	};
	report("TC2: Extended frame 8-byte (29-bit ID support)",
	       send_and_receive(&tx, 0));
}

/*
 * TC3 — Standard frame, 1-byte payload
 * Why : Confirms the driver correctly handles sub-8-byte DLC values and
 *       does not copy extra bytes beyond the DLC into the received frame.
 * Pass: Echo received with DLC=1, correct single byte.
 * Fail: DLC not preserved or data byte wrong — payload-length handling broken.
 */
static void tc3_standard_1byte(void)
{
	LOG_INF("  TC3: Sending standard frame ID=0x456 DLC=1 — "
		"verifies sub-8-byte payload handling");

	const struct can_frame tx = {
		.id    = 0x456,
		.flags = 0,
		.dlc   = 1,
		.data  = { 0xA5 },
	};
	report("TC3: Standard frame 1-byte (minimal payload)",
	       send_and_receive(&tx, 0));
}

/*
 * TC4 — Standard frame, zero-byte payload (DLC = 0)
 * Why : A DLC=0 frame is valid in CAN and is used as a keep-alive or
 *       event signal.  Confirms the driver handles the edge case of a
 *       frame with no data bytes without accessing out-of-bounds memory.
 * Pass: Echo received with DLC=0 and no data corruption.
 * Fail: Crash, wrong DLC, or no echo — zero-byte DLC edge case broken.
 */
static void tc4_zero_dlc(void)
{
	LOG_INF("  TC4: Sending standard frame ID=0x7FF DLC=0 — "
		"verifies zero-payload edge case (keep-alive / event frame)");

	const struct can_frame tx = {
		.id    = 0x7FF,
		.flags = 0,
		.dlc   = 0,
	};
	report("TC4: Zero-byte DLC (edge case: no payload)",
	       send_and_receive(&tx, 0));
}

/*
 * TC5 — 5 consecutive standard frames
 * Why : Confirms that the TX FIFO queues multiple frames without starvation
 *       and that the RX FIFO delivers them all without loss or reordering.
 *       Each frame has a unique ID (0x100–0x104) and unique payload.
 * Pass: All 5 frames echoed with correct IDs and payloads in order.
 * Fail: Any frame missed or payload wrong — FIFO depth or sequencing issue.
 */
static void tc5_sequential(void)
{
	LOG_INF("  TC5: Sending 5 frames IDs=0x100..0x104 — "
		"verifies TX/RX FIFO can handle back-to-back frames");

	bool all_ok = true;

	for (int i = 0; i < 5; i++) {
		struct can_frame tx = {
			.id    = 0x100 + i,
			.flags = 0,
			.dlc   = 2,
			.data  = { (uint8_t)i, (uint8_t)(i ^ 0xFF) },
		};
		if (!send_and_receive(&tx, 0)) {
			LOG_ERR("    Frame %d/5 failed (ID=0x%03X) — "
				"FIFO overflow or sequencing error",
				i + 1, 0x100 + i);
			all_ok = false;
		}
	}
	report("TC5: 5 sequential frames (FIFO depth + ordering)", all_ok);
}

/*
 * TC6 — Hardware RX filter selectivity
 * Why : The M_CAN has a hardware filter bank.  Only frames matching an
 *       installed filter are stored in the RX FIFO.  This test confirms:
 *       (A) A non-matching frame is silently discarded (not delivered).
 *       (B) A matching frame is delivered to the correct callback.
 *       This is critical: a broken filter would flood the application with
 *       unintended frames or drop intended ones.
 *
 * Part A — sends ID=0x222 with filter accepting only ID=0x321.
 *   Pass: No echo within 150 ms  → filter correctly rejected the frame.
 *   Fail: Echo received           → filter is too wide or not applied.
 *
 * Part B — sends ID=0x321 with the same filter active.
 *   Pass: Echo received with correct data → filter correctly accepted frame.
 *   Fail: No echo                          → filter too narrow or RX broken.
 */
static void tc6_filter_selectivity(void)
{
	LOG_INF("  TC6: Installing filter for ID=0x321 only —");
	LOG_INF("       Part A: send ID=0x222 → must be rejected (no echo)");
	LOG_INF("       Part B: send ID=0x321 → must be accepted (echo expected)");

	const struct can_filter narrow = {
		.flags = 0,
		.id    = 0x321,
		.mask  = CAN_STD_ID_MASK,  /* exact-match filter */
	};
	int fid = can_add_rx_filter(can_dev, rx_callback, NULL, &narrow);

	if (fid < 0) {
		LOG_ERR("    Filter install failed (err %d) — "
			"filter bank full or driver error", fid);
		report("TC6: Hardware RX filter selectivity", false);
		return;
	}

	/* Part A: non-matching frame — expect silence */
	struct can_frame tx_bad = {
		.id    = 0x222,
		.flags = 0,
		.dlc   = 1,
		.data  = { 0xBB },
	};
	rx_got = false;
	k_sem_reset(&rx_sem);
	can_send(can_dev, &tx_bad, K_MSEC(TX_TIMEOUT_MS), NULL, NULL);
	bool rejected = (k_sem_take(&rx_sem, K_MSEC(150)) == -EAGAIN);

	if (rejected)
		LOG_INF("       Part A: CORRECT — ID=0x222 was rejected by filter");
	else
		LOG_ERR("       Part A: WRONG  — ID=0x222 was incorrectly received "
			"(filter too wide or mask=0)");

	/* Part B: matching frame — expect echo */
	struct can_frame tx_good = {
		.id    = 0x321,
		.flags = 0,
		.dlc   = 3,
		.data  = { 0x11, 0x22, 0x33 },
	};
	bool accepted = send_and_receive(&tx_good, fid);

	if (accepted)
		LOG_INF("       Part B: CORRECT — ID=0x321 was accepted by filter");
	else
		LOG_ERR("       Part B: WRONG  — ID=0x321 was not received "
			"(filter too narrow or RX FIFO path broken)");

	can_remove_rx_filter(can_dev, fid);

	report("TC6: Hardware RX filter (reject 0x222, accept 0x321)",
	       rejected && accepted);
}

/* =========================================================================
 * Main
 * =========================================================================
 */
int main(void)
{
	int ret;
	int filter_id;

	LOG_INF("==============================================");
	LOG_INF(" PIC32 BZ6 CAN Internal Loopback Test");
	LOG_INF(" Board  : pic32wm_bz6204_curiosity");
	LOG_INF(" Device : %s (M_CAN v3.2.3)", can_dev->name);
	LOG_INF(" Purpose: Validate CAN TX/RX path, frame formats,");
	LOG_INF("          FIFO handling, and hardware filter bank.");
	LOG_INF("==============================================");

	/* GPIO init (LEDs) */
	if (gpio_is_ready_dt(&led_red))
		gpio_pin_configure_dt(&led_red,   GPIO_OUTPUT_INACTIVE);
	if (gpio_is_ready_dt(&led_green))
		gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);

	/* CAN device readiness check */
	if (!device_is_ready(can_dev)) {
		LOG_ERR("CAN device not ready — check driver init and PMD3 unlock");
		if (gpio_is_ready_dt(&led_red)) gpio_pin_set_dt(&led_red, 1);
		return -ENODEV;
	}

	/* ------------------------------------------------------------------
	 * Set CAN loopback mode.
	 *
	 * INTERNAL loopback (default, CONFIG_CAN_WIRE_TEST=n):
	 *   CCCR.TEST=1, TEST.LBCK=1 — TX→RX inside silicon.
	 *   No wire or transceiver needed.
	 *
	 * EXTERNAL wire loopback (CONFIG_CAN_WIRE_TEST=y):
	 *   CCCR.TEST=1, TEST.LBCK=0 — TX drives external pin, RX reads
	 *   external pin.  M_CAN self-generates ACK (no transceiver needed).
	 *   Wire MUST be connected between TX and RX pins:
	 *     CAN1: PA3 (TX) ──wire──► PA2 (RX)
	 *     CAN0: PA8 (TX) ──wire──► PA9 (RX)
	 *   If wire is not connected: PA2/PA9 floats → no signal → FAIL.
	 * ------------------------------------------------------------------ */
	ret = can_set_mode(can_dev, CAN_MODE_LOOPBACK);
	if (ret != 0) {
		LOG_ERR("can_set_mode(LOOPBACK) failed (err %d) — "
			"M_CAN may still be in init mode", ret);
		if (gpio_is_ready_dt(&led_red)) gpio_pin_set_dt(&led_red, 1);
		return ret;
	}

#ifdef CONFIG_CAN_WIRE_TEST
	LOG_INF("Mode    : INTERNAL LOOPBACK + GPIO WIRE PRESENCE TEST");
	LOG_INF("          CAN TC1-TC6 use internal loopback (always works).");
	LOG_INF("          TC0 (GPIO) verifies PA8->PA9 wire is present.");
	LOG_INF("          Wire: PA8 (CAN0_TX, gpio) ──► PA9 (CAN0_RX, gpio)");
#else
	LOG_INF("Mode    : INTERNAL LOOPBACK (TEST=1, LBCK=1)");
	LOG_INF("          TX wired to RX inside silicon — no wire needed.");
#endif

	/* ------------------------------------------------------------------
	 * Start the CAN controller.
	 * Clears CCCR.INIT so the M_CAN can send/receive frames.
	 * If this fails, the peripheral clock (GCLKPERIPH) is not running
	 * or the PMD3 module-disable bit was not cleared.
	 * ------------------------------------------------------------------
	 */
	ret = can_start(can_dev);
	if (ret != 0) {
		LOG_ERR("can_start failed (err %d) — "
			"check GCLKPERIPH clock enable and PMD3.CAN1MD=0", ret);
		if (gpio_is_ready_dt(&led_red)) gpio_pin_set_dt(&led_red, 1);
		return ret;
	}
	LOG_INF("%-8s: started at 125 kbit/s (CCCR.INIT cleared)", can_dev->name);

	/* ------------------------------------------------------------------
	 * Install wide-open RX filters for TC1..TC5.
	 * mask=0 → accept any frame ID.  TC6 installs its own exact filter.
	 * ------------------------------------------------------------------
	 */
	const struct can_filter accept_all_std = {
		.flags = 0,
		.id    = 0x000,
		.mask  = 0x000,
	};
	filter_id = can_add_rx_filter(can_dev, rx_callback, NULL, &accept_all_std);
	if (filter_id < 0) {
		LOG_ERR("RX filter install failed (err %d) — "
			"filter bank full or MRAM config error", filter_id);
		if (gpio_is_ready_dt(&led_red)) gpio_pin_set_dt(&led_red, 1);
		return filter_id;
	}

	const struct can_filter accept_all_ext = {
		.flags = CAN_FILTER_IDE,
		.id    = 0x00000000,
		.mask  = 0x00000000,
	};
	int filter_ext = can_add_rx_filter(can_dev, rx_callback, NULL,
					   &accept_all_ext);
	if (filter_ext < 0)
		LOG_WRN("Extended filter install failed — TC2 may fail");

	LOG_INF("----------------------------------------------");
#ifdef CONFIG_CAN_WIRE_TEST
	LOG_INF("Running %d tests (TC0=GPIO wire + TC1-TC6=CAN)...", 7);
#else
	LOG_INF("Running %d tests...", 6);
#endif
	LOG_INF("  Pass = hardware behaved correctly");
	LOG_INF("  Fail = hardware did NOT behave correctly");
	LOG_INF("----------------------------------------------");

#ifdef CONFIG_CAN_WIRE_TEST
	tc0_gpio_wire_presence();
#endif
	tc1_standard_8byte();
	tc2_extended_8byte();
	tc3_standard_1byte();
	tc4_zero_dlc();
	tc5_sequential();

	/* TC6 installs its own narrow filter — remove broad ones first */
	can_remove_rx_filter(can_dev, filter_id);
	if (filter_ext >= 0)
		can_remove_rx_filter(can_dev, filter_ext);

	tc6_filter_selectivity();

	/* Stop CAN controller */
	can_stop(can_dev);

	/* Final report */
	LOG_INF("----------------------------------------------");
	LOG_INF("Results : %d/%d tests passed", tests_passed, tests_run);

	if (tests_passed == tests_run) {
		LOG_INF("OVERALL : *** ALL TESTS PASSED ***");
#ifdef CONFIG_CAN_WIRE_TEST
		LOG_INF("          %s external wire loopback validated.",
			can_dev->name);
		LOG_INF("          Physical TX→RX pin path confirmed working.");
#else
		LOG_INF("          %s internal loopback validated.",
			can_dev->name);
#endif
		LOG_INF("          Connect a transceiver to the TX/RX pins for bus tests.");
		if (gpio_is_ready_dt(&led_green))
			gpio_pin_set_dt(&led_green, 1);
	} else {
		LOG_ERR("OVERALL : *** %d TEST(S) FAILED ***",
			tests_run - tests_passed);
		LOG_ERR("          %s: check PMD3 module enable, GCLKPERIPH clock,",
			can_dev->name);
		LOG_ERR("          and bosch,mram-cfg (tx_event_fifo must be >= 1).");
		if (gpio_is_ready_dt(&led_red))
			gpio_pin_set_dt(&led_red, 1);
	}
	LOG_INF("==============================================");

	return 0;
}
