/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/uart.h>
#include <zephyr/ztest.h>

#if DT_NODE_EXISTS(DT_NODELABEL(dut))
#define UART_NODE DT_NODELABEL(dut)
#else
#define UART_NODE DT_CHOSEN(zephyr_console)
#endif

#define SLEEP_TIME_US   1000
#define TEST_BUFFER_LEN 10

static const struct device *const uart_dev = DEVICE_DT_GET(UART_NODE);

const uint8_t test_pattern[TEST_BUFFER_LEN] = {0x11, 0x12, 0x13, 0x14, 0x15,
					       0x16, 0x17, 0x18, 0x19, 0x20};
static uint8_t test_buffer[TEST_BUFFER_LEN];
static volatile uint8_t uart_error_counter;

/*
 * ISR for UART TX action
 */
static void uart_tx_interrupt_service(const struct device *dev, int *tx_byte_offset)
{
	uint8_t bytes_sent = 0;
	uint8_t *tx_data_pointer = (uint8_t *)(test_pattern + *tx_byte_offset);

	if (*tx_byte_offset < TEST_BUFFER_LEN) {
		bytes_sent = uart_fifo_fill(dev, tx_data_pointer, 1);
		*tx_byte_offset += bytes_sent;
	} else {
		*tx_byte_offset = 0;
		uart_irq_tx_disable(dev);
	}
}

/*
 * ISR for UART RX action
 */
static void uart_rx_interrupt_service(const struct device *dev, uint8_t *receive_buffer_pointer,
				      int *rx_byte_offset)
{
	int rx_data_length = 0;

	do {
		rx_data_length = uart_fifo_read(dev, receive_buffer_pointer + *rx_byte_offset,
						TEST_BUFFER_LEN);
		*rx_byte_offset += rx_data_length;
	} while (rx_data_length);
}

/*
 * Callback function for MAIN UART interrupt based transmission test
 */
static void interrupt_driven_uart_callback_main_uart(const struct device *dev, void *user_data)
{
	int err;
	static int tx_byte_offset;
	static int rx_byte_offset;

	uart_irq_update(dev);
	err = uart_err_check(dev);
	if (err != 0) {
		uart_error_counter++;
	}
	while (uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			if (rx_byte_offset >= TEST_BUFFER_LEN) {
				rx_byte_offset = 0;
			}
			uart_rx_interrupt_service(dev, (uint8_t *)user_data, &rx_byte_offset);
		}
		if (uart_irq_tx_ready(dev)) {
			uart_tx_interrupt_service(dev, &tx_byte_offset);
		}
	}
}

/*
 * Test UART proper configuration call
 */
ZTEST(uart_elementary, test_uart_proper_configuration)
{
	Z_TEST_SKIP_IFDEF(CONFIG_DUAL_UART_TEST);

	int err;
	struct uart_config test_expected_uart_config;
	struct uart_config test_uart_config = {.baudrate = 115200,
					       .parity = UART_CFG_PARITY_NONE,
					       .stop_bits = UART_CFG_STOP_BITS_1,
					       .data_bits = UART_CFG_DATA_BITS_8,
					       .flow_ctrl = UART_CFG_FLOW_CTRL_NONE};

	err = uart_configure(uart_dev, &test_uart_config);
	zassert_equal(err, 0, "'uart_configure' api call - unexpected error: %d", err);

	err = uart_config_get(uart_dev, &test_expected_uart_config);
	zassert_equal(err, 0, "'uart_config_get' api call - unexpected error raised : %d", err);

	zassert_equal(test_uart_config.baudrate, test_expected_uart_config.baudrate,
		      "Set and actual UART config baudrate mismatch: %d != %d",
		      test_uart_config.baudrate, test_expected_uart_config.baudrate);

	zassert_equal(test_uart_config.parity, test_expected_uart_config.parity,
		      "Set and actual UART config parity mismatch: %d != %d",
		      test_uart_config.parity, test_expected_uart_config.parity);

	zassert_equal(test_uart_config.stop_bits, test_expected_uart_config.stop_bits,
		      "Set and actual UART config stop_bits mismatch: %d != %d",
		      test_uart_config.stop_bits, test_expected_uart_config.stop_bits);

	zassert_equal(test_uart_config.data_bits, test_expected_uart_config.data_bits,
		      "Set and actual UART config data_bits mismatch: %d != %d",
		      test_uart_config.data_bits, test_expected_uart_config.data_bits);

	zassert_equal(test_uart_config.flow_ctrl, test_expected_uart_config.flow_ctrl,
		      "Set and actual UART config flow_ctrl mismatch: %d != %d",
		      test_uart_config.flow_ctrl, test_expected_uart_config.flow_ctrl);
}

/*
 * Test UART improper configuration call
 */
ZTEST(uart_elementary, test_uart_improper_configuration)
{
	Z_TEST_SKIP_IFDEF(CONFIG_DUAL_UART_TEST);

	int err;
	struct uart_config test_uart_config = {.baudrate = 115200,
					       .parity = 7,
					       .stop_bits = UART_CFG_STOP_BITS_1,
					       .data_bits = UART_CFG_DATA_BITS_8,
					       .flow_ctrl = UART_CFG_FLOW_CTRL_NONE};

	err = uart_configure(uart_dev, &test_uart_config);
	zassert_not_equal(
		err, 0,
		"'uart_configure' with incorrect configuration havent't raised an error, err=%d",
		err);
}

/*
 * Test UART basic interrupt based transmission (with loopback)
 */
ZTEST(uart_elementary, test_uart_basic_transmission)
{
	int err;
	struct uart_config test_uart_config = {.baudrate = 115200,
					       .parity = UART_CFG_PARITY_ODD,
					       .stop_bits = UART_CFG_STOP_BITS_1,
					       .data_bits = UART_CFG_DATA_BITS_8,
					       .flow_ctrl = UART_CFG_FLOW_CTRL_NONE};

	err = uart_configure(uart_dev, &test_uart_config);
	zassert_equal(err, 0, "Unexpected error when configuring UART0: %d", err);
	err = uart_irq_callback_user_data_set(uart_dev, interrupt_driven_uart_callback_main_uart,
					      (void *)test_buffer);
	zassert_equal(err, 0, "Unexpected error when setting user data for callback %d", err);
	uart_irq_err_enable(uart_dev);
	uart_irq_rx_enable(uart_dev);
	uart_irq_tx_enable(uart_dev);

	/* wait for the tramission to finish (no polling is intentional) */
	k_sleep(K_USEC(100 * SLEEP_TIME_US));

	uart_irq_tx_disable(uart_dev);
	uart_irq_rx_disable(uart_dev);
	uart_irq_err_disable(uart_dev);
	for (int index = 0; index < TEST_BUFFER_LEN; index++) {
		zassert_equal(test_buffer[index], test_pattern[index],
			      "Received data byte %d does not match pattern 0x%x != 0x%x", index,
			      test_buffer[index], test_pattern[index]);
	}
}

/*
 * Test setup
 */
void *test_setup(void)
{
	zassert_true(device_is_ready(uart_dev), "UART0 device is not ready");
#if defined(CONFIG_DUAL_UART_TEST)
	zassert_true(device_is_ready(uart_dev_aux), "UART1 device is not ready");
#endif

	return NULL;
}

ZTEST_SUITE(uart_elementary, NULL, test_setup, NULL, NULL, NULL);
