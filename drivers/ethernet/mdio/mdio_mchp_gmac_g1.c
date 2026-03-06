/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/net/mdio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mdio_mchp_gmac_g1, CONFIG_MDIO_LOG_LEVEL);

#define DT_DRV_COMPAT microchip_gmac_g1_mdio

#define MDIO_MCHP_CLOCK_RATE_20MHZ  (20000000U)
#define MDIO_MCHP_CLOCK_RATE_40MHZ  (40000000U)
#define MDIO_MCHP_CLOCK_RATE_80MHZ  (80000000U)
#define MDIO_MCHP_CLOCK_RATE_120MHZ (120000000U)
#define MDIO_MCHP_CLOCK_RATE_160MHZ (160000000U)
#define MDIO_MCHP_CLOCK_RATE_240MHZ (240000000U)

#define MDIO_MCHP_ESUCCESS   0
#define MDIO_MCHP_OP_TIMEOUT 25

struct mdio_clock_config {
	const struct device *clock_dev;
	clock_control_subsys_t mclk_apb_sys;
	clock_control_subsys_t mclk_ahb_sys;
};

struct mdio_dev_data {
	struct k_sem reg_sem;
};

struct mdio_dev_config {
	const struct pinctrl_dev_config *pinctrl_cfg;
	gmac_registers_t *const gmac_regs;
	struct mdio_clock_config mdio_clock_cfg;
};

/* clang-format off */
#define MDIO_MCHP_CLOCK_DEFN(n)                                                             \
	.mdio_clock_cfg.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                            \
	.mdio_clock_cfg.mclk_apb_sys = (void *)DT_INST_CLOCKS_CELL_BY_NAME(n, mclk_apb, subsystem),\
	.mdio_clock_cfg.mclk_ahb_sys = (void *)DT_INST_CLOCKS_CELL_BY_NAME(n, mclk_ahb, subsystem)
/* clang-format on */

struct mdio_config_transfer {
	enum mdio_opcode op;
	uint16_t data_in;
	uint16_t *data_out;
	uint8_t port_addr;
	uint8_t reg_addr;
	bool c45;
};

static inline int mdio_transfer(gmac_registers_t *regs, struct mdio_config_transfer *cfg)
{
	uint32_t timeout = MDIO_MCHP_OP_TIMEOUT;
	uint32_t reg_val = (cfg->c45 ? 0U : GMAC_MAN_CLTTO_Msk) | GMAC_MAN_OP(cfg->op) |
			   GMAC_MAN_WTN(0x02) | GMAC_MAN_PHYA(cfg->port_addr) |
			   GMAC_MAN_REGA(cfg->reg_addr) | GMAC_MAN_DATA(cfg->data_in);

	regs->GMAC_MAN = reg_val;
	while (!(regs->GMAC_NSR & GMAC_NSR_IDLE_Msk) && timeout > 0) {
		k_busy_wait(1); /* 1us busy wait */
		timeout--;
	}

	if ((cfg->data_out) != NULL) {
		*(cfg->data_out) = regs->GMAC_MAN & GMAC_MAN_DATA_Msk;
	}

	return MDIO_MCHP_ESUCCESS;
}

static int mdio_mchp_read(const struct device *dev, uint8_t port_addr, uint8_t reg_addr,
			  uint16_t *data)
{
	int ret;
	struct mdio_dev_data *const mdio_data = dev->data;
	const struct mdio_dev_config *const cfg = dev->config;
	struct mdio_config_transfer cfg_xfer;

	k_sem_take(&mdio_data->reg_sem, K_FOREVER);

	cfg_xfer.port_addr = port_addr;
	cfg_xfer.reg_addr = reg_addr;
	cfg_xfer.op = MDIO_OP_C22_READ;
	cfg_xfer.c45 = false;
	cfg_xfer.data_in = 0;
	cfg_xfer.data_out = data;

	ret = mdio_transfer(cfg->gmac_regs, &cfg_xfer);

	k_sem_give(&mdio_data->reg_sem);
	return ret;
}

static int mdio_mchp_write(const struct device *dev, uint8_t port_addr, uint8_t reg_addr,
			   uint16_t data)
{
	int ret;
	struct mdio_dev_data *const mdio_data = dev->data;
	const struct mdio_dev_config *const cfg = dev->config;
	struct mdio_config_transfer cfg_xfer;

	k_sem_take(&mdio_data->reg_sem, K_FOREVER);

	cfg_xfer.port_addr = port_addr;
	cfg_xfer.reg_addr = reg_addr;
	cfg_xfer.op = MDIO_OP_C22_WRITE;
	cfg_xfer.c45 = false;
	cfg_xfer.data_in = data;
	cfg_xfer.data_out = NULL;

	ret = mdio_transfer(cfg->gmac_regs, &cfg_xfer);

	k_sem_give(&mdio_data->reg_sem);
	return ret;
}

static int mdio_mchp_read_c45(const struct device *dev, uint8_t port_addr, uint8_t devad,
			      uint16_t reg_addr, uint16_t *data)
{
	int err;
	struct mdio_dev_data *const mdio_data = dev->data;
	const struct mdio_dev_config *const cfg = dev->config;
	struct mdio_config_transfer cfg_xfer;

	k_sem_take(&mdio_data->reg_sem, K_FOREVER);

	cfg_xfer.port_addr = port_addr;
	cfg_xfer.reg_addr = devad;
	cfg_xfer.op = MDIO_OP_C45_ADDRESS;
	cfg_xfer.c45 = true;
	cfg_xfer.data_in = reg_addr;
	cfg_xfer.data_out = NULL;

	err = mdio_transfer(cfg->gmac_regs, &cfg_xfer);
	if (err == MDIO_MCHP_ESUCCESS) {
		cfg_xfer.port_addr = port_addr;
		cfg_xfer.reg_addr = devad;
		cfg_xfer.op = MDIO_OP_C45_READ;
		cfg_xfer.c45 = true;
		cfg_xfer.data_in = 0;
		cfg_xfer.data_out = data;

		err = mdio_transfer(cfg->gmac_regs, &cfg_xfer);
	}

	k_sem_give(&mdio_data->reg_sem);
	return err;
}

static int mdio_mchp_write_c45(const struct device *dev, uint8_t port_addr, uint8_t devad,
			       uint16_t reg_addr, uint16_t data)
{
	int err;
	struct mdio_dev_data *const mdio_data = dev->data;
	const struct mdio_dev_config *const cfg = dev->config;
	struct mdio_config_transfer cfg_xfer;

	k_sem_take(&mdio_data->reg_sem, K_FOREVER);

	cfg_xfer.port_addr = port_addr;
	cfg_xfer.reg_addr = devad;
	cfg_xfer.op = MDIO_OP_C45_ADDRESS;
	cfg_xfer.c45 = true;
	cfg_xfer.data_in = reg_addr;
	cfg_xfer.data_out = NULL;

	err = mdio_transfer(cfg->gmac_regs, &cfg_xfer);
	if (err == MDIO_MCHP_ESUCCESS) {
		cfg_xfer.port_addr = port_addr;
		cfg_xfer.reg_addr = devad;
		cfg_xfer.op = MDIO_OP_C45_WRITE;
		cfg_xfer.c45 = true;
		cfg_xfer.data_in = data;
		cfg_xfer.data_out = NULL;

		err = mdio_transfer(cfg->gmac_regs, &cfg_xfer);
	}

	k_sem_give(&mdio_data->reg_sem);
	return err;
}

static inline int mdio_get_mck_clock_divisor(uint32_t mck)
{
	uint32_t mck_divisor;

	if (mck <= MDIO_MCHP_CLOCK_RATE_20MHZ) {
		mck_divisor = GMAC_NCFGR_CLK_MCK8;
	} else if (mck <= MDIO_MCHP_CLOCK_RATE_40MHZ) {
		mck_divisor = GMAC_NCFGR_CLK_MCK16;
	} else if (mck <= MDIO_MCHP_CLOCK_RATE_80MHZ) {
		mck_divisor = GMAC_NCFGR_CLK_MCK32;
	} else if (mck <= MDIO_MCHP_CLOCK_RATE_120MHZ) {
		mck_divisor = GMAC_NCFGR_CLK_MCK48;
	} else if (mck <= MDIO_MCHP_CLOCK_RATE_160MHZ) {
		mck_divisor = GMAC_NCFGR_CLK_MCK64;
	} else if (mck <= MDIO_MCHP_CLOCK_RATE_240MHZ) {
		mck_divisor = GMAC_NCFGR_CLK_MCK96;
	} else {
		LOG_ERR("No valid MDC clock");
		return -ENOTSUP;
	}

	LOG_INF("mck %d mck_divisor = 0x%x", mck, mck_divisor);
	return mck_divisor;
}

static int mdio_mchp_initialize(const struct device *dev)
{
	const struct mdio_dev_config *const cfg = dev->config;
	struct mdio_dev_data *const data = dev->data;
	int retval;
	uint32_t clk_freq_hz = 0;
	int mck_divisor;

	k_sem_init(&data->reg_sem, 1, 1);
	retval = clock_control_on(cfg->mdio_clock_cfg.clock_dev, cfg->mdio_clock_cfg.mclk_apb_sys);
	if ((retval != 0) && (retval != -EALREADY)) {
		LOG_ERR("Failed to enable the MCLK APB for Mdio: %d", retval);
		return retval;
	}

	retval = clock_control_on(cfg->mdio_clock_cfg.clock_dev, cfg->mdio_clock_cfg.mclk_ahb_sys);
	if ((retval != 0) && (retval != -EALREADY)) {
		LOG_ERR("Failed to enable the MCLK AHB for Mdio: %d", retval);
		return retval;
	}

	retval = clock_control_get_rate(
		((const struct mdio_dev_config *)(dev->config))->mdio_clock_cfg.clock_dev,
		(((const struct mdio_dev_config *)(dev->config))->mdio_clock_cfg.mclk_apb_sys),
		&clk_freq_hz);
	if (retval < 0) {
		LOG_ERR("ETH_MCHP_GET_CLOCK_FREQ Failed");
		return retval;
	}

	mck_divisor = mdio_get_mck_clock_divisor(clk_freq_hz);
	if (mck_divisor < 0) {
		return mck_divisor;
	}

	cfg->gmac_regs->GMAC_NCFGR &=
		~(GMAC_NCFGR_CLK_MCK8 | GMAC_NCFGR_CLK_MCK16 | GMAC_NCFGR_CLK_MCK32 |
		  GMAC_NCFGR_CLK_MCK48 | GMAC_NCFGR_CLK_MCK64 | GMAC_NCFGR_CLK_MCK96);
	cfg->gmac_regs->GMAC_NCFGR |= mck_divisor;
	retval = pinctrl_apply_state(cfg->pinctrl_cfg, PINCTRL_STATE_DEFAULT);
	if (retval != 0) {
		LOG_ERR("pinctrl_apply_state() Failed for Mdio driver: %d", retval);
	}

	return retval;
}

static DEVICE_API(mdio, mdio_mchp_driver_api) = {
	.read = mdio_mchp_read,
	.write = mdio_mchp_write,
	.read_c45 = mdio_mchp_read_c45,
	.write_c45 = mdio_mchp_write_c45,
};

#define MDIO_MCHP_CONFIG(n)                                                                        \
	static const struct mdio_dev_config mdio_dev_config_##n = {                                \
		.pinctrl_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                  \
		.gmac_regs = (gmac_registers_t *)DT_INST_REG_ADDR(n),                              \
		MDIO_MCHP_CLOCK_DEFN(n)};

#define MDIO_MCHP_DEVICE(n)                                                                        \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	MDIO_MCHP_CONFIG(n);                                                                       \
	static struct mdio_dev_data mdio_dev_data##n;                                              \
	DEVICE_DT_INST_DEFINE(n, &mdio_mchp_initialize, NULL, &mdio_dev_data##n,                   \
			      &mdio_dev_config_##n, POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,        \
			      &mdio_mchp_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_MCHP_DEVICE)
