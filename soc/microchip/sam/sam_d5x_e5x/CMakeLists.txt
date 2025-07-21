# Copyright (c) 2025 Microchip Technology Inc.
# SPDX-License-Identifier: Apache-2.0

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")

zephyr_include_directories(common)
zephyr_include_directories_ifdef(CONFIG_SOC_SERIES_MCHP_SAMD51 samd51)
zephyr_include_directories_ifdef(CONFIG_SOC_SERIES_MCHP_SAME51 same51)
zephyr_include_directories_ifdef(CONFIG_SOC_SERIES_MCHP_SAME53 same53)
zephyr_include_directories_ifdef(CONFIG_SOC_SERIES_MCHP_SAME54 same54)
