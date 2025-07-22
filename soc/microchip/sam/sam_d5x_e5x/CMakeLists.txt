# Copyright (c) 2025 Microchip Technology Inc.
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(common)

zephyr_include_directories_ifdef(CONFIG_SOC_SERIES_MCHP_SAMD51 samd51)
zephyr_include_directories_ifdef(CONFIG_SOC_SERIES_MCHP_SAME51 same51)
zephyr_include_directories_ifdef(CONFIG_SOC_SERIES_MCHP_SAME53 same53)
zephyr_include_directories_ifdef(CONFIG_SOC_SERIES_MCHP_SAME54 same54)
