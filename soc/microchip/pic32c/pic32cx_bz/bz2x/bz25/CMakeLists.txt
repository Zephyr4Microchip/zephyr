# Copyright (c) 2025 Microchip Technology Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

zephyr_library_sources(system_config.c)

zephyr_library_sources(soc.c)

zephyr_linker_sources(SECTIONS linker.ld)

zephyr_include_directories(${ZEPHYR_BASE}/drivers)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
