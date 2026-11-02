# Copyright (c) 2025 Microchip Technology Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

zephyr_sources_ifdef(CONFIG_PM
  ../common/power.c
  ../common/device_sleep.c
  )

# The device configuration words in boot flash belong to the image the ROM
# boots. An image booted by MCUboot must not carry them.
if(NOT CONFIG_BOOTLOADER_MCUBOOT)
  zephyr_library_sources(system_config.c)
endif()

zephyr_library_sources(soc.c)

zephyr_linker_sources(SECTIONS linker.ld)

zephyr_include_directories(${ZEPHYR_BASE}/drivers)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
