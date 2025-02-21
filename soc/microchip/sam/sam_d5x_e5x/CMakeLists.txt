# Copyright (c) 2025 Microchip Technology Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(${ZEPHYR_BASE}/drivers)

zephyr_include_directories(../common)

add_subdirectory(${SOC_SERIES})
