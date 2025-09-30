# Copyright (c) 2025 Microchip Technology Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(common)

add_subdirectory(${SOC_SERIES})

if(CONFIG_DEVICE_SUPPORT_REQUIRED)
  zephyr_blobs_verify(MODULE hal_microchip REQUIRED)
endif()
