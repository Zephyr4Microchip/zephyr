# Copyright (c) 2025 Microchip Technology Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_BT_TRS
	trs.c
)

zephyr_library_sources_ifdef(CONFIG_BT_TRS_SERVER
	trs_server.c
)
