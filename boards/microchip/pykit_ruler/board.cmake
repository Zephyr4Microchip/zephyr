# Copyright (c) 2026 Microchip Technology Inc.
# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=ATSAME51J20A" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
