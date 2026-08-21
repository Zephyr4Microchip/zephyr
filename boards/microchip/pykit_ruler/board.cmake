# Copyright (c) 2026 Microchip Technology Inc.
# SPDX-License-Identifier: Apache-2.0

board_runner_args(mplab_ipe "--tool" "SNAP" "--part" "atsame51j20a" "--erase" "--verify")

include(${ZEPHYR_BASE}/boards/common/mplab_ipe.board.cmake)
