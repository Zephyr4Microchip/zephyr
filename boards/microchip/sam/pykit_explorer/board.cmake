# SPDX-License-Identifier: Apache-2.0
#
# PyKit Explorer / SAME51 Curiosity Nano
# On-board programmer: EDBG (CMSIS-DAP HID), SWD interface.

board_runner_args(pyocd "--target=atsame51j20a")
board_runner_args(openocd "--config=${BOARD_DIR}/support/openocd.cfg")
board_runner_args(jlink "--device=ATSAME51J20A" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
