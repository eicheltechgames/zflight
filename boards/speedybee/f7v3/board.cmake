#
# Copyright 2024 Arthur Eichelberger
#
# SPDX-License-Identifier: Apache-2.0
#

# TODO pid?: board_runner_args(dfu-util "--pid=0483:df11" "--alt=0" "--dfuse")
# TODO has jtag?: board_runner_args(jlink "--device=STM32F722RE" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/dfu-util.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
