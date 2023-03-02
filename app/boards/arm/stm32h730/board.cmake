# Copyright (c) 2023 XiNGRZ
# SPDX-License-Identifier: Apache-2.0

board_runner_args(dfu-util "--pid=0483:df11" "--alt=0" "--dfuse" "--dfuse-modifiers=leave" "--dfuse-modifiers=force")

set_ifndef(BOARD_FLASH_RUNNER dfu-util)

include(${ZEPHYR_BASE}/boards/common/dfu-util.board.cmake)
