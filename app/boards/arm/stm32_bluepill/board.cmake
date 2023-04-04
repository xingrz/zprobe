# Copyright (c) 2023 XiNGRZ
# SPDX-License-Identifier: Apache-2.0

board_runner_args(pyocd "--target=stm32f103cb")

set_ifndef(BOARD_FLASH_RUNNER pyocd)

include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
