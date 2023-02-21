# Copyright (c) 2023 XiNGRZ
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(cmsis_dap)
zephyr_library_sources_ifdef(CONFIG_CMSIS_DAP DAP.c)

zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
