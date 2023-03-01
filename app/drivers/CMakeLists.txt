# Copyright (c) 2023 XiNGRZ
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/include)

add_subdirectory(swdio)
add_subdirectory(usb)
