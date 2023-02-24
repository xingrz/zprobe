# Copyright (c) 2023 XiNGRZ
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(ZephyrBuildConfiguration_ROOT ${CMAKE_SOURCE_DIR}/cmake)
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_SOURCE_DIR}/drivers)

find_package(Zephyr REQUIRED HINTS ../zephyr)
project(zprobe)

target_sources_ifdef(CONFIG_ZPROBE_BOOTLOADER app PRIVATE src/main_bl.c)
target_sources_ifdef(CONFIG_ZPROBE_INTERFACE app PRIVATE src/main_if.c)

target_sources(app PRIVATE src/usb_status.c)

target_sources_ifdef(CONFIG_ZPROBE_CDC_ACM app PRIVATE src/usb_cdc_acm.c)
