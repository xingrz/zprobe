# Copyright (c) 2023 XiNGRZ
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(ZephyrBuildConfiguration_ROOT ${CMAKE_SOURCE_DIR}/cmake)
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_SOURCE_DIR}/drivers)

find_package(Zephyr REQUIRED HINTS ../zephyr)
project(zprobe)

target_sources(app PRIVATE src/main.c)
