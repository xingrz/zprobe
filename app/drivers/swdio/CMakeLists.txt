# Copyright (c) 2023 XiNGRZ
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_HAS_SWDIO)

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_SWDIO_BITBANG swdio_bitbang.c)

endif() # CONFIG_HAS_SWDIO
