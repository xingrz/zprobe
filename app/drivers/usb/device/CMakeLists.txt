# Copyright (c) 2023 XiNGRZ
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_USB_DEVICE_DRIVER)

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_USB_DC_ESP32SX usb_dc_esp32sx.c)

endif()
