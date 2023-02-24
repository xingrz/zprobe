/*
 * Copyright (c) 2023 XiNGRZ
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <zephyr/usb/usb_device.h>

void usb_status_changed(enum usb_dc_status_code cb_status, const uint8_t *param);
