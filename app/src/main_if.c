/*
 * Copyright (c) 2023 XiNGRZ
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/usb/usb_device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zprobe, CONFIG_ZPROBE_LOG_LEVEL);

#include "usb_status.h"

void main(void)
{
	int ret;
	ARG_UNUSED(ret);

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
	ret = usb_enable(usb_status_changed);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB: %d", ret);
		return;
	}
#endif // CONFIG_USB_DEVICE_STACK

	LOG_INF("Hello world!");
}
