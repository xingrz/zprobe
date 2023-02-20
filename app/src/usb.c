/*
 * Copyright (c) 2023 XiNGRZ
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/usb/usb_device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zprobe, CONFIG_ZPROBE_LOG_LEVEL);

static int zprobe_usb_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	int ret;

	ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB: %d", ret);
		return ret;
	}

	return 0;
}

SYS_INIT(zprobe_usb_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
