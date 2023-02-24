/*
 * Copyright (c) 2023 XiNGRZ
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zprobe, CONFIG_ZPROBE_LOG_LEVEL);

#include "usb_status.h"

#define LED_CON_NODE DT_ALIAS(led_con)
#define LED_CON_OKAY DT_NODE_HAS_STATUS(LED_CON_NODE, okay)

#if LED_CON_OKAY

static const struct gpio_dt_spec led_con = GPIO_DT_SPEC_GET(LED_CON_NODE, gpios);

void usb_status_changed(enum usb_dc_status_code cb_status, const uint8_t *param)
{
	if (!gpio_is_ready_dt(&led_con)) {
		return;
	}

	if (cb_status == USB_DC_CONFIGURED) {
		gpio_pin_set_dt(&led_con, 1);
	} else {
		gpio_pin_set_dt(&led_con, 0);
	}
}

static int usb_status_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	int ret;

	ret = gpio_pin_configure_dt(&led_con, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("LED_CON not configured: %d", ret);
		return ret;
	}

	return 0;
}

SYS_INIT(usb_status_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#else // LED_CON_OKAY

void usb_status_changed(enum usb_dc_status_code cb_status, const uint8_t *param)
{
	ARG_UNUSED(cb_status);
	ARG_UNUSED(param);
}

#endif // LED_CON_OKAY
