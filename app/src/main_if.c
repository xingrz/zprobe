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

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#define LED_RUN_NODE DT_ALIAS(led_run)
#define LED_CON_NODE DT_ALIAS(led_con)

static struct gpio_dt_spec led_run = GPIO_DT_SPEC_GET(LED_RUN_NODE, gpios);
static struct gpio_dt_spec led_con = GPIO_DT_SPEC_GET(LED_CON_NODE, gpios);

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

	gpio_pin_configure_dt(&led_run, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led_con, GPIO_OUTPUT_ACTIVE);

	while (1) {
		gpio_pin_toggle_dt(&led_run);
		gpio_pin_toggle_dt(&led_con);
		k_msleep(1000);
	}
}
