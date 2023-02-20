/*
 * Copyright (c) 2023 XiNGRZ
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zprobe, CONFIG_ZPROBE_LOG_LEVEL);

void main(void)
{
	LOG_INF("Hello world!");
}
