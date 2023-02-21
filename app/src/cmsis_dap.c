/*
 * Copyright (c) 2023 XiNGRZ
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zprobe/drivers/swdio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zprobe, CONFIG_ZPROBE_LOG_LEVEL);

#include "DAP_config.h"
#include "DAP.h"

#define SWDIO_NODE DT_CHOSEN(zprobe_swdio)

#define LED_CON_NODE DT_ALIAS(led_con)
#define LED_CON_OKAY DT_NODE_HAS_STATUS(LED_CON_NODE, okay)

#define LED_RUN_NODE DT_ALIAS(led_run)
#define LED_RUN_OKAY DT_NODE_HAS_STATUS(LED_RUN_NODE, okay)

static const struct device *swdio_dev;

#if LED_CON_OKAY
static const struct gpio_dt_spec led_con = GPIO_DT_SPEC_GET(LED_CON_NODE, gpios);
#endif

#if LED_RUN_OKAY
static const struct gpio_dt_spec led_run = GPIO_DT_SPEC_GET(LED_RUN_NODE, gpios);
#endif

void PIN_SWDIO_OUT_ENABLE(void)
{
	swdio_set_write_mode(swdio_dev);
}

void PIN_SWDIO_OUT_DISABLE(void)
{
	swdio_set_read_mode(swdio_dev);
}

uint32_t PIN_nRESET_IN(void)
{
	return 1;
}

void PIN_nRESET_OUT(uint32_t bit)
{
	LOG_DBG("reset: %d", bit);
}

void LED_CONNECTED_OUT(uint32_t bit)
{
#if LED_CON_OKAY
	gpio_pin_set_dt(&led_con, bit);
#else
	ARG_UNUSED(bit);
#endif
}

void LED_RUNNING_OUT(uint32_t bit)
{
#if LED_RUN_OKAY
	gpio_pin_set_dt(&led_run, bit);
#else
	ARG_UNUSED(bit);
#endif
}

void SWJ_Sequence(uint32_t count, const uint8_t *data)
{
	// LOG_DBG("size: %d", count);
	// LOG_HEXDUMP_DBG(data, (count + 7) / 8, "data:");
	swdio_set_clock_freq(swdio_dev, DAP_Data.clock_freq);
	LED_RUNNING_OUT(1);
	swdio_write_bits(swdio_dev, data, count);
	LED_RUNNING_OUT(0);
}

void SWD_Sequence(uint32_t info, const uint8_t *swdo, uint8_t *swdi)
{
	// LOG_DBG("info: %02x", info);

	swdio_set_clock_freq(swdio_dev, DAP_Data.clock_freq);

	LED_RUNNING_OUT(1);

	uint32_t n = info & SWD_SEQUENCE_CLK;
	if (n == 0U) {
		n = 64U;
	}

	if (info & SWD_SEQUENCE_DIN) {
		swdio_read_bits(swdio_dev, swdi, n);
	} else {
		swdio_write_bits(swdio_dev, swdo, n);
	}

	LED_RUNNING_OUT(0);
}

static inline uint8_t calc_parity(uint32_t val)
{
	uint32_t parity = 0;
	for (int i = 0; i < 32; i++) {
		parity = (parity + (val >> i)) & 0x1;
	}
	return parity;
}

uint8_t SWD_Transfer(uint32_t request, uint32_t *data)
{
	// LOG_DBG("request: %02x", request);

	uint32_t req;
	uint32_t ack;
	uint32_t val;
	uint32_t parity;

	swdio_set_clock_freq(swdio_dev, DAP_Data.clock_freq);

	LED_RUNNING_OUT(1);

	/* Packet Request */
	req = request & BIT_MASK(4);
	// LOG_DBG("req: %02x", req);
	parity = calc_parity(req);
	val = BIT(7) | (parity << 5) | (req << 1) | BIT(0);
	swdio_write_uint32(swdio_dev, val, 8);

	/* Turnaround */
	swdio_set_read_mode(swdio_dev);
	swdio_turn_around(swdio_dev, DAP_Data.swd_conf.turnaround);

	/* Acknowledge response */
	ack = swdio_read_uint32(swdio_dev, 3);
	// LOG_DBG("ack: %02x", ack);

	if (ack == DAP_TRANSFER_OK) {
		/* Data transfer */
		if (request & DAP_TRANSFER_RnW) {
			uint8_t bit;
			val = swdio_read_uint32(swdio_dev, 32);
			parity = calc_parity(val);
			bit = swdio_read_uint32(swdio_dev, 1);
			if ((parity ^ bit) & 0x1) {
				ack = DAP_TRANSFER_ERROR;
				LOG_DBG("Read error");
			}
			if (data) {
				*data = val;
			}

			// LOG_DBG("read: %08x", val);
			swdio_turn_around(swdio_dev, DAP_Data.swd_conf.turnaround);
			swdio_set_write_mode(swdio_dev);
		} else {
			swdio_turn_around(swdio_dev, DAP_Data.swd_conf.turnaround);
			swdio_set_write_mode(swdio_dev);

			val = *data;
			swdio_write_uint32(swdio_dev, val, 32);
			parity = calc_parity(val);
			swdio_write_uint32(swdio_dev, parity, 1);
			// LOG_DBG("wrote: %08x", val);
		}

		/* Capture Timestamp */
		if (request & DAP_TRANSFER_TIMESTAMP) {
			DAP_Data.timestamp = TIMESTAMP_GET();
		}

		/* Idle cycles */
		swdio_write_dummy(swdio_dev, DAP_Data.transfer.idle_cycles);
	} else if (ack == DAP_TRANSFER_WAIT || ack == DAP_TRANSFER_FAULT) {
		LOG_DBG("WAIT or FAULT response");
		if (DAP_Data.swd_conf.data_phase && (request & DAP_TRANSFER_RnW) != 0) {
			swdio_read_dummy(swdio_dev, 32 + 1);
		}

		/* Turnaround */
		swdio_turn_around(swdio_dev, DAP_Data.swd_conf.turnaround);
		swdio_set_write_mode(swdio_dev);

		if (DAP_Data.swd_conf.data_phase && (request & DAP_TRANSFER_RnW) != 0) {
			swdio_write_dummy(swdio_dev, 32 + 1);
		}
	} else {
		LOG_DBG("Protocol error: %02x", ack);
		swdio_turn_around(swdio_dev, DAP_Data.swd_conf.turnaround + 32U + 1U);
		swdio_set_write_mode(swdio_dev);
	}

	LED_RUNNING_OUT(0);

	return (uint8_t)ack;
}

static int cmsis_dap_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	int ret = 0;

	swdio_dev = DEVICE_DT_GET(SWDIO_NODE);
	if (!swdio_dev) {
		LOG_ERR("SWDIO device not found");
		return -ENODEV;
	}

#if LED_CON_OKAY
	ret = gpio_pin_configure_dt(&led_con, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("LED_CON not configured: %d", ret);
		return ret;
	}
#endif

#if LED_RUN_OKAY
	ret = gpio_pin_configure_dt(&led_run, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("LED_RUN not configured: %d", ret);
		return ret;
	}
#endif

	DAP_Setup();

	return ret;
}

SYS_INIT(cmsis_dap_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
