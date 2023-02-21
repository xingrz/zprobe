/*
 * Copyright (c) 2023 XiNGRZ
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zprobe_swdio_bitbang

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zprobe/drivers/swdio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(swdio, CONFIG_SWDIO_LOG_LEVEL);

struct swdio_bitbang_config {
	struct gpio_dt_spec swclk_tck_gpio;
	struct gpio_dt_spec swdio_out_gpio;
	struct gpio_dt_spec swdio_in_gpio;
};

static void swdio_bitbang_write_bits(const struct device *dev, const uint8_t *bits, uint32_t count)
{
	const struct swdio_bitbang_config *config = dev->config;

	LOG_DBG("writing %d bits", count);

	unsigned int key = irq_lock();

	uint8_t val;
	for (uint32_t n = count, k; n;) {
		val = *bits++;
		for (k = 8; k && n; k--, n--) {
			gpio_pin_set_dt(&config->swdio_out_gpio, val & 0x1);
			gpio_pin_set_dt(&config->swclk_tck_gpio, 0);
			gpio_pin_set_dt(&config->swclk_tck_gpio, 1);
			val >>= 1;
		}
	}

	irq_unlock(key);
}

static void swdio_bitbang_read_bits(const struct device *dev, uint8_t *bits, uint32_t count)
{
	const struct swdio_bitbang_config *config = dev->config;

	LOG_DBG("reading %d bits", count);

	unsigned int key = irq_lock();

	uint8_t val, bit;
	for (uint32_t n = count, k; n;) {
		val = 0;
		for (k = 8; k && n; k--, n--) {
			gpio_pin_set_dt(&config->swclk_tck_gpio, 0);
			bit = gpio_pin_get_dt(&config->swdio_in_gpio);
			gpio_pin_set_dt(&config->swclk_tck_gpio, 1);
			val >>= 1;
			val |= bit << 7;
		}
		val >>= k;
		*bits++ = val;
	}

	irq_unlock(key);
}

static void swdio_bitbang_write_dummy(const struct device *dev, uint32_t cycles)
{
	const struct swdio_bitbang_config *config = dev->config;

	LOG_DBG("dummy write %d cycles", cycles);

	unsigned int key = irq_lock();

	gpio_pin_set_dt(&config->swdio_out_gpio, 0);
	for (; cycles; cycles--) {
		gpio_pin_set_dt(&config->swclk_tck_gpio, 0);
		gpio_pin_set_dt(&config->swclk_tck_gpio, 1);
	}
	gpio_pin_set_dt(&config->swdio_out_gpio, 1);

	irq_unlock(key);
}

static void swdio_bitbang_read_dummy(const struct device *dev, uint32_t cycles)
{
	const struct swdio_bitbang_config *config = dev->config;

	LOG_DBG("dummy read %d cycles", cycles);

	unsigned int key = irq_lock();

	for (; cycles; cycles--) {
		gpio_pin_set_dt(&config->swclk_tck_gpio, 0);
		gpio_pin_set_dt(&config->swclk_tck_gpio, 1);
	}

	irq_unlock(key);
}

static void swdio_bitbang_turn_around(const struct device *dev, uint8_t cycles)
{
	const struct swdio_bitbang_config *config = dev->config;

	LOG_DBG("turning around %d cycles", cycles);

	unsigned int key = irq_lock();

	for (; cycles; cycles--) {
		gpio_pin_set_dt(&config->swclk_tck_gpio, 0);
		gpio_pin_set_dt(&config->swclk_tck_gpio, 1);
	}

	irq_unlock(key);
}

static void swdio_bitbang_set_read_mode(const struct device *dev)
{
	const struct swdio_bitbang_config *config = dev->config;
	gpio_pin_configure_dt(&config->swdio_in_gpio, GPIO_INPUT);
}

static void swdio_bitbang_set_write_mode(const struct device *dev)
{
	const struct swdio_bitbang_config *config = dev->config;
	gpio_pin_configure_dt(&config->swdio_out_gpio, GPIO_OUTPUT_HIGH);
}

static void swdio_bitbang_set_clock_freq(const struct device *dev, uint32_t clk_freq)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(clk_freq);
}

static const struct swdio_driver_api swdio_bitbang_driver_api = {
	.write_bits = swdio_bitbang_write_bits,
	.read_bits = swdio_bitbang_read_bits,
	.write_dummy = swdio_bitbang_write_dummy,
	.read_dummy = swdio_bitbang_read_dummy,
	.turn_around = swdio_bitbang_turn_around,
	.set_read_mode = swdio_bitbang_set_read_mode,
	.set_write_mode = swdio_bitbang_set_write_mode,
	.set_clock_freq = swdio_bitbang_set_clock_freq,
};

static int swdio_bitbang_init(const struct device *dev)
{
	const struct swdio_bitbang_config *config = dev->config;

	int ret;

	if (!device_is_ready(config->swclk_tck_gpio.port)) {
		LOG_ERR("swclk-tck-gpio is not ready");
		return -ENODEV;
	}

	if (!device_is_ready(config->swdio_out_gpio.port)) {
		LOG_ERR("swdio-out-gpio is not ready");
		return -ENODEV;
	}

	if (!device_is_ready(config->swdio_in_gpio.port)) {
		LOG_ERR("swdio-in-gpio is not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->swclk_tck_gpio, GPIO_OUTPUT_HIGH);
	if (ret != 0) {
		LOG_ERR("swclk-tck-gpio configure failed");
		return ret;
	}

	return 0;
}

const struct swdio_bitbang_config swdio_bitbang_config = {
	.swclk_tck_gpio = GPIO_DT_SPEC_INST_GET(0, swclk_tck_gpios),
	.swdio_out_gpio = GPIO_DT_SPEC_INST_GET(0, swdio_out_gpios),
	.swdio_in_gpio = GPIO_DT_SPEC_INST_GET(0, swdio_in_gpios),
};

DEVICE_DT_INST_DEFINE(0, swdio_bitbang_init, NULL, NULL, &swdio_bitbang_config, POST_KERNEL,
		      CONFIG_APPLICATION_INIT_PRIORITY, &swdio_bitbang_driver_api);
