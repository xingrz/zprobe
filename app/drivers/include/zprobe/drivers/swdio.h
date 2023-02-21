/*
 * Copyright (c) 2023 XiNGRZ
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>

struct swdio_driver_api {
	void (*write_bits)(const struct device *dev, const uint8_t *bits, uint32_t count);
	void (*read_bits)(const struct device *dev, uint8_t *bits, uint32_t count);
	void (*write_dummy)(const struct device *dev, uint32_t cycles);
	void (*read_dummy)(const struct device *dev, uint32_t cycles);
	void (*turn_around)(const struct device *dev, uint8_t cycles);
	void (*set_read_mode)(const struct device *dev);
	void (*set_write_mode)(const struct device *dev);
	void (*set_clock_freq)(const struct device *dev, uint32_t clk_freq);
};

static inline void swdio_write_bits(const struct device *dev, const uint8_t *bits, uint32_t count)
{
	const struct swdio_driver_api *api = (const struct swdio_driver_api *)dev->api;
	api->write_bits(dev, bits, count);
}

static inline void swdio_read_bits(const struct device *dev, uint8_t *bits, uint32_t count)
{
	const struct swdio_driver_api *api = (const struct swdio_driver_api *)dev->api;
	api->read_bits(dev, bits, count);
}

static inline void swdio_write_uint32(const struct device *dev, uint32_t val, uint32_t bit_count)
{
	if (bit_count > 32) {
		bit_count = 32;
	}
	val = sys_cpu_to_le32(val);
	swdio_write_bits(dev, (uint8_t *)&val, bit_count);
}

static inline uint32_t swdio_read_uint32(const struct device *dev, uint32_t bit_count)
{
	uint32_t val = 0;
	swdio_read_bits(dev, (uint8_t *)&val, bit_count);
	return sys_le32_to_cpu(val);
}

static inline void swdio_write_dummy(const struct device *dev, uint32_t cycles)
{
	const struct swdio_driver_api *api = (const struct swdio_driver_api *)dev->api;
	api->write_dummy(dev, cycles);
}

static inline void swdio_read_dummy(const struct device *dev, uint32_t cycles)
{
	const struct swdio_driver_api *api = (const struct swdio_driver_api *)dev->api;
	api->read_dummy(dev, cycles);
}

static inline void swdio_turn_around(const struct device *dev, uint8_t cycles)
{
	const struct swdio_driver_api *api = (const struct swdio_driver_api *)dev->api;
	api->turn_around(dev, cycles);
}

static inline void swdio_set_read_mode(const struct device *dev)
{
	const struct swdio_driver_api *api = (const struct swdio_driver_api *)dev->api;
	api->set_read_mode(dev);
}

static inline void swdio_set_write_mode(const struct device *dev)
{
	const struct swdio_driver_api *api = (const struct swdio_driver_api *)dev->api;
	api->set_write_mode(dev);
}

static inline void swdio_set_clock_freq(const struct device *dev, uint32_t clk_freq)
{
	const struct swdio_driver_api *api = (const struct swdio_driver_api *)dev->api;
	api->set_clock_freq(dev, clk_freq);
}
