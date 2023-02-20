/*
 * Copyright (c) 2023 XiNGRZ
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zprobe_usb_cdc_acm

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/uart/cdc_acm.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zprobe, CONFIG_ZPROBE_LOG_LEVEL);

enum {
	PEER_0,
	PEER_1,
	PEER_CNT,
};

struct cdc_acm_peer {
	const struct device *dev;
	struct cdc_acm_peer *target;
	struct ring_buf rb;
	uint8_t buffer[CONFIG_ZPROBE_CDC_ACM_RING_BUF_SIZE];
};

struct cdc_acm_data {
	struct cdc_acm_peer peers[PEER_CNT];
};

struct cdc_acm_config {
	const struct device *devices[PEER_CNT];
};

#define LIST_PEERS(n)                                                                              \
	DEVICE_DT_GET(DT_INST_PHANDLE(n, peer_0)), DEVICE_DT_GET(DT_INST_PHANDLE(n, peer_1)),

static const struct device *cdc_acm_peer_devs[] = {DT_INST_FOREACH_STATUS_OKAY(LIST_PEERS)};

static void cdc_acm_irq_handler(const struct device *dev, void *user_data)
{
	struct cdc_acm_peer *peer = user_data;

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uint8_t buf[64];
			int read;
			size_t wrote;
			struct ring_buf *rb = &peer->target->rb;

			read = uart_fifo_read(dev, buf, sizeof(buf));
			if (read < 0) {
				LOG_ERR("Failed to read UART FIFO from %s", peer->dev->name);
				read = 0;
			};

			wrote = ring_buf_put(rb, buf, read);
			if (wrote < read) {
				LOG_ERR("Drop %zu bytes", read - wrote);
			}

			if (wrote) {
				uart_irq_tx_enable(peer->dev);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t buf[64];
			size_t len;

			len = ring_buf_get(&peer->rb, buf, sizeof(buf));
			if (!len) {
				uart_irq_tx_disable(dev);
			} else {
				uart_fifo_fill(dev, buf, len);
			}
		}
	}
}

static inline int cdc_acm_set_target_baudrate(const struct device *target, uint32_t rate)
{
	int ret;
	struct uart_config cfg;

	ret = uart_line_ctrl_set(target, UART_LINE_CTRL_BAUD_RATE, rate);
	if (ret == 0 || ret != -ENOSYS) {
		return ret;
	}

	ret = uart_config_get(target, &cfg);
	if (ret != 0) {
		return ret;
	}

	cfg.baudrate = rate;

	return uart_configure(target, &cfg);
}

static void cdc_acm_baudrate_callback(const struct device *dev, uint32_t rate)
{
	LOG_DBG("Baud rate of %p %s is changing to %d", dev, dev->name, rate);

	int ret;
	const struct device *target;

	for (int i = 0; i < ARRAY_SIZE(cdc_acm_peer_devs) - 1; i++) {
		if (cdc_acm_peer_devs[i] == dev) {
			target = cdc_acm_peer_devs[i + 1];
			LOG_DBG("Setting baud rate of %p %s to %d", target, target->name, rate);
			ret = cdc_acm_set_target_baudrate(target, rate);
			if (ret != 0) {
				LOG_ERR("Failed setting baud rate for %s: %d %s", target->name, ret,
					strerror(ret));
			} else {
				LOG_DBG("Baud rate changed to %d", rate);
			}
			break;
		}
	}
}

static int cdc_acm_init(const struct device *dev)
{
	struct cdc_acm_data *data = dev->data;
	const struct cdc_acm_config *config = dev->config;

	for (int i = 0; i < PEER_CNT; i++) {
		if (!device_is_ready(config->devices[i])) {
			LOG_ERR("UART/CDC device %s is not ready", config->devices[i]->name);
			return -ENODEV;
		}
	}

	LOG_DBG("Bridging %p %s <--> %p %s", config->devices[PEER_0], config->devices[PEER_0]->name,
		config->devices[PEER_1], config->devices[PEER_1]->name);

	data->peers[PEER_0].dev = config->devices[PEER_0];
	data->peers[PEER_0].target = &data->peers[PEER_1];
	ring_buf_init(&data->peers[PEER_0].rb, sizeof(data->peers[PEER_0].buffer),
		      data->peers[PEER_0].buffer);

	data->peers[PEER_1].dev = config->devices[PEER_1];
	data->peers[PEER_1].target = &data->peers[PEER_0];
	ring_buf_init(&data->peers[PEER_1].rb, sizeof(data->peers[PEER_1].buffer),
		      data->peers[PEER_1].buffer);

	uart_irq_callback_user_data_set(config->devices[PEER_1], cdc_acm_irq_handler,
					&data->peers[PEER_0]);

	uart_irq_callback_user_data_set(config->devices[PEER_0], cdc_acm_irq_handler,
					&data->peers[PEER_1]);

	uart_irq_rx_enable(data->peers[PEER_0].dev);
	uart_irq_rx_enable(data->peers[PEER_1].dev);

	// Assuming peer-0 is the USB CDC-ACM device
	cdc_acm_dte_rate_callback_set(config->devices[PEER_0], cdc_acm_baudrate_callback);

	return 0;
}

#define CDC_ACM_INST(n)                                                                            \
	static struct cdc_acm_data cdc_acm_data_##n = {};                                          \
                                                                                                   \
	static const struct cdc_acm_config cdc_acm_config_##n = {                                  \
		.devices = {                                                                       \
			DEVICE_DT_GET(DT_INST_PHANDLE(n, peer_0)),                                 \
			DEVICE_DT_GET(DT_INST_PHANDLE(n, peer_1)),                                 \
		}};                                                                                \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, cdc_acm_init, NULL, &cdc_acm_data_##n, &cdc_acm_config_##n,       \
			      POST_KERNEL, CONFIG_SERIAL_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(CDC_ACM_INST);
