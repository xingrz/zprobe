/*
 * Copyright (c) 2023 XiNGRZ
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usb_device.h>
#include <usb_descriptor.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zprobe_cmsis_dap, CONFIG_ZPROBE_CMSIS_DAP_LOG_LEVEL);

#include "DAP.h"

#define USB_OUT_EP_IDX 0
#define USB_IN_EP_IDX  1

#define USB_BULK_EP_MPS 64

USBD_CLASS_DESCR_DEFINE(primary, 0) struct {
	struct usb_if_descriptor if0;
	struct usb_ep_descriptor if0_out_ep;
	struct usb_ep_descriptor if0_in_ep;
} dap_v2_desc = {
	.if0 =
		{
			.bLength = sizeof(struct usb_if_descriptor),
			.bDescriptorType = USB_DESC_INTERFACE,
			.bInterfaceNumber = 0,
			.bAlternateSetting = 0,
			.bNumEndpoints = 2,
			.bInterfaceClass = USB_BCC_VENDOR,
			.bInterfaceSubClass = 0,
			.bInterfaceProtocol = 0,
			.iInterface = 0,
		},
	.if0_out_ep =
		{
			.bLength = sizeof(struct usb_ep_descriptor),
			.bDescriptorType = USB_DESC_ENDPOINT,
			.bEndpointAddress = AUTO_EP_OUT,
			.bmAttributes = USB_DC_EP_BULK,
			.wMaxPacketSize = sys_cpu_to_le16(USB_BULK_EP_MPS),
			.bInterval = 0x00,
		},
	.if0_in_ep =
		{
			.bLength = sizeof(struct usb_ep_descriptor),
			.bDescriptorType = USB_DESC_ENDPOINT,
			.bEndpointAddress = AUTO_EP_IN,
			.bmAttributes = USB_DC_EP_BULK,
			.wMaxPacketSize = sys_cpu_to_le16(USB_BULK_EP_MPS),
			.bInterval = 0x00,
		},
};

static struct usb_ep_cfg_data dap_v2_ep_data[] = {
	{.ep_cb = usb_transfer_ep_callback, .ep_addr = AUTO_EP_OUT},
	{.ep_cb = usb_transfer_ep_callback, .ep_addr = AUTO_EP_IN},
};

#define DAP_V2_MAX_PACKET_SIZE 512

static uint8_t usb_rx_ep, usb_tx_ep;
static uint8_t usb_rx_buf[DAP_V2_MAX_PACKET_SIZE];
static uint8_t usb_tx_buf[DAP_V2_MAX_PACKET_SIZE];

static void dap_v2_write_cb(uint8_t ep, int size, void *priv)
{
	LOG_DBG("tx size: %u", size);
	LOG_HEXDUMP_DBG(usb_tx_buf, size, "tx data:");
}

static void dap_v2_read_cb(uint8_t ep, int size, void *priv)
{
	if (size <= 0) {
		goto next;
	}

	LOG_DBG("rx size: %u", size);
	LOG_HEXDUMP_DBG(usb_rx_buf, size, "rx data:");

	uint32_t ret, req_len, res_len;
	ret = DAP_ExecuteCommand(usb_rx_buf, usb_tx_buf);
	req_len = ret >> 16;
	res_len = ret & BIT_MASK(16);
	LOG_DBG("dap req size: %d", req_len);
	LOG_HEXDUMP_DBG(usb_rx_buf, req_len, "dap req data:");
	LOG_DBG("dap res size: %d", res_len);
	LOG_HEXDUMP_DBG(usb_tx_buf, res_len, "dap res data:");

	usb_transfer(usb_tx_ep, usb_tx_buf, res_len, USB_TRANS_WRITE, dap_v2_write_cb, NULL);

next:
	usb_transfer(usb_rx_ep, usb_rx_buf, sizeof(usb_rx_buf), USB_TRANS_READ, dap_v2_read_cb,
		     NULL);
}

static void dap_v2_status_cb(struct usb_cfg_data *cfg, enum usb_dc_status_code status,
			     const uint8_t *param)
{
	ARG_UNUSED(cfg);

	switch (status) {
	case USB_DC_CONFIGURED:
		LOG_DBG("USB device configured");
		usb_rx_ep = cfg->endpoint[USB_OUT_EP_IDX].ep_addr;
		usb_tx_ep = cfg->endpoint[USB_IN_EP_IDX].ep_addr;
		dap_v2_read_cb(usb_rx_ep, 0, NULL);
		break;
	default:
		break;
	}
}

USBD_DEFINE_CFG_DATA(dap_v2_config) = {
	.usb_device_description = NULL,
	.interface_descriptor = &dap_v2_desc.if0,
	.cb_usb_status = dap_v2_status_cb,
	.num_endpoints = ARRAY_SIZE(dap_v2_ep_data),
	.endpoint = dap_v2_ep_data,
};
