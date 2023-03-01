/*
 * Copyright (c) 2023 XiNGRZ
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT espressif_esp32_usb

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/usb_dc.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb_dc_esp32sx, CONFIG_USB_DRIVER_LOG_LEVEL);

#include <zephyr/drivers/interrupt_controller/intc_esp32.h>

#include <soc/soc.h>
#include <soc/usb_struct.h>
#include <soc/usb_types.h>
#include <soc/usb_pins.h>
#include <soc/usb_reg.h>
#include <soc/periph_defs.h>
#include <soc/gpio_pins.h>
#include <hal/clk_gate_ll.h>
#include <hal/usb_phy_ll.h>
#include <hal/gpio_ll.h>
#include <esp_rom_gpio.h>

#define GPIO0 ((gpio_dev_t *)DT_REG_ADDR(DT_NODELABEL(gpio0)))

#define USB0			 ((usb_dev_t *)DT_INST_REG_ADDR(0))
#define USB0_NUM_BIDIR_ENDPOINTS DT_INST_PROP(0, num_bidir_endpoints)

#define USB_WRAP ((usb_wrap_dev_t *)DR_REG_USB_WRAP_BASE)

#define EP_MPS 64U

// FIFO size in bytes
#define EP_FIFO_SIZE 1024

// Max number of IN EP FIFOs
#define EP_FIFO_NUM 5

static usb_dc_status_callback status_cb;

struct usb_dc_esp32_ep_state {
	uint16_t ep_mps;
	uint8_t ep_type;
	uint8_t ep_stalled;
	usb_dc_ep_callback cb;
};

struct usb_dc_esp32_state {
	usb_dc_status_callback status_cb;
	struct usb_dc_esp32_ep_state out_ep_state[USB0_NUM_BIDIR_ENDPOINTS];
	struct usb_dc_esp32_ep_state in_ep_state[USB0_NUM_BIDIR_ENDPOINTS];
	uint8_t ep_buf[USB0_NUM_BIDIR_ENDPOINTS][EP_MPS];
};

static struct usb_dc_esp32_state usb_dc_esp32_state;

static struct usb_dc_esp32_ep_state *usb_dc_esp32_get_ep_state(uint8_t ep)
{
	struct usb_dc_esp32_ep_state *ep_state_base;

	if (USB_EP_GET_IDX(ep) >= USB0_NUM_BIDIR_ENDPOINTS) {
		return NULL;
	}

	if (USB_EP_DIR_IS_OUT(ep)) {
		ep_state_base = usb_dc_esp32_state.out_ep_state;
	} else {
		ep_state_base = usb_dc_esp32_state.in_ep_state;
	}

	return ep_state_base + USB_EP_GET_IDX(ep);
}

// Keep count of how many FIFOs are in use
static uint8_t _allocated_fifos = 1; // FIFO0 is always in use

// Will either return an unused FIFO number, or 0 if all are used.
static uint8_t get_free_fifo(void)
{
	if (_allocated_fifos < EP_FIFO_NUM) {
		return _allocated_fifos++;
	}
	return 0;
}

static void usb_phy_init(void)
{
	periph_ll_enable_clk_clear_rst(PERIPH_USB_MODULE);
	periph_ll_reset(PERIPH_USB_MODULE);

	usb_phy_ll_int_otg_enable(USB_WRAP);

	gpio_ll_set_drive_capability(GPIO0, USBPHY_DM_NUM, GPIO_DRIVE_CAP_3);
	gpio_ll_set_drive_capability(GPIO0, USBPHY_DP_NUM, GPIO_DRIVE_CAP_3);

	esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, USB_OTG_IDDIG_IN_IDX,
				       false); // connected connector is mini-B side
	esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, USB_SRP_BVALID_IN_IDX,
				       false); // HIGH to force USB device mode
	esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, USB_OTG_VBUSVALID_IN_IDX,
				       false); // receiving a valid Vbus from device
	esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, USB_OTG_AVALID_IN_IDX, false);

	usb_phy_ll_int_load_conf(USB_WRAP, true, false, false, false);
}

static void usb_dc_init(void)
{
	// A. Disconnect
	LOG_DBG("soft DISCONNECT and setting up");
	USB0->dctl |= USB_SFTDISCON_M; // Soft disconnect

	// B. Programming DCFG
	/* If USB host misbehaves during status portion of control xfer
	  (non zero-length packet), send STALL back and discard. Full speed. */
	USB0->dcfg |= USB_NZSTSOUTHSHK_M | // NonZero .... STALL
		      (3 << 0); // dev speed: fullspeed 1.1 on 48 mhz  // TODO no value in usb_reg.h
				// (IDF-1476)

	USB0->gahbcfg |= USB_NPTXFEMPLVL_M | USB_GLBLLNTRMSK_M; // Global interruptions ON
	USB0->gusbcfg |= USB_FORCEDEVMODE_M;			// force devmode
	USB0->gotgctl &= ~(USB_BVALIDOVVAL_M | USB_BVALIDOVEN_M | USB_VBVALIDOVVAL_M); // no
										       // overrides

	// C. Setting SNAKs, then connect
	for (int n = 0; n < USB_OUT_EP_NUM; n++) {
		USB0->out_ep_reg[n].doepctl |= USB_DO_SNAK0_M; // DOEPCTL0_SNAK
	}

	// D. Interruption masking
	USB0->gintmsk = 0;   // mask all
	USB0->gotgint = ~0U; // clear OTG ints
	USB0->gintsts = ~0U; // clear pending ints
	USB0->gintmsk = USB_OTGINTMSK_M | USB_MODEMISMSK_M | USB_RXFLVIMSK_M | USB_ERLYSUSPMSK_M |
			USB_USBSUSPMSK_M | USB_USBRSTMSK_M | USB_ENUMDONEMSK_M | USB_RESETDETMSK_M |
			USB_DISCONNINTMSK_M; // host most only
}

static void IRAM_ATTR usb_dc_intr(void *arg)
{
	ARG_UNUSED(arg);

	const uint32_t int_msk = USB0->gintmsk;
	const uint32_t int_status = USB0->gintsts & int_msk;

	if (int_status & USB_USBRST_M) {
		// start of reset
		LOG_DBG("reset");
		USB0->gintsts = USB_USBRST_M;
		// FIFOs will be reassigned when the endpoints are reopen
		_allocated_fifos = 1;
		usb_dc_reset();
		if (usb_dc_esp32_state.status_cb) {
			usb_dc_esp32_state.status_cb(USB_DC_RESET, NULL);
		}
	}

	if (int_status & USB_RESETDET_M) {
		LOG_DBG("reset while suspend");
		USB0->gintsts = USB_RESETDET_M;
		usb_dc_reset();
		if (usb_dc_esp32_state.status_cb) {
			usb_dc_esp32_state.status_cb(USB_DC_RESET, NULL);
		}
	}

	if (int_status & USB_USBSUSP_M) {
		USB0->gintsts = USB_USBSUSP_M;
		if (usb_dc_esp32_state.status_cb) {
			usb_dc_esp32_state.status_cb(USB_DC_SUSPEND, NULL);
		}
	}

	if (int_status & USB_WKUPINT_M) {
		USB0->gintsts = USB_WKUPINT_M;
		if (usb_dc_esp32_state.status_cb) {
			usb_dc_esp32_state.status_cb(USB_DC_RESUME, NULL);
		}
	}

	if (int_status & USB_OTGINT_M) {
		// OTG INT bit is read-only
		LOG_DBG("disconnected");

		uint32_t const otg_int = USB0->gotgint;

		if (otg_int & USB_SESENDDET_M && usb_dc_esp32_state.status_cb) {
			usb_dc_esp32_state.status_cb(USB_DC_DISCONNECTED, NULL);
		}

		USB0->gotgint = otg_int;
	}

	if (int_status & USB_SOF_M) {
		USB0->gintsts = USB_SOF_M;

		// Disable SOF interrupt since currently only used for remote wakeup detection
		USB0->gintmsk &= ~USB_SOFMSK_M;

		if (usb_dc_esp32_state.status_cb) {
			usb_dc_esp32_state.status_cb(USB_DC_SOF, NULL);
		}
	}

	if (int_status & USB_RXFLVI_M) {
		// RXFLVL bit is read-only
		LOG_DBG("rx!");

		// Mask out RXFLVL while reading data from FIFO
		USB0->gintmsk &= ~USB_RXFLVIMSK_M;
		// read_rx_fifo();
		USB0->gintmsk |= USB_RXFLVIMSK_M;
	}

	// OUT endpoint interrupt handling.
	if (int_status & USB_OEPINT_M) {
		// OEPINT is read-only
		LOG_DBG("OUT endpoint!");
		// handle_epout_ints();
	}

	// IN endpoint interrupt handling.
	if (int_status & USB_IEPINT_M) {
		// IEPINT bit read-only
		LOG_DBG("IN endpoint!");
		// handle_epin_ints();
	}

	// Without handling
	USB0->gintsts |= USB_CURMOD_INT_M | USB_MODEMIS_M | USB_OTGINT_M | USB_NPTXFEMP_M |
			 USB_GINNAKEFF_M | USB_GOUTNAKEFF | USB_ERLYSUSP_M | USB_USBSUSP_M |
			 USB_ISOOUTDROP_M | USB_EOPF_M | USB_EPMIS_M | USB_INCOMPISOIN_M |
			 USB_INCOMPIP_M | USB_FETSUSP_M | USB_PTXFEMP_M;
}

int usb_dc_attach(void)
{
	usb_phy_init();
	usb_dc_init();

	esp_intr_alloc(DT_INST_IRQN(0), ESP_INTR_FLAG_LOWMED, (intr_handler_t)usb_dc_intr, NULL,
		       NULL);

	LOG_DBG("connect");
	USB0->dctl &= ~USB_SFTDISCON_M;

	return 0;
}

int usb_dc_detach(void)
{
	LOG_DBG("disconnect");
	USB0->dctl |= USB_SFTDISCON_M;
	return 0;
}

int usb_dc_reset(void)
{
	for (int ep_num = 0; ep_num < USB_OUT_EP_NUM; ep_num++) {
		USB0->out_ep_reg[ep_num].doepctl |= USB_DO_SNAK0_M; // DOEPCTL0_SNAK
	}

	// clear device address
	USB0->dcfg &= ~USB_DEVADDR_M;

	USB0->daintmsk = USB_OUTEPMSK0_M | USB_INEPMSK0_M;
	USB0->doepmsk = USB_SETUPMSK_M | USB_XFERCOMPLMSK;
	USB0->diepmsk = USB_TIMEOUTMSK_M | USB_DI_XFERCOMPLMSK_M /*| USB_INTKNTXFEMPMSK_M*/;

	// "USB Data FIFOs" section in reference manual
	// Peripheral FIFO architecture
	//
	// --------------- 320 or 1024 ( 1280 or 4096 bytes )
	// | IN FIFO MAX |
	// ---------------
	// |    ...      |
	// --------------- y + x + 16 + GRXFSIZ
	// | IN FIFO 2   |
	// --------------- x + 16 + GRXFSIZ
	// | IN FIFO 1   |
	// --------------- 16 + GRXFSIZ
	// | IN FIFO 0   |
	// --------------- GRXFSIZ
	// | OUT FIFO    |
	// | ( Shared )  |
	// --------------- 0
	//
	// According to "FIFO RAM allocation" section in RM, FIFO RAM are allocated as follows (each
	// word 32-bits):
	// - Each EP IN needs at least max packet size, 16 words is sufficient for EP0 IN
	//
	// - All EP OUT shared a unique OUT FIFO which uses
	//   * 10 locations in hardware for setup packets + setup control words (up to 3 setup
	//   packets).
	//   * 2 locations for OUT endpoint control words.
	//   * 16 for largest packet size of 64 bytes. ( TODO Highspeed is 512 bytes)
	//   * 1 location for global NAK (not required/used here).
	//   * It is recommended to allocate 2 times the largest packet size, therefore
	//   Recommended value = 10 + 1 + 2 x (16+2) = 47 --> Let's make it 52
	USB0->grstctl |= 0x10 << USB_TXFNUM_S; // fifo 0x10,
	USB0->grstctl |= USB_TXFFLSH_M;	       // Flush fifo
	USB0->grxfsiz = 52;

	// Control IN uses FIFO 0 with 64 bytes ( 16 32-bit word )
	USB0->gnptxfsiz = (16 << USB_NPTXFDEP_S) | (USB0->grxfsiz & 0x0000ffffUL);

	// Ready to receive SETUP packet
	USB0->out_ep_reg[0].doeptsiz |= USB_SUPCNT0_M;

	USB0->gintmsk |= USB_IEPINTMSK_M | USB_OEPINTMSK_M;

	return 0;
}

int usb_dc_set_address(const uint8_t addr)
{
	LOG_DBG("addr: %u", addr);
	USB0->dcfg |= ((addr & USB_DEVADDR_V) << USB_DEVADDR_S);
	return 0;
}

void usb_dc_set_status_callback(const usb_dc_status_callback cb)
{
	usb_dc_esp32_state.status_cb = cb;
}

int usb_dc_ep_check_cap(const struct usb_dc_ep_cfg_data *const cfg)
{
	uint8_t ep_idx = USB_EP_GET_IDX(cfg->ep_addr);

	if ((cfg->ep_type == USB_DC_EP_CONTROL) && ep_idx) {
		LOG_ERR("Invalid type %d for ep %d", cfg->ep_type, ep_idx);
		return -1;
	}

	if (ep_idx > USB0_NUM_BIDIR_ENDPOINTS) {
		LOG_ERR("Endpoint index/address too high: %d", ep_idx);
		return -1;
	}

	return 0;
}

int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data *const cfg)
{
	uint8_t ep = cfg->ep_addr;
	struct usb_dc_esp32_ep_state *ep_state = usb_dc_esp32_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	LOG_DBG("ep 0x%02x, mps %u -> %u, type %u", cfg->ep_addr, ep_state->ep_mps, cfg->ep_mps,
		cfg->ep_type);

	ep_state->ep_mps = cfg->ep_mps;

	switch (cfg->ep_type) {
	case USB_DC_EP_CONTROL:
	case USB_DC_EP_ISOCHRONOUS:
	case USB_DC_EP_BULK:
	case USB_DC_EP_INTERRUPT:
		ep_state->ep_type = cfg->ep_type;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int usb_dc_ep_set_stall(const uint8_t ep)
{
	struct usb_dc_esp32_ep_state *ep_state = usb_dc_esp32_get_ep_state(ep);

	usb_out_endpoint_t *out_ep = &(USB0->out_ep_reg[0]);
	usb_in_endpoint_t *in_ep = &(USB0->in_ep_reg[0]);

	uint8_t const epnum = USB_EP_GET_IDX(ep);
	uint8_t const dir_in = USB_EP_GET_DIR(ep);

	if (dir_in) {
		// Only disable currently enabled non-control endpoint
		if ((epnum == 0) || !(in_ep[epnum].diepctl & USB_D_EPENA1_M)) {
			in_ep[epnum].diepctl |= (USB_DI_SNAK1_M | USB_D_STALL1_M);
		} else {
			// Stop transmitting packets and NAK IN xfers.
			in_ep[epnum].diepctl |= USB_DI_SNAK1_M;
			while ((in_ep[epnum].diepint & USB_DI_SNAK1_M) == 0) {
				;
			}

			// Disable the endpoint. Note that both SNAK and STALL are set here.
			in_ep[epnum].diepctl |= (USB_DI_SNAK1_M | USB_D_STALL1_M | USB_D_EPDIS1_M);
			while ((in_ep[epnum].diepint & USB_D_EPDISBLD0_M) == 0) {
				;
			}
			in_ep[epnum].diepint = USB_D_EPDISBLD0_M;
		}

		// Flush the FIFO, and wait until we have confirmed it cleared.
		uint8_t const fifo_num =
			((in_ep[epnum].diepctl >> USB_D_TXFNUM1_S) & USB_D_TXFNUM1_V);
		USB0->grstctl |= (fifo_num << USB_TXFNUM_S);
		USB0->grstctl |= USB_TXFFLSH_M;
		while ((USB0->grstctl & USB_TXFFLSH_M) != 0) {
			;
		}
	} else {
		// Only disable currently enabled non-control endpoint
		if ((epnum == 0) || !(out_ep[epnum].doepctl & USB_EPENA0_M)) {
			out_ep[epnum].doepctl |= USB_STALL0_M;
		} else {
			// Asserting GONAK is required to STALL an OUT endpoint.
			// Simpler to use polling here, we don't use the "B"OUTNAKEFF interrupt
			// anyway, and it can't be cleared by user code. If this while loop never
			// finishes, we have bigger problems than just the stack.
			USB0->dctl |= USB_SGOUTNAK_M;
			while ((USB0->gintsts & USB_GOUTNAKEFF_M) == 0) {
				;
			}

			// Ditto here- disable the endpoint. Note that only STALL and not SNAK
			// is set here.
			out_ep[epnum].doepctl |= (USB_STALL0_M | USB_EPDIS0_M);
			while ((out_ep[epnum].doepint & USB_EPDISBLD0_M) == 0) {
				;
			}
			out_ep[epnum].doepint = USB_EPDISBLD0_M;

			// Allow other OUT endpoints to keep receiving.
			USB0->dctl |= USB_CGOUTNAK_M;
		}
	}

	ep_state->ep_stalled = 1U;

	return 0;
}

int usb_dc_ep_clear_stall(const uint8_t ep)
{
	struct usb_dc_esp32_ep_state *ep_state = usb_dc_esp32_get_ep_state(ep);

	usb_out_endpoint_t *out_ep = &(USB0->out_ep_reg[0]);
	usb_in_endpoint_t *in_ep = &(USB0->in_ep_reg[0]);

	uint8_t const epnum = USB_EP_GET_IDX(ep);
	uint8_t const dir_in = USB_EP_GET_DIR(ep);

	if (dir_in) {
		in_ep[epnum].diepctl &= ~USB_D_STALL1_M;

		uint8_t eptype = (in_ep[epnum].diepctl & USB_D_EPTYPE1_M) >> USB_D_EPTYPE1_S;
		// Required by USB spec to reset DATA toggle bit to DATA0 on interrupt
		// and bulk endpoints.
		if (eptype == 2 || eptype == 3) {
			in_ep[epnum].diepctl |= USB_DI_SETD0PID1_M;
		}
	} else {
		out_ep[epnum].doepctl &= ~USB_STALL1_M;

		uint8_t eptype = (out_ep[epnum].doepctl & USB_EPTYPE1_M) >> USB_EPTYPE1_S;
		// Required by USB spec to reset DATA toggle bit to DATA0 on interrupt
		// and bulk endpoints.
		if (eptype == 2 || eptype == 3) {
			out_ep[epnum].doepctl |= USB_DO_SETD0PID1_M;
		}
	}

	ep_state->ep_stalled = 0U;

	return 0;
}

int usb_dc_ep_is_stalled(const uint8_t ep, uint8_t *const stalled)
{
	struct usb_dc_esp32_ep_state *ep_state = usb_dc_esp32_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state || !stalled) {
		return -EINVAL;
	}

	*stalled = ep_state->ep_stalled;

	return 0;
}

int usb_dc_ep_halt(const uint8_t ep)
{
	return usb_dc_ep_set_stall(ep);
}

int usb_dc_ep_enable(const uint8_t ep)
{
	struct usb_dc_esp32_ep_state *ep_state = usb_dc_esp32_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

	usb_out_endpoint_t *out_ep = &(USB0->out_ep_reg[0]);
	usb_in_endpoint_t *in_ep = &(USB0->in_ep_reg[0]);

	uint8_t const epnum = USB_EP_GET_IDX(ep);
	uint8_t const dir_in = USB_EP_GET_DIR(ep);

	if (dir_in) {
		out_ep[epnum].doepctl |=
			(ep_state->ep_type != USB_DC_EP_ISOCHRONOUS ? USB_DO_SETD0PID1_M : 0) |
			(ep_state->ep_type << USB_EPTYPE1_S) | (USB_USBACTEP1_M) |
			(ep_state->ep_mps << USB_MPS1_S);
		USB0->daintmsk |= (1 << (16 + epnum));
	} else {
		// "USB Data FIFOs" section in reference manual
		// Peripheral FIFO architecture
		//
		// --------------- 320 or 1024 ( 1280 or 4096 bytes )
		// | IN FIFO MAX |
		// ---------------
		// |    ...      |
		// --------------- y + x + 16 + GRXFSIZ
		// | IN FIFO 2   |
		// --------------- x + 16 + GRXFSIZ
		// | IN FIFO 1   |
		// --------------- 16 + GRXFSIZ
		// | IN FIFO 0   |
		// --------------- GRXFSIZ
		// | OUT FIFO    |
		// | ( Shared )  |
		// --------------- 0
		//
		// Since OUT FIFO = GRXFSIZ, FIFO 0 = 16, for simplicity, we equally allocated for
		// the rest of endpoints
		// - Size  : (FIFO_SIZE/4 - GRXFSIZ - 16) / (EP_MAX-1)
		// - Offset: GRXFSIZ + 16 + Size*(epnum-1)
		// - IN EP 1 gets FIFO 1, IN EP "n" gets FIFO "n".

		uint8_t fifo_num = get_free_fifo();
		assert(fifo_num != 0);

		in_ep[epnum].diepctl &=
			~(USB_D_TXFNUM1_M | USB_D_EPTYPE1_M | USB_DI_SETD0PID1 | USB_D_MPS1_M);
		in_ep[epnum].diepctl |=
			(ep_state->ep_type != USB_DC_EP_ISOCHRONOUS ? USB_DI_SETD0PID1_M : 0) |
			(fifo_num << USB_D_TXFNUM1_S) | (ep_state->ep_type << USB_D_EPTYPE1_S) |
			(USB_D_USBACTEP1_M) | (ep_state->ep_mps << USB_D_MPS1_S);

		USB0->daintmsk |= (1 << (0 + epnum));

		// Both TXFD and TXSA are in unit of 32-bit words.
		// IN FIFO 0 was configured during enumeration, hence the "+ 16".
		uint16_t const allocated_size = (USB0->grxfsiz & 0x0000ffff) + 16;
		uint16_t const fifo_size = (EP_FIFO_SIZE / 4 - allocated_size) / (EP_FIFO_NUM - 1);
		uint32_t const fifo_offset = allocated_size + fifo_size * (fifo_num - 1);

		// DIEPTXF starts at FIFO #1.
		USB0->dieptxf[epnum - 1] = (fifo_size << USB_NPTXFDEP_S) | fifo_offset;
	}

	return 0;
}

int usb_dc_ep_disable(const uint8_t ep)
{
	struct usb_dc_esp32_ep_state *ep_state = usb_dc_esp32_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

	usb_out_endpoint_t *out_ep = &(USB0->out_ep_reg[0]);
	usb_in_endpoint_t *in_ep = &(USB0->in_ep_reg[0]);

	uint8_t const epnum = USB_EP_GET_IDX(ep);
	uint8_t const dir_in = USB_EP_GET_DIR(ep);

	if (dir_in) {
		out_ep[epnum].doepctl = 0;
	} else {
		in_ep[epnum].diepctl = 0;
	}

	return 0;
}

int usb_dc_ep_flush(const uint8_t ep)
{
	struct usb_dc_esp32_ep_state *ep_state = usb_dc_esp32_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	return 0;
}

int usb_dc_ep_write(const uint8_t ep, const uint8_t *const data, const uint32_t data_len,
		    uint32_t *const ret_bytes)
{
	return -ENOTSUP;
}

int usb_dc_ep_read(const uint8_t ep, uint8_t *const data, const uint32_t max_data_len,
		   uint32_t *const read_bytes)
{
	return -ENOTSUP;
}

int usb_dc_ep_set_callback(const uint8_t ep, const usb_dc_ep_callback cb)
{
	struct usb_dc_esp32_ep_state *ep_state = usb_dc_esp32_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

	ep_state->cb = cb;

	return 0;
}

int usb_dc_ep_read_wait(uint8_t ep, uint8_t *data, uint32_t max_data_len, uint32_t *read_bytes)
{
	return -ENOTSUP;
}

int usb_dc_ep_read_continue(uint8_t ep)
{
	return -ENOTSUP;
}

int usb_dc_ep_mps(uint8_t ep)
{
	struct usb_dc_esp32_ep_state *ep_state = usb_dc_esp32_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	return ep_state->ep_mps;
}

int usb_dc_wakeup_request(void)
{
	// set remote wakeup
	USB0->dctl |= USB_RMTWKUPSIG_M;

	// enable SOF to detect bus resume
	USB0->gintsts = USB_SOF_M;
	USB0->gintmsk |= USB_SOFMSK_M;

	// Per specs: remote wakeup signal bit must be clear within 1-15ms
	k_msleep(1);

	USB0->dctl &= ~USB_RMTWKUPSIG_M;

	return 0;
}
