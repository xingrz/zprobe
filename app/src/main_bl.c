/*
 * Copyright (c) 2023 XiNGRZ
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/arch/arm/aarch32/cortex_m/cmsis.h>

#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zprobe, CONFIG_ZPROBE_LOG_LEVEL);

#define FLASH_ADDR DT_REG_ADDR(DT_CHOSEN(zephyr_flash))
#define SRAM_ADDR  DT_REG_ADDR(DT_CHOSEN(zephyr_sram))
#define BOOT_ADDR  DT_REG_ADDR(DT_CHOSEN(zprobe_boot_partition))

#define APP_ADDR (FLASH_ADDR + BOOT_ADDR)

#define NRESET_NODE DT_ALIAS(nreset)
#define NRESET_OKAY DT_NODE_HAS_STATUS(NRESET_NODE, okay)

#if NRESET_OKAY
static struct gpio_dt_spec nreset = GPIO_DT_SPEC_GET(NRESET_NODE, gpios);
#endif

#define LED_RUN_NODE DT_ALIAS(led_run)
#define LED_RUN_OKAY DT_NODE_HAS_STATUS(LED_RUN_NODE, okay)

#if LED_RUN_OKAY
static struct gpio_dt_spec led_run = GPIO_DT_SPEC_GET(LED_RUN_NODE, gpios);
#endif

struct arm_vector_table {
	uint32_t msp;
	uint32_t reset;
};

static const struct arm_vector_table *vt = (struct arm_vector_table *)APP_ADDR;

static inline bool check_app(void)
{
	if ((vt->msp & 0x2FFE0000) != SRAM_ADDR) {
		LOG_ERR("Incorrect header");
		return false;
	}

	return true;
}

static inline void jump_to_app(void)
{
#if IS_ENABLED(CONFIG_SYSTEM_TIMER_HAS_DISABLE_SUPPORT)
	sys_clock_disable();
#endif

	irq_lock();

	SCB->VTOR = (uint32_t)vt;
	__set_MSP(vt->msp);

	((void (*)(void))vt->reset)();
}

void main(void)
{
	LOG_DBG("Hello world!");

	bool update_mode = false;

#if NRESET_OKAY
	if (!gpio_is_ready_dt(&nreset)) {
		LOG_ERR("NRESET is not ready");
		return;
	}

	gpio_pin_configure_dt(&nreset, GPIO_INPUT);
#endif

#if LED_RUN_OKAY
	if (!gpio_is_ready_dt(&led_run)) {
		LOG_ERR("LED_RUN is not ready");
		return;
	}

	gpio_pin_configure_dt(&led_run, GPIO_OUTPUT_INACTIVE);
#endif

#if NRESET_OKAY
	update_mode = gpio_pin_get_dt(&nreset);
	LOG_DBG("nreset: %d", update_mode);
#endif

	if (!update_mode && !check_app()) {
		update_mode = true;
	}

	if (!update_mode) {
		jump_to_app();
		while (1) {
		}
	}

	LOG_DBG("Bootloader mode");

	// TODO: Set up MSC

	while (1) {
#if LED_RUN_OKAY
		gpio_pin_toggle_dt(&led_run);
		k_msleep(500);
#endif
	}
}
