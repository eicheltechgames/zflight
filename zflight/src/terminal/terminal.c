/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cdc_acm_echo, LOG_LEVEL_INF);

const struct device *const uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

int init_terminal(void)
{
	int ret;

	if (!device_is_ready(uart_dev)) {
		LOG_ERR("CDC ACM device not ready");
		return 0;
	}

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
		ret = enable_usb_device_next();
#else
		ret = usb_enable(NULL);
#endif

	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return 0;
	}

	LOG_INF("Wait for DTR");

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
	k_sem_take(&dtr_sem, K_FOREVER);
#else
	while (true) {
		uint32_t dtr = 0U;

		uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr);
		if (dtr) {
			break;
		} else {
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
	}
#endif

	LOG_INF("DTR set");

	/* They are optional, we use them to test the interrupt endpoint */
	ret = uart_line_ctrl_set(uart_dev, UART_LINE_CTRL_DCD, 1);
	if (ret) {
		LOG_WRN("Failed to set DCD, ret code %d", ret);
	}

	ret = uart_line_ctrl_set(uart_dev, UART_LINE_CTRL_DSR, 1);
	if (ret) {
		LOG_WRN("Failed to set DSR, ret code %d", ret);
	}

	/* Wait 100ms for the host to do all settings */
	k_msleep(100);

	return 0;
}