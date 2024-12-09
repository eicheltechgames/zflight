/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zflight/terminal.h>
#include <zflight/app.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_backend.h>
#include <zephyr/sys/util.h>

const struct device *const uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

USBD_DEVICE_DEFINE(usbd_dev,
                   DEVICE_DT_GET(DT_NODELABEL(zflight_terminal)),
                   CONFIG_USB_DEVICE_VID, CONFIG_USB_DEVICE_PID);

USBD_DESC_LANG_DEFINE(usbd_lang);
USBD_DESC_MANUFACTURER_DEFINE(usbd_mfr, CONFIG_USB_DEVICE_MANUFACTURER);
USBD_DESC_PRODUCT_DEFINE(usbd_product, "zflight");
USBD_DESC_SERIAL_NUMBER_DEFINE(usbd_sn);

USBD_CONFIGURATION_DEFINE(usbd_fs_config,
                          USB_SCD_SELF_POWERED,
                          125); // @todo make configurable per flight controller

int zflight_terminal_start()
{
    return shell_start(shell_backend_get(0));
}

int zflight_terminal_stop()
{
    return shell_stop(shell_backend_get(0));
}

static void usbd_msg_cb(struct usbd_context *const ctx,
                          const struct usbd_msg *msg)
{
    int err = 0;

    if (usbd_can_detect_vbus(ctx)) {
        if (msg->type == USBD_MSG_VBUS_READY) {
            err = usbd_enable(ctx);
            if (err) {
                // @todo error
            }
        }

        if (msg->type == USBD_MSG_VBUS_REMOVED) {
            err = usbd_disable(ctx);
            if (err) {
                // @todo error
            }
        }
    }
    
    if (msg->type == USBD_MSG_CDC_ACM_CONTROL_LINE_STATE) {
        uint32_t dtr = 0U;
        err = uart_line_ctrl_get(msg->dev, UART_LINE_CTRL_DTR, &dtr);
        if (err) {
            // @todo error
            return;
        }

        if (dtr) {
            zflight_app_notify_event(
                ZFLIGHT_APP_EVENT_TERMINAL, ZFLIGHT_TERMINAL_CONNECTED);
        } else {
            (void)zflight_terminal_stop();
            zflight_app_notify_event(
                ZFLIGHT_APP_EVENT_TERMINAL, ZFLIGHT_TERMINAL_DISCONNECTED);
        }
    }
}

int zflight_terminal_init()
{
    int err;

    if (!device_is_ready(uart_dev)) {
        return -EIO;
    }

    err = usbd_add_descriptor(&usbd_dev, &usbd_lang);
    if (err) {
        return err;
    }

    err = usbd_add_descriptor(&usbd_dev, &usbd_mfr);
    if (err) {
        return err;
    }

    err = usbd_add_descriptor(&usbd_dev, &usbd_product);
    if (err) {
        return err;
    }

    err = usbd_add_descriptor(&usbd_dev, &usbd_sn);
    if (err) {
        return err;
    }

    err = usbd_add_configuration(&usbd_dev, USBD_SPEED_FS, &usbd_fs_config);
    if (err) {
        return err;
    }

    err = usbd_register_all_classes(&usbd_dev, USBD_SPEED_FS, 1);
    if (err) {
        return err;
    }

    err = usbd_msg_register_cb(&usbd_dev, usbd_msg_cb);
    if (err) {
        return err;
    }

    err = usbd_init(&usbd_dev);
    if (err) {
        return err;
    }

    if (!usbd_can_detect_vbus(&usbd_dev)) {
        err = usbd_enable(&usbd_dev);
        if (err) {
            return err;
        }
    }

    return 0;
}