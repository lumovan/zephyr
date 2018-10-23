/*
 * Copyright (c) 2018, hackin, zhao.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "board.h"
#include <device.h>
#include <init.h>
#include <kernel.h>
#include <uart.h>

#if defined(CONFIG_IEEE802154_DW1000)

#include <gpio.h>
#include <ieee802154/dw1000.h>

static struct dw1000_gpio_configuration dw1000_gpios[DW1000_GPIO_IDX_MAX] = {
    {
        .dev = NULL, .pin = DW1000_GPIO_ISR_PIN,
    },
    {
        .dev = NULL, .pin = DW1000_GPIO_WAKEUP_PIN,
    },
    {
        .dev = NULL, .pin = DW1000_GPIO_RST_PIN,
    },
    {
        .dev = NULL, .pin = DW1000_GPIO_EXTON_PIN,
    },
    {
        .dev = NULL, .pin = DW1000_GPIO_GPIO_5_PIN, /* SPI POL */
    },
    {
        .dev = NULL, .pin = DW1000_GPIO_GPIO_6_PIN, /* SPI PHA */
    },
};

struct dw1000_gpio_configuration* dw1000_configure_gpios(void)
{
    const int flags_noint_out = GPIO_DIR_OUT;
    struct device* gpio;

    gpio = device_get_binding(CONFIG_IEEE802154_DW1000_GPIO_ISR_DRV_NAME);
    dw1000_gpios[DW1000_GPIO_IDX_ISR].dev = gpio;

    gpio = device_get_binding(CONFIG_IEEE802154_DW1000_GPIO_WAKEUP_DRV_NAME);
    dw1000_gpios[DW1000_GPIO_IDX_WAKEUP].dev = gpio;

    gpio = device_get_binding(CONFIG_IEEE802154_DW1000_GPIO_RST_DRV_NAME);
    dw1000_gpios[DW1000_GPIO_IDX_RST].dev = gpio;

    gpio = device_get_binding(CONFIG_IEEE802154_DW1000_GPIO_EXTON_DRV_NAME);
    dw1000_gpios[DW1000_GPIO_IDX_EXTON].dev = gpio;

    gpio = device_get_binding(CONFIG_IEEE802154_DW1000_GPIO_GPIO_5_DRV_NAME);
    dw1000_gpios[DW1000_GPIO_IDX_GPIO_5].dev = gpio;

    gpio = device_get_binding(CONFIG_IEEE802154_DW1000_GPIO_GPIO_6_DRV_NAME);
    dw1000_gpios[DW1000_GPIO_IDX_GPIO_6].dev = gpio;

    return dw1000_gpios;
}

#endif /* CONFIG_IEEE802154_DW1000 */
