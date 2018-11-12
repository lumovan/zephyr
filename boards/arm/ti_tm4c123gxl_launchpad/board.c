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
        .dev = NULL, .pin = DW1000_GPIO_RST_PIN,
    }
};

struct dw1000_gpio_configuration* dw1000_configure_gpios(void)
{
    struct device* gpio;
    const int flags_int_in = (GPIO_DIR_IN | GPIO_INT
        | GPIO_INT_EDGE | GPIO_INT_ACTIVE_HIGH | GPIO_INT_DEBOUNCE);

    gpio = device_get_binding(CONFIG_IEEE802154_DW1000_GPIO_RST_DRV_NAME);
    dw1000_gpios[DW1000_GPIO_IDX_RST].dev = gpio;

    gpio = device_get_binding(CONFIG_IEEE802154_DW1000_GPIO_ISR_DRV_NAME);
    dw1000_gpios[DW1000_GPIO_IDX_ISR].dev = gpio;
    gpio_pin_configure(gpio, dw1000_gpios[DW1000_GPIO_IDX_ISR].pin,
        flags_int_in);

    return dw1000_gpios;
}

#endif /* CONFIG_IEEE802154_DW1000 */
