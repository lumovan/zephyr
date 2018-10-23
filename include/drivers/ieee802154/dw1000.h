/*
 * Copyright (c) 2018 hackin zhao
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DW1000_H__
#define __DW1000_H__

#include <device.h>

/* RF settings
 *
 * First 42 entries are for the 42 first registers from
 * address 0x04 to 0x2D included.
 * Next, the last 58 entries are for the 58 registers from
 * extended address 0x00 to 0x39 included
 *
 * If CONFIG_IEEE802154_DW1000_RF_PRESET is not used, one will need
 * no provide 'dw1000_rf_settings' with proper settings. These can
 * be generated through TI's SmartRF application.
 *
 */
struct dw1000_rf_registers_set {
    u32_t chan_center_freq0;
    u16_t channel_limit;
    /* to fit in u16_t, spacing is a multiple of 100 Hz,
	 * 12.5KHz for instance will be 125.
	 */
    u16_t channel_spacing;
    u8_t registers[100];
};

#ifndef CONFIG_IEEE802154_DW1000_RF_PRESET
extern const struct dw1000_rf_registers_set dw1000_rf_settings;
#endif

enum dw1000_gpio_index {
    DW1000_GPIO_IDX_ISR = 0,
    DW1000_GPIO_IDX_WAKEUP,
    DW1000_GPIO_IDX_RST,
    DW1000_GPIO_IDX_EXTON,
    DW1000_GPIO_IDX_GPIO_5,
    DW1000_GPIO_IDX_GPIO_6,
    DW1000_GPIO_IDX_MAX,
};

struct dw1000_gpio_configuration {
    struct device* dev;
    u32_t pin;
};

struct dw1000_gpio_configuration* dw1000_configure_gpios(void);

#endif /* __DW1000_H__ */
