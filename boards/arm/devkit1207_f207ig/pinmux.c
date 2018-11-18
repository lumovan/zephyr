/*
 * Copyright (c) 2018 hackin Zhao
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <init.h>
#include <kernel.h>
#include <pinmux.h>
#include <sys_io.h>

#include <pinmux/stm32/pinmux_stm32.h>

static const struct pin_config pinconf[] = {
#ifdef CONFIG_UART_STM32_PORT_3
    { STM32_PIN_PC10, STM32F2_PINMUX_FUNC_PC10_USART3_TX },
    { STM32_PIN_PC11, STM32F2_PINMUX_FUNC_PC11_USART3_RX },
#endif /* #ifdef CONFIG_UART_STM32_PORT_3 */
#ifdef CONFIG_SPI_1
    { STM32_PIN_PA4, STM32F2_PINMUX_FUNC_PA4_SPI1_NSS },
    { STM32_PIN_PA5, STM32F2_PINMUX_FUNC_PA5_SPI1_SCK },
    { STM32_PIN_PA6, STM32F2_PINMUX_FUNC_PA6_SPI1_MISO },
    { STM32_PIN_PA7, STM32F2_PINMUX_FUNC_PA7_SPI1_MOSI },
#endif /* CONFIG_SPI_1 */
};

static int pinmux_stm32_init(struct device* port)
{
    ARG_UNUSED(port);

    stm32_setup_pins(pinconf, ARRAY_SIZE(pinconf));

    return 0;
}

SYS_INIT(pinmux_stm32_init, PRE_KERNEL_1,
    CONFIG_PINMUX_STM32_DEVICE_INITIALIZATION_PRIORITY);
