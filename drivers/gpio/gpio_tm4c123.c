/*
 * Copyright (c) 2018, hackin, zhao
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>

#include <device.h>
#include <gpio.h>
#include <init.h>
#include <kernel.h>
#include <sys_io.h>

/* Driverlib includes */
#include <driverlib/pin_map.h>
#include <driverlib/rom.h>
#include <hw_ints.h>
#include <inc/hw_gpio.h>
#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>

#undef __GPIO_H__ /* Zephyr and TM4C123xx SDK gpio.h conflict */
#include <driverlib/gpio.h>
#include <driverlib/interrupt.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>

#include "gpio_utils.h"

/* Note: Zephyr uses exception numbers, vs the IRQ #s used by the TM4C123 SDK */
#define EXCEPTION_GPIOA 0 /* (INT_GPIOA - 16) = (16-16) */
#define EXCEPTION_GPIOB 1 /* (INT_GPIOB - 16) = (17-16) */
#define EXCEPTION_GPIOC 2 /* (INT_GPIOC - 16) = (18-16) */
#define EXCEPTION_GPIOD 3 /* (INT_GPIOD - 16) = (19-16) */
#define EXCEPTION_GPIOE 4 /* (INT_GPIOE - 16) = (20-16) */
#define EXCEPTION_GPIOF 30 /* (INT_GPIOF - 16) = (46-16) */
#define EXCEPTION_GPIOG 31 /* (INT_GPIOG - 16) = (47-16) */
#define EXCEPTION_GPIOH 32 /* (INT_GPIOH - 16) = (48-16) */

struct gpio_tm4c123_config {
    /* base address of GPIO port */
    unsigned long port_base;
    /* GPIO IRQ number */
    unsigned long irq_num;
};

struct gpio_tm4c123_data {
    /* list of registered callbacks */
    sys_slist_t callbacks;
    /* callback enable pin bitmask */
    u32_t pin_callback_enables;
};

#define DEV_CFG(dev) \
    ((const struct gpio_tm4c123_config*)(dev)->config->config_info)
#define DEV_DATA(dev) \
    ((struct gpio_tm4c123_data*)(dev)->driver_data)

static inline int gpio_tm4c123_config(struct device* port,
    int access_op, u32_t pin, int flags)
{
    const struct gpio_tm4c123_config* gpio_config = DEV_CFG(port);
    unsigned long port_base = gpio_config->port_base;
    unsigned long int_type;

    /*
	 * See pinmux_initialize(): which leverages TI's recommended
	 * method of using the PinMux utility for most pin configuration.
	 */

    if (access_op == GPIO_ACCESS_BY_PIN) {
        /* Just handle runtime interrupt type config here: */
        if (flags & GPIO_INT) {
            if (flags & GPIO_INT_EDGE) {
                if (flags & GPIO_INT_ACTIVE_HIGH) {
                    int_type = GPIO_RISING_EDGE;
                } else if (flags & GPIO_INT_DOUBLE_EDGE) {
                    int_type = GPIO_BOTH_EDGES;
                } else {
                    int_type = GPIO_FALLING_EDGE;
                }
            } else { /* GPIO_INT_LEVEL */
                if (flags & GPIO_INT_ACTIVE_HIGH) {
                    int_type = GPIO_HIGH_LEVEL;
                } else {
                    int_type = GPIO_LOW_LEVEL;
                }
            }
            MAP_GPIOIntTypeSet(port_base, (1 << pin), int_type);
            GPIOIntClear(port_base, (1 << pin));
            GPIOIntEnable(port_base, (1 << pin));
        }
    } else {
        return -ENOTSUP;
    }

    return 0;
}

static inline int gpio_tm4c123_write(struct device* port,
    int access_op, u32_t pin, u32_t value)
{
    const struct gpio_tm4c123_config* gpio_config = DEV_CFG(port);
    unsigned long port_base = gpio_config->port_base;

    if (access_op == GPIO_ACCESS_BY_PIN) {
        value = value << pin;
        /* Bitpack external GPIO pin number for GPIOPinWrite API: */
        pin = 1 << pin;

        MAP_GPIOPinWrite(port_base, (unsigned char)pin, value);
    } else {
        return -ENOTSUP;
    }

    return 0;
}

static inline int gpio_tm4c123_read(struct device* port,
    int access_op, u32_t pin, u32_t* value)
{
    const struct gpio_tm4c123_config* gpio_config = DEV_CFG(port);
    unsigned long port_base = gpio_config->port_base;
    long status;
    unsigned char pin_packed;

    if (access_op == GPIO_ACCESS_BY_PIN) {
        /* Bitpack external GPIO pin number for GPIOPinRead API: */
        pin_packed = 1 << pin;
        status = MAP_GPIOPinRead(port_base, pin_packed);
        *value = status >> pin;
    } else {
        return -ENOTSUP;
    }

    return 0;
}

static int gpio_tm4c123_manage_callback(struct device* dev,
    struct gpio_callback* callback, bool set)
{
    struct gpio_tm4c123_data* data = DEV_DATA(dev);

    _gpio_manage_callback(&data->callbacks, callback, set);

    return 0;
}

static int gpio_tm4c123_enable_callback(struct device* dev,
    int access_op, u32_t pin)
{
    struct gpio_tm4c123_data* data = DEV_DATA(dev);

    if (access_op == GPIO_ACCESS_BY_PIN) {
        data->pin_callback_enables |= (1 << pin);
    } else {
        data->pin_callback_enables = 0xFFFFFFFF;
    }

    return 0;
}

static int gpio_tm4c123_disable_callback(struct device* dev,
    int access_op, u32_t pin)
{
    struct gpio_tm4c123_data* data = DEV_DATA(dev);

    if (access_op == GPIO_ACCESS_BY_PIN) {
        data->pin_callback_enables &= ~(1 << pin);
    } else {
        data->pin_callback_enables = 0;
    }

    return 0;
}

static void gpio_tm4c123_port_isr(void* arg)
{
    struct device* dev = arg;
    const struct gpio_tm4c123_config* config = DEV_CFG(dev);
    struct gpio_tm4c123_data* data = DEV_DATA(dev);
    u32_t enabled_int, int_status;

    /* See which interrupts triggered: */
    int_status = (u32_t)GPIOIntStatus(config->port_base, 1);

    enabled_int = int_status & data->pin_callback_enables;

    /* Clear and Disable GPIO Interrupt */
    GPIOIntDisable(config->port_base, int_status);
    GPIOIntClear(config->port_base, int_status);

    /* Call the registered callbacks */
    _gpio_fire_callbacks(&data->callbacks, (struct device*)dev,
        enabled_int);

    /* Re-enable the interrupts */
    GPIOIntEnable(config->port_base, int_status);
}

static const struct gpio_driver_api gpio_tm4c123_driver = {
    .config = gpio_tm4c123_config,
    .write = gpio_tm4c123_write,
    .read = gpio_tm4c123_read,
    .manage_callback = gpio_tm4c123_manage_callback,
    .enable_callback = gpio_tm4c123_enable_callback,
    .disable_callback = gpio_tm4c123_disable_callback,
};

#define GPIO_DEVICE_INIT(__name, __suffix, __base_addr, __irq_num)          \
    static const struct gpio_tm4c123_config gpio_tm4c123_cfg_##__suffix = { \
        .port_base = __base_addr,                                           \
        .irq_num = __irq_num,                                               \
    };                                                                      \
                                                                            \
    static struct device DEVICE_NAME_GET(gpio_tm4c123_##__suffix);          \
    static struct gpio_tm4c123_data gpio_tm4c123_data_##__suffix;           \
                                                                            \
    static int gpio_tm4c123_init_##__suffix(struct device* dev)             \
    {                                                                       \
        ARG_UNUSED(dev);                                                    \
                                                                            \
        IRQ_CONNECT(INT_##__name - 16, CONFIG_TM4C123_##__name##_IRQ_PRI,   \
            gpio_tm4c123_port_isr, DEVICE_GET(gpio_tm4c123_##__suffix), 0); \
                                                                            \
        MAP_IntPendClear(INT_##__name);                                     \
        irq_enable(INT_##__name - 16);                                      \
                                                                            \
        return 0;                                                           \
    }                                                                       \
                                                                            \
    static struct gpio_tm4c123_data gpio_tm4c123_data_##__suffix;           \
    DEVICE_AND_API_INIT(gpio_tm4c123_##__suffix,                            \
        CONFIG_TM4C123_##__name##_NAME,                                     \
        &gpio_tm4c123_init_##__suffix,                                      \
        &gpio_tm4c123_data_##__suffix,                                      \
        &gpio_tm4c123_cfg_##__suffix,                                       \
        POST_KERNEL,                                                        \
        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                \
        &gpio_tm4c123_driver);

#define GPIO_DEVICE_INIT_TM4C123(__suffix, __SUFFIX) \
    GPIO_DEVICE_INIT(GPIO##__SUFFIX, __suffix,       \
        GPIO_PORT##__SUFFIX##_BASE, INT_GPIO##__SUFFIX)

#ifdef CONFIG_GPIO_TM4C123_PORTA
GPIO_DEVICE_INIT_TM4C123(a, A);
#endif /* CONFIG_GPIO_TM4C123_PORTA */

#ifdef CONFIG_GPIO_TM4C123_PORTB
GPIO_DEVICE_INIT_TM4C123(b, B);
#endif /* CONFIG_GPIO_TM4C123_PORTB */

#ifdef CONFIG_GPIO_TM4C123_PORTC
GPIO_DEVICE_INIT_TM4C123(c, C);
#endif /* CONFIG_GPIO_TM4C123_PORTC */

#ifdef CONFIG_GPIO_TM4C123_PORTD
GPIO_DEVICE_INIT_TM4C123(d, D);
#endif /* CONFIG_GPIO_TM4C123_PORTD */

#ifdef CONFIG_GPIO_TM4C123_PORTE
GPIO_DEVICE_INIT_TM4C123(e, E);
#endif /* CONFIG_GPIO_TM4C123_PORTE */

#ifdef CONFIG_GPIO_TM4C123_PORTF
GPIO_DEVICE_INIT_TM4C123(f, F);
#endif /* CONFIG_GPIO_TM4C123_PORTF */

#ifdef CONFIG_GPIO_TM4C123_PORTG
GPIO_DEVICE_INIT_TM4C123(g, G);
#endif /* CONFIG_GPIO_TM4C123_PORTG */

#ifdef CONFIG_GPIO_TM4C123_PORTH
GPIO_DEVICE_INIT_TM4C123(h, H);
#endif /* CONFIG_GPIO_TM4C123_PORTH */
