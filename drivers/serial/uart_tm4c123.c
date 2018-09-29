/*
 * Copyright (c) 2017, johnny@clspring.com
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <uart.h>

/* Driverlib includes */
#include <driverlib/gpio.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>
#include <hw_ints.h>

struct uart_tm4c123_dev_data_t {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    uart_irq_callback_t cb; /**< Callback function pointer */
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#define DEV_CFG(dev) \
    ((const struct uart_device_config* const)(dev)->config->config_info)
#define DEV_DATA(dev) \
    ((struct uart_tm4c123_dev_data_t* const)(dev)->driver_data)

static struct device DEVICE_NAME_GET(uart_tm4c123_0);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_tm4c123_isr(void* arg);
#endif

static const struct uart_device_config uart_tm4c123_dev_cfg_0 = {
    .base = (void*)CONFIG_UART_TM4C123_BASE_ADDRESS,
    .sys_clk_freq = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
};

static struct uart_tm4c123_dev_data_t uart_tm4c123_dev_data_0 = {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    .cb = NULL,
#endif
};

static int uart_tm4c123_init(struct device* dev)
{
    const struct uart_device_config* config = DEV_CFG(dev);

    if (!MAP_SysCtlPeripheralPresent(SYSCTL_PERIPH_UART0)) {
        return false;
    }

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    MAP_UARTConfigSetExpClk((unsigned long)config->base, uart_tm4c123_dev_cfg_0.sys_clk_freq, TI_TM4C123_UART_4000C000_CURRENT_SPEED,
        (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE | UART_CONFIG_WLEN_8));

    MAP_UARTFIFODisable((unsigned long)config->base);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

    MAP_UARTIntClear((unsigned long)config->base,
        (UART_INT_RX | UART_INT_TX));

    IRQ_CONNECT(TI_TM4C123_UART_4000C000_IRQ_0,
        TI_TM4C123_UART_4000C000_IRQ_0,
        uart_tm4c123_isr, DEVICE_GET(uart_tm4c123_0),
        0);
    irq_enable(TI_TM4C123_UART_4000C000_IRQ_0);

#endif

    MAP_UARTEnable((unsigned long)config->base);
    return 0;
}

static int uart_tm4c123_poll_in(struct device* dev, unsigned char* c)
{
    const struct uart_device_config* config = DEV_CFG(dev);

    *c = MAP_UARTCharGet((unsigned long)config->base);

    return 0;
}

static unsigned char uart_tm4c123_poll_out(struct device* dev,
    unsigned char c)
{
    const struct uart_device_config* config = DEV_CFG(dev);

    MAP_UARTCharPut((unsigned long)config->base, c);

    return c;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_tm4c123_fifo_fill(struct device* dev,
    const u8_t* tx_data, int size)
{
    const struct uart_device_config* config = DEV_CFG(dev);
    unsigned int num_tx = 0;

    while ((size - num_tx) > 0) {
        /* Send a character */
        if (MAP_UARTCharPutNonBlocking((unsigned long)config->base,
                tx_data[num_tx])) {
            num_tx++;
        } else {
            break;
        }
    }

    return (int)num_tx;
}

static int uart_tm4c123_fifo_read(struct device* dev, u8_t* rx_data,
    const int size)
{
    const struct uart_device_config* config = DEV_CFG(dev);
    unsigned int num_rx = 0;

    while (((size - num_rx) > 0) && MAP_UARTCharsAvail((unsigned long)config->base)) {

        /* Receive a character */
        rx_data[num_rx++] = MAP_UARTCharGetNonBlocking((unsigned long)config->base);
    }

    return num_rx;
}

static void uart_tm4c123_irq_tx_enable(struct device* dev)
{
    const struct uart_device_config* config = DEV_CFG(dev);

    MAP_UARTIntEnable((unsigned long)config->base, UART_INT_TX);
}

static void uart_tm4c123_irq_tx_disable(struct device* dev)
{
    const struct uart_device_config* config = DEV_CFG(dev);

    MAP_UARTIntDisable((unsigned long)config->base, UART_INT_TX);
}

static int uart_tm4c123_irq_tx_ready(struct device* dev)
{
    const struct uart_device_config* config = DEV_CFG(dev);
    unsigned int int_status;

    int_status = MAP_UARTIntStatus((unsigned long)config->base, UART_INT_TX);

    return (int_status & UART_INT_TX);
}

static void uart_tm4c123_irq_rx_enable(struct device* dev)
{
    const struct uart_device_config* config = DEV_CFG(dev);

    MAP_UARTIntEnable((unsigned long)config->base, UART_INT_RX);
}

static void uart_tm4c123_irq_rx_disable(struct device* dev)
{
    const struct uart_device_config* config = DEV_CFG(dev);

    MAP_UARTIntDisable((unsigned long)config->base, UART_INT_RX);
}

static int uart_tm4c123_irq_tx_complete(struct device* dev)
{
    const struct uart_device_config* config = DEV_CFG(dev);

    return (!MAP_UARTBusy((unsigned long)config->base));
}

static int uart_tm4c123_irq_rx_ready(struct device* dev)
{
    const struct uart_device_config* config = DEV_CFG(dev);
    unsigned int int_status;

    int_status = MAP_UARTIntStatus((unsigned long)config->base, 1);

    return (int_status & UART_INT_RX);
}

static void uart_tm4c123_irq_err_enable(struct device* dev)
{
    /* Not yet used in zephyr */
}

static void uart_tm4c123_irq_err_disable(struct device* dev)
{
    /* Not yet used in zephyr */
}

static int uart_tm4c123_irq_is_pending(struct device* dev)
{
    const struct uart_device_config* config = DEV_CFG(dev);
    unsigned int int_status;

    int_status = MAP_UARTIntStatus((unsigned long)config->base, 1);

    return (int_status & (UART_INT_TX | UART_INT_RX));
}

static int uart_tm4c123_irq_update(struct device* dev)
{
    return 1;
}

static void uart_tm4c123_irq_callback_set(struct device* dev,
    uart_irq_callback_t cb)
{
    struct uart_tm4c123_dev_data_t* const dev_data = DEV_DATA(dev);

    dev_data->cb = cb;
}

/**
 * @brief Interrupt service routine.
 *
 * This simply calls the callback function, if one exists.
 *
 * @param arg Argument to ISR.
 *
 * @return N/A
 */
static void uart_tm4c123_isr(void* arg)
{
    struct device* dev = arg;
    const struct uart_device_config* config = DEV_CFG(dev);
    struct uart_tm4c123_dev_data_t* const dev_data = DEV_DATA(dev);
    unsigned int int_status;

    int_status = MAP_UARTIntStatus((unsigned long)config->base, UART_INT_RX | UART_INT_RT);

    if (dev_data->cb) {
        dev_data->cb(dev);
    }

    /*
    * Clear interrupts only after cb called, as Zephyr UART clients expect
    * to check interrupt status during the callback.
    */
    MAP_UARTIntClear((unsigned long)config->base, int_status);
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_tm4c123_driver_api = {
    .poll_in = uart_tm4c123_poll_in,
    .poll_out = uart_tm4c123_poll_out,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    .fifo_fill = uart_tm4c123_fifo_fill,
    .fifo_read = uart_tm4c123_fifo_read,
    .irq_tx_enable = uart_tm4c123_irq_tx_enable,
    .irq_tx_disable = uart_tm4c123_irq_tx_disable,
    .irq_tx_ready = uart_tm4c123_irq_tx_ready,
    .irq_rx_enable = uart_tm4c123_irq_rx_enable,
    .irq_rx_disable = uart_tm4c123_irq_rx_disable,
    .irq_tx_complete = uart_tm4c123_irq_tx_complete,
    .irq_rx_ready = uart_tm4c123_irq_rx_ready,
    .irq_err_enable = uart_tm4c123_irq_err_enable,
    .irq_err_disable = uart_tm4c123_irq_err_disable,
    .irq_is_pending = uart_tm4c123_irq_is_pending,
    .irq_update = uart_tm4c123_irq_update,
    .irq_callback_set = uart_tm4c123_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

DEVICE_AND_API_INIT(uart_tm4c123_0, CONFIG_UART_TM4C123_NAME,
    uart_tm4c123_init, &uart_tm4c123_dev_data_0,
    &uart_tm4c123_dev_cfg_0,
    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
    (void*)&uart_tm4c123_driver_api);
