/* ieee802154_dw1000.c - DECAWAVE DW1000 device driver */

/*
 * Copyright (c) 2018 hackin zhao.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_IEEE802154_DRIVER_LEVEL
#define SYS_LOG_DOMAIN "dev/dw1000"
#include <logging/sys_log.h>
#include <misc/__assert.h>

#include <errno.h>

#include <arch/cpu.h>
#include <kernel.h>

#include <board.h>
#include <device.h>
#include <init.h>
#include <net/net_if.h>
#include <net/net_pkt.h>

#include <misc/byteorder.h>
#include <random/rand32.h>
#include <string.h>

#include <gpio.h>
#include <net/ieee802154_radio.h>

#include "ieee802154_dw1000.h"

#define _usleep(usec) k_busy_wait(usec)

#if defined(CONFIG_IEEE802154_DW1000_GPIO_SPI_CS)
static struct spi_cs_control cs_ctrl;
#endif

#if CONFIG_SYS_LOG_IEEE802154_DRIVER_LEVEL == 4

static void _dw1000_print_hex_buffer(char* str, u8_t* buffer, size_t length)
{
    SYS_LOG_BACKEND_FN("\t%s: length: %d\tbuffer: ", str, length);
    for (size_t i = 0; i < length; i++) {
        SYS_LOG_BACKEND_FN("%02x ", buffer[i]);
    }
    SYS_LOG_BACKEND_FN("\n");
}

#else
#define _dw1000_print_hex_buffer(...)
#endif /* CONFIG_SYS_LOG_IEEE802154_DRIVER_LEVEL == 4 */

bool _dw1000_access(struct dw1000_context* ctx, bool read, u8_t reg_num,
    u16_t index, void* data, size_t length)
{
    __ASSERT((reg_num <= 0x3F), "Invalue register id: %d", reg_num);
    __ASSERT((index <= 0x7fff), "Invalue offset: %d", index);
    __ASSERT(((index + length) <= 0x7fff), "Invalue offset: %d", index + length);

    uint8_t header[] = {
        [0] = (!read) << 7 | (0 != index) << 6 | reg_num,
        [1] = (index > 128) << 7 | (uint8_t)(index),
        [2] = (uint8_t)(index >> 7)
    };

    struct spi_buf buf[2] = {
        {
            .buf = header,
            .len = index ? (index > 128 ? 3 : 2) : 1,
        },
        {
            .buf = (u8_t*)data,
            .len = length,
        }
    };

    struct spi_buf_set tx = {
        .buffers = buf,
        .count = read ? 1 : 2
    };

    if (read) {
        const struct spi_buf_set rx = {
            .buffers = buf,
            .count = 2
        };

        return (spi_transceive(ctx->spi, &ctx->spi_cfg, &tx, &rx) == 0);
    }

    __ASSERT((data != NULL), "the send data buffer should not be NULL");

    return (spi_write(ctx->spi, &ctx->spi_cfg, &tx) == 0);
}

/***************** hardware interface functions ***************************/

static inline int configure_gpios(struct device* dev)
{
    struct dw1000_context* dw1000 = dev->driver_data;

    /* setup gpio for the modem interrupt */
    dw1000->irq_gpio = device_get_binding(CONFIG_IEEE802154_DW1000_GPIO_ISR_NAME);
    if (dw1000->irq_gpio == NULL) {
        SYS_LOG_ERR("Failed to get pointer to %s device",
            CONFIG_IEEE802154_DW1000_GPIO_ISR_NAME);
        return -EINVAL;
    }

    gpio_pin_configure(dw1000->irq_gpio,
        CONFIG_IEEE802154_DW1000_GPIO_ISR_PIN,
        GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE | GPIO_PUD_PULL_DOWN | GPIO_INT_ACTIVE_HIGH);

    /* setup gpio for the modems reset */
    dw1000->reset_gpio = device_get_binding(CONFIG_IEEE802154_DW1000_GPIO_RST_NAME);
    if (dw1000->reset_gpio == NULL) {
        SYS_LOG_ERR("Failed to get pointer to %s device",
            CONFIG_IEEE802154_DW1000_GPIO_RST_NAME);
        return -EINVAL;
    }

    gpio_pin_configure(dw1000->reset_gpio, CONFIG_IEEE802154_DW1000_GPIO_RST_PIN,
        GPIO_DIR_OUT);

    return 0;
}

static inline int configure_spi(struct device* dev, bool high_speed)
{
    struct dw1000_context* dw1000 = dev->driver_data;

    dw1000->spi = device_get_binding(
        CONFIG_IEEE802154_DW1000_SPI_DRV_NAME);
    if (!dw1000->spi) {
        SYS_LOG_ERR("Unable to get SPI device");
        return -ENODEV;
    }

#if defined(CONFIG_IEEE802154_DW1000_GPIO_SPI_CS)
    cs_ctrl.gpio_dev = device_get_binding(
        CONFIG_IEEE802154_DW1000_GPIO_SPI_CS_NAME);
    if (!cs_ctrl.gpio_dev) {
        SYS_LOG_ERR("Unable to get GPIO SPI CS device");
        return -ENODEV;
    }

    cs_ctrl.gpio_pin = CONFIG_IEEE802154_DW1000_GPIO_SPI_CS_PIN;
    cs_ctrl.delay = 0;

    dw1000->spi_cfg.cs = &cs_ctrl;

    SYS_LOG_DBG("SPI GPIO CS configured on %s:%u",
        CONFIG_IEEE802154_DW1000_GPIO_SPI_CS_NAME,
        CONFIG_IEEE802154_DW1000_GPIO_SPI_CS_PIN);
#endif /* CONFIG_IEEE802154_DW1000_GPIO_SPI_CS */

    if (high_speed)
        dw1000->spi_cfg.frequency = CONFIG_IEEE802154_DW1000_SPI_FREQ;
    else
        dw1000->spi_cfg.frequency = 2000000;

    dw1000->spi_cfg.operation = SPI_WORD_SET(8);
    dw1000->spi_cfg.slave = CONFIG_IEEE802154_DW1000_SPI_SLAVE;

    return 0;
}

static inline void isr_int_handler(struct device* port,
    struct gpio_callback* cb, u32_t pins)
{
    // struct dw1000_context* dw1000 = CONTAINER_OF(cb, struct dw1000_context, isr_cb);
    //TODO: isr, receive or transtive callback
}

static void enable_isr_interrupt(struct dw1000_context* dw1000,
    bool enable)
{
    if (enable) {
        gpio_pin_enable_callback(dw1000->irq_gpio,
            CONFIG_IEEE802154_DW1000_GPIO_ISR_PIN);
    } else {
        gpio_pin_disable_callback(dw1000->irq_gpio,
            CONFIG_IEEE802154_DW1000_GPIO_ISR_PIN);
    }
}

static inline void setup_gpio_callbacks(struct device* dev)
{
    struct dw1000_context* dw1000 = dev->driver_data;

    gpio_init_callback(&dw1000->isr_cb, isr_int_handler,
        BIT(CONFIG_IEEE802154_DW1000_GPIO_ISR_PIN));
    gpio_add_callback(dw1000->irq_gpio, &dw1000->isr_cb);
}

static inline void set_reset(struct dw1000_context* dw1000)
{
    gpio_pin_write(dw1000->reset_gpio,
        CONFIG_IEEE802154_DW1000_GPIO_RST_PIN, 0);

    _usleep(10);

    gpio_pin_configure(dw1000->reset_gpio, CONFIG_IEEE802154_DW1000_GPIO_RST_PIN,
        GPIO_DIR_IN);

    _usleep(2000);
}

/***  dw1000 setup, reset, wakeup, reconfigure, reload functions  ***/

static inline void set_sysclk_XTAL(struct dw1000_context* dw1000)
{
    // uint8_t reg = (uint8_t)dw1000_read_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, sizeof(uint8_t));
    // reg &= (uint8_t)~PMSC_CTRL0_SYSCLKS_19M & (uint8_t)~PMSC_CTRL0_SYSCLKS_125M;
    // reg |= (uint8_t)PMSC_CTRL0_SYSCLKS_19M;
    // dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, reg, sizeof(uint8_t));
}

static inline void set_soft_reset(struct dw1000_context* dw1000)
{
    // // Set system clock to XTI
    // dw1000_phy_sysclk_XTAL(dw1000);
    // dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL1_OFFSET, PMSC_CTRL1_PKTSEQ_DISABLE, sizeof(uint16_t));
    // dw1000_write_reg(inst, AON_ID, AON_WCFG_OFFSET, 0x0, sizeof(uint16_t));
    // dw1000_write_reg(inst, AON_ID, AON_CFG0_OFFSET, 0x0, sizeof(uint8_t));
    // // Uploads always-on (AON) data array and configuration
    // dw1000_write_reg(inst, AON_ID, AON_CTRL_OFFSET, 0x0, sizeof(uint8_t));
    // dw1000_write_reg(inst, AON_ID, AON_CTRL_OFFSET, AON_CTRL_SAVE, sizeof(uint8_t));
    // dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_ALL, sizeof(uint8_t));

    // _usleep(10);

    // dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_CLEAR, sizeof(uint8_t));
}

static int power_on_and_setup(struct device* dev)
{
    struct dw1000_context* dw1000 = dev->driver_data;

    set_reset(dw1000);

    if (read_register_device_id(dw1000) != (u32_t)0xDECA0130) {
        SYS_LOG_ERR("Read device id value not correct");
        return -EIO;
    }

    setup_gpio_callbacks(dev);

    return 0;
}

static int _dw1000_set_pan_id(struct device* dev, u16_t pan_id)
{
    struct dw1000_context* dw1000 = dev->driver_data;

    SYS_LOG_DBG("0x%x", pan_id);

    pan_id = sys_le16_to_cpu(pan_id);

    if (!write_reg_pan_id(dw1000, pan_id)) {
        SYS_LOG_ERR("Failed");
        return -EIO;
    }

    return 0;
}

static int _dw1000_set_short_addr(struct device* dev, u16_t short_addr)
{
    struct dw1000_context* dw1000 = dev->driver_data;

    SYS_LOG_DBG("0x%x", short_addr);

    short_addr = sys_le16_to_cpu(short_addr);

    if (!write_reg_short_address(dw1000, short_addr)) {
        SYS_LOG_ERR("Failed");
        return -EIO;
    }

    return 0;
}

static int _dw1000_set_ieee_addr(struct device* dev, u8_t* ieee_addr)
{
    struct dw1000_context* dw1000 = dev->driver_data;

    if (!write_reg_eui_id_64(dw1000, ieee_addr)) {
        SYS_LOG_ERR("Failed");
        return -EIO;
    }

    return 0;
}

static int dw1000_filter(struct device* dev,
    bool set,
    enum ieee802154_filter_type type,
    const struct ieee802154_filter* filter)
{
    SYS_LOG_DBG("Applying filter %u", type);

    if (!set) {
        return -ENOTSUP;
    }

    if (type == IEEE802154_FILTER_TYPE_IEEE_ADDR) {
        return _dw1000_set_ieee_addr(dev, filter->ieee_addr);
    } else if (type == IEEE802154_FILTER_TYPE_SHORT_ADDR) {
        return _dw1000_set_short_addr(dev, filter->short_addr);
    } else if (type == IEEE802154_FILTER_TYPE_PAN_ID) {
        return _dw1000_set_pan_id(dev, filter->pan_id);
    }

    return -ENOTSUP;
}

static enum ieee802154_hw_caps dw1000_get_capabilities(struct device* dev)
{
    return IEEE802154_HW_FCS | IEEE802154_HW_FILTER;
    return 0;
}

static int dw1000_cca(struct device* dev)
{
    return 0;
}

static int dw1000_set_channel(struct device* dev, u16_t channel)
{
    return 0;
}

static int dw1000_set_txpower(struct device* dev, s16_t dbm)
{
    return 0;
}

static void dw1000_rx(int arg)
{
}

static int dw1000_tx(struct device* dev,
    struct net_pkt* pkt,
    struct net_buf* frag)
{
    return 0;
}

static int dw1000_init(struct device* dev)
{
    struct dw1000_context* dw1000 = dev->driver_data;

    atomic_set(&dw1000->tx, 0);
    atomic_set(&dw1000->rx, 0);
    k_sem_init(&dw1000->rx_lock, 0, 1);
    k_sem_init(&dw1000->tx_sync, 0, 1);

    if (configure_gpios(dev)) {
        SYS_LOG_ERR("Configuring GPIOS failed");
        return -EIO;
    }

    if (configure_spi(dev, false) != 0) {
        SYS_LOG_ERR("Configuring SPI failed");
        return -EIO;
    }

    if (power_on_and_setup(dev) != 0) {
        SYS_LOG_ERR("Configuring DW1000 failed");
        return -EIO;
    }

    if (configure_spi(dev, true) != 0) {
        SYS_LOG_ERR("Configuring SPI failed");
        return -EIO;
    }

    k_thread_create(&dw1000->dw1000_rx_thread, dw1000->dw1000_rx_stack,
        CONFIG_IEEE802154_DW1000_RX_STACK_SIZE,
        (k_thread_entry_t)dw1000_rx,
        dev, NULL, NULL, K_PRIO_COOP(2), 0, 0);

    SYS_LOG_INF("DW1000 initialized");

    return 0;
}

/* dw1000 enter sleep mode */
static int dw1000_start(struct device* dev)
{
    struct dw1000_context* dw1000 = dev->driver_data;

    enable_isr_interrupt(dw1000, true);
    return 0;
}

/* wakeup dw1000 into idle mode */
static int dw1000_stop(struct device* dev)
{
    return 0;
}

static void dw1000_iface_init(struct net_if* iface)
{
    struct device* dev = net_if_get_device(iface);
    struct dw1000_context* dw1000 = dev->driver_data;
    static u8_t mac[8];

    read_register_eui_id_64(dw1000, mac);

    net_if_set_link_addr(iface, mac, 8, NET_LINK_IEEE802154);

    dw1000->iface = iface;

    ieee802154_init(iface);

    NET_INFO("ieee802154 iface initialized\n");
}

static struct dw1000_context dw1000_context_data;

static struct ieee802154_radio_api dw1000_radio_api = {
    .iface_api.init = dw1000_iface_init,
    .iface_api.send = ieee802154_radio_send,
    .get_capabilities = dw1000_get_capabilities,
    .cca = dw1000_cca,
    .set_channel = dw1000_set_channel,
    .set_txpower = dw1000_set_txpower,
    .filter = dw1000_filter,
    .tx = dw1000_tx,
    .start = dw1000_start,
    .stop = dw1000_stop,
};

NET_DEVICE_INIT(dw1000, CONFIG_IEEE802154_DW1000_DRV_NAME,
    dw1000_init, &dw1000_context_data, NULL,
    CONFIG_IEEE802154_DW1000_INIT_PRIO,
    &dw1000_radio_api, IEEE802154_L2,
    NET_L2_GET_CTX_TYPE(IEEE802154_L2), 125);
