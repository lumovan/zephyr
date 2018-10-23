#define SYS_LOG_LEVEL CONFIG_SYS_LOG_IEEE802154_DRIVER_LEVEL
#define SYS_LOG_DOMAIN "dev/dw1000"
#include <logging/sys_log.h>

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
#include <ieee802154/dw1000.h>
#include <net/ieee802154_radio.h>

#include "ieee802154_dw1000.h"

#define _usleep(usec) k_busy_wait(usec)

bool _dw1000_access(struct dw1000_context* ctx, bool read, u8_t reg_num,
    u16_t index, void* data, size_t length)
{
    u8_t header[3];
    u8_t cmd_buf[2];
    int cnt = 0;
    struct spi_buf_set tx = {
        .buffers = buf,
    };

    if (reg_num > 0x3F) {
        return -ENOTSUP;
    }

    if (index == 0) {
        header[cnt++] = 0x80 | reg_num;
    } else {
        if (index > 0x7fff) {
            return -ENOTSUP;
        }
        if ((index + length) > 0x7fff) {
            return -ENOTSUP;
        }

        if (read) {
            header[cnt++] = 0x40 | reg_num;
        }else{
            header[cnt++] = 0xc0 | reg_num;
        }

        if (index < 127) {
            header[cnt++] = 0xc0 | reg_num;
        } else {

            header[cnt++] = 0x80 | (u8_t)index;

            if (read) {
                header[cnt++] = 0xc0 | (u8_t)(index >> 7);
            }else{
                header[cnt++] = (u8_t)(index >> 7);
            }
        }
    }

    tx.count = cnt;

    if (read) {
        const struct spi_buf_set rx = {
            .buffers = buf,
            .count = cnt
        };

        return (spi_transceive(ctx->spi, &ctx->spi_cfg, &tx, &rx) == 0);
    }

    return (spi_write(ctx->spi, &ctx->spi_cfg, &tx) == 0);
}

static inline int configure_spi(struct device* dev)
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
        CONFIG_IEEE802154_DW1000_GPIO_SPI_CS_DRV_NAME);
    if (!cs_ctrl.gpio_dev) {
        SYS_LOG_ERR("Unable to get GPIO SPI CS device");
        return -ENODEV;
    }

    cs_ctrl.gpio_pin = CONFIG_IEEE802154_DW1000_GPIO_SPI_CS_PIN;
    cs_ctrl.delay = 0;

    dw1000->spi_cfg.cs = &cs_ctrl;

    SYS_LOG_DBG("SPI GPIO CS configured on %s:%u",
        CONFIG_IEEE802154_DW1000_GPIO_SPI_CS_DRV_NAME,
        CONFIG_IEEE802154_DW1000_GPIO_SPI_CS_PIN);
#endif /* CONFIG_IEEE802154_DW1000_GPIO_SPI_CS */

    dw1000->spi_cfg.frequency = CONFIG_IEEE802154_DW1000_SPI_FREQ;
    dw1000->spi_cfg.operation = SPI_WORD_SET(8);
    dw1000->spi_cfg.slave = CONFIG_IEEE802154_DW1000_SPI_SLAVE;

    return 0;
}

static inline void set_reset(struct device* dev)
{
    struct dw1000_context* dw1000 = dev->driver_data;
    struct device* gpio;

    gpio_pin_write(dw1000->gpios[DW1000_GPIO_IDX_RST].dev,
        dw1000->gpios[DW1000_GPIO_IDX_RST].pin, 0);

    _usleep(2000);

    gpio = device_get_binding(CONFIG_IEEE802154_DW1000_GPIO_RST_DRV_NAME);
    gpio_pin_configure(gpio, dw1000->gpios[DW1000_GPIO_IDX_RST].pin,
        GPIO_DIR_IN);
}

static inline void set_exton(struct device* dev, u32_t value)
{
    struct dw1000_context* dw1000 = dev->driver_data;

    gpio_pin_write(dw1000->gpios[DW1000_GPIO_IDX_RST].dev,
        dw1000->gpios[DW1000_GPIO_IDX_RST].pin, value);
}

static inline void set_wakeup(struct device* dev, u32_t value)
{
    struct dw1000_context* dw1000 = dev->driver_data;

    gpio_pin_write(dw1000->gpios[DW1000_GPIO_IDX_RST].dev,
        dw1000->gpios[DW1000_GPIO_IDX_RST].pin, value);
}

static int power_on_and_setup(struct device* dev)
{
    return 0;
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

    dw1000->gpios = dw1000_configure_gpios();
    if (!dw1000->gpios) {
        SYS_LOG_ERR("Configuring GPIOS failed");
        return -EIO;
    }

    if (configure_spi(dev) != 0) {
        SYS_LOG_ERR("Configuring SPI failed");
        return -EIO;
    }

    SYS_LOG_DBG("GPIO and SPI configured");

    if (power_on_and_setup(dev) != 0) {
        SYS_LOG_ERR("Configuring DW1000 failed");
        return -EIO;
    }

    k_thread_create(&dw1000->dw1000_rx_thread, dw1000->dw1000_rx_stack,
        CONFIG_IEEE802154_DW1000_RX_STACK_SIZE,
        (k_thread_entry_t)dw1000_rx,
        dev, NULL, NULL, K_PRIO_COOP(2), 0, 0);

    SYS_LOG_INF("DW1000 initialized");

    return 0;
}

static int dw1000_start(struct device* dev)
{
    return 0;
}

static int dw1000_stop(struct device* dev)
{
    return 0;
}

static void dw1000_iface_init(struct net_if* iface)
{
    struct ieee802154_context* ctx = net_if_l2_data(iface);
    static u8_t mac[8] = { 0x00, 0x12, 0x4b, 0x00,
        0x00, 0x9e, 0xa3, 0xc2 };

    net_if_set_link_addr(iface, mac, 8, NET_LINK_IEEE802154);

    ctx->pan_id = 0xabcd;
    ctx->channel = 26;
    ctx->sequence = 62;

    NET_INFO("FAKE ieee802154 iface initialized\n");
}

static struct dw1000_context dw1000_context_data;

static struct ieee802154_radio_api dw1000_radio_api = {
    .iface_api.init = dw1000_iface_init,
    .iface_api.send = ieee802154_radio_send,
    .get_capabilities = dw1000_get_capabilities,
    .cca = dw1000_cca,
    .set_channel = dw1000_set_channel,
    .set_txpower = dw1000_set_txpower,
    .tx = dw1000_tx,
    .start = dw1000_start,
    .stop = dw1000_stop,
};

NET_DEVICE_INIT(dw1000, CONFIG_IEEE802154_DW1000_DRV_NAME,
    dw1000_init, &dw1000_context_data, NULL,
    CONFIG_IEEE802154_DW1000_INIT_PRIO,
    &dw1000_radio_api, IEEE802154_L2,
    NET_L2_GET_CTX_TYPE(IEEE802154_L2), 125);
