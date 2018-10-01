/*
 * Copyright (c) 2018 hackin, zhao
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_SPI_LEVEL
#include <logging/sys_log.h>

#include <board.h>
#include <errno.h>
#include <kernel.h>
#include <misc/util.h>
#include <spi.h>
#include <stdbool.h>
#include <stdint.h>
#include <toolchain.h>

/* Driverlib includes */
#include <driverlib/gpio.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/ssi.h>
#include <driverlib/sysctl.h>
#include <hw_ints.h>
#include <inc/hw_memmap.h>

#include "spi_tm4c123.h"

#define DEV_CFG(dev) \
    ((const struct spi_tm4c123_config* const)(dev->config->config_info))

#define DEV_DATA(dev) \
    ((struct spi_tm4c123_data* const)(dev->driver_data))

static bool spi_tm4c123_transfer_ongoing(struct spi_tm4c123_data* data)
{
    return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

static inline u16_t spi_tm4c123_next_tx(struct spi_tm4c123_data* data)
{
    u16_t tx_frame = 0;

    if (spi_context_tx_buf_on(&data->ctx)) {
        if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
            tx_frame = UNALIGNED_GET((u8_t*)(data->ctx.tx_buf));
        } else if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 16) {
            tx_frame = UNALIGNED_GET((u16_t*)(data->ctx.tx_buf));
        }
    }

    return tx_frame;
}

static int spi_tm4c123_release(struct device* dev,
    const struct spi_config* config)
{
    struct spi_tm4c123_data* data = DEV_DATA(dev);

    spi_context_unlock_unconditionally(&data->ctx);

    return 0;
}

/* Shift a SPI frame as master. */
static void spi_tm4c123_shift_m(struct device* dev, struct spi_tm4c123_data* data)
{
    u32_t tx_frame;
    u32_t rx_frame;
    const struct spi_tm4c123_config* cfg = DEV_CFG(dev);

    tx_frame = (u32_t)spi_tm4c123_next_tx(data);
    while (MAP_SSIDataGetNonBlocking(cfg->spi_base, &rx_frame)) {
        /* NOP */
    }

    MAP_SSIDataPut(cfg->spi_base, tx_frame);

    if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
        /* The update is ignored if TX is off. */
        spi_context_update_tx(&data->ctx, 1, 1);
    } else {
        /* The update is ignored if TX is off. */
        spi_context_update_tx(&data->ctx, 2, 1);
    }

    while (MAP_SSIBusy(cfg->spi_base)) {
        /* NOP */
    }

    if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
        MAP_SSIDataGet(cfg->spi_base, &rx_frame);
        if (spi_context_rx_buf_on(&data->ctx)) {
            UNALIGNED_PUT((u16_t)rx_frame, (u8_t*)data->ctx.rx_buf);
        }
        spi_context_update_rx(&data->ctx, 1, 1);
    } else {
        MAP_SSIDataGet(cfg->spi_base, &rx_frame);
        if (spi_context_rx_buf_on(&data->ctx)) {
            UNALIGNED_PUT((u16_t)rx_frame, (u16_t*)data->ctx.rx_buf);
        }
        spi_context_update_rx(&data->ctx, 2, 1);
    }
}

/* Shift a SPI frame as slave. */
static void spi_tm4c123_shift_s(struct device* dev, struct spi_tm4c123_data* data)
{
    u32_t tx_frame;
    u32_t rx_frame;
    const struct spi_tm4c123_config* cfg = DEV_CFG(dev);

    tx_frame = (u32_t)spi_tm4c123_next_tx(data);

    MAP_SSIDataPut(cfg->spi_base, tx_frame);
    if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
        /* The update is ignored if TX is off. */
        spi_context_update_tx(&data->ctx, 1, 1);
    } else {
        /* The update is ignored if TX is off. */
        spi_context_update_tx(&data->ctx, 2, 1);
    }

    if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
        MAP_SSIDataGet(cfg->spi_base, &rx_frame);
        if (spi_context_rx_buf_on(&data->ctx)) {
            UNALIGNED_PUT((u16_t)rx_frame,
                (u8_t*)data->ctx.rx_buf);
        }
        spi_context_update_rx(&data->ctx, 1, 1);
    } else {
        MAP_SSIDataGet(cfg->spi_base, &rx_frame);
        if (spi_context_rx_buf_on(&data->ctx)) {
            UNALIGNED_PUT((u16_t)rx_frame,
                (u16_t*)data->ctx.rx_buf);
        }
        spi_context_update_rx(&data->ctx, 2, 1);
    }
}

static int spi_tm4c123_shift_frames(struct device* dev, struct spi_tm4c123_data* data)
{
    u16_t operation = data->ctx.config->operation;

    if (SPI_OP_MODE_GET(operation) == SPI_OP_MODE_MASTER) {
        spi_tm4c123_shift_m(dev, data);
        return 0;
    } else {
        spi_tm4c123_shift_s(dev, data);
        return 0;
    }

    return -1;
}

static int spi_tm4c123_configure(struct device* dev, const struct spi_config* config)
{
    const struct spi_tm4c123_config* cfg = DEV_CFG(dev);
    struct spi_tm4c123_data* data = DEV_DATA(dev);
    u32_t protocol, mode, bit_rate;

    if (spi_context_configured(&data->ctx, config)) {
        /* Nothing to do */
        return 0;
    }

    if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) {
        if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) {
            protocol = SSI_FRF_MOTO_MODE_3;
        } else {
            protocol = SSI_FRF_MOTO_MODE_1;
        }
    } else {
        if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) {
            protocol = SSI_FRF_MOTO_MODE_2;
        } else {
            protocol = SSI_FRF_MOTO_MODE_0;
        }
    }

    bit_rate = config->frequency;
    if ((cfg->spi_clk / bit_rate) > (254 * 256))
        return -ENOTSUP;

    if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
        mode = SSI_MODE_MASTER;
        if (bit_rate > (cfg->spi_clk / 2))
            return -ENOTSUP;
    } else {
        mode = SSI_MODE_SLAVE;
        if (bit_rate > (cfg->spi_clk / 12))
            return -ENOTSUP;
    }

    if ((SPI_WORD_SIZE_GET(config->operation) < 8) || (SPI_WORD_SIZE_GET(config->operation) > 16)) {
        return -ENOTSUP;
    }

    /* At this point, it's mandatory to set this on the context! */
    data->ctx.config = config;

    spi_context_cs_configure(&data->ctx);

    MAP_SSIConfigSetExpClk(cfg->spi_base, cfg->spi_clk,
        protocol, mode, bit_rate, (SPI_WORD_SIZE_GET(config->operation)));

    return 0;
}

static void spi_tm4c123_complete(struct device* dev, struct spi_tm4c123_data* data,
    int status)
{
    const struct spi_tm4c123_config* cfg = DEV_CFG(dev);

#ifdef CONFIG_SPI_TM4C123_INTERRUPT
    MAP_SSIIntDisable(cfg->spi_base, SSI_TXFF | SSI_RXFF);
#endif

    MAP_SSIDisable(cfg->spi_base);
    spi_context_cs_control(&data->ctx, false);

#ifdef CONFIG_SPI_TM4C123_INTERRUPT
    spi_context_complete(&data->ctx, status);
#endif
}

static int transceive(struct device* dev,
    const struct spi_config* config,
    const struct spi_buf_set* tx_bufs,
    const struct spi_buf_set* rx_bufs,
    bool asynchronous, struct k_poll_signal* signal)
{
    struct spi_tm4c123_data* data = DEV_DATA(dev);
    const struct spi_tm4c123_config* cfg = DEV_CFG(dev);
    int ret;

    if (!tx_bufs && !tx_bufs) {
        return 0;
    }

#ifndef CONFIG_SPI_TM4C123_INTERRUPT
    if (asynchronous) {
        return -ENOTSUP;
    }
#endif

    spi_context_lock(&data->ctx, asynchronous, signal);

    ret = spi_tm4c123_configure(dev, config);
    if (ret) {
        goto out;
    }

    MAP_SSIEnable(cfg->spi_base);

    /* Set buffers info */
    spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

    /* This is turned off in spi_tm4c123_complete(). */
    spi_context_cs_control(&data->ctx, true);

#ifdef CONFIG_SPI_TM4C123_INTERRUPT

    MAP_SSIIntClear((unsigned long)cfg->spi_base, SSI_TXFF | SSI_RXFF);
    MAP_SSIIntEnable(cfg->spi_base, SSI_TXFF | SSI_RXFF);
    ret = spi_context_wait_for_completion(&data->ctx);
#else
    do {
        ret = spi_tm4c123_shift_frames(dev, data);
    } while (!ret && spi_tm4c123_transfer_ongoing(data));

    spi_tm4c123_complete(dev, data, ret);
#endif

out:
    spi_context_release(&data->ctx, ret);

    return ret ? -EIO : 0;
}

static int spi_tm4c123_transceive(struct device* dev,
    const struct spi_config* config,
    const struct spi_buf_set* tx_bufs,
    const struct spi_buf_set* rx_bufs)
{
    return transceive(dev, config, tx_bufs, rx_bufs, false, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_tm4c123_transceive_async(struct device* dev,
    const struct spi_config* config,
    const struct spi_buf_set* tx_bufs,
    const struct spi_buf_set* rx_bufs,
    struct k_poll_signal* async)
{
    return transceive(dev, config, tx_bufs, rx_bufs, true, async);
}
#endif /* CONFIG_SPI_ASYNC */

static const struct spi_driver_api api_funcs = {
    .transceive = spi_tm4c123_transceive,
#ifdef CONFIG_SPI_ASYNC
    .transceive_async = spi_tm4c123_transceive_async,
#endif
    .release = spi_tm4c123_release,
};

static int spi_tm4c123_init(struct device* dev)
{
    const struct spi_tm4c123_config* cfg = DEV_CFG(dev);
    struct spi_tm4c123_data* data = DEV_DATA(dev);

    if (!MAP_SysCtlPeripheralPresent(cfg->peripheral)) {
        return -1;
    }

    MAP_SysCtlPeripheralEnable(cfg->peripheral);

    spi_context_unlock_unconditionally(&data->ctx);

#ifdef CONFIG_SPI_TM4C123_INTERRUPT
    cfg->irq_config(dev);
#endif

    return 0;
}

#ifdef CONFIG_SPI_TM4C123_INTERRUPT
static void spi_tm4c123_isr(void* arg)
{
    struct device* const dev = (struct device*)arg;
    const struct spi_tm4c123_config* cfg = DEV_CFG(dev);
    struct spi_tm4c123_data* data = DEV_DATA(dev);
    int err;
    unsigned int int_status;

    int_status = MAP_SSIIntStatus((unsigned long)cfg->spi_base, true);
    MAP_SSIIntClear((unsigned long)cfg->spi_base, int_status);

    if (spi_tm4c123_transfer_ongoing(data)) {
        err = spi_tm4c123_shift_frames(dev, data);
    }

    if (err || !spi_tm4c123_transfer_ongoing(data)) {
        spi_tm4c123_complete(dev, data, err);
    }
}
#endif

#ifdef CONFIG_SPI_0

#ifdef CONFIG_SPI_TM4C123_INTERRUPT
static void spi_tm4c123_irq_config_func_0(struct device* dev);
#endif

static const struct spi_tm4c123_config spi_tm4c123_cfg_0 = {
    .spi_clk = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
    .spi_base = CONFIG_SPI_0_TM4C123_BASE_ADDRESS,
    .peripheral = SYSCTL_PERIPH_SSI0,
#ifdef CONFIG_SPI_TM4C123_INTERRUPT
    .irq_config = spi_tm4c123_irq_config_func_0,
#endif
};

static struct spi_tm4c123_data spi_tm4c123_dev_data_0 = {
    SPI_CONTEXT_INIT_LOCK(spi_tm4c123_dev_data_0, ctx),
    SPI_CONTEXT_INIT_SYNC(spi_tm4c123_dev_data_0, ctx),
};

DEVICE_AND_API_INIT(spi_tm4c123_0, CONFIG_SPI_0_TM4C123_NAME, &spi_tm4c123_init,
    &spi_tm4c123_dev_data_0, &spi_tm4c123_cfg_0,
    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,
    &api_funcs);

#ifdef CONFIG_SPI_TM4C123_INTERRUPT
static void spi_tm4c123_irq_config_func_0(struct device* dev)
{
    IRQ_CONNECT(CONFIG_SPI_0_TM4C123_IRQ, CONFIG_SPI_0_TM4C123_IRQ_PRI,
        spi_tm4c123_isr, DEVICE_GET(spi_tm4c123_0), 0);
    irq_enable(CONFIG_SPI_0_TM4C123_IRQ);
}
#endif

#endif /* CONFIG_SPI_0 */
