/* ieee802154_dw1000.h - Registers definition for dw1000 */

/*
 * Copyright (c) 2018 hackin zhao.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __IEEE802154_dw1000_H__
#define __IEEE802154_dw1000_H__

#include <atomic.h>
#include <linker/sections.h>
#include <spi.h>

/* Runtime context structure
 ***************************
 */

struct dw1000_context {

    struct net_if* iface;
    /**************************/
    struct dw1000_gpio_configuration* gpios;
    struct gpio_callback sfd_cb;
    struct gpio_callback fifop_cb;
    struct device* spi;
    struct spi_config spi_cfg;
    u8_t mac_addr[8];
    /************TX************/
    struct k_sem tx_sync;
    atomic_t tx;
    atomic_t rx;
    /************RX************/
    K_THREAD_STACK_MEMBER(dw1000_rx_stack,
        CONFIG_IEEE802154_DW1000_RX_STACK_SIZE);
    struct k_thread dw1000_rx_thread;
    struct k_sem rx_lock;
#ifdef CONFIG_IEEE802154_CC2520_CRYPTO
    struct k_sem access_lock;
#endif
    bool overflow;
};

#include "ieee802154_dw1000_regs.h"

/* Registers useful routines
 ***************************
 */

bool _dw1000_access_reg(struct spi_config* spi, bool read, u8_t addr,
    void* data, size_t length, bool extended, bool burst);

static inline u8_t _dw1000_read_single_reg(struct spi_config* spi,
    u8_t addr, bool extended)
{
    u8_t val;

    if (_dw1000_access_reg(spi, true, addr, &val, 1, extended, false)) {
        return val;
    }

    return 0;
}

static inline bool _dw1000_write_single_reg(struct spi_config* spi,
    u8_t addr, u8_t val, bool extended)
{
    return _dw1000_access_reg(spi, false, addr, &val, 1, extended, false);
}

static inline bool _dw1000_instruct(struct spi_config* spi, u8_t addr)
{
    return _dw1000_access_reg(spi, false, addr, NULL, 0, false, false);
}

#define DEFINE_REG_READ(__reg_name, __reg_addr, __ext)               \
    static inline u8_t read_reg_##__reg_name(struct spi_config* spi) \
    {                                                                \
        return _dw1000_read_single_reg(spi, __reg_addr, __ext);      \
    }

#define DEFINE_REG_WRITE(__reg_name, __reg_addr, __ext)               \
    static inline bool write_reg_##__reg_name(struct spi_config* spi, \
        u8_t val)                                                     \
    {                                                                 \
        return _dw1000_write_single_reg(spi, __reg_addr,              \
            val, __ext);                                              \
    }

/* Instructions useful routines
 ******************************
 */

#define DEFINE_STROBE_INSTRUCTION(__ins_name, __ins_addr)            \
    static inline bool instruct_##__ins_name(struct spi_config* spi) \
    {                                                                \
        /*SYS_LOG_DBG("");*/                                         \
        return _dw1000_instruct(spi, __ins_addr);                    \
    }

#endif /* __IEEE802154_dw1000_H__ */
