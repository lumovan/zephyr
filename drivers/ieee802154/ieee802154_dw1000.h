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
    struct gpio_callback isr_cb;
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
bool _dw1000_access(struct dw1000_context* ctx, bool read, u8_t reg_num,
    u16_t index, void* data, size_t length);

static inline bool _dw1000_write_reg_8bit(struct dw1000_context* ctx,
    u8_t reg_num, u16_t index, u8_t val)
{
    return _dw1000_access(ctx, false, reg_num, index, &val, 1);
}

static inline bool _dw1000_write_reg_16bit(struct dw1000_context* ctx,
    u8_t reg_num, u16_t index, u16_t val)
{
    return _dw1000_access(ctx, false, reg_num, index, &val, 2);
}

static inline bool _dw1000_write_reg_32bit(struct dw1000_context* ctx,
    u8_t reg_num, u16_t index, u32_t val)
{
    return _dw1000_access(ctx, false, reg_num, index, &val, 4);
}

static inline bool _dw1000_write_reg_multi_byte(struct dw1000_context* ctx,
    u8_t reg_num, u16_t index, u8_t* data, size_t length)
{
    return _dw1000_access(ctx, false, reg_num, index, data, length);
}

static inline u8_t _dw1000_read_reg_8bit(struct dw1000_context* ctx,
    u8_t reg_num, u16_t index)
{
    u8_t val;

    if (_dw1000_access(ctx, true, reg_num, index, &val, 1)) {
        return val;
    }

    return 0;
}

static inline u16_t _dw1000_read_reg_16bit(struct dw1000_context* ctx,
    u8_t reg_num, u16_t index)
{
    u16_t val;

    if (_dw1000_access(ctx, true, reg_num, index, &val, 2)) {
        return val;
    }

    return 0;
}

static inline u32_t _dw1000_read_reg_32bit(struct dw1000_context* ctx,
    u8_t reg_num, u16_t index)
{
    u32_t val;

    if (_dw1000_access(ctx, true, reg_num, index, &val, 4)) {
        return val;
    }

    return 0;
}

static inline bool _dw1000_read_reg_multi_byte(struct dw1000_context* ctx,
    u8_t reg_num, u16_t index, u8_t* data, size_t length)
{
    return _dw1000_access(ctx, true, reg_num, index, data, length);
}

#define DEFINE_REG_READ_8(__reg_name, __reg_num, __offset)                    \
    static inline u8_t read_register_##__reg_name(struct dw1000_context* ctx) \
    {                                                                         \
        return _dw1000_read_reg_8bit(ctx, __reg_num, __offset);               \
    }

#define DEFINE_REG_READ_16(__reg_name, __reg_num, __offset)                    \
    static inline u16_t read_register_##__reg_name(struct dw1000_context* ctx) \
    {                                                                          \
        return _dw1000_read_reg_16bit(ctx, __reg_num, __offset);               \
    }

DEFINE_REG_READ_16(pan_id, DW1000_PANADR_ID, 2)
DEFINE_REG_READ_16(short_address, DW1000_PANADR_ID, 0)

#define DEFINE_REG_READ_32(__reg_name, __reg_num, __offset)                    \
    static inline u32_t read_register_##__reg_name(struct dw1000_context* ctx) \
    {                                                                          \
        return _dw1000_read_reg_32bit(ctx, __reg_num, __offset);               \
    }

DEFINE_REG_READ_32(device_id, DW1000_DEV_ID_ID, 0)

#define DEFINE_REG_WRITE_8(__reg_name, __reg_num, __offset)               \
    static inline bool write_reg_##__reg_name(struct dw1000_context* ctx, \
        u8_t val)                                                         \
    {                                                                     \
        return _dw1000_write_reg_8bit(ctx, __reg_num, __offset, val);     \
    }

#define DEFINE_REG_WRITE_16(__reg_name, __reg_num, __offset)              \
    static inline bool write_reg_##__reg_name(struct dw1000_context* ctx, \
        u16_t val)                                                        \
    {                                                                     \
        return _dw1000_write_reg_16bit(ctx, __reg_num, __offset, val);    \
    }

DEFINE_REG_WRITE_16(pan_id, DW1000_PANADR_ID, 2)
DEFINE_REG_WRITE_16(short_address, DW1000_PANADR_ID, 0)

#define DEFINE_REG_WRITE_32(__reg_name, __reg_num, __offset)              \
    static inline bool write_reg_##__reg_name(struct dw1000_context* ctx, \
        u32_t val)                                                        \
    {                                                                     \
        return _dw1000_write_reg_32bit(ctx, __reg_num, __offset, val);    \
    }

#define DEFINE_REG_WRITE_MULTI_BYTE(__reg_name, __reg_num, __offset, __length)         \
    static inline bool write_reg_##__reg_name(struct dw1000_context* ctx,              \
        u8_t* data)                                                                    \
    {                                                                                  \
        return _dw1000_write_reg_multi_byte(ctx, __reg_num, __offset, data, __length); \
    }

#endif /* __IEEE802154_dw1000_H__ */
