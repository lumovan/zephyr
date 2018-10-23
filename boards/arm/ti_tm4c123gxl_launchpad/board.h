/*
 * Copyright (c) 2017, hackin, zhao.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __INC_BOARD_H
#define __INC_BOARD_H

/* Push button switch 1 */
#define SW1_GPIO_NAME "GPIOF"
#define SW1__GPIO_PIN 4

/* Push button switch 2 */
#define SW2_GPIO_NAME "GPIOF"
#define SW2_GPIO_PIN 0

/* Onboard GREEN LED */
#define LEDG_GPIO_PORT "GPIOF"
#define LEDG_GPIO_PIN 3

/* Onboard RED LED */
#define LEDR_GPIO_PORT "GPIOF"
#define LEDR_GPIO_PIN 1

/* Onboard BLUE LED */
#define LEDB_GPIO_PORT "GPIOF"
#define LEDB_GPIO_PIN 2

#if defined(CONFIG_IEEE802154_DW1000)
/* GPIO numbers where the DW1000 chip is connected to */
#define DW1000_GPIO_ISR_PIN 3
#define DW1000_GPIO_WAKEUP_PIN 5
#define DW1000_GPIO_RST_PIN 3
#define DW1000_GPIO_EXTON_PIN 4
#define DW1000_GPIO_GPIO_5_PIN 2
#define DW1000_GPIO_GPIO_6_PIN 0
#endif /* CONFIG_IEEE802154_DW1000 */

#endif /* __INC_BOARD_H */
