/* This file is a temporary workaround for mapping of the generated information
 * to the current driver definitions.  This will be removed when the drivers
 * are modified to handle the generated information, or the mapping of
 * generated data matches the driver definitions.
 */

#define DT_NUM_IRQ_PRIO_BITS	DT_ARM_V7M_NVIC_E000E100_ARM_NUM_IRQ_PRIORITY_BITS

#define CONFIG_UART_TM4C123_NAME            DT_TI_TM4C123_UART_4000C000_LABEL
#define CONFIG_UART_TM4C123_BASE_ADDRESS    DT_TI_TM4C123_UART_4000C000_BASE_ADDRESS
#define CONFIG_UART_TM4C123_BAUD_RATE       DT_TI_TM4C123_UART_4000C000_CURRENT_SPEED

#define CONFIG_SPI_0_NAME                    DT_TI_TM4C123_SPI_40008000_LABEL
#define CONFIG_SPI_0_BASE_ADDRESS            DT_TI_TM4C123_SPI_40008000_BASE_ADDRESS
#define CONFIG_SPI_0_IRQ                     DT_TI_TM4C123_SPI_40008000_IRQ_0
#define CONFIG_SPI_0_IRQ_PRI                 DT_TI_TM4C123_SPI_40008000_IRQ_0_PRIORITY

#define CONFIG_IEEE802154_DW1000_SPI_SLAVE     DT_TI_TM4C123_SPI_40008000_DECAWAVE_DW1000_0_BASE_ADDRESS
#define CONFIG_IEEE802154_DW1000_SPI_DRV_NAME  DT_TI_TM4C123_SPI_40008000_DECAWAVE_DW1000_0_BUS_NAME
#define CONFIG_IEEE802154_DW1000_SPI_FREQ      DT_TI_TM4C123_SPI_40008000_DECAWAVE_DW1000_0_SPI_MAX_FREQUENCY
