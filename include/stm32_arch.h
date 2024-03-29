//Copyright (c) 2011-2022 Tyler Gilbert and Stratify Labs, Inc. See LICENSE.md for details


#ifndef STM32_ARCH_H_
#define STM32_ARCH_H_

#if (defined __stm32f446xx) || (defined __stm32f401xc) || (defined __stm32f405xx)  \
 || (defined __stm32f411xe) || (defined __stm32f413xx)                        \
 || (defined __stm32f417xx) || (defined __stm32f412zx)                        \
 || (defined __stm32f429xx) || (defined __stm32f746xx)                        \
 || (defined __stm32f722xx) || (defined __stm32f723xx)                        \
 || (defined __stm32f767xx) || (defined __stm32l475xx)                        \
 || (defined __stm32l432xx) || (defined __stm32f401xe)                        \
 || (defined __stm32h743xx) || (defined __stm32h750xx)                        \
 || (defined __stm32h735xx) || (defined __stm32f205xx)                        \
 || (defined __stm32f207xx) || (defined __stm32f215xx)                        \
 || (defined __stm32f217xx)
#undef __FPU_USED

#include <mcu/adc.h>
#include <mcu/dac.h>
#include <mcu/eth.h>
#include <mcu/i2s.h>
#include <mcu/mmc.h>
#include <mcu/qspi.h>
#include <mcu/sdio.h>
#include <mcu/spi.h>
#include <mcu/uart.h>
#include <sdk/types.h>

#if defined __stm32f205xx
#define CORE_M4 1
#define ARM_MATH_CM4 1
#define STM32_FLASH_LAYOUT_16_16_16_16_64_128_128_128 1
#if !defined STM32F205xx
#define STM32F205xx 1
#endif
#include "mcu_stm32f205xx.h"
#endif

#if defined __stm32f401xc
#define CORE_M4 1
#define ARM_MATH_CM4 1
#define STM32_FLASH_LAYOUT_16_16_16_16_64_128_128_128 1
#if !defined STM32F401xC
#define STM32F401xC 1
#endif
#include "mcu_stm32f401xc.h"
#endif

#if defined __stm32f401xe
#define CORE_M4 1
#define ARM_MATH_CM4 1
#define STM32_FLASH_LAYOUT_16_16_16_16_64_128_128_128 1
#if !defined STM32F401xE
#define STM32F401xE 1
#endif
#include "mcu_stm32f401xe.h"
#endif

#if defined __stm32f405xx
#define CORE_M4 1
#define ARM_MATH_CM4 1
#define STM32_FLASH_LAYOUT_16_16_16_16_64_128_128_128 1
#if !defined STM32F405xx
#define STM32F405xx 1
#endif
#include "mcu_stm32f405xx.h"
#endif

#if defined __stm32f411xe
#define CORE_M4 1
#define ARM_MATH_CM4 1
#define STM32_FLASH_LAYOUT_16_16_16_16_64_128_128_128 1
#if !defined STM32F411xE
#define STM32F411xE 1
#endif
#include "mcu_stm32f411xe.h"
#endif

#if defined __stm32f412zx
#define CORE_M4 1
#define ARM_MATH_CM4 1
#define STM32_FLASH_LAYOUT_16_16_16_16_64_128_128_128 1
#if !defined STM32F412Zx
#define STM32F412Zx 1
#endif
#include "mcu_stm32f412zx.h"
#endif

#if defined __stm32f413xx
#define CORE_M4 1
#define ARM_MATH_CM4 1
#define STM32_FLASH_LAYOUT_16_16_16_16_64_128x11 1
#if !defined STM32F413xx
#define STM32F413xx 1
#endif
#include "mcu_stm32f413xx.h"
#endif

#if defined __stm32f417xx
#define CORE_M4 1
#define ARM_MATH_CM4 1
#define STM32_FLASH_LAYOUT_16_16_16_16_64_128_128_128 1
#if !defined STM32F417xx
#define STM32F417xx 1
#endif
#include "mcu_stm32f417xx.h"
#endif

#if defined __stm32f429xx
#define CORE_M4 1
#define ARM_MATH_CM4 1
#define STM32_FLASH_LAYOUT_16_16_16_16_64_128_128_128 1
#if !defined STM32F429xx
#define STM32F429xx 1
#endif
#include "mcu_stm32f429xx.h"
#endif

#if defined __stm32f446xx
#define CORE_M4 1
#define ARM_MATH_CM4 1
#define STM32_FLASH_LAYOUT_16_16_16_16_64_128_128_128 1
#if !defined STM32F446xx
#define STM32F446xx 1
#endif
#include "mcu_stm32f446xx.h"
#endif

#if defined __stm32f722xx
#define CORE_M7 1
#define ARM_MATH_CM7 1
#define STM32_FLASH_LAYOUT_16_16_16_16_64_128_128_128 1
#if !defined STM32F722xx
#define STM32F722xx 1
#endif
#include "mcu_stm32f722xx.h"
#endif

#if defined __stm32f723xx
#define CORE_M7 1
#define ARM_MATH_CM7 1
#define STM32_FLASH_LAYOUT_16_16_16_16_64_128_128_128 1
#if !defined STM32F723xx
#define STM32F723xx 1
#endif
#include "mcu_stm32f723xx.h"
#endif

#if defined __stm32f746xx
#define CORE_M7 1
#define ARM_MATH_CM7 1
#define STM32_FLASH_LAYOUT_32_32_32_32_128_256_256_256 1
#if !defined STM32F746xx
#define STM32F746xx 1
#endif
#include "mcu_stm32f746xx.h"
#endif

#if defined __stm32f767xx
#define CORE_M7 1
#define ARM_MATH_CM7 1
#define STM32_FLASH_LAYOUT_32_32_32_32_128_256_256_256 1
#if !defined STM32F767xx
#define STM32F767xx 1
#endif
#include "mcu_stm32f767xx.h"
#endif

#if defined __stm32l475xx
#define CORE_M4 1
#define ARM_MATH_CM4 1
#define STM32_FLASH_LAYOUT_2K_PAGES 1
#if !defined STM32L475xx
#define STM32L475xx 1
#endif
#include "mcu_stm32l475xx.h"
#endif

#if defined __stm32l432xx
#define CORE_M4 1
#define ARM_MATH_CM4 1
#define STM32_FLASH_LAYOUT_2K_PAGES 1
#if !defined STM32L432xx
#define STM32L432xx 1
#endif
#include "mcu_stm32l432xx.h"
#endif

#if defined __stm32h743xx
#define CORE_M7 1
#define ARM_MATH_CM7 1
#define STM32_FLASH_LAYOUT_128K_PAGES 1
#if !defined STM32H743xx
#define STM32H743xx 1
#endif
#include "mcu_stm32h743xx.h"
#endif

#if defined __stm32h750xx
#define CORE_M7 1
#define ARM_MATH_CM7 1
#define STM32_FLASH_LAYOUT_128K_PAGES 1
#if !defined STM32H750xx
#define STM32H750xx 1
#endif
#include "mcu_stm32h750xx.h"
#endif

#if defined __stm32h735xx
#define CORE_M7 1
#define ARM_MATH_CM7 1
#define STM32_FLASH_LAYOUT_128K_PAGES 1
#if !defined STM32H735xx
#define STM32H735xx 1
#endif
#include "mcu_stm32h735xx.h"
#endif


enum {
    STM32_DMA_PRIORITY_LOW,
    STM32_DMA_PRIORITY_MEDIUM,
    STM32_DMA_PRIORITY_HIGH,
    STM32_DMA_PRIORITY_VERY_HIGH,
};

enum {
    STM32_DMA1 = 0, STM32_DMA2 = 1, STM32_DMA_DISABLED = 100
};

typedef struct {
    u8 dma_number;
    u8 stream_number;
    u8 channel_number;
    u8 priority;
    u32 o_flags;
} stm32_dma_channel_config_t;

typedef struct {
    u8 dma_number;
    u8 stream_number;
    u8 channel_number;
    u8 priority;
    u32 o_flags;
} stm32_mdma_channel_config_t;

typedef struct {
    stm32_dma_channel_config_t tx;
    stm32_dma_channel_config_t rx;
#if defined STM32H7 || defined STM32F7
    void *misaligned_read_cache_buffer;
    size_t misaligned_read_cache_buffer_size;
#endif
} stm32_dma_config_t;

typedef struct {
    void *rx_overflow_buffer;
    u32 rx_overflow_buffer_size;
} stm32_dma_state_t;

typedef struct {
    spi_config_t spi_config;
    stm32_dma_config_t dma_config;
} stm32_spi_dma_config_t;

typedef struct {
    qspi_config_t qspi_config;
    stm32_dma_config_t dma_config;
} stm32_qspi_dma_config_t;

typedef struct {
    i2s_config_t i2s_config;
    stm32_dma_config_t dma_config;
} stm32_i2s_spi_dma_config_t;

typedef struct {
    i2s_config_t i2s_config;
    stm32_dma_channel_config_t dma_config;
} stm32_sai_dma_config_t;

typedef struct {
    sdio_config_t sdio_config;
    stm32_dma_config_t dma_config;
} stm32_sdio_dma_config_t;

typedef struct {
    uart_config_t uart_config;
    stm32_dma_config_t dma_config;
} stm32_uart_dma_config_t;

typedef struct {
    mmc_config_t mmc_config;
    stm32_dma_config_t dma_config;
} stm32_mmc_dma_config_t;

typedef struct {
    adc_config_t adc_config;
    stm32_dma_channel_config_t dma_config;
} stm32_adc_dma_config_t;

typedef struct {
    dac_config_t dac_config;
    stm32_dma_channel_config_t dma_config;
} stm32_dac_dma_config_t;

#define STM32_ETH_DMA_MAX_PACKET_SIZE                                          \
  (1536U) // must match ETH_MAX_PACKET_SIZE in hal_conf.h
#define STM32_ETH_DMA_DESCRIPTOR_COUNT                                         \
  (4U) // must match ETH_RXBUFNB and ETH_TXBUFNB in hal_conf.h

#define STM32_ETH_DMA_BUFFER_SIZE                                              \
  (STM32_ETH_DMA_MAX_PACKET_SIZE * STM32_ETH_DMA_DESCRIPTOR_COUNT)

/*
 * for declaring buffers
 * u8 eth_tx_buffer[STM32_ETH_DMA_BUFFER_SIZE];
 * u8 eth_rx_buffer[STM32_ETH_DMA_BUFFER_SIZE];
 */

typedef struct {
    eth_config_t eth_config;
    void *tx_buffer;
    void *rx_buffer;
} stm32_eth_dma_config_t;

enum {
    STM32_DMA_FLAG_IS_NORMAL = 0,
    STM32_DMA_FLAG_IS_CIRCULAR = (1 << 0),
    STM32_DMA_FLAG_IS_FIFO = (1 << 1),
    STM32_DMA_FLAG_IS_MEMORY_TO_PERIPH = (1 << 2), //
    STM32_DMA_FLAG_IS_PERIPH_TO_MEMORY = 0,        // default value
    STM32_DMA_FLAG_IS_MEMORY_BYTE = (1 << 4),      // default size is word
    STM32_DMA_FLAG_IS_MEMORY_HALFWORD = (1 << 5),
    STM32_DMA_FLAG_IS_MEMORY_WORD = 0,
    STM32_DMA_FLAG_IS_PERIPH_BYTE = (1 << 7), // default size is word
    STM32_DMA_FLAG_IS_PERIPH_HALFWORD = (1 << 8),
    STM32_DMA_FLAG_IS_PERIPH_WORD = 0,         // default
    STM32_DMA_FLAG_IS_PERIPH_SINGLE = 0,       // default inc is single
    STM32_DMA_FLAG_IS_MEMORY_SINGLE = 0,       // default inc is single
    STM32_DMA_FLAG_IS_MEMORY_INC4 = (1 << 11), // default inc is single
    STM32_DMA_FLAG_IS_MEMORY_INC8 = (1 << 12),
    STM32_DMA_FLAG_IS_MEMORY_INC16 = (1 << 13),
    STM32_DMA_FLAG_IS_PERIPH_INC4 = (1 << 15),
    STM32_DMA_FLAG_IS_PERIPH_INC8 = (1 << 16),
    STM32_DMA_FLAG_IS_PERIPH_INC16 = (1 << 17),
    STM32_DMA_FLAG_IS_MEMORY_INC_ENABLE = 0, // default is increment memory
    STM32_DMA_FLAG_IS_MEMORY_INC_DISABLE
    = (1 << 18), // default is increment memory
    STM32_DMA_FLAG_IS_PERIPH_INC_ENABLE
    = (1 << 19), // default is don't increment peripheral
    STM32_DMA_FLAG_IS_PERIPH_INC_DISABLE
    = 0, // default is don't increment peripheral
    STM32_DMA_FLAG_IS_FIFO_THRESHOLD_QUARTER = (1 << 20),
    STM32_DMA_FLAG_IS_FIFO_THRESHOLD_THREE_QUARTER = (1 << 21),
    STM32_DMA_FLAG_IS_FIFO_THRESHOLD_HALF = 0, // default
    STM32_DMA_FLAG_IS_FIFO_THRESHOLD_FULL = (1 << 22),
    STM32_DMA_FLAG_IS_PFCTRL = (1 << 23)
};

#define STM32_NUCLEO144_DECLARE_MCU_BOARD_CONFIG(\
  cpu_frequency, \
  event_handler_value, \
  arch_config_value, \
  o_sos_debug_value)                                                                                                                                                        \
  MCU_DECLARE_SECRET_KEY_32(secret_key)                                                                                                                                     \
  const mcu_board_config_t mcu_board_config = {                                                                                                                             \
    .core_cpu_freq = cpu_frequency,                                                                                                                                         \
    .usb_max_packet_zero = 64,                                                                                                                                              \
    .debug_uart_port = 2,                                                                                                                                                   \
    .debug_uart_attr                                                                                                                                                        \
    = {.pin_assignment = {.rx = {3, 9}, .tx = {3, 8}, .cts = {0xff, 0xff}, .rts = {0xff, 0xff}}, .freq = 115200, .o_flags = UART_FLAG_SET_LINE_CODING_DEFAULT, .width = 8}, \
    .secret_key_address = secret_key,                                                                                                                                       \
    .secret_key_size = 32,                                                                                                                                                  \
    .o_flags = MCU_BOARD_CONFIG_FLAG_LED_ACTIVE_HIGH,                                                                                                                       \
    .event_handler = event_handler_value,                                                                                                                                   \
    .led = {1, 7},                                                                                                                                                          \
    .arch_config = arch_config_value,                                                                                                                                       \
    .o_mcu_debug = o_sos_debug_value}

#define STM32_NUCLEO144_DECLARE_BOOT_BOARD_CONFIG(link_transport_driver_value) \
  extern u32 _flash_start;                                                     \
  const bootloader_board_config_t boot_board_config = {                        \
    .sw_req_loc = 0x20004000,                                                  \
    .sw_req_value = 0x55AA55AA,                                                \
    .program_start_addr = __KERNEL_START_ADDRESS,                              \
    .hw_req = {2, 13},                                                         \
    .o_flags = BOOT_BOARD_CONFIG_FLAG_HW_REQ_ACTIVE_HIGH,                      \
    .link_transport_driver = link_transport_driver_value,                      \
    .id = __HARDWARE_ID}

#endif
#endif /* STM32_ARCH_H_ */
