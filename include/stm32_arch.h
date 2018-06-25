/* Copyright 2011-2017 Tyler Gilbert;
 * This file is part of Stratify OS.
 *
 * Stratify OS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Stratify OS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Stratify OS.  If not, see <http://www.gnu.org/licenses/>. */

#ifndef STM32_ARCH_H_
#define STM32_ARCH_H_

#include <mcu/types.h>
#include <mcu/spi.h>
#include <mcu/i2s.h>

#if defined __stm32f401xc
#define CORE_M4 1
#define ARM_MATH_CM4 1
#define STM32_FLASH_LAYOUT_16_16_16_16_64_128_128_128 1
#if !defined STM32F401xC
#define STM32F401xC 1
#endif
#include "mcu_stm32f401xc.h"
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


typedef struct MCU_PACK {
    u32 o_flags;
    u16 clock_pllm;
    u16 clock_plln;
    u16 clock_pllp;
    u16 clock_pllq;
    u16 clock_pllr;
    u16 clock_ahb_clock_divider;
    u16 clock_apb1_clock_divider;
    u16 clock_apb2_clock_divider;
    u16 clock_voltage_scale;
    u16 clock_flash_latency;
    u16 usb_rx_buffer_size;
    void * usb_rx_buffer;
} stm32_config_t;

enum {
    STM32_CONFIG_FLAG_IS_HSE_ON = (1<<0),
    STM32_CONFIG_FLAG_IS_48_CLOCK_PLLQ = 0, //default values
    STM32_CONFIG_FLAG_IS_OVERDRIVE_ON = (1<<1)
};

enum {
    STM32_DMA_PRIORITY_LOW,
    STM32_DMA_PRIORITY_MEDIUM,
    STM32_DMA_PRIORITY_HIGH,
    STM32_DMA_PRIORITY_VERY_HIGH,
};

typedef struct {
    u8 dma_number;
    u8 stream_number;
    u8 channel_number;
    u8 priority;
} stm32_dma_channel_config_t;

typedef struct {
    stm32_dma_channel_config_t tx;
    stm32_dma_channel_config_t rx;
} stm32_dma_config_t;

typedef struct {
    void * rx_overflow_buffer;
    u32 rx_overflow_buffer_size;
} stm32_dma_state_t;

typedef struct {
    spi_config_t spi_config;
    stm32_dma_config_t dma_config;
} stm32_spi_dma_config_t;

typedef struct {
    i2s_config_t i2s_config;
    stm32_dma_config_t dma_config;
} stm32_i2s_spi_dma_config_t;

#define STM32_NUCLEO144_DECLARE_MCU_BOARD_CONFIG(cpu_frequency, event_handler_value, arch_config_value) \
    const mcu_board_config_t mcu_board_config = { \
    .core_cpu_freq = cpu_frequency, \
    .usb_max_packet_zero = 64, \
    .debug_uart_port = 2, \
    .debug_uart_attr = { \
    .pin_assignment = { \
    .rx = {3, 9}, \
    .tx = {3, 8}, \
    .cts = {0xff, 0xff}, \
    .rts = {0xff, 0xff} \
    }, \
    .freq = 115200, \
    .o_flags = UART_FLAG_SET_LINE_CODING_DEFAULT, \
    .width = 8 \
    }, \
    .o_flags = MCU_BOARD_CONFIG_FLAG_LED_ACTIVE_HIGH, \
    .event_handler = event_handler_value, \
    .led = {1, 7}, \
    .arch_config = arch_config_value }

#define STM32_NUCLEO144_DECLARE_BOOT_BOARD_CONFIG(link_transport_driver_value) \
    extern u32 _flash_start; \
    const bootloader_board_config_t boot_board_config = { \
    .sw_req_loc = 0x20004000, \
    .sw_req_value = 0x55AA55AA, \
    .program_start_addr = __KERNEL_START_ADDRESS, \
    .hw_req = {2, 13}, \
    .o_flags = BOOT_BOARD_CONFIG_FLAG_HW_REQ_ACTIVE_HIGH, \
    .link_transport_driver = link_transport_driver_value, \
    .id = __HARDWARE_ID }



#endif /* STM32_ARCH_H_ */
