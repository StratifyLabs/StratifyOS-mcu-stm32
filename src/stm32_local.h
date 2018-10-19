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
 * along with Stratify OS.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */

#ifndef STM32_LOCAL_H_
#define STM32_LOCAL_H_

#include <mcu/types.h>
#include <mcu/core.h>

#define USE_FULL_LL_DRIVER

#include "stm32_arch.h"

#include <fcntl.h>
#include <cortexm/cortexm.h>
#include <mcu/core.h>
#include <mcu/pio.h>
#include <mcu/debug.h>

#if defined STM32F4
#include "stm32f4xx/stm32f4xx_hal_conf.h"

//#define STM32_LOCAL_HAS_PERIPH_CLOCK_SELECTOR 1

#if defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx) || defined(STM32F446xx) || defined(STM32F469xx) ||\
	defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) ||\
	defined(STM32F413xx) || defined(STM32F423xx)
#define STM32_LOCAL_HAS_RCC_PLLR 1
#endif

#if defined(STM32F446xx) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx) || \
	defined(STM32F412Zx) || defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
#define STM32_LOCAL_HAS_PERIPH_CLOCK_48 1
#endif

#endif

#if defined STM32F7
#include "stm32f7xx/stm32f7xx_hal_conf.h"

typedef SDMMC_TypeDef SDIO_TypeDef;
#define SDIO_CLOCK_EDGE_RISING SDMMC_CLOCK_EDGE_RISING
#define SDIO_CLOCK_EDGE_FALLING SDMMC_CLOCK_EDGE_FALLING
#define SDIO_CLOCK_EDGE_BOTH SDMMC_CLOCK_EDGE_BOTH
#define SDIO_CLOCK_BYPASS_DISABLE SDMMC_CLOCK_BYPASS_DISABLE
#define SDIO_CLOCK_BYPASS_ENABLE SDMMC_CLOCK_BYPASS_ENABLE
#define SDIO_CLOCK_POWER_SAVE_DISABLE SDMMC_CLOCK_POWER_SAVE_DISABLE
#define SDIO_CLOCK_POWER_SAVE_ENABLE SDMMC_CLOCK_POWER_SAVE_ENABLE
#define SDIO_BUS_WIDE_1B SDMMC_BUS_WIDE_1B
#define SDIO_BUS_WIDE_4B SDMMC_BUS_WIDE_4B
#define SDIO_BUS_WIDE_8B SDMMC_BUS_WIDE_8B
#define SDIO_HARDWARE_FLOW_CONTROL_DISABLE SDMMC_HARDWARE_FLOW_CONTROL_DISABLE
#define SDIO_HARDWARE_FLOW_CONTROL_ENABLE SDMMC_HARDWARE_FLOW_CONTROL_ENABLE

#if defined(STM32F723xx)
#define STM32_LOCAL_HAS_PERIPH_CLOCK_48 1
#endif

#include "stm32f7xx/stm32f7xx_hal_rcc_ex.h"

#if defined (STM32F745xx) || defined (STM32F746xx) || defined (STM32F756xx) || defined (STM32F765xx) || defined (STM32F767xx) || \
	defined (STM32F769xx) || defined (STM32F777xx) || defined (STM32F779xx)
#define STM32_LOCAL_HAS_PERIPH_CLOCK_48 1
#endif

#define RCC_CLK48CLKSOURCE_PLLQ RCC_CLK48SOURCE_PLL

#endif

#if defined STM32H7

#include "stm32h7xx/stm32h7xx_hal_conf.h"

typedef SDMMC_TypeDef SDIO_TypeDef;
#define SDIO_CLOCK_EDGE_RISING SDMMC_CLOCK_EDGE_RISING
#define SDIO_CLOCK_EDGE_FALLING SDMMC_CLOCK_EDGE_FALLING
#define SDIO_CLOCK_POWER_SAVE_DISABLE SDMMC_CLOCK_POWER_SAVE_DISABLE
#define SDIO_CLOCK_POWER_SAVE_ENABLE SDMMC_CLOCK_POWER_SAVE_ENABLE
#define SDIO_BUS_WIDE_1B SDMMC_BUS_WIDE_1B
#define SDIO_BUS_WIDE_4B SDMMC_BUS_WIDE_4B
#define SDIO_BUS_WIDE_8B SDMMC_BUS_WIDE_8B
#define SDIO_HARDWARE_FLOW_CONTROL_DISABLE SDMMC_HARDWARE_FLOW_CONTROL_DISABLE
#define SDIO_HARDWARE_FLOW_CONTROL_ENABLE SDMMC_HARDWARE_FLOW_CONTROL_ENABLE
//STM32H7 drops the Q but it is still connected to PLLQ?
#define RCC_CLK48CLKSOURCE_PLLQ RCC_CLK48SOURCE_PLL
#define FLASH_TYPEPROGRAM_WORD FLASH_TYPEPROGRAM_FLASHWORD
#endif


#if defined STM32L4
#include "stm32l4xx/stm32l4xx_hal_conf.h"
#define STM32_LOCAL_HAS_RCC_PLLR 1
#define STM32_LOCAL_HAS_PERIPH_CLOCK_SELECTOR 1
#define SDIO_CLOCK_EDGE_RISING SDMMC_CLOCK_EDGE_RISING
#define SDIO_CLOCK_EDGE_FALLING SDMMC_CLOCK_EDGE_FALLING
#define SDIO_CLOCK_EDGE_BOTH SDMMC_CLOCK_EDGE_BOTH
#define SDIO_CLOCK_BYPASS_DISABLE SDMMC_CLOCK_BYPASS_DISABLE
#define SDIO_CLOCK_BYPASS_ENABLE SDMMC_CLOCK_BYPASS_ENABLE
#define SDIO_CLOCK_POWER_SAVE_DISABLE SDMMC_CLOCK_POWER_SAVE_DISABLE
#define SDIO_CLOCK_POWER_SAVE_ENABLE SDMMC_CLOCK_POWER_SAVE_ENABLE
#define SDIO_BUS_WIDE_1B SDMMC_BUS_WIDE_1B
#define SDIO_BUS_WIDE_4B SDMMC_BUS_WIDE_4B
#define SDIO_BUS_WIDE_8B SDMMC_BUS_WIDE_8B
#define SDIO_HARDWARE_FLOW_CONTROL_DISABLE SDMMC_HARDWARE_FLOW_CONTROL_DISABLE
#define SDIO_HARDWARE_FLOW_CONTROL_ENABLE SDMMC_HARDWARE_FLOW_CONTROL_ENABLE
#endif

int hal_set_alternate_pin_function(mcu_pin_t pin, core_periph_t function, int periph_port, int mode, int speed, int pull);
GPIO_TypeDef * const hal_get_pio_regs(u8 port);
int hal_get_alternate_function(int gpio_port, int pin, core_periph_t function, int periph_port);

u32 stm32_local_decode_flash_latency(u16 value);
u32 stm32_local_decode_voltage_scale(u16 value);
u32 stm32_local_decode_sysclk_divider(u16 value);
u32 stm32_local_decode_hclk_divider(u16 value);
u32 stm32_local_decode_hclk_divider(u16 value);

#endif /* STM32_LOCAL_H_ */
