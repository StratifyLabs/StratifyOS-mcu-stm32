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

#include <errno.h>
#include <fcntl.h>
#include <cortexm/cortexm.h>
#include <mcu/core.h>
#include <mcu/pio.h>
#include <mcu/debug.h>

#if defined STM32F4
#include "stm32f4xx/stm32f4xx_hal_conf.h"
#elif defined STM32F7
#include "stm32f7xx/stm32f7xx_hal_conf.h"
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
