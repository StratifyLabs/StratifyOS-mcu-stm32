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




#endif /* STM32_ARCH_H_ */
