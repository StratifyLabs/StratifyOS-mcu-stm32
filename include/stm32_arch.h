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

#if defined __stm32f446xx
#define CORE_M4 1
#define ARM_MATH_CM4 1
#if !defined STM32F446xx
#define STM32F446xx 1
#endif
#include "mcu_stm32f446xx.h"
#endif

#if defined __stm32f401xc
#define CORE_M4 1
#define ARM_MATH_CM4 1
#if !defined STM32F401xC
#define STM32F401xC 1
#endif
#include "mcu_stm32f401xc.h"
#endif

#if defined __stm32f411xe
#define CORE_M4 1
#define ARM_MATH_CM4 1
#if !defined STM32F411xE
#define STM32F411xE 1
#endif
#include "mcu_stm32f411xe.h"
#endif

#if defined __stm32f417xx
#define CORE_M4 1
#define ARM_MATH_CM4 1
#if !defined STM32F417xx
#define STM32F417xx 1
#endif
#include "mcu_stm32f417xx.h"
#endif



#endif /* STM32_ARCH_H_ */
