/* Copyright 2011-2019 Tyler Gilbert;
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

#ifndef TMR_LOCAL_H_
#define TMR_LOCAL_H_

#include <mcu/debug.h>
#include <mcu/tmr.h>

#include "stm32_dma.h"

typedef struct MCU_PACK {
  TIM_HandleTypeDef hal_handle; // must be first
  mcu_event_handler_t handler[MCU_TMR_CHANNELS * 2];
  mcu_event_handler_t period_handler;
  u8 ref_count;
} tmr_local_t;

enum {
  CHANNEL_TYPE_NONE,
  CHANNEL_TYPE_OUTPUT_COMPARE,
  CHANNEL_TYPE_INPUT_CAPTURE,
  CHANNEL_TYPE_PWM
};

extern TIM_TypeDef *const tmr_regs_table[MCU_TMR_PORTS];
extern u8 const tmr_irqs[MCU_TMR_PORTS];
extern u32 const tmr_channels[MCU_TMR_CHANNELS];

#define TIM_CASE_CLOCK_UNAVAILABLE

#define TIM_CASE_CLOCK_ENABLE(x)                                               \
  case x - 1:                                                                  \
    __HAL_RCC_TIM##x##_CLK_ENABLE();                                           \
    break;
#define TIM_CASE_CLOCK_DISABLE(x)                                              \
  case x - 1:                                                                  \
    __HAL_RCC_TIM##x##_CLK_DISABLE();                                          \
    break;

#if defined TIM1
#define TIM1_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_ENABLE(1)
#define TIM1_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_DISABLE(1)
#else
#define TIM1_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM1_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM2
#define TIM2_CASE_CLOCK_ENABLE()                                               \
  case 1:                                                                      \
    __HAL_RCC_TIM2_CLK_ENABLE();                                               \
    break;
#define TIM2_CASE_CLOCK_DISABLE()                                              \
  case 1:                                                                      \
    __HAL_RCC_TIM2_CLK_DISABLE();                                              \
    break;
#else
#define TIM2_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM2_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM3
#define TIM3_CASE_CLOCK_ENABLE()                                               \
  case 2:                                                                      \
    __HAL_RCC_TIM3_CLK_ENABLE();                                               \
    break;
#define TIM3_CASE_CLOCK_DISABLE()                                              \
  case 2:                                                                      \
    __HAL_RCC_TIM3_CLK_DISABLE();                                              \
    break;
#else
#define TIM3_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM3_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM4
#define TIM4_CASE_CLOCK_ENABLE()                                               \
  case 3:                                                                      \
    __HAL_RCC_TIM4_CLK_ENABLE();                                               \
    break;
#define TIM4_CASE_CLOCK_DISABLE()                                              \
  case 3:                                                                      \
    __HAL_RCC_TIM4_CLK_DISABLE();                                              \
    break;
#else
#define TIM4_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM4_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM5
#define TIM5_CASE_CLOCK_ENABLE()                                               \
  case 4:                                                                      \
    __HAL_RCC_TIM5_CLK_ENABLE();                                               \
    break;
#define TIM5_CASE_CLOCK_DISABLE()                                              \
  case 4:                                                                      \
    __HAL_RCC_TIM5_CLK_DISABLE();                                              \
    break;
#else
#define TIM5_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM5_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM6
#define TIM6_CASE_CLOCK_ENABLE()                                               \
  case 5:                                                                      \
    __HAL_RCC_TIM6_CLK_ENABLE();                                               \
    break;
#define TIM6_CASE_CLOCK_DISABLE()                                              \
  case 5:                                                                      \
    __HAL_RCC_TIM6_CLK_DISABLE();                                              \
    break;
#else
#define TIM6_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM6_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM7
#define TIM7_CASE_CLOCK_ENABLE()                                               \
  case 6:                                                                      \
    __HAL_RCC_TIM7_CLK_ENABLE();                                               \
    break;
#define TIM7_CASE_CLOCK_DISABLE()                                              \
  case 6:                                                                      \
    __HAL_RCC_TIM7_CLK_DISABLE();                                              \
    break;
#else
#define TIM7_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM7_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM8
#define TIM8_CASE_CLOCK_ENABLE()                                               \
  case 7:                                                                      \
    __HAL_RCC_TIM8_CLK_ENABLE();                                               \
    break;
#define TIM8_CASE_CLOCK_DISABLE()                                              \
  case 7:                                                                      \
    __HAL_RCC_TIM8_CLK_DISABLE();                                              \
    break;
#else
#define TIM8_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM8_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM9
#define TIM9_CASE_CLOCK_ENABLE()                                               \
  case 8:                                                                      \
    __HAL_RCC_TIM9_CLK_ENABLE();                                               \
    break;
#define TIM9_CASE_CLOCK_DISABLE()                                              \
  case 8:                                                                      \
    __HAL_RCC_TIM9_CLK_DISABLE();                                              \
    break;
#else
#define TIM9_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM9_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM10
#define TIM10_CASE_CLOCK_ENABLE()                                              \
  case 9:                                                                      \
    __HAL_RCC_TIM10_CLK_ENABLE();                                              \
    break;
#define TIM10_CASE_CLOCK_DISABLE()                                             \
  case 9:                                                                      \
    __HAL_RCC_TIM10_CLK_DISABLE();                                             \
    break;
#else
#define TIM10_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM10_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM11
#define TIM11_CASE_CLOCK_ENABLE()                                              \
  case 10:                                                                     \
    __HAL_RCC_TIM11_CLK_ENABLE();                                              \
    break;
#define TIM11_CASE_CLOCK_DISABLE()                                             \
  case 10:                                                                     \
    __HAL_RCC_TIM11_CLK_DISABLE();                                             \
    break;
#else
#define TIM11_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM11_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM12
#define TIM12_CASE_CLOCK_ENABLE()                                              \
  case 11:                                                                     \
    __HAL_RCC_TIM12_CLK_ENABLE();                                              \
    break;
#define TIM12_CASE_CLOCK_DISABLE()                                             \
  case 11:                                                                     \
    __HAL_RCC_TIM12_CLK_DISABLE();                                             \
    break;
#else
#define TIM12_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM12_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM13
#define TIM13_CASE_CLOCK_ENABLE()                                              \
  case 12:                                                                     \
    __HAL_RCC_TIM13_CLK_ENABLE();                                              \
    break;
#define TIM13_CASE_CLOCK_DISABLE()                                             \
  case 12:                                                                     \
    __HAL_RCC_TIM13_CLK_DISABLE();                                             \
    break;
#else
#define TIM13_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM13_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM14
#define TIM14_CASE_CLOCK_ENABLE()                                              \
  case 13:                                                                     \
    __HAL_RCC_TIM14_CLK_ENABLE();                                              \
    break;
#define TIM14_CASE_CLOCK_DISABLE()                                             \
  case 13:                                                                     \
    __HAL_RCC_TIM14_CLK_DISABLE();                                             \
    break;
#else
#define TIM14_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM14_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM15
#define TIM15_CASE_CLOCK_ENABLE()                                              \
  case 14:                                                                     \
    __HAL_RCC_TIM15_CLK_ENABLE();                                              \
    break;
#define TIM15_CASE_CLOCK_DISABLE()                                             \
  case 14:                                                                     \
    __HAL_RCC_TIM15_CLK_DISABLE();                                             \
    break;
#else
#define TIM15_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM15_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM16
#define TIM16_CASE_CLOCK_ENABLE()                                              \
  case 15:                                                                     \
    __HAL_RCC_TIM16_CLK_ENABLE();                                              \
    break;
#define TIM16_CASE_CLOCK_DISABLE()                                             \
  case 15:                                                                     \
    __HAL_RCC_TIM16_CLK_DISABLE();                                             \
    break;
#else
#define TIM16_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM16_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM17
#define TIM17_CASE_CLOCK_ENABLE()                                              \
  case 16:                                                                     \
    __HAL_RCC_TIM17_CLK_ENABLE();                                              \
    break;
#define TIM17_CASE_CLOCK_DISABLE()                                             \
  case 16:                                                                     \
    __HAL_RCC_TIM17_CLK_DISABLE();                                             \
    break;
#else
#define TIM17_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM17_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#endif /* TMR_LOCAL_H_ */
