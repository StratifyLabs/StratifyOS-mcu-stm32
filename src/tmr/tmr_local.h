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

#include "stm32_local.h"

#include "stm32_dma.h"

enum {
  CHANNEL_TYPE_NONE,
  CHANNEL_TYPE_OUTPUT_COMPARE,
  CHANNEL_TYPE_INPUT_CAPTURE,
  CHANNEL_TYPE_PWM
};

extern TIM_TypeDef *const tmr_local_regs_table[MCU_TMR_PORTS];
extern u8 const tmr_local_irqs[MCU_TMR_PORTS];
extern u32 const tmr_local_channels[MCU_TMR_CHANNELS];
extern tmr_local_t m_tmr_local[MCU_TMR_PORTS] MCU_SYS_MEM;

#define TMR_DECLARE_LOCAL(x, y) DEVFS_DRIVER_DECLARE_LOCAL(x, y)

int tmr_local_open(const devfs_handle_t *handle);
int tmr_local_close(const devfs_handle_t *handle);
int tmr_local_getinfo(const devfs_handle_t *handle, void *ctl);
int tmr_local_setattr(const devfs_handle_t *handle, void *ctl);
int tmr_local_enable(const devfs_handle_t *handle, void *ctl);
int tmr_local_disable(const devfs_handle_t *handle, void *ctl);
int tmr_local_setchannel(const devfs_handle_t *handle, void *ctl);
int tmr_local_getchannel(const devfs_handle_t *handle, void *ctl);
int tmr_local_setaction(const devfs_handle_t *handle, void *ctl);
int tmr_local_set(const devfs_handle_t *handle, void *ctl);
int tmr_local_get(const devfs_handle_t *handle, void *ctl);

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
#define TIM2_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_ENABLE(2)
#define TIM2_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_DISABLE(2)
#else
#define TIM2_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM2_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM3
#define TIM3_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_ENABLE(3)
#define TIM3_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_DISABLE(3)
#else
#define TIM3_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM3_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM4
#define TIM4_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_ENABLE(4)
#define TIM4_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_DISABLE(4)
#else
#define TIM4_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM4_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM5
#define TIM5_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_ENABLE(5)
#define TIM5_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_DISABLE(5)
#else
#define TIM5_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM5_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM6
#define TIM6_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_ENABLE(6)
#define TIM6_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_DISABLE(6)
#else
#define TIM6_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM6_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM7
#define TIM7_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_ENABLE(7)
#define TIM7_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_DISABLE(7)
#else
#define TIM7_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM7_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM8
#define TIM8_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_ENABLE(8)
#define TIM8_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_DISABLE(8)
#else
#define TIM8_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM8_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM9
#define TIM9_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_ENABLE(9)
#define TIM9_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_DISABLE(9)
#else
#define TIM9_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM9_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM10
#define TIM10_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_ENABLE(10)
#define TIM10_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_DISABLE(10)
#else
#define TIM10_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM10_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM11
#define TIM11_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_ENABLE(11)
#define TIM11_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_DISABLE(11)
#else
#define TIM11_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM11_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM12
#define TIM12_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_ENABLE(12)
#define TIM12_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_DISABLE(12)
#else
#define TIM12_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM12_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM13
#define TIM13_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_ENABLE(13)
#define TIM13_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_DISABLE(13)
#else
#define TIM13_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM13_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM14
#define TIM14_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_ENABLE(14)
#define TIM14_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_DISABLE(14)
#else
#define TIM14_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM14_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM15
#define TIM15_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_ENABLE(15)
#define TIM15_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_DISABLE(15)
#else
#define TIM15_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM15_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM16
#define TIM16_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_ENABLE(16)
#define TIM16_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_DISABLE(16)
#else
#define TIM16_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM16_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#if defined TIM17
#define TIM17_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_ENABLE(17)
#define TIM17_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_DISABLE(17)
#else
#define TIM17_CASE_CLOCK_ENABLE() TIM_CASE_CLOCK_UNAVAILABLE
#define TIM17_CASE_CLOCK_DISABLE() TIM_CASE_CLOCK_UNAVAILABLE
#endif

#endif /* TMR_LOCAL_H_ */
