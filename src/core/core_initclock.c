/* Copyright 2011-2016 Tyler Gilbert;
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

#include "stm32_local.h"

void sos_led_svcall_enable(void *args);

// requires mcu_core_osc_freq, mcu_board_config.core_cpu_freq, and
// mcu_board_config.core_periph_freq to be defined ext
int mcu_core_initclock(int div) {

  mcu_board_execute_event_handler(
    MCU_BOARD_CONFIG_EVENT_ROOT_INITIALIZE_CLOCK,
    0);
  cortexm_set_systick_reload(0x00ffffff);
  cortexm_start_systick();
  SystemCoreClock = mcu_board_config.core_cpu_freq;

  /* Configure Flash prefetch */
#if (PREFETCH_ENABLE != 0U)
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
#endif /* PREFETCH_ENABLE */

#if (ART_ACCLERATOR_ENABLE != 0)
  __HAL_FLASH_ART_ENABLE();
#endif /* ART_ACCLERATOR_ENABLE */

  /* Configure Flash prefetch, Instruction cache, Data cache */
#if (INSTRUCTION_CACHE_ENABLE != 0U)
  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
#endif /* INSTRUCTION_CACHE_ENABLE */

#if (DATA_CACHE_ENABLE != 0U)
  __HAL_FLASH_DATA_CACHE_ENABLE();
#endif /* DATA_CACHE_ENABLE */

  return 0;
}

// this overrides the weak function in the STM32 HAL library so that Stratify OS
// can take care of the SYS tick
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) { return HAL_OK; }

uint32_t HAL_GetTick(void) {
#if THIS_DOESNT_WORK
  static u32 tick = 0;
  // cortexm_delay_ms(1); //this needs to check if SYSTICK is running before
  // delaying
  return tick++;
#else
  return 1;
#endif
}

void HAL_Delay(__IO uint32_t Delay) { cortexm_delay_systick(Delay); }

/*! @} */
