/// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include <sos/config.h>

#include "stm32_local.h"

extern void SystemInit();
extern void *mcu_core_vector_table;

void stm32_initialize() {

#if defined STM32H7 || defined STM32F7
  SystemInit();
#endif

#if defined STM32H7
  __HAL_RCC_SYSCFG_CLK_ENABLE();
#endif

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
#if defined GPIOD
  __HAL_RCC_GPIOD_CLK_ENABLE();
#endif
#if defined GPIOE
  __HAL_RCC_GPIOE_CLK_ENABLE();
#endif
#if defined GPIOF
  __HAL_RCC_GPIOF_CLK_ENABLE();
#endif
#if defined GPIOG
  __HAL_RCC_GPIOG_CLK_ENABLE();
#endif
#if defined GPIOH
  __HAL_RCC_GPIOH_CLK_ENABLE();
#endif
#if defined GPIOI
  __HAL_RCC_GPIOI_CLK_ENABLE();
#endif
#if defined GPIOJ
  __HAL_RCC_GPIOJ_CLK_ENABLE();
#endif
#if defined GPIOK
  __HAL_RCC_GPIOK_CLK_ENABLE();
#endif
}

void stm32_initialize_systick() {

  cortexm_set_systick_reload(0x00ffffff);
  cortexm_start_systick();
  SystemCoreClock = sos_config.clock.frequency;

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
