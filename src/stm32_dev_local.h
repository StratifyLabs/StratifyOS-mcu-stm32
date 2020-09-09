#ifndef MCU_ARCH_STM32_STM32_DEV_LOCAL_H
#define MCU_ARCH_STM32_STM32_DEV_LOCAL_H

#include "stm32_dev_local_conf.h"

// local types
typedef struct MCU_PACK {
  TIM_HandleTypeDef hal_handle; // must be first
  mcu_event_handler_t handler[MCU_TMR_CHANNELS * 2];
  mcu_event_handler_t period_handler;
  u8 ref_count;
} tmr_local_t;

#endif // MCU_ARCH_STM32_STM32_DEV_LOCAL_H
