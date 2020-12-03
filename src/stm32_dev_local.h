#ifndef MCU_ARCH_STM32_STM32_DEV_LOCAL_H
#define MCU_ARCH_STM32_STM32_DEV_LOCAL_H

#include <device/fifo.h>

#include "stm32_dev_local_conf.h"

// local types
typedef struct MCU_PACK {
  TIM_HandleTypeDef hal_handle; // must be first
  mcu_event_handler_t handler[MCU_TMR_CHANNELS * 2];
  mcu_event_handler_t period_handler;
  u8 ref_count;
} tmr_local_t;

typedef struct {
  UART_HandleTypeDef hal_handle;
  devfs_transfer_handler_t transfer_handler;
  fifo_state_t fifo_state;
  const fifo_config_t *fifo_config;
  u8 ref_count;
  u8 o_flags;
} uart_local_t;

typedef struct {
  u8 ref_count;
} pio_local_t;

#endif // MCU_ARCH_STM32_STM32_DEV_LOCAL_H
