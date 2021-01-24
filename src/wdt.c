// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include "stm32_local.h"

int mcu_wdt_init(int mode, int interval) {
  __HAL_RCC_WWDG_CLK_ENABLE();

  return 0;
}

int mcu_wdt_setinterval(int interval) { return 0; }

void mcu_wdt_root_reset(void *args) {
 
}

void mcu_wdt_reset() {

}

// ISR handler is in the cortex_m/fault.c source file
