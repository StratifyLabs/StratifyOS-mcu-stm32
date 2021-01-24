// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include "stm32_local.h"
#include <cortexm/cortexm.h>
#include <mcu/core.h>
#include <mcu/mcu.h>

extern void mcu_set_sleep_mode(int *level);

int mcu_core_user_sleep(core_sleep_t level) {
  cortexm_svcall((cortexm_svcall_t)mcu_set_sleep_mode, &level);
  if (level < 0) {
    return level;
  }

  // Wait for an interrupt
  __WFI();
  return 0;
}
