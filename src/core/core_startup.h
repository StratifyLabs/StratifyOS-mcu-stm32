// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md


#ifndef SRC_CORE_CORE_STARTUP_H_
#define SRC_CORE_CORE_STARTUP_H_

#include <cortexm/cortexm.h>
#include <cortexm/task.h>
#include <mcu/mcu.h>
#include <sdk/types.h>

void core_init();
int _main();

#define _DECLARE_ISR(name)                                                     \
  void mcu_core_##name##_isr() MCU_ALIAS(mcu_core_default_isr)
#define _ISR(name) mcu_core_##name##_isr

#endif /* SRC_CORE_CORE_STARTUP_H_ */
