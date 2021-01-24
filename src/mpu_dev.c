// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include "cortexm/mpu.h"
#include <stdbool.h>

// not used
int mpu_dev_init() {
  int err;

  err = mpu_enable_region(
    1,
    (void *)0x40000000, // APB/AHB Peripherals
    0x60000000 - 0x40000000,
    MPU_ACCESS_PRW_UR,
    MPU_MEMORY_PERIPHERALS,
    false);

  if (err < 0) {
    return err;
  }

  return 0;
}
