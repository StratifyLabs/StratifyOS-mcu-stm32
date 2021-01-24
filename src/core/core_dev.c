// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include <sos/config.h>

//#include "config.h"
#include "stm32_config.h"
#include "stm32_flash.h"

static u32 mcu_core_get_reset_src();
static int enable_clock_out(int o_flags, int div);
static u32 mcu_core_reset_source = CORE_FLAG_IS_RESET_SYSTEM;

static void set_deep_sleep_registers();
static void set_deep_sleep_stop_registers();
static void set_deep_sleep_standby_registers();

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  core,
  CORE_VERSION,
  CORE_IOC_IDENT_CHAR,
  I_MCU_TOTAL + I_CORE_TOTAL,
  0,
  0,
  0,
  0)

int mcu_core_open(const devfs_handle_t *handle) { return 0; }
int mcu_core_close(const devfs_handle_t *handle) { return 0; }

int mcu_core_getinfo(const devfs_handle_t *handle, void *arg) {
  MCU_UNUSED_ARGUMENT(handle);
  core_info_t *info = arg;
  info->o_flags = 0;
  info->freq = sos_config.sys.core_clock_frequency;
  if (mcu_core_reset_source == CORE_FLAG_IS_RESET_SYSTEM) {
    mcu_core_reset_source = mcu_core_get_reset_src();
  }
  info->o_flags |= mcu_core_reset_source;

  stm32_get_serial_number(&info->serial);

  return 0;
}

int mcu_core_setattr(const devfs_handle_t *handle, void *arg) {
  const int port = 0;
  core_attr_t *attr = arg;

  if (attr == 0) {
    return SYSFS_SET_RETURN(EINVAL);
  }

  u32 o_flags = attr->o_flags;

  if (o_flags & CORE_FLAG_SET_CLKOUT) {
    enable_clock_out(o_flags, attr->freq);
  }

  if (o_flags & CORE_FLAG_EXEC_RESET) {
    mcu_core_reset(port, 0);
  }

  if (o_flags & CORE_FLAG_EXEC_INVOKE_BOOTLOADER) {
    mcu_core_invokebootloader(port, 0);
  }

  if (o_flags & CORE_FLAG_EXEC_SLEEP) {
    mcu_core_execsleep(port, (void *)CORE_SLEEP);
  } else if (o_flags & CORE_FLAG_EXEC_DEEPSLEEP) {
    mcu_core_execsleep(port, (void *)CORE_DEEPSLEEP);
  } else if (o_flags & CORE_FLAG_EXEC_DEEPSLEEP_STOP) {
    mcu_core_execsleep(port, (void *)CORE_DEEPSLEEP_STOP);
  } else if (o_flags & CORE_FLAG_EXEC_DEEPSLEEP_STANDBY) {
    mcu_core_execsleep(port, (void *)CORE_DEEPSLEEP_STANDBY);
  }

  return 0;
}

int mcu_core_setaction(const devfs_handle_t *handle, void *arg) {
  return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_core_execsleep(int port, void *arg) {
  int level;
  level = (int)arg;

  mcu_set_sleep_mode(&level);
  if (level < 0) {
    return level;
  }

  // Wait for an interrupts
  __WFI();
  return 0;
}

int mcu_core_reset(int port, void *arg) {
  // delay first
  cortexm_delay_ms(250);
  cortexm_reset(NULL);
  // doesn't arrive here
  return 0;
}

int mcu_core_invokebootloader(int port, void *arg) {
  bootloader_api_t api;
  mcu_get_bootloader_api(&api);
  cortexm_delay_ms(500);
  api.exec(0);
  return 0;
}


void mcu_set_sleep_mode(int *level) {
  SCB->SCR &= ~(1 << SCB_SCR_SLEEPDEEP_Pos);

  switch (*level) {
  case CORE_DEEPSLEEP_STOP:
    // turn off the flash as well
    set_deep_sleep_stop_registers();
    break;
  case CORE_DEEPSLEEP:
    set_deep_sleep_registers();
    break;
  case CORE_SLEEP:
    break;
  case CORE_DEEPSLEEP_STANDBY:
    set_deep_sleep_standby_registers();
    break;
  default:
    *level = -1;
    return;
  }
  *level = 0;
}

void set_deep_sleep_registers() {
  SCB->SCR |= (1 << SCB_SCR_SLEEPDEEP_Pos);
#if defined PWR_CR_PDDS
  MODIFY_REG(PWR->CR, (PWR_CR_PDDS | PWR_CR_LPDS), PWR_LOWPOWERREGULATOR_ON);
#endif
  HAL_PWREx_DisableFlashPowerDown();
}

void set_deep_sleep_stop_registers() {
  SCB->SCR |= (1 << SCB_SCR_SLEEPDEEP_Pos);
  HAL_PWREx_EnableFlashPowerDown();
}

void set_deep_sleep_standby_registers() {
  SCB->SCR |= (1 << SCB_SCR_SLEEPDEEP_Pos);
#if defined PWR_CR_PDDS
  SET_BIT(PWR->CR, PWR_CR_PDDS);
#endif
}

u32 mcu_core_get_reset_src() {
  u32 src = CORE_FLAG_IS_RESET_SYSTEM;

  u32 src_reg;
#if defined RCC_RSR_PINRSTF
  src_reg = RCC->RSR;
  RCC->RSR |= RCC_RSR_RMVF; // clear flags
#else
  src_reg = RCC->CSR;
  RCC->CSR |= RCC_CSR_RMVF; // clear flags
#endif

#if defined RCC_CSR_BORRSTF
  if (src_reg & RCC_CSR_BORRSTF) {
    return CORE_FLAG_IS_RESET_BOR;
  }
#endif

#if defined RCC_RSR_BORRSTF
  if (src_reg & RCC_RSR_BORRSTF) {
    return CORE_FLAG_IS_RESET_BOR;
  }
#endif

#if defined RCC_RSR_IWDGRSTF
  if (src_reg & (RCC_RSR_IWDGRSTF | RCC_RSR_WWDGRSTF)) {
    return CORE_FLAG_IS_RESET_WDT;
  }
#endif

#if defined RCC_RSR_WWDG1RSTF
  if (src_reg & (RCC_RSR_WWDG1RSTF)) {
    return CORE_FLAG_IS_RESET_WDT;
  }
#endif

#if defined RCC_CSR_PORRSTF
  if (src_reg & RCC_CSR_PORRSTF) {
    return CORE_FLAG_IS_RESET_POR;
  }
#endif

#if defined RCC_RSR_PORRSTF
  if (src_reg & RCC_RSR_PORRSTF) {
    return CORE_FLAG_IS_RESET_POR;
  }
#endif

#if defined RCC_CSR_SFTRSTF
  if (src_reg & RCC_CSR_SFTRSTF) {
    return CORE_FLAG_IS_RESET_SOFTWARE;
  }
#endif

#if defined RCC_RSR_SFTRSTF
  if (src_reg & RCC_RSR_SFTRSTF) {
    return CORE_FLAG_IS_RESET_SOFTWARE;
  }
#endif

#if defined RCC_CSR_PINRSTF
  if (src_reg & RCC_CSR_PINRSTF) {
    return CORE_FLAG_IS_RESET_EXTERNAL;
  }
#endif

#if defined RCC_RSR_PINRSTF
  if (src_reg & RCC_RSR_PINRSTF) {
    return CORE_FLAG_IS_RESET_EXTERNAL;
  }
#endif

  return src;
}

int enable_clock_out(int o_flags, int div) { return 0; }

void mcu_core_set_nvic_priority(int irq, int prio) {
  NVIC_SetPriority((IRQn_Type)irq, prio);
}

int mcu_core_write(const devfs_handle_t *cfg, devfs_async_t *async) {
  return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_core_read(const devfs_handle_t *cfg, devfs_async_t *async) {
  return SYSFS_SET_RETURN(ENOTSUP);
}
