// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include "mmc_local.h"
#include <mcu/sdio.h>

#if MCU_SDIO_PORTS > 0 && defined MMC_TypeDef

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  mmc,
  MMC_VERSION,
  SDIO_IOC_IDENT_CHAR,
  I_MCU_TOTAL + I_SDIO_TOTAL,
  mcu_mmc_getcid,
  mcu_mmc_getcsd,
  mcu_mmc_getstatus)

int mcu_mmc_open(const devfs_handle_t *handle) {
  return mmc_local_open(handle);
}

int mcu_mmc_close(const devfs_handle_t *handle) {
  return mmc_local_close(handle);
}

int mcu_mmc_getinfo(const devfs_handle_t *handle, void *ctl) {
  return mmc_local_getinfo(handle, ctl);
}

int mcu_mmc_setattr(const devfs_handle_t *handle, void *ctl) {
  return mmc_local_setattr(handle, ctl);
}

int mcu_mmc_setaction(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(mmc);
  mcu_action_t *action = ctl;

  if (action->handler.callback == 0) {
    if (action->o_events & MCU_EVENT_FLAG_DATA_READY) {
      HAL_MMC_Abort_IT(&state->hal_handle);
      devfs_execute_read_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(EIO),
        MCU_EVENT_FLAG_CANCELED);
    }

    if (action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE) {
      HAL_MMC_Abort_IT(&state->hal_handle);
      devfs_execute_write_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(EIO),
        MCU_EVENT_FLAG_CANCELED);
    }
  }

  cortexm_set_irq_priority(
    mmc_irqs[config->port],
    action->prio,
    action->o_events);
  return 0;
}

int mcu_mmc_getcid(const devfs_handle_t *handle, void *ctl) {
  return mmc_local_getcid(handle, ctl);
}

int mcu_mmc_getcsd(const devfs_handle_t *handle, void *ctl) {
  return mmc_local_getcsd(handle, ctl);
}

int mcu_mmc_getstatus(const devfs_handle_t *handle, void *ctl) {
  return mmc_local_getstatus(handle, ctl);
}

int mcu_mmc_write(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(mmc);
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.write, async);

  int loc;
  if (state->o_flags & MMC_LOCAL_FLAG_IS_BYTE_ADDRESSING) {
    loc = async->loc * 512;
  } else {
    loc = async->loc;
  }
  if (
    (HAL_MMC_WriteBlocks_IT(
      &state->hal_handle,
      async->buf,
      loc,
      async->nbyte / BLOCKSIZE))
    == HAL_OK) {
    return 0;
  }

  state->transfer_handler.write = 0;
  return SYSFS_SET_RETURN(EIO);
}

int mcu_mmc_read(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(mmc);
  int hal_result;
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.read, async);

  int loc;
  if (state->o_flags & MMC_LOCAL_FLAG_IS_BYTE_ADDRESSING) {
    loc = async->loc * 512;
  } else {
    loc = async->loc;
  }
  if (
    (hal_result = HAL_MMC_ReadBlocks_IT(
       &state->hal_handle,
       async->buf,
       loc,
       async->nbyte / BLOCKSIZE))
    == HAL_OK) {
    return 0;
  }

  state->transfer_handler.read = 0;
  return SYSFS_SET_RETURN(EIO);
}

#endif
