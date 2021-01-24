// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include "cortexm/cortexm.h"
#include "mcu/core.h"
#include "sos/debug.h"
#include "mcu/pio.h"
#include "mcu/qspi.h"
#include "stm32_local.h"
#include <fcntl.h>
#include <sched.h>

#if MCU_QSPI_PORTS > 0

#include "qspi_local.h"

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  qspi,
  QSPI_VERSION,
  QSPI_IOC_IDENT_CHAR,
  I_QSPI_TOTAL + I_MCU_TOTAL,
  mcu_qspi_execcommand)

int mcu_qspi_open(const devfs_handle_t *handle) {
  return qspi_local_open(handle);
}

int mcu_qspi_close(const devfs_handle_t *handle) {
  return qspi_local_close(handle);
}

int mcu_qspi_getinfo(const devfs_handle_t *handle, void *ctl) {
  qspi_info_t *info = ctl;
  info->o_flags = QSPI_FLAG_SET_MASTER | QSPI_FLAG_EXECUTE_COMMAND;
  return 0;
}

int mcu_qspi_setattr(const devfs_handle_t *handle, void *ctl) {
  return qspi_local_setattr(handle, ctl);
}

int mcu_qspi_setaction(const devfs_handle_t *handle, void *ctl) {
  return qspi_local_setaction(handle, ctl);
}

int mcu_qspi_execcommand(const devfs_handle_t *handle, void *ctl) {
  return qspi_local_execcommand(handle, ctl);
}

int mcu_qspi_read(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(qspi);

  // can't read and write at the same time
  if (state->transfer_handler.write != 0) {
    return SYSFS_SET_RETURN(EBUSY);
  }
  // borrow async to qspi->transfer_handler.read
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.read, async);

  if (HAL_QSPI_Receive_IT(&state->hal_handle, async->buf) != HAL_OK) {
    state->transfer_handler.read = 0;
    return SYSFS_SET_RETURN(EIO);
  }
  return 0;
}

int mcu_qspi_write(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(qspi);
  // can't read and write at the same time
  if (state->transfer_handler.read != 0) {
    return SYSFS_SET_RETURN(EBUSY);
  }

  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.write, async);

  int result;
  if (
    (result = HAL_QSPI_Transmit_IT(&state->hal_handle, async->buf)) != HAL_OK) {
    state->transfer_handler.write = 0;
    return SYSFS_SET_RETURN(EIO);
  }

  return 0;
}

#endif
