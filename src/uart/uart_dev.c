// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include <fcntl.h>
#include <sos/debug.h>
#include <mcu/pio.h>

#include "uart_local.h"

#if MCU_UART_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  uart,
  UART_VERSION,
  UART_IOC_IDENT_CHAR,
  I_MCU_TOTAL + I_UART_TOTAL,
  mcu_uart_get,
  mcu_uart_put,
  mcu_uart_flush)

int mcu_uart_open(const devfs_handle_t *handle) {
  return uart_local_open(handle);
}

int mcu_uart_close(const devfs_handle_t *handle) {
  return uart_local_close(handle);
}

int mcu_uart_getinfo(const devfs_handle_t *handle, void *ctl) {
  return uart_local_getinfo(handle, ctl);
}

int mcu_uart_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(uart);

  int result = uart_local_setattr(handle, ctl);
  if (result < 0) {
    return result;
  }

  if (state->fifo_config != 0) {
    result = HAL_UART_Receive_IT(
      &state->hal_handle,
      (u8 *)state->fifo_config->buffer,
      state->fifo_config->size);
    if (result != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }
  }

  return SYSFS_RETURN_SUCCESS;
}

int mcu_uart_setaction(const devfs_handle_t *handle, void *ctl) {
  return uart_local_setaction(handle, ctl);
}

int mcu_uart_put(const devfs_handle_t *handle, void *ctl) {
  return uart_local_put(handle, ctl);
}

int mcu_uart_flush(const devfs_handle_t *handle, void *ctl) {
  return uart_local_flush(handle, ctl);
}

int mcu_uart_get(const devfs_handle_t *handle, void *ctl) {
  return uart_local_get(handle, ctl);
}

int mcu_uart_read(const devfs_handle_t *handle, devfs_async_t *async) {
  return uart_local_read(handle, async);
}

int mcu_uart_write(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(uart);
  int result;

  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.write, async);

  result = HAL_UART_Transmit_IT(&state->hal_handle, async->buf, async->nbyte);
  if (result == HAL_OK) {
    return 0;
  }

  state->transfer_handler.write = 0;
  return SYSFS_SET_RETURN(EIO);
}

#endif
