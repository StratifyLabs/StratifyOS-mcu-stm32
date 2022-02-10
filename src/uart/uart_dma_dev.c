// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include "uart_local.h"
#include <fcntl.h>

#if MCU_UART_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  uart_dma,
  UART_VERSION,
  UART_IOC_IDENT_CHAR,
  I_MCU_TOTAL + I_UART_TOTAL,
  mcu_uart_dma_get,
  mcu_uart_dma_put,
  mcu_uart_dma_flush)

int mcu_uart_dma_open(const devfs_handle_t *handle) {
  return uart_local_open(handle);
}

int mcu_uart_dma_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_STATE(uart);

  if (state->ref_count == 1) {
    // disable the DMA
    const stm32_uart_dma_config_t *config;

    if (state->transfer_handler.read || state->transfer_handler.write) {
      HAL_UART_DMAStop(&state->hal_handle);
      devfs_execute_cancel_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(EIO),
        0);
    }

    config = handle->config;
    if (config) {
      stm32_dma_clear_handle(
        config->dma_config.rx.dma_number,
        config->dma_config.rx.stream_number);
      stm32_dma_clear_handle(
        config->dma_config.tx.dma_number,
        config->dma_config.tx.stream_number);
    }
  }

  return uart_local_close(handle);
}

int mcu_uart_dma_getinfo(const devfs_handle_t *handle, void *ctl) {
  return uart_local_getinfo(handle, ctl);
}

int mcu_uart_dma_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_STATE(uart);
  const stm32_uart_dma_config_t *dma_config;

  // BSP *MUST* provide DMA configuration information
  dma_config = handle->config;
  if (dma_config == 0) {
    return SYSFS_SET_RETURN(ENOSYS);
  }

  // setup the DMA for receiving
  stm32_dma_channel_t *channel = stm32_dma_setattr(&dma_config->dma_config.rx);
  if (channel != 0) {
    __HAL_LINKDMA((&state->hal_handle), hdmarx, channel->handle);
  }

  channel = stm32_dma_setattr(&dma_config->dma_config.tx);
  if (channel != 0) {
    __HAL_LINKDMA((&state->hal_handle), hdmatx, channel->handle);
  }

  int result = uart_local_setattr(handle, ctl);
  if (result < 0) {
    return result;
  }

  // initiate the DMA circular read
  if (state->fifo_config) {
    if (
      (result = HAL_UART_Receive_DMA(
         &state->hal_handle,
         (u8 *)state->fifo_config->buffer,
         state->fifo_config->size))
      != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }
  }

  return SYSFS_RETURN_SUCCESS;
}

int mcu_uart_dma_setaction(const devfs_handle_t *handle, void *ctl) {
  const stm32_uart_dma_config_t *config = handle->config;
  if (config == NULL) {
    return SYSFS_SET_RETURN(ENOSYS);
  }
  // need to pass the interrupt number
  stm32_dma_set_interrupt_priority(&config->dma_config.rx, ctl);
  stm32_dma_set_interrupt_priority(&config->dma_config.tx, ctl);
  return uart_local_setaction(handle, ctl);
}

int mcu_uart_dma_put(const devfs_handle_t *handle, void *ctl) {
  return uart_local_put(handle, ctl);
}

int mcu_uart_dma_flush(const devfs_handle_t *handle, void *ctl) {
  return uart_local_flush(handle, ctl);
}

int mcu_uart_dma_get(const devfs_handle_t *handle, void *ctl) {
  return uart_local_get(handle, ctl);
}

int mcu_uart_dma_read(const devfs_handle_t *handle, devfs_async_t *async) {
  // read circularly writes to the FIFO, user reads the FIFO
  return uart_local_read(handle, async);
}

int mcu_uart_dma_write(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_STATE(uart);
  int ret;

  // write won't be circular like read
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.write, async);

#if 0
  sos_debug_printf("w:\n");
  for (int i = 0; i < async->nbyte; i++) {
    if (i && (i % 16 == 0)) {
      sos_debug_printf("\n");
    }
    sos_debug_printf("%02X ", ((char *)async->buf)[i]);
  }
  sos_debug_printf("\n");
#endif

#if defined STM32F7 || defined STM32H7
  sos_config.cache.clean_data_block(async->buf, async->nbyte);
#endif

  if (state->hal_handle.hdmatx) {
    ret = HAL_UART_Transmit_DMA(&state->hal_handle, async->buf, async->nbyte);
  } else {
    ret = HAL_UART_Transmit_IT(&state->hal_handle, async->buf, async->nbyte);
  }
  if (ret == HAL_OK) {
    return 0;
  }

  state->transfer_handler.write = NULL;
  return SYSFS_SET_RETURN(EIO);
}

#endif
