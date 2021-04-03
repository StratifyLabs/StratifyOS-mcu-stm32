// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include <cortexm/cortexm.h>
#include <fcntl.h>
#include <mcu/core.h>
#include <mcu/dac.h>
#include <mcu/qspi.h>
#include <sos/config.h>
#include <sos/debug.h>

#include "stm32_local.h"

#if MCU_QSPI_PORTS > 0

#include "qspi_local.h"

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  qspi_dma,
  QSPI_VERSION,
  QSPI_IOC_IDENT_CHAR,
  I_QSPI_TOTAL + I_MCU_TOTAL,
  mcu_qspi_dma_execcommand)

int mcu_qspi_dma_open(const devfs_handle_t *handle) {
  return qspi_local_open(handle);
}

int mcu_qspi_dma_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(qspi);

  if (state->ref_count == 1) {
    // disable the DMA
    const stm32_qspi_dma_config_t *config;

    if (state->transfer_handler.read || state->transfer_handler.write) {
      // HAL_QSPI_DMAStop(&state->hal_handle);
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

  return qspi_local_close(handle);
}

int mcu_qspi_dma_getinfo(const devfs_handle_t *handle, void *ctl) {
  qspi_info_t *info = ctl;
  info->o_flags = QSPI_FLAG_SET_MASTER;
  return 0;
}

int mcu_qspi_dma_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(qspi);

  if (handle->config == 0) {
    return SYSFS_SET_RETURN(ENOSYS);
  }


  // setup the DMA for receiving
#if MCU_QSPI_API == 1

  __HAL_RCC_MDMA_CLK_ENABLE();

  cortexm_enable_irq(MDMA_IRQn);

  // API 1 uses the MDMA which has a fixed configuration
  stm32_dma_mdma_handle.Instance = MDMA_Channel0;
  stm32_dma_mdma_handle.Init.Request = MDMA_REQUEST_QUADSPI_FIFO_TH;
  stm32_dma_mdma_handle.Init.TransferTriggerMode = MDMA_BUFFER_TRANSFER;
  stm32_dma_mdma_handle.Init.Priority = MDMA_PRIORITY_LOW;
  stm32_dma_mdma_handle.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  stm32_dma_mdma_handle.Init.SourceInc = MDMA_SRC_INC_BYTE;
  stm32_dma_mdma_handle.Init.DestinationInc = MDMA_DEST_INC_DISABLE;
  stm32_dma_mdma_handle.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  stm32_dma_mdma_handle.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  stm32_dma_mdma_handle.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  stm32_dma_mdma_handle.Init.BufferTransferLength = 1;
  stm32_dma_mdma_handle.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  stm32_dma_mdma_handle.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  stm32_dma_mdma_handle.Init.SourceBlockAddressOffset = 0;
  stm32_dma_mdma_handle.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&stm32_dma_mdma_handle) != HAL_OK) {
    return SYSFS_SET_RETURN(EIO);
  }

  __HAL_LINKDMA(&state->hal_handle, hmdma, stm32_dma_mdma_handle);
#else
  const stm32_spi_dma_config_t *dma_config = handle->config;
  stm32_dma_channel_t *channel = stm32_dma_setattr(&dma_config->dma_config.rx);
  if (channel == 0) {
    return SYSFS_SET_RETURN(EIO);
  }
  __HAL_LINKDMA((&state->hal_handle), hdma, channel->handle);

  channel = stm32_dma_setattr(&dma_config->dma_config.tx);
  if (channel == 0) {
    return SYSFS_SET_RETURN(EIO);
  }

  __HAL_LINKDMA((&state->hal_handle), hdma, channel->handle);
#endif

  return qspi_local_setattr(handle, ctl);
}

int mcu_qspi_dma_setaction(const devfs_handle_t *handle, void *ctl) {
  return qspi_local_setaction(handle, ctl);
}

int mcu_qspi_dma_execcommand(const devfs_handle_t *handle, void *ctl) {
  return qspi_local_execcommand(handle, ctl);
}

int mcu_qspi_dma_read(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(qspi);

#if defined STM32F7 || defined STM32H7 && 0
  // doing an unaligned read will cause problems
  if( (u32)async->buf & 0x1f ){
    return SYSFS_SET_RETURN(EINVAL);
  }

  if( async->nbyte & 0x1f ){
    return SYSFS_SET_RETURN(EINVAL);
  }
#endif

  // can't read and write at the same time
  if (state->transfer_handler.write != 0) {
    return SYSFS_SET_RETURN(EBUSY);
  }
  // borrow async to qspi->transfer_handler.read
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.read, async);


  if (HAL_QSPI_Receive_DMA(&state->hal_handle, async->buf) != HAL_OK) {
    state->transfer_handler.read = 0;
    return SYSFS_SET_RETURN(EIO);
  }
  return 0;
}

int mcu_qspi_dma_write(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(qspi);

  // can't read and write at the same time
  if (state->transfer_handler.read != 0) {
    return SYSFS_SET_RETURN(EBUSY);
  }
  // borrow async to qspi->transfer_handler.read
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.write, async);

#if defined STM32F7 || defined STM32H7
  // ensure data in cache is written to memory before writing
  sos_config.cache.clean_data_block(async->buf, async->nbyte);
#endif

  if (HAL_QSPI_Transmit_DMA(&state->hal_handle, async->buf) != HAL_OK) {
    state->transfer_handler.write = 0;
    return SYSFS_SET_RETURN(EIO);
  }

  return 0;
}

#endif
