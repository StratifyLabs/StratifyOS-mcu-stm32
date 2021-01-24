// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include "mmc_local.h"
#include <mcu/mmc.h>

// STM32H7 has built in DMA inside the MMC -- DMA is automatic
#if MCU_SDIO_PORTS > 0 && defined MMC_TypeDef

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  mmc_dma,
  MMC_VERSION,
  MMC_IOC_IDENT_CHAR,
  I_MCU_TOTAL + I_SDIO_TOTAL,
  mcu_mmc_dma_getcid,
  mcu_mmc_dma_getcsd,
  mcu_mmc_dma_getstatus)

static int configure_dma(const devfs_handle_t *handle);

int mcu_mmc_dma_open(const devfs_handle_t *handle) {
  // mmc_state_list[config->port].o_flags = MMC_LOCAL_IS_DMA;
  return mmc_local_open(handle);
}

int mcu_mmc_dma_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(mmc);

  if (state->ref_count == 1) {
    // disable the DMA
    if (state->transfer_handler.read || state->transfer_handler.write) {
      return SYSFS_SET_RETURN(EBUSY);
    }

    const stm32_mmc_dma_config_t *config;
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

  // do the opposite of mcu_mmc_dma_open() -- ref_count is zero -- turn off
  // interrupt
  return mmc_local_close(handle);
}

int mcu_mmc_dma_getinfo(const devfs_handle_t *handle, void *ctl) {
  return mmc_local_getinfo(handle, ctl);
}

int configure_dma(const devfs_handle_t *handle) {

#if MCU_SDIO_API == 0
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(mmc);

  const stm32_mmc_dma_config_t *dma_config = handle->config;
  if (dma_config == 0) {
    return SYSFS_SET_RETURN(ENOSYS);
  }

  stm32_dma_channel_t *rx_channel
    = stm32_dma_setattr(&dma_config->dma_config.rx);
  if (rx_channel == 0) {
    return SYSFS_SET_RETURN(EIO);
  }

  stm32_dma_channel_t *tx_channel
    = stm32_dma_setattr(&dma_config->dma_config.tx);
  if (tx_channel == 0) {
    return SYSFS_SET_RETURN(EIO);
  }

  __HAL_LINKDMA((&state->hal_handle), hdmatx, tx_channel->handle);
  __HAL_LINKDMA((&state->hal_handle), hdmarx, rx_channel->handle);
#endif

  return 0;
}

int mcu_mmc_dma_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(mmc);
  const mmc_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(mmc, ctl);
  if (attr == NULL) {
    return SYSFS_SET_RETURN(ENOSYS);
  }

  u32 o_flags = attr->o_flags;

  if (o_flags & MMC_FLAG_SET_INTERFACE) {

    state->o_flags |= MMC_LOCAL_FLAG_IS_DMA;

    int result = configure_dma(handle);
    if (result < 0) {
      return result;
    }
  }

  return mmc_local_setattr(handle, ctl);
}

int mcu_mmc_dma_setaction(const devfs_handle_t *handle, void *ctl) {
  return mmc_local_setaction(handle, ctl);
}

int mcu_mmc_dma_getcid(const devfs_handle_t *handle, void *ctl) {
  return mmc_local_getcid(handle, ctl);
}

int mcu_mmc_dma_getcsd(const devfs_handle_t *handle, void *ctl) {
  return mmc_local_getcsd(handle, ctl);
}

int mcu_mmc_dma_getstatus(const devfs_handle_t *handle, void *ctl) {
  return mmc_local_getstatus(handle, ctl);
}

int mcu_mmc_dma_write(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(mmc);
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.write, async);

  int result;
  state->hal_handle.ErrorCode = 0;
  int loc;
  if (state->o_flags & MMC_LOCAL_FLAG_IS_BYTE_ADDRESSING) {
    loc = async->loc * 512;
  } else {
    loc = async->loc;
  }

#if MCU_SDIO_API == 0
  if (state->hal_handle.hdmatx == 0) {
    // driver unlinked the DMA object -- relink it here
    configure_dma(handle);
  }
#endif

  state->hal_handle.TxXferSize
    = async
        ->nbyte; // used by the callback but not set by HAL_SD_WriteBlocks_DMA
  if (
    (result = HAL_MMC_WriteBlocks_DMA(
       &state->hal_handle,
       async->buf,
       loc,
       async->nbyte / BLOCKSIZE))
    == HAL_OK) {
    // check the state to make sure the transfer is underway
    return 0;
  }

  state->transfer_handler.write = 0;
  sos_debug_log_error(
    SOS_DEBUG_DEVICE,
    "W->HAL Not OK  %d, 0x%lX (%d + %d > %d?)",
    result,
    state->hal_handle.ErrorCode,
    async->loc,
    async->nbyte / BLOCKSIZE,
    state->hal_handle.MmcCard.LogBlockNbr);
  return SYSFS_SET_RETURN(EIO);
}

int mcu_mmc_dma_read(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(mmc);
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.read, async);

  int hal_result;

  state->hal_handle.RxXferSize
    = async->nbyte; // used by the callback but not set by HAL_SD_ReadBlocks_DMA
  int loc;
  if (state->o_flags & MMC_LOCAL_FLAG_IS_BYTE_ADDRESSING) {
    loc = async->loc * 512;
  } else {
    loc = async->loc;
  }

#if MCU_SDIO_API == 0
  if (state->hal_handle.hdmarx == 0) {
    // driver unlinked the DMA object -- relink it here
    configure_dma(handle);
  }
#endif

  if (
    (hal_result = HAL_MMC_ReadBlocks_DMA(
       &state->hal_handle,
       async->buf,
       loc,
       async->nbyte / BLOCKSIZE))
    == HAL_OK) {
    return 0;
  }

  sos_debug_log_warning(
    SOS_DEBUG_DEVICE,
    "R->HAL Not OK %d %d %d 0x%lX",
    async->loc,
    async->nbyte,
    hal_result,
    state->hal_handle.ErrorCode);

  state->transfer_handler.read = 0;
  return SYSFS_SET_RETURN(EIO);
}

#endif
