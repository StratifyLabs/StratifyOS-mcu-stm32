// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include "i2s_spi_local.h"
#include <mcu/i2s.h>
#include <mcu/spi.h>

#if defined STM32F4
#include "stm32f4xx/stm32f4xx_hal_i2s_ex.h"
#endif

#if MCU_I2S_SPI_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  i2s_spi_dma,
  I2S_VERSION,
  I2S_IOC_IDENT_CHAR,
  I_MCU_TOTAL + I_I2S_TOTAL,
  mcu_i2s_spi_dma_mute,
  mcu_i2s_spi_dma_unmute)

#if defined STM32H7
extern HAL_StatusTypeDef HAL_I2S_Transmit_DMA_DelayedStart(
  I2S_HandleTypeDef *hi2s,
  uint16_t *pData,
  uint16_t Size);
#endif

int mcu_i2s_spi_dma_open(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);
  state->o_flags = SPI_LOCAL_IS_DMA;
  return i2s_spi_local_open(handle);
}

int mcu_i2s_spi_dma_close(const devfs_handle_t *handle) {
  // same as SPI
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);

  if (state->ref_count == 1) {
    // disable the DMA
    const stm32_i2s_spi_dma_config_t *config;

    if (state->transfer_handler.read || state->transfer_handler.write) {
      HAL_I2S_DMAStop(&state->i2s_hal_handle);
      devfs_execute_cancel_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(EIO),
        0);
      state->transfer_handler.read = 0;
      state->transfer_handler.write = 0;
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

  return i2s_spi_local_close(handle);
}

int mcu_i2s_spi_dma_getinfo(const devfs_handle_t *handle, void *ctl) {
  i2s_info_t *info = ctl;

  // set I2S Capability flags
  info->o_flags = I2S_FLAG_SET_MASTER | I2S_FLAG_SET_SLAVE | I2S_FLAG_IS_WIDTH_8
                  | I2S_FLAG_IS_WIDTH_16 | I2S_FLAG_IS_WIDTH_16_EXTENDED
                  | I2S_FLAG_IS_WIDTH_24 | I2S_FLAG_IS_WIDTH_32;

  info->o_events = MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_WRITE_COMPLETE
                   | MCU_EVENT_FLAG_HIGH | MCU_EVENT_FLAG_LOW;

  return 0;
}

int mcu_i2s_spi_dma_mute(const devfs_handle_t *handle, void *ctl) {
  return i2s_spi_local_mute(handle, ctl);
}

int mcu_i2s_spi_dma_unmute(const devfs_handle_t *handle, void *ctl) {
  return i2s_spi_local_unmute(handle, ctl);
}

int mcu_i2s_spi_dma_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);
  const stm32_i2s_spi_dma_config_t *dma_config;
  const i2s_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(i2s, ctl);
  if (attr == 0) {
    return SYSFS_SET_RETURN(ENOSYS);
  }

  // setup the DMA
  // BSP *MUST* provide DMA configuration information
  dma_config = handle->config;
  if (config == NULL) {
    return SYSFS_SET_RETURN(ENOSYS);
  }

  if (attr->o_flags & (I2S_FLAG_SET_MASTER | I2S_FLAG_SET_SLAVE)) {

    if (attr->o_flags & I2S_FLAG_IS_RECEIVER) {

      sos_debug_log_info(
        SOS_DEBUG_DEVICE,
        "Set I2S DMA as receiver %d.%d.%d",
        dma_config->dma_config.rx.dma_number,
        dma_config->dma_config.rx.stream_number,
        dma_config->dma_config.rx.channel_number);

      stm32_dma_channel_t *channel
        = stm32_dma_setattr(&dma_config->dma_config.rx);
      if (channel == 0) {
        return SYSFS_SET_RETURN(EIO);
      }

      __HAL_LINKDMA((&state->i2s_hal_handle), hdmarx, channel->handle);
    }

    if (attr->o_flags & I2S_FLAG_IS_TRANSMITTER) {

      sos_debug_log_info(
        SOS_DEBUG_DEVICE,
        "Set I2S DMA as transmitter %d.%d.%d",
        dma_config->dma_config.tx.dma_number,
        dma_config->dma_config.tx.stream_number,
        dma_config->dma_config.tx.channel_number);

      stm32_dma_channel_t *channel
        = stm32_dma_setattr(&dma_config->dma_config.tx);

      if (channel == 0) {
        return SYSFS_SET_RETURN(EIO);
      }

      __HAL_LINKDMA((&state->i2s_hal_handle), hdmatx, channel->handle);
    }
  }

  return i2s_spi_local_setattr(handle, ctl);
}

int mcu_i2s_spi_dma_setaction(const devfs_handle_t *handle, void *ctl) {
  return mcu_spi_dma_setaction(handle, ctl);
}

int mcu_i2s_spi_dma_write(const devfs_handle_t *handle, devfs_async_t *async) {
  int result;
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.write, async);

  if (
    (state->o_flags & SPI_LOCAL_IS_FULL_DUPLEX)
    && state->transfer_handler.read) {

    if (state->transfer_handler.read->nbyte < async->nbyte) {
      return SYSFS_SET_RETURN(EINVAL);
    }

    i2s_spi_local_wait_for_errata_level(state);

#if defined SPI_I2S_FULLDUPLEX_SUPPORT
    result = HAL_I2SEx_TransmitReceive_DMA(
      &state->i2s_hal_handle,
      async->buf,
      state->transfer_handler.read->buf,
      async->nbyte);
#elif defined STM32H7

    result = HAL_I2S_Transmit_DMA_DelayedStart(
      &state->i2s_hal_handle,
      async->buf,
      (async->nbyte / state->size_mult));

    if (result != HAL_OK) {
      return SYSFS_SET_RETURN(ENOTSUP);
    }

    state->i2s_hal_handle.State = HAL_I2S_STATE_READY;
    result = HAL_I2S_Receive_DMA(
      &state->i2s_hal_handle,
      state->transfer_handler.read->buf,
      (async->nbyte / state->size_mult));

#else
    return SYSFS_SET_RETURN(ENOTSUP);
#endif

  } else {
    sos_debug_log_info(
      SOS_DEBUG_DEVICE,
      "Write->I2S DMA 0x%lX %p %d %d 0x%lX",
      state->i2s_hal_handle.Init.Mode,
      async->buf,
      async->nbyte,
      state->size_mult,
      state->o_flags);

    i2s_spi_local_wait_for_errata_level(state);

    result = HAL_I2S_Transmit_DMA(
      &state->i2s_hal_handle,
      async->buf,
      (async->nbyte / state->size_mult));
  }

  if (result != HAL_OK) {
    state->transfer_handler.write = 0;
    if (result == HAL_BUSY) {
      return SYSFS_SET_RETURN(EIO);
    } else if (result == HAL_ERROR) {
      return SYSFS_SET_RETURN(EIO);
    } else if (result == HAL_TIMEOUT) {
      return SYSFS_SET_RETURN(EIO);
    }
    return SYSFS_SET_RETURN(EIO);
  }

  return 0;
}

int mcu_i2s_spi_dma_read(const devfs_handle_t *handle, devfs_async_t *async) {
  int ret;
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.read, async);

  if (state->o_flags & SPI_LOCAL_IS_FULL_DUPLEX) {
    // Receive assigns the transfer handler but then blocks until a write
    // happens
    return 0;
  }

  i2s_spi_local_wait_for_errata_level(state);
  ret = HAL_I2S_Receive_DMA(
    &state->i2s_hal_handle,
    async->buf,
    (async->nbyte / state->size_mult));

  if (ret != HAL_OK) {
    sos_debug_log_error(
      SOS_DEBUG_DEVICE,
      "Failed to start I2S DMA Read (%d, %d) %d/%d",
      ret,
      state->i2s_hal_handle.ErrorCode,
      async->nbyte,
      state->size_mult);
    state->transfer_handler.read = 0;
    if (ret == HAL_BUSY) {
      return SYSFS_SET_RETURN(EIO);
    } else if (ret == HAL_ERROR) {
      return SYSFS_SET_RETURN(EIO);
    } else if (ret == HAL_TIMEOUT) {
      return SYSFS_SET_RETURN(EIO);
    } else if (ret != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }
  }

  return 0;
}

#endif
