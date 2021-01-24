// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include "sai_local.h"
#include <mcu/i2s.h>

#if defined STM32F4
#include "stm32f4xx/stm32f4xx_hal_i2s_ex.h"
#endif

#if MCU_SAI_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  sai_dma,
  I2S_VERSION,
  I2S_IOC_IDENT_CHAR,
  I_MCU_TOTAL + I_I2S_TOTAL,
  mcu_sai_dma_mute,
  mcu_sai_dma_unmute)


int mcu_sai_dma_open(const devfs_handle_t *handle) {
  return sai_local_open(handle);
}

int mcu_sai_dma_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sai);

  if (state->ref_count == 1) {
    // disable the DMA
    const stm32_sai_dma_config_t *config;
    config = handle->config;
    HAL_SAI_DMAStop(&state->hal_handle);
    if((state->o_flags & SAI_LOCAL_IS_DMA )/*||
					  (sai_dma_state_list[config->port].dma_channel.interrupt_number > 0 )*/){
      // HAL_DMA_DeInit(&state->sai);
      state->o_flags = 0;
    }

    if (config) {
      stm32_dma_clear_handle(
        config->dma_config.dma_number,
        config->dma_config.stream_number);
    }
  }

  sai_local_close(handle);

  return 0;
}

int mcu_sai_dma_getinfo(const devfs_handle_t *handle, void *ctl) {
  i2s_info_t *info = ctl;

  // set I2S Capability flags
  info->o_flags = I2S_FLAG_SET_MASTER | I2S_FLAG_SET_SLAVE | I2S_FLAG_IS_WIDTH_8
                  | I2S_FLAG_IS_WIDTH_16 | I2S_FLAG_IS_WIDTH_16_EXTENDED
                  | I2S_FLAG_IS_WIDTH_24 | I2S_FLAG_IS_WIDTH_32;

  info->o_events = MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_WRITE_COMPLETE
                   | MCU_EVENT_FLAG_HIGH | MCU_EVENT_FLAG_LOW;

  return 0;
}

int mcu_sai_dma_mute(const devfs_handle_t *handle, void *ctl) {
  return sai_local_mute(
    handle,
    ctl);
}

int mcu_sai_dma_unmute(const devfs_handle_t *handle, void *ctl) {
  return sai_local_unmute(
    handle,
    ctl);
}

int mcu_sai_dma_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sai);
  const i2s_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(i2s, ctl);
  if (attr == 0) {
    return SYSFS_SET_RETURN(ENOSYS);
  }
  // setup the DMA
  // BSP *MUST* provide DMA configuration information
  if (handle->config == 0) {
    return SYSFS_SET_RETURN(ENOSYS);
  }

  const stm32_sai_dma_config_t *dma_config = handle->config;

  state->o_flags = SAI_LOCAL_IS_DMA;
  stm32_dma_channel_t *channel = stm32_dma_setattr(&dma_config->dma_config);
  if (channel == 0) {
    return SYSFS_SET_RETURN(EIO);
  }
  sai_dma_state_list[config->port].dma_channel.handle = channel->handle;
  if (attr->o_flags & I2S_FLAG_IS_RECEIVER) {
    state->o_flags |= SAI_LOCAL_IS_RX;
    sos_debug_log_info(
      SOS_DEBUG_DEVICE,
      "Set I2S DMA as receiver %d.%d.%d",
      dma_config->dma_config.dma_number,
      dma_config->dma_config.stream_number,
      dma_config->dma_config.channel_number);

    __HAL_LINKDMA((&state->hal_handle), hdmarx, channel->handle);
  }

  if (attr->o_flags & I2S_FLAG_IS_TRANSMITTER) {
    state->o_flags |= SAI_LOCAL_IS_TX;

    sos_debug_log_info(
      SOS_DEBUG_DEVICE,
      "Set I2S DMA as transmitter %d.%d.%d",
      dma_config->dma_config.dma_number,
      dma_config->dma_config.stream_number,
      dma_config->dma_config.channel_number);
    // setup the DMA for transmitting

    __HAL_LINKDMA((&state->hal_handle), hdmatx, channel->handle);
  }

  return sai_local_setattr(handle, ctl);
}

int mcu_sai_dma_setaction(const devfs_handle_t *handle, void *ctl) {
  return sai_local_setaction(handle, ctl, 9);
}

int mcu_sai_dma_write(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sai);
  int result;
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.write, async);

  result = HAL_SAI_Transmit_DMA(
    &state->hal_handle,
    async->buf,
    async->nbyte / state->size_mult);

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

int mcu_sai_dma_read(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sai);
  int ret;
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.read, async);
  sos_debug_log_info(
    SOS_DEBUG_DEVICE,
    "SAI DMA RX: %p %ld %d ",
    async->buf,
    async->nbyte,
    state->size_mult);

  ret = HAL_SAI_Receive_DMA(
    &state->hal_handle,
    async->buf,
    async->nbyte / state->size_mult);

  if (ret != HAL_OK) {
    sos_debug_log_error(
      SOS_DEBUG_DEVICE,
      "Failed to start I2S DMA Read (%d, %d) %d/%d",
      ret,
      state->hal_handle.ErrorCode,
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
