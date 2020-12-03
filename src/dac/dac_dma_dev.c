/* Copyright 2011-2016 Tyler Gilbert;
 * This file is part of Stratify OS.
 *
 * Stratify OS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Stratify OS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Stratify OS.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */

#include "dac_local.h"
#include <fcntl.h>

#if MCU_DAC_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  dac_dma,
  DAC_VERSION,
  DAC_IOC_IDENT_CHAR,
  I_MCU_TOTAL + I_DAC_TOTAL,
  mcu_dac_dma_get,
  mcu_dac_dma_set)

int mcu_dac_dma_open(const devfs_handle_t *handle) {
  m_dac_local[handle->port].o_flags = DAC_LOCAL_FLAG_IS_DMA;
  return dac_local_open(handle);
}

int mcu_dac_dma_close(const devfs_handle_t *handle) {

  // cancel DMA operations	adc_local_t * local = adc_local + handle->port;
  dac_local_t *local = m_dac_local + handle->port;
  if (local->ref_count == 1) {
    // disable the DMA

    const stm32_adc_dma_config_t *config;
    config = handle->config;
    u32 channel;
    if (handle->port) {
      channel = DAC_CHANNEL_1;
    } else {
      channel = DAC_CHANNEL_2;
    }
    HAL_DAC_Stop_DMA(&local->hal_handle, channel);
    devfs_execute_cancel_handler(
      &local->transfer_handler,
      0,
      SYSFS_SET_RETURN(EIO),
      0);

    if (config) {
      stm32_dma_clear_handle(
        config->dma_config.dma_number,
        config->dma_config.stream_number);
    }
  }

  return dac_local_close(handle);
}

int mcu_dac_dma_getinfo(const devfs_handle_t *handle, void *ctl) {
  return dac_local_getinfo(handle, ctl);
}

int mcu_dac_dma_setattr(const devfs_handle_t *handle, void *ctl) {
  const stm32_dac_dma_config_t *config = handle->config;
  dac_local_t *local = m_dac_local + handle->port;

  if (config == 0) {
    return SYSFS_SET_RETURN(ENOSYS);
  }

  stm32_dma_channel_t *channel = stm32_dma_setattr(&config->dma_config);
  if (channel == 0) {
    return SYSFS_SET_RETURN(EIO);
  }

  if (handle->port == 0) {
    __HAL_LINKDMA((&local->hal_handle), DMA_Handle1, channel->handle);
  } else {
    __HAL_LINKDMA((&local->hal_handle), DMA_Handle2, channel->handle);
  }

  return dac_local_setattr(handle, ctl);
}

int mcu_dac_dma_setaction(const devfs_handle_t *handle, void *ctl) {
  mcu_action_t *action = (mcu_action_t *)ctl;
  int port = handle->port;
  dac_local_t *local = m_dac_local + port;

  if (action->handler.callback == 0) {
    // if there is an ongoing operation -- cancel it
    if (action->o_events & MCU_EVENT_FLAG_DATA_READY) {
      // execute the read callback if not null
      devfs_execute_read_handler(
        &local->transfer_handler,
        0,
        SYSFS_SET_RETURN(EAGAIN),
        MCU_EVENT_FLAG_CANCELED);
      HAL_DAC_Stop_DMA(&local->hal_handle, DAC_CHANNEL_1);
    }
  }

  cortexm_set_irq_priority(m_dac_irqs[port], action->prio, action->o_events);
  return 0;
}

int mcu_dac_dma_get(const devfs_handle_t *handle, void *ctl) {
  return dac_local_get(handle, ctl);
}

int mcu_dac_dma_set(const devfs_handle_t *handle, void *ctl) {
  return dac_local_set(handle, ctl);
}

int mcu_dac_dma_read(const devfs_handle_t *handle, devfs_async_t *async) {
  return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_dac_dma_write(const devfs_handle_t *handle, devfs_async_t *async) {
  int port = handle->port;
  dac_local_t *local = m_dac_local + port;
  u32 channel;

  DEVFS_DRIVER_IS_BUSY(local->transfer_handler.write, async);

  if (async->nbyte < 2) {
    local->transfer_handler.write = 0;
    return SYSFS_SET_RETURN(EINVAL);
  }

  async->nbyte &= ~0x01; // align to 2 byte boundary
  if ((port < MCU_DAC_PORTS)) {
    channel = m_dac_channels[port];
  } else {
    local->transfer_handler.write = 0;
    return SYSFS_SET_RETURN(ENOSYS);
  }

  sos_debug_log_info(
    SOS_DEBUG_DEVICE,
    "DAC DMA write %d words on channel 0x%X",
#if defined STM32H7
    async->nbyte / 4,
#else
    async->nbyte / 2,
#endif
    channel);

  mcu_core_clean_data_cache_block(async->buf, async->nbyte);

  if (
    HAL_DAC_Start_DMA(
      &local->hal_handle,
      channel,
      async->buf,
#if defined STM32H7
      async->nbyte / 4,
#else
      async->nbyte / 2,
#endif
      dac_local_get_alignment(local))
    == HAL_OK) {
    // sos_debug_root_printf("wait DMA\n");
    return 0;
  }

  // this needs to read 1 byte at a time
  local->transfer_handler.write = 0;
  return SYSFS_SET_RETURN(EIO);
}

#endif
