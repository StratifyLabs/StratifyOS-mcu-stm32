/* Copyright 2011-2018 Tyler Gilbert;
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

#include <mcu/sdio.h>
#include <sos/config.h>

#include "sdio_local.h"

#if MCU_SDIO_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  sdio_dma,
  SDIO_VERSION,
  SDIO_IOC_IDENT_CHAR,
  I_MCU_TOTAL + I_SDIO_TOTAL,
  mcu_sdio_dma_getcid,
  mcu_sdio_dma_getcsd,
  mcu_sdio_dma_getstatus)

int mcu_sdio_dma_open(const devfs_handle_t *handle) {
  return sdio_local_open(handle);
}

int mcu_sdio_dma_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sdio);
  // do the opposite of mcu_sdio_dma_open() -- ref_count is zero -- turn off
  // interrupt

  if (state->ref_count == 1) {
    // disable the DMA
    if (state->transfer_handler.read || state->transfer_handler.write) {
      return SYSFS_SET_RETURN(EBUSY);
    }

    const stm32_i2s_spi_dma_config_t *dma_config = handle->config;

    devfs_execute_cancel_handler(
      &state->transfer_handler,
      0,
      SYSFS_SET_RETURN(EIO),
      0);

    if (config) {
      stm32_dma_clear_handle(
        dma_config->dma_config.rx.dma_number,
        dma_config->dma_config.rx.stream_number);
      stm32_dma_clear_handle(
        dma_config->dma_config.tx.dma_number,
        dma_config->dma_config.tx.stream_number);
    }
  }

  return sdio_local_close(handle);
}

int mcu_sdio_dma_getinfo(const devfs_handle_t *handle, void *ctl) {
  return sdio_local_getinfo(handle, ctl);
}

int mcu_sdio_dma_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sdio);

  int result;
  const sdio_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(sdio, ctl);
  if (attr == NULL) {
    return SYSFS_SET_RETURN(ENOSYS);
  }

  u32 o_flags = attr->o_flags;

#if MCU_SDIO_API == 0 // API 1 has built-in DMA
  if (o_flags & SDIO_FLAG_SET_INTERFACE) {

    const stm32_sdio_dma_config_t *dma_config = handle->config;
    if (dma_config == 0) {
      return SYSFS_SET_RETURN(ENOSYS);
    }

    stm32_dma_channel_t *tx_channel
      = stm32_dma_setattr(&dma_config->dma_config.tx);
    if (tx_channel == 0) {
      return SYSFS_SET_RETURN(EIO);
    }

    stm32_dma_channel_t *rx_channel
      = stm32_dma_setattr(&dma_config->dma_config.rx);
    if (rx_channel == 0) {
      return SYSFS_SET_RETURN(EIO);
    }

    __HAL_LINKDMA((&state->hal_handle), hdmatx, tx_channel->handle);
    __HAL_LINKDMA((&state->hal_handle), hdmarx, rx_channel->handle);
  }
#endif

  result = sdio_local_setattr(handle, ctl);
  if ((result < 0) || (o_flags & SDIO_FLAG_GET_CARD_STATE)) {
    return result;
  }

  return SYSFS_RETURN_SUCCESS;
}

int mcu_sdio_dma_setaction(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sdio);
  mcu_action_t *action = ctl;

  if (action->handler.callback == 0) {
    if (action->o_events & MCU_EVENT_FLAG_DATA_READY) {
      // HAL_SD_Abort_IT(&sdio_state_list[config->port].hal_handle);
      devfs_execute_read_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(EIO),
        MCU_EVENT_FLAG_CANCELED);
    }

    if (action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE) {
      // HAL_SD_Abort_IT(&sdio_state_list[config->port].hal_handle);
      devfs_execute_write_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(EIO),
        MCU_EVENT_FLAG_CANCELED);
    }
  }

  cortexm_set_irq_priority(sdio_irqs[config->port], action->prio, action->o_events);
  return 0;
}

int mcu_sdio_dma_getcid(const devfs_handle_t *handle, void *ctl) {
  return sdio_local_getcid(handle, ctl);
}

int mcu_sdio_dma_getcsd(const devfs_handle_t *handle, void *ctl) {
  return sdio_local_getcsd(handle, ctl);
}

int mcu_sdio_dma_getstatus(const devfs_handle_t *handle, void *ctl) {
  return sdio_local_getstatus(handle, ctl);
}

int mcu_sdio_dma_write(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sdio);
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.write, async);

#if defined STM32F7 || defined STM32H7
  sos_config.cache.clean_data_block(async->buf, async->nbyte);
#endif

  state->hal_handle.TxXferSize
    = async
        ->nbyte; // used by the callback but not set by HAL_SD_WriteBlocks_DMA
  if (
    (HAL_SD_WriteBlocks_DMA(
      &state->hal_handle,
      async->buf,
      async->loc,
      async->nbyte / BLOCKSIZE))
    == HAL_OK) {
    return 0;
  }

  state->transfer_handler.write = 0;
  return SYSFS_SET_RETURN(EIO);
}

int mcu_sdio_dma_read(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sdio);
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.read, async);

  state->hal_handle.RxXferSize
    = async->nbyte; // used by the callback but not set by HAL_SD_ReadBlocks_DMA
  int hal_result;
  if (
    (hal_result = HAL_SD_ReadBlocks_DMA(
       &state->hal_handle,
       async->buf,
       async->loc,
       async->nbyte / BLOCKSIZE))
    == HAL_OK) {
    return 0;
  }

  state->transfer_handler.read = 0;
  return SYSFS_SET_RETURN(EIO);
}

#endif
