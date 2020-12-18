/* Copyright 2011-2021 Tyler Gilbert;
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

#include <mcu/spi.h>
#include <sos/config.h>

#include "spi_local.h"
#include "stm32_dma.h"

#if MCU_SPI_PORTS > 0

#define DMA_INTERRUPT_NUMBER 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  spi_dma,
  SPI_VERSION,
  SPI_IOC_IDENT_CHAR,
  I_MCU_TOTAL + I_SPI_TOTAL,
  mcu_spi_dma_swap)

int mcu_spi_dma_open(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);
  state->o_flags = SPI_LOCAL_IS_DMA;
  return spi_local_open(handle);
}

int mcu_spi_dma_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);

  if (state->ref_count == 1) {
    // disable the DMA
    const stm32_spi_dma_config_t *dma_config;

    if (state->transfer_handler.read || state->transfer_handler.write) {
      HAL_SPI_DMAStop(&state->hal_handle);
      devfs_execute_cancel_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(EIO),
        0);
    }

    dma_config = handle->config;
    if (config) {
      stm32_dma_clear_handle(
        dma_config->dma_config.rx.dma_number,
        dma_config->dma_config.rx.stream_number);
      stm32_dma_clear_handle(
        dma_config->dma_config.tx.dma_number,
        dma_config->dma_config.tx.stream_number);
    }
  }

  return spi_local_close(handle);
}

int mcu_spi_dma_getinfo(const devfs_handle_t *handle, void *ctl) {
  spi_info_t *info = ctl;

  // set flags
  info->o_flags = SPI_FLAG_SET_MASTER | SPI_FLAG_IS_MODE0 | SPI_FLAG_IS_MODE1
                  | SPI_FLAG_IS_MODE2 | SPI_FLAG_IS_MODE3
                  | SPI_FLAG_IS_FORMAT_SPI | SPI_FLAG_IS_FORMAT_TI
                  | SPI_FLAG_SET_FULL_DUPLEX | SPI_FLAG_SET_HALF_DUPLEX;

  info->o_events = MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_WRITE_COMPLETE
                   | MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR;

  return 0;
}

int mcu_spi_dma_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);
  const stm32_spi_dma_config_t *dma_config;

  // BSP *MUST* provide DMA configuration information
  dma_config = handle->config;
  if (dma_config == NULL) {
    return SYSFS_SET_RETURN(ENOSYS);
  }

  const spi_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(spi, ctl);
  if (attr == NULL) {
    return SYSFS_SET_RETURN(EINVAL);
  }

  if (attr->o_flags & (SPI_FLAG_SET_MASTER | SPI_FLAG_SET_SLAVE)) {
    // setup the DMA for receiving
    stm32_dma_channel_t *channel
      = stm32_dma_setattr(&dma_config->dma_config.rx);
    if (channel == 0) {
      return SYSFS_SET_RETURN(EIO);
    }

    __HAL_LINKDMA((&state->hal_handle), hdmarx, channel->handle);

    channel = stm32_dma_setattr(&dma_config->dma_config.tx);
    if (channel == 0) {
      return SYSFS_SET_RETURN(EIO);
    }

    __HAL_LINKDMA((&state->hal_handle), hdmatx, channel->handle);
  }

  return spi_local_setattr(handle, ctl);
}

int mcu_spi_dma_swap(const devfs_handle_t *handle, void *ctl) {
  return spi_local_swap(handle, ctl);
}

int mcu_spi_dma_setaction(const devfs_handle_t *handle, void *ctl) {
  const stm32_spi_dma_config_t *config = handle->config;
  if (config == 0) {
    return SYSFS_SET_RETURN(ENOSYS);
  }
  // need to pass the interrupt number
  stm32_dma_set_interrupt_priority(&config->dma_config.rx, ctl);
  stm32_dma_set_interrupt_priority(&config->dma_config.tx, ctl);

  return spi_local_setaction(handle, ctl);
}

int mcu_spi_dma_write(const devfs_handle_t *handle, devfs_async_t *async) {
  int ret;
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.write, async);

#if defined STM32F7 || defined STM32H7
  sos_config.cache.clean_data_block(async->buf, async->nbyte);
#endif

  if (
    (state->o_flags & SPI_LOCAL_IS_FULL_DUPLEX)
    && state->transfer_handler.read) {

    if (state->transfer_handler.read->nbyte < async->nbyte) {
      return SYSFS_SET_RETURN(EINVAL);
    }

    ret = HAL_SPI_TransmitReceive_DMA(
      &state->hal_handle,
      async->buf,
      state->transfer_handler.read->buf,
      async->nbyte);

  } else {
    ret = HAL_SPI_Transmit_DMA(&state->hal_handle, async->buf, async->nbyte);
  }

  if (ret != HAL_OK) {
    state->transfer_handler.write = 0;
    return SYSFS_SET_RETURN_WITH_VALUE(EIO, ret);
  }

  return 0;
}

int mcu_spi_dma_read(const devfs_handle_t *handle, devfs_async_t *async) {
  int ret;
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.read, async);

  if (state->o_flags & SPI_LOCAL_IS_FULL_DUPLEX) {
    // just set up the buffer
    ret = HAL_OK;
  } else {
    ret = HAL_SPI_Receive_DMA(&state->hal_handle, async->buf, async->nbyte);
  }

  if (ret != HAL_OK) {
    state->transfer_handler.read = 0;
    return SYSFS_SET_RETURN_WITH_VALUE(EIO, ret);
  }

  return 0;
}

#endif
