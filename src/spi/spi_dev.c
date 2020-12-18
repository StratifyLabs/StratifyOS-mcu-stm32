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

#include "spi_local.h"
#include <mcu/spi.h>

#if MCU_SPI_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  spi,
  SPI_VERSION,
  SPI_IOC_IDENT_CHAR,
  I_MCU_TOTAL + I_SPI_TOTAL,
  mcu_spi_swap)

int mcu_spi_open(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);
  state->o_flags = 0;
  return spi_local_open(handle);
}

int mcu_spi_close(const devfs_handle_t *handle) {
  return spi_local_close(handle);
}

int mcu_spi_getinfo(const devfs_handle_t *handle, void *ctl) {
  spi_info_t *info = ctl;

  // set flags
  info->o_flags = SPI_FLAG_SET_MASTER | SPI_FLAG_IS_MODE0 | SPI_FLAG_IS_MODE1
                  | SPI_FLAG_IS_MODE2 | SPI_FLAG_IS_MODE3
                  | SPI_FLAG_IS_FORMAT_SPI | SPI_FLAG_IS_FORMAT_TI
                  | SPI_FLAG_SET_FULL_DUPLEX | SPI_FLAG_SET_HALF_DUPLEX;

  return 0;
}

int mcu_spi_setattr(const devfs_handle_t *handle, void *ctl) {
  return spi_local_setattr(handle, ctl);
}

int mcu_spi_swap(const devfs_handle_t *handle, void *ctl) {
  return spi_local_swap(handle, ctl);
}

int mcu_spi_setaction(const devfs_handle_t *handle, void *ctl) {
  return spi_local_setaction(handle, ctl);
}

int mcu_spi_write(const devfs_handle_t *handle, devfs_async_t *async) {
  int ret;
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);

  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.write, async);

  if (
    (state->o_flags & SPI_LOCAL_IS_FULL_DUPLEX)
    && state->transfer_handler.read) {

    if (state->transfer_handler.read->nbyte < async->nbyte) {
      sos_debug_log_info(
        SOS_DEBUG_DEVICE,
        "Read bytes error %d < %d",
        state->transfer_handler.read->nbyte,
        async->nbyte);
      return SYSFS_SET_RETURN(EINVAL);
    }

    // execute the TX/RX transfer
    ret = HAL_SPI_TransmitReceive_IT(
      &state->hal_handle,
      async->buf,
      state->transfer_handler.read->buf,
      async->nbyte);
  } else {
    // this should be busy if a read is in progress
    if (state->transfer_handler.read == 0) {
      ret = HAL_SPI_Transmit_IT(&state->hal_handle, async->buf, async->nbyte);
    } else {
      sos_debug_log_error(SOS_DEBUG_DEVICE, "SPI BUSY WRITE FAIL");
      state->transfer_handler.write = 0;
      return SYSFS_SET_RETURN(EBUSY);
    }
  }

  if (ret != HAL_OK) {
    sos_debug_log_error(SOS_DEBUG_DEVICE, "SPI ERROR:%d", ret);
    state->transfer_handler.write = 0;
    return SYSFS_SET_RETURN_WITH_VALUE(EIO, ret);
  }

  return 0;
}

int mcu_spi_read(const devfs_handle_t *handle, devfs_async_t *async) {
  int ret;
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.read, async);

  if (state->o_flags & SPI_LOCAL_IS_FULL_DUPLEX) {
    ret = HAL_OK;
  } else {
    ret = HAL_SPI_Receive_IT(&state->hal_handle, async->buf, async->nbyte);
  }

  if (ret != HAL_OK) {
    state->transfer_handler.read = 0;
    sos_debug_log_error(
      SOS_DEBUG_DEVICE,
      "read failed:%d",
      state->hal_handle.ErrorCode);
    return SYSFS_SET_RETURN_WITH_VALUE(EIO, ret);
  }

  return 0;
}

#endif
