// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include "i2s_spi_local.h"

#include <mcu/i2s.h>
#include <mcu/spi.h>

#if MCU_I2S_SPI_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  i2s_spi,
  I2S_VERSION,
  I2S_IOC_IDENT_CHAR,
  I_MCU_TOTAL + I_I2S_TOTAL,
  mcu_i2s_spi_mute,
  mcu_i2s_spi_unmute)

int mcu_i2s_spi_open(const devfs_handle_t *handle) {
  // make sure port supports I2S -- not all ports support I2S
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);
  state->o_flags = 0;
  return i2s_spi_local_open(handle);
}

int mcu_i2s_spi_close(const devfs_handle_t *handle) {
  // same as SPI
  return i2s_spi_local_close(handle);
}

int mcu_i2s_spi_getinfo(const devfs_handle_t *handle, void *ctl) {
  i2s_info_t *info = ctl;

  // set I2S Capability flags
  info->o_flags = 0;

  return 0;
}

int mcu_i2s_spi_mute(const devfs_handle_t *handle, void *ctl) { return 0; }

int mcu_i2s_spi_unmute(const devfs_handle_t *handle, void *ctl) { return 0; }

int mcu_i2s_spi_setattr(const devfs_handle_t *handle, void *ctl) {
  return i2s_spi_local_setattr(handle, ctl);
}

int mcu_i2s_spi_setaction(const devfs_handle_t *handle, void *ctl) {
  return mcu_spi_setaction(handle, ctl);
}

int mcu_i2s_spi_write(const devfs_handle_t *handle, devfs_async_t *async) {
  int ret;
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.write, async);

  if (state->o_flags & SPI_LOCAL_IS_FULL_DUPLEX) {

#if defined SPI_I2S_FULLDUPLEX_SUPPORT
    if (state->transfer_handler.read->nbyte < async->nbyte) {
      return SYSFS_SET_RETURN(EINVAL);
    }

    i2s_spi_local_wait_for_errata_level(state);
    ret = HAL_I2SEx_TransmitReceive_IT(
      &state->i2s_hal_handle,
      async->buf,
      state->transfer_handler.read->buf,
      async->nbyte / state->size_mult);
#else
    return SYSFS_SET_RETURN(ENOTSUP);
#endif

  } else {
    i2s_spi_local_wait_for_errata_level(state);
    ret = HAL_I2S_Transmit_IT(
      &state->i2s_hal_handle,
      async->buf,
      async->nbyte / state->size_mult);
  }

  if (ret != HAL_OK) {
    state->transfer_handler.write = 0;
    return SYSFS_SET_RETURN_WITH_VALUE(EIO, ret);
  }

  return 0;
}

int mcu_i2s_spi_read(const devfs_handle_t *handle, devfs_async_t *async) {
  int ret;
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);

  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.read, async);

#if defined SPI_I2S_FULLDUPLEX_SUPPORT
  if (state->o_flags & SPI_LOCAL_IS_FULL_DUPLEX) {
    return SYSFS_RETURN_SUCCESS;
  }
#endif
  // check for overrun
  i2s_spi_local_wait_for_errata_level(state);
  ret = HAL_I2S_Receive_IT(
    &state->i2s_hal_handle,
    async->buf,
    async->nbyte / state->size_mult);

  if (ret != HAL_OK) {
    state->transfer_handler.read = 0;
    sos_debug_log_error(SOS_DEBUG_DEVICE, "I2S SPI Read failed");
    return SYSFS_SET_RETURN_WITH_VALUE(EIO, ret);
  }

  return 0;
}

#endif
