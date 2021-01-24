// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include "crypt_local.h"
#include <fcntl.h>

#if MCU_CRYPT_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  crypt,
  CRYPT_VERSION,
  CRYPT_IOC_CHAR,
  I_MCU_TOTAL + I_CRYPT_TOTAL,
  mcu_crypt_getiv)

int mcu_crypt_open(const devfs_handle_t *handle) {
  return crypt_local_open(handle);
}

int mcu_crypt_close(const devfs_handle_t *handle) {
  return crypt_local_close(handle);
}

int mcu_crypt_getinfo(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(crypt);
  crypt_info_t *info = ctl;
  info->o_flags
    = CRYPT_FLAG_SET_CIPHER | CRYPT_FLAG_IS_AES_128 | CRYPT_FLAG_IS_AES_192
      | CRYPT_FLAG_IS_AES_256 | CRYPT_FLAG_IS_AES_ECB | CRYPT_FLAG_IS_AES_CBC
      | CRYPT_FLAG_IS_AES_CTR |
#if defined CRYP_CR_ALGOMODE_AES_GCM
      CRYPT_FLAG_IS_AES_GCM | CRYPT_FLAG_IS_AES_CCM |
#endif
      CRYPT_FLAG_IS_DATA_1 | CRYPT_FLAG_IS_DATA_8 | CRYPT_FLAG_IS_DATA_16
      | CRYPT_FLAG_IS_DATA_32 | CRYPT_FLAG_SET_MODE | CRYPT_FLAG_IS_ENCRYPT
      | CRYPT_FLAG_IS_DECRYPT;

  return SYSFS_RETURN_SUCCESS;
}

int mcu_crypt_setattr(const devfs_handle_t *handle, void *ctl) {
  return crypt_local_setattr(handle, ctl);
}

int mcu_crypt_setaction(const devfs_handle_t *handle, void *ctl) {
  return crypt_local_setaction(handle, ctl);
}

int mcu_crypt_getiv(const devfs_handle_t *handle, void *ctl) {
  return crypt_local_getiv(handle, ctl);
}

int mcu_crypt_read(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(crypt);
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.read, async);
  // read just sets up the buffer - the action starts when the device is written

  return SYSFS_RETURN_SUCCESS;
}

int mcu_crypt_write(const devfs_handle_t *handle, devfs_async_t *async) {

  DEVFS_DRIVER_DECLARE_CONFIG_STATE(crypt);
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.write, async);

  if (state->transfer_handler.read == 0) {
    // read must be setup first -- that is where the destination data will go
    state->transfer_handler.write = 0;
    return SYSFS_SET_RETURN(EINVAL);
  }

  if (
    state->transfer_handler.read->nbyte
    < state->transfer_handler.write->nbyte) {
    // read and write need to have the same number of bytes
    state->transfer_handler.write = 0;
    return SYSFS_SET_RETURN(EINVAL);
  }

  state->transfer_handler.read->nbyte = state->transfer_handler.write->nbyte;

  // blocks are always 128-bit (or 16 bytes)
  if ((state->transfer_handler.write->nbyte % 16) != 0) {
    state->transfer_handler.write = 0;
    return SYSFS_SET_RETURN(EINVAL);
  }

  int result;
  if (state->o_flags & CRYPT_FLAG_IS_ENCRYPT) {
    // nbyte is /4 based on DataWidthUnit
    result = HAL_CRYP_Encrypt_IT(
      &state->hal_handle,
      async->buf,
      async->nbyte / 4,
      state->transfer_handler.read->buf);
  } else if (state->o_flags & CRYPT_FLAG_IS_DECRYPT) {
    // nbyte is /4 based on DataWidthUnit
    result = HAL_CRYP_Decrypt_IT(
      &state->hal_handle,
      async->buf,
      async->nbyte / 4,
      state->transfer_handler.read->buf);
  } else {
    // must set either encrypt or decrypt
    state->transfer_handler.write = 0;
    return SYSFS_SET_RETURN(EINVAL);
  }

  if (result != HAL_OK) {
    state->transfer_handler.write = 0;
    return SYSFS_SET_RETURN(EIO);
  }

  // read just sets up the buffer - the action starts when the device is written

  return SYSFS_RETURN_SUCCESS;
}

#endif
