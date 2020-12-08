/* Copyright 2011-2019 Tyler Gilbert;
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

#include "hash_local.h"
#include <fcntl.h>

#if MCU_HASH_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(hash, HASH_VERSION, HASH_IOC_CHAR)

int mcu_hash_open(const devfs_handle_t *handle) {
  return hash_local_open(handle);
}

int mcu_hash_close(const devfs_handle_t *handle) {
  return hash_local_close(handle);
}

int mcu_hash_getinfo(const devfs_handle_t *handle, void *ctl) {
  // hash_info_t * info = ctl;
  // info->o_flags = 0;

  return SYSFS_RETURN_SUCCESS;
}

int mcu_hash_setattr(const devfs_handle_t *handle, void *ctl) {
  return hash_local_setattr(handle, ctl);
}

int mcu_hash_setaction(const devfs_handle_t *handle, void *ctl) {
  return hash_local_setaction(handle, ctl);
}

int mcu_hash_read(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(hash);
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.read, async);
  // read just sets up the buffer - the action starts when the device is written
  int minimum_size = 0;
  if (state->o_flags & HASH_FLAG_IS_SHA1) {
    minimum_size = 20;
  } else if (state->o_flags & HASH_FLAG_IS_MD5) {
    minimum_size = 16;
  } else if (state->o_flags & HASH_FLAG_IS_SHA224) {
    minimum_size = 28;
  } else if (state->o_flags & HASH_FLAG_IS_SHA256) {
    minimum_size = 32;
  } else {
    state->transfer_handler.read = 0;
    return SYSFS_SET_RETURN(EINVAL);
  }

  if (async->nbyte < minimum_size) {
    state->transfer_handler.read = 0;
    return SYSFS_SET_RETURN(EINVAL);
  }

  return SYSFS_RETURN_SUCCESS;
}

int mcu_hash_write(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(hash);
  if (async->nbyte == 0) {
    HAL_HASHEx_SHA256_Finish(
      &state->hal_handle,
      state->transfer_handler.read->buf,
      HAL_MAX_DELAY);
    return 0;
  }

  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.write, async);

  if (state->transfer_handler.read == 0) {
    // read must be setup first -- that is where the destination data will go
    state->transfer_handler.write = 0;
    return SYSFS_SET_RETURN(EINVAL);
  }

  int result;
  if (state->o_flags & HASH_FLAG_IS_SHA1) {
    // nbyte is /4 based on DataWidthUnit
    result = HAL_HASH_SHA1_Start_IT(
      &state->hal_handle,
      async->buf,
      async->nbyte,
      state->transfer_handler.read->buf);
  } else if (state->o_flags & HASH_FLAG_IS_MD5) {
    // nbyte is /4 based on DataWidthUnit
    result = HAL_HASH_MD5_Start_IT(
      &state->hal_handle,
      async->buf,
      async->nbyte,
      state->transfer_handler.read->buf);
  } else if (state->o_flags & HASH_FLAG_IS_SHA224) {
    // nbyte is /4 based on DataWidthUnit
    result = HAL_HASHEx_SHA224_Start_IT(
      &state->hal_handle,
      async->buf,
      async->nbyte,
      state->transfer_handler.read->buf);
  } else if (state->o_flags & HASH_FLAG_IS_SHA256) {
    // nbyte is /4 based on DataWidthUnit
    result = HAL_HASHEx_SHA256_Start_IT(
      &state->hal_handle,
      async->buf,
      async->nbyte,
      state->transfer_handler.read->buf);
  } else {
    // must have set a hash type
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
