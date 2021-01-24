// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include "tmr_local.h"

#define NUM_OCS MCU_TMR_CHANNELS
#define NUM_ICS MCU_TMR_CHANNELS

#if MCU_TMR_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  tmr_io,
  TMR_VERSION,
  TMR_IOC_IDENT_CHAR,
  I_MCU_TOTAL + I_TMR_TOTAL,
  mcu_tmr_io_setchannel,
  mcu_tmr_io_getchannel,
  mcu_tmr_io_set,
  mcu_tmr_io_get,
  mcu_tmr_io_enable,
  mcu_tmr_io_disable)

int mcu_tmr_io_open(const devfs_handle_t *handle) {
  return tmr_local_open(handle);
}

int mcu_tmr_io_close(const devfs_handle_t *handle) {
  return tmr_local_close(handle);
}

int mcu_tmr_io_getinfo(const devfs_handle_t *handle, void *ctl) {
  int result = tmr_local_getinfo(handle, ctl);

  return result;
}

int mcu_tmr_io_setattr(const devfs_handle_t *handle, void *ctl) {
  return 0;
}

int mcu_tmr_io_enable(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_enable(handle, ctl);
}

int mcu_tmr_io_disable(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_disable(handle, ctl);
}

int mcu_tmr_io_setchannel(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_setchannel(handle, ctl);
}

int mcu_tmr_io_getchannel(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_getchannel(handle, ctl);
}

int mcu_tmr_io_write(const devfs_handle_t *handle, devfs_async_t *wop) {
  return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_tmr_io_setaction(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_setaction(handle, ctl);
}

int mcu_tmr_io_read(const devfs_handle_t *handle, devfs_async_t *rop) {
  return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_tmr_io_set(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_set(handle, ctl);
}

int mcu_tmr_io_get(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_get(handle, ctl);
}

#endif
