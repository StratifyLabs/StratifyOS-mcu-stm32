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

#include "tmr_local.h"

#define NUM_OCS MCU_TMR_CHANNELS
#define NUM_ICS MCU_TMR_CHANNELS

#if MCU_TMR_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  tmr_pwm,
  TMR_VERSION,
  TMR_IOC_IDENT_CHAR,
  I_MCU_TOTAL + I_TMR_TOTAL,
  mcu_tmr_pwm_setchannel,
  mcu_tmr_pwm_getchannel,
  mcu_tmr_pwm_set,
  mcu_tmr_pwm_get,
  mcu_tmr_pwm_enable,
  mcu_tmr_pwm_disable)

int mcu_tmr_pwm_open(const devfs_handle_t *handle) {
  return tmr_local_open(handle);
}

int mcu_tmr_pwm_close(const devfs_handle_t *handle) {
  return tmr_local_close(handle);
}

int mcu_tmr_pwm_getinfo(const devfs_handle_t *handle, void *ctl) {
  int result = tmr_local_getinfo(handle, ctl);

  return result;
}

int mcu_tmr_pwm_setattr(const devfs_handle_t *handle, void *ctl) { return 0; }

int mcu_tmr_pwm_enable(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_enable(handle, ctl);
}

int mcu_tmr_pwm_disable(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_disable(handle, ctl);
}

int mcu_tmr_pwm_setchannel(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_setchannel(handle, ctl);
}

int mcu_tmr_pwm_getchannel(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_getchannel(handle, ctl);
}

int mcu_tmr_pwm_write(const devfs_handle_t *handle, devfs_async_t *wop) {
  return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_tmr_pwm_setaction(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_setaction(handle, ctl);
}

int mcu_tmr_pwm_read(const devfs_handle_t *handle, devfs_async_t *rop) {
  return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_tmr_pwm_set(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_set(handle, ctl);
}

int mcu_tmr_pwm_get(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_get(handle, ctl);
}

#endif
