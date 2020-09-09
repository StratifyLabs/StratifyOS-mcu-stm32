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

#include "adc_local.h"
#include <fcntl.h>

#if MCU_ADC_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(adc, ADC_VERSION, ADC_IOC_IDENT_CHAR)

int mcu_adc_open(const devfs_handle_t *handle) {
  m_adc_local[handle->port].o_flags = 0;
  return adc_local_open(handle);
}

int mcu_adc_close(const devfs_handle_t *handle) {
  return adc_local_close(handle);
}

int mcu_adc_getinfo(const devfs_handle_t *handle, void *ctl) {
  return adc_local_getinfo(handle, ctl);
}

int mcu_adc_setattr(const devfs_handle_t *handle, void *ctl) {
  return adc_local_setattr(handle, ctl);
}

int mcu_adc_setaction(const devfs_handle_t *handle, void *ctl) {
  mcu_action_t *action = (mcu_action_t *)ctl;
  const u32 port = handle->port;
  adc_local_t *local = m_adc_local + port;

  if (action->handler.callback == 0) {
    // if there is an ongoing operation -- cancel it
    if (action->o_events & MCU_EVENT_FLAG_DATA_READY) {
      // execute the read callback if not null
      devfs_execute_read_handler(
        &local->transfer_handler,
        0,
        SYSFS_SET_RETURN(EAGAIN),
        MCU_EVENT_FLAG_CANCELED);
      HAL_ADC_Stop_IT(&local->hal_handle);
    }
  }

  cortexm_set_irq_priority(adc_irqs[port], action->prio, action->o_events);
  return 0;
}

int mcu_adc_read(const devfs_handle_t *handle, devfs_async_t *async) {
  int port = handle->port;
  adc_local_t *local = m_adc_local + port;

  DEVFS_DRIVER_IS_BUSY(local->transfer_handler.read, async);

  if (async->nbyte < 2) {
    local->transfer_handler.read = 0;
    return SYSFS_SET_RETURN(EINVAL);
  }

  // if location is not the group value -- configure the channel to read the
  // group
  if ((u32)async->loc < MCU_ADC_CHANNELS) {
    // configure the channel to read
    ADC_ChannelConfTypeDef channel_config;
    channel_config.Offset = 0;
    channel_config.Channel = adc_channels[async->loc];
    channel_config.Rank = 1;
#if defined ADC_SAMPLETIME_15CYCLES
    channel_config.SamplingTime = ADC_SAMPLETIME_15CYCLES;
#endif
    if (HAL_ADC_ConfigChannel(&local->hal_handle, &channel_config) != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }
  }

  local->words_read = 0;
  async->nbyte &= ~0x01; // align to 2 byte boundary

  if (HAL_ADC_Start_IT(&local->hal_handle) == HAL_OK) {
    return 0;
  }

  // this needs to read 1 byte at a time
  local->transfer_handler.read = 0;
  return SYSFS_SET_RETURN(EIO);
}

int mcu_adc_write(const devfs_handle_t *handle, devfs_async_t *async) {
  return SYSFS_SET_RETURN(ENOTSUP);
}

#endif
