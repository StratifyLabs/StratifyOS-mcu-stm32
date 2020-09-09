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

#include "hash_local.h"
#include <mcu/hash.h>

#if MCU_HASH_PORTS > 0

hash_local_t m_hash_local[MCU_HASH_PORTS] MCU_SYS_MEM;
HASH_TypeDef *const hash_regs[MCU_HASH_PORTS] = MCU_HASH_REGS;
u8 const hash_irqs[MCU_HASH_PORTS] = MCU_HASH_IRQS;

int hash_local_open(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_LOCAL(hash, MCU_HASH_PORTS);
  if (port < MCU_SPI_PORTS) {
    if (local->ref_count == 0) {
      // turn on RCC clock
      switch (port) {
      case 0:
        __HAL_RCC_HASH_CLK_ENABLE();
        break;
      }
      local->transfer_handler.read = NULL;
      local->transfer_handler.write = NULL;
      // local->hal_handle.Instance = hash_regs[port];
      cortexm_enable_irq(hash_irqs[port]);
    }
    local->ref_count++;
    return 0;
  }

  return SYSFS_SET_RETURN(EINVAL);
}

int hash_local_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_LOCAL(hash, MCU_HASH_PORTS);
  if (local->ref_count > 0) {
    if (local->ref_count == 1) {
      HAL_HASH_DeInit(&local->hal_handle);
      cortexm_disable_irq(hash_irqs[port]);
      devfs_execute_cancel_handler(
        &local->transfer_handler,
        0,
        SYSFS_SET_RETURN(EDEADLK),
        MCU_EVENT_FLAG_CANCELED);
      // turn off RCC clock
      switch (port) {
      case 0:
        __HAL_RCC_HASH_CLK_DISABLE();
        break;
      }
    }
    local->ref_count--;
  }
  return 0;
}

int hash_local_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_LOCAL(hash, MCU_HASH_PORTS);
  const hash_attr_t *attr = mcu_select_attr(handle, ctl);
  if (attr == 0) {
    return SYSFS_SET_RETURN(EINVAL);
  }

  u32 o_flags = attr->o_flags;
  local->o_flags = 0;

  if (o_flags & HASH_FLAG_SET) {

    local->hal_handle.Init.pKey = 0;    // used for HMAC not implemented
    local->hal_handle.Init.KeySize = 0; // used for HMAC not implemented

    local->hal_handle.Init.DataType = HASH_DATATYPE_32B;
    local->o_flags = o_flags;
    if (o_flags & HASH_FLAG_IS_DATA_8) {
      local->hal_handle.Init.DataType = HASH_DATATYPE_8B;
    } else if (o_flags & HASH_FLAG_IS_DATA_16) {
      local->hal_handle.Init.DataType = HASH_DATATYPE_16B;
    }

    if (HAL_HASH_Init(&local->hal_handle) != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }
  }

  return SYSFS_RETURN_SUCCESS;
}

int hash_local_setaction(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_LOCAL(hash, MCU_HASH_PORTS);
  mcu_action_t *action = ctl;

  if (action->handler.callback == 0) {
    devfs_execute_cancel_handler(&local->transfer_handler, 0, 0, 0);
  }

  // update the priority
  cortexm_set_irq_priority(hash_irqs[port], action->prio, action->o_events);

  return SYSFS_RETURN_SUCCESS;
}

void HAL_HASH_InCpltCallback(HASH_HandleTypeDef *hhash) {
  // hash_local_t * local = m_hash_local + 0;
  // execute the callbacks
  // devfs_execute_read_handler(&local->transfer_handler, 0, 0,
  // MCU_EVENT_FLAG_DATA_READY);
  // devfs_execute_write_handler(&local->transfer_handler, 0, 0,
  // MCU_EVENT_FLAG_WRITE_COMPLETE);
}

void HAL_HASH_DgstCpltCallback(HASH_HandleTypeDef *hhash) {
  hash_local_t *local = m_hash_local + 0;

  // finish the transaction?
  HAL_HASHEx_SHA256_Finish(
    hhash,
    local->transfer_handler.read->buf,
    HAL_MAX_DELAY);

  // execute the callbacks
  devfs_execute_read_handler(
    &local->transfer_handler,
    0,
    0,
    MCU_EVENT_FLAG_DATA_READY);
  devfs_execute_write_handler(
    &local->transfer_handler,
    0,
    0,
    MCU_EVENT_FLAG_WRITE_COMPLETE);
}

void HAL_HASH_ErrorCallback(HASH_HandleTypeDef *hhash) {
  hash_local_t *local = m_hash_local + 0;
  devfs_execute_cancel_handler(
    &local->transfer_handler,
    0,
    0,
    MCU_EVENT_FLAG_ERROR);
}

#if 0 // must use the random driver when using hash
void mcu_core_hash_rng_isr(){
	hash_local_t * local = m_hash_local + 0;
	HAL_HASH_IRQHandler(&local->hal_handle);
}
#endif

#endif
