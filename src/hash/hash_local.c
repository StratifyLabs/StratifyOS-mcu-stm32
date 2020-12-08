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

hash_state_t *m_hash_state_list[MCU_HASH_PORTS] MCU_SYS_MEM;
HASH_TypeDef *const hash_regs[MCU_HASH_PORTS] = MCU_HASH_REGS;
u8 const hash_irqs[MCU_HASH_PORTS] = MCU_HASH_IRQS;

int hash_local_open(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(hash);
  if (state->ref_count == 0) {
    DEVFS_DRIVER_OPEN_STATE_LOCAL_V4(hash);
    // turn on RCC clock
    switch (config->port) {
    case 0:
      __HAL_RCC_HASH_CLK_ENABLE();
      break;
    }
    state->transfer_handler.read = NULL;
    state->transfer_handler.write = NULL;
    // state->hal_handle.Instance = hash_regs[config->port];
    cortexm_enable_irq(hash_irqs[config->port]);
  }
  state->ref_count++;
  return 0;
}

int hash_local_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(hash);
  if (state->ref_count > 0) {
    if (state->ref_count == 1) {
      HAL_HASH_DeInit(&state->hal_handle);
      cortexm_disable_irq(hash_irqs[config->port]);
      devfs_execute_cancel_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(EDEADLK),
        MCU_EVENT_FLAG_CANCELED);
      // turn off RCC clock
      switch (config->port) {
      case 0:
        __HAL_RCC_HASH_CLK_DISABLE();
        break;
      }
      DEVFS_DRIVER_CLOSE_STATE_LOCAL_V4(hash);
    }
    state->ref_count--;
  }
  return 0;
}

int hash_local_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(hash);
  const hash_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(hash, ctl);
  if (attr == 0) {
    return SYSFS_SET_RETURN(EINVAL);
  }

  const u32 o_flags = attr->o_flags;
  state->o_flags = 0;

  if (o_flags & HASH_FLAG_SET) {

    state->hal_handle.Init.pKey = 0;    // used for HMAC not implemented
    state->hal_handle.Init.KeySize = 0; // used for HMAC not implemented

    state->hal_handle.Init.DataType = HASH_DATATYPE_32B;
    state->o_flags = o_flags;
    if (o_flags & HASH_FLAG_IS_DATA_8) {
      state->hal_handle.Init.DataType = HASH_DATATYPE_8B;
    } else if (o_flags & HASH_FLAG_IS_DATA_16) {
      state->hal_handle.Init.DataType = HASH_DATATYPE_16B;
    }

    if (HAL_HASH_Init(&state->hal_handle) != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }
  }

  return SYSFS_RETURN_SUCCESS;
}

int hash_local_setaction(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(hash);
  mcu_action_t *action = ctl;

  if (action->handler.callback == 0) {
    devfs_execute_cancel_handler(&state->transfer_handler, 0, 0, 0);
  }

  // update the priority
  cortexm_set_irq_priority(
    hash_irqs[config->port],
    action->prio,
    action->o_events);

  return SYSFS_RETURN_SUCCESS;
}

void HAL_HASH_InCpltCallback(HASH_HandleTypeDef *hhash) {
  // hash_state_t * local = m_hash_local + 0;
  // execute the callbacks
  // devfs_execute_read_handler(&state->transfer_handler, 0, 0,
  // MCU_EVENT_FLAG_DATA_READY);
  // devfs_execute_write_handler(&state->transfer_handler, 0, 0,
  // MCU_EVENT_FLAG_WRITE_COMPLETE);
}

void HAL_HASH_DgstCpltCallback(HASH_HandleTypeDef *hhash) {
  hash_state_t *state = (hash_state_t *)hhash;

  // finish the transaction?
  HAL_HASHEx_SHA256_Finish(
    hhash,
    state->transfer_handler.read->buf,
    HAL_MAX_DELAY);

  // execute the callbacks
  devfs_execute_read_handler(
    &state->transfer_handler,
    0,
    0,
    MCU_EVENT_FLAG_DATA_READY);
  devfs_execute_write_handler(
    &state->transfer_handler,
    0,
    0,
    MCU_EVENT_FLAG_WRITE_COMPLETE);
}

void HAL_HASH_ErrorCallback(HASH_HandleTypeDef *hhash) {
  hash_state_t *state = (hash_state_t *)hhash;
  devfs_execute_cancel_handler(
    &state->transfer_handler,
    0,
    0,
    MCU_EVENT_FLAG_ERROR);
}

#if 0 // must use the random driver when using hash
void mcu_core_hash_rng_isr() {
  hash_state_t *local = m_hash_local + 0;
  HAL_HASH_IRQHandler(&state->hal_handle);
}
#endif

#endif
