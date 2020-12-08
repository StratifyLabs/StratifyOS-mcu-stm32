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

#include "crypt_local.h"
#include <mcu/crypt.h>

#if MCU_CRYPT_PORTS > 0

crypt_state_t *m_crypt_state_list[MCU_CRYPT_PORTS] MCU_SYS_MEM;
CRYP_TypeDef *const crypt_regs[MCU_CRYPT_PORTS] = MCU_CRYPT_REGS;
u8 const crypt_irqs[MCU_CRYPT_PORTS] = MCU_CRYPT_IRQS;

int crypt_local_open(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(crypt);
  if (state->ref_count == 0) {
    DEVFS_DRIVER_OPEN_STATE_LOCAL_V4(crypt);
    // turn on RCC clock
    switch (config->port) {
    case 0:
      __HAL_RCC_CRYP_CLK_ENABLE();
      break;
    }
    state->transfer_handler.read = NULL;
    state->transfer_handler.write = NULL;
    state->hal_handle.Instance = crypt_regs[config->port];
    cortexm_enable_irq(crypt_irqs[config->port]);
  }
    state->ref_count++;
    return 0;
}

int crypt_local_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(crypt);
  if (state->ref_count > 0) {
    if (state->ref_count == 1) {
      HAL_CRYP_DeInit(&state->hal_handle);
      cortexm_disable_irq(crypt_irqs[config->port]);
      devfs_execute_cancel_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(EDEADLK),
        MCU_EVENT_FLAG_CANCELED);
      // turn off RCC clock
      switch (config->port) {
      case 0:
        __HAL_RCC_CRYP_CLK_DISABLE();
        break;
      }
      DEVFS_DRIVER_CLOSE_STATE_LOCAL_V4(crypt);
    }
    state->ref_count--;
  }
  return 0;
}

int crypt_local_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(crypt);
  const crypt_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(crypt, ctl);
  if (attr == 0) {
    return SYSFS_SET_RETURN(EINVAL);
  }

  const u32 o_flags = attr->o_flags;
  state->o_flags = 0;

  if (o_flags & CRYPT_FLAG_SET_CIPHER) {

    state->o_flags = o_flags;

    state->hal_handle.Init.DataType = CRYP_DATATYPE_32B;
    if (o_flags & CRYPT_FLAG_IS_DATA_1) {
      state->hal_handle.Init.DataType = CRYP_DATATYPE_1B;
    } else if (o_flags & CRYPT_FLAG_IS_DATA_8) {
      state->hal_handle.Init.DataType = CRYP_DATATYPE_8B;
    } else if (o_flags & CRYPT_FLAG_IS_DATA_16) {
      state->hal_handle.Init.DataType = CRYP_DATATYPE_16B;
    }

    u32 key_size = 128 / (16);
    state->hal_handle.Init.KeySize = CRYP_KEYSIZE_128B;
    if (o_flags & CRYPT_FLAG_IS_AES_192) {
      state->hal_handle.Init.KeySize = CRYP_KEYSIZE_192B;
      key_size = 192 / 16;
    } else if (o_flags & CRYPT_FLAG_IS_AES_256) {
      state->hal_handle.Init.KeySize = CRYP_KEYSIZE_256B;
      key_size = 256 / 16;
    }

    // Key and initialization vector
    state->hal_handle.Init.pKey = (u32 *)attr->key;
    memcpy(state->iv, attr->iv, 16);

    u32 *ptr_key = (u32 *)attr->key;
    for (u32 i = 0; i < key_size; i++) {
      ptr_key[i] = __REV(ptr_key[i]);
    }

    u32 *ptr = (u32 *)state->iv;
    for (u32 i = 0; i < 4; i++) {
      ptr[i] = __REV(ptr[i]);
    }
    state->hal_handle.Init.pInitVect = (u32 *)state->iv;

    // Algorithm
    state->hal_handle.Init.Algorithm = CRYP_AES_CTR;
    if (o_flags & CRYPT_FLAG_IS_AES_CBC) {
      state->hal_handle.Init.Algorithm = CRYP_AES_CBC;
    } else if (o_flags & CRYPT_FLAG_IS_AES_ECB) {
      state->hal_handle.Init.Algorithm = CRYP_AES_ECB;
    }

    // GCM and CCM for CRYPT units that support it (not all chips do)
#if defined CRYP_CR_ALGOMODE_AES_GCM
    if (o_flags & CRYPT_FLAG_IS_AES_GCM) {
      state->hal_handle.Init.Algorithm = CRYP_AES_GCM;
    }

    state->hal_handle.Init.B0 = 0; // \todo This needs to be updated
    if (o_flags & CRYPT_FLAG_IS_AES_CCM) {
      state->hal_handle.Init.Algorithm = CRYP_AES_CCM;
    }
#else
    // only used with CCM
    state->hal_handle.Init.B0 = 0;
#endif

    // header buffer
    // header size
    state->hal_handle.Init.Header = (u32 *)state->header;
    state->hal_handle.Init.HeaderSize = 0;

    // DataWidthUnit
    state->hal_handle.Init.DataWidthUnit = CRYP_DATAWIDTHUNIT_WORD;

    // init if not yet initialized
    if (HAL_CRYP_Init(&state->hal_handle) != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }
  }

  if (o_flags & CRYPT_FLAG_SET_MODE) {

    // assume encryption if decrypt is not specified
    state->o_flags |= CRYPT_FLAG_IS_ENCRYPT;
    state->o_flags &= ~CRYPT_FLAG_IS_DECRYPT;

    if (o_flags & CRYPT_FLAG_IS_DECRYPT) {
      state->o_flags |= CRYPT_FLAG_IS_DECRYPT;
      state->o_flags &= ~CRYPT_FLAG_IS_ENCRYPT;
    }
  }

  return SYSFS_RETURN_SUCCESS;
}

int crypt_local_setaction(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(crypt);
  mcu_action_t *action = ctl;

  if (action->handler.callback == 0) {
    devfs_execute_cancel_handler(&state->transfer_handler, 0, 0, 0);
  }

  // update the priority
  cortexm_set_irq_priority(crypt_irqs[config->port], action->prio, action->o_events);

  return SYSFS_RETURN_SUCCESS;
}

int crypt_local_getiv(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(crypt);
  memcpy(ctl, state->iv, sizeof(state->iv));
  u32 *ptr = (u32 *)ctl;
  for (u32 i = 0; i < 4; i++) {
    ptr[i] = __REV(ptr[i]);
  }
  return 0;
}

void HAL_CRYP_InCpltCallback(CRYP_HandleTypeDef *hcryp) {
  crypt_state_t *state = (crypt_state_t *)hcryp;
  // execute the callbacks
  devfs_execute_write_handler(
    &state->transfer_handler,
    0,
    0,
    MCU_EVENT_FLAG_WRITE_COMPLETE);
}

void HAL_CRYP_OutCpltCallback(CRYP_HandleTypeDef *hcryp) {
  crypt_state_t *state = (crypt_state_t *)hcryp;
  // execute the callbacks
  devfs_execute_read_handler(
    &state->transfer_handler,
    0,
    0,
    MCU_EVENT_FLAG_DATA_READY);
  // update the IV once reading is complete
  *(hcryp->Init.pInitVect) = hcryp->Instance->IV0LR;
  *(hcryp->Init.pInitVect + 1) = hcryp->Instance->IV0RR;
  *(hcryp->Init.pInitVect + 2) = hcryp->Instance->IV1LR;
  *(hcryp->Init.pInitVect + 3) = hcryp->Instance->IV1RR;
}

void HAL_CRYP_ErrorCallback(CRYP_HandleTypeDef *hcryp) {
  crypt_state_t *state = (crypt_state_t *)hcryp;
  devfs_execute_cancel_handler(
    &state->transfer_handler,
    0,
    0,
    MCU_EVENT_FLAG_ERROR);
}

void mcu_core_cryp_isr() {
  crypt_state_t *state = m_crypt_state_list[0];
  HAL_CRYP_IRQHandler(&state->hal_handle);
}

#endif
