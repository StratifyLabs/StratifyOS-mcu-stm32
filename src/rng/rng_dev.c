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

#include <cortexm/cortexm.h>
#include <fcntl.h>
#include <sos/debug.h>
#include <mcu/rng.h>

#include "../hash/hash_local.h"
#include "stm32_local.h"

#if MCU_RNG_PORTS > 0

static rng_state_t *m_rng_state_list[MCU_RNG_PORTS] MCU_SYS_MEM;
RNG_TypeDef *const rng_regs_table[MCU_RNG_PORTS] = MCU_RNG_REGS;
s8 const rng_irqs[MCU_RNG_PORTS] = MCU_RNG_IRQS;

DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(rng, RANDOM_VERSION, RANDOM_IOC_CHAR)

int mcu_rng_open(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(rng);
  if (state->ref_count == 0) {
    state->hal_handle.Instance = rng_regs_table[config->port];
    DEVFS_DRIVER_OPEN_STATE_LOCAL_V4(rng);
    state->transfer_handler.read = NULL;
    state->transfer_handler.write = NULL;
    switch (config->port) {
    case 0:
      __HAL_RCC_RNG_CLK_ENABLE();
      break;
    }
    cortexm_enable_irq(rng_irqs[config->port]);
  }
  state->ref_count++;
  return 0;
}

int mcu_rng_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(rng);
  if (state->ref_count > 0) {
    if (state->ref_count == 1) {
      cortexm_disable_irq(rng_irqs[config->port]);
      switch (config->port) {
      case 0:
        __HAL_RCC_RNG_CLK_DISABLE();
        break;
      }
      state->hal_handle.Instance = 0;
      DEVFS_DRIVER_CLOSE_STATE_LOCAL_V4(rng);
    }
    state->ref_count--;
  }
  return 0;
}

int mcu_rng_getinfo(const devfs_handle_t *handle, void *ctl) {
  random_info_t *info = ctl;
  memset(info, 0, sizeof(random_info_t));
  info->o_flags
    = RANDOM_FLAG_IS_TRUE | RANDOM_FLAG_ENABLE | RANDOM_FLAG_DISABLE;
  return 0;
}

int mcu_rng_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(rng);
  const random_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(random, ctl);
  if (attr == 0) {
    return SYSFS_SET_RETURN(ENOSYS);
  }

  u32 o_flags = attr->o_flags;

  if (o_flags & RANDOM_FLAG_ENABLE) {
    if (HAL_RNG_Init(&state->hal_handle) != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }
  }

  if (o_flags & RANDOM_FLAG_DISABLE) {
    SOS_DEBUG_LINE_TRACE();
    if (HAL_RNG_DeInit(&state->hal_handle) != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }
  }

  return SYSFS_RETURN_SUCCESS;
}

int mcu_rng_setaction(const devfs_handle_t *handle, void *ctl) { return 0; }

int mcu_rng_read(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(rng);

  if (async->nbyte & 0x03) {
    return SYSFS_SET_RETURN(EINVAL);
  }

  if (state->transfer_handler.read) {
    sos_debug_printf("RNG is busy %p\n", state->transfer_handler.read);
  }
  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.read, async);

  // need to operate on 4 byte boundaries
  state->bytes_read = 0;

  if (HAL_RNG_GenerateRandomNumber_IT(&state->hal_handle) != HAL_OK) {
    sos_debug_printf("EIO here %d\n", state->hal_handle.ErrorCode);
    state->transfer_handler.read = NULL;
    return SYSFS_SET_RETURN(EIO);
  }

  return SYSFS_RETURN_SUCCESS;
}

int mcu_rng_write(const devfs_handle_t *handle, devfs_async_t *async) {
  return SYSFS_SET_RETURN(ENOSYS);
}

void HAL_RNG_ErrorCallback(RNG_HandleTypeDef *hrng) {
  rng_state_t *state = (rng_state_t *)hrng;
  SOS_DEBUG_LINE_TRACE();
  hrng->State = HAL_RNG_STATE_READY;

  sos_debug_printf("RNG ERROR: %d\n", hrng->ErrorCode);
  // TODO error
}

void HAL_RNG_ReadyDataCallback(RNG_HandleTypeDef *hrng, uint32_t random32bit) {
  rng_state_t *state = (rng_state_t *)hrng;

  memcpy(
    state->transfer_handler.read->buf + state->bytes_read,
    &random32bit,
    sizeof(uint32_t));
  state->bytes_read += sizeof(u32);

  if (state->bytes_read == state->transfer_handler.read->nbyte) {
    devfs_execute_read_handler(
      &state->transfer_handler,
      0,
      0,
      MCU_EVENT_FLAG_DATA_READY);
  } else {
    HAL_RNG_GenerateRandomNumber_IT(hrng);
  }
}

void mcu_core_hash_isr() {
  if (m_rng_state_list[0]) {
    HAL_RNG_IRQHandler(&m_rng_state_list[0]->hal_handle);
  }

#if MCU_HASH_PORTS > 0
  if (m_hash_state_list[0]) {
    HAL_HASH_IRQHandler(&m_hash_state_list[0]->hal_handle);
  }
#endif
}

#endif
