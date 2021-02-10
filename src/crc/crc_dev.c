// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include <fcntl.h>

#include <cortexm/cortexm.h>
#include <mcu/crc.h>
#include <sos/debug.h>

#include "stm32_local.h"

#if MCU_CRC_PORTS > 0

static crc_state_t m_crc_state_list[MCU_CRC_PORTS] MCU_SYS_MEM;
CRC_TypeDef *const crc_regs_table[MCU_CRC_PORTS] = MCU_CRC_REGS;
s8 const crc_irqs[MCU_CRC_PORTS] = MCU_CRC_IRQS;

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  crc,
  CRC_VERSION,
  CRC_IOC_IDENT_CHAR,
  I_MCU_TOTAL + I_CRC_TOTAL,
  mcu_crc_get)

u32 mcu_calc_crc32(u32 seed, u32 polynomial, const u8 *buffer, u32 nbyte) {
  crc_state_t * state = NULL;
#if MCU_CRC_API == 1
  state->hal_handle.Instance = CRC;
  state->hal_handle.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  state->hal_handle.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  state->hal_handle.Init.InitValue = seed;
  state->hal_handle.Init.GeneratingPolynomial = polynomial;
  state->hal_handle.Init.CRCLength = CRC_POLYLENGTH_32B;
  state->hal_handle.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  state->hal_handle.Init.OutputDataInversionMode
    = CRC_OUTPUTDATA_INVERSION_DISABLE;
  state->hal_handle.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
#endif
  __HAL_RCC_CRC_CLK_ENABLE();
  HAL_CRC_Init(&state->hal_handle);

  return HAL_CRC_Accumulate(&state->hal_handle, (u32 *)buffer, nbyte);
}

u16 mcu_calc_crc16(u16 seed, u16 polynomial, const u8 *buffer, u32 nbyte) {
  crc_state_t * state = NULL;

#if MCU_CRC_API == 1
  state->hal_handle.Instance = CRC;
  state->hal_handle.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  state->hal_handle.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  state->hal_handle.Init.InitValue = seed;
  state->hal_handle.Init.GeneratingPolynomial = polynomial;
  state->hal_handle.Init.CRCLength = CRC_POLYLENGTH_16B;
  state->hal_handle.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  state->hal_handle.Init.OutputDataInversionMode
    = CRC_OUTPUTDATA_INVERSION_DISABLE;
  state->hal_handle.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
#endif
  __HAL_RCC_CRC_CLK_ENABLE();
  if (HAL_CRC_Init(&state->hal_handle) != HAL_OK) {
    return 0xffff;
  }

  return HAL_CRC_Accumulate(&state->hal_handle, (u32 *)buffer, nbyte);
}

u8 mcu_calc_crc7(u8 seed, u8 polynomial, const u8 *chr, u32 len) {
  crc_state_t * state = NULL;
  int i, a;
  unsigned char crc, data;

  // this can be done in hardware on STM32
  crc = seed;
  for (a = 0; a < len; a++) {
    data = chr[a];
    for (i = 0; i < 8; i++) {
      crc <<= 1;
      if ((data & 0x80) ^ (crc & 0x80)) {
        crc ^= 0x09;
      }
      data <<= 1;
    }
  }
  crc = (crc << 1) | 1;
  return (crc);
}

int mcu_crc_open(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(crc);
  if (state->ref_count == 0) {

    state->hal_handle.Instance = crc_regs_table[config->port];

    switch (config->port) {
    case 0:
      __HAL_RCC_CRC_CLK_ENABLE();
      break;
    }

    if (crc_irqs[config->port] > 0) {
      cortexm_enable_irq(crc_irqs[config->port]);
    }
  }
  m_crc_state_list[config->port].ref_count++;

  return 0;
}

int mcu_crc_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(crc);
  if (state->ref_count > 0) {
    if (state->ref_count == 1) {
      if (crc_irqs[config->port] > 0) {
        cortexm_disable_irq(crc_irqs[config->port]);
      }
      switch (config->port) {
      case 0:
        __HAL_RCC_CRC_CLK_DISABLE();
        break;
      }
      state->hal_handle.Instance = 0;
    }
    state->ref_count--;
  }
  return 0;
}

int mcu_crc_getinfo(const devfs_handle_t *handle, void *ctl) {
  crc_info_t *info = ctl;
  memset(info, 0, sizeof(crc_info_t));
  info->o_flags = CRC_FLAG_ENABLE | CRC_FLAG_DISABLE | CRC_FLAG_IS_32BIT;
  info->polynomial = 0x4C11DB7;
  return 0;
}

int mcu_crc_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(crc);
  const crc_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(crc, ctl);
  if (attr == 0) {
    return SYSFS_SET_RETURN(ENOSYS);
  }

  u32 o_flags = attr->o_flags;

  if (o_flags & CRC_FLAG_ENABLE) {
#if MCU_CRC_API == 1
    state->hal_handle.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
    state->hal_handle.Init.GeneratingPolynomial = attr->seed;
    if (o_flags & CRC_FLAG_IS_DEFAULT_POLYNOMIAL) {
      state->hal_handle.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
    }

    state->hal_handle.Init.DefaultPolynomialUse = DEFAULT_INIT_VALUE_DISABLE;
    state->hal_handle.Init.InitValue = attr->initial_value;
    if (o_flags & CRC_FLAG_IS_DEFAULT_INTIAL_VALUE) {
      state->hal_handle.Init.DefaultPolynomialUse = DEFAULT_INIT_VALUE_ENABLE;
    }

    state->hal_handle.Init.CRCLength = CRC_POLYLENGTH_32B;
    if (o_flags & CRC_FLAG_IS_7BIT) {
      state->hal_handle.Init.CRCLength = CRC_POLYLENGTH_7B;
    } else if (o_flags & CRC_FLAG_IS_8BIT) {
      state->hal_handle.Init.CRCLength = CRC_POLYLENGTH_8B;
    } else if (o_flags & CRC_FLAG_IS_16BIT) {
      state->hal_handle.Init.CRCLength = CRC_POLYLENGTH_16B;
    }

    state->hal_handle.Init.InputDataInversionMode
      = CRC_INPUTDATA_INVERSION_NONE;
    if (o_flags & CRC_FLAG_IS_INVERT_INPUT_8BIT) {
      state->hal_handle.Init.InputDataInversionMode
        = CRC_INPUTDATA_INVERSION_BYTE;
    } else if (o_flags & CRC_FLAG_IS_INVERT_INPUT_16BIT) {
      state->hal_handle.Init.InputDataInversionMode
        = CRC_INPUTDATA_INVERSION_HALFWORD;
    } else if (o_flags & CRC_FLAG_IS_INVERT_INPUT_32BIT) {
      state->hal_handle.Init.InputDataInversionMode
        = CRC_INPUTDATA_INVERSION_WORD;
    }

    state->hal_handle.Init.OutputDataInversionMode
      = CRC_OUTPUTDATA_INVERSION_DISABLE;
    if (o_flags & CRC_FLAG_IS_INVERT_OUTPUT) {
      state->hal_handle.Init.OutputDataInversionMode
        = CRC_OUTPUTDATA_INVERSION_ENABLE;
    }
#endif

    state->value = 0UL;
    if (HAL_CRC_Init(&state->hal_handle) != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }
  }

  if (o_flags & CRC_FLAG_DISABLE) {
    state->value = 0UL;
    if (HAL_CRC_DeInit(&state->hal_handle) != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }
  }

  return SYSFS_RETURN_SUCCESS;
}

int mcu_crc_setaction(const devfs_handle_t *handle, void *ctl) { return 0; }

int mcu_crc_read(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(crc);
  int nbyte = async->nbyte;
  if (nbyte > 4) {
    nbyte = 4;
  }
  memcpy(async->buf, &(state->value), nbyte);
  return nbyte;
}

int mcu_crc_write(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(crc);
  int nbyte = async->nbyte / 4;
  if (async->loc == 0) {
    m_crc_state_list[config->port].value
      = HAL_CRC_Calculate(&state->hal_handle, async->buf, nbyte * 4);
  } else {
    state->value = HAL_CRC_Accumulate(
      &state->hal_handle,
      async->buf,
      nbyte * 4);
  }
  // when data is written - return the CRC value -- this is a problem if the
  // value is negative
  return nbyte;
}

int mcu_crc_get(const devfs_handle_t *handle, void *arg) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(crc);
  u32 *value = arg;
  *value = state->value;
  return 0;
}

#endif
