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

#include <fcntl.h>
#include <sched.h>

#include <cortexm/cortexm.h>
#include <mcu/pio.h>
#include <mcu/qspi.h>
#include <sos/config.h>
#include <sos/debug.h>

#include "stm32_local.h"

#if MCU_QSPI_PORTS > 0

#include "qspi_local.h"

qspi_state_t *m_qspi_state_list[MCU_QSPI_PORTS] MCU_SYS_MEM;
QUADSPI_TypeDef *const qspi_regs_table[MCU_QSPI_PORTS] = MCU_QSPI_REGS;
u8 const qspi_irqs[MCU_QSPI_PORTS] = MCU_QSPI_IRQS;

static void populate_command(
  QSPI_CommandTypeDef *command,
  u32 o_flags,
  u32 opcode,
  u32 address,
  int data_size,
  int dummy_cycles);

int qspi_local_open(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(qspi);

  if (state->ref_count == 0) {
    DEVFS_DRIVER_OPEN_STATE_LOCAL_V4(qspi);
    memset(&state->hal_handle, 0, sizeof(QSPI_HandleTypeDef));
    state->hal_handle.Instance = qspi_regs_table[config->port];

    switch (config->port) {
    case 0:
      __HAL_RCC_QSPI_CLK_ENABLE();
      break;
    }
    // reset HAL UART
    cortexm_enable_irq(qspi_irqs[config->port]);
  }
  state->ref_count++;

  return 0;
}

int qspi_local_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(qspi);
  if (state->ref_count > 0) {
    if (state->ref_count == 1) {
      cortexm_disable_irq(qspi_irqs[config->port]);
      switch (config->port) {
      case 0:
        __HAL_RCC_QSPI_CLK_DISABLE();
        break;
      }
      state->hal_handle.Instance = 0;
    }
    state->ref_count--;
  }
  return 0;
}

int qspi_local_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(qspi);
  const qspi_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(qspi, ctl);
  if (attr == NULL) {
    return SYSFS_SET_RETURN(EINVAL);
  }
  const u32 o_flags = attr->o_flags;
  state->state = o_flags;
  if (o_flags & QSPI_FLAG_SET_MASTER) {
    // uint32_t flash_size = 24;
    __HAL_RCC_QSPI_FORCE_RESET();
    __HAL_RCC_QSPI_RELEASE_RESET();

    if (
      mcu_set_pin_assignment(
        &(attr->pin_assignment),
        MCU_CONFIG_PIN_ASSIGNMENT(qspi_config_t, handle),
        MCU_PIN_ASSIGNMENT_COUNT(qspi_pin_assignment_t),
        CORE_PERIPH_QSPI,
        config->port,
        0,
        0,
        0)
      < 0) {
      return SYSFS_SET_RETURN(EINVAL);
    }

    // prescalar can be between 0 and 255
    u32 prescalar;
    if (attr->freq) {
      prescalar = sos_config.clock.frequency / attr->freq;
      if (prescalar > 255) {
        prescalar = 255;
      }
    } else {
      prescalar = 0;
    }

    state->hal_handle.Init.ClockPrescaler = prescalar; // need to calculate
    state->hal_handle.Init.FifoThreshold
      = 1; // interrupt fires when FIFO is half full
    state->hal_handle.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
    state->hal_handle.Init.FlashSize = 31; /*attribute size 2^size-1*/
    state->hal_handle.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_2_CYCLE;
    state->hal_handle.Init.ClockMode = QSPI_CLOCK_MODE_0;
    if (o_flags & QSPI_FLAG_IS_CLOCK_MODE_3) {
      state->hal_handle.Init.ClockMode = QSPI_CLOCK_MODE_3;
    }
    // Clock mode QSPI_CLOCK_MODE_3 is double data rate
    if (o_flags & QSPI_FLAG_IS_FLASH_ID_2) {
      state->hal_handle.Init.FlashID = QSPI_FLASH_ID_2;
      state->hal_handle.Init.DualFlash = QSPI_DUALFLASH_ENABLE;
    } else {
      /*by default*/
      state->hal_handle.Init.FlashID = QSPI_FLASH_ID_1;
      state->hal_handle.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
    }
    if (HAL_QSPI_Init(&state->hal_handle) != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }

    if (o_flags & QSPI_FLAG_IS_MEMORY_MAPPED) {
      QSPI_CommandTypeDef s_command;
      QSPI_MemoryMappedTypeDef s_mem_mapped_cfg;

      populate_command(
        &s_command,
        attr->o_flags,
        attr->memory_mapped_read_instruction,
        0,
        4,
        attr->memory_mapped_read_dummy_cycles);

      /* Configure the memory mapped mode */
      s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
      s_mem_mapped_cfg.TimeOutPeriod = 0;

      if (
        HAL_QSPI_MemoryMapped(&state->hal_handle, &s_command, &s_mem_mapped_cfg)
        != HAL_OK) {
        return SYSFS_SET_RETURN(EIO);
      }
    }
  }

  return SYSFS_RETURN_SUCCESS;
}

int qspi_local_setaction(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(qspi);
  mcu_action_t *action = ctl;
  if (action->handler.callback != 0) {
    return SYSFS_SET_RETURN(ENOTSUP);
  }
  cortexm_set_irq_priority(
    qspi_irqs[config->port],
    action->prio,
    action->o_events);
  return 0;
}

void populate_command(
  QSPI_CommandTypeDef *command,
  u32 o_flags,
  u32 opcode,
  u32 address,
  int data_size,
  int dummy_cycles) {

  if (o_flags & QSPI_FLAG_IS_OPCODE_WRITE) {
    command->Instruction = opcode;
    command->InstructionMode = QSPI_INSTRUCTION_1_LINE;
    if (o_flags & QSPI_FLAG_IS_OPCODE_DUAL) {
      command->InstructionMode = QSPI_INSTRUCTION_2_LINES;
    } else if (o_flags & QSPI_FLAG_IS_OPCODE_QUAD) {
      command->InstructionMode = QSPI_INSTRUCTION_4_LINES;
    }
  } else {
    command->InstructionMode = QSPI_INSTRUCTION_NONE;
  }

  if (o_flags & QSPI_FLAG_IS_ADDRESS_WRITE) {
    command->Address = address;
    command->AddressSize = QSPI_ADDRESS_8_BITS;
    command->AddressMode = QSPI_ADDRESS_1_LINE;
    if (o_flags & QSPI_FLAG_IS_ADDRESS_DUAL) {
      command->AddressMode = QSPI_ADDRESS_2_LINES;
    } else if (o_flags & QSPI_FLAG_IS_ADDRESS_QUAD) {
      command->AddressMode = QSPI_ADDRESS_4_LINES;
    }

    if (o_flags & QSPI_FLAG_IS_ADDRESS_16_BITS) {
      command->AddressSize = QSPI_ADDRESS_16_BITS;
    } else if (o_flags & QSPI_FLAG_IS_ADDRESS_24_BITS) {
      command->AddressSize = QSPI_ADDRESS_24_BITS;
    } else if (o_flags & QSPI_FLAG_IS_ADDRESS_32_BITS) {
      command->AddressSize = QSPI_ADDRESS_32_BITS;
    }
  } else {
    command->AddressMode = QSPI_ADDRESS_NONE;
  }

  // not using alternate bytes
  command->AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command->AlternateBytes = 0; // up to 32 bits can be sent
  command->AlternateBytesSize
    = QSPI_ALTERNATE_BYTES_8_BITS; // 8,16,24 or 32 bits

  // setup the data mode
  command->DataMode = QSPI_DATA_NONE;
  command->NbData = data_size;
  if (data_size > 0) {
    command->DataMode = QSPI_DATA_1_LINE;
    if (o_flags & QSPI_FLAG_IS_DATA_DUAL) {
      command->DataMode = QSPI_DATA_2_LINES;
    } else if (o_flags & QSPI_FLAG_IS_DATA_QUAD) {
      command->DataMode = QSPI_DATA_4_LINES;
    }
  }

  command->DummyCycles = dummy_cycles;

  // no double data rate
  command->DdrMode = QSPI_DDR_MODE_DISABLE;
  command->DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command->SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
}

int qspi_local_execcommand(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(qspi);

  qspi_command_t *qspi_command = ctl;
  u32 o_flags = qspi_command->o_flags;

  QSPI_CommandTypeDef command;

#if 1
  populate_command(
    &command,
    qspi_command->o_flags,
    qspi_command->opcode,
    qspi_command->address,
    qspi_command->data_size,
    qspi_command->dummy_cycles);

#else
  if (o_flags & QSPI_FLAG_IS_OPCODE_WRITE) {
    command.Instruction = qspi_command->opcode;
    command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    if (o_flags & QSPI_FLAG_IS_OPCODE_DUAL) {
      command.InstructionMode = QSPI_INSTRUCTION_2_LINES;
    } else if (o_flags & QSPI_FLAG_IS_OPCODE_QUAD) {
      command.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    }
  } else {
    command.InstructionMode = QSPI_INSTRUCTION_NONE;
  }

  if (o_flags & QSPI_FLAG_IS_ADDRESS_WRITE) {
    command.Address = qspi_command->address;
    command.AddressSize = QSPI_ADDRESS_8_BITS;
    command.AddressMode = QSPI_ADDRESS_1_LINE;
    if (o_flags & QSPI_FLAG_IS_ADDRESS_DUAL) {
      command.AddressMode = QSPI_ADDRESS_2_LINES;
    } else if (o_flags & QSPI_FLAG_IS_ADDRESS_QUAD) {
      command.AddressMode = QSPI_ADDRESS_4_LINES;
    }

    if (o_flags & QSPI_FLAG_IS_ADDRESS_16_BITS) {
      command.AddressSize = QSPI_ADDRESS_16_BITS;
    } else if (o_flags & QSPI_FLAG_IS_ADDRESS_24_BITS) {
      command.AddressSize = QSPI_ADDRESS_24_BITS;
    } else if (o_flags & QSPI_FLAG_IS_ADDRESS_32_BITS) {
      command.AddressSize = QSPI_ADDRESS_32_BITS;
    }
  } else {
    command.AddressMode = QSPI_ADDRESS_NONE;
  }

  // not using alternate bytes
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command.AlternateBytes = 0; // up to 32 bits can be sent
  command.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS; // 8,16,24 or 32
                                                            // bits

  // setup the data mode
  command.DataMode = QSPI_DATA_NONE;
  command.NbData = qspi_command->data_size;
  if (qspi_command->data_size > 0) {
    command.DataMode = QSPI_DATA_1_LINE;
    if (o_flags & QSPI_FLAG_IS_DATA_DUAL) {
      command.DataMode = QSPI_DATA_2_LINES;
    } else if (o_flags & QSPI_FLAG_IS_DATA_QUAD) {
      command.DataMode = QSPI_DATA_4_LINES;
    }
  }

  command.DummyCycles = qspi_command->dummy_cycles;

  // no double data rate
  command.DdrMode = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
#endif

  int result;
  if (
    (result = HAL_QSPI_Command(
       &state->hal_handle,
       &command,
       HAL_QPSI_TIMEOUT_DEFAULT_VALUE))
    != HAL_OK) {
    state->transfer_handler.write = 0;
    return SYSFS_SET_RETURN(EIO);
  }

  if (command.NbData > 0) {
    if (o_flags & QSPI_FLAG_IS_DATA_READ) {
      if (
        HAL_QSPI_Receive(
          &state->hal_handle,
          qspi_command->data,
          HAL_QPSI_TIMEOUT_DEFAULT_VALUE)
        != HAL_OK) {
        state->transfer_handler.read = 0;
        return SYSFS_SET_RETURN(EIO);
      }
    } else if (o_flags & QSPI_FLAG_IS_DATA_WRITE) {
      if (
        HAL_QSPI_Transmit(
          &state->hal_handle,
          qspi_command->data,
          HAL_QPSI_TIMEOUT_DEFAULT_VALUE)
        != HAL_OK) {
        return SYSFS_SET_RETURN(EIO);
      }
    }
  }

  return 0;
}

void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hqspi) {
  sos_debug_printf("error 0x%X\n", hqspi->ErrorCode);
  qspi_state_t *state = (qspi_state_t *)hqspi;
  devfs_execute_cancel_handler(
    &state->transfer_handler,
    0,
    SYSFS_SET_RETURN(EIO),
    MCU_EVENT_FLAG_ERROR);
}

void HAL_QSPI_AbortCpltCallback(QSPI_HandleTypeDef *hqspi) {
  sos_debug_printf("abort\n");
  qspi_state_t *state = (qspi_state_t *)hqspi;
  devfs_execute_cancel_handler(
    &state->transfer_handler,
    0,
    SYSFS_SET_RETURN(ECANCELED),
    MCU_EVENT_FLAG_CANCELED);
}

void HAL_QSPI_FifoThresholdCallback(QSPI_HandleTypeDef *hqspi) {}

void HAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *hqspi) {
  int ret;
  qspi_state_t *state = (qspi_state_t *)hqspi;
  if (state->transfer_handler.read) {
    // command is the start of a read operation -- complete the read
    // ret = HAL_QSPI_Receive_IT(hqspi, state->transfer_handler.read->buf);
  } else if (state->transfer_handler.write) {
    // ret = HAL_QSPI_Transmit_IT(hqspi, state->transfer_handler.write->buf);
  }

  if (ret != HAL_OK) {
    // there was an error -- execute the callback
  }
}

void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi) {
  qspi_state_t *state = (qspi_state_t *)hqspi;
  if (
#if MCU_QSPI_API == 1
    state->hal_handle.hmdma != 0 &&
#else
    state->hal_handle.hdma != 0 &&
#endif
    state->transfer_handler.read) {
    // pull in values from memory to cache if using DMA

    /*
     * reads must be aligned to cache lines
     *
     */
    sos_config.cache.invalidate_data_block(
      state->transfer_handler.read->buf,
      state->transfer_handler.read->nbyte + 31);
  }
  devfs_execute_read_handler(
    &state->transfer_handler,
    0,
    hqspi->RxXferCount,
    MCU_EVENT_FLAG_DATA_READY);
}

void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi) {
  qspi_state_t *state = (qspi_state_t *)hqspi;
  devfs_execute_write_handler(
    &state->transfer_handler,
    0,
    hqspi->TxXferCount,
    MCU_EVENT_FLAG_WRITE_COMPLETE);
}

void HAL_QSPI_RxHalfCpltCallback(QSPI_HandleTypeDef *hqspi) {
  // this is for DMA only
}

void HAL_QSPI_TxHalfCpltCallback(QSPI_HandleTypeDef *hqspi) {
  // this is for DMA only
}

void mcu_core_quadspi_isr() {
  HAL_QSPI_IRQHandler(&m_qspi_state_list[0]->hal_handle);
}

#endif
