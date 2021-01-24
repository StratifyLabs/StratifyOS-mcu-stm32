// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include "sai_local.h"
#include <mcu/i2s.h>

#if MCU_SAI_PORTS > 0

sai_state_t *m_sai_state_list[MCU_SAI_PORTS] MCU_SYS_MEM;

SAI_Block_TypeDef *const sai_regs[MCU_SAI_PORTS] = MCU_SAI_REGS;
u8 const sai_irqs[MCU_SAI_PORTS] = MCU_SAI_IRQS;

int sai_local_open(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sai);
  if (state->ref_count == 0) {
    DEVFS_DRIVER_OPEN_STATE_LOCAL_V4(sai);
    u8 irq_number = 0;
    // turn on RCC clock
    switch (config->port) {
    case 0:
    case 1:
      __HAL_RCC_SAI1_CLK_ENABLE();
      irq_number = 0;
      break;
#if defined SAI2
    case 2:
    case 3:
      sos_debug_log_info(SOS_DEBUG_DEVICE, "Turn on SAI2 clock\n");
      __HAL_RCC_SAI2_CLK_ENABLE();
      irq_number = 1;
      break;
#endif
#if defined SAI3
    case 4:
    case 5:
      sos_debug_log_info(SOS_DEBUG_DEVICE, "Turn on SAI2 clock\n");
      __HAL_RCC_SAI3_CLK_ENABLE();
      irq_number = 1;
      break;
#endif
#if defined SAI4
    case 6:
    case 7:
      sos_debug_log_info(SOS_DEBUG_DEVICE, "Turn on SAI2 clock\n");
      __HAL_RCC_SAI4_CLK_ENABLE();
      irq_number = 1;
      break;
#endif
    }
    state->transfer_handler.read = NULL;
    state->transfer_handler.write = NULL;
    state->hal_handle.Instance = sai_regs[config->port];
    cortexm_enable_irq(sai_irqs[irq_number]);
  }
  state->ref_count++;
  return 0;
}

int sai_local_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sai);
  if (state->ref_count > 0) {
    if (state->ref_count == 1) {
      __HAL_SAI_DISABLE(&state->hal_handle);
      HAL_SAI_DeInit(&state->hal_handle);
      cortexm_disable_irq(sai_irqs[config->port]);
      devfs_execute_cancel_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(EDEADLK),
        MCU_EVENT_FLAG_CANCELED);
      // turn off RCC clock
      switch (config->port) {
      case 0:
      case 1:
        __HAL_RCC_SAI1_CLK_DISABLE();
        break;
#if defined SAI2
      case 2:
      case 3:
        __HAL_RCC_SAI2_CLK_DISABLE();
        break;
#endif
#if defined SAI3
      case 4:
      case 5:
        __HAL_RCC_SAI3_CLK_DISABLE();
        break;
#endif
#if defined SAI4
      case 6:
      case 7:
        __HAL_RCC_SAI4_CLK_DISABLE();
        break;
#endif
      }
      DEVFS_DRIVER_CLOSE_STATE_LOCAL_V4(sai);
    }
    state->ref_count--;
  }
  return 0;
}

int sai_local_mute(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sai);
  MCU_UNUSED_ARGUMENT(ctl);
  HAL_SAI_EnableTxMuteMode(&state->hal_handle, 0);
  return SYSFS_SET_RETURN(ENOTSUP);
}

int sai_local_unmute(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sai);
  MCU_UNUSED_ARGUMENT(ctl);
  // HAL_SAI_DisableRxMuteMode()
  HAL_SAI_DisableTxMuteMode(&state->hal_handle);
  return SYSFS_SET_RETURN(ENOTSUP);
}

int sai_local_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sai);
  const i2s_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(i2s, ctl);
  if (attr == 0) {
    return SYSFS_SET_RETURN(EINVAL);
  }
  const u32 o_flags = attr->o_flags;
  // set I2S Flags
  if (o_flags & (I2S_FLAG_SET_MASTER | I2S_FLAG_SET_SLAVE)) {
    state->o_flags |= SAI_LOCAL_IS_I2S;
    // state->hal_handle.Init.AudioMode = 0; //handled below
    if (o_flags & I2S_FLAG_SET_SLAVE) {
      state->hal_handle.Init.AudioMode = SAI_MODESLAVE_TX;
      if (o_flags & I2S_FLAG_IS_RECEIVER) {
        state->hal_handle.Init.AudioMode = SAI_MODESLAVE_RX;
      }
    } else {
      state->hal_handle.Init.AudioMode = SAI_MODEMASTER_TX;
      if (o_flags & I2S_FLAG_IS_RECEIVER) {
        state->hal_handle.Init.AudioMode = SAI_MODEMASTER_RX;
      }
    }
    /*
     * Slave mode synchronous -- receives signals from another SAI block
     *   asynchronous -- receives signals from external pins
     *
     * For master mode, this value doesn't seem to matter
     *
     */
    state->hal_handle.Init.Synchro
      = SAI_ASYNCHRONOUS; // synchronous means it should receive signals from
                          // another SAI unit or sub block
    if (o_flags & I2S_FLAG_IS_SYNCHRONOUS) {
      state->hal_handle.Init.Synchro = SAI_SYNCHRONOUS;
    } else if (o_flags & I2S_FLAG_IS_SYNCHRONOUS_EXT_SAI1) {
      state->hal_handle.Init.Synchro = SAI_SYNCHRONOUS_EXT_SAI1;
    } else if (o_flags & I2S_FLAG_IS_SYNCHRONOUS_EXT_SAI2) {
      state->hal_handle.Init.Synchro = SAI_SYNCHRONOUS_EXT_SAI2;
    }
    // SAI_SYNCEXT_OUTBLOCKA_ENABLE -- sync with block A of other SAI unit
    // SAI_SYNCEXT_OUTBLOCKB_ENABLE
    state->hal_handle.Init.SynchroExt
      = SAI_SYNCEXT_DISABLE; // this means synchronize with another SAI unit
                             // (not a sub block in teh same unit)
    // this will probably be SAI_OUTPUTDRIVE_ENABLE to drive pins
    state->hal_handle.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
    if (o_flags & I2S_FLAG_IS_OUTPUTDRIVE_DISABLE) {
      state->hal_handle.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
    }
    // SAI_MASTERDIVIDER_ENABLE
    state->hal_handle.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
    // pick a value here that covers most cases
    state->hal_handle.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_HF;
    if (o_flags & I2S_FLAG_IS_FIFOTHRESHOLD_EMPTY) {
      state->hal_handle.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
    } else if (o_flags & I2S_FLAG_IS_FIFOTHRESHOLD_1QF) {
      state->hal_handle.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
    } else if (o_flags & I2S_FLAG_IS_FIFOTHRESHOLD_3QF) {
      state->hal_handle.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_3QF;
    } else if (o_flags & I2S_FLAG_IS_FIFOTHRESHOLD_FULL) {
      state->hal_handle.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_FULL;
    }

    u32 frequency = SAI_AUDIO_FREQUENCY_8K;
    switch (attr->freq) {
    case 11000:
      frequency = SAI_AUDIO_FREQUENCY_11K;
      break;
    case 16000:
      frequency = SAI_AUDIO_FREQUENCY_16K;
      break;
    case 22050:
      frequency = SAI_AUDIO_FREQUENCY_22K;
      break;
    case 32000:
      frequency = SAI_AUDIO_FREQUENCY_32K;
      break;
    case 44100:
      frequency = SAI_AUDIO_FREQUENCY_44K;
      break;
    case 48000:
      frequency = SAI_AUDIO_FREQUENCY_48K;
      break;
    case 96000:
      frequency = SAI_AUDIO_FREQUENCY_96K;
      break;
    case 192000:
      frequency = SAI_AUDIO_FREQUENCY_192K;
      break;
    default:
      return SYSFS_SET_RETURN(EINVAL);
    }
    state->hal_handle.Init.AudioFrequency = frequency;
    // state->hal_handle.Init.AudioFrequency = 0; //handled below
    // 0 to 63 or 0 to 15 depending on the device
    // this value is set based on the call to the HAL libraries and the audio
    // frequency
    state->hal_handle.Init.Mckdiv = 0;
#if defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx)       \
  || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)
    state->hal_handle.Init.MckOverSampling = 0;
    state->hal_handle.Init.PdmInit = 0;
#endif
    state->hal_handle.Init.MonoStereoMode = SAI_STEREOMODE;
    if (o_flags & I2S_FLAG_IS_MONO) {
      state->hal_handle.Init.MonoStereoMode = SAI_MONOMODE;
    }
    state->hal_handle.Init.CompandingMode = SAI_NOCOMPANDING;
    if (o_flags & I2S_FLAG_IS_ULAW_1CPL_COMPANDING) {
      state->hal_handle.Init.CompandingMode = SAI_ULAW_1CPL_COMPANDING;
    } else if (o_flags & I2S_FLAG_IS_ALAW_1CPL_COMPANDING) {
      state->hal_handle.Init.FIFOThreshold = SAI_ALAW_1CPL_COMPANDING;
    } else if (o_flags & I2S_FLAG_IS_ULAW_2CPL_COMPANDING) {
      state->hal_handle.Init.FIFOThreshold = SAI_ULAW_2CPL_COMPANDING;
    } else if (o_flags & I2S_FLAG_IS_ALAW_2CPL_COMPANDING) {
      state->hal_handle.Init.FIFOThreshold = SAI_ALAW_2CPL_COMPANDING;
    }
    // OR SAI_OUTPUT_RELEASED
    state->hal_handle.Init.TriState = SAI_OUTPUT_NOTRELEASED;
    // slot
    u32 slot_count = 0;
    for (u8 i = 0; i < 32; i++) {
      if (attr->slot & (((u32)1) << i)) {
        slot_count++;
      }
    }
    if (!slot_count) {
      slot_count = 2;
    }

    u32 protocol = SAI_I2S_STANDARD;
    if (o_flags & I2S_FLAG_IS_FORMAT_MSB) {
      protocol = SAI_I2S_MSBJUSTIFIED;
    } else if (o_flags & I2S_FLAG_IS_FORMAT_LSB) {
      protocol = SAI_I2S_LSBJUSTIFIED;
    } else if (o_flags & I2S_FLAG_IS_FORMAT_PCM_SHORT) {
      protocol = SAI_PCM_SHORT;
    } else if (o_flags & I2S_FLAG_IS_FORMAT_PCM_LONG) {
      protocol = SAI_PCM_LONG;
    }

    u32 data_size = SAI_PROTOCOL_DATASIZE_16BIT;
    state->size_mult = 2;
    if (o_flags & I2S_FLAG_IS_WIDTH_24) {
      data_size = SAI_PROTOCOL_DATASIZE_24BIT;
      state->size_mult = 4;
    } else if (o_flags & I2S_FLAG_IS_WIDTH_32) {
      data_size = SAI_PROTOCOL_DATASIZE_32BIT;
      state->size_mult = 4;
    } else if (o_flags & I2S_FLAG_IS_WIDTH_16_EXTENDED) {
      data_size = SAI_PROTOCOL_DATASIZE_16BITEXTENDED;
    }
    if (
      mcu_set_pin_assignment(
        &(attr->pin_assignment),
        MCU_CONFIG_PIN_ASSIGNMENT(i2s_config_t, handle),
        MCU_PIN_ASSIGNMENT_COUNT(i2s_pin_assignment_t),
        CORE_PERIPH_I2S,
        config->port,
        0,
        0,
        0)
      < 0) {
      return SYSFS_SET_RETURN(EINVAL);
    }
#if 0
    // state->hal_handle.Instance = SAI2_Block_A;
    state->hal_handle.Init.Protocol = SAI_FREE_PROTOCOL;
    state->hal_handle.Init.AudioMode = SAI_MODEMASTER_RX;
    state->hal_handle.Init.DataSize = SAI_DATASIZE_16;
    state->hal_handle.Init.FirstBit = SAI_FIRSTBIT_MSB;
    state->hal_handle.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
    state->hal_handle.Init.Synchro = SAI_ASYNCHRONOUS;
    state->hal_handle.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
    state->hal_handle.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
    state->hal_handle.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
    state->hal_handle.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_16K;
    state->hal_handle.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
    state->hal_handle.Init.MonoStereoMode = SAI_STEREOMODE;
    state->hal_handle.Init.CompandingMode = SAI_NOCOMPANDING;
    state->hal_handle.FrameInit.FrameLength = 16;
    state->hal_handle.FrameInit.ActiveFrameLength = 1;
    state->hal_handle.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
    state->hal_handle.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
    state->hal_handle.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
    state->hal_handle.SlotInit.FirstBitOffset = 0;
    state->hal_handle.SlotInit.SlotSize = SAI_SLOTSIZE_16B;
    state->hal_handle.SlotInit.SlotNumber = 1;
    state->hal_handle.SlotInit.SlotActive = 0x00000001;
    int sai_result;
    if ((sai_result = HAL_SAI_Init(&state->hal_handle)) != HAL_OK) {
      sos_debug_log_error(
        SOS_DEBUG_DEVICE,
        "Failed to init SAI %d",
        sai_result);
      return SYSFS_SET_RETURN(EIO);
    }
#else
    sos_debug_log_info(
      SOS_DEBUG_DEVICE,
      "SAI: %d %d %d",
      protocol,
      data_size,
      slot_count);
    int sai_result;
    if (
      (sai_result = HAL_SAI_InitProtocol(
         &state->hal_handle,
         protocol,
         data_size,
         slot_count))
      != HAL_OK) {
      sos_debug_log_error(
        SOS_DEBUG_DEVICE,
        "Failed to init SAI %d",
        sai_result);
      return SYSFS_SET_RETURN(EIO);
    }

#endif
  } else if (o_flags & I2S_FLAG_SET_SLOT) {
    __HAL_SAI_DISABLE(&state->hal_handle);
    /* Update the SAI audio frame slot configuration */
    state->hal_handle.SlotInit.SlotActive = attr->slot;
    HAL_SAI_Init(&state->hal_handle);

    /* Enable SAI peripheral to generate MCLK */
    __HAL_SAI_ENABLE(&state->hal_handle);
  }
  if (o_flags & I2S_FLAG_ENABLE) {
    __HAL_SAI_ENABLE(&state->hal_handle);
  }

  return SYSFS_RETURN_SUCCESS;
}

int sai_local_setaction(
  const devfs_handle_t *handle,
  void *ctl,
  int interrupt_number) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sai);
  const mcu_action_t *action = ctl;
  if (action->handler.callback) {
    state->special_event_handler.callback = action->handler.callback;
    state->special_event_handler.context = action->handler.context;
  }
  cortexm_set_irq_priority(interrupt_number, action->prio, action->o_events);
  return 0;
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
  sai_state_t *state = (sai_state_t *)hsai;
  int result;
  devfs_async_t *async;

  async = state->transfer_handler.write;
  result = devfs_execute_write_handler(
    &state->transfer_handler,
    0,
    0,
    MCU_EVENT_FLAG_LOW | MCU_EVENT_FLAG_HALF_TRANSFER);
  if (result) {
    state->transfer_handler.write = async;
  } else {
    // stop -- half transfer only happens on DMA
    if (state->hal_handle.hdmatx) {
      HAL_SAI_DMAStop(&state->hal_handle);
    }
  }
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai) {
  sai_state_t *state = (sai_state_t *)hsai;
  int result;
  devfs_async_t *async;
  async = state->transfer_handler.write;
  result = devfs_execute_write_handler(
    &state->transfer_handler,
    0,
    0, // zero means leave nbyte value alone
    MCU_EVENT_FLAG_HIGH | MCU_EVENT_FLAG_WRITE_COMPLETE);

  if (result) {
    state->transfer_handler.write = async;
  } else {
    // stop
    if (hsai->hdmatx) {
      HAL_SAI_DMAStop(&state->hal_handle);
    }
  }
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
  // no action when half complete -- could fire an event
  sai_state_t *state = (sai_state_t *)hsai;
  int result;
  devfs_async_t *async;

  async = state->transfer_handler.read;
  result = devfs_execute_read_handler(
    &state->transfer_handler,
    0,
    0,
    MCU_EVENT_FLAG_LOW | MCU_EVENT_FLAG_HALF_TRANSFER);

  if (result) {
    state->transfer_handler.read = async;
  } else {
    if (hsai->hdmarx) {
      HAL_SAI_DMAStop(&state->hal_handle);
    }
  }
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai) {
  sai_state_t *state = (sai_state_t *)hsai;
  int result;
  devfs_async_t *async;

  async = state->transfer_handler.read;
  result = devfs_execute_read_handler(
    &state->transfer_handler,
    0,
    async->nbyte / 2,
    MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_HIGH);

  if (result) {
    state->transfer_handler.read = async;
  } else if (hsai->hdmarx) {
    HAL_SAI_DMAStop(&state->hal_handle);
  }
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai) {
  // called on overflow and underrun
  volatile u32 status = hsai->Instance->SR;
  MCU_UNUSED_ARGUMENT(status);
  status = hsai->Instance->DR;
  sos_debug_log_error(
    SOS_DEBUG_DEVICE,
    "SAI Error %d on %p",
    hsai->ErrorCode,
    hsai->Instance);
  // devfs_execute_cancel_handler(&state->transfer_handler, (void*)&status,
  // SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_ERROR);
}

#if defined SAI1
void mcu_core_sai1_isr() {
  if (m_sai_state_list[0]->hal_handle.State != HAL_SAI_STATE_RESET) {
    HAL_SAI_IRQHandler(&m_sai_state_list[0]->hal_handle);
  }
  if (m_sai_state_list[1]->hal_handle.State != HAL_SAI_STATE_RESET) {
    HAL_SAI_IRQHandler(&m_sai_state_list[1]->hal_handle);
  }
}
#endif

#if defined SAI2
void mcu_core_sai2_isr() {
  if (m_sai_state_list[2]->hal_handle.State != HAL_SAI_STATE_RESET) {
    HAL_SAI_IRQHandler(&m_sai_state_list[2]->hal_handle);
  }
  if (m_sai_state_list[3]->hal_handle.State != HAL_SAI_STATE_RESET) {
    HAL_SAI_IRQHandler(&m_sai_state_list[3]->hal_handle);
  }
}
#endif

#if defined SAI3
void mcu_core_sai3_isr() {
  if (m_sai_state_list[4]->hal_handle.State != HAL_SAI_STATE_RESET) {
    HAL_SAI_IRQHandler(&m_sai_state_list[2]->hal_handle);
  }
  if (m_sai_state_list[5]->hal_handle.State != HAL_SAI_STATE_RESET) {
    HAL_SAI_IRQHandler(&m_sai_state_list[3]->hal_handle);
  }
}
#endif

#if defined SAI4
void mcu_core_sai4_isr() {
  if (m_sai_state_list[6]->hal_handle.State != HAL_SAI_STATE_RESET) {
    HAL_SAI_IRQHandler(&m_sai_state_list[2]->hal_handle);
  }
  if (m_sai_state_list[7]->hal_handle.State != HAL_SAI_STATE_RESET) {
    HAL_SAI_IRQHandler(&m_sai_state_list[3]->hal_handle);
  }
}
#endif

#endif
