// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include "mmc_local.h"

#if MCU_SDIO_PORTS > 0 && defined MMC_TypeDef

SDIO_TypeDef *const mmc_regs[MCU_SDIO_PORTS] = MCU_SDIO_REGS;
const int mmc_irqs[MCU_SDIO_PORTS] = MCU_SDIO_IRQS;
mmc_state_t *m_mmc_state_list[MCU_SDIO_PORTS] MCU_SYS_MEM;

int mmc_local_open(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(mmc);

  if (config->port < MCU_SDIO_PORTS) {
    if (state->ref_count == 0) {
      DEVFS_DRIVER_OPEN_STATE_LOCAL_V4(mmc);
      // turn on RCC clock
      switch (config->port) {
      case 0:
#if defined __HAL_RCC_SDIO_CLK_ENABLE
        __HAL_RCC_SDIO_CLK_ENABLE();
#endif
#if defined __HAL_RCC_SDMMC1_CLK_ENABLE
        __HAL_RCC_SDMMC1_CLK_ENABLE();
#endif
        break;
      case 1:
#if defined __HAL_RCC_SDMMC2_CLK_ENABLE
        __HAL_RCC_SDMMC2_CLK_ENABLE();
#endif
        break;
      }
      state->transfer_handler.read = NULL;
      state->transfer_handler.write = NULL;
      state->hal_handle.Instance = mmc_regs[config->port];
      cortexm_enable_irq(mmc_irqs[config->port]);
    }
    state->ref_count++;
  }
  return 0;
}

int mmc_local_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(mmc);

  if (state->ref_count > 0) {
    // do the opposite of mmc_local_open() -- ref_count is zero -- turn off
    // interrupt
    if (state->ref_count == 1) {
      HAL_MMC_DeInit(&state->hal_handle);
      switch (config->port) {
      case 0:
#if defined __HAL_RCC_SDIO_CLK_DISABLE
        __HAL_RCC_SDIO_CLK_DISABLE();
#endif
#if defined __HAL_RCC_SDMMC1_CLK_DISABLE
        __HAL_RCC_SDMMC1_CLK_DISABLE();
#endif
        break;
      case 1:
#if defined __HAL_RCC_SDMMC2_CLK_DISABLE
        __HAL_RCC_SDMMC2_CLK_DISABLE();
#endif
        break;
      }

      state->transfer_handler.read = NULL;
      state->transfer_handler.write = NULL;
      state->hal_handle.Instance = 0;
      cortexm_disable_irq(mmc_irqs[config->port]);
      DEVFS_DRIVER_CLOSE_STATE_LOCAL_V4(mmc);
    }
    state->ref_count--;
  }

  return 0;
}

int mmc_local_getinfo(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(mmc);
  mmc_info_t *info = ctl;
  // const u32 port = config->port;

  // set flags that are supported by this driver
  info->o_flags = MMC_FLAG_SET_INTERFACE;
  info->o_events = MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_WRITE_COMPLETE
                   | MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR
                   | MCU_EVENT_FLAG_SET_PRIORITY;

  info->freq = 25000000UL;
  info->block_count = state->hal_handle.MmcCard.BlockNbr;
  info->block_size = state->hal_handle.MmcCard.BlockSize;
  info->card_class = state->hal_handle.MmcCard.Class;
  info->logical_block_count = state->hal_handle.MmcCard.LogBlockNbr;
  info->logical_block_size = state->hal_handle.MmcCard.LogBlockSize;
  info->type = state->hal_handle.MmcCard.CardType;
  info->relative_address = state->hal_handle.MmcCard.RelCardAdd;
  info->version = 0;

  return SYSFS_RETURN_SUCCESS;
}

int mmc_local_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(mmc);

  const mmc_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(mmc, ctl);
  if (attr == NULL) {
    return SYSFS_SET_RETURN(ENOSYS);
  }

  u32 o_flags = attr->o_flags;

  if (o_flags & MMC_FLAG_SET_INTERFACE) {

    if (o_flags & MMC_FLAG_IS_BYTE_ADDRESSING) {
      state->o_flags = MMC_LOCAL_FLAG_IS_BYTE_ADDRESSING;
    }

    // SDIO_CLOCK_EDGE_RISING
    // SDIO_CLOCK_EDGE_FALLING
    state->hal_handle.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    if (o_flags & MMC_FLAG_IS_CLOCK_FALLING) {
      state->hal_handle.Init.ClockEdge = SDIO_CLOCK_EDGE_FALLING;
    }

    // SDIO_CLOCK_BYPASS_DISABLE
    // SDIO_CLOCK_BYPASS_ENABLE
#if defined SDIO_CLOCK_BYPASS_DISABLE
    state->hal_handle.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
    if (o_flags & MMC_FLAG_IS_CLOCK_BYPASS_ENABLED) {
      state->hal_handle.Init.ClockBypass = SDIO_CLOCK_BYPASS_ENABLE;
    }
#endif

    // SDIO_CLOCK_POWER_SAVE_DISABLE
    // SDIO_CLOCK_POWER_SAVE_ENABLE
    state->hal_handle.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    if (o_flags & MMC_FLAG_IS_CLOCK_POWER_SAVE_ENABLED) {
      state->hal_handle.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_ENABLE;
    }

    // initialize using 1B bus
    state->hal_handle.Init.BusWide = SDIO_BUS_WIDE_1B;

    // MMC_HARDWARE_FLOW_CONTROL_DISABLE
    // SDIO_HARDWARE_FLOW_CONTROL_ENABLE
    state->hal_handle.Init.HardwareFlowControl
      = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    if (o_flags & MMC_FLAG_IS_HARDWARE_FLOW_CONTROL_ENABLED) {
      state->hal_handle.Init.HardwareFlowControl
        = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
    }

    // must be <= 255
    state->hal_handle.Init.ClockDiv = 0;
    if (attr->freq && (attr->freq < 25000000UL)) {
      u32 divider_value = 48000000UL / attr->freq;
      if (divider_value > 2) {
        divider_value -= 2;
      } else {
        divider_value = 0;
      }
      state->hal_handle.Init.ClockDiv = divider_value;
    }

    // pin assignments
    if (
      mcu_set_pin_assignment(
        &(attr->pin_assignment),
        MCU_CONFIG_PIN_ASSIGNMENT(mmc_config_t, handle),
        MCU_PIN_ASSIGNMENT_COUNT(mmc_pin_assignment_t),
        CORE_PERIPH_SDIO,
        config->port,
        0,
        0,
        0)
      < 0) {
      return SYSFS_SET_RETURN(EINVAL);
    }

    if (HAL_MMC_Init(&state->hal_handle) != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }

    // SDIO_BUS_WIDE_1B -- set as default for initialziation
    // SDIO_BUS_WIDE_4B
    // SDIO_BUS_WIDE_8B -- not compatible with SDIO
    if (o_flags & MMC_FLAG_IS_BUS_WIDTH_4) {
      state->hal_handle.Init.BusWide = SDIO_BUS_WIDE_4B;
    } else if (o_flags & MMC_FLAG_IS_BUS_WIDTH_8) {
      state->hal_handle.Init.BusWide = SDIO_BUS_WIDE_8B;
    }

    if (state->hal_handle.Init.BusWide != SDIO_BUS_WIDE_1B) {
      if (
        HAL_MMC_ConfigWideBusOperation(
          &state->hal_handle,
          state->hal_handle.Init.BusWide)
        != HAL_OK) {
        return SYSFS_SET_RETURN(EIO);
      }
    }
  }

  if (o_flags & MMC_FLAG_GET_CARD_STATE) {
    return HAL_MMC_GetCardState(&state->hal_handle);
  }

  if (o_flags & MMC_FLAG_RESET) {
    // if the EMMC has a timeout error, this will get it to recover from that
    u32 width = state->hal_handle.Init.BusWide;
    sos_debug_log_info(SOS_DEBUG_DEVICE, "Reset EMMC");
    HAL_MMC_Abort(&state->hal_handle);
    HAL_MMC_DeInit(&state->hal_handle);
    if (HAL_MMC_Init(&state->hal_handle) != HAL_OK) {
      sos_debug_log_error(SOS_DEBUG_DEVICE, "failed to reset MMC");
      return SYSFS_SET_RETURN(EIO);
    }

    HAL_MMC_ConfigWideBusOperation(&state->hal_handle, width);
  }

  if (o_flags & MMC_FLAG_ERASE_BLOCKS) {
    if (HAL_MMC_Erase(&state->hal_handle, attr->start, attr->end) != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }
  }

  return SYSFS_RETURN_SUCCESS;
}

int mmc_local_setaction(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(mmc);
  mcu_action_t *action = ctl;

  if (action->handler.callback == 0) {
    if (
      action->o_events
      & (MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_WRITE_COMPLETE)) {
      if (state->o_flags & MMC_LOCAL_FLAG_IS_DMA) {
        HAL_MMC_Abort(&state->hal_handle);
      } else {
        HAL_MMC_Abort_IT(&state->hal_handle);
      }
    }

    if (action->o_events & MCU_EVENT_FLAG_DATA_READY) {
      devfs_execute_read_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(EIO),
        MCU_EVENT_FLAG_CANCELED);
    }

    if (action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE) {
      devfs_execute_write_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(EIO),
        MCU_EVENT_FLAG_CANCELED);
    }
  }

  cortexm_set_irq_priority(
    mmc_irqs[config->port],
    action->prio,
    action->o_events);
  return 0;
}

int mmc_local_getcid(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(mmc);
  if (HAL_MMC_GetCardCID(&state->hal_handle, ctl) == HAL_OK) {
    return SYSFS_RETURN_SUCCESS;
  }

  return SYSFS_SET_RETURN(EIO);
}

int mmc_local_getcsd(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(mmc);
  if (HAL_MMC_GetCardCSD(&state->hal_handle, ctl) == HAL_OK) {
    return SYSFS_RETURN_SUCCESS;
  }

  return SYSFS_SET_RETURN(EIO);
}

int mmc_local_getstatus(const devfs_handle_t *handle, void *ctl) {
  MCU_UNUSED_ARGUMENT(handle);
  MCU_UNUSED_ARGUMENT(handle);
  return SYSFS_SET_RETURN(ENOTSUP);
}

void HAL_MMC_TxCpltCallback(MMC_HandleTypeDef *hmmc) {
  mmc_state_t *state = (mmc_state_t *)hmmc;
  devfs_execute_write_handler(
    &state->transfer_handler,
    0,
    hmmc->TxXferSize,
    MCU_EVENT_FLAG_WRITE_COMPLETE);
}

void HAL_MMC_RxCpltCallback(MMC_HandleTypeDef *hmmc) {
  mmc_state_t *state = (mmc_state_t *)hmmc;
  devfs_execute_read_handler(
    &state->transfer_handler,
    0,
    hmmc->RxXferSize,
    MCU_EVENT_FLAG_DATA_READY);
}

void HAL_MMC_ErrorCallback(MMC_HandleTypeDef *hmmc) {
  mmc_state_t *state = (mmc_state_t *)hmmc;
  // sos_debug_log_warning(SOS_DEBUG_DEVICE, "MMC Error? 0x%lX 0x%lX 0x%lX",
  // hmmc->ErrorCode, hmmc->hdmatx->ErrorCode, hmmc->hdmarx->ErrorCode);
  if (hmmc->ErrorCode) {
    sos_debug_log_warning(
      SOS_DEBUG_DEVICE,
      "MMC Error? 0x%lX",
      hmmc->ErrorCode);
    devfs_execute_cancel_handler(
      &state->transfer_handler,
      0,
      SYSFS_SET_RETURN(EIO),
      MCU_EVENT_FLAG_ERROR);
  }
}

void HAL_MMC_AbortCallback(MMC_HandleTypeDef *hmmc) {
  mmc_state_t *state = (mmc_state_t *)hmmc;
  // abort read and write
  sos_debug_log_warning(SOS_DEBUG_DEVICE, "Abort MMC");
  devfs_execute_cancel_handler(
    &state->transfer_handler,
    0,
    SYSFS_SET_RETURN(EIO),
    0);
}

// mmc actually uses the SDIO -- doesn't have it's own interrupt
void mcu_core_sdio_isr() {
  HAL_MMC_IRQHandler(&m_mmc_state_list[0]->hal_handle);
}

#endif
