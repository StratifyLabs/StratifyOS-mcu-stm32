// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include <sos/config.h>

#include "sdio_local.h"

#if MCU_SDIO_PORTS > 0

SDIO_TypeDef *const sdio_regs[MCU_SDIO_PORTS] = MCU_SDIO_REGS;
const int sdio_irqs[MCU_SDIO_PORTS] = MCU_SDIO_IRQS;
sdio_state_t *m_sdio_state_list[MCU_SDIO_PORTS] MCU_SYS_MEM;

int sdio_local_open(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sdio);
  if (state->ref_count == 0) {
    DEVFS_DRIVER_OPEN_STATE_LOCAL_V4(sdio);
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
    state->hal_handle.Instance = sdio_regs[config->port];
    cortexm_enable_irq(sdio_irqs[config->port]);
  }
  state->ref_count++;
  return 0;
}

int sdio_local_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sdio);
  if (state->ref_count > 0) {
    // do the opposite of mmc_local_open() -- ref_count is zero -- turn off
    // interrupt
    if (state->ref_count == 1) {
      HAL_SD_DeInit(&state->hal_handle);

      devfs_execute_cancel_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(ECANCELED),
        MCU_EVENT_FLAG_CANCELED);

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
      cortexm_disable_irq(sdio_irqs[config->port]);
      DEVFS_DRIVER_CLOSE_STATE_LOCAL_V4(sdio);
    }
    state->ref_count--;
  }

  return 0;
}

int sdio_local_getinfo(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sdio);
  sdio_info_t *info = ctl;

  // set flags that are supported by this driver
  info->o_flags = SDIO_FLAG_SET_INTERFACE;
  info->o_events = MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_WRITE_COMPLETE
                   | MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR
                   | MCU_EVENT_FLAG_SET_PRIORITY;

#if defined STM32H7
  info->freq = HAL_RCC_GetHCLKFreq() / (state->hal_handle.Init.ClockDiv + 1);
#else
  info->freq = 48000000UL / (state->hal_handle.Init.ClockDiv + 2);
#endif
  info->block_count = state->hal_handle.SdCard.BlockNbr;
  info->block_size = state->hal_handle.SdCard.BlockSize;
  info->card_class = state->hal_handle.SdCard.Class;
  info->logical_block_count = state->hal_handle.SdCard.LogBlockNbr;
  info->logical_block_size = state->hal_handle.SdCard.LogBlockSize;
  info->type = state->hal_handle.SdCard.CardType;
  info->relative_address = state->hal_handle.SdCard.RelCardAdd;
  info->version = state->hal_handle.SdCard.CardVersion;

  return SYSFS_RETURN_SUCCESS;
}

int sdio_local_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sdio);
  const sdio_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(sdio, ctl);
  if (attr == 0) {
    return SYSFS_SET_RETURN(ENOSYS);
  }

  u32 o_flags = attr->o_flags;

  if (o_flags & SDIO_FLAG_SET_INTERFACE) {

    // SDIO_CLOCK_EDGE_RISING
    // SDIO_CLOCK_EDGE_FALLING
    state->hal_handle.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    if (o_flags & SDIO_FLAG_IS_CLOCK_FALLING) {
      state->hal_handle.Init.ClockEdge = SDIO_CLOCK_EDGE_FALLING;
    }

    // SDIO_CLOCK_BYPASS_DISABLE
    // SDIO_CLOCK_BYPASS_ENABLE
#if defined SDIO_CLOCK_BYPASS_DISABLE
    state->hal_handle.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
    if (o_flags & SDIO_FLAG_IS_CLOCK_BYPASS_ENABLED) {
      state->hal_handle.Init.ClockBypass = SDIO_CLOCK_BYPASS_ENABLE;
    }
#endif

    // SDIO_CLOCK_POWER_SAVE_DISABLE
    // SDIO_CLOCK_POWER_SAVE_ENABLE
    state->hal_handle.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    if (o_flags & SDIO_FLAG_IS_CLOCK_POWER_SAVE_ENABLED) {
      state->hal_handle.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_ENABLE;
    }

    // initialize using 1B bus
    state->hal_handle.Init.BusWide = SDIO_BUS_WIDE_1B;

    // SDIO_HARDWARE_FLOW_CONTROL_DISABLE
    // SDIO_HARDWARE_FLOW_CONTROL_ENABLE
    state->hal_handle.Init.HardwareFlowControl
      = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    if (o_flags & SDIO_FLAG_IS_HARDWARE_FLOW_CONTROL_ENABLED) {
      state->hal_handle.Init.HardwareFlowControl
        = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
    }

    // must be <= 255
    state->hal_handle.Init.ClockDiv = 0;
    if (attr->freq && (attr->freq < 25000000UL)) {
#if defined STM32H7
      u32 divider_value = HAL_RCC_GetHCLKFreq() / attr->freq;
      if (divider_value > 0) {
        state->hal_handle.Init.ClockDiv = divider_value - 1;
        sos_debug_printf("clock div %d\n", state->hal_handle.Init.ClockDiv);
      }
#else
      // this is probably wrong -- should be HCLK like above, need to verify
      u32 divider_value = 48000000UL / attr->freq;
      if (divider_value > 2) {
        divider_value -= 2;
      } else {
        divider_value = 0;
      }
      state->hal_handle.Init.ClockDiv = divider_value;
#endif
    }

    // pin assignments
    int pin_result;
    if (
      (pin_result = mcu_set_pin_assignment(
         &(attr->pin_assignment),
         MCU_CONFIG_PIN_ASSIGNMENT(sdio_config_t, handle),
         MCU_PIN_ASSIGNMENT_COUNT(sdio_pin_assignment_t),
         CORE_PERIPH_SDIO,
         config->port,
         0,
         0,
         0))
      < 0) {
      return pin_result;
    }

    int hal_result;
    if ((hal_result = HAL_SD_Init(&state->hal_handle)) != HAL_OK) {
      sos_debug_printf(
        "hal result: %d, %d\n",
        hal_result,
        state->hal_handle.ErrorCode);
      return SYSFS_SET_RETURN(EIO);
    }

    // SDIO_BUS_WIDE_1B -- set as default for initialziation
    // SDIO_BUS_WIDE_4B
    // SDIO_BUS_WIDE_8B -- not compatible with SDIO
    if (o_flags & SDIO_FLAG_IS_BUS_WIDTH_4) {
      if (
        HAL_SD_ConfigWideBusOperation(&state->hal_handle, SDIO_BUS_WIDE_4B)
        != HAL_OK) {
        sos_debug_log_error(
          SOS_DEBUG_DEVICE,
          "failed to config 4 bit width 0x%X",
          state->hal_handle.ErrorCode);
        return SYSFS_SET_RETURN(EIO);
      }
    }
  }

  if (o_flags & SDIO_FLAG_RESET) {
    // if the EMMC has a timeout error, this will get it to recover from that

#if 0
    u32 width = state->hal_handle.Init.BusWide;
    sos_debug_log_info(SOS_DEBUG_DEVICE, "Reset SDIO");
    HAL_SD_Abort(&state->hal_handle);
    HAL_SD_DeInit(&state->hal_handle);
    if (HAL_SD_Init(&state->hal_handle) != HAL_OK) {
      sos_debug_log_error(SOS_DEBUG_DEVICE, "failed to reset SDIO");
      return SYSFS_SET_RETURN(EIO);
    }

    HAL_SD_ConfigWideBusOperation(&state->hal_handle, width);
#endif
  }

  if (o_flags & SDIO_FLAG_GET_CARD_STATE) {
    return HAL_SD_GetCardState(&state->hal_handle);
  }

  if (o_flags & SDIO_FLAG_ERASE_BLOCKS) {
    if (HAL_SD_Erase(&state->hal_handle, attr->start, attr->end) != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }
  }

  return SYSFS_RETURN_SUCCESS;
}

int sdio_local_setaction(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sdio);
  mcu_action_t *action = ctl;

  if (action->handler.callback == 0) {
    if (action->o_events & MCU_EVENT_FLAG_DATA_READY) {
      HAL_SD_Abort_IT(&state->hal_handle);
      devfs_execute_read_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(EIO),
        MCU_EVENT_FLAG_CANCELED);
    }

    if (action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE) {
      HAL_SD_Abort_IT(&state->hal_handle);
      devfs_execute_write_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(EIO),
        MCU_EVENT_FLAG_CANCELED);
    }
  }

  cortexm_set_irq_priority(
    sdio_irqs[config->port],
    action->prio,
    action->o_events);
  return 0;
}

int sdio_local_getcid(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sdio);
  if (HAL_SD_GetCardCID(&state->hal_handle, ctl) == HAL_OK) {
    return SYSFS_RETURN_SUCCESS;
  }

  return SYSFS_SET_RETURN(EIO);
}

int sdio_local_getcsd(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sdio);
  if (HAL_SD_GetCardCSD(&state->hal_handle, ctl) == HAL_OK) {
    return SYSFS_RETURN_SUCCESS;
  }

  return SYSFS_SET_RETURN(EIO);
}

int sdio_local_getstatus(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(sdio);
  if (HAL_SD_GetCardStatus(&state->hal_handle, ctl) == HAL_OK) {
    return SYSFS_RETURN_SUCCESS;
  }
  return SYSFS_SET_RETURN(EIO);
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd) {
  sdio_state_t *state = (sdio_state_t *)hsd;
  devfs_execute_write_handler(
    &state->transfer_handler,
    0,
    hsd->TxXferSize,
    MCU_EVENT_FLAG_WRITE_COMPLETE);
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd) {
  sdio_state_t *state = (sdio_state_t *)hsd;

#if defined STM32F7 || defined STM32H7
  if (state->transfer_handler.read) {
    sos_config.cache.invalidate_data_block(
      state->transfer_handler.read->buf,
      state->transfer_handler.read->nbyte);
  }
#endif

  // sos_debug_root_printf("read complete %d 0x%lX %ld\n", hsd->RxXferSize,
  // hsd->Instance->STA, TIM2->CNT - state->start_time);
  devfs_execute_read_handler(
    &state->transfer_handler,
    0,
    hsd->RxXferSize,
    MCU_EVENT_FLAG_DATA_READY);
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd) {
  sdio_state_t *state = (sdio_state_t *)hsd;
  // sos_debug_log_warning(SOS_DEBUG_DEVICE, "SD Error? 0x%lX 0x%lX %ld",
  // hsd->ErrorCode, hsd->hdmatx->ErrorCode, TIM2->CNT - state->start_time);
  if (hsd->ErrorCode) {
    sos_debug_log_error(SOS_DEBUG_DEVICE, "SD Error 0x%lX", hsd->ErrorCode);
    devfs_execute_cancel_handler(
      &state->transfer_handler,
      0,
      SYSFS_SET_RETURN(EIO),
      MCU_EVENT_FLAG_ERROR);
  }
}

void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd) {
  sdio_state_t *state = (sdio_state_t *)hsd;
  // abort read and write
  sos_debug_log_warning(SOS_DEBUG_DEVICE, "Abort\n");
  devfs_execute_cancel_handler(
    &state->transfer_handler,
    0,
    SYSFS_SET_RETURN(EIO),
    0);
}

#if defined STM32F7 || defined STM32H7

#if MCU_SDIO_PORTS > 1
void mcu_core_sdmmc1_isr() {
  HAL_SD_IRQHandler(&m_sdio_state_list[0]->hal_handle);
}

void mcu_core_sdmmc2_isr() {
  HAL_SD_IRQHandler(&m_sdio_state_list[1]->hal_handle);
}
#else
void mcu_core_sdmmc_isr() {
  HAL_SD_IRQHandler(&m_sdio_state_list[0]->hal_handle);
}
#endif
#else
void mcu_core_sdio_isr() {
  // sos_debug_log_info(SOS_DEBUG_DEVICE, "SDIO IRQ 0x%lX",
  // sd_handle[0]->Instance->STA);
  HAL_SD_IRQHandler(&m_sdio_state_list[0]->hal_handle);
}
#endif

#endif
