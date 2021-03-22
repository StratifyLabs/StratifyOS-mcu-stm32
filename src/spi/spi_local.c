// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include <mcu/spi.h>
#include <sos/config.h>

#include "spi_local.h"

#if MCU_SPI_PORTS > 0

SPI_TypeDef *const spi_regs[MCU_SPI_PORTS] = MCU_SPI_REGS;
u8 const spi_irqs[MCU_SPI_PORTS] = MCU_SPI_IRQS;
spi_state_t *m_spi_state_list[MCU_SPI_PORTS] MCU_SYS_MEM;

static void clear_overrun_state(spi_state_t *spi);

int spi_local_open(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);
  if (state->ref_count == 0) {
    DEVFS_DRIVER_OPEN_STATE_LOCAL_V4(spi);
    // turn on RCC clock
    switch (config->port) {
    case 0:
      __HAL_RCC_SPI1_CLK_ENABLE();
      break;
#if defined SPI2
    case 1:
      __HAL_RCC_SPI2_CLK_ENABLE();
      break;
#endif
#if defined SPI3
    case 2:
      __HAL_RCC_SPI3_CLK_ENABLE();
      break;
#endif
#if defined SPI4
    case 3:
      __HAL_RCC_SPI4_CLK_ENABLE();
      break;
#endif
#if defined SPI5
    case 4:
      __HAL_RCC_SPI5_CLK_ENABLE();
      break;
#endif
#if defined SPI6
    case 5:
      __HAL_RCC_SPI6_CLK_ENABLE();
      break;
#endif
    }
    state->transfer_handler.read = NULL;
    state->transfer_handler.write = NULL;
    state->hal_handle.Instance = spi_regs[config->port];
    cortexm_enable_irq(spi_irqs[config->port]);
  }
  state->ref_count++;
  return 0;
}

int spi_local_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);
  if (state->ref_count > 0) {
    if (state->ref_count == 1) {

#if MCU_I2S_SPI_PORTS > 0
      if (state->o_flags & SPI_LOCAL_IS_I2S) {
        HAL_I2S_DeInit(&state->i2s_hal_handle);
        sos_debug_log_info(SOS_DEBUG_DEVICE, "Done I2S DeInit");
      } else {
        HAL_SPI_DeInit(&state->hal_handle);
      }
#else
      HAL_SPI_DeInit(&state->hal_handle);
#endif

      cortexm_disable_irq(spi_irqs[config->port]);
      devfs_execute_cancel_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(EDEADLK),
        MCU_EVENT_FLAG_CANCELED);

      // turn off RCC clock
      switch (config->port) {
      case 0:
        __HAL_RCC_SPI1_CLK_DISABLE();
        break;
#if defined SPI2
      case 1:
        __HAL_RCC_SPI2_CLK_DISABLE();
        break;
#endif
#if defined SPI3
      case 2:
        __HAL_RCC_SPI3_CLK_DISABLE();
        break;
#endif
#if defined SPI4
      case 3:
        __HAL_RCC_SPI4_CLK_DISABLE();
        break;
#endif
#if defined SPI5
      case 4:
        __HAL_RCC_SPI5_CLK_DISABLE();
        break;
#endif
#if defined SPI6
      case 5:
        __HAL_RCC_SPI6_CLK_DISABLE();
        break;
#endif
      }
      DEVFS_DRIVER_CLOSE_STATE_LOCAL_V4(spi);
    }
    state->ref_count--;
  }
  return 0;
}

int spi_local_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);

  u32 pclk;
  u32 prescalar;

  const spi_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(spi, ctl);
  if (attr == NULL) {
    return SYSFS_SET_RETURN(EINVAL);
  }

  u32 o_flags = attr->o_flags;

  state->o_flags = 0;

  if (o_flags & SPI_FLAG_SET_MASTER) {
    state->hal_handle.Init.Mode = SPI_MODE_MASTER;

    if (attr->freq == 0) {
      state->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    } else {

      //#if defined __stm32f401xc
      if ((((u32)state->hal_handle.Instance) & ~0xFFFF) == APB1PERIPH_BASE) {
        pclk = HAL_RCC_GetPCLK1Freq();
      } else {
        pclk = HAL_RCC_GetPCLK2Freq();
      }
      //#endif

      // get as close to the target freq as possible without going over
      sos_debug_log_info(SOS_DEBUG_DEVICE, "pclk %ld", pclk);
      prescalar = pclk / attr->freq;
      if (prescalar <= 2) {
        sos_debug_log_info(SOS_DEBUG_DEVICE, "SPI PS 2");
        state->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
      } else if (prescalar <= 4) {
        sos_debug_log_info(SOS_DEBUG_DEVICE, "SPI PS 4");
        state->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
      } else if (prescalar <= 8) {
        sos_debug_log_info(SOS_DEBUG_DEVICE, "SPI PS 8");
        state->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
      } else if (prescalar <= 16) {
        sos_debug_log_info(SOS_DEBUG_DEVICE, "SPI PS 16");
        state->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
      } else if (prescalar <= 32) {
        sos_debug_log_info(SOS_DEBUG_DEVICE, "SPI PS 32");
        state->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
      } else if (prescalar <= 64) {
        sos_debug_log_info(SOS_DEBUG_DEVICE, "SPI PS 64");
        state->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
      } else if (prescalar <= 128) {
        sos_debug_log_info(SOS_DEBUG_DEVICE, "SPI PS 128");
        state->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
      } else {
        sos_debug_log_info(SOS_DEBUG_DEVICE, "SPI PS 256");
        state->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
      }
    }

  } else if (o_flags & SPI_FLAG_SET_SLAVE) {
    state->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    state->hal_handle.Init.Mode = SPI_MODE_SLAVE;
  }

  if (o_flags & (SPI_FLAG_SET_SLAVE | SPI_FLAG_SET_MASTER)) {
    state->hal_handle.Init.CLKPolarity = SPI_POLARITY_LOW;
    state->hal_handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    if (o_flags & SPI_FLAG_IS_MODE1) {
      state->hal_handle.Init.CLKPolarity = SPI_POLARITY_LOW;
      state->hal_handle.Init.CLKPhase = SPI_PHASE_2EDGE;
    } else if (o_flags & SPI_FLAG_IS_MODE2) {
      state->hal_handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
      state->hal_handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    } else if (o_flags & SPI_FLAG_IS_MODE3) {
      state->hal_handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
      state->hal_handle.Init.CLKPhase = SPI_PHASE_2EDGE;
    }

    if (attr->width == 8) {
      state->hal_handle.Init.DataSize = SPI_DATASIZE_8BIT;
    } else if (attr->width == 16) {
      state->hal_handle.Init.DataSize = SPI_DATASIZE_16BIT;
    } else {
      return SYSFS_SET_RETURN(EINVAL);
    }

    state->hal_handle.Init.Direction = SPI_DIRECTION_2LINES;

    state->hal_handle.Init.NSS = SPI_NSS_SOFT;
    state->hal_handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    if (o_flags & SPI_FLAG_IS_LSB_FIRST) {
      state->hal_handle.Init.FirstBit = SPI_FIRSTBIT_LSB;
    }

    if (o_flags & SPI_FLAG_IS_FORMAT_TI) {
      state->hal_handle.Init.TIMode = SPI_TIMODE_ENABLE;
    } else {
      state->hal_handle.Init.TIMode = SPI_TIMODE_DISABLE;
    }
    state->hal_handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    state->hal_handle.Init.CRCPolynomial = 10;

#if MCU_SPI_API > 0
    state->hal_handle.Init.CRCLength = 0;
    state->hal_handle.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    state->hal_handle.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    state->hal_handle.Init.FifoThreshold = SPI_FIFO_THRESHOLD_08DATA;
    state->hal_handle.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_01CYCLE;
    state->hal_handle.Init.MasterInterDataIdleness
      = SPI_MASTER_INTERDATA_IDLENESS_01CYCLE;
    state->hal_handle.Init.MasterReceiverAutoSusp
      = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    state->hal_handle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    state->hal_handle.Init.IOSwap = SPI_IO_SWAP_DISABLE;
#endif

    if (
      mcu_set_pin_assignment(
        &(attr->pin_assignment),
        MCU_CONFIG_PIN_ASSIGNMENT(spi_config_t, handle),
        MCU_PIN_ASSIGNMENT_COUNT(spi_pin_assignment_t),
        CORE_PERIPH_SPI,
        config->port,
        0,
        0,
        0)
      < 0) {
      return SYSFS_SET_RETURN(EINVAL);
    }

    if (HAL_SPI_Init(&state->hal_handle) != HAL_OK) {
      return SYSFS_SET_RETURN(EINVAL);
    }
  }

  if (o_flags & SPI_FLAG_SET_FULL_DUPLEX) {
    state->o_flags |= SPI_LOCAL_IS_FULL_DUPLEX;
  } else if (o_flags & SPI_FLAG_SET_HALF_DUPLEX) {
    state->o_flags &= ~SPI_LOCAL_IS_FULL_DUPLEX;
  }

  return 0;
}

int spi_local_swap(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);

  u8 tx_data;
  u8 rx_data;
  int ret;
  tx_data = (u32)ctl;
  ret = HAL_SPI_TransmitReceive(
    &state->hal_handle,
    &tx_data,
    &rx_data,
    1,
    HAL_MAX_DELAY);

  // clear any status or overrun state
  clear_overrun_state(state);

  if (ret != HAL_OK) {
    return SYSFS_SET_RETURN(EIO);
  }
  // must always be a positive (int)
  return rx_data;
}

int spi_local_setaction(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(spi);
  mcu_action_t *action = (mcu_action_t *)ctl;

  // callback = 0 with flags set will cancel an ongoing operation
  if (action->handler.callback == 0) {
    if (action->o_events & MCU_EVENT_FLAG_DATA_READY) {
      // cancel the action based on interrupt or DMA

      devfs_execute_read_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(EINTR),
        MCU_EVENT_FLAG_CANCELED);
    }

    if (action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE) {
      // cancel the action based on interrupt or DMA

      devfs_execute_write_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(EINTR),
        MCU_EVENT_FLAG_CANCELED);
    }
  }

  cortexm_set_irq_priority(
    spi_irqs[config->port],
    action->prio,
    action->o_events);
  return 0;
}

// these callbacks work the same for both DMA and interrupt driven operations
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  // execute the handler
  spi_state_t *state = (spi_state_t *)hspi;
  // sos_debug_log_info(SOS_DEBUG_DEVICE, "SPI TX DONE:%d,%d", hspi->TxXferSize,
  // spi->transfer_handler.write ? spi->transfer_handler.write->nbyte : 0);
  devfs_execute_write_handler(
    &state->transfer_handler,
    0,
    hspi->TxXferSize,
    MCU_EVENT_FLAG_WRITE_COMPLETE);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
  // execute the handler
  spi_state_t *spi = (spi_state_t *)hspi;

#if defined STM32F7 || defined STM32H7
  if (hspi->hdmarx && spi->transfer_handler.read) {
    sos_config.cache.invalidate_data_block(
      spi->transfer_handler.read->buf,
      spi->transfer_handler.read->nbyte);
  }
#endif

  // sos_debug_log_info(SOS_DEBUG_DEVICE, "SPI RX DONE:%d,%d", hspi->RxXferSize,
  // spi->transfer_handler.read ? spi->transfer_handler.read->nbyte : 0);
  devfs_execute_read_handler(
    &spi->transfer_handler,
    0,
    hspi->RxXferSize,
    MCU_EVENT_FLAG_DATA_READY);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  // execute the handler
  spi_state_t *spi = (spi_state_t *)hspi;

#if defined STM32F7 || defined STM32H7
  if (hspi->hdmarx && spi->transfer_handler.read) {
    sos_config.cache.invalidate_data_block(
      spi->transfer_handler.read->buf,
      spi->transfer_handler.read->nbyte);
  }
#endif

  // sos_debug_log_info(SOS_DEBUG_DEVICE, "SPI FD DONE %d", hspi->TxXferSize);
  devfs_execute_read_handler(
    &spi->transfer_handler,
    0,
    hspi->RxXferSize,
    MCU_EVENT_FLAG_DATA_READY);
  devfs_execute_write_handler(
    &spi->transfer_handler,
    0,
    hspi->TxXferSize,
    MCU_EVENT_FLAG_WRITE_COMPLETE);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
  spi_state_t *spi = (spi_state_t *)hspi;

  clear_overrun_state(spi);

  sos_debug_log_error(
    SOS_DEBUG_DEVICE,
    "SPI Error:0x%X %p %p",
    hspi->ErrorCode,
    spi->transfer_handler.read,
    spi->transfer_handler.write);

  devfs_execute_cancel_handler(
    &spi->transfer_handler,
    0,
    SYSFS_SET_RETURN_WITH_VALUE(EIO, hspi->ErrorCode),
    MCU_EVENT_FLAG_ERROR);
}

void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi) {
  spi_state_t *spi = (spi_state_t *)hspi;

  sos_debug_log_warning(SOS_DEBUG_DEVICE, "SPI Abort:0x%X", hspi->ErrorCode);
  devfs_execute_cancel_handler(
    &spi->transfer_handler,
    0,
    SYSFS_SET_RETURN(EAGAIN),
    0);
}

void clear_overrun_state(spi_state_t *spi) {
  // deal with overrrun errors
#if defined STM32H7
  spi->hal_handle.Instance->UDRDR;
#else
  spi->hal_handle.Instance->DR;
#endif
  spi->hal_handle.Instance->SR;
}

// these handlers need to move to the local file
void mcu_core_spi1_isr() {
  // No I2S on SPI 1
#if MCU_I2S_ON_SPI1 != 0
  if (m_spi_state_list[0]->o_flags & SPI_LOCAL_IS_I2S) {
    HAL_I2S_IRQHandler(&m_spi_state_list[0]->i2s_hal_handle);
  } else {
    HAL_SPI_IRQHandler(&m_spi_state_list[0]->hal_handle);
  }
#else
  HAL_SPI_IRQHandler(&m_spi_state_list[0]->hal_handle);
#endif
}

void mcu_core_spi2_isr() {
#if MCU_I2S_ON_SPI2 != 0
  if (m_spi_state_list[1]->o_flags & SPI_LOCAL_IS_I2S) {
    HAL_I2S_IRQHandler(&m_spi_state_list[1]->i2s_hal_handle);
  } else {
    HAL_SPI_IRQHandler(&m_spi_state_list[1]->hal_handle);
  }
#elif MCU_SPI_PORTS > 1
  HAL_SPI_IRQHandler(&m_spi_state_list[1]->hal_handle);
#endif
}

void mcu_core_spi3_isr() {
#if MCU_I2S_ON_SPI3 != 0
  if (m_spi_state_list[2]->o_flags & SPI_LOCAL_IS_I2S) {
    HAL_I2S_IRQHandler(&m_spi_state_list[2]->i2s_hal_handle);
  } else {
    HAL_SPI_IRQHandler(&m_spi_state_list[2]->hal_handle);
  }
#elif MCU_SPI_PORTS > 2
  HAL_SPI_IRQHandler(&m_spi_state_list[2]->hal_handle);
#endif
}

void mcu_core_spi4_isr() {
#if MCU_I2S_ON_SPI4 != 0
  if (m_spi_state_list[3]->o_flags & SPI_LOCAL_IS_I2S) {
    HAL_I2S_IRQHandler(&m_spi_state_list[3]->i2s_hal_handle);
  } else {
    HAL_SPI_IRQHandler(&m_spi_state_list[3]->hal_handle);
  }
#elif MCU_SPI_PORTS > 3
  HAL_SPI_IRQHandler(&m_spi_state_list[3]->hal_handle);
#endif
}

#if MCU_SPI_PORTS > 4
void mcu_core_spi5_isr() {
  HAL_SPI_IRQHandler(&m_spi_state_list[4]->hal_handle);
}
#endif

#if MCU_SPI_PORTS > 5
void mcu_core_spi6_isr() {
  HAL_SPI_IRQHandler(&m_spi_state_list[5]->hal_handle);
}
#endif

#endif
