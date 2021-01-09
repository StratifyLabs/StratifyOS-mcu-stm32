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
#include <mcu/core.h>
#include <mcu/pio.h>
#include <sos/config.h>
#include <sos/debug.h>

#include "uart_local.h"

#if MCU_UART_PORTS > 0

#include "uart_local.h"

uart_state_t *m_uart_state_list[MCU_UART_PORTS] MCU_SYS_MEM;

USART_TypeDef *const uart_regs_table[MCU_UART_PORTS] = MCU_UART_REGS;
u8 const uart_irqs[MCU_UART_PORTS] = MCU_UART_IRQS;

static void handle_bytes_received(uart_state_t *state, u16 bytes_received);

int uart_local_open(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(uart);
  if (state->ref_count == 0) {
    DEVFS_DRIVER_OPEN_STATE_LOCAL_V4(uart);

    state->hal_handle.Instance = uart_regs_table[config->port];

    switch (config->port) {
    case 0:
      __HAL_RCC_USART1_CLK_ENABLE();
      break;
#if defined USART2
    case 1:
      __HAL_RCC_USART2_CLK_ENABLE();
      break;
#endif
#if defined USART3
    case 2:
      __HAL_RCC_USART3_CLK_ENABLE();
      break;
#endif
#if defined UART4
    case 3:
      __HAL_RCC_UART4_CLK_ENABLE();
      break;
#endif
#if defined UART5
    case 4:
      __HAL_RCC_UART5_CLK_ENABLE();
      break;
#endif
#if defined USART6
    case 5:
      __HAL_RCC_USART6_CLK_ENABLE();
      break;
#endif
#if defined UART7
    case 6:
      __HAL_RCC_UART7_CLK_ENABLE();
      break;
#endif
#if defined UART8
    case 7:
      __HAL_RCC_UART8_CLK_ENABLE();
      break;
#endif
#if defined UART9
    case 8:
      __HAL_RCC_UART9_CLK_ENABLE();
      break;
#endif
#if defined UART10
    case 9:
      __HAL_RCC_UART10_CLK_ENABLE();
      break;
#endif
    }
    // reset HAL UART
    cortexm_enable_irq(uart_irqs[config->port]);
  }
  state->ref_count++;

  return 0;
}

int uart_local_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(uart);

  // FIFO won't exist on write only devices
  if (state->fifo_config != 0) {
    // unblock the read FIFO
    fifo_cancel_async_read(&state->fifo_state);
  }

  if (state->ref_count > 0) {
    if (state->ref_count == 1) {
      cortexm_disable_irq(uart_irqs[config->port]);
      switch (config->port) {
      case 0:
        __HAL_RCC_USART1_CLK_DISABLE();
        break;
#if defined USART2
      case 1:
        __HAL_RCC_USART2_CLK_DISABLE();
        break;
#endif
#if defined USART3
      case 2:
        __HAL_RCC_USART3_CLK_DISABLE();
        break;
#endif
#if defined UART4
      case 3:
        __HAL_RCC_UART4_CLK_DISABLE();
        break;
#endif
#if defined UART5
      case 4:
        __HAL_RCC_UART5_CLK_DISABLE();
        break;
#endif
#if defined USART6
      case 5:
        __HAL_RCC_USART6_CLK_DISABLE();
        break;
#endif
#if defined UART7
      case 6:
        __HAL_RCC_UART7_CLK_DISABLE();
        break;
#endif
#if defined UART8
      case 7:
        __HAL_RCC_UART8_CLK_DISABLE();
        break;
#endif
#if defined UART9
      case 8:
        __HAL_RCC_UART9_CLK_DISABLE();
        break;
#endif
#if defined UART10
      case 9:
        __HAL_RCC_UART10_CLK_DISABLE();
        break;
#endif
      }
      state->hal_handle.Instance = 0;
      DEVFS_DRIVER_CLOSE_STATE_LOCAL_V4(uart);
    }
    state->ref_count--;
  }
  return 0;
}

int uart_local_getinfo(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(uart);

  uart_info_t *info = ctl;

  info->o_flags = UART_FLAG_IS_PARITY_NONE | UART_FLAG_IS_PARITY_ODD
                  | UART_FLAG_IS_PARITY_EVEN | UART_FLAG_IS_STOP1
                  | UART_FLAG_IS_STOP2 | UART_FLAG_IS_RX_FIFO;

  if (state->fifo_config) {
    fifo_info_t fifo_info;
    info->o_flags |= UART_FLAG_IS_RX_FIFO;
    fifo_getinfo(&fifo_info, state->fifo_config, &state->fifo_state);
    info->size_ready = fifo_info.size_ready;
    info->size = fifo_info.size;
  } else {
    info->size_ready = 0;
    info->size = 0;
  }

  return 0;
}

int uart_local_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(uart);
  u32 o_flags;
  u32 freq;

  const uart_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(uart, ctl);
  if (attr == NULL) {
    return SYSFS_SET_RETURN(EINVAL);
  }

  const uart_config_t *uart_config = handle->config;
  if (uart_config) {
    state->fifo_config = uart_config->fifo_config;
  } else {
    state->fifo_config = 0;
  }

  o_flags = attr->o_flags;

  if (o_flags & UART_FLAG_SET_LINE_CODING) {
    freq = attr->freq;
    if (freq == 0) {
      freq = 115200;
    }

    state->hal_handle.Init.BaudRate = freq;

    state->hal_handle.Init.WordLength = UART_WORDLENGTH_8B;
    if (attr->width == 9) {
      state->hal_handle.Init.WordLength = UART_WORDLENGTH_9B;
    }

    state->hal_handle.Init.StopBits = UART_STOPBITS_1;
    if (o_flags & UART_FLAG_IS_STOP2) {
      state->hal_handle.Init.StopBits = UART_STOPBITS_2;
    }

    state->hal_handle.Init.Parity = UART_PARITY_NONE;
    if (o_flags & UART_FLAG_IS_PARITY_EVEN) {
      state->hal_handle.Init.Parity = UART_PARITY_EVEN;
    } else if (o_flags & UART_FLAG_IS_PARITY_ODD) {
      state->hal_handle.Init.Parity = UART_PARITY_ODD;
    }

    const uart_pin_assignment_t *pin_assignment = mcu_select_pin_assignment(
      &attr->pin_assignment,
      MCU_CONFIG_PIN_ASSIGNMENT(uart_config_t, handle),
      MCU_PIN_ASSIGNMENT_COUNT(uart_pin_assignment_t));

    state->hal_handle.Init.Mode = UART_MODE_TX_RX;
    if (pin_assignment) {
      if (pin_assignment->tx.port == 0xff) {
        state->hal_handle.Init.Mode = UART_MODE_RX;
      } else if (pin_assignment->rx.port == 0xff) {
        state->hal_handle.Init.Mode = UART_MODE_TX;
      }
    }
    state->hal_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    state->hal_handle.Init.OverSampling = UART_OVERSAMPLING_16;

    // pin assignments
    if (
      mcu_set_pin_assignment(
        &(attr->pin_assignment),
        MCU_CONFIG_PIN_ASSIGNMENT(uart_config_t, handle),
        MCU_PIN_ASSIGNMENT_COUNT(uart_pin_assignment_t),
        CORE_PERIPH_UART,
        config->port,
        0,
        0,
        0)
      < 0) {
      return SYSFS_SET_RETURN(EINVAL);
    }

    if (HAL_UART_Init(&state->hal_handle) != HAL_OK) {
      SOS_DEBUG_LINE_TRACE();
      return SYSFS_SET_RETURN(EIO);
    }

    // enable the idle interrupt and initialize the fifo
    if (state->fifo_config) {
      fifo_ioctl_local(state->fifo_config, &state->fifo_state, I_FIFO_INIT, 0);
      // enables idle interrupt
#if defined USART_CR1_RTOIE
      // config timeout for a half second
      HAL_UART_ReceiverTimeout_Config(&state->hal_handle, freq / 50);
      HAL_UART_EnableReceiverTimeout(&state->hal_handle);
      SET_BIT(state->hal_handle.Instance->CR1, USART_CR1_RTOIE);
#else
      SET_BIT(state->hal_handle.Instance->CR1, USART_CR1_IDLEIE);
#endif
    }
  }

  return SYSFS_RETURN_SUCCESS;
}

int uart_local_setaction(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(uart);
  mcu_action_t *action = (mcu_action_t *)ctl;

  if (action->handler.callback == 0) {
    // if there is an ongoing operation -- cancel it
    if (action->o_events & MCU_EVENT_FLAG_DATA_READY) {
      // execute the read callback if not null
      if (state->fifo_config != 0) {
        fifo_cancel_async_read(&state->fifo_state);
      }
    }

    if (action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE) {
#if defined STM32F4
      if (state->o_flags & UART_LOCAL_IS_DMA) {
        // stop the DMA write but not the DMA read -- there isn't a simple call
        // to do this
      } else {
        HAL_UART_AbortTransmit_IT(&state->hal_handle);
      }
#endif
      devfs_execute_write_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(EIO),
        MCU_EVENT_FLAG_CANCELED);
    }
  }

  cortexm_set_irq_priority(
    uart_irqs[config->port],
    action->prio,
    action->o_events);
  return 0;
}

int uart_local_put(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(uart);
  u8 c = (u32)ctl;

  if (HAL_UART_Transmit(&state->hal_handle, &c, 1, HAL_MAX_DELAY) != HAL_OK) {
    return SYSFS_SET_RETURN(EIO);
  }

  return SYSFS_RETURN_SUCCESS;
}

int uart_local_flush(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(uart);

  /*
   * The interrupt and DMA reads rely on the head staying
   * in the right place for the incoming data. So the
   * head is restored and the tail is assiged to
   * the head when the fifo is flushed.
   *
   */
  u16 head = state->fifo_state.atomic_position.access.head;
  fifo_flush(&state->fifo_state);
  state->fifo_state.atomic_position.access.head = head;
  state->fifo_state.atomic_position.access.tail = head;

  return SYSFS_RETURN_SUCCESS;
}

int uart_local_get(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(uart);

  if (state->fifo_config == 0) {
    return SYSFS_SET_RETURN(ENOSYS);
  }

  // is there a byte on the FIFO to read?
  int result = fifo_read_buffer(state->fifo_config, &state->fifo_state, ctl, 1);
  if (result == 1) {
    return SYSFS_RETURN_SUCCESS;
  }

  return SYSFS_SET_RETURN(ENODATA);
}

int uart_local_read(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(uart);

  if (state->fifo_config == NULL) {
    return SYSFS_SET_RETURN(ENOSYS);
  }

  // check the DMA for enough bytes to fulfill the request
  if (state->hal_handle.hdmarx) {
    u16 bytes_ready = state->fifo_config->size
                      - __HAL_DMA_GET_COUNTER(state->hal_handle.hdmarx)
                      - state->fifo_state.atomic_position.access.head;

    // if there aren't enough to fulfill the request
    // the timeout will kick in later
    if (bytes_ready > async->nbyte) {
      handle_bytes_received(state, bytes_ready);
    }
  }

  // read the fifo, block if no bytes are available
  int result
    = fifo_read_local(state->fifo_config, &state->fifo_state, async, 0);

  return result;
}

void handle_bytes_received(uart_state_t *state, u16 bytes_received) {

  // invalidate the cache
  sos_config.cache.invalidate_data_block(
    state->fifo_config->buffer + state->fifo_state.atomic_position.access.head,
    bytes_received + 31);

  // increment the head by the number of bytes received
  for (u16 i = 0; i < bytes_received; i++) {
    fifo_inc_head(&state->fifo_state, state->fifo_config->size);
  }

  // now tell the fifo the head has been updated so it can return data to the
  // user asynchronously
  fifo_data_received(state->fifo_config, &state->fifo_state);
}

#define SHOW_DMA_FIFO_PROGRESS 0

void HAL_UART_RxIdleCallback(UART_HandleTypeDef *huart) {
  uart_state_t *state = (uart_state_t *)huart;
  u16 bytes_received;
  u16 head = state->fifo_state.atomic_position.access.head;
  if (huart->hdmarx) {
    bytes_received
      = state->fifo_config->size - __HAL_DMA_GET_COUNTER(huart->hdmarx) - head;

#if SHOW_DMA_FIFO_PROGRESS
    sos_debug_printf(
      "i: %d - %d\n",
      __HAL_DMA_GET_COUNTER(huart->hdmarx),
      head);
#endif
  } else {
    bytes_received = (huart->RxXferSize - huart->RxXferCount) - head;
  }
  handle_bytes_received(state, bytes_received);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
  MCU_UNUSED_ARGUMENT(huart);
  // idle will handle partial reception until complete is called

  // allow the caller to get the bytes that have arrived so far
  uart_state_t *state = (uart_state_t *)huart;
  u16 bytes_received;
  bytes_received = state->fifo_config->size / 2
                   - state->fifo_state.atomic_position.access.head;
  handle_bytes_received(state, bytes_received);
#if SHOW_DMA_FIFO_PROGRESS
  sos_debug_printf("h: %d\n", state->fifo_state.atomic_position.access.head);
#endif
}

// called when RX IT is complete or when DMA does full transfer
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  uart_state_t *state = (uart_state_t *)huart;
  u16 bytes_received;
  bytes_received
    = state->fifo_config->size - state->fifo_state.atomic_position.access.head;
  handle_bytes_received(state, bytes_received);

#if SHOW_DMA_FIFO_PROGRESS
  sos_debug_printf("f: %d\n", state->fifo_state.atomic_position.access.head);
#endif

  if (state->hal_handle.hdmarx == NULL) {
    // if not in circular DMA mode -- start the next interrupt based read
    HAL_UART_Receive_IT(
      &state->hal_handle,
      (u8 *)state->fifo_config->buffer,
      state->fifo_config->size);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  uart_state_t *state = (uart_state_t *)huart;

#if defined USART_CR1_RTOIE
  // not an error
  if (huart->ErrorCode == HAL_UART_ERROR_RTO) {
    sos_debug_printf("idle\n");
    HAL_UART_RxIdleCallback(&state->hal_handle);
    return;
  }
#endif

  sos_debug_printf("Error is %d\n", huart->ErrorCode);
  devfs_execute_read_handler(
    &state->transfer_handler,
    0,
    SYSFS_SET_RETURN(EIO),
    MCU_EVENT_FLAG_ERROR | MCU_EVENT_FLAG_CANCELED);
  // reset the head
  fifo_flush(&state->fifo_state);
  HAL_UART_AbortReceive_IT(huart);
  if (state->hal_handle.hdmarx == NULL) {
    // if not in circular DMA mode -- start the next interrupt based read
    HAL_UART_Receive_IT(
      &state->hal_handle,
      (u8 *)state->fifo_config->buffer,
      state->fifo_config->size);
  } else {
    HAL_UART_Receive_DMA(
      &state->hal_handle,
      (u8 *)state->fifo_config->buffer,
      state->fifo_config->size);
  }
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart) {
  // this does not need to do anything
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart) {}

void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart) {}

void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart) {
  // sos_debug_printf("abort rx\n");
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  uart_state_t *state = (uart_state_t *)huart;
  int nbyte = 0;
  // when transfer is complete, count is 0 and size is the original async->nbyte
  // value
  nbyte = huart->TxXferSize - huart->TxXferCount;
  devfs_execute_write_handler(
    &state->transfer_handler,
    0,
    nbyte,
    MCU_EVENT_FLAG_DATA_READY);
}

void mcu_uart_isr(int port) {
  uart_state_t *state = m_uart_state_list[port];
  HAL_UART_IRQHandler(&state->hal_handle);

#if !defined USART_CR1_RTOIE
  if (__HAL_UART_GET_FLAG(&state->hal_handle, UART_FLAG_IDLE)) {
    __HAL_UART_CLEAR_IDLEFLAG(&state->hal_handle);
    HAL_UART_RxIdleCallback(&state->hal_handle);
    return;
  }
#endif
}

#if defined USART1
void mcu_core_usart1_isr() { mcu_uart_isr(0); }
#endif

#if defined USART2
void mcu_core_usart2_isr() { mcu_uart_isr(1); }
#endif

#if defined USART3
void mcu_core_usart3_isr() { mcu_uart_isr(2); }
#endif

#if defined UART4
void mcu_core_uart4_isr() { mcu_uart_isr(3); }
#endif

#if defined UART5
void mcu_core_uart5_isr() { mcu_uart_isr(4); }
#endif

#if defined USART6
void mcu_core_usart6_isr() { mcu_uart_isr(5); }
#endif

#if defined UART7
void mcu_core_uart7_isr() { mcu_uart_isr(6); }
#endif

#if defined UART8
void mcu_core_uart8_isr() { mcu_uart_isr(7); }
#endif

#if defined UART9
void mcu_core_uart9_isr() { mcu_uart_isr(8); }
#endif

#if defined UART10
void mcu_core_uart10_isr() { mcu_uart_isr(9); }
#endif

#endif
