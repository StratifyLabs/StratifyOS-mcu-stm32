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
#include <mcu/dac.h>
#include <mcu/pio.h>
#include <sos/config.h>
#include <sos/debug.h>

#include "stm32_local.h"

#include "dac_local.h"

#if MCU_DAC_PORTS > 0

DAC_TypeDef *const m_dac_regs_table[MCU_DAC_PORTS] = MCU_DAC_REGS;
u8 const m_dac_irqs[MCU_DAC_PORTS] = MCU_DAC_IRQS;
u32 const m_dac_channels[MCU_DAC_CHANNELS] = {DAC_CHANNEL_1, DAC_CHANNEL_2};
dac_state_t *m_dac_state_list[MCU_DAC_PORTS] MCU_SYS_MEM;

#if 0
const u32 dac_channels[MCU_DAC_CHANNELS] = {
	DAC_CHANNEL_0, DAC_CHANNEL_1, DAC_CHANNEL_2, DAC_CHANNEL_3,
	DAC_CHANNEL_4, DAC_CHANNEL_5, DAC_CHANNEL_6, DAC_CHANNEL_7,
	DAC_CHANNEL_8, DAC_CHANNEL_9, DAC_CHANNEL_10, DAC_CHANNEL_11,
	DAC_CHANNEL_12, DAC_CHANNEL_13, DAC_CHANNEL_14, DAC_CHANNEL_15,
	DAC_CHANNEL_16, DAC_CHANNEL_17, DAC_CHANNEL_18, DAC_CHANNEL_VREFINT,
	DAC_CHANNEL_VBAT
};
#endif

int dac_local_open(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(dac);
  if (state->ref_count == 0) {

    DEVFS_DRIVER_OPEN_STATE_LOCAL_V4(dac);
    state->hal_handle.Instance = m_dac_regs_table[config->port];

    switch (config->port) {
    case 0:
#if defined __HAL_RCC_DAC1_CLK_ENABLE
      __HAL_RCC_DAC1_CLK_ENABLE();
#elif defined __HAL_RCC_DAC_CLK_ENABLE
      __HAL_RCC_DAC_CLK_ENABLE();
#elif defined __HAL_RCC_DAC12_CLK_ENABLE
      __HAL_RCC_DAC12_CLK_ENABLE();
#else
#error("__HAL_RCC_DAC_CLK_ENABLE is not defined")
#endif
      break;
    case 1:
      if (m_dac_regs_table[1] == m_dac_regs_table[0]) {
#if defined __HAL_RCC_DAC1_CLK_ENABLE
        __HAL_RCC_DAC1_CLK_ENABLE();
#elif defined __HAL_RCC_DAC_CLK_ENABLE
        __HAL_RCC_DAC_CLK_ENABLE();
#elif defined __HAL_RCC_DAC12_CLK_ENABLE
        __HAL_RCC_DAC12_CLK_ENABLE();
#else
#error("__HAL_RCC_DAC_CLK_ENABLE is not defined")
#endif
      }
#if defined __HAL_RCC_DAC2_CLK_ENABLE
      __HAL_RCC_DAC2_CLK_ENABLE();
#endif
      break;
#if defined __HAL_RCC_DAC3_CLK_ENABLE
    case 2:
      __HAL_RCC_DAC3_CLK_ENABLE();
      break;
#endif
#if defined __HAL_RCC_DAC4_CLK_ENABLE
    case 3:
      __HAL_RCC_DAC4_CLK_ENABLE();
      break;
#endif
    }
    cortexm_enable_irq(m_dac_irqs[config->port]);
  }
  state->ref_count++;

  return 0;
}

int dac_local_close(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(dac);
  if (state->ref_count > 0) {
    if (state->ref_count == 1) {
      cortexm_disable_irq(m_dac_irqs[config->port]);
      switch (config->port) {
      case 0:
#if defined __HAL_RCC_DAC1_CLK_DISABLE
        __HAL_RCC_DAC1_CLK_DISABLE();
#elif defined __HAL_RCC_DAC_CLK_DISABLE
        __HAL_RCC_DAC_CLK_DISABLE();
#elif defined __HAL_RCC_DAC12_CLK_DISABLE
        __HAL_RCC_DAC12_CLK_DISABLE();
#else
#error("__HAL_RCC_DAC_CLK_DISABLE not defined")
#endif
        break;
#if defined __HAL_RCC_DAC2_CLK_DISABLE
      case 1:
        __HAL_RCC_DAC2_CLK_DISABLE();
        break;
#endif
#if defined __HAL_RCC_DAC3_CLK_DISABLE
      case 2:
        __HAL_RCC_DAC3_CLK_DISABLE();
        break;
#endif
#if defined __HAL_RCC_DAC4_CLK_DISABLE
      case 3:
        __HAL_RCC_DAC4_CLK_DISABLE();
        break;
#endif
      }
      state->hal_handle.Instance = 0;
      DEVFS_DRIVER_CLOSE_STATE_LOCAL_V4(dac);
    }
    state->ref_count--;
  }
  return 0;
}

int dac_local_getinfo(const devfs_handle_t *handle, void *ctl) {

  dac_info_t *info = ctl;
  const dac_config_t *config = handle->config;
  // dac_state_t * local = dac_local + config->port;

  info->o_flags = DAC_FLAG_IS_LEFT_JUSTIFIED | DAC_FLAG_IS_RIGHT_JUSTIFIED
                  | DAC_FLAG_SET_CONVERTER;
  info->o_events = MCU_EVENT_FLAG_DATA_READY;
  info->maximum = (1 << 12) - 1; // max value
  info->freq = 1000000; // max frequency
  info->bytes_per_sample = 2;

  if (config) {
    info->reference_mv = config->reference_mv;
  } else {
    info->reference_mv = 3000;
  }

  return SYSFS_RETURN_SUCCESS;
}

int dac_local_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(dac);
  u32 o_flags;
  const dac_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(dac, ctl);
  if (attr == NULL) {
    return SYSFS_SET_RETURN(EINVAL);
  }

  o_flags = attr->o_flags;

  if (o_flags & DAC_FLAG_SET_CONVERTER) {
    if (HAL_DAC_Init(&state->hal_handle) != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }
  }

  if (o_flags & DAC_FLAG_SET_CHANNELS) {
    // pin assignments

    sos_debug_printf(
      "PA:%d.%d %d.%d %d.%d %d.%d\n",
      attr->pin_assignment.channel[0].port,
      attr->pin_assignment.channel[0].pin,
      attr->pin_assignment.channel[1].port,
      attr->pin_assignment.channel[1].pin,
      attr->pin_assignment.channel[2].port,
      attr->pin_assignment.channel[2].pin,
      attr->pin_assignment.channel[3].port,
      attr->pin_assignment.channel[3].pin);

    if (
      mcu_set_pin_assignment(
        &(attr->pin_assignment),
        MCU_CONFIG_PIN_ASSIGNMENT(dac_config_t, handle),
        MCU_PIN_ASSIGNMENT_COUNT(dac_pin_assignment_t),
        CORE_PERIPH_DAC,
        config->port,
        0,
        0,
        0)
      < 0) {
      return SYSFS_SET_RETURN(EINVAL);
    }

    DAC_ChannelConfTypeDef channel_config;
    u32 channel;

    channel = m_dac_channels[config->port];

    channel_config.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    channel_config.DAC_Trigger = DAC_TRIGGER_NONE;
    channel_config.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
    channel_config.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;

    channel_config.DAC_UserTrimming = DAC_TRIMMING_FACTORY;

    if (o_flags & DAC_FLAG_IS_SAMPLE_AND_HOLD) {
      channel_config.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_ENABLE;
    }
    if (o_flags & DAC_FLAG_IS_OUTPUT_BUFFERED) {
      channel_config.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    }
    if (o_flags & DAC_FLAG_IS_ON_CHIP) {
      channel_config.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
    }

    if (state->o_flags & DAC_LOCAL_FLAG_IS_DMA) {
      // DMA requires an external trigger

      if (o_flags & DAC_FLAG_IS_TRIGGER_EINT) {
        channel_config.DAC_Trigger = DAC_TRIGGER_EXT_IT9;

      } else if (o_flags & DAC_FLAG_IS_TRIGGER_TMR) {

        switch (attr->trigger.port) {
        case 1: // TIM2
          channel_config.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
          break;
#if defined DAC_TRIGGER_T4_TRGO
        case 3: // TIM4
          channel_config.DAC_Trigger = DAC_TRIGGER_T4_TRGO;
          break;
#endif
#if defined DAC_TRIGGER_T5_TRGO
        case 4: // TIM5
          channel_config.DAC_Trigger = DAC_TRIGGER_T5_TRGO;
          break;
#endif
        case 5: // TIM6
          channel_config.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
          break;
        case 6: // TIM7
          channel_config.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
          break;
#if defined DAC_TRIGGER_T8_TRGO
        case 7: // TIM8
          channel_config.DAC_Trigger = DAC_TRIGGER_T8_TRGO;
          break;
#endif
#if defined DAC_TRIGGER_T15_TRGO
        case 14: // TIM15
          channel_config.DAC_Trigger = DAC_TRIGGER_T15_TRGO;
          break;
#endif

        default:
          return SYSFS_SET_RETURN(EINVAL);
        }

        sos_debug_log_info(
          SOS_DEBUG_DEVICE,
          "Trigger on tmr %d",
          attr->trigger.port);
      }
    }

    sos_debug_log_info(
      SOS_DEBUG_DEVICE,
      "config port:%d channel:0x%X",
      config->port,
      channel);
    if (
      HAL_DAC_ConfigChannel(&state->hal_handle, &channel_config, channel)
      != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }
  }

  return SYSFS_RETURN_SUCCESS;
}

int dac_local_get(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(dac);
  u32 channel;
  mcu_channel_t *mcu_channel = ctl;

  if (config->port < MCU_DAC_PORTS) {
    channel = m_dac_channels[config->port];
  } else {
    return SYSFS_SET_RETURN(ENOSYS);
  }

  mcu_channel->value = HAL_DAC_GetValue(&state->hal_handle, channel);

  return SYSFS_RETURN_SUCCESS;
}

u32 dac_local_get_alignment(dac_state_t *dac) {
  if (dac->o_flags & DAC_FLAG_IS_LEFT_JUSTIFIED) {
    return DAC_ALIGN_12B_L;
  }

  return DAC_ALIGN_12B_R;
}

int dac_local_set(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(dac);
  u32 channel;
  const mcu_channel_t *mcu_channel = ctl;

  if (config->port < MCU_DAC_PORTS) {
    channel = m_dac_channels[config->port];
  } else {
    return SYSFS_SET_RETURN(ENOSYS);
  }

  sos_debug_printf(
    "set channel value to %d.%d\n",
    mcu_channel->loc,
    mcu_channel->value);
  if (
    HAL_DAC_SetValue(
      &state->hal_handle,
      channel,
      dac_local_get_alignment(state),
      mcu_channel->value)
    != HAL_OK) {
    return SYSFS_SET_RETURN(EIO);
  }

  if (HAL_DAC_Start(&state->hal_handle, channel) != HAL_OK) {
    return SYSFS_SET_RETURN(EIO);
  }

  return SYSFS_RETURN_SUCCESS;
}

void HAL_DAC_ErrorCallback(DAC_HandleTypeDef *hdac) {
  dac_state_t *dac = (dac_state_t *)hdac;
  sos_debug_log_error(SOS_DEBUG_DEVICE, "DAC 1 Error %d", hdac->ErrorCode);
#if defined DAC_SR_OVR
  hdac->Instance->SR &= ~DAC_SR_OVR;
#endif
  devfs_execute_write_handler(
    &dac->transfer_handler,
    0,
    SYSFS_SET_RETURN(EIO),
    MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR);
  if ((dac->o_flags & DAC_LOCAL_FLAG_IS_DMA) == 0) {
    // HAL_DAC_Stop_IT(hdac);
  } else {
    // HAL_DAC_Stop_DMA(hdac);
  }
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {

  // MCU_EVENT_FLAG_HALF_TRANSFER

  dac_state_t *state = (dac_state_t *)hdac;

  // since this is streaming, the transfer handler is restored if requested
  devfs_async_t *async = state->transfer_handler.write;
  int result;

  result = devfs_execute_write_handler(
    &state->transfer_handler,
    0,
    2,
    MCU_EVENT_FLAG_LOW | MCU_EVENT_FLAG_WRITE_COMPLETE);
  if (result) {
    state->transfer_handler.write = async;
    if (state->hal_handle.DMA_Handle1) {
      sos_config.cache.clean_data_block(async->buf, async->nbyte / 2);
    }
  } else {
    HAL_DAC_Stop_DMA(hdac, DAC_CHANNEL_1);
    sos_debug_log_info(SOS_DEBUG_DEVICE, "STOP DMA H");
  }
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
  dac_state_t *state = (dac_state_t *)hdac;
  devfs_async_t *async = state->transfer_handler.write;
  int result;

  result = devfs_execute_write_handler(
    &state->transfer_handler,
    0,
    0,
    MCU_EVENT_FLAG_HIGH | MCU_EVENT_FLAG_WRITE_COMPLETE);
  if (result) {
    state->transfer_handler.write = async;
    if (state->hal_handle.DMA_Handle1) {
      sos_config.cache.clean_data_block(
        (char *)async->buf + async->nbyte / 2,
        async->nbyte / 2);
    }
  } else {
    HAL_DAC_Stop_DMA(hdac, DAC_CHANNEL_1);
    sos_debug_log_info(SOS_DEBUG_DEVICE, "STOP DMA");
  }
}

void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac) {
  dac_state_t *state = (dac_state_t *)hdac;
  devfs_async_t *async = state->transfer_handler.write;
  int result;

  result = devfs_execute_write_handler(
    &state->transfer_handler,
    0,
    0,
    MCU_EVENT_FLAG_HIGH | MCU_EVENT_FLAG_WRITE_COMPLETE);
  if (result) {
    state->transfer_handler.write = async;
    if (state->hal_handle.DMA_Handle2) {
      sos_config.cache.clean_data_block(
        (char *)async->buf + async->nbyte / 2,
        async->nbyte / 2);
    }
  } else {
    HAL_DAC_Stop_DMA(hdac, DAC_CHANNEL_2);
    sos_debug_log_info(SOS_DEBUG_DEVICE, "STOP DMA");
  }
}

void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac) {

  // MCU_EVENT_FLAG_HALF_TRANSFER
  dac_state_t *state = (dac_state_t *)hdac;

  // since this is streaming, the transfer handler is restored if requested
  devfs_async_t *async = state->transfer_handler.write;
  int result;

  result = devfs_execute_write_handler(
    &state->transfer_handler,
    0,
    2,
    MCU_EVENT_FLAG_LOW | MCU_EVENT_FLAG_WRITE_COMPLETE);
  if (result) {
    state->transfer_handler.write = async;
    if (state->hal_handle.DMA_Handle2) {
      sos_config.cache.clean_data_block(async->buf, async->nbyte / 2);
    }
  } else {

    HAL_DAC_Stop_DMA(hdac, DAC_CHANNEL_2);
    sos_debug_log_info(SOS_DEBUG_DEVICE, "STOP DMA H");
  }
}

void HAL_DACEx_ErrorCallbackCh2(DAC_HandleTypeDef *hdac) {
  dac_state_t *state = (dac_state_t *)hdac;
  sos_debug_log_error(SOS_DEBUG_DEVICE, "DAC 2 Error %d", hdac->ErrorCode);
#if defined DAC_SR_OVR
  hdac->Instance->SR &= ~DAC_SR_OVR;
#endif
  devfs_execute_write_handler(
    &state->transfer_handler,
    0,
    SYSFS_SET_RETURN(EIO),
    MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR);
  if ((state->o_flags & DAC_LOCAL_FLAG_IS_DMA) == 0) {
    // HAL_DAC_Stop_IT(hdac);
  } else {
    // HAL_DAC_Stop_DMA(hdac, DAC_CHANNEL_2);
  }
}

void HAL_DACEx_DMAUnderrunCallbackCh2(DAC_HandleTypeDef *hdac) {
  dac_state_t *state = (dac_state_t *)hdac;
  sos_debug_log_error(SOS_DEBUG_DEVICE, "DAC Under Error %d", hdac->ErrorCode);
  devfs_execute_write_handler(
    &state->transfer_handler,
    0,
    SYSFS_SET_RETURN(EIO),
    MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR);
  HAL_DAC_Stop_DMA(hdac, DAC_CHANNEL_2);
}

// shared with TIM6 -- called from mcu_core_tim6_dac_isr()
void mcu_core_dac_isr() {
  if (m_dac_state_list[0] != 0) {
    HAL_DAC_IRQHandler(&m_dac_state_list[0]->hal_handle);
  }
#if MCU_DAC_PORTS > 1
  HAL_DAC_IRQHandler(&m_dac_state_list[1]->hal_handle);
#endif
#if MCU_DAC_PORTS > 2
  if (m_dac_state_list[2]) {
    HAL_DAC_IRQHandler(&m_dac_state_list[2]->hal_handle);
  }
#endif
}

#endif
