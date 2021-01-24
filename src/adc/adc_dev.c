// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include "adc_local.h"
#include <fcntl.h>

#if MCU_ADC_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(adc, ADC_VERSION, ADC_IOC_IDENT_CHAR)

int mcu_adc_open(const devfs_handle_t *handle) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(adc);
  state->o_flags = 0;
  return adc_local_open(handle);
}

int mcu_adc_close(const devfs_handle_t *handle) {
  return adc_local_close(handle);
}

int mcu_adc_getinfo(const devfs_handle_t *handle, void *ctl) {
  return adc_local_getinfo(handle, ctl);
}

int mcu_adc_setattr(const devfs_handle_t *handle, void *ctl) {
  return adc_local_setattr(handle, ctl);
}

int mcu_adc_setaction(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(adc);
  mcu_action_t *action = (mcu_action_t *)ctl;

  if (action->handler.callback == 0) {
    // if there is an ongoing operation -- cancel it
    if (action->o_events & MCU_EVENT_FLAG_DATA_READY) {
      // execute the read callback if not null
      devfs_execute_read_handler(
        &state->transfer_handler,
        0,
        SYSFS_SET_RETURN(EAGAIN),
        MCU_EVENT_FLAG_CANCELED);
      HAL_ADC_Stop_IT(&state->hal_handle);
    }
  }

  cortexm_set_irq_priority(
    adc_irqs[config->port],
    action->prio,
    action->o_events);
  return 0;
}

int mcu_adc_read(const devfs_handle_t *handle, devfs_async_t *async) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(adc);

  DEVFS_DRIVER_IS_BUSY(state->transfer_handler.read, async);

  if (async->nbyte < 2) {
    state->transfer_handler.read = NULL;
    return SYSFS_SET_RETURN(EINVAL);
  }

  // if location is not the group value -- configure the channel to read the
  // group
  if ((u32)async->loc < MCU_ADC_CHANNELS) {
    // configure the channel to read
    ADC_ChannelConfTypeDef channel_config = {0};
    channel_config.Channel = adc_channels[async->loc];
#if defined ADC_REGULAR_RANK_1
    channel_config.Rank = ADC_REGULAR_RANK_1;
#else
    channel_config.Rank = 1;
#endif
#if defined ADC_SAMPLETIME_15CYCLES
    channel_config.SamplingTime = ADC_SAMPLETIME_15CYCLES;
#elif defined ADC_SAMPLETIME_32CYCLES_5
    channel_config.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
#endif

#if defined ADC_SINGLE_ENDED
    channel_config.SingleDiff = ADC_SINGLE_ENDED;
#endif

#if defined ADC_OFFSET_NONE
    channel_config.OffsetNumber = ADC_OFFSET_NONE;
#endif

    if (HAL_ADC_ConfigChannel(&state->hal_handle, &channel_config) != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }
  }

  state->words_read = 0;
  async->nbyte &= ~0x01; // align to 2 byte boundary

  if (HAL_ADC_Start_IT(&state->hal_handle) == HAL_OK) {
    return 0;
  }

  // this needs to read 1 byte at a time
  state->transfer_handler.read = NULL;
  return SYSFS_SET_RETURN(EIO);
}

int mcu_adc_write(const devfs_handle_t *handle, devfs_async_t *async) {
  return SYSFS_SET_RETURN(ENOTSUP);
}

#endif
