// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include <sos/config.h>

#include "tmr_local.h"

#define NUM_OCS MCU_TMR_CHANNELS
#define NUM_ICS MCU_TMR_CHANNELS

#if MCU_TMR_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  tmr,
  TMR_VERSION,
  TMR_IOC_IDENT_CHAR,
  I_MCU_TOTAL + I_TMR_TOTAL,
  mcu_tmr_setchannel,
  mcu_tmr_getchannel,
  mcu_tmr_set,
  mcu_tmr_get,
  mcu_tmr_enable,
  mcu_tmr_disable)

int mcu_tmr_open(const devfs_handle_t *handle) {
  return tmr_local_open(handle);
}

int mcu_tmr_close(const devfs_handle_t *handle) {
  return tmr_local_close(handle);
}

int mcu_tmr_getinfo(const devfs_handle_t *handle, void *ctl) {
  int result = tmr_local_getinfo(handle, ctl);

  return result;
}

int mcu_tmr_setattr(const devfs_handle_t *handle, void *ctl) {
  TMR_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);

  const tmr_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(tmr, ctl);

  if (attr == NULL) {
    return -1;
  }

  u32 o_flags = attr->o_flags;
  u32 freq = attr->freq;
  // regs = tmr_local_regs_table[config->port];

  if (o_flags & TMR_FLAG_SET_TIMER) {

    if (
      (o_flags
       & (TMR_FLAG_IS_SOURCE_CPU | TMR_FLAG_IS_SOURCE_IC0 | TMR_FLAG_IS_SOURCE_IC1))) {

      if (o_flags & TMR_FLAG_IS_SOURCE_CPU) {
        if (attr->freq == 0) {
          freq = 1000000;
        }

      } else {
        if (o_flags & TMR_FLAG_IS_SOURCE_EDGEFALLING) {

        } else if (o_flags & TMR_FLAG_IS_SOURCE_EDGEBOTH) {
        }
      }

      if (o_flags & TMR_FLAG_IS_SOURCE_IC1) {
      }

      // Set the prescalar so that the freq is correct
      // get the peripheral clock frequency

      u32 pclk;
      if ((((u32)state->hal_handle.Instance) & ~0xFFFF) == APB1PERIPH_BASE) {
        pclk = HAL_RCC_GetPCLK1Freq();
      } else {
        pclk = HAL_RCC_GetPCLK2Freq();
      }

      if (pclk <= sos_config.sys.core_clock_frequency / 2) {
        // timer clocks are double pclks
        pclk = pclk * 2;
      }

      sos_debug_log_info(SOS_DEBUG_DEVICE, "Use pclk is %ld", pclk);

      if (freq < pclk * 2) {
        state->hal_handle.Init.Prescaler = ((pclk + freq / 2) / freq) - 1;
      } else {
        state->hal_handle.Init.Prescaler = 0;
      }

      if (state->hal_handle.Init.Prescaler > 0xffff) {
        state->hal_handle.Init.Prescaler = 0xffff;
      }
      sos_debug_log_info(
        SOS_DEBUG_DEVICE,
        "Prescaler:%ld",
        state->hal_handle.Init.Prescaler);

      if (o_flags & TMR_FLAG_IS_SOURCE_COUNTDOWN) {
        state->hal_handle.Init.CounterMode = TIM_COUNTERMODE_DOWN;
      } else {
        state->hal_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
      }

      if (o_flags & TMR_FLAG_IS_AUTO_RELOAD) {
        state->hal_handle.Init.Period = attr->period;
      } else {
        state->hal_handle.Init.Period = (u32)-1; // set to the max
      }
      state->hal_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

#if defined TIM_AUTORELOAD_PRELOAD_DISABLE
      state->hal_handle.Init.RepetitionCounter = 0x00;
      state->hal_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
#endif

      if (HAL_TIM_Base_Init(&state->hal_handle) != HAL_OK) {
        return SYSFS_SET_RETURN(EIO);
      }

      if (o_flags & TMR_FLAG_IS_SOURCE_CPU) {
        // configure to use the internal clock
        TIM_ClockConfigTypeDef clock_source_config;
        clock_source_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
        if (
          HAL_TIM_ConfigClockSource(&state->hal_handle, &clock_source_config)
          != HAL_OK) {
          return SYSFS_SET_RETURN(EIO);
        }
      }
    }
  }

  if (o_flags & TMR_FLAG_SET_TRIGGER) {

    if (o_flags & TMR_FLAG_IS_SLAVE) {

    } else {
      TIM_MasterConfigTypeDef master_config;

      master_config.MasterOutputTrigger = TIM_TRGO_RESET;
      if (o_flags & TMR_FLAG_IS_TRIGGER_RELOAD) {
        master_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
      } else if (o_flags & TMR_FLAG_IS_TRIGGER_OC0) {
        master_config.MasterOutputTrigger = TIM_TRGO_OC1REF;
      } else if (o_flags & TMR_FLAG_IS_TRIGGER_OC1) {
        master_config.MasterOutputTrigger = TIM_TRGO_OC2REF;
      } else if (o_flags & TMR_FLAG_IS_TRIGGER_OC2) {
        master_config.MasterOutputTrigger = TIM_TRGO_OC3REF;
      } else if (o_flags & TMR_FLAG_IS_TRIGGER_OC3) {
        master_config.MasterOutputTrigger = TIM_TRGO_OC4REF;
      }

      if (o_flags & TMR_FLAG_IS_MASTER) {
        master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
      } else {
        master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
      }
      if (
        HAL_TIMEx_MasterConfigSynchronization(
          &state->hal_handle,
          &master_config)
        != HAL_OK) {
        return SYSFS_SET_RETURN(EIO);
      }
    }
  }

  if (o_flags & TMR_FLAG_SET_CHANNEL) {
    int chan = attr->channel.loc & ~MCU_CHANNEL_FLAG_IS_INPUT;
    int channel_configure_type;
    int ret;
    TIM_OC_InitTypeDef init_oc;
    TIM_IC_InitTypeDef init_ic;

    if (chan >= MCU_TMR_CHANNELS) {
      return SYSFS_SET_RETURN(EINVAL);
    }

    channel_configure_type = CHANNEL_TYPE_NONE;

    u32 tim_channel = tmr_local_channels[chan];

    if (attr->channel.loc & MCU_CHANNEL_FLAG_IS_INPUT) {
      TIM_IC_InitTypeDef init_ic;
      channel_configure_type = CHANNEL_TYPE_INPUT_CAPTURE;
      memset(&init_ic, 0, sizeof(init_ic));

    } else {

      if (o_flags & TMR_FLAG_IS_CHANNEL_PWM_MODE) {
        channel_configure_type = CHANNEL_TYPE_PWM;
        init_oc.OCMode = TIM_OCMODE_PWM1; // output is ON until the value is hit
        if (o_flags & TMR_FLAG_IS_CHANNEL_SET_OUTPUT_ON_MATCH) {
          init_oc.OCMode = TIM_OCMODE_PWM2; // output if OFF until the value
                                            // hits
        }
      } else if (o_flags & TMR_FLAG_IS_CHANNEL_TOGGLE_OUTPUT_ON_MATCH) {
        channel_configure_type = CHANNEL_TYPE_OUTPUT_COMPARE;
        init_oc.OCMode = TIM_OCMODE_TOGGLE;
      } else if (o_flags & TMR_FLAG_IS_CHANNEL_SET_OUTPUT_ON_MATCH) {
        channel_configure_type = CHANNEL_TYPE_OUTPUT_COMPARE;
        init_oc.OCMode = TIM_OCMODE_ACTIVE;
      } else if (o_flags & TMR_FLAG_IS_CHANNEL_CLEAR_OUTPUT_ON_MATCH) {
        channel_configure_type = CHANNEL_TYPE_OUTPUT_COMPARE;
        init_oc.OCMode = TIM_OCMODE_INACTIVE;
      } else {
        init_oc.OCMode = TIM_OCMODE_TIMING;
      }

      ret = 0;

      init_oc.Pulse = 0;
      init_oc.OCPolarity = TIM_OCPOLARITY_HIGH;
      init_oc.OCFastMode = TIM_OCFAST_DISABLE;
    }

    switch (channel_configure_type) {
    case CHANNEL_TYPE_OUTPUT_COMPARE:
      ret = HAL_TIM_OC_Init(&state->hal_handle);
      if (ret != HAL_OK) {
        return SYSFS_SET_RETURN(EIO);
      }
      ret = HAL_TIM_OC_ConfigChannel(&state->hal_handle, &init_oc, tim_channel);
      if (ret != HAL_OK) {
        return SYSFS_SET_RETURN(EIO);
      }
      ret = HAL_TIM_OC_Start(&state->hal_handle, tim_channel);
      break;

    case CHANNEL_TYPE_PWM:
      ret = HAL_TIM_PWM_Init(&state->hal_handle);
      if (ret != HAL_OK) {
        return SYSFS_SET_RETURN(EIO);
      }
      ret
        = HAL_TIM_PWM_ConfigChannel(&state->hal_handle, &init_oc, tim_channel);
      if (ret != HAL_OK) {
        return SYSFS_SET_RETURN(EIO);
      }
      ret = HAL_TIM_PWM_Start(&state->hal_handle, tim_channel);
      break;

    case CHANNEL_TYPE_INPUT_CAPTURE:
      ret = HAL_OK;
      ret = HAL_TIM_IC_Init(&state->hal_handle);
      if (ret != HAL_OK) {
        return SYSFS_SET_RETURN(EIO);
      }
      ret = HAL_TIM_IC_ConfigChannel(&state->hal_handle, &init_ic, tim_channel);
      if (ret != HAL_OK) {
        return SYSFS_SET_RETURN(EIO);
      }
      ret = HAL_TIM_IC_Start(&state->hal_handle, tim_channel);
      break;

    case CHANNEL_TYPE_NONE:
      HAL_TIM_OC_Stop(&state->hal_handle, tim_channel);
      HAL_TIM_PWM_Stop(&state->hal_handle, tim_channel);
      HAL_TIM_IC_Stop(&state->hal_handle, tim_channel);
      ret = HAL_OK;
      break;
    }

    if (ret != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }

    // this sets the value of the channel
    if (mcu_tmr_setchannel(handle, (void *)&attr->channel) < 0) {
      return SYSFS_SET_RETURN(EIO);
    }
  }

  if (o_flags & (TMR_FLAG_SET_TIMER | TMR_FLAG_SET_CHANNEL)) {
    if (
      mcu_set_pin_assignment(
        &(attr->pin_assignment),
        MCU_CONFIG_PIN_ASSIGNMENT(tmr_config_t, handle),
        MCU_PIN_ASSIGNMENT_COUNT(tmr_pin_assignment_t),
        CORE_PERIPH_TMR,
        DEVFS_DRIVER_PORT(tmr),
        0,
        0,
        0)
      < 0) {
      return SYSFS_SET_RETURN(EINVAL);
    }
  }

  return 0;
}

int mcu_tmr_enable(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_enable(handle, ctl);
}

int mcu_tmr_disable(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_disable(handle, ctl);
}

int mcu_tmr_setchannel(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_setchannel(handle, ctl);
}

int mcu_tmr_getchannel(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_getchannel(handle, ctl);
}

int mcu_tmr_write(const devfs_handle_t *handle, devfs_async_t *wop) {
  return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_tmr_setaction(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_setaction(handle, ctl);
}

int mcu_tmr_read(const devfs_handle_t *handle, devfs_async_t *rop) {
  return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_tmr_set(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_set(handle, ctl);
}

int mcu_tmr_get(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_get(handle, ctl);
}

#endif
