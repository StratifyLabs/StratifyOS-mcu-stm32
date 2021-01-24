// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include <sos/config.h>

#include "tmr_local.h"

#if MCU_TMR_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(
  tmr_pwm,
  TMR_VERSION,
  TMR_IOC_IDENT_CHAR,
  I_MCU_TOTAL + I_TMR_TOTAL,
  mcu_tmr_pwm_setchannel,
  mcu_tmr_pwm_getchannel,
  mcu_tmr_pwm_set,
  mcu_tmr_pwm_get,
  mcu_tmr_pwm_enable,
  mcu_tmr_pwm_disable)

int mcu_tmr_pwm_open(const devfs_handle_t *handle) {
  return tmr_local_open(handle);
}

int mcu_tmr_pwm_close(const devfs_handle_t *handle) {
  return tmr_local_close(handle);
}

int mcu_tmr_pwm_getinfo(const devfs_handle_t *handle, void *ctl) {
  int result = tmr_local_getinfo(handle, ctl);

  tmr_info_t *info = ctl;
  info->o_flags = TMR_FLAG_SET_TIMER | TMR_FLAG_IS_CHANNEL_PWM_MODE
                  | TMR_FLAG_SET_CHANNEL
                  | TMR_FLAG_IS_CHANNEL_SET_OUTPUT_ON_MATCH;

  return result;
}

int mcu_tmr_pwm_setattr(const devfs_handle_t *handle, void *ctl) {
  DEVFS_DRIVER_DECLARE_CONFIG_STATE(tmr);

  const tmr_attr_t *attr = DEVFS_ASSIGN_ATTRIBUTES(tmr, ctl);

  if (attr == NULL) {
    return -1;
  }

  u32 o_flags = attr->o_flags;
  u32 freq = attr->freq;
  // regs = tmr_local_regs_table[config->port];

  if (o_flags & TMR_FLAG_SET_TIMER) {

    // Set the prescalar so that the freq is correct
    // get the peripheral clock frequency

    u32 pclk;
    if ((((u32)state->hal_handle.Instance) & ~0xFFFF) == APB1PERIPH_BASE) {
      pclk = HAL_RCC_GetPCLK1Freq();
    } else {
      pclk = HAL_RCC_GetPCLK2Freq();
    }

    if (pclk <= sos_config.clock.frequency / 2) {
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

    state->hal_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    state->hal_handle.Init.Period = attr->period;
    state->hal_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

#if defined TIM_AUTORELOAD_PRELOAD_DISABLE
    state->hal_handle.Init.RepetitionCounter = 0x00;
    state->hal_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
#endif

    if (HAL_TIM_Base_Init(&state->hal_handle) != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }

    // configure to use the internal clock
    TIM_ClockConfigTypeDef clock_source_config;
    clock_source_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (
      HAL_TIM_ConfigClockSource(&state->hal_handle, &clock_source_config)
      != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }
  }

  if (o_flags & TMR_FLAG_SET_CHANNEL) {
    TIM_OC_InitTypeDef init_oc;

    const int chan = attr->channel.loc & ~MCU_CHANNEL_FLAG_IS_INPUT;
    if (chan >= MCU_TMR_CHANNELS) {
      return SYSFS_SET_RETURN(EINVAL);
    }

    const u32 tim_channel = tmr_local_channels[chan];
    init_oc.OCMode = TIM_OCMODE_PWM1; // output is ON until the value is hit
    if (o_flags & TMR_FLAG_IS_CHANNEL_SET_OUTPUT_ON_MATCH) {
      // output if OFF until the value hits
      init_oc.OCMode = TIM_OCMODE_PWM2;
    }

    init_oc.Pulse = 0;
    init_oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    init_oc.OCFastMode = TIM_OCFAST_DISABLE;

    int ret = HAL_TIM_PWM_Init(&state->hal_handle);
    if (ret != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }
    ret = HAL_TIM_PWM_ConfigChannel(&state->hal_handle, &init_oc, tim_channel);
    if (ret != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }
    ret = HAL_TIM_PWM_Start(&state->hal_handle, tim_channel);

    if (ret != HAL_OK) {
      return SYSFS_SET_RETURN(EIO);
    }

    // this sets the value of the channel
    if (tmr_local_setchannel(handle, (void *)&attr->channel) < 0) {
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

int mcu_tmr_pwm_enable(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_enable(handle, ctl);
}

int mcu_tmr_pwm_disable(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_disable(handle, ctl);
}

int mcu_tmr_pwm_setchannel(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_setchannel(handle, ctl);
}

int mcu_tmr_pwm_getchannel(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_getchannel(handle, ctl);
}

int mcu_tmr_pwm_write(const devfs_handle_t *handle, devfs_async_t *wop) {
  return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_tmr_pwm_setaction(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_setaction(handle, ctl);
}

int mcu_tmr_pwm_read(const devfs_handle_t *handle, devfs_async_t *rop) {
  return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_tmr_pwm_set(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_set(handle, ctl);
}

int mcu_tmr_pwm_get(const devfs_handle_t *handle, void *ctl) {
  return tmr_local_get(handle, ctl);
}

#endif
