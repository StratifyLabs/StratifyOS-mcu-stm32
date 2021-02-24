// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include <sos/config.h>

#include "tmr_local.h"

#if MCU_TMR_PORTS > 0

TIM_TypeDef *const tmr_local_regs_table[MCU_TMR_PORTS] = MCU_TMR_REGS;
u8 const tmr_local_irqs[MCU_TMR_PORTS] = MCU_TMR_IRQS;
u32 const tmr_local_channels[MCU_TMR_CHANNELS] = MCU_TMR_CHANNEL_NAMES;

tmr_state_t *m_tmr_state_list[MCU_TMR_PORTS] MCU_SYS_MEM;

static u8 tmr_local_decode_hal_channel(u8 channel) {
  switch (channel) {
  case HAL_TIM_ACTIVE_CHANNEL_1:
    return 0;
  case HAL_TIM_ACTIVE_CHANNEL_2:
    return 1;
  case HAL_TIM_ACTIVE_CHANNEL_3:
    return 2;
  case HAL_TIM_ACTIVE_CHANNEL_4:
    return 3;
#if MCU_TMR_API > 0
  case HAL_TIM_ACTIVE_CHANNEL_5:
    return 4;
  case HAL_TIM_ACTIVE_CHANNEL_6:
    return 5;
#endif
  }
  return 0;
}

static void clear_actions(tmr_state_t *state);
void clear_actions(tmr_state_t *state) {
  memset(&state->handler, 0, sizeof(mcu_event_handler_t));
  memset(&state->period_handler, 0, sizeof(mcu_event_handler_t));
}

int tmr_local_open(const devfs_handle_t *handle) {
  TMR_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);
  if (state->ref_count == 0) {
    DEVFS_DRIVER_OPEN_STATE_LOCAL_V4(tmr);
    clear_actions(state);
    //state->hal_handle = {0};
    state->hal_handle.Instance = tmr_local_regs_table[config->port];
    switch (config->port) {
    default:
      return SYSFS_SET_RETURN(EINVAL);
      TIM1_CASE_CLOCK_ENABLE()
      TIM2_CASE_CLOCK_ENABLE()
      TIM3_CASE_CLOCK_ENABLE()
      TIM4_CASE_CLOCK_ENABLE()
      TIM5_CASE_CLOCK_ENABLE()
      TIM6_CASE_CLOCK_ENABLE()
      TIM7_CASE_CLOCK_ENABLE()
      TIM8_CASE_CLOCK_ENABLE()
      TIM9_CASE_CLOCK_ENABLE()
      TIM10_CASE_CLOCK_ENABLE()
      TIM11_CASE_CLOCK_ENABLE()
      TIM12_CASE_CLOCK_ENABLE()
      TIM13_CASE_CLOCK_ENABLE()
      TIM14_CASE_CLOCK_ENABLE()
      TIM15_CASE_CLOCK_ENABLE()
      TIM16_CASE_CLOCK_ENABLE()
      TIM17_CASE_CLOCK_ENABLE()
    }
    if (tmr_local_irqs[config->port] != (u8)-1) {
      cortexm_enable_irq(tmr_local_irqs[config->port]);
    }
  }
  state->ref_count++;
  return 0;
}

int tmr_local_close(const devfs_handle_t *handle) {
  TMR_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);
  if (state->ref_count > 0) {
    if (state->ref_count == 1) {
      clear_actions(state);

      switch (DEVFS_DRIVER_PORT(tmr)) {
      default:
        return SYSFS_SET_RETURN(EINVAL);
        TIM1_CASE_CLOCK_DISABLE()
        TIM2_CASE_CLOCK_DISABLE()
        TIM3_CASE_CLOCK_DISABLE()
        TIM4_CASE_CLOCK_DISABLE()
        TIM5_CASE_CLOCK_DISABLE()
        TIM6_CASE_CLOCK_DISABLE()
        TIM7_CASE_CLOCK_DISABLE()
        TIM8_CASE_CLOCK_DISABLE()
        TIM9_CASE_CLOCK_DISABLE()
        TIM10_CASE_CLOCK_DISABLE()
        TIM11_CASE_CLOCK_DISABLE()
        TIM12_CASE_CLOCK_DISABLE()
        TIM13_CASE_CLOCK_DISABLE()
        TIM14_CASE_CLOCK_DISABLE()
        TIM15_CASE_CLOCK_DISABLE()
        TIM16_CASE_CLOCK_DISABLE()
        TIM17_CASE_CLOCK_DISABLE()
      }
      state->hal_handle.Instance = 0;
      const u8 interrupt_number = tmr_local_irqs[config->port];
      if (interrupt_number != (u8)-1) {
        cortexm_disable_irq(interrupt_number);
      }
      DEVFS_DRIVER_CLOSE_STATE_LOCAL_V4(tmr);
    }
    state->ref_count--;
  }
  return 0;
}

int tmr_local_getinfo(const devfs_handle_t *handle, void *ctl) {
  TMR_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);
  tmr_info_t *info = ctl;

  // set supported flags and events
  info->o_flags = TMR_FLAG_IS_AUTO_RELOAD | TMR_FLAG_SET_TIMER
                  | TMR_FLAG_IS_SOURCE_COUNTDOWN | TMR_FLAG_SET_CHANNEL
                  | TMR_FLAG_IS_CHANNEL_TOGGLE_OUTPUT_ON_MATCH
                  | TMR_FLAG_IS_CHANNEL_CLEAR_OUTPUT_ON_MATCH
                  | TMR_FLAG_IS_CHANNEL_SET_OUTPUT_ON_MATCH
                  | TMR_FLAG_IS_CHANNEL_PWM_MODE;
  info->o_events = 0;

  return 0;
}

int tmr_local_setattr(const devfs_handle_t *handle, void *ctl) {
  // attributes should be set by the non-shared part of the driver
  return 0;
}

int tmr_local_enable(const devfs_handle_t *handle, void *ctl) {
  TMR_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);
  if (state->period_handler.callback != 0) {
    HAL_TIM_Base_Start_IT(&state->hal_handle);
  } else {
    HAL_TIM_Base_Start(&state->hal_handle);
  }
  return SYSFS_RETURN_SUCCESS;
}

int tmr_local_disable(const devfs_handle_t *handle, void *ctl) {
  TMR_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);
  // HAL_TIM_Base_Stop_IT(&state->hal_handle);
  state->hal_handle.Instance->CR1 &= ~(TIM_CR1_CEN);
  return SYSFS_RETURN_SUCCESS;
}

int tmr_local_setchannel(const devfs_handle_t *handle, void *ctl) {
  TMR_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);
  TIM_TypeDef *regs = state->hal_handle.Instance;
  // Write the output compare value
  mcu_channel_t *req = (mcu_channel_t *)ctl;
  u8 chan;

  if (req->loc & MCU_CHANNEL_FLAG_IS_INPUT) {
    chan = req->loc & ~MCU_CHANNEL_FLAG_IS_INPUT;
    if (chan >= MCU_TMR_CHANNELS) {
      return SYSFS_SET_RETURN(EINVAL);
    }
  } else {
    chan = req->loc;
    if (chan >= MCU_TMR_CHANNELS) {
      return SYSFS_SET_RETURN(EINVAL);
    }
  }

  ((u32 *)&(regs->CCR1))[chan] = req->value;

  return SYSFS_RETURN_SUCCESS;
}

int tmr_local_getchannel(const devfs_handle_t *handle, void *ctl) {
  TMR_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);
  TIM_TypeDef *regs = state->hal_handle.Instance;
  mcu_channel_t *req = (mcu_channel_t *)ctl;
  u8 chan;

  if (req->loc & MCU_CHANNEL_FLAG_IS_INPUT) {
    chan = req->loc & ~MCU_CHANNEL_FLAG_IS_INPUT;
    if (chan >= MCU_TMR_CHANNELS) {
      return SYSFS_SET_RETURN(EINVAL);
    }
  } else {
    chan = req->loc;
    if (chan >= MCU_TMR_CHANNELS) {
      return SYSFS_SET_RETURN(EINVAL);
    }
  }

  req->value = ((u32 *)&(regs->CCR1))[chan];
  return SYSFS_RETURN_SUCCESS;
}

int tmr_local_write(const devfs_handle_t *handle, devfs_async_t *wop) {
  return SYSFS_SET_RETURN(ENOTSUP);
}

int tmr_local_setaction(const devfs_handle_t *handle, void *ctl) {
  TMR_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);
  mcu_action_t *action = (mcu_action_t *)ctl;
  // regs = tmr_local_regs_table[config->port];
  u32 chan;
  u32 o_events;
  u32 tim_channel;

  o_events = action->o_events;
  chan = action->channel & ~MCU_CHANNEL_FLAG_IS_INPUT;

  if (chan >= MCU_TMR_CHANNELS) {
    return SYSFS_SET_RETURN(EINVAL);
  }

  tim_channel = tmr_local_channels[chan];

  if (o_events == MCU_EVENT_FLAG_NONE) { // Check to see if all actions are
    // disabled for the channel
    state->handler[chan].callback = 0;
    if (action->channel & MCU_CHANNEL_FLAG_IS_INPUT) {
      HAL_TIM_IC_Stop_IT(&state->hal_handle, tim_channel);
    } else {
      HAL_TIM_OC_Stop_IT(&state->hal_handle, tim_channel);
    }

    // execute a cancelled callback

  } else if (o_events & MCU_EVENT_FLAG_MATCH) {

    if (action->channel & MCU_CHANNEL_FLAG_IS_INPUT) {

      if (action->handler.callback != 0) {
        HAL_TIM_IC_Start_IT(&state->hal_handle, tim_channel);
      } else {
        HAL_TIM_IC_Stop_IT(&state->hal_handle, tim_channel);
      }

    } else {

      if (action->handler.callback != 0) {
        HAL_TIM_OC_Start_IT(&state->hal_handle, tim_channel);
      } else {
        HAL_TIM_OC_Stop_IT(&state->hal_handle, tim_channel);
      }
    }

    state->handler[chan] = action->handler;
  } else if (o_events & MCU_EVENT_FLAG_OVERFLOW) {

    if (action->handler.callback != 0) {
      HAL_TIM_Base_Start_IT(&state->hal_handle);
    } else {
      HAL_TIM_Base_Stop_IT(&state->hal_handle);
    }

    state->period_handler = action->handler;
  }

  return SYSFS_RETURN_SUCCESS;
}

int tmr_local_read(const devfs_handle_t *handle, devfs_async_t *rop) {
  return SYSFS_SET_RETURN(ENOTSUP);
}

int tmr_local_set(const devfs_handle_t *handle, void *ctl) {
  TMR_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);
  TIM_TypeDef *regs = state->hal_handle.Instance;
  regs->CNT = (u32)ctl;
  return SYSFS_RETURN_SUCCESS;
}

int tmr_local_get(const devfs_handle_t *handle, void *ctl) {
  TMR_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);
  TIM_TypeDef *regs = state->hal_handle.Instance;
  u32 *value = ctl;
  if (value) {
    *value = regs->CNT;
    return SYSFS_RETURN_SUCCESS;
  }
  return SYSFS_SET_RETURN(EINVAL);
}

int execute_handler(
  mcu_event_handler_t *handler,
  u32 o_events,
  u32 channel,
  u32 value) {
  tmr_event_t event;
  event.channel.loc = channel;
  event.channel.value = value;
  return devfs_execute_event_handler(handler, o_events, &event);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  // this happens on an overflow
  tmr_state_t *state = (tmr_state_t *)htim;
  execute_handler(
    &state->period_handler,
    MCU_EVENT_FLAG_OVERFLOW,
    -1,
    htim->Instance->ARR);
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
  // this happens on an output compare match
  tmr_state_t *state = (tmr_state_t *)htim;
  u8 channel = tmr_local_decode_hal_channel(htim->Channel);
  execute_handler(
    &state->handler[channel],
    MCU_EVENT_FLAG_MATCH,
    channel,
    htim->Instance->CNT);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  // this happens on an output compare match
  tmr_state_t *state = (tmr_state_t *)htim;
  u8 channel = tmr_local_decode_hal_channel(htim->Channel);
  execute_handler(
    &state->handler[channel],
    MCU_EVENT_FLAG_MATCH,
    channel | MCU_CHANNEL_FLAG_IS_INPUT,
    htim->Instance->CNT);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
  // this belongs in the PWM driver (NOT here)
}

void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim) {
}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim) {
}

// Four timers with 4 OC's and 2 IC's each
void tmr_isr(int port) {
  HAL_TIM_IRQHandler(&(m_tmr_state_list[port]->hal_handle));
}

#if defined TIM1
void mcu_core_tim1_cc_isr() { tmr_isr(0); }
#endif

#if defined TIM2
void mcu_core_tim2_isr() { tmr_isr(1); }
#endif

#if defined TIM3
void mcu_core_tim3_isr() { tmr_isr(2); }
#endif

#if defined TIM4
void mcu_core_tim4_isr() { tmr_isr(3); }
#endif

#if defined TIM5
void mcu_core_tim5_isr() { tmr_isr(4); }
#endif

void mcu_core_dac_isr() MCU_WEAK;
void mcu_core_dac_isr() {}
#if defined TIM6
void mcu_core_tim6_dac_isr() {
  // TIM6 is shared with the DAC
  mcu_core_dac_isr();
  if (m_tmr_state_list[5] != NULL) {
    tmr_isr(5);
  }
}
#endif

#if defined TIM7
void mcu_core_tim7_isr() { tmr_isr(6); }
#endif

#if defined TIM8
void mcu_core_tim8_cc_isr() { tmr_isr(7); }
#endif

#if defined TIM9
void mcu_core_tim1_brk_tim9_isr() { tmr_isr(8); }
#endif

#if defined TIM10
void mcu_core_tim1_up_tim10_isr() { tmr_isr(9); }
#endif

#if defined TIM11
void mcu_core_tim1_trg_com_tim11_isr() { tmr_isr(10); }
#endif

#if defined TIM12
void mcu_core_tim8_brk_tim12_isr() { tmr_isr(11); }
#endif

#if defined TIM13
void mcu_core_tim8_up_tim13_isr() { tmr_isr(12); }
#endif

#if defined TIM14
void mcu_core_tim8_trg_com_tim14_isr() { tmr_isr(13); }
#endif

#if defined TIM15
void mcu_core_tim15_isr() { tmr_isr(14); }
#endif

#if defined TIM16
void mcu_core_tim16_isr() { tmr_isr(15); }
#endif

#if defined TIM17
void mcu_core_tim17_isr() { tmr_isr(16); }
#endif

#endif
