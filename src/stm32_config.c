
#include <mcu/usb.h>

#include <sos/config.h>
#include <sos/dev/tmr.h>
#include <sos/fs/devfs.h>

#include "stm32_config.h"
#include "stm32_local.h"
#include "tmr/tmr_local.h"
#include "uart/uart_local.h"

const stm32_git_hash stm32_config_git_hash = {.git_hash = SOS_GIT_HASH};

extern u32 _unique_id;
void stm32_get_serial_number(mcu_sn_t *serial_number) {
  const u32 *serial_addr = &_unique_id;
  serial_number->sn[0] = serial_addr[0];
  serial_number->sn[1] = serial_addr[1];
  serial_number->sn[2] = serial_addr[2];
  serial_number->sn[3] = 0;
}

static tmr_state_t m_clock_tmr_state;
const tmr_config_t m_clock_tmr_config = {
  .port = 1,
  .attr = {
    .o_flags
    = TMR_FLAG_SET_TIMER | TMR_FLAG_IS_SOURCE_CPU | TMR_FLAG_IS_AUTO_RELOAD,
    .period = SOS_USECOND_PERIOD,
    .freq = 1000000UL,
    .pin_assignment = {
      .channel[0] = {0xff, 0xff},
      .channel[1] = {0xff, 0xff},
      .channel[2] = {0xff, 0xff},
      .channel[3] = {0xff, 0xff}}}};

static const devfs_handle_t m_clock_tmr_handle
  = {.state = &m_clock_tmr_state, .config = &m_clock_tmr_config};
void stm32_clock_initialize(
  int (*handle_match_channel0)(void *context, const mcu_event_t *data),
  int (*handle_match_channel1)(void *context, const mcu_event_t *data),
  int (*handle_overflow)(void *context, const mcu_event_t *data)) {
  // use TIM2 -- 32-bit timer

  mcu_action_t action;
  mcu_channel_t chan_req;

  // Open the microsecond timer
  mcu_tmr_open(&m_clock_tmr_handle);
  mcu_tmr_setattr(&m_clock_tmr_handle, NULL);

  // Initialize the value of the timer to zero
  mcu_tmr_set(&m_clock_tmr_handle, (void *)0);

  action.prio = 0;
  action.channel = 0; // doesn't matter
  action.o_events = MCU_EVENT_FLAG_OVERFLOW;
  action.handler.callback = handle_overflow;
  action.handler.context = 0;
  tmr_local_setaction(&m_clock_tmr_handle, &action);

  // This sets up the output compare unit used with the usleep() function
  chan_req.loc = 0;
  chan_req.value = SOS_USECOND_PERIOD + 1;
  stm32_clock_set_channel(&chan_req);

  action.channel = 0;
  action.o_events = MCU_EVENT_FLAG_MATCH;
  action.handler.callback = handle_match_channel0;
  action.handler.context = 0;
  tmr_local_setaction(&m_clock_tmr_handle, &action);

  chan_req.loc = 0;
  stm32_clock_set_channel(&chan_req);

  if (handle_match_channel1) {
    action.channel = 1;
    action.o_events = MCU_EVENT_FLAG_MATCH;
    action.handler.callback = handle_match_channel1;
    action.handler.context = 0;
    tmr_local_setaction(&m_clock_tmr_handle, &action);
  }

  stm32_clock_enable();
}

void stm32_clock_enable() {
  HAL_TIM_Base_Start_IT(&m_clock_tmr_state.hal_handle);
}

u32 stm32_clock_disable() {
  m_clock_tmr_state.hal_handle.Instance->CR1 &= ~(TIM_CR1_CEN);
  return m_clock_tmr_state.hal_handle.Instance->CNT;
}

void stm32_clock_set_channel(const mcu_channel_t *channel) {
  tmr_local_setchannel(&m_clock_tmr_handle, (void *)channel);
}

void stm32_clock_get_channel(mcu_channel_t *channel) {
  tmr_local_getchannel(&m_clock_tmr_handle, (void *)channel);
}

u32 stm32_clock_microseconds() {
  return m_clock_tmr_state.hal_handle.Instance->CNT;
}

u32 stm32_clock_nanoseconds() { return 0; }

static uart_state_t m_debug_uart_state;

static const devfs_handle_t m_debug_uart_handle
  = {.state = &m_debug_uart_state, .config = &stm32_config.debug_uart_config};

void stm32_debug_initialize() {
  uart_local_open(&m_debug_uart_handle);
  uart_local_setattr(&m_debug_uart_handle, NULL);
}

void stm32_debug_write(const void *buf, int nbyte) {
  const char *cbuf = buf;
  for (int i = 0; i < nbyte; i++) {
    char c = cbuf[i];
    HAL_UART_Transmit(&m_debug_uart_state.hal_handle, &c, 1, HAL_MAX_DELAY);
  }
}

void stm32_pio_set_attributes(int port, const pio_attr_t *attr) {
  hal_pio_setattr(port, (void *)attr);
}

void stm32_pio_write(int port, u32 mask, int value) {
  if (value) {
    hal_pio_setmask(port, (void *)mask);
  } else {
    hal_pio_clrmask(port, (void *)mask);
  }
}

u32 stm32_pio_read(int port, u32 mask) {
  u32 value;
  hal_pio_get(port, &value);
  return (mask & value);
}

int stm32_usb_set_attributes(const devfs_handle_t *handle, void *ctl) {
  return mcu_usb_setattr(handle, ctl);
}

int stm32_usb_set_action(const devfs_handle_t *handle, mcu_action_t *action) {
  return mcu_usb_setaction(handle, action);
}

void stm32_usb_write_endpoint(
  const devfs_handle_t *handle,
  u32 endpoint_num,
  const void *src,
  u32 size) {
  mcu_usb_root_write_endpoint(handle, endpoint_num, src, size);
}

int stm32_usb_read_endpoint(
  const devfs_handle_t *handle,
  u32 endpoint_num,
  void *dest) {
  return mcu_usb_root_read_endpoint(handle, endpoint_num, dest);
}
