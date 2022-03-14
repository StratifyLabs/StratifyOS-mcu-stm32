//Copyright (c) 2011-2022 Tyler Gilbert and Stratify Labs, Inc. See LICENSE.md for details

#ifndef STM32_CONFIG_H
#define STM32_CONFIG_H

#include <sdk/types.h>

#include <mcu/uart.h>
#include <sos/dev/pio.h>

void stm32_initialize();
void stm32_initialize_systick();
void stm32_get_serial_number(mcu_sn_t *serial_number);

// mcu
int stm32_mcu_set_pin_function(
  const mcu_pin_t *pin,
  int periph,
  int periph_port);

// clock
void stm32_clock_initialize(
  int (*handle_match_channel0)(void *, const mcu_event_t *),
  int (*handle_match_channel1)(void *, const mcu_event_t *),
  int (*handle_overflow)(void *, const mcu_event_t *));
void stm32_clock_enable();
u32 stm32_clock_disable();
void stm32_clock_set_channel(const mcu_channel_t *channel);
void stm32_clock_get_channel(mcu_channel_t *channel);
u32 stm32_clock_microseconds();
u32 stm32_clock_nanoseconds();

// debug
void stm32_debug_initialize();
void stm32_debug_write(const void *buf, int nbyte);

// pio
void stm32_pio_set_attributes(int port, const pio_attr_t *attr);
void stm32_pio_write(int port, u32 mask, int value);
u32 stm32_pio_read(int port, u32 mask);

// usb
int stm32_usb_set_attributes(const devfs_handle_t *handle, void *ctl);
int stm32_usb_set_action(const devfs_handle_t *handle, mcu_action_t *action);
void stm32_usb_write_endpoint(
  const devfs_handle_t *handle,
  u32 endpoint_num,
  const void *src,
  u32 size);
int stm32_usb_read_endpoint(
  const devfs_handle_t *handle,
  u32 endpoint_num,
  void *dest);

enum {
  STM32_CONFIG_FLAG_IS_CACHE_ENABLED = (1 << 0),
  STM32_CONFIG_FLAG_IS_DEBUG_LED_ACTIVE_HIGH = (1 << 1)
};

#define USB_TX_FIFO_WORD_SIZE_COUNT 9

typedef struct {
  void *rx_buffer;
  u16 rx_buffer_size;
  u16 rx_fifo_word_size /*! RX FIFO word size for all endpoints (STM32) */;
  u8 tx_fifo_word_size
    [USB_TX_FIFO_WORD_SIZE_COUNT] /*! TX FIFO word size (used on STM32) */;
} stm32_usb_config_t;

typedef struct MCU_PACK {
  u16 flash_program_millivolts;
  u16 flags;
  u32 oscillator_frequency;
  stm32_usb_config_t usb;
} stm32_config_t;

// must be defined and provided by board support package
extern const stm32_config_t stm32_config;

typedef struct {
  const char *git_hash;
} stm32_git_hash_t;

extern const stm32_git_hash_t stm32_config_git_hash;

#endif // STM32_CONFIG_H
