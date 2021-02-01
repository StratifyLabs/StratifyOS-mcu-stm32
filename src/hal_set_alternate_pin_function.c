// Copyright 2011-2021 Tyler Gilbert and Stratify Labs, Inc; see LICENSE.md

#include "stm32_local.h"
#include "stm32_pin_local.h"
#include <mcu/core.h>
#include <sos/debug.h>

int hal_get_alternate_function(
  int gpio_port,
  int pin,
  core_periph_t function,
  int periph_port) {
  int i;
  alternate_function_entry_t entry;
  u16 value = gpio_port * 16 + pin;
  i = -1;
  if (value < MCU_TOTAL_PINS) {
    entry = alternate_function_table[value];
    value = -1;
    for (i = 0; i < TOTAL_ENTRIES; i++) {
      if (
        (function == ENTRY_GET_FUNCTION(entry.entry[i]))
        && (periph_port == ENTRY_GET_PORT(entry.entry[i]))) {
        // this is a valid pin
        return i;
      }
    }
  }

  if (gpio_port != 0xff) {
    // sos_debug_log_warning(SOS_DEBUG_DEVICE, "Failed to resolve pin:%d.%d ->
    // %d.%d", gpio_port, pin, function, periph_port);
  }
  return -1;
}

int hal_set_alternate_pin_function(
  mcu_pin_t pin,
  core_periph_t function,
  int periph_port,
  int mode,
  int speed,
  int pull) {
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_TypeDef *gpio_regs;
  int alternate_function;

  gpio_regs = hal_get_pio_regs(pin.port);
  if (gpio_regs == 0) {
    return -1;
  }

  if (
    (mode != GPIO_MODE_ANALOG)
#if defined GPIO_MODE_ANALOG_ADC_CONTROL
    && (mode != GPIO_MODE_ANALOG_ADC_CONTROL)
#endif
  ) {
    alternate_function
      = hal_get_alternate_function(pin.port, pin.pin, function, periph_port);
    if (alternate_function < 0) {
      return -1;
    }
  } else {
    alternate_function = 0;
  }

  GPIO_InitStruct.Pin = (1 << pin.pin);
  GPIO_InitStruct.Mode = mode;
  GPIO_InitStruct.Pull = pull;
  GPIO_InitStruct.Speed = speed;
  GPIO_InitStruct.Alternate = alternate_function;
  HAL_GPIO_Init(gpio_regs, &GPIO_InitStruct);
  return 0;
}

int mcu_core_set_pinsel_func(
  const mcu_pin_t *pin,
  core_periph_t function,
  int periph_port) {
  int mode = GPIO_MODE_AF_PP;
  int speed = GPIO_SPEED_FREQ_LOW;
  int pull = GPIO_NOPULL;
  switch (function) {
  default:
    break;
  case CORE_PERIPH_I2C:
    mode = GPIO_MODE_AF_OD;
    break;
  case CORE_PERIPH_ADC:
  case CORE_PERIPH_DAC:
    mode = GPIO_MODE_ANALOG;
    break;
  case CORE_PERIPH_USB:
  case CORE_PERIPH_SPI:
  case CORE_PERIPH_I2S:
  case CORE_PERIPH_TMR:
    speed = GPIO_SPEED_FREQ_VERY_HIGH;
    break;
  case CORE_PERIPH_QSPI:
  case CORE_PERIPH_EMC:
  case CORE_PERIPH_SDIO:
    speed = GPIO_SPEED_FREQ_HIGH;
    pull = GPIO_PULLUP;
    break;
  }
  return hal_set_alternate_pin_function(
    *pin,
    function,
    periph_port,
    mode,
    speed,
    pull);
}
