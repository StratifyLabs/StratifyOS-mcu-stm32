/*
 * hal.h
 *
 *  Created on: Jun 29, 2017
 *      Author: tgil
 */

#ifndef HAL_H_
#define HAL_H_

#include <mcu/types.h>
#include <mcu/core.h>

#define USE_FULL_LL_DRIVER


#if defined __stm32f446xx
#include "mcu_stm32f446xx.h"
#endif

#include "stm32f4xx/stm32f4xx_hal_conf.h"

int hal_set_alternate_pin_function(mcu_pin_t pin, core_periph_t function, int periph_port, int mode);
GPIO_TypeDef * const hal_get_pio_regs(u8 port);
int hal_get_alternate_function(int gpio_port, int pin, core_periph_t function, int periph_port);

#endif /* HAL_H_ */
