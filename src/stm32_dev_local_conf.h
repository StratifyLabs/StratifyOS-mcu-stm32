#ifndef MCU_ARCH_STM32_STM32_DEV_LOCAL_CONF_H
#define MCU_ARCH_STM32_STM32_DEV_LOCAL_CONF_H

#include "stm32_arch.h"

#if defined STM32F2
#include "stm32f2xx/stm32f2xx_hal_conf.h"
#endif

#if defined STM32F4
#include "stm32f4xx/stm32f4xx_hal_conf.h"
#endif

#if defined STM32F7
#include "stm32f7xx/stm32f7xx_hal_conf.h"
#endif

#if defined STM32H7
#include "stm32h7xx/stm32h7xx_hal_conf.h"
#endif

#if defined STM32L4
#include "stm32l4xx/stm32l4xx_hal_conf.h"
#endif

#endif // MCU_ARCH_STM32_STM32_DEV_LOCAL_CONF_H
