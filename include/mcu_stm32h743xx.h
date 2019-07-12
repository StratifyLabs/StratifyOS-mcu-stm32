/* Copyright 2011-2018 Tyler Gilbert;
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
 * along with Stratify OS.  If not, see <http://www.gnu.org/licenses/>. */

#ifndef MCU_STM32H743XX_H_
#define MCU_STM32H743XX_H_

#include <mcu/types.h>
#include "cmsis/stm32h7xx.h"
#include "cmsis/stm32h743xx.h"


#define MCU_NO_HARD_FAULT 1

#define MCU_ADC_API 0
#define MCU_ADC_PORTS 0
#define MCU_ADC_REGS { ADC1, ADC2, ADC3 }
#define MCU_ADC_IRQS { ADC_IRQn, ADC_IRQn, ADC_IRQn }
#define MCU_ADC_CHANNELS 21

#define MCU_ENET_PORTS 1
#define MCU_FLASH_PORTS 1
#define MCU_MEM_PORTS 1

#define MCU_I2C_API 0
#define MCU_I2C_PORTS 3
#define MCU_I2C_REGS { I2C1, I2C2, I2C3 }
#define MCU_I2C_IRQS { I2C1_EV_IRQn, I2C2_EV_IRQn, I2C3_EV_IRQn }
#define MCU_I2C_ER_IRQS { I2C1_ER_IRQn, I2C2_ER_IRQn, I2C3_ER_IRQn }

#define MCU_I2C_FMP_PORTS 0

#define MCU_CORE_PORTS 1
#define MCU_EEPROM_PORTS 0
#define MCU_SPI_API 0
#define MCU_SPI_PORTS 6
#define MCU_SPI_REGS { SPI1, SPI2, SPI3, SPI4, SPI5, SPI6 }
#define MCU_SPI_IRQS { SPI1_IRQn, SPI2_IRQn, SPI3_IRQn, SPI4_IRQn, SPI5_IRQn }

//I2S is available on 2 SPI ports
#define MCU_I2S_SPI_PORTS 2
#define MCU_I2S_ON_SPI1 0
#define MCU_I2S_ON_SPI2 1
#define MCU_I2S_ON_SPI3 1
#define MCU_I2S_ON_SPI4 0
#define MCU_I2S_ON_SPI5 0
#define MCU_I2S_ON_SPI6 0

#define MCU_SAI_API 0
#define MCU_SAI_PORTS 0
#define MCU_SAI_REGS { SAI1 }
#define MCU_SAI_IRQS { SAI1_IRQn }

#define MCU_SDIO_API 0
#define MCU_SDIO_PORTS 0
#define MCU_SDIO_REGS { SDMMC1, SDMMC2 }
#define MCU_SDIO_IRQS { SDMMC1_IRQn, SDMMC2_IRQn }

#define MCU_TMR_PORTS 17
#define MCU_TMR_REGS { TIM1, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM8, 0, 0, 0, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17 }
#define MCU_TMR_IRQS { TIM1_CC_IRQn, TIM2_IRQn, TIM3_IRQn, TIM4_IRQn, TIM5_IRQn, TIM6_DAC_IRQn, TIM7_IRQn, TIM8_CC_IRQn, -1, -1, -1, TIM8_BRK_TIM12_IRQn, TIM8_UP_TIM13_IRQn, TIM8_TRG_COM_TIM14_IRQn, TIM15_IRQn, TIM16_IRQn, TIM17_IRQn }

#define MCU_PIO_PORTS 11
#define MCU_PIO_REGS { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI, GPIOJ, GPIOK }
#define MCU_PIO_IRQS { 0 }

#define MCU_UART_PORTS 8
#define MCU_UART_REGS { USART1, USART2, USART3, UART4, UART5, USART6, UART7, UART8 }
#define MCU_UART_IRQS { USART1_IRQn, USART2_IRQn, USART3_IRQn, UART4_IRQn, UART5_IRQn, USART6_IRQn, UART7_IRQn, UART8_IRQn }

#define MCU_USB_API 1
#define MCU_USB_PORTS 1
#define MCU_USB_REGS { USB_OTG_FS }
#define MCU_USB_IRQS { OTG_FS_IRQn }

#define MCU_ETH_PORTS 0
#define MCU_ETH_REGS { ETH }
#define MCU_ETH_IRQS { ETH_IRQn }

#define MCU_RTC_PORTS 1
#define MCU_RTC_REGS { RTC }
#define MCU_RTC_IRQS { RTC_Alarm_IRQn }

#define DEV_USB_LOGICAL_ENDPOINT_COUNT 9

#define MCU_LAST_IRQ WAKEUP_PIN_IRQn
#define MCU_MIDDLE_IRQ_PRIORITY 8

#define MCU_RAM_PAGES 864
#define MCU_DELAY_FACTOR 12
#define MCU_TOTAL_PINS (10*16+8)

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif


#endif /* MCU_STM32H743XX_H_ */
