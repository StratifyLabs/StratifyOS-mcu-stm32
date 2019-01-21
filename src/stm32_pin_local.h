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
 * along with Stratify OS.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */

#ifndef STM32_PIN_LOCAL_H_
#define STM32_PIN_LOCAL_H_

#define TOTAL_ENTRIES 16
#define DMA_STREAMS_PER_CHANNEL 8

typedef struct {
	u16 entry[DMA_STREAMS_PER_CHANNEL]; //eight streams per channel
} dma_channel_entry_t;

typedef struct {
	u16 entry[TOTAL_ENTRIES];
} alternate_function_entry_t;


#define OUTPUT_ENTRY(function, port) (port<<8 | function)
#define INPUT_ENTRY(function, port) (port<<8 | function | 0x8000)
#define TMR_ENTRY(function, port, channel) (port << 8 | function | channel << 12)

#define DTMR(port,channel) TMR_ENTRY(CORE_PERIPH_TMR, (port-1), channel)

#define O_TMR_(port) OUTPUT_ENTRY(CORE_PERIPH_TMR,(port-1)) //TIM
#define O_LPTM(port) OUTPUT_ENTRY(CORE_PERIPH_TMR,(port-1)) //LPTIM
#define O_TMR(port) OUTPUT_ENTRY(CORE_PERIPH_TMR,(port-1)) //TIM (for values 10 and above
#define O_UART(port) OUTPUT_ENTRY(CORE_PERIPH_UART,(port-1)) //UART
#define O_USRT(port) OUTPUT_ENTRY(CORE_PERIPH_UART,(port-1)) //USART
#define O_I2S_(port) OUTPUT_ENTRY(CORE_PERIPH_I2S,(port-1)) //SAI (use SPI for I2S)
#define O_SYS_(port) OUTPUT_ENTRY(CORE_PERIPH_SYS,(port-1))
#define O_QSPI(port) OUTPUT_ENTRY(CORE_PERIPH_QSPI,(port-1)) //Quad SPI
#define O_SPI_(port) OUTPUT_ENTRY(CORE_PERIPH_SPI,(port-1)) //SPI (also use for I2S)
#define O_USB_(port) OUTPUT_ENTRY(CORE_PERIPH_USB,(port-1))
#define O_LCD_(port) OUTPUT_ENTRY(CORE_PERIPH_LCD,(port-1))
#define O_FMC_(port) OUTPUT_ENTRY(CORE_PERIPH_EMC,(port-1))
#define O_FSMC(port) OUTPUT_ENTRY(CORE_PERIPH_EMC,(port-1))
#define O_MCO_(port) OUTPUT_ENTRY(CORE_PERIPH_MCO,(port-1))
#define O_I2C_(port) OUTPUT_ENTRY(CORE_PERIPH_I2C,(port-1))
#define O_HDMI(port) OUTPUT_ENTRY(CORE_PERIPH_HDMI,(port-1))
#define O_CAN_(port) OUTPUT_ENTRY(CORE_PERIPH_CAN,(port-1))
#define O_JTAG(port) OUTPUT_ENTRY(CORE_PERIPH_JTAG,(port-1))
#define O_RTC_(port) OUTPUT_ENTRY(CORE_PERIPH_RTC,(port-1))
#define O_SPDF(port) OUTPUT_ENTRY(CORE_PERIPH_SPDIF,(port-1)) //SPDIF
#define O_SDIO(port) OUTPUT_ENTRY(CORE_PERIPH_SDIO,(port-1))
#define OSDMMC(port) O_SDIO(port)
#define O_TRAC(port) OUTPUT_ENTRY(CORE_PERIPH_TRACE,(port-1))
#define O_FMP_(port) OUTPUT_ENTRY(CORE_PERIPH_I2C,(port-1))
#define O_DFSD(port) OUTPUT_ENTRY(CORE_PERIPH_DFSDM,(port-1))
#define O_FI2C(port) OUTPUT_ENTRY(CORE_PERIPH_FMP_I2C,(port-1))
#define O_ENET(port) OUTPUT_ENTRY(CORE_PERIPH_ENET,(port-1))
#define O_ETH_(port) OUTPUT_ENTRY(CORE_PERIPH_ENET,(port-1))
#define O_DCMI(port) OUTPUT_ENTRY(CORE_PERIPH_DCMI,(port-1))

#define I_ADC_(port) INPUT_ENTRY(CORE_PERIPH_ADC,(port-1)) //ADC
#define I_TMR_(port) INPUT_ENTRY(CORE_PERIPH_TMR,(port-1)) //TIM
#define I_LPTM(port) INPUT_ENTRY(CORE_PERIPH_TMR,(port-1)) //LPTIM
#define I_TMR(port) INPUT_ENTRY(CORE_PERIPH_TMR,(port-1)) //TIM (for values 10 and above
#define I_UART(port) INPUT_ENTRY(CORE_PERIPH_UART,(port-1)) //UART
#define I_USRT(port) INPUT_ENTRY(CORE_PERIPH_UART,(port-1)) //USART
#define I_I2S_(port) INPUT_ENTRY(CORE_PERIPH_I2S,(port-1)) //SAI (use SPI for I2S)
#define I_SYS_(port) INPUT_ENTRY(CORE_PERIPH_SYS,(port-1))
#define I_QSPI(port) INPUT_ENTRY(CORE_PERIPH_QSPI,(port-1)) //Quad SPI
#define I_SPI_(port) INPUT_ENTRY(CORE_PERIPH_SPI,(port-1)) //SPI (also use for I2S)
#define I_USB_(port) INPUT_ENTRY(CORE_PERIPH_USB,(port-1))
#define I_LCD_(port) INPUT_ENTRY(CORE_PERIPH_LCD,(port-1))
#define I_FMC_(port) INPUT_ENTRY(CORE_PERIPH_EMC,(port-1))
#define I_FSMC(port) INPUT_ENTRY(CORE_PERIPH_EMC,(port-1))
#define I_MCO_(port) INPUT_ENTRY(CORE_PERIPH_MCO,(port-1))
#define I_I2C_(port) INPUT_ENTRY(CORE_PERIPH_I2C,(port-1))
#define I_HDMI(port) INPUT_ENTRY(CORE_PERIPH_HDMI,(port-1))
#define I_CAN_(port) INPUT_ENTRY(CORE_PERIPH_CAN,(port-1))
#define I_JTAG(port) INPUT_ENTRY(CORE_PERIPH_JTAG,(port-1))
#define I_RTC_(port) INPUT_ENTRY(CORE_PERIPH_RTC,(port-1))
#define I_SPDF(port) INPUT_ENTRY(CORE_PERIPH_SPDIF,(port-1)) //SPDIF
#define I_SDIO(port) INPUT_ENTRY(CORE_PERIPH_SDIO,(port-1))
#define ISDMMC(port) I_SDIO(port)
#define I_TRAC(port) INPUT_ENTRY(CORE_PERIPH_TRACE,(port-1))
#define I_FMP_(port) INPUT_ENTRY(CORE_PERIPH_I2C,(port-1))
#define I_DFSD(port) INPUT_ENTRY(CORE_PERIPH_DFSDM,(port-1))
#define I_FI2C(port) INPUT_ENTRY(CORE_PERIPH_FMP_I2C,(port-1))
#define I_ENET(port) INPUT_ENTRY(CORE_PERIPH_ENET,(port-1))
#define I_ETH_(port) INPUT_ENTRY(CORE_PERIPH_ENET,(port-1))
#define I_DCMI(port) INPUT_ENTRY(CORE_PERIPH_DCMI,(port-1))


#define ENTRY(function, port) (port<<8|function)
#define RESERVED_ (ENTRY(CORE_PERIPH_RESERVED, 0))
#define ENTRY_GET_FUNCTION(entry) (entry & 0xFF)
#define ENTRY_GET_PORT(entry) ( entry >> 8)

#define E_TMR_(port) ENTRY(CORE_PERIPH_TMR,(port-1)) //TIM
#define E_LPTM(port) ENTRY(CORE_PERIPH_TMR,(port-1)) //LPTIM
#define E_TMR(port) ENTRY(CORE_PERIPH_TMR,(port-1)) //TIM (for values 10 and above
#define E_UART(port) ENTRY(CORE_PERIPH_UART,(port-1)) //UART
#define E_USRT(port) ENTRY(CORE_PERIPH_UART,(port-1)) //USART
#define E_I2S_(port) ENTRY(CORE_PERIPH_I2S,(port-1)) //SAI (use SPI for I2S)
#define E_SYS_(port) ENTRY(CORE_PERIPH_SYS,(port-1))
#define E_QSPI(port) ENTRY(CORE_PERIPH_QSPI,(port-1)) //Quad SPI
#define E_SPI_(port) ENTRY(CORE_PERIPH_SPI,(port-1)) //SPI (also use for I2S)
#define E_USB_(port) ENTRY(CORE_PERIPH_USB,(port-1))
#define E_LCD_(port) ENTRY(CORE_PERIPH_LCD,(port-1))
#define E_FMC_(port) ENTRY(CORE_PERIPH_EMC,(port-1))
#define E_FSMC(port) ENTRY(CORE_PERIPH_EMC,(port-1))
#define E_MCO_(port) ENTRY(CORE_PERIPH_MCO,(port-1))
#define E_I2C_(port) ENTRY(CORE_PERIPH_I2C,(port-1))
#define E_HDMI(port) ENTRY(CORE_PERIPH_HDMI,(port-1))
#define E_CAN_(port) ENTRY(CORE_PERIPH_CAN,(port-1))
#define E_JTAG(port) ENTRY(CORE_PERIPH_JTAG,(port-1))
#define E_RTC_(port) ENTRY(CORE_PERIPH_RTC,(port-1))
#define E_SPDF(port) ENTRY(CORE_PERIPH_SPDIF,(port-1)) //SPDIF
#define E_SDIO(port) ENTRY(CORE_PERIPH_SDIO,(port-1))
#define ESDMMC(port) ENTRY(CORE_PERIPH_SDIO,(port-1))
#define E_TRAC(port) ENTRY(CORE_PERIPH_TRACE,(port-1))
#define E_FMP_(port) ENTRY(CORE_PERIPH_I2C,(port-1))
#define E_DFSD(port) ENTRY(CORE_PERIPH_DFSDM,(port-1))
#define E_FI2C(port) ENTRY(CORE_PERIPH_FMP_I2C,(port-1))
#define E_ENET(port) ENTRY(CORE_PERIPH_ENET,(port-1))
#define E_ETH_(port) ENTRY(CORE_PERIPH_ENET,(port-1))
#define E_DCMI(port) ENTRY(CORE_PERIPH_DCMI,(port-1))
#define E_IR(port) RESERVED_
#define ESWPMI(port) RESERVED_
#define E_TSC_(port) RESERVED_
#define ELUART(port) RESERVED_

extern const alternate_function_entry_t alternate_function_table[];

#endif /* STM32_PIN_LOCAL_H_ */
