/* Copyright 2011-2017 Tyler Gilbert;
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

#define ENTRY(function, port) (port<<8|function)
#define RESERVED_ (ENTRY(CORE_PERIPH_RESERVED, 0))
#define ENTRY_GET_FUNCTION(entry) (entry & 0xFF)
#define ENTRY_GET_PORT(entry) ( entry >> 8)

#define E_TMR_(port) ENTRY(CORE_PERIPH_TMR,(port-1))
#define E_TMR(port) ENTRY(CORE_PERIPH_TMR,(port-1))
#define E_UART(port) ENTRY(CORE_PERIPH_UART,(port-1))
#define E_USRT(port) ENTRY(CORE_PERIPH_UART,(port-1))
#define E_I2S_(port) ENTRY(CORE_PERIPH_I2S,(port-1))
#define E_SYS_(port) ENTRY(CORE_PERIPH_SYS,(port-1))
#define E_QSPI(port) ENTRY(CORE_PERIPH_QSPI,(port-1))
#define E_SPI_(port) ENTRY(CORE_PERIPH_SPI,(port-1))
#define E_USB_(port) ENTRY(CORE_PERIPH_USB,(port-1))
#define E_LCD_(port) ENTRY(CORE_PERIPH_LCD,(port-1))
#define E_FMC_(port) ENTRY(CORE_PERIPH_EMC,(port-1))
#define E_MCO_(port) ENTRY(CORE_PERIPH_MCO,(port-1))
#define E_I2C_(port) ENTRY(CORE_PERIPH_I2C,(port-1))
#define E_HDMI(port) ENTRY(CORE_PERIPH_HDMI,(port-1))
#define E_CAN_(port) ENTRY(CORE_PERIPH_CAN,(port-1))
#define E_JTAG(port) ENTRY(CORE_PERIPH_JTAG,(port-1))
#define E_RTC_(port) ENTRY(CORE_PERIPH_RTC,(port-1))
#define E_SPDF(port) ENTRY(CORE_PERIPH_SPDIF,(port-1))
#define E_SDIO(port) ENTRY(CORE_PERIPH_SDIO,(port-1))
#define E_TRAC(port) ENTRY(CORE_PERIPH_TRACE,(port-1))
#define E_FMP_(port) ENTRY(CORE_PERIPH_I2C,(port-1))

#endif /* STM32_PIN_LOCAL_H_ */
