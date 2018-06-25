/* Copyright 2011-2016 Tyler Gilbert;
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

#include <mcu/core.h>
#include <mcu/debug.h>
#include "stm32_local.h"
#include "stm32_pin_local.h"


const dma_channel_entry_t dma_mapping_table[8*MCU_DMA_PORTS] = {
    //   0           1          2          3          4         5           6         7
    {{ I_SPI_(3), RESERVED_, I_SPI_(3), I_SPI_(2), O_SPI_(2), O_SPI_(3), RESERVED_, O_SPI_(3) }}, //DMA1 Channel 0
    {{ I_I2C_(1), I_I2C_(3), RESERVED_, RESERVED_, RESERVED_, I_I2C_(1), O_I2C_(1), O_I2C_(1) }}, //DMA1 Channel 1
    {{ DTMR(4,1), RESERVED_, I_I2S_(3), DTMR(4,2), O_I2S_(2), O_I2S_(2), DTMR(4,0), DTMR(4,3) }}, //DMA1 Channel 2
    {{ I_I2S_(3), DTMR(2,3), I_I2C_(3), I_I2S_(2), O_I2C_(3), DTMR(2,1), DTMR(2,2), DTMR(2,0) }}, //DMA1 Channel 3
    {{ RESERVED_, RESERVED_, RESERVED_, RESERVED_, RESERVED_, I_UART(2), O_UART(2), RESERVED_ }}, //DMA1 Channel 4
    {{ RESERVED_, RESERVED_, DTMR(3,4), RESERVED_, DTMR(3,1), DTMR(3,2), RESERVED_, DTMR(3,3) }}, //DMA1 Channel 5
    {{ DTMR(5,3), DTMR(5,4), DTMR(5,1), DTMR(5,4), DTMR(5,2), RESERVED_, DTMR(5,0), RESERVED_ }}, //DMA1 Channel 6
    {{ RESERVED_, RESERVED_, I_I2C_(2), I_I2C_(2), RESERVED_, RESERVED_, RESERVED_, O_I2C_(2) }}, //DMA1 Channel 7


    {{ I_ADC_(1), RESERVED_, RESERVED_, RESERVED_, I_ADC_(1), RESERVED_, DTMR(1,1), RESERVED_ }}, //DMA2 Channel 0
    {{ RESERVED_, RESERVED_, RESERVED_, RESERVED_, RESERVED_, RESERVED_, RESERVED_, RESERVED_ }}, //DMA2 Channel 1
    {{ RESERVED_, RESERVED_, RESERVED_, RESERVED_, RESERVED_, RESERVED_, RESERVED_, RESERVED_ }}, //DMA2 Channel 2
    {{ I_SPI_(1), RESERVED_, I_SPI_(1), O_SPI_(1), RESERVED_, O_SPI_(1), RESERVED_, RESERVED_ }}, //DMA2 Channel 3
    {{ I_SPI_(4), O_SPI_(4), I_UART(2), I_SDIO(1), RESERVED_, I_UART(1), I_SDIO(1), O_UART(1) }}, //DMA2 Channel 4
    {{ RESERVED_, I_UART(2), I_UART(2), RESERVED_, RESERVED_, RESERVED_, RESERVED_, RESERVED_ }}, //DMA2 Channel 5
    {{ DTMR(1,0), DTMR(1,1), DTMR(1,2), DTMR(1,1), DTMR(1,4), DTMR(1,0), DTMR(1,3), RESERVED_ }}, //DMA2 Channel 6
    {{ RESERVED_, RESERVED_, RESERVED_, RESERVED_, RESERVED_, RESERVED_, RESERVED_, RESERVED_ }}, //DMA2 Channel 7


};

