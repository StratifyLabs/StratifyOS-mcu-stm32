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

#ifndef SPI_LOCAL_H_
#define SPI_LOCAL_H_

#include <mcu/adc.h>

#include "stm32_dma.h"

#define ADC_LOCAL_FLAG_IS_DMA (1<<0)

typedef struct {
    ADC_HandleTypeDef hal_handle;
    devfs_transfer_handler_t transfer_handler;
    int words_read;
    u32 o_flags;
    u8 ref_count;
} adc_local_t;

typedef struct {
    adc_local_t adc;
    stm32_dma_channel_t dma_rx_channel;
} adc_dma_local_t;

extern ADC_TypeDef * const adc_regs_table[MCU_ADC_PORTS];
extern u8 const adc_irqs[MCU_ADC_PORTS];
extern const u32 adc_channels[MCU_ADC_CHANNELS];

int adc_local_open(adc_local_t * adc, const devfs_handle_t * handle);
int adc_local_close(adc_local_t * adc, const devfs_handle_t * handle);
int adc_local_setattr(adc_local_t * adc, const devfs_handle_t * handle, void * ctl);
int adc_local_getinfo(adc_local_t * adc, const devfs_handle_t * handle, void * ctl);



#endif /* SPI_LOCAL_H_ */
