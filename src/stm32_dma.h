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

#ifndef STM32_DMA_H_
#define STM32_DMA_H_

#include "stm32_local.h"

typedef struct {
    DMA_HandleTypeDef handle;
    int interrupt_number;
    void * next;
} stm32_dma_channel_t;

u32 stm32_dma_decode_priority(u8 priority);
u32 stm32_dma_decode_channel(u32 channel_number);
DMA_Stream_TypeDef * stm32_dma_get_stream_instance(u32 dma_number, u32 stream_number);
void stm32_dma_set_handle(stm32_dma_channel_t * channel, u32 dma_number, u32 stream_number);
void stm32_dma_clear_handle(stm32_dma_channel_t * channel, u32 dma_number, u32 stream_number);

enum {
    STM32_DMA_LOCAL_FLAG_IS_CIRCULAR = (1<<0),
    STM32_DMA_LOCAL_FLAG_IS_FIFO = (1<<1),
    STM32_DMA_LOCAL_FLAG_IS_MEMORY_TO_PERIPH = (1<<2),
    STM32_DMA_LOCAL_FLAG_IS_PERIPH_TO_MEMORY = (1<<3),
    STM32_DMA_LOCAL_FLAG_IS_MEMORY_BYTE = (1<<4),
    STM32_DMA_LOCAL_FLAG_IS_MEMORY_HALFWORD = (1<<5),
    STM32_DMA_LOCAL_FLAG_IS_MEMORY_WORD = (1<<6),
    STM32_DMA_LOCAL_FLAG_IS_PERIPH_BYTE = (1<<7),
    STM32_DMA_LOCAL_FLAG_IS_PERIPH_HALFWORD = (1<<8),
    STM32_DMA_LOCAL_FLAG_IS_PERIPH_WORD = (1<<9),
    STM32_DMA_LOCAL_FLAG_IS_MEMORY_INC4 = (1<<11),
    STM32_DMA_LOCAL_FLAG_IS_MEMORY_INC8 = (1<<12),
    STM32_DMA_LOCAL_FLAG_IS_MEMORY_INC16 = (1<<13),
    STM32_DMA_LOCAL_FLAG_IS_PERIPH_INC4 = (1<<15),
    STM32_DMA_LOCAL_FLAG_IS_PERIPH_INC8 = (1<<16),
    STM32_DMA_LOCAL_FLAG_IS_PERIPH_INC16 = (1<<17),
    STM32_DMA_LOCAL_FLAG_IS_MEMORY_INC_DISABLE = (1<<18),
    STM32_DMA_LOCAL_FLAG_IS_PERIPH_INC_ENABLE = (1<<19),
    STM32_DMA_LOCAL_FLAG_IS_FIFO_THRESHOLD_QUARTER = (1<<20),
    STM32_DMA_LOCAL_FLAG_IS_FIFO_THRESHOLD_THREE_QUARTER = (1<<21),
    STM32_DMA_LOCAL_FLAG_IS_FIFO_THRESHOLD_FULL = (1<<22),


};

int stm32_dma_setattr(stm32_dma_channel_t * channel,
                      const stm32_dma_channel_config_t * config,
                      u32 o_flags);

#endif /* STM32_DMA_H_ */
