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

#include <sos/debug.h>

#include "stm32_types_conf.h"

typedef struct {
  DMA_HandleTypeDef handle;
  int interrupt_number;
} stm32_dma_channel_t;

#if defined STM32H7
extern MDMA_HandleTypeDef stm32_dma_mdma_handle;
#endif

void stm32_dma_set_interrupt_priority(
  const stm32_dma_channel_config_t *config,
  const mcu_action_t *action);
u32 stm32_dma_decode_priority(u8 priority);
u32 stm32_dma_decode_channel(u32 channel_number);
DMA_Stream_TypeDef *
stm32_dma_get_stream_instance(u32 dma_number, u32 stream_number);
stm32_dma_channel_t *stm32_dma_set_handle(u32 dma_number, u32 stream_number);
void stm32_dma_clear_handle(u32 dma_number, u32 stream_number);

stm32_dma_channel_t *
stm32_dma_setattr(const stm32_dma_channel_config_t *config);

#endif /* STM32_DMA_H_ */
