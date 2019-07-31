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

#if defined MDMA

typedef struct {
	MDMA_HandleTypeDef handle;
	int interrupt_number;
} stm32_mdma_channel_t;


void stm32_mdma_set_interrupt_priority(const stm32_dma_channel_config_t * config, const mcu_action_t * action);
u32 stm32_mdma_decode_priority(u8 priority);
u32 stm32_mdma_decode_channel(u32 channel_number);
MDMA_Channel_TypeDef * stm32_mdma_get_stream_instance(u32 dma_number, u32 stream_number);
stm32_mdma_channel_t * stm32_mdma_set_handle(u32 dma_number, u32 stream_number);
void stm32_mdma_clear_handle(u32 dma_number, u32 stream_number);

stm32_mdma_channel_t * stm32_mdma_setattr(const stm32_mdma_channel_config_t * config);

#endif //MDMA

#endif /* STM32_DMA_H_ */
