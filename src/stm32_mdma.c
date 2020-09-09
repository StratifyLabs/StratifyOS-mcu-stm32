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

#include "stm32_mdma.h"

#if 0

#if defined MDMA
#define MCU_MDMA_PORTS 1
#define MCU_MDMA_CHANNEL_COUNT 16
#endif

#if MCU_MDMA_PORTS > 0

typedef struct {
	stm32_mdma_channel_t channel[MCU_MDMA_CHANNEL_COUNT];
} stm32_dma_channels_t;

stm32_dma_channels_t stm32_dma_handle[MCU_MDMA_PORTS] MCU_SYS_MEM;

MDMA_Channel_TypeDef * const stm32_mdma_channels[MCU_MDMA_CHANNEL_COUNT] = {
	MDMA_Channel0,
	MDMA_Channel1,
	MDMA_Channel2,
	MDMA_Channel3,
	MDMA_Channel4,
	MDMA_Channel5,
	MDMA_Channel6,
	MDMA_Channel7,
	MDMA_Channel8,
	MDMA_Channel9,
	MDMA_Channel10,
	MDMA_Channel11,
	MDMA_Channel12,
	MDMA_Channel13,
	MDMA_Channel14,
	MDMA_Channel15
};

const u32 stm32_mdma_priorities[4] = {
	MDMA_PRIORITY_LOW, MDMA_PRIORITY_MEDIUM, MDMA_PRIORITY_HIGH, MDMA_PRIORITY_VERY_HIGH
};

u32 stm32_dma_decode_channel(u32 channel_number){
	if( channel_number > 7 ){
		channel_number = 7;
	}
	return stm32_dma_channels[channel_number];
}

u32 stm32_dma_decode_priority(u8 priority){
	if( priority > 3 ){
		priority = 3;
	}
	return stm32_dma_priorities[priority];
}


MDMA_Channel_TypeDef * stm32_mdma_get_channel(u32 channel){
	return 0;
}



stm32_mdma_channel_t * stm32_mdma_set_handle(u32 dma_number, u32 stream_number){
	if( (dma_number < MCU_DMA_PORTS) && (stream_number < MCU_DMA_STREAM_COUNT) ){

		int interrupt_number;
		interrupt_number = stm32_dma_get_interrupt_number(dma_number, stream_number);
		if( interrupt_number < 0 ){
			return 0;
		}

		__HAL_RCC_MDMA_CLK_ENABLE();

		stm32_dma_channel_t * channel = &stm32_dma_handle[dma_number].stream[stream_number];
		channel->interrupt_number = interrupt_number;
		cortexm_enable_irq( interrupt_number );
		mcu_debug_log_info(MCU_DEBUG_DEVICE, "Enable interrupt %d", channel->interrupt_number);
		return channel;
	}

	return 0;
}

int stm32_dma_get_interrupt_number(u32 dma_number, u32 stream_number){
	if( stream_number < 8 ){
		if( dma_number == 0 ){
			return stm32_dma0_irqs[stream_number];
		} else if( dma_number == 1 ){
			return stm32_dma1_irqs[stream_number];
		}
	}
	return -1;
}

void stm32_dma_clear_handle(u32 dma_number, u32 stream_number){
	//remove a handle that is in the list
	if( (dma_number < MCU_DMA_PORTS) && (stream_number < MCU_DMA_STREAM_COUNT) ){

		stm32_dma_channel_t * channel = &stm32_dma_handle[dma_number].stream[stream_number];
		//ON STM32 Interrupt zero is the WDT (or something else that typically doesn't support DMA -- so zero is not valid
		if( channel->interrupt_number <= 0 ){
			return;
		}

		HAL_DMA_DeInit(&channel->handle);
		mcu_debug_log_info(MCU_DEBUG_DEVICE, "Disable interrupt %d", channel->interrupt_number);
		cortexm_enable_irq( channel->interrupt_number );
		channel->interrupt_number = -1;
	}
}

stm32_mdma_channel_t * stm32_dma_setattr(const stm32_mdma_channel_config_t * config){

	stm32_mdma_channel_t * channel;
	u32 o_flags = config->o_flags;
	channel = stm32_dma_set_handle(config->dma_number, config->stream_number);
	if( channel == 0 ){ return 0; }
	channel->handle.Instance = stm32_mdma_get_channel_instance(config->channel_number);

#if defined DMA_REQUEST_0
	channel->handle.Init.Request = stm32_dma_decode_channel(config->channel_number);
#elif defined DMA_CHANNEL_0
	channel->handle.Init.Channel = stm32_dma_decode_channel(config->channel_number);
#else

#endif

	channel->handle.Init.Direction = DMA_PERIPH_TO_MEMORY; //read is always periph to memory
	if( o_flags & STM32_DMA_FLAG_IS_MEMORY_TO_PERIPH ){
		channel->handle.Init.Direction = DMA_MEMORY_TO_PERIPH; //read is always periph to memory
	}

	channel->handle.Init.PeriphInc = DMA_PINC_DISABLE; //don't inc peripheral
	channel->handle.Init.MemInc = DMA_MINC_ENABLE; //do inc the memory

	if( o_flags & STM32_DMA_FLAG_IS_MEMORY_INC_DISABLE ){ channel->handle.Init.MemInc = DMA_MINC_DISABLE; }
	if( o_flags & STM32_DMA_FLAG_IS_PERIPH_INC_ENABLE ){ channel->handle.Init.PeriphInc = DMA_PINC_ENABLE; }

#if defined DMA_FIFOMODE_ENABLE
	if( o_flags & STM32_DMA_FLAG_IS_FIFO ){
		channel->handle.Init.FIFOMode = DMA_FIFOMODE_ENABLE; //?
		channel->handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
		if( o_flags & STM32_DMA_FLAG_IS_FIFO_THRESHOLD_QUARTER ){ channel->handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL; }
		if( o_flags & STM32_DMA_FLAG_IS_FIFO_THRESHOLD_THREE_QUARTER ){ channel->handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_3QUARTERSFULL; }
		if( o_flags & STM32_DMA_FLAG_IS_FIFO_THRESHOLD_FULL ){ channel->handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL; }

		channel->handle.Init.MemBurst = DMA_MBURST_SINGLE;
		if( o_flags & STM32_DMA_FLAG_IS_MEMORY_INC4 ){ channel->handle.Init.MemBurst = DMA_MBURST_INC4; }
		if( o_flags & STM32_DMA_FLAG_IS_MEMORY_INC8 ){ channel->handle.Init.MemBurst = DMA_MBURST_INC8; }
		if( o_flags & STM32_DMA_FLAG_IS_MEMORY_INC16 ){ channel->handle.Init.MemBurst = DMA_MBURST_INC16; }
		channel->handle.Init.PeriphBurst = DMA_PBURST_SINGLE;
		if( o_flags & STM32_DMA_FLAG_IS_PERIPH_INC4 ){ channel->handle.Init.PeriphBurst = DMA_PBURST_INC4; }
		if( o_flags & STM32_DMA_FLAG_IS_PERIPH_INC8 ){ channel->handle.Init.PeriphBurst = DMA_PBURST_INC8; }
		if( o_flags & STM32_DMA_FLAG_IS_PERIPH_INC16 ){ channel->handle.Init.PeriphBurst = DMA_PBURST_INC16; }

	} else {
		channel->handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE; //?
	}
#endif

	channel->handle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	if( o_flags & STM32_DMA_FLAG_IS_MEMORY_BYTE ){ channel->handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE; }
	if( o_flags & STM32_DMA_FLAG_IS_MEMORY_HALFWORD ){ channel->handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD; }

	channel->handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	if( o_flags & STM32_DMA_FLAG_IS_PERIPH_BYTE ){ channel->handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; }
	if( o_flags & STM32_DMA_FLAG_IS_PERIPH_HALFWORD ){ channel->handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; }

	channel->handle.Init.Mode = DMA_NORMAL;
	if( o_flags & STM32_DMA_FLAG_IS_CIRCULAR ){ channel->handle.Init.Mode = DMA_CIRCULAR; }
#if defined DMA_PFCTRL
	if( o_flags & STM32_DMA_FLAG_IS_PFCTRL ){ channel->handle.Init.Mode = DMA_PFCTRL; }
#endif
	channel->handle.Init.Priority = stm32_dma_decode_priority(config->priority);

	int result = HAL_DMA_Init(&channel->handle);
	if ( result != HAL_OK ){
		mcu_debug_log_error(MCU_DEBUG_DEVICE, "failed to init DMA %d", result);
		return 0;
	}

	return channel;
}


void mcu_core_mdma_isr(){
	HAL_MDMA_IRQHandler(0);
}

#endif
#endif // 0
