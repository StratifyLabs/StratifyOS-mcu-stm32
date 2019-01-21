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


#include "stm32_dma.h"

#define MCU_DMA_PORTS 2


#if defined STM32L4
#define MCU_DMA_STREAM_COUNT 7
#define MCU_DMA_CHANNEL_COUNT 7
#define MCU_DMA0_REGS { DMA1_Channel1, DMA1_Channel2, DMA1_Channel3, DMA1_Channel4, DMA1_Channel5, DMA1_Channel6, DMA1_Channel7 }
#define MCU_DMA1_REGS { DMA2_Channel1, DMA2_Channel2, DMA2_Channel3, DMA2_Channel4, DMA2_Channel5, DMA2_Channel6, DMA2_Channel7 }
#define MCU_DMA0_IRQS { DMA1_Channel1_IRQn, DMA1_Channel2_IRQn, DMA1_Channel3_IRQn, DMA1_Channel4_IRQn, DMA1_Channel5_IRQn, DMA1_Channel6_IRQn, DMA1_Channel7_IRQn }
#define MCU_DMA1_IRQS { DMA2_Channel1_IRQn, DMA2_Channel2_IRQn, DMA2_Channel3_IRQn, DMA2_Channel4_IRQn, DMA2_Channel5_IRQn, DMA2_Channel6_IRQn, DMA2_Channel7_IRQn }
#else
#define MCU_DMA_STREAM_COUNT 8
#define MCU_DMA_CHANNEL_COUNT 8
#define MCU_DMA0_REGS { DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7 }
#define MCU_DMA1_REGS { DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7 }
#define MCU_DMA0_IRQS { DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn, DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn }
#define MCU_DMA1_IRQS { DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn, DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn }
#endif

#if MCU_DMA_PORTS > 0

typedef struct {
	stm32_dma_channel_t stream[MCU_DMA_STREAM_COUNT];
} stm32_dma_streams_t;

stm32_dma_streams_t stm32_dma_handle[MCU_DMA_PORTS] MCU_SYS_MEM;

static DMA_Stream_TypeDef * const stm32_dma0_regs[MCU_DMA_STREAM_COUNT] = MCU_DMA0_REGS;
static DMA_Stream_TypeDef * const stm32_dma1_regs[MCU_DMA_STREAM_COUNT] = MCU_DMA1_REGS;

static int stm32_dma_get_interrupt_number(u32 dma_number, u32 stream_number);

static const u8 stm32_dma0_irqs[MCU_DMA_STREAM_COUNT] = MCU_DMA0_IRQS;
static const u8 stm32_dma1_irqs[MCU_DMA_STREAM_COUNT] = MCU_DMA1_IRQS;


const u32 stm32_dma_channels[8] = {
	#if defined DMA_CHANNEL_0
	DMA_CHANNEL_0, DMA_CHANNEL_1, DMA_CHANNEL_2, DMA_CHANNEL_3, DMA_CHANNEL_4, DMA_CHANNEL_5, DMA_CHANNEL_6, DMA_CHANNEL_7
	#endif
};

const u32 stm32_dma_priorities[4] = {
	DMA_PRIORITY_LOW, DMA_PRIORITY_MEDIUM, DMA_PRIORITY_HIGH, DMA_PRIORITY_VERY_HIGH
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


DMA_Stream_TypeDef * stm32_dma_get_stream_instance(u32 dma_number, u32 stream_number){
	if( dma_number ){
		return stm32_dma1_regs[stream_number];
	}

	return stm32_dma0_regs[stream_number];
}

void stm32_dma_set_interrupt_priority(const stm32_dma_channel_config_t * config, const mcu_action_t * action){
	u32 dma_number = config->dma_number;
	u32 stream_number = config->stream_number;
	if( (dma_number < MCU_DMA_PORTS) && (stream_number < MCU_DMA_STREAM_COUNT) ){
		stm32_dma_channel_t * channel = &stm32_dma_handle[dma_number].stream[stream_number];
		if( channel->interrupt_number > 0 ){
			cortexm_set_irq_priority(channel->interrupt_number, action->prio, action->o_events);
		}
	}
}


stm32_dma_channel_t * stm32_dma_set_handle(u32 dma_number, u32 stream_number){
	if( (dma_number < MCU_DMA_PORTS) && (stream_number < MCU_DMA_STREAM_COUNT) ){

		int interrupt_number;
		interrupt_number = stm32_dma_get_interrupt_number(dma_number, stream_number);
		if( interrupt_number < 0 ){
			return 0;
		}

		if( dma_number == 0 ){
			__HAL_RCC_DMA1_CLK_ENABLE();
#if MCU_DMA_PORTS > 1
		} else {
			__HAL_RCC_DMA2_CLK_ENABLE();
#endif
		}

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

stm32_dma_channel_t * stm32_dma_setattr(const stm32_dma_channel_config_t * config){

	stm32_dma_channel_t * channel;
	u32 o_flags = config->o_flags;
	channel = stm32_dma_set_handle(config->dma_number, config->stream_number);
	if( channel == 0 ){ return 0; }
	channel->handle.Instance = stm32_dma_get_stream_instance(config->dma_number, config->stream_number);

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


static void mcu_core_dma_handler(int dma_number, int stream_number){
	HAL_DMA_IRQHandler(&stm32_dma_handle[dma_number].stream[stream_number].handle);
}

void mcu_core_dma1_stream0_isr(){ mcu_core_dma_handler(0, 0); }
void mcu_core_dma1_stream1_isr(){ mcu_core_dma_handler(0, 1); }
void mcu_core_dma1_stream2_isr(){ mcu_core_dma_handler(0, 2); }
void mcu_core_dma1_stream3_isr(){ mcu_core_dma_handler(0, 3); }
void mcu_core_dma1_stream4_isr(){ mcu_core_dma_handler(0, 4); }
void mcu_core_dma1_stream5_isr(){ mcu_core_dma_handler(0, 5); }
void mcu_core_dma1_stream6_isr(){ mcu_core_dma_handler(0, 6); }
#if !defined STM32L4
void mcu_core_dma1_stream7_isr(){ mcu_core_dma_handler(0, 7); }
#endif


#if MCU_DMA_PORTS > 1

void mcu_core_dma2_stream0_isr(){ mcu_core_dma_handler(1, 0); }
void mcu_core_dma2_stream1_isr(){ mcu_core_dma_handler(1, 1); }
void mcu_core_dma2_stream2_isr(){ mcu_core_dma_handler(1, 2); }
void mcu_core_dma2_stream3_isr(){ mcu_core_dma_handler(1, 3); }
void mcu_core_dma2_stream4_isr(){ mcu_core_dma_handler(1, 4); }
void mcu_core_dma2_stream5_isr(){ mcu_core_dma_handler(1, 5); }
void mcu_core_dma2_stream6_isr(){ mcu_core_dma_handler(1, 6); }
#if !defined STM32L4
void mcu_core_dma2_stream7_isr(){ mcu_core_dma_handler(1, 7); }
#endif
#endif

#endif
