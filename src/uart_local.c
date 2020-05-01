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

#include <fcntl.h>
#include <cortexm/cortexm.h>
#include <mcu/pio.h>
#include <mcu/debug.h>
#include <mcu/core.h>
#include "uart_local.h"

#if MCU_UART_PORTS > 0

#include "uart_local.h"


uart_local_t m_uart_local[MCU_UART_PORTS] MCU_SYS_MEM;
USART_TypeDef * const uart_regs_table[MCU_UART_PORTS] = MCU_UART_REGS;
u8 const uart_irqs[MCU_UART_PORTS] = MCU_UART_IRQS;

static void handle_bytes_received(uart_local_t * local, u16 bytes_received);

int uart_local_open(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);
	if ( local->ref_count == 0 ){

		local->hal_handle.Instance = uart_regs_table[port];

		switch(port){
			case 0:
				__HAL_RCC_USART1_CLK_ENABLE();
				break;
#if defined USART2
			case 1:
				__HAL_RCC_USART2_CLK_ENABLE();
				break;
#endif
#if defined USART3
			case 2:
				__HAL_RCC_USART3_CLK_ENABLE();
				break;
#endif
#if defined UART4
			case 3:
				__HAL_RCC_UART4_CLK_ENABLE();
				break;
#endif
#if defined UART5
			case 4:
				__HAL_RCC_UART5_CLK_ENABLE();
				break;
#endif
#if defined USART6
			case 5:
				__HAL_RCC_USART6_CLK_ENABLE();
				break;
#endif
#if defined UART7
			case 6:
				__HAL_RCC_UART7_CLK_ENABLE();
				break;
#endif
#if defined UART8
			case 7:
				__HAL_RCC_UART8_CLK_ENABLE();
				break;
#endif
#if defined UART9
			case 8:
				__HAL_RCC_UART9_CLK_ENABLE();
				break;
#endif
#if defined UART10
			case 9:
				__HAL_RCC_UART10_CLK_ENABLE();
				break;
#endif

		}
		//reset HAL UART
		cortexm_enable_irq(uart_irqs[port]);

	}
	local->ref_count++;

	return 0;
}

int uart_local_close(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);

	if ( local->ref_count > 0 ){
		if ( local->ref_count == 1 ){
			cortexm_disable_irq(uart_irqs[port]);
			switch(port){
				case 0:
					__HAL_RCC_USART1_CLK_DISABLE();
					break;
#if defined USART2
				case 1:
					__HAL_RCC_USART2_CLK_DISABLE();
					break;
#endif
#if defined USART3
				case 2:
					__HAL_RCC_USART3_CLK_DISABLE();
					break;
#endif
#if defined UART4
				case 3:
					__HAL_RCC_UART4_CLK_DISABLE();
					break;
#endif
#if defined UART5
				case 4:
					__HAL_RCC_UART5_CLK_DISABLE();
					break;
#endif
#if defined USART6
				case 5:
					__HAL_RCC_USART6_CLK_DISABLE();
					break;
#endif
#if defined UART7
				case 6:
					__HAL_RCC_UART7_CLK_DISABLE();
					break;
#endif
#if defined UART8
				case 7:
					__HAL_RCC_UART8_CLK_DISABLE();
					break;
#endif
#if defined UART9
				case 8:
					__HAL_RCC_UART9_CLK_DISABLE();
					break;
#endif
#if defined UART10
				case 9:
					__HAL_RCC_UART10_CLK_DISABLE();
					break;
#endif

			}
			local->hal_handle.Instance = 0;
		}
		local->ref_count--;
	}
	return 0;
}

int uart_local_getinfo(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);

	uart_info_t * info = ctl;

	info->o_flags = UART_FLAG_IS_PARITY_NONE |
			UART_FLAG_IS_PARITY_ODD |
			UART_FLAG_IS_PARITY_EVEN |
			UART_FLAG_IS_STOP1 |
			UART_FLAG_IS_STOP2 |
			UART_FLAG_IS_RX_FIFO;

	if( local->fifo_config ){
		fifo_info_t fifo_info;
		info->o_flags |= UART_FLAG_IS_RX_FIFO;
		fifo_getinfo(&fifo_info, local->fifo_config, &local->fifo_state);
		info->size_ready = fifo_info.size_ready;
		info->size = fifo_info.size;
	} else {
		info->size_ready = 0;
		info->size = 0;
	}

	return 0;
}

int uart_local_setattr(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);
	u32 o_flags;
	u32 freq;

	const uart_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){
		return SYSFS_SET_RETURN(EINVAL);
	}

	const uart_config_t * config = handle->config;
	if( config ){
		local->fifo_config = config->fifo_config;
	} else {
		local->fifo_config = 0;
	}

	o_flags = attr->o_flags;

	if( o_flags & UART_FLAG_SET_LINE_CODING ){
		freq = attr->freq;
		if( freq == 0 ){
			freq = 115200;
		}

		local->hal_handle.Init.BaudRate = freq;

		local->hal_handle.Init.WordLength = UART_WORDLENGTH_8B;
		if( attr->width == 9 ){
			local->hal_handle.Init.WordLength = UART_WORDLENGTH_9B;
		}

		local->hal_handle.Init.StopBits = UART_STOPBITS_1;
		if( o_flags & UART_FLAG_IS_STOP2 ){
			local->hal_handle.Init.StopBits = UART_STOPBITS_2;
		}

		local->hal_handle.Init.Parity = UART_PARITY_NONE;
		if( o_flags & UART_FLAG_IS_PARITY_EVEN ){
			local->hal_handle.Init.Parity = UART_PARITY_EVEN;
		} else if( o_flags & UART_FLAG_IS_PARITY_ODD ){
			local->hal_handle.Init.Parity = UART_PARITY_ODD;
		}

		const uart_pin_assignment_t * pin_assignment
				= mcu_select_pin_assignment(
					&attr->pin_assignment,
					MCU_CONFIG_PIN_ASSIGNMENT(uart_config_t, handle),
					MCU_PIN_ASSIGNMENT_COUNT(uart_pin_assignment_t)
					);

		local->hal_handle.Init.Mode = UART_MODE_TX_RX;
		if( pin_assignment ){
			if( pin_assignment->tx.port == 0xff ){
				local->hal_handle.Init.Mode = UART_MODE_RX;
			} else if ( pin_assignment->rx.port == 0xff ){
				local->hal_handle.Init.Mode = UART_MODE_TX;
			}
		}
		local->hal_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		local->hal_handle.Init.OverSampling = UART_OVERSAMPLING_8;

		//pin assignments
		if( mcu_set_pin_assignment(
					&(attr->pin_assignment),
					MCU_CONFIG_PIN_ASSIGNMENT(uart_config_t, handle),
					MCU_PIN_ASSIGNMENT_COUNT(uart_pin_assignment_t),
					CORE_PERIPH_UART, port, 0, 0, 0) < 0 ){
			return SYSFS_SET_RETURN(EINVAL);
		}

		if( HAL_UART_Init(&local->hal_handle) != HAL_OK ){
			return SYSFS_SET_RETURN(EIO);
		}

		//enable the idle interrupt and initialize the fifo
		if( local->fifo_config ){
			fifo_ioctl_local(local->fifo_config, &local->fifo_state, I_FIFO_INIT, 0);
			//enables idle interrupt
			SET_BIT(local->hal_handle.Instance->CR1, USART_CR1_IDLEIE);
		}

	}

	return SYSFS_RETURN_SUCCESS;
}

int uart_local_setaction(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);
	mcu_action_t * action = (mcu_action_t*)ctl;

	if( action->handler.callback == 0 ){
		//if there is an ongoing operation -- cancel it
		if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
			//execute the read callback if not null
			fifo_cancel_async_read(&local->fifo_state);
		}

		if( action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE ){
#if defined STM32F4
			if( local->o_flags & UART_LOCAL_IS_DMA ){
				//stop the DMA write but not the DMA read -- there isn't a simple call to do this
			} else {
				HAL_UART_AbortTransmit_IT(&local->hal_handle);
			}
#endif
			devfs_execute_write_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_CANCELED);
		}
	}

	cortexm_set_irq_priority(uart_irqs[port], action->prio, action->o_events);
	return 0;
}

int uart_local_put(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);
	u8 c = (u32)ctl;

	if( HAL_UART_Transmit(&local->hal_handle, &c, 1, HAL_MAX_DELAY) != HAL_OK ){
		return SYSFS_SET_RETURN(EIO);
	}

	return SYSFS_RETURN_SUCCESS;
}

int uart_local_flush(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);

	/*
	 * The interrupt and DMA reads rely on the head staying
	 * in the right place for the incoming data. So the
	 * head is restored and the tail is assiged to
	 * the head when the fifo is flushed.
	 *
	 */
	u16 head = local->fifo_state.atomic_position.access.head;
	fifo_flush(&local->fifo_state);
	local->fifo_state.atomic_position.access.head = head;
	local->fifo_state.atomic_position.access.tail = head;

	return SYSFS_RETURN_SUCCESS;
}


int uart_local_get(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);

	if( local->fifo_config == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

	//is there a byte on the FIFO to read?
	int result = fifo_read_buffer(local->fifo_config, &local->fifo_state, ctl, 1);
	if( result == 1 ){
		return SYSFS_RETURN_SUCCESS;
	}

	return SYSFS_SET_RETURN(ENODATA);
}

int uart_local_read(const devfs_handle_t * handle, devfs_async_t * async){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);

	if( local->fifo_config == 0 ){
		return SYSFS_SET_RETURN(ENOSYS);
	}

	//read the fifo, block if no bytes are available
	int result = fifo_read_local(local->fifo_config, &local->fifo_state, async, 0);

	return result;
}

void handle_bytes_received(uart_local_t * local, u16 bytes_received){

	//increment the head by the number of bytes received
	for(u16 i=0; i < bytes_received; i++){
		fifo_inc_head(
					&local->fifo_state,
					local->fifo_config->size
					);

	}

	//now tell the fifo the head has been updated so it can return data to the user asynchronously
	fifo_data_received(
				local->fifo_config,
				&local->fifo_state
				);
}

void HAL_UART_RxIdleCallback(UART_HandleTypeDef *huart){
	uart_local_t * local = (uart_local_t*)huart;
	u16 bytes_received;
	u16 head = local->fifo_state.atomic_position.access.head;
	if( huart->hdmarx ){
		bytes_received = local->fifo_config->size - __HAL_DMA_GET_COUNTER(huart->hdmarx) - head;
	} else {
		bytes_received = (huart->RxXferSize - huart->RxXferCount) - head;
	}
	handle_bytes_received(local, bytes_received);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart){
	MCU_UNUSED_ARGUMENT(huart);
}

//called when RX IT is complete or when DMA does full transfer
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	uart_local_t * local = (uart_local_t*)huart;
	u16 bytes_received;
	bytes_received = local->fifo_config->size - local->fifo_state.atomic_position.access.head;
	handle_bytes_received(local, bytes_received);

	if( local->hal_handle.hdmarx == 0 ){
		//if not in circular DMA mode -- start the next interrupt based read
		HAL_UART_Receive_IT(&local->hal_handle,
												(u8*)local->fifo_config->buffer,
												local->fifo_config->size);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	uart_local_t * local = (uart_local_t*)huart;
	devfs_execute_read_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_ERROR | MCU_EVENT_FLAG_CANCELED);
	u16 bytes_received;

	if( local->hal_handle.hdmarx == 0 ){

		bytes_received = local->fifo_config->size - huart->RxXferCount - local->fifo_state.atomic_position.access.head;
		handle_bytes_received(local, bytes_received);

		//reset the head
		fifo_flush(&local->fifo_state);


		//if not in circular DMA mode -- start the next interrupt based read
		HAL_UART_Receive_IT(&local->hal_handle,
												(u8*)local->fifo_config->buffer,
												local->fifo_config->size);
	} else {
		bytes_received = local->fifo_config->size - __HAL_DMA_GET_COUNTER(huart->hdmarx) - local->fifo_state.atomic_position.access.head;
		handle_bytes_received(local, bytes_received);

		//reset the head
		fifo_flush(&local->fifo_state);


		//if not in circular DMA mode -- start the next interrupt based read
		HAL_UART_Receive_DMA(&local->hal_handle,
												 (u8*)local->fifo_config->buffer,
												 local->fifo_config->size);
	}

}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart){
	//this does not need to do anything
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart){

}

void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart){

}

void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart){
	mcu_debug_printf("abort rx\n");
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	uart_local_t * local = (uart_local_t*)huart;
	int nbyte = 0;
	//when transfer is complete, count is 0 and size is the original async->nbyte value
	nbyte = huart->TxXferSize - huart->TxXferCount;
	devfs_execute_write_handler(&local->transfer_handler, 0, nbyte, MCU_EVENT_FLAG_DATA_READY);
}


void mcu_uart_isr(int port){
	uart_local_t * local = m_uart_local + port;
	HAL_UART_IRQHandler(&local->hal_handle);

#if defined USART_SR_IDLE
	if(  READ_REG(local->hal_handle.Instance->SR) & USART_SR_IDLE ){
		local->hal_handle.Instance->DR; //this will clear the idle bit
#elif defined USART_ISR_IDLE
	if(  READ_REG(local->hal_handle.Instance->ISR) & USART_ISR_IDLE ){
		SET_BIT(local->hal_handle.Instance->ICR, USART_ICR_IDLECF); //this will clear the idle bit
#endif
		HAL_UART_RxIdleCallback((UART_HandleTypeDef*)local);
		return;
	}
}

#if defined USART1
void mcu_core_usart1_isr(){
	mcu_uart_isr(0);
}
#endif

#if defined USART2
void mcu_core_usart2_isr(){
	mcu_uart_isr(1);
}
#endif

#if defined USART3
void mcu_core_usart3_isr(){
	mcu_uart_isr(2);
}
#endif

#if defined UART4
void mcu_core_uart4_isr(){
	mcu_uart_isr(3);
}
#endif

#if defined UART5
void mcu_core_uart5_isr(){
	mcu_uart_isr(4);
}
#endif

#if defined USART6
void mcu_core_usart6_isr(){
	mcu_uart_isr(5);
}
#endif

#if defined UART7
void mcu_core_uart7_isr(){
	mcu_uart_isr(6);
}
#endif


#if defined UART8
void mcu_core_uart8_isr(){
	mcu_uart_isr(7);
}
#endif

#if defined UART9
void mcu_core_uart9_isr(){
	mcu_uart_isr(8);
}
#endif

#if defined UART10
void mcu_core_uart10_isr(){
	mcu_uart_isr(9);
}
#endif


#endif
