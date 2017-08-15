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

#include <errno.h>
#include <fcntl.h>
#include "cortexm/cortexm.h"
#include "mcu/uart.h"
#include "mcu/pio.h"
#include "mcu/debug.h"
#include "mcu/core.h"

#include "hal.h"


#define UART_DMA_START_CHAN 0
#define UART_PORTS MCU_UART_PORTS

#if MCU_UART_PORTS > 0

typedef struct {
	UART_HandleTypeDef hal_handle;
	mcu_event_handler_t write;
	mcu_event_handler_t read;
	u16 received_len;
	u16 bytes_received;
	u8 ref_count;
} uart_local_t;

static uart_local_t uart_local[UART_PORTS] MCU_SYS_MEM;
USART_TypeDef * const uart_regs_table[UART_PORTS] = MCU_UART_REGS;
u8 const uart_irqs[UART_PORTS] = MCU_UART_IRQS;

static void exec_readcallback(int port, USART_TypeDef * uart_regs, u32 o_events);
static void exec_writecallback(int port, USART_TypeDef * uart_regs, u32 o_events);

void mcu_uart_dev_power_on(const devfs_handle_t * handle){
	int port = handle->port;
	if ( uart_local[port].ref_count == 0 ){
		switch(port){
		case 0:
			//mcu_lpc_core_enable_pwr(PCUART0);
			break;
#if MCU_UART_PORTS > 1
		case 1:
			//mcu_lpc_core_enable_pwr(PCUART1);
			break;
#endif
#if MCU_UART_PORTS > 2
		case 2:
			//mcu_lpc_core_enable_pwr(PCUART2);
			break;
#endif
#if MCU_UART_PORTS > 3
		case 3:
			//mcu_lpc_core_enable_pwr(PCUART3);
			break;
#endif
#if MCU_UART_PORTS > 4
		case 4:
			//mcu_lpc_core_enable_pwr(PCUART4);
			break;
#endif
		}
		//uart_local[port].tx_bufp = NULL;
		//uart_local[port].rx_bufp = NULL;
	}
	uart_local[port].ref_count++;


}

void mcu_uart_dev_power_off(const devfs_handle_t * handle){
	int port = handle->port;

	if ( uart_local[port].ref_count > 0 ){
		if ( uart_local[port].ref_count == 1 ){
			cortexm_disable_irq((void*)(u32)(uart_irqs[port]));
			switch(port){
			case 0:

				//mcu_lpc_core_disable_pwr(PCUART0);
				break;
#if MCU_UART_PORTS > 1
			case 1:
				//mcu_lpc_core_disable_pwr(PCUART1);
				break;
#endif
#if MCU_UART_PORTS > 2
			case 2:
				//mcu_lpc_core_disable_pwr(PCUART2);
				break;
#endif
#if MCU_UART_PORTS > 3
			case 3:
				//mcu_lpc_core_disable_pwr(PCUART3);
				break;
#endif
#if MCU_UART_PORTS > 4
			case 4:
				//mcu_lpc_core_disable_pwr(PCUART4);
				break;
#endif

			}
			//uart_local[port].tx_bufp = NULL;
			//uart_local[port].rx_bufp = NULL;
		}
		uart_local[port].ref_count--;
	}
}

int mcu_uart_dev_is_powered(const devfs_handle_t * handle){
	return ( uart_local[handle->port].ref_count != 0 );
}

int mcu_uart_getinfo(const devfs_handle_t * handle, void * ctl){

	uart_info_t * info = ctl;

	info->o_flags = UART_FLAG_IS_PARITY_NONE |
			UART_FLAG_IS_PARITY_ODD |
			UART_FLAG_IS_PARITY_EVEN |
			UART_FLAG_IS_STOP1 |
			UART_FLAG_IS_STOP2;

	return 0;
}

int mcu_uart_setattr(const devfs_handle_t * handle, void * ctl){
	USART_TypeDef * uart_regs;
	u32 o_flags;
	int port = handle->port;

	uart_local_t * uart = uart_local + port;
	const uart_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){
		return -1;
	}

	o_flags = attr->o_flags;

	uart->hal_handle.Init.StopBits = UART_STOPBITS_1;
	if( o_flags & UART_FLAG_IS_STOP2 ){
		uart->hal_handle.Init.StopBits = UART_STOPBITS_2;
	}

	uart->hal_handle.Init.BaudRate = attr->freq;
	uart->hal_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart->hal_handle.Init.OverSampling = UART_OVERSAMPLING_16;
	uart->hal_handle.Init.WordLength = UART_WORDLENGTH_8B;
	if( attr->width == 9 ){
		uart->hal_handle.Init.WordLength = UART_WORDLENGTH_9B;
	}

	//pin assignments

	//clock setup (HAL MSP stuff)

	HAL_UART_INIT(&uart->hal_handle);


	return 0;
}

static void exec_readcallback(int port, USART_TypeDef * uart_regs, u32 o_events){
	uart_event_t uart_event;
	mcu_execute_event_handler(&(uart_local[port].read), o_events, &uart_event);

	//if the callback is NULL now, disable the interrupt
	if( uart_local[port].read.callback == NULL ){
		//uart_regs->IER &= ~UIER_RBRIE; //disable the receive interrupt
	}
}

static void exec_writecallback(int port, USART_TypeDef * uart_regs, u32 o_events){
	uart_event_t uart_event;
	//uart_local[port].tx_bufp = NULL;
	//call the write callback
	mcu_execute_event_handler(&(uart_local[port].write), o_events, &uart_event);

	//if the callback is NULL now, disable the interrupt
	if( uart_local[port].write.callback == NULL ){
		//uart_regs->IER &= ~UIER_ETBEI; //disable the transmit interrupt
	}
}

int mcu_uart_setaction(const devfs_handle_t * handle, void * ctl){
	mcu_action_t * action = (mcu_action_t*)ctl;
	int port = handle->port;

	USART_TypeDef * uart_regs = uart_regs_table[port];

	if( action->handler.callback == 0 ){
		//if there is an ongoing operation -- cancel it

		if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
			//execute the read callback if not null
			exec_readcallback(port, uart_regs, MCU_EVENT_FLAG_CANCELED);
			uart_local[port].read.callback = 0;
			//uart_regs->IER &= ~UIER_RBRIE; //disable the receive interrupt
		}

		if( action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE ){
			exec_writecallback(port, uart_regs, MCU_EVENT_FLAG_CANCELED);
			uart_local[port].write.callback = 0;
		}

	} else {

		if( cortexm_validate_callback(action->handler.callback) < 0 ){
			return -1;
		}

		if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
			uart_local[port].read = action->handler;
			//uart_regs->IER |= (UIER_RBRIE);  //enable the receiver interrupt
			//uart_local[port].rx_bufp = NULL; //the callback will need to read the incoming data
		}

		if ( action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE ){
			uart_local[port].write = action->handler;
		}
	}

	cortexm_set_irq_prio(uart_irqs[port], action->prio);


	return 0;
}

int mcu_uart_put(const devfs_handle_t * handle, void * ctl){
	char c = (u32)ctl;
	int port = handle->port;
	uart_local_t * uart = uart_local + port;

	if( HAL_UART_Transmit(&uart->hal_handle, &c, 1, 100) != HAL_OK ){
		return -1;
	}

	return 0;
}

int mcu_uart_flush(const devfs_handle_t * handle, void * ctl){
	char c;
	mcu_uart_get(handle, &c);
	return 0;
}


int mcu_uart_get(const devfs_handle_t * handle, void * ctl){
	u8 * dest = ctl;
	int port = handle->port;
	uart_local_t * uart = uart_local + port;

	if( HAL_UART_Receive(&uart->hal_handle, dest, 1, 0) == HAL_OK ){
		return 0;
	}

	errno = ENODATA;
	return -1;
}

int mcu_uart_getall(const devfs_handle_t * handle, void * ctl){
	return mcu_uart_get(handle, ctl); //only has a one byte buffer
}



int mcu_uart_dev_read(const devfs_handle_t * handle, devfs_async_t * async){
	int ret;
	int port = handle->port;
	uart_local_t * uart = uart_local + port;


	if( uart->read.callback ){
		errno = EBUSY;
		return -1;
	}

	//this driver will only read one byte at a time
	async->nbyte = 1;

	//check to see if a byte is ready right now
	if( mcu_uart_get(handle, async->buf) == 0 ){
		return 1;
	}

	if ( async->flags & O_NONBLOCK ){
		errno = EAGAIN;
		return -1;
	} else {
		//no bytes
		if( cortexm_validate_callback(async->handler.callback) < 0 ){
			return -1;
		}

		uart->read = async->handler;
		if( HAL_UART_Receive_IT(&uart->hal_handle, async->buf, async->nbyte) == HAL_OK ){
			return 0;
		}
	}

	//this needs to read 1 byte at a time

	uart->read.callback = 0;
	errno = EIO;
	return -1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	uart_local_t * uart = (uart_local_t*)huart;
	uart_event_t uart_event;
	mcu_execute_event_handler(&uart->read, MCU_EVENT_FLAG_DATA_READY, &uart_event);
}

int mcu_uart_dev_write(const devfs_handle_t * handle, devfs_async_t * async){
	int ret;
	int port = handle->port;
	uart_local_t * uart = uart_local + port;


	if( uart->write.callback ){
		errno = EBUSY;
		return -1;
	}

	ret = HAL_UART_Transmit_IT(&uart->hal_handle, async->buf, async->nbyte);
	if( ret == HAL_OK ){
		return 0;
	}

	errno = EIO;
	return -1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	uart_local_t * uart = (uart_local_t*)huart;
	uart_event_t uart_event;
	mcu_execute_event_handler(&uart->write, MCU_EVENT_FLAG_WRITE_COMPLETE, &uart_event);
}


void mcu_uart_isr(int port){
	//first determine if this is a UART0 interrupt
	uart_local_t * uart = uart_local + port;
	HAL_UART_IRQHandler(&uart->hal_handle);
}

void mcu_core_uart0_isr(){
	mcu_uart_isr(0);
}

#if MCU_UART_PORTS > 1
void mcu_core_uart1_isr(){
	mcu_uart_isr(1);
}
#endif

#if MCU_UART_PORTS > 2
void mcu_core_uart2_isr(){
	mcu_uart_isr(2);
}
#endif

#if MCU_UART_PORTS > 3
void mcu_core_uart3_isr(){
	mcu_uart_isr(3);
}
#endif

#if MCU_UART_PORTS > 4
void mcu_core_uart4_isr(){
	mcu_uart_isr(4);
}
#endif

#if MCU_UART_PORTS > 5
void mcu_core_uart5_isr(){
	mcu_uart_isr(5);
}
#endif

#endif

