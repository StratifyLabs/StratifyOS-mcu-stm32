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

#include <mcu/uart.h>
#include <mcu/core.h>
#include <mcu/cortexm.h>

#include "hal.h"




#if MCU_UART_PORTS > 0

typedef struct {
	UART_HandleTypeDef handle;
	mcu_event_handler_t write;
	mcu_event_handler_t read;
	int * rx_nbyte;
	u8 ref_count;
} uart_local_t;

USART_TypeDef * const uart_regs[] = {
		USART1,
		USART2,
		USART3,
		UART4,
		UART5,
		USART6
};

uart_local_t m_uart_local[MCU_UART_PORTS] MCU_SYS_MEM;

USART_TypeDef * const uart_regs_table[MCU_UART_PORTS] = MCU_UART_REGS;
u8 const uart_irqs[MCU_UART_PORTS] = MCU_UART_IRQS;

void _mcu_uart_dev_power_on(int port){
	if ( m_uart_local[port].ref_count == 0 ){
		m_uart_local[port].handle.Instance = uart_regs_table[port];
		m_uart_local[port].handle.context = &(m_uart_local[port]);
		switch(port){
		case 0:
			break;
#if MCU_UART_PORTS > 1
		case 1:
			break;
#endif
#if MCU_UART_PORTS > 2
		case 2:
			break;
#endif
#if MCU_UART_PORTS > 3
		case 3:
			break;
#endif
#if MCU_UART_PORTS > 4
		case 4:
			break;
#endif
		}
		_mcu_cortexm_priv_enable_irq((void*)(u32)(uart_irqs[port]));

	}
	m_uart_local[port].ref_count++;
}

void _mcu_uart_dev_power_off(int port){
	if ( m_uart_local[port].ref_count > 0 ){
		if ( m_uart_local[port].ref_count == 1 ){
			_mcu_cortexm_priv_disable_irq((void*)(u32)(uart_irqs[port]));
			switch(port){
			case 0:

				break;
#if MCU_UART_PORTS > 1
			case 1:
				break;
#endif
#if MCU_UART_PORTS > 2
			case 2:
				break;
#endif
#if MCU_UART_PORTS > 3
			case 3:
				break;
#endif
#if MCU_UART_PORTS > 4
			case 4:
				break;
#endif

			}
		}
		m_uart_local[port].ref_count--;
	}
}

int _mcu_uart_dev_powered_on(int port){
	return ( m_uart_local[port].ref_count != 0 );
}


int mcu_uart_setattr(int port, void * ctl){
	int i;
	uart_attr_t * attr = ctl;

	u32 o_flags = attr->o_flags;

	m_uart_local[port].handle.Init.StopBits = UART_STOPBITS_1;
	if( o_flags & UART_FLAG_IS_STOP2 ){
		m_uart_local[port].handle.Init.StopBits = UART_STOPBITS_2;
	}

	m_uart_local[port].handle.Init.Parity = HAL_UART_PARITY_NONE;
	if( o_flags & UART_FLAG_IS_PARITY_EVEN ){
		m_uart_local[port].handle.Init.StopBits = HAL_UART_PARITY_EVEN;
	} else if( o_flags & UART_FLAG_IS_PARITY_ODD ){
		m_uart_local[port].handle.Init.StopBits = HAL_UART_PARITY_ODD;
	}

	m_uart_local[port].handle.Init.BaudRate = attr->freq;
	m_uart_local[port].handle.Init.WordLength = attr->width;
	m_uart_local[port].handle.Init.Mode = UART_MODE_TX_RX;
	m_uart_local[port].handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	m_uart_local[port].handle.Init.OverSampling = UART_OVERSAMPLING_16;

	for(i=0; i < UART_PIN_ASSIGNMENT_COUNT; i++){
		if( attr->pin_assignment[i].port != 0xFF ){
			if( hal_set_alternate_pin_function(attr->pin_assignment[i], CORE_PERIPH_UART, port, GPIO_MODE_AF_PP) < 0 ){
				return -1;
			}
		}
	}

	switch(port){
	case 0:
		__HAL_RCC_USART1_FORCE_RESET();
		__HAL_RCC_USART1_RELEASE_RESET();
		__HAL_RCC_USART1_CLK_ENABLE();
		break;
	case 1:
		__HAL_RCC_USART2_FORCE_RESET();
		__HAL_RCC_USART2_RELEASE_RESET();
		__HAL_RCC_USART2_CLK_ENABLE();
		break;
	case 2:
		__HAL_RCC_USART3_FORCE_RESET();
		__HAL_RCC_USART3_RELEASE_RESET();
		__HAL_RCC_USART3_CLK_ENABLE();
		break;
	case 3:
		__HAL_RCC_UART4_FORCE_RESET();
		__HAL_RCC_UART4_RELEASE_RESET();
		__HAL_RCC_UART4_CLK_ENABLE();
		break;
	case 4:
		__HAL_RCC_UART5_FORCE_RESET();
		__HAL_RCC_UART5_RELEASE_RESET();
		__HAL_RCC_UART5_CLK_ENABLE();
		break;
	case 5:
		__HAL_RCC_USART6_FORCE_RESET();
		__HAL_RCC_USART6_RELEASE_RESET();
		__HAL_RCC_USART6_CLK_ENABLE();
		break;
	}

	HAL_UART_Init(&(m_uart_local[port].handle));
	return 0;
}

int mcu_uart_setaction(int port, void * ctl){



	return 0;
}

int mcu_uart_clear(int port, void * ctl){
	return 0;
}

int mcu_uart_flush(int port, void * ctl){

	return 0;
}


int mcu_uart_getbyte(int port, void * ctl){

	return -1;
}

int mcu_uart_getall(int port, void * ctl){
	return -1;
}



int _mcu_uart_dev_read(const devfs_handle_t * handle, devfs_async_t * rop){
	int len;
	int port;

	//grab the port and registers
	port = DEVFS_GET_PORT(handle);

	//check to see if the port is busy
	if( m_uart_local[port].handle.RxState !=  HAL_UART_STATE_READY ){
		errno = EBUSY;
		return -1;
	}

	//initialize the transfer
	m_uart_local[port].rx_nbyte = &rop->nbyte;

	//Check the local buffer for bytes that are immediately available
	// call HAL_UART_Receive() to get bytes that are ready right now
	len = 0;
	if ( len == 0 ){  //nothing available to read
		if ( rop->flags & O_NONBLOCK ){
			rop->nbyte = 0; //no bytes were read
			errno = EAGAIN;
			len = -1;
		} else {
			if( _mcu_cortexm_priv_validate_callback(rop->handler.callback) < 0 ){
				return -1;
			}
			m_uart_local[port].read = rop->handler;
		}
	}
	return len;
}

int _mcu_uart_dev_write(const devfs_handle_t * handle, devfs_async_t * wop){
	int port;
	port = DEVFS_GET_PORT(handle);

	//Check to see if the port is busy
	if( m_uart_local[port].handle.gState !=  HAL_UART_STATE_READY ){
		errno = EBUSY;
		return -1;
	}

	if ( wop->nbyte == 0 ){
		return 0;
	}

	//Initialize variables
	if( _mcu_cortexm_priv_validate_callback(wop->handler.callback) < 0 ){
		return -1;
	}

	m_uart_local[port].write = wop->handler;
	HAL_UART_Transmit_IT(&(m_uart_local[port].handle), wop->buf, wop->nbyte);

	return 0;
}

void _mcu_uart_isr(int port){
	//first determine if this is a UART0 interrupt
	HAL_UART_IRQHandler(&(m_uart_local[port].handle));
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	//called when a transmission is complete
	uart_local_t * local = huart->context;
	mcu_execute_event_handler(&(local->write), MCU_EVENT_FLAG_WRITE_COMPLETE, 0);

	if( local->write.callback == 0 ){
		//callback action has been disabled
	}


}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//receive complete
	uart_local_t * local = huart->context;
	*(local->rx_nbyte) = huart->RxXferSize - huart->RxXferCount;
	mcu_execute_event_handler(&(local->read), MCU_EVENT_FLAG_DATA_READY, 0);

	if( local->read.callback == 0 ){
		//callback action has been disabled
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	uart_local_t * local = huart->context;
	//error during an operation
	mcu_execute_event_handler(&(local->read), MCU_EVENT_FLAG_ERROR, 0); //operation cancelled
	mcu_execute_event_handler(&(local->write), MCU_EVENT_FLAG_ERROR, 0); //operation cancelled
}

void _mcu_core_usart1_isr(){
	_mcu_uart_isr(0);
}

#if MCU_UART_PORTS > 1
void _mcu_core_usart2_isr(){
	_mcu_uart_isr(1);
}
#endif

#if MCU_UART_PORTS > 2
void _mcu_core_usart3_isr(){
	_mcu_uart_isr(2);
}
#endif

#if MCU_UART_PORTS > 3
void _mcu_core_uart4_isr(){
	_mcu_uart_isr(3);
}
#endif

#if MCU_UART_PORTS > 4
void _mcu_core_uart5_isr(){
	_mcu_uart_isr(4);
}
#endif

#if MCU_UART_PORTS > 5
void _mcu_core_usart6_isr(){
	_mcu_uart_isr(5);
}
#endif

#endif

