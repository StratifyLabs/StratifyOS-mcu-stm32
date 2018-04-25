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
#include "stm32_local.h"
#include "cortexm/cortexm.h"
#include "mcu/uart.h"
#include "mcu/pio.h"
#include "mcu/debug.h"
#include "mcu/core.h"



#define UART_DMA_START_CHAN 0
#define UART_PORTS MCU_UART_PORTS

#if MCU_UART_PORTS > 0

enum {
    UART_LOCAL_FLAG_IS_INCOMING_ENABLED = (1<<0),
    UART_LOCAL_FLAG_IS_INCOMING_AVAILABLE = (1<<1)
};

typedef struct {
	UART_HandleTypeDef hal_handle;
	mcu_event_handler_t write;
	mcu_event_handler_t read;
	u8 ref_count;
	const uart_attr_t * attr;
    u8 o_flags;
    u8 incoming;
    u8 incoming_received;
} uart_local_t;

static uart_local_t uart_local[UART_PORTS] MCU_SYS_MEM;
USART_TypeDef * const uart_regs_table[UART_PORTS] = MCU_UART_REGS;
u8 const uart_irqs[UART_PORTS] = MCU_UART_IRQS;

static void exec_readcallback(uart_local_t * uart, u32 o_events);
static void exec_writecallback(uart_local_t * uart, u32 o_events);

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(uart, UART_VERSION, I_MCU_TOTAL + I_UART_TOTAL, mcu_uart_get, mcu_uart_put, mcu_uart_flush)

int mcu_uart_open(const devfs_handle_t * handle){
	int port = handle->port;
	if ( uart_local[port].ref_count == 0 ){

		uart_local[port].hal_handle.Instance = uart_regs_table[port];

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

		}
		//reset HAL UART
        cortexm_enable_irq(uart_irqs[port]);

	}
	uart_local[port].ref_count++;

    return 0;
}

int mcu_uart_close(const devfs_handle_t * handle){
	int port = handle->port;

	if ( uart_local[port].ref_count > 0 ){
		if ( uart_local[port].ref_count == 1 ){
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

			}
			uart_local[port].hal_handle.Instance = 0;
		}
		uart_local[port].ref_count--;
    }
    return 0;
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
	u32 o_flags;
	int port = handle->port;
	u32 freq;

	uart_local_t * uart = uart_local + port;
	uart->attr = mcu_select_attr(handle, ctl);
	if( uart->attr == 0 ){
        return SYSFS_SET_RETURN(EINVAL);
	}

	o_flags = uart->attr->o_flags;

	if( o_flags & UART_FLAG_SET_LINE_CODING ){
		freq = uart->attr->freq;
		if( freq == 0 ){
			freq = 115200;
		}

		uart->hal_handle.Init.BaudRate = freq;

		uart->hal_handle.Init.WordLength = UART_WORDLENGTH_8B;
		if( uart->attr->width == 9 ){
			uart->hal_handle.Init.WordLength = UART_WORDLENGTH_9B;
		}

		uart->hal_handle.Init.StopBits = UART_STOPBITS_1;
		if( o_flags & UART_FLAG_IS_STOP2 ){
			uart->hal_handle.Init.StopBits = UART_STOPBITS_2;
		}

        uart->hal_handle.Init.Parity = UART_PARITY_NONE;
		if( o_flags & UART_FLAG_IS_PARITY_EVEN ){
            uart->hal_handle.Init.Parity = UART_PARITY_EVEN;
		} else if( o_flags & UART_FLAG_IS_PARITY_ODD ){
            uart->hal_handle.Init.Parity = UART_PARITY_ODD;
		}

		uart->hal_handle.Init.Mode = UART_MODE_TX_RX;
		uart->hal_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		uart->hal_handle.Init.OverSampling = UART_OVERSAMPLING_16;

		//pin assignments
		if( mcu_set_pin_assignment(
				&(uart->attr->pin_assignment),
				MCU_CONFIG_PIN_ASSIGNMENT(uart_config_t, handle),
				MCU_PIN_ASSIGNMENT_COUNT(uart_pin_assignment_t),
                CORE_PERIPH_UART, port, 0, 0, 0) < 0 ){
            return SYSFS_SET_RETURN(EINVAL);
        }

		if( HAL_UART_Init(&uart->hal_handle) != HAL_OK ){
            return SYSFS_SET_RETURN(EIO);
        }
	}

	return 0;
}

static void exec_readcallback(uart_local_t * uart, u32 o_events){
	uart_event_t uart_event;
    uart_event.value = uart->incoming_received;
    mcu_execute_event_handler(&(uart->read), o_events, &uart_event);

	//if the callback is NULL now, disable the interrupt
    if( uart->read.callback == NULL ){

	}
}

static void exec_writecallback(uart_local_t * uart, u32 o_events){
	uart_event_t uart_event;
	//call the write callback
    mcu_execute_event_handler(&(uart->write), o_events, &uart_event);

	//if the callback is NULL now, disable the interrupt
    if( uart->write.callback == NULL ){

    }
}

int mcu_uart_setaction(const devfs_handle_t * handle, void * ctl){
	mcu_action_t * action = (mcu_action_t*)ctl;
	int port = handle->port;
    uart_local_t * uart = uart_local + port;

	if( action->handler.callback == 0 ){
		//if there is an ongoing operation -- cancel it

		if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
			//execute the read callback if not null
            if( uart->o_flags & UART_LOCAL_FLAG_IS_INCOMING_ENABLED ){
                uart->o_flags &= ~UART_LOCAL_FLAG_IS_INCOMING_ENABLED;
#if defined STM32F4
                HAL_UART_AbortReceive_IT(&uart->hal_handle);
#endif
            }

            exec_readcallback(uart, MCU_EVENT_FLAG_CANCELED);
            uart->read.callback = 0;
		}

		if( action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE ){
#if defined STM32F4
            HAL_UART_AbortTransmit_IT(&uart->hal_handle);
#endif
            exec_writecallback(uart, MCU_EVENT_FLAG_CANCELED);
            uart->write.callback = 0;
		}

	} else {

		if( cortexm_validate_callback(action->handler.callback) < 0 ){
            return SYSFS_SET_RETURN(EPERM);
        }

		if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
            uart->read = action->handler;

            //enable the receiver so that the action is called when a byte arrives
            uart->o_flags |= UART_LOCAL_FLAG_IS_INCOMING_ENABLED;
            uart->o_flags &= ~UART_LOCAL_FLAG_IS_INCOMING_AVAILABLE;
            if( HAL_UART_Receive_IT(&uart->hal_handle, &uart->incoming, 1) != HAL_OK ){
                return SYSFS_SET_RETURN(EIO);
            }
		}

		if ( action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE ){
            uart->write = action->handler;
		}
	}

	cortexm_set_irq_priority(uart_irqs[port], action->prio);
	return 0;
}

int mcu_uart_put(const devfs_handle_t * handle, void * ctl){
	u8 c = (u32)ctl;
	int port = handle->port;
	uart_local_t * uart = uart_local + port;

	if( HAL_UART_Transmit(&uart->hal_handle, &c, 1, HAL_MAX_DELAY) != HAL_OK ){
        return SYSFS_SET_RETURN(EIO);
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


    //if action was set return uart->incoming;
    if( uart->o_flags & UART_LOCAL_FLAG_IS_INCOMING_ENABLED ){
        if( uart->o_flags & UART_LOCAL_FLAG_IS_INCOMING_AVAILABLE ){
            uart->o_flags &= ~UART_LOCAL_FLAG_IS_INCOMING_AVAILABLE;
            *dest = uart->incoming_received;
            return 0;
        }
        return SYSFS_SET_RETURN(ENODATA);
    }


	if( HAL_UART_Receive(&uart->hal_handle, dest, 1, 0) == HAL_OK ){
		return 0;
	}

    return SYSFS_SET_RETURN(ENODATA);
}

int mcu_uart_getall(const devfs_handle_t * handle, void * ctl){
	return mcu_uart_get(handle, ctl); //only has a one byte buffer
}

int mcu_uart_read(const devfs_handle_t * handle, devfs_async_t * async){
	int port = handle->port;
	uart_local_t * uart = uart_local + port;


	if( uart->read.callback ){
        return SYSFS_SET_RETURN(EBUSY);
	}

	//this driver will only read one byte at a time
	async->nbyte = 1;

	//check to see if a byte is ready right now
	if( mcu_uart_get(handle, async->buf) == 0 ){
        //got one byte
		return 1;
	}

	if ( async->flags & O_NONBLOCK ){
        return SYSFS_SET_RETURN(EAGAIN);
	} else {
		//no bytes
		if( cortexm_validate_callback(async->handler.callback) < 0 ){
            return SYSFS_SET_RETURN(EPERM);
        }

		uart->read = async->handler;
		if( HAL_UART_Receive_IT(&uart->hal_handle, async->buf, async->nbyte) == HAL_OK ){
			return 0;
		}
	}

	//this needs to read 1 byte at a time

	uart->read.callback = 0;
    return SYSFS_SET_RETURN(EIO);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	uart_local_t * uart = (uart_local_t*)huart;
	uart_event_t uart_event;

    if( uart->o_flags & UART_LOCAL_FLAG_IS_INCOMING_ENABLED ){
        uart->o_flags |= UART_LOCAL_FLAG_IS_INCOMING_AVAILABLE;
        uart->incoming_received = uart->incoming;
        uart_event.value = uart->incoming_received;
        HAL_UART_Receive_IT(&uart->hal_handle, &uart->incoming, 1);
    }

	mcu_execute_event_handler(&uart->read, MCU_EVENT_FLAG_DATA_READY, &uart_event);
}

int mcu_uart_write(const devfs_handle_t * handle, devfs_async_t * async){
	int ret;
	int port = handle->port;
	uart_local_t * uart = uart_local + port;


	if( uart->write.callback ){
        return SYSFS_SET_RETURN(EBUSY);
	}

    uart->write = async->handler;
	ret = HAL_UART_Transmit_IT(&uart->hal_handle, async->buf, async->nbyte);
	if( ret == HAL_OK ){
		return 0;
	}

    return SYSFS_SET_RETURN(EIO);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	uart_local_t * uart = (uart_local_t*)huart;
	uart_event_t uart_event;
    uart_event.value = 0;
	mcu_execute_event_handler(&uart->write, MCU_EVENT_FLAG_WRITE_COMPLETE, &uart_event);
}


void mcu_uart_isr(int port){
	uart_local_t * uart = uart_local + port;
	HAL_UART_IRQHandler(&uart->hal_handle);
}

void mcu_core_usart1_isr(){
	mcu_uart_isr(0);
}

#if MCU_UART_PORTS > 1
void mcu_core_usart2_isr(){
	mcu_uart_isr(1);
}
#endif

#if MCU_UART_PORTS > 2
void mcu_core_usart3_isr(){
	mcu_uart_isr(2);
}
#endif

#if MCU_UART_PORTS > 3
void mcu_core_uart4_isr(){
	mcu_uart_isr(3);
}
#endif

#if MCU_UART_PORTS > 4
void mcu_core_uart5_isr(){
	mcu_uart_isr(4);
}
#endif

#if MCU_UART_PORTS > 5
void mcu_core_usart6_isr(){
	mcu_uart_isr(5);
}
#endif

#if MCU_UART_PORTS > 6
void mcu_core_uart7_isr(){
    mcu_uart_isr(6);
}
#endif


#if MCU_UART_PORTS > 7
void mcu_core_uart8_isr(){
    mcu_uart_isr(7);
}
#endif

#endif
