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
#include "uart_local.h"

#if MCU_UART_PORTS > 0


DEVFS_MCU_DRIVER_IOCTL_FUNCTION(uart_dma, UART_VERSION, UART_IOC_IDENT_CHAR, I_MCU_TOTAL + I_UART_TOTAL, mcu_uart_dma_get, mcu_uart_dma_put, mcu_uart_dma_flush)

int mcu_uart_dma_open(const devfs_handle_t * handle){
	return uart_local_open(handle);
}

int mcu_uart_dma_close(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);

	if( local->ref_count == 1 ){
		//disable the DMA
		const stm32_uart_dma_config_t * config;

		if( local->transfer_handler.read || local->transfer_handler.write ){
			HAL_UART_DMAStop(&local->hal_handle);
			devfs_execute_cancel_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EIO), 0);
		}

		config = handle->config;
		if( config ){
			stm32_dma_clear_handle(config->dma_config.rx.dma_number, config->dma_config.rx.stream_number);
			stm32_dma_clear_handle(config->dma_config.tx.dma_number, config->dma_config.tx.stream_number);
		}

	}

	return uart_local_close(handle);
}

int mcu_uart_dma_dev_is_powered(const devfs_handle_t * handle){
	return ( m_uart_local[handle->port].ref_count != 0 );
}

int mcu_uart_dma_getinfo(const devfs_handle_t * handle, void * ctl){

	uart_info_t * info = ctl;

	info->o_flags = UART_FLAG_IS_PARITY_NONE |
			UART_FLAG_IS_PARITY_ODD |
			UART_FLAG_IS_PARITY_EVEN |
			UART_FLAG_IS_STOP1 |
			UART_FLAG_IS_STOP2;

	return 0;
}

int mcu_uart_dma_setattr(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);
	const stm32_uart_dma_config_t * config;

	//BSP *MUST* provide DMA configuration information
	config = handle->config;
	if( config == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

	//setup the DMA for receiving
	stm32_dma_channel_t * channel = stm32_dma_setattr(&config->dma_config.rx);
	if( channel == 0 ){ return SYSFS_SET_RETURN(EIO); }

	__HAL_LINKDMA((&local->hal_handle), hdmarx, channel->handle);

	channel = stm32_dma_setattr(&config->dma_config.tx);
	if( channel == 0 ){ return SYSFS_SET_RETURN(EIO); }

	__HAL_LINKDMA((&local->hal_handle), hdmatx, channel->handle);


	int result = uart_local_setattr(handle, ctl);
	if( result < 0 ){ return result; }

	//initiate the DMA circular read
	if( local->fifo_config ){
		if( (result = HAL_UART_Receive_DMA(&local->hal_handle,
										 (u8*)local->fifo_config->buffer,
										 local->fifo_config->size)) != HAL_OK ){
			return SYSFS_SET_RETURN(EIO);
		}
	}


	return SYSFS_RETURN_SUCCESS;
}

int mcu_uart_dma_setaction(const devfs_handle_t * handle, void * ctl){
	const stm32_uart_dma_config_t * config = handle->config;
	if(config == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }
	//need to pass the interrupt number
	stm32_dma_set_interrupt_priority(&config->dma_config.rx, ctl);
	stm32_dma_set_interrupt_priority(&config->dma_config.tx, ctl);
	return uart_local_setaction(handle, ctl);
}

int mcu_uart_dma_put(const devfs_handle_t * handle, void * ctl){
	return uart_local_put(handle, ctl);
}

int mcu_uart_dma_flush(const devfs_handle_t * handle, void * ctl){
	return uart_local_flush(handle, ctl);
}

int mcu_uart_dma_get(const devfs_handle_t * handle, void * ctl){
	return uart_local_get(handle, ctl);
}

int mcu_uart_dma_read(const devfs_handle_t * handle, devfs_async_t * async){
	//read circularly writes to the FIFO, user reads the FIFO
	return uart_local_read(handle, async);
}

int mcu_uart_dma_write(const devfs_handle_t * handle, devfs_async_t * async){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);
	int ret;

	//write won't be circular like read
	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.write, async);

	ret =	HAL_UART_Transmit_DMA(&local->hal_handle, async->buf, async->nbyte);
	if( ret == HAL_OK ){
		return 0;
	}

	local->transfer_handler.write = 0;
	return SYSFS_SET_RETURN(EIO);
}


#endif
