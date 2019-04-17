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
#include <mcu/pio.h>
#include <mcu/debug.h>

#include "uart_local.h"

#if MCU_UART_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(uart, UART_VERSION, UART_IOC_IDENT_CHAR, I_MCU_TOTAL + I_UART_TOTAL, mcu_uart_get, mcu_uart_put, mcu_uart_flush)

int mcu_uart_open(const devfs_handle_t * handle){
	return uart_local_open(handle);
}

int mcu_uart_close(const devfs_handle_t * handle){
	return uart_local_close(handle);
}

int mcu_uart_dev_is_powered(const devfs_handle_t * handle){
	return ( m_uart_local[handle->port].ref_count != 0 );
}

int mcu_uart_getinfo(const devfs_handle_t * handle, void * ctl){
	return uart_local_getinfo(handle, ctl);
}

int mcu_uart_setattr(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);

	int result =  uart_local_setattr(handle, ctl);
	if( result < 0 ){ return result; }

	if( local->fifo_config != 0 ){
		result = HAL_UART_Receive_IT(&local->hal_handle,
											  (u8*)local->fifo_config->buffer,
											  local->fifo_config->size);
		if( result != HAL_OK ){ return SYSFS_SET_RETURN(EIO); }
	}

	return SYSFS_RETURN_SUCCESS;
}

int mcu_uart_setaction(const devfs_handle_t * handle, void * ctl){
	return uart_local_setaction(handle, ctl);
}

int mcu_uart_put(const devfs_handle_t * handle, void * ctl){
	return uart_local_put(handle, ctl);
}

int mcu_uart_flush(const devfs_handle_t * handle, void * ctl){
	return uart_local_flush(handle, ctl);
}


int mcu_uart_get(const devfs_handle_t * handle, void * ctl){
	return uart_local_get(handle, ctl);
}

int mcu_uart_read(const devfs_handle_t * handle, devfs_async_t * async){
	return uart_local_read(handle, async);
}

int mcu_uart_write(const devfs_handle_t * handle, devfs_async_t * async){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);
	int result;

	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.write, async);

	result =	HAL_UART_Transmit_IT(&local->hal_handle, async->buf, async->nbyte);
	if( result == HAL_OK ){
		return 0;
	}

	local->transfer_handler.write = 0;
	return SYSFS_SET_RETURN(EIO);
}

#endif
