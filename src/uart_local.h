/* Copyright 2011-2018 Tyler Gilbert;
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

#ifndef UART_LOCAL_H_
#define UART_LOCAL_H_

#include <mcu/uart.h>
#include <device/fifo.h>

#include "stm32_dma.h"

#if MCU_UART_PORTS > 0

enum {
	UART_LOCAL_IS_DMA = (1<<0)
};

typedef struct {
	UART_HandleTypeDef hal_handle;
	devfs_transfer_handler_t transfer_handler;
	fifo_state_t fifo_state;
	const fifo_config_t * fifo_config;
	u16 bytes_received;
	u8 ref_count;
	u8 o_flags;
} uart_local_t;

extern uart_local_t m_uart_local[MCU_UART_PORTS] MCU_SYS_MEM;
extern USART_TypeDef * const uart_regs_table[MCU_UART_PORTS];
extern u8 const uart_irqs[MCU_UART_PORTS];

int uart_local_open(const devfs_handle_t * handle);
int uart_local_close(const devfs_handle_t * handle);
int uart_local_setattr(const devfs_handle_t * handle, void * ctl);
int uart_local_setaction(const devfs_handle_t * handle, void * ctl);
int uart_local_getinfo(const devfs_handle_t * handle, void * ctl);
int uart_local_get(const devfs_handle_t * handle, void * ctl);
int uart_local_put(const devfs_handle_t * handle, void * ctl);
int uart_local_flush(const devfs_handle_t * handle, void * ctl);
int uart_local_read(const devfs_handle_t * handle, devfs_async_t * async);


#endif


#endif /* UART_LOCAL_H_ */
