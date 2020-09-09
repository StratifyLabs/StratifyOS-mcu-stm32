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

#ifndef SPI_LOCAL_H_
#define SPI_LOCAL_H_

#include <sos/dev/spi.h>

#include "stm32_dma.h"

typedef struct {
	union { //must be first
		SPI_HandleTypeDef hal_handle;
#if MCU_I2S_SPI_PORTS > 0
		I2S_HandleTypeDef i2s_hal_handle;
#endif
	};
	devfs_transfer_handler_t transfer_handler;
	mcu_pin_t ws_pin;
	u16 o_flags;
	u8 ref_count;
	u8 size_mult;
} spi_local_t;

enum {
	SPI_LOCAL_IS_DMA = (1<<0),
	SPI_LOCAL_IS_I2S = (1<<1),
	SPI_LOCAL_IS_FULL_DUPLEX = (1<<2),
	SPI_LOCAL_IS_ERRATA_REQUIRED = (1<<3),
	SPI_LOCAL_IS_ERRATA_I2S = (1<<4)
};


extern spi_local_t m_spi_local[MCU_SPI_PORTS] MCU_SYS_MEM;
extern SPI_TypeDef * const spi_regs[MCU_SPI_PORTS];
extern u8 const spi_irqs[MCU_SPI_PORTS];

int spi_local_open(const devfs_handle_t * handle);
int spi_local_close(const devfs_handle_t * handle);
int spi_local_setattr(const devfs_handle_t * handle, void * ctl);
int spi_local_setaction(const devfs_handle_t * handle, void * ctl);
int spi_local_swap(const devfs_handle_t * handle, void * ctl);



#endif /* SPI_LOCAL_H_ */
