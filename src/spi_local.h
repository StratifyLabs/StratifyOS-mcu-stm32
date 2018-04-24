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

typedef struct MCU_PACK {
    union { //must be first
        SPI_HandleTypeDef hal_handle;
        I2S_HandleTypeDef i2s_hal_handle;
    };
    devfs_async_t * write;
    devfs_async_t * read;
    u8 is_full_duplex;
    u8 is_i2s;
    u8 ref_count;
    u8 resd;
} spi_local_t;

typedef struct {
    spi_local_t spi;
    stm32_dma_channel_t dma_rx_channel;
    stm32_dma_channel_t dma_tx_channel;
} spi_dma_local_t;

extern spi_local_t spi_local[MCU_SPI_PORTS] MCU_SYS_MEM;
extern spi_dma_local_t spi_dma_local[MCU_SPI_PORTS] MCU_SYS_MEM;
extern SPI_TypeDef * const spi_regs[MCU_SPI_PORTS];
extern u8 const spi_irqs[MCU_SPI_PORTS];

SPI_TypeDef * spi_local_open(int port);
void spi_local_close(int port);
int spi_local_setattr(const devfs_handle_t * handle, void * ctl, spi_local_t * spi);
int spi_local_setaction(const devfs_handle_t * handle, void * ctl, spi_local_t * spi, int interrupt_number);
int spi_local_swap(const devfs_handle_t * handle, void * ctl, spi_local_t * spi);
int spi_local_execute_handler(devfs_async_t ** async, u32 o_events, u32 value, int ret);



#endif /* SPI_LOCAL_H_ */
