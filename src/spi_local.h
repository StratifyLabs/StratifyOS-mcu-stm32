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
    u16 o_flags;
    u8 ref_count;
    u8 size_mult;
} spi_local_t;

enum {
    SPI_LOCAL_IS_DMA = (1<<0),
    SPI_LOCAL_IS_I2S = (1<<1),
    SPI_LOCAL_IS_FULL_DUPLEX = (1<<2)
};

typedef struct {
    spi_local_t spi;
    u32 burst_size;
    stm32_dma_channel_t dma_rx_channel;
    stm32_dma_channel_t dma_tx_channel;
} spi_dma_local_t;

extern spi_local_t spi_local[MCU_SPI_PORTS] MCU_SYS_MEM;
extern spi_dma_local_t spi_dma_local[MCU_SPI_PORTS] MCU_SYS_MEM;
extern SPI_TypeDef * const spi_regs[MCU_SPI_PORTS];
extern u8 const spi_irqs[MCU_SPI_PORTS];

int spi_local_open(spi_local_t * spi, const devfs_handle_t * handle);
int spi_local_close(spi_local_t * spi, const devfs_handle_t * handle);
int spi_local_setattr(spi_local_t * spi, const devfs_handle_t * handle, void * ctl);
int spi_local_setaction(spi_local_t * spi, const devfs_handle_t * handle, void * ctl);
int spi_local_swap(spi_local_t * spi, const devfs_handle_t * handle, void * ctl);
int spi_local_execute_handler(devfs_async_t ** async, u32 o_events, u32 value, int ret);



#endif /* SPI_LOCAL_H_ */
