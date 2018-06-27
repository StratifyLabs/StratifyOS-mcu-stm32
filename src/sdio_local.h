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
    SD_HandleTypeDef hal_handle; //must be the first member of the struct
    devfs_transfer_handler_t transfer_handler;
    u32 start_time;
    u8 ref_count;
} sdio_local_t;

typedef struct {
    sdio_local_t sdio;
    stm32_dma_channel_t dma_rx_channel;
    stm32_dma_channel_t dma_tx_channel;
} sdio_dma_local_t;

extern SDIO_TypeDef * const sdio_regs[MCU_SDIO_PORTS];
extern const int sdio_irqs[MCU_SDIO_PORTS];

int sdio_local_open(sdio_local_t * sdio, const devfs_handle_t * handle);
int sdio_local_close(sdio_local_t * sdio, const devfs_handle_t * handle);
int sdio_local_setattr(sdio_local_t * sdio, const devfs_handle_t * handle, void * ctl);
int sdio_local_getcid(sdio_local_t * sdio, const devfs_handle_t * handle, void * ctl);
int sdio_local_getcsd(sdio_local_t * sdio, const devfs_handle_t * handle, void * ctl);
int sdio_local_getstatus(sdio_local_t * sdio, const devfs_handle_t * handle, void * ctl);



#endif /* SPI_LOCAL_H_ */
