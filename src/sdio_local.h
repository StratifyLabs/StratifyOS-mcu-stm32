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

#include <mcu/sdio.h>

#include "stm32_dma.h"

#if MCU_SDIO_PORTS > 0

typedef struct {
    SD_HandleTypeDef hal_handle; //must be the first member of the struct
    devfs_transfer_handler_t transfer_handler;
    stm32_dma_channel_t dma_rx_channel;
    stm32_dma_channel_t dma_tx_channel;
    u32 o_flags;
    u8 ref_count;
} sdio_local_t;

extern SDIO_TypeDef * const sdio_regs[MCU_SDIO_PORTS];
extern const int sdio_irqs[MCU_SDIO_PORTS];
static sdio_local_t sdio_local[MCU_SDIO_PORTS] MCU_SYS_MEM;

int sdio_local_open(const devfs_handle_t * handle);
int sdio_local_close(const devfs_handle_t * handle);
int sdio_local_setattr(const devfs_handle_t * handle, void * ctl);
int sdio_local_getinfo(const devfs_handle_t * handle, void * ctl);
int sdio_local_getcid(const devfs_handle_t * handle, void * ctl);
int sdio_local_getcsd(const devfs_handle_t * handle, void * ctl);
int sdio_local_getstatus(const devfs_handle_t * handle, void * ctl);

#endif


#endif /* SPI_LOCAL_H_ */
