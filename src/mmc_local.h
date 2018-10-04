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

#include <mcu/mmc.h>

#include "stm32_dma.h"

#if MCU_SDIO_PORTS > 0 && defined MMC_TypeDef

typedef struct {
    MMC_HandleTypeDef hal_handle; //must be the first member of the struct
    devfs_transfer_handler_t transfer_handler;
    stm32_dma_channel_t dma_rx_channel;
    stm32_dma_channel_t dma_tx_channel;
    u32 o_flags;
    u8 ref_count;
} mmc_local_t;

enum {
	EMMC_LOCAL_FLAG_IS_BYTE_ADDRESSING = (1<<0)
};

extern SDIO_TypeDef * const mmc_regs[MCU_SDIO_PORTS];
extern const int mmc_irqs[MCU_SDIO_PORTS];
extern mmc_local_t mmc_local[MCU_SDIO_PORTS] MCU_SYS_MEM;

int mmc_local_open(const devfs_handle_t * handle);
int mmc_local_close(const devfs_handle_t * handle);
int mmc_local_setattr(const devfs_handle_t * handle, void * ctl);
int mmc_local_getinfo(const devfs_handle_t * handle, void * ctl);
int mmc_local_getcid(const devfs_handle_t * handle, void * ctl);
int mmc_local_getcsd(const devfs_handle_t * handle, void * ctl);
int mmc_local_getstatus(const devfs_handle_t * handle, void * ctl);

#endif


#endif /* SPI_LOCAL_H_ */
