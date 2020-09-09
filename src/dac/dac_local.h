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

#ifndef DAC_LOCAL_H_
#define DAC_LOCAL_H_

#include <mcu/dac.h>
#include "stm32_dma.h"

#if MCU_DAC_PORTS > 0

#define DAC_LOCAL_FLAG_IS_DMA (1<<0)

typedef struct {
	DAC_HandleTypeDef hal_handle;
	devfs_transfer_handler_t transfer_handler;
	int words_written;
	u32 o_flags;
	u8 ref_count;
	stm32_dma_channel_t dma_tx_channel;
} dac_local_t;

extern DAC_TypeDef * const m_dac_regs_table[MCU_DAC_PORTS];
extern u8 const m_dac_irqs[MCU_DAC_PORTS];
extern const u32 m_dac_channels[MCU_DAC_CHANNELS];
extern dac_local_t m_dac_local[MCU_DAC_PORTS] MCU_SYS_MEM;

int dac_local_open(const devfs_handle_t * handle);
int dac_local_close(const devfs_handle_t * handle);
int dac_local_setattr(const devfs_handle_t * handle, void * ctl);
int dac_local_getinfo(const devfs_handle_t * handle, void * ctl);

int dac_local_set(const devfs_handle_t * handle, void * ctl);
int dac_local_get(const devfs_handle_t * handle, void * ctl);

u32 dac_local_get_alignment(dac_local_t * dac);


#endif

#endif /* DAC_LOCAL_H_ */
