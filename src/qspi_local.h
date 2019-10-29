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

#ifndef QSPI_LOCAL_H_
#define QSPI_LOCAL_H_

#include <sos/dev/qspi.h>

#include "stm32_dma.h"

typedef struct {
	QSPI_HandleTypeDef hal_handle;
	devfs_transfer_handler_t transfer_handler;
	u32 state;
	u32 ref_count;
#if MCU_QSPI_API == 1
	MDMA_HandleTypeDef hmdma;
#endif
} qspi_local_t;


extern qspi_local_t m_qspi_local[MCU_QSPI_PORTS] MCU_SYS_MEM;
extern QUADSPI_TypeDef * const qspi_regs_table[MCU_QSPI_PORTS];
extern u8 const qspi_irqs[MCU_QSPI_PORTS];

int qspi_local_open(const devfs_handle_t * handle);
int qspi_local_close(const devfs_handle_t * handle);
int qspi_local_setattr(const devfs_handle_t * handle, void * ctl);
int qspi_local_setaction(const devfs_handle_t * handle, void * ctl);
int qspi_local_execcommand(const devfs_handle_t * handle, void * ctl);



#endif /* QSPI_LOCAL_H_ */
