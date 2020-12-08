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

#include "stm32_local.h"
#include "stm32_dma.h"

#if MCU_QSPI_PORTS > 0
extern qspi_state_t *m_qspi_state_list[MCU_QSPI_PORTS] MCU_SYS_MEM;
extern QUADSPI_TypeDef *const qspi_regs_table[MCU_QSPI_PORTS];
extern u8 const qspi_irqs[MCU_QSPI_PORTS];

int qspi_local_open(const devfs_handle_t *handle);
int qspi_local_close(const devfs_handle_t *handle);
int qspi_local_setattr(const devfs_handle_t *handle, void *ctl);
int qspi_local_setaction(const devfs_handle_t *handle, void *ctl);
int qspi_local_execcommand(const devfs_handle_t *handle, void *ctl);
#endif

#endif /* QSPI_LOCAL_H_ */
