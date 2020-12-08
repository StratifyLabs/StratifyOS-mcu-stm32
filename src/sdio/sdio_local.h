/* Copyright 2011-2019 Tyler Gilbert;
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

#ifndef SDIO_LOCAL_H_
#define SDIO_LOCAL_H_

#include <mcu/sdio.h>

#include "stm32_local.h"

#include "stm32_dma.h"

#if MCU_SDIO_PORTS > 0

extern SDIO_TypeDef *const sdio_regs[MCU_SDIO_PORTS];
extern const int sdio_irqs[MCU_SDIO_PORTS];
extern sdio_state_t *m_sdio_state_list[MCU_SDIO_PORTS] MCU_SYS_MEM;

int sdio_local_open(const devfs_handle_t *handle);
int sdio_local_close(const devfs_handle_t *handle);
int sdio_local_setattr(const devfs_handle_t *handle, void *ctl);
int sdio_local_getinfo(const devfs_handle_t *handle, void *ctl);
int sdio_local_getcid(const devfs_handle_t *handle, void *ctl);
int sdio_local_getcsd(const devfs_handle_t *handle, void *ctl);
int sdio_local_getstatus(const devfs_handle_t *handle, void *ctl);

#endif

#endif /* SDIO_LOCAL_H_ */
