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

#ifndef CRYPT_LOCAL_H_
#define CRYPT_LOCAL_H_

#include <mcu/hash.h>

#include "stm32_local.h"

#if MCU_HASH_PORTS > 0

enum { HASH_LOCAL_IS_DMA = (1 << 0) };

extern hash_state_t *m_hash_state_list[MCU_HASH_PORTS] MCU_SYS_MEM;
extern HASH_TypeDef * const hash_regs[MCU_HASH_PORTS];
extern u8 const hash_irqs[MCU_HASH_PORTS];

int hash_local_open(const devfs_handle_t * handle);
int hash_local_close(const devfs_handle_t * handle);
int hash_local_setattr(const devfs_handle_t * handle, void * ctl);
int hash_local_setaction(const devfs_handle_t * handle, void * ctl);

#endif


#endif /* CRYPT_LOCAL_H_ */
