/* Copyright 2011-2016 Tyler Gilbert;
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

#include <core_startup.h>
#include "stm32_local.h"

extern u32 _unique_id;

static const char sys_proc_name[] = "sys";


void mcu_core_nmi_isr() MCU_WEAK;
int pthread_mutex_init(pthread_mutex_t *mutex, const pthread_mutexattr_t *attr) MCU_WEAK;


void mcu_core_getserialno(mcu_sn_t * serial_number){
    const u32 * serial_addr = &_unique_id;
	serial_number->sn[0] = serial_addr[0];
	serial_number->sn[1] = serial_addr[1];
	serial_number->sn[2] = serial_addr[2];
	serial_number->sn[3] = 0;
}


void core_init(){
	u32 *src, *dest;
	src = &_etext; //point src to copy of data that is stored in flash
	for(dest = &_data; dest < &_edata; ){ *dest++ = *src++; } //Copy from flash to RAM (data)
	for(src = &_bss; src < &_ebss; ) *src++ = 0; //Zero out BSS section
	for(src = &_sys; src < &_esys; ) *src++ = 0; //Zero out sysmem

	//Re-entrancy initialization
	//If the program faults on the next line, make sure the etext and data are aligned properly in the linker script (4 byte boundary)
	_REENT->procmem_base = (proc_mem_t*)&_ebss;
	_REENT->procmem_base->size = 0;
	_REENT->procmem_base->sigactions = NULL;
	_REENT->procmem_base->siginfos = NULL;
	_REENT->procmem_base->proc_name = sys_proc_name;

	memset(_REENT->procmem_base->open_file, 0, sizeof(open_file_t)*OPEN_MAX);

	//Initialize the global mutexes
	__lock_init_recursive_global(__malloc_lock_object);
	_REENT->procmem_base->__malloc_lock_object.flags |= PTHREAD_MUTEX_FLAGS_PSHARED; //Make the malloc lock pshared

	__lock_init_global(__tz_lock_object);
	__lock_init_recursive_global(__atexit_lock);
	__lock_init_recursive_global(__sfp_lock);
	__lock_init_recursive_global(__sinit_lock);
	__lock_init_recursive_global(__env_lock_object);

	//This is the de facto MCU initialization -- turn off power to peripherals that must be "open()"ed.

	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOC_CLK_ENABLE();
#if defined GPIOD
	  __HAL_RCC_GPIOD_CLK_ENABLE();
#endif
#if defined GPIOE
	  __HAL_RCC_GPIOE_CLK_ENABLE();
#endif
#if defined GPIOF
	  __HAL_RCC_GPIOF_CLK_ENABLE();
#endif
#if defined GPIOG
	  __HAL_RCC_GPIOG_CLK_ENABLE();
#endif
#if defined GPIOH
	  __HAL_RCC_GPIOH_CLK_ENABLE();
#endif
#if defined GPIOI
      __HAL_RCC_GPIOI_CLK_ENABLE();
#endif
#if defined GPIOJ
      __HAL_RCC_GPIOJ_CLK_ENABLE();
#endif
#if defined GPIOK
      __HAL_RCC_GPIOK_CLK_ENABLE();
#endif
}


void mcu_core_nmi_isr(){
	mcu_board_execute_event_handler(MCU_BOARD_CONFIG_EVENT_ROOT_FATAL, 0);
}


int pthread_mutex_init(pthread_mutex_t *mutex, const pthread_mutexattr_t *attr){
	return 0;
}
