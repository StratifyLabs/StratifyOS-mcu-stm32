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

#include <fcntl.h>
#include <cortexm/cortexm.h>
#include <mcu/rng.h>
#include <mcu/debug.h>

#include "stm32_local.h"

#if MCU_RNG_PORTS > 0

typedef struct {
	RNG_HandleTypeDef hal_handle;
	devfs_transfer_handler_t transfer_handler;
	u32 bytes_read;
	u8 ref_count;
} rng_local_t;

static rng_local_t rng_local[MCU_RNG_PORTS] MCU_SYS_MEM;
RNG_TypeDef * const rng_regs_table[MCU_RNG_PORTS] = MCU_RNG_REGS;
s8 const rng_irqs[MCU_RNG_PORTS] = MCU_RNG_IRQS;

DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(rng, RANDOM_VERSION, RANDOM_IOC_CHAR)

int mcu_rng_open(const devfs_handle_t * handle){
	int port = handle->port;
	if ( rng_local[port].ref_count == 0 ){
		rng_local[port].hal_handle.Instance = rng_regs_table[port];
		switch(port){
			case 0:
				__HAL_RCC_RNG_CLK_ENABLE();
				break;
		}
		cortexm_enable_irq(rng_irqs[port]);
	}
	rng_local[port].ref_count++;
	return 0;
}

int mcu_rng_close(const devfs_handle_t * handle){
	int port = handle->port;
	if ( rng_local[port].ref_count > 0 ){
		if ( rng_local[port].ref_count == 1 ){
			cortexm_disable_irq(rng_irqs[port]);
			switch(port){
				case 0:
					__HAL_RCC_RNG_CLK_DISABLE();
					break;
			}
			rng_local[port].hal_handle.Instance = 0;
		}
		rng_local[port].ref_count--;
	}
	return 0;
}

int mcu_rng_getinfo(const devfs_handle_t * handle, void * ctl){
	random_info_t * info = ctl;
	memset(info, 0, sizeof(random_info_t));
	info->o_flags = RANDOM_FLAG_IS_TRUE | RANDOM_FLAG_ENABLE | RANDOM_FLAG_DISABLE;
	return 0;
}

int mcu_rng_setattr(const devfs_handle_t * handle, void * ctl){
	rng_local_t * rng = rng_local + handle->port;
	const random_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

	u32 o_flags = attr->o_flags;

	if( o_flags & RANDOM_FLAG_ENABLE ){
		if( HAL_RNG_Init(&rng->hal_handle) != HAL_OK ){
			return SYSFS_SET_RETURN(EIO);
		}
	}

	if( o_flags & RANDOM_FLAG_DISABLE ){
		if( HAL_RNG_DeInit(&rng->hal_handle) != HAL_OK ){
			return SYSFS_SET_RETURN(EIO);
		}
	}

	return SYSFS_RETURN_SUCCESS;
}


int mcu_rng_setaction(const devfs_handle_t * handle, void * ctl){
	return 0;
}

int mcu_rng_read(const devfs_handle_t * handle, devfs_async_t * async){
	const int port = handle->port;
	DEVFS_DRIVER_IS_BUSY(rng_local[port].transfer_handler.read, async);

	rng_local[port].bytes_read = 0;

	if( HAL_RNG_GenerateRandomNumber_IT(&rng_local[port].hal_handle) != HAL_OK ){
		rng_local[port].transfer_handler.read = 0;
		return SYSFS_SET_RETURN(EIO);
	}

	return SYSFS_RETURN_SUCCESS;
}

int mcu_rng_write(const devfs_handle_t * handle, devfs_async_t * async){
	return SYSFS_SET_RETURN(ENOSYS);
}

void HAL_RNG_ErrorCallback(RNG_HandleTypeDef *hrng){
	rng_local_t * rng = (rng_local_t *)hrng;
	devfs_execute_read_handler(&rng->transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_ERROR | MCU_EVENT_FLAG_CANCELED);
}

void HAL_RNG_ReadyDataCallback(RNG_HandleTypeDef* hrng, uint32_t random32bit){
	rng_local_t * rng = (rng_local_t *)hrng;

	memcpy(rng->transfer_handler.read->buf + rng->bytes_read, &random32bit, sizeof(uint32_t));
	rng->bytes_read += sizeof(uint32_t);

	if( rng->bytes_read == rng->transfer_handler.read->nbyte ){
		devfs_execute_read_handler(&rng->transfer_handler, 0, 0, MCU_EVENT_FLAG_DATA_READY);
	} else {
		HAL_RNG_GenerateRandomNumber_IT(hrng);
	}
}

void mcu_core_hash_isr(){
	HAL_RNG_IRQHandler(&rng_local[0].hal_handle);
	//HAL_HASH_IRQHandler() //to be implemented
}


#endif
