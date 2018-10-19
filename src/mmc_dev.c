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

#include <mcu/sdio.h>
#include "mmc_local.h"

#if MCU_SDIO_PORTS > 0 && defined MMC_TypeDef


DEVFS_MCU_DRIVER_IOCTL_FUNCTION(sdio, MMC_VERSION, SDIO_IOC_IDENT_CHAR, I_MCU_TOTAL + I_SDIO_TOTAL, mcu_mmc_getcid, mcu_mmc_getcsd, mcu_mmc_getstatus)

int mcu_mmc_open(const devfs_handle_t * handle){
	return mmc_local_open(handle);
}

int mcu_mmc_close(const devfs_handle_t * handle){
	return mmc_local_close(handle);
}

int mcu_mmc_getinfo(const devfs_handle_t * handle, void * ctl){
	return mmc_local_getinfo(handle, ctl);
}

int mcu_mmc_setattr(const devfs_handle_t * handle, void * ctl){
	return mmc_local_setattr(handle, ctl);
}


int mcu_mmc_setaction(const devfs_handle_t * handle, void * ctl){
	mcu_action_t * action = ctl;
	u32 port = handle->port;
	mmc_local_t * local = mmc_local + handle->port;

	if( action->handler.callback == 0 ){
		if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
			HAL_MMC_Abort_IT(&local->hal_handle);
			devfs_execute_read_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_CANCELED);
		}

		if( action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE ){
			HAL_MMC_Abort_IT(&local->hal_handle);
			devfs_execute_write_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_CANCELED);
		}
	}

	cortexm_set_irq_priority(mmc_irqs[port], action->prio, action->o_events);
	return 0;
}

int mcu_mmc_getcid(const devfs_handle_t * handle, void * ctl){
	return mmc_local_getcid(handle, ctl);
}

int mcu_mmc_getcsd(const devfs_handle_t * handle, void * ctl){
	return mmc_local_getcsd(handle, ctl);
}

int mcu_mmc_getstatus(const devfs_handle_t * handle, void * ctl){
	return mmc_local_getstatus(handle, ctl);
}

int mcu_mmc_write(const devfs_handle_t * handle, devfs_async_t * async){
	mmc_local_t * local = mmc_local + handle->port;

	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.write, async);

	int loc;
	if( local->o_flags & MMC_LOCAL_FLAG_IS_BYTE_ADDRESSING ){
		loc = async->loc*512;
	} else {
		loc = async->loc;
	}
	if( (HAL_MMC_WriteBlocks_IT(&local->hal_handle, async->buf, loc, async->nbyte / BLOCKSIZE)) == HAL_OK ){
		return 0;
	}

	local->transfer_handler.write = 0;
	return SYSFS_SET_RETURN(EIO);
}

int mcu_mmc_read(const devfs_handle_t * handle, devfs_async_t * async){
	mmc_local_t * local = mmc_local + handle->port;
	int hal_result;
	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.read, async);

	int loc;
	if( local->o_flags & MMC_LOCAL_FLAG_IS_BYTE_ADDRESSING ){
		loc = async->loc*512;
	} else {
		loc = async->loc;
	}
	if( (hal_result = HAL_MMC_ReadBlocks_IT(&local->hal_handle, async->buf, loc, async->nbyte / BLOCKSIZE)) == HAL_OK ){
		return 0;
	}

	local->transfer_handler.read = 0;
	return SYSFS_SET_RETURN(EIO);
}




#endif

