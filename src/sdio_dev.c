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
#include "sdio_local.h"

#if MCU_SDIO_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(sdio, SDIO_VERSION, SDIO_IOC_IDENT_CHAR, I_MCU_TOTAL + I_SDIO_TOTAL, mcu_sdio_getcid, mcu_sdio_getcsd, mcu_sdio_getstatus)

int mcu_sdio_open(const devfs_handle_t * handle){
	return sdio_local_open(handle);
}

int mcu_sdio_close(const devfs_handle_t * handle){
	return sdio_local_close(handle);
}

int mcu_sdio_getinfo(const devfs_handle_t * handle, void * ctl){

	sdio_info_t * info = ctl;
	sdio_local_t * local = sdio_local + handle->port;

	//set flags that are supported by this driver
	info->o_flags = SDIO_FLAG_SET_INTERFACE;
	info->o_events = MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_WRITE_COMPLETE | MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR | MCU_EVENT_FLAG_SET_PRIORITY;

	info->freq = 25000000UL;
	info->block_count = local->hal_handle.SdCard.BlockNbr;
	info->block_size = local->hal_handle.SdCard.BlockSize;
	info->card_class = local->hal_handle.SdCard.Class;
	info->logical_block_count = local->hal_handle.SdCard.LogBlockNbr;
	info->logical_block_size = local->hal_handle.SdCard.LogBlockSize;
	info->type = local->hal_handle.SdCard.CardType;
	info->relative_address = local->hal_handle.SdCard.RelCardAdd;
	info->version = local->hal_handle.SdCard.CardVersion;

	return SYSFS_RETURN_SUCCESS;
}

int mcu_sdio_setattr(const devfs_handle_t * handle, void * ctl){    
	const sdio_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

	u32 o_flags = attr->o_flags;
	sdio_local_t * local = sdio_local + handle->port;

	if( o_flags & SDIO_FLAG_SET_INTERFACE ){

		//SDIO_CLOCK_EDGE_RISING
		//SDIO_CLOCK_EDGE_FALLING
		local->hal_handle.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
		if( o_flags & SDIO_FLAG_IS_CLOCK_FALLING ){
			local->hal_handle.Init.ClockEdge = SDIO_CLOCK_EDGE_FALLING;
		}

		//SDIO_CLOCK_BYPASS_DISABLE
		//SDIO_CLOCK_BYPASS_ENABLE
#if defined SDIO_CLOCK_BYPASS_DISABLE
		local->hal_handle.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
		if( o_flags & SDIO_FLAG_IS_CLOCK_BYPASS_ENABLED ){
			local->hal_handle.Init.ClockBypass = SDIO_CLOCK_BYPASS_ENABLE;
		}
#endif

		//SDIO_CLOCK_POWER_SAVE_DISABLE
		//SDIO_CLOCK_POWER_SAVE_ENABLE
		local->hal_handle.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
		if( o_flags & SDIO_FLAG_IS_CLOCK_POWER_SAVE_ENABLED ){
			local->hal_handle.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_ENABLE;
		}

		//initialize using 1B bus
		local->hal_handle.Init.BusWide = SDIO_BUS_WIDE_1B;


		//SDIO_HARDWARE_FLOW_CONTROL_DISABLE
		//SDIO_HARDWARE_FLOW_CONTROL_ENABLE
		local->hal_handle.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
		if( o_flags & SDIO_FLAG_IS_HARDWARE_FLOW_CONTROL_ENABLED ){
			local->hal_handle.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
		}

		//must be <= 255
		local->hal_handle.Init.ClockDiv = 0;
		if( attr->freq && (attr->freq < 25000000UL) ){
			u32 divider_value = 25000000UL / attr->freq;
			local->hal_handle.Init.ClockDiv = divider_value-1;
		}

		//pin assignments
		if( mcu_set_pin_assignment(
				 &(attr->pin_assignment),
				 MCU_CONFIG_PIN_ASSIGNMENT(sdio_config_t, handle),
				 MCU_PIN_ASSIGNMENT_COUNT(sdio_pin_assignment_t),
				 CORE_PERIPH_SDIO, handle->port, 0, 0, 0) < 0 ){
			return SYSFS_SET_RETURN(EINVAL);
		}

		if( HAL_SD_Init(&local->hal_handle) != HAL_OK ){
			return SYSFS_SET_RETURN(EIO);
		}

		//SDIO_BUS_WIDE_1B -- set as default for initialziation
		//SDIO_BUS_WIDE_4B
		//SDIO_BUS_WIDE_8B -- not compatible with SDIO
		if( o_flags & SDIO_FLAG_IS_BUS_WIDTH_4 ){
			HAL_SD_ConfigWideBusOperation(&local->hal_handle, SDIO_BUS_WIDE_4B);
		}

	}

	if( o_flags & SDIO_FLAG_GET_CARD_STATE ){
		return HAL_SD_GetCardState(&local->hal_handle);
	}

	if( o_flags & SDIO_FLAG_ERASE_BLOCKS ){
		if( HAL_SD_Erase(&local->hal_handle, attr->start, attr->end) != HAL_OK ){
			return SYSFS_SET_RETURN(EIO);
		}
	}


	return SYSFS_RETURN_SUCCESS;
}


int mcu_sdio_setaction(const devfs_handle_t * handle, void * ctl){
	mcu_action_t * action = ctl;
	const u32 port = handle->port;
	sdio_local_t * local = sdio_local + port;

	if( action->handler.callback == 0 ){
		if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
			HAL_SD_Abort_IT(&local->hal_handle);
			devfs_execute_read_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_CANCELED);
		}

		if( action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE ){
			HAL_SD_Abort_IT(&local->hal_handle);
			devfs_execute_write_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_CANCELED);
		}
	}

	cortexm_set_irq_priority(sdio_irqs[port], action->prio, action->o_events);
	return 0;
}

int mcu_sdio_getcid(const devfs_handle_t * handle, void * ctl){
	sdio_local_t * local = sdio_local + handle->port;
	if( HAL_SD_GetCardCID(&local->hal_handle, ctl) == HAL_OK ){
		return SYSFS_RETURN_SUCCESS;
	}

	return SYSFS_SET_RETURN(EIO);
}

int mcu_sdio_getcsd(const devfs_handle_t * handle, void * ctl){
	sdio_local_t * local = sdio_local + handle->port;
	if( HAL_SD_GetCardCSD(&local->hal_handle, ctl) == HAL_OK ){
		return SYSFS_RETURN_SUCCESS;
	}

	return SYSFS_SET_RETURN(EIO);
}

int mcu_sdio_getstatus(const devfs_handle_t * handle, void * ctl){
	sdio_local_t * local = sdio_local + handle->port;
	if( HAL_SD_GetCardStatus(&local->hal_handle, ctl) == HAL_OK ){
		return SYSFS_RETURN_SUCCESS;
	}

	return SYSFS_SET_RETURN(EIO);
}

int mcu_sdio_write(const devfs_handle_t * handle, devfs_async_t * async){
	sdio_local_t * local = sdio_local + handle->port;
	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.write, async);

	if( (HAL_SD_WriteBlocks_IT(&local->hal_handle, async->buf, async->loc, async->nbyte / BLOCKSIZE)) == HAL_OK ){
		return 0;
	}

	local->transfer_handler.write = 0;
	return SYSFS_SET_RETURN(EIO);
}

int mcu_sdio_read(const devfs_handle_t * handle, devfs_async_t * async){
	sdio_local_t * local = sdio_local + handle->port;
	int hal_result;
	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.read, async);

	if( (hal_result = HAL_SD_ReadBlocks_IT(&local->hal_handle, async->buf, async->loc, async->nbyte / BLOCKSIZE)) == HAL_OK ){
		return SYSFS_RETURN_SUCCESS;
	}

	local->transfer_handler.read = 0;
	return SYSFS_SET_RETURN(EIO);
}




#endif

