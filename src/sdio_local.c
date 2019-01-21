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

#include "sdio_local.h"

#if MCU_SDIO_PORTS > 0

SDIO_TypeDef * const sdio_regs[MCU_SDIO_PORTS] = MCU_SDIO_REGS;
const int sdio_irqs[MCU_SDIO_PORTS] = MCU_SDIO_IRQS;
sdio_local_t sdio_local[MCU_SDIO_PORTS] MCU_SYS_MEM;

int sdio_local_open(const devfs_handle_t * handle){
	const u32 port = handle->port;
	sdio_local_t * local = sdio_local + port;
	if( port < MCU_SDIO_PORTS ){
		if ( local->ref_count == 0 ){
			//turn on RCC clock
			switch(port){
				case 0:
					__HAL_RCC_SDIO_CLK_ENABLE();
					break;
			}
			local->transfer_handler.read = NULL;
			local->transfer_handler.write = NULL;
			local->hal_handle.Instance = sdio_regs[port];
			cortexm_enable_irq(sdio_irqs[port]);
		}
		local->ref_count++;
	}
	return 0;
}

int sdio_local_close(const devfs_handle_t * handle){

	//do the opposite of sdio_local_open() -- ref_count is zero -- turn off interrupt
	return 0;
}

int sdio_local_getinfo(const devfs_handle_t * handle, void * ctl){
	sdio_info_t * info = ctl;
	//const u32 port = handle->port;
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

int sdio_local_setattr(const devfs_handle_t * handle, void * ctl){
	sdio_local_t * local = sdio_local + handle->port;
	const sdio_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

	u32 o_flags = attr->o_flags;

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


int sdio_local_setaction(const devfs_handle_t * handle, void * ctl){
	mcu_action_t * action = ctl;
	u32 port = handle->port;
	sdio_local_t * local = sdio_local + handle->port;

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

int sdio_local_getcid(const devfs_handle_t * handle, void * ctl){
	sdio_local_t * local = sdio_local + handle->port;
	if( HAL_SD_GetCardCID(&local->hal_handle, ctl) == HAL_OK ){
		return SYSFS_RETURN_SUCCESS;
	}

	return SYSFS_SET_RETURN(EIO);
}

int sdio_local_getcsd(const devfs_handle_t * handle, void * ctl){
	sdio_local_t * local = sdio_local + handle->port;
	if( HAL_SD_GetCardCSD(&local->hal_handle, ctl) == HAL_OK ){
		return SYSFS_RETURN_SUCCESS;
	}

	return SYSFS_SET_RETURN(EIO);
}

int sdio_local_getstatus(const devfs_handle_t * handle, void * ctl){
	sdio_local_t * local = sdio_local + handle->port;
	if( HAL_SD_GetCardStatus(&local->hal_handle, ctl) == HAL_OK ){
		return SYSFS_RETURN_SUCCESS;
	}
	return SYSFS_SET_RETURN(EIO);
}


void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd){
	sdio_local_t * local = (sdio_local_t *)hsd;
	//mcu_debug_root_printf("w:%ld\n", TIM2->CNT - local->start_time);
	devfs_execute_write_handler(&local->transfer_handler, 0, hsd->TxXferSize, MCU_EVENT_FLAG_WRITE_COMPLETE);
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd){
	sdio_local_t * local = (sdio_local_t *)hsd;
	//mcu_debug_root_printf("read complete %d 0x%lX %ld\n", hsd->RxXferSize, hsd->Instance->STA, TIM2->CNT - local->start_time);
	devfs_execute_read_handler(&local->transfer_handler, 0, hsd->RxXferSize, MCU_EVENT_FLAG_DATA_READY);
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd){
	sdio_local_t * local = (sdio_local_t *)hsd;
	//mcu_debug_log_warning(MCU_DEBUG_DEVICE, "SD Error? 0x%lX 0x%lX %ld", hsd->ErrorCode, hsd->hdmatx->ErrorCode, TIM2->CNT - local->start_time);
	if( hsd->ErrorCode ){
		mcu_debug_log_error(MCU_DEBUG_DEVICE, "SD Error 0x%lX", hsd->ErrorCode);
		devfs_execute_cancel_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_ERROR);
	}
}

void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd){
	sdio_local_t * local = (sdio_local_t *)hsd;
	//abort read and write
	mcu_debug_log_warning(MCU_DEBUG_DEVICE, "Abort\n");
	devfs_execute_cancel_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EIO), 0);
}

void mcu_core_sdio_isr(){
	//mcu_debug_log_info(MCU_DEBUG_DEVICE, "SDIO IRQ 0x%lX", sd_handle[0]->Instance->STA);
	HAL_SD_IRQHandler(&sdio_local[0].hal_handle);
}




#endif

