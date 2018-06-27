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


SDIO_TypeDef * const sdio_regs[MCU_SDIO_PORTS] = MCU_SDIO_REGS;
const int sdio_irqs[MCU_SDIO_PORTS] = MCU_SDIO_IRQS;

int sdio_local_open(sdio_local_t * sdio, const devfs_handle_t * handle){
    int port = handle->port;
    if( port < MCU_SDIO_PORTS ){
        if ( sdio->ref_count == 0 ){
            //turn on RCC clock
            switch(port){
            case 0:
                __HAL_RCC_SDIO_CLK_ENABLE();
                break;
            }
            sdio->transfer_handler.read = NULL;
            sdio->transfer_handler.write = NULL;
            sdio->hal_handle.Instance = sdio_regs[port];
            cortexm_enable_irq(sdio_irqs[port]);
        }
        sdio->ref_count++;
    }
    return 0;
}

int sdio_local_close(sdio_local_t * sdio, const devfs_handle_t * handle){

    //do the opposite of sdio_local_open() -- ref_count is zero -- turn off interrupt
    return 0;
}

int sdio_local_getinfo(sdio_local_t * sdio, const devfs_handle_t * handle, void * ctl){
    sdio_info_t * info = ctl;
    const u32 port = handle->port;

    //set flags that are supported by this driver
    info->o_flags = SDIO_FLAG_SET_INTERFACE;
    info->o_events = MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_WRITE_COMPLETE | MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR | MCU_EVENT_FLAG_SET_PRIORITY;

    info->freq = 25000000UL;
    info->block_count = sdio->hal_handle.SdCard.BlockNbr;
    info->block_size = sdio->hal_handle.SdCard.BlockSize;
    info->card_class = sdio->hal_handle.SdCard.Class;
    info->logical_block_count = sdio->hal_handle.SdCard.LogBlockNbr;
    info->logical_block_size = sdio->hal_handle.SdCard.LogBlockSize;
    info->type = sdio->hal_handle.SdCard.CardType;
    info->relative_address = sdio->hal_handle.SdCard.RelCardAdd;
    info->version = sdio->hal_handle.SdCard.CardVersion;

    return SYSFS_RETURN_SUCCESS;
}

int sdio_local_setattr(sdio_local_t * sdio, const devfs_handle_t * handle, void * ctl){

    const sdio_attr_t * attr = mcu_select_attr(handle, ctl);
    if( attr == 0 ){ return SYSFS_SET_RETURN(EINVAL); }

    u32 o_flags = attr->o_flags;

    if( o_flags & SDIO_FLAG_GET_CARD_STATE ){
        return HAL_SD_GetCardState(&sdio->hal_handle);
    }

    if( o_flags & SDIO_FLAG_ERASE_BLOCKS ){
        if( HAL_SD_Erase(&sdio->hal_handle, attr->start, attr->end) != HAL_OK ){
            return SYSFS_SET_RETURN(EIO);
        }
    }


    return SYSFS_RETURN_SUCCESS;
}


int sdio_local_setaction(sdio_local_t * sdio, const devfs_handle_t * handle, void * ctl){
    mcu_action_t * action = ctl;
    u32 port = handle->port;

    if( action->handler.callback == 0 ){
        if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
            HAL_SD_Abort_IT(&sdio->hal_handle);
            mcu_execute_read_handler_with_flags(&sdio->transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_CANCELED);
        }

        if( action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE ){
            HAL_SD_Abort_IT(&sdio->hal_handle);
            mcu_execute_write_handler_with_flags(&sdio->transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_CANCELED);
        }
    }

    cortexm_set_irq_priority(sdio_irqs[port], action->prio, action->o_events);
    return 0;
}

int sdio_local_getcid(sdio_local_t * sdio, const devfs_handle_t * handle, void * ctl){
    if( HAL_SD_GetCardCID(&sdio->hal_handle, ctl) == HAL_OK ){
        return SYSFS_RETURN_SUCCESS;
    }

    return SYSFS_SET_RETURN(EIO);
}

int sdio_local_getcsd(sdio_local_t * sdio, const devfs_handle_t * handle, void * ctl){
    if( HAL_SD_GetCardCSD(&sdio->hal_handle, ctl) == HAL_OK ){
        return SYSFS_RETURN_SUCCESS;
    }

    return SYSFS_SET_RETURN(EIO);
}

int sdio_local_getstatus(sdio_local_t * sdio, const devfs_handle_t * handle, void * ctl){
    u32 port = handle->port;
    if( HAL_SD_GetCardStatus(&sdio->hal_handle, ctl) == HAL_OK ){
        return SYSFS_RETURN_SUCCESS;
    }

    return SYSFS_SET_RETURN(EIO);
}


void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd){
    sdio_local_t * sdio = (sdio_local_t *)hsd;
    //mcu_debug_root_printf("w:%ld\n", TIM2->CNT - sdio->start_time);
    mcu_execute_write_handler(&sdio->transfer_handler, 0, hsd->TxXferSize);
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd){
    sdio_local_t * sdio = (sdio_local_t *)hsd;
    //mcu_debug_root_printf("r:%ld\n", TIM2->CNT - sdio->start_time);
    mcu_execute_read_handler(&sdio->transfer_handler, 0, hsd->RxXferSize);
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd){
    sdio_local_t * sdio = (sdio_local_t *)hsd;
    mcu_debug_root_printf("SD Error 0x%lX\n", hsd->ErrorCode);
    mcu_execute_transfer_handlers(&sdio->transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR);
}

void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd){
    sdio_local_t * sdio = (sdio_local_t *)hsd;
    //abort read and write
    mcu_debug_root_printf("Abort\n");
    mcu_execute_transfer_handlers(&sdio->transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_CANCELED);
}




#endif

