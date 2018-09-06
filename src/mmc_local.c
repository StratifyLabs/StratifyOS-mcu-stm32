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

#include "mmc_local.h"

#if MCU_SDIO_PORTS > 0 && defined MMC_TypeDef

SDIO_TypeDef * const mmc_regs[MCU_SDIO_PORTS] = MCU_SDIO_REGS;
const int mmc_irqs[MCU_SDIO_PORTS] = MCU_SDIO_IRQS;

int mmc_local_open(const devfs_handle_t * handle){
    int port = handle->port;
    mmc_local_t * local = mmc_local + handle->port;
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
            local->hal_handle.Instance = mmc_regs[port];
            cortexm_enable_irq(mmc_irqs[port]);
        }
        local->ref_count++;
    }
    return 0;
}

int mmc_local_close(const devfs_handle_t * handle){

    //do the opposite of mmc_local_open() -- ref_count is zero -- turn off interrupt
    return 0;
}

int mmc_local_getinfo(const devfs_handle_t * handle, void * ctl){
    mmc_info_t * info = ctl;
    mmc_local_t * local = mmc_local + handle->port;
    //const u32 port = handle->port;

    //set flags that are supported by this driver
    info->o_flags = MMC_FLAG_SET_INTERFACE;
    info->o_events = MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_WRITE_COMPLETE | MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR | MCU_EVENT_FLAG_SET_PRIORITY;

    info->freq = 25000000UL;
    info->block_count = local->hal_handle.MmcCard.BlockNbr;
    info->block_size = local->hal_handle.MmcCard.BlockSize;
    info->card_class = local->hal_handle.MmcCard.Class;
    info->logical_block_count = local->hal_handle.MmcCard.LogBlockNbr;
    info->logical_block_size = local->hal_handle.MmcCard.LogBlockSize;
    info->type = local->hal_handle.MmcCard.CardType;
    info->relative_address = local->hal_handle.MmcCard.RelCardAdd;
    info->version = 0;

    return SYSFS_RETURN_SUCCESS;
}

int mmc_local_setattr(const devfs_handle_t * handle, void * ctl){

    const mmc_attr_t * attr = mcu_select_attr(handle, ctl);
    if( attr == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }
    mmc_local_t * local = mmc_local + handle->port;

    u32 o_flags = attr->o_flags;

    if( o_flags & MMC_FLAG_SET_INTERFACE ){

        //SDIO_CLOCK_EDGE_RISING
        //SDIO_CLOCK_EDGE_FALLING
        local->hal_handle.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
        if( o_flags & MMC_FLAG_IS_CLOCK_FALLING ){
            local->hal_handle.Init.ClockEdge = SDIO_CLOCK_EDGE_FALLING;
        }

        //SDIO_CLOCK_BYPASS_DISABLE
        //SDIO_CLOCK_BYPASS_ENABLE
        local->hal_handle.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
        if( o_flags & MMC_FLAG_IS_CLOCK_BYPASS_ENABLED ){
            local->hal_handle.Init.ClockBypass = SDIO_CLOCK_BYPASS_ENABLE;
        }

        //SDIO_CLOCK_POWER_SAVE_DISABLE
        //SDIO_CLOCK_POWER_SAVE_ENABLE
        local->hal_handle.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
        if( o_flags & MMC_FLAG_IS_CLOCK_POWER_SAVE_ENABLED ){
            local->hal_handle.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_ENABLE;
        }

        //initialize using 1B bus
        local->hal_handle.Init.BusWide = 0;

        //MMC_HARDWARE_FLOW_CONTROL_DISABLE
        //SDIO_HARDWARE_FLOW_CONTROL_ENABLE
        local->hal_handle.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
        if( o_flags & MMC_FLAG_IS_HARDWARE_FLOW_CONTROL_ENABLED ){
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
                    MCU_CONFIG_PIN_ASSIGNMENT(mmc_config_t, handle),
                    MCU_PIN_ASSIGNMENT_COUNT(mmc_pin_assignment_t),
                    CORE_PERIPH_SDIO, handle->port, 0, 0, 0) < 0 ){
            return SYSFS_SET_RETURN(EINVAL);
        }

        if( HAL_MMC_Init(&local->hal_handle) != HAL_OK ){
            return SYSFS_SET_RETURN(EINVAL);
        }

        if( HAL_MMC_Init(&local->hal_handle) != HAL_OK ){
            return SYSFS_SET_RETURN(EIO);
        }

        //SDIO_BUS_WIDE_1B -- set as default for initialziation
        //SDIO_BUS_WIDE_4B
        //SDIO_BUS_WIDE_8B -- not compatible with SDIO
        if( o_flags & MMC_FLAG_IS_BUS_WIDTH_8 ){
            HAL_MMC_ConfigWideBusOperation(&local->hal_handle, SDIO_BUS_WIDE_4B);
        } else if ( o_flags & MMC_FLAG_IS_BUS_WIDTH_8 ){
            HAL_MMC_ConfigWideBusOperation(&local->hal_handle, SDIO_BUS_WIDE_8B);
        }

    }

    if( o_flags & MMC_FLAG_GET_CARD_STATE ){
        return HAL_MMC_GetCardState(&local->hal_handle);
    }

    if( o_flags & MMC_FLAG_ERASE_BLOCKS ){
        if( HAL_MMC_Erase(&local->hal_handle, attr->start, attr->end) != HAL_OK ){
            return SYSFS_SET_RETURN(EIO);
        }
    }


    return SYSFS_RETURN_SUCCESS;
}


int mmc_local_setaction(const devfs_handle_t * handle, void * ctl){
    mcu_action_t * action = ctl;
    const u32 port = handle->port;
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

int mmc_local_getcid(const devfs_handle_t * handle, void * ctl){
    mmc_local_t * local = mmc_local + handle->port;
    if( HAL_MMC_GetCardCID(&local->hal_handle, ctl) == HAL_OK ){
        return SYSFS_RETURN_SUCCESS;
    }

    return SYSFS_SET_RETURN(EIO);
}

int mmc_local_getcsd(const devfs_handle_t * handle, void * ctl){
    mmc_local_t * local = mmc_local + handle->port;
    if( HAL_MMC_GetCardCSD(&local->hal_handle, ctl) == HAL_OK ){
        return SYSFS_RETURN_SUCCESS;
    }

    return SYSFS_SET_RETURN(EIO);
}

int mmc_local_getstatus(const devfs_handle_t * handle, void * ctl){
    MCU_UNUSED_ARGUMENT(handle);
    MCU_UNUSED_ARGUMENT(handle);
    return SYSFS_SET_RETURN(ENOTSUP);
}


void HAL_MMC_TxCpltCallback(MMC_HandleTypeDef *hmmc){
    mmc_local_t * local = (mmc_local_t *)hmmc;
    devfs_execute_write_handler(&local->transfer_handler, 0, hmmc->TxXferSize, MCU_EVENT_FLAG_WRITE_COMPLETE);
}

void HAL_MMC_RxCpltCallback(MMC_HandleTypeDef *hmmc){
    mmc_local_t * local = (mmc_local_t *)hmmc;
    devfs_execute_read_handler(&local->transfer_handler, 0, hmmc->RxXferSize, MCU_EVENT_FLAG_DATA_READY);
}

void HAL_MMC_ErrorCallback(MMC_HandleTypeDef *hmmc){
    mmc_local_t * local = (mmc_local_t *)hmmc;
    //mcu_debug_log_warning(MCU_DEBUG_DEVICE, "SD Error? 0x%lX 0x%lX %ld", hsd->ErrorCode, hsd->hdmatx->ErrorCode, TIM2->CNT - sdio->start_time);
    if( hmmc->ErrorCode ){
        mcu_debug_log_error(MCU_DEBUG_DEVICE, "SD Error 0x%lX", hmmc->ErrorCode);
        devfs_execute_cancel_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_ERROR);
    }
}

void HAL_MMC_AbortCallback(MMC_HandleTypeDef *hmmc){
    mmc_local_t * local = (mmc_local_t *)hmmc;
    //abort read and write
    mcu_debug_log_warning(MCU_DEBUG_DEVICE, "Abort\n");
    devfs_execute_cancel_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EIO), 0);
}

void mcu_core_mmc_isr(){
    //mcu_debug_log_info(MCU_DEBUG_DEVICE, "SDIO IRQ 0x%lX", sd_handle[0]->Instance->STA);
    HAL_MMC_IRQHandler(&mmc_local[0].hal_handle);
}




#endif

