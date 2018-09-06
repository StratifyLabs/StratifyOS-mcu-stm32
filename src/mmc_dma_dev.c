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

#include <mcu/mmc.h>
#include "mmc_local.h"

#if MCU_SDIO_PORTS > 0 && defined MMC_TypeDef

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(mmc_dma, MMC_VERSION, MMC_IOC_IDENT_CHAR, I_MCU_TOTAL + I_SDIO_TOTAL, mcu_mmc_dma_getcid, mcu_mmc_dma_getcsd, mcu_mmc_dma_getstatus)

int mcu_mmc_dma_open(const devfs_handle_t * handle){
    //mmc_local[handle->port].o_flags = MMC_LOCAL_IS_DMA;
    return mmc_local_open(handle);
}

int mcu_mmc_dma_close(const devfs_handle_t * handle){

    //do the opposite of mcu_mmc_dma_open() -- ref_count is zero -- turn off interrupt
    return 0;
}

int mcu_mmc_dma_getinfo(const devfs_handle_t * handle, void * ctl){
    return mmc_local_getinfo(handle, ctl);
}

int mcu_mmc_dma_setattr(const devfs_handle_t * handle, void * ctl){
    const mmc_attr_t * attr = mcu_select_attr(handle, ctl);
    if( attr == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

    u32 o_flags = attr->o_flags;
    mmc_local_t * local = mmc_local + handle->port;

    if( o_flags & MMC_FLAG_SET_INTERFACE ){

        const stm32_mmc_dma_config_t * config = handle->config;
        if( config == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

        int dma_result = stm32_dma_setattr(&local->dma_tx_channel, &config->dma_config.tx);
        if( dma_result < 0 ){
            return SYSFS_SET_RETURN(EIO);
        }

        dma_result = stm32_dma_setattr(&local->dma_rx_channel, &config->dma_config.rx);
        if( dma_result < 0 ){
            return SYSFS_SET_RETURN(EIO);
        }

        __HAL_LINKDMA((&local->hal_handle), hdmatx, local->dma_tx_channel.handle);
        __HAL_LINKDMA((&local->hal_handle), hdmarx, local->dma_rx_channel.handle);
    }

    return mmc_local_setattr(handle, ctl);
}


int mcu_mmc_dma_setaction(const devfs_handle_t * handle, void * ctl){
    mcu_action_t * action = ctl;
    u32 port = handle->port;
    mmc_local_t * local = mmc_local + handle->port;

    if( action->handler.callback == 0 ){
        if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
            HAL_MMC_Abort(&local->hal_handle);
            devfs_execute_read_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_CANCELED);
        }

        if( action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE ){
            HAL_MMC_Abort(&local->hal_handle);
            devfs_execute_write_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_CANCELED);
        }
    }

    cortexm_set_irq_priority(mmc_irqs[port], action->prio, action->o_events);
    return 0;
}

int mcu_mmc_dma_getcid(const devfs_handle_t * handle, void * ctl){
    return mmc_local_getcid(handle, ctl);
}

int mcu_mmc_dma_getcsd(const devfs_handle_t * handle, void * ctl){
    return mmc_local_getcsd(handle, ctl);
}

int mcu_mmc_dma_getstatus(const devfs_handle_t * handle, void * ctl){
    return mmc_local_getstatus(handle, ctl);
}

int mcu_mmc_dma_write(const devfs_handle_t * handle, devfs_async_t * async){
    mmc_local_t * local = mmc_local + handle->port;
    DEVFS_DRIVER_IS_BUSY(local->transfer_handler.write, async);

    local->hal_handle.TxXferSize = async->nbyte; //used by the callback but not set by HAL_SD_WriteBlocks_DMA
    if( (HAL_MMC_WriteBlocks_DMA(&local->hal_handle, async->buf, async->loc, async->nbyte / BLOCKSIZE)) == HAL_OK ){
        return 0;
    }

    local->transfer_handler.write = 0;
    return SYSFS_SET_RETURN(EIO);
}

int mcu_mmc_dma_read(const devfs_handle_t * handle, devfs_async_t * async){
    mmc_local_t * local = mmc_local + handle->port;
    int hal_result;
    DEVFS_DRIVER_IS_BUSY(local->transfer_handler.read, async);

#if 0
    mcu_debug_log_info(MCU_DEBUG_DEVICE, "RState:%d", HAL_SD_GetCardState(&local->hal_handle));
#endif

    local->hal_handle.RxXferSize = async->nbyte; //used by the callback but not set by HAL_SD_ReadBlocks_DMA
    if( (hal_result = HAL_MMC_ReadBlocks_DMA(&local->hal_handle, async->buf, async->loc, async->nbyte / BLOCKSIZE)) == HAL_OK ){
        return 0;
    }

    //mcu_debug_root_printf("R:HAL Not OK %d %d %d 0x%lX\n", async->loc, async->nbyte, hal_result, mmc_local[port].hal_handle.ErrorCode);
    //mcu_debug_root_printf("State: %d\n", HAL_SD_GetCardState(&mmc_local[port].hal_handle));
    local->transfer_handler.read = 0;
    return SYSFS_SET_RETURN(EIO);
}


#endif

