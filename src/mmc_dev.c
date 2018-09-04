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

static mmc_local_t mmc_local[MCU_SDIO_PORTS] MCU_SYS_MEM;


DEVFS_MCU_DRIVER_IOCTL_FUNCTION(sdio, MMC_VERSION, SDIO_IOC_IDENT_CHAR, I_MCU_TOTAL + I_SDIO_TOTAL, mcu_mmc_getcid, mcu_mmc_getcsd, mcu_mmc_getstatus)

int mcu_mmc_open(const devfs_handle_t * handle){
    const u32 port = handle->port;
    return mmc_local_open(mmc_local + port, handle);
}

int mcu_mmc_close(const devfs_handle_t * handle){
    return mmc_local_close(mmc_local + handle->port, handle);
}

int mcu_mmc_getinfo(const devfs_handle_t * handle, void * ctl){
    return mmc_local_getinfo(mmc_local + handle->port, handle, ctl);
}

int mcu_mmc_setattr(const devfs_handle_t * handle, void * ctl){
    return mmc_local_setattr(mmc_local + handle->port, handle, ctl);
}


int mcu_mmc_setaction(const devfs_handle_t * handle, void * ctl){
    mcu_action_t * action = ctl;
    u32 port = handle->port;

    if( action->handler.callback == 0 ){
        if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
            HAL_MMC_Abort_IT(&mmc_local[port].hal_handle);
            devfs_execute_read_handler(&mmc_local[port].transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_CANCELED);
        }

        if( action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE ){
            HAL_MMC_Abort_IT(&mmc_local[port].hal_handle);
            devfs_execute_write_handler(&mmc_local[port].transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_CANCELED);
        }
    }

    cortexm_set_irq_priority(mmc_irqs[port], action->prio, action->o_events);
    return 0;
}

int mcu_mmc_getcid(const devfs_handle_t * handle, void * ctl){
    return mmc_local_getcid(mmc_local + handle->port, handle, ctl);
}

int mcu_mmc_getcsd(const devfs_handle_t * handle, void * ctl){
    return mmc_local_getcsd(mmc_local + handle->port, handle, ctl);
}

int mcu_mmc_getstatus(const devfs_handle_t * handle, void * ctl){
    return mmc_local_getstatus(mmc_local + handle->port, handle, ctl);
}

int mcu_mmc_write(const devfs_handle_t * handle, devfs_async_t * async){
    int port = handle->port;
    DEVFS_DRIVER_IS_BUSY(mmc_local[port].transfer_handler.write, async);

    if( (HAL_MMC_WriteBlocks_IT(&mmc_local[port].hal_handle, async->buf, async->loc, async->nbyte / BLOCKSIZE)) == HAL_OK ){
        return 0;
    }

    mmc_local[port].transfer_handler.write = 0;
    return SYSFS_SET_RETURN(EIO);
}

int mcu_mmc_read(const devfs_handle_t * handle, devfs_async_t * async){
    int port = handle->port;
    int hal_result;
    DEVFS_DRIVER_IS_BUSY(mmc_local[port].transfer_handler.read, async);

    if( (hal_result = HAL_MMC_ReadBlocks_IT(&mmc_local[port].hal_handle, async->buf, async->loc, async->nbyte / BLOCKSIZE)) == HAL_OK ){
        return 0;
    }

    mmc_local[port].transfer_handler.read = 0;
    return SYSFS_SET_RETURN(EIO);
}




#endif

