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
#include "dac_local.h"

#if MCU_DAC_PORTS > 0

static dac_local_t dac_local[MCU_DAC_PORTS] MCU_SYS_MEM;

DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(dac, DAC_VERSION, DAC_IOC_IDENT_CHAR)

int mcu_dac_open(const devfs_handle_t * handle){
    int result;
    dac_local[handle->port].o_flags = 0;


    result = dac_local_open(&dac_local[handle->port], handle);
    if( result < 0 ){ return result; }

    cortexm_enable_irq(dac_irqs[handle->port]);

    return SYSFS_RETURN_SUCCESS;
}

int mcu_dac_close(const devfs_handle_t * handle){
    int result;

    result = dac_local_close(&dac_local[handle->port], handle);
    if( result < 0 ){ return result; }

    if( dac_local[handle->port].ref_count == 0 ){
        cortexm_enable_irq(dac_irqs[handle->port]);
    }

    return SYSFS_RETURN_SUCCESS;
}


int mcu_dac_getinfo(const devfs_handle_t * handle, void * ctl){
    return dac_local_getinfo(&dac_local[handle->port], handle, ctl);
}

int mcu_dac_setattr(const devfs_handle_t * handle, void * ctl){
    int result;
    u32 o_flags;
    const dac_attr_t * attr;
    attr = mcu_select_attr(handle, ctl);
    if( attr == 0 ){
        return SYSFS_SET_RETURN(EINVAL);
    }

    o_flags = attr->o_flags;

    if( (result = dac_local_setattr(&dac_local[handle->port], handle, (void*)attr)) < 0 ){
        return result;
    }

    if( o_flags & DAC_FLAG_SET_CONVERTER ){
        if( HAL_DAC_Init(&dac_local[handle->port].hal_handle) != HAL_OK ){
            return SYSFS_SET_RETURN(EIO);
        }
    }

    return SYSFS_RETURN_SUCCESS;
}


int mcu_dac_setaction(const devfs_handle_t * handle, void * ctl){
    mcu_action_t * action = (mcu_action_t*)ctl;
    int port = handle->port;
    dac_local_t * adc = dac_local + port;

    if( action->handler.callback == 0 ){
        //if there is an ongoing operation -- cancel it
        if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
            //execute the read callback if not null
            mcu_execute_read_handler_with_flags(&adc->transfer_handler, 0, SYSFS_SET_RETURN(EAGAIN), MCU_EVENT_FLAG_CANCELED);
            HAL_DAC_Stop_IT(&adc->hal_handle);
        }
    }

    cortexm_set_irq_priority(dac_irqs[port], action->prio, action->o_events);
    return 0;
}

int mcu_dac_read(const devfs_handle_t * handle, devfs_async_t * async){
    return SYSFS_SET_RETURN(ENOTSUP);
}


int mcu_dac_write(const devfs_handle_t * handle, devfs_async_t * async){


    return SYSFS_SET_RETURN(ENOTSUP);
}

void mcu_core_dac_isr(){
    HAL_DAC_IRQHandler(&dac_local[0].hal_handle);
#if MCU_DAC_PORTS > 1
    HAL_DAC_IRQHandler(&dac_local[1].hal_handle);
#endif
#if MCU_DAC_PORTS > 2
    HAL_DAC_IRQHandler(&dac_local[2].hal_handle);
#endif
}


#endif
