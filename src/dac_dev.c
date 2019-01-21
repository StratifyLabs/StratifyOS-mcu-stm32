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

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(dac, DAC_VERSION, DAC_IOC_IDENT_CHAR, I_MCU_TOTAL + I_DAC_TOTAL, mcu_dac_get, mcu_dac_set)

int mcu_dac_open(const devfs_handle_t * handle){
    int result;
    dac_local[handle->port].o_flags = 0;


    result = dac_local_open(handle);
    if( result < 0 ){ return result; }

    cortexm_enable_irq(dac_irqs[handle->port]);

    return SYSFS_RETURN_SUCCESS;
}

int mcu_dac_close(const devfs_handle_t * handle){
    int result;

    result = dac_local_close(handle);
    if( result < 0 ){ return result; }

    if( dac_local[handle->port].ref_count == 0 ){
        cortexm_enable_irq(dac_irqs[handle->port]);
    }

    return SYSFS_RETURN_SUCCESS;
}


int mcu_dac_getinfo(const devfs_handle_t * handle, void * ctl){
    return dac_local_getinfo(handle, ctl);
}

int mcu_dac_setattr(const devfs_handle_t * handle, void * ctl){
    return dac_local_setattr(handle, ctl);
}


int mcu_dac_setaction(const devfs_handle_t * handle, void * ctl){
    mcu_action_t * action = (mcu_action_t*)ctl;
    int port = handle->port;
    dac_local_t * adc = dac_local + port;

    if( action->handler.callback == 0 ){
        //if there is an ongoing operation -- cancel it
        if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
            //execute the read callback if not null
            devfs_execute_read_handler(&adc->transfer_handler, 0, SYSFS_SET_RETURN(EAGAIN), MCU_EVENT_FLAG_CANCELED);
        }
    }

    cortexm_set_irq_priority(dac_irqs[port], action->prio, action->o_events);
    return 0;
}

int mcu_dac_get(const devfs_handle_t * handle, void * ctl){
    return dac_local_get(handle, ctl);
}

int mcu_dac_set(const devfs_handle_t * handle, void * ctl){
    return dac_local_set(handle, ctl);
}

int mcu_dac_read(const devfs_handle_t * handle, devfs_async_t * async){
    return SYSFS_SET_RETURN(ENOTSUP);
}


int mcu_dac_write(const devfs_handle_t * handle, devfs_async_t * async){

    //this driver is only capable of writing a single value -- no interrupt -- value is written synchronously
    int result;
    mcu_channel_t mcu_channel;
    mcu_channel.loc = handle->port;
    mcu_channel.value = ((u16*)async->buf_const)[0];
    result = dac_local_set(handle, &mcu_channel);
    if( result < 0 ){
        return result;
    }
    return 2;
}



#endif
