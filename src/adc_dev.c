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
#include "adc_local.h"

#if MCU_ADC_PORTS > 0

static adc_local_t adc_local[MCU_ADC_PORTS] MCU_SYS_MEM;

DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(adc, ADC_VERSION, ADC_IOC_IDENT_CHAR)

int mcu_adc_open(const devfs_handle_t * handle){
    int result;
    adc_local[handle->port].o_flags = 0;


    result = adc_local_open(&adc_local[handle->port], handle);
    if( result < 0 ){ return result; }

    cortexm_enable_irq(adc_irqs[handle->port]);

    return SYSFS_RETURN_SUCCESS;

#if 0
    int port = handle->port;
    if ( adc_local[port].ref_count == 0 ){

        adc_local[port].hal_handle.Instance = adc_regs_table[port];

        switch(port){
        case 0:
            __HAL_RCC_ADC1_CLK_ENABLE();
            break;
#if defined ADC2
        case 1:
            __HAL_RCC_ADC2_CLK_ENABLE();
            break;
#endif
#if defined ADC3
        case 2:
            __HAL_RCC_ADC3_CLK_ENABLE();
            break;
#endif
#if defined ADC4
        case 3:
            __HAL_RCC_ADC4_CLK_ENABLE();
            break;
#endif
        }
        //reset HAL UART
        cortexm_enable_irq(adc_irqs[port]);

    }
    adc_local[port].ref_count++;
#endif

    return 0;
}

int mcu_adc_close(const devfs_handle_t * handle){
    int result;

    result = adc_local_close(&adc_local[handle->port], handle);
    if( result < 0 ){ return result; }

    if( adc_local[handle->port].ref_count == 0 ){
        cortexm_enable_irq(adc_irqs[handle->port]);
    }

    return SYSFS_RETURN_SUCCESS;
#if 0
    int port = handle->port;

    if ( adc_local[port].ref_count > 0 ){
        if ( adc_local[port].ref_count == 1 ){
            cortexm_disable_irq(adc_irqs[port]);
            switch(port){
            case 0:
                __HAL_RCC_ADC1_CLK_DISABLE();
                break;
#if defined ADC2
            case 1:
                __HAL_RCC_ADC2_CLK_DISABLE();
                break;
#endif
#if defined ADC3
            case 2:
                __HAL_RCC_ADC3_CLK_DISABLE();
                break;
#endif
#if defined ADC4
            case 3:
                __HAL_RCC_ADC4_CLK_DISABLE();
                break;
#endif
            }
            adc_local[port].hal_handle.Instance = 0;
        }
        adc_local[port].ref_count--;
    }
    return 0;
#endif
}


int mcu_adc_getinfo(const devfs_handle_t * handle, void * ctl){
    return adc_local_getinfo(&adc_local[handle->port], handle, ctl);
}

int mcu_adc_setattr(const devfs_handle_t * handle, void * ctl){
    int result;
    u32 o_flags;
    const adc_attr_t * attr;
    attr = mcu_select_attr(handle, ctl);
    if( attr == 0 ){
        return SYSFS_SET_RETURN(EINVAL);
    }

    o_flags = attr->o_flags;

    if( (result = adc_local_setattr(&adc_local[handle->port], handle, (void*)attr)) < 0 ){
        return result;
    }

    if( o_flags & ADC_FLAG_SET_CONVERTER ){
        if( HAL_ADC_Init(&adc_local[handle->port].hal_handle) != HAL_OK ){
            return SYSFS_SET_RETURN(EIO);
        }
    }

    return SYSFS_RETURN_SUCCESS;
}


int mcu_adc_setaction(const devfs_handle_t * handle, void * ctl){
    mcu_action_t * action = (mcu_action_t*)ctl;
    int port = handle->port;
    adc_local_t * adc = adc_local + port;

    if( action->handler.callback == 0 ){
        //if there is an ongoing operation -- cancel it
        if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
            //execute the read callback if not null
            mcu_execute_read_handler_with_flags(&adc->transfer_handler, 0, SYSFS_SET_RETURN(EAGAIN), MCU_EVENT_FLAG_CANCELED);
            HAL_ADC_Abort_IT(&adc->hal_handle);
        }
    }

    cortexm_set_irq_priority(adc_irqs[port], action->prio, action->o_events);
    return 0;
}

int mcu_adc_read(const devfs_handle_t * handle, devfs_async_t * async){
    int port = handle->port;
    adc_local_t * adc = adc_local + port;

    DEVFS_DRIVER_IS_BUSY(adc->transfer_handler.read, async);

    if( async->nbyte < 2 ){
        adc->transfer_handler.read = 0;
        return SYSFS_SET_RETURN(EINVAL);
    }

    //if location is not the group value -- configure the channel to read the group
    if( (u32)async->loc < MCU_ADC_CHANNELS ){
        //configure the channel to read
        ADC_ChannelConfTypeDef channel_config;
        channel_config.Offset = 0;
        channel_config.Channel = adc_channels[async->loc];
        channel_config.Rank = 1;
        channel_config.SamplingTime = ADC_SAMPLETIME_15CYCLES;
        if( HAL_ADC_ConfigChannel(&adc->hal_handle, &channel_config) != HAL_OK ){
            return SYSFS_SET_RETURN(EIO);
        }
    }

    adc->words_read = 0;
    async->nbyte &= ~0x01; //align to 2 byte boundary

    if( HAL_ADC_Start_IT(&adc->hal_handle) == HAL_OK ){
        return 0;
    }

    //this needs to read 1 byte at a time
    adc->transfer_handler.read = 0;
    return SYSFS_SET_RETURN(EIO);
}


int mcu_adc_write(const devfs_handle_t * handle, devfs_async_t * async){
    return SYSFS_SET_RETURN(ENOTSUP);
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc){
    //this could be used to set a custom event when the ADC is out of a window
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc){
    adc_local_t * adc = (adc_local_t*)hadc;
    mcu_debug_root_printf("ADC Error %d\n", hadc->ErrorCode);
    hadc->Instance->SR &= ~ADC_SR_OVR;
    mcu_execute_read_handler_with_flags(&adc->transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR);
    HAL_ADC_Stop_IT(hadc);
}


void mcu_core_adc_isr(){
    HAL_ADC_IRQHandler(&adc_local[0].hal_handle);
#if MCU_ADC_PORTS > 1
    HAL_ADC_IRQHandler(&adc_local[1].hal_handle);
#endif
#if MCU_ADC_PORTS > 2
    HAL_ADC_IRQHandler(&adc_local[2].hal_handle);
#endif
}


#endif
