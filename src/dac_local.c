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
#include "stm32_local.h"
#include "cortexm/cortexm.h"
#include "mcu/dac.h"
#include "mcu/pio.h"
#include "mcu/debug.h"
#include "mcu/core.h"

#include "dac_local.h"

#if MCU_DAC_PORTS > 0

DAC_TypeDef * const dac_regs_table[MCU_DAC_PORTS] = MCU_DAC_REGS;
u8 const dac_irqs[MCU_DAC_PORTS] = MCU_DAC_IRQS;

#if 0
const u32 dac_channels[MCU_DAC_CHANNELS] = {
    DAC_CHANNEL_0, DAC_CHANNEL_1, DAC_CHANNEL_2, DAC_CHANNEL_3,
    DAC_CHANNEL_4, DAC_CHANNEL_5, DAC_CHANNEL_6, DAC_CHANNEL_7,
    DAC_CHANNEL_8, DAC_CHANNEL_9, DAC_CHANNEL_10, DAC_CHANNEL_11,
    DAC_CHANNEL_12, DAC_CHANNEL_13, DAC_CHANNEL_14, DAC_CHANNEL_15,
    DAC_CHANNEL_16, DAC_CHANNEL_17, DAC_CHANNEL_18, DAC_CHANNEL_VREFINT,
    DAC_CHANNEL_VBAT
};
#endif

int dac_local_open(dac_local_t * dac, const devfs_handle_t * handle){
    int port = handle->port;
    if ( dac->ref_count == 0 ){

        dac->hal_handle.Instance = dac_regs_table[port];

        switch(port){
        case 0:
#if defined __HAL_RCC_DAC1_CLK_ENABLE
            __HAL_RCC_DAC1_CLK_ENABLE();
#elif defined __HAL_RCC_DAC_CLK_ENABLE
            __HAL_RCC_DAC_CLK_ENABLE();
#else
#error("__HAL_RCC_DAC_CLK_ENABLE is not defined")
#endif
            break;
#if defined __HAL_RCC_DAC2_CLK_ENABLE
        case 1:
            __HAL_RCC_DAC2_CLK_ENABLE();
            break;
#endif
#if defined __HAL_RCC_DAC3_CLK_ENABLE
        case 2:
            __HAL_RCC_DAC3_CLK_ENABLE();
            break;
#endif
#if defined __HAL_RCC_DAC4_CLK_ENABLE
        case 3:
            __HAL_RCC_DAC4_CLK_ENABLE();
            break;
#endif
        }
        cortexm_enable_irq(dac_irqs[port]);
    }
    dac->ref_count++;

    return 0;
}

int dac_local_close(dac_local_t * dac, const devfs_handle_t * handle){
    int port = handle->port;

    if ( dac->ref_count > 0 ){
        if ( dac->ref_count == 1 ){
            cortexm_disable_irq(dac_irqs[port]);
            switch(port){
            case 0:
#if defined __HAL_RCC_DAC1_CLK_DISABLE
                __HAL_RCC_DAC1_CLK_DISABLE();
#elif defined __HAL_RCC_DAC_CLK_DISABLE
                __HAL_RCC_DAC_CLK_DISABLE();
#else
#error("__HAL_RCC_DAC_CLK_DISABLE not defined")
#endif
                break;
#if defined __HAL_RCC_DAC2_CLK_DISABLE
            case 1:
                __HAL_RCC_DAC2_CLK_DISABLE();
                break;
#endif
#if defined __HAL_RCC_DAC3_CLK_DISABLE
            case 2:
                __HAL_RCC_DAC3_CLK_DISABLE();
                break;
#endif
#if defined __HAL_RCC_DAC4_CLK_DISABLE
            case 3:
                __HAL_RCC_DAC4_CLK_DISABLE();
                break;
#endif
            }
            dac->hal_handle.Instance = 0;
        }
        dac->ref_count--;
    }
    return 0;
}


int dac_local_getinfo(dac_local_t * dac, const devfs_handle_t * handle, void * ctl){

    dac_info_t * info = ctl;
    const dac_config_t * config = handle->config;

    info->o_flags = DAC_FLAG_IS_LEFT_JUSTIFIED |
            DAC_FLAG_IS_RIGHT_JUSTIFIED |
            DAC_FLAG_SET_CONVERTER;
    info->o_events = MCU_EVENT_FLAG_DATA_READY;
    //info->maximum = 0xffff; //max value
    info->freq = 1000000; //max frequency
    //info->bytes_per_sample = 2;

    if( config ){
        //info->reference_mv = config->reference_mv;
    } else {
        //info->reference_mv = 0;
    }


    return SYSFS_RETURN_SUCCESS;
}

int dac_local_setattr(dac_local_t * dac, const devfs_handle_t * handle, void * ctl){
    u32 o_flags;
    int port = handle->port;
    const dac_attr_t * attr;

    attr = mcu_select_attr(handle, ctl);
    if( attr == 0 ){
        return SYSFS_SET_RETURN(EINVAL);
    }

    o_flags = attr->o_flags;

    if( o_flags & DAC_FLAG_SET_CONVERTER ){



        if( HAL_DAC_Init(&dac->hal_handle) != HAL_OK ){
            return SYSFS_SET_RETURN(EIO);
        }


    }

    if( o_flags & DAC_FLAG_SET_CHANNELS ){
        //pin assignments
        if( mcu_set_pin_assignment(
                    &(attr->pin_assignment),
                    MCU_CONFIG_PIN_ASSIGNMENT(dac_config_t, handle),
                    MCU_PIN_ASSIGNMENT_COUNT(dac_pin_assignment_t),
                    CORE_PERIPH_DAC, port, 0, 0, 0) < 0 ){
            return SYSFS_SET_RETURN(EINVAL);
        }

        DAC_ChannelConfTypeDef channel_config;

        channel_config.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
        channel_config.DAC_Trigger = DAC_TRIGGER_NONE;
        channel_config.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
        channel_config.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
        channel_config.DAC_UserTrimming = DAC_TRIMMING_FACTORY;

    }



    return SYSFS_RETURN_SUCCESS;
}

void HAL_DAC_LevelOutOfWindowCallback(DAC_HandleTypeDef* hdac){
    //this could be used to set a custom event when the DAC is out of a window
}

void HAL_DAC_ErrorCallback(DAC_HandleTypeDef *hdac){
    dac_local_t * dac = (dac_local_t*)hdac;
    mcu_debug_log_error(MCU_DEBUG_DEVICE, "DAC Error %d", hdac->ErrorCode);
#if defined DAC_SR_OVR
    hdac->Instance->SR &= ~DAC_SR_OVR;
#endif
    mcu_execute_read_handler_with_flags(&dac->transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR);
    if( (dac->o_flags & DAC_LOCAL_FLAG_IS_DMA) == 0 ){
        //HAL_DAC_Stop_IT(hdac);
    } else {
        //HAL_DAC_Stop_DMA(hdac);
    }
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac){


}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef * hdac){
#if 0
    dac_local_t * dac = (dac_local_t*)hdac;
    devfs_async_t * write_async = dac->transfer_handler.read;
    if( write_async ){
        //write to buffer
        if( dac->o_flags & DAC_LOCAL_FLAG_IS_DMA ){
            mcu_execute_write_handler(&dac->transfer_handler, 0, write_async->nbyte);
        } else {
            u16 * dest = write_async->buf;
            HAL_DAC_SetValue(hdac, 0, 0, 0);
            //dest[dac->words_read] = HAL_DAC_GetValue(hdac);
            dac->words_written++;
            if( dac->words_written * 2 == write_async->nbyte ){
                HAL_DAC_Stop_IT(hdac); //only needed for non software triggers
                mcu_execute_write_handler(&dac->transfer_handler, 0, write_async->nbyte);
            } else {
                HAL_DAC_Start_IT(hdac);
                return;
            }
        }
    }
#endif
}


#endif
