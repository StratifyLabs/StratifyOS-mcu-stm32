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
#include "mcu/adc.h"
#include "mcu/pio.h"
#include "mcu/debug.h"
#include "mcu/core.h"
#include "adc_local.h"

#if MCU_ADC_PORTS > 0

static adc_dma_local_t adc_local[MCU_ADC_PORTS] MCU_SYS_MEM;


DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(adc_dma, ADC_VERSION, ADC_IOC_IDENT_CHAR)

int mcu_adc_dma_open(const devfs_handle_t * handle){
    adc_local[handle->port].adc.o_flags = ADC_LOCAL_FLAG_IS_DMA;

    return adc_local_open(&adc_local[handle->port].adc, handle);
}

int mcu_adc_dma_close(const devfs_handle_t * handle){
    return adc_local_close(&adc_local[handle->port].adc, handle);
}


int mcu_adc_dma_getinfo(const devfs_handle_t * handle, void * ctl){
    adc_info_t * info = ctl;
    adc_local_getinfo(&adc_local[handle->port].adc, handle, ctl);
    info->o_flags |= ADC_FLAG_IS_GROUP | ADC_FLAG_IS_SCAN_MODE;
    return SYSFS_RETURN_SUCCESS;
}

int mcu_adc_dma_setattr(const devfs_handle_t * handle, void * ctl){
    u32 o_flags;
    int port = handle->port;
    int result;
    const adc_attr_t * attr;


    adc_local_t * adc = &adc_local[handle->port].adc;

    attr = mcu_select_attr(handle, ctl);
    if( attr == 0 ){
        return SYSFS_SET_RETURN(EINVAL);
    }

    o_flags = attr->o_flags;

    if( (result = adc_local_setattr(adc, handle, ctl)) < 0 ){
        return result;
    }

    if( o_flags & ADC_FLAG_SET_CONVERTER ){

        //ENABLE or DISABLE (if ENABLE DMA must be in circular buffer mode)
        adc->hal_handle.Init.DMAContinuousRequests = DISABLE; //DMA is not in circular mode

        //use continous mode -- DMA is triggered as soon as previous conversion completes
        //ENABLE or DISABLE
        adc->hal_handle.Init.ContinuousConvMode = ENABLE;

        //ADC_EOC_SEQ_CONV
        //ADC_EOC_SINGLE_CONV
        //ADC_EOC_SINGLE_SEQ_CONV
        adc->hal_handle.Init.EOCSelection = ADC_EOC_SEQ_CONV; //DMA needs to use end of sequence
        adc->hal_handle.Init.ScanConvMode = ENABLE; //always do scan mode with DMA
        adc->hal_handle.Init.NbrOfConversion = 1;

        if( o_flags & ADC_FLAG_IS_SCAN_MODE ){
            //up to 16 conversions
            if( attr->channel_count <= 16 ){
                adc->hal_handle.Init.NbrOfConversion = attr->channel_count;
            } else {
                adc->hal_handle.Init.NbrOfConversion = 16;
            }
        }

        //configure DMA
        //setup the DMA for receiving
        const stm32_adc_dma_config_t * config;
        config = handle->config;
        if( config == 0 ){ return SYSFS_SET_RETURN(EINVAL); }

        mcu_debug_root_printf("Configure DMA %d %d %d\n", config->dma_config.dma_number, config->dma_config.stream_number, config->dma_config.channel_number);
        stm32_dma_channel_t * dma_channel = &adc_local[port].dma_rx_channel;
        stm32_dma_set_handle(dma_channel, config->dma_config.dma_number, config->dma_config.stream_number); //need to get the DMA# and stream# from a table -- or from config
        dma_channel->handle.Instance = stm32_dma_get_stream_instance(config->dma_config.dma_number, config->dma_config.stream_number); //DMA1 stream 0
        dma_channel->handle.Init.Channel = stm32_dma_decode_channel(config->dma_config.channel_number);
        dma_channel->handle.Init.Mode = DMA_NORMAL;
        dma_channel->handle.Init.Direction = DMA_PERIPH_TO_MEMORY; //read is always periph to memory
        dma_channel->handle.Init.PeriphInc = DMA_PINC_DISABLE; //don't inc peripheral
        dma_channel->handle.Init.MemInc = DMA_MINC_ENABLE; //do inc the memory

        dma_channel->handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        dma_channel->handle.Init.FIFOMode = DMA_FIFO_THRESHOLD_HALFFULL;

        dma_channel->handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        dma_channel->handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
        dma_channel->handle.Init.MemBurst = DMA_MBURST_INC4;
        dma_channel->handle.Init.PeriphBurst = DMA_MBURST_INC4;

        dma_channel->handle.Init.Priority = stm32_dma_decode_priority(config->dma_config.priority);

        if (HAL_DMA_Init(&dma_channel->handle) != HAL_OK){
            return SYSFS_SET_RETURN(EIO);
        }

        __HAL_LINKDMA((&adc_local[port].adc.hal_handle), DMA_Handle, dma_channel->handle);

        if( HAL_ADC_Init(&adc->hal_handle) != HAL_OK ){
            return SYSFS_SET_RETURN(EIO);
        }
    }

    return SYSFS_RETURN_SUCCESS;
}


int mcu_adc_dma_setaction(const devfs_handle_t * handle, void * ctl){
    mcu_action_t * action = (mcu_action_t*)ctl;
    int port = handle->port;
    adc_dma_local_t * adc = adc_local + port;

    if( action->handler.callback == 0 ){
        //if there is an ongoing operation -- cancel it
        if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
            //execute the read callback if not null
            mcu_execute_read_handler_with_flags(&adc->adc.transfer_handler, 0, SYSFS_SET_RETURN(EAGAIN), MCU_EVENT_FLAG_CANCELED);
        }
    }

    //get interrupt from STM32 DMA
    if( adc->dma_rx_channel.interrupt_number >= 0 ){
        cortexm_set_irq_priority(adc->dma_rx_channel.interrupt_number, action->prio, action->o_events);
    }
    return 0;
}

int mcu_adc_dma_read(const devfs_handle_t * handle, devfs_async_t * async){
    int port = handle->port;
    adc_dma_local_t * adc = adc_local + port;

    DEVFS_DRIVER_IS_BUSY(adc->adc.transfer_handler.read, async);

    if( async->nbyte < 2 ){
        adc->adc.transfer_handler.read = 0;
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
        if( HAL_ADC_ConfigChannel(&adc->adc.hal_handle, &channel_config) != HAL_OK ){
            return SYSFS_SET_RETURN(EIO);
        }
    }

    adc->adc.words_read = 0;
    async->nbyte &= ~0x01; //align to 2 byte boundary

    if( HAL_ADC_Start_DMA(&adc->adc.hal_handle, async->buf, async->nbyte/2) == HAL_OK ){
        //mcu_debug_root_printf("wait DMA\n");
        return 0;
    }

    //this needs to read 1 byte at a time
    adc->adc.transfer_handler.read = 0;
    return SYSFS_SET_RETURN(EIO);
}


int mcu_adc_dma_write(const devfs_handle_t * handle, devfs_async_t * async){
    return SYSFS_SET_RETURN(ENOTSUP);
}




#endif
