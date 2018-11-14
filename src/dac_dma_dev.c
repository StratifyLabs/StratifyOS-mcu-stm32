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

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(dac_dma, DAC_VERSION, DAC_IOC_IDENT_CHAR, I_MCU_TOTAL + I_DAC_TOTAL, mcu_dac_dma_get, mcu_dac_dma_set)

int mcu_dac_dma_open(const devfs_handle_t * handle){
    dac_local[handle->port].o_flags = DAC_LOCAL_FLAG_IS_DMA;
    return dac_local_open(handle);
}

int mcu_dac_dma_close(const devfs_handle_t * handle){

    //cancel DMA operations

    return dac_local_close(handle);
}


int mcu_dac_dma_getinfo(const devfs_handle_t * handle, void * ctl){
    return dac_local_getinfo(handle, ctl);
}

int mcu_dac_dma_setattr(const devfs_handle_t * handle, void * ctl){
    const stm32_dac_dma_config_t * config = handle->config;
    dac_local_t * local = dac_local + handle->port;

    if( config == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

#if 0
    mcu_debug_log_info(MCU_DEBUG_DEVICE, "DAC Configure DMA %d %d %d", config->dma_config.dma_number, config->dma_config.stream_number, config->dma_config.channel_number);
    stm32_dma_channel_t * dma_channel = &dac_local[port].dma_tx_channel;
    stm32_dma_set_handle(dma_channel, config->dma_config.dma_number, config->dma_config.stream_number); //need to get the DMA# and stream# from a table -- or from config
    dma_channel->handle.Instance = stm32_dma_get_stream_instance(config->dma_config.dma_number, config->dma_config.stream_number); //DMA1 stream 0

#if defined DMA_REQUEST_0
    dma_channel->handle.Init.Request = stm32_dma_decode_channel(config->dma_config.channel_number);
#else
    dma_channel->handle.Init.Channel = stm32_dma_decode_channel(config->dma_config.channel_number);
#endif
    dma_channel->handle.Init.Mode = DMA_CIRCULAR;
    dma_channel->handle.Init.Direction = DMA_MEMORY_TO_PERIPH; //read is always periph to memory
    dma_channel->handle.Init.PeriphInc = DMA_PINC_DISABLE; //don't inc peripheral
    dma_channel->handle.Init.MemInc = DMA_MINC_ENABLE; //do inc the memory

#if defined DMA_FIFOMODE_ENABLE
    dma_channel->handle.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    dma_channel->handle.Init.FIFOMode = DMA_FIFO_THRESHOLD_HALFFULL;
    dma_channel->handle.Init.MemBurst = DMA_MBURST_INC4;
    dma_channel->handle.Init.PeriphBurst = DMA_MBURST_INC4;
#endif

    dma_channel->handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    dma_channel->handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;


    dma_channel->handle.Init.Priority = stm32_dma_decode_priority(config->dma_config.priority);

    if (HAL_DMA_Init(&dma_channel->handle) != HAL_OK){
        return SYSFS_SET_RETURN(EIO);
    }
#else

	 stm32_dma_channel_t * channel = stm32_dma_setattr(&config->dma_config);
	 if( channel == 0 ){ return SYSFS_SET_RETURN(EIO); }

#endif

    if( handle->port == 0 ){
		  __HAL_LINKDMA((&local->hal_handle), DMA_Handle1, channel->handle);
    } else {
		  __HAL_LINKDMA((&local->hal_handle), DMA_Handle2, channel->handle);
    }

    return dac_local_setattr(handle, ctl);
}


int mcu_dac_dma_setaction(const devfs_handle_t * handle, void * ctl){
    mcu_action_t * action = (mcu_action_t*)ctl;
    int port = handle->port;
    dac_local_t * local = dac_local + port;

    if( action->handler.callback == 0 ){
        //if there is an ongoing operation -- cancel it
        if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
            //execute the read callback if not null
            devfs_execute_read_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EAGAIN), MCU_EVENT_FLAG_CANCELED);
            HAL_DAC_Stop_DMA(&local->hal_handle, DAC_CHANNEL_1);
        }
    }

    cortexm_set_irq_priority(dac_irqs[port], action->prio, action->o_events);
    return 0;
}

int mcu_dac_dma_get(const devfs_handle_t * handle, void * ctl){
    return dac_local_get(handle, ctl);
}

int mcu_dac_dma_set(const devfs_handle_t * handle, void * ctl){
    return dac_local_set(handle, ctl);
}

int mcu_dac_dma_read(const devfs_handle_t * handle, devfs_async_t * async){
    return SYSFS_SET_RETURN(ENOTSUP);
}


int mcu_dac_dma_write(const devfs_handle_t * handle, devfs_async_t * async){
    int port = handle->port;
    dac_local_t * local = dac_local + port;
    u32 channel;

    DEVFS_DRIVER_IS_BUSY(local->transfer_handler.write, async);

    if( async->nbyte < 2 ){
        local->transfer_handler.write = 0;
        return SYSFS_SET_RETURN(EINVAL);
    }

    async->nbyte &= ~0x01; //align to 2 byte boundary
    if( (port < MCU_DAC_PORTS) ){
        channel = dac_channels[port];
    } else {
        local->transfer_handler.write = 0;
        return SYSFS_SET_RETURN(ENOSYS);
    }

    mcu_debug_log_info(MCU_DEBUG_DEVICE, "DAC DMA write %d words on channel 0x%X", async->nbyte/2, channel);

    if( HAL_DAC_Start_DMA(&local->hal_handle, channel, async->buf, async->nbyte/2, dac_local_get_alignment(local)) == HAL_OK ){
        //mcu_debug_root_printf("wait DMA\n");
        return 0;
    }

    //this needs to read 1 byte at a time
    local->transfer_handler.write = 0;
    return SYSFS_SET_RETURN(EIO);
}


#endif
