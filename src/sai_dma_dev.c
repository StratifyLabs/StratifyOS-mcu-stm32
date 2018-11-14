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

#include "sai_local.h"
#include <mcu/i2s.h>

#if defined STM32F4
#include "stm32f4xx/stm32f4xx_hal_i2s_ex.h"
#endif

#if MCU_SAI_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(sai_dma, I2S_VERSION, I2S_IOC_IDENT_CHAR, I_MCU_TOTAL + I_I2S_TOTAL, mcu_sai_dma_mute, mcu_sai_dma_unmute)

sai_dma_local_t sai_dma_local[MCU_SAI_PORTS] MCU_SYS_MEM;

int mcu_sai_dma_open(const devfs_handle_t * handle){
	return sai_local_open(&sai_dma_local[handle->port].sai, handle);
}

int mcu_sai_dma_close(const devfs_handle_t * handle){
	//same as SPI
	const u32 port = handle->port;

	if( sai_dma_local[handle->port].sai.ref_count == 1 ){
		//disable the DMA
		const stm32_sai_dma_config_t * config;
		config = handle->config;
		HAL_SAI_DMAStop(&sai_dma_local[handle->port].sai.hal_handle);

		if( sai_dma_local[port].dma_channel.interrupt_number > 0 ){
			HAL_DMA_DeInit(&sai_dma_local[port].dma_channel.handle);
		}

		if( config ){
			stm32_dma_clear_handle(config->dma_config.dma_number, config->dma_config.stream_number);
		}

	}

	sai_local_close(&sai_dma_local[port].sai, handle);


	return 0;
}

int mcu_sai_dma_getinfo(const devfs_handle_t * handle, void * ctl){
	i2s_info_t * info = ctl;

	//set I2S Capability flags
	info->o_flags = I2S_FLAG_SET_MASTER |
			I2S_FLAG_SET_SLAVE |
			I2S_FLAG_IS_WIDTH_8 |
			I2S_FLAG_IS_WIDTH_16 |
			I2S_FLAG_IS_WIDTH_16_EXTENDED |
			I2S_FLAG_IS_WIDTH_24 |
			I2S_FLAG_IS_WIDTH_32;

	info->o_events = MCU_EVENT_FLAG_DATA_READY |
			MCU_EVENT_FLAG_WRITE_COMPLETE |
			MCU_EVENT_FLAG_HIGH |
			MCU_EVENT_FLAG_LOW;


	return 0;
}

int mcu_sai_dma_mute(const devfs_handle_t * handle, void * ctl){
	return sai_local_mute((sai_local_t*)(sai_dma_local + handle->port), handle, ctl);
}

int mcu_sai_dma_unmute(const devfs_handle_t * handle, void * ctl){
	return sai_local_unmute((sai_local_t*)(sai_dma_local + handle->port), handle, ctl);
}

int mcu_sai_dma_setattr(const devfs_handle_t * handle, void * ctl){
	const u32 port = handle->port;
	const stm32_sai_dma_config_t * config;
	const i2s_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

	//setup the DMA
	//BSP *MUST* provide DMA configuration information
	config = handle->config;
	if( config == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

	if( attr->o_flags & (I2S_FLAG_SET_MASTER | I2S_FLAG_SET_SLAVE) ){

		sai_dma_local[port].sai.o_flags = SAI_LOCAL_IS_DMA;

		stm32_dma_channel_t * channel = stm32_dma_setattr(&config->dma_config);
		if( channel == 0 ){ return SYSFS_SET_RETURN(EIO); }

		if( attr->o_flags & I2S_FLAG_IS_RECEIVER ){
			mcu_debug_log_info(MCU_DEBUG_DEVICE, "Set I2S DMA as receiver %d.%d.%d",
									 config->dma_config.dma_number,
									 config->dma_config.stream_number,
									 config->dma_config.channel_number
									 );


			__HAL_LINKDMA((&sai_dma_local[port].sai.hal_handle), hdmarx, channel->handle);
		}

		if( attr->o_flags & I2S_FLAG_IS_TRANSMITTER ){

			mcu_debug_log_info(MCU_DEBUG_DEVICE, "Set I2S DMA as transmitter %d.%d.%d",
									 config->dma_config.dma_number,
									 config->dma_config.stream_number,
									 config->dma_config.channel_number
									 );
			//setup the DMA for transmitting
			__HAL_LINKDMA((&sai_dma_local[port].sai.hal_handle), hdmatx, sai_dma_local[port].dma_channel.handle);
		}
	}

	return sai_local_setattr(&sai_dma_local[handle->port].sai, handle, ctl);
}


int mcu_sai_dma_setaction(const devfs_handle_t * handle, void * ctl){
	return mcu_sai_dma_setaction(handle, ctl);
}

int mcu_sai_dma_write(const devfs_handle_t * handle, devfs_async_t * async){
	int result;
	int port = handle->port;
	sai_dma_local_t * local = sai_dma_local + port;

	DEVFS_DRIVER_IS_BUSY(local->sai.transfer_handler.write, async);

	result = HAL_SAI_Transmit_DMA(&local->sai.hal_handle, async->buf,  async->nbyte/local->sai.size_mult);

	if( result != HAL_OK ){
		sai_dma_local[port].sai.transfer_handler.write = 0;
		if( result == HAL_BUSY ){
			return SYSFS_SET_RETURN(EIO);
		} else if( result == HAL_ERROR ){
			return SYSFS_SET_RETURN(EIO);
		} else if( result == HAL_TIMEOUT ){
			return SYSFS_SET_RETURN(EIO);
		}
		return SYSFS_SET_RETURN(EIO);

	}

	return 0;
}

int mcu_sai_dma_read(const devfs_handle_t * handle, devfs_async_t * async){
	int ret;
	int port = handle->port;
	sai_dma_local_t * local = sai_dma_local + port;

	DEVFS_DRIVER_IS_BUSY(local->sai.transfer_handler.read, async);

	mcu_debug_log_info(MCU_DEBUG_DEVICE, "SAI DMA RX: %p %ld %d", async->buf, async->nbyte, local->sai.size_mult);
	ret = HAL_SAI_Receive_DMA(&local->sai.hal_handle, async->buf,  async->nbyte/local->sai.size_mult);

	if( ret != HAL_OK ){
		mcu_debug_log_error(MCU_DEBUG_DEVICE, "Failed to start I2S DMA Read (%d, %d) %d/%d", ret, local->sai.hal_handle.ErrorCode, async->nbyte, local->sai.size_mult);
		local->sai.transfer_handler.read = 0;
		if( ret == HAL_BUSY ){
			return SYSFS_SET_RETURN(EIO);
		} else if( ret == HAL_ERROR ){
			return SYSFS_SET_RETURN(EIO);
		} else if( ret == HAL_TIMEOUT ){
			return SYSFS_SET_RETURN(EIO);
		} else if( ret != HAL_OK ){
			return SYSFS_SET_RETURN(EIO);
		}
	}

	return 0;
}


#endif

