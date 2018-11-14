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

#include "i2s_spi_local.h"
#include <mcu/i2s.h>
#include <mcu/spi.h>

#if defined STM32F4
#include "stm32f4xx/stm32f4xx_hal_i2s_ex.h"
#endif

#if MCU_I2S_SPI_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(i2s_spi_dma, I2S_VERSION, I2S_IOC_IDENT_CHAR, I_MCU_TOTAL + I_I2S_TOTAL, mcu_i2s_spi_dma_mute, mcu_i2s_spi_dma_unmute)

int mcu_i2s_spi_dma_open(const devfs_handle_t * handle){
	spi_local_t * local = spi_local + handle->port;
	local->o_flags = SPI_LOCAL_IS_DMA;
	return i2s_spi_local_open(handle);
}

int mcu_i2s_spi_dma_close(const devfs_handle_t * handle){
	//same as SPI
	spi_local_t * local = spi_local + handle->port;

	if( local->ref_count == 1 ){
		//disable the DMA
		const stm32_i2s_spi_dma_config_t * config;
		config = handle->config;

		HAL_I2S_DMAStop(&local->i2s_hal_handle);
		devfs_execute_cancel_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EIO), 0);

		if( config ){
			stm32_dma_clear_handle(config->dma_config.rx.dma_number, config->dma_config.rx.stream_number);
			stm32_dma_clear_handle(config->dma_config.tx.dma_number, config->dma_config.tx.stream_number);
		}

	}

	return i2s_spi_local_close(handle);
}

int mcu_i2s_spi_dma_getinfo(const devfs_handle_t * handle, void * ctl){
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

int mcu_i2s_spi_dma_mute(const devfs_handle_t * handle, void * ctl){
	return i2s_spi_local_mute(handle, ctl);
}

int mcu_i2s_spi_dma_unmute(const devfs_handle_t * handle, void * ctl){
	return i2s_spi_local_unmute(handle, ctl);
}

int mcu_i2s_spi_dma_setattr(const devfs_handle_t * handle, void * ctl){
	spi_local_t * local = spi_local + handle->port;
	const stm32_i2s_spi_dma_config_t * config;
	const i2s_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

	//setup the DMA
	//BSP *MUST* provide DMA configuration information
	config = handle->config;
	if( config == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

	if( attr->o_flags & (I2S_FLAG_SET_MASTER | I2S_FLAG_SET_SLAVE) ){

		if( attr->o_flags & I2S_FLAG_IS_RECEIVER ){

			mcu_debug_log_info(MCU_DEBUG_DEVICE, "Set I2S DMA as receiver %d.%d.%d",
									 config->dma_config.rx.dma_number,
									 config->dma_config.rx.stream_number,
									 config->dma_config.rx.channel_number
									 );

			stm32_dma_channel_t * channel = stm32_dma_setattr(&config->dma_config.rx);
			if( channel == 0 ){ return SYSFS_SET_RETURN(EIO); }

			__HAL_LINKDMA((&local->i2s_hal_handle), hdmarx, channel->handle);

		}
		if( attr->o_flags & I2S_FLAG_IS_TRANSMITTER ){

			mcu_debug_log_info(MCU_DEBUG_DEVICE, "Set I2S DMA as transmitter %d.%d.%d",
									 config->dma_config.tx.dma_number,
									 config->dma_config.tx.stream_number,
									 config->dma_config.tx.channel_number
									 );

			stm32_dma_channel_t * channel = stm32_dma_setattr(&config->dma_config.tx);

			if( channel == 0 ){ return SYSFS_SET_RETURN(EIO); }

			__HAL_LINKDMA((&local->i2s_hal_handle), hdmatx, channel->handle);

		}
	}

	return i2s_spi_local_setattr(handle, ctl);
}


int mcu_i2s_spi_dma_setaction(const devfs_handle_t * handle, void * ctl){
	return mcu_spi_dma_setaction(handle, ctl);
}

int mcu_i2s_spi_dma_write(const devfs_handle_t * handle, devfs_async_t * async){
	int result;
	spi_local_t * local = spi_local + handle->port;

	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.write, async);

	if( (local->o_flags & SPI_LOCAL_IS_DMA) && local->transfer_handler.read ){

#if defined SPI_I2S_FULLDUPLEX_SUPPORT
		if( local->transfer_handler.read->nbyte < async->nbyte ){
			return SYSFS_SET_RETURN(EINVAL);
		}
		i2s_spi_local_wait_for_errata_level(local);

		result = HAL_I2SEx_TransmitReceive_DMA(
					&local->i2s_hal_handle,
					async->buf,
					local->transfer_handler.read->buf,
					async->nbyte);
#else
		return SYSFS_SET_RETURN(ENOTSUP);
#endif

	} else {
		mcu_debug_log_info(MCU_DEBUG_DEVICE, "Write I2S DMA 0x%lX %p %d %d 0x%lX", local->i2s_hal_handle.Init.Mode, async->buf, async->nbyte, local->size_mult, local->o_flags);
		i2s_spi_local_wait_for_errata_level(local);

		result = HAL_I2S_Transmit_DMA(&local->i2s_hal_handle, async->buf,  (async->nbyte/local->size_mult));
	}

	if( result != HAL_OK ){
		local->transfer_handler.write = 0;
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

int mcu_i2s_spi_dma_read(const devfs_handle_t * handle, devfs_async_t * async){
	int ret;
	spi_local_t * local = spi_local + handle->port;

	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.read, async);

#if defined SPI_I2S_FULLDUPLEX_SUPPORT
	if( local->o_flags & SPI_LOCAL_IS_FULL_DUPLEX ){
		//Receive assigns the transfer handler but then blocks until a write happens
		mcu_debug_log_info(MCU_DEBUG_DEVICE, "Wait FD");
		return 0;
	}
#endif

	mcu_debug_log_info(MCU_DEBUG_DEVICE, "Read I2S DMA 0x%lX %p %d %d", local->i2s_hal_handle.Init.Mode, async->buf, async->nbyte, local->size_mult);

	i2s_spi_local_wait_for_errata_level(local);

	ret = HAL_I2S_Receive_DMA(&local->i2s_hal_handle, async->buf,  (async->nbyte/local->size_mult));

	if( ret != HAL_OK ){
		mcu_debug_log_error(MCU_DEBUG_DEVICE, "Failed to start I2S DMA Read (%d, %d) %d/%d", ret, local->i2s_hal_handle.ErrorCode, async->nbyte, local->size_mult);
		local->transfer_handler.read = 0;
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

