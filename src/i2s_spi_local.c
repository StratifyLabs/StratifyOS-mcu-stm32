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

#if MCU_I2S_SPI_PORTS > 0


int i2s_spi_local_open(const devfs_handle_t * handle){
	spi_local_t * local = m_spi_local + handle->port;
	local->o_flags |= SPI_LOCAL_IS_I2S;
	return spi_local_open(handle);
}

int i2s_spi_local_close(const devfs_handle_t * handle){
	return spi_local_close(handle);
}

int i2s_spi_local_mute(const devfs_handle_t * handle, void * ctl){
	MCU_UNUSED_ARGUMENT(handle);
	MCU_UNUSED_ARGUMENT(ctl);
	return SYSFS_SET_RETURN(ENOTSUP);
}

int i2s_spi_local_unmute(const devfs_handle_t * handle, void * ctl){
	MCU_UNUSED_ARGUMENT(handle);
	MCU_UNUSED_ARGUMENT(ctl);
	return SYSFS_SET_RETURN(ENOTSUP);
}

int i2s_spi_local_setattr(const devfs_handle_t * handle, void * ctl){
	const u32 port = handle->port;
	const i2s_attr_t * attr = mcu_select_attr(handle, ctl);
	spi_local_t * local = m_spi_local + handle->port;
	if( attr == 0 ){ return SYSFS_SET_RETURN(EINVAL); }

	u32 o_flags = attr->o_flags;

	//set I2S Flags

	if( o_flags & (I2S_FLAG_SET_MASTER|I2S_FLAG_SET_SLAVE) ){
#if defined SPI_I2S_FULLDUPLEX_SUPPORT
#if defined I2S_FULLDUPLEXMODE_DISABLE
		local->i2s_hal_handle.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
#endif
#endif


		if( o_flags & I2S_FLAG_SET_SLAVE ){
			local->o_flags |= SPI_LOCAL_IS_ERRATA_REQUIRED;
			if( o_flags & I2S_FLAG_IS_TRANSMITTER ){
				local->i2s_hal_handle.Init.Mode = I2S_MODE_SLAVE_TX;
				if( o_flags & I2S_FLAG_IS_RECEIVER ){
#if defined SPI_I2S_FULLDUPLEX_SUPPORT && defined I2S_FULLDUPLEXMODE_ENABLE
					local->i2s_hal_handle.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
#endif
#if defined I2S_MODE_SLAVE_FULLDUPLEX
					local->i2s_hal_handle.Init.Mode = I2S_MODE_SLAVE_FULLDUPLEX;
#endif
#if defined SPI_I2S_FULLDUPLEX_SUPPORT
					local->o_flags |= SPI_LOCAL_IS_FULL_DUPLEX;
#endif
				}
			} else if ( o_flags & I2S_FLAG_IS_RECEIVER ){
				local->i2s_hal_handle.Init.Mode = I2S_MODE_SLAVE_RX;
			}
		} else {

			if( o_flags & I2S_FLAG_IS_TRANSMITTER ){
				local->i2s_hal_handle.Init.Mode = I2S_MODE_MASTER_TX;
				if( o_flags & I2S_FLAG_IS_RECEIVER ){
#if defined SPI_I2S_FULLDUPLEX_SUPPORT && defined I2S_FULLDUPLEXMODE_ENABLE
					local->i2s_hal_handle.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
#endif
#if defined I2S_MODE_MASTER_FULLDUPLEX
					local->i2s_hal_handle.Init.Mode = I2S_MODE_MASTER_FULLDUPLEX;
#endif
#if defined SPI_I2S_FULLDUPLEX_SUPPORT || defined STM32H7
					local->o_flags |= SPI_LOCAL_IS_FULL_DUPLEX;
#endif
				}
			} else if ( o_flags & I2S_FLAG_IS_RECEIVER ){
				local->i2s_hal_handle.Init.Mode = I2S_MODE_MASTER_RX;
			}
		}

		local->i2s_hal_handle.Init.Standard = I2S_STANDARD_PHILIPS;
		if( o_flags & I2S_FLAG_IS_FORMAT_MSB ){
			local->i2s_hal_handle.Init.Standard = I2S_STANDARD_MSB;
		} else if( o_flags & I2S_FLAG_IS_FORMAT_LSB ){
			local->i2s_hal_handle.Init.Standard = I2S_STANDARD_LSB;
		} else if( o_flags & I2S_FLAG_IS_FORMAT_PCM_SHORT ){
			local->i2s_hal_handle.Init.Standard = I2S_STANDARD_PCM_SHORT;
		} else if( o_flags & I2S_FLAG_IS_FORMAT_PCM_LONG ){
			local->i2s_hal_handle.Init.Standard = I2S_STANDARD_PCM_LONG;
		} else {
			local->o_flags |= SPI_LOCAL_IS_ERRATA_I2S;
		}

		local->i2s_hal_handle.Init.DataFormat = I2S_DATAFORMAT_16B;
		local->size_mult = 2;
		if( o_flags & I2S_FLAG_IS_WIDTH_24 ){
			local->i2s_hal_handle.Init.DataFormat = I2S_DATAFORMAT_24B;
			local->size_mult = 4;
		} else if( o_flags & I2S_FLAG_IS_WIDTH_32 ){
			local->i2s_hal_handle.Init.DataFormat = I2S_DATAFORMAT_32B;
			local->size_mult = 4;
		} else if ( o_flags & I2S_FLAG_IS_WIDTH_16_EXTENDED ){
			local->i2s_hal_handle.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
		}

		local->i2s_hal_handle.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
		if( o_flags & I2S_FLAG_IS_MCK_ENABLED ){
			local->i2s_hal_handle.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
		}

		local->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_8K;
		switch(attr->freq){
			case 11000: local->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_11K; break;
			case 16000: local->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_16K; break;
			case 22050: local->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_22K; break;
			case 32000: local->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_32K; break;
			case 44100: local->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_44K; break;
			case 48000: local->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_48K; break;
			case 96000: local->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_96K; break;
			case 192000: local->i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_192K; break;
			default:
				return SYSFS_SET_RETURN(EINVAL);
		}

		local->i2s_hal_handle.Init.CPOL = I2S_CPOL_LOW;
#if defined I2S_CLOCK_PLL
		local->i2s_hal_handle.Init.ClockSource = I2S_CLOCK_PLL;
#endif

#if defined I2S_FIRSTBIT_MSB
		local->i2s_hal_handle.Init.FirstBit = I2S_FIRSTBIT_MSB;
#endif

#if defined I2S_WS_INVERSION_DISABLE
		local->i2s_hal_handle.Init.FirstBit = I2S_WS_INVERSION_DISABLE;
#endif

#if defined I2S_DATA_24BIT_ALIGNMENT_RIGHT
		local->i2s_hal_handle.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
#endif

#if defined I2S_MASTER_KEEP_IO_STATE_DISABLE
		local->i2s_hal_handle.Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_DISABLE;
#endif

#if !defined STM32H7
		//errata: http://www.st.com/content/ccc/resource/technical/document/errata_sheet/0a/98/58/84/86/b6/47/a2/DM00037591.pdf/files/DM00037591.pdf/jcr:content/translations/en.DM00037591.pdf
		if( local->o_flags & SPI_LOCAL_IS_ERRATA_REQUIRED ){
			local->ws_pin = attr->pin_assignment.ws;
			if( local->ws_pin.port == 0xff ){
				//try the other config
				if( handle->config ){
					const i2s_attr_t * config_attr = handle->config;
					local->ws_pin = config_attr->pin_assignment.ws;
				}
			}
		}
#endif

		if( mcu_set_pin_assignment(
				 &(attr->pin_assignment),
				 MCU_CONFIG_PIN_ASSIGNMENT(i2s_config_t, handle),
				 MCU_PIN_ASSIGNMENT_COUNT(i2s_pin_assignment_t),
				 CORE_PERIPH_SPI,
				 port,
				 0,
				 0,
				 0) < 0 ){
			return SYSFS_SET_RETURN(EINVAL);
		}

		//cancel any ongoing transfers (if available)
		devfs_execute_cancel_handler(
					&local->transfer_handler,
					0,
					SYSFS_SET_RETURN(ECANCELED),
					0
					);

		int hal_result;
		if( (hal_result = HAL_I2S_Init(&local->i2s_hal_handle)) != HAL_OK ){
			mcu_debug_log_error(MCU_DEBUG_DEVICE, "Init I2S failed %d", hal_result);
			return SYSFS_SET_RETURN(EIO);
		}
	}

	return 0;
}

void i2s_spi_local_wait_for_errata_level(
		spi_local_t * local
		){

	if( local->o_flags & SPI_LOCAL_IS_ERRATA_REQUIRED ){
		u32 pio_level;
		u32 pio_mask;
		u32 target_level;
		pio_mask = 1 << local->ws_pin.pin;

		GPIO_TypeDef * gpio = hal_get_pio_regs(local->ws_pin.port);


		mcu_debug_log_info(MCU_DEBUG_DEVICE, "execute I2S slave errata on %d.%d", local->ws_pin.port, local->ws_pin.pin);

		//errata: http://www.st.com/content/ccc/resource/technical/document/errata_sheet/0a/98/58/84/86/b6/47/a2/DM00037591.pdf/files/DM00037591.pdf/jcr:content/translations/en.DM00037591.pdf

		//MSB Mode errata_required & (1<<0) == 0 -- wait until high then wait until low
		//I2S Mode errata_required & (1<<0) == 1 -- wait until low then wait until high

		target_level = (local->o_flags & SPI_LOCAL_IS_ERRATA_I2S) == 0; //MSB : 1, I2S: 0
		do {
			pio_level = (HAL_GPIO_ReadPin(gpio, pio_mask) == GPIO_PIN_SET);
		} while( pio_level != target_level );

		target_level = !target_level; //MSB : 0, I2S: 1
		do {
			pio_level = (HAL_GPIO_ReadPin(gpio, pio_mask) == GPIO_PIN_SET);
		} while( pio_level != target_level );
	}
}


void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
	HAL_I2S_TxHalfCpltCallback(hi2s);
	HAL_I2S_RxHalfCpltCallback(hi2s);
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s){
	HAL_I2S_TxCpltCallback(hi2s);
	HAL_I2S_RxCpltCallback(hi2s);
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
	//no action when half complete -- could fire an event
	spi_local_t * local = (spi_local_t *)hi2s;
	int result;
	devfs_async_t * async;

	async = local->transfer_handler.write;
	result = devfs_execute_write_handler(
				&local->transfer_handler,
				0,
				0,
				MCU_EVENT_FLAG_WRITE_COMPLETE | MCU_EVENT_FLAG_LOW);
	if( result ){
		local->transfer_handler.write = async;
	} else {
		//stop -- half transfer only happens on DMA
		if( local->o_flags & SPI_LOCAL_IS_DMA ){
			HAL_I2S_DMAPause(hi2s);
		}
	}
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s){
	spi_local_t * local = (spi_local_t *)hi2s;
	int result;
	devfs_async_t * async;

	async = local->transfer_handler.write;
	result = devfs_execute_write_handler(
				&local->transfer_handler,
				0,
				0, //zero means leave nbyte value alone
				MCU_EVENT_FLAG_HIGH | MCU_EVENT_FLAG_WRITE_COMPLETE);

	if( result ){
		local->transfer_handler.write = async;
	} else {
		//stop -- half transfer only happens on DMA
		if( local->o_flags & SPI_LOCAL_IS_DMA ){
			HAL_I2S_DMAPause(hi2s);
		}
	}
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
	spi_local_t * local = (spi_local_t *)hi2s;
	int result;
	devfs_async_t * async;

	async = local->transfer_handler.read;
	result = devfs_execute_read_handler(
				&local->transfer_handler,
				0,
				0,
				MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_LOW);

	if( result ){
		local->transfer_handler.read = async;
	} else {
		//stop -- half transfer only happens on DMA
		HAL_I2S_DMAPause(hi2s);
	}
}


void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s){
	spi_local_t * local = (spi_local_t *)hi2s;
	int result;
	devfs_async_t * async;

	async = local->transfer_handler.read;
	result = devfs_execute_read_handler(
				&local->transfer_handler,
				0,
				0,
				MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_HIGH);

	if( result ){
		//restore the callback if the callback requests it -- good for circular DMA only
		local->transfer_handler.read = async;
	} else if( local->o_flags & SPI_LOCAL_IS_DMA ){
		if( local->o_flags & SPI_LOCAL_IS_DMA ){
			HAL_I2S_DMAPause(hi2s);
		}
	}
}

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s){
	//called on overflow and underrun
	spi_local_t * spi = (spi_local_t *)hi2s;
	volatile u32 status = hi2s->Instance->SR;
#if defined STM32H7
	status = hi2s->Instance->UDRDR;
#else
	status = hi2s->Instance->DR;
#endif
	mcu_debug_log_error(
				MCU_DEBUG_DEVICE,
				"I2S Error %d on %p",
				hi2s->ErrorCode,
				hi2s->Instance
				);
	devfs_execute_cancel_handler(&spi->transfer_handler, (void*)&status, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_ERROR);
}


#endif

