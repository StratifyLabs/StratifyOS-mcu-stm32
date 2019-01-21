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
	spi_local_t * local = spi_local + handle->port;
	local->o_flags = SPI_LOCAL_IS_I2S;
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
	spi_local_t * local = spi_local + handle->port;
	if( attr == 0 ){ return SYSFS_SET_RETURN(EINVAL); }

	u32 o_flags = attr->o_flags;

	//set I2S Flags

	if( o_flags & (I2S_FLAG_SET_MASTER|I2S_FLAG_SET_SLAVE) ){
#if defined SPI_I2S_FULLDUPLEX_SUPPORT
		local->i2s_hal_handle.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
#endif


		if( o_flags & I2S_FLAG_SET_SLAVE ){
			local->o_flags |= SPI_LOCAL_IS_ERRATA_REQUIRED;
			if( o_flags & I2S_FLAG_IS_TRANSMITTER ){
				local->i2s_hal_handle.Init.Mode = I2S_MODE_SLAVE_TX;
				if( o_flags & I2S_FLAG_IS_RECEIVER ){
#if defined SPI_I2S_FULLDUPLEX_SUPPORT
					local->i2s_hal_handle.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
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
#if defined SPI_I2S_FULLDUPLEX_SUPPORT
					local->i2s_hal_handle.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
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

#if 0
		//this might be better implemented in the "core" driver for controlling the clocks
#if defined RCC_PERIPHCLK_I2S
		RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
		//PeriphClkInitStruct.PLLI2S.PLLI2SM = 4;
		PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
		PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
		int result;
		mcu_debug_log_info(MCU_DEBUG_DEVICE, "Start I2S Clock");
		if ( (result = HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct)) != HAL_OK){
			mcu_debug_log_error(MCU_DEBUG_DEVICE, "PERIPH CLOCK SET FAILED %d", result);
			return SYSFS_SET_RETURN(EIO);
		}
#endif

#if defined RCC_PERIPHCLK_I2S_APB1 && defined RCC_PERIPHCLK_I2S_APB2
		RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1|RCC_PERIPHCLK_I2S_APB2|RCC_PERIPHCLK_CLK48;
		PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
#if defined RCC_PLLI2SP_DIV2
		//PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLI2SP_DIV2;
#endif
		PeriphClkInitStruct.PLLI2S.PLLI2SM = 4;
		PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
		PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
		//PeriphClkInitStruct.PLLI2SDivQ = 1;
		PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
		PeriphClkInitStruct.I2sApb2ClockSelection = RCC_I2SAPB2CLKSOURCE_PLLI2S;
		PeriphClkInitStruct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
		int result;
		mcu_debug_log_info(MCU_DEBUG_DEVICE, "Start I2S Clock APB1/2");
		if ( (result = HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct)) != HAL_OK){
			mcu_debug_log_error(MCU_DEBUG_DEVICE, "PERIPH CLOCK SET FAILED %d", result);
			return SYSFS_SET_RETURN(EIO);
		}
#endif

#endif

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

		if( mcu_set_pin_assignment(
				 &(attr->pin_assignment),
				 MCU_CONFIG_PIN_ASSIGNMENT(i2s_config_t, handle),
				 MCU_PIN_ASSIGNMENT_COUNT(i2s_pin_assignment_t),
				 CORE_PERIPH_SPI, port, 0, 0, 0) < 0 ){
			return SYSFS_SET_RETURN(EINVAL);
		}

		int hal_result;
		if( (hal_result = HAL_I2S_Init(&local->i2s_hal_handle)) != HAL_OK ){
			mcu_debug_log_error(MCU_DEBUG_DEVICE, "Init I2S failed %d", hal_result);
			return SYSFS_SET_RETURN(EIO);
		}
	}

	return 0;
}

void i2s_spi_local_wait_for_errata_level(spi_local_t * local){

	if( local->o_flags & SPI_LOCAL_IS_ERRATA_REQUIRED ){
		devfs_handle_t pio_handle;
		u32 pio_level;
		u32 pio_value;
		u32 pio_mask;
		u32 target_level;
		pio_handle.port = local->ws_pin.port;
		pio_mask = 1 << local->ws_pin.pin;

		mcu_debug_log_info(MCU_DEBUG_DEVICE, "execute I2S slave errata on %d.%d", local->ws_pin.port, local->ws_pin.pin);

		//errata: http://www.st.com/content/ccc/resource/technical/document/errata_sheet/0a/98/58/84/86/b6/47/a2/DM00037591.pdf/files/DM00037591.pdf/jcr:content/translations/en.DM00037591.pdf

		//MSB Mode errata_required & (1<<0) == 0 -- wait until high then wait until low
		//I2S Mode errata_required & (1<<0) == 1 -- wait until low then wait until high

		target_level = (local->o_flags & SPI_LOCAL_IS_ERRATA_I2S) == 0; //MSB : 1, I2S: 0
		do {
			mcu_pio_get(&pio_handle, &pio_value);
			pio_level = (pio_value & pio_mask) != 0; //1 for set, 0 for not
		} while( pio_level != target_level );

		target_level = !target_level; //MSB : 0, I2S: 1
		do {
			mcu_pio_get(&pio_handle, &pio_value);
			pio_level = (pio_value & pio_mask) != 0; //1 for set, 0 for not
		} while( pio_level != target_level );
	}
}

//where is the half callback??
void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s){
	//spi_local_t * spi = (spi_local_t *)hi2s;


}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
	//no action when half complete -- could fire an event
	spi_local_t * spi = (spi_local_t *)hi2s;
	int result;
	devfs_async_t * async;
	async = spi->transfer_handler.write;
	result = devfs_execute_write_handler(
				&spi->transfer_handler,
				0,
				0,
				MCU_EVENT_FLAG_WRITE_COMPLETE | MCU_EVENT_FLAG_LOW);
	if( result ){
		spi->transfer_handler.write = async;
	} else {
		//stop -- half transfer only happens on DMA
		if( spi->o_flags & SPI_LOCAL_IS_DMA ){
			HAL_I2S_DMAStop(hi2s);
		}
	}
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s){
	spi_local_t * spi = (spi_local_t *)hi2s;
	int result;
	devfs_async_t * async;

	async = spi->transfer_handler.write;
	result = devfs_execute_write_handler(
				&spi->transfer_handler,
				0,
				0, //zero means leave nbyte value alone
				MCU_EVENT_FLAG_HIGH | MCU_EVENT_FLAG_WRITE_COMPLETE);

	if( result ){
		spi->transfer_handler.write = async;
	} else {
		//stop -- half transfer only happens on DMA
		if( spi->o_flags & SPI_LOCAL_IS_DMA ){
			HAL_I2S_DMAStop(hi2s);
		}
	}
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
	//no action when half complete -- could fire an event
	spi_local_t * spi = (spi_local_t *)hi2s;
	int result;
	devfs_async_t * async;

	async = spi->transfer_handler.read;
	result = devfs_execute_read_handler(
				&spi->transfer_handler,
				0,
				0,
				MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_LOW);

	if( result ){
		spi->transfer_handler.read = async;
	} else {
		//stop -- half transfer only happens on DMA
		HAL_I2S_DMAStop(hi2s);
	}
}


void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s){
	spi_local_t * spi = (spi_local_t *)hi2s;
	int result;
	devfs_async_t * async;

	async = spi->transfer_handler.read;
	result = devfs_execute_read_handler(
				&spi->transfer_handler,
				0,
				0,
				MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_HIGH);

	if( result ){
		//restore the callback if the callback requests it -- good for DMA only
		spi->transfer_handler.read = async;
	} else if( spi->o_flags & SPI_LOCAL_IS_DMA ){
		if( spi->o_flags & SPI_LOCAL_IS_DMA ){
			HAL_I2S_DMAStop(hi2s);
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
	mcu_debug_log_error(MCU_DEBUG_DEVICE, " I2S Error %d on %p", hi2s->ErrorCode, hi2s->Instance);
	devfs_execute_cancel_handler(&spi->transfer_handler, (void*)&status, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_ERROR);
}


#endif

