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

ADC_TypeDef * const adc_regs_table[MCU_ADC_PORTS] = MCU_ADC_REGS;
u8 const adc_irqs[MCU_ADC_PORTS] = MCU_ADC_IRQS;
adc_local_t adc_local[MCU_ADC_PORTS] MCU_SYS_MEM;

const u32 adc_channels[MCU_ADC_CHANNELS] = {
	ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
	ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7,
	ADC_CHANNEL_8, ADC_CHANNEL_9, ADC_CHANNEL_10, ADC_CHANNEL_11,
	ADC_CHANNEL_12, ADC_CHANNEL_13, ADC_CHANNEL_14, ADC_CHANNEL_15,
	ADC_CHANNEL_16, ADC_CHANNEL_17, ADC_CHANNEL_18, ADC_CHANNEL_VREFINT,
	ADC_CHANNEL_VBAT
};

int adc_local_open(const devfs_handle_t * handle){
	const u32 port = handle->port;
	adc_local_t * local = adc_local + port;
	if ( local->ref_count == 0 ){

		local->hal_handle.Instance = adc_regs_table[port];

		switch(port){
			case 0:
#if defined __HAL_RCC_ADC1_CLK_ENABLE
				__HAL_RCC_ADC1_CLK_ENABLE();
#elif defined __HAL_RCC_ADC_CLK_ENABLE
				__HAL_RCC_ADC_CLK_ENABLE();
#elif defined __HAL_RCC_ADC12_CLK_ENABLE
				__HAL_RCC_ADC12_CLK_ENABLE();
#else
#error("__HAL_RCC_ADC_CLK_ENABLE is not defined")
#endif
				break;
#if defined __HAL_RCC_ADC2_CLK_ENABLE
			case 1:
				__HAL_RCC_ADC2_CLK_ENABLE();
#elif defined __HAL_RCC_ADC12_CLK_ENABLE
				__HAL_RCC_ADC12_CLK_ENABLE();
#else
#endif
				break;
#if defined __HAL_RCC_ADC3_CLK_ENABLE
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
		cortexm_enable_irq(adc_irqs[port]);
	}
	local->ref_count++;

	return 0;
}

int adc_local_close(const devfs_handle_t * handle){
	const u32 port = handle->port;
	adc_local_t * local = adc_local + port;

	if ( local->ref_count > 0 ){
		if ( local->ref_count == 1 ){
			cortexm_disable_irq(adc_irqs[port]);
			switch(port){
				case 0:
#if defined __HAL_RCC_ADC1_CLK_DISABLE
					__HAL_RCC_ADC1_CLK_DISABLE();
#elif defined __HAL_RCC_ADC_CLK_DISABLE
					__HAL_RCC_ADC_CLK_DISABLE();
#elif defined __HAL_RCC_ADC12_CLK_DISABLE
					__HAL_RCC_ADC12_CLK_DISABLE();
#else
#error("__HAL_RCC_ADC_CLK_DISABLE not defined")
#endif
					break;
				case 1:
#if defined __HAL_RCC_ADC2_CLK_DISABLE
					__HAL_RCC_ADC2_CLK_DISABLE();
#elif defined __HAL_RCC_ADC12_CLK_DISABLE
					__HAL_RCC_ADC12_CLK_DISABLE();
#endif
					break;
#if defined __HAL_RCC_ADC3_CLK_DISABLE
				case 2:
					__HAL_RCC_ADC3_CLK_DISABLE();
#endif
					break;
#if defined __HAL_RCC_ADC4_CLK_DISABLE
				case 3:
					__HAL_RCC_ADC4_CLK_DISABLE();
					break;
#endif
			}
			local->hal_handle.Instance = 0;
		}
		local->ref_count--;
	}
	return 0;
}


int adc_local_getinfo(const devfs_handle_t * handle, void * ctl){
	adc_info_t * info = ctl;
	const adc_config_t * config = handle->config;
	adc_local_t * local = adc_local + handle->port;

	info->o_flags = ADC_FLAG_IS_LEFT_JUSTIFIED |
			ADC_FLAG_IS_RIGHT_JUSTIFIED |
			ADC_FLAG_IS_TRIGGER_EINT |
			ADC_FLAG_IS_TRIGGER_TMR |
			ADC_FLAG_IS_TRIGGER_EINT_EDGE_FALLING |
			ADC_FLAG_IS_TRIGGER_EINT_EDGE_RISING |
			ADC_FLAG_SET_CONVERTER |
			ADC_FLAG_SET_CHANNELS;

	if( local->o_flags & ADC_LOCAL_IS_DMA ){
		info->o_flags |= ADC_FLAG_IS_GROUP | ADC_FLAG_IS_SCAN_MODE;
	}

	info->o_events = MCU_EVENT_FLAG_DATA_READY;
	info->maximum = 0xffff; //max value
	info->freq = 1000000; //max frequency
	info->bytes_per_sample = 2;

	if( config ){
		info->reference_mv = config->reference_mv;
	} else {
		info->reference_mv = 0;
	}


	return SYSFS_RETURN_SUCCESS;
}

int adc_local_setattr(const devfs_handle_t * handle, void * ctl){
	u32 o_flags;
	int port = handle->port;
	adc_local_t * local = adc_local + port;
	const adc_attr_t * attr;

	attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

	o_flags = attr->o_flags;

	if( o_flags & ADC_FLAG_SET_CHANNELS ){
		//pin assignments
		if( mcu_set_pin_assignment(
				 &(attr->pin_assignment),
				 MCU_CONFIG_PIN_ASSIGNMENT(adc_config_t, handle),
				 MCU_PIN_ASSIGNMENT_COUNT(adc_pin_assignment_t),
				 CORE_PERIPH_ADC, port, 0, 0, 0) < 0 ){
			return SYSFS_SET_RETURN(EINVAL);
		}
	}

	if( o_flags & ADC_FLAG_SET_CONVERTER ){

#if MCU_ADC_API == 0
		local->hal_handle.Init.DMAContinuousRequests = DISABLE;
#else

#endif
		local->hal_handle.Init.ContinuousConvMode = DISABLE;
		local->hal_handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
		//scan mode only works with DMA (see adc_dma_dev.c for options)
		local->hal_handle.Init.ScanConvMode = DISABLE;
		//don't support discontinuous conversions
		//ENABLE or DISABLE
		local->hal_handle.Init.DiscontinuousConvMode = DISABLE;
		local->hal_handle.Init.NbrOfDiscConversion = 0;

		if(local->o_flags & ADC_LOCAL_IS_DMA ){
			mcu_debug_log_info(MCU_DEBUG_DEVICE, "Set ADC DMA Converter");
			const stm32_adc_dma_config_t * config;
			config = handle->config;
			if( config == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

#if MCU_ADC_API == 0
			//ENABLE or DISABLE (if ENABLE DMA must be in circular buffer mode)
			if( config->dma_config.o_flags & STM32_DMA_FLAG_IS_CIRCULAR ){
				local->hal_handle.Init.DMAContinuousRequests = ENABLE;
			}

			//ADC_DATAALIGN_RIGHT
			//ADC_DATAALIGN_LEFT
			local->hal_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
			if( o_flags & ADC_FLAG_IS_LEFT_JUSTIFIED ){
				local->hal_handle.Init.DataAlign = ADC_DATAALIGN_LEFT;
			}
#else

#endif
			if( o_flags & ADC_FLAG_IS_SCAN_MODE ){
				//up to 1 to 16 conversions
				if( attr->channel_count && attr->channel_count <= 16 ){
					local->hal_handle.Init.NbrOfConversion = attr->channel_count;
				} else {
					return SYSFS_SET_RETURN(EINVAL);
				}
			} else {
				//scan mode is always on for DMA based operations
				return SYSFS_SET_RETURN(EINVAL);
			}

			//use continous mode -- convert the entire sequence (not just one then stop)
			//ENABLE or DISABLE
			if( o_flags & ADC_FLAG_IS_CONTINOUS_CONVERSION ){
				local->hal_handle.Init.ContinuousConvMode = ENABLE;
			}

			//ADC_EOC_SEQ_CONV
			//ADC_EOC_SINGLE_CONV
			//ADC_EOC_SINGLE_SEQ_CONV
			local->hal_handle.Init.EOCSelection = ADC_EOC_SEQ_CONV; //DMA needs to use end of sequence
			local->hal_handle.Init.ScanConvMode = ENABLE; //always do scan mode with DMA
		}

		local->hal_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; //set based on the frequency

		//ADC_RESOLUTION_12B
		//ADC_RESOLUTION_10B
		//ADC_RESOLUTION_8B
		//ADC_RESOLUTION_6B
		local->hal_handle.Init.Resolution = ADC_RESOLUTION_12B; //default is max resolution
#if defined ADC_RESOLUTION_6B
		if( attr->width == 6 ){
			local->hal_handle.Init.Resolution = ADC_RESOLUTION_6B;
		} else
#endif
			if( attr->width == 8 ){
				local->hal_handle.Init.Resolution = ADC_RESOLUTION_8B;
			} else if( attr->width == 10 ){
				local->hal_handle.Init.Resolution = ADC_RESOLUTION_10B;
			}
#if defined ADC_RESOLUTION_14B
			else if( attr->width == 14 ){
				local->hal_handle.Init.Resolution = ADC_RESOLUTION_14B;
			}
#endif
#if defined ADC_RESOLUTION_16B
			else if( attr->width == 16 ){
				local->hal_handle.Init.Resolution = ADC_RESOLUTION_16B;
			}
#endif

		//ADC_SOFTWARE_START
		//ADC_EXTERNALTRIGCONV_T1_CC1 0.0
		//ADC_EXTERNALTRIGCONV_T1_CC2 0.1
		//ADC_EXTERNALTRIGCONV_T1_CC3 0.2
		//ADC_EXTERNALTRIGCONV_T2_CC2 1.1
		//ADC_EXTERNALTRIGCONV_T2_CC3 1.2
		//ADC_EXTERNALTRIGCONV_T2_CC4 1.3
		//ADC_EXTERNALTRIGCONV_T2_TRGO
		//ADC_EXTERNALTRIGCONV_T3_CC1
		//ADC_EXTERNALTRIGCONV_T3_TRGO
		//ADC_EXTERNALTRIGCONV_T4_CC4
		//ADC_EXTERNALTRIGCONV_T5_CC1
		//ADC_EXTERNALTRIGCONV_T5_CC2
		//ADC_EXTERNALTRIGCONV_T5_CC3
		//ADC_EXTERNALTRIGCONV_T8_CC1
		//ADC_EXTERNALTRIGCONV_T8_TRGO
		//ADC_EXTERNALTRIGCONV_Ext_IT11
		local->hal_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
		local->hal_handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
#if !defined STM32L4
		if( o_flags & ADC_FLAG_IS_TRIGGER_TMR ){
			mcu_debug_log_info(MCU_DEBUG_DEVICE, "tmr trigger:%d.%d", attr->trigger.port, attr->trigger.pin);
			local->hal_handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
			if( attr->trigger.port == 0 ){
				switch(attr->trigger.pin){
#if defined ADC_EXTERNALTRIGCONV_T1_CC1
					case 1: local->hal_handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1; break;
#endif
#if defined ADC_EXTERNALTRIGCONV_T1_CC2
					case 2: local->hal_handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC2; break;
#endif
#if defined ADC_EXTERNALTRIGCONV_T1_CC3
					case 3: local->hal_handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC3; break;
#endif
					default: return SYSFS_SET_RETURN(EINVAL);
				}
#if defined ADC_EXTERNALTRIGCONV_T2_TRGO
			} else if( attr->trigger.port == 1 ){
				switch(attr->trigger.pin){
					case 0: local->hal_handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO; break;
					case 2: local->hal_handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC2; break;
#if defined ADC_EXTERNALTRIGCONV_T2_CC3
					case 3: local->hal_handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC3; break;
					case 4: local->hal_handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC4; break;
#endif
					default: return SYSFS_SET_RETURN(EINVAL);
				}
#endif
			} else if( attr->trigger.port == 2 ){
#if defined ADC_EXTERNALTRIGCONV_T3_TRGO
				switch(attr->trigger.pin){
					case 0: local->hal_handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO; break;
					case 1: local->hal_handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_CC1; break;
					default: return SYSFS_SET_RETURN(EINVAL);
				}
#else
				return SYSFS_SET_RETURN(EINVAL);
#endif
			} else if( attr->trigger.port == 3 ){
				switch(attr->trigger.pin){
#if defined ADC_EXTERNALTRIGCONV_T4_CC4
					case 4: local->hal_handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T4_CC4; break;
#endif
					default: return SYSFS_SET_RETURN(EINVAL);
				}
#if defined ADC_EXTERNALTRIGCONV_T5_CC1
			} else if( attr->trigger.port == 4 ){
				switch(attr->trigger.pin){
					case 1: local->hal_handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T5_CC1; break;
					case 2: local->hal_handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T5_CC2; break;
					case 3: local->hal_handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T5_CC3; break;
					default: return SYSFS_SET_RETURN(EINVAL);
				}
#endif
			} else if( attr->trigger.port == 7 ){
				switch(attr->trigger.pin){
#if defined ADC_EXTERNALTRIGCONV_T8_TRGO
					case 0: local->hal_handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO; break;
#endif
#if defined ADC_EXTERNALTRIGCONV_T8_CC1
					case 1: local->hal_handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_CC1; break;
#endif
					default: return SYSFS_SET_RETURN(EINVAL);
				}
			} else {
				return SYSFS_SET_RETURN(EINVAL);
			}
		} else
#endif
			if( o_flags & ADC_FLAG_IS_TRIGGER_EINT ){
#if defined ADC_EXTERNALTRIGCONV_Ext_IT11
				local->hal_handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_Ext_IT11;
#elif defined ADC_EXTERNALTRIGCONV_EXT_IT11
				local->hal_handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_EXT_IT11;
#endif

				//ADC_EXTERNALTRIGCONVEDGE_NONE
				//ADC_EXTERNALTRIGCONVEDGE_RISING
				//ADC_EXTERNALTRIGCONVEDGE_FALLING
				//ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING
				if( attr->o_flags & ADC_FLAG_IS_TRIGGER_EINT_EDGE_RISING ){
					if( attr->o_flags & ADC_FLAG_IS_TRIGGER_EINT_EDGE_FALLING ){
						local->hal_handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
					} else {
						local->hal_handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
					}
				} else if( attr->o_flags & ADC_FLAG_IS_TRIGGER_EINT_EDGE_FALLING ){
					local->hal_handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
				}
			}

		mcu_debug_log_info(MCU_DEBUG_DEVICE, "ADC Trig: 0x%X", local->hal_handle.Init.ExternalTrigConv);
		if( HAL_ADC_Init(&local->hal_handle) < 0 ){
			return SYSFS_SET_RETURN(EIO);
		}
	}

	if( (o_flags & ADC_FLAG_SET_CHANNELS) && (o_flags & ADC_FLAG_IS_GROUP) ){

		ADC_ChannelConfTypeDef channel_config;
		channel_config.Channel = 0;
		if( attr->channel < MCU_ADC_CHANNELS ){
			channel_config.Channel = adc_channels[attr->channel];
		} else {
			return SYSFS_SET_RETURN(EINVAL);
		}
		channel_config.Offset = 0;
		channel_config.Rank = attr->rank;
		if( channel_config.Rank < 1 ){ channel_config.Rank = 1; }
		if( channel_config.Rank > 16 ){ channel_config.Rank = 16;  }

#if defined STM32L4

#else
#if MCU_ADC_API == 0
		channel_config.SamplingTime = ADC_SAMPLETIME_3CYCLES;
		if( attr->sampling_time >= 480 ){
			channel_config.SamplingTime = ADC_SAMPLETIME_480CYCLES;
		} else if( attr->sampling_time >= 144 ){
			channel_config.SamplingTime = ADC_SAMPLETIME_144CYCLES;
		} else if( attr->sampling_time >= 112 ){
			channel_config.SamplingTime = ADC_SAMPLETIME_112CYCLES;
		} else if( attr->sampling_time >= 84 ){
			channel_config.SamplingTime = ADC_SAMPLETIME_84CYCLES;
		} else if( attr->sampling_time >= 56 ){
			channel_config.SamplingTime = ADC_SAMPLETIME_56CYCLES;
		} else if( attr->sampling_time >= 28 ){
			channel_config.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		} else if( attr->sampling_time >= 15 ){
			channel_config.SamplingTime = ADC_SAMPLETIME_15CYCLES;
		}
#else
		channel_config.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
		if( attr->sampling_time >= 810 ){
			channel_config.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
		} else if( attr->sampling_time >= 387 ){
			channel_config.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
		} else if( attr->sampling_time >= 64 ){
			channel_config.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
		} else if( attr->sampling_time >= 32 ){
			channel_config.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
		} else if( attr->sampling_time >= 16 ){
			channel_config.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;
		} else if( attr->sampling_time >= 8 ){
			channel_config.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
		} else if( attr->sampling_time >= 2 ){
			channel_config.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
		}
#endif
#endif

		if( HAL_ADC_ConfigChannel(&local->hal_handle, &channel_config) != HAL_OK ){
			return SYSFS_SET_RETURN(EIO);
		}
	}


	return SYSFS_RETURN_SUCCESS;
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc){
	//this could be used to set a custom event when the ADC is out of a window
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc){
	adc_local_t * adc = (adc_local_t*)hadc;
	mcu_debug_log_error(MCU_DEBUG_DEVICE, "ADC Error %d", hadc->ErrorCode);
#if defined ADC_SR_OVR
	hadc->Instance->SR &= ~ADC_SR_OVR;
#endif
	devfs_execute_read_handler(&adc->transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR);
	if( (adc->o_flags & ADC_LOCAL_IS_DMA) == 0 ){
		HAL_ADC_Stop_IT(hadc);
	} else {
		HAL_ADC_Stop_DMA(hadc);
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef * hadc){
	//this is DMA only
	//mcu_debug_printf("h\n");
	adc_local_t * local = (adc_local_t *)hadc;
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
		HAL_ADC_Stop_DMA(hadc);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc){
	adc_local_t * local = (adc_local_t*)hadc;
	//mcu_debug_printf("f\n");

	if( local->o_flags & ADC_LOCAL_IS_DMA ){
		adc_local_t * local = (adc_local_t *)hadc;
		int result;
		devfs_async_t * async;

		async = local->transfer_handler.read;
		result = devfs_execute_read_handler(
					&local->transfer_handler,
					0,
					0,
					MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_HIGH);
		if( result ){
			local->transfer_handler.read = async;
		} else {
			//stop -- half transfer only happens on DMA
			HAL_ADC_Stop_DMA(hadc);
		}

	} else {
		devfs_async_t * read_async = local->transfer_handler.read;
		if( read_async ){
			//write to buffer

			u16 * dest = read_async->buf;
			dest[local->words_read] = HAL_ADC_GetValue(hadc);
			local->words_read++;
			if( local->words_read * 2 == read_async->nbyte ){
				HAL_ADC_Stop_IT(hadc); //only needed for non software triggers
				devfs_execute_read_handler(&local->transfer_handler, 0, read_async->nbyte, MCU_EVENT_FLAG_DATA_READY);
			} else {
				HAL_ADC_Start_IT(hadc);
				return;
			}
		}
	}
}

void mcu_core_adc_isr(){
	if( adc_local[0].hal_handle.Instance ){ HAL_ADC_IRQHandler(&adc_local[0].hal_handle); }
#if MCU_ADC_PORTS > 1
	if( adc_local[1].hal_handle.Instance ){ HAL_ADC_IRQHandler(&adc_local[1].hal_handle); }
#endif
#if MCU_ADC_PORTS > 2
	if( adc_local[2].hal_handle.Instance ){ HAL_ADC_IRQHandler(&adc_local[2].hal_handle); }
#endif
}


#endif
