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

#include <mcu/i2s.h>
#include "sai_local.h"


#if MCU_SAI_PORTS > 0


sai_local_t * sai_local_ptrs[MCU_SAI_PORTS] MCU_SYS_MEM;

SAI_Block_TypeDef * const sai_regs[MCU_SAI_PORTS] = MCU_SAI_REGS;
u8 const sai_irqs[MCU_SAI_PORTS] = MCU_SAI_IRQS;

int sai_local_open(sai_local_t * local, const devfs_handle_t * handle){
	const u32 port = handle->port;
	if( port < MCU_SAI_PORTS ){
		if ( local->ref_count == 0 ){
			//turn on RCC clock
			switch(port){
				case 0:
				case 1:
					__HAL_RCC_SAI1_CLK_ENABLE();
					break;
#if defined SAI2
				case 2:
				case 3:
					mcu_debug_log_info(MCU_DEBUG_DEVICE, "Turn on SAI2 clock");
					__HAL_RCC_SAI2_CLK_ENABLE();
					break;
#endif
			}
			sai_local_ptrs[port] = local;
			local->transfer_handler.read = NULL;
			local->transfer_handler.write = NULL;
			local->hal_handle.Instance = sai_regs[port];
			cortexm_enable_irq( sai_irqs[port] );
		}
		local->ref_count++;
		return 0;
	}

	return SYSFS_SET_RETURN(EINVAL);
}

int sai_local_close(sai_local_t * local, const devfs_handle_t * handle){
	const u32 port = handle->port;
	if ( local->ref_count > 0 ){
		if ( local->ref_count == 1 ){

			HAL_SAI_DeInit(&local->hal_handle);
			cortexm_disable_irq(sai_irqs[port]);
			devfs_execute_cancel_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EDEADLK), MCU_EVENT_FLAG_CANCELED);


			//turn off RCC clock
			switch(port){
				case 0:
				case 1:
					__HAL_RCC_SAI1_CLK_DISABLE();
					break;
#if defined SAI2
				case 2:
				case 3:
					__HAL_RCC_SAI2_CLK_DISABLE();
					break;
#endif
			}
		}
		local->ref_count--;
	}
	return 0;
}



int sai_local_mute(sai_local_t * local, const devfs_handle_t * handle, void * ctl){
	MCU_UNUSED_ARGUMENT(handle);
	MCU_UNUSED_ARGUMENT(ctl);
	//HAL_SAI_EnableTxMuteMode()
	//HAL_SAI_EnableRxMuteMode()
	return SYSFS_SET_RETURN(ENOTSUP);
}

int sai_local_unmute(sai_local_t * local, const devfs_handle_t * handle, void * ctl){
	MCU_UNUSED_ARGUMENT(handle);
	MCU_UNUSED_ARGUMENT(ctl);
	//HAL_SAI_DisableRxMuteMode()
	//HAL_SAI_DisableTxMuteMode()
	return SYSFS_SET_RETURN(ENOTSUP);
}

int sai_local_setattr(sai_local_t * local, const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	const i2s_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){
		return SYSFS_SET_RETURN(EINVAL);
	}

	u32 o_flags = attr->o_flags;

	//set I2S Flags

	if( o_flags & (I2S_FLAG_SET_MASTER|I2S_FLAG_SET_SLAVE) ){
		local->o_flags = SAI_LOCAL_IS_I2S;

		//local->hal_handle.Init.AudioMode = 0; //handled below
		if( o_flags & I2S_FLAG_SET_SLAVE ){
			local->hal_handle.Init.AudioMode = SAI_MODESLAVE_TX;
			if ( o_flags & I2S_FLAG_IS_RECEIVER ){
				local->hal_handle.Init.AudioMode = SAI_MODESLAVE_RX;
			}
		} else {
			local->hal_handle.Init.AudioMode = SAI_MODEMASTER_TX;
			if ( o_flags & I2S_FLAG_IS_RECEIVER ){
				local->hal_handle.Init.AudioMode = SAI_MODEMASTER_RX;
			}
		}


		/*
			* Slave mode synchronous -- receives signals from another SAI block
			*   asynchronous -- receives signals from external pins
			*
			* For master mode, this value doesn't seem to matter
			*
			*/
		local->hal_handle.Init.Synchro = SAI_ASYNCHRONOUS; //synchronous means it should receive signals from another SAI unit or sub block

		//SAI_SYNCEXT_OUTBLOCKA_ENABLE -- sync with block A of other SAI unit
		//SAI_SYNCEXT_OUTBLOCKB_ENABLE
		local->hal_handle.Init.SynchroExt = SAI_SYNCEXT_DISABLE; //this means synchronize with another SAI unit (not a sub block in teh same unit)

		//this will probably be SAI_OUTPUTDRIVE_ENABLE to drive pins
		local->hal_handle.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;

		//SAI_MASTERDIVIDER_ENABLE
		local->hal_handle.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;

		//pick a value here that covers most cases
		//SAI_FIFOTHRESHOLD_EMPTY
		//SAI_FIFOTHRESHOLD_1QF
		//SAI_FIFOTHRESHOLD_HF
		//SAI_FIFOTHRESHOLD_3QF
		//SAI_FIFOTHRESHOLD_FULL
		local->hal_handle.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_HF;

		u32 frequency = SAI_AUDIO_FREQUENCY_8K;
		switch(attr->freq){
			case 11000: frequency = SAI_AUDIO_FREQUENCY_11K; break;
			case 16000: frequency = SAI_AUDIO_FREQUENCY_16K; break;
			case 22050: frequency = SAI_AUDIO_FREQUENCY_22K; break;
			case 32000: frequency = SAI_AUDIO_FREQUENCY_32K; break;
			case 44100: frequency = SAI_AUDIO_FREQUENCY_44K; break;
			case 48000: frequency = SAI_AUDIO_FREQUENCY_48K; break;
			case 96000: frequency = SAI_AUDIO_FREQUENCY_96K; break;
			case 192000: frequency = SAI_AUDIO_FREQUENCY_192K; break;
			default:
				return SYSFS_SET_RETURN(EINVAL);
		}

		local->hal_handle.Init.AudioFrequency = frequency;

		//local->hal_handle.Init.AudioFrequency = 0; //handled below
		//0 to 63 or 0 to 15 depending on the device
		//this value is set based on the call to the HAL libraries and the audio frequency
		local->hal_handle.Init.Mckdiv = 0;

#if defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)
		local->hal_handle.Init.MckOverSampling = 0;
		local->hal_handle.Init.PdmInit = 0;
#endif

		local->hal_handle.Init.MonoStereoMode = SAI_STEREOMODE;
		if( o_flags & I2S_FLAG_IS_MONO){
			local->hal_handle.Init.MonoStereoMode = SAI_MONOMODE;
		}

		//SAI_ULAW_1CPL_COMPANDING
		//SAI_ALAW_1CPL_COMPANDING
		//SAI_ULAW_2CPL_COMPANDING
		//SAI_ALAW_2CPL_COMPANDING
		local->hal_handle.Init.CompandingMode = SAI_NOCOMPANDING;

		//OR SAI_OUTPUT_RELEASED
		local->hal_handle.Init.TriState = SAI_OUTPUT_NOTRELEASED;

		//slot
		u32 slot_count = 2;
		u32 protocol = SAI_I2S_STANDARD;
		if( o_flags & I2S_FLAG_IS_FORMAT_MSB ){
			protocol = SAI_I2S_MSBJUSTIFIED;
		} else if( o_flags & I2S_FLAG_IS_FORMAT_LSB ){
			protocol = SAI_I2S_LSBJUSTIFIED;
		} else if( o_flags & I2S_FLAG_IS_FORMAT_PCM_SHORT ){
			protocol = SAI_PCM_SHORT;
		} else if( o_flags & I2S_FLAG_IS_FORMAT_PCM_LONG ){
			protocol = SAI_PCM_LONG;
		}

		u32 data_size = SAI_PROTOCOL_DATASIZE_16BIT;
		local->size_mult = 2;
		if( o_flags & I2S_FLAG_IS_WIDTH_24 ){
			data_size = SAI_PROTOCOL_DATASIZE_24BIT;
			local->size_mult = 4;
		} else if( o_flags & I2S_FLAG_IS_WIDTH_32 ){
			data_size = SAI_PROTOCOL_DATASIZE_32BIT;
			local->size_mult = 4;
		} else if ( o_flags & I2S_FLAG_IS_WIDTH_16_EXTENDED ){
			data_size = SAI_PROTOCOL_DATASIZE_16BITEXTENDED;
		}

		if( mcu_set_pin_assignment(
				 &(attr->pin_assignment),
				 MCU_CONFIG_PIN_ASSIGNMENT(i2s_config_t, handle),
				 MCU_PIN_ASSIGNMENT_COUNT(i2s_pin_assignment_t),
				 CORE_PERIPH_I2S, port, 0, 0, 0) < 0 ){
			return SYSFS_SET_RETURN(EINVAL);
		}


#if 0
		//local->hal_handle.Instance = SAI2_Block_A;
		local->hal_handle.Init.Protocol = SAI_FREE_PROTOCOL;
		local->hal_handle.Init.AudioMode = SAI_MODEMASTER_RX;
		local->hal_handle.Init.DataSize = SAI_DATASIZE_16;
		local->hal_handle.Init.FirstBit = SAI_FIRSTBIT_MSB;
		local->hal_handle.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
		local->hal_handle.Init.Synchro = SAI_ASYNCHRONOUS;
		local->hal_handle.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
		local->hal_handle.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
		local->hal_handle.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
		local->hal_handle.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_16K;
		local->hal_handle.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
		local->hal_handle.Init.MonoStereoMode = SAI_STEREOMODE;
		local->hal_handle.Init.CompandingMode = SAI_NOCOMPANDING;
		local->hal_handle.FrameInit.FrameLength = 16;
		local->hal_handle.FrameInit.ActiveFrameLength = 1;
		local->hal_handle.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
		local->hal_handle.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
		local->hal_handle.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
		local->hal_handle.SlotInit.FirstBitOffset = 0;
		local->hal_handle.SlotInit.SlotSize = SAI_SLOTSIZE_16B;
		local->hal_handle.SlotInit.SlotNumber = 1;
		local->hal_handle.SlotInit.SlotActive = 0x00000001;
		int sai_result;
		if ( (sai_result = HAL_SAI_Init(&local->hal_handle)) != HAL_OK){
			mcu_debug_log_error(MCU_DEBUG_DEVICE, "Failed to init SAI %d", sai_result);
			return SYSFS_SET_RETURN(EIO);
		}
#else
		mcu_debug_log_info(MCU_DEBUG_DEVICE, "SAI: %d %d %d", protocol, data_size, slot_count);
		int sai_result;
		if( (sai_result = HAL_SAI_InitProtocol(&local->hal_handle, protocol, data_size, slot_count)) != HAL_OK ){
			mcu_debug_log_error(MCU_DEBUG_DEVICE, "Failed to init SAI %d", sai_result);
			return SYSFS_SET_RETURN(EIO);
		}
#endif
	}

	return SYSFS_RETURN_SUCCESS;
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai){
	sai_local_t * local = (sai_local_t *)hsai;
	int result;
	devfs_async_t * async;

	async = local->transfer_handler.write;
	result = devfs_execute_write_handler(
				&local->transfer_handler,
				0,
				0,
				MCU_EVENT_FLAG_WRITE_COMPLETE | MCU_EVENT_FLAG_LOW);
	if( result ){
		local->transfer_handler.read = async;
	} else {
		//stop -- half transfer only happens on DMA
		HAL_SAI_DMAStop(hsai);
	}
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai){
	sai_local_t * local = (sai_local_t *)hsai;

	int result;
	devfs_async_t * async;
	async = local->transfer_handler.write;

	result = devfs_execute_write_handler(
				&local->transfer_handler,
				0,
				0, //zero means leave nbyte value alone
				MCU_EVENT_FLAG_WRITE_COMPLETE | MCU_EVENT_FLAG_HIGH);

	if( result ){
		local->transfer_handler.write = async;
	} else {
		//stop -- half transfer only happens on DMA
		if( local->o_flags & SAI_LOCAL_IS_DMA ){
			HAL_SAI_DMAStop(hsai);
		}
	}
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai){
	//no action when half complete -- could fire an event
	sai_local_t * local = (sai_local_t *)hsai;
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
		HAL_SAI_DMAStop(hsai);
	}
}


void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai){
	sai_local_t * local = (sai_local_t *)hsai;
	int result;
	devfs_async_t * async;

	async = local->transfer_handler.read;
	result = devfs_execute_read_handler(
				&local->transfer_handler,
				0,
				0,
				MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_HIGH);

	if( result ){
		//restore the callback if the callback requests it -- good for DMA only
		local->transfer_handler.read = async;
	} else if( local->o_flags & SAI_LOCAL_IS_DMA ){
		if( local->o_flags & SAI_LOCAL_IS_DMA ){
			HAL_SAI_DMAStop(hsai);
		}
	}
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai){
	//called on overflow and underrun
	sai_local_t * local = (sai_local_t *)hsai;
	volatile u32 status = hsai->Instance->SR;
	status = hsai->Instance->DR;
	mcu_debug_log_error(MCU_DEBUG_DEVICE, "SAI Error %d on %p", hsai->ErrorCode, hsai->Instance);
	devfs_execute_cancel_handler(&local->transfer_handler, (void*)&status, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_ERROR);
}

#if defined SAI1
void mcu_core_sai1_isr(){
	mcu_debug_log_info(MCU_DEBUG_DEVICE, "SAI 1 interrupt");
	if( sai_local_ptrs[0] ){ HAL_SAI_IRQHandler(&sai_local_ptrs[0]->hal_handle); }
	if( sai_local_ptrs[1] ){ HAL_SAI_IRQHandler(&sai_local_ptrs[1]->hal_handle); }

}
#endif

#if defined SAI2
void mcu_core_sai2_isr(){
	mcu_debug_log_info(MCU_DEBUG_DEVICE, "SAI 2 interrupt");
	if( sai_local_ptrs[2] ){ HAL_SAI_IRQHandler(&sai_local_ptrs[2]->hal_handle); }
	if( sai_local_ptrs[3] ){ HAL_SAI_IRQHandler(&sai_local_ptrs[3]->hal_handle); }
}
#endif


#endif

