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
#include <mcu/spi.h>

#if MCU_SAI_PORTS > 0


int sai_local_mute(const devfs_handle_t * handle, void * ctl){
    MCU_UNUSED_ARGUMENT(handle);
    MCU_UNUSED_ARGUMENT(ctl);
    //HAL_SAI_EnableTxMuteMode()
    //HAL_SAI_EnableRxMuteMode()
    return SYSFS_SET_RETURN(ENOTSUP);
}

int sai_local_unmute(const devfs_handle_t * handle, void * ctl){
    MCU_UNUSED_ARGUMENT(handle);
    MCU_UNUSED_ARGUMENT(ctl);
    //HAL_SAI_DisableRxMuteMode()
    //HAL_SAI_EnableTxMuteMode()
    return SYSFS_SET_RETURN(ENOTSUP);
}

int sai_local_setattr(sai_local_t * local, const devfs_handle_t * handle, void * ctl){
    int port = handle->port;
    const i2s_attr_t * attr = mcu_select_attr(handle, ctl);
    if( attr == 0 ){
        return SYSFS_SET_RETURN(EINVAL);
    }

    int is_errata_required = 0;
    u32 o_flags = attr->o_flags;

    //set I2S Flags

    if( o_flags & (I2S_FLAG_SET_MASTER|I2S_FLAG_SET_SLAVE) ){
        local->o_flags = SAI_LOCAL_IS_I2S;


        //local->hal_handle.Init.AudioMode = 0; //handled below
        local->hal_handle.Init.Synchro = SAI_SYNCHRONOUS;
        local->hal_handle.Init.SynchroExt = SAI_SYNCEXT_DISABLE;

        //this will probably be SAI_OUTPUTDRIVE_ENABLE to drive pins
        local->hal_handle.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;

        //SAI_MASTERDIVIDER_ENABLE
        local->hal_handle.Init.NoDivider = SAI_MASTERDIVIDER_DISABLE;

        //pick a value here that covers most cases
        //SAI_FIFOTHRESHOLD_EMPTY
        //SAI_FIFOTHRESHOLD_1QF
        //SAI_FIFOTHRESHOLD_HF
        //SAI_FIFOTHRESHOLD_3QF
        //SAI_FIFOTHRESHOLD_FULL
        local->hal_handle.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_HF;

        //local->hal_handle.Init.AudioFrequency = 0; //handled below
        //0 to 63 or 0 to 15 depending on the device
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
        //local->hal_handle.Init.Protocol = 0; //handled below
        //local->hal_handle.Init.DataSize = 0; //handled below

        //could also be SAI_FIRSTBIT_LSB
        local->hal_handle.Init.FirstBit = SAI_FIRSTBIT_MSB;

        //could also be SAI_CLOCKSTROBING_RISINGEDGE
        local->hal_handle.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;


        if( o_flags & I2S_FLAG_SET_SLAVE ){
            if( o_flags & I2S_FLAG_IS_TRANSMITTER ){
                local->hal_handle.Init.AudioMode = SAI_MODESLAVE_TX;
                if( o_flags & I2S_FLAG_IS_RECEIVER ){

                }
            } else if ( o_flags & I2S_FLAG_IS_RECEIVER ){
                local->hal_handle.Init.AudioMode = SAI_MODESLAVE_RX;
            }
        } else {
            if( o_flags & I2S_FLAG_IS_TRANSMITTER ){
                local->hal_handle.Init.AudioMode = SAI_MODEMASTER_TX;
                if( o_flags & I2S_FLAG_IS_RECEIVER ){
#if defined SPI_I2S_FULLDUPLEX_SUPPORT
                    local->hal_handle.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
                    local->o_flags |= SPI_LOCAL_IS_FULL_DUPLEX;
#endif
                }
            } else if ( o_flags & I2S_FLAG_IS_RECEIVER ){
                local->hal_handle.Init.AudioMode = SAI_MODEMASTER_RX;
            }
        }

        local->hal_handle.Init.Protocol = SAI_I2S_STANDARD;
        if( o_flags & I2S_FLAG_IS_FORMAT_MSB ){
            local->hal_handle.Init.Protocol = SAI_I2S_MSBJUSTIFIED;
        } else if( o_flags & I2S_FLAG_IS_FORMAT_LSB ){
            local->hal_handle.Init.Protocol = SAI_I2S_LSBJUSTIFIED;
        } else if( o_flags & I2S_FLAG_IS_FORMAT_PCM_SHORT ){
            local->hal_handle.Init.Protocol = SAI_PCM_SHORT;
        } else if( o_flags & I2S_FLAG_IS_FORMAT_PCM_LONG ){
            local->hal_handle.Init.Protocol = SAI_PCM_LONG;
        }

        local->hal_handle.Init.DataSize = SAI_PROTOCOL_DATASIZE_16BIT;
        local->size_mult = 2;
        if( o_flags & I2S_FLAG_IS_WIDTH_24 ){
            local->hal_handle.Init.DataSize = SAI_PROTOCOL_DATASIZE_24BIT;
            local->size_mult = 4;
        } else if( o_flags & I2S_FLAG_IS_WIDTH_32 ){
            local->hal_handle.Init.DataSize = SAI_PROTOCOL_DATASIZE_32BIT;
            local->size_mult = 4;
        } else if ( o_flags & I2S_FLAG_IS_WIDTH_16_EXTENDED ){
            local->hal_handle.Init.DataSize = SAI_PROTOCOL_DATASIZE_16BITEXTENDED;
        }

        local->hal_handle.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
        if( o_flags & I2S_FLAG_IS_MCK_ENABLED ){
            local->hal_handle.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
        }

        local->hal_handle.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_8K;
        switch(attr->freq){
        case 11000: local->hal_handle.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_11K; break;
        case 16000: local->hal_handle.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_16K; break;
        case 22050: local->hal_handle.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_22K; break;
        case 32000: local->hal_handle.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_32K; break;
        case 44100: local->hal_handle.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_44K; break;
        case 48000: local->hal_handle.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K; break;
        case 96000: local->hal_handle.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_96K; break;
        case 192000: local->hal_handle.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K; break;
        default:
            return SYSFS_SET_RETURN(EINVAL);
        }

        local->hal_handle.Init.CPOL = I2S_CPOL_LOW;
        local->hal_handle.Init.ClockSource = I2S_CLOCK_PLL;



        if( mcu_set_pin_assignment(
                    &(attr->pin_assignment),
                    MCU_CONFIG_PIN_ASSIGNMENT(i2s_config_t, handle),
                    MCU_PIN_ASSIGNMENT_COUNT(i2s_pin_assignment_t),
                    CORE_PERIPH_I2S, port, 0, 0, 0) < 0 ){
            return SYSFS_SET_RETURN(EINVAL);
        }

        if( HAL_SAI_InitProtocol(&local->hal_handle, 0, 0, 0) != HAL_OK ){
            return SYSFS_SET_RETURN(EIO);
        }
    }

    return 0;
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai){
    //no action when half complete -- could fire an event
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
        HAL_I2S_DMAStop(hsai);
    }
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai){
    sai_local_t * spi = (sai_local_t *)hsai;
    devfs_execute_write_handler(
                &local->transfer_handler,
                0,
                0, //zero means leave nbyte value alone
                MCU_EVENT_FLAG_WRITE_COMPLETE);
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai){
    //no action when half complete -- could fire an event
    sai_local_t * spi = (sai_local_t *)hsai;
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
        HAL_I2S_DMAStop(hsai);
    }
}


void HAL_I2S_RxCpltCallback(SAI_HandleTypeDef *hsai){
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
        HAL_I2S_DMAStop(hsai);
    }
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai){
    //called on overflow and underrun
    sai_local_t * spi = (sai_local_t *)hsai;
    volatile u32 status = hsai->Instance->SR;
    status = hsai->Instance->DR;
    mcu_debug_log_error(MCU_DEBUG_DEVICE, " I2S Error %d on %p", hsai->ErrorCode, hsai->Instance);
    devfs_execute_cancel_handler(&local->transfer_handler, (void*)&status, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_ERROR);
}


#endif

