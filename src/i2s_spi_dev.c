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

#include "spi_local.h"
#include <mcu/i2s.h>
#include <mcu/spi.h>

#if MCU_SPI_PORTS > 0

static int execute_handler(mcu_event_handler_t * handler, u32 o_events);

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(i2s_spi, I2S_VERSION, I_MCU_TOTAL + I_I2S_TOTAL, mcu_i2s_spi_mute, mcu_i2s_spi_unmute)

int mcu_i2s_spi_open(const devfs_handle_t * handle){
    u32 port = handle->port;
    if( port < MCU_SPI_PORTS ){

        //same as SPI
        if( mcu_spi_open(handle) < 0 ){
            return SYSFS_SET_RETURN(EINVAL);
        }

        //ensure Instance is correctly assigned
        if ( spi_local[port].ref_count == 0 ){
            spi_local[port].i2s_hal_handle.Instance = spi_regs[port];
        }
        return 0;
    }
    return SYSFS_SET_RETURN(EINVAL);
}

int mcu_i2s_spi_close(const devfs_handle_t * handle){
    //same as SPI
    mcu_spi_close(handle);
    return 0;
}

int mcu_i2s_spi_getinfo(const devfs_handle_t * handle, void * ctl){
    i2s_info_t * info = ctl;

    //set I2S Capability flags
    info->o_flags = 0;

    return 0;
}

int mcu_i2s_spi_mute(const devfs_handle_t * handle, void * ctl){
    return 0;
}

int mcu_i2s_spi_unmute(const devfs_handle_t * handle, void * ctl){
    return 0;
}

int mcu_i2s_spi_setattr(const devfs_handle_t * handle, void * ctl){
    int port = handle->port;


    const i2s_attr_t * attr = mcu_select_attr(handle, ctl);
    if( attr == 0 ){
        return SYSFS_SET_RETURN(EINVAL);
    }

    u32 o_flags = attr->o_flags;

    //set I2S Flags

    spi_local[port].is_i2s = 1;


    if( o_flags & (I2S_FLAG_SET_MASTER|I2S_FLAG_SET_SLAVE) ){
#if defined I2S_FULLDUPLEXMODE_ENABLE
        spi_local[port].i2s_hal_handle.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
#endif

        if( o_flags & I2S_FLAG_SET_SLAVE ){
            if( o_flags & I2S_FLAG_IS_TRANSMITTER ){
                spi_local[port].i2s_hal_handle.Init.Mode = I2S_MODE_SLAVE_TX;
                if( o_flags & I2S_FLAG_IS_RECEIVER ){
#if defined I2S_FULLDUPLEXMODE_ENABLE
                    spi_local[port].i2s_hal_handle.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
#endif
                }
            } else if ( o_flags & I2S_FLAG_IS_RECEIVER ){
                spi_local[port].i2s_hal_handle.Init.Mode = I2S_MODE_SLAVE_RX;
            }
        } else {
            if( o_flags & I2S_FLAG_IS_TRANSMITTER ){
                spi_local[port].i2s_hal_handle.Init.Mode = I2S_MODE_MASTER_TX;
                if( o_flags & I2S_FLAG_IS_RECEIVER ){
#if defined I2S_FULLDUPLEXMODE_ENABLE
                    spi_local[port].i2s_hal_handle.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
#endif
                }
            } else if ( o_flags & I2S_FLAG_IS_RECEIVER ){
                spi_local[port].i2s_hal_handle.Init.Mode = I2S_MODE_MASTER_RX;
            }
        }

        spi_local[port].i2s_hal_handle.Init.Standard = I2S_STANDARD_PHILIPS;
        if( o_flags & I2S_FLAG_IS_FORMAT_MSB ){
            spi_local[port].i2s_hal_handle.Init.Standard = I2S_STANDARD_MSB;
        } else if( o_flags & I2S_FLAG_IS_FORMAT_LSB ){
            spi_local[port].i2s_hal_handle.Init.Standard = I2S_STANDARD_LSB;
        } else if( o_flags & I2S_FLAG_IS_FORMAT_PCM_SHORT ){
            spi_local[port].i2s_hal_handle.Init.Standard = I2S_STANDARD_PCM_SHORT;
        } else if( o_flags & I2S_FLAG_IS_FORMAT_PCM_LONG ){
            spi_local[port].i2s_hal_handle.Init.Standard = I2S_STANDARD_PCM_LONG;
        }

        spi_local[port].i2s_hal_handle.Init.DataFormat = I2S_DATAFORMAT_16B;
        if( o_flags & I2S_FLAG_IS_WIDTH_16 ){
            spi_local[port].i2s_hal_handle.Init.DataFormat = I2S_DATAFORMAT_16B;
        } else if( o_flags & I2S_FLAG_IS_WIDTH_24 ){
            spi_local[port].i2s_hal_handle.Init.DataFormat = I2S_DATAFORMAT_24B;
        } else if( o_flags & I2S_FLAG_IS_WIDTH_32 ){
            spi_local[port].i2s_hal_handle.Init.DataFormat = I2S_DATAFORMAT_32B;
        }

        spi_local[port].i2s_hal_handle.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
        if( o_flags & I2S_FLAG_IS_MCK_ENABLED ){
            spi_local[port].i2s_hal_handle.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
        }

        spi_local[port].i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_8K;
        switch(attr->freq){
        case 11000: spi_local[port].i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_11K; break;
        case 16000: spi_local[port].i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_16K; break;
        case 22050: spi_local[port].i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_22K; break;
        case 32000: spi_local[port].i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_32K; break;
        case 44100: spi_local[port].i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_44K; break;
        case 48000: spi_local[port].i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_48K; break;
        case 96000: spi_local[port].i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_96K; break;
        case 192000: spi_local[port].i2s_hal_handle.Init.AudioFreq = I2S_AUDIOFREQ_192K; break;
        default:
            return SYSFS_SET_RETURN(EINVAL);
        }

        spi_local[port].i2s_hal_handle.Init.CPOL = I2S_CPOL_LOW;
        spi_local[port].i2s_hal_handle.Init.ClockSource = I2S_CLOCK_PLL;

        //this might be better implemented in the "core" driver for controlling the clocks
#if defined RCC_PERIPHCLK_I2S
        RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
        PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
        PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK){
            return SYSFS_SET_RETURN(EIO);
        }
#endif

        if( mcu_set_pin_assignment(
                    &(attr->pin_assignment),
                    MCU_CONFIG_PIN_ASSIGNMENT(i2s_config_t, handle),
                    MCU_PIN_ASSIGNMENT_COUNT(i2s_pin_assignment_t),
                    CORE_PERIPH_SPI, port, 0, 0, 0) < 0 ){
            return SYSFS_SET_RETURN(EINVAL);
        }

        if( HAL_I2S_Init(&spi_local[port].i2s_hal_handle) != HAL_OK ){
            return SYSFS_SET_RETURN(EIO);
        }
    }

    return 0;
}


int mcu_i2s_spi_setaction(const devfs_handle_t * handle, void * ctl){
    return mcu_spi_setaction(handle, ctl);
}

int mcu_i2s_spi_write(const devfs_handle_t * handle, devfs_async_t * async){
    int ret;
    int port = handle->port;

    //check to see if SPI bus is busy -- check to see if the interrupt is enabled?

    if( async->nbyte == 0 ){
        return 0;
    }

    spi_local[port].nbyte_ptr = &(async->nbyte);
    spi_local[port].handler = async->handler;
    ret = HAL_I2S_Transmit_IT(&spi_local[port].i2s_hal_handle, async->buf, async->nbyte);


    if( ret == HAL_BUSY ){
        return SYSFS_SET_RETURN(EIO);
    } else if( ret == HAL_ERROR ){
        return SYSFS_SET_RETURN(EIO);
    } else if( ret == HAL_TIMEOUT ){
        return SYSFS_SET_RETURN(EIO);
    } else if( ret != HAL_OK ){
        return SYSFS_SET_RETURN(EIO);
    }

    return 0;
}

int mcu_i2s_spi_read(const devfs_handle_t * handle, devfs_async_t * async){
    int ret;
    int port = handle->port;

    if( async->nbyte == 0 ){
        return 0;
    }

    spi_local[port].nbyte_ptr = &(async->nbyte);
    spi_local[port].handler = async->handler;
    ret = HAL_I2S_Receive_IT(&spi_local[port].i2s_hal_handle, async->buf, async->nbyte);
    if( ret == HAL_BUSY ){
        return SYSFS_SET_RETURN(EIO);
    } else if( ret == HAL_ERROR ){
        return SYSFS_SET_RETURN(EIO);
    } else if( ret == HAL_TIMEOUT ){
        return SYSFS_SET_RETURN(EIO);
    } else if( ret != HAL_OK ){
        return SYSFS_SET_RETURN(EIO);
    }

    return 0;
}

int execute_handler(mcu_event_handler_t * handler, u32 o_events){
    i2s_event_t event;
    event.value = 0;
    return mcu_execute_event_handler(handler, o_events, &event);
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s){}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s){
    spi_local_t * local = (spi_local_t *)hi2s;
    execute_handler(&local->handler, MCU_EVENT_FLAG_WRITE_COMPLETE);
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s){}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s){
    spi_local_t * local = (spi_local_t *)hi2s;
    execute_handler(&local->handler, MCU_EVENT_FLAG_DATA_READY);
}

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s){
    //called on overflow and underrun
    spi_local_t * local = (spi_local_t *)hi2s;
    execute_handler(&local->handler, MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR);
}


#endif

