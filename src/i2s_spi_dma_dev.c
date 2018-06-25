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

#if MCU_SPI_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(i2s_spi, I2S_VERSION, I2S_IOC_IDENT_CHAR, I_MCU_TOTAL + I_I2S_TOTAL, mcu_i2s_spi_dma_mute, mcu_i2s_spi_dma_unmute)

int mcu_i2s_spi_dma_open(const devfs_handle_t * handle){
    u32 port = handle->port;
    if( port < MCU_SPI_PORTS ){

        //same as SPI
        if( mcu_spi_dma_open(handle) < 0 ){
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

int mcu_i2s_spi_dma_close(const devfs_handle_t * handle){
    //same as SPI
    mcu_spi_close(handle);
    return 0;
}

int mcu_i2s_spi_dma_getinfo(const devfs_handle_t * handle, void * ctl){
    i2s_info_t * info = ctl;

    //set I2S Capability flags
    info->o_flags = 0;


    return 0;
}

int mcu_i2s_spi_dma_mute(const devfs_handle_t * handle, void * ctl){
    return 0;
}

int mcu_i2s_spi_dma_unmute(const devfs_handle_t * handle, void * ctl){
    return 0;
}

int mcu_i2s_spi_dma_setattr(const devfs_handle_t * handle, void * ctl){

    //setup the DMA

    return i2s_spi_local_setattr(&spi_dma_local[handle->port].spi, handle, ctl);
}


int mcu_i2s_spi_dma_setaction(const devfs_handle_t * handle, void * ctl){
    return mcu_spi_dma_setaction(handle, ctl);
}

int mcu_i2s_spi_dma_write(const devfs_handle_t * handle, devfs_async_t * async){
    int ret;
    int port = handle->port;

    //check to see if SPI bus is busy -- check to see if the interrupt is enabled?

    if( async->nbyte == 0 ){
        return 0;
    }

    spi_dma_local[port].spi.transfer_handler.write = async;
    if( spi_dma_local[port].spi.is_full_duplex && spi_dma_local[port].spi.transfer_handler.read ){

#if defined I2S_FULLDUPLEXMODE_ENABLE
        if( spi_dma_local[port].spi.transfer_handler.read->nbyte < async->nbyte ){
            return SYSFS_SET_RETURN(EINVAL);
        }

        ret = HAL_I2SEx_TransmitReceive_DMA(
                    &spi_dma_local[port].spi.i2s_hal_handle,
                    async->buf,
                    spi_dma_local[port].spi.transfer_handler.read->buf,
                    async->nbyte);
#else
        return SYSFS_SET_RETURN(ENOTSUP);
#endif

    } else {
        ret = HAL_I2S_Transmit_DMA(&spi_dma_local[port].spi.i2s_hal_handle, async->buf, async->nbyte);
    }


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

int mcu_i2s_spi_dma_read(const devfs_handle_t * handle, devfs_async_t * async){
    int ret;
    int port = handle->port;

    if( async->nbyte == 0 ){
        return 0;
    }

    spi_local[port].transfer_handler.read = async;

    //Receive is going to
    if( spi_local[port].is_full_duplex ){
        ret = 0;
    } else {
        ret = HAL_I2S_Receive_DMA(&spi_dma_local[port].spi.i2s_hal_handle, async->buf, async->nbyte);
    }

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

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s){}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s){
    spi_local_t * local = (spi_local_t *)hi2s;
    mcu_execute_write_handler(&local->transfer_handler, 0, hi2s->TxXferSize);
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s){}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s){
    spi_local_t * local = (spi_local_t *)hi2s;
    mcu_execute_read_handler(&local->transfer_handler, 0, hi2s->TxXferSize);
}

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s){
    //called on overflow and underrun
    spi_local_t * local = (spi_local_t *)hi2s;

    mcu_execute_transfer_handlers(&local->transfer_handler, 0, SYSFS_SET_RETURN(EIO), hi2s->TxXferSize);
}


#endif

