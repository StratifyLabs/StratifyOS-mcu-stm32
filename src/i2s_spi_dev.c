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

#if MCU_SPI_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(i2s_spi, I2S_VERSION, I_MCU_TOTAL + I_I2S_TOTAL, mcu_i2s_spi_mute, mcu_i2s_spi_unmute)

int mcu_i2s_spi_open(const devfs_handle_t * handle){
    //make sure port supports I2S -- not all ports support I2S
    return mcu_spi_open(handle);
}

int mcu_i2s_spi_close(const devfs_handle_t * handle){
    //same as SPI
    return mcu_spi_close(handle);
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
    return i2s_spi_local_setattr(spi_local + handle->port, handle, ctl);
}


int mcu_i2s_spi_setaction(const devfs_handle_t * handle, void * ctl){
    return mcu_spi_setaction(handle, ctl);
}

int mcu_i2s_spi_write(const devfs_handle_t * handle, devfs_async_t * async){
    int ret;
    int port = handle->port;

    DEVFS_DRIVER_IS_BUSY(spi_local[port].transfer_handler.write, async);


    if( spi_local[port].is_full_duplex ){

        if( spi_local[port].transfer_handler.read->nbyte < async->nbyte ){
            return SYSFS_SET_RETURN(EINVAL);
        }

        ret = HAL_I2SEx_TransmitReceive_IT(
                    &spi_local[port].i2s_hal_handle,
                    async->buf,
                    spi_local[port].transfer_handler.read->buf,
                    async->nbyte);

    } else {
        ret = HAL_I2S_Transmit_IT(&spi_local[port].i2s_hal_handle, async->buf, async->nbyte);
    }


    if( ret != HAL_OK ){
        spi_local[port].transfer_handler.write = 0;
        return SYSFS_SET_RETURN_WITH_VALUE(EIO, ret);
    }

    return 0;
}

int mcu_i2s_spi_read(const devfs_handle_t * handle, devfs_async_t * async){
    int ret;
    int port = handle->port;

    DEVFS_DRIVER_IS_BUSY(spi_local[port].transfer_handler.read, async);

    if( spi_local[port].is_full_duplex ){
        ret = HAL_OK;
    } else {
        ret = HAL_I2S_Receive_IT(&spi_local[port].i2s_hal_handle, async->buf, async->nbyte);
    }

    if( ret != HAL_OK ){
        spi_local[port].transfer_handler.read = 0;
        return SYSFS_SET_RETURN_WITH_VALUE(EIO, ret);
    }

    return 0;
}


void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
   //no action when half complete -- could fire an event
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s){
    spi_local_t * spi = (spi_local_t *)hi2s;
    mcu_execute_write_handler(&spi->transfer_handler, 0, hi2s->TxXferSize);
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
    //no action when half complete -- could fire an event
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s){
    spi_local_t * spi = (spi_local_t *)hi2s;
    mcu_execute_read_handler(&spi->transfer_handler, 0, hi2s->RxXferSize);
}

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s){
    //called on overflow and underrun
    spi_local_t * spi = (spi_local_t *)hi2s;
    mcu_execute_transfer_handlers(&spi->transfer_handler, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR);
}

#endif

