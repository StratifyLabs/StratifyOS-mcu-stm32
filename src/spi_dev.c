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
#include <mcu/spi.h>


#if MCU_SPI_PORTS > 0

spi_local_t spi_local[MCU_SPI_PORTS] MCU_SYS_MEM;

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(spi, SPI_VERSION, SPI_IOC_IDENT_CHAR, I_MCU_TOTAL + I_SPI_TOTAL, mcu_spi_swap)

int mcu_spi_open(const devfs_handle_t * handle){
    return spi_local_open(spi_local + handle->port, handle, spi_irqs[handle->port]);
}

int mcu_spi_close(const devfs_handle_t * handle){
    return spi_local_close(spi_local + handle->port, handle, spi_irqs[handle->port]);
}

int mcu_spi_getinfo(const devfs_handle_t * handle, void * ctl){
    spi_info_t * info = ctl;

    //set flags
    info->o_flags = SPI_FLAG_SET_MASTER |
            SPI_FLAG_IS_MODE0 |
            SPI_FLAG_IS_MODE1 |
            SPI_FLAG_IS_MODE2 |
            SPI_FLAG_IS_MODE3 |
            SPI_FLAG_IS_FORMAT_SPI |
            SPI_FLAG_IS_FORMAT_TI |
            SPI_FLAG_IS_FULL_DUPLEX |
            SPI_FLAG_IS_HALF_DUPLEX;

    return 0;
}

int mcu_spi_setattr(const devfs_handle_t * handle, void * ctl){
    return spi_local_setattr(spi_local + handle->port, handle, ctl);
}

int mcu_spi_swap(const devfs_handle_t * handle, void * ctl){
    return spi_local_swap(spi_local + handle->port, handle, ctl);
}


int mcu_spi_setaction(const devfs_handle_t * handle, void * ctl){
    int port = handle->port;
    return spi_local_setaction(spi_local + handle->port, handle, ctl, spi_irqs[port]);
}

int mcu_spi_write(const devfs_handle_t * handle, devfs_async_t * async){
    int ret;
    const int port = handle->port;

    DEVFS_DRIVER_IS_BUSY(spi_local[port].transfer_handler.write, async);

    if( spi_local[port].is_full_duplex && spi_local[port].transfer_handler.read ){

        if( spi_local[port].transfer_handler.read->nbyte < async->nbyte ){
            mcu_debug_log_info(MCU_DEBUG_DEVICE, "Read bytes error %d < %d", spi_local[port].transfer_handler.read->nbyte, async->nbyte);
            return SYSFS_SET_RETURN(EINVAL);
        }

        //execute the TX/RX transfer
        ret = HAL_SPI_TransmitReceive_IT(
                    &spi_local[port].hal_handle,
                    async->buf,
                    spi_local[port].transfer_handler.read->buf,
                    async->nbyte);
    } else {
        //this should be busy if a read is in progress
        if( spi_local[port].transfer_handler.read == 0 ){
            ret = HAL_SPI_Transmit_IT(&spi_local[port].hal_handle, async->buf, async->nbyte);
        } else {
            mcu_debug_log_error(MCU_DEBUG_DEVICE, "SPI BUSY WRITE FAIL");
            spi_local[port].transfer_handler.write = 0;
            return SYSFS_SET_RETURN(EBUSY);
        }
    }

    if( ret != HAL_OK ){
        mcu_debug_log_error(MCU_DEBUG_DEVICE, "SPI ERROR:%d", ret);
        spi_local[port].transfer_handler.write = 0;
        return SYSFS_SET_RETURN_WITH_VALUE(EIO, ret);
    }

    return 0;
}

int mcu_spi_read(const devfs_handle_t * handle, devfs_async_t * async){
    int ret;
    const int port = handle->port;

    DEVFS_DRIVER_IS_BUSY(spi_local[port].transfer_handler.read, async);

    if( spi_local[port].is_full_duplex ){
        ret = HAL_OK;
    } else {
        ret = HAL_SPI_Receive_IT(&spi_local[port].hal_handle, async->buf, async->nbyte);
    }

    if( ret != HAL_OK ){
        spi_local[port].transfer_handler.read = 0;
        mcu_debug_log_error(MCU_DEBUG_DEVICE, "read failed:%d", spi_local[port].hal_handle.ErrorCode);
        return SYSFS_SET_RETURN_WITH_VALUE(EIO, ret);
    }

    return 0;
}

void mcu_core_spi1_isr(){
    //No I2S on SPI 1
#if MCU_I2S_ON_SPI1 != 0
    if( spi_local[0].is_i2s ){
        HAL_I2S_IRQHandler(&spi_local[0].i2s_hal_handle);
    } else {
        HAL_SPI_IRQHandler(&spi_local[0].hal_handle);
    }
#else
    HAL_SPI_IRQHandler(&spi_local[0].hal_handle);
#endif
}

void mcu_core_spi2_isr(){
#if MCU_I2S_ON_SPI2 != 0
    if( spi_local[1].is_i2s ){
        HAL_I2S_IRQHandler(&spi_local[1].i2s_hal_handle);
    } else {
        HAL_SPI_IRQHandler(&spi_local[1].hal_handle);
    }
#else
    HAL_SPI_IRQHandler(&spi_local[1].hal_handle);
#endif
}

void mcu_core_spi3_isr(){
#if MCU_I2S_ON_SPI3 != 0
    if( spi_local[2].is_i2s ){
        HAL_I2S_IRQHandler(&spi_local[2].i2s_hal_handle);
    } else {
        HAL_SPI_IRQHandler(&spi_local[2].hal_handle);
    }
#else
    HAL_SPI_IRQHandler(&spi_local[2].hal_handle);
#endif
}

void mcu_core_spi4_isr(){
#if MCU_I2S_ON_SPI4 != 0
    if( spi_local[3].is_i2s ){
        HAL_I2S_IRQHandler(&spi_local[3].i2s_hal_handle);
    } else {
        HAL_SPI_IRQHandler(&spi_local[3].hal_handle);
    }
#else
    HAL_SPI_IRQHandler(&spi_local[3].hal_handle);
#endif
}

#if MCU_SPI_PORTS > 4
void mcu_core_spi5_isr(){
    HAL_SPI_IRQHandler(&spi_local[4].hal_handle);
}
#endif

#if MCU_SPI_PORTS > 5
void mcu_core_spi6_isr(){
    HAL_SPI_IRQHandler(&spi_local[5].hal_handle);
}
#endif

#endif

