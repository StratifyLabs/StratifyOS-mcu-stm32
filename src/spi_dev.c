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

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(spi, SPI_VERSION, I_MCU_TOTAL + I_SPI_TOTAL, mcu_spi_swap)


int mcu_spi_open(const devfs_handle_t * handle){
    int port = handle->port;
    if ( spi_local[port].ref_count == 0 ){
        spi_local[port].read = NULL;
        spi_local[port].write = NULL;
        spi_local[port].hal_handle.Instance = spi_local_open(port);
        cortexm_enable_irq((void*)(u32)(spi_irqs[port]));
    }
    spi_local[port].ref_count++;
    return 0;

}

int mcu_spi_close(const devfs_handle_t * handle){
    int port = handle->port;
    if ( spi_local[port].ref_count > 0 ){
        if ( spi_local[port].ref_count == 1 ){
            cortexm_disable_irq((void*)(u32)(spi_irqs[port]));
            spi_local_close(port);

        }
        spi_local[port].ref_count--;
    }
    return 0;
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
    return spi_local_setattr(handle, ctl, spi_local + handle->port);
}

int mcu_spi_swap(const devfs_handle_t * handle, void * ctl){
    return spi_local_swap(handle, ctl, spi_local + handle->port);
}


int mcu_spi_setaction(const devfs_handle_t * handle, void * ctl){
    int port = handle->port;
    return spi_local_setaction(handle, ctl, spi_local + port, spi_irqs[port]);
}

int mcu_spi_write(const devfs_handle_t * handle, devfs_async_t * async){
    int ret;
    int port = handle->port;

    //check to see if SPI bus is busy -- check to see if the interrupt is enabled?
    if( spi_local[port].write ){
        return SYSFS_SET_RETURN(EBUSY);
    }

    if( async->nbyte == 0 ){
        return 0;
    }

    spi_local[port].write = async;

    if( spi_local[port].is_full_duplex && spi_local[port].read ){

        if( spi_local[port].read->nbyte < async->nbyte ){
            return SYSFS_SET_RETURN(EINVAL);
        }

        //execute the TX/RX transfer
        ret = HAL_SPI_TransmitReceive_IT(
                    &spi_local[port].hal_handle,
                    async->buf,
                    spi_local[port].read->buf,
                    async->nbyte);
    } else {
        ret = HAL_SPI_Transmit_IT(&spi_local[port].hal_handle, async->buf, async->nbyte);
    }

    if( ret != HAL_OK ){
        spi_local[port].write = 0;
    }

    if( ret == HAL_BUSY ){
        return SYSFS_SET_RETURN(EBUSY);
    } else if( ret == HAL_ERROR ){
        return SYSFS_SET_RETURN(EIO);
    } else if( ret == HAL_TIMEOUT ){
        return SYSFS_SET_RETURN(ETIMEDOUT);
    } else if( ret != HAL_OK ){
        return SYSFS_SET_RETURN(EIO);
    }

    return 0;
}

int mcu_spi_read(const devfs_handle_t * handle, devfs_async_t * async){
    int ret;
    int port = handle->port;

    if( spi_local[port].read ){
        return SYSFS_SET_RETURN(EBUSY);
    }

    if( async->nbyte == 0 ){
        return 0;
    }

    spi_local[port].read = async;

    if( spi_local[port].is_full_duplex ){
        ret = HAL_OK;
    } else {
        ret = HAL_SPI_Receive_IT(&spi_local[port].hal_handle, async->buf, async->nbyte);
    }

    if( ret != HAL_OK ){
        spi_local[port].read = 0;
    }

    if( ret == HAL_BUSY ){
        return SYSFS_SET_RETURN(EBUSY);
    } else if( ret == HAL_ERROR ){
        return SYSFS_SET_RETURN(EIO);
    } else if( ret == HAL_TIMEOUT ){
        return SYSFS_SET_RETURN(ETIMEDOUT);
    } else if( ret != HAL_OK ){
        return SYSFS_SET_RETURN(EIO);
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

