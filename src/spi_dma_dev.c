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
#include "stm32_dma.h"
#include <mcu/spi.h>


#if MCU_SPI_PORTS > 0

spi_dma_local_t spi_dma_local[MCU_SPI_PORTS] MCU_SYS_MEM;

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(spi_dma, SPI_VERSION, I_MCU_TOTAL + I_SPI_TOTAL, mcu_spi_dma_swap)

int mcu_spi_dma_open(const devfs_handle_t * handle){
    int port = handle->port;
    if ( spi_dma_local[port].spi.ref_count == 0 ){
        spi_dma_local[port].spi.hal_handle.Instance = spi_local_open(port);
        spi_dma_local[port].spi.read = NULL;
        spi_dma_local[port].spi.write = NULL;
    }
    spi_dma_local[port].spi.ref_count++;
    return 0;

}

int mcu_spi_dma_close(const devfs_handle_t * handle){
    int port = handle->port;
    if ( spi_dma_local[port].spi.ref_count > 0 ){
        if ( spi_dma_local[port].spi.ref_count == 1 ){
            spi_local_close(port);
        }
        spi_dma_local[port].spi.ref_count--;
    }
    return 0;
}

int mcu_spi_dma_getinfo(const devfs_handle_t * handle, void * ctl){
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

int mcu_spi_dma_setattr(const devfs_handle_t * handle, void * ctl){
    int port = handle->port;

    const spi_attr_t * attr = ctl;

    //setup the DMA for receiving
    stm32_dma_set_handle(&spi_dma_local[port].dma_rx_channel, 0, 0);
    spi_dma_local[port].dma_rx_channel.handle.Instance = stm32_dma_get_stream_instance(0, 0); //DMA1 stream 0
    spi_dma_local[port].dma_rx_channel.handle.Init.Channel = stm32_dma_decode_channel(3);
    spi_dma_local[port].dma_rx_channel.handle.Init.Direction = DMA_PERIPH_TO_MEMORY; //read is always periph to memory
    spi_dma_local[port].dma_rx_channel.handle.Init.PeriphInc = DMA_PINC_DISABLE; //don't inc peripheral
    spi_dma_local[port].dma_rx_channel.handle.Init.MemInc = DMA_MINC_ENABLE; //do inc the memory
    if( attr->width == 8 ){
        spi_dma_local[port].dma_rx_channel.handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        spi_dma_local[port].dma_rx_channel.handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    } else if( attr->width == 16 ){
        spi_dma_local[port].dma_rx_channel.handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        spi_dma_local[port].dma_rx_channel.handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    } else if( attr->width == 32 ){
        spi_dma_local[port].dma_rx_channel.handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        spi_dma_local[port].dma_rx_channel.handle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    }

    spi_dma_local[port].dma_rx_channel.handle.Init.Mode = DMA_NORMAL;
    spi_dma_local[port].dma_rx_channel.handle.Init.Priority = DMA_PRIORITY_LOW;
    spi_dma_local[port].dma_rx_channel.handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&spi_dma_local[port].dma_rx_channel.handle) != HAL_OK){
      return SYSFS_SET_RETURN(EIO);
    }

    __HAL_LINKDMA((&spi_dma_local[port].spi.hal_handle), hdmarx, spi_dma_local[port].dma_rx_channel.handle);

    return spi_local_setattr(handle, ctl, &spi_dma_local[handle->port].spi);
}

int mcu_spi_dma_swap(const devfs_handle_t * handle, void * ctl){
    return spi_local_swap(handle, ctl, &spi_dma_local[handle->port].spi);
}

int mcu_spi_dma_setaction(const devfs_handle_t * handle, void * ctl){
    //need to pass the interrupt number
    return spi_local_setaction(handle, ctl, &spi_dma_local[handle->port].spi, 0);
}

int mcu_spi_dma_write(const devfs_handle_t * handle, devfs_async_t * async){
    int ret;
    int port = handle->port;

    //check to see if SPI bus is busy -- check to see if the interrupt is enabled?
    if( spi_dma_local[port].spi.write ){
        return SYSFS_SET_RETURN(EBUSY);
    }

    //this can be used to see if the write is busy
    if( async->nbyte == 0 ){
        return 0;
    }

    spi_dma_local[port].spi.write = async;

    if( spi_dma_local[port].spi.is_full_duplex && spi_dma_local[port].spi.read ){

        if( spi_dma_local[port].spi.read->nbyte < async->nbyte ){
            return SYSFS_SET_RETURN(EINVAL);
        }

        ret = HAL_SPI_TransmitReceive_DMA(&spi_dma_local[port].spi.hal_handle, async->buf, async->buf, async->nbyte);
    } else {
        ret = HAL_SPI_Transmit_DMA(&spi_dma_local[port].spi.hal_handle, async->buf, async->nbyte);
    }

    if( ret != HAL_OK ){
        return SYSFS_SET_RETURN(EIO);
    }

    return 0;
}

int mcu_spi_dma_read(const devfs_handle_t * handle, devfs_async_t * async){
    int ret;
    int port = handle->port;

    if( spi_dma_local[port].spi.read ){
        return SYSFS_SET_RETURN(EBUSY);
    }

    if( async->nbyte == 0 ){
        return 0;
    }

    spi_dma_local[port].spi.read = async;

    if( spi_dma_local[port].spi.is_full_duplex ){
        //just set up the buffer
        ret = HAL_OK;
    } else {
        ret = HAL_SPI_Receive_DMA(&spi_dma_local[port].spi.hal_handle, async->buf, async->nbyte);
    }

    if( ret != HAL_OK ){
        return SYSFS_SET_RETURN(EIO);
    }

    return 0;
}



#endif

