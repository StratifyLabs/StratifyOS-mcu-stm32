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

#if MCU_I2S_SPI_PORTS > 0

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(i2s_spi_dma, I2S_VERSION, I2S_IOC_IDENT_CHAR, I_MCU_TOTAL + I_I2S_TOTAL, mcu_i2s_spi_dma_mute, mcu_i2s_spi_dma_unmute)

int mcu_i2s_spi_dma_open(const devfs_handle_t * handle){
    return spi_local_open(&spi_dma_local[handle->port].spi, handle);

}

int mcu_i2s_spi_dma_close(const devfs_handle_t * handle){
    //same as SPI
    const u32 port = handle->port;

    if( spi_dma_local[handle->port].spi.ref_count == 1 ){
        //disable the DMA
        const stm32_i2s_spi_dma_config_t * config;
        config = handle->config;
        HAL_I2S_DMAStop(&spi_dma_local[handle->port].spi.i2s_hal_handle);

        if( spi_dma_local[port].dma_rx_channel.interrupt_number > 0 ){
            HAL_DMA_DeInit(&spi_dma_local[port].dma_rx_channel.handle);
        }

        if( spi_dma_local[port].dma_tx_channel.interrupt_number > 0 ){
            HAL_DMA_DeInit(&spi_dma_local[port].dma_tx_channel.handle);
        }

        if( config ){
            stm32_dma_clear_handle(&spi_dma_local[port].dma_rx_channel, config->dma_config.rx.dma_number, config->dma_config.rx.stream_number);
            stm32_dma_clear_handle(&spi_dma_local[port].dma_tx_channel, config->dma_config.tx.dma_number, config->dma_config.tx.stream_number);
        }

    }

    spi_local_close(&spi_dma_local[port].spi, handle);


    return 0;
}

int mcu_i2s_spi_dma_getinfo(const devfs_handle_t * handle, void * ctl){
    i2s_info_t * info = ctl;

    //set I2S Capability flags
    info->o_flags = I2S_FLAG_SET_MASTER |
            I2S_FLAG_SET_SLAVE |
            I2S_FLAG_IS_WIDTH_8 |
            I2S_FLAG_IS_WIDTH_16 |
            I2S_FLAG_IS_WIDTH_16_EXTENDED |
            I2S_FLAG_IS_WIDTH_24 |
            I2S_FLAG_IS_WIDTH_32;

    info->o_events = MCU_EVENT_FLAG_DATA_READY |
            MCU_EVENT_FLAG_WRITE_COMPLETE |
            MCU_EVENT_FLAG_HIGH |
            MCU_EVENT_FLAG_LOW;


    return 0;
}

int mcu_i2s_spi_dma_mute(const devfs_handle_t * handle, void * ctl){
    return i2s_spi_local_mute(handle, ctl);
}

int mcu_i2s_spi_dma_unmute(const devfs_handle_t * handle, void * ctl){
    return i2s_spi_local_unmute(handle, ctl);
}

int mcu_i2s_spi_dma_setattr(const devfs_handle_t * handle, void * ctl){
    const u32 port = handle->port;
    const stm32_i2s_spi_dma_config_t * config;
    const i2s_attr_t * attr = mcu_select_attr(handle, ctl);
    if( attr == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

    //setup the DMA
    //BSP *MUST* provide DMA configuration information
    config = handle->config;
    if( config == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }

    if( attr->o_flags & (I2S_FLAG_SET_MASTER | I2S_FLAG_SET_SLAVE) ){

        spi_dma_local[port].spi.o_flags = SPI_LOCAL_IS_DMA | SPI_LOCAL_IS_I2S;

#if 0
        u32 dma_flags = STM32_DMA_FLAG_IS_MEMORY_WORD | STM32_DMA_FLAG_IS_PERIPH_WORD;
        if( attr->o_flags & I2S_FLAG_IS_WIDTH_8 ){
            dma_flags = STM32_DMA_FLAG_IS_MEMORY_BYTE | STM32_DMA_FLAG_IS_PERIPH_BYTE | STM32_DMA_FLAG_IS_MEMORY_INC8;
        } else if( attr->o_flags & (I2S_FLAG_IS_WIDTH_16 | I2S_FLAG_IS_WIDTH_16_EXTENDED) ){
            dma_flags = STM32_DMA_FLAG_IS_MEMORY_HALFWORD | STM32_DMA_FLAG_IS_PERIPH_HALFWORD | STM32_DMA_FLAG_IS_MEMORY_INC4;
        }
#endif

        if( attr->o_flags & I2S_FLAG_IS_RECEIVER ){

            mcu_debug_log_info(MCU_DEBUG_DEVICE, "Set I2S DMA as receiver %d.%d.%d",
                               config->dma_config.rx.dma_number,
                               config->dma_config.rx.stream_number,
                               config->dma_config.rx.channel_number
                               );

#if 0
            //setup the DMA for receiving
            stm32_dma_set_handle(&spi_dma_local[port].dma_rx_channel, config->dma_config.rx.dma_number, config->dma_config.rx.stream_number);
            spi_dma_local[port].dma_rx_channel.handle.Instance = stm32_dma_get_stream_instance(config->dma_config.rx.dma_number, config->dma_config.rx.stream_number); //DMA1 stream 0
            spi_dma_local[port].dma_rx_channel.handle.Init.Channel = stm32_dma_decode_channel(config->dma_config.rx.channel_number);
            spi_dma_local[port].dma_rx_channel.handle.Init.Direction = DMA_PERIPH_TO_MEMORY; //read is always periph to memory
            spi_dma_local[port].dma_rx_channel.handle.Init.PeriphInc = DMA_PINC_DISABLE; //don't inc peripheral
            spi_dma_local[port].dma_rx_channel.handle.Init.MemInc = DMA_MINC_ENABLE; //do inc the memory

            spi_dma_local[port].dma_rx_channel.handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE; //?
            spi_dma_local[port].dma_rx_channel.handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;

            if( attr->o_flags & I2S_FLAG_IS_WIDTH_8 ){
                spi_dma_local[port].dma_rx_channel.handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
                spi_dma_local[port].dma_rx_channel.handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
                spi_dma_local[port].dma_rx_channel.handle.Init.MemBurst = DMA_MBURST_SINGLE;
                spi_dma_local[port].dma_rx_channel.handle.Init.PeriphBurst = DMA_PBURST_SINGLE;
            } else if( attr->o_flags & (I2S_FLAG_IS_WIDTH_16 | I2S_FLAG_IS_WIDTH_16_EXTENDED) ){
                spi_dma_local[port].dma_rx_channel.handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
                spi_dma_local[port].dma_rx_channel.handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
                spi_dma_local[port].dma_rx_channel.handle.Init.MemBurst = DMA_MBURST_SINGLE;
                spi_dma_local[port].dma_rx_channel.handle.Init.PeriphBurst = DMA_PBURST_SINGLE;
            } else { //I2S_FLAG_IS_WIDTH_24 | I2S_FLAG_IS_WIDTH_32
                spi_dma_local[port].dma_rx_channel.handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
                spi_dma_local[port].dma_rx_channel.handle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
                spi_dma_local[port].dma_rx_channel.handle.Init.MemBurst = DMA_MBURST_SINGLE;
                spi_dma_local[port].dma_rx_channel.handle.Init.PeriphBurst = DMA_PBURST_SINGLE;
            }

            spi_dma_local[port].dma_rx_channel.handle.Init.Mode = DMA_CIRCULAR;
            spi_dma_local[port].dma_rx_channel.handle.Init.Priority = stm32_dma_decode_priority(config->dma_config.rx.priority);

            if (HAL_DMA_Init(&spi_dma_local[port].dma_rx_channel.handle) != HAL_OK){
                return SYSFS_SET_RETURN(EIO);
            }
#else

#if 0
            int dma_result = stm32_dma_setattr(&spi_dma_local[port].dma_rx_channel,
                                               &config->dma_config.rx,
                                               STM32_DMA_FLAG_IS_CIRCULAR |
                                               STM32_DMA_FLAG_IS_FIFO |
                                               STM32_DMA_FLAG_IS_PERIPH_TO_MEMORY |
                                               dma_flags
                                               );
#else
            int dma_result = stm32_dma_setattr(&spi_dma_local[port].dma_rx_channel,
                                               &config->dma_config.rx);

#endif
            if( dma_result < 0 ){ return dma_result; }
#endif

            __HAL_LINKDMA((&spi_dma_local[port].spi.i2s_hal_handle), hdmarx, spi_dma_local[port].dma_rx_channel.handle);


        }
        if( attr->o_flags & I2S_FLAG_IS_TRANSMITTER ){

            mcu_debug_log_info(MCU_DEBUG_DEVICE, "Set I2S DMA as transmitter %d.%d.%d",
                               config->dma_config.tx.dma_number,
                               config->dma_config.tx.stream_number,
                               config->dma_config.tx.channel_number
                               );
#if 0
            //setup the DMA for transmitting
            stm32_dma_set_handle(&spi_dma_local[port].dma_tx_channel, config->dma_config.tx.dma_number, config->dma_config.tx.stream_number); //need to get the DMA# and stream# from a table -- or from config
            spi_dma_local[port].dma_tx_channel.handle.Instance = stm32_dma_get_stream_instance(config->dma_config.tx.dma_number, config->dma_config.tx.stream_number); //DMA1 stream 0
            spi_dma_local[port].dma_tx_channel.handle.Init.Channel = stm32_dma_decode_channel(config->dma_config.tx.channel_number);
            spi_dma_local[port].dma_tx_channel.handle.Init.Direction = DMA_MEMORY_TO_PERIPH; //write is always memory to periph
            spi_dma_local[port].dma_tx_channel.handle.Init.PeriphInc = DMA_PINC_DISABLE; //don't inc peripheral
            spi_dma_local[port].dma_tx_channel.handle.Init.MemInc = DMA_MINC_ENABLE; //do inc the memory

            spi_dma_local[port].dma_tx_channel.handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
            spi_dma_local[port].dma_tx_channel.handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;

            if( attr->o_flags & I2S_FLAG_IS_WIDTH_8 ){
                spi_dma_local[port].dma_tx_channel.handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
                spi_dma_local[port].dma_tx_channel.handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
                spi_dma_local[port].dma_tx_channel.handle.Init.MemBurst = DMA_MBURST_INC8;
                spi_dma_local[port].dma_tx_channel.handle.Init.PeriphBurst = DMA_PBURST_INC8;
            } else if( attr->o_flags & (I2S_FLAG_IS_WIDTH_16 | I2S_FLAG_IS_WIDTH_16_EXTENDED) ){
                spi_dma_local[port].dma_tx_channel.handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
                spi_dma_local[port].dma_tx_channel.handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
                spi_dma_local[port].dma_tx_channel.handle.Init.MemBurst = DMA_MBURST_INC4;
                spi_dma_local[port].dma_tx_channel.handle.Init.PeriphBurst = DMA_PBURST_INC4;
            } else { //I2S_FLAG_IS_WIDTH_24 | I2S_FLAG_IS_WIDTH_32
                spi_dma_local[port].dma_tx_channel.handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
                spi_dma_local[port].dma_tx_channel.handle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
                spi_dma_local[port].dma_tx_channel.handle.Init.MemBurst = DMA_MBURST_SINGLE;
                spi_dma_local[port].dma_tx_channel.handle.Init.PeriphBurst = DMA_PBURST_SINGLE;
            }

            spi_dma_local[port].dma_tx_channel.handle.Init.Mode = DMA_CIRCULAR;
            spi_dma_local[port].dma_tx_channel.handle.Init.Priority = stm32_dma_decode_priority(config->dma_config.tx.priority);

            if (HAL_DMA_Init(&spi_dma_local[port].dma_tx_channel.handle) != HAL_OK){
                return SYSFS_SET_RETURN(EIO);
            }
#else

#if 0
            int dma_result = stm32_dma_setattr(&spi_dma_local[port].dma_tx_channel,
                                               &config->dma_config.tx,
                                               STM32_DMA_FLAG_IS_CIRCULAR |
                                               STM32_DMA_FLAG_IS_FIFO |
                                               STM32_DMA_FLAG_IS_MEMORY_TO_PERIPH |
                                               dma_flags);
#else
            int dma_result = stm32_dma_setattr(&spi_dma_local[port].dma_tx_channel,
                                               &config->dma_config.tx);
#endif

            if( dma_result < 0 ){ return dma_result; }

#endif

            __HAL_LINKDMA((&spi_dma_local[port].spi.i2s_hal_handle), hdmatx, spi_dma_local[port].dma_tx_channel.handle);

        }
    }

    return i2s_spi_local_setattr(&spi_dma_local[handle->port].spi, handle, ctl);
}


int mcu_i2s_spi_dma_setaction(const devfs_handle_t * handle, void * ctl){
    return mcu_spi_dma_setaction(handle, ctl);
}

int mcu_i2s_spi_dma_write(const devfs_handle_t * handle, devfs_async_t * async){
    int result;
    int port = handle->port;
    spi_dma_local_t * spi = spi_dma_local + port;

    DEVFS_DRIVER_IS_BUSY(spi->spi.transfer_handler.write, async);

    if( (spi->spi.o_flags & SPI_LOCAL_IS_DMA) && spi_dma_local[port].spi.transfer_handler.read ){

#if defined SPI_I2S_FULLDUPLEX_SUPPORT
        if( spi->spi.transfer_handler.read->nbyte < async->nbyte ){
            return SYSFS_SET_RETURN(EINVAL);
        }

        result = HAL_I2SEx_TransmitReceive_DMA(
                    &spi->spi.i2s_hal_handle,
                    async->buf,
                    spi->spi.transfer_handler.read->buf,
                    async->nbyte);
#else
        return SYSFS_SET_RETURN(ENOTSUP);
#endif

    } else {
        result = HAL_I2S_Transmit_DMA(&spi->spi.i2s_hal_handle, async->buf,  async->nbyte/spi->spi.size_mult);
    }

    if( result != HAL_OK ){
        spi_dma_local[port].spi.transfer_handler.write = 0;
        if( result == HAL_BUSY ){
            return SYSFS_SET_RETURN(EIO);
        } else if( result == HAL_ERROR ){
            return SYSFS_SET_RETURN(EIO);
        } else if( result == HAL_TIMEOUT ){
            return SYSFS_SET_RETURN(EIO);
        }
        return SYSFS_SET_RETURN(EIO);

    }

    return 0;
}

int mcu_i2s_spi_dma_read(const devfs_handle_t * handle, devfs_async_t * async){
    int ret;
    int port = handle->port;
    spi_dma_local_t * spi = spi_dma_local + port;

    DEVFS_DRIVER_IS_BUSY(spi->spi.transfer_handler.read, async);

#if defined SPI_I2S_FULLDUPLEX_SUPPORT
    if( spi->spi.o_flags & SPI_LOCAL_IS_FULL_DUPLEX ){
        //Receive assigns the transfer handler but then blocks until a write happens
        mcu_debug_log_info(MCU_DEBUG_DEVICE, "Wait FD");
        return 0;
    }
#endif

    mcu_debug_log_info(MCU_DEBUG_DEVICE, "Read DMA 0x%lX %p %d %d", spi->spi.i2s_hal_handle.Init.Mode, async->buf, async->nbyte/spi->spi.size_mult, spi->spi.size_mult);

    ret = HAL_I2S_Receive_DMA(&spi->spi.i2s_hal_handle, async->buf,  (async->nbyte/spi->spi.size_mult));

    if( ret != HAL_OK ){
        mcu_debug_log_error(MCU_DEBUG_DEVICE, "Failed to start I2S DMA Read (%d, %d) %d/%d", ret, spi->spi.i2s_hal_handle.ErrorCode, async->nbyte, spi->spi.size_mult);
        spi->spi.transfer_handler.read = 0;
        if( ret == HAL_BUSY ){
            return SYSFS_SET_RETURN(EIO);
        } else if( ret == HAL_ERROR ){
            return SYSFS_SET_RETURN(EIO);
        } else if( ret == HAL_TIMEOUT ){
            return SYSFS_SET_RETURN(EIO);
        } else if( ret != HAL_OK ){
            return SYSFS_SET_RETURN(EIO);
        }
    }

    return 0;
}


#endif

