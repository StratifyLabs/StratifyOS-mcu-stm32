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
    info->o_events = MCU_EVENT_FLAG_DATA_READY |
            MCU_EVENT_FLAG_WRITE_COMPLETE |
            MCU_EVENT_FLAG_HIGH |
            MCU_EVENT_FLAG_LOW;


    return 0;
}

int mcu_i2s_spi_dma_mute(const devfs_handle_t * handle, void * ctl){
    return 0;
}

int mcu_i2s_spi_dma_unmute(const devfs_handle_t * handle, void * ctl){
    return 0;
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

    if( attr->o_flags & I2S_FLAG_SET_MASTER | I2S_FLAG_SET_SLAVE ){

        if( attr->o_flags & I2S_FLAG_IS_RECEIVER ){

            //setup the DMA for receiving
            stm32_dma_set_handle(&spi_dma_local[port].dma_rx_channel, config->dma_config.rx.dma_number, config->dma_config.rx.stream_number); //need to get the DMA# and stream# from a table -- or from config
            spi_dma_local[port].dma_rx_channel.handle.Instance = stm32_dma_get_stream_instance(config->dma_config.rx.dma_number, config->dma_config.rx.stream_number); //DMA1 stream 0
            spi_dma_local[port].dma_rx_channel.handle.Init.Channel = stm32_dma_decode_channel(config->dma_config.rx.channel_number);
            spi_dma_local[port].dma_rx_channel.handle.Init.Mode = DMA_CIRCULAR;
            spi_dma_local[port].dma_rx_channel.handle.Init.Direction = DMA_PERIPH_TO_MEMORY; //read is always periph to memory
            spi_dma_local[port].dma_rx_channel.handle.Init.PeriphInc = DMA_PINC_DISABLE; //don't inc peripheral
            spi_dma_local[port].dma_rx_channel.handle.Init.MemInc = DMA_MINC_ENABLE; //do inc the memory

            spi_dma_local[port].dma_rx_channel.handle.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
            spi_dma_local[port].dma_rx_channel.handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;

            if( attr->o_flags & I2S_FLAG_IS_WIDTH_8 ){
                spi_dma_local[port].dma_rx_channel.handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
                spi_dma_local[port].dma_rx_channel.handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
                spi_dma_local[port].dma_rx_channel.handle.Init.MemBurst = DMA_MBURST_INC8;
                spi_dma_local[port].dma_rx_channel.handle.Init.PeriphBurst = DMA_PBURST_INC8;
            } else if( attr->o_flags & (I2S_FLAG_IS_WIDTH_16 | I2S_FLAG_IS_WIDTH_16_EXTENDED) ){
                spi_dma_local[port].dma_rx_channel.handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
                spi_dma_local[port].dma_rx_channel.handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
                spi_dma_local[port].dma_rx_channel.handle.Init.MemBurst = DMA_MBURST_INC4;
                spi_dma_local[port].dma_rx_channel.handle.Init.PeriphBurst = DMA_PBURST_INC4;
            } else { //I2S_FLAG_IS_WIDTH_24 | I2S_FLAG_IS_WIDTH_32
                spi_dma_local[port].dma_rx_channel.handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
                spi_dma_local[port].dma_rx_channel.handle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
                spi_dma_local[port].dma_rx_channel.handle.Init.MemBurst = DMA_MBURST_SINGLE;
                spi_dma_local[port].dma_rx_channel.handle.Init.PeriphBurst = DMA_PBURST_SINGLE;
            }

            spi_dma_local[port].dma_rx_channel.handle.Init.Priority = stm32_dma_decode_priority(config->dma_config.rx.priority);

            if (HAL_DMA_Init(&spi_dma_local[port].dma_rx_channel.handle) != HAL_OK){
                return SYSFS_SET_RETURN(EIO);
            }

            __HAL_LINKDMA((&spi_dma_local[port].spi.hal_handle), hdmarx, spi_dma_local[port].dma_rx_channel.handle);

        }
        if( attr->o_flags & I2S_FLAG_IS_TRANSMITTER ){

            //setup the DMA for transmitting
            stm32_dma_set_handle(&spi_dma_local[port].dma_tx_channel, config->dma_config.tx.dma_number, config->dma_config.tx.stream_number); //need to get the DMA# and stream# from a table -- or from config
            spi_dma_local[port].dma_tx_channel.handle.Instance = stm32_dma_get_stream_instance(config->dma_config.tx.dma_number, config->dma_config.tx.stream_number); //DMA1 stream 0
            spi_dma_local[port].dma_tx_channel.handle.Init.Channel = stm32_dma_decode_channel(config->dma_config.tx.channel_number);
            spi_dma_local[port].dma_tx_channel.handle.Init.Direction = DMA_PERIPH_TO_MEMORY; //read is always periph to memory
            spi_dma_local[port].dma_tx_channel.handle.Init.PeriphInc = DMA_PINC_DISABLE; //don't inc peripheral
            spi_dma_local[port].dma_tx_channel.handle.Init.MemInc = DMA_MINC_ENABLE; //do inc the memory

            spi_dma_local[port].dma_tx_channel.handle.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
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

            __HAL_LINKDMA((&spi_dma_local[port].spi.hal_handle), hdmatx, spi_dma_local[port].dma_tx_channel.handle);
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

    if( spi->spi.is_full_duplex && spi_dma_local[port].spi.transfer_handler.read ){

#if defined I2S_FULLDUPLEXMODE_ENABLE
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
        result = HAL_I2S_Transmit_DMA(&spi->spi.i2s_hal_handle, async->buf, async->nbyte);
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

    if( spi->spi.is_full_duplex ){
        //Receive assigns the transfer handler but then blocks until a write happens
        ret = 0;
    } else {
        ret = HAL_I2S_Receive_DMA(&spi->spi.i2s_hal_handle, async->buf, async->nbyte);
    }

    if( ret != HAL_OK ){
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

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
    spi_local_t * local = (spi_local_t *)hi2s;
    int result;

    //MCU_EVENT_FLAG_WRITE_COMPLETE | MCU_EVENT_FLAG_LOW -- first half

    //mcu_execute_transfer_event_handler -- won't get rid of the callback -- needs to be circular -- setaction can abort the reception of data

    result = mcu_execute_transfer_handler(
                &local->transfer_handler.write->handler,
                MCU_EVENT_FLAG_WRITE_COMPLETE | MCU_EVENT_FLAG_LOW,
                0);
    if( result == 0 ){
        //abort reception
        HAL_I2S_DMAStop(hi2s);
    }
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s){
    spi_local_t * local = (spi_local_t *)hi2s;
    int result;

    //MCU_EVENT_FLAG_WRITE_COMPLETE | MCU_EVENT_FLAG_LOW -- first half

    result = mcu_execute_transfer_handler(
                &local->transfer_handler.write->handler,
                MCU_EVENT_FLAG_WRITE_COMPLETE | MCU_EVENT_FLAG_HIGH,
                0);
    if( result == 0 ){
        //abort reception
        HAL_I2S_DMAStop(hi2s);
    }
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
    spi_local_t * local = (spi_local_t *)hi2s;
    int result;

    //MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_LOW -- first half

    //mcu_execute_transfer_event_handler -- won't get rid of the callback -- needs to be circular -- setaction can abort the reception of data

    result = mcu_execute_transfer_handler(
                &local->transfer_handler.read->handler,
                MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_LOW,
                0);
    if( result == 0 ){
        //abort reception
        HAL_I2S_DMAStop(hi2s);
    }
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s){
    int result;
    //MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_HIGH -- second half -- preserve callback
    spi_local_t * local = (spi_local_t *)hi2s;
    //mcu_execute_read_handler(&local->transfer_handler, 0, hi2s->TxXferSize);

    result = mcu_execute_transfer_handler(
                &local->transfer_handler.read->handler,
                MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_HIGH,
                0);
    if( result == 0 ){
        //abort reception
        HAL_I2S_DMAStop(hi2s);
    }
}

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s){
    //called on overflow and underrun
    spi_local_t * local = (spi_local_t *)hi2s;

    mcu_execute_transfer_handlers(&local->transfer_handler, 0, SYSFS_SET_RETURN(EIO), hi2s->TxXferSize);
}


#endif

