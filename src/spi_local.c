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

SPI_TypeDef * const spi_regs[MCU_SPI_PORTS] = MCU_SPI_REGS;
u8 const spi_irqs[MCU_SPI_PORTS] = MCU_SPI_IRQS;

SPI_TypeDef * spi_local_open(int port){
    //turn on RCC clock
    switch(port){
    case 0:
        __HAL_RCC_SPI1_CLK_ENABLE();
        break;
#if MCU_SPI_PORTS > 1
    case 1:
        __HAL_RCC_SPI2_CLK_ENABLE();
        break;
#endif
#if MCU_SPI_PORTS > 2
    case 2:
        __HAL_RCC_SPI3_CLK_ENABLE();
        break;
#endif
#if MCU_SPI_PORTS > 3
    case 3:
        __HAL_RCC_SPI4_CLK_ENABLE();
        break;
#endif
#if MCU_SPI_PORTS > 4
    case 4:
        __HAL_RCC_SPI5_CLK_ENABLE();
        break;
#endif
#if MCU_SPI_PORTS > 5
    case 5:
        __HAL_RCC_SPI6_CLK_ENABLE();
        break;
#endif
    }
    return spi_regs[port];
}

void spi_local_close(int port){

    cortexm_disable_irq((void*)(u32)(spi_irqs[port]));

    //turn off RCC clock
    switch(port){
    case 0:
        __HAL_RCC_SPI1_CLK_DISABLE();
        break;
#if MCU_SPI_PORTS > 1
    case 1:
        __HAL_RCC_SPI2_CLK_DISABLE();
        break;
#endif
#if MCU_SPI_PORTS > 2
    case 2:
        __HAL_RCC_SPI3_CLK_DISABLE();
        break;
#endif
#if MCU_SPI_PORTS > 3
    case 3:
        __HAL_RCC_SPI4_CLK_DISABLE();
        break;
#endif
#if MCU_SPI_PORTS > 4
    case 4:
        __HAL_RCC_SPI5_CLK_DISABLE();
        break;
#endif
#if MCU_SPI_PORTS > 5
    case 5:
        __HAL_RCC_SPI6_CLK_DISABLE();
        break;
#endif
    }
}

int spi_local_setattr(const devfs_handle_t * handle, void * ctl, spi_local_t * spi){
    u32 pclk;
    u32 prescalar;

    const spi_attr_t * attr = mcu_select_attr(handle, ctl);
    if( attr == 0 ){
        return SYSFS_SET_RETURN(EINVAL);
    }

    u32 o_flags = attr->o_flags;

    spi->is_i2s = 0;

    if( o_flags & SPI_FLAG_SET_MASTER ){
        spi->hal_handle.Init.Mode = SPI_MODE_MASTER;

        if( attr->freq == 0 ){
            spi->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
        } else {

            //#if defined __stm32f401xc
            if( (handle->port == 0) || (handle->port == 3) ){
                pclk = HAL_RCC_GetPCLK1Freq(); //42MHz max
            } else {
                pclk = HAL_RCC_GetPCLK2Freq(); //84MHz max
            }
            //#endif

            //get as close to the target freq as possible without going over
            prescalar = pclk / attr->freq;
            if( prescalar < 2 ){
                spi->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
            } else if( prescalar < 4 ){
                spi->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
            } else if( prescalar < 8 ){
                spi->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
            } else if( prescalar < 16 ){
                spi->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
            } else if( prescalar < 32 ){
                spi->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
            }else if( prescalar < 64 ){
                spi->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
            }else if( prescalar < 128 ){
                spi->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
            }else {
                spi->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
            }
        }

    } else if( o_flags & SPI_FLAG_SET_SLAVE ){
        spi->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
        spi->hal_handle.Init.Mode = SPI_MODE_SLAVE;
    }

    spi->hal_handle.Init.CLKPolarity = SPI_POLARITY_LOW;
    spi->hal_handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    if( o_flags & SPI_FLAG_IS_MODE1 ){
        spi->hal_handle.Init.CLKPolarity = SPI_POLARITY_LOW;
        spi->hal_handle.Init.CLKPhase = SPI_PHASE_2EDGE;
    } else if( o_flags & SPI_FLAG_IS_MODE2 ){
        spi->hal_handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
        spi->hal_handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    } else if( o_flags & SPI_FLAG_IS_MODE3 ){
        spi->hal_handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
        spi->hal_handle.Init.CLKPhase = SPI_PHASE_2EDGE;
    }

    if( attr->width == 8 ){
        spi->hal_handle.Init.DataSize = SPI_DATASIZE_8BIT;
    } else if( attr->width == 16 ){
        spi->hal_handle.Init.DataSize = SPI_DATASIZE_16BIT;
    } else {
        return SYSFS_SET_RETURN(EINVAL);
    }

    spi->hal_handle.Init.Direction = SPI_DIRECTION_2LINES;

    spi->hal_handle.Init.NSS = SPI_NSS_SOFT;
    spi->hal_handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    if( o_flags & SPI_FLAG_IS_FORMAT_TI ){
        spi->hal_handle.Init.TIMode = SPI_TIMODE_ENABLE;
    } else {
        spi->hal_handle.Init.TIMode = SPI_TIMODE_DISABLE;
    }
    spi->hal_handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi->hal_handle.Init.CRCPolynomial = 10;

    if( mcu_set_pin_assignment(
                &(attr->pin_assignment),
                MCU_CONFIG_PIN_ASSIGNMENT(spi_config_t, handle),
                MCU_PIN_ASSIGNMENT_COUNT(spi_pin_assignment_t),
                CORE_PERIPH_SPI, handle->port, 0, 0, 0) < 0 ){
        return SYSFS_SET_RETURN(EINVAL);
    }

    if( HAL_SPI_Init(&spi->hal_handle) != HAL_OK ){
        return SYSFS_SET_RETURN(EINVAL);
    }

    return 0;
}

int spi_local_swap(const devfs_handle_t * handle, void * ctl, spi_local_t * spi){
    u8 tx_data;
    u8 rx_data;
    int ret;
    tx_data = (u32)ctl;
    ret = HAL_SPI_TransmitReceive(&spi->hal_handle, &tx_data, &rx_data, 1, HAL_MAX_DELAY);
    if( ret != HAL_OK ){
        return SYSFS_SET_RETURN(EIO);
    }
    //must always be a positive (int)
    return rx_data;
}

int spi_local_setaction(const devfs_handle_t * handle, void * ctl, spi_local_t * spi, int interrupt_number){
    mcu_action_t * action = (mcu_action_t*)ctl;

    if(action->handler.callback == 0){
        if (action->o_events & MCU_EVENT_FLAG_DATA_READY){
            spi_local_execute_handler(&spi->read, MCU_EVENT_FLAG_CANCELED, 0, -1);
        }

        if (action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE){
            spi_local_execute_handler(&spi->write, MCU_EVENT_FLAG_CANCELED, 0, -1);
        }
    }

    cortexm_set_irq_prio(interrupt_number, action->prio);
    return 0;
}

int spi_local_execute_handler(devfs_async_t ** async_ptr, u32 o_events, u32 value, int ret){
    spi_event_data_t event;
    if( *async_ptr != 0 ){
        devfs_async_t * async = *async_ptr;
        *async_ptr = 0;
        async->nbyte = ret;
        event.value = value;
        return mcu_execute_event_handler(&async->handler, o_events, &event);
    }
    return 0;
}


//these callbacks work the same for both DMA and interrupt driven operations
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
    //execute the handler
    spi_local_t * spi = (spi_local_t*)hspi;
    spi_local_execute_handler(&spi->write, MCU_EVENT_FLAG_WRITE_COMPLETE, hspi->TxXferCount, hspi->TxXferSize);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
    //execute the handler
    spi_local_t * spi = (spi_local_t*)hspi;
    spi_local_execute_handler(&spi->read, MCU_EVENT_FLAG_DATA_READY, hspi->RxXferCount, hspi->RxXferSize);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
    //execute the handler
    spi_local_t * spi = (spi_local_t*)hspi;
    spi_local_execute_handler(&spi->write, MCU_EVENT_FLAG_WRITE_COMPLETE, hspi->TxXferCount, hspi->TxXferSize);
    spi_local_execute_handler(&spi->read, MCU_EVENT_FLAG_DATA_READY, hspi->RxXferCount, hspi->RxXferSize);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi){
    spi_local_t * spi = (spi_local_t*)hspi;
    spi_local_execute_handler(&spi->write,
                              MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR,
                              0,
                              SYSFS_SET_RETURN_WITH_VALUE(EIO, hspi->ErrorCode));

    spi_local_execute_handler(&spi->read,
                              MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR,
                              0,
                              SYSFS_SET_RETURN_WITH_VALUE(EIO, hspi->ErrorCode | 0x80));
}

void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi){
    spi_local_t * spi = (spi_local_t*)hspi;
    spi_local_execute_handler(&spi->write, MCU_EVENT_FLAG_WRITE_COMPLETE | MCU_EVENT_FLAG_CANCELED, 0, hspi->TxXferSize - hspi->TxXferCount);
    spi_local_execute_handler(&spi->read, MCU_EVENT_FLAG_DATA_READY | MCU_EVENT_FLAG_CANCELED, 0, hspi->RxXferSize - hspi->RxXferCount);
}

#endif

