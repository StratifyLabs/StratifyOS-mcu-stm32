/* Copyright 2011-2018 Tyler Gilbert;
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

spi_local_t * spi_local_ptrs[MCU_SPI_PORTS];

int spi_local_open(spi_local_t * spi, const devfs_handle_t * handle){
    int port = handle->port;
    if( port < MCU_SPI_PORTS ){
        if ( spi->ref_count == 0 ){
            //turn on RCC clock
            switch(port){
            case 0:
                __HAL_RCC_SPI1_CLK_ENABLE();
                break;
#if defined SPI2
            case 1:
                __HAL_RCC_SPI2_CLK_ENABLE();
                break;
#endif
#if defined SPI3
            case 2:
                __HAL_RCC_SPI3_CLK_ENABLE();
                break;
#endif
#if defined SPI4
            case 3:
                __HAL_RCC_SPI4_CLK_ENABLE();
                break;
#endif
#if defined SPI5
            case 4:
                __HAL_RCC_SPI5_CLK_ENABLE();
                break;
#endif
#if defined SPI6
            case 5:
                __HAL_RCC_SPI6_CLK_ENABLE();
                break;
#endif
            }
            spi_local_ptrs[port] = spi;
            spi->transfer_handler.read = NULL;
            spi->transfer_handler.write = NULL;
            spi->hal_handle.Instance = spi_regs[port];
            cortexm_enable_irq(spi_irqs[port]);
        }
        spi->ref_count++;
        return 0;
    }

    return SYSFS_SET_RETURN(EINVAL);
}

int spi_local_close(spi_local_t * spi, const devfs_handle_t * handle){
    int port = handle->port;
    if ( spi->ref_count > 0 ){
        if ( spi->ref_count == 1 ){

#if MCU_I2S_SPI_PORTS > 0
            if( spi->o_flags & SPI_LOCAL_IS_I2S ){
                HAL_I2S_DeInit(&spi->i2s_hal_handle);
                mcu_debug_log_info(MCU_DEBUG_DEVICE, "Done I2S DeInit");
            } else {
                HAL_SPI_DeInit(&spi->hal_handle);
            }
#else
            HAL_SPI_DeInit(&spi->hal_handle);
#endif

            cortexm_disable_irq(spi_irqs[port]);
            devfs_execute_cancel_handler(&spi->transfer_handler, 0, SYSFS_SET_RETURN(EDEADLK), MCU_EVENT_FLAG_CANCELED);


            //turn off RCC clock
            switch(port){
            case 0:
                __HAL_RCC_SPI1_CLK_DISABLE();
                break;
#if defined SPI2
            case 1:
                __HAL_RCC_SPI2_CLK_DISABLE();
                break;
#endif
#if defined SPI3
            case 2:
                __HAL_RCC_SPI3_CLK_DISABLE();
                break;
#endif
#if defined SPI4
            case 3:
                __HAL_RCC_SPI4_CLK_DISABLE();
                break;
#endif
#if defined SPI5
            case 4:
                __HAL_RCC_SPI5_CLK_DISABLE();
                break;
#endif
#if defined SPI6
            case 5:
                __HAL_RCC_SPI6_CLK_DISABLE();
                break;
#endif
            }
        }
        spi->ref_count--;
    }
    return 0;
}

int spi_local_setattr(spi_local_t * spi, const devfs_handle_t * handle, void * ctl){
    u32 pclk;
    u32 prescalar;

    const spi_attr_t * attr = mcu_select_attr(handle, ctl);
    if( attr == 0 ){
        return SYSFS_SET_RETURN(EINVAL);
    }

    u32 o_flags = attr->o_flags;

    spi->o_flags = 0;

    if( o_flags & SPI_FLAG_SET_MASTER ){
        spi->hal_handle.Init.Mode = SPI_MODE_MASTER;

        if( attr->freq == 0 ){
            spi->hal_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
        } else {

            //#if defined __stm32f401xc
            if( (handle->port == 0) || (handle->port == 3) ){
                pclk = HAL_RCC_GetPCLK1Freq();
            } else {
                pclk = HAL_RCC_GetPCLK2Freq();
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


    if( o_flags & (SPI_FLAG_SET_SLAVE | SPI_FLAG_SET_MASTER) ){
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
    }

    if( o_flags & SPI_FLAG_IS_FULL_DUPLEX ){
        spi->o_flags |= SPI_LOCAL_IS_FULL_DUPLEX;
    } else {
        spi->o_flags &= ~SPI_LOCAL_IS_FULL_DUPLEX;
    }




    return 0;
}

int spi_local_swap(spi_local_t * spi, const devfs_handle_t * handle, void * ctl){
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

int spi_local_setaction(spi_local_t * spi, const devfs_handle_t * handle, void * ctl){
    mcu_action_t * action = (mcu_action_t*)ctl;

    //callback = 0 with flags set will cancel an ongoing operation
    if(action->handler.callback == 0){
        if (action->o_events & MCU_EVENT_FLAG_DATA_READY){
            devfs_execute_read_handler(
                        &spi->transfer_handler, 0,
                        SYSFS_SET_RETURN(EINTR),
                        MCU_EVENT_FLAG_CANCELED);
        }

        if (action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE){
            devfs_execute_write_handler(
                        &spi->transfer_handler, 0,
                        SYSFS_SET_RETURN(EINTR),
                        MCU_EVENT_FLAG_CANCELED);
        }
    }

    cortexm_set_irq_priority(spi_irqs[handle->port], action->prio, action->o_events);
    return 0;
}


//these callbacks work the same for both DMA and interrupt driven operations
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
    //execute the handler
    spi_local_t * spi = (spi_local_t*)hspi;
    //mcu_debug_log_info(MCU_DEBUG_DEVICE, "SPI TX DONE:%d,%d", hspi->TxXferSize, spi->transfer_handler.write ? spi->transfer_handler.write->nbyte : 0);
    devfs_execute_write_handler(&spi->transfer_handler, 0, hspi->TxXferSize, MCU_EVENT_FLAG_WRITE_COMPLETE);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
    //execute the handler
    spi_local_t * spi = (spi_local_t*)hspi;
    //mcu_debug_log_info(MCU_DEBUG_DEVICE, "SPI RX DONE:%d,%d", hspi->RxXferSize, spi->transfer_handler.read ? spi->transfer_handler.read->nbyte : 0);
    devfs_execute_read_handler(&spi->transfer_handler, 0, hspi->RxXferSize, MCU_EVENT_FLAG_DATA_READY);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
    //execute the handler
    spi_local_t * spi = (spi_local_t*)hspi;
    //mcu_debug_log_info(MCU_DEBUG_DEVICE, "SPI FD DONE %d", hspi->TxXferSize);
    devfs_execute_write_handler(&spi->transfer_handler,0, hspi->TxXferSize, MCU_EVENT_FLAG_WRITE_COMPLETE);
    devfs_execute_read_handler(&spi->transfer_handler,0, hspi->RxXferSize, MCU_EVENT_FLAG_DATA_READY);

}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi){
    spi_local_t * spi = (spi_local_t*)hspi;

    mcu_debug_log_error(MCU_DEBUG_DEVICE, "SPI Error:0x%X", hspi->ErrorCode);

    //deal with overrrun errors
    spi->hal_handle.Instance->DR;
    spi->hal_handle.Instance->SR;

    devfs_execute_cancel_handler(&spi->transfer_handler,0, SYSFS_SET_RETURN_WITH_VALUE(EIO, hspi->ErrorCode), MCU_EVENT_FLAG_ERROR);
}

void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi){
    spi_local_t * spi = (spi_local_t*)hspi;

    mcu_debug_log_warning(MCU_DEBUG_DEVICE, "SPI Abort:0x%X", hspi->ErrorCode);
    devfs_execute_cancel_handler(&spi->transfer_handler,0, hspi->RxXferSize - hspi->RxXferCount, 0);
}

//these handlers need to move to the local file
void mcu_core_spi1_isr(){
    //No I2S on SPI 1
#if MCU_I2S_ON_SPI1 != 0
    if( spi_local_ptrs[0]->o_flags & SPI_LOCAL_IS_I2S ){
        HAL_I2S_IRQHandler(&spi_local_ptrs[0]->i2s_hal_handle);
    } else {
        HAL_SPI_IRQHandler(&spi_local_ptrs[0]->hal_handle);
    }
#else
    HAL_SPI_IRQHandler(&spi_local_ptrs[0]->hal_handle);
#endif
}

void mcu_core_spi2_isr(){
#if MCU_I2S_ON_SPI2 != 0
    if( spi_local_ptrs[1]->o_flags & SPI_LOCAL_IS_I2S ){
        HAL_I2S_IRQHandler(&spi_local_ptrs[1]->i2s_hal_handle);
    } else {
        HAL_SPI_IRQHandler(&spi_local_ptrs[1]->hal_handle);
    }
#elif MCU_SPI_PORTS > 1
    HAL_SPI_IRQHandler(&spi_local_ptrs[1]->hal_handle);
#endif
}

void mcu_core_spi3_isr(){
#if MCU_I2S_ON_SPI3 != 0
    mcu_debug_printf("SPI3 IRQ\n");
    if( spi_local_ptrs[2]->o_flags & SPI_LOCAL_IS_I2S ){
        HAL_I2S_IRQHandler(&spi_local_ptrs[2]->i2s_hal_handle);
    } else {
        HAL_SPI_IRQHandler(&spi_local_ptrs[2]->hal_handle);
    }
#elif MCU_SPI_PORTS > 2
    HAL_SPI_IRQHandler(&spi_local_ptrs[2]->hal_handle);
#endif
}

void mcu_core_spi4_isr(){
#if MCU_I2S_ON_SPI4 != 0
    if( spi_local_ptrs[3]->o_flags & SPI_LOCAL_IS_I2S ){
        HAL_I2S_IRQHandler(&spi_local_ptrs[3]->i2s_hal_handle);
    } else {
        HAL_SPI_IRQHandler(&spi_local_ptrs[3]->hal_handle);
    }
#elif MCU_SPI_PORTS > 3
    HAL_SPI_IRQHandler(&spi_local_ptrs[3]->hal_handle);
#endif
}

#if MCU_SPI_PORTS > 4
void mcu_core_spi5_isr(){
    HAL_SPI_IRQHandler(&spi_local_ptrs[4]->hal_handle);
}
#endif

#if MCU_SPI_PORTS > 5
void mcu_core_spi6_isr(){
    HAL_SPI_IRQHandler(&spi_local_ptrs[5]->hal_handle);
}
#endif

#endif

